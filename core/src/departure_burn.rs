//! `departure_burn` — the escape burn a parked stack really flies: finite, split
//! into several firings at perigee (Phase 3's orbital assembly, its first
//! unmodelled cost).
//!
//! [`crate::orbital_assembly`] prices a parked launch's escape burn as one impulse:
//! `Δv = √(2μ/r + C3) − √(μ/r)` applied all at once. A real departure stage cannot
//! do that. The shipping engine ([`crate::orbital_assembly::ORION_OMS_E`]) pushes
//! 26.7 kN against a 26.5 t stack — about 0.1 g — and its rated **longest single
//! burn is 1 030 s** with **at most 10 starts** (Belair et al., *Artemis I
//! Orion-ESM Propulsion System Engine Performance*, Space Propulsion 2024,
//! SP2024_382, Table 2: "Thrust 26.7 kN … Max Burn Duration 1030 seconds … Number
//! of Starts 10 max"). The whole escape burn at `C3` ~43 is ~2 450 s, so it is at
//! least three firings, and every second of it is spent away from the ideal
//! point. That costs propellant: the **loss** this module measures.
//!
//! # The flight
//! Planar two-body (Earth's point mass) from a circular parking orbit. The engine
//! thrusts along the velocity (tangential steering — near-optimal for perigee
//! burns and never better than the optimum, so any error is against parking),
//! with the mass falling at `F / (Isp·g0)`. The burn is split into `n` firings:
//!
//! - `n − 1` **phasing firings** of one equal length `τ`, each centred on the
//!   current perigee, raising the apogee step by step while the orbit stays bound;
//! - one **final firing**, also centred on perigee, whose length is solved so the
//!   stack leaves on exactly the target `C3`.
//!
//! `τ` is chosen to leave the most mass, subject to: no firing longer than the
//! engine's maximum, every intermediate orbit bound, and no intermediate apogee
//! beyond a cap (see below). Equal phasing firings are a *rule*, not an optimiser;
//! measured against a free optimisation of every firing length (Nelder–Mead,
//! offline) at `C3` 43 it costs 33.5 / 26.8 / 23.9 m/s where the optimum is
//! 33.4 / 26.7 / 23.9 m/s (apogee caps 100 / 200 / 400 thousand km) — at or a
//! hair above it, so the rule never credits parking with more than is possible.
//!
//! # Why the apogee is capped
//! More firings mean shorter ones, and shorter ones lose less — but the last orbit
//! before escape then swings out further, and left free the schedule pushes it
//! past the Moon (over a million km). An orbit out there is flown by the Moon as
//! much as by Earth, and this two-body flight cannot say where its perigee ends
//! up. The cap is a stated choice with the loss reported across it; the Moon's
//! tidal pull relative to Earth's at apogee `r` is `2 (μ☾/μ⊕)(r/d☾)³` — 0.04 % at
//! 100 000 km, 0.35 % at 200 000 km, 2.8 % at 400 000 km.
//!
//! # What the loss is
//! `loss = Isp·g0·ln(m₀ / m_out) − Δv_impulsive` — the extra Δv the finite burn
//! pays for the same departure, in m/s. It goes to zero as thrust grows without
//! bound (pinned) and is what [`crate::orbital_assembly`] charges a parked launch.
//!
//! # Not modelled
//! The Moon and Sun during the phasing loops, Earth's oblateness (it turns the
//! line of apsides by well under a degree over the few days of phasing, which a
//! real schedule plans around), and the starts the stack spends on anything but
//! escape — counted against the engine's 10 by the caller's choice of `n`.

use nalgebra::Vector3;

use crate::epoch::Epoch;
use crate::forces::{ForceError, ForceModel};
use crate::impactor_mass::G0_M_S2;
use crate::integrator::{Dop853, Integrator};
use crate::orbital_assembly::EARTH_GM_M3_S2;
use crate::state::StateVector;

/// The engine and schedule limits of one finite escape burn.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FiniteBurn {
    /// Vacuum thrust, N.
    pub thrust_n: f64,
    /// Vacuum specific impulse, s.
    pub isp_s: f64,
    /// The engine's longest rated single firing, s.
    pub max_firing_s: f64,
    /// Firings the escape uses — phasing firings plus the final one. At least 1.
    pub firings: u32,
    /// The highest apogee any intermediate orbit may reach, m from Earth's centre.
    pub apogee_cap_m: f64,
}

/// What one flown escape burn came to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BurnOutcome {
    /// The mass that leaves on the target `C3`, kg.
    pub separated_kg: f64,
    /// Length of each phasing firing, s (`0` for a single firing).
    pub phasing_firing_s: f64,
    /// Length of the final firing, s.
    pub final_firing_s: f64,
    /// The highest apogee the phasing orbits reached, m (the parking radius for a
    /// single firing).
    pub highest_apogee_m: f64,
    /// Extra Δv against the impulsive burn to the same `C3`, m/s.
    pub loss_m_s: f64,
}

/// The impulsive escape burn from a circular orbit of radius `r_m` to `c3_km2_s2`,
/// m/s.
pub fn impulsive_dv_m_s(r_m: f64, c3_km2_s2: f64) -> f64 {
    (2.0 * EARTH_GM_M3_S2 / r_m + c3_km2_s2 * 1.0e6).sqrt() - (EARTH_GM_M3_S2 / r_m).sqrt()
}

/// Earth's point mass plus a constant thrust along the velocity, with the mass
/// falling linearly from `m0_kg` at epoch 0.
struct ThrustArc {
    thrust_n: f64,
    m0_kg: f64,
    mdot_kg_s: f64,
}

impl ForceModel for ThrustArc {
    fn acceleration(&self, epoch: Epoch, s: &StateVector) -> Result<Vector3<f64>, ForceError> {
        let r = s.position.norm();
        let m = self.m0_kg - self.mdot_kg_s * epoch.tdb_seconds_past_j2000();
        let gravity = -EARTH_GM_M3_S2 / (r * r * r) * s.position;
        Ok(gravity + (self.thrust_n / m) * s.velocity.normalize())
    }
}

/// Specific orbital energy, m²/s².
fn energy(s: &StateVector) -> f64 {
    0.5 * s.velocity.norm_squared() - EARTH_GM_M3_S2 / s.position.norm()
}

/// Eccentricity vector.
fn ecc_vec(s: &StateVector) -> Vector3<f64> {
    let (r, v) = (s.position, s.velocity);
    ((v.norm_squared() - EARTH_GM_M3_S2 / r.norm()) * r - r.dot(&v) * v) / EARTH_GM_M3_S2
}

/// Apogee radius of a bound orbit, m.
fn apogee_m(s: &StateVector) -> f64 {
    let a = -EARTH_GM_M3_S2 / (2.0 * energy(s));
    a * (1.0 + ecc_vec(s).norm())
}

/// The same bound, eccentric orbit, at the point `dt` seconds before perigee.
fn before_perigee(s: &StateVector, dt: f64) -> StateVector {
    let h = s.position.cross(&s.velocity);
    let ev = ecc_vec(s);
    let e = ev.norm();
    let a = -EARTH_GM_M3_S2 / (2.0 * energy(s));
    let n = (EARTH_GM_M3_S2 / (a * a * a)).sqrt();
    let mean = -n * dt;
    let mut ecc_anom = mean;
    for _ in 0..60 {
        let d = (ecc_anom - e * ecc_anom.sin() - mean) / (1.0 - e * ecc_anom.cos());
        ecc_anom -= d;
        if d.abs() < 1e-15 {
            break;
        }
    }
    let nu = 2.0
        * ((1.0 + e).sqrt() * (ecc_anom / 2.0).sin())
            .atan2((1.0 - e).sqrt() * (ecc_anom / 2.0).cos());
    let p_hat = ev / e;
    let h_hat = h / h.norm();
    let q_hat = h_hat.cross(&p_hat);
    let hn = h.norm();
    let r = hn * hn / EARTH_GM_M3_S2 / (1.0 + e * nu.cos());
    let position = r * (nu.cos() * p_hat + nu.sin() * q_hat);
    let velocity = EARTH_GM_M3_S2 / hn * (-nu.sin() * p_hat + (e + nu.cos()) * q_hat);
    StateVector::new(position, velocity)
}

impl FiniteBurn {
    fn mdot_kg_s(&self) -> f64 {
        self.thrust_n / (self.isp_s * G0_M_S2)
    }

    /// One firing of `dur` seconds from `s` with mass `m_kg`.
    fn fire(&self, s: &StateVector, m_kg: f64, dur: f64) -> (StateVector, f64) {
        if dur <= 0.0 {
            return (*s, m_kg);
        }
        let arc = ThrustArc {
            thrust_n: self.thrust_n,
            m0_kg: m_kg,
            mdot_kg_s: self.mdot_kg_s(),
        };
        let out = Dop853::new()
            .with_tolerances(1e-12, 1e-6)
            .step(&arc, Epoch::from_tdb_seconds_past_j2000(0.0), s, dur)
            .expect("a two-body thrust arc integrates");
        (out, m_kg - self.mdot_kg_s() * dur)
    }

    /// The final firing from `s` (bound, eccentric — or the circular parking orbit
    /// when `centre` is false) to `C3`, centred on perigee. `None` when the engine's
    /// longest firing or the propellant cannot reach it.
    fn final_firing(
        &self,
        s: &StateVector,
        m_kg: f64,
        c3_km2_s2: f64,
        centre: bool,
    ) -> Option<(f64, f64)> {
        let target = 0.5 * c3_km2_s2 * 1.0e6;
        let fly = |d: f64| {
            let s0 = if centre {
                before_perigee(s, d / 2.0)
            } else {
                *s
            };
            let (out, m) = self.fire(&s0, m_kg, d);
            (energy(&out) - target, m)
        };
        let hi = self.max_firing_s.min(0.999 * m_kg / self.mdot_kg_s());
        if fly(hi).0 < 0.0 {
            return None;
        }
        let (mut lo, mut up) = (0.0, hi);
        for _ in 0..60 {
            let mid = 0.5 * (lo + up);
            if fly(mid).0 < 0.0 {
                lo = mid;
            } else {
                up = mid;
            }
            if up - lo < 1e-4 {
                break;
            }
        }
        Some((up, fly(up).1))
    }

    /// The `firings − 1` phasing firings of length `tau` from the parking orbit, or
    /// `None` where a phasing limit breaks. Returns `(state, mass kg, highest
    /// apogee m)` before the final firing.
    fn phasing(&self, stack_kg: f64, r_park_m: f64, tau: f64) -> Option<(StateVector, f64, f64)> {
        let mut s = StateVector::new(
            Vector3::new(r_park_m, 0.0, 0.0),
            Vector3::new(0.0, (EARTH_GM_M3_S2 / r_park_m).sqrt(), 0.0),
        );
        let mut m = stack_kg;
        let mut highest = r_park_m;
        for k in 0..self.firings.saturating_sub(1) {
            if tau > self.max_firing_s || self.mdot_kg_s() * tau >= m {
                return None;
            }
            let s0 = if k == 0 {
                s
            } else {
                before_perigee(&s, tau / 2.0)
            };
            (s, m) = self.fire(&s0, m, tau);
            if energy(&s) >= 0.0 {
                return None;
            }
            highest = highest.max(apogee_m(&s));
            if highest > self.apogee_cap_m {
                return None;
            }
        }
        Some((s, m, highest))
    }

    /// The whole escape for one phasing length `tau`, or `None` where a limit
    /// breaks. Returns `(m_out, final firing s, highest apogee m)`.
    fn fly_with(
        &self,
        stack_kg: f64,
        r_park_m: f64,
        c3_km2_s2: f64,
        tau: f64,
    ) -> Option<(f64, f64, f64)> {
        let (s, m, highest) = self.phasing(stack_kg, r_park_m, tau)?;
        let (d, m_out) = self.final_firing(&s, m, c3_km2_s2, self.firings > 1)?;
        Some((m_out, d, highest))
    }

    /// The escape burn from a circular orbit of radius `r_park_m` to `c3_km2_s2`
    /// for a stack of `stack_kg`, by the equal-phasing rule (module doc). `None`
    /// when no phasing length satisfies every limit.
    pub fn fly(&self, stack_kg: f64, r_park_m: f64, c3_km2_s2: f64) -> Option<BurnOutcome> {
        assert!(self.firings >= 1, "an escape takes at least one firing");
        let outcome = |tau: f64, (m_out, d, highest): (f64, f64, f64)| {
            let dv = self.isp_s * G0_M_S2 * (stack_kg / m_out).ln();
            BurnOutcome {
                separated_kg: m_out,
                phasing_firing_s: tau,
                final_firing_s: d,
                highest_apogee_m: highest,
                loss_m_s: dv - impulsive_dv_m_s(r_park_m, c3_km2_s2),
            }
        };
        if self.firings == 1 {
            return self
                .fly_with(stack_kg, r_park_m, c3_km2_s2, 0.0)
                .map(|r| outcome(0.0, r));
        }
        // The phasing limits (bound orbits, the apogee cap, the longest firing, the
        // propellant) all tighten as `tau` grows, so the phasing-feasible lengths
        // are one interval from the shortest firing up to `tau_max`, found exactly
        // first: where the cap binds the best `tau` sits on that edge, and at high
        // C3 the lengths that also let the final firing reach are a sliver below
        // it that a blind scan steps over.
        let phasing_ok = |tau: f64| self.phasing(stack_kg, r_park_m, tau).is_some();
        let lo_tau = 2.0;
        if !phasing_ok(lo_tau) {
            return None;
        }
        let hi_tau = if phasing_ok(self.max_firing_s) {
            self.max_firing_s
        } else {
            let (mut ok, mut bad) = (lo_tau, self.max_firing_s);
            while bad - ok > 1e-6 {
                let mid = 0.5 * (ok + bad);
                if phasing_ok(mid) {
                    ok = mid;
                } else {
                    bad = mid;
                }
            }
            ok
        };
        let mass = |tau: f64| {
            self.fly_with(stack_kg, r_park_m, c3_km2_s2, tau)
                .map_or(f64::NEG_INFINITY, |r| r.0)
        };
        // Scan inside it (the edge included), then refine around the best sample.
        const SCAN: usize = 64;
        let at = |i: usize| lo_tau + (hi_tau - lo_tau) * i as f64 / (SCAN - 1) as f64;
        let samples: Vec<f64> = (0..SCAN).map(|i| mass(at(i))).collect();
        let best = (0..SCAN).max_by(|&a, &b| samples[a].total_cmp(&samples[b]))?;
        if samples[best] == f64::NEG_INFINITY {
            return None;
        }
        // Golden section on the bracket around it (an infeasible point reads -inf,
        // which the search walks away from).
        let (mut a, mut b) = (at(best.saturating_sub(1)), at((best + 1).min(SCAN - 1)));
        let g = 0.5 * (5.0_f64.sqrt() - 1.0);
        let (mut x1, mut x2) = (b - g * (b - a), a + g * (b - a));
        let (mut f1, mut f2) = (mass(x1), mass(x2));
        while b - a > 0.05 {
            if f1 >= f2 {
                b = x2;
                (x2, f2) = (x1, f1);
                x1 = b - g * (b - a);
                f1 = mass(x1);
            } else {
                a = x1;
                (x1, f1) = (x2, f2);
                x2 = a + g * (b - a);
                f2 = mass(x2);
            }
        }
        let mut tau = at(best);
        let mut f = samples[best];
        for (x, fx) in [(x1, f1), (x2, f2)] {
            if fx > f {
                (tau, f) = (x, fx);
            }
        }
        debug_assert!(f.is_finite());
        self.fly_with(stack_kg, r_park_m, c3_km2_s2, tau)
            .map(|r| outcome(tau, r))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const R_PARK: f64 = 6_378_137.0 + 185_000.0;

    fn oms(firings: u32, cap_km: f64) -> FiniteBurn {
        FiniteBurn {
            thrust_n: 26_700.0,
            isp_s: 315.1,
            max_firing_s: 1_030.0,
            firings,
            apogee_cap_m: cap_km * 1e3,
        }
    }

    /// The loss vanishes as thrust grows: a very strong engine flies the impulsive
    /// burn.
    #[test]
    fn the_loss_vanishes_with_unbounded_thrust() {
        let mut last = f64::INFINITY;
        for thrust in [26_700.0, 267_000.0, 2_670_000.0] {
            let b = FiniteBurn {
                thrust_n: thrust,
                max_firing_s: 1.0e6,
                firings: 1,
                ..oms(1, 1.0e5)
            };
            let o = b
                .fly(26_500.0, R_PARK, 43.0)
                .expect("one firing reaches C3 43");
            assert!(
                o.loss_m_s > 0.0 && o.loss_m_s < last,
                "{thrust}: {}",
                o.loss_m_s
            );
            last = o.loss_m_s;
        }
        assert!(last < 0.5, "{last}");
    }

    /// The shipping engine's limits: 3 firings is the fewest that reach `C3` 43
    /// (two would need a firing past 1 030 s), and more firings lose less.
    #[test]
    fn more_firings_lose_less_and_two_cannot_escape() {
        assert!(oms(2, 1.0e5).fly(26_500.0, R_PARK, 43.0).is_none());
        let mut last = f64::INFINITY;
        for n in [3, 5, 8, 10] {
            let o = oms(n, 1.0e5).fly(26_500.0, R_PARK, 43.0).expect("reaches");
            assert!(o.loss_m_s < last, "{n}: {} not below {last}", o.loss_m_s);
            assert!(o.final_firing_s <= 1_030.0 && o.phasing_firing_s <= 1_030.0);
            assert!(o.highest_apogee_m <= 1.0e8 * (1.0 + 1e-12));
            last = o.loss_m_s;
        }
    }

    /// Against the offline Python flight of the same rule (scipy DOP853, rtol
    /// 1e-11): C3 43 from 185 km, OMS-E, 100 000 km cap.
    #[test]
    fn matches_the_offline_rule() {
        for (n, cap_km, expect) in [
            (3, 1.0e5, 121.8),
            (5, 1.0e5, 53.2),
            (10, 1.0e5, 33.5),
            (10, 4.0e5, 23.9),
        ] {
            let o = oms(n, cap_km).fly(26_500.0, R_PARK, 43.0).expect("reaches");
            assert!(
                (o.loss_m_s - expect).abs() < 0.3,
                "n {n} cap {cap_km}: {} vs {expect}",
                o.loss_m_s
            );
        }
    }

    /// The mass out is the rocket equation on the Δv actually spent.
    #[test]
    fn the_mass_out_is_consistent_with_the_loss() {
        let o = oms(8, 2.0e5).fly(26_500.0, R_PARK, 38.4).expect("reaches");
        let dv = impulsive_dv_m_s(R_PARK, 38.4) + o.loss_m_s;
        let m = 26_500.0 * (-dv / (315.1 * G0_M_S2)).exp();
        assert!(
            (m - o.separated_kg).abs() < 1e-6,
            "{m} vs {}",
            o.separated_kg
        );
    }
}
