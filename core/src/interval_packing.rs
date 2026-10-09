//! `interval_packing` — the most valuable whole-number packing under caps on runs
//! of consecutive items, exactly.
//!
//! The campaign planner's **independent check**. Its production-line plans
//! ([`crate::campaign`], `LotChains`) come from chains with release dates, an
//! argument about ranks; this module states the same rules the plain way - "at most
//! N in any 12 months", "at most S before the build date", "at most S + k·B before
//! the k-th lot" are each a cap on a run of launch windows in date order - and
//! solves that exactly, by a different route. The campaign tests pin the chains
//! against it on random cases. It is too slow to plan with: it was tried first, and
//! took 10-60 s for a row of plans the chains make in a tenth of a second.
//!
//! # The problem
//! Items `0..n` (launch windows in date order), each with a value `vᵢ`; choose
//! whole numbers `xᵢ ≥ 0` (launches through each) adding up to exactly `total`,
//! maximising `Σ vᵢ·xᵢ`, subject to caps `Σ_{a ≤ i < b} xᵢ ≤ c` — each cap a sum over
//! a **run** of consecutive items. Every rule above is one: a 12-month window is the
//! run of windows dated inside it, "before a date" is the run from the first.
//!
//! # Exact, because every row is a run
//! A matrix whose rows each hold a run of ones (an *interval matrix*) is totally
//! unimodular, so the linear programme has a whole-number optimum and no branching
//! is needed. Writing `X_k = Σ_{i<k} xᵢ` turns every row into a difference,
//! `X_b − X_a ≤ c`, and `xᵢ ≥ 0` into `X_i − X_{i+1} ≤ 0`; the objective is
//! `Σ_k w_k·X_k` with `w_k = v_{k−1} − v_k`. That programme is the dual of a
//! minimum-cost flow on the nodes `0..=n` — one arc `a → b` of cost `c` per
//! difference row, node `k` taking in `w_k` net — and the flow's optimal node
//! potentials `π` are the prefix counts themselves, `X = π` (a reduced cost
//! `c + π_a − π_b ≥ 0` on every arc is the row `X_b − X_a ≤ c`). The flow is solved by successive
//! shortest paths; its arc costs are the caps (whole numbers), so the potentials,
//! and so the `xᵢ`, come out whole.
//!
//! A total that no packing reaches shows up as a negative cycle among the
//! difference rows, and the solver says `None`.

/// `Σ_{from ≤ i < to} xᵢ ≤ cap`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunCap {
    /// First item of the run.
    pub from: usize,
    /// One past the last item of the run.
    pub to: usize,
    /// The most the run may hold in all.
    pub cap: i64,
}

/// One arc of the flow and the arc that undoes it.
#[derive(Debug, Clone, Copy)]
struct Arc {
    to: usize,
    cost: i64,
    /// What can still go along it: unbounded for a difference row, the flow already
    /// sent for the arc that undoes one.
    room: f64,
    /// Index of the paired arc.
    pair: usize,
}

/// The packing of exactly `total` that maximises `Σ values[i]·x[i]` under `caps`,
/// or `None` if no whole-number packing of `total` fits them. Every cap's run must
/// lie inside `0..values.len()`. Values may be any finite numbers; an item whose
/// value is not positive is never worth a launch, and a caller that does not want
/// one filters it out first.
pub fn best_packing(values: &[f64], caps: &[RunCap], total: u32) -> Option<Vec<u32>> {
    let n = values.len();
    if n == 0 {
        return (total == 0).then(Vec::new);
    }
    assert!(
        caps.iter().all(|c| c.from <= c.to && c.to <= n),
        "a cap's run lies outside the items"
    );
    let total = i64::from(total);
    let nodes = n + 1;
    let mut arcs: Vec<Arc> = Vec::new();
    let mut out: Vec<Vec<usize>> = vec![Vec::new(); nodes];
    let mut add = |a: usize, b: usize, cost: i64, arcs: &mut Vec<Arc>| {
        let i = arcs.len();
        arcs.push(Arc {
            to: b,
            cost,
            room: f64::INFINITY,
            pair: i + 1,
        });
        arcs.push(Arc {
            to: a,
            cost: -cost,
            room: 0.0,
            pair: i,
        });
        out[a].push(i);
        out[b].push(i + 1);
    };
    // xᵢ ≥ 0: X_i − X_{i+1} ≤ 0.
    for i in 0..n {
        add(i + 1, i, 0, &mut arcs);
    }
    for c in caps {
        add(c.from, c.to, c.cap, &mut arcs);
    }
    // Exactly `total`: X_n − X_0 ≤ total and X_0 − X_n ≤ −total.
    add(0, n, total, &mut arcs);
    add(n, 0, -total, &mut arcs);

    // Starting potentials: shortest distances from a virtual node joined to every
    // node at zero cost (Bellman-Ford over the difference rows; the arcs that undo
    // them carry nothing yet). One more round that still improves is a negative
    // cycle: the rows contradict each other and `total` does not fit.
    let mut pi = vec![0i64; nodes];
    for round in 0..=nodes {
        let mut changed = false;
        for a in 0..nodes {
            for &e in &out[a] {
                let arc = arcs[e];
                if arc.room > 0.0 && pi[a] + arc.cost < pi[arc.to] {
                    pi[arc.to] = pi[a] + arc.cost;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
        if round == nodes {
            return None;
        }
    }

    // Node k must take in w_k net: what it has to send out is −w_k.
    let mut excess: Vec<f64> = (0..nodes)
        .map(|k| {
            let before = if k == 0 { 0.0 } else { values[k - 1] };
            let here = if k == n { 0.0 } else { values[k] };
            -(before - here)
        })
        .collect();
    let scale: f64 = excess.iter().map(|e| e.abs()).sum();
    let eps = 1e-12 * scale.max(f64::MIN_POSITIVE);

    // Successive shortest paths: from every node with something to send at once,
    // to the nearest node short of something, on costs reduced by the potentials
    // (never negative, so Dijkstra holds).
    loop {
        if !excess.iter().any(|&e| e < -eps) || !excess.iter().any(|&e| e > eps) {
            break;
        }
        let mut dist = vec![i64::MAX; nodes];
        let mut via: Vec<Option<usize>> = vec![None; nodes];
        let mut heap = std::collections::BinaryHeap::new();
        for k in 0..nodes {
            if excess[k] > eps {
                dist[k] = 0;
                heap.push(std::cmp::Reverse((0i64, k)));
            }
        }
        while let Some(std::cmp::Reverse((d, a))) = heap.pop() {
            if d > dist[a] {
                continue;
            }
            for &e in &out[a] {
                let arc = arcs[e];
                if arc.room <= eps {
                    continue;
                }
                let reduced = arc.cost + pi[a] - pi[arc.to];
                debug_assert!(reduced >= 0, "a negative reduced cost");
                let nd = d + reduced;
                if nd < dist[arc.to] {
                    dist[arc.to] = nd;
                    via[arc.to] = Some(e);
                    heap.push(std::cmp::Reverse((nd, arc.to)));
                }
            }
        }
        // The nearest node still short; every node is reachable (the difference
        // rows alone join them all: back along `xᵢ ≥ 0`, forward along the total).
        let Some(sink) = (0..nodes)
            .filter(|&k| excess[k] < -eps && dist[k] < i64::MAX)
            .min_by_key(|&k| dist[k])
        else {
            break;
        };
        for k in 0..nodes {
            if dist[k] < i64::MAX {
                pi[k] += dist[k];
            }
        }
        // Walk the path back to its source and send what it can carry.
        let mut path = Vec::new();
        let mut k = sink;
        while let Some(e) = via[k] {
            path.push(e);
            k = arcs[arcs[e].pair].to;
        }
        let source = k;
        let mut send = excess[source].min(-excess[sink]);
        for &e in &path {
            send = send.min(arcs[e].room);
        }
        for &e in &path {
            arcs[e].room -= send;
            let p = arcs[e].pair;
            arcs[p].room += send;
        }
        excess[source] -= send;
        excess[sink] += send;
    }

    // X = π (any constant off): xᵢ = X_{i+1} − X_i.
    let x: Vec<i64> = (0..n).map(|i| pi[i + 1] - pi[i]).collect();
    let fits = x.iter().all(|&xi| xi >= 0)
        && x.iter().sum::<i64>() == total
        && caps
            .iter()
            .all(|c| x[c.from..c.to].iter().sum::<i64>() <= c.cap);
    assert!(fits, "the flow's potentials do not give a packing: {x:?}");
    Some(x.into_iter().map(|xi| xi as u32).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every packing of `total` that fits, by brute force; the best value.
    fn brute(values: &[f64], caps: &[RunCap], total: u32) -> Option<f64> {
        fn go(
            i: usize,
            left: u32,
            x: &mut Vec<u32>,
            values: &[f64],
            caps: &[RunCap],
            best: &mut Option<f64>,
        ) {
            if i == values.len() {
                if left == 0
                    && caps.iter().all(|c| {
                        x[c.from..c.to].iter().map(|&v| i64::from(v)).sum::<i64>() <= c.cap
                    })
                {
                    let v: f64 = x.iter().zip(values).map(|(&n, v)| f64::from(n) * v).sum();
                    if best.is_none_or(|b| v > b) {
                        *best = Some(v);
                    }
                }
                return;
            }
            for n in 0..=left {
                x.push(n);
                go(i + 1, left - n, x, values, caps, best);
                x.pop();
            }
        }
        let mut best = None;
        go(0, total, &mut Vec::new(), values, caps, &mut best);
        best
    }

    /// A small deterministic generator, so the cases are the same every run.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            self.0 >> 33
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    #[test]
    fn matches_brute_force_on_random_runs() {
        let mut rng = Lcg(7);
        let mut fitted = 0;
        let mut refused = 0;
        for _ in 0..400 {
            let n = 1 + rng.below(6) as usize;
            let values: Vec<f64> = (0..n)
                .map(|_| 0.1 + rng.below(1000) as f64 / 100.0)
                .collect();
            let caps: Vec<RunCap> = (0..rng.below(5))
                .map(|_| {
                    let a = rng.below(n as u64) as usize;
                    let b = a + 1 + rng.below((n - a) as u64) as usize;
                    RunCap {
                        from: a,
                        to: b,
                        cap: rng.below(4) as i64,
                    }
                })
                .collect();
            for total in 0..=5 {
                let want = brute(&values, &caps, total);
                let got = best_packing(&values, &caps, total);
                match (want, &got) {
                    (None, None) => refused += 1,
                    (Some(w), Some(x)) => {
                        let v: f64 = x.iter().zip(&values).map(|(&k, v)| f64::from(k) * v).sum();
                        assert!(
                            (v - w).abs() <= 1e-9 * w.abs().max(1.0),
                            "value {v} vs brute force {w}: {values:?} {caps:?} {total} -> {x:?}"
                        );
                        fitted += 1;
                    }
                    _ => panic!(
                        "feasibility differs: {values:?} {caps:?} {total}: {want:?} vs {got:?}"
                    ),
                }
            }
        }
        // Both branches are exercised, not only one.
        assert!(
            fitted > 500 && refused > 100,
            "{fitted} fitted, {refused} refused"
        );
    }

    #[test]
    fn a_strong_late_item_takes_the_whole_budget_when_the_run_allows() {
        // Items 0 and 1 are weak, item 2 strong; at most 1 in the first two and at
        // most 3 anywhere: all three go to item 2.
        let caps = [
            RunCap {
                from: 0,
                to: 2,
                cap: 1,
            },
            RunCap {
                from: 0,
                to: 3,
                cap: 3,
            },
        ];
        assert_eq!(
            best_packing(&[1.0, 2.0, 5.0], &caps, 3),
            Some(vec![0, 0, 3])
        );
        // A cap on the strong one alone pushes the rest back, best first.
        let caps = [
            RunCap {
                from: 0,
                to: 2,
                cap: 1,
            },
            RunCap {
                from: 2,
                to: 3,
                cap: 1,
            },
        ];
        assert_eq!(
            best_packing(&[1.0, 2.0, 5.0], &caps, 2),
            Some(vec![0, 1, 1])
        );
        assert_eq!(best_packing(&[1.0, 2.0, 5.0], &caps, 3), None);
    }

    #[test]
    fn nothing_to_pack_is_only_a_total_of_zero() {
        assert_eq!(best_packing(&[], &[], 0), Some(vec![]));
        assert_eq!(best_packing(&[], &[], 1), None);
        assert_eq!(best_packing(&[3.0], &[], 0), Some(vec![0]));
    }
}
