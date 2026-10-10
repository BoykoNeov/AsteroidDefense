//! A small deterministic random-number generator for the shipped code.
//!
//! The sky shots put a measurement error on the asteroid's image
//! (`sky_shot`), and the same shots must come out the same every time — for the
//! player, who can retake a measurement and get the same picture, and for the
//! tests. The core links only `hifitime`, `anise` and `nalgebra` (HANDOFF §3),
//! so this is the same xorshift64* plus Box–Muller that the `sbdb` and
//! `uncertainty` Monte Carlo tests already carry privately, promoted to one
//! public copy.
//!
//! Not cryptographic, and not meant to be: its job is a reproducible normal
//! deviate. Its first outputs are pinned by a test, so a change to the algorithm
//! (which would silently change every published shot) fails loudly.

/// xorshift64* with a Box–Muller normal on top.
#[derive(Debug, Clone)]
pub struct NormalRng {
    state: u64,
    spare: Option<f64>,
}

impl NormalRng {
    /// A generator from `seed`. Zero is not a valid xorshift state, so the low
    /// bit is forced on.
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed | 1,
            spare: None,
        }
    }

    /// The next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform on `(0, 1)`: 53 bits of mantissa, open at zero so a logarithm of
    /// it is finite.
    pub fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// A standard normal deviate (mean 0, variance 1).
    pub fn normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        let (u1, u2) = (self.uniform(), self.uniform());
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = std::f64::consts::TAU * u2;
        self.spare = Some(r * theta.sin());
        r * theta.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The first outputs for one seed, written down once. If this fails, every
    /// shot the game has ever shown would come out different.
    #[test]
    fn the_sequence_is_pinned() {
        let mut r = NormalRng::new(42);
        let got: Vec<u64> = (0..3).map(|_| r.next_u64()).collect();
        assert_eq!(got, PINNED_U64);
        let mut r = NormalRng::new(42);
        let z = r.normal();
        assert!((z - PINNED_NORMAL).abs() < 1e-15, "{z:.17}");
    }

    // Computed independently in Python (64-bit masked xorshift64*, then
    // Box–Muller's cosine branch), not copied from this module's output.
    const PINNED_U64: [u64; 3] = [
        11_435_511_379_416_088_765,
        8_363_626_497_947_505_399,
        2_103_083_356_132_978_009,
    ];
    const PINNED_NORMAL: f64 = -0.936_288_520_581_743_7;

    #[test]
    fn normals_have_unit_variance_and_zero_mean() {
        let mut r = NormalRng::new(7);
        let n = 200_000;
        let (mut s, mut s2) = (0.0, 0.0);
        for _ in 0..n {
            let z = r.normal();
            s += z;
            s2 += z * z;
        }
        let mean = s / n as f64;
        let var = s2 / n as f64 - mean * mean;
        // Standard errors: 1/sqrt(n) = 0.0022 for the mean, sqrt(2/n) = 0.0032
        // for the variance; five of each.
        assert!(mean.abs() < 0.011, "{mean}");
        assert!((var - 1.0).abs() < 0.016, "{var}");
    }
}
