//! Per-purpose random streams.
//!
//! Every random quantity comes from a stream keyed by (seed, purpose, key).
//! Keys are vehicle ids or ramp ids, so vehicle #4812 gets the same arrival
//! time, traits and noise in both worlds of the split view (common random
//! numbers), no matter what happens elsewhere in either world.

use rand::{Rng, RngCore, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// What a random stream is used for. Each purpose is an independent stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u64)]
pub enum Purpose {
    Arrivals = 1,
    Traits = 2,
    Noise = 3,
    Kind = 4,
    Bus = 5,
    Init = 6,
    Route = 7,
}

/// SplitMix64 finalizer, used to decorrelate seeds.
pub fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// A ChaCha stream for one (seed, purpose, key) triple.
pub fn stream(seed: u64, purpose: Purpose, key: u64) -> ChaCha8Rng {
    let mut r = ChaCha8Rng::seed_from_u64(mix(seed ^ mix(purpose as u64)));
    r.set_stream(key);
    r
}

/// One uniform draw in [0, 1) from a keyed stream. Used for one-off
/// per-vehicle decisions such as "is this vehicle an auto".
pub fn uniform(seed: u64, purpose: Purpose, key: u64) -> f64 {
    stream(seed, purpose, key).random::<f64>()
}

/// Small, fast per-vehicle generator for continuous noise processes.
/// Seeded from the vehicle's ChaCha noise stream, so it is fully determined
/// by (seed, vehicle id).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VehRng(u64);

impl VehRng {
    pub fn new(seed: u64, id: u64) -> Self {
        VehRng(stream(seed, Purpose::Noise, id).next_u64())
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        mix(self.0)
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal (Box-Muller, one value per call).
    pub fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-300);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

/// Standard normal from any rand generator (Box-Muller).
pub fn normal<R: Rng>(r: &mut R) -> f64 {
    let u1: f64 = r.random::<f64>().max(1e-300);
    let u2: f64 = r.random();
    (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn streams_are_keyed_and_repeatable() {
        let a = uniform(1, Purpose::Traits, 7);
        assert_eq!(a, uniform(1, Purpose::Traits, 7));
        assert_ne!(a, uniform(1, Purpose::Traits, 8));
        assert_ne!(a, uniform(1, Purpose::Kind, 7));
        assert_ne!(a, uniform(2, Purpose::Traits, 7));
    }

    #[test]
    fn veh_rng_normal_has_unit_variance() {
        let mut r = VehRng::new(3, 9);
        let n = 20_000;
        let xs: Vec<f64> = (0..n).map(|_| r.normal()).collect();
        let mean = xs.iter().sum::<f64>() / n as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        assert!(mean.abs() < 0.03, "mean {mean}");
        assert!((var - 1.0).abs() < 0.05, "var {var}");
    }
}
