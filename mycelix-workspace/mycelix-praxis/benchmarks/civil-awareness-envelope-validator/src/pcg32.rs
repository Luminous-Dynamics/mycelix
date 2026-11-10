//! Deterministic PCG-XSH-RR-64/32 streams for synthetic challenge generation.
//!
//! Case-specific stream derivation is domain-separated and does not depend on
//! execution order, so adding a new case cannot shift existing case streams.

use sha2::{Digest, Sha256};

const MULTIPLIER: u64 = 6_364_136_223_846_793_005;
const STREAM_DOMAIN: &[u8] = b"luminous-dynamics/gcpn/coverage-calibration/v0.1\0";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pcg32 {
    state: u64,
    increment: u64,
}

impl Pcg32 {
    /// Initialize PCG-XSH-RR-64/32 with the reference two-step seeding scheme.
    pub fn new(initial_state: u64, initial_sequence: u64) -> Self {
        let mut rng = Self {
            state: 0,
            increment: (initial_sequence << 1) | 1,
        };
        let _ = rng.next_u32();
        rng.state = rng.state.wrapping_add(initial_state);
        let _ = rng.next_u32();
        rng
    }

    /// Generate one PCG-XSH-RR-64/32 output word.
    pub fn next_u32(&mut self) -> u32 {
        let old_state = self.state;
        self.state = old_state
            .wrapping_mul(MULTIPLIER)
            .wrapping_add(self.increment);
        let xorshifted = (((old_state >> 18) ^ old_state) >> 27) as u32;
        let rotation = (old_state >> 59) as u32;
        xorshifted.rotate_right(rotation)
    }

    /// Uniform variate on [0, 1), using all 32 output bits.
    pub fn next_f64(&mut self) -> f64 {
        f64::from(self.next_u32()) / 4_294_967_296.0
    }

    /// Deterministic Bernoulli draw; invalid probabilities are rejected.
    pub fn bernoulli(&mut self, probability: f64) -> Result<bool, String> {
        if !probability.is_finite() || !(0.0..=1.0).contains(&probability) {
            return Err("pcg.probability_out_of_range".into());
        }
        Ok(self.next_f64() < probability)
    }
}

/// Derive an independent, deterministic stream for a case ID. The SHA-256
/// domain separator and little-endian seed encoding are part of the contract.
pub fn for_case(global_seed: u64, case_id: &str) -> Result<Pcg32, String> {
    if case_id.trim().is_empty() {
        return Err("pcg.case_id_missing".into());
    }
    let mut hasher = Sha256::new();
    hasher.update(STREAM_DOMAIN);
    hasher.update(global_seed.to_le_bytes());
    hasher.update(case_id.as_bytes());
    let digest = hasher.finalize();
    let initial_state = u64::from_le_bytes(
        digest[0..8]
            .try_into()
            .map_err(|_| "pcg.seed_derivation_failed")?,
    );
    let initial_sequence = u64::from_le_bytes(
        digest[8..16]
            .try_into()
            .map_err(|_| "pcg.sequence_derivation_failed")?,
    );
    Ok(Pcg32::new(initial_state, initial_sequence))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_reference_pcg_xsh_rr_64_32_vector() {
        let mut rng = Pcg32::new(42, 54);
        assert_eq!(
            [
                rng.next_u32(),
                rng.next_u32(),
                rng.next_u32(),
                rng.next_u32(),
                rng.next_u32(),
                rng.next_u32(),
            ],
            [
                0xa15c02b7,
                0x7b47f409,
                0xba1d3330,
                0x83d2f293,
                0xbfa4784b,
                0xcbed606e,
            ]
        );
    }

    #[test]
    fn case_stream_is_reproducible_and_independent_of_call_order() {
        let mut first = for_case(6_008_001, "CC-001").unwrap();
        let mut other = for_case(6_008_001, "CC-002").unwrap();
        let expected: Vec<_> = (0..8).map(|_| first.next_u32()).collect();
        let _other_values: Vec<_> = (0..100).map(|_| other.next_u32()).collect();
        let mut repeated = for_case(6_008_001, "CC-001").unwrap();
        let actual: Vec<_> = (0..8).map(|_| repeated.next_u32()).collect();
        assert_eq!(actual, expected);
    }

    #[test]
    fn different_case_ids_do_not_share_the_same_stream() {
        let mut first = for_case(6_008_001, "CC-001").unwrap();
        let mut second = for_case(6_008_001, "CC-002").unwrap();
        let first_values: Vec<_> = (0..8).map(|_| first.next_u32()).collect();
        let second_values: Vec<_> = (0..8).map(|_| second.next_u32()).collect();
        assert_ne!(first_values, second_values);
    }

    #[test]
    fn bernoulli_validates_probability_and_respects_endpoints() {
        let mut rng = Pcg32::new(42, 54);
        assert!(!rng.bernoulli(0.0).unwrap());
        assert!(rng.bernoulli(1.0).unwrap());
        assert_eq!(
            rng.bernoulli(1.1).unwrap_err(),
            "pcg.probability_out_of_range"
        );
    }

    #[test]
    fn blank_case_id_is_rejected() {
        assert_eq!(for_case(1, " ").unwrap_err(), "pcg.case_id_missing");
    }
}
