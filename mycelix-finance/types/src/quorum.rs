//! Pure weighted-quorum arithmetic for scarce-resource protocols.
//!
//! This module intentionally does NOT verify signatures, witness identity,
//! Sybil resistance, or live Holochain state. It only implements the
//! arithmetic boundary used by a separately qualified quorum protocol.

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WeightedQuorumPolicy {
    pub total_weight: u64,
    pub max_byzantine_weight: u64,
}

impl WeightedQuorumPolicy {
    pub fn validate(&self) -> Result<(), QuorumArithmeticError> {
        if self.total_weight == 0 {
            return Err(QuorumArithmeticError::ZeroTotalWeight);
        }
        if self.max_byzantine_weight * 3 >= self.total_weight * 3 {
            return Err(QuorumArithmeticError::InvalidFaultBound);
        }
        if self.max_byzantine_weight >= self.total_weight / 3
            && self.total_weight % 3 == 0
        {
            return Err(QuorumArithmeticError::InvalidFaultBound);
        }
        Ok(())
    }

    /// Minimum integer voting weight strictly greater than two thirds.
    pub fn supermajority_threshold(&self) -> u64 {
        two_thirds_plus_one(self.total_weight)
    }

    /// Whether a certificate has strictly more than two thirds of weight.
    pub fn has_supermajority(&self, certified_weight: u64) -> bool {
        certified_weight >= self.supermajority_threshold()
    }

    /// Whether a certificate exceeds the declared exposure-independent fault
    /// boundary. This is arithmetic only and does not prove witness honesty.
    pub fn has_declared_honest_intersection(&self, certified_weight: u64) -> bool {
        self.validate().is_ok()
            && self.has_supermajority(certified_weight)
            && certified_weight <= self.total_weight
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QuorumArithmeticError {
    ZeroTotalWeight,
    InvalidFaultBound,
}

fn two_thirds_plus_one(total: u64) -> u64 {
    let thirds = total / 3;
    match total % 3 {
        0 | 1 => thirds.saturating_mul(2).saturating_add(1),
        2 => thirds.saturating_mul(2).saturating_add(2),
        _ => unreachable!(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn threshold_is_strictly_above_two_thirds() {
        assert_eq!(two_thirds_plus_one(3), 3);
        assert_eq!(two_thirds_plus_one(4), 3);
        assert_eq!(two_thirds_plus_one(5), 4);
        assert_eq!(two_thirds_plus_one(6), 5);
        assert_eq!(two_thirds_plus_one(7), 5);
        assert_eq!(two_thirds_plus_one(8), 6);
    }

    #[test]
    fn supermajority_boundary_is_fail_closed() {
        let p = WeightedQuorumPolicy {
            total_weight: 6,
            max_byzantine_weight: 1,
        };
        assert!(!p.has_supermajority(4));
        assert!(p.has_supermajority(5));
    }

    #[test]
    fn certificate_cannot_exceed_total_weight() {
        let p = WeightedQuorumPolicy {
            total_weight: 10,
            max_byzantine_weight: 3,
        };
        assert!(!p.has_declared_honest_intersection(11));
    }

    #[test]
    fn zero_total_weight_is_invalid() {
        let p = WeightedQuorumPolicy {
            total_weight: 0,
            max_byzantine_weight: 0,
        };
        assert_eq!(p.validate(), Err(QuorumArithmeticError::ZeroTotalWeight));
    }

    #[test]
    fn fault_bound_is_strictly_below_one_third() {
        let divisible = WeightedQuorumPolicy {
            total_weight: 9,
            max_byzantine_weight: 3,
        };
        assert_eq!(
            divisible.validate(),
            Err(QuorumArithmeticError::InvalidFaultBound)
        );

        let remainder = WeightedQuorumPolicy {
            total_weight: 10,
            max_byzantine_weight: 3,
        };
        assert!(remainder.validate().is_ok());
    }

    #[test]
    fn arithmetic_does_not_prove_signature_validity() {
        let p = WeightedQuorumPolicy {
            total_weight: 10,
            max_byzantine_weight: 3,
        };
        assert!(p.has_supermajority(7));
        // The caller must still authenticate every witness separately.
    }
}
