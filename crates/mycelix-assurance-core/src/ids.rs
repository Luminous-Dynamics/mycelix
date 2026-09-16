//! Normative logical identifiers.

/// Errors constructing normative identifiers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IdError {
    /// Zero is reserved and cannot identify a normative node.
    ReservedZero,
}

/// Intra-capsule logical node identifier.
///
/// A `NodeId` is not a content digest and has meaning only within the capsule
/// that contains it.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct NodeId(u64);

impl NodeId {
    /// Construct a nonzero node identifier.
    pub const fn new(value: u64) -> Result<Self, IdError> {
        if value == 0 {
            Err(IdError::ReservedZero)
        } else {
            Ok(Self(value))
        }
    }

    /// Return the numeric identifier.
    pub const fn get(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_is_reserved() {
        assert_eq!(NodeId::new(0), Err(IdError::ReservedZero));
    }

    #[test]
    fn nonzero_id_round_trips() {
        let id = NodeId::new(7).expect("nonzero ID must be valid");
        assert_eq!(id.get(), 7);
    }
}
