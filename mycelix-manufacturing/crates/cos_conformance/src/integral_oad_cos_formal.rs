//! IF01 formal-refinement manifest for the candidate OAD -> COS seam.
//!
//! This is a machine-readable obligation map, not a proof claim. Each theorem
//! remains open until a formal prover artifact and production refinement witness
//! are attached. The executable tests in integral_oad_cos_interface.rs are
//! counterexample/conformance witnesses only.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClosureState {
    Open,
    BoundedExecutableWitness,
    FormallyClosed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obligation {
    pub id: &'static str,
    pub proposition: &'static str,
    pub witness: &'static str,
    pub claim_ceiling: &'static str,
    pub state: ClosureState,
}

pub const OBLIGATIONS: [Obligation; 12] = [
    Obligation {
        id: "IF01-FV-001",
        proposition: "Authentication evidence does not imply authorization.",
        witness: "valid_authentication_and_authorization_are_separate",
        claim_ceiling: "Authentication/authorization semantic separation only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-002",
        proposition: "A schema version different from the admitted schema cannot be semantically admitted.",
        witness: "stale_schema_fails_closed",
        claim_ceiling: "Schema-currentness semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-003",
        proposition: "Transport acceptance does not imply semantic admission.",
        witness: "transport_acceptance_is_not_semantic_admission",
        claim_ceiling: "Receipt semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-004",
        proposition: "A retry preserves logical delivery identity while attempt identity may change.",
        witness: "retry_may_change_attempt_but_not_logical_delivery_or_payload",
        claim_ceiling: "Retry identity semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-005",
        proposition: "A stable logical delivery cannot be reused with a mutated payload.",
        witness: "same_logical_delivery_with_mutated_payload_is_rejected",
        claim_ceiling: "Replay/mutation semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-006",
        proposition: "Repeated semantic admission of the same logical delivery and payload is idempotent.",
        witness: "duplicate_semantic_admission_is_idempotent",
        claim_ceiling: "Reference-model idempotency semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-007",
        proposition: "An indeterminate delivery receipt cannot be promoted to a definite failure.",
        witness: "unknown_receipt_never_becomes_definite_failure",
        claim_ceiling: "Unknown-state semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-008",
        proposition: "Recognition does not rewrite the originating node.",
        witness: "foreign_origin_is_preserved",
        claim_ceiling: "Federation provenance semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-010",
        proposition: "Certification failure is distinct from authorization failure.",
        witness: "uncertified_design_is_rejected_as_certification",
        claim_ceiling: "Certification-state semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-011",
        proposition: "Granted authorization requires a matching explicit authorization reference.",
        witness: "wrong_or_missing_authorization_reference_is_rejected",
        claim_ceiling: "Authorization-reference binding semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-012",
        proposition: "A known recipient semantic rejection is not indeterminate delivery.",
        witness: "known_recipient_rejection_is_distinguished_from_unknown_delivery",
        claim_ceiling: "Delivery outcome classification only.",
        state: ClosureState::BoundedExecutableWitness,
    },
    Obligation {
        id: "IF01-FV-009",
        proposition: "A transport or semantic receipt does not itself mint production authority.",
        witness: "receipts_never_grant_production_authority",
        claim_ceiling: "Authority-boundary semantics only.",
        state: ClosureState::BoundedExecutableWitness,
    },
];

pub fn manifest_is_explicitly_open() -> bool {
    OBLIGATIONS.iter().all(|o| o.state != ClosureState::FormallyClosed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_nine_obligations_have_executable_witnesses_and_are_not_overclaimed() {
        assert_eq!(OBLIGATIONS.len(), 12);
        assert!(manifest_is_explicitly_open());
        for obligation in OBLIGATIONS {
            assert!(!obligation.id.is_empty());
            assert!(!obligation.proposition.is_empty());
            assert!(!obligation.witness.is_empty());
            assert!(!obligation.claim_ceiling.is_empty());
            assert_ne!(obligation.state, ClosureState::FormallyClosed);
        }
    }

    #[test]
    fn no_scalar_score_can_close_if01() {
        assert!(manifest_is_explicitly_open());
    }
}
