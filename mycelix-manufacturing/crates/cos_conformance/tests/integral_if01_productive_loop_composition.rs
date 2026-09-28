//! Composition-level adversarial witnesses for the candidate IF01 seam.
//!
//! These tests intentionally cross the IF01 admission boundary into the existing
//! COS/ProductiveLoop semantic vocabulary. They do not claim runtime integration;
//! they prove that a successful seam receipt cannot be silently reinterpreted as
//! execution, observation, qualification, or authority.

#[cfg(test)]
mod tests {
    use crate::integral_oad_cos_interface::{
        admit_after_receipt, validate_envelope, AdmissionState, Authn, Authz,
        DeliveryOutcome, InterfaceDecision, InterfaceProfile, Receipt,
    };
    use crate::productive_loop::{refine, Domain, ProductiveLoopObligation};
    use crate::{Evidence, Origin, Bindings};

    fn envelope() -> super::integral_oad_cos_interface::DesignEnvelope {
        super::integral_oad_cos_interface::DesignEnvelope {
            design_id: "design-1",
            design_generation: 7,
            schema_version: "oad-certified-design/0.1-draft",
            certified: true,
            superseded: false,
            logical_delivery_id: "delivery-1",
            attempt_id: "attempt-1",
            payload_commitment: "sha256:abc",
            origin_node: "foreign-node",
            authorization_ref: Some("authz-1"),
            presented_authorization_ref: Some("authz-1"),
        }
    }

    fn admitted() -> InterfaceDecision {
        validate_envelope(
            &InterfaceProfile::CANDIDATE_V1,
            &envelope(),
            Authn::Valid,
            Authz::Granted,
            7,
            "oad-certified-design/0.1-draft",
            Some("authz-1"),
        )
    }

    #[test]
    fn semantic_admission_does_not_become_observed_work() {
        let e = envelope();
        let receipt = Receipt {
            logical_delivery_id: "delivery-1",
            attempt_id: "attempt-1",
            outcome: DeliveryOutcome::SemanticAdmitted,
        };
        let state = AdmissionState {
            admitted_delivery: None,
            admitted_payload: None,
        };
        assert_eq!(
            admit_after_receipt(&state, &receipt, &e, admitted()),
            InterfaceDecision::Admitted
        );

        let witness = refine(
            Domain::Manufacturing,
            ProductiveLoopObligation::DeclaredWorkVsObservedWork,
            &Bindings::default(),
            &Evidence { id: "delivery-1", origin: Origin::Local, validity: crate::Validity { valid_from: 0, valid_until: None }, superseded: false, conflicting: false },
        );
        assert_eq!(witness.decision, crate::Decision::Rejected);
    }

    #[test]
    fn semantic_admission_does_not_become_qualified_output() {
        let witness = refine(
            Domain::Manufacturing,
            ProductiveLoopObligation::UsefulVsQualifiedOutput,
            &Bindings::default(),
            &Evidence { id: "delivery-1", origin: Origin::Local, validity: crate::Validity { valid_from: 0, valid_until: None }, superseded: false, conflicting: false },
        );
        assert_eq!(witness.decision, crate::Decision::Rejected);
    }

    #[test]
    fn semantic_admission_does_not_become_current_availability() {
        let witness = refine(
            Domain::Manufacturing,
            ProductiveLoopObligation::CapabilityVsAvailability,
            &Bindings::default(),
            &Evidence { id: "delivery-1", origin: Origin::Local, validity: crate::Validity { valid_from: 0, valid_until: None }, superseded: false, conflicting: false },
        );
        assert_eq!(witness.decision, crate::Decision::Rejected);
    }

    #[test]
    fn semantic_admission_does_not_become_n2_or_authority() {
        let e = envelope();
        let receipt = Receipt {
            logical_delivery_id: "delivery-1",
            attempt_id: "attempt-1",
            outcome: DeliveryOutcome::SemanticAdmitted,
        };
        assert!(!crate::integral_oad_cos_interface::receipt_grants_production_authority(&receipt));
        assert_eq!(
            admit_after_receipt(
                &AdmissionState { admitted_delivery: None, admitted_payload: None },
                &receipt,
                &e,
                admitted()
            ),
            InterfaceDecision::Admitted
        );
        let witness = refine(
            Domain::Manufacturing,
            ProductiveLoopObligation::ProductiveClosureVsN2,
            &Bindings::default(),
            &Evidence { id: "delivery-1", origin: Origin::Local, validity: crate::Validity { valid_from: 0, valid_until: None }, superseded: false, conflicting: false },
        );
        assert_eq!(witness.decision, crate::Decision::Rejected);
    }

    #[test]
    fn timeout_does_not_become_execution_failure_or_success() {
        let e = envelope();
        let receipt = Receipt {
            logical_delivery_id: "delivery-1",
            attempt_id: "attempt-1",
            outcome: DeliveryOutcome::Indeterminate,
        };
        assert_eq!(
            admit_after_receipt(
                &AdmissionState { admitted_delivery: None, admitted_payload: None },
                &receipt,
                &e,
                admitted()
            ),
            InterfaceDecision::Indeterminate
        );
    }
}
