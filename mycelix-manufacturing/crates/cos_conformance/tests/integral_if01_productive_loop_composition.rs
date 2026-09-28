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

    pub(super) fn empty_bindings() -> Bindings { Bindings { plan_execution:false, requirement_availability:false, assignment_observed_work:false, plan_consumption:false, output_qualification:false, foreign_recognition:false, itc_projection:false, frs_projection:false, recommendation_authorized:false, effect_recorded:false, quality_current:false, denominator_explicit:false, general_capability_evidence:false, failure_history_preserved:false, source_observation_created:false, physical_work_binding:false } }

    pub(super) fn evidence() -> Evidence { Evidence { id: "delivery-1", origin: Origin::Local, validity: crate::Validity { valid_from: 0, valid_until: None }, superseded: false, conflicting: false } }

    pub(super) fn envelope() -> super::integral_oad_cos_interface::DesignEnvelope {
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

    pub(super) fn admitted() -> InterfaceDecision {
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
            &empty_bindings(),
            &evidence(),
        );
        assert_eq!(witness.decision, crate::Decision::Rejected);
    }

    #[test]
    fn semantic_admission_does_not_become_qualified_output() {
        let witness = refine(
            Domain::Manufacturing,
            ProductiveLoopObligation::UsefulVsQualifiedOutput,
            &empty_bindings(),
            &evidence(),
        );
        assert_eq!(witness.decision, crate::Decision::Rejected);
    }

    #[test]
    fn semantic_admission_does_not_become_current_availability() {
        let witness = refine(
            Domain::Manufacturing,
            ProductiveLoopObligation::CapabilityVsAvailability,
            &empty_bindings(),
            &evidence(),
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
            &empty_bindings(),
            &evidence(),
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

    
#[cfg(test)]
mod mutation_tests {
    use crate::integral_oad_cos_interface::{
        admit_after_receipt, validate_envelope, AdmissionState, Authn, Authz,
        DeliveryOutcome, InterfaceDecision, InterfaceProfile, Receipt,
    };
    use super::tests::{admitted, empty_bindings, envelope};

    #[test]
    fn valid_authorization_cannot_rescue_stale_schema() {
        let mut e = envelope();
        e.schema_version = "oad-certified-design/0.0";
        assert_eq!(
            validate_envelope(
                &InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7,
                "oad-certified-design/0.1-draft", Some("authz-1"),
            ),
            InterfaceDecision::RejectedStaleSchema
        );
    }

    #[test]
    fn valid_authorization_cannot_rescue_wrong_authorization_reference() {
        let mut e = envelope();
        e.presented_authorization_ref = Some("authz-other");
        assert_eq!(
            validate_envelope(
                &InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7,
                "oad-certified-design/0.1-draft", Some("authz-1"),
            ),
            InterfaceDecision::RejectedAuthorizationReference
        );
    }

    #[test]
    fn duplicate_retry_with_new_attempt_id_remains_idempotent() {
        let e = envelope();
        let receipt = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-2", outcome: DeliveryOutcome::SemanticAdmitted };
        let state = AdmissionState { admitted_delivery: Some("delivery-1"), admitted_payload: Some("sha256:abc") };
        assert_eq!(admit_after_receipt(&state, &receipt, &e, admitted()), InterfaceDecision::DuplicateIdempotent);
    }

    #[test]
    fn retry_payload_mutation_cannot_reuse_logical_delivery() {
        let first = envelope();
        let mut retry = first.clone();
        retry.attempt_id = "attempt-2";
        retry.payload_commitment = "sha256:mutated";
        assert!(!crate::integral_oad_cos_interface::retry_compatible(&first, &retry));
    }

    #[test]
    fn known_rejection_then_retry_stays_explicitly_rejected() {
        let e = envelope();
        let rejected = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-1", outcome: DeliveryOutcome::Rejected };
        let state = AdmissionState { admitted_delivery: None, admitted_payload: None };
        assert_eq!(admit_after_receipt(&state, &rejected, &e, admitted()), InterfaceDecision::RejectedSemanticDelivery);
    }

    #[test]
    fn foreign_origin_survives_admission_and_cannot_become_local_origin() {
        let e = envelope();
        assert_eq!(crate::integral_oad_cos_interface::recognized_origin(&e), "foreign-node");
    }

    #[test]
    fn authorization_for_generation_n_does_not_admit_generation_n_plus_one() {
        let mut e = envelope();
        e.design_generation = 8;
        assert_eq!(
            validate_envelope(
                &InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7,
                "oad-certified-design/0.1-draft", Some("authz-1"),
            ),
            InterfaceDecision::RejectedStaleDesign
        );
    }

    #[test]
    fn certification_change_after_authorization_is_not_execution_evidence() {
        let mut e = envelope();
        e.certified = false;
        assert_eq!(
            validate_envelope(
                &InterfaceProfile::CANDIDATE_V1, &e, Authn::Valid, Authz::Granted, 7,
                "oad-certified-design/0.1-draft", Some("authz-1"),
            ),
            InterfaceDecision::RejectedCertification
        );
    }

    #[test]
    fn admission_cannot_mint_itc_frs_or_production_authority() {
        let e = envelope();
        let receipt = Receipt { logical_delivery_id: "delivery-1", attempt_id: "attempt-1", outcome: DeliveryOutcome::SemanticAdmitted };
        assert!(!crate::integral_oad_cos_interface::receipt_grants_production_authority(&receipt));
        let b = empty_bindings();
        assert!(!b.itc_projection);
        assert!(!b.frs_projection);
        assert!(!b.effect_recorded);
    }
}
