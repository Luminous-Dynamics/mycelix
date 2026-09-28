//! Deterministic scenario runner for Integral/Mycelix semantic seam mutations.
//!
//! This is intentionally a small scenario DSL: scenarios describe semantic
//! events, not network mechanics. A concrete runtime adapter can replay the
//! same scenario without changing its expected semantic outcomes.

use crate::seam_profile::{
    admit_after_receipt, recognized_origin, retry_compatible, validate_envelope,
    Receipt, ReceiptStage, SeamEnvelope, SeamProfile, SemanticDecision,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mutation {
    ProviderAccepted,
    SemanticAdmission,
    StaleSchema,
    DeliveryIdentityMismatch,
    RetrySameLogicalDelivery,
    RetryPayloadMutation,
    UnknownDelivery,
    ForeignRecognition,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScenarioStep {
    pub mutation: Mutation,
    pub expected: SemanticDecision,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScenarioResult {
    pub mutation: Mutation,
    pub actual: SemanticDecision,
    pub passed: bool,
}

pub fn run(profile: &SeamProfile, envelope: &SeamEnvelope, steps: &[ScenarioStep]) -> Vec<ScenarioResult> {
    steps.iter().map(|step| {
        let actual = match step.mutation {
            Mutation::ProviderAccepted => admit_after_receipt(profile, envelope, &Receipt {
                delivery_id: envelope.delivery_id.clone(),
                attempt_id: envelope.attempt_id.clone(),
                stage: ReceiptStage::TransportAccepted,
                semantic_decision: None,
            }),
            Mutation::SemanticAdmission => admit_after_receipt(profile, envelope, &Receipt {
                delivery_id: envelope.delivery_id.clone(),
                attempt_id: envelope.attempt_id.clone(),
                stage: ReceiptStage::RecipientSemanticallyAdmitted,
                semantic_decision: Some(SemanticDecision::Accepted),
            }),
            Mutation::StaleSchema => {
                let mut stale = envelope.clone();
                stale.profile_version = stale.profile_version.saturating_add(1);
                validate_envelope(profile, &stale)
            }
            Mutation::DeliveryIdentityMismatch => admit_after_receipt(profile, envelope, &Receipt {
                delivery_id: format!("{}-other", envelope.delivery_id),
                attempt_id: envelope.attempt_id.clone(),
                stage: ReceiptStage::RecipientSemanticallyAdmitted,
                semantic_decision: Some(SemanticDecision::Accepted),
            }),
            Mutation::RetrySameLogicalDelivery => {
                let mut retry = envelope.clone();
                retry.attempt_id = format!("{}-retry", retry.attempt_id);
                if retry_compatible(envelope, &retry) {
                    SemanticDecision::Indeterminate
                } else {
                    SemanticDecision::Rejected
                }
            }
            Mutation::RetryPayloadMutation => {
                let mut retry = envelope.clone();
                retry.attempt_id = format!("{}-retry", retry.attempt_id);
                retry.payload_commitment.push_str("-mutated");
                if retry_compatible(envelope, &retry) {
                    SemanticDecision::Accepted
                } else {
                    SemanticDecision::PayloadMismatch
                }
            }
            Mutation::UnknownDelivery => SemanticDecision::Indeterminate,
            Mutation::ForeignRecognition => {
                let origin = recognized_origin(&envelope.origin, &profile.receiver_domain);
                if origin.starts_with("foreign:") {
                    SemanticDecision::Accepted
                } else {
                    SemanticDecision::Rejected
                }
            }
        };
        ScenarioResult {
            mutation: step.mutation,
            actual,
            passed: actual == step.expected,
        }
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> SeamProfile {
        SeamProfile {
            profile_id: "INTEGRAL-REF-IF-001".into(),
            profile_version: 1,
            sender_domain: "OAD".into(),
            receiver_domain: "COS".into(),
            source_schema_version: "oad.v1".into(),
            delivery_mode: crate::seam_profile::DeliveryMode::Event,
            retry_idempotency_profile: "stable-delivery-id".into(),
            ordering_guaranteed: false,
        }
    }

    fn envelope() -> SeamEnvelope {
        SeamEnvelope {
            profile_id: "INTEGRAL-REF-IF-001".into(),
            profile_version: 1,
            semantic_subject_id: "design-1".into(),
            payload_commitment: "sha256:payload".into(),
            delivery_id: "delivery-1".into(),
            attempt_id: "attempt-1".into(),
            source_schema_version: "oad.v1".into(),
            origin: "node-a".into(),
            authority_reference: None,
        }
    }

    #[test]
    fn oracle_hidden_mutation_set_is_deterministic() {
        let steps = [
            ScenarioStep { mutation: Mutation::ProviderAccepted, expected: SemanticDecision::Indeterminate },
            ScenarioStep { mutation: Mutation::SemanticAdmission, expected: SemanticDecision::Accepted },
            ScenarioStep { mutation: Mutation::StaleSchema, expected: SemanticDecision::StaleSchema },
            ScenarioStep { mutation: Mutation::DeliveryIdentityMismatch, expected: SemanticDecision::Rejected },
            ScenarioStep { mutation: Mutation::RetrySameLogicalDelivery, expected: SemanticDecision::Indeterminate },
            ScenarioStep { mutation: Mutation::RetryPayloadMutation, expected: SemanticDecision::PayloadMismatch },
            ScenarioStep { mutation: Mutation::UnknownDelivery, expected: SemanticDecision::Indeterminate },
            ScenarioStep { mutation: Mutation::ForeignRecognition, expected: SemanticDecision::Accepted },
        ];
        let results = run(&profile(), &envelope(), &steps);
        assert!(results.iter().all(|r| r.passed));
    }

    #[test]
    fn transport_acceptance_cannot_close_effect() {
        let result = run(&profile(), &envelope(), &[
            ScenarioStep { mutation: Mutation::ProviderAccepted, expected: SemanticDecision::Indeterminate },
        ]);
        assert_eq!(result[0].actual, SemanticDecision::Indeterminate);
    }
}
