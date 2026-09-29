//! D6X layered verification ceilings and partial-proof integrity.
//!
//! Status: ReferenceModelOnly.
//!
//! D6X makes partial verification explicit. A verifier may verify a lower
//! commitment layer without possessing all higher layers, but the resulting
//! claim scope MUST stop at the highest layer actually verified.
//!
//! Governing law:
//!
//!     verified scope <= supplied verified layers
//!
//! A higher-layer commitment is never a substitute for a missing lower layer.

use crate::derivation_commitment_layers::{
    verify_d6w_receipt, D6WDerivationLayerV1, D6WInputLayerV1, D6WReceiptV1,
    D6WResultLayerV1,
};
use crate::canonical_derivation_receipt::{
    CanonicalDerivationReceiptV1, DerivationProfileV1, QualifiedProjectionV1,
    SemanticEnvironmentV1,
};
use crate::finality_eligibility_composition::CurrentFinalityEligibilityReceiptV1;
use serde::{Deserialize, Serialize};

pub const D6X_CLAIM_CEILING: &str =
    "ReferenceModelOnly; layered verification scope only; no semantic truth, authority, authorization, or actuation claim.";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum D6XVerificationScopeV1 {
    InputOnly,
    InputAndDerivation,
    Result,
    FullReceipt,
}

impl D6XVerificationScopeV1 {
    fn rank(self) -> u8 {
        match self {
            Self::InputOnly => 1,
            Self::InputAndDerivation => 2,
            Self::Result => 3,
            Self::FullReceipt => 4,
        }
    }

    pub fn is_no_stronger_than(self, other: Self) -> bool {
        self.rank() <= other.rank()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum D6XVerificationDispositionV1 {
    Verified,
    BlockedMissingInput,
    BlockedMissingDerivation,
    BlockedMissingResult,
    BlockedBinding,
    BlockedD6SReceipt,
    InvalidLayer,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct D6XVerificationReceiptV1 {
    pub verification_id: String,
    pub scope: D6XVerificationScopeV1,
    pub input_commitment: Option<String>,
    pub derivation_commitment: Option<String>,
    pub result_commitment: Option<String>,
    pub d6s_receipt_commitment: Option<String>,
    pub disposition: D6XVerificationDispositionV1,
    pub claim_ceiling: String,
    pub verification_commitment: String,
}

impl D6XVerificationReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        !self.verification_id.trim().is_empty()
            && self.claim_ceiling == D6X_CLAIM_CEILING
            && !self.verification_commitment.trim().is_empty()
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.verification_commitment.clear();
        crate::canonical_derivation_receipt::canonical_sha256("d6x-verification", &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid()
            && self.verification_commitment == self.recomputed_commitment()
    }
}

fn verified_receipt(
    verification_id: &str,
    scope: D6XVerificationScopeV1,
    input_commitment: Option<String>,
    derivation_commitment: Option<String>,
    result_commitment: Option<String>,
    d6s_receipt_commitment: Option<String>,
    disposition: D6XVerificationDispositionV1,
) -> D6XVerificationReceiptV1 {
    let mut receipt = D6XVerificationReceiptV1 {
        verification_id: verification_id.into(),
        scope,
        input_commitment,
        derivation_commitment,
        result_commitment,
        d6s_receipt_commitment,
        disposition,
        claim_ceiling: D6X_CLAIM_CEILING.into(),
        verification_commitment: String::new(),
    };
    receipt.verification_commitment = receipt.recomputed_commitment();
    receipt
}

pub fn verify_input_scope(
    input: Option<&D6WInputLayerV1>,
    expected_input_commitment: &str,
) -> D6XVerificationDispositionV1 {
    let Some(input) = input else {
        return D6XVerificationDispositionV1::BlockedMissingInput;
    };
    if !input.structurally_valid() {
        return D6XVerificationDispositionV1::InvalidLayer;
    }
    if input.commitment() != expected_input_commitment {
        return D6XVerificationDispositionV1::BlockedBinding;
    }
    D6XVerificationDispositionV1::Verified
}

pub fn verify_derivation_scope(
    input: Option<&D6WInputLayerV1>,
    derivation: Option<&D6WDerivationLayerV1>,
    expected_input_commitment: &str,
    expected_derivation_commitment: &str,
) -> D6XVerificationDispositionV1 {
    match verify_input_scope(input, expected_input_commitment) {
        D6XVerificationDispositionV1::Verified => {}
        disposition => return disposition,
    }
    let Some(derivation) = derivation else {
        return D6XVerificationDispositionV1::BlockedMissingDerivation;
    };
    if !derivation.structurally_valid() {
        return D6XVerificationDispositionV1::InvalidLayer;
    }
    if derivation.input_commitment != expected_input_commitment
        || derivation.commitment() != expected_derivation_commitment
    {
        return D6XVerificationDispositionV1::BlockedBinding;
    }
    D6XVerificationDispositionV1::Verified
}

pub fn verify_result_scope(
    input: Option<&D6WInputLayerV1>,
    derivation: Option<&D6WDerivationLayerV1>,
    result: Option<&D6WResultLayerV1>,
    expected_input_commitment: &str,
    expected_derivation_commitment: &str,
    expected_result_commitment: &str,
) -> D6XVerificationDispositionV1 {
    match verify_derivation_scope(
        input,
        derivation,
        expected_input_commitment,
        expected_derivation_commitment,
    ) {
        D6XVerificationDispositionV1::Verified => {}
        disposition => return disposition,
    }
    let Some(result) = result else {
        return D6XVerificationDispositionV1::BlockedMissingResult;
    };
    if !result.structurally_valid() {
        return D6XVerificationDispositionV1::InvalidLayer;
    }
    if result.derivation_commitment != expected_derivation_commitment
        || result.commitment() != expected_result_commitment
    {
        return D6XVerificationDispositionV1::BlockedBinding;
    }
    D6XVerificationDispositionV1::Verified
}

pub fn verify_full_scope(
    input: Option<&D6WInputLayerV1>,
    derivation: Option<&D6WDerivationLayerV1>,
    result: Option<&D6WResultLayerV1>,
    receipt: Option<&D6WReceiptV1>,
    d6s_receipt: &CanonicalDerivationReceiptV1,
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
    profile: &DerivationProfileV1,
    current_receipts: &[CurrentFinalityEligibilityReceiptV1],
    execution_trace_commitment: &str,
) -> D6XVerificationDispositionV1 {
    match verify_result_scope(
        input,
        derivation,
        result,
        receipt
            .map(|r| r.input_commitment.as_str())
            .unwrap_or_default(),
        receipt
            .map(|r| r.derivation_commitment.as_str())
            .unwrap_or_default(),
        receipt
            .map(|r| r.result_commitment.as_str())
            .unwrap_or_default(),
    ) {
        D6XVerificationDispositionV1::Verified => {}
        disposition => return disposition,
    }
    let Some(receipt) = receipt else {
        return D6XVerificationDispositionV1::BlockedD6SReceipt;
    };
    if !verify_d6w_receipt(
        receipt,
        d6s_receipt,
        projection,
        environment,
        profile,
        current_receipts,
        execution_trace_commitment,
    ) {
        return D6XVerificationDispositionV1::BlockedD6SReceipt;
    }
    D6XVerificationDispositionV1::Verified
}

pub fn build_verification_receipt(
    verification_id: &str,
    scope: D6XVerificationScopeV1,
    input: Option<&D6WInputLayerV1>,
    derivation: Option<&D6WDerivationLayerV1>,
    result: Option<&D6WResultLayerV1>,
    d6s_receipt_commitment: Option<&str>,
    disposition: D6XVerificationDispositionV1,
) -> D6XVerificationReceiptV1 {
    verified_receipt(
        verification_id,
        scope,
        input.map(|v| v.commitment()),
        derivation.map(|v| v.commitment()),
        result.map(|v| v.commitment()),
        d6s_receipt_commitment.map(str::to_owned),
        disposition,
    )
}

pub fn d6x_scope_cannot_create_authorization(
    receipt: &D6XVerificationReceiptV1,
) -> bool {
    receipt.structurally_valid()
        && receipt.claim_ceiling == D6X_CLAIM_CEILING
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::canonical_derivation_receipt::{
        build_canonical_receipt, D6S_CLAIM_CEILING, D6S_REFERENCE_CANONICALIZATION_VERSION,
        DerivationResultStatusV1, QualifiedEdgeV1, QualifiedNodeV1,
    };
    use crate::derivation_commitment_layers::{
        D6WInputLayerV1, D6WDerivationLayerV1, D6WResultLayerV1,
        build_d6w_receipt,
    };
    use crate::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
    use crate::finality_eligibility_composition::{
        FinalityEligibilityDispositionV1, FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING,
    };
    use std::collections::{BTreeMap, BTreeSet};

    fn env() -> SemanticEnvironmentV1 {
        SemanticEnvironmentV1 {
            semantic_profile_id: "integral".into(),
            semantic_profile_version: "1".into(),
            current_frontier_root: Some("frontier-1".into()),
            d6p_eligibility_context_root: Some("d6p-context-1".into()),
            d6n_observer_context_root: Some("d6n-1".into()),
            d6o_lifecycle_context_root: Some("d6o-1".into()),
            membership_authority_scope_root: Some("membership-1".into()),
            dependency_snapshot_root: Some("deps-1".into()),
            historical_cutoff: Some(100),
            policy_version: "policy-1".into(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        }
    }

    fn profile() -> DerivationProfileV1 {
        DerivationProfileV1 {
            profile_id: "support-v1".into(),
            version: "1".into(),
            rule_ids: ["support-v1".into()].into_iter().collect(),
            permits_recursive_fixpoint: false,
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        }
    }

    fn projection() -> QualifiedProjectionV1 {
        let mut nodes = BTreeMap::new();
        for (id, kind) in [
            ("e", ClaimGraphNodeKindV1::Evidence),
            ("a", ClaimGraphNodeKindV1::Assessment),
            ("c", ClaimGraphNodeKindV1::Conclusion),
        ] {
            nodes.insert(
                id.into(),
                QualifiedNodeV1 {
                    node_id: id.into(),
                    kind,
                    node_commitment: format!("node-{id}"),
                    historical_only: false,
                    current_frontier_root: Some("frontier-1".into()),
                    claim_ceiling: D6S_CLAIM_CEILING.into(),
                },
            );
        }
        let mut edges = BTreeMap::new();
        edges.insert(
            "s".into(),
            QualifiedEdgeV1 {
                edge_id: "s".into(),
                from_node_id: "a".into(),
                to_node_id: "c".into(),
                kind: ClaimGraphEdgeKindV1::Supports,
                edge_commitment: "edge-s".into(),
                claim_ceiling: D6S_CLAIM_CEILING.into(),
            },
        );
        let e = env();
        let p = profile();
        QualifiedProjectionV1 {
            projection_id: "projection-1".into(),
            projection_version: "1".into(),
            canonicalization_version: D6S_REFERENCE_CANONICALIZATION_VERSION.into(),
            source_dkg_snapshot_commitment: "dkg-1".into(),
            nodes,
            edges,
            d6p_current_receipt_commitments: ["d6p-1".into()].into_iter().collect(),
            d6n_context_commitment: Some("d6n-1".into()),
            d6o_context_commitment: Some("d6o-1".into()),
            semantic_environment_commitment: e.commitment(),
            derivation_profile_commitment: p.commitment(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        }
    }

    fn d6p_receipt() -> CurrentFinalityEligibilityReceiptV1 {
        CurrentFinalityEligibilityReceiptV1 {
            receipt_id: "d6p-1".into(),
            effect_id: "effect-1".into(),
            effect_lineage_id: "lineage-1".into(),
            lifecycle_generation_id: "generation-1".into(),
            route_id: "route-1".into(),
            provider_id: "provider-1".into(),
            provider_operation_id: "operation-1".into(),
            provider_profile_root: "provider-profile-1".into(),
            semantic_environment_root: "env-1".into(),
            observation_set_id: "set-1".into(),
            observation_set_commitment: "set-1".into(),
            d6n_assessment_commitment: "assessment-1".into(),
            witness_eligibility_ids: ["eligibility-1".into()].into_iter().collect(),
            observer_generation_ids: ["observer-generation-1".into()].into_iter().collect(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-1".into(),
            eligible_independent_count: 2,
            preserved_contradictory_count: 0,
            disposition: FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition-1".into(),
            receipt_commitment: "d6p-1".into(),
            claim_ceiling: FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        }
    }

    fn layers() -> (D6WInputLayerV1, D6WDerivationLayerV1, D6WResultLayerV1, D6WReceiptV1) {
        let d6p = d6p_receipt();
        let d6s = build_canonical_receipt(
            &projection(),
            &env(),
            &profile(),
            &[d6p.clone()],
            DerivationResultStatusV1::Supported,
            "result-1".into(),
            false,
            false,
        ).unwrap();
        build_d6w_receipt(
            &d6s,
            &projection(),
            &env(),
            &profile(),
            &[d6p],
            "trace-1".into(),
        ).unwrap()
    }

    #[test]
    fn input_only_verification_stops_at_input() {
        let (input, _, _, _) = layers();
        let disposition = verify_input_scope(Some(&input), &input.commitment());
        assert_eq!(disposition, D6XVerificationDispositionV1::Verified);
        let receipt = build_verification_receipt(
            "verification-input",
            D6XVerificationScopeV1::InputOnly,
            Some(&input),
            None,
            None,
            None,
            disposition,
        );
        assert_eq!(receipt.scope, D6XVerificationScopeV1::InputOnly);
        assert!(receipt.derivation_commitment.is_none());
        assert!(d6x_scope_cannot_create_authorization(&receipt));
    }

    #[test]
    fn derivation_requires_exact_input_layer() {
        let (input, derivation, _, _) = layers();
        let mut changed = input.clone();
        changed.projection_version = "2".into();
        let disposition = verify_derivation_scope(
            Some(&changed),
            Some(&derivation),
            &input.commitment(),
            &derivation.commitment(),
        );
        assert_eq!(disposition, D6XVerificationDispositionV1::BlockedBinding);
    }

    #[test]
    fn derivation_scope_succeeds_with_exact_lower_layer() {
        let (input, derivation, _, _) = layers();
        assert_eq!(
            verify_derivation_scope(
                Some(&input),
                Some(&derivation),
                &input.commitment(),
                &derivation.commitment(),
            ),
            D6XVerificationDispositionV1::Verified
        );
    }

    #[test]
    fn result_requires_derivation_layer() {
        let (input, derivation, result, _) = layers();
        assert_eq!(
            verify_result_scope(
                Some(&input),
                None,
                Some(&result),
                &input.commitment(),
                &derivation.commitment(),
                &result.commitment(),
            ),
            D6XVerificationDispositionV1::BlockedMissingDerivation
        );
    }

    #[test]
    fn result_scope_succeeds_with_exact_chain() {
        let (input, derivation, result, _) = layers();
        assert_eq!(
            verify_result_scope(
                Some(&input),
                Some(&derivation),
                Some(&result),
                &input.commitment(),
                &derivation.commitment(),
                &result.commitment(),
            ),
            D6XVerificationDispositionV1::Verified
        );
    }

    #[test]
    fn higher_layer_commitment_cannot_substitute_for_missing_input() {
        let (_, derivation, result, receipt) = layers();
        assert_eq!(
            verify_result_scope(
                None,
                Some(&derivation),
                Some(&result),
                &receipt.input_commitment,
                &receipt.derivation_commitment,
                &receipt.result_commitment,
            ),
            D6XVerificationDispositionV1::BlockedMissingInput
        );
    }

    #[test]
    fn full_scope_requires_d6s_receipt_verification() {
        let (input, derivation, result, receipt) = layers();
        let d6p = d6p_receipt();
        let d6s = build_canonical_receipt(
            &projection(),
            &env(),
            &profile(),
            &[d6p.clone()],
            DerivationResultStatusV1::Supported,
            "result-1".into(),
            false,
            false,
        ).unwrap();
        assert_eq!(
            verify_full_scope(
                Some(&input),
                Some(&derivation),
                Some(&result),
                Some(&receipt),
                &d6s,
                &projection(),
                &env(),
                &profile(),
                &[d6p],
                "trace-1",
            ),
            D6XVerificationDispositionV1::Verified
        );
    }

    #[test]
    fn full_scope_rejects_tampered_d6s_receipt() {
        let (input, derivation, result, receipt) = layers();
        let d6p = d6p_receipt();
        let mut d6s = build_canonical_receipt(
            &projection(),
            &env(),
            &profile(),
            &[d6p.clone()],
            DerivationResultStatusV1::Supported,
            "result-1".into(),
            false,
            false,
        ).unwrap();
        d6s.result_commitment = "tampered".into();
        assert_eq!(
            verify_full_scope(
                Some(&input),
                Some(&derivation),
                Some(&result),
                Some(&receipt),
                &d6s,
                &projection(),
                &env(),
                &profile(),
                &[d6p],
                "trace-1",
            ),
            D6XVerificationDispositionV1::BlockedD6SReceipt
        );
    }

    #[test]
    fn verification_receipt_is_deterministic() {
        let (input, derivation, result, _) = layers();
        let a = build_verification_receipt(
            "verification-1",
            D6XVerificationScopeV1::Result,
            Some(&input),
            Some(&derivation),
            Some(&result),
            None,
            D6XVerificationDispositionV1::Verified,
        );
        let b = build_verification_receipt(
            "verification-1",
            D6XVerificationScopeV1::Result,
            Some(&input),
            Some(&derivation),
            Some(&result),
            None,
            D6XVerificationDispositionV1::Verified,
        );
        assert_eq!(a, b);
        assert!(a.commitment_matches());
    }

    #[test]
    fn verification_scope_is_monotone_only_when_layers_are_supplied() {
        assert!(D6XVerificationScopeV1::InputOnly.is_no_stronger_than(
            D6XVerificationScopeV1::FullReceipt
        ));
        assert!(!D6XVerificationScopeV1::FullReceipt.is_no_stronger_than(
            D6XVerificationScopeV1::InputOnly
        ));
    }

    #[test]
    fn missing_input_is_not_treated_as_rejection_of_semantics() {
        let disposition = verify_input_scope(None, "input-1");
        assert_eq!(
            disposition,
            D6XVerificationDispositionV1::BlockedMissingInput
        );
    }

    #[test]
    fn symthaea_boundary_is_claim_bounded() {
        let (input, _, _, _) = layers();
        let receipt = build_verification_receipt(
            "verification-symthaea",
            D6XVerificationScopeV1::InputOnly,
            Some(&input),
            None,
            None,
            None,
            D6XVerificationDispositionV1::Verified,
        );
        assert!(d6x_scope_cannot_create_authorization(&receipt));
    }
}
