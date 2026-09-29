//! D6W layered commitment reference model.
//!
//! Status: ReferenceModelOnly.
//!
//! D6W decomposes a D6S canonical derivation receipt into three diagnostic
//! commitment layers:
//!
//!     C_input
//!       -> C_derivation
//!         -> C_result
//!           -> C_receipt
//!
//! The decomposition does not add semantic authority. It only makes it
//! possible to identify whether a mutation changed qualified inputs,
//! derivation configuration/execution, or the result.
//!
//! Governing law:
//!
//!     input mutation   -> input + downstream commitments change
//!     derivation mutation -> derivation + downstream commitments change
//!     result mutation  -> result + receipt commitments change
//!
//! The final receipt remains ReferenceModelOnly.

use crate::canonical_derivation_receipt::{
    canonical_sha256, verify_canonical_receipt, CanonicalDerivationReceiptV1,
    DerivationProfileV1, DerivationResultStatusV1, QualifiedProjectionV1,
    SemanticEnvironmentV1, D6S_CLAIM_CEILING,
};
use crate::finality_eligibility_composition::CurrentFinalityEligibilityReceiptV1;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const D6W_CLAIM_CEILING: &str =
    "ReferenceModelOnly; layered D6S commitment decomposition only; no truth, authority, authorization, or actuation claim.";
pub const D6W_DOMAIN_INPUT: &str = "d6w-input";
pub const D6W_DOMAIN_DERIVATION: &str = "d6w-derivation";
pub const D6W_DOMAIN_RESULT: &str = "d6w-result";
pub const D6W_DOMAIN_RECEIPT: &str = "d6w-receipt";

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct D6WInputLayerV1 {
    pub source_dkg_snapshot_commitment: String,
    pub projection_id: String,
    pub projection_version: String,
    pub input_node_commitments: BTreeSet<String>,
    pub input_edge_commitments: BTreeSet<String>,
    pub d6p_current_receipt_commitments: BTreeSet<String>,
    pub d6n_context_commitment: Option<String>,
    pub d6o_context_commitment: Option<String>,
    pub semantic_environment_commitment: String,
    pub claim_ceiling: String,
}

impl D6WInputLayerV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.source_dkg_snapshot_commitment)
            && non_empty(&self.projection_id)
            && non_empty(&self.projection_version)
            && !self.input_node_commitments.is_empty()
            && !self.input_edge_commitments.is_empty()
            && self.input_node_commitments.iter().all(|v| non_empty(v))
            && self.input_edge_commitments.iter().all(|v| non_empty(v))
            && self
                .d6p_current_receipt_commitments
                .iter()
                .all(|v| non_empty(v))
            && self
                .d6n_context_commitment
                .as_deref()
                .map_or(true, non_empty)
            && self
                .d6o_context_commitment
                .as_deref()
                .map_or(true, non_empty)
            && non_empty(&self.semantic_environment_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
    }

    pub fn commitment(&self) -> String {
        canonical_sha256(D6W_DOMAIN_INPUT, self)
    }

    pub fn from_projection(projection: &QualifiedProjectionV1) -> Option<Self> {
        if !projection.structurally_valid() {
            return None;
        }
        let layer = Self {
            source_dkg_snapshot_commitment: projection.source_dkg_snapshot_commitment.clone(),
            projection_id: projection.projection_id.clone(),
            projection_version: projection.projection_version.clone(),
            input_node_commitments: projection
                .nodes
                .values()
                .map(|node| node.node_commitment.clone())
                .collect(),
            input_edge_commitments: projection
                .edges
                .values()
                .map(|edge| edge.edge_commitment.clone())
                .collect(),
            d6p_current_receipt_commitments: projection
                .d6p_current_receipt_commitments
                .clone(),
            d6n_context_commitment: projection.d6n_context_commitment.clone(),
            d6o_context_commitment: projection.d6o_context_commitment.clone(),
            semantic_environment_commitment: projection.semantic_environment_commitment.clone(),
            claim_ceiling: D6S_CLAIM_CEILING.into(),
        };
        layer.structurally_valid().then_some(layer)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct D6WDerivationLayerV1 {
    pub input_commitment: String,
    pub derivation_profile_commitment: String,
    pub canonicalization_version: String,
    pub execution_trace_commitment: String,
    pub claim_ceiling: String,
}

impl D6WDerivationLayerV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.input_commitment)
            && non_empty(&self.derivation_profile_commitment)
            && self.canonicalization_version == "D6S-CANON-1"
            && non_empty(&self.execution_trace_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
    }

    pub fn commitment(&self) -> String {
        canonical_sha256(D6W_DOMAIN_DERIVATION, self)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct D6WResultLayerV1 {
    pub derivation_commitment: String,
    pub result_status: DerivationResultStatusV1,
    pub result_commitment: String,
    pub contradiction_preserved: bool,
    pub unresolved_preserved: bool,
    pub claim_ceiling: String,
}

impl D6WResultLayerV1 {
    pub fn structurally_valid(&self) -> bool {
        non_empty(&self.derivation_commitment)
            && non_empty(&self.result_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
            && result_flags_are_consistent(
                self.result_status,
                self.contradiction_preserved,
                self.unresolved_preserved,
            )
    }

    pub fn commitment(&self) -> String {
        canonical_sha256(D6W_DOMAIN_RESULT, self)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct D6WReceiptV1 {
    pub schema_version: String,
    pub canonicalization_version: String,
    pub input_commitment: String,
    pub derivation_commitment: String,
    pub result_commitment: String,
    pub d6s_receipt_commitment: String,
    pub claim_ceiling: String,
    pub receipt_commitment: String,
}

impl D6WReceiptV1 {
    pub fn structurally_valid(&self) -> bool {
        self.schema_version == "D6W-1"
            && self.canonicalization_version == "D6S-CANON-1"
            && non_empty(&self.input_commitment)
            && non_empty(&self.derivation_commitment)
            && non_empty(&self.result_commitment)
            && non_empty(&self.d6s_receipt_commitment)
            && non_empty(&self.receipt_commitment)
            && self.claim_ceiling == D6S_CLAIM_CEILING
    }

    pub fn recomputed_commitment(&self) -> String {
        let mut unsigned = self.clone();
        unsigned.receipt_commitment.clear();
        canonical_sha256(D6W_DOMAIN_RECEIPT, &unsigned)
    }

    pub fn commitment_matches(&self) -> bool {
        self.structurally_valid() && self.receipt_commitment == self.recomputed_commitment()
    }
}

fn result_flags_are_consistent(
    status: DerivationResultStatusV1,
    contradiction_preserved: bool,
    unresolved_preserved: bool,
) -> bool {
    match status {
        DerivationResultStatusV1::Supported => !contradiction_preserved && !unresolved_preserved,
        DerivationResultStatusV1::Disputed => contradiction_preserved,
        DerivationResultStatusV1::Unresolved
        | DerivationResultStatusV1::BlockedMissingEvidence
        | DerivationResultStatusV1::BlockedCurrentness
        | DerivationResultStatusV1::BlockedQualification => unresolved_preserved,
        DerivationResultStatusV1::Rejected => true,
    }
}

pub fn build_d6w_receipt(
    d6s_receipt: &CanonicalDerivationReceiptV1,
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
    profile: &DerivationProfileV1,
    current_receipts: &[CurrentFinalityEligibilityReceiptV1],
    execution_trace_commitment: String,
) -> Option<(D6WInputLayerV1, D6WDerivationLayerV1, D6WResultLayerV1, D6WReceiptV1)> {
    if !verify_canonical_receipt(
        d6s_receipt,
        projection,
        environment,
        profile,
        current_receipts,
    ) || execution_trace_commitment.trim().is_empty()
    {
        return None;
    }

    let input = D6WInputLayerV1::from_projection(projection)?;
    let derivation = D6WDerivationLayerV1 {
        input_commitment: input.commitment(),
        derivation_profile_commitment: profile.commitment(),
        canonicalization_version: projection.canonicalization_version.clone(),
        execution_trace_commitment,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    };
    if !derivation.structurally_valid() {
        return None;
    }

    let result = D6WResultLayerV1 {
        derivation_commitment: derivation.commitment(),
        result_status: d6s_receipt.result_status,
        result_commitment: d6s_receipt.result_commitment.clone(),
        contradiction_preserved: d6s_receipt.contradiction_preserved,
        unresolved_preserved: d6s_receipt.unresolved_preserved,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    };
    if !result.structurally_valid() {
        return None;
    }

    let mut receipt = D6WReceiptV1 {
        schema_version: "D6W-1".into(),
        canonicalization_version: "D6S-CANON-1".into(),
        input_commitment: derivation.input_commitment.clone(),
        derivation_commitment: result.derivation_commitment.clone(),
        result_commitment: result.commitment(),
        d6s_receipt_commitment: d6s_receipt.receipt_commitment.clone(),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
        receipt_commitment: String::new(),
    };
    receipt.receipt_commitment = receipt.recomputed_commitment();

    Some((input, derivation, result, receipt))
}

pub fn verify_d6w_receipt(
    receipt: &D6WReceiptV1,
    d6s_receipt: &CanonicalDerivationReceiptV1,
    projection: &QualifiedProjectionV1,
    environment: &SemanticEnvironmentV1,
    profile: &DerivationProfileV1,
    current_receipts: &[CurrentFinalityEligibilityReceiptV1],
    execution_trace_commitment: &str,
) -> bool {
    if !receipt.commitment_matches() || receipt.d6s_receipt_commitment != d6s_receipt.receipt_commitment {
        return false;
    }
    let Some((input, derivation, result, expected)) = build_d6w_receipt(
        d6s_receipt,
        projection,
        environment,
        profile,
        current_receipts,
        execution_trace_commitment.to_owned(),
    ) else {
        return false;
    };
    receipt == &expected
        && receipt.input_commitment == input.commitment()
        && receipt.derivation_commitment == derivation.commitment()
        && receipt.result_commitment == result.commitment()
}

pub fn d6w_cannot_create_authorization() -> bool {
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::canonical_derivation_receipt::{
        build_canonical_receipt, D6S_REFERENCE_CANONICALIZATION_VERSION,
    };
    use crate::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
    use std::collections::BTreeMap;

    fn env() -> SemanticEnvironmentV1 {
        SemanticEnvironmentV1 {
            semantic_profile_id: "integral".into(),
            semantic_profile_version: "1".into(),
            current_frontier_root: Some("frontier-1".into()),
            d6p_eligibility_context_root: Some(
                crate::canonical_derivation_receipt::commitment_set_digest(
                    &["d6p-receipt-1".into()].into_iter().collect(),
                ),
            ),
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
                crate::canonical_derivation_receipt::QualifiedNodeV1 {
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
            "p".into(),
            crate::canonical_derivation_receipt::QualifiedEdgeV1 {
                edge_id: "p".into(),
                from_node_id: "e".into(),
                to_node_id: "a".into(),
                kind: ClaimGraphEdgeKindV1::Provenance,
                edge_commitment: "edge-p".into(),
                claim_ceiling: D6S_CLAIM_CEILING.into(),
            },
        );
        edges.insert(
            "s".into(),
            crate::canonical_derivation_receipt::QualifiedEdgeV1 {
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
            source_dkg_snapshot_commitment: "dkg-snapshot-1".into(),
            nodes,
            edges,
            d6p_current_receipt_commitments: ["d6p-receipt-1".into()].into_iter().collect(),
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
            observation_set_commitment: "set-commitment".into(),
            d6n_assessment_commitment: "assessment:set-commitment".into(),
            witness_eligibility_ids: ["eligibility-1".into()].into_iter().collect(),
            observer_generation_ids: ["generation-1".into()].into_iter().collect(),
            current_frontier_root: "frontier-1".into(),
            lifecycle_profile_id: "life-profile-1".into(),
            eligible_independent_count: 2,
            preserved_contradictory_count: 0,
            disposition: crate::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent,
            qualification_transition_id: "transition-1".into(),
            receipt_commitment: "d6p-receipt-1".into(),
            claim_ceiling: crate::finality_eligibility_composition::FINALITY_ELIGIBILITY_COMPOSITION_CLAIM_CEILING.into(),
        }
    }

    fn d6s_receipt() -> CanonicalDerivationReceiptV1 {
        build_canonical_receipt(
            &projection(),
            &env(),
            &profile(),
            &[d6p_receipt()],
            DerivationResultStatusV1::Supported,
            "result-1".into(),
            false,
            false,
        )
        .expect("fixture D6S receipt")
    }

    fn layers(trace: &str) -> (D6WInputLayerV1, D6WDerivationLayerV1, D6WResultLayerV1, D6WReceiptV1) {
        build_d6w_receipt(
            &d6s_receipt(),
            &projection(),
            &env(),
            &profile(),
            &[d6p_receipt()],
            trace.into(),
        )
        .expect("fixture D6W receipt")
    }

    #[test]
    fn deterministic_layers_repeat_exactly() {
        assert_eq!(layers("trace-1"), layers("trace-1"));
    }

    #[test]
    fn input_mutation_changes_input_and_all_downstream_layers() {
        let (input, derivation, result, receipt) = layers("trace-1");
        let mut p = projection();
        p.nodes.get_mut("e").unwrap().node_commitment = "node-e-mutated".into();
        let d6s = build_canonical_receipt(
            &p,
            &env(),
            &profile(),
            &[d6p_receipt()],
            DerivationResultStatusV1::Supported,
            "result-1".into(),
            false,
            false,
        ).unwrap();
        let (_, d2, r2, rc2) = build_d6w_receipt(
            &d6s, &p, &env(), &profile(), &[d6p_receipt()], "trace-1".into()
        ).unwrap();
        assert_ne!(input.commitment(), D6WInputLayerV1::from_projection(&p).unwrap().commitment());
        assert_ne!(derivation.commitment(), d2.commitment());
        assert_ne!(result.commitment(), r2.commitment());
        assert_ne!(receipt.receipt_commitment, rc2.receipt_commitment);
    }

    #[test]
    fn derivation_profile_mutation_changes_derivation_not_input() {
        let (input, derivation, _, _) = layers("trace-1");
        let mut p = profile();
        p.version = "2".into();
        let mut projection = projection();
        projection.derivation_profile_commitment = p.commitment();
        let d6s = build_canonical_receipt(
            &projection, &env(), &p, &[d6p_receipt()],
            DerivationResultStatusV1::Supported, "result-1".into(), false, false
        ).unwrap();
        let (_, d2, _, _) = build_d6w_receipt(
            &d6s, &projection, &env(), &p, &[d6p_receipt()], "trace-1".into()
        ).unwrap();
        assert_eq!(input.commitment(), D6WInputLayerV1::from_projection(&projection).unwrap().commitment());
        assert_ne!(derivation.commitment(), d2.commitment());
    }

    #[test]
    fn canonicalization_version_mutation_changes_derivation_not_input() {
        let (input, derivation, _, _) = layers("trace-1");
        let mut projection = projection();
        projection.canonicalization_version = "D6S-CANON-2".into();
        let d6s = build_canonical_receipt(
            &projection,
            &env(),
            &profile(),
            &[d6p_receipt()],
            DerivationResultStatusV1::Supported,
            "result-1".into(),
            false,
            false,
        );
        assert!(d6s.is_none());
        assert_eq!(input.commitment(), layers("trace-1").0.commitment());
        assert_ne!(derivation.canonicalization_version, "D6S-CANON-2");
    }

    #[test]
    fn execution_trace_mutation_changes_derivation_not_input() {
        let (input, derivation, _, _) = layers("trace-1");
        let (_, d2, _, _) = layers("trace-2");
        assert_eq!(input.commitment(), layers("trace-2").0.commitment());
        assert_ne!(derivation.commitment(), d2.commitment());
    }

    #[test]
    fn result_mutation_changes_result_and_receipt_not_input_or_derivation() {
        let (input, derivation, result, receipt) = layers("trace-1");
        let p = projection();
        let d6s = build_canonical_receipt(
            &p, &env(), &profile(), &[d6p_receipt()],
            DerivationResultStatusV1::Rejected, "result-2".into(), false, false
        ).unwrap();
        let (_, d2, r2, rc2) = build_d6w_receipt(
            &d6s, &p, &env(), &profile(), &[d6p_receipt()], "trace-1".into()
        ).unwrap();
        assert_eq!(input.commitment(), D6WInputLayerV1::from_projection(&p).unwrap().commitment());
        assert_eq!(derivation.commitment(), d2.commitment());
        assert_ne!(result.commitment(), r2.commitment());
        assert_ne!(receipt.receipt_commitment, rc2.receipt_commitment);
    }

    #[test]
    fn claim_ceiling_mutation_changes_result_and_receipt() {
        let (input, derivation, result, receipt) = layers("trace-1");
        let mut mutated = result.clone();
        mutated.claim_ceiling = "different-claim-ceiling".into();
        assert!(!mutated.structurally_valid());
        let mutated_result_commitment = mutated.commitment();
        assert_ne!(result.commitment(), mutated_result_commitment);

        let mut mutated_receipt = receipt.clone();
        mutated_receipt.result_commitment = mutated_result_commitment;
        mutated_receipt.receipt_commitment = mutated_receipt.recomputed_commitment();
        assert_ne!(receipt.receipt_commitment, mutated_receipt.receipt_commitment);

        assert_eq!(input.claim_ceiling, D6S_CLAIM_CEILING);
        assert_eq!(derivation.claim_ceiling, D6S_CLAIM_CEILING);
    }

    #[test]
    fn malformed_d6s_receipt_blocks_decomposition() {
        let mut d6s = d6s_receipt();
        d6s.receipt_commitment = "tampered".into();
        assert!(build_d6w_receipt(
            &d6s, &projection(), &env(), &profile(), &[d6p_receipt()], "trace-1".into()
        ).is_none());
    }

    #[test]
    fn input_layer_binds_d6p_context() {
        let (input, _, _, _) = layers("trace-1");
        assert!(input.d6p_current_receipt_commitments.contains("d6p-receipt-1"));
    }

    #[test]
    fn d6w_receipt_chains_all_three_layers() {
        let (input, derivation, result, receipt) = layers("trace-1");
        assert_eq!(receipt.input_commitment, input.commitment());
        assert_eq!(receipt.derivation_commitment, derivation.commitment());
        assert_eq!(receipt.result_commitment, result.commitment());
        assert_eq!(receipt.d6s_receipt_commitment, d6s_receipt().receipt_commitment);
        assert!(receipt.commitment_matches());
    }

    #[test]
    fn verifier_reconstructs_exact_layered_receipt() {
        let (_, _, _, receipt) = layers("trace-1");
        assert!(verify_d6w_receipt(
            &receipt,
            &d6s_receipt(),
            &projection(),
            &env(),
            &profile(),
            &[d6p_receipt()],
            "trace-1",
        ));
    }

    #[test]
    fn trace_mismatch_is_rejected() {
        let (_, _, _, receipt) = layers("trace-1");
        assert!(!verify_d6w_receipt(
            &receipt,
            &d6s_receipt(),
            &projection(),
            &env(),
            &profile(),
            &[d6p_receipt()],
            "trace-2",
        ));
    }

    #[test]
    fn d6w_cannot_create_authorization() {
        assert!(d6w_cannot_create_authorization());
    }
}
