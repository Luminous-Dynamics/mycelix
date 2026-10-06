//! Cross-layer Integral interoperability conformance.
//!
//! ReferenceModelOnly: one mutation is classified at the OAD semantic boundary
//! and then checked through D6X closure and D6W input/derivation commitments.
//! This intentionally tests propagation properties, not an Integral wire schema.

use cos_conformance::canonical_derivation_receipt::{
    build_canonical_receipt_with_authoritative_d6p, canonical_sha256, commitment_set_digest,
    verify_canonical_receipt_with_authoritative_d6p, DerivationProfileV1, DerivationResultStatusV1,
    QualifiedEdgeV1, QualifiedNodeV1, QualifiedProjectionV1, SemanticEnvironmentV1, D6S_CLAIM_CEILING,
};
use cos_conformance::evidence_claim_graph::{ClaimGraphEdgeKindV1, ClaimGraphNodeKindV1};
use cos_conformance::integral_interop::{
    selected_oad_design_semantic_commitment_checked, validate_selected_oad_design_semantics,
};
use cos_conformance::layered_derivation_commitment::{
    DerivationCommitmentV1, InputCommitmentV1,
};
use cos_conformance::qualified_dependency_closure_d6x::{
    compute_dependency_closure, compute_dependency_closure_from_authoritative_d6p_at_frontier,
    DependencyClosureProfileV1, DependencyCurrentnessV1, DependencyRuleV1,
    DependencyClosureStatusV1, SemanticDependencyReferenceV1, SemanticDependencyResolutionEvidenceV1,
};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_design.json"
    ))
    .expect("Integral fixture must be valid JSON")
}

fn snapshot_commitment() -> String {
    canonical_sha256(
        "integral-interop-1-snapshot",
        &serde_json::json!({
            "fixture": "Integral-Interop-1",
            "version": 1,
            "snapshot": "integral-snapshot-1",
        }),
    )
}

fn actual_d6p_fixture() -> (
    cos_conformance::finality_eligibility_composition::FinalityEligibilityCompositionV1,
    cos_conformance::finality_eligibility_composition::CurrentFinalityEligibilityReceiptV1,
) {
    use cos_conformance::contestable_finality::{
        assess_observation_set, verify_observation_set_assessment_provenance,
        ExternalObservedEvidenceV1,
        ExternalObserverProfileV1, ExternalObserverRoleV1, ExternalObservationSetV1,
        FinalityQualificationProfileV1, ObservationIndependenceV1,
    };
    use cos_conformance::effect_finality::{
        assess_external_finality, ExternalEffectObservationV1, ExternalFinalityProfileV1,
        ExternalFinalityReceiptV1, ExternalFinalityStateV1, ExternalObservationSourceV1,
        ExternalObservedStateV1, FinalityUsePurposeV1, FinalityDispositionV1,
    };
    use cos_conformance::observer_lifecycle::{
        assess_evidence_eligibility, verify_eligibility_receipt_provenance,
        EvidenceDependencySnapshotV1, EvidenceEligibilityReceiptV1, EvidenceEligibilityDispositionV1,
        ObserverEvidenceProvenanceV1, ObserverGenerationV1, ObserverLifecycleLedgerV1,
        ObserverLifecycleProfileV1, ObserverLifecycleUsePurposeV1, ObserverStatusV1,
        OBSERVER_LIFECYCLE_CLAIM_CEILING,
    };
    use cos_conformance::finality_eligibility_composition::{
        compose_finality_eligibility_from_authoritative_d6n_d6o,
        CurrentFinalityEligibilityReceiptV1,
        FinalityEligibilityCompositionV1,
    };
    use cos_conformance::substitution_continuity::{
        IdempotencyScopeV1, ProviderOutcomeKindV1, ProviderOutcomeV1, ProviderRouteV1,
        SemanticEffectV1, SemanticSubstitutionProfileV1,
    };
    use std::collections::{BTreeMap, BTreeSet};

    let semantic_environment_root = environment().commitment();

    // D6M: provider-neutral semantic effect + provider route + observed outcome
    // + independently sourced observation + current-finality receipt.
    let effect = SemanticEffectV1 {
        effect_id: "integral-effect-1".into(),
        lineage_id: "integral-lineage-1".into(),
        generation_id: "integral-effect-generation-1".into(),
        request_commitment: "integral-request-1".into(),
        semantic_environment_root: semantic_environment_root.clone(),
        effect_class: "manufacturing-task".into(),
        resource_id: "water-purifier".into(),
        tenant_id: "integral-tenant".into(),
        amount: "1".into(),
        unit: "unit".into(),
        authority_claim_id: "integral-authority-1".into(),
        consent_claim_id: "integral-consent-1".into(),
        idempotency_key: "integral-idem-1".into(),
    };
    let substitution_profile = SemanticSubstitutionProfileV1 {
        profile_id: "integral-substitution-1".into(),
        semantic_environment_root: semantic_environment_root.clone(),
        request_commitment: effect.request_commitment.clone(),
        effect_class: effect.effect_class.clone(),
        resource_id: effect.resource_id.clone(),
        tenant_id: effect.tenant_id.clone(),
        amount: effect.amount.clone(),
        unit: effect.unit.clone(),
        authority_claim_id: effect.authority_claim_id.clone(),
        consent_claim_id: effect.consent_claim_id.clone(),
        idempotency_key: effect.idempotency_key.clone(),
        idempotency_scope: IdempotencyScopeV1::ContractWide,
        idempotency_policy_root: Some("integral-idempotency-policy".into()),
        allow_retry_after_no_effect: true,
        allow_same_provider_retry: true,
        allowed_provider_profile_roots: BTreeMap::from([
            ("integral-provider-1".into(), "integral-provider-profile-1".into()),
        ]),
        profile_commitment: "integral-substitution-profile-commitment".into(),
    };
    let route = ProviderRouteV1 {
        route_id: "integral-route-1".into(),
        effect_id: effect.effect_id.clone(),
        substitution_profile_id: substitution_profile.profile_id.clone(),
        provider_id: "integral-provider-1".into(),
        provider_profile_root: "integral-provider-profile-1".into(),
        provider_operation_id: "integral-operation-1".into(),
        route_generation: 0,
        lifecycle_generation_id: effect.generation_id.clone(),
        request_commitment: effect.request_commitment.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        effect_class: effect.effect_class.clone(),
        resource_id: effect.resource_id.clone(),
        tenant_id: effect.tenant_id.clone(),
        amount: effect.amount.clone(),
        unit: effect.unit.clone(),
        authority_claim_id: effect.authority_claim_id.clone(),
        consent_claim_id: effect.consent_claim_id.clone(),
        idempotency_key: effect.idempotency_key.clone(),
        route_frontier_root: "integral-frontier-1".into(),
    };
    let outcome = ProviderOutcomeV1 {
        outcome_id: "integral-outcome-1".into(),
        effect_id: effect.effect_id.clone(),
        route_id: route.route_id.clone(),
        provider_operation_id: route.provider_operation_id.clone(),
        provider_id: route.provider_id.clone(),
        provider_profile_root: route.provider_profile_root.clone(),
        request_commitment: effect.request_commitment.clone(),
        idempotency_key: effect.idempotency_key.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        observed_frontier_root: "integral-frontier-1".into(),
        outcome_kind: ProviderOutcomeKindV1::Succeeded,
        evidence_root: "integral-provider-evidence-1".into(),
        claim_ceiling: "Provider observation only; no independent outcome-truth or authorization claim.".into(),
    };
    let mut d6m_observation = ExternalEffectObservationV1 {
        observation_id: "integral-d6m-observation-1".into(),
        effect_id: effect.effect_id.clone(),
        effect_lineage_id: effect.lineage_id.clone(),
        lifecycle_generation_id: effect.generation_id.clone(),
        route_id: route.route_id.clone(),
        provider_id: route.provider_id.clone(),
        provider_operation_id: route.provider_operation_id.clone(),
        provider_profile_root: route.provider_profile_root.clone(),
        provider_outcome_id: outcome.outcome_id.clone(),
        request_commitment: effect.request_commitment.clone(),
        idempotency_key: effect.idempotency_key.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        observed_frontier_root: "integral-frontier-1".into(),
        observed_state: ExternalObservedStateV1::Applied,
        source: ExternalObservationSourceV1::IndependentObserver,
        evidence_root: "integral-independent-evidence-1".into(),
        observation_commitment: String::new(),
        claim_ceiling: cos_conformance::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
    };
    d6m_observation.observation_commitment = d6m_observation.recomputed_commitment();
    let observer_profile = ExternalObserverProfileV1 {
        observer_id: "integral-observer-1".into(),
        role: ExternalObserverRoleV1::IndependentObserver,
        observation_method: "independent-state-read".into(),
        provider_relationship: "external".into(),
        evidence_root: "integral-independent-evidence-1".into(),
        custody_root: "integral-independent-custody-1".into(),
        upstream_observer_ids: BTreeSet::new(),
        upstream_evidence_roots: BTreeSet::new(),
        semantic_environment_root: semantic_environment_root.clone(),
        observation_profile_id: "integral-observation-profile-1".into(),
        independence: ObservationIndependenceV1::DeclaredIndependent,
        independence_commitment: "integral-independence-1".into(),
        claim_ceiling: cos_conformance::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
    };
    let d6m_evidence = ExternalObservedEvidenceV1 {
        observation: d6m_observation.clone(),
        observer_id: observer_profile.observer_id.clone(),
        observer: observer_profile.clone(),
    };
    let d6m_profile = ExternalFinalityProfileV1 {
        profile_id: "integral-d6m-finality-profile-1".into(),
        effect_id: effect.effect_id.clone(),
        substitution_profile_id: substitution_profile.profile_id.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        request_commitment: effect.request_commitment.clone(),
        idempotency_key: effect.idempotency_key.clone(),
        allowed_observation_sources: BTreeSet::from([
            ExternalObservationSourceV1::IndependentObserver,
        ]),
        required_finality_state: ExternalFinalityStateV1::Applied,
        current_frontier_required: true,
        profile_commitment: "integral-d6m-finality-profile-commitment".into(),
        claim_ceiling: cos_conformance::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
    };
    let mut d6m_receipt = ExternalFinalityReceiptV1 {
        receipt_id: "integral-d6m-finality-1".into(),
        effect_id: effect.effect_id.clone(),
        effect_lineage_id: effect.lineage_id.clone(),
        lifecycle_generation_id: effect.generation_id.clone(),
        route_id: route.route_id.clone(),
        provider_id: route.provider_id.clone(),
        provider_operation_id: route.provider_operation_id.clone(),
        provider_profile_root: route.provider_profile_root.clone(),
        provider_outcome_id: outcome.outcome_id.clone(),
        observation_id: d6m_observation.observation_id.clone(),
        observation_frontier_root: d6m_observation.observed_frontier_root.clone(),
        finality_profile_id: d6m_profile.profile_id.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        finality_state: ExternalFinalityStateV1::Applied,
        evidence_root: d6m_observation.evidence_root.clone(),
        finality_commitment: String::new(),
        claim_ceiling: cos_conformance::effect_finality::EXTERNAL_FINALITY_CLAIM_CEILING.into(),
    };
    d6m_receipt.finality_commitment = d6m_receipt.recomputed_commitment();
    assert_eq!(
        assess_external_finality(
            &effect,
            &substitution_profile,
            &d6m_profile,
            &route,
            &outcome,
            &d6m_observation,
            &d6m_receipt,
            "integral-frontier-1",
            FinalityUsePurposeV1::CurrentFinality,
            None,
        ),
        FinalityDispositionV1::AcceptedCurrent
    );

    // D6O: bind the same observation to a live generation, dependency snapshot,
    // lifecycle profile, and exact eligibility receipt.
    let mut lifecycle_profile = ObserverLifecycleProfileV1 {
        profile_id: "integral-lifecycle-profile-1".into(),
        semantic_environment_root: semantic_environment_root.clone(),
        observation_profile_id: "integral-observation-profile-1".into(),
        allowed_roles: BTreeSet::from([ExternalObserverRoleV1::IndependentObserver]),
        current_frontier_required: true,
        historical_evidence_allowed: true,
        profile_commitment: String::new(),
        claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
    };
    lifecycle_profile.profile_commitment = lifecycle_profile.recomputed_commitment();

    let mut generation = ObserverGenerationV1 {
        generation_id: "integral-observer-generation-1".into(),
        observer_id: observer_profile.observer_id.clone(),
        generation_sequence: 1,
        predecessor_generation_id: None,
        role: ExternalObserverRoleV1::IndependentObserver,
        observation_method: observer_profile.observation_method.clone(),
        provider_relationship: observer_profile.provider_relationship.clone(),
        evidence_root: observer_profile.evidence_root.clone(),
        custody_root: observer_profile.custody_root.clone(),
        upstream_observer_ids: BTreeSet::new(),
        upstream_evidence_roots: BTreeSet::new(),
        semantic_environment_root: semantic_environment_root.clone(),
        observation_profile_id: observer_profile.observation_profile_id.clone(),
        created_frontier_root: "integral-frontier-1".into(),
        created_frontier_sequence: 1,
        initial_status: ObserverStatusV1::Active,
        generation_commitment: String::new(),
        claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
    };
    generation.generation_commitment = generation.recomputed_commitment();

    let mut snapshot = EvidenceDependencySnapshotV1 {
        snapshot_id: "integral-d6o-snapshot-1".into(),
        observer_generation_id: generation.generation_id.clone(),
        observation_profile_id: generation.observation_profile_id.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        evidence_root: generation.evidence_root.clone(),
        custody_root: generation.custody_root.clone(),
        upstream_observer_ids: BTreeSet::new(),
        upstream_evidence_roots: BTreeSet::new(),
        independence: ObservationIndependenceV1::DeclaredIndependent,
        effective_frontier_root: "integral-frontier-1".into(),
        effective_frontier_sequence: 1,
        predecessor_snapshot_id: None,
        snapshot_commitment: String::new(),
        claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
    };
    snapshot.snapshot_commitment = snapshot.recomputed_commitment();

    let mut d6o_ledger = ObserverLifecycleLedgerV1::default();
    assert!(matches!(
        d6o_ledger.record_generation(generation.clone()),
        cos_conformance::observer_lifecycle::LifecycleRecordDispositionV1::Recorded
    ));
    assert!(matches!(
        d6o_ledger.record_dependency_snapshot(snapshot.clone()),
        cos_conformance::observer_lifecycle::LifecycleRecordDispositionV1::Recorded
    ));

    let classification = cos_conformance::contestable_finality::ObservationClassificationV1::CorroboratingIndependent;
    let disposition = assess_evidence_eligibility(
        &d6m_evidence,
        &generation,
        &snapshot,
        &lifecycle_profile,
        &d6o_ledger,
        classification,
        ObserverEvidenceProvenanceV1::Live,
        "integral-frontier-1",
        1,
        "integral-frontier-1",
        1,
        ObserverLifecycleUsePurposeV1::CurrentFinalityEligibility,
    );
    assert_eq!(disposition, EvidenceEligibilityDispositionV1::EligibleCurrent);

    let mut d6o_receipt = EvidenceEligibilityReceiptV1 {
        eligibility_id: "integral-d6o-eligibility-1".into(),
        observation_id: d6m_observation.observation_id.clone(),
        observer_id: generation.observer_id.clone(),
        observer_generation_id: generation.generation_id.clone(),
        observation_profile_id: generation.observation_profile_id.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        dependency_snapshot_id: snapshot.snapshot_id.clone(),
        observation_frontier_root: "integral-frontier-1".into(),
        observation_frontier_sequence: 1,
        current_frontier_root: "integral-frontier-1".into(),
        current_frontier_sequence: 1,
        current_generation_id: Some(generation.generation_id.clone()),
        qualification_profile_id: lifecycle_profile.profile_id.clone(),
        provenance: ObserverEvidenceProvenanceV1::Live,
        classification,
        disposition,
        lifecycle_transition_ids: BTreeSet::new(),
        eligibility_commitment: String::new(),
        claim_ceiling: OBSERVER_LIFECYCLE_CLAIM_CEILING.into(),
    };
    d6o_receipt.eligibility_commitment = d6o_receipt.recomputed_commitment();
    assert!(verify_eligibility_receipt_provenance(
        &d6o_receipt,
        &d6m_evidence,
        &generation,
        &snapshot,
        &lifecycle_profile,
        &d6o_ledger,
    ));

    // D6N: reconstruct the exact assessment from the D6M observation.
    let d6n_profile = FinalityQualificationProfileV1 {
        profile_id: "integral-d6n-finality-profile-1".into(),
        semantic_environment_root: semantic_environment_root.clone(),
        allowed_observation_sources: BTreeSet::from([
            ExternalObservationSourceV1::IndependentObserver,
        ]),
        required_independent_observations: 1,
        current_frontier_required: true,
        provider_reports_may_satisfy_independence: false,
        allow_explicit_conflict_resolution: false,
        profile_commitment: "integral-d6n-profile-commitment".into(),
        claim_ceiling: cos_conformance::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
    };
    let expected_finality_profile_commitment = d6n_profile.recomputed_commitment();
    d6n_profile.profile_commitment = expected_finality_profile_commitment.clone();
    let d6n_set = ExternalObservationSetV1 {
        set_id: "integral-d6n-set-1".into(),
        effect_id: effect.effect_id.clone(),
        effect_lineage_id: effect.lineage_id.clone(),
        lifecycle_generation_id: effect.generation_id.clone(),
        route_id: route.route_id.clone(),
        provider_id: route.provider_id.clone(),
        provider_operation_id: route.provider_operation_id.clone(),
        provider_profile_root: route.provider_profile_root.clone(),
        semantic_environment_root: semantic_environment_root.clone(),
        observation_frontier_root: "integral-frontier-1".into(),
        qualification_profile_id: d6n_profile.profile_id.clone(),
        observation_ids: BTreeSet::from([d6m_observation.observation_id.clone()]),
        target_state: ExternalFinalityStateV1::Applied,
        set_commitment: "integral-d6n-set-commitment".into(),
        claim_ceiling: cos_conformance::contestable_finality::CONTESTABLE_FINALITY_CLAIM_CEILING.into(),
    };
    d6n_set.set_commitment = d6n_set.recomputed_commitment();
    assert!(cos_conformance::contestable_finality::verify_observation_set_provenance(
        &d6n_set,
        &effect,
        &route,
        &d6n_profile,
        std::slice::from_ref(&d6m_evidence),
        "integral-frontier-1",
        &effect.generation_id,
    ));
    let d6n_assessment = assess_observation_set(
        &effect,
        &route,
        &d6n_profile,
        &d6n_set,
        std::slice::from_ref(&d6m_evidence),
        "integral-frontier-1",
        &effect.generation_id,
    );

    let mut mutated_observation = d6m_evidence.clone();
    mutated_observation.observation.observed_state = ExternalObservedStateV1::NotApplied;
    mutated_observation.observation.observation_commitment =
        mutated_observation.observation.recomputed_commitment();
    assert!(
        !cos_conformance::contestable_finality::verify_observation_set_provenance(
            &d6n_set,
            &effect,
            &route,
            &d6n_profile,
            std::slice::from_ref(&mutated_observation),
            "integral-frontier-1",
            &effect.generation_id,
        ),
        "D6M semantic mutation must not cross the D6N provenance boundary"
    );

    let mut mutated_d6n = d6n_assessment.clone();
    mutated_d6n.disposition =
        cos_conformance::contestable_finality::ObservationSetDispositionV1::Contested;
    assert!(
        !verify_observation_set_assessment_provenance(
            &mutated_d6n,
            &effect,
            &route,
            &d6n_profile,
            &d6n_set,
            std::slice::from_ref(&d6m_evidence),
            "integral-frontier-1",
            &effect.generation_id,
        ),
        "D6N assessment substitution must not cross the D6P admission boundary"
    );

    let mut mutated_d6o = d6o_receipt.clone();
    mutated_d6o.observer_generation_id = "integral-attacker-generation".into();
    mutated_d6o.eligibility_commitment = mutated_d6o.recomputed_commitment();
    assert!(
        !verify_eligibility_receipt_provenance(
            &mutated_d6o,
            &d6m_evidence,
            &generation,
            &snapshot,
            &lifecycle_profile,
            &d6o_ledger,
        ),
        "D6O lifecycle substitution must not cross the D6P admission boundary"
    );

    assert!(verify_observation_set_assessment_provenance(
        &d6n_assessment,
        &effect,
        &route,
        &d6n_profile,
        &d6n_set,
        std::slice::from_ref(&d6m_evidence),
        "integral-frontier-1",
        &effect.generation_id,
    ));

    // D6P: compose only after authoritative D6N + D6O inputs have been
    // reconstructed and verified.
    let d6p = compose_finality_eligibility_from_authoritative_d6n_d6o(
        &effect,
        &route,
        &d6n_profile,
        &expected_finality_profile_commitment,
        &d6n_set,
        &d6n_assessment,
        std::slice::from_ref(&d6m_evidence),
        std::slice::from_ref(&d6o_receipt),
        &lifecycle_profile,
        &lifecycle_profile.profile_commitment,
        &d6o_ledger,
        "integral-frontier-1",
        &effect.generation_id,
        1,
    )
    .expect("D6P composition must consume the verified D6M/D6N/D6O chain");

    assert_eq!(
        d6p.disposition,
        cos_conformance::finality_eligibility_composition::FinalityEligibilityDispositionV1::EligibleCurrent
    );
    assert!(d6p.semantically_valid());
    assert!(d6p.commitment_matches());

    let mut d6p_receipt = CurrentFinalityEligibilityReceiptV1 {
        receipt_id: "integral-d6p-receipt-1".into(),
        effect_id: d6p.effect_id.clone(),
        effect_lineage_id: d6p.effect_lineage_id.clone(),
        lifecycle_generation_id: d6p.lifecycle_generation_id.clone(),
        route_id: d6p.route_id.clone(),
        provider_id: d6p.provider_id.clone(),
        provider_operation_id: d6p.provider_operation_id.clone(),
        provider_profile_root: d6p.provider_profile_root.clone(),
        semantic_environment_root: d6p.semantic_environment_root.clone(),
        observation_set_id: d6p.observation_set_id.clone(),
        observation_set_commitment: d6p.observation_set_commitment.clone(),
        d6n_assessment_commitment: d6p.d6n_assessment_commitment.clone(),
        composition_commitment: d6p.composition_commitment.clone(),
        witness_eligibility_ids: d6p.witnesses.iter()
            .filter_map(|w| w.d6o_eligibility_id.clone()).collect(),
        observer_generation_ids: d6p.witnesses.iter()
            .filter_map(|w| w.observer_generation_id.clone()).collect(),
        current_frontier_root: d6p.current_frontier_root.clone(),
        lifecycle_profile_id: d6p.lifecycle_profile_id.clone(),
        eligible_independent_count: d6p.eligible_independent_count,
        preserved_contradictory_count: d6p.preserved_contradictory_count,
        disposition: d6p.disposition,
        qualification_transition_id: d6p.qualification_transition_id.clone().unwrap_or_default(),
        receipt_commitment: String::new(),
        claim_ceiling: d6p.claim_ceiling.clone(),
    };
    d6p_receipt.receipt_commitment = d6p_receipt.recomputed_commitment();
    assert!(cos_conformance::finality_eligibility_composition::verify_current_receipt_provenance_from_composition(
        &d6p_receipt, &d6p
    ));

    // Frozen layered golden commitments. D6M/D6O use their explicit
    // serde-based reference-model commitment domains; D6N's set-level
    // assessment identity is deliberately explicit; D6P binds the D6N/D6O
    // join and D6S/D6X consume only those committed products.
    assert_eq!(
        d6m_observation.observation_commitment,
        "0d3297752f853efb0c091cd9a8edb747972039f5e3eef47b921d5bd2a4077d6b"
    );
    assert_eq!(
        d6m_receipt.finality_commitment,
        "631bc0ce6ee5fed3bf37902ed57e24173197c1176cedc541c1b608518d6249f4"
    );
    assert_eq!(
        lifecycle_profile.profile_commitment,
        "bebf29c932e74485c2edbbc5bd22b7da9ac4744a92709777b75fd8a39b181311"
    );
    assert_eq!(
        generation.generation_commitment,
        "24d781092b9a049e7989715bc413decd183ade934c88964b9d6d05bff19493bd"
    );
    assert_eq!(
        snapshot.snapshot_commitment,
        "183136d3dbae695d09ccaa2c10710e7cc323de9da341a0ff7063bfc9b31de404"
    );
    assert_eq!(
        d6o_receipt.eligibility_commitment,
        "18a7aa203feecb180fecad4fad792aa2495e47c8d26e04904417a78bff4150c4"
    );
    assert_eq!(
        d6n_assessment.assessment_commitment,
        "assessment:integral-d6n-set-commitment"
    );
    assert_eq!(
        d6n_assessment.assessments[0].observation_commitment,
        d6m_observation.observation_commitment
    );
    assert_eq!(
        d6n_assessment.assessments[0].assessment_commitment,
        "assessment:integral-d6n-set-commitment"
    );
    assert_eq!(
        d6p.witnesses[0].witness_commitment,
        "ea29e32dffe0e92edc3ee18e820c1247d838345c3b1379066b2f67e1b12b3ddc"
    );
    assert_eq!(
        d6p.composition_commitment,
        "a89169af5a2fba65dbe60a137f102a5397fcb7002ba17945f694d26f053bbea2"
    );
    assert_eq!(
        d6p_receipt.receipt_commitment,
        "0d3003e880bf4621f30b81277454d84e5d90ecf50187219e831e5299e788694f"
    );

    (d6p, d6p_receipt)
}
fn d6p_receipt_commitment() -> String {
    canonical_sha256(
        "integral-interop-1-d6p-receipt",
        &serde_json::json!({
            "receipt": "integral-d6p-receipt-1",
            "frontier": "integral-frontier-1",
        }),
    )
}

fn environment() -> SemanticEnvironmentV1 {
    SemanticEnvironmentV1 {
        semantic_profile_id: "integral-interop-1".into(),
        semantic_profile_version: "1".into(),
        current_frontier_root: Some("integral-frontier-1".into()),
        d6p_eligibility_context_root: None,
        d6n_observer_context_root: None,
        d6o_lifecycle_context_root: None,
        membership_authority_scope_root: None,
        dependency_snapshot_root: Some(snapshot_commitment()),
        historical_cutoff: None,
        policy_version: "integral-interop-1".into(),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn derivation_profile() -> DerivationProfileV1 {
    DerivationProfileV1 {
        profile_id: "integral-d6w-reference-1".into(),
        version: "1".into(),
        rule_ids: ["integral-interop-1".into()].into_iter().collect(),
        permits_recursive_fixpoint: false,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn node(id: &str, kind: ClaimGraphNodeKindV1, content_commitment: String) -> QualifiedNodeV1 {
    QualifiedNodeV1 {
        node_id: id.into(),
        kind,
        node_commitment: canonical_sha256(
            "integral-interop-1-node",
            &serde_json::json!({
                "id": id,
                "content_commitment": content_commitment,
                "kind": format!("{kind:?}"),
            }),
        ),
        content_commitment,
        historical_only: false,
        current_frontier_root: Some("integral-frontier-1".into()),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn edge(id: &str, from: &str, to: &str, kind: ClaimGraphEdgeKindV1) -> QualifiedEdgeV1 {
    QualifiedEdgeV1 {
        edge_id: id.into(),
        from_node_id: from.into(),
        to_node_id: to.into(),
        kind,
        edge_commitment: canonical_sha256(
            "integral-interop-1-edge",
            &serde_json::json!({
                "id": id,
                "from": from,
                "to": to,
                "kind": format!("{kind:?}"),
            }),
        ),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

fn projection(
    value: &Value,
    with_candidate_noise: bool,
    d6p_receipt: bool,
) -> (QualifiedProjectionV1, SemanticEnvironmentV1, DerivationProfileV1) {
    let env = environment();
    let derivation = derivation_profile();
    let design_commitment = canonical_sha256("integral-interop-1-design", value);
    let production_steps = value["design_version"]["parameters"]["production_steps"].clone();
    let bom = value["design_version"]["parameters"]["bill_of_materials_kg"].clone();

    let task_value = serde_json::json!({
        "design_version_id": value["design_version"]["id"],
        "production_steps": production_steps,
    });
    let task_commitment = canonical_sha256("integral-interop-1-cos-task", &task_value);

    let mut nodes = BTreeMap::new();
    nodes.insert(
        "oad:design-water-purifier:v1".into(),
        node(
            "oad:design-water-purifier:v1",
            ClaimGraphNodeKindV1::Statement,
            design_commitment,
        ),
    );
    nodes.insert(
        "cos:task:water-purifier:v1".into(),
        node(
            "cos:task:water-purifier:v1",
            ClaimGraphNodeKindV1::Evidence,
            task_commitment,
        ),
    );

    for material in ["silicone", "stainless-steel"] {
        let material_value = serde_json::json!({
            "design_version_id": value["design_version"]["id"],
            "material": material,
            "kg": bom[material],
        });
        let id = format!("oad:material:{material}");
        nodes.insert(
            id.clone(),
            node(
                &id,
                ClaimGraphNodeKindV1::Evidence,
                canonical_sha256("integral-interop-1-material", &material_value),
            ),
        );
    }

    let mut edges = BTreeMap::new();
    edges.insert(
        "oad->cos:production-plan".into(),
        edge(
            "oad->cos:production-plan",
            "oad:design-water-purifier:v1",
            "cos:task:water-purifier:v1",
            ClaimGraphEdgeKindV1::DerivedFrom,
        ),
    );
    for material in ["silicone", "stainless-steel"] {
        let id = format!("cos->material:{material}");
        edges.insert(
            id.clone(),
            edge(
                &id,
                "cos:task:water-purifier:v1",
                &format!("oad:material:{material}"),
                ClaimGraphEdgeKindV1::Supports,
            ),
        );
    }

    if with_candidate_noise {
        nodes.insert(
            "frs:observation:noise".into(),
            node(
                "frs:observation:noise",
                ClaimGraphNodeKindV1::Source,
                canonical_sha256(
                    "integral-interop-1-frs-noise",
                    &serde_json::json!({"signal": "candidate-only"}),
                ),
            ),
        );
        nodes.insert(
            "cos:candidate:unused".into(),
            node(
                "cos:candidate:unused",
                ClaimGraphNodeKindV1::Evidence,
                canonical_sha256(
                    "integral-interop-1-cos-noise",
                    &serde_json::json!({"candidate": "unused"}),
                ),
            ),
        );
        edges.insert(
            "frs->cos:candidate".into(),
            edge(
                "frs->cos:candidate",
                "frs:observation:noise",
                "cos:candidate:unused",
                ClaimGraphEdgeKindV1::Provenance,
            ),
        );
    }

    let d6p_current_receipt_commitments = if d6p_receipt {
        [d6p_receipt_commitment()].into_iter().collect()
    } else {
        BTreeSet::new()
    };

    let projection = QualifiedProjectionV1 {
        projection_id: "integral-interop-1-projection".into(),
        projection_version: "1".into(),
        canonicalization_version: "D6S-CANON-1".into(),
        source_dkg_snapshot_commitment: snapshot_commitment(),
        nodes,
        edges,
        d6p_current_receipt_commitments,
        d6n_context_commitment: None,
        d6o_context_commitment: None,
        semantic_environment_commitment: env.commitment(),
        derivation_profile_commitment: derivation.commitment(),
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    };

    (projection, env, derivation)
}

fn closure_profile(require_d6p_receipt: bool) -> DependencyClosureProfileV1 {
    let rules = [
        DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::DerivedFrom,
            from_kind: Some(ClaimGraphNodeKindV1::Statement),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        },
        DependencyRuleV1 {
            edge_kind: ClaimGraphEdgeKindV1::Supports,
            from_kind: Some(ClaimGraphNodeKindV1::Evidence),
            to_kind: Some(ClaimGraphNodeKindV1::Evidence),
            currentness: DependencyCurrentnessV1::Any,
        },
    ];

    DependencyClosureProfileV1 {
        profile_id: "integral-oad-cos-material-closure".into(),
        version: "1".into(),
        root_node_ids: ["oad:design-water-purifier:v1".into()]
            .into_iter()
            .collect(),
        required_node_ids: BTreeSet::new(),
        required_d6p_receipt_commitments: if require_d6p_receipt {
            [d6p_receipt_commitment()].into_iter().collect()
        } else {
            BTreeSet::new()
        },
        rules: rules.into_iter().collect(),
        excluded_boundary_policy:
            "Only selected OAD->COS and COS->material semantic edges expand the closure; FRS/provenance candidates are excluded."
                .into(),
        max_nodes: 16,
        max_edges: 16,
        claim_ceiling: D6S_CLAIM_CEILING.into(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CrossLayerObservation {
    oad_semantic_commitment: String,
    d6x_identity: String,
    d6x_certificate: String,
    d6x_status: DependencyClosureStatusV1,
    d6w_input: Option<String>,
    d6w_derivation: Option<String>,
}

impl CrossLayerObservation {
    fn from_closure(
        selected: String,
        closure: &cos_conformance::qualified_dependency_closure_d6x::DependencyClosureCertificateV1,
        input: Option<InputCommitmentV1>,
        derivation: Option<String>,
    ) -> Self {
        Self {
            oad_semantic_commitment: selected,
            d6x_identity: closure.closure_identity_commitment.clone(),
            d6x_certificate: closure.commitment.clone(),
            d6x_status: closure.status,
            d6w_input: input.map(|v| v.commitment),
            d6w_derivation: derivation,
        }
    }
}

fn evaluate(
    value: &Value,
    with_candidate_noise: bool,
    d6p_receipt: bool,
    require_d6p_receipt: bool,
    runtime_evidence: bool,
) -> Option<CrossLayerObservation> {
    evaluate_with_graph_mutation(
        value,
        with_candidate_noise,
        d6p_receipt,
        require_d6p_receipt,
        runtime_evidence,
        None,
    )
}

fn evaluate_with_graph_mutation(
    value: &Value,
    with_candidate_noise: bool,
    d6p_receipt: bool,
    require_d6p_receipt: bool,
    runtime_evidence: bool,
    graph_mutation: Option<&str>,
) -> Option<CrossLayerObservation> {
    let selected = selected_oad_design_semantic_commitment_checked(value).ok()?;
    let (mut projection, env, derivation) =
        projection(value, with_candidate_noise, d6p_receipt);

    if let Some(mutation) = graph_mutation {
        match mutation {
            "selected-node-commitment" => {
                let node = projection.nodes.get_mut("oad:design-water-purifier:v1")?;
                node.content_commitment = "mutated-selected-node-content".into();
                node.node_commitment = node.recomputed_commitment();
            }
            "selected-edge-commitment" => {
                let edge = projection.edges.get_mut("oad->cos:production-plan")?;
                edge.kind = ClaimGraphEdgeKindV1::Supports;
                edge.edge_commitment = edge.recomputed_commitment();
            }
            "selected-edge-removal" => {
                projection.edges.remove("oad->cos:production-plan");
            }
            "wrong-edge-kind" => {
                let edge = projection.edges.get_mut("oad->cos:production-plan")?;
                edge.kind = ClaimGraphEdgeKindV1::Provenance;
                edge.edge_commitment = canonical_sha256(
                    "integral-interop-1-edge",
                    &serde_json::json!({
                        "id": edge.edge_id,
                        "from": edge.from_node_id,
                        "to": edge.to_node_id,
                        "kind": format!("{:?}", edge.kind),
                    }),
                );
            }
            "irrelevant-node-commitment" => {
                let node = projection.nodes.get_mut("frs:observation:noise")?;
                node.node_commitment = "mutated-irrelevant-node".into();
            }
            "irrelevant-edge-commitment" => {
                let edge = projection.edges.get_mut("frs->cos:candidate")?;
                edge.edge_commitment = "mutated-irrelevant-edge".into();
            }
            other => panic!("unsupported graph mutation {other}"),
        }
    }

    let mut closure = compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(require_d6p_receipt),
    )?;

    if runtime_evidence {
        let dependency = SemanticDependencyReferenceV1::node(
            "oad:design-water-purifier:v1",
            projection
                .nodes
                .get("oad:design-water-purifier:v1")
                .map(|node| node.node_commitment.clone()),
        );
        closure.resolution_evidence.insert(
            dependency,
            SemanticDependencyResolutionEvidenceV1 {
                retrieval_reference: Some("runtime://resolver/42".into()),
                observed_commitment: projection
                    .nodes
                    .get("oad:design-water-purifier:v1")
                    .map(|node| node.node_commitment.clone()),
                qualification_context_commitment: Some("runtime-qualification-1".into()),
            },
        );
        closure.commitment = closure.recompute();
    }

    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(require_d6p_receipt), &derivation);
    let derivation_id = input
        .as_ref()
        .and_then(|input| DerivationCommitmentV1::new(input, &derivation, None))
        .map(|d| d.commitment);

    Some(CrossLayerObservation::from_closure(
        selected,
        &closure,
        input,
        derivation_id,
    ))
}

fn assert_delta(
    id: &str,
    baseline: &CrossLayerObservation,
    actual: &CrossLayerObservation,
    expected: &Value,
) {
    let checks = [
        ("oad_semantic_commitment", baseline.oad_semantic_commitment != actual.oad_semantic_commitment),
        ("d6x_identity", baseline.d6x_identity != actual.d6x_identity),
        ("d6x_certificate", baseline.d6x_certificate != actual.d6x_certificate),
        ("d6w_input", baseline.d6w_input != actual.d6w_input),
        ("d6w_derivation", baseline.d6w_derivation != actual.d6w_derivation),
    ];
    for (field, changed) in checks {
        assert_eq!(
            expected[field].as_bool().expect("expected delta boolean"),
            changed,
            "{id}: unexpected {field} propagation"
        );
    }
    assert_eq!(
        expected["d6x_status"].as_str().expect("expected status"),
        serde_json::to_string(&actual.d6x_status)
            .expect("status serialization")
            .trim_matches('"'),
        "{id}: unexpected D6X status"
    );
}



fn path_tokens(path: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    for segment in path.split('.') {
        let mut rest = segment;
        while let Some(open) = rest.find('[') {
            if open > 0 {
                tokens.push(rest[..open].to_owned());
            }
            let close = rest[open..]
                .find(']')
                .expect("array path token must close");
            tokens.push(rest[open + 1..open + close].to_owned());
            rest = &rest[open + close + 1..];
        }
        if !rest.is_empty() {
            tokens.push(rest.to_owned());
        }
    }
    tokens
}

fn set_path(root: &mut Value, path: &str, replacement: Value) {
    let tokens = path_tokens(path);
    assert!(!tokens.is_empty());

    fn descend(current: &mut Value, tokens: &[String], replacement: Value) {
        if tokens.len() == 1 {
            match current {
                Value::Object(map) => {
                    map.insert(tokens[0].clone(), replacement);
                }
                Value::Array(values) => {
                    let index: usize = tokens[0].parse().expect("array index");
                    values[index] = replacement;
                }
                _ => panic!("cannot descend into scalar"),
            }
            return;
        }

        match current {
            Value::Object(map) => descend(
                map.get_mut(&tokens[0]).expect("object path component"),
                &tokens[1..],
                replacement,
            ),
            Value::Array(values) => {
                let index: usize = tokens[0].parse().expect("array index");
                descend(&mut values[index], &tokens[1..], replacement);
            }
            _ => panic!("cannot descend into scalar"),
        }
    }

    descend(root, &tokens, replacement);
}

#[test]
fn declarative_cross_layer_corpus_is_self_describing_and_executable() {
    let corpus: Value = serde_json::from_str(include_str!(
        "../testdata/integral_interop_1_cross_layer_vectors.json"
    ))
    .expect("cross-layer corpus must be valid JSON");

    assert_eq!(corpus["profile"], "integral-interop-1");
    assert_eq!(
        corpus["projection_version"],
        "integral-interop-1-design-semantic-v1"
    );
    assert_eq!(corpus["classification"], "cross-layer-propagation-v2");

    let baseline = fixture();
    let baseline_eval =
        evaluate(&baseline, false, false, false, false).expect("baseline must resolve");

    for vector in corpus["vectors"].as_array().expect("vectors array") {
        let id = vector["id"].as_str().expect("vector id");
        let operation = vector["operation"].as_str().expect("vector operation");
        let expected = &vector["expected"];
        let expected_status = expected["d6x_status"]
            .as_str()
            .expect("expected D6X status");
        let expected_rejection_boundary = expected["earliest_rejection_boundary"]
            .as_str()
            .expect("expected earliest rejection boundary");

        assert!(
            matches!(
                expected_status,
                "Complete"
                    | "BlockedMissingDependency"
                    | "BlockedCurrentness"
                    | "BlockedResourceLimit"
            ),
            "{id}: unknown D6X status"
        );

        let mut mutated = baseline.clone();
        let mut with_candidate_noise = false;
        let mut d6p_receipt = false;
        let mut require_d6p_receipt = false;
        let mut runtime_evidence = false;

        match operation {
            "set" => set_path(
                &mut mutated,
                vector["path"].as_str().expect("set path"),
                vector["value"].clone(),
            ),
            "candidate-noise" => with_candidate_noise = true,
            "require-d6p-receipt" => {
                require_d6p_receipt = true;
                d6p_receipt = vector["value"].as_bool().expect("receipt boolean");
            }
            "runtime-evidence" => runtime_evidence = true,
            "graph-mutation" => {
                let mutation = vector["mutation"].as_str().expect("graph mutation");
                if mutation.starts_with("irrelevant-") {
                    with_candidate_noise = true;
                }
            }
            other => panic!("{id}: unsupported operation {other}"),
        }

        let computed_rejection_boundary = if expected["valid"].as_bool() == Some(false) {
            "OADSemanticValidation"
        } else if expected_status == "BlockedMissingDependency" {
            "D6WInputConsumption"
        } else {
            "None"
        };
        assert_eq!(
            expected_rejection_boundary,
            computed_rejection_boundary,
            "{id}: rejection boundary must match the executable gate"
        );

        if expected["valid"].as_bool() == Some(false) {
            assert!(
                validate_selected_oad_design_semantics(&mutated).is_err(),
                "{id}: validation must reject"
            );
            assert!(
                evaluate(
                    &mutated,
                    with_candidate_noise,
                    d6p_receipt,
                    require_d6p_receipt,
                    runtime_evidence,
                )
                .is_none(),
                "{id}: invalid mutation must produce no observation"
            );
            continue;
        }

        let actual = evaluate_with_graph_mutation(
            &mutated,
            with_candidate_noise,
            d6p_receipt,
            require_d6p_receipt,
            runtime_evidence,
            vector["mutation"].as_str(),
        )
        .expect(id);
        assert_delta(id, &baseline_eval, &actual, expected);
    }
}

#[test]
fn cross_layer_golden_vectors_pin_exact_commitments_and_gate() {
    let baseline = fixture();
    let baseline_eval = evaluate(&baseline, false, false, false, false).expect("baseline");

    // These are cryptographic golden values for the current ReferenceModelOnly
    // fixture. The preimage at every boundary is the exact canonical D6S-CANON-1
    // serialization produced by the corresponding production commitment function.
    assert_eq!(
        selected_oad_design_semantic_commitment_checked(&baseline).unwrap(),
        "e06ba541daf85da423cea4d6cd80b6efd0dd106c122079450e9acfc8ee0cd460"
    );
    assert_eq!(snapshot_commitment(),
        "2e07a630c8b1579aaee1c6accc623b917b99759bd115445c45d9af330a299ad3");
    assert_eq!(
        environment().commitment(),
        "e9c7b6422e52c35dab2e8c9d1fc1ad093a34c7ee91efe8267b479ce610233895"
    );
    assert_eq!(
        derivation_profile().commitment(),
        "69859b57a8cd86d7f0f84aef2cf9eeb3c793d2f4c399f5fe7819ebf961dcb640"
    );
    assert_eq!(
        closure_profile(false).commitment(),
        "3781501a08dd2cb090d577171ed431062875660a7b336844ea363f1f60ad9352"
    );
    assert_eq!(
        baseline_eval.d6x_identity,
        "9c311aac60e317b088eeeb310a9888f0fcef617dd6a22d5cf278725e00a4dc08"
    );
    assert_eq!(
        baseline_eval.d6x_certificate,
        "e95ddbe042c48fe5f9f98a4186c2df0dcbfc8057eae0b76849a9194883e43301"
    );
    assert_eq!(
        baseline_eval.d6w_input.as_deref(),
        Some("d7bafeb259fe2df610f7f5c6801ef466a556484318a84bb4d9211e0ab90480d8")
    );
    assert_eq!(
        baseline_eval.d6w_derivation.as_deref(),
        Some("1e9e7cb826206360c6c9500fd807c716707695f6b0a16f6aec10ddf92683ab05")
    );

    let blocked = evaluate(&baseline, false, false, true, false).expect("blocked");
    assert_eq!(
        blocked.d6x_status,
        DependencyClosureStatusV1::BlockedMissingDependency
    );
    assert_eq!(
        blocked.d6x_identity,
        "37073b6d18388659d72815159107f80206bf6c192eaf332d403ac1834098df7c"
    );
    assert_eq!(
        blocked.d6x_certificate,
        "f420b1c9fc3d77edc4a4a1c419fab2f810c2e52e7d48241597953bdfeb935c8f"
    );
    assert!(blocked.d6w_input.is_none());
    assert!(blocked.d6w_derivation.is_none());

    let present = evaluate(&baseline, false, true, true, false).expect("present");
    assert_eq!(present.d6x_status, DependencyClosureStatusV1::Complete);
    assert_eq!(
        d6p_receipt_commitment(),
        "12699e2590d7e3b303a0f36908228dd296428949621e2a24757f6a7fb05022a4"
    );
    assert_eq!(
        closure_profile(true).commitment(),
        "6276d703e5f6e05989e91f000b31472ac87c3a828f7f2cb868e40619d79fbb6f"
    );
    assert_eq!(
        present.d6x_identity,
        "ff4f9e50756e664f56da2b5a154ef69f438db7430de2e549243dbffe6c00b2bc"
    );
    assert_eq!(
        present.d6x_certificate,
        "5841b36ab8b4f044c6ff20982b27fef8847f726472765fa6e9b952c1d2662948"
    );
    assert_eq!(
        present.d6w_input.as_deref(),
        Some("d79a33934897a889456e1f18fbae361f806203a0f7d1cec573f6ee902425f9a6")
    );
    assert_eq!(
        present.d6w_derivation.as_deref(),
        Some("b0555ba3a566f940eccfc2a459a5655efe41f9d164d4711e6f4b4917371da8b5")
    );

    // The missing dependency is admitted into D6X's semantic state but is
    // rejected at the first downstream consumption boundary; no D6W derivation
    // may be constructed from the blocked closure.
    assert_ne!(blocked.d6x_identity, baseline_eval.d6x_identity);
    assert_ne!(present.d6x_identity, blocked.d6x_identity);
}
 
#[test]
fn end_to_end_authoritative_d6p_to_d6s_d6x_witness() {
    let baseline = fixture();
    let (_d6p, d6p_receipt) = actual_d6p_fixture();
    let (mut projection, mut environment, derivation_profile) =
        projection(&baseline, false, false);

    let receipt_commitment = d6p_receipt.receipt_commitment.clone();
    projection.d6p_current_receipt_commitments =
        [receipt_commitment.clone()].into_iter().collect();
    environment.d6p_eligibility_context_root =
        Some(commitment_set_digest(&projection.d6p_current_receipt_commitments));
    projection.semantic_environment_commitment = environment.commitment();

    let mut closure_profile = closure_profile(false);
    closure_profile
        .required_d6p_receipt_commitments
        .insert(receipt_commitment.clone());

    let d6x = compute_dependency_closure_from_authoritative_d6p_at_frontier(
        &projection,
        &environment,
        &derivation_profile,
        &closure_profile,
        std::slice::from_ref(&d6p_receipt),
        std::slice::from_ref(&_d6p),
        Some("integral-frontier-1"),
    )
    .expect("strict D6X must consume the authoritative D6P receipt/composition");
    assert_eq!(d6x.status, DependencyClosureStatusV1::Complete);
    assert!(d6x.valid());
    assert!(d6x.included_d6p_receipt_commitments.contains(&receipt_commitment));

    let d6w_input = InputCommitmentV1::from_projection_with_authoritative_d6p(
        &projection,
        &environment,
        &closure_profile,
        &derivation_profile,
        std::slice::from_ref(&d6p_receipt),
        std::slice::from_ref(&_d6p),
        Some("integral-frontier-1"),
    )
    .expect("strict D6W input must consume the same qualified D6P boundary");
    assert!(d6w_input.valid());

    let d6w_derivation =
        DerivationCommitmentV1::new(&d6w_input, &derivation_profile, None)
            .expect("D6W derivation must bind the qualified input");

    let result_commitment = d6w_derivation.commitment.clone();
    let d6s = build_canonical_receipt_with_authoritative_d6p(
        &projection,
        &environment,
        &derivation_profile,
        std::slice::from_ref(&d6p_receipt),
        std::slice::from_ref(&_d6p),
        DerivationResultStatusV1::Supported,
        result_commitment,
        false,
        false,
    )
    .expect("strict D6S must consume the same committed D6P composition");
    assert!(d6s.commitment_matches());
    assert!(verify_canonical_receipt_with_authoritative_d6p(
        &d6s,
        &projection,
        &environment,
        &derivation_profile,
        std::slice::from_ref(&d6p_receipt),
        std::slice::from_ref(&_d6p),
    ));

    // The D6S receipt binds the D6W derivation as its result commitment,
    // while D6X independently binds the semantic dependency closure. The
    // branches share the same committed D6P input without conflating their
    // different semantic purposes.
    assert_eq!(d6s.result_commitment, d6w_derivation.commitment);
    assert_eq!(d6s.d6p_current_receipt_commitments, projection.d6p_current_receipt_commitments);
}

#[test]
fn strict_d6s_d6p_boundary_rejects_self_consistent_receipt_substitution() {
    let baseline = fixture();
    let (composition, receipt) = actual_d6p_fixture();
    let (projection, base_environment, derivation_profile) = projection(&baseline, false, false);
    let receipt_set: BTreeSet<String> = [receipt.receipt_commitment.clone()].into_iter().collect();
    let mut environment = base_environment;
    environment.d6p_eligibility_context_root = Some(commitment_set_digest(&receipt_set));
    let mut projection = projection;
    projection.d6p_current_receipt_commitments = receipt_set;
    projection.semantic_environment_commitment = environment.commitment();

    let result_commitment = canonical_sha256(
        "integral-interop-1-result",
        &serde_json::json!({"result": "supported"}),
    );
    let receipt = std::slice::from_ref(&receipt);
    let d6s = build_canonical_receipt_with_authoritative_d6p(
        &projection,
        &environment,
        &derivation_profile,
        receipt,
        std::slice::from_ref(&composition),
        DerivationResultStatusV1::Supported,
        result_commitment,
        false,
        false,
    ).expect("strict D6S gate must accept the committed D6P composition");
    assert!(verify_canonical_receipt_with_authoritative_d6p(
        &d6s,
        &projection,
        &environment,
        &derivation_profile,
        receipt,
        std::slice::from_ref(&composition),
    ));

    let mut forged = receipt[0].clone();
    forged.provider_id = "integral-provider-attacker".into();
    forged.receipt_commitment = forged.recomputed_commitment();
    assert!(forged.semantically_valid());

    let forged_receipts = [forged.clone()];
    let forged_set: BTreeSet<String> = [forged.receipt_commitment.clone()].into_iter().collect();
    let mut forged_environment = environment.clone();
    forged_environment.d6p_eligibility_context_root = Some(commitment_set_digest(&forged_set));
    let mut forged_projection = projection.clone();
    forged_projection.d6p_current_receipt_commitments = forged_set;
    forged_projection.semantic_environment_commitment = forged_environment.commitment();

    assert!(
        cos_conformance::canonical_derivation_receipt::build_canonical_receipt(
            &forged_projection,
            &forged_environment,
            &derivation_profile,
            &forged_receipts,
            DerivationResultStatusV1::Supported,
            canonical_sha256(
                "integral-interop-1-result",
                &serde_json::json!({"result": "supported-forged"}),
            ),
            false,
            false,
        ).is_some(),
        "plain D6S integrity builder intentionally does not claim D6P composition provenance"
    );
    assert!(
        build_canonical_receipt_with_authoritative_d6p(
            &forged_projection,
            &forged_environment,
            &derivation_profile,
            &forged_receipts,
            std::slice::from_ref(&composition),
            DerivationResultStatusV1::Supported,
            canonical_sha256(
                "integral-interop-1-result",
                &serde_json::json!({"result": "supported-forged"}),
            ),
            false,
            false,
        ).is_none(),
        "strict D6S gate must reject a self-consistent receipt that diverges from its composition"
    );
}

#[test]
fn strict_d6x_d6p_boundary_accepts_committed_receipt_and_rejects_substitution() {
    let baseline = fixture();
    let (composition, receipt) = actual_d6p_fixture();
    let (projection, environment, derivation_profile) = projection(&baseline, false, false);

    let mut qualified_projection = projection;
    qualified_projection.d6p_current_receipt_commitments =
        [receipt.receipt_commitment.clone()].into_iter().collect();

    let profile = DependencyClosureProfileV1 {
        required_d6p_receipt_commitments:
            [receipt.receipt_commitment.clone()].into_iter().collect(),
        ..closure_profile(true)
    };

    let qualified = compute_dependency_closure_from_authoritative_d6p_at_frontier(
        &qualified_projection,
        &environment,
        &derivation_profile,
        &profile,
        std::slice::from_ref(&receipt),
        std::slice::from_ref(&composition),
        Some("integral-frontier-1"),
    )
    .expect("committed D6P receipt must qualify the D6X boundary");

    assert_eq!(qualified.status, DependencyClosureStatusV1::Complete);
    assert!(qualified.valid());
    assert!(qualified.included_d6p_receipt_commitments.contains(&receipt.receipt_commitment));

    let mut forged = receipt.clone();
    forged.provider_id = "integral-provider-attacker".into();
    forged.receipt_commitment = forged.recomputed_commitment();
    assert!(forged.commitment_matches());
    assert!(
        compute_dependency_closure_from_authoritative_d6p_at_frontier(
            &qualified_projection,
            &environment,
            &derivation_profile,
            &profile,
            std::slice::from_ref(&forged),
            std::slice::from_ref(&composition),
            Some("integral-frontier-1"),
        )
        .is_none(),
        "self-consistent receipt substitution must fail at the D6P provenance boundary"
    );
}

#[test]
fn cross_layer_mutation_matrix_is_executable() {
    let baseline = fixture();
    let baseline_eval =
        evaluate(&baseline, false, false, false, false).expect("baseline must resolve");
    assert!(validate_selected_oad_design_semantics(&baseline).is_ok());
    assert!(baseline_eval.d6w_input.is_some());

    let mut metadata = baseline.clone();
    metadata["certification"]["documentation_bundle_uri"] =
        Value::from("urn:integral:bundle:changed");
    let metadata_eval =
        evaluate(&metadata, false, false, false, false).expect("metadata must resolve");
    assert_eq!(baseline_eval.oad_semantic_commitment, metadata_eval.oad_semantic_commitment);
    assert_eq!(baseline_eval.d6x_identity, metadata_eval.d6x_identity);
    assert_eq!(baseline_eval.d6x_certificate, metadata_eval.d6x_certificate);
    assert_eq!(baseline_eval.d6w_input, metadata_eval.d6w_input);
    assert_eq!(baseline_eval.d6w_derivation, metadata_eval.d6w_derivation);

    let mut selected = baseline.clone();
    selected["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] =
        Value::from(3);
    let selected_eval =
        evaluate(&selected, false, false, false, false).expect("selected mutation must resolve");
    assert_ne!(baseline_eval.oad_semantic_commitment, selected_eval.oad_semantic_commitment);
    assert_ne!(baseline_eval.d6x_identity, selected_eval.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, selected_eval.d6x_certificate);
    assert_ne!(baseline_eval.d6w_input, selected_eval.d6w_input);
    assert_ne!(baseline_eval.d6w_derivation, selected_eval.d6w_derivation);

    let noisy_eval =
        evaluate(&baseline, true, false, false, false).expect("candidate noise must resolve");
    assert_eq!(baseline_eval.oad_semantic_commitment, noisy_eval.oad_semantic_commitment);
    assert_eq!(baseline_eval.d6x_identity, noisy_eval.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, noisy_eval.d6x_certificate);
    assert_eq!(baseline_eval.d6w_input, noisy_eval.d6w_input);
    assert_eq!(baseline_eval.d6w_derivation, noisy_eval.d6w_derivation);

    let mut invalid = baseline.clone();
    invalid["design_version"]["parameters"]["bill_of_materials_kg"]["silicone"] =
        serde_json::json!(0.25);
    assert!(validate_selected_oad_design_semantics(&invalid).is_err());
    assert!(evaluate(&invalid, false, false, false, false).is_none());

    let missing_receipt =
        evaluate(&baseline, false, false, true, false).expect("blocked D6X must still certify its boundary");
    assert_eq!(
        missing_receipt.d6x_status,
        DependencyClosureStatusV1::BlockedMissingDependency
    );
    assert!(missing_receipt.d6w_input.is_none());
    assert!(missing_receipt.d6w_derivation.is_none());
    assert_ne!(baseline_eval.d6x_identity, missing_receipt.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, missing_receipt.d6x_certificate);

    let present_receipt =
        evaluate(&baseline, false, true, true, false).expect("required D6P receipt should unblock D6X");
    assert_eq!(present_receipt.d6x_status, DependencyClosureStatusV1::Complete);
    assert_ne!(baseline_eval.d6x_identity, present_receipt.d6x_identity);
    assert_ne!(baseline_eval.d6x_certificate, present_receipt.d6x_certificate);
    assert!(present_receipt.d6w_input.is_some());
    assert!(present_receipt.d6w_derivation.is_some());
    assert_ne!(baseline_eval.d6w_input, present_receipt.d6w_input);
    assert_ne!(baseline_eval.d6w_derivation, present_receipt.d6w_derivation);
}

#[test]
fn runtime_resolution_evidence_does_not_propagate_into_identity_layers() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(false),
    )
    .expect("baseline closure");

    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
        .expect("baseline D6W input");
    let mut observed = closure.clone();
    observed.resolution_evidence.insert(
        cos_conformance::qualified_dependency_closure_d6x::SemanticDependencyReferenceV1::node(
            "oad:design-water-purifier:v1",
            Some(
                projection
                    .nodes
                    .get("oad:design-water-purifier:v1")
                    .unwrap()
                    .node_commitment
                    .clone(),
            ),
        ),
        cos_conformance::qualified_dependency_closure_d6x::SemanticDependencyResolutionEvidenceV1 {
            retrieval_reference: Some("runtime://resolver/42".into()),
            observed_commitment: Some(
                projection
                    .nodes
                    .get("oad:design-water-purifier:v1")
                    .unwrap()
                    .node_commitment
                    .clone(),
            ),
            qualification_context_commitment: Some("qualification-context-1".into()),
        },
    );

    assert_eq!(closure.closure_identity_commitment, observed.closure_identity_commitment);
    assert_ne!(closure.commitment, observed.recompute());

    let observed_input = InputCommitmentV1::from_projection(&projection, &env, &observed, &closure_profile(false), &derivation)
        .expect("runtime evidence must not block a complete closure");
    assert_eq!(input.commitment, observed_input.commitment);
}

#[test]
fn stale_selected_node_commitment_is_rejected_at_d6x_boundary() {
    let baseline = fixture();
    let (mut projection, env, derivation) = projection(&baseline, false, false);
    let node = projection.nodes.get_mut("oad:design-water-purifier:v1").expect("selected node");
    let stale = node.node_commitment.clone();
    node.content_commitment = "semantic-content-mutated-with-stale-binding".into();
    node.node_commitment = stale;

    assert!(!projection.nodes["oad:design-water-purifier:v1"].commitment_matches());
    assert!(compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(false),
    ).is_none());
}

#[test]
fn stale_selected_edge_commitment_is_rejected_at_d6x_boundary() {
    let baseline = fixture();
    let (mut projection, env, derivation) = projection(&baseline, false, false);
    let edge = projection.edges.get_mut("oad->cos:production-plan").expect("selected edge");
    let stale = edge.edge_commitment.clone();
    edge.kind = ClaimGraphEdgeKindV1::DerivedFrom;
    edge.edge_commitment = stale;

    assert!(!projection.edges["oad->cos:production-plan"].commitment_matches());
    assert!(compute_dependency_closure(
        &projection,
        &env,
        &derivation,
        &closure_profile(false),
    ).is_none());
}


#[test]
fn d6w_standalone_input_rejects_duplicate_selected_commitments() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(false))
        .expect("baseline closure");
    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
        .expect("baseline input");

    let mut duplicated = input.clone();
    duplicated.nodes.push(duplicated.nodes[0].clone());
    duplicated.commitment = duplicated.recompute();
    assert!(
        !duplicated.valid(),
        "re-hashing must not make duplicate selected commitments canonical"
    );
}

#[test]
fn d6w_standalone_input_rejects_noncanonical_selected_commitment_order() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(false))
        .expect("baseline closure");
    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
        .expect("baseline input");

    assert!(input.nodes.windows(2).all(|pair| pair[0] < pair[1]));
    assert!(input.edges.windows(2).all(|pair| pair[0] < pair[1]));

    let mut reordered = input.clone();
    reordered.nodes.reverse();
    reordered.commitment = reordered.recompute();
    assert!(
        !reordered.valid(),
        "re-hashing must not make reordered selected commitments canonical"
    );
}

#[test]
fn d6w_standalone_input_rejects_noncanonical_selected_commitment() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, false);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(false))
        .expect("baseline closure");
    let mut input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(false), &derivation)
        .expect("baseline input");
    input.nodes[0] = "LEGACY-SYMBOLIC".into();
    input.commitment = input.recompute();
    assert!(!input.valid(), "re-hashing cannot make a noncanonical selected commitment valid");
}

#[test]
fn d6w_standalone_input_rejects_noncanonical_edge_and_receipt_commitments() {
    let baseline = fixture();
    let (projection, env, derivation) = projection(&baseline, false, true);
    let closure = compute_dependency_closure(&projection, &env, &derivation, &closure_profile(true))
        .expect("baseline closure");
    let input = InputCommitmentV1::from_projection(&projection, &env, &closure, &closure_profile(true), &derivation)
        .expect("baseline input");

    let mut edge_mutated = input.clone();
    edge_mutated.edges[0] = "edge-symbolic".into();
    edge_mutated.commitment = edge_mutated.recompute();
    assert!(!edge_mutated.valid());

    let mut receipt_mutated = input;
    receipt_mutated.d6p_receipts[0].clear();
    receipt_mutated.commitment = receipt_mutated.recompute();
    assert!(!receipt_mutated.valid());
}