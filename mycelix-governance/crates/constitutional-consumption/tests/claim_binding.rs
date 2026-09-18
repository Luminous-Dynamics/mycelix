use constitutional_consumption::{
    BoundFinalityProof, ClaimBinding, ConsumptionClaim, ConsumptionError, ConsumptionKey,
    EvidenceAvailability, FinalityProfile, FinalityProof, FinalityRequirement, RevocationCutoff,
    WitnessAttestation,
};
use constitutional_envelope::MatterId;

fn claim() -> ConsumptionClaim {
    ConsumptionClaim {
        claim_id: "claim-a".into(),
        key: ConsumptionKey {
            envelope_digest: "sha256:envelope-a".into(),
            nonce: "nonce-a".into(),
            use_index: 0,
            jurisdiction: "region-a".into(),
        },
        matter: MatterId {
            namespace: "appropriation".into(),
            stable_id: "matter-42".into(),
        },
        target_digest: "sha256:target-a".into(),
        payload_digest: "sha256:payload-a".into(),
        budget_id: "budget-root".into(),
    }
}

fn proof() -> FinalityProof {
    FinalityProof {
        proof_id: "proof-a".into(),
        claim_id: "claim-a".into(),
        profile: FinalityProfile::WitnessedSingleSpend,
        finalized_at_seq: 10,
        dependency_state: EvidenceAvailability::Complete,
        witnesses: vec![
            WitnessAttestation {
                witness_id: "w1".into(),
                domain_id: "d1".into(),
            },
            WitnessAttestation {
                witness_id: "w2".into(),
                domain_id: "d2".into(),
            },
        ],
        consensus_ref: None,
    }
}

fn requirement() -> FinalityRequirement {
    FinalityRequirement {
        minimum_profile: FinalityProfile::WitnessedSingleSpend,
        min_witnesses: 2,
        min_distinct_domains: 2,
        revocation_cutoff: RevocationCutoff::Finality,
    }
}

fn assert_binding_rejected(binding: ClaimBinding) {
    let bound = BoundFinalityProof {
        proof: proof(),
        claim_binding: binding,
    };
    assert_eq!(
        bound.validate_against(&claim(), &requirement()),
        Err(ConsumptionError::ProofClaimBindingMismatch)
    );
}

#[test]
fn canonical_binding_is_deterministic_and_domain_separated() {
    let c = claim();
    let a = c.canonical_binding_bytes().unwrap();
    let b = c.canonical_binding_bytes().unwrap();
    assert_eq!(a, b);
    assert!(a.starts_with(constitutional_consumption::CLAIM_BINDING_DOMAIN_SEPARATOR));
    assert_eq!(c.binding().schema_version, 1);
}

#[test]
fn exact_bound_proof_validates() {
    let c = claim();
    let bound = BoundFinalityProof::new(proof(), &c).unwrap();
    assert!(bound.validate_against(&c, &requirement()).is_ok());
}

#[test]
fn same_claim_id_with_mutated_envelope_digest_is_rejected() {
    let mut binding = claim().binding();
    binding.envelope_digest = "sha256:other-envelope".into();
    assert_binding_rejected(binding);
}

#[test]
fn same_claim_id_with_mutated_nonce_is_rejected() {
    let mut binding = claim().binding();
    binding.nonce = "other-nonce".into();
    assert_binding_rejected(binding);
}

#[test]
fn same_claim_id_with_mutated_use_index_is_rejected() {
    let mut binding = claim().binding();
    binding.use_index = 1;
    assert_binding_rejected(binding);
}

#[test]
fn same_claim_id_with_mutated_jurisdiction_is_rejected() {
    let mut binding = claim().binding();
    binding.jurisdiction = "region-b".into();
    assert_binding_rejected(binding);
}

#[test]
fn same_claim_id_with_mutated_matter_is_rejected() {
    let mut binding = claim().binding();
    binding.matter.stable_id = "matter-99".into();
    assert_binding_rejected(binding);
}

#[test]
fn same_claim_id_with_mutated_target_is_rejected() {
    let mut binding = claim().binding();
    binding.target_digest = "sha256:other-target".into();
    assert_binding_rejected(binding);
}

#[test]
fn same_claim_id_with_mutated_payload_is_rejected() {
    let mut binding = claim().binding();
    binding.payload_digest = "sha256:other-payload".into();
    assert_binding_rejected(binding);
}

#[test]
fn same_claim_id_with_mutated_budget_is_rejected() {
    let mut binding = claim().binding();
    binding.budget_id = "other-budget".into();
    assert_binding_rejected(binding);
}

#[test]
fn copied_claim_id_cannot_authorize_different_claim_body() {
    let a = claim();
    let bound = BoundFinalityProof::new(proof(), &a).unwrap();

    let mut b = a.clone();
    b.target_digest = "sha256:attacker-target".into();
    assert_eq!(b.claim_id, a.claim_id);

    assert_eq!(
        bound.validate_against(&b, &requirement()),
        Err(ConsumptionError::ProofClaimBindingMismatch)
    );
}

#[test]
fn canonical_bytes_change_for_each_security_relevant_field() {
    let base = claim();
    let base_bytes = base.canonical_binding_bytes().unwrap();

    let mut variants = Vec::new();

    let mut c = base.clone();
    c.key.envelope_digest = "sha256:e2".into();
    variants.push(c);

    let mut c = base.clone();
    c.key.nonce = "n2".into();
    variants.push(c);

    let mut c = base.clone();
    c.key.use_index = 1;
    variants.push(c);

    let mut c = base.clone();
    c.key.jurisdiction = "region-b".into();
    variants.push(c);

    let mut c = base.clone();
    c.matter.namespace = "other".into();
    variants.push(c);

    let mut c = base.clone();
    c.target_digest = "sha256:t2".into();
    variants.push(c);

    let mut c = base.clone();
    c.payload_digest = "sha256:p2".into();
    variants.push(c);

    let mut c = base.clone();
    c.budget_id = "budget-b".into();
    variants.push(c);

    for variant in variants {
        assert_ne!(base_bytes, variant.canonical_binding_bytes().unwrap());
    }
}


#[test]
fn canonical_binding_v1_has_a_stable_golden_encoding() {
    let encoded = claim().canonical_binding_bytes().unwrap();
    let hex = encoded
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();

    assert_eq!(
        hex,
        "4d5943454c49582d434f4e535449545554494f4e414c2d434c41494d2d42494e44494e4700563100000100000007636c61696d2d61000000117368613235363a656e76656c6f70652d61000000076e6f6e63652d610000000000000008726567696f6e2d610000000d617070726f7072696174696f6e000000096d61747465722d34320000000f7368613235363a7461726765742d61000000107368613235363a7061796c6f61642d610000000b6275646765742d726f6f74"
    );
}
