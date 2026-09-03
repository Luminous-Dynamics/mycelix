use content_fabric_integrity::{
    algorithm_anchor_v1, all_providers_anchor_v1, digest_anchor_v1, is_valid_anchor_v1,
    provider_binding_bytes_v1, ContentDigestRefV1, DigestAlgorithmV1, FailureDomainClaimV1,
    FailureDomainKindV1, ProviderAdvertisementV1, IROH_ALPN_V1, SCHEMA_VERSION_V1,
};
use ed25519_dalek::{Signer, SigningKey, Verifier};
use hdi::prelude::{ActionHash, AgentPubKey};

fn sample_advertisement() -> (SigningKey, ProviderAdvertisementV1) {
    let endpoint_key = SigningKey::from_bytes(&[7_u8; 32]);
    let mut ad = ProviderAdvertisementV1 {
        schema_version: SCHEMA_VERSION_V1,
        provider: AgentPubKey::from_raw_32(vec![3_u8; 32]),
        binding_prev_action: ActionHash::from_raw_32(vec![4_u8; 32]),
        iroh_endpoint_id: endpoint_key.verifying_key().to_bytes(),
        endpoint_binding_signature: [0_u8; 64],
        protocol: IROH_ALPN_V1.to_string(),
        supported_algorithms: vec![DigestAlgorithmV1::Blake3_256, DigestAlgorithmV1::Sha256],
        max_blob_size_bytes: 1024,
        ttl_seconds: 300,
        failure_domains: vec![FailureDomainClaimV1 {
            kind: FailureDomainKindV1::Region,
            value: "za-gauteng".to_string(),
        }],
    };
    ad.endpoint_binding_signature = endpoint_key
        .sign(&provider_binding_bytes_v1(&ad))
        .to_bytes();
    (endpoint_key, ad)
}

#[test]
fn endpoint_binding_commits_capability_and_chain_head_but_not_signature_bytes() {
    let (key, ad) = sample_advertisement();
    key.verifying_key()
        .verify(
            &provider_binding_bytes_v1(&ad),
            &ed25519_dalek::Signature::from_bytes(&ad.endpoint_binding_signature),
        )
        .unwrap();

    let original = provider_binding_bytes_v1(&ad);
    let mut changed = ad.clone();
    changed.max_blob_size_bytes += 1;
    assert_ne!(original, provider_binding_bytes_v1(&changed));

    let mut replayed_at_new_head = ad.clone();
    replayed_at_new_head.binding_prev_action = ActionHash::from_raw_32(vec![5_u8; 32]);
    assert_ne!(original, provider_binding_bytes_v1(&replayed_at_new_head));

    let mut signature_only = ad.clone();
    signature_only.endpoint_binding_signature = [9_u8; 64];
    assert_eq!(original, provider_binding_bytes_v1(&signature_only));
}

#[test]
fn anchor_grammar_is_narrow_and_algorithm_tagged() {
    assert!(is_valid_anchor_v1(all_providers_anchor_v1()));
    assert!(is_valid_anchor_v1(&algorithm_anchor_v1(
        DigestAlgorithmV1::Sha256
    )));
    let digest = ContentDigestRefV1 {
        algorithm: DigestAlgorithmV1::Blake3_256,
        bytes: [0xab; 32],
    };
    let anchor = digest_anchor_v1(&digest);
    assert!(is_valid_anchor_v1(&anchor));
    assert!(!is_valid_anchor_v1("digest/sha256/ABCDEF"));
    assert!(!is_valid_anchor_v1("anything/goes"));
}
