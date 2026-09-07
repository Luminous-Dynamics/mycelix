// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence-only diagnostic for #311.
//!
//! This test does not claim either dummy signature is authentic. It demonstrates a
//! narrower pre-crypto fact: two different public keys can occupy the same verifier
//! DID/key ID and both produce `PreparedSignatureVerificationV2` values over the exact
//! same #235 signing digest because the signed record transcript does not bind an exact
//! verifier-key generation digest.

use mycelix_agent_activity_coverage_policy::{
    AgentActivityObservationV2, ObservedActivityActionV2, ObservedChainStatusV2,
    ObservedHighestActivityV2,
};
use mycelix_crypto::{AlgorithmId, TaggedPublicKey, TaggedSignature};
use mycelix_did_observer_assembly_policy::{
    assemble_observed_verifier_key_v2, FetchedDidDocumentActionKindV2,
    FetchedDidDocumentRecordV2, FetchedDidSecurityRecordV2, FetchedDidVerificationMethodV2,
    QualifiedDidObserverAssemblyV2,
};
use mycelix_kvector_proof_statement_policy::{
    derive_kvector_proof_statement_digest_v2, KVectorProofPublicStatementV2,
};
use mycelix_kvector_signature_verification_request_policy::prepare_kvector_signature_verification_v2;
use mycelix_kvector_verification_record_policy::{
    KVectorProofVerificationOutcomeV2, KVectorProofVerificationRecordBodyV2,
};
use mycelix_kvector_verifier_policy_body::{
    derive_kvector_verifier_policy_digest_v2, KVectorVerifierPolicyBodyV2,
};

const DID: &str = "did:mycelix:verifier";
const KEY_ID: &str = "did:mycelix:verifier#key-1";
const PROOF: &[u8] = b"kvector-proof-v2-test";
const PROOF_SHA256: [u8; 32] = [
    0xe9, 0x06, 0x44, 0x17, 0x7e, 0xbc, 0xbb, 0x04, 0x9d, 0xab, 0xa9, 0xdf, 0xd4, 0xfc,
    0x94, 0xae, 0xe5, 0x9d, 0xe5, 0x74, 0x67, 0xe1, 0xf6, 0x4e, 0xec, 0xd9, 0x7d, 0x67,
    0x5c, 0x4a, 0x8a, 0x4f,
];

fn policy<'a>() -> KVectorVerifierPolicyBodyV2<'a> {
    KVectorVerifierPolicyBodyV2 {
        policy_id: "policy:kvector-prod-v2",
        policy_version: "2.0.0",
        backend_id: "winterfell-v2",
        circuit_id: "mycelix-kvector-range-v2",
        circuit_version: "2.0.0",
        verifier_did: DID,
        verifier_key_id: KEY_ID,
        signature_scheme_id: "ed25519-v1",
        valid_from_micros: 1_000_000,
        valid_until_micros: 9_000_000,
        max_record_lifetime_micros: 2_000_000,
    }
}

fn public_statement() -> KVectorProofPublicStatementV2 {
    KVectorProofPublicStatementV2 {
        fulfillment_id: [0x11; 32],
        kvector_commitment: [0x22; 32],
        trust_score_lower_scaled: 600_000,
        trust_score_upper_scaled: 700_000,
    }
}

fn observer_with_key(key_multibase: &str) -> QualifiedDidObserverAssemblyV2 {
    let methods = [FetchedDidVerificationMethodV2 {
        id: "#key-1",
        type_: AlgorithmId::Ed25519.did_verification_method_type(),
        controller: DID,
        public_key_multibase: key_multibase,
        algorithm: Some(AlgorithmId::Ed25519.as_u16()),
    }];
    let authentication = ["#key-1"];
    let key_agreement: [&str; 0] = [];
    let actions = [ObservedActivityActionV2 {
        action_seq: 8,
        action_id: "did-create",
    }];
    let highest = ["head-42"];
    let activity = AgentActivityObservationV2 {
        status: ObservedChainStatusV2::Valid {
            action_seq: 42,
            action_id: "head-42",
        },
        valid_activity: &actions,
        rejected_activity: &[],
        highest_observed: Some(ObservedHighestActivityV2 {
            action_seq: 42,
            action_ids: &highest,
        }),
        warrant_count: 0,
    };
    let records = [FetchedDidSecurityRecordV2::Document(
        FetchedDidDocumentRecordV2 {
            action_seq: 8,
            action_id: "did-create",
            action_kind: FetchedDidDocumentActionKindV2::Create,
            version: 1,
            did: DID,
            verification_methods: &methods,
            authentication: &authentication,
            key_agreement: &key_agreement,
        },
    )];

    assemble_observed_verifier_key_v2(activity, &records, policy())
        .expect("each exact observed key is structurally admissible under the same DID/key ID")
}

fn record<'a>(
    statement_digest: &'a [u8],
    policy_digest: &'a [u8],
) -> KVectorProofVerificationRecordBodyV2<'a> {
    KVectorProofVerificationRecordBodyV2 {
        fulfillment_id: &[0x11; 32],
        proof_statement_sha256: statement_digest,
        proof_sha256: &PROOF_SHA256,
        backend_id: "winterfell-v2",
        circuit_id: "mycelix-kvector-range-v2",
        circuit_version: "2.0.0",
        verifier_did: DID,
        verifier_key_id: KEY_ID,
        signature_scheme_id: "ed25519-v1",
        verification_policy_sha256: policy_digest,
        outcome: KVectorProofVerificationOutcomeV2::Accepted,
        verified_at_micros: 2_000_000,
        valid_until_micros: 4_000_000,
    }
}

#[test]
fn different_key_material_same_key_id_prepares_the_same_signed_digest() {
    let key_generation_a = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x31; 32])
        .unwrap()
        .to_multibase();
    let key_generation_b = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x52; 32])
        .unwrap()
        .to_multibase();
    assert_ne!(key_generation_a, key_generation_b);

    let observer_a = observer_with_key(&key_generation_a);
    let observer_b = observer_with_key(&key_generation_b);

    let statement = public_statement();
    let statement_digest = derive_kvector_proof_statement_digest_v2(statement);
    let policy = policy();
    let policy_digest = derive_kvector_verifier_policy_digest_v2(policy).unwrap();
    let record = record(&statement_digest, &policy_digest);

    // Signature authenticity is intentionally outside #303. Correct wire shape is enough
    // to reach the preparation boundary, which is exactly what this diagnostic isolates.
    let signature = TaggedSignature::new(AlgorithmId::Ed25519, vec![0xA5; 64]).unwrap();

    let prepared_a = prepare_kvector_signature_verification_v2(
        &observer_a,
        record,
        statement,
        PROOF,
        policy,
        &signature,
    )
    .expect("first key generation should reach the pre-crypto boundary");

    let prepared_b = prepare_kvector_signature_verification_v2(
        &observer_b,
        record,
        statement,
        PROOF,
        policy,
        &signature,
    )
    .expect("second key generation under the same DID/key ID also reaches preparation");

    assert_ne!(
        prepared_a.public_key(),
        prepared_b.public_key(),
        "the two prepared requests must retain genuinely different public keys"
    );
    assert_eq!(prepared_a.verifier_did(), prepared_b.verifier_did());
    assert_eq!(prepared_a.verifier_key_id(), prepared_b.verifier_key_id());
    assert_eq!(
        prepared_a.signing_digest(),
        prepared_b.signing_digest(),
        "#235's current signed transcript is identical because it does not contain an exact verifier-key-generation digest"
    );
    assert_eq!(prepared_a.verified_at_micros(), prepared_b.verified_at_micros());
}
