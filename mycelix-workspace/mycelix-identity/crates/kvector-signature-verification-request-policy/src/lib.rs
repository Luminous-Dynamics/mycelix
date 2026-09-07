// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure preparation theorem for Identity V2 K-vector signature verification.
//!
//! This crate freezes the **exact request** a future cryptographic backend may verify.
//! It performs no signature verification itself and cannot produce authenticity,
//! policy-currentness, or positive-evidence authority.
//!
//! Preparation composes four already-separated boundaries:
//!
//! 1. #235: the record body is bound to the exact public statement and proof bytes;
//! 2. #248: the record's verifier/backend/circuit/policy tuple matches one exact static policy;
//! 3. #294: the verifier key comes from the opaque observed-DID capability rather than
//!    caller-constructed key metadata;
//! 4. #250: key/signature algorithm tags and exact wire sizes match the policy scheme.
//!
//! Only after all four pass is an opaque [`PreparedSignatureVerificationV2`] emitted.
//! The future #252-qualified crypto adapter should accept this capability, verify its exact
//! digest/key/signature tuple, and return a separate opaque authenticity capability.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedPublicKey, TaggedSignature};
use mycelix_did_observer_assembly_policy::QualifiedDidObserverAssemblyV2;
use mycelix_kvector_proof_statement_policy::KVectorProofPublicStatementV2;
use mycelix_kvector_signature_material_policy::{
    validate_kvector_signature_material_for_policy_v2, KVectorSignatureMaterialErrorV2,
};
use mycelix_kvector_verification_record_policy::{
    derive_kvector_verification_record_signing_digest_v2,
    validate_kvector_verification_record_binding_v2, KVectorProofVerificationOutcomeV2,
    KVectorProofVerificationRecordBodyV2, KVectorVerificationRecordErrorV2,
    SHA256_DIGEST_LEN_V2,
};
use mycelix_kvector_verifier_policy_body::{
    validate_kvector_verification_record_against_policy_body_v2,
    KVectorVerifierPolicyBodyErrorV2, KVectorVerifierPolicyBodyV2,
};

/// Exact, verifier-owned request for one cryptographic signature check.
///
/// Fields are private and there is no public constructor. The future crypto adapter may
/// inspect only the exact tuple that passed record/proof, policy, observed-key, and
/// material-shape preparation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreparedSignatureVerificationV2 {
    signing_digest: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    public_key: TaggedPublicKey,
    signature: TaggedSignature,
    verifier_did: String,
    verifier_key_id: String,
    verification_policy_sha256: [u8; SHA256_DIGEST_LEN_V2],
    observed_document_action_id: String,
    coverage_head_action_id: String,
    record_outcome: KVectorProofVerificationOutcomeV2,
    verified_at_micros: i64,
    valid_until_micros: i64,
}

impl PreparedSignatureVerificationV2 {
    pub fn signing_digest(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.signing_digest
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn public_key(&self) -> &TaggedPublicKey {
        &self.public_key
    }

    pub fn signature(&self) -> &TaggedSignature {
        &self.signature
    }

    pub fn verifier_did(&self) -> &str {
        &self.verifier_did
    }

    pub fn verifier_key_id(&self) -> &str {
        &self.verifier_key_id
    }

    pub fn verification_policy_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.verification_policy_sha256
    }

    pub fn observed_document_action_id(&self) -> &str {
        &self.observed_document_action_id
    }

    pub fn coverage_head_action_id(&self) -> &str {
        &self.coverage_head_action_id
    }

    pub fn record_outcome(&self) -> KVectorProofVerificationOutcomeV2 {
        self.record_outcome
    }

    pub fn verified_at_micros(&self) -> i64 {
        self.verified_at_micros
    }

    pub fn valid_until_micros(&self) -> i64 {
        self.valid_until_micros
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignatureVerificationRequestErrorV2 {
    RecordBinding(KVectorVerificationRecordErrorV2),
    StaticPolicy(KVectorVerifierPolicyBodyErrorV2),
    ObservedVerifierDidMismatch,
    ObservedVerifierKeyIdMismatch,
    SignatureMaterial(KVectorSignatureMaterialErrorV2),
    SigningDigest(KVectorVerificationRecordErrorV2),
}

/// Prepare the exact tuple a future cryptographic verifier may authenticate.
///
/// This function deliberately accepts #294's opaque composite capability rather than
/// raw observed-key DTOs. It re-establishes the record's exact proof/statement binding,
/// freezes the static policy tuple, checks that the observed DID/key identity matches the
/// policy, validates exact signature material shape, and derives #235's signing digest.
///
/// A successful result still says **nothing about signature authenticity**. In particular,
/// arbitrary signature bytes of the correct algorithm/length can prepare successfully and
/// must still fail later if the cryptographic backend rejects them.
pub fn prepare_kvector_signature_verification_v2<'a>(
    observer: &QualifiedDidObserverAssemblyV2,
    record: KVectorProofVerificationRecordBodyV2<'a>,
    public_statement: KVectorProofPublicStatementV2,
    proof_bytes: &[u8],
    policy: KVectorVerifierPolicyBodyV2<'a>,
    signature: &TaggedSignature,
) -> Result<PreparedSignatureVerificationV2, SignatureVerificationRequestErrorV2> {
    validate_kvector_verification_record_binding_v2(record, public_statement, proof_bytes)
        .map_err(SignatureVerificationRequestErrorV2::RecordBinding)?;

    validate_kvector_verification_record_against_policy_body_v2(record, policy)
        .map_err(SignatureVerificationRequestErrorV2::StaticPolicy)?;

    let observed = observer.observed_verifier_key();
    if observed.did != policy.verifier_did {
        return Err(SignatureVerificationRequestErrorV2::ObservedVerifierDidMismatch);
    }
    if observed.key.canonical_key_id != policy.verifier_key_id {
        return Err(SignatureVerificationRequestErrorV2::ObservedVerifierKeyIdMismatch);
    }

    validate_kvector_signature_material_for_policy_v2(
        policy,
        &observed.key.public_key,
        signature,
    )
    .map_err(SignatureVerificationRequestErrorV2::SignatureMaterial)?;

    let signing_digest = derive_kvector_verification_record_signing_digest_v2(record)
        .map_err(SignatureVerificationRequestErrorV2::SigningDigest)?;

    let mut policy_digest = [0u8; SHA256_DIGEST_LEN_V2];
    policy_digest.copy_from_slice(record.verification_policy_sha256);

    Ok(PreparedSignatureVerificationV2 {
        signing_digest,
        algorithm: observed.key.public_key.algorithm,
        public_key: observed.key.public_key.clone(),
        signature: signature.clone(),
        verifier_did: policy.verifier_did.to_string(),
        verifier_key_id: policy.verifier_key_id.to_string(),
        verification_policy_sha256: policy_digest,
        observed_document_action_id: observed.document_action_id.clone(),
        coverage_head_action_id: observer.coverage_head_action_id().to_string(),
        record_outcome: record.outcome,
        verified_at_micros: record.verified_at_micros,
        valid_until_micros: record.valid_until_micros,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_agent_activity_coverage_policy::{
        AgentActivityObservationV2, ObservedActivityActionV2, ObservedChainStatusV2,
        ObservedHighestActivityV2,
    };
    use mycelix_did_observer_assembly_policy::{
        assemble_observed_verifier_key_v2, FetchedDidDocumentActionKindV2,
        FetchedDidDocumentRecordV2, FetchedDidSecurityRecordV2,
        FetchedDidVerificationMethodV2,
    };
    use mycelix_kvector_proof_statement_policy::derive_kvector_proof_statement_digest_v2;
    use mycelix_kvector_verifier_policy_body::derive_kvector_verifier_policy_digest_v2;

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";
    const ED_KEY: &str = "z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK";
    const PROOF: &[u8] = b"kvector-proof-v2-test";
    const PROOF_SHA256: [u8; 32] = [
        0xe9, 0x06, 0x44, 0x17, 0x7e, 0xbc, 0xbb, 0x04, 0x9d, 0xab, 0xa9, 0xdf,
        0xd4, 0xfc, 0x94, 0xae, 0xe5, 0x9d, 0xe5, 0x74, 0x67, 0xe1, 0xf6, 0x4e,
        0xec, 0xd9, 0x7d, 0x67, 0x5c, 0x4a, 0x8a, 0x4f,
    ];

    static METHODS: [FetchedDidVerificationMethodV2<'static>; 1] =
        [FetchedDidVerificationMethodV2 {
            id: "#key-1",
            type_: "Ed25519VerificationKey2020",
            controller: DID,
            public_key_multibase: ED_KEY,
            algorithm: Some(0xed01),
        }];
    static AUTH: [&str; 1] = ["#key-1"];
    static NO_KA: [&str; 0] = [];

    fn policy() -> KVectorVerifierPolicyBodyV2<'static> {
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

    fn observer(policy: KVectorVerifierPolicyBodyV2<'static>) -> QualifiedDidObserverAssemblyV2 {
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
                verification_methods: &METHODS,
                authentication: &AUTH,
                key_agreement: &NO_KA,
            },
        )];

        assemble_observed_verifier_key_v2(activity, &records, policy).unwrap()
    }

    fn record<'a>(
        statement_digest: &'a [u8],
        policy_digest: &'a [u8],
        verifier_key_id: &'a str,
    ) -> KVectorProofVerificationRecordBodyV2<'a> {
        KVectorProofVerificationRecordBodyV2 {
            fulfillment_id: &[0x11; 32],
            proof_statement_sha256: statement_digest,
            proof_sha256: &PROOF_SHA256,
            backend_id: "winterfell-v2",
            circuit_id: "mycelix-kvector-range-v2",
            circuit_version: "2.0.0",
            verifier_did: DID,
            verifier_key_id,
            signature_scheme_id: "ed25519-v1",
            verification_policy_sha256: policy_digest,
            outcome: KVectorProofVerificationOutcomeV2::Accepted,
            verified_at_micros: 2_000_000,
            valid_until_micros: 4_000_000,
        }
    }

    fn dummy_ed25519_signature() -> TaggedSignature {
        TaggedSignature::new(AlgorithmId::Ed25519, vec![0xAA; 64]).unwrap()
    }

    #[test]
    fn exact_request_prepares_without_claiming_signature_authenticity() {
        let policy = policy();
        let observer = observer(policy);
        let statement = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(statement);
        let policy_digest = derive_kvector_verifier_policy_digest_v2(policy).unwrap();
        let record = record(&statement_digest, &policy_digest, KEY_ID);
        let signature = dummy_ed25519_signature();

        let prepared = prepare_kvector_signature_verification_v2(
            &observer,
            record,
            statement,
            PROOF,
            policy,
            &signature,
        )
        .expect("structurally exact request should prepare");

        assert_eq!(prepared.algorithm(), AlgorithmId::Ed25519);
        assert_eq!(prepared.verifier_did(), DID);
        assert_eq!(prepared.verifier_key_id(), KEY_ID);
        assert_eq!(prepared.observed_document_action_id(), "did-create");
        assert_eq!(prepared.coverage_head_action_id(), "head-42");
        assert_eq!(prepared.verification_policy_sha256(), &policy_digest);
        assert_eq!(prepared.signature(), &signature);
        assert_eq!(prepared.record_outcome(), KVectorProofVerificationOutcomeV2::Accepted);
    }

    #[test]
    fn proof_substitution_fails_before_signature_preparation() {
        let policy = policy();
        let observer = observer(policy);
        let statement = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(statement);
        let policy_digest = derive_kvector_verifier_policy_digest_v2(policy).unwrap();
        let record = record(&statement_digest, &policy_digest, KEY_ID);
        let signature = dummy_ed25519_signature();

        assert_eq!(
            prepare_kvector_signature_verification_v2(
                &observer,
                record,
                statement,
                b"substituted-proof",
                policy,
                &signature,
            ),
            Err(SignatureVerificationRequestErrorV2::RecordBinding(
                KVectorVerificationRecordErrorV2::ProofDigestMismatch
            ))
        );
    }

    #[test]
    fn static_policy_substitution_fails_closed() {
        let policy = policy();
        let observer = observer(policy);
        let statement = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(statement);
        let policy_digest = derive_kvector_verifier_policy_digest_v2(policy).unwrap();
        let wrong_policy_digest = [0u8; 32];
        let record = record(&statement_digest, &wrong_policy_digest, KEY_ID);
        let signature = dummy_ed25519_signature();

        assert!(matches!(
            prepare_kvector_signature_verification_v2(
                &observer,
                record,
                statement,
                PROOF,
                policy,
                &signature,
            ),
            Err(SignatureVerificationRequestErrorV2::StaticPolicy(_))
        ));
        assert_ne!(policy_digest, wrong_policy_digest);
    }

    #[test]
    fn observed_key_capability_cannot_be_rebound_to_another_policy_key() {
        let original_policy = policy();
        let observer = observer(original_policy);
        let statement = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(statement);

        let mut other_policy = original_policy;
        other_policy.verifier_key_id = "did:mycelix:verifier#other-key";
        let other_policy_digest = derive_kvector_verifier_policy_digest_v2(other_policy).unwrap();
        let record = record(
            &statement_digest,
            &other_policy_digest,
            other_policy.verifier_key_id,
        );
        let signature = dummy_ed25519_signature();

        assert_eq!(
            prepare_kvector_signature_verification_v2(
                &observer,
                record,
                statement,
                PROOF,
                other_policy,
                &signature,
            ),
            Err(SignatureVerificationRequestErrorV2::ObservedVerifierKeyIdMismatch)
        );
    }

    #[test]
    fn signature_algorithm_substitution_fails_closed() {
        let policy = policy();
        let observer = observer(policy);
        let statement = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(statement);
        let policy_digest = derive_kvector_verifier_policy_digest_v2(policy).unwrap();
        let record = record(&statement_digest, &policy_digest, KEY_ID);
        let wrong_signature = TaggedSignature {
            algorithm: AlgorithmId::MlDsa65,
            signature_bytes: vec![0xBB; AlgorithmId::MlDsa65.signature_size()],
        };

        assert!(matches!(
            prepare_kvector_signature_verification_v2(
                &observer,
                record,
                statement,
                PROOF,
                policy,
                &wrong_signature,
            ),
            Err(SignatureVerificationRequestErrorV2::SignatureMaterial(_))
        ));
    }
}
