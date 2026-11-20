//! DSSE-compatible, scenario-bound correctness-gate verification for MCRB.
//!
//! Required gates come from the embedded scenario manifest, not caller input.
//! A verified result is an opaque capability. Signatures authenticate the
//! attestation bytes; they do not prove that the verifier observed truthfully.

use base64::{engine::general_purpose::STANDARD, Engine as _};
use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const EMBEDDED_SCENARIO_MANIFEST: &str =
    include_str!("../../../../docs/benchmarks/MCRB_V0_1_SCENARIO_MANIFEST.json");
pub const GATE_ATTESTATION_SCHEMA_VERSION: &str = "mycelix-mcrb-gate-attestation-v0.1";
pub const GATE_PREDICATE_TYPE: &str =
    "https://luminousdynamics.io/attestations/mcrb-correctness-gate/v0.1";
pub const GATE_PAYLOAD_TYPE: &str = "application/vnd.in-toto+json";
pub const IN_TOTO_STATEMENT_V1: &str = "https://in-toto.io/Statement/v1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GateOutcome { Pass, Fail, Incomplete }

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GateAttestationPredicate {
    pub schema_version: String,
    pub predicate_type: String,
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub source_sha: String,
    pub source_tree_git_sha: String,
    pub runtime_identity_digest_sha256: String,
    pub topology_profile_digest_sha256: String,
    pub gate_id: String,
    pub outcome: GateOutcome,
    pub evidence_artifact_digests_sha256: Vec<String>,
    pub verifier_identity: String,
    pub verifier_version: String,
    pub verifier_binary_digest_sha256: String,
    pub builder_identity: String,
    pub trust_policy_id: String,
    pub trust_policy_digest_sha256: String,
    pub issued_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DsseSignature {
    pub keyid: String,
    pub sig: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DsseEnvelope {
    pub payload_type: String,
    pub payload: String,
    pub signatures: Vec<DsseSignature>,
}

/// in-toto Statement v1 carried inside the DSSE payload. Each subject binds
/// one raw evidence artifact digest; the predicate binds run, source, runtime,
/// topology, gate, verifier and trust-policy context.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InTotoSubject {
    pub name: String,
    pub digest: BTreeMap<String, String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InTotoStatement {
    #[serde(rename = "_type")]
    pub statement_type: String,
    pub subject: Vec<InTotoSubject>,
    #[serde(rename = "predicateType")]
    pub predicate_type: String,
    pub predicate: GateAttestationPredicate,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrustedGateSigner {
    pub key_id: String,
    pub verifier_identity: String,
    pub public_key_hex: String,
    pub allowed_gate_ids: Vec<String>,
    pub allowed_builder_ids: Vec<String>,
    pub valid_from_unix_ms: u64,
    pub expires_at_unix_ms: u64,
    pub revoked: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GateTrustPolicyDocument {
    pub schema_version: String,
    pub policy_id: String,
    pub policy_owner: String,
    pub valid_from_unix_ms: u64,
    pub expires_at_unix_ms: u64,
    pub signers: Vec<TrustedGateSigner>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedGateTrustPolicy {
    document: GateTrustPolicyDocument,
    digest_sha256: String,
}

impl VerifiedGateTrustPolicy {
    /// The expected digest must come from protected configuration outside the
    /// benchmark runner's mutable input. This function pins bytes; it does not
    /// decide who is entitled to change the external trust root.
    pub fn load_pinned(
        document: GateTrustPolicyDocument,
        expected_digest_sha256: &str,
    ) -> Result<Self, TrustPolicyError> {
        if document.schema_version != "mycelix-mcrb-trust-policy-v0.1" {
            return Err(TrustPolicyError::UnsupportedSchema);
        }
        if document.policy_id.trim().is_empty() || document.policy_owner.trim().is_empty() {
            return Err(TrustPolicyError::MissingPolicyIdentity);
        }
        if document.valid_from_unix_ms >= document.expires_at_unix_ms {
            return Err(TrustPolicyError::InvalidPolicyWindow);
        }

        let mut key_ids = BTreeSet::new();
        for signer in &document.signers {
            if signer.key_id.trim().is_empty()
                || signer.verifier_identity.trim().is_empty()
                || signer.allowed_gate_ids.is_empty()
                || signer.allowed_builder_ids.is_empty()
                || signer.valid_from_unix_ms >= signer.expires_at_unix_ms
            {
                return Err(TrustPolicyError::InvalidSigner(signer.key_id.clone()));
            }
            if !key_ids.insert(signer.key_id.clone()) {
                return Err(TrustPolicyError::DuplicateKeyId(signer.key_id.clone()));
            }
            decode_hex_32(&signer.public_key_hex)
                .map_err(|_| TrustPolicyError::InvalidPublicKey(signer.key_id.clone()))?;
            if signer.allowed_gate_ids.iter().any(|v| v.trim().is_empty())
                || signer.allowed_builder_ids.iter().any(|v| v.trim().is_empty())
            {
                return Err(TrustPolicyError::InvalidSigner(signer.key_id.clone()));
            }
        }

        let canonical = serde_json::to_vec(&document)
            .map_err(|e| TrustPolicyError::Serialization(e.to_string()))?;
        let actual = sha256_hex(&canonical);
        if !valid_sha256_hex(expected_digest_sha256) || actual != expected_digest_sha256 {
            return Err(TrustPolicyError::DigestMismatch {
                expected: expected_digest_sha256.to_string(),
                actual,
            });
        }
        Ok(Self { document, digest_sha256: expected_digest_sha256.to_string() })
    }

    pub fn policy_id(&self) -> &str { &self.document.policy_id }
    pub fn digest_sha256(&self) -> &str { &self.digest_sha256 }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TrustPolicyError {
    UnsupportedSchema,
    MissingPolicyIdentity,
    InvalidPolicyWindow,
    InvalidSigner(String),
    DuplicateKeyId(String),
    InvalidPublicKey(String),
    Serialization(String),
    DigestMismatch { expected: String, actual: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GateAttestationContext {
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub source_sha: String,
    pub source_tree_git_sha: String,
    pub runtime_identity_digest_sha256: String,
    pub topology_profile_digest_sha256: String,
    pub now_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateSetStatus { Valid, Invalid, Incomplete }

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GateSetIssue {
    EmbeddedScenarioManifestInvalid,
    ScenarioContractDigestMismatch,
    UnknownScenario(String),
    EmptyRequiredGate(String),
    EmptyEnvelopeSet,
    WrongPayloadType(usize),
    InvalidPayloadBase64(usize),
    InvalidPredicateJson(usize),
    NonCanonicalPredicate(usize),
    UnsupportedPredicate(usize),
    ContextMismatch { gate_id: String, field: &'static str },
    UnexpectedGate(String),
    DuplicateGate(String),
    MissingGate(String),
    WrongSignatureCount { gate_id: String, count: usize },
    UnknownSigner(String),
    RevokedSigner(String),
    SignerNotYetValid(String),
    ExpiredSigner(String),
    UnauthorizedGate { key_id: String, gate_id: String },
    UnauthorizedBuilder { key_id: String, builder_id: String },
    VerifierIdentityMismatch { key_id: String },
    PolicyMismatch { gate_id: String },
    InvalidEvidenceDigest { gate_id: String },
    EmptyEvidenceSet(String),
    InvalidSignature(String),
    FailedGate(String),
    IncompleteGate(String),
    EnvelopeDigestFailure(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GateSetReport {
    pub status: GateSetStatus,
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub trust_policy_id: String,
    pub trust_policy_digest_sha256: String,
    pub required_gate_count: usize,
    pub verified_pass_count: usize,
    pub gate_set_digest_sha256: Option<String>,
    pub attestation_digests_sha256: Vec<String>,
    pub issues: Vec<GateSetIssue>,
}

/// Opaque capability: only the verifier can create a complete, valid gate set.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedCorrectnessGateSet {
    run_id: String,
    scenario_id: String,
    scenario_contract_digest_sha256: String,
    trust_policy_id: String,
    trust_policy_digest_sha256: String,
    required_gate_ids: Vec<String>,
    attestation_digests_sha256: Vec<String>,
    gate_set_digest_sha256: String,
}

impl VerifiedCorrectnessGateSet {
    pub fn run_id(&self) -> &str { &self.run_id }
    pub fn scenario_id(&self) -> &str { &self.scenario_id }
    pub fn scenario_contract_digest_sha256(&self) -> &str { &self.scenario_contract_digest_sha256 }
    pub fn trust_policy_id(&self) -> &str { &self.trust_policy_id }
    pub fn trust_policy_digest_sha256(&self) -> &str { &self.trust_policy_digest_sha256 }
    pub fn required_gate_ids(&self) -> &[String] { &self.required_gate_ids }
    pub fn attestation_digests_sha256(&self) -> &[String] { &self.attestation_digests_sha256 }
    pub fn gate_set_digest_sha256(&self) -> &str { &self.gate_set_digest_sha256 }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GateSetVerification {
    Verified(VerifiedCorrectnessGateSet),
    Rejected(GateSetReport),
}

impl GateSetVerification {
    pub fn verified(&self) -> Option<&VerifiedCorrectnessGateSet> {
        match self { Self::Verified(v) => Some(v), Self::Rejected(_) => None }
    }
    pub fn report(&self) -> Option<&GateSetReport> {
        match self { Self::Verified(_) => None, Self::Rejected(r) => Some(r) }
    }
}

#[derive(Serialize)]
struct GateSetDigestMaterial<'a> {
    schema_version: &'static str,
    run_id: &'a str,
    scenario_id: &'a str,
    scenario_contract_digest_sha256: &'a str,
    trust_policy_id: &'a str,
    trust_policy_digest_sha256: &'a str,
    required_gate_ids: &'a [String],
    attestation_digests_sha256: &'a [String],
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut out, "{byte:02x}");
    }
    out
}

fn valid_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

fn valid_git_sha(value: &str) -> bool {
    value.len() == 40 && value.bytes().all(|b| b.is_ascii_hexdigit())
}

fn decode_hex_32(value: &str) -> Result<[u8; 32], ()> {
    if value.len() != 64 { return Err(()); }
    let mut out = [0_u8; 32];
    for (i, pair) in value.as_bytes().chunks_exact(2).enumerate() {
        out[i] = (hex_nibble(pair[0]).ok_or(())? << 4) | hex_nibble(pair[1]).ok_or(())?;
    }
    Ok(out)
}

fn hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        _ => None,
    }
}

fn dsse_pae(payload_type: &str, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload_type.len() + payload.len() + 64);
    out.extend_from_slice(b"DSSEv1 ");
    out.extend_from_slice(payload_type.len().to_string().as_bytes());
    out.push(b' ');
    out.extend_from_slice(payload_type.as_bytes());
    out.push(b' ');
    out.extend_from_slice(payload.len().to_string().as_bytes());
    out.push(b' ');
    out.extend_from_slice(payload);
    out
}

fn required_gate_ids(scenario_id: &str) -> Result<Vec<String>, GateSetIssue> {
    let manifest: Value = serde_json::from_str(EMBEDDED_SCENARIO_MANIFEST)
        .map_err(|_| GateSetIssue::EmbeddedScenarioManifestInvalid)?;
    let scenarios = manifest["scenarios"].as_array()
        .ok_or(GateSetIssue::EmbeddedScenarioManifestInvalid)?;
    let scenario = scenarios.iter()
        .find(|s| s["id"].as_str() == Some(scenario_id))
        .ok_or_else(|| GateSetIssue::UnknownScenario(scenario_id.to_string()))?;
    let invariants = scenario["hard_invariants"].as_array()
        .ok_or(GateSetIssue::EmbeddedScenarioManifestInvalid)?;
    let global = manifest["hard_gates"].as_array()
        .ok_or(GateSetIssue::EmbeddedScenarioManifestInvalid)?;

    let mut required = BTreeSet::new();
    for invariant in invariants {
        let id = invariant.as_str().ok_or(GateSetIssue::EmbeddedScenarioManifestInvalid)?;
        if id.trim().is_empty() { return Err(GateSetIssue::EmptyRequiredGate(scenario_id.to_string())); }
        if !required.insert(id.to_string()) { return Err(GateSetIssue::EmbeddedScenarioManifestInvalid); }
    }
    for gate in global {
        let id = gate["id"].as_str().ok_or(GateSetIssue::EmbeddedScenarioManifestInvalid)?;
        if id.trim().is_empty() || !required.insert(id.to_string()) {
            return Err(GateSetIssue::EmbeddedScenarioManifestInvalid);
        }
    }
    Ok(required.into_iter().collect())
}

pub fn sign_gate_predicate(
    predicate: &GateAttestationPredicate,
    key_id: &str,
    signing_key: &ed25519_dalek::SigningKey,
) -> Result<DsseEnvelope, String> {
    use ed25519_dalek::Signer;

    let subjects = predicate
        .evidence_artifact_digests_sha256
        .iter()
        .enumerate()
        .map(|(index, digest)| {
            let mut digests = BTreeMap::new();
            digests.insert("sha256".to_string(), digest.clone());
            InTotoSubject {
                name: format!("mcrb-evidence/{}/{index}", predicate.gate_id),
                digest: digests,
            }
        })
        .collect();
    let statement = InTotoStatement {
        statement_type: IN_TOTO_STATEMENT_V1.to_string(),
        subject: subjects,
        predicate_type: GATE_PREDICATE_TYPE.to_string(),
        predicate: predicate.clone(),
    };
    let payload = serde_json::to_vec(&statement).map_err(|e| e.to_string())?;
    let signature = signing_key.sign(&dsse_pae(GATE_PAYLOAD_TYPE, &payload));
    Ok(DsseEnvelope {
        payload_type: GATE_PAYLOAD_TYPE.to_string(),
        payload: STANDARD.encode(payload),
        signatures: vec![DsseSignature {
            keyid: key_id.to_string(),
            sig: STANDARD.encode(signature.to_bytes()),
        }],
    })
}

fn verify_one_envelope(
    envelope: &DsseEnvelope,
    index: usize,
    policy: &VerifiedGateTrustPolicy,
    context: &GateAttestationContext,
    required: &BTreeSet<String>,
) -> Result<(String, GateOutcome, String), GateSetIssue> {
    if envelope.payload_type != GATE_PAYLOAD_TYPE {
        return Err(GateSetIssue::WrongPayloadType(index));
    }
    let payload = STANDARD.decode(&envelope.payload)
        .map_err(|_| GateSetIssue::InvalidPayloadBase64(index))?;
    let statement: InTotoStatement = serde_json::from_slice(&payload)
        .map_err(|_| GateSetIssue::InvalidPredicateJson(index))?;
    if serde_json::to_vec(&statement)
        .map_err(|_| GateSetIssue::InvalidPredicateJson(index))? != payload
    {
        return Err(GateSetIssue::NonCanonicalPredicate(index));
    }
    if statement.statement_type != IN_TOTO_STATEMENT_V1
        || statement.predicate_type != GATE_PREDICATE_TYPE
    {
        return Err(GateSetIssue::UnsupportedPredicate(index));
    }
    let predicate = statement.predicate;

    let expected_subjects = predicate
        .evidence_artifact_digests_sha256
        .iter()
        .enumerate()
        .map(|(subject_index, digest)| {
            let mut digests = BTreeMap::new();
            digests.insert("sha256".to_string(), digest.clone());
            InTotoSubject {
                name: format!("mcrb-evidence/{}/{subject_index}", predicate.gate_id),
                digest: digests,
            }
        })
        .collect::<Vec<_>>();
    if statement.subject != expected_subjects {
        return Err(GateSetIssue::InvalidEvidenceDigest {
            gate_id: predicate.gate_id.clone(),
        });
    }

    if predicate.schema_version != GATE_ATTESTATION_SCHEMA_VERSION {
        return Err(GateSetIssue::UnsupportedPredicate(index));
    }

    let context_fields = [
        ("run_id", predicate.run_id.as_str(), context.run_id.as_str()),
        ("scenario_id", predicate.scenario_id.as_str(), context.scenario_id.as_str()),
        ("scenario_contract_digest_sha256", predicate.scenario_contract_digest_sha256.as_str(), context.scenario_contract_digest_sha256.as_str()),
        ("source_sha", predicate.source_sha.as_str(), context.source_sha.as_str()),
        ("source_tree_git_sha", predicate.source_tree_git_sha.as_str(), context.source_tree_git_sha.as_str()),
        ("runtime_identity_digest_sha256", predicate.runtime_identity_digest_sha256.as_str(), context.runtime_identity_digest_sha256.as_str()),
        ("topology_profile_digest_sha256", predicate.topology_profile_digest_sha256.as_str(), context.topology_profile_digest_sha256.as_str()),
    ];
    for (field, actual, expected) in context_fields {
        if actual != expected {
            return Err(GateSetIssue::ContextMismatch { gate_id: predicate.gate_id.clone(), field });
        }
    }
    if !required.contains(&predicate.gate_id) {
        return Err(GateSetIssue::UnexpectedGate(predicate.gate_id.clone()));
    }
    if !valid_git_sha(&predicate.source_sha) || !valid_git_sha(&predicate.source_tree_git_sha)
        || !valid_sha256_hex(&predicate.scenario_contract_digest_sha256)
        || !valid_sha256_hex(&predicate.runtime_identity_digest_sha256)
        || !valid_sha256_hex(&predicate.topology_profile_digest_sha256)
        || !valid_sha256_hex(&predicate.verifier_binary_digest_sha256)
        || !valid_sha256_hex(&predicate.trust_policy_digest_sha256)
    {
        return Err(GateSetIssue::InvalidEvidenceDigest { gate_id: predicate.gate_id.clone() });
    }
    if predicate.evidence_artifact_digests_sha256.is_empty() {
        return Err(GateSetIssue::EmptyEvidenceSet(predicate.gate_id.clone()));
    }
    if predicate.evidence_artifact_digests_sha256.iter().any(|d| !valid_sha256_hex(d)) {
        return Err(GateSetIssue::InvalidEvidenceDigest { gate_id: predicate.gate_id.clone() });
    }
    if predicate.verifier_identity.trim().is_empty()
        || predicate.verifier_version.trim().is_empty()
        || predicate.builder_identity.trim().is_empty()
    {
        return Err(GateSetIssue::InvalidEvidenceDigest { gate_id: predicate.gate_id.clone() });
    }
    if predicate.trust_policy_id != policy.document.policy_id
        || predicate.trust_policy_digest_sha256 != policy.digest_sha256
    {
        return Err(GateSetIssue::PolicyMismatch { gate_id: predicate.gate_id.clone() });
    }
    if predicate.issued_at_unix_ms > context.now_unix_ms {
        return Err(GateSetIssue::SignerNotYetValid("future-issued-attestation".to_string()));
    }
    if context.now_unix_ms < policy.document.valid_from_unix_ms {
        return Err(GateSetIssue::SignerNotYetValid(policy.document.policy_id.clone()));
    }
    if context.now_unix_ms > policy.document.expires_at_unix_ms {
        return Err(GateSetIssue::ExpiredSigner(policy.document.policy_id.clone()));
    }
    if envelope.signatures.len() != 1 {
        return Err(GateSetIssue::WrongSignatureCount {
            gate_id: predicate.gate_id.clone(),
            count: envelope.signatures.len(),
        });
    }

    let dsse_sig = &envelope.signatures[0];
    let signer = policy.document.signers.iter()
        .find(|s| s.key_id == dsse_sig.keyid)
        .ok_or_else(|| GateSetIssue::UnknownSigner(dsse_sig.keyid.clone()))?;
    if signer.revoked { return Err(GateSetIssue::RevokedSigner(signer.key_id.clone())); }
    if context.now_unix_ms < signer.valid_from_unix_ms
        || predicate.issued_at_unix_ms < signer.valid_from_unix_ms
    {
        return Err(GateSetIssue::SignerNotYetValid(signer.key_id.clone()));
    }
    if context.now_unix_ms > signer.expires_at_unix_ms
        || predicate.issued_at_unix_ms > signer.expires_at_unix_ms
    {
        return Err(GateSetIssue::ExpiredSigner(signer.key_id.clone()));
    }
    if predicate.verifier_identity != signer.verifier_identity {
        return Err(GateSetIssue::VerifierIdentityMismatch { key_id: signer.key_id.clone() });
    }
    if !signer.allowed_gate_ids.contains(&predicate.gate_id) {
        return Err(GateSetIssue::UnauthorizedGate { key_id: signer.key_id.clone(), gate_id: predicate.gate_id.clone() });
    }
    if !signer.allowed_builder_ids.contains(&predicate.builder_identity) {
        return Err(GateSetIssue::UnauthorizedBuilder { key_id: signer.key_id.clone(), builder_id: predicate.builder_identity.clone() });
    }

    let public_key_bytes = decode_hex_32(&signer.public_key_hex)
        .map_err(|_| GateSetIssue::UnknownSigner(signer.key_id.clone()))?;
    let public_key = VerifyingKey::from_bytes(&public_key_bytes)
        .map_err(|_| GateSetIssue::UnknownSigner(signer.key_id.clone()))?;
    let signature_bytes = STANDARD.decode(&dsse_sig.sig)
        .map_err(|_| GateSetIssue::InvalidSignature(predicate.gate_id.clone()))?;
    let signature = Signature::from_slice(&signature_bytes)
        .map_err(|_| GateSetIssue::InvalidSignature(predicate.gate_id.clone()))?;
    public_key.verify(&dsse_pae(GATE_PAYLOAD_TYPE, &payload), &signature)
        .map_err(|_| GateSetIssue::InvalidSignature(predicate.gate_id.clone()))?;

    let envelope_bytes = serde_json::to_vec(envelope)
        .map_err(|e| GateSetIssue::EnvelopeDigestFailure(e.to_string()))?;
    Ok((predicate.gate_id, predicate.outcome, sha256_hex(&envelope_bytes)))
}

pub fn verify_gate_set(
    envelopes: &[DsseEnvelope],
    policy: &VerifiedGateTrustPolicy,
    context: &GateAttestationContext,
) -> GateSetVerification {
    let mut report = GateSetReport {
        status: GateSetStatus::Invalid,
        run_id: context.run_id.clone(),
        scenario_id: context.scenario_id.clone(),
        scenario_contract_digest_sha256: context.scenario_contract_digest_sha256.clone(),
        trust_policy_id: policy.policy_id().to_string(),
        trust_policy_digest_sha256: policy.digest_sha256().to_string(),
        required_gate_count: 0,
        verified_pass_count: 0,
        gate_set_digest_sha256: None,
        attestation_digests_sha256: Vec::new(),
        issues: Vec::new(),
    };

    let embedded_digest = sha256_hex(EMBEDDED_SCENARIO_MANIFEST.as_bytes());
    if context.scenario_contract_digest_sha256 != embedded_digest {
        report.issues.push(GateSetIssue::ScenarioContractDigestMismatch);
        return GateSetVerification::Rejected(report);
    }
    let required_vec = match required_gate_ids(&context.scenario_id) {
        Ok(v) => v,
        Err(issue) => {
            report.issues.push(issue);
            return GateSetVerification::Rejected(report);
        }
    };
    report.required_gate_count = required_vec.len();
    if envelopes.is_empty() {
        report.status = GateSetStatus::Incomplete;
        report.issues.push(GateSetIssue::EmptyEnvelopeSet);
        for gate in required_vec {
            report.issues.push(GateSetIssue::MissingGate(gate));
        }
        return GateSetVerification::Rejected(report);
    }

    let required: BTreeSet<String> = required_vec.iter().cloned().collect();
    let mut seen = BTreeSet::new();
    let mut verified = BTreeMap::<String, GateOutcome>::new();
    let mut digests = BTreeMap::<String, String>::new();

    for (index, envelope) in envelopes.iter().enumerate() {
        match verify_one_envelope(envelope, index, policy, context, &required) {
            Ok((gate_id, outcome, digest)) => {
                if !seen.insert(gate_id.clone()) {
                    report.issues.push(GateSetIssue::DuplicateGate(gate_id));
                    continue;
                }
                verified.insert(gate_id.clone(), outcome);
                digests.insert(gate_id, digest);
            }
            Err(issue) => report.issues.push(issue),
        }
    }
    for gate in &required {
        if !seen.contains(gate) {
            report.issues.push(GateSetIssue::MissingGate(gate.clone()));
        }
    }
    for (gate, outcome) in &verified {
        match outcome {
            GateOutcome::Pass => report.verified_pass_count += 1,
            GateOutcome::Fail => report.issues.push(GateSetIssue::FailedGate(gate.clone())),
            GateOutcome::Incomplete => report.issues.push(GateSetIssue::IncompleteGate(gate.clone())),
        }
    }

    let invalid = report.issues.iter().any(|issue| {
        !matches!(issue, GateSetIssue::MissingGate(_) | GateSetIssue::IncompleteGate(_))
    });
    let status = if invalid {
        GateSetStatus::Invalid
    } else if !report.issues.is_empty() {
        GateSetStatus::Incomplete
    } else {
        GateSetStatus::Valid
    };
    report.status = status;
    report.attestation_digests_sha256 = digests.values().cloned().collect();
    if status != GateSetStatus::Valid {
        return GateSetVerification::Rejected(report);
    }

    let required_gate_ids = required_vec;
    let attestation_digests_sha256 = report.attestation_digests_sha256.clone();
    let material = GateSetDigestMaterial {
        schema_version: GATE_ATTESTATION_SCHEMA_VERSION,
        run_id: &context.run_id,
        scenario_id: &context.scenario_id,
        scenario_contract_digest_sha256: &context.scenario_contract_digest_sha256,
        trust_policy_id: policy.policy_id(),
        trust_policy_digest_sha256: policy.digest_sha256(),
        required_gate_ids: &required_gate_ids,
        attestation_digests_sha256: &attestation_digests_sha256,
    };
    let bytes = match serde_json::to_vec(&material) {
        Ok(v) => v,
        Err(error) => {
            report.status = GateSetStatus::Invalid;
            report.issues.push(GateSetIssue::EnvelopeDigestFailure(error.to_string()));
            return GateSetVerification::Rejected(report);
        }
    };
    let gate_set_digest_sha256 = sha256_hex(&bytes);
    GateSetVerification::Verified(VerifiedCorrectnessGateSet {
        run_id: context.run_id.clone(),
        scenario_id: context.scenario_id.clone(),
        scenario_contract_digest_sha256: context.scenario_contract_digest_sha256.clone(),
        trust_policy_id: policy.policy_id().to_string(),
        trust_policy_digest_sha256: policy.digest_sha256().to_string(),
        required_gate_ids,
        attestation_digests_sha256,
        gate_set_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::SigningKey;

    fn digest(seed: &str) -> String { sha256_hex(seed.as_bytes()) }
    fn key() -> SigningKey { SigningKey::from_bytes(&[42_u8; 32]) }

    fn ctx(scenario: &str) -> GateAttestationContext {
        GateAttestationContext {
            run_id: "run-test-1".into(),
            scenario_id: scenario.into(),
            scenario_contract_digest_sha256: sha256_hex(EMBEDDED_SCENARIO_MANIFEST.as_bytes()),
            source_sha: "a".repeat(40),
            source_tree_git_sha: "b".repeat(40),
            runtime_identity_digest_sha256: digest("runtime"),
            topology_profile_digest_sha256: digest("topology"),
            now_unix_ms: 1_800_000_000_000,
        }
    }

    fn policy(k: &SigningKey, gates: Vec<String>, revoked: bool) -> VerifiedGateTrustPolicy {
        let public_hex = k.verifying_key().to_bytes().iter().map(|b| format!("{b:02x}")).collect::<String>();
        let document = GateTrustPolicyDocument {
            schema_version: "mycelix-mcrb-trust-policy-v0.1".into(),
            policy_id: "test-policy-v0.1".into(),
            policy_owner: "independent-test-policy-owner".into(),
            valid_from_unix_ms: 1_700_000_000_000,
            expires_at_unix_ms: 1_900_000_000_000,
            signers: vec![TrustedGateSigner {
                key_id: "test-key".into(),
                verifier_identity: "independent-test-verifier".into(),
                public_key_hex: public_hex,
                allowed_gate_ids: gates,
                allowed_builder_ids: vec!["trusted-builder".into()],
                valid_from_unix_ms: 1_700_000_000_000,
                expires_at_unix_ms: 1_900_000_000_000,
                revoked,
            }],
        };
        let pin = sha256_hex(&serde_json::to_vec(&document).unwrap());
        VerifiedGateTrustPolicy::load_pinned(document, &pin).unwrap()
    }

    fn signed(ctx: &GateAttestationContext, p: &VerifiedGateTrustPolicy, k: &SigningKey, gate: &str, outcome: GateOutcome) -> DsseEnvelope {
        let predicate = GateAttestationPredicate {
            schema_version: GATE_ATTESTATION_SCHEMA_VERSION.into(),
            predicate_type: GATE_PREDICATE_TYPE.into(),
            run_id: ctx.run_id.clone(),
            scenario_id: ctx.scenario_id.clone(),
            scenario_contract_digest_sha256: ctx.scenario_contract_digest_sha256.clone(),
            source_sha: ctx.source_sha.clone(),
            source_tree_git_sha: ctx.source_tree_git_sha.clone(),
            runtime_identity_digest_sha256: ctx.runtime_identity_digest_sha256.clone(),
            topology_profile_digest_sha256: ctx.topology_profile_digest_sha256.clone(),
            gate_id: gate.into(),
            outcome,
            evidence_artifact_digests_sha256: vec![digest(&format!("evidence:{gate}"))],
            verifier_identity: "independent-test-verifier".into(),
            verifier_version: "0.1.0".into(),
            verifier_binary_digest_sha256: digest("verifier-binary"),
            builder_identity: "trusted-builder".into(),
            trust_policy_id: p.policy_id().into(),
            trust_policy_digest_sha256: p.digest_sha256().into(),
            issued_at_unix_ms: ctx.now_unix_ms - 1,
        };
        sign_gate_predicate(&predicate, "test-key", k).unwrap()
    }

    fn complete(ctx: &GateAttestationContext) -> (Vec<DsseEnvelope>, VerifiedGateTrustPolicy, SigningKey) {
        let gates = required_gate_ids(&ctx.scenario_id).unwrap();
        let k = key();
        let p = policy(&k, gates.clone(), false);
        let envelopes = gates.iter().map(|g| signed(ctx, &p, &k, g, GateOutcome::Pass)).collect();
        (envelopes, p, k)
    }

    #[test]
    fn complete_manifest_derived_gate_set_verifies() {
        let c = ctx("MCRB-01");
        let (envelopes, p, _) = complete(&c);
        let result = verify_gate_set(&envelopes, &p, &c);
        let set = result.verified().unwrap();
        assert_eq!(set.required_gate_ids().len(), 7);
        assert_eq!(set.attestation_digests_sha256().len(), 7);
        assert_eq!(set.run_id(), c.run_id);
        assert_eq!(set.scenario_id(), c.scenario_id);
    }

    #[test]
    fn missing_gate_is_incomplete() {
        let c = ctx("MCRB-01");
        let (mut envelopes, p, _) = complete(&c);
        envelopes.pop();
        let result = verify_gate_set(&envelopes, &p, &c);
        assert!(result.verified().is_none());
        assert_eq!(result.report().unwrap().status, GateSetStatus::Incomplete);
    }

    #[test]
    fn duplicate_and_unexpected_gates_are_invalid() {
        let c = ctx("MCRB-01");
        let (mut envelopes, p, k) = complete(&c);
        envelopes.push(envelopes[0].clone());
        envelopes.push(signed(&c, &p, &k, "not-in-contract", GateOutcome::Pass));
        let result = verify_gate_set(&envelopes, &p, &c);
        assert!(result.verified().is_none());
        assert!(result.report().unwrap().issues.iter().any(|i| matches!(i, GateSetIssue::DuplicateGate(_))));
        assert!(result.report().unwrap().issues.iter().any(|i| matches!(i, GateSetIssue::UnexpectedGate(_))));
    }

    #[test]
    fn failed_gate_is_invalid_and_incomplete_gate_is_incomplete() {
        let c = ctx("MCRB-01");
        let (mut envelopes, p, k) = complete(&c);
        let gate = required_gate_ids(&c.scenario_id).unwrap()[0].clone();
        envelopes[0] = signed(&c, &p, &k, &gate, GateOutcome::Fail);
        let failed = verify_gate_set(&envelopes, &p, &c);
        assert_eq!(failed.report().unwrap().status, GateSetStatus::Invalid);

        let (mut envelopes, p, k) = complete(&c);
        envelopes[0] = signed(&c, &p, &k, &gate, GateOutcome::Incomplete);
        let incomplete = verify_gate_set(&envelopes, &p, &c);
        assert_eq!(incomplete.report().unwrap().status, GateSetStatus::Incomplete);
    }

    #[test]
    fn wrong_source_signature_and_revoked_signer_are_rejected() {
        let c = ctx("MCRB-01");
        let (envelopes, p, _) = complete(&c);
        let mut wrong = c.clone();
        wrong.source_sha = "c".repeat(40);
        assert!(verify_gate_set(&envelopes, &p, &wrong).verified().is_none());

        let (mut envelopes, p, _) = complete(&c);
        envelopes[0].signatures[0].sig = STANDARD.encode([0_u8; 64]);
        assert!(verify_gate_set(&envelopes, &p, &c).verified().is_none());

        let gates = required_gate_ids(&c.scenario_id).unwrap();
        let k = key();
        let revoked = policy(&k, gates.clone(), true);
        let envelopes = gates.iter().map(|g| signed(&c, &revoked, &k, g, GateOutcome::Pass)).collect::<Vec<_>>();
        assert!(verify_gate_set(&envelopes, &revoked, &c).verified().is_none());
    }

    #[test]
    fn untrusted_signer_is_rejected() {
        let c = ctx("MCRB-01");
        let (mut envelopes, p, _) = complete(&c);
        envelopes[0].signatures[0].keyid = "unknown-key".into();
        assert!(verify_gate_set(&envelopes, &p, &c).verified().is_none());
    }

    #[test]
    fn trust_policy_pin_mismatch_is_rejected() {
        let c = ctx("MCRB-01");
        let gates = required_gate_ids(&c.scenario_id).unwrap();
        let k = key();
        let public_hex = k.verifying_key().to_bytes().iter().map(|b| format!("{b:02x}")).collect::<String>();
        let document = GateTrustPolicyDocument {
            schema_version: "mycelix-mcrb-trust-policy-v0.1".into(),
            policy_id: "test-policy-v0.1".into(),
            policy_owner: "independent-test-policy-owner".into(),
            valid_from_unix_ms: 1_700_000_000_000,
            expires_at_unix_ms: 1_900_000_000_000,
            signers: vec![TrustedGateSigner {
                key_id: "test-key".into(),
                verifier_identity: "independent-test-verifier".into(),
                public_key_hex: public_hex,
                allowed_gate_ids: gates,
                allowed_builder_ids: vec!["trusted-builder".into()],
                valid_from_unix_ms: 1_700_000_000_000,
                expires_at_unix_ms: 1_900_000_000_000,
                revoked: false,
            }],
        };
        let error = VerifiedGateTrustPolicy::load_pinned(document, &digest("wrong"))
            .unwrap_err();
        assert!(matches!(error, TrustPolicyError::DigestMismatch { .. }));
    }
}


/// Test-only helper for exercising QOPS with a genuine verifier-produced gate
/// capability. It still signs and verifies every manifest-derived gate.
#[cfg(test)]
pub(crate) fn verified_gate_set_for_test(
    run_id: &str,
    scenario_id: &str,
    scenario_contract_digest_sha256: &str,
) -> VerifiedCorrectnessGateSet {
    use ed25519_dalek::SigningKey;

    let now = 1_800_000_000_000_u64;
    let context = GateAttestationContext {
        run_id: run_id.to_string(),
        scenario_id: scenario_id.to_string(),
        scenario_contract_digest_sha256: scenario_contract_digest_sha256.to_string(),
        source_sha: "a".repeat(40),
        source_tree_git_sha: "b".repeat(40),
        runtime_identity_digest_sha256: sha256_hex(b"test-runtime"),
        topology_profile_digest_sha256: sha256_hex(b"test-topology"),
        now_unix_ms: now,
    };
    let required = required_gate_ids(scenario_id).expect("test scenario must exist");
    let signing_key = SigningKey::from_bytes(&[73_u8; 32]);
    let public_key_hex = signing_key
        .verifying_key()
        .to_bytes()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();
    let document = GateTrustPolicyDocument {
        schema_version: "mycelix-mcrb-trust-policy-v0.1".to_string(),
        policy_id: "mcrb-qops-test-policy-v0.1".to_string(),
        policy_owner: "independent-qops-test-policy-owner".to_string(),
        valid_from_unix_ms: now - 1000,
        expires_at_unix_ms: now + 1_000_000,
        signers: vec![TrustedGateSigner {
            key_id: "mcrb-qops-test-key".to_string(),
            verifier_identity: "mcrb-qops-test-verifier".to_string(),
            public_key_hex,
            allowed_gate_ids: required.clone(),
            allowed_builder_ids: vec!["mcrb-qops-test-builder".to_string()],
            valid_from_unix_ms: now - 1000,
            expires_at_unix_ms: now + 1_000_000,
            revoked: false,
        }],
    };
    let pin = sha256_hex(
        &serde_json::to_vec(&document).expect("test trust policy must serialize"),
    );
    let policy = VerifiedGateTrustPolicy::load_pinned(document, &pin)
        .expect("test trust policy pin must match");

    let envelopes = required
        .iter()
        .map(|gate_id| {
            let predicate = GateAttestationPredicate {
                schema_version: GATE_ATTESTATION_SCHEMA_VERSION.to_string(),
                predicate_type: GATE_PREDICATE_TYPE.to_string(),
                run_id: context.run_id.clone(),
                scenario_id: context.scenario_id.clone(),
                scenario_contract_digest_sha256: context
                    .scenario_contract_digest_sha256
                    .clone(),
                source_sha: context.source_sha.clone(),
                source_tree_git_sha: context.source_tree_git_sha.clone(),
                runtime_identity_digest_sha256: context
                    .runtime_identity_digest_sha256
                    .clone(),
                topology_profile_digest_sha256: context
                    .topology_profile_digest_sha256
                    .clone(),
                gate_id: gate_id.clone(),
                outcome: GateOutcome::Pass,
                evidence_artifact_digests_sha256: vec![sha256_hex(gate_id.as_bytes())],
                verifier_identity: "mcrb-qops-test-verifier".to_string(),
                verifier_version: "0.1.0".to_string(),
                verifier_binary_digest_sha256: sha256_hex(b"test-verifier-binary"),
                builder_identity: "mcrb-qops-test-builder".to_string(),
                trust_policy_id: policy.policy_id().to_string(),
                trust_policy_digest_sha256: policy.digest_sha256().to_string(),
                issued_at_unix_ms: now - 1,
            };
            sign_gate_predicate(&predicate, "mcrb-qops-test-key", &signing_key)
                .expect("test gate must sign")
        })
        .collect::<Vec<_>>();

    match verify_gate_set(&envelopes, &policy, &context) {
        GateSetVerification::Verified(value) => value,
        GateSetVerification::Rejected(report) => {
            panic!("test gate set must verify: {:?}", report.issues)
        }
    }
}
