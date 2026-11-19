//! Scenario-bound correctness-gate attestations for MCRB.
//!
//! A caller-supplied boolean cannot establish that a scenario's hard
//! invariants passed. This module verifies signed, artifact-bound gate
//! attestations against a policy supplied by an independently trusted
//! verifier environment. The policy digest is returned for inclusion in the
//! run receipt and must be checked against an out-of-band pinned trust policy.
//!
//! Signature validity proves that the configured signer key signed the
//! assertion. It does not prove that the sensor or evaluator was honest.

use std::collections::{BTreeMap, BTreeSet};

use ed25519_dalek::{Signature, VerifyingKey};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::offer_ledger::embedded_scenario_manifest_digest;

pub const GATE_ATTESTATION_SCHEMA_VERSION: &str = "mycelix-mcrb-gate-attestation-v0.1";
pub const GATE_POLICY_SCHEMA_VERSION: &str = "mycelix-mcrb-gate-policy-v0.1";
const ATTESTATION_DOMAIN: &[u8] = b"MYCELIX-MCRB-GATE-ATTESTATION\0v0.1\0";
const POLICY_DOMAIN: &[u8] = b"MYCELIX-MCRB-GATE-POLICY\0v0.1\0";
const ATTESTATION_SET_DOMAIN: &[u8] = b"MYCELIX-MCRB-GATE-ATTESTATION-SET\0v0.1\0";
const EMBEDDED_SCENARIO_MANIFEST: &str =
    include_str!("../../../../docs/benchmarks/MCRB_V0_1_SCENARIO_MANIFEST.json");

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum GateOutcome {
    Pass,
    Fail,
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GateAttestation {
    pub schema_version: String,
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub source_sha: String,
    pub runtime_identity_digest_sha256: String,
    pub gate_id: String,
    pub outcome: GateOutcome,
    pub evidence_artifact_digests_sha256: Vec<String>,
    pub verifier_key_id: String,
    pub verifier_version: String,
    pub signature_ed25519_hex: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrustedGateSigner {
    pub key_id: String,
    pub public_key_ed25519_hex: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GateVerifierPolicy {
    pub schema_version: String,
    pub policy_id: String,
    pub run_id: String,
    pub scenario_id: String,
    pub scenario_contract_digest_sha256: String,
    pub source_sha: String,
    pub runtime_identity_digest_sha256: String,
    pub minimum_distinct_signers_per_gate: usize,
    pub trusted_signers: Vec<TrustedGateSigner>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateVerificationStatus {
    VerifiedPass,
    Failed,
    Incomplete,
    Invalid,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GateVerificationIssue {
    UnsupportedPolicySchema,
    UnsupportedAttestationSchema(usize),
    EmptyPolicyId,
    EmptyRunId,
    EmptyScenarioId,
    InvalidSourceSha,
    InvalidRuntimeIdentityDigest,
    ScenarioContractDigestMismatch,
    InvalidMinimumSignerCount,
    NoTrustedSigners,
    DuplicateTrustedSigner(String),
    MalformedTrustedPublicKey(String),
    UnknownScenario(String),
    ScenarioHasNoHardInvariants(String),
    DuplicateRequiredGate(String),
    UnexpectedGate(String),
    DuplicateGateSigner(String, String),
    MissingGate(String),
    InsufficientDistinctSigners(String),
    ConflictingGateOutcomes(String),
    GateFailed(String),
    GateIncomplete(String),
    RunIdMismatch(String),
    ScenarioIdMismatch(String),
    ScenarioContractMismatch(String),
    SourceShaMismatch(String),
    RuntimeIdentityMismatch(String),
    EmptyVerifierVersion(String),
    EmptyEvidenceDigestSet(String),
    InvalidEvidenceDigest(String, String),
    UnknownVerifierKey(String),
    MalformedSignature(String),
    InvalidSignature(String),
    InvalidPolicyDigest,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GateVerificationReport {
    status: GateVerificationStatus,
    run_id: String,
    scenario_id: String,
    source_sha: String,
    scenario_contract_digest_sha256: String,
    runtime_identity_digest_sha256: String,
    trust_policy_digest_sha256: String,
    attestation_set_digest_sha256: String,
    required_gate_count: usize,
    accepted_attestation_count: usize,
    distinct_signer_count: usize,
    passed_gate_count: usize,
    failed_gate_count: usize,
    incomplete_gate_count: usize,
    issues: Vec<GateVerificationIssue>,
}

impl GateVerificationReport {
    pub fn status(&self) -> GateVerificationStatus { self.status }
    pub fn run_id(&self) -> &str { &self.run_id }
    pub fn scenario_id(&self) -> &str { &self.scenario_id }
    pub fn source_sha(&self) -> &str { &self.source_sha }
    pub fn scenario_contract_digest_sha256(&self) -> &str { &self.scenario_contract_digest_sha256 }
    pub fn runtime_identity_digest_sha256(&self) -> &str { &self.runtime_identity_digest_sha256 }
    pub fn trust_policy_digest_sha256(&self) -> &str { &self.trust_policy_digest_sha256 }
    pub fn attestation_set_digest_sha256(&self) -> &str { &self.attestation_set_digest_sha256 }
    pub fn required_gate_count(&self) -> usize { self.required_gate_count }
    pub fn accepted_attestation_count(&self) -> usize { self.accepted_attestation_count }
    pub fn distinct_signer_count(&self) -> usize { self.distinct_signer_count }
    pub fn passed_gate_count(&self) -> usize { self.passed_gate_count }
    pub fn failed_gate_count(&self) -> usize { self.failed_gate_count }
    pub fn incomplete_gate_count(&self) -> usize { self.incomplete_gate_count }
    pub fn issues(&self) -> &[GateVerificationIssue] { &self.issues }

    pub fn matches_offer_context(
        &self,
        run_id: &str,
        scenario_id: &str,
        scenario_contract_digest_sha256: &str,
    ) -> bool {
        self.run_id == run_id
            && self.scenario_id == scenario_id
            && self.scenario_contract_digest_sha256 == scenario_contract_digest_sha256
    }
}

fn push_unique(issues: &mut Vec<GateVerificationIssue>, issue: GateVerificationIssue) {
    if !issues.contains(&issue) {
        issues.push(issue);
    }
}

fn valid_hex(value: &str, bytes: usize) -> bool {
    value.len() == bytes * 2 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn decode_hex(value: &str) -> Option<Vec<u8>> {
    if value.len() % 2 != 0 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return None;
    }
    let mut result = Vec::with_capacity(value.len() / 2);
    for pair in value.as_bytes().chunks_exact(2) {
        let high = (pair[0] as char).to_digit(16)?;
        let low = (pair[1] as char).to_digit(16)?;
        result.push(((high << 4) | low) as u8);
    }
    Some(result)
}

fn encode_hex(bytes: &[u8]) -> String {
    let mut value = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write as _;
        let _ = write!(&mut value, "{byte:02x}");
    }
    value
}

fn push_field(buffer: &mut Vec<u8>, value: &str) {
    let bytes = value.as_bytes();
    buffer.extend_from_slice(&(bytes.len() as u64).to_be_bytes());
    buffer.extend_from_slice(bytes);
}

fn push_digest(buffer: &mut Vec<u8>, value: &str) {
    let bytes = decode_hex(value).expect("digest fields are validated before encoding");
    buffer.extend_from_slice(&bytes);
}

fn digest_hex(bytes: &[u8]) -> String {
    encode_hex(&Sha256::digest(bytes))
}

fn canonical_attestation_message(attestation: &GateAttestation) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(ATTESTATION_DOMAIN);
    push_field(&mut bytes, &attestation.schema_version);
    push_field(&mut bytes, &attestation.run_id);
    push_field(&mut bytes, &attestation.scenario_id);
    push_digest(&mut bytes, &attestation.scenario_contract_digest_sha256);
    push_field(&mut bytes, &attestation.source_sha);
    push_digest(&mut bytes, &attestation.runtime_identity_digest_sha256);
    push_field(&mut bytes, &attestation.gate_id);
    bytes.push(match attestation.outcome {
        GateOutcome::Pass => 1,
        GateOutcome::Fail => 2,
        GateOutcome::Incomplete => 3,
    });

    let mut evidence = attestation.evidence_artifact_digests_sha256.clone();
    evidence.sort();
    bytes.extend_from_slice(&(evidence.len() as u64).to_be_bytes());
    for digest in evidence {
        push_digest(&mut bytes, &digest);
    }
    push_field(&mut bytes, &attestation.verifier_key_id);
    push_field(&mut bytes, &attestation.verifier_version);
    bytes
}

pub fn gate_attestation_signing_bytes(attestation: &GateAttestation) -> Vec<u8> {
    canonical_attestation_message(attestation)
}

fn canonical_policy_bytes(policy: &GateVerifierPolicy) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(POLICY_DOMAIN);
    push_field(&mut bytes, &policy.schema_version);
    push_field(&mut bytes, &policy.policy_id);
    push_field(&mut bytes, &policy.run_id);
    push_field(&mut bytes, &policy.scenario_id);
    push_digest(&mut bytes, &policy.scenario_contract_digest_sha256);
    push_field(&mut bytes, &policy.source_sha);
    push_digest(&mut bytes, &policy.runtime_identity_digest_sha256);
    bytes.extend_from_slice(&(policy.minimum_distinct_signers_per_gate as u64).to_be_bytes());

    let mut signers = policy.trusted_signers.clone();
    signers.sort_by(|left, right| left.key_id.cmp(&right.key_id));
    bytes.extend_from_slice(&(signers.len() as u64).to_be_bytes());
    for signer in signers {
        push_field(&mut bytes, &signer.key_id);
        push_field(&mut bytes, &signer.public_key_ed25519_hex.to_ascii_lowercase());
    }
    bytes
}

pub fn gate_verifier_policy_digest(policy: &GateVerifierPolicy) -> String {
    digest_hex(&canonical_policy_bytes(policy))
}

fn embedded_required_gate_ids(scenario_id: &str) -> Result<Vec<String>, GateVerificationIssue> {
    let manifest: serde_json::Value = serde_json::from_str(EMBEDDED_SCENARIO_MANIFEST)
        .map_err(|_| GateVerificationIssue::UnknownScenario(scenario_id.to_string()))?;
    let scenarios = manifest["scenarios"]
        .as_array()
        .ok_or_else(|| GateVerificationIssue::UnknownScenario(scenario_id.to_string()))?;
    let scenario = scenarios
        .iter()
        .find(|scenario| scenario["id"].as_str() == Some(scenario_id))
        .ok_or_else(|| GateVerificationIssue::UnknownScenario(scenario_id.to_string()))?;
    let invariants = scenario["hard_invariants"]
        .as_array()
        .ok_or_else(|| GateVerificationIssue::ScenarioHasNoHardInvariants(scenario_id.to_string()))?;

    let mut gates = Vec::with_capacity(invariants.len());
    let mut seen = BTreeSet::new();
    for invariant in invariants {
        let gate_id = invariant.as_str().unwrap_or_default().trim();
        if gate_id.is_empty() {
            return Err(GateVerificationIssue::ScenarioHasNoHardInvariants(scenario_id.to_string()));
        }
        if !seen.insert(gate_id.to_string()) {
            return Err(GateVerificationIssue::DuplicateRequiredGate(gate_id.to_string()));
        }
        gates.push(gate_id.to_string());
    }
    if gates.is_empty() {
        return Err(GateVerificationIssue::ScenarioHasNoHardInvariants(scenario_id.to_string()));
    }
    gates.sort();
    Ok(gates)
}

fn verify_attestation_signature(
    attestation: &GateAttestation,
    signer_keys: &BTreeMap<String, VerifyingKey>,
) -> Result<(), GateVerificationIssue> {
    let verifying_key = signer_keys
        .get(&attestation.verifier_key_id)
        .ok_or_else(|| GateVerificationIssue::UnknownVerifierKey(attestation.verifier_key_id.clone()))?;
    let signature_bytes = decode_hex(&attestation.signature_ed25519_hex)
        .ok_or_else(|| GateVerificationIssue::MalformedSignature(attestation.gate_id.clone()))?;
    let signature = Signature::from_slice(&signature_bytes)
        .map_err(|_| GateVerificationIssue::MalformedSignature(attestation.gate_id.clone()))?;
    verifying_key
        .verify_strict(&canonical_attestation_message(attestation), &signature)
        .map_err(|_| GateVerificationIssue::InvalidSignature(attestation.gate_id.clone()))
}

fn report_attestation_set_digest(attestations: &[GateAttestation]) -> String {
    let mut sorted = attestations.to_vec();
    sorted.sort_by(|left, right| {
        left.gate_id
            .cmp(&right.gate_id)
            .then(left.verifier_key_id.cmp(&right.verifier_key_id))
            .then(left.signature_ed25519_hex.cmp(&right.signature_ed25519_hex))
    });
    let mut bytes = Vec::new();
    bytes.extend_from_slice(ATTESTATION_SET_DOMAIN);
    bytes.extend_from_slice(&(sorted.len() as u64).to_be_bytes());
    for attestation in sorted {
        let message = canonical_attestation_message(&attestation);
        push_field(&mut bytes, &encode_hex(&message));
        push_field(&mut bytes, &attestation.signature_ed25519_hex.to_ascii_lowercase());
    }
    digest_hex(&bytes)
}

/// Verify a complete scenario-bound set of signed gate outcomes.
///
/// The caller must load this policy from an independently governed trust
/// source and compare the returned policy digest with its pinned trust root.
/// A caller-created key set is not an independent trust decision.
pub fn verify_gate_attestations(
    policy: &GateVerifierPolicy,
    attestations: &[GateAttestation],
) -> GateVerificationReport {
    let policy_digest = gate_verifier_policy_digest(policy);
    let attestation_digest = report_attestation_set_digest(attestations);
    let mut issues = Vec::new();

    if policy.schema_version != GATE_POLICY_SCHEMA_VERSION {
        push_unique(&mut issues, GateVerificationIssue::UnsupportedPolicySchema);
    }
    if policy.policy_id.trim().is_empty() {
        push_unique(&mut issues, GateVerificationIssue::EmptyPolicyId);
    }
    if policy.run_id.trim().is_empty() {
        push_unique(&mut issues, GateVerificationIssue::EmptyRunId);
    }
    if policy.scenario_id.trim().is_empty() {
        push_unique(&mut issues, GateVerificationIssue::EmptyScenarioId);
    }
    if !valid_hex(&policy.source_sha, 20) {
        push_unique(&mut issues, GateVerificationIssue::InvalidSourceSha);
    }
    if !valid_hex(&policy.runtime_identity_digest_sha256, 32) {
        push_unique(&mut issues, GateVerificationIssue::InvalidRuntimeIdentityDigest);
    }
    if !valid_hex(&policy.scenario_contract_digest_sha256, 32)
        || policy.scenario_contract_digest_sha256 != embedded_scenario_manifest_digest()
    {
        push_unique(&mut issues, GateVerificationIssue::ScenarioContractDigestMismatch);
    }
    if policy.minimum_distinct_signers_per_gate == 0 {
        push_unique(&mut issues, GateVerificationIssue::InvalidMinimumSignerCount);
    }
    if policy.trusted_signers.is_empty() {
        push_unique(&mut issues, GateVerificationIssue::NoTrustedSigners);
    }

    let mut signer_keys = BTreeMap::<String, VerifyingKey>::new();
    for signer in &policy.trusted_signers {
        if signer.key_id.trim().is_empty() || signer_keys.contains_key(&signer.key_id) {
            push_unique(
                &mut issues,
                GateVerificationIssue::DuplicateTrustedSigner(signer.key_id.clone()),
            );
            continue;
        }
        let Some(key_bytes) = decode_hex(&signer.public_key_ed25519_hex) else {
            push_unique(
                &mut issues,
                GateVerificationIssue::MalformedTrustedPublicKey(signer.key_id.clone()),
            );
            continue;
        };
        let Ok(key_array) = <[u8; 32]>::try_from(key_bytes.as_slice()) else {
            push_unique(
                &mut issues,
                GateVerificationIssue::MalformedTrustedPublicKey(signer.key_id.clone()),
            );
            continue;
        };
        match VerifyingKey::from_bytes(&key_array) {
            Ok(key) => { signer_keys.insert(signer.key_id.clone(), key); }
            Err(_) => push_unique(
                &mut issues,
                GateVerificationIssue::MalformedTrustedPublicKey(signer.key_id.clone()),
            ),
        }
    }

    let required_gates = match embedded_required_gate_ids(&policy.scenario_id) {
        Ok(gates) => gates,
        Err(issue) => {
            push_unique(&mut issues, issue);
            Vec::new()
        }
    };
    let required: BTreeSet<String> = required_gates.iter().cloned().collect();
    let mut gate_signers = BTreeMap::<String, BTreeSet<String>>::new();
    let mut gate_outcomes = BTreeMap::<String, BTreeSet<GateOutcome>>::new();
    let mut seen_gate_signers = BTreeSet::<(String, String)>::new();
    let mut accepted_attestation_count = 0usize;
    let mut all_signers = BTreeSet::new();

    for (index, attestation) in attestations.iter().enumerate() {
        if attestation.schema_version != GATE_ATTESTATION_SCHEMA_VERSION {
            push_unique(&mut issues, GateVerificationIssue::UnsupportedAttestationSchema(index));
        }
        if !required.contains(&attestation.gate_id) {
            push_unique(&mut issues, GateVerificationIssue::UnexpectedGate(attestation.gate_id.clone()));
            continue;
        }
        if !seen_gate_signers.insert((attestation.gate_id.clone(), attestation.verifier_key_id.clone())) {
            push_unique(
                &mut issues,
                GateVerificationIssue::DuplicateGateSigner(
                    attestation.gate_id.clone(),
                    attestation.verifier_key_id.clone(),
                ),
            );
            continue;
        }
        if attestation.run_id != policy.run_id {
            push_unique(&mut issues, GateVerificationIssue::RunIdMismatch(attestation.gate_id.clone()));
        }
        if attestation.scenario_id != policy.scenario_id {
            push_unique(&mut issues, GateVerificationIssue::ScenarioIdMismatch(attestation.gate_id.clone()));
        }
        if attestation.scenario_contract_digest_sha256 != policy.scenario_contract_digest_sha256 {
            push_unique(&mut issues, GateVerificationIssue::ScenarioContractMismatch(attestation.gate_id.clone()));
        }
        if attestation.source_sha != policy.source_sha {
            push_unique(&mut issues, GateVerificationIssue::SourceShaMismatch(attestation.gate_id.clone()));
        }
        if attestation.runtime_identity_digest_sha256 != policy.runtime_identity_digest_sha256 {
            push_unique(&mut issues, GateVerificationIssue::RuntimeIdentityMismatch(attestation.gate_id.clone()));
        }
        if attestation.verifier_version.trim().is_empty() {
            push_unique(&mut issues, GateVerificationIssue::EmptyVerifierVersion(attestation.gate_id.clone()));
        }
        if attestation.evidence_artifact_digests_sha256.is_empty() {
            push_unique(&mut issues, GateVerificationIssue::EmptyEvidenceDigestSet(attestation.gate_id.clone()));
        }
        let mut evidence_seen = BTreeSet::new();
        for digest in &attestation.evidence_artifact_digests_sha256 {
            if !valid_hex(digest, 32) || !evidence_seen.insert(digest.to_ascii_lowercase()) {
                push_unique(
                    &mut issues,
                    GateVerificationIssue::InvalidEvidenceDigest(attestation.gate_id.clone(), digest.clone()),
                );
            }
        }
        if verify_attestation_signature(attestation, &signer_keys).is_ok() {
            accepted_attestation_count += 1;
            all_signers.insert(attestation.verifier_key_id.clone());
            gate_signers.entry(attestation.gate_id.clone()).or_default().insert(attestation.verifier_key_id.clone());
            gate_outcomes.entry(attestation.gate_id.clone()).or_default().insert(attestation.outcome);
        } else if let Err(issue) = verify_attestation_signature(attestation, &signer_keys) {
            push_unique(&mut issues, issue);
        }
    }

    let mut passed_gate_count = 0usize;
    let mut failed_gate_count = 0usize;
    let mut incomplete_gate_count = 0usize;
    for gate_id in &required_gates {
        let signer_count = gate_signers.get(gate_id).map(BTreeSet::len).unwrap_or(0);
        if signer_count == 0 {
            push_unique(&mut issues, GateVerificationIssue::MissingGate(gate_id.clone()));
            incomplete_gate_count += 1;
            continue;
        }
        if signer_count < policy.minimum_distinct_signers_per_gate {
            push_unique(&mut issues, GateVerificationIssue::InsufficientDistinctSigners(gate_id.clone()));
            incomplete_gate_count += 1;
            continue;
        }
        let outcomes = gate_outcomes.get(gate_id).cloned().unwrap_or_default();
        if outcomes.len() > 1 {
            push_unique(&mut issues, GateVerificationIssue::ConflictingGateOutcomes(gate_id.clone()));
            failed_gate_count += 1;
            continue;
        }
        match outcomes.iter().next().copied() {
            Some(GateOutcome::Pass) => passed_gate_count += 1,
            Some(GateOutcome::Fail) => {
                push_unique(&mut issues, GateVerificationIssue::GateFailed(gate_id.clone()));
                failed_gate_count += 1;
            }
            Some(GateOutcome::Incomplete) | None => {
                push_unique(&mut issues, GateVerificationIssue::GateIncomplete(gate_id.clone()));
                incomplete_gate_count += 1;
            }
        }
    }

    let invalid = issues.iter().any(|issue| {
        !matches!(
            issue,
            GateVerificationIssue::MissingGate(_)
                | GateVerificationIssue::InsufficientDistinctSigners(_)
                | GateVerificationIssue::GateIncomplete(_)
                | GateVerificationIssue::GateFailed(_)
        )
    });
    let status = if invalid || failed_gate_count > 0 {
        GateVerificationStatus::Invalid
    } else if incomplete_gate_count > 0 {
        GateVerificationStatus::Incomplete
    } else {
        GateVerificationStatus::VerifiedPass
    };

    GateVerificationReport {
        status,
        run_id: policy.run_id.clone(),
        scenario_id: policy.scenario_id.clone(),
        source_sha: policy.source_sha.clone(),
        scenario_contract_digest_sha256: policy.scenario_contract_digest_sha256.clone(),
        runtime_identity_digest_sha256: policy.runtime_identity_digest_sha256.clone(),
        trust_policy_digest_sha256: policy_digest,
        attestation_set_digest_sha256: attestation_digest,
        required_gate_count: required_gates.len(),
        accepted_attestation_count,
        distinct_signer_count: all_signers.len(),
        passed_gate_count,
        failed_gate_count,
        incomplete_gate_count,
        issues,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};

    fn hex(bytes: &[u8]) -> String { encode_hex(bytes) }

    fn policy_and_key() -> (GateVerifierPolicy, SigningKey) {
        let signing_key = SigningKey::from_bytes(&[17u8; 32]);
        let policy = GateVerifierPolicy {
            schema_version: GATE_POLICY_SCHEMA_VERSION.to_string(),
            policy_id: "fixture-trust-policy-v0.1".to_string(),
            run_id: "run-fixture-001".to_string(),
            scenario_id: "MCRB-01".to_string(),
            scenario_contract_digest_sha256: embedded_scenario_manifest_digest(),
            source_sha: "a".repeat(40),
            runtime_identity_digest_sha256: "b".repeat(64),
            minimum_distinct_signers_per_gate: 1,
            trusted_signers: vec![TrustedGateSigner {
                key_id: "fixture-verifier-a".to_string(),
                public_key_ed25519_hex: hex(&signing_key.verifying_key().to_bytes()),
            }],
        };
        (policy, signing_key)
    }

    fn attestation(
        policy: &GateVerifierPolicy,
        signing_key: &SigningKey,
        gate_id: &str,
        outcome: GateOutcome,
    ) -> GateAttestation {
        let mut attestation = GateAttestation {
            schema_version: GATE_ATTESTATION_SCHEMA_VERSION.to_string(),
            run_id: policy.run_id.clone(),
            scenario_id: policy.scenario_id.clone(),
            scenario_contract_digest_sha256: policy.scenario_contract_digest_sha256.clone(),
            source_sha: policy.source_sha.clone(),
            runtime_identity_digest_sha256: policy.runtime_identity_digest_sha256.clone(),
            gate_id: gate_id.to_string(),
            outcome,
            evidence_artifact_digests_sha256: vec![digest_hex(format!("evidence:{gate_id}").as_bytes())],
            verifier_key_id: "fixture-verifier-a".to_string(),
            verifier_version: "fixture-verifier/0.1".to_string(),
            signature_ed25519_hex: String::new(),
        };
        attestation.signature_ed25519_hex =
            hex(&signing_key.sign(&canonical_attestation_message(&attestation)).to_bytes());
        attestation
    }

    fn complete_attestations(
        policy: &GateVerifierPolicy,
        signing_key: &SigningKey,
        outcome: GateOutcome,
    ) -> Vec<GateAttestation> {
        embedded_required_gate_ids(&policy.scenario_id)
            .unwrap()
            .iter()
            .map(|gate| attestation(policy, signing_key, gate, outcome))
            .collect()
    }

    #[test]
    fn verifies_all_frozen_scenario_gates_and_binds_the_trust_policy() {
        let (policy, key) = policy_and_key();
        let attestations = complete_attestations(&policy, &key, GateOutcome::Pass);
        let report = verify_gate_attestations(&policy, &attestations);
        assert_eq!(report.status(), GateVerificationStatus::VerifiedPass);
        assert_eq!(report.required_gate_count(), 2);
        assert_eq!(report.passed_gate_count(), 2);
        assert_eq!(report.distinct_signer_count(), 1);
        assert_eq!(report.trust_policy_digest_sha256(), gate_verifier_policy_digest(&policy));
        assert_eq!(report.run_id(), "run-fixture-001");
        assert_eq!(report.scenario_id(), "MCRB-01");
    }

    #[test]
    fn missing_gate_is_incomplete_not_a_pass() {
        let (policy, key) = policy_and_key();
        let mut attestations = complete_attestations(&policy, &key, GateOutcome::Pass);
        attestations.pop();
        let report = verify_gate_attestations(&policy, &attestations);
        assert_eq!(report.status(), GateVerificationStatus::Incomplete);
        assert_eq!(report.incomplete_gate_count(), 1);
    }

    #[test]
    fn failed_gate_suppresses_verified_pass() {
        let (policy, key) = policy_and_key();
        let gates = embedded_required_gate_ids(&policy.scenario_id).unwrap();
        let mut attestations = vec![
            attestation(&policy, &key, &gates[0], GateOutcome::Pass),
            attestation(&policy, &key, &gates[1], GateOutcome::Fail),
        ];
        let report = verify_gate_attestations(&policy, &attestations);
        assert_eq!(report.status(), GateVerificationStatus::Invalid);
        assert_eq!(report.failed_gate_count(), 1);
        attestations.clear();
    }

    #[test]
    fn altered_evidence_digest_breaks_signature() {
        let (policy, key) = policy_and_key();
        let mut attestations = complete_attestations(&policy, &key, GateOutcome::Pass);
        attestations[0].evidence_artifact_digests_sha256[0] = "c".repeat(64);
        let report = verify_gate_attestations(&policy, &attestations);
        assert_eq!(report.status(), GateVerificationStatus::Invalid);
        assert!(report.issues().contains(&GateVerificationIssue::InvalidSignature(attestations[0].gate_id.clone())));
    }

    #[test]
    fn wrong_source_or_runtime_identity_invalidates_attestation() {
        let (policy, key) = policy_and_key();
        let mut attestations = complete_attestations(&policy, &key, GateOutcome::Pass);
        attestations[0].source_sha = "d".repeat(40);
        let report = verify_gate_attestations(&policy, &attestations);
        assert_eq!(report.status(), GateVerificationStatus::Invalid);
        assert!(report.issues().contains(&GateVerificationIssue::SourceShaMismatch(attestations[0].gate_id.clone())));
    }

    #[test]
    fn an_untrusted_signer_cannot_create_a_gate_pass() {
        let (policy, key) = policy_and_key();
        let mut attestations = complete_attestations(&policy, &key, GateOutcome::Pass);
        attestations[0].verifier_key_id = "unknown-key".to_string();
        let report = verify_gate_attestations(&policy, &attestations);
        assert_eq!(report.status(), GateVerificationStatus::Invalid);
        assert!(report.issues().contains(&GateVerificationIssue::UnknownVerifierKey("unknown-key".into())));
    }
}
