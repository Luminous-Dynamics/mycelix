use base64::{Engine as _, engine::general_purpose::STANDARD};
use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Bundle {
    schema_version: u32,
    bundle_id: String,
    evidence_class: String,
    subject: Subject,
    trust_policy: TrustPolicyRef,
    attestations: Vec<Attestation>,
    causal_edges: Vec<CausalEdge>,
    claim_ceiling: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Subject {
    repository: String,
    source_commit: String,
    trace_sha256: String,
    schema_sha256: String,
    tool_build_sha256: String,
    conductor_lineages: Vec<ConductorLineage>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ConductorLineage {
    conductor_id: String,
    agent_key_ref: String,
    conductor_build: String,
    dna_hash: String,
    cell_id: String,
    integrity_wasm_sha256: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TrustPolicyRef {
    policy_id: String,
    policy_version: String,
    policy_epoch: u64,
    policy_sha256: String,
    root_keyset_id: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Attestation {
    attestation_id: String,
    event_id: String,
    issuer_role: String,
    exposure: String,
    envelope: Envelope,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "format")]
enum Envelope {
    #[serde(rename = "dsse_in_toto_v1")]
    Dsse {
        payload_type: String,
        payload_base64: String,
        signatures: Vec<DsseSignature>,
    },
    #[serde(rename = "scitt_rfc9943_cose")]
    Scitt {
        media_type: String,
        signed_statement_cose_base64: String,
        statement_sha256: String,
        receipts: Vec<ScittReceipt>,
    },
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsseSignature {
    keyid: String,
    sig_base64: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScittReceipt {
    media_type: String,
    ts_issuer: String,
    ts_keyid: String,
    receipt_cose_base64: String,
    registration_policy_sha256: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CausalEdge {
    from_event_id: String,
    to_event_id: String,
    relation: String,
    basis_attestation_ids: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TrustPolicy {
    policy_id: String,
    policy_version: String,
    policy_epoch: u64,
    root_keyset_id: String,
    keys: Vec<TrustedKey>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RootPolicyEnvelope {
    payload_type: String,
    payload_base64: String,
    signatures: Vec<DsseSignature>,
}

/// This file is provisioned out-of-band by the relying party. It pins one
/// exact root key and one exact current policy snapshot; it is not accepted
/// from the evidence bundle.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TrustedRootPin {
    schema_version: u32,
    root_key_id: String,
    root_public_key_hex: String,
    policy_id: String,
    policy_version: String,
    policy_epoch: u64,
    policy_sha256: String,
    root_keyset_id: String,
}

#[derive(Debug, Clone)]
struct RootPolicyVerification {
    root_key_id: String,
    policy_epoch: u64,
    policy_sha256: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TrustedKey {
    keyid: String,
    public_key_hex: String,
    roles: Vec<String>,
    predicate_types: Vec<String>,
    status: String,
    subject_binding: Option<TrustSubjectBinding>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TrustSubjectBinding {
    conductor_id: String,
    agent_key_ref: String,
    dna_hash: String,
    cell_id: String,
    integrity_wasm_sha256: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct InTotoStatement {
    #[serde(rename = "_type")]
    statement_type: String,
    subject: Vec<InTotoSubject>,
    #[serde(rename = "predicateType")]
    predicate_type: String,
    predicate: Predicate,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct InTotoSubject {
    name: String,
    digest: BTreeMap<String, String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Predicate {
    role: String,
    event_id: String,
    issuer_key_id: String,
    source_commit: String,
    trace_sha256: String,
    details: Value,
}

#[derive(Debug, Clone, Serialize)]
struct Finding {
    code: String,
    severity: String,
    attestation_id: Option<String>,
    message: String,
}

#[derive(Debug, Clone, Serialize)]
struct Report {
    schema_version: u32,
    status: String,
    evidence_class: String,
    bundle_id: String,
    bundle_sha256: String,
    trace_sha256: String,
    trust_policy_sha256: String,
    trust_policy_root_authenticated: bool,
    trust_root_key_id: Option<String>,
    trust_policy_epoch: Option<u64>,
    attestations: usize,
    dsse_verified: usize,
    scitt_unverified: usize,
    causal_edges: usize,
    findings: Vec<Finding>,
    claim_ceiling: String,
}

fn finding(
    findings: &mut Vec<Finding>,
    code: &str,
    severity: &str,
    attestation_id: Option<&str>,
    message: impl Into<String>,
) {
    findings.push(Finding {
        code: code.to_string(),
        severity: severity.to_string(),
        attestation_id: attestation_id.map(ToString::to_string),
        message: message.into(),
    });
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn valid_policy_version(value: &str) -> bool {
    let parts: Vec<&str> = value.split('.').collect();
    parts.len() == 3
        && parts.iter().all(|part| {
            !part.is_empty()
                && part.bytes().all(|byte| byte.is_ascii_digit())
                && (*part == "0" || !part.starts_with('0'))
        })
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn decode_hex_32(value: &str) -> Option<[u8; 32]> {
    if value.len() != 64 {
        return None;
    }
    let mut out = [0u8; 32];
    for (i, slot) in out.iter_mut().enumerate() {
        let start = i * 2;
        *slot = u8::from_str_radix(&value[start..start + 2], 16).ok()?;
    }
    Some(out)
}

const TRUST_POLICY_PAYLOAD_TYPE: &str = "application/vnd.pulse-trust-policy+json;version=1";

fn verify_root_signed_trust_policy(
    policy_bytes: &[u8],
    envelope_bytes: &[u8],
    pin_bytes: &[u8],
) -> Result<RootPolicyVerification, String> {
    // The root pin and exact policy digest are relying-party inputs. They must
    // be provisioned outside the evidence bundle and protected by deployment
    // configuration/secret-management controls.
    let pin: TrustedRootPin = serde_json::from_slice(pin_bytes)
        .map_err(|error| format!("trusted root pin is invalid JSON: {error}"))?;
    if pin.schema_version != 1 {
        return Err("trusted root pin schema_version must be 1".into());
    }
    if pin.root_key_id.trim().is_empty()
        || !valid_sha256(&pin.policy_sha256)
        || pin.policy_epoch == 0
        || pin.policy_id.trim().is_empty()
        || !valid_policy_version(&pin.policy_version)
        || pin.root_keyset_id.trim().is_empty()
    {
        return Err("trusted root pin is missing a required identity, digest or positive epoch".into());
    }

    let root_public_key = decode_hex_32(&pin.root_public_key_hex)
        .ok_or_else(|| "trusted root public key must be 32-byte lowercase hex".to_string())?;
    if pin.root_public_key_hex.bytes().any(|byte| byte.is_ascii_uppercase()) {
        return Err("trusted root public key hex must be lowercase".into());
    }
    let expected_root_key_id = format!("sha256:{}", sha256_hex(&root_public_key));
    if pin.root_key_id != expected_root_key_id {
        return Err("trusted root key ID does not match the pinned public-key fingerprint".into());
    }

    let policy_digest = sha256_hex(policy_bytes);
    if policy_digest != pin.policy_sha256 {
        return Err("trust-policy bytes do not match the out-of-band policy digest pin".into());
    }

    let envelope: RootPolicyEnvelope = serde_json::from_slice(envelope_bytes)
        .map_err(|error| format!("root policy signature envelope is invalid JSON: {error}"))?;
    if envelope.payload_type != TRUST_POLICY_PAYLOAD_TYPE {
        return Err("root policy envelope payload_type is unsupported".into());
    }
    if envelope.signatures.len() != 1 {
        return Err("trust-policy root envelope v1 requires exactly one pinned-root signature".into());
    }
    let signature_entry = &envelope.signatures[0];
    if signature_entry.keyid != pin.root_key_id {
        return Err("trust-policy envelope signer does not match the out-of-band root key ID".into());
    }

    let signed_payload = STANDARD
        .decode(&envelope.payload_base64)
        .map_err(|_| "root policy payload is not valid standard base64".to_string())?;
    if signed_payload != policy_bytes {
        return Err("root policy envelope payload is not byte-identical to the supplied trust policy".into());
    }
    let signature_bytes = STANDARD
        .decode(&signature_entry.sig_base64)
        .map_err(|_| "root policy signature is not valid standard base64".to_string())?;
    let signature = Signature::from_slice(&signature_bytes)
        .map_err(|_| "root policy signature must be a 64-byte Ed25519 signature".to_string())?;
    let verifying_key = VerifyingKey::from_bytes(&root_public_key)
        .map_err(|_| "pinned root public key is not a valid Ed25519 key".to_string())?;
    let signing_bytes = pae(TRUST_POLICY_PAYLOAD_TYPE, policy_bytes);
    verifying_key
        .verify_strict(&signing_bytes, &signature)
        .map_err(|_| "trust-policy root signature verification failed".to_string())?;

    // Parse the policy only after the signature over its exact bytes verifies.
    let policy: TrustPolicy = serde_json::from_slice(policy_bytes)
        .map_err(|error| format!("root-authenticated trust policy JSON is invalid: {error}"))?;
    if policy.policy_id.trim().is_empty()
        || !valid_policy_version(&policy.policy_version)
        || policy.policy_id != pin.policy_id
        || policy.policy_version != pin.policy_version
        || policy.policy_epoch != pin.policy_epoch
        || policy.root_keyset_id != pin.root_keyset_id
    {
        return Err("root-authenticated policy identity/epoch does not match the relying-party pin".into());
    }

    Ok(RootPolicyVerification {
        root_key_id: pin.root_key_id,
        policy_epoch: policy.policy_epoch,
        policy_sha256: policy_digest,
    })
}

fn pae(payload_type: &str, payload: &[u8]) -> Vec<u8> {
    let mut out = format!(
        "DSSEv1 {} {} {} ",
        payload_type.len(),
        payload_type,
        payload.len()
    )
    .into_bytes();
    out.extend_from_slice(payload);
    out
}

fn predicate_type_for_role(role: &str) -> Option<&'static str> {
    match role {
        "host_authority" => {
            Some("https://luminousdynamics.io/attestations/pulse/host-authority/v1")
        }
        "action_author" => {
            Some("https://luminousdynamics.io/attestations/pulse/action-authorship/v1")
        }
        "validator" => {
            Some("https://luminousdynamics.io/attestations/pulse/validator-disposition/v1")
        }
        "observer" => {
            Some("https://luminousdynamics.io/attestations/pulse/observer-integration/v1")
        }
        "client_outcome" => {
            Some("https://luminousdynamics.io/attestations/pulse/client-outcome/v1")
        }
        "independent_auditor" => {
            Some("https://luminousdynamics.io/attestations/pulse/independent-audit/v1")
        }
        _ => None,
    }
}

fn graph_has_cycle(edges: &[CausalEdge], event_ids: &BTreeSet<String>) -> bool {
    let mut indegree: BTreeMap<String, usize> =
        event_ids.iter().map(|id| (id.clone(), 0)).collect();
    let mut adjacency: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();

    for edge in edges {
        if event_ids.contains(&edge.from_event_id) && event_ids.contains(&edge.to_event_id) {
            adjacency
                .entry(edge.from_event_id.clone())
                .or_default()
                .insert(edge.to_event_id.clone());
        }
    }
    for targets in adjacency.values() {
        for target in targets {
            *indegree.entry(target.clone()).or_default() += 1;
        }
    }
    let mut queue: VecDeque<String> = indegree
        .iter()
        .filter(|(_, degree)| **degree == 0)
        .map(|(id, _)| id.clone())
        .collect();
    let mut visited = 0usize;
    while let Some(id) = queue.pop_front() {
        visited += 1;
        if let Some(targets) = adjacency.get(&id) {
            for target in targets {
                if let Some(degree) = indegree.get_mut(target) {
                    *degree -= 1;
                    if *degree == 0 {
                        queue.push_back(target.clone());
                    }
                }
            }
        }
    }
    visited != event_ids.len()
}

fn verify_issuer_subject_binding(
    role: &str,
    details: &Value,
    key: &TrustedKey,
    subject: &Subject,
) -> Result<(), String> {
    if role == "independent_auditor" {
        return Ok(());
    }
    let binding = key
        .subject_binding
        .as_ref()
        .ok_or_else(|| "trusted key has no conductor/agent subject_binding".to_string())?;
    let get = |name: &str| details.get(name).and_then(Value::as_str);
    if get("conductor_id") != Some(binding.conductor_id.as_str())
        || get("agent_key_ref") != Some(binding.agent_key_ref.as_str())
        || get("dna_hash") != Some(binding.dna_hash.as_str())
        || get("cell_id") != Some(binding.cell_id.as_str())
        || get("integrity_wasm_sha256") != Some(binding.integrity_wasm_sha256.as_str())
    {
        return Err("signed predicate conductor/agent/DNA/CellId/WASM identity differs from trusted key binding".into());
    }
    let Some(lineage) = subject
        .conductor_lineages
        .iter()
        .find(|c| c.conductor_id == binding.conductor_id)
    else {
        return Err("trusted key conductor_id is absent from bundle subject lineage".into());
    };
    if lineage.agent_key_ref != binding.agent_key_ref
        || lineage.dna_hash != binding.dna_hash
        || lineage.cell_id != binding.cell_id
        || lineage.integrity_wasm_sha256 != binding.integrity_wasm_sha256
    {
        return Err("bundle conductor lineage differs from trusted key subject binding".into());
    }
    Ok(())
}

fn verify_statement_trace_event_binding(
    role: &str,
    event_id: &str,
    details: &Value,
    trace: &Value,
    bundle_subject: &Subject,
    trace_events: &BTreeMap<String, &Value>,
) -> Result<(), String> {
    if role == "independent_auditor" {
        let trace_id = trace
            .get("trace_id")
            .and_then(Value::as_str)
            .ok_or_else(|| "trace-level auditor statement requires trace.trace_id".to_string())?;
        let expected_event_id = format!("trace:{trace_id}");
        let exact_fields = [
            "trace_id",
            "source_commit",
            "trace_sha256",
            "schema_sha256",
            "tool_build_sha256",
        ];
        let Some(map) = details.as_object() else {
            return Err("independent auditor details must be an object".into());
        };
        if map.len() != exact_fields.len()
            || !exact_fields.iter().all(|key| map.contains_key(*key))
            || event_id != expected_event_id
            || details["trace_id"].as_str() != Some(trace_id)
            || details["source_commit"].as_str() != Some(bundle_subject.source_commit.as_str())
            || details["trace_sha256"].as_str() != Some(bundle_subject.trace_sha256.as_str())
            || details["schema_sha256"].as_str() != Some(bundle_subject.schema_sha256.as_str())
            || details["tool_build_sha256"].as_str()
                != Some(bundle_subject.tool_build_sha256.as_str())
        {
            return Err("independent auditor statement must bind the exact trace ID and bundle subject digests".into());
        }
        return Ok(());
    }

    let event = trace_events.get(event_id).ok_or_else(|| {
        format!("attested event_id {event_id} does not exist exactly once in trace.events")
    })?;

    let expected_source = match role {
        "host_authority" => "conductor_host",
        "action_author" => "source_chain",
        "validator" => "validator",
        "observer" => "observer",
        "client_outcome" => "client",
        _ => return Err(format!("unsupported event-attestation role {role}")),
    };
    let event_type = event
        .get("event_type")
        .and_then(Value::as_str)
        .unwrap_or_default();
    let source = event
        .get("source")
        .and_then(Value::as_str)
        .unwrap_or_default();
    let role_event_type_valid = match role {
        "host_authority" => matches!(
            event_type,
            "host_grant_issued" | "host_grant_revoked" | "host_call"
        ),
        "action_author" => event_type == "action_authored",
        "validator" => event_type == "validation_disposition",
        "observer" => event_type == "projection_observed",
        "client_outcome" => event_type == "caller_outcome",
        _ => false,
    };
    if source != expected_source || !role_event_type_valid {
        return Err(format!(
            "attestation role {role} is incompatible with trace event source={source} type={event_type}"
        ));
    }

    let required_fields = [
        "event_type",
        "source",
        "conductor_id",
        "local_sequence",
        "evidence_ref",
        "event_data",
        "agent_key_ref",
        "dna_hash",
        "cell_id",
        "integrity_wasm_sha256",
    ];
    let Some(map) = details.as_object() else {
        return Err("signed event details must be an object".into());
    };
    if map.len() != required_fields.len()
        || !required_fields.iter().all(|key| map.contains_key(*key))
    {
        return Err(
            "signed event details must contain the exact closed event/lineage field set".into(),
        );
    }

    let conductor_id = event
        .get("conductor_id")
        .and_then(Value::as_str)
        .ok_or_else(|| "trace event has no conductor_id".to_string())?;
    let lineage = bundle_subject
        .conductor_lineages
        .iter()
        .find(|lineage| lineage.conductor_id == conductor_id)
        .ok_or_else(|| {
            format!("trace event conductor {conductor_id} has no bundle subject lineage")
        })?;

    if details["event_type"].as_str() != Some(event_type)
        || details["source"].as_str() != Some(source)
        || details["conductor_id"].as_str() != Some(conductor_id)
        || details["local_sequence"] != event["local_sequence"]
        || details["evidence_ref"] != event["evidence_ref"]
        || details["event_data"] != event["data"]
        || details["agent_key_ref"].as_str() != Some(lineage.agent_key_ref.as_str())
        || details["dna_hash"].as_str() != Some(lineage.dna_hash.as_str())
        || details["cell_id"].as_str() != Some(lineage.cell_id.as_str())
        || details["integrity_wasm_sha256"].as_str() != Some(lineage.integrity_wasm_sha256.as_str())
    {
        return Err(format!(
            "signed predicate details do not exactly match trace event {event_id} and its conductor lineage"
        ));
    }
    Ok(())
}

fn edge_basis_matches_authenticated_endpoints(
    basis_attestation_ids: &[String],
    attestation_event_ids: &BTreeMap<String, String>,
    accepted_attestation_ids: &BTreeSet<String>,
    accepted_event_ids: &BTreeSet<String>,
    from_event_id: &str,
    to_event_id: &str,
) -> bool {
    if !accepted_event_ids.contains(from_event_id) || !accepted_event_ids.contains(to_event_id) {
        return false;
    }
    let mut from_authenticated = false;
    let mut to_authenticated = false;
    for attestation_id in basis_attestation_ids {
        if !accepted_attestation_ids.contains(attestation_id) {
            return false;
        }
        match attestation_event_ids
            .get(attestation_id)
            .map(String::as_str)
        {
            Some(event_id) if event_id == from_event_id => from_authenticated = true,
            Some(event_id) if event_id == to_event_id => to_authenticated = true,
            _ => return false,
        }
    }
    from_authenticated && to_authenticated
}

enum RelationValidation {
    Valid,
    Incomplete(String),
    Violation(String),
}

fn event_data_str<'a>(event: &'a Value, key: &str) -> Option<&'a str> {
    event.get("data")?.get(key)?.as_str()
}

fn event_data_u64(event: &Value, key: &str) -> Option<u64> {
    event.get("data")?.get(key)?.as_u64()
}

fn validate_causal_relation(
    relation: &str,
    from: &Value,
    to: &Value,
    evidence_class: &str,
) -> RelationValidation {
    let from_type = from
        .get("event_type")
        .and_then(Value::as_str)
        .unwrap_or_default();
    let to_type = to
        .get("event_type")
        .and_then(Value::as_str)
        .unwrap_or_default();
    match relation {
        "program_order" => {
            let (Some(from_conductor), Some(to_conductor)) = (
                from.get("conductor_id").and_then(Value::as_str),
                to.get("conductor_id").and_then(Value::as_str),
            ) else {
                return RelationValidation::Incomplete(
                    "program_order requires both conductor identities".into(),
                );
            };
            if from_conductor != to_conductor {
                return RelationValidation::Violation(
                    "program_order cannot cross conductor-local sequence domains".into(),
                );
            }
            let (Some(from_sequence), Some(to_sequence)) = (
                from.get("local_sequence").and_then(Value::as_u64),
                to.get("local_sequence").and_then(Value::as_u64),
            ) else {
                return RelationValidation::Incomplete(
                    "program_order requires positive local_sequence values".into(),
                );
            };
            if from_sequence >= to_sequence {
                return RelationValidation::Violation(
                    "program_order requires strictly increasing local_sequence".into(),
                );
            }
            RelationValidation::Valid
        }
        "request_response" => {
            let allowed_pair = matches!(
                (from_type, to_type),
                ("host_call", "domain_effect")
                    | ("host_call", "caller_outcome")
                    | ("domain_effect", "caller_outcome")
            );
            if !allowed_pair {
                return RelationValidation::Incomplete(format!(
                    "request_response pair {from_type} -> {to_type} is not defined by Pulse causal profile v1"
                ));
            }
            let (Some(from_operation), Some(to_operation)) = (
                event_data_str(from, "operation_id"),
                event_data_str(to, "operation_id"),
            ) else {
                return RelationValidation::Incomplete(
                    "request_response requires operation_id on both events".into(),
                );
            };
            let (Some(from_attempt), Some(to_attempt)) = (
                event_data_u64(from, "attempt_id"),
                event_data_u64(to, "attempt_id"),
            ) else {
                return RelationValidation::Incomplete(
                    "request_response requires attempt_id on both events".into(),
                );
            };
            if from_operation != to_operation || from_attempt != to_attempt {
                return RelationValidation::Violation(
                    "request_response endpoints must share exact operation_id and attempt_id"
                        .into(),
                );
            }
            RelationValidation::Valid
        }
        "source_action_dependency" => {
            if (from_type, to_type) != ("host_grant_issued", "capability_claim_accepted") {
                return RelationValidation::Incomplete(format!(
                    "source_action_dependency pair {from_type} -> {to_type} is not defined by Pulse causal profile v1"
                ));
            }
            let (Some(from_hash), Some(to_hash)) = (
                event_data_str(from, "grant_action_hash"),
                event_data_str(to, "grant_action_hash"),
            ) else {
                return RelationValidation::Incomplete(
                    "grant-to-claim dependency requires exact grant_action_hash on both events"
                        .into(),
                );
            };
            if from_hash != to_hash {
                return RelationValidation::Violation(
                    "capability claim depends on a different grant ActionHash".into(),
                );
            }
            RelationValidation::Valid
        }
        "validator_receipt" => {
            if (from_type, to_type) != ("action_authored", "validation_disposition") {
                return RelationValidation::Incomplete(format!(
                    "validator_receipt pair {from_type} -> {to_type} is not an action/validation pair"
                ));
            }
            let action_hash = event_data_str(from, "action_hash");
            let validated_hash = event_data_str(to, "action_hash");
            let author_conductor = to
                .get("data")
                .and_then(|data| data.get("author_conductor_id"))
                .and_then(Value::as_str);
            let receiving_conductor = to
                .get("data")
                .and_then(|data| data.get("receiving_conductor_id"))
                .and_then(Value::as_str);
            let source_conductor = from.get("conductor_id").and_then(Value::as_str);
            let target_conductor = to.get("conductor_id").and_then(Value::as_str);
            let (
                Some(action_hash),
                Some(validated_hash),
                Some(author_conductor),
                Some(receiving_conductor),
                Some(source_conductor),
                Some(target_conductor),
            ) = (
                action_hash,
                validated_hash,
                author_conductor,
                receiving_conductor,
                source_conductor,
                target_conductor,
            )
            else {
                return RelationValidation::Incomplete(
                    "validator_receipt requires exact action and author/receiver conductor identities".into(),
                );
            };
            if action_hash != validated_hash
                || source_conductor != author_conductor
                || target_conductor != receiving_conductor
            {
                return RelationValidation::Violation(
                    "validator receipt does not match the exact authored action and author/receiver conductors".into(),
                );
            }
            RelationValidation::Valid
        }
        "observer_confirmation" => {
            if (from_type, to_type) != ("validation_disposition", "projection_observed") {
                return RelationValidation::Incomplete(format!(
                    "observer_confirmation pair {from_type} -> {to_type} is not a validator/projection pair"
                ));
            }
            let (
                Some(validated_hash),
                Some(projection_hash),
                Some(observed_hash),
                Some(required_hash),
            ) = (
                event_data_str(from, "action_hash"),
                event_data_str(to, "action_hash"),
                event_data_str(to, "observed_action_hash"),
                event_data_str(to, "required_action_hash"),
            )
            else {
                return RelationValidation::Incomplete(
                    "observer_confirmation requires exact validation and projection ActionHashes"
                        .into(),
                );
            };
            if validated_hash != projection_hash
                || validated_hash != observed_hash
                || validated_hash != required_hash
            {
                return RelationValidation::Violation(
                    "observer projection does not identify the exact action accepted/rejected by the validator".into(),
                );
            }
            let validator_event_id = to
                .get("data")
                .and_then(|data| data.get("validator_event_id"))
                .and_then(Value::as_str);
            let from_event_id = from.get("event_id").and_then(Value::as_str);
            if validator_event_id.is_none() || from_event_id.is_none() {
                return RelationValidation::Incomplete(
                    "observer_confirmation requires validator_event_id linked to the exact validation event".into(),
                );
            }
            if validator_event_id != from_event_id {
                return RelationValidation::Violation(
                    "observer projection references a different validator event".into(),
                );
            }
            RelationValidation::Valid
        }
        "test_harness_barrier" => {
            if evidence_class != "synthetic" {
                RelationValidation::Violation(
                    "test_harness_barrier is permitted only for synthetic histories".into(),
                )
            } else {
                RelationValidation::Valid
            }
        }
        _ => RelationValidation::Incomplete(format!(
            "causal relation {relation} is unsupported by Pulse causal profile v1"
        )),
    }
}

fn audit(
    bundle_bytes: &[u8],
    trace_bytes: &[u8],
    policy_bytes: &[u8],
) -> Result<Report, Box<dyn Error>> {
    let bundle_sha256 = sha256_hex(bundle_bytes);
    let trace_sha256 = sha256_hex(trace_bytes);
    let policy_sha256 = sha256_hex(policy_bytes);

    let bundle: Bundle = serde_json::from_slice(bundle_bytes)?;
    let policy: TrustPolicy = serde_json::from_slice(policy_bytes)?;
    let trace: Value = serde_json::from_slice(trace_bytes)?;
    let mut findings = Vec::new();
    let mut trace_event_map: BTreeMap<String, &Value> = BTreeMap::new();
    match trace.get("events").and_then(Value::as_array) {
        Some(events) => {
            for event in events {
                let Some(event_id) = event.get("event_id").and_then(Value::as_str) else {
                    finding(
                        &mut findings,
                        "TRACE_EVENT_ID_MISSING",
                        "incomplete",
                        None,
                        "trace event has no string event_id",
                    );
                    continue;
                };
                if event_id.trim().is_empty() {
                    finding(
                        &mut findings,
                        "TRACE_EVENT_ID_EMPTY",
                        "incomplete",
                        None,
                        "trace event_id must be nonempty",
                    );
                }
                if trace_event_map
                    .insert(event_id.to_string(), event)
                    .is_some()
                {
                    finding(
                        &mut findings,
                        "TRACE_EVENT_ID_DUPLICATE",
                        "violation",
                        None,
                        format!("trace event_id {event_id} is not unique"),
                    );
                }
            }
        }
        None => finding(
            &mut findings,
            "TRACE_EVENTS_MISSING",
            "incomplete",
            None,
            "signed event attestations require trace.events array",
        ),
    }
    let mut dsse_verified = 0usize;
    let mut scitt_unverified = 0usize;

    if bundle.schema_version != 1 {
        finding(
            &mut findings,
            "BUNDLE_SCHEMA_VERSION_UNSUPPORTED",
            "incomplete",
            None,
            "expected signed-evidence bundle schema version 1",
        );
    }
    if bundle.bundle_id.trim().is_empty()
        || !matches!(
            bundle.evidence_class.as_str(),
            "synthetic" | "runtime_capture"
        )
        || !matches!(
            bundle.claim_ceiling.as_str(),
            "STRUCTURAL_ONLY" | "ROLE_AUTHENTICATED" | "TRANSPARENTLY_REGISTERED"
        )
    {
        finding(
            &mut findings,
            "BUNDLE_HEADER_INVALID",
            "incomplete",
            None,
            "bundle identity, evidence class or claim ceiling is invalid",
        );
    }
    if !valid_sha256(&bundle.subject.source_commit) && bundle.subject.source_commit.len() != 40
        || !bundle
            .subject
            .source_commit
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
    {
        finding(
            &mut findings,
            "SOURCE_COMMIT_INVALID",
            "violation",
            None,
            "source_commit must be lowercase 40-character hex",
        );
    }
    if !valid_sha256(&bundle.subject.trace_sha256)
        || !valid_sha256(&bundle.subject.schema_sha256)
        || !valid_sha256(&bundle.subject.tool_build_sha256)
    {
        finding(
            &mut findings,
            "SUBJECT_DIGEST_INVALID",
            "violation",
            None,
            "bundle subject digests must be lowercase SHA-256",
        );
    }
    if bundle.subject.trace_sha256 != trace_sha256 {
        finding(
            &mut findings,
            "TRACE_SUBJECT_DIGEST_MISMATCH",
            "violation",
            None,
            "provided trace bytes do not match the bundle's trace_sha256",
        );
    }
    if trace.get("source_commit").and_then(Value::as_str)
        != Some(bundle.subject.source_commit.as_str())
    {
        finding(
            &mut findings,
            "TRACE_SOURCE_COMMIT_MISMATCH",
            "violation",
            None,
            "trace source_commit does not match bundle subject",
        );
    }
    if bundle.trust_policy.policy_sha256 != policy_sha256
        || bundle.trust_policy.policy_id != policy.policy_id
        || bundle.trust_policy.policy_version != policy.policy_version
        || bundle.trust_policy.policy_epoch != policy.policy_epoch
        || bundle.trust_policy.root_keyset_id != policy.root_keyset_id
    {
        finding(
            &mut findings,
            "TRUST_POLICY_BINDING_MISMATCH",
            "violation",
            None,
            "supplied trust policy bytes/identity differ from the digest-bound policy reference",
        );
    }
    if policy.keys.is_empty() {
        finding(
            &mut findings,
            "TRUST_POLICY_HAS_NO_KEYS",
            "incomplete",
            None,
            "trust policy contains no issuer keys",
        );
    }
    let mut policy_key_ids = BTreeSet::new();
    for key in &policy.keys {
        if key.keyid.trim().is_empty() || !policy_key_ids.insert(key.keyid.clone()) {
            finding(
                &mut findings,
                "TRUST_POLICY_KEY_ID_INVALID_OR_DUPLICATE",
                "violation",
                None,
                "trust policy key IDs must be nonempty and unique",
            );
        }
    }
    let mut conductor_ids = BTreeSet::new();
    for lineage in &bundle.subject.conductor_lineages {
        if !conductor_ids.insert(lineage.conductor_id.clone()) {
            finding(
                &mut findings,
                "DUPLICATE_CONDUCTOR_LINEAGE",
                "violation",
                None,
                "conductor_id must be unique in bundle subject",
            );
        }
        if lineage.conductor_id.trim().is_empty()
            || lineage.agent_key_ref.trim().is_empty()
            || lineage.conductor_build.trim().is_empty()
            || lineage.dna_hash.trim().is_empty()
            || lineage.cell_id.trim().is_empty()
            || !valid_sha256(&lineage.integrity_wasm_sha256)
        {
            finding(
                &mut findings,
                "CONDUCTOR_LINEAGE_INVALID",
                "incomplete",
                None,
                "conductor lineage is missing an identity field or a valid integrity-WASM digest",
            );
        }
    }

    let mut attestation_ids = BTreeSet::new();
    let mut event_ids = BTreeSet::new();
    let mut accepted_event_ids = BTreeSet::new();
    let mut accepted_attestation_ids = BTreeSet::new();
    let mut attestation_event_ids: BTreeMap<String, String> = BTreeMap::new();

    for attestation in &bundle.attestations {
        if !attestation_ids.insert(attestation.attestation_id.clone()) {
            finding(
                &mut findings,
                "DUPLICATE_ATTESTATION_ID",
                "violation",
                Some(&attestation.attestation_id),
                "attestation_id must be unique",
            );
        }
        if !event_ids.insert(attestation.event_id.clone()) {
            finding(
                &mut findings,
                "DUPLICATE_SIGNED_EVENT_ID",
                "violation",
                Some(&attestation.attestation_id),
                "event_id must be unique within a signed bundle",
            );
        }
        attestation_event_ids.insert(
            attestation.attestation_id.clone(),
            attestation.event_id.clone(),
        );
        if !trace_event_map.contains_key(&attestation.event_id)
            && attestation.issuer_role != "independent_auditor"
        {
            finding(
                &mut findings,
                "ATTESTATION_EVENT_NOT_IN_TRACE",
                "violation",
                Some(&attestation.attestation_id),
                format!(
                    "attested event_id {} is absent from trace.events",
                    attestation.event_id
                ),
            );
        }
        if !matches!(
            attestation.exposure.as_str(),
            "public_commitment" | "restricted" | "local_only"
        ) {
            finding(
                &mut findings,
                "ATTESTATION_EXPOSURE_INVALID",
                "incomplete",
                Some(&attestation.attestation_id),
                "unsupported exposure classification",
            );
        }

        match &attestation.envelope {
            Envelope::Dsse {
                payload_type,
                payload_base64,
                signatures,
            } => {
                if payload_type != "application/vnd.in-toto+json" {
                    finding(
                        &mut findings,
                        "DSSE_PAYLOAD_TYPE_INVALID",
                        "violation",
                        Some(&attestation.attestation_id),
                        "Pulse v1 requires application/vnd.in-toto+json",
                    );
                    continue;
                }
                let payload = match STANDARD.decode(payload_base64) {
                    Ok(v) => v,
                    Err(_) => {
                        finding(
                            &mut findings,
                            "DSSE_PAYLOAD_BASE64_INVALID",
                            "violation",
                            Some(&attestation.attestation_id),
                            "DSSE payload is not valid standard base64",
                        );
                        continue;
                    }
                };
                if signatures.len() != 1 {
                    finding(
                        &mut findings,
                        "DSSE_SIGNATURE_COUNT_UNSUPPORTED",
                        "incomplete",
                        Some(&attestation.attestation_id),
                        "Pulse signed-evidence v1 requires exactly one role-specific DSSE signature per attestation",
                    );
                    continue;
                }

                // The wrapper's issuer_role is untrusted. Use the key ID only to
                // locate a candidate public key, verify the exact DSSE PAE bytes,
                // and parse the signed statement only after signature success.
                let sig = &signatures[0];
                let Some(trusted) = policy.keys.iter().find(|key| key.keyid == sig.keyid) else {
                    finding(
                        &mut findings,
                        "DSSE_SIGNER_UNKNOWN",
                        "violation",
                        Some(&attestation.attestation_id),
                        format!(
                            "signer key ID {} is not present in the supplied trust policy",
                            sig.keyid
                        ),
                    );
                    continue;
                };
                if trusted.status != "active" {
                    finding(
                        &mut findings,
                        "DSSE_SIGNER_NOT_ACTIVE",
                        "violation",
                        Some(&attestation.attestation_id),
                        format!(
                            "signer key {} is not active under the supplied policy",
                            sig.keyid
                        ),
                    );
                    continue;
                }
                let public_bytes = match decode_hex_32(&trusted.public_key_hex) {
                    Some(value) => value,
                    None => {
                        finding(
                            &mut findings,
                            "TRUSTED_PUBLIC_KEY_INVALID",
                            "incomplete",
                            Some(&attestation.attestation_id),
                            format!(
                                "trusted key {} is not a 32-byte Ed25519 public key",
                                sig.keyid
                            ),
                        );
                        continue;
                    }
                };
                let verifying_key = match VerifyingKey::from_bytes(&public_bytes) {
                    Ok(value) => value,
                    Err(_) => {
                        finding(
                            &mut findings,
                            "TRUSTED_PUBLIC_KEY_INVALID",
                            "incomplete",
                            Some(&attestation.attestation_id),
                            format!("trusted key {} is not a valid Ed25519 point", sig.keyid),
                        );
                        continue;
                    }
                };
                let signature_bytes = match STANDARD.decode(&sig.sig_base64) {
                    Ok(value) => value,
                    Err(_) => {
                        finding(
                            &mut findings,
                            "DSSE_SIGNATURE_BASE64_INVALID",
                            "violation",
                            Some(&attestation.attestation_id),
                            "signature is not valid standard base64",
                        );
                        continue;
                    }
                };
                let signature = match Signature::from_slice(&signature_bytes) {
                    Ok(value) => value,
                    Err(_) => {
                        finding(
                            &mut findings,
                            "DSSE_SIGNATURE_ENCODING_INVALID",
                            "violation",
                            Some(&attestation.attestation_id),
                            "signature has an invalid Ed25519 encoding",
                        );
                        continue;
                    }
                };
                let pae_bytes = pae(payload_type, &payload);
                if verifying_key.verify(&pae_bytes, &signature).is_err() {
                    finding(
                        &mut findings,
                        "DSSE_SIGNATURE_INVALID",
                        "violation",
                        Some(&attestation.attestation_id),
                        format!(
                            "signature from key {} does not verify over DSSE PAE bytes",
                            sig.keyid
                        ),
                    );
                    continue;
                }

                // Never parse or reserialize payload JSON before verifying the
                // signature over the original decoded byte sequence.
                let statement: InTotoStatement = match serde_json::from_slice(&payload) {
                    Ok(value) => value,
                    Err(error) => {
                        finding(
                            &mut findings,
                            "INTOTO_STATEMENT_INVALID",
                            "violation",
                            Some(&attestation.attestation_id),
                            format!("cannot parse signed in-toto statement: {error}"),
                        );
                        continue;
                    }
                };
                let expected_predicate_type = predicate_type_for_role(&attestation.issuer_role);
                if statement.statement_type != "https://in-toto.io/Statement/v1"
                    || expected_predicate_type != Some(statement.predicate_type.as_str())
                    || statement.predicate.role != attestation.issuer_role
                    || statement.predicate.event_id != attestation.event_id
                    || statement.predicate.source_commit != bundle.subject.source_commit
                    || statement.predicate.trace_sha256 != bundle.subject.trace_sha256
                {
                    finding(
                        &mut findings,
                        "SIGNED_STATEMENT_BINDING_MISMATCH",
                        "violation",
                        Some(&attestation.attestation_id),
                        "signed in-toto statement role, predicate type, event ID, source commit or trace digest does not match its bundle context",
                    );
                    continue;
                }
                let expected_subject_name = format!(
                    "pulse-trace:{}@{}",
                    bundle.subject.repository, bundle.subject.source_commit
                );
                let exact_subject = statement.subject.len() == 1
                    && statement.subject[0].name == expected_subject_name
                    && statement.subject[0].digest.get("sha256")
                        == Some(&bundle.subject.trace_sha256);
                if !exact_subject {
                    finding(
                        &mut findings,
                        "INTOTO_SUBJECT_MISMATCH",
                        "violation",
                        Some(&attestation.attestation_id),
                        "in-toto subject must bind the exact repository/source commit and trace digest",
                    );
                    continue;
                }
                if statement.predicate.issuer_key_id != sig.keyid {
                    finding(
                        &mut findings,
                        "PREDICATE_ISSUER_KEY_MISMATCH",
                        "violation",
                        Some(&attestation.attestation_id),
                        "signed predicate issuer_key_id must match the exact DSSE signature key ID",
                    );
                    continue;
                }
                if !trusted
                    .roles
                    .iter()
                    .any(|role| role == &attestation.issuer_role)
                    || !trusted
                        .predicate_types
                        .iter()
                        .any(|kind| kind == &statement.predicate_type)
                {
                    finding(
                        &mut findings,
                        "DSSE_SIGNER_NOT_AUTHORIZED_FOR_ROLE",
                        "violation",
                        Some(&attestation.attestation_id),
                        format!(
                            "key {} is not authorized for role {} and predicate {}",
                            sig.keyid, attestation.issuer_role, statement.predicate_type
                        ),
                    );
                    continue;
                }
                if let Err(message) = verify_issuer_subject_binding(
                    &attestation.issuer_role,
                    &statement.predicate.details,
                    trusted,
                    &bundle.subject,
                ) {
                    finding(
                        &mut findings,
                        "DSSE_SIGNER_SUBJECT_BINDING_MISMATCH",
                        "violation",
                        Some(&attestation.attestation_id),
                        message,
                    );
                    continue;
                }
                if let Err(message) = verify_statement_trace_event_binding(
                    &attestation.issuer_role,
                    &attestation.event_id,
                    &statement.predicate.details,
                    &trace,
                    &bundle.subject,
                    &trace_event_map,
                ) {
                    finding(
                        &mut findings,
                        "DSSE_TRACE_EVENT_BINDING_MISMATCH",
                        "violation",
                        Some(&attestation.attestation_id),
                        message,
                    );
                    continue;
                }
                dsse_verified += 1;
                accepted_event_ids.insert(attestation.event_id.clone());
                accepted_attestation_ids.insert(attestation.attestation_id.clone());
            }
            Envelope::Scitt {
                media_type,
                signed_statement_cose_base64,
                statement_sha256,
                receipts,
            } => {
                let bytes = match STANDARD.decode(signed_statement_cose_base64) {
                    Ok(value) => value,
                    Err(_) => {
                        finding(
                            &mut findings,
                            "SCITT_COSE_BASE64_INVALID",
                            "violation",
                            Some(&attestation.attestation_id),
                            "SCITT signed statement is not valid standard base64",
                        );
                        continue;
                    }
                };
                if media_type != "application/scitt-statement+cose"
                    || !valid_sha256(statement_sha256)
                    || sha256_hex(&bytes) != *statement_sha256
                {
                    finding(
                        &mut findings,
                        "SCITT_STATEMENT_COMMITMENT_INVALID",
                        "violation",
                        Some(&attestation.attestation_id),
                        "SCITT media type or signed-statement SHA-256 commitment is invalid",
                    );
                    continue;
                }

                // Validate the declared receipt envelope shape and content
                // commitments, but do not authenticate TS keys or VDS proofs.
                // A syntactically valid receipt remains INCOMPLETE until the
                // selected COSE/SCITT verifier and trust policy are implemented.
                if receipts.is_empty() {
                    finding(
                        &mut findings,
                        "SCITT_RECEIPTS_MISSING",
                        "incomplete",
                        Some(&attestation.attestation_id),
                        "SCITT evidence has no transparency-service receipts",
                    );
                }
                let mut receipt_ids = BTreeSet::new();
                for receipt in receipts {
                    if receipt.media_type != "application/scitt-receipt+cose"
                        || receipt.ts_issuer.trim().is_empty()
                        || receipt.ts_keyid.trim().is_empty()
                        || !receipt_ids
                            .insert((receipt.ts_issuer.clone(), receipt.ts_keyid.clone()))
                        || !valid_sha256(&receipt.registration_policy_sha256)
                    {
                        finding(
                            &mut findings,
                            "SCITT_RECEIPT_METADATA_INVALID",
                            "violation",
                            Some(&attestation.attestation_id),
                            "SCITT receipt media type, issuer, key ID or registration-policy digest is invalid or duplicated",
                        );
                        continue;
                    }
                    if STANDARD.decode(&receipt.receipt_cose_base64).is_err() {
                        finding(
                            &mut findings,
                            "SCITT_RECEIPT_BASE64_INVALID",
                            "violation",
                            Some(&attestation.attestation_id),
                            "SCITT receipt is not valid standard base64",
                        );
                    }
                }
                scitt_unverified += 1;
                finding(
                    &mut findings,
                    "SCITT_COSE_VERIFICATION_UNIMPLEMENTED",
                    "incomplete",
                    Some(&attestation.attestation_id),
                    "COSE signatures, receipt authenticity and transparency-service VDS proofs are not verified by this DSSE-only verifier",
                );
            }
        }
    }

    for edge in &bundle.causal_edges {
        if !event_ids.contains(&edge.from_event_id)
            || !event_ids.contains(&edge.to_event_id)
            || !trace_event_map.contains_key(&edge.from_event_id)
            || !trace_event_map.contains_key(&edge.to_event_id)
        {
            finding(
                &mut findings,
                "CAUSAL_EDGE_ENDPOINT_MISSING",
                "incomplete",
                None,
                "causal edge endpoints must exist in the exact trace and have signed attestations",
            );
        }
        if edge.basis_attestation_ids.is_empty()
            || !edge
                .basis_attestation_ids
                .iter()
                .all(|id| attestation_ids.contains(id))
        {
            finding(
                &mut findings,
                "CAUSAL_EDGE_ATTESTATION_MISSING",
                "incomplete",
                None,
                "causal edge must reference one or more attestation IDs in this bundle",
            );
        } else {
            if !edge_basis_matches_authenticated_endpoints(
                &edge.basis_attestation_ids,
                &attestation_event_ids,
                &accepted_attestation_ids,
                &accepted_event_ids,
                &edge.from_event_id,
                &edge.to_event_id,
            ) {
                let all_basis_authenticated = edge
                    .basis_attestation_ids
                    .iter()
                    .all(|id| accepted_attestation_ids.contains(id));
                finding(
                    &mut findings,
                    if all_basis_authenticated {
                        "CAUSAL_EDGE_BASIS_EVENT_MISMATCH"
                    } else {
                        "CAUSAL_EDGE_BASIS_NOT_AUTHENTICATED"
                    },
                    if all_basis_authenticated {
                        "violation"
                    } else {
                        "incomplete"
                    },
                    None,
                    "causal edge basis must include DSSE-authenticated attestations for both exact endpoint events",
                );
            }
        }
        if edge.from_event_id == edge.to_event_id {
            finding(
                &mut findings,
                "CAUSAL_EDGE_SELF_REFERENCE",
                "violation",
                None,
                "causal edge cannot point to itself",
            );
        }
        if let (Some(from_event), Some(to_event)) = (
            trace_event_map.get(&edge.from_event_id),
            trace_event_map.get(&edge.to_event_id),
        ) {
            match validate_causal_relation(
                &edge.relation,
                from_event,
                to_event,
                &bundle.evidence_class,
            ) {
                RelationValidation::Valid => {}
                RelationValidation::Incomplete(message) => finding(
                    &mut findings,
                    "CAUSAL_EDGE_SEMANTICS_INCOMPLETE",
                    "incomplete",
                    None,
                    message,
                ),
                RelationValidation::Violation(message) => finding(
                    &mut findings,
                    "CAUSAL_EDGE_SEMANTICS_MISMATCH",
                    "violation",
                    None,
                    message,
                ),
            }
        }
        if !matches!(
            edge.relation.as_str(),
            "program_order"
                | "request_response"
                | "source_action_dependency"
                | "validator_receipt"
                | "observer_confirmation"
                | "test_harness_barrier"
        ) {
            finding(
                &mut findings,
                "CAUSAL_EDGE_RELATION_INVALID",
                "incomplete",
                None,
                "unsupported causal relation",
            );
        }
    }
    if graph_has_cycle(&bundle.causal_edges, &event_ids) {
        finding(
            &mut findings,
            "CAUSAL_GRAPH_CYCLE",
            "violation",
            None,
            "signed evidence causal graph must be acyclic",
        );
    }

    let violation = findings.iter().any(|f| f.severity == "violation");
    let incomplete = findings.iter().any(|f| f.severity == "incomplete");
    let status = if violation {
        "FAIL"
    } else if incomplete {
        "INCOMPLETE"
    } else if dsse_verified == bundle.attestations.len() && !bundle.attestations.is_empty() {
        "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY"
    } else {
        "INCOMPLETE"
    };

    Ok(Report {
        schema_version: 1,
        status: status.to_string(),
        evidence_class: bundle.evidence_class.clone(),
        bundle_id: bundle.bundle_id,
        bundle_sha256,
        trace_sha256,
        trust_policy_sha256: policy_sha256,
        trust_policy_root_authenticated: false,
        trust_root_key_id: None,
        trust_policy_epoch: Some(policy.policy_epoch),
        attestations: bundle.attestations.len(),
        dsse_verified,
        scitt_unverified,
        causal_edges: bundle.causal_edges.len(),
        findings,
        claim_ceiling: "DSSE_ROLE_AUTHENTICATION_RELATIVE_TO_SUPPLIED_POLICY_ONLY: the trust policy is an external input whose authenticity/currentness is not established here; SCITT COSE and transparency receipts are not yet verified; signatures do not prove asserted events are true or authorize actions".to_string(),
    })
}

fn require_root_for_runtime_capture(report: &mut Report) {
    if report.evidence_class == "runtime_capture"
        && report.status == "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY"
    {
        finding(
            &mut report.findings,
            "RUNTIME_CAPTURE_REQUIRES_PINNED_ROOT",
            "incomplete",
            None,
            "runtime_capture evidence must be verified with the relying party's out-of-band pinned root; relative-to-supplied-policy authentication is insufficient",
        );
        report.status = "INCOMPLETE".to_string();
        report.claim_ceiling = "INCOMPLETE: runtime_capture cannot be accepted from a caller-supplied trust policy without a separately provisioned and verified out-of-band root pin; SCITT and event-truth claims remain separate".to_string();
    }
}

fn audit_with_root_pin(
    bundle_bytes: &[u8],
    trace_bytes: &[u8],
    policy_bytes: &[u8],
    root_envelope_bytes: &[u8],
    root_pin_bytes: &[u8],
) -> Result<Report, Box<dyn Error>> {
    let root = verify_root_signed_trust_policy(
        policy_bytes,
        root_envelope_bytes,
        root_pin_bytes,
    )
    .map_err(|error| format!("trust-policy root verification failed: {error}"))?;
    let mut report = audit(bundle_bytes, trace_bytes, policy_bytes)?;
    report.trust_policy_root_authenticated = true;
    report.trust_root_key_id = Some(root.root_key_id);
    report.trust_policy_epoch = Some(root.policy_epoch);
    if report.status == "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY" {
        report.status = "ROOT_AUTHENTICATED_POLICY_SNAPSHOT".to_string();
        report.claim_ceiling = "ROOT_AUTHENTICATED_POLICY_SNAPSHOT_ONLY: the exact trust-policy bytes and epoch are signed by the relying party's out-of-band pinned Ed25519 root; issuer roles remain relative to that policy; SCITT COSE and transparency receipts are not verified; signatures do not prove asserted events true or authorize actions".to_string();
    } else {
        report.claim_ceiling.push_str("; trust policy root signature and exact policy snapshot pin verified, but other evidence findings still limit the report");
    }
    if root.policy_sha256 != report.trust_policy_sha256 {
        return Err("root-verified trust policy digest differs from the evidence report".into());
    }
    Ok(report)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut bundle = None;
    let mut trace = None;
    let mut policy = None;
    let mut root_policy_envelope = None;
    let mut trusted_root_pin = None;
    let mut output = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--bundle" => {
                bundle = Some(PathBuf::from(
                    args.next().ok_or("--bundle requires a path")?,
                ))
            }
            "--trace" => trace = Some(PathBuf::from(args.next().ok_or("--trace requires a path")?)),
            "--trust-policy" => {
                policy = Some(PathBuf::from(
                    args.next().ok_or("--trust-policy requires a path")?,
                ))
            }
            "--root-policy-envelope" => {
                root_policy_envelope = Some(PathBuf::from(
                    args.next().ok_or("--root-policy-envelope requires a path")?,
                ))
            }
            "--trusted-root-pin" => {
                trusted_root_pin = Some(PathBuf::from(
                    args.next().ok_or("--trusted-root-pin requires a path")?,
                ))
            }
            "--output" => {
                output = Some(PathBuf::from(
                    args.next().ok_or("--output requires a path")?,
                ))
            }
            "--help" | "-h" => {
                println!(
                    "pulse-signed-evidence-audit --bundle BUNDLE.json --trace TRACE.json --trust-policy TRUST.json [--root-policy-envelope ROOT-DSSE.json --trusted-root-pin ROOT-PIN.json] [--output REPORT.json]"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}").into()),
        }
    }
    let bundle_path = bundle.ok_or("--bundle is required")?;
    let trace_path = trace.ok_or("--trace is required")?;
    let policy_path = policy.ok_or("--trust-policy is required")?;
    if root_policy_envelope.is_some() != trusted_root_pin.is_some() {
        return Err("--root-policy-envelope and --trusted-root-pin must be supplied together".into());
    }
    let bundle_bytes = fs::read(bundle_path)?;
    let trace_bytes = fs::read(trace_path)?;
    let policy_bytes = fs::read(policy_path)?;
    let root_pinned_mode = root_policy_envelope.is_some() && trusted_root_pin.is_some();
    let mut report = match (root_policy_envelope, trusted_root_pin) {
        (Some(envelope_path), Some(pin_path)) => audit_with_root_pin(
            &bundle_bytes,
            &trace_bytes,
            &policy_bytes,
            &fs::read(envelope_path)?,
            &fs::read(pin_path)?,
        )?,
        (None, None) => audit(&bundle_bytes, &trace_bytes, &policy_bytes)?,
        _ => unreachable!("the paired trust-root CLI arguments were checked above"),
    };
    if !root_pinned_mode {
        require_root_for_runtime_capture(&mut report);
    }
    let json = serde_json::to_string_pretty(&report)?;
    if let Some(path) = output {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(path, &json)?;
    } else {
        println!("{json}");
    }
    match report.status.as_str() {
        "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY"
        | "ROOT_AUTHENTICATED_POLICY_SNAPSHOT" => Ok(()),
        "FAIL" => Err("signed evidence verification failed".into()),
        _ => Err("signed evidence verification incomplete".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use serde_json::json;

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    fn fixture() -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let source_commit = "a".repeat(40);
        let event_data = json!({
            "grant_ref": "grant-A",
            "grant_action_hash": "uhCkk-test-grant",
            "grantor": "agent-owner-pseudonym",
            "grantee": "agent-recipient",
            "assigned": true,
            "functions": ["messages.send"],
            "idempotency_key": "grant-op-1",
            "request_digest_sha256": "a".repeat(64),
            "host_authority_index": 1
        });
        let trace_bytes = serde_json::to_vec(&json!({
            "source_commit": source_commit,
            "trace_id": "synthetic-signed-trace-1",
            "events": [{
                "event_id": "host-grant-observation-1",
                "conductor_id": "owner-host",
                "local_sequence": 1,
                "source": "conductor_host",
                "evidence_ref": "fixture://owner/grant",
                "event_type": "host_grant_issued",
                "data": event_data.clone()
            }]
        }))
        .unwrap();
        let trace_digest = sha256_hex(&trace_bytes);
        let signing_key = SigningKey::from_bytes(&[7u8; 32]);
        let verifying_key = signing_key.verifying_key();
        let key_id = "test-host-key-1";
        let role = "host_authority";
        let predicate_type = predicate_type_for_role(role).unwrap();

        let statement = json!({
            "_type": "https://in-toto.io/Statement/v1",
            "subject": [{
                "name": format!("pulse-trace:example/mycelix@{source_commit}"),
                "digest": { "sha256": trace_digest }
            }],
            "predicateType": predicate_type,
            "predicate": {
                "role": role,
                "event_id": "host-grant-observation-1",
                "issuer_key_id": key_id,
                "source_commit": source_commit,
                "trace_sha256": trace_digest,
                "details": {
                    "event_type": "host_grant_issued",
                    "source": "conductor_host",
                    "conductor_id": "owner-host",
                    "local_sequence": 1,
                    "evidence_ref": "fixture://owner/grant",
                    "event_data": event_data,
                    "agent_key_ref": "agent-owner-pseudonym",
                    "dna_hash": "uhCkk-dna-mail",
                    "cell_id": "mail-cell-owner",
                    "integrity_wasm_sha256": "d".repeat(64)
                }
            }
        });
        let payload = serde_json::to_vec(&statement).unwrap();
        let payload_type = "application/vnd.in-toto+json";
        let signature = signing_key.sign(&pae(payload_type, &payload));
        let bundle = json!({
            "schema_version": 1,
            "bundle_id": "synthetic-bundle-1",
            "evidence_class": "synthetic",
            "subject": {
                "repository": "example/mycelix",
                "source_commit": source_commit,
                "trace_sha256": trace_digest,
                "schema_sha256": "b".repeat(64),
                "tool_build_sha256": "c".repeat(64),
                "conductor_lineages": [{
                    "conductor_id": "owner-host",
                    "agent_key_ref": "agent-owner-pseudonym",
                    "conductor_build": "holochain-test",
                    "dna_hash": "uhCkk-dna-mail",
                    "cell_id": "mail-cell-owner",
                    "integrity_wasm_sha256": "d".repeat(64)
                }]
            },
            "trust_policy": {
                "policy_id": "pulse-test-root",
                "policy_version": "1.0.0",
                "policy_epoch": 1,
                "policy_sha256": "__POLICY_DIGEST__",
                "root_keyset_id": "test-root-set-1"
            },
            "attestations": [{
                "attestation_id": "att-host-grant-1",
                "event_id": "host-grant-observation-1",
                "issuer_role": role,
                "exposure": "restricted",
                "envelope": {
                    "format": "dsse_in_toto_v1",
                    "payload_type": payload_type,
                    "payload_base64": STANDARD.encode(payload),
                    "signatures": [{
                        "keyid": key_id,
                        "sig_base64": STANDARD.encode(signature.to_bytes())
                    }]
                }
            }],
            "causal_edges": [],
            "claim_ceiling": "ROLE_AUTHENTICATED"
        });
        let policy = json!({
            "policy_id": "pulse-test-root",
            "policy_version": "1.0.0",
            "policy_epoch": 1,
            "root_keyset_id": "test-root-set-1",
            "keys": [{
                "keyid": key_id,
                "public_key_hex": hex(verifying_key.as_bytes()),
                "roles": [role],
                "predicate_types": [predicate_type],
                "status": "active",
                "subject_binding": {
                    "conductor_id": "owner-host",
                    "agent_key_ref": "agent-owner-pseudonym",
                    "dna_hash": "uhCkk-dna-mail",
                    "cell_id": "mail-cell-owner",
                    "integrity_wasm_sha256": "d".repeat(64)
                }
            }]
        });
        let policy_bytes = serde_json::to_vec(&policy).unwrap();
        let policy_digest = sha256_hex(&policy_bytes);
        let mut bundle = bundle;
        bundle["trust_policy"]["policy_sha256"] = Value::String(policy_digest);
        (
            serde_json::to_vec(&bundle).unwrap(),
            trace_bytes,
            policy_bytes,
        )
    }

    fn resign_first_attestation(bundle: &mut Value, mutate: impl FnOnce(&mut Value)) {
        let payload_type = bundle["attestations"][0]["envelope"]["payload_type"]
            .as_str()
            .unwrap()
            .to_string();
        let payload = STANDARD
            .decode(
                bundle["attestations"][0]["envelope"]["payload_base64"]
                    .as_str()
                    .unwrap(),
            )
            .unwrap();
        let mut statement: Value = serde_json::from_slice(&payload).unwrap();
        mutate(&mut statement);
        let changed_payload = serde_json::to_vec(&statement).unwrap();
        let signing_key = SigningKey::from_bytes(&[7u8; 32]);
        let signature = signing_key.sign(&pae(&payload_type, &changed_payload));
        bundle["attestations"][0]["envelope"]["payload_base64"] =
            Value::String(STANDARD.encode(&changed_payload));
        bundle["attestations"][0]["envelope"]["signatures"][0]["sig_base64"] =
            Value::String(STANDARD.encode(signature.to_bytes()));
    }

    #[test]
    fn signed_event_id_must_exist_in_the_exact_trace() {
        let (bundle, trace, policy) = fixture();
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        bundle_value["attestations"][0]["event_id"] = Value::String("event-not-in-trace".into());
        resign_first_attestation(&mut bundle_value, |statement| {
            statement["predicate"]["event_id"] = Value::String("event-not-in-trace".into());
        });
        let report = audit(&serde_json::to_vec(&bundle_value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report.findings.iter().any(|finding| {
                finding.code == "ATTESTATION_EVENT_NOT_IN_TRACE"
                    || finding.code == "DSSE_TRACE_EVENT_BINDING_MISMATCH"
            }),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn signed_event_data_must_equal_the_trace_event_data() {
        let (bundle, trace, policy) = fixture();
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        resign_first_attestation(&mut bundle_value, |statement| {
            statement["predicate"]["details"]["event_data"]["grantee"] =
                Value::String("agent-attacker".into());
        });
        let report = audit(&serde_json::to_vec(&bundle_value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|finding| { finding.code == "DSSE_TRACE_EVENT_BINDING_MISMATCH" }),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn signed_event_source_and_type_must_match_the_role_and_trace() {
        let (bundle, trace, policy) = fixture();
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        resign_first_attestation(&mut bundle_value, |statement| {
            statement["predicate"]["details"]["source"] = Value::String("client".into());
            statement["predicate"]["details"]["event_type"] =
                Value::String("caller_outcome".into());
        });
        let report = audit(&serde_json::to_vec(&bundle_value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|finding| { finding.code == "DSSE_TRACE_EVENT_BINDING_MISMATCH" }),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn causal_edge_basis_requires_authenticated_statements_for_both_endpoints() {
        let attestation_events = BTreeMap::from([
            ("att-from".to_string(), "event-from".to_string()),
            ("att-to".to_string(), "event-to".to_string()),
            ("att-unrelated".to_string(), "event-unrelated".to_string()),
        ]);
        let accepted_attestations = BTreeSet::from([
            "att-from".to_string(),
            "att-to".to_string(),
            "att-unrelated".to_string(),
        ]);
        let accepted_events = BTreeSet::from([
            "event-from".to_string(),
            "event-to".to_string(),
            "event-unrelated".to_string(),
        ]);

        assert!(edge_basis_matches_authenticated_endpoints(
            &["att-from".into(), "att-to".into()],
            &attestation_events,
            &accepted_attestations,
            &accepted_events,
            "event-from",
            "event-to",
        ));
        assert!(!edge_basis_matches_authenticated_endpoints(
            &["att-from".into(), "att-unrelated".into()],
            &attestation_events,
            &accepted_attestations,
            &accepted_events,
            "event-from",
            "event-to",
        ));
        assert!(!edge_basis_matches_authenticated_endpoints(
            &["att-from".into(), "att-to".into(), "att-unrelated".into()],
            &attestation_events,
            &accepted_attestations,
            &accepted_events,
            "event-from",
            "event-to",
        ));
        assert!(!edge_basis_matches_authenticated_endpoints(
            &["att-from".into()],
            &attestation_events,
            &accepted_attestations,
            &accepted_events,
            "event-from",
            "event-to",
        ));
    }

    #[test]
    fn unverified_attestation_cannot_support_a_causal_edge() {
        let attestation_events = BTreeMap::from([
            ("att-from".to_string(), "event-from".to_string()),
            ("att-to".to_string(), "event-to".to_string()),
            ("att-extra".to_string(), "event-extra".to_string()),
        ]);
        // Only the source endpoint is independently authenticated. The
        // presence of an attestation ID in the bundle or event map is not
        // sufficient to make the destination endpoint trusted.
        let accepted_attestations = BTreeSet::from(["att-from".to_string()]);
        let accepted_events = BTreeSet::from(["event-from".to_string(), "event-to".to_string()]);
        assert!(!edge_basis_matches_authenticated_endpoints(
            &["att-from".into(), "att-to".into()],
            &attestation_events,
            &accepted_attestations,
            &accepted_events,
            "event-from",
            "event-to",
        ));

        // Even when a third attestation is validly signed, it cannot replace
        // evidence for either causal endpoint.
        let all_attestations_accepted = BTreeSet::from([
            "att-from".to_string(),
            "att-to".to_string(),
            "att-extra".to_string(),
        ]);
        assert!(!edge_basis_matches_authenticated_endpoints(
            &["att-from".into(), "att-to".into(), "att-extra".into()],
            &attestation_events,
            &all_attestations_accepted,
            &accepted_events,
            "event-from",
            "event-to",
        ));

        assert!(!edge_basis_matches_authenticated_endpoints(
            &["att-from".into()],
            &attestation_events,
            &all_attestations_accepted,
            &accepted_events,
            "event-from",
            "event-to",
        ));
    }

    fn fixture_with_two_signed_events_and_edge() -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let (bundle, trace, policy) = fixture();
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        let mut trace_value: Value = serde_json::from_slice(&trace).unwrap();

        let second_event_data = json!({
            "operation_id": "send-op-1",
            "attempt_id": 1,
            "grant_ref": "grant-A",
            "grant_action_hash": "uhCkk-test-grant",
            "caller": "agent-recipient",
            "function": "messages.send",
            "host_authority_index": 2,
            "host_decision": "authorized",
            "host_state_at_decision": "active",
            "domain_decision": "allow",
            "projection_used_for_authorization": false
        });
        trace_value["events"].as_array_mut().unwrap().push(json!({
            "event_id": "host-call-observation-2",
            "conductor_id": "owner-host",
            "local_sequence": 2,
            "source": "conductor_host",
            "evidence_ref": "fixture://owner/call",
            "event_type": "host_call",
            "data": second_event_data
        }));
        let trace_bytes = serde_json::to_vec(&trace_value).unwrap();
        let trace_digest = sha256_hex(&trace_bytes);
        let source_commit = bundle_value["subject"]["source_commit"]
            .as_str()
            .unwrap()
            .to_string();
        let repository = bundle_value["subject"]["repository"]
            .as_str()
            .unwrap()
            .to_string();
        bundle_value["subject"]["trace_sha256"] = Value::String(trace_digest.clone());

        let payload_type = "application/vnd.in-toto+json";
        let signing_key = SigningKey::from_bytes(&[7u8; 32]);
        let key_id = "test-host-key-1";
        let predicate_type = predicate_type_for_role("host_authority").unwrap();

        let first_payload = STANDARD
            .decode(
                bundle_value["attestations"][0]["envelope"]["payload_base64"]
                    .as_str()
                    .unwrap(),
            )
            .unwrap();
        let mut first_statement: Value = serde_json::from_slice(&first_payload).unwrap();
        first_statement["subject"][0]["digest"]["sha256"] = Value::String(trace_digest.clone());
        first_statement["predicate"]["trace_sha256"] = Value::String(trace_digest.clone());
        let first_bytes = serde_json::to_vec(&first_statement).unwrap();
        let first_signature = signing_key.sign(&pae(payload_type, &first_bytes));
        bundle_value["attestations"][0]["envelope"]["payload_base64"] =
            Value::String(STANDARD.encode(&first_bytes));
        bundle_value["attestations"][0]["envelope"]["signatures"][0]["sig_base64"] =
            Value::String(STANDARD.encode(first_signature.to_bytes()));

        let second_statement = json!({
            "_type": "https://in-toto.io/Statement/v1",
            "subject": [{
                "name": format!("pulse-trace:{repository}@{source_commit}"),
                "digest": { "sha256": trace_digest }
            }],
            "predicateType": predicate_type,
            "predicate": {
                "role": "host_authority",
                "event_id": "host-call-observation-2",
                "issuer_key_id": key_id,
                "source_commit": source_commit,
                "trace_sha256": bundle_value["subject"]["trace_sha256"].clone(),
                "details": {
                    "event_type": "host_call",
                    "source": "conductor_host",
                    "conductor_id": "owner-host",
                    "local_sequence": 2,
                    "evidence_ref": "fixture://owner/call",
                    "event_data": {
                        "operation_id": "send-op-1",
                        "attempt_id": 1,
                        "grant_ref": "grant-A",
                        "grant_action_hash": "uhCkk-test-grant",
                        "caller": "agent-recipient",
                        "function": "messages.send",
                        "host_authority_index": 2,
                        "host_decision": "authorized",
                        "host_state_at_decision": "active",
                        "domain_decision": "allow",
                        "projection_used_for_authorization": false
                    },
                    "agent_key_ref": "agent-owner-pseudonym",
                    "dna_hash": "uhCkk-dna-mail",
                    "cell_id": "mail-cell-owner",
                    "integrity_wasm_sha256": "d".repeat(64)
                }
            }
        });
        let second_bytes = serde_json::to_vec(&second_statement).unwrap();
        let second_signature = signing_key.sign(&pae(payload_type, &second_bytes));
        bundle_value["attestations"]
            .as_array_mut()
            .unwrap()
            .push(json!({
                "attestation_id": "att-host-call-2",
                "event_id": "host-call-observation-2",
                "issuer_role": "host_authority",
                "exposure": "restricted",
                "envelope": {
                    "format": "dsse_in_toto_v1",
                    "payload_type": payload_type,
                    "payload_base64": STANDARD.encode(&second_bytes),
                    "signatures": [{
                        "keyid": key_id,
                        "sig_base64": STANDARD.encode(second_signature.to_bytes())
                    }]
                }
            }));
        bundle_value["causal_edges"] = json!([{
            "from_event_id": "host-grant-observation-1",
            "to_event_id": "host-call-observation-2",
            "relation": "program_order",
            "basis_attestation_ids": ["att-host-grant-1", "att-host-call-2"]
        }]);

        (
            serde_json::to_vec(&bundle_value).unwrap(),
            trace_bytes,
            policy,
        )
    }

    #[test]
    fn program_order_is_conductor_local_and_strictly_increasing() {
        let from = json!({
            "conductor_id": "peer-a",
            "local_sequence": 4,
            "event_type": "host_call"
        });
        let mut to = json!({
            "conductor_id": "peer-a",
            "local_sequence": 5,
            "event_type": "caller_outcome"
        });
        assert!(matches!(
            validate_causal_relation("program_order", &from, &to, "runtime_capture"),
            RelationValidation::Valid
        ));
        to["conductor_id"] = Value::String("peer-b".into());
        assert!(matches!(
            validate_causal_relation("program_order", &from, &to, "runtime_capture"),
            RelationValidation::Violation(_)
        ));
        to["conductor_id"] = Value::String("peer-a".into());
        to["local_sequence"] = Value::from(4);
        assert!(matches!(
            validate_causal_relation("program_order", &from, &to, "runtime_capture"),
            RelationValidation::Violation(_)
        ));
    }

    #[test]
    fn request_response_requires_exact_operation_and_attempt() {
        let from = json!({
            "event_type": "host_call",
            "data": {"operation_id": "op-1", "attempt_id": 2}
        });
        let mut to = json!({
            "event_type": "domain_effect",
            "data": {"operation_id": "op-1", "attempt_id": 2}
        });
        assert!(matches!(
            validate_causal_relation("request_response", &from, &to, "runtime_capture"),
            RelationValidation::Valid
        ));
        to["data"]["attempt_id"] = Value::from(3);
        assert!(matches!(
            validate_causal_relation("request_response", &from, &to, "runtime_capture"),
            RelationValidation::Violation(_)
        ));
        to["event_type"] = Value::String("projection_observed".into());
        assert!(matches!(
            validate_causal_relation("request_response", &from, &to, "runtime_capture"),
            RelationValidation::Incomplete(_)
        ));
    }

    #[test]
    fn grant_claim_dependency_requires_exact_grant_action_hash() {
        let from = json!({
            "event_type": "host_grant_issued",
            "data": {"grant_action_hash": "grant-A"}
        });
        let mut to = json!({
            "event_type": "capability_claim_accepted",
            "data": {"grant_action_hash": "grant-A"}
        });
        assert!(matches!(
            validate_causal_relation("source_action_dependency", &from, &to, "runtime_capture"),
            RelationValidation::Valid
        ));
        to["data"]["grant_action_hash"] = Value::String("grant-B".into());
        assert!(matches!(
            validate_causal_relation("source_action_dependency", &from, &to, "runtime_capture"),
            RelationValidation::Violation(_)
        ));
    }

    #[test]
    fn validator_receipt_requires_exact_authored_action_and_conductors() {
        let from = json!({
            "event_id": "authored-1",
            "conductor_id": "author-host",
            "event_type": "action_authored",
            "data": {"action_hash": "action-A"}
        });
        let mut to = json!({
            "conductor_id": "validator-host",
            "event_type": "validation_disposition",
            "data": {
                "action_hash": "action-A",
                "author_conductor_id": "author-host",
                "receiving_conductor_id": "validator-host",
                "disposition": "accepted"
            }
        });
        assert!(matches!(
            validate_causal_relation("validator_receipt", &from, &to, "runtime_capture"),
            RelationValidation::Valid
        ));
        to["data"]["action_hash"] = Value::String("action-B".into());
        assert!(matches!(
            validate_causal_relation("validator_receipt", &from, &to, "runtime_capture"),
            RelationValidation::Violation(_)
        ));
    }

    #[test]
    fn observer_confirmation_requires_the_exact_validator_event_and_action() {
        let from = json!({
            "event_id": "validator-1",
            "event_type": "validation_disposition",
            "data": {"action_hash": "action-A"}
        });
        let mut to = json!({
            "event_type": "projection_observed",
            "data": {
                "action_hash": "action-A",
                "observed_action_hash": "action-A",
                "required_action_hash": "action-A",
                "validator_event_id": "validator-1"
            }
        });
        assert!(matches!(
            validate_causal_relation("observer_confirmation", &from, &to, "runtime_capture"),
            RelationValidation::Valid
        ));
        to["data"]["validator_event_id"] = Value::String("validator-other".into());
        assert!(matches!(
            validate_causal_relation("observer_confirmation", &from, &to, "runtime_capture"),
            RelationValidation::Violation(_)
        ));
        to["data"]
            .as_object_mut()
            .unwrap()
            .remove("validator_event_id");
        assert!(matches!(
            validate_causal_relation("observer_confirmation", &from, &to, "runtime_capture"),
            RelationValidation::Incomplete(_)
        ));
    }

    #[test]
    fn test_harness_barrier_is_synthetic_only() {
        let from = json!({});
        let to = json!({});
        assert!(matches!(
            validate_causal_relation("test_harness_barrier", &from, &to, "synthetic"),
            RelationValidation::Valid
        ));
        assert!(matches!(
            validate_causal_relation("test_harness_barrier", &from, &to, "runtime_capture"),
            RelationValidation::Violation(_)
        ));
    }

    #[test]
    fn valid_causal_edge_is_backed_by_two_exact_signed_endpoint_events() {
        let (bundle, trace, policy) = fixture_with_two_signed_events_and_edge();
        let report = audit(&bundle, &trace, &policy).unwrap();
        assert_eq!(
            report.status, "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY",
            "{:?}",
            report.findings
        );
        assert_eq!(report.dsse_verified, 2);
        assert_eq!(report.causal_edges, 1);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    #[test]
    fn valid_dsse_statement_verifies_against_supplied_role_policy() {
        let (bundle, trace, policy) = fixture();
        let report = audit(&bundle, &trace, &policy).unwrap();
        assert_eq!(
            report.status, "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY",
            "{:?}",
            report.findings
        );
        assert_eq!(report.dsse_verified, 1);
    }

    #[test]
    fn trusted_host_key_cannot_sign_for_a_different_conductor() {
        let (bundle, trace, policy) = fixture();
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        let payload_type = bundle_value["attestations"][0]["envelope"]["payload_type"]
            .as_str()
            .unwrap()
            .to_string();
        let payload_b64 = bundle_value["attestations"][0]["envelope"]["payload_base64"]
            .as_str()
            .unwrap();
        let payload = STANDARD.decode(payload_b64).unwrap();
        let mut statement: Value = serde_json::from_slice(&payload).unwrap();
        statement["predicate"]["details"]["conductor_id"] = Value::String("validator-host".into());
        statement["predicate"]["details"]["agent_key_ref"] =
            Value::String("agent-validator".into());
        statement["predicate"]["details"]["cell_id"] = Value::String("mail-cell-validator".into());
        let changed_payload = serde_json::to_vec(&statement).unwrap();
        let signing_key = SigningKey::from_bytes(&[7u8; 32]);
        let signature = signing_key.sign(&pae(&payload_type, &changed_payload));
        bundle_value["attestations"][0]["envelope"]["payload_base64"] =
            Value::String(STANDARD.encode(&changed_payload));
        bundle_value["attestations"][0]["envelope"]["signatures"][0]["sig_base64"] =
            Value::String(STANDARD.encode(signature.to_bytes()));
        let report = audit(&serde_json::to_vec(&bundle_value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "DSSE_SIGNER_SUBJECT_BINDING_MISMATCH")
        );
    }

    #[test]
    fn host_authority_key_without_subject_binding_is_not_authorized() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&policy).unwrap();
        value["keys"][0]
            .as_object_mut()
            .unwrap()
            .remove("subject_binding");
        let policy_bytes = serde_json::to_vec(&value).unwrap();
        let policy_digest = sha256_hex(&policy_bytes);
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        bundle_value["trust_policy"]["policy_sha256"] = Value::String(policy_digest);
        let report = audit(
            &serde_json::to_vec(&bundle_value).unwrap(),
            &trace,
            &policy_bytes,
        )
        .unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "DSSE_SIGNER_SUBJECT_BINDING_MISMATCH")
        );
    }

    #[test]
    fn signature_fails_after_signed_payload_mutation() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        let payload = STANDARD
            .decode(
                value["attestations"][0]["envelope"]["payload_base64"]
                    .as_str()
                    .unwrap(),
            )
            .unwrap();
        let text = String::from_utf8(payload)
            .unwrap()
            .replace("grant_issued", "grant_denied");
        value["attestations"][0]["envelope"]["payload_base64"] =
            Value::String(STANDARD.encode(text.as_bytes()));
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "DSSE_SIGNATURE_INVALID")
        );
    }

    #[test]
    fn wrapper_role_cannot_override_signed_predicate_role() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        value["attestations"][0]["issuer_role"] = Value::String("validator".into());
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "SIGNED_STATEMENT_BINDING_MISMATCH")
        );
    }

    #[test]
    fn duplicate_trust_policy_key_ids_fail_closed() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&policy).unwrap();
        let duplicate = value["keys"][0].clone();
        value["keys"].as_array_mut().unwrap().push(duplicate);
        let policy_bytes = serde_json::to_vec(&value).unwrap();
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        bundle_value["trust_policy"]["policy_sha256"] = Value::String(sha256_hex(&policy_bytes));
        let report = audit(
            &serde_json::to_vec(&bundle_value).unwrap(),
            &trace,
            &policy_bytes,
        )
        .unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| { f.code == "TRUST_POLICY_KEY_ID_INVALID_OR_DUPLICATE" })
        );
    }

    #[test]
    fn root_keyset_reference_must_match_supplied_policy() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        value["trust_policy"]["root_keyset_id"] = Value::String("different-root-set".into());
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| { f.code == "TRUST_POLICY_BINDING_MISMATCH" })
        );
    }

    #[test]
    fn v1_rejects_ambiguous_multi_signature_thresholds() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        let signature = value["attestations"][0]["envelope"]["signatures"][0].clone();
        value["attestations"][0]["envelope"]["signatures"]
            .as_array_mut()
            .unwrap()
            .push(signature);
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "INCOMPLETE");
        assert!(
            report
                .findings
                .iter()
                .any(|f| { f.code == "DSSE_SIGNATURE_COUNT_UNSUPPORTED" })
        );
    }

    #[test]
    fn trust_policy_bytes_are_digest_bound() {
        let (bundle, trace, policy) = fixture();
        let mut altered = policy.clone();
        altered.extend_from_slice(b" ");
        let report = audit(&bundle, &trace, &altered).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "TRUST_POLICY_BINDING_MISMATCH")
        );
    }

    #[test]
    fn trace_bytes_are_subject_digest_bound() {
        let (bundle, trace, policy) = fixture();
        let mut altered = trace.clone();
        altered.push(b' ');
        let report = audit(&bundle, &altered, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "TRACE_SUBJECT_DIGEST_MISMATCH")
        );
    }

    #[test]
    fn scitt_envelope_is_incomplete_until_cose_receipt_verifier_exists() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        value["attestations"][0]["envelope"] = json!({
            "format": "scitt_rfc9943_cose",
            "media_type": "application/scitt-statement+cose",
            "signed_statement_cose_base64": STANDARD.encode(b"opaque-cose-placeholder"),
            "statement_sha256": sha256_hex(b"opaque-cose-placeholder"),
            "receipts": []
        });
        // Recompute the external trust-policy reference after the bundle substitution is not needed:
        // the trust policy itself remains unchanged and the SCITT branch is expected to be incomplete.
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "INCOMPLETE");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "SCITT_COSE_VERIFICATION_UNIMPLEMENTED")
        );
    }

    #[test]
    fn scitt_receipt_metadata_mutation_fails_closed() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        value["attestations"][0]["envelope"] = json!({
            "format": "scitt_rfc9943_cose",
            "media_type": "application/scitt-statement+cose",
            "signed_statement_cose_base64": STANDARD.encode(b"statement"),
            "statement_sha256": sha256_hex(b"statement"),
            "receipts": [{
                "media_type": "application/scitt-receipt+cose",
                "ts_issuer": "test-ts",
                "ts_keyid": "test-ts-key",
                "receipt_cose_base64": STANDARD.encode(b"receipt"),
                "registration_policy_sha256": "invalid"
            }]
        });
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| { f.code == "SCITT_RECEIPT_METADATA_INVALID" })
        );
    }

    #[test]
    fn structurally_valid_scitt_receipt_still_requires_cryptographic_verification() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        value["attestations"][0]["envelope"] = json!({
            "format": "scitt_rfc9943_cose",
            "media_type": "application/scitt-statement+cose",
            "signed_statement_cose_base64": STANDARD.encode(b"statement"),
            "statement_sha256": sha256_hex(b"statement"),
            "receipts": [{
                "media_type": "application/scitt-receipt+cose",
                "ts_issuer": "test-ts",
                "ts_keyid": "test-ts-key",
                "receipt_cose_base64": STANDARD.encode(b"receipt"),
                "registration_policy_sha256": "a".repeat(64)
            }]
        });
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "INCOMPLETE");
        assert!(
            report
                .findings
                .iter()
                .any(|f| { f.code == "SCITT_COSE_VERIFICATION_UNIMPLEMENTED" })
        );
    }

    #[test]
    fn causal_self_edge_fails_closed() {
        let (bundle, trace, policy) = fixture();
        let mut value: Value = serde_json::from_slice(&bundle).unwrap();
        value["causal_edges"] = json!([{
            "from_event_id": "host-grant-observation-1",
            "to_event_id": "host-grant-observation-1",
            "relation": "program_order",
            "basis_attestation_ids": ["att-host-grant-1"]
        }]);
        let report = audit(&serde_json::to_vec(&value).unwrap(), &trace, &policy).unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(
            report
                .findings
                .iter()
                .any(|f| f.code == "CAUSAL_EDGE_SELF_REFERENCE")
        );
    }
    fn root_policy_fixture(policy_bytes: &[u8]) -> (Vec<u8>, Vec<u8>) {
        let policy: TrustPolicy = serde_json::from_slice(policy_bytes).unwrap();
        let root_signing_key = SigningKey::from_bytes(&[99u8; 32]);
        let root_public_key = root_signing_key.verifying_key();
        let root_public_key_hex = hex(root_public_key.as_bytes());
        let root_key_id = format!("sha256:{}", sha256_hex(root_public_key.as_bytes()));
        let signature = root_signing_key.sign(&pae(TRUST_POLICY_PAYLOAD_TYPE, policy_bytes));
        let envelope = json!({
            "payload_type": TRUST_POLICY_PAYLOAD_TYPE,
            "payload_base64": STANDARD.encode(policy_bytes),
            "signatures": [{
                "keyid": root_key_id,
                "sig_base64": STANDARD.encode(signature.to_bytes())
            }]
        });
        let pin = json!({
            "schema_version": 1,
            "root_key_id": root_key_id,
            "root_public_key_hex": root_public_key_hex,
            "policy_id": policy.policy_id,
            "policy_version": policy.policy_version,
            "policy_epoch": policy.policy_epoch,
            "policy_sha256": sha256_hex(policy_bytes),
            "root_keyset_id": policy.root_keyset_id
        });
        (
            serde_json::to_vec(&envelope).unwrap(),
            serde_json::to_vec(&pin).unwrap(),
        )
    }

    #[test]
    fn root_signature_authenticates_exact_policy_bytes_and_epoch() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let verified =
            verify_root_signed_trust_policy(&policy_bytes, &envelope, &pin).unwrap();
        let policy: TrustPolicy = serde_json::from_slice(&policy_bytes).unwrap();
        assert_eq!(verified.policy_sha256, sha256_hex(&policy_bytes));
        assert_eq!(verified.policy_epoch, 1);
        assert_eq!(verified.root_key_id, format!("sha256:{}", {
            let pin: TrustedRootPin = serde_json::from_slice(&pin).unwrap();
            sha256_hex(&decode_hex_32(&pin.root_public_key_hex).unwrap())
        }));
        assert_eq!(verified.policy_epoch, policy.policy_epoch);
    }

    #[test]
    fn root_signature_rejects_a_different_out_of_band_root() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut pin_value: Value = serde_json::from_slice(&pin).unwrap();
        let replacement_root = SigningKey::from_bytes(&[100u8; 32]).verifying_key();
        pin_value["root_public_key_hex"] = Value::String(hex(replacement_root.as_bytes()));
        pin_value["root_key_id"] =
            Value::String(format!("sha256:{}", sha256_hex(replacement_root.as_bytes())));
        let error = verify_root_signed_trust_policy(
            &policy_bytes,
            &envelope,
            &serde_json::to_vec(&pin_value).unwrap(),
        )
        .unwrap_err();
        assert!(error.contains("signer does not match"), "{error}");
    }

    #[test]
    fn root_signature_rejects_policy_digest_substitution() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut pin_value: Value = serde_json::from_slice(&pin).unwrap();
        pin_value["policy_sha256"] = Value::String("0".repeat(64));
        let error = verify_root_signed_trust_policy(
            &policy_bytes,
            &envelope,
            &serde_json::to_vec(&pin_value).unwrap(),
        )
        .unwrap_err();
        assert!(error.contains("out-of-band policy digest pin"), "{error}");
    }

    #[test]
    fn root_signature_rejects_an_unaccepted_policy_epoch() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut pin_value: Value = serde_json::from_slice(&pin).unwrap();
        pin_value["policy_epoch"] = Value::from(2u64);
        let error = verify_root_signed_trust_policy(
            &policy_bytes,
            &envelope,
            &serde_json::to_vec(&pin_value).unwrap(),
        )
        .unwrap_err();
        assert!(error.contains("identity/epoch"), "{error}");
    }

    #[test]
    fn root_signature_rejects_a_changed_payload_type() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut envelope_value: Value = serde_json::from_slice(&envelope).unwrap();
        envelope_value["payload_type"] = Value::String("application/json".into());
        let error = verify_root_signed_trust_policy(
            &policy_bytes,
            &serde_json::to_vec(&envelope_value).unwrap(),
            &pin,
        )
        .unwrap_err();
        assert!(error.contains("payload_type"), "{error}");
    }

    #[test]
    fn root_signature_rejects_signed_payload_byte_mismatch() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut envelope_value: Value = serde_json::from_slice(&envelope).unwrap();
        let changed_policy = [policy_bytes.as_slice(), b" "].concat();
        envelope_value["payload_base64"] = Value::String(STANDARD.encode(&changed_policy));
        let mut pin_value: Value = serde_json::from_slice(&pin).unwrap();
        pin_value["policy_sha256"] = Value::String(sha256_hex(&changed_policy));
        let error = verify_root_signed_trust_policy(
            &changed_policy,
            &serde_json::to_vec(&envelope_value).unwrap(),
            &serde_json::to_vec(&pin_value).unwrap(),
        )
        .unwrap_err();
        assert!(error.contains("byte-identical"), "{error}");
    }

    #[test]
    fn runtime_capture_cannot_succeed_relative_to_an_unpinned_policy() {
        let (bundle, trace, policy) = fixture();
        let mut bundle_value: Value = serde_json::from_slice(&bundle).unwrap();
        bundle_value["evidence_class"] = Value::String("runtime_capture".into());
        let runtime_bundle = serde_json::to_vec(&bundle_value).unwrap();

        let mut report = audit(&runtime_bundle, &trace, &policy).unwrap();
        assert_eq!(report.status, "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY");
        require_root_for_runtime_capture(&mut report);
        assert_eq!(report.status, "INCOMPLETE");
        assert!(report.findings.iter().any(|finding| {
            finding.code == "RUNTIME_CAPTURE_REQUIRES_PINNED_ROOT"
                && finding.severity == "incomplete"
        }));
    }

    #[test]
    fn synthetic_evidence_retains_explicit_relative_policy_mode() {
        let (bundle, trace, policy) = fixture();
        let mut report = audit(&bundle, &trace, &policy).unwrap();
        require_root_for_runtime_capture(&mut report);
        assert_eq!(
            report.status,
            "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY"
        );
        assert!(report.findings.iter().all(|finding| {
            finding.code != "RUNTIME_CAPTURE_REQUIRES_PINNED_ROOT"
        }));
    }

    #[test]
    fn pinned_root_rejects_non_semver_policy_versions() {
        let (_, _, policy_bytes) = fixture();
        let mut policy_value: Value = serde_json::from_slice(&policy_bytes).unwrap();
        policy_value["policy_version"] = Value::String("1.0".into());
        let changed_policy = serde_json::to_vec(&policy_value).unwrap();

        // A fresh pin and signature are generated over these exact bytes. The
        // invalid semantic version must be rejected independently of digest and
        // signature correctness.
        let (envelope, pin) = root_policy_fixture(&changed_policy);
        let error = verify_root_signed_trust_policy(&changed_policy, &envelope, &pin)
            .unwrap_err();
        assert!(error.contains("identity/epoch"), "{error}");
    }

    #[test]
    fn root_authenticated_audit_upgrades_the_claim_ceiling_only_after_both_layers_pass() {
        let (bundle, trace, policy) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy);
        let report = audit_with_root_pin(&bundle, &trace, &policy, &envelope, &pin).unwrap();
        assert_eq!(report.status, "ROOT_AUTHENTICATED_POLICY_SNAPSHOT");
        assert!(report.trust_policy_root_authenticated);
        assert_eq!(report.trust_policy_epoch, Some(1));
        assert!(report.trust_root_key_id.is_some());

        let mut changed_bundle: Value = serde_json::from_slice(&bundle).unwrap();
        changed_bundle["subject"]["source_commit"] = Value::String("f".repeat(40));
        let report = audit_with_root_pin(
            &serde_json::to_vec(&changed_bundle).unwrap(),
            &trace,
            &policy,
            &envelope,
            &pin,
        )
        .unwrap();
        assert_eq!(report.status, "FAIL");
        assert!(report.trust_policy_root_authenticated);
    }

    #[test]
    fn root_signature_rejects_a_mutated_signature_byte() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut envelope_value: Value = serde_json::from_slice(&envelope).unwrap();
        let signature = STANDARD
            .decode(envelope_value["signatures"][0]["sig_base64"].as_str().unwrap())
            .unwrap();
        let mut mutated = signature;
        mutated[0] ^= 0x80;
        envelope_value["signatures"][0]["sig_base64"] =
            Value::String(STANDARD.encode(mutated));
        let error = verify_root_signed_trust_policy(
            &policy_bytes,
            &serde_json::to_vec(&envelope_value).unwrap(),
            &pin,
        )
        .unwrap_err();
        assert!(error.contains("root signature verification failed"), "{error}");
    }

    #[test]
    fn root_pin_rejects_public_key_fingerprint_mismatch() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut pin_value: Value = serde_json::from_slice(&pin).unwrap();
        let replacement_root = SigningKey::from_bytes(&[101u8; 32]).verifying_key();
        // Keep the original key ID while changing only the public key bytes.
        pin_value["root_public_key_hex"] = Value::String(hex(replacement_root.as_bytes()));
        let error = verify_root_signed_trust_policy(
            &policy_bytes,
            &envelope,
            &serde_json::to_vec(&pin_value).unwrap(),
        )
        .unwrap_err();
        assert!(error.contains("fingerprint"), "{error}");
    }

    #[test]
    fn root_envelope_rejects_duplicate_root_signatures() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut envelope_value: Value = serde_json::from_slice(&envelope).unwrap();
        let first = envelope_value["signatures"][0].clone();
        envelope_value["signatures"].as_array_mut().unwrap().push(first);
        let error = verify_root_signed_trust_policy(
            &policy_bytes,
            &serde_json::to_vec(&envelope_value).unwrap(),
            &pin,
        )
        .unwrap_err();
        assert!(error.contains("exactly one"), "{error}");
    }

    #[test]
    fn root_pin_rejects_stale_policy_bytes_even_with_valid_old_signature() {
        let (_, _, policy_bytes) = fixture();
        let (envelope, pin) = root_policy_fixture(&policy_bytes);
        let mut stale_bytes = policy_bytes.clone();
        stale_bytes.push(b' ');
        let error =
            verify_root_signed_trust_policy(&stale_bytes, &envelope, &pin).unwrap_err();
        assert!(error.contains("out-of-band policy digest pin"), "{error}");
    }

}
