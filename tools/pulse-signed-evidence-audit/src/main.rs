use base64::{Engine as _, engine::general_purpose::STANDARD};
use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
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
    root_keyset_id: String,
    keys: Vec<TrustedKey>,
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
    bundle_id: String,
    bundle_sha256: String,
    trace_sha256: String,
    trust_policy_sha256: String,
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
                dsse_verified += 1;
                accepted_event_ids.insert(attestation.event_id.clone());
            }
            Envelope::Scitt {
                media_type,
                signed_statement_cose_base64,
                statement_sha256,
                receipts: _,
            } => {
                let bytes = match STANDARD.decode(signed_statement_cose_base64) {
                    Ok(v) => v,
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
                } else {
                    scitt_unverified += 1;
                    finding(
                        &mut findings,
                        "SCITT_COSE_VERIFICATION_UNIMPLEMENTED",
                        "incomplete",
                        Some(&attestation.attestation_id),
                        "COSE signature and receipt verification are not implemented by this first DSSE-only verifier",
                    );
                }
            }
        }
    }

    for edge in &bundle.causal_edges {
        if !event_ids.contains(&edge.from_event_id) || !event_ids.contains(&edge.to_event_id) {
            finding(
                &mut findings,
                "CAUSAL_EDGE_ENDPOINT_MISSING",
                "incomplete",
                None,
                "causal edge references an event without an attestation",
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

    let _ = accepted_event_ids;
    Ok(Report {
        schema_version: 1,
        status: status.to_string(),
        bundle_id: bundle.bundle_id,
        bundle_sha256,
        trace_sha256,
        trust_policy_sha256: policy_sha256,
        attestations: bundle.attestations.len(),
        dsse_verified,
        scitt_unverified,
        causal_edges: bundle.causal_edges.len(),
        findings,
        claim_ceiling: "DSSE_ROLE_AUTHENTICATION_RELATIVE_TO_SUPPLIED_POLICY_ONLY: the trust policy is an external input whose authenticity/currentness is not established here; SCITT COSE and transparency receipts are not yet verified; signatures do not prove asserted events are true or authorize actions".to_string(),
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut bundle = None;
    let mut trace = None;
    let mut policy = None;
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
            "--output" => {
                output = Some(PathBuf::from(
                    args.next().ok_or("--output requires a path")?,
                ))
            }
            "--help" | "-h" => {
                println!(
                    "pulse-signed-evidence-audit --bundle BUNDLE.json --trace TRACE.json --trust-policy TRUST.json [--output REPORT.json]"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}").into()),
        }
    }
    let bundle_path = bundle.ok_or("--bundle is required")?;
    let trace_path = trace.ok_or("--trace is required")?;
    let policy_path = policy.ok_or("--trust-policy is required")?;
    let bundle_bytes = fs::read(bundle_path)?;
    let trace_bytes = fs::read(trace_path)?;
    let policy_bytes = fs::read(policy_path)?;
    let report = audit(&bundle_bytes, &trace_bytes, &policy_bytes)?;
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
        "ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY" => Ok(()),
        "FAIL" => Err("signed evidence verification failed".into()),
        _ => Err("signed evidence verification incomplete".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    fn fixture() -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let source_commit = "a".repeat(40);
        let trace_bytes = serde_json::to_vec(&json!({
            "source_commit": source_commit,
            "trace_id": "synthetic-signed-trace-1"
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
                    "decision": "grant_issued",
                    "grant_action_hash": "uhCkk-test-grant",
                    "conductor_id": "owner-host",
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
}
