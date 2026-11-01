//! Versioned Rego input projection that turns unique role arrays into
//! boolean-keyed objects for indexed membership lookup.
use serde::Serialize;
use serde_json::{Map, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

use crate::shrink::parse_unique_json_bytes;

pub const ROLE_INDEX_PROFILE: &str = "mycelix-rego-role-index-v0.1";
pub const MAX_SOURCE_BYTES: u64 = 256 * 1024 * 1024;
pub const MAX_STRATIFIED_SAMPLES_PER_ACTION: usize = 24;
pub const MIN_STRATIFIED_SAMPLES_PER_ACTION: usize = 20;
const MAX_SAMPLES_PER_STRATUM: usize = 2;
const STRATIFIED_SAMPLE_PROFILE: &str = "mycelix-role-index-stratified-sample-v0.1";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RoleIndexError(pub String);

impl fmt::Display for RoleIndexError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for RoleIndexError {}

fn role_index_error(message: impl Into<String>) -> RoleIndexError {
    RoleIndexError(message.into())
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

/// Replace actor.roles with actor.role_index = { role: true, ... }.
/// Duplicate role strings are rejected rather than silently collapsed.
pub fn index_role_projection(input: &Value) -> Result<Value, RoleIndexError> {
    let mut output = input.clone();
    let actor = output
        .get_mut("actor")
        .and_then(Value::as_object_mut)
        .ok_or_else(|| role_index_error("Rego input.actor must be an object"))?;

    if actor.contains_key("role_index") {
        return Err(role_index_error("Rego input.actor already contains role_index"));
    }
    let roles = actor
        .remove("roles")
        .ok_or_else(|| role_index_error("Rego input.actor.roles is required"))?;
    let roles = roles
        .as_array()
        .ok_or_else(|| role_index_error("Rego input.actor.roles must be an array"))?;

    let mut seen = BTreeSet::new();
    let mut role_index = Map::new();
    for role in roles {
        let role = role
            .as_str()
            .filter(|role| !role.is_empty())
            .ok_or_else(|| role_index_error("each actor role must be a non-empty string"))?;
        if !seen.insert(role.to_owned()) {
            return Err(role_index_error(format!("duplicate actor role {role:?}")));
        }
        role_index.insert(role.to_owned(), Value::Bool(true));
    }
    actor.insert("role_index".to_owned(), Value::Object(role_index));
    Ok(output)
}

#[derive(Debug, Clone, Serialize)]
pub struct ActionInputDigest {
    pub action: String,
    pub case_id: String,
    pub sha256: String,
    pub byte_length: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct StratifiedActionInputDigest {
    pub action: String,
    pub sample_index: usize,
    pub case_id: String,
    pub stratum: String,
    pub sample_rank_sha256: String,
    pub baseline_path: String,
    pub indexed_path: String,
    pub baseline_sha256: String,
    pub indexed_sha256: String,
    pub baseline_byte_length: usize,
    pub indexed_byte_length: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct RoleIndexManifest {
    pub schema_version: String,
    pub projection_profile: String,
    pub source_engine_inputs_sha256: String,
    pub indexed_engine_inputs_sha256: String,
    pub source_case_count: usize,
    pub indexed_case_count: usize,
    pub action_count: usize,
    pub action_inputs: Vec<String>,
    pub action_input_digests: Vec<ActionInputDigest>,
    pub stratified_sample_profile: String,
    pub stratified_sample_count: usize,
    pub stratified_action_sample_counts: BTreeMap<String, usize>,
    pub stratified_action_inputs: Vec<StratifiedActionInputDigest>,
    pub claim_ceiling: String,
}


fn read_bounded_source<R: Read>(
    reader: R,
    maximum: u64,
    initial_capacity: usize,
) -> Result<Vec<u8>, RoleIndexError> {
    let mut bytes = Vec::with_capacity(initial_capacity);
    reader
        .take(maximum.saturating_add(1))
        .read_to_end(&mut bytes)
        .map_err(|error| role_index_error(format!("bounded-read source engine inputs: {error}")))?;
    if bytes.len() as u64 > maximum {
        return Err(role_index_error(format!("source engine inputs exceed {maximum} bytes")));
    }
    Ok(bytes)
}

#[derive(Debug, Clone)]
struct StratifiedCandidate {
    action: String,
    case_id: String,
    stratum: String,
    rank_sha256: String,
    baseline_bytes: Vec<u8>,
    indexed_bytes: Vec<u8>,
}

fn optional_string_field<'a>(
    object: &'a Map<String, Value>,
    field: &str,
    label: &str,
) -> Result<Option<&'a str>, RoleIndexError> {
    match object.get(field) {
        None => Ok(None),
        Some(Value::String(value)) => Ok(Some(value.as_str())),
        Some(_) => Err(role_index_error(format!(
            "{label}.{field} must be a string when present"
        ))),
    }
}

fn stratum_key(input: &Value) -> Result<String, RoleIndexError> {
    let actor = input
        .get("actor")
        .and_then(Value::as_object)
        .ok_or_else(|| role_index_error("Rego input.actor must be an object for stratification"))?;
    let actor_id = actor
        .get("id")
        .and_then(Value::as_str)
        .ok_or_else(|| role_index_error("Rego input.actor.id must be a string for stratification"))?;
    let roles = actor
        .get("roles")
        .and_then(Value::as_array)
        .ok_or_else(|| role_index_error("Rego input.actor.roles must be an array for stratification"))?;
    let role_bucket = match roles.len() {
        0 => "0",
        1..=2 => "1-2",
        3..=4 => "3-4",
        _ => "5+",
    };

    let claim = input
        .get("claim")
        .and_then(Value::as_object)
        .ok_or_else(|| role_index_error("Rego input.claim must be an object for stratification"))?;
    let claim_exists = claim
        .get("exists")
        .and_then(Value::as_bool)
        .ok_or_else(|| role_index_error("Rego input.claim.exists must be boolean for stratification"))?;
    let creator_actor = optional_string_field(claim, "creatorActorId", "Rego input.claim")?;
    let owner_relation = if !claim_exists || creator_actor.is_none() {
        "not_applicable"
    } else if creator_actor == Some(actor_id) {
        "owner"
    } else {
        "non_owner"
    };

    let creator_org = optional_string_field(claim, "creatorOrganizationId", "Rego input.claim")?;
    let input_object = input
        .as_object()
        .ok_or_else(|| role_index_error("Rego input must be an object for stratification"))?;
    let acting_org = optional_string_field(input_object, "actingOrganizationId", "Rego input")?;
    let organization_relation = match (acting_org, creator_org) {
        (None, _) => "absent",
        (Some(_), None) => "orphan",
        (Some(acting), Some(creator)) if acting == creator => "same",
        (Some(_), Some(_)) => "different",
    };

    let attestation_kind = input
        .pointer("/attestation/kind")
        .and_then(Value::as_str)
        .ok_or_else(|| role_index_error("Rego input.attestation.kind must be a string for stratification"))?;
    let attestation_bucket = match attestation_kind {
        "ordinary" => "ordinary",
        "independent_replication" => "independent_replication",
        _ => "other",
    };

    let target = input
        .get("targetAttestation")
        .and_then(Value::as_object)
        .ok_or_else(|| role_index_error("Rego input.targetAttestation must be an object for stratification"))?;
    let target_exists = target
        .get("exists")
        .and_then(Value::as_bool)
        .ok_or_else(|| role_index_error("Rego input.targetAttestation.exists must be boolean for stratification"))?;
    let target_author = optional_string_field(target, "authorActorId", "Rego input.targetAttestation")?;
    if target_exists && target_author.is_none() {
        return Err(role_index_error(
            "Rego input.targetAttestation.authorActorId is required when the target attestation exists",
        ));
    }
    let target_relation = if !target_exists {
        "absent"
    } else if target_author == Some(actor_id) {
        "self"
    } else {
        "other"
    };

    Ok(format!(
        "roles={role_bucket};claim={};owner={owner_relation};org={organization_relation};attestation={attestation_bucket};target={target_relation}",
        if claim_exists { "present" } else { "absent" }
    ))
}

fn sample_rank(action: &str, case_id: &str) -> String {
    sha256(
        format!("{STRATIFIED_SAMPLE_PROFILE}\n{action}\n{case_id}").as_bytes(),
    )
}

fn stratum_rank(action: &str, stratum: &str) -> String {
    sha256(
        format!("{STRATIFIED_SAMPLE_PROFILE}\n{action}\n{stratum}").as_bytes(),
    )
}

fn insert_stratified_candidate(
    buckets: &mut BTreeMap<(String, String), Vec<StratifiedCandidate>>,
    candidate: StratifiedCandidate,
) {
    let key = (candidate.action.clone(), candidate.stratum.clone());
    let bucket = buckets.entry(key).or_default();
    bucket.push(candidate);
    bucket.sort_by(|left, right| left.rank_sha256.cmp(&right.rank_sha256));
    bucket.truncate(MAX_SAMPLES_PER_STRATUM);
}

fn select_stratified_candidates(
    buckets: &BTreeMap<(String, String), Vec<StratifiedCandidate>>,
    actions: &[&str],
) -> Result<(Vec<StratifiedCandidate>, BTreeMap<String, usize>), RoleIndexError> {
    let mut selected = Vec::new();
    let mut counts = BTreeMap::new();

    for action in actions {
        let mut strata = buckets
            .iter()
            .filter(|((candidate_action, _), _)| candidate_action == action)
            .map(|((_, stratum), candidates)| {
                (
                    stratum_rank(action, stratum),
                    candidates,
                )
            })
            .collect::<Vec<_>>();
        strata.sort_by(|left, right| left.0.cmp(&right.0));

        let action_start = selected.len();
        for round in 0..MAX_SAMPLES_PER_STRATUM {
            for (_, candidates) in &strata {
                if selected.len() - action_start >= MAX_STRATIFIED_SAMPLES_PER_ACTION {
                    break;
                }
                if let Some(candidate) = candidates.get(round) {
                    selected.push(candidate.clone());
                }
            }
            if selected.len() - action_start >= MAX_STRATIFIED_SAMPLES_PER_ACTION {
                break;
            }
        }

        let count = selected.len() - action_start;
        if count < MIN_STRATIFIED_SAMPLES_PER_ACTION {
            return Err(role_index_error(format!(
                "action {action} produced only {count} stratified samples; at least {MIN_STRATIFIED_SAMPLES_PER_ACTION} are required"
            )));
        }
        counts.insert((*action).to_owned(), count);
    }

    Ok((selected, counts))
}

fn write_stratified_samples(
    output_directory: &Path,
    buckets: &BTreeMap<(String, String), Vec<StratifiedCandidate>>,
    actions: &[&str],
) -> Result<(Vec<StratifiedActionInputDigest>, BTreeMap<String, usize>), RoleIndexError> {
    let (selected, counts) = select_stratified_candidates(buckets, actions)?;
    let mut manifest_rows = Vec::with_capacity(selected.len());
    let mut per_action_index = BTreeMap::<String, usize>::new();

    for candidate in selected {
        let sample_index = per_action_index
            .entry(candidate.action.clone())
            .and_modify(|index| *index += 1)
            .or_insert(1);
        let baseline_path = format!(
            "stratified-inputs/baseline/{}/sample-{:02}.json",
            candidate.action, *sample_index
        );
        let indexed_path = format!(
            "stratified-inputs/indexed/{}/sample-{:02}.json",
            candidate.action, *sample_index
        );
        for relative_path in [&baseline_path, &indexed_path] {
            let path = output_directory.join(relative_path);
            let parent = path
                .parent()
                .ok_or_else(|| role_index_error("stratified sample path has no parent"))?;
            fs::create_dir_all(parent).map_err(|error| {
                role_index_error(format!("create stratified sample directory: {error}"))
            })?;
        }
        fs::write(output_directory.join(&baseline_path), &candidate.baseline_bytes)
            .map_err(|error| role_index_error(format!("write baseline stratified sample: {error}")))?;
        fs::write(output_directory.join(&indexed_path), &candidate.indexed_bytes)
            .map_err(|error| role_index_error(format!("write indexed stratified sample: {error}")))?;

        manifest_rows.push(StratifiedActionInputDigest {
            action: candidate.action,
            sample_index: *sample_index,
            case_id: candidate.case_id,
            stratum: candidate.stratum,
            sample_rank_sha256: candidate.rank_sha256,
            baseline_path,
            indexed_path,
            baseline_sha256: sha256(&candidate.baseline_bytes),
            indexed_sha256: sha256(&candidate.indexed_bytes),
            baseline_byte_length: candidate.baseline_bytes.len(),
            indexed_byte_length: candidate.indexed_bytes.len(),
        });
    }

    Ok((manifest_rows, counts))
}


/// Read the exact source-pinned paired engine-input JSONL, create one
/// indexed Rego input per case, and export the first source-bound representative
/// input for each of the eight policy actions.
pub fn generate_indexed_role_corpus(
    source_jsonl_path: &Path,
    expected_source_sha256: &str,
    output_directory: &Path,
) -> Result<RoleIndexManifest, RoleIndexError> {
    let source_file = File::open(source_jsonl_path)
        .map_err(|error| role_index_error(format!("open source engine inputs: {error}")))?;
    let metadata = source_file
        .metadata()
        .map_err(|error| role_index_error(format!("inspect opened source engine inputs: {error}")))?;
    if !metadata.is_file() {
        return Err(role_index_error("source engine inputs must be a regular file"));
    }
    if metadata.len() > MAX_SOURCE_BYTES {
        return Err(role_index_error(format!("source engine inputs exceed {MAX_SOURCE_BYTES} bytes")));
    }
    let initial_capacity = usize::try_from(metadata.len().min(MAX_SOURCE_BYTES))
        .map_err(|error| role_index_error(format!("source engine input size does not fit memory index: {error}")))?;
    let source_bytes = read_bounded_source(
        source_file,
        MAX_SOURCE_BYTES,
        initial_capacity,
    )?;
    let source_sha256 = sha256(&source_bytes);
    if source_sha256 != expected_source_sha256 {
        return Err(role_index_error("source engine input SHA-256 differs from its pinned manifest"));
    }

    fs::create_dir_all(output_directory)
        .map_err(|error| role_index_error(format!("create indexed-role output directory: {error}")))?;
    let action_directory = output_directory.join("action-inputs");
    fs::create_dir_all(&action_directory)
        .map_err(|error| role_index_error(format!("create action-input directory: {error}")))?;

    let output_path = output_directory.join("indexed-engine-inputs-v0.1.jsonl");
    let file = File::create(&output_path)
        .map_err(|error| role_index_error(format!("create indexed engine input JSONL: {error}")))?;
    let mut writer = BufWriter::new(file);
    let mut output_hasher = Sha256::new();
    let mut case_ids = BTreeSet::new();
    let expected_actions = [
        "claim_proposed",
        "legacy_claim_imported",
        "evidence_attached",
        "attestation_recorded",
        "attestation_corrected",
        "attestation_withdrawn",
        "claim_superseded",
        "claim_retracted",
    ];
    let mut action_inputs = BTreeSet::new();
    let mut action_input_digests = Vec::with_capacity(expected_actions.len());
    let mut stratified_buckets = BTreeMap::<(String, String), Vec<StratifiedCandidate>>::new();
    let mut action_count = 0usize;
    let mut case_count = 0usize;

    for (line_index, line) in BufReader::new(source_bytes.as_slice()).lines().enumerate() {
        let line = line
            .map_err(|error| role_index_error(format!("read source JSONL line {}: {error}", line_index + 1)))?;
        if line.trim().is_empty() {
            return Err(role_index_error(format!("blank source JSONL record at line {}", line_index + 1)));
        }
        let row = parse_unique_json_bytes(line.as_bytes())
            .map_err(|error| role_index_error(format!("strictly parse source JSONL line {}: {error}", line_index + 1)))?;
        let case_id = row
            .get("case_id")
            .and_then(Value::as_str)
            .filter(|value| !value.is_empty())
            .ok_or_else(|| role_index_error(format!("source line {} has no case_id", line_index + 1)))?;
        if !case_ids.insert(case_id.to_owned()) {
            return Err(role_index_error(format!("duplicate source case ID {case_id:?}")));
        }
        let input = row
            .pointer("/rego/input")
            .ok_or_else(|| role_index_error(format!("source case {case_id} has no rego.input")))?;
        let indexed = index_role_projection(input)?;
        let action = input
            .get("action")
            .and_then(Value::as_str)
            .ok_or_else(|| role_index_error(format!("source case {case_id} has no action")))?;

        if expected_actions.contains(&action) {
            if !action_inputs.contains(&action) {
                action_inputs.insert(action.to_owned());
                let action_path = action_directory.join(format!("{action}.json"));
                let bytes = serde_json::to_vec_pretty(&indexed)
                    .map_err(|error| role_index_error(format!("serialize indexed action input: {error}")))?;
                fs::write(&action_path, &bytes)
                    .map_err(|error| role_index_error(format!("write indexed action input: {error}")))?;
                action_input_digests.push(ActionInputDigest {
                    action: action.to_owned(),
                    case_id: case_id.to_owned(),
                    sha256: sha256(&bytes),
                    byte_length: bytes.len(),
                });
                action_count += 1;
            }

            let stratum = stratum_key(input)?;
            let rank_sha256 = sample_rank(action, case_id);
            let baseline_bytes = serde_json::to_vec_pretty(input)
                .map_err(|error| role_index_error(format!("serialize baseline stratified input: {error}")))?;
            let indexed_bytes = serde_json::to_vec_pretty(&indexed)
                .map_err(|error| role_index_error(format!("serialize indexed stratified input: {error}")))?;
            insert_stratified_candidate(
                &mut stratified_buckets,
                StratifiedCandidate {
                    action: action.to_owned(),
                    case_id: case_id.to_owned(),
                    stratum,
                    rank_sha256,
                    baseline_bytes,
                    indexed_bytes,
                },
            );
        }

        let mut output_row = serde_json::to_vec(&serde_json::json!({
            "case_id": case_id,
            "input": indexed
        }))
        .map_err(|error| role_index_error(format!("serialize indexed case row: {error}")))?;
        output_row.push(b'\n');
        writer
            .write_all(&output_row)
            .map_err(|error| role_index_error(format!("write indexed case row: {error}")))?;
        output_hasher.update(&output_row);
        case_count += 1;
    }
    writer
        .flush()
        .map_err(|error| role_index_error(format!("flush indexed case rows: {error}")))?;

    if case_count != 62_464 {
        return Err(role_index_error(format!("expected 62464 indexed cases, got {case_count}")));
    }
    if action_count != expected_actions.len() {
        return Err(role_index_error(format!(
            "expected one indexed representative for all eight actions, got {action_count}"
        )));
    }

    let (stratified_action_inputs, stratified_action_sample_counts) =
        write_stratified_samples(output_directory, &stratified_buckets, &expected_actions)?;
    let stratified_sample_count = stratified_action_inputs.len();

    let manifest = RoleIndexManifest {
        schema_version: "mycelix-indexed-role-corpus-v0.1".to_owned(),
        projection_profile: ROLE_INDEX_PROFILE.to_owned(),
        source_engine_inputs_sha256: source_sha256,
        indexed_engine_inputs_sha256: hex::encode(output_hasher.finalize()),
        source_case_count: case_count,
        indexed_case_count: case_count,
        action_count,
        action_inputs: action_inputs.into_iter().collect(),
        action_input_digests,
        stratified_sample_profile: STRATIFIED_SAMPLE_PROFILE.to_owned(),
        stratified_sample_count,
        stratified_action_sample_counts,
        stratified_action_inputs,
        claim_ceiling: concat!(
            "deterministic role-array to boolean-keyed-object input projection over the frozen paired corpus; ",
            "no policy decision, Cedar result, or authentic Rust authorization parity is inferred"
        ).to_owned(),
    };
    let manifest_bytes = serde_json::to_vec_pretty(&manifest)
        .map_err(|error| role_index_error(format!("serialize indexed-role manifest: {error}")))?;
    fs::write(output_directory.join("manifest.json"), manifest_bytes)
        .map_err(|error| role_index_error(format!("write indexed-role manifest: {error}")))?;
    Ok(manifest)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_source_read_accepts_exact_limit_and_rejects_one_byte_over() {
        let at_limit = read_bounded_source(
            std::io::Cursor::new(b"1234".as_slice()),
            4,
            4,
        )
        .expect("exactly-at-limit input is accepted");
        assert_eq!(at_limit, b"1234");

        let over_limit = read_bounded_source(
            std::io::Cursor::new(b"12345".as_slice()),
            4,
            4,
        );
        assert!(over_limit.is_err(), "one byte over the cap must be rejected");
    }

    #[test]
    fn stratification_covers_declared_semantic_dimensions() {
        let base = serde_json::json!({
            "action":"attestation_recorded",
            "actor":{"id":"actor:a","roles":["reviewer"]},
            "claim":{"exists":true,"creatorActorId":"actor:b","creatorOrganizationId":"org:a"},
            "attestation":{"kind":"ordinary"},
            "targetAttestation":{"exists":true,"authorActorId":"actor:a"},
            "actingOrganizationId":"org:a"
        });
        let base_key = stratum_key(&base).expect("base input has valid strata");
        assert!(base_key.contains("roles=1-2"));
        assert!(base_key.contains("owner=non_owner"));
        assert!(base_key.contains("org=same"));
        assert!(base_key.contains("attestation=ordinary"));
        assert!(base_key.contains("target=self"));

        let changed = serde_json::json!({
            "action":"attestation_recorded",
            "actor":{"id":"actor:a","roles":["reviewer","editor","institution","contributor","service"]},
            "claim":{"exists":true,"creatorActorId":"actor:a"},
            "attestation":{"kind":"independent_replication"},
            "targetAttestation":{"exists":false}
        });
        let changed_key = stratum_key(&changed).expect("changed input has valid strata");
        assert_ne!(base_key, changed_key);
        assert!(changed_key.contains("roles=5+"));
        assert!(changed_key.contains("owner=owner"));
        assert!(changed_key.contains("org=absent"));
        assert!(changed_key.contains("attestation=independent_replication"));
        assert!(changed_key.contains("target=absent"));
    }

    #[test]
    fn optional_stratification_fields_reject_non_string_values() {
        let input = serde_json::json!({
            "action":"evidence_attached",
            "actor":{"id":"actor:a","roles":[]},
            "claim":{"exists":true,"creatorActorId":17},
            "attestation":{"kind":"ordinary"},
            "targetAttestation":{"exists":false}
        });
        assert!(stratum_key(&input).is_err());
    }

    #[test]
    fn stratified_selector_round_robins_hash_ranked_strata() {
        let mut buckets = BTreeMap::new();
        for stratum_index in 0..32 {
            for candidate_index in 0..2 {
                let case_id = format!("case-{stratum_index:02}-{candidate_index}");
                insert_stratified_candidate(
                    &mut buckets,
                    StratifiedCandidate {
                        action: "claim_proposed".to_owned(),
                        case_id: case_id.clone(),
                        stratum: format!("stratum-{stratum_index:02}"),
                        rank_sha256: sample_rank("claim_proposed", &case_id),
                        baseline_bytes: case_id.as_bytes().to_vec(),
                        indexed_bytes: case_id.as_bytes().to_vec(),
                    },
                );
            }
        }
        let (selected, counts) = select_stratified_candidates(&buckets, &["claim_proposed"])
            .expect("enough semantic strata should meet the minimum sample count");
        assert_eq!(counts["claim_proposed"], MAX_STRATIFIED_SAMPLES_PER_ACTION);
        assert_eq!(selected.len(), MAX_STRATIFIED_SAMPLES_PER_ACTION);
        let distinct_strata = selected
            .iter()
            .map(|candidate| candidate.stratum.as_str())
            .collect::<BTreeSet<_>>();
        assert_eq!(distinct_strata.len(), MAX_STRATIFIED_SAMPLES_PER_ACTION);
    }

    #[test]
    fn stratified_candidate_retention_is_hash_ranked_and_bounded() {
        let mut buckets = BTreeMap::new();
        for case_id in ["case-z", "case-a", "case-m"] {
            let rank = sample_rank("claim_proposed", case_id);
            insert_stratified_candidate(
                &mut buckets,
                StratifiedCandidate {
                    action: "claim_proposed".to_owned(),
                    case_id: case_id.to_owned(),
                    stratum: "roles=0;claim=absent".to_owned(),
                    rank_sha256: rank,
                    baseline_bytes: case_id.as_bytes().to_vec(),
                    indexed_bytes: case_id.as_bytes().to_vec(),
                },
            );
        }
        let kept = buckets
            .get(&("claim_proposed".to_owned(), "roles=0;claim=absent".to_owned()))
            .expect("stratum exists");
        assert_eq!(kept.len(), MAX_SAMPLES_PER_STRATUM);
        assert!(kept[0].rank_sha256 <= kept[1].rank_sha256);
        assert_ne!(kept[0].case_id, "case-m");
    }

    #[test]
    fn role_projection_is_deterministic_and_preserves_other_actor_fields() {
        let input = serde_json::json!({
            "action": "claim_proposed",
            "actor": {
                "id": "actor:1",
                "roles": ["editor", "contributor"],
                "organizationIds": ["org:1"]
            },
            "claim": {"exists": false}
        });
        let indexed = index_role_projection(&input).unwrap();
        assert!(indexed["actor"].get("roles").is_none());
        assert_eq!(indexed["actor"]["role_index"]["editor"], true);
        assert_eq!(indexed["actor"]["role_index"]["contributor"], true);
        assert_eq!(indexed["actor"]["id"], "actor:1");
        assert_eq!(indexed["actor"]["organizationIds"][0], "org:1");
    }

    #[test]
    fn role_projection_is_independent_of_source_array_order() {
        let first = serde_json::json!({
            "action": "attestation_recorded",
            "actor": {"id": "actor:1", "roles": ["reviewer", "contributor"]}
        });
        let second = serde_json::json!({
            "action": "attestation_recorded",
            "actor": {"id": "actor:1", "roles": ["contributor", "reviewer"]}
        });
        let first_indexed = index_role_projection(&first).unwrap();
        let second_indexed = index_role_projection(&second).unwrap();
        assert_eq!(first_indexed, second_indexed);
        assert_eq!(
            serde_json::to_vec(&first_indexed).unwrap(),
            serde_json::to_vec(&second_indexed).unwrap()
        );
    }

    #[test]
    fn role_projection_accepts_empty_set_and_rejects_duplicate_or_invalid_roles() {
        let empty = serde_json::json!({"actor":{"roles":[]}});
        assert_eq!(
            index_role_projection(&empty).unwrap()["actor"]["role_index"],
            serde_json::json!({})
        );

        for roles in [
            serde_json::json!(["contributor", "contributor"]),
            serde_json::json!(["contributor", 7]),
            serde_json::json!([""]),
        ] {
            let input = serde_json::json!({"actor":{"roles":roles}});
            assert!(index_role_projection(&input).is_err());
        }
    }

    #[test]
    fn role_projection_rejects_preexisting_index_to_prevent_ambiguous_dual_source() {
        let input = serde_json::json!({
            "actor": {
                "roles": ["contributor"],
                "role_index": {"editor": true}
            }
        });
        assert!(index_role_projection(&input).is_err());
    }
}
