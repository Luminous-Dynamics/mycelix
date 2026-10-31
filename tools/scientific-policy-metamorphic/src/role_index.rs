//! Versioned Rego input projection that turns unique role arrays into
//! boolean-keyed objects for indexed membership lookup.
use serde::Serialize;
use serde_json::{Map, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

use crate::shrink::parse_unique_json_bytes;

pub const ROLE_INDEX_PROFILE: &str = "mycelix-rego-role-index-v0.1";
pub const MAX_SOURCE_BYTES: u64 = 256 * 1024 * 1024;

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
    pub claim_ceiling: String,
}

/// Read the exact source-pinned paired engine-input JSONL, create one
/// indexed Rego input per case, and export the first source-bound representative
/// input for each of the eight policy actions.
pub fn generate_indexed_role_corpus(
    source_jsonl_path: &Path,
    expected_source_sha256: &str,
    output_directory: &Path,
) -> Result<RoleIndexManifest, RoleIndexError> {
    let metadata = fs::metadata(source_jsonl_path)
        .map_err(|error| role_index_error(format!("stat source engine inputs: {error}")))?;
    if !metadata.file_type().is_file() {
        return Err(role_index_error("source engine inputs must be a regular file"));
    }
    if metadata.len() > MAX_SOURCE_BYTES {
        return Err(role_index_error(format!("source engine inputs exceed {MAX_SOURCE_BYTES} bytes")));
    }
    let initial_capacity = usize::try_from(metadata.len().min(MAX_SOURCE_BYTES))
        .map_err(|error| role_index_error(format!("source engine input size does not fit memory index: {error}")))?;
    let mut source_bytes = Vec::with_capacity(initial_capacity);
    File::open(source_jsonl_path)
        .map_err(|error| role_index_error(format!("open source engine inputs: {error}")))?
        .take(MAX_SOURCE_BYTES + 1)
        .read_to_end(&mut source_bytes)
        .map_err(|error| role_index_error(format!("bounded-read source engine inputs: {error}")))?;
    // Metadata is an early rejection only. The read cap and post-read check
    // protect against a source file that grows or is replaced after stat.
    if source_bytes.len() as u64 > MAX_SOURCE_BYTES {
        return Err(role_index_error(format!("source engine inputs exceed {MAX_SOURCE_BYTES} bytes")));
    }
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

        if expected_actions.contains(&action) && action_inputs.insert(action.to_owned()) {
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
