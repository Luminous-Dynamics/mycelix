//! Deterministic metamorphic variants for the source-pinned Cedar/OPA corpus.
//!
//! This tool does not predict decisions. It creates explicitly classified
//! transformations and binds each transformed corpus to its exact source bytes.

use serde::Serialize;
use serde_json::{Map, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

pub const TOOL_ID: &str = "mycelix-scientific-policy-metamorphic";
pub const TOOL_VERSION: &str = env!("CARGO_PKG_VERSION");
pub const EXPECTED_CASE_COUNT: usize = 62_464;
pub const MAX_CORPUS_BYTES: u64 = 256 * 1024 * 1024;
pub const IRRELEVANT_ACTOR_ID: &str = "actor:metamorphic-irrelevant";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MetamorphicError(pub String);

impl fmt::Display for MetamorphicError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for MetamorphicError {}

fn error(message: impl Into<String>) -> MetamorphicError {
    MetamorphicError(message.into())
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TransformId {
    JsonObjectKeyOrderReversed,
    JsonWhitespaceExpanded,
    CedarEntityOrderReversed,
    CedarIrrelevantActorAdded,
    ClaimProposedContributorRemoved,
}

impl TransformId {
    pub const ALL: [Self; 5] = [
        Self::JsonObjectKeyOrderReversed,
        Self::JsonWhitespaceExpanded,
        Self::CedarEntityOrderReversed,
        Self::CedarIrrelevantActorAdded,
        Self::ClaimProposedContributorRemoved,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::JsonObjectKeyOrderReversed => "json-object-key-order-reversed",
            Self::JsonWhitespaceExpanded => "json-whitespace-expanded",
            Self::CedarEntityOrderReversed => "cedar-entity-order-reversed",
            Self::CedarIrrelevantActorAdded => "cedar-irrelevant-actor-added",
            Self::ClaimProposedContributorRemoved => "claim-proposed-contributor-removed",
        }
    }

    pub const fn relation(self) -> &'static str {
        match self {
            Self::JsonObjectKeyOrderReversed
            | Self::JsonWhitespaceExpanded
            | Self::CedarEntityOrderReversed => "decision_invariant",
            Self::CedarIrrelevantActorAdded => {
                "decision_invariant_when_new_entity_uid_is_unreferenced"
            }
            Self::ClaimProposedContributorRemoved => {
                "one_targeted_claim_proposed_case_changes_from_allow_to_deny"
            }
        }
    }

    pub const fn precondition(self) -> &'static str {
        match self {
            Self::JsonObjectKeyOrderReversed => {
                "all records parse as JSON values; only object member serialization order changes"
            }
            Self::JsonWhitespaceExpanded => {
                "all records parse as JSON values; only insignificant JSON whitespace changes"
            }
            Self::CedarEntityOrderReversed => {
                "Cedar entity UIDs are unique; entity set is permuted without changing membership"
            }
            Self::CedarIrrelevantActorAdded => {
                "the inserted Actor UID is absent from request principal/action/resource and existing entities; the new entity has only schema-defined attributes and no parents"
            }
            Self::ClaimProposedContributorRemoved => {
                "exactly one source case has action claim_proposed, absent claim, and exactly the contributor role; remove that role in Cedar and Rego while preserving all other fields and case identity"
            }
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct VariantManifest {
    pub schema_version: String,
    pub tool_id: String,
    pub tool_version: String,
    pub transform_id: TransformId,
    pub expected_relation: String,
    pub source_engine_inputs_sha256: String,
    pub transformed_engine_inputs_sha256: String,
    pub source_case_count: usize,
    pub transformed_case_count: usize,
    pub expected_changed_case_count: usize,
    pub changed_case_count: usize,
    pub changed_case_ids: Vec<String>,
    pub baseline_revision: String,
    pub baseline_source_blob_sha: String,
    pub precondition: String,
    pub claim_ceiling: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct VariantSummary {
    pub transform_id: TransformId,
    pub output_directory: String,
    pub source_sha256: String,
    pub transformed_sha256: String,
    pub case_count: usize,
}

fn required_string<'a>(value: &'a Value, key: &str) -> Result<&'a str, MetamorphicError> {
    value
        .get(key)
        .and_then(Value::as_str)
        .filter(|value| !value.is_empty())
        .ok_or_else(|| error(format!("manifest field {key} must be a non-empty string")))
}

fn required_usize(value: &Value, key: &str) -> Result<usize, MetamorphicError> {
    value
        .get(key)
        .and_then(Value::as_u64)
        .and_then(|value| usize::try_from(value).ok())
        .ok_or_else(|| error(format!("manifest field {key} must be a non-negative integer")))
}

fn validate_generator_manifest(manifest: &Value) -> Result<(), MetamorphicError> {
    if required_usize(manifest, "case_count")? != EXPECTED_CASE_COUNT
        || required_usize(manifest, "engine_input_count")? != EXPECTED_CASE_COUNT
        || required_string(manifest, "expected_decision_status")?
            != "pending_authentic_rust_oracle"
        || required_string(manifest, "baseline_revision")?
            != "f89b4bcdcfb901961e2ca11a577775acbf53a74d"
        || required_string(manifest, "baseline_source_blob_sha")?
            != "dd9ea9267cdae2e2c67244e8a90821defaacefb3"
        || required_string(manifest, "corpus_file")?
            != "scientific-policy-decision-space-v0.1.jsonl"
        || required_string(manifest, "engine_inputs_file")?
            != "scientific-policy-engine-inputs-v0.1.jsonl"
        || required_string(manifest, "engine_input_format")?
            != "case_id+cedar_native_request+rego_input"
        || required_string(manifest, "encoding")? != "utf-8-jsonl-one-record-per-line"
    {
        return Err(error("generator manifest does not match the frozen v0.1 contract"));
    }

    for key in ["corpus_sha256", "engine_inputs_sha256"] {
        let digest = required_string(manifest, key)?;
        if digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
            return Err(error(format!("manifest field {key} is not a SHA-256 hex digest")));
        }
    }
    Ok(())
}

fn validate_case_shape(
    case: &Value,
    allow_irrelevant_actor: bool,
) -> Result<(), MetamorphicError> {
    let case_id = required_string(case, "case_id")?;
    let cedar = case
        .get("cedar")
        .and_then(Value::as_object)
        .ok_or_else(|| error("case.cedar must be an object"))?;
    let rego = case
        .get("rego")
        .and_then(Value::as_object)
        .ok_or_else(|| error("case.rego must be an object"))?;
    if cedar.get("case_id").and_then(Value::as_str) != Some(case_id)
        || rego.get("case_id").and_then(Value::as_str) != Some(case_id)
    {
        return Err(error("top-level, Cedar and Rego case IDs must agree"));
    }
    let request = cedar
        .get("request")
        .and_then(Value::as_object)
        .ok_or_else(|| error("Cedar request must be an object"))?;
    for field in ["principal", "action", "resource"] {
        required_string_value(request, field)?;
    }
    if !request.get("context").is_some_and(Value::is_object) {
        return Err(error("Cedar request context must be an object"));
    }
    if !rego.get("input").is_some_and(Value::is_object) {
        return Err(error("Rego input must be an object"));
    }

    let entities = cedar
        .get("entities")
        .and_then(Value::as_array)
        .ok_or_else(|| error("Cedar entities must be an array"))?;
    let expected_entity_count = if allow_irrelevant_actor { 3 } else { 2 };
    if entities.len() != expected_entity_count {
        return Err(error(format!(
            "expected {expected_entity_count} explicit Cedar entities for this transform, found {}",
            entities.len()
        )));
    }
    let mut uids = BTreeSet::new();
    for entity in entities {
        let uid = entity
            .get("uid")
            .ok_or_else(|| error("Cedar entity is missing uid"))?;
        let uid_bytes = serde_json::to_vec(uid)
            .map_err(|serialize_error| error(format!("serialize Cedar UID: {serialize_error}")))?;
        if !uids.insert(uid_bytes) {
            return Err(error("Cedar entity UIDs must be unique before transformation"));
        }
        if !entity.get("attrs").is_some_and(Value::is_object)
            || !entity.get("parents").is_some_and(Value::is_array)
        {
            return Err(error("Cedar entity requires object attrs and array parents"));
        }
    }
    Ok(())
}

fn required_string_value<'a>(
    object: &'a Map<String, Value>,
    key: &str,
) -> Result<&'a str, MetamorphicError> {
    object
        .get(key)
        .and_then(Value::as_str)
        .filter(|value| !value.is_empty())
        .ok_or_else(|| error(format!("Cedar request field {key} must be a non-empty string")))
}

fn reverse_object_member_order(value: &Value, output: &mut String) -> Result<(), MetamorphicError> {
    match value {
        Value::Null => output.push_str("null"),
        Value::Bool(value) => output.push_str(if *value { "true" } else { "false" }),
        Value::Number(value) => output.push_str(&value.to_string()),
        Value::String(value) => {
            output.push_str(
                &serde_json::to_string(value)
                    .map_err(|serialize_error| error(format!("serialize JSON string: {serialize_error}")))?,
            );
        }
        Value::Array(values) => {
            output.push('[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                reverse_object_member_order(value, output)?;
            }
            output.push(']');
        }
        Value::Object(object) => {
            output.push('{');
            for (index, (key, value)) in object.iter().rev().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                output.push_str(
                    &serde_json::to_string(key)
                        .map_err(|serialize_error| error(format!("serialize JSON key: {serialize_error}")))?,
                );
                output.push(':');
                reverse_object_member_order(value, output)?;
            }
            output.push('}');
        }
    }
    Ok(())
}

fn append_irrelevant_actor(case: &mut Value) -> Result<(), MetamorphicError> {
    let cedar = case
        .get_mut("cedar")
        .and_then(Value::as_object_mut)
        .ok_or_else(|| error("Cedar projection must be an object"))?;
    let request = cedar
        .get("request")
        .and_then(Value::as_object)
        .ok_or_else(|| error("Cedar request must be an object"))?;
    let principal = required_string_value(request, "principal")?.to_owned();
    let action = required_string_value(request, "action")?.to_owned();
    let resource = required_string_value(request, "resource")?.to_owned();
    let irrelevant_uid = format!(
        "Mycelix::Actor::{}",
        serde_json::to_string(IRRELEVANT_ACTOR_ID)
            .map_err(|serialize_error| error(format!("serialize irrelevant UID: {serialize_error}")))?
    );
    if [principal.as_str(), action.as_str(), resource.as_str()].contains(&irrelevant_uid.as_str()) {
        return Err(error("irrelevant Actor UID unexpectedly matches a request reference"));
    }

    let entities = cedar
        .get_mut("entities")
        .and_then(Value::as_array_mut)
        .ok_or_else(|| error("Cedar entities must be an array"))?;
    for entity in entities.iter() {
        let uid = entity
            .get("uid")
            .ok_or_else(|| error("Cedar entity is missing uid"))?;
        let uid_type = uid.get("type").and_then(Value::as_str).unwrap_or_default();
        let uid_id = uid.get("id").and_then(Value::as_str).unwrap_or_default();
        if uid_type == "Mycelix::Actor" && uid_id == IRRELEVANT_ACTOR_ID {
            return Err(error("irrelevant Actor UID already exists in the entity set"));
        }
    }

    entities.push(serde_json::json!({
        "uid": {
            "type": "Mycelix::Actor",
            "id": IRRELEVANT_ACTOR_ID
        },
        "attrs": {
            "actorId": IRRELEVANT_ACTOR_ID,
            "roles": [],
            "organizations": []
        },
        "parents": []
    }));
    Ok(())
}

fn remove_contributor_from_claim_proposed(case: &mut Value) -> Result<(), MetamorphicError> {
    let (action, resource) = {
        let request = case
            .get("cedar")
            .and_then(|cedar| cedar.get("request"))
            .and_then(Value::as_object)
            .ok_or_else(|| error("Cedar request must be an object"))?;
        (
            required_string_value(request, "action")?.to_owned(),
            required_string_value(request, "resource")?.to_owned(),
        )
    };
    if action != r#"Mycelix::Action::"claim_proposed""# {
        return Ok(());
    }

    {
        let rego_input = case
            .get("rego")
            .and_then(|rego| rego.get("input"))
            .and_then(Value::as_object)
            .ok_or_else(|| error("Rego input must be an object"))?;
        if rego_input.get("action").and_then(Value::as_str) != Some("claim_proposed")
            || rego_input
                .get("claim")
                .and_then(|claim| claim.get("exists"))
                .and_then(Value::as_bool)
                != Some(false)
        {
            return Ok(());
        }
    }

    let target_actor_index = {
        let entities = case
            .get("cedar")
            .and_then(|cedar| cedar.get("entities"))
            .and_then(Value::as_array)
            .ok_or_else(|| error("Cedar entities must be an array"))?;
        let mut actor_index = None;
        let mut absent_claim = false;
        for (index, entity) in entities.iter().enumerate() {
            let uid = entity
                .get("uid")
                .ok_or_else(|| error("Cedar entity is missing uid"))?;
            let entity_type = uid.get("type").and_then(Value::as_str).unwrap_or_default();
            let entity_id = uid.get("id").and_then(Value::as_str).unwrap_or_default();
            if entity_type == "Mycelix::Actor" {
                let roles = entity
                    .get("attrs")
                    .and_then(|attrs| attrs.get("roles"))
                    .and_then(Value::as_array)
                    .ok_or_else(|| error("Cedar Actor roles must be an array"))?;
                if roles.as_slice() == [Value::String("contributor".to_owned())] {
                    actor_index = Some(index);
                }
            } else if entity_type == "Mycelix::Claim" && entity_id == resource {
                absent_claim = entity
                    .get("attrs")
                    .and_then(|attrs| attrs.get("exists"))
                    .and_then(Value::as_bool)
                    == Some(false);
            }
        }
        if absent_claim { actor_index } else { None }
    };

    let Some(actor_index) = target_actor_index else {
        return Ok(());
    };

    {
        let actor_roles = case
            .get("rego")
            .and_then(|rego| rego.get("input"))
            .and_then(|input| input.get("actor"))
            .and_then(|actor| actor.get("roles"))
            .and_then(Value::as_array)
            .ok_or_else(|| error("Rego actor roles must be an array"))?;
        if actor_roles.as_slice() != [Value::String("contributor".to_owned())] {
            return Err(error("target Cedar and Rego actor roles are not identical contributor-only sets"));
        }
    }

    {
        let entities = case
            .get_mut("cedar")
            .and_then(Value::as_object_mut)
            .and_then(|cedar| cedar.get_mut("entities"))
            .and_then(Value::as_array_mut)
            .ok_or_else(|| error("Cedar entities must be an array"))?;
        entities[actor_index]
            .get_mut("attrs")
            .and_then(Value::as_object_mut)
            .ok_or_else(|| error("target Cedar Actor attrs must be an object"))?
            .insert("roles".to_owned(), Value::Array(Vec::new()));
    }
    {
        let actor_roles = case
            .get_mut("rego")
            .and_then(Value::as_object_mut)
            .and_then(|rego| rego.get_mut("input"))
            .and_then(Value::as_object_mut)
            .and_then(|input| input.get_mut("actor"))
            .and_then(Value::as_object_mut)
            .and_then(|actor| actor.get_mut("roles"))
            .and_then(Value::as_array_mut)
            .ok_or_else(|| error("Rego actor roles must be an array"))?;
        actor_roles.clear();
    }
    Ok(())
}

fn expected_changed_case_count(transform: TransformId) -> usize {
    match transform {
        TransformId::JsonObjectKeyOrderReversed | TransformId::JsonWhitespaceExpanded => 0,
        TransformId::CedarEntityOrderReversed | TransformId::CedarIrrelevantActorAdded => {
            EXPECTED_CASE_COUNT
        }
        TransformId::ClaimProposedContributorRemoved => 1,
    }
}

pub fn transform_case(
    source: &Value,
    transform: TransformId,
) -> Result<(Value, Vec<u8>), MetamorphicError> {
    validate_case_shape(source, false)?;
    let mut transformed = source.clone();

    match transform {
        TransformId::JsonObjectKeyOrderReversed => {
            // The Value is unchanged; the custom byte writer below reverses
            // every JSON object's key order while preserving parsed values.
        }
        TransformId::JsonWhitespaceExpanded => {
            // The Value is unchanged; serialization happens in the writer.
        }
        TransformId::CedarEntityOrderReversed => {
            let entities = transformed
                .get_mut("cedar")
                .and_then(|cedar| cedar.get_mut("entities"))
                .and_then(Value::as_array_mut)
                .ok_or_else(|| error("Cedar entities must be an array"))?;
            entities.reverse();
        }
        TransformId::CedarIrrelevantActorAdded => append_irrelevant_actor(&mut transformed)?,
        TransformId::ClaimProposedContributorRemoved => {
            remove_contributor_from_claim_proposed(&mut transformed)?;
        }
    }

    let bytes = match transform {
        TransformId::JsonObjectKeyOrderReversed => {
            let mut output = String::new();
            reverse_object_member_order(&transformed, &mut output)?;
            output.into_bytes()
        }
        TransformId::JsonWhitespaceExpanded => serde_json::to_string_pretty(&transformed)
            .map_err(|serialize_error| error(format!("serialize expanded JSON: {serialize_error}")))?
            .replace('\n', " ")
            .into_bytes(),
        TransformId::CedarEntityOrderReversed
        | TransformId::CedarIrrelevantActorAdded
        | TransformId::ClaimProposedContributorRemoved => {
            serde_json::to_vec(&transformed)
                .map_err(|serialize_error| error(format!("serialize transformed case: {serialize_error}")))?
        }
    };

    let reparsed: Value = serde_json::from_slice(&bytes)
        .map_err(|parse_error| error(format!("transformed JSON did not reparse: {parse_error}")))?;
    if reparsed != transformed {
        return Err(error("transformed JSON bytes do not preserve the intended Value"));
    }
    validate_case_shape(&transformed, transform == TransformId::CedarIrrelevantActorAdded)?;
    Ok((transformed, bytes))
}

fn bounded_read(path: &Path, label: &str) -> Result<Vec<u8>, MetamorphicError> {
    let metadata = fs::metadata(path)
        .map_err(|io_error| error(format!("inspect {label}: {io_error}")))?;
    if !metadata.file_type().is_file() || metadata.len() > MAX_CORPUS_BYTES {
        return Err(error(format!("{label} is not a regular file within the byte limit")));
    }
    let bytes = fs::read(path).map_err(|io_error| error(format!("read {label}: {io_error}")))?;
    if bytes.len() as u64 > MAX_CORPUS_BYTES {
        return Err(error(format!("{label} exceeded the byte limit while reading")));
    }
    Ok(bytes)
}

fn write_variant(
    manifest: &Value,
    source_corpus: &[u8],
    source_inputs: &[u8],
    output_root: &Path,
    transform: TransformId,
) -> Result<VariantSummary, MetamorphicError> {
    let transform_directory = output_root.join(transform.as_str());
    fs::create_dir_all(&transform_directory)
        .map_err(|io_error| error(format!("create {}: {io_error}", transform_directory.display())))?;

    let corpus_file = required_string(manifest, "corpus_file")?;
    let engine_inputs_file = required_string(manifest, "engine_inputs_file")?;
    fs::write(transform_directory.join(corpus_file), source_corpus)
        .map_err(|io_error| error(format!("write copied abstract corpus: {io_error}")))?;

    let output_path = transform_directory.join(engine_inputs_file);
    let output_file = File::create(&output_path)
        .map_err(|io_error| error(format!("create transformed input corpus: {io_error}")))?;
    let mut writer = BufWriter::new(output_file);
    let mut output_hasher = Sha256::new();
    let reader = BufReader::new(source_inputs);
    let mut seen_case_ids = BTreeSet::new();
    let mut changed_case_ids = Vec::new();
    let mut case_count = 0usize;

    for (line_index, line) in reader.lines().enumerate() {
        let line = line.map_err(|io_error| error(format!("read source JSONL line {}: {io_error}", line_index + 1)))?;
        if line.trim().is_empty() {
            return Err(error(format!("blank source JSONL line {}", line_index + 1)));
        }
        let source_case: Value = serde_json::from_str(&line)
            .map_err(|parse_error| error(format!("parse source JSONL line {}: {parse_error}", line_index + 1)))?;
        let case_id = required_string(&source_case, "case_id")?.to_owned();
        if !seen_case_ids.insert(case_id.clone()) {
            return Err(error(format!("duplicate source case ID at line {}", line_index + 1)));
        }
        let (transformed_case, row) = transform_case(&source_case, transform)?;
        if transformed_case != source_case {
            changed_case_ids.push(case_id.clone());
        }
        let mut row = row;
        row.push(b'\n');
        writer
            .write_all(&row)
            .map_err(|io_error| error(format!("write transformed JSONL line {}: {io_error}", line_index + 1)))?;
        output_hasher.update(&row);
        case_count += 1;
    }
    writer
        .flush()
        .map_err(|io_error| error(format!("flush transformed corpus: {io_error}")))?;
    if case_count != EXPECTED_CASE_COUNT {
        return Err(error(format!(
            "transformed case count {case_count} does not equal frozen count {EXPECTED_CASE_COUNT}"
        )));
    }
    let expected_changed_count = expected_changed_case_count(transform);
    if changed_case_ids.len() != expected_changed_count {
        return Err(error(format!(
            "transform {} changed {} semantic case values; expected exactly {}",
            transform.as_str(),
            changed_case_ids.len(),
            expected_changed_count
        )));
    }

    let transformed_digest = hex::encode(output_hasher.finalize());
    let mut output_manifest = manifest.clone();
    output_manifest
        .as_object_mut()
        .ok_or_else(|| error("generator manifest must be a JSON object"))?
        .insert(
            "engine_inputs_sha256".to_owned(),
            Value::String(transformed_digest.clone()),
        );
    fs::write(
        transform_directory.join("manifest.json"),
        serde_json::to_vec_pretty(&output_manifest)
            .map_err(|serialize_error| error(format!("serialize transformed runner manifest: {serialize_error}")))?,
    )
    .map_err(|io_error| error(format!("write transformed runner manifest: {io_error}")))?;

    let baseline_revision = required_string(manifest, "baseline_revision")?.to_owned();
    let baseline_source_blob_sha = required_string(manifest, "baseline_source_blob_sha")?.to_owned();
    let output_record = VariantManifest {
        schema_version: "scientific-policy-metamorphic-v0.1".to_owned(),
        tool_id: TOOL_ID.to_owned(),
        tool_version: TOOL_VERSION.to_owned(),
        transform_id: transform,
        expected_relation: transform.relation().to_owned(),
        source_engine_inputs_sha256: required_string(manifest, "engine_inputs_sha256")?.to_owned(),
        transformed_engine_inputs_sha256: transformed_digest.clone(),
        source_case_count: required_usize(manifest, "engine_input_count")?,
        transformed_case_count: case_count,
        expected_changed_case_count: expected_changed_count,
        changed_case_count: changed_case_ids.len(),
        changed_case_ids,
        baseline_revision,
        baseline_source_blob_sha,
        precondition: transform.precondition().to_owned(),
        claim_ceiling: concat!(
            "deterministic input transformation and source-corpus binding only; ",
            "decision invariance must still be tested by the pinned Cedar and OPA engines; ",
            "no authentic Rust oracle parity or credential-registry correctness is established"
        ).to_owned(),
    };
    fs::write(
        transform_directory.join("transform-manifest.json"),
        serde_json::to_vec_pretty(&output_record)
            .map_err(|serialize_error| error(format!("serialize transform manifest: {serialize_error}")))?,
    )
    .map_err(|io_error| error(format!("write transform manifest: {io_error}")))?;

    Ok(VariantSummary {
        transform_id: transform,
        output_directory: transform_directory.display().to_string(),
        source_sha256: required_string(manifest, "engine_inputs_sha256")?.to_owned(),
        transformed_sha256: transformed_digest,
        case_count,
    })
}

pub fn generate_variants(
    generator_manifest_path: &Path,
    output_root: &Path,
) -> Result<Vec<VariantSummary>, MetamorphicError> {
    let manifest_bytes = bounded_read(generator_manifest_path, "generator manifest")?;
    let manifest: Value = serde_json::from_slice(&manifest_bytes)
        .map_err(|parse_error| error(format!("parse generator manifest: {parse_error}")))?;
    validate_generator_manifest(&manifest)?;

    let source_directory = generator_manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let corpus_path = source_directory.join(required_string(&manifest, "corpus_file")?);
    let input_path = source_directory.join(required_string(&manifest, "engine_inputs_file")?);
    let source_corpus = bounded_read(&corpus_path, "abstract source corpus")?;
    let source_inputs = bounded_read(&input_path, "paired engine-input corpus")?;
    if sha256(&source_corpus) != required_string(&manifest, "corpus_sha256")? {
        return Err(error("abstract source corpus SHA-256 does not match generator manifest"));
    }
    if sha256(&source_inputs) != required_string(&manifest, "engine_inputs_sha256")? {
        return Err(error("paired engine-input SHA-256 does not match generator manifest"));
    }

    fs::create_dir_all(output_root)
        .map_err(|io_error| error(format!("create metamorphic output root: {io_error}")))?;
    let mut summaries = Vec::with_capacity(TransformId::ALL.len());
    for transform in TransformId::ALL {
        summaries.push(write_variant(
            &manifest,
            &source_corpus,
            &source_inputs,
            output_root,
            transform,
        )?);
    }
    let root_manifest = serde_json::to_vec_pretty(&summaries)
        .map_err(|serialize_error| error(format!("serialize variant summary: {serialize_error}")))?;
    fs::write(output_root.join("variants.json"), root_manifest)
        .map_err(|io_error| error(format!("write variant summary: {io_error}")))?;
    Ok(summaries)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_case() -> Value {
        serde_json::json!({
            "case_id": "AUTH-GEN-test",
            "cedar": {
                "case_id": "AUTH-GEN-test",
                "request": {
                    "principal": "Mycelix::Actor::\"actor:candidate\"",
                    "action": "Mycelix::Action::\"evidence_attached\"",
                    "resource": "Mycelix::Claim::\"AUTH-GEN-test\"",
                    "context": {"attestationKind":"ordinary","targetAttestationExists":false}
                },
                "entities": [
                    {
                        "uid": {"type":"Mycelix::Actor","id":"actor:candidate"},
                        "attrs": {"actorId":"actor:candidate","roles":["contributor"],"organizations":[]},
                        "parents": []
                    },
                    {
                        "uid": {"type":"Mycelix::Claim","id":"AUTH-GEN-test"},
                        "attrs": {"exists":true,"creatorActorId":"actor:creator"},
                        "parents": []
                    }
                ]
            },
            "rego": {
                "case_id": "AUTH-GEN-test",
                "input": {"action":"evidence_attached","actor":{"id":"actor:candidate","roles":["contributor"]},"claim":{"exists":true}}
            }
        })
    }

    #[test]
    fn object_key_reordering_preserves_parsed_value_and_changes_bytes() {
        let source = sample_case();
        let (_, bytes) = transform_case(&source, TransformId::JsonObjectKeyOrderReversed).unwrap();
        let reparsed: Value = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(source, reparsed);
        assert_ne!(serde_json::to_vec(&source).unwrap(), bytes);
    }

    #[test]
    fn whitespace_transformation_preserves_parsed_value() {
        let source = sample_case();
        let (_, bytes) = transform_case(&source, TransformId::JsonWhitespaceExpanded).unwrap();
        let reparsed: Value = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(source, reparsed);
        assert!(String::from_utf8(bytes).unwrap().contains("  "));
    }

    #[test]
    fn reversing_cedar_entity_order_preserves_membership() {
        let source = sample_case();
        let (transformed, _) =
            transform_case(&source, TransformId::CedarEntityOrderReversed).unwrap();
        let source_entities = source["cedar"]["entities"].as_array().unwrap();
        let transformed_entities = transformed["cedar"]["entities"].as_array().unwrap();
        assert_eq!(source_entities[0], transformed_entities[1]);
        assert_eq!(source_entities[1], transformed_entities[0]);
        assert_eq!(source["rego"], transformed["rego"]);
    }

    #[test]
    fn irrelevant_actor_is_schema_shaped_and_does_not_rebind_request() {
        let source = sample_case();
        let (transformed, _) =
            transform_case(&source, TransformId::CedarIrrelevantActorAdded).unwrap();
        let entities = transformed["cedar"]["entities"].as_array().unwrap();
        assert_eq!(entities.len(), 3);
        let request = &transformed["cedar"]["request"];
        assert_eq!(request["principal"], source["cedar"]["request"]["principal"]);
        assert_eq!(request["action"], source["cedar"]["request"]["action"]);
        assert_eq!(request["resource"], source["cedar"]["request"]["resource"]);
        assert_eq!(transformed["rego"], source["rego"]);
        assert_eq!(entities[2]["uid"]["type"], "Mycelix::Actor");
        assert_eq!(entities[2]["attrs"]["roles"], serde_json::json!([]));
    }

    #[test]
    fn claim_proposed_contributor_removal_is_a_single_meaning_changing_control() {
        let mut source = sample_case();
        source["cedar"]["request"]["action"] =
            Value::String("Mycelix::Action::\\"claim_proposed\\"".to_owned());
        source["cedar"]["request"]["resource"] =
            Value::String("Mycelix::Claim::\\"new-claim\\"".to_owned());
        source["cedar"]["entities"][1]["uid"]["id"] =
            Value::String("new-claim".to_owned());
        source["cedar"]["entities"][1]["attrs"]["exists"] = Value::Bool(false);
        source["rego"]["input"]["action"] = Value::String("claim_proposed".to_owned());
        source["rego"]["input"]["claim"]["exists"] = Value::Bool(false);

        let (transformed, _) = transform_case(
            &source,
            TransformId::ClaimProposedContributorRemoved,
        )
        .unwrap();
        assert_eq!(transformed["case_id"], source["case_id"]);
        assert_eq!(transformed["rego"]["case_id"], source["rego"]["case_id"]);
        assert_eq!(transformed["cedar"]["entities"][0]["attrs"]["roles"], serde_json::json!([]));
        assert_eq!(transformed["rego"]["input"]["actor"]["roles"], serde_json::json!([]));
        assert_eq!(source["cedar"]["entities"][0]["attrs"]["roles"], serde_json::json!(["contributor"]));
    }

    #[test]
    fn claim_proposed_control_leaves_non_target_cases_unchanged() {
        let source = sample_case();
        let (transformed, _) = transform_case(
            &source,
            TransformId::ClaimProposedContributorRemoved,
        )
        .unwrap();
        assert_eq!(transformed, source);
    }

    #[test]
    fn rejects_duplicate_cedar_entity_uids_and_mismatched_case_ids() {
        let mut duplicate = sample_case();
        duplicate["cedar"]["entities"][1]["uid"] =
            duplicate["cedar"]["entities"][0]["uid"].clone();
        assert!(transform_case(&duplicate, TransformId::CedarEntityOrderReversed).is_err());

        let mut mismatch = sample_case();
        mismatch["rego"]["case_id"] = Value::String("different".to_owned());
        assert!(transform_case(&mismatch, TransformId::JsonWhitespaceExpanded).is_err());
    }
}
