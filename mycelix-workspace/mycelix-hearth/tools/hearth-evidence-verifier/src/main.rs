// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::{collections::{BTreeMap, HashSet}, fs, io::Cursor, path::PathBuf, process::ExitCode};

use anyhow::{anyhow, bail, Context, Result};
use base64::{engine::general_purpose::URL_SAFE_NO_PAD, Engine as _};
use blake2b_simd::Params as Blake2Params;
use clap::Parser;
use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use rmpv::Value as MpValue;
use serde::Serialize;
use serde_json::{Map, Number, Value};
use sha2::{Digest, Sha256};

const V5_SCHEMA: &str = "mycelix-hearth-migration-evidence/v5";
const V3_SCHEMA: &str = "mycelix-hearth-migration-evidence/v3";
const V2_SCHEMA: &str = "mycelix-hearth-migration-evidence/v2";
const ACTION_PREFIX: [u8; 3] = [132, 41, 36];
const ENTRY_PREFIX: [u8; 3] = [132, 33, 36];
const AGENT_PREFIX: [u8; 3] = [132, 32, 36];
const CONVERGENCE_MICROS: i128 = 5 * 60 * 1_000_000;
const MAX_SNAPSHOT_TTL_MICROS: i128 = 30 * 60 * 1_000_000;
const MAX_OBSERVATION_TTL_MICROS: i128 = 60 * 60 * 1_000_000;

#[derive(Parser)]
#[command(about = "Verify a Mycelix Hearth v5 migration-evidence bundle without JavaScript")]
struct Args {
    /// Path to a migration-evidence v5 JSON bundle.
    bundle: PathBuf,
    /// Verification time as Unix microseconds. Defaults to the system clock.
    #[arg(long)]
    now_micros: Option<i128>,
    /// Emit human-readable text instead of JSON.
    #[arg(long)]
    human: bool,
}

#[derive(Default, Serialize)]
struct Report {
    valid: bool,
    schema_verified: bool,
    manifests_verified: bool,
    portable_records_verified: usize,
    action_hashes_verified: bool,
    entry_hashes_verified: bool,
    signatures_verified: bool,
    semantic_chain_verified: bool,
    freshness_verified: bool,
    evidence_disputed: bool,
    errors: Vec<String>,
    warnings: Vec<String>,
}

fn main() -> ExitCode {
    let args = Args::parse();
    let mut report = Report::default();
    match run(&args, &mut report) {
        Ok(()) => report.valid = report.errors.is_empty() && !report.evidence_disputed,
        Err(error) => report.errors.push(format!("{error:#}")),
    }
    if args.human {
        println!("Hearth migration evidence: {}", if report.valid { "VALID" } else { "INVALID" });
        for error in &report.errors { println!("error: {error}"); }
        for warning in &report.warnings { println!("warning: {warning}"); }
    } else {
        println!("{}", serde_json::to_string_pretty(&report).expect("serialize report"));
    }
    if report.valid { ExitCode::SUCCESS } else { ExitCode::FAILURE }
}

fn run(args: &Args, report: &mut Report) -> Result<()> {
    let bytes = fs::read(&args.bundle).with_context(|| format!("read {}", args.bundle.display()))?;
    let root: Value = serde_json::from_slice(&bytes).context("parse evidence JSON")?;
    expect_str(&root, "schema")?.eq(V5_SCHEMA).then_some(()).ok_or_else(|| anyhow!("unsupported schema"))?;
    report.schema_verified = true;
    verify_manifests(&root)?;
    report.manifests_verified = true;

    let mut records = Vec::new();
    collect_portable_records(&root, "$", &mut records);
    if records.is_empty() { bail!("bundle contains no portable signed records"); }
    for (path, record) in records {
        verify_portable_record(record).with_context(|| path.clone())?;
        report.portable_records_verified += 1;
    }
    report.action_hashes_verified = true;
    report.entry_hashes_verified = true;
    report.signatures_verified = true;

    verify_semantics(&root, args.now_micros.unwrap_or_else(system_micros), report)?;
    report.semantic_chain_verified = true;
    report.freshness_verified = true;
    Ok(())
}

fn system_micros() -> i128 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_micros() as i128
}

fn canonical_json(value: &Value, out: &mut String) {
    match value {
        Value::Null => out.push_str("null"),
        Value::Bool(value) => out.push_str(if *value { "true" } else { "false" }),
        Value::Number(value) => out.push_str(&value.to_string()),
        Value::String(value) => out.push_str(&serde_json::to_string(value).expect("string JSON")),
        Value::Array(values) => {
            out.push('[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 { out.push(','); }
                canonical_json(value, out);
            }
            out.push(']');
        }
        Value::Object(values) => {
            out.push('{');
            let sorted: BTreeMap<_, _> = values.iter().collect();
            for (index, (key, value)) in sorted.into_iter().enumerate() {
                if index > 0 { out.push(','); }
                out.push_str(&serde_json::to_string(key).expect("key JSON"));
                out.push(':');
                canonical_json(value, out);
            }
            out.push('}');
        }
    }
}

fn digest_json(value: &Value) -> (String, usize) {
    let mut canonical = String::new();
    canonical_json(value, &mut canonical);
    let digest = Sha256::digest(canonical.as_bytes());
    (URL_SAFE_NO_PAD.encode(digest), canonical.len())
}

fn verify_manifest_object(object: &Map<String, Value>, field: &str, check_size: bool) -> Result<()> {
    let manifest = object.get(field).and_then(Value::as_object).ok_or_else(|| anyhow!("missing {field}"))?;
    if manifest.get("algorithm").and_then(Value::as_str) != Some("SHA-256") { bail!("{field} algorithm is not SHA-256"); }
    let expected = manifest.get("digest_base64url").and_then(Value::as_str).ok_or_else(|| anyhow!("missing {field} digest"))?;
    let mut unsigned = object.clone();
    unsigned.remove(field);
    let (actual, size) = digest_json(&Value::Object(unsigned));
    if expected != actual { bail!("{field} digest mismatch"); }
    if check_size {
        let declared = manifest.get("payload_size_bytes").and_then(Value::as_u64).ok_or_else(|| anyhow!("missing core payload size"))?;
        if declared as usize != size { bail!("core payload size mismatch"); }
    }
    Ok(())
}

fn verify_manifests(root: &Value) -> Result<()> {
    let root_obj = root.as_object().ok_or_else(|| anyhow!("root is not an object"))?;
    verify_manifest_object(root_obj, "bundle_manifest", false)?;
    let closure = root_obj.get("closure_bundle").and_then(Value::as_object).ok_or_else(|| anyhow!("missing closure_bundle"))?;
    if closure.get("schema").and_then(Value::as_str) != Some(V3_SCHEMA) { bail!("closure schema mismatch"); }
    verify_manifest_object(closure, "bundle_manifest", false)?;
    let core = closure.get("core").and_then(Value::as_object).ok_or_else(|| anyhow!("missing evidence core"))?;
    if core.get("schema").and_then(Value::as_str) != Some(V2_SCHEMA) { bail!("core schema mismatch"); }
    verify_manifest_object(core, "core_manifest", true)?;
    Ok(())
}

fn collect_portable_records<'a>(value: &'a Value, path: &str, out: &mut Vec<(String, &'a Map<String, Value>)>) {
    match value {
        Value::Object(object) => {
            let required = ["action_hash", "author", "signature", "action_msgpack", "app_entry_msgpack", "record_msgpack", "entry"];
            if required.iter().all(|key| object.contains_key(*key)) {
                out.push((path.to_string(), object));
                return;
            }
            for (key, child) in object { collect_portable_records(child, &format!("{path}.{key}"), out); }
        }
        Value::Array(values) => for (index, child) in values.iter().enumerate() {
            collect_portable_records(child, &format!("{path}[{index}]"), out);
        },
        _ => {}
    }
}

fn decode_b64(object: &Map<String, Value>, key: &str) -> Result<Vec<u8>> {
    let value = object.get(key).and_then(Value::as_str).ok_or_else(|| anyhow!("missing {key}"))?;
    URL_SAFE_NO_PAD.decode(value).with_context(|| format!("decode {key}"))
}

fn holo_hash(content: &[u8], prefix: [u8; 3]) -> Vec<u8> {
    let core = Blake2Params::new().hash_length(32).hash(content);
    let location_hash = Blake2Params::new().hash_length(16).hash(core.as_bytes());
    let mut location = [0u8; 4];
    for chunk in location_hash.as_bytes().chunks_exact(4) {
        for index in 0..4 { location[index] ^= chunk[index]; }
    }
    [prefix.as_slice(), core.as_bytes(), location.as_slice()].concat()
}

fn encode_app_entry(entry: &[u8]) -> Result<Vec<u8>> {
    let mut output = Vec::new();
    rmp::encode::write_map_len(&mut output, 2)?;
    rmp::encode::write_str(&mut output, "entry_type")?;
    rmp::encode::write_str(&mut output, "App")?;
    rmp::encode::write_str(&mut output, "entry")?;
    rmp::encode::write_bin(&mut output, entry)?;
    Ok(output)
}

fn map_get<'a>(value: &'a MpValue, wanted: &str) -> Option<&'a MpValue> {
    match value {
        MpValue::Map(entries) => {
            for (key, child) in entries {
                if key.as_str() == Some(wanted) { return Some(child); }
                if let Some(found) = map_get(child, wanted) { return Some(found); }
            }
            None
        }
        MpValue::Array(values) => values.iter().find_map(|value| map_get(value, wanted)),
        _ => None,
    }
}

fn mp_binary(value: &MpValue, label: &str) -> Result<Vec<u8>> {
    match value {
        MpValue::Binary(bytes) => Ok(bytes.clone()),
        _ => bail!("{label} is not MessagePack binary"),
    }
}


fn mp_timestamp_string(value: &MpValue, label: &str) -> Result<String> {
    match value {
        MpValue::Integer(value) => {
            if let Some(value) = value.as_i64() {
                Ok(format!("{}", value as f64))
            } else {
                Ok(format!("{}", value.as_u64().ok_or_else(|| anyhow!("{label} is not an integer"))? as f64))
            }
        }
        _ => bail!("{label} is not a MessagePack integer"),
    }
}

fn encode_mp_value(value: &MpValue) -> Result<Vec<u8>> {
    let mut output = Vec::new();
    rmpv::encode::write_value(&mut output, value)?;
    Ok(output)
}


fn mp_to_json(value: &MpValue) -> Result<Value> {
    Ok(match value {
        MpValue::Nil => Value::Null,
        MpValue::Boolean(value) => Value::Bool(*value),
        MpValue::Integer(value) => {
            // @msgpack/msgpack decodes int64 values to JavaScript Number by
            // default. Reproduce that rounded integer representation so the
            // native verifier compares the same portable JSON view.
            if let Some(value) = value.as_i64() {
                Value::Number(Number::from((value as f64) as i64))
            } else {
                let value = value.as_u64().ok_or_else(|| anyhow!("invalid MessagePack integer"))?;
                Value::Number(Number::from((value as f64) as u64))
            }
        }
        MpValue::F32(value) => Value::Number(Number::from_f64(*value as f64).ok_or_else(|| anyhow!("non-finite f32"))?),
        MpValue::F64(value) => Value::Number(Number::from_f64(*value).ok_or_else(|| anyhow!("non-finite f64"))?),
        MpValue::String(value) => Value::String(value.as_str().ok_or_else(|| anyhow!("invalid UTF-8 MessagePack string"))?.to_string()),
        MpValue::Binary(value) => Value::String(URL_SAFE_NO_PAD.encode(value)),
        MpValue::Array(values) => Value::Array(values.iter().map(mp_to_json).collect::<Result<Vec<_>>>()?),
        MpValue::Map(values) => {
            let mut object = Map::new();
            for (key, value) in values {
                let key = key.as_str().ok_or_else(|| anyhow!("non-string MessagePack map key"))?;
                if object.insert(key.to_string(), mp_to_json(value)?).is_some() {
                    bail!("duplicate MessagePack map key");
                }
            }
            Value::Object(object)
        }
        MpValue::Ext(_, _) => bail!("unsupported MessagePack extension in App entry"),
    })
}

fn verify_portable_record(object: &Map<String, Value>) -> Result<()> {
    let action_hash = decode_b64(object, "action_hash")?;
    let author = decode_b64(object, "author")?;
    let signature_bytes = decode_b64(object, "signature")?;
    let action_bytes = decode_b64(object, "action_msgpack")?;
    let entry_bytes = decode_b64(object, "app_entry_msgpack")?;
    let record_bytes = decode_b64(object, "record_msgpack")?;
    if action_hash.len() != 39 || action_hash[..3] != ACTION_PREFIX { bail!("invalid ActionHash"); }
    if author.len() != 39 || author[..3] != AGENT_PREFIX { bail!("invalid AgentPubKey"); }
    if signature_bytes.len() != 64 { bail!("invalid signature length"); }
    if holo_hash(&action_bytes, ACTION_PREFIX) != action_hash { bail!("action hash mismatch"); }

    let action = rmpv::decode::read_value(&mut Cursor::new(&action_bytes)).context("decode action MessagePack")?;
    if encode_mp_value(&action)? != action_bytes { bail!("action MessagePack is not canonical"); }
    let action_author = mp_binary(map_get(&action, "author").ok_or_else(|| anyhow!("action author missing"))?, "action author")?;
    if action_author != author { bail!("action author mismatch"); }
    let action_timestamp = mp_timestamp_string(
        map_get(&action, "timestamp").ok_or_else(|| anyhow!("action timestamp missing"))?,
        "action timestamp",
    )?;
    if object.get("action_timestamp_micros").and_then(Value::as_str) != Some(action_timestamp.as_str()) {
        bail!("portable action timestamp mismatch");
    }
    let entry_hash = mp_binary(map_get(&action, "entry_hash").ok_or_else(|| anyhow!("action entry_hash missing"))?, "entry_hash")?;
    if holo_hash(&encode_app_entry(&entry_bytes)?, ENTRY_PREFIX) != entry_hash { bail!("App entry hash mismatch"); }
    let decoded_entry = rmpv::decode::read_value(&mut Cursor::new(&entry_bytes)).context("decode App entry MessagePack")?;
    let portable_entry = object.get("entry").ok_or_else(|| anyhow!("portable entry missing"))?;
    if mp_to_json(&decoded_entry)? != *portable_entry { bail!("portable entry differs from signed App entry bytes"); }

    let key = VerifyingKey::from_bytes(author[3..35].try_into().expect("32 bytes"))?;
    let signature = Signature::from_slice(&signature_bytes)?;
    key.verify(&action_bytes, &signature).context("invalid action signature")?;

    let record = rmpv::decode::read_value(&mut Cursor::new(&record_bytes)).context("decode record MessagePack")?;
    if encode_mp_value(&record)? != record_bytes { bail!("record MessagePack is not canonical"); }
    let record_hash = mp_binary(map_get(&record, "hash").ok_or_else(|| anyhow!("record hash missing"))?, "record hash")?;
    let record_signature = mp_binary(map_get(&record, "signature").ok_or_else(|| anyhow!("record signature missing"))?, "record signature")?;
    let record_action = map_get(&record, "content").ok_or_else(|| anyhow!("record action content missing"))?;
    let record_entry = mp_binary(map_get(&record, "entry").ok_or_else(|| anyhow!("record App entry missing"))?, "record App entry")?;
    if record_hash != action_hash
        || record_signature != signature_bytes
        || encode_mp_value(record_action)? != action_bytes
        || record_entry != entry_bytes
    {
        bail!("record envelope differs from portable fields");
    }
    Ok(())
}

fn at<'a>(value: &'a Value, path: &[&str]) -> Result<&'a Value> {
    let mut current = value;
    for key in path { current = current.get(*key).ok_or_else(|| anyhow!("missing {}", path.join(".")))?; }
    Ok(current)
}
fn expect_str<'a>(value: &'a Value, key: &str) -> Result<&'a str> {
    value.get(key).and_then(Value::as_str).ok_or_else(|| anyhow!("missing string {key}"))
}
fn entry<'a>(record: &'a Value) -> Result<&'a Value> { record.get("entry").ok_or_else(|| anyhow!("portable record entry missing")) }
fn integer(value: &Value) -> Result<i128> {
    if let Some(value) = value.as_str() { return value.parse().context("parse integer string"); }
    if let Some(value) = value.as_i64() { return Ok(value as i128); }
    if let Some(value) = value.as_u64() { return Ok(value as i128); }
    bail!("value is not an integer")
}
fn string_array(value: &Value) -> Result<Vec<&str>> {
    value.as_array().ok_or_else(|| anyhow!("expected array"))?.iter()
        .map(|value| value.as_str().ok_or_else(|| anyhow!("expected string array"))).collect()
}

fn verify_semantics(root: &Value, now: i128, report: &mut Report) -> Result<()> {
    let closure = at(root, &["closure_bundle"])?;
    let core = at(closure, &["core"])?;
    let plan_record = at(core, &["plan"])?;
    let statement_record = at(core, &["statement"])?;
    let completion_record = at(core, &["completion"])?;
    let plan = entry(plan_record)?;
    let statement = entry(statement_record)?;
    let completion = entry(completion_record)?;
    let plan_hash = expect_str(plan_record, "action_hash")?;
    let statement_hash = expect_str(statement_record, "action_hash")?;
    let completion_hash = expect_str(completion_record, "action_hash")?;
    let source_dna_hash = expect_str(core, "source_dna_hash")?;
    if expect_str(plan_record, "author")? != expect_str(plan, "created_by")?
        || expect_str(statement_record, "author")? != expect_str(statement, "created_by")?
        || expect_str(completion_record, "author")? != expect_str(completion, "completed_by")?
    {
        bail!("plan, statement, or completion author claim differs from its signed action");
    }
    if expect_str(statement, "plan_hash")? != plan_hash { bail!("statement does not cite plan"); }
    if expect_str(completion, "plan_hash")? != plan_hash || expect_str(completion, "statement_hash")? != statement_hash { bail!("completion chain mismatch"); }
    if completion.get("permanently_freezes_source").and_then(Value::as_bool) != Some(true) { bail!("completion does not freeze source"); }
    for field in ["hearth_hash", "migration_id", "active_roster_commitment", "destination_network_commitment", "destination_properties_commitment"] {
        if plan.get(field) != statement.get(field) || statement.get(field) != completion.get(field) {
            bail!("migration chain field {field} differs across plan, statement, and completion");
        }
    }
    for field in ["destination_receipt_hash", "destination_hearth_hash", "destination_dna_hash", "destination_access_epoch_hash", "destination_recovery_epoch_hash"] {
        if statement.get(field) != completion.get(field) {
            bail!("destination field {field} differs between statement and completion");
        }
    }

    let roster = string_array(statement.get("recipient_agents").ok_or_else(|| anyhow!("statement roster missing"))?)?;
    let plan_recipients = plan.get("recipients").and_then(Value::as_array).ok_or_else(|| anyhow!("plan recipients missing"))?;
    let plan_roster = plan_recipients.iter().map(|recipient| expect_str(recipient, "agent")).collect::<Result<Vec<_>>>()?;
    if plan_roster != roster { bail!("canonical statement recipient order differs from the migration plan"); }
    let acks = at(core, &["acknowledgements"])?.as_array().ok_or_else(|| anyhow!("acknowledgements missing"))?;
    let expected_ack_hashes = string_array(completion.get("acknowledgement_hashes").ok_or_else(|| anyhow!("completion acknowledgement hashes missing"))?)?;
    if acks.len() != roster.len() || expected_ack_hashes.len() != roster.len() { bail!("destination acknowledgement quorum incomplete"); }
    for (index, ack_record) in acks.iter().enumerate() {
        if expect_str(ack_record, "action_hash")? != expected_ack_hashes[index] { bail!("destination acknowledgement order mismatch"); }
        let ack = entry(ack_record)?;
        if expect_str(ack_record, "author")? != roster[index]
            || expect_str(ack, "acknowledged_by")? != roster[index]
            || expect_str(ack, "statement_hash")? != statement_hash
            || expect_str(ack, "plan_hash")? != plan_hash
        {
            bail!("destination acknowledgement roster or chain mismatch");
        }
    }

    let closure_data = at(closure, &["closure"])?;
    let publications = closure_data.get("publications").and_then(Value::as_array).ok_or_else(|| anyhow!("publications missing"))?;
    let checkpoint_record = at(closure_data, &["checkpoint"])?;
    let checkpoint = entry(checkpoint_record)?;
    let publication_hashes = string_array(checkpoint.get("publication_hashes").ok_or_else(|| anyhow!("checkpoint publication hashes missing"))?)?;
    if publications.len() != roster.len() || publication_hashes.len() != roster.len() { bail!("publication quorum incomplete"); }
    let mut publication_authors = HashSet::new();
    for (index, publication_record) in publications.iter().enumerate() {
        if expect_str(publication_record, "action_hash")? != publication_hashes[index] { bail!("publication order mismatch"); }
        let publication = entry(publication_record)?;
        let author = expect_str(publication, "published_by")?;
        if expect_str(publication_record, "author")? != author
            || author != roster[index]
            || !publication_authors.insert(author)
            || expect_str(publication, "plan_hash")? != plan_hash
            || expect_str(publication, "statement_hash")? != statement_hash
            || expect_str(publication, "completion_hash")? != completion_hash
            || expect_str(publication, "source_dna_hash")? != source_dna_hash
        {
            bail!("publication roster or migration-chain mismatch");
        }
    }
    let checkpoint_hash = expect_str(checkpoint_record, "action_hash")?;
    if expect_str(checkpoint_record, "author")? != expect_str(checkpoint, "created_by")?
        || expect_str(checkpoint, "plan_hash")? != plan_hash
        || expect_str(checkpoint, "statement_hash")? != statement_hash
        || expect_str(checkpoint, "completion_hash")? != completion_hash
        || expect_str(checkpoint, "source_dna_hash")? != source_dna_hash
    {
        bail!("checkpoint author or migration-chain mismatch");
    }
    let seal_record = at(closure_data, &["seal"])?;
    let seal = entry(seal_record)?;
    if expect_str(seal_record, "author")? != expect_str(seal, "sealed_by")?
        || expect_str(seal, "checkpoint_hash")? != checkpoint_hash
        || expect_str(seal, "plan_hash")? != plan_hash
        || expect_str(seal, "completion_hash")? != completion_hash
        || seal.get("closes_observed_publication_set").and_then(Value::as_bool) != Some(true)
    {
        bail!("terminal seal mismatch");
    }
    let checkpoint_acks = closure_data.get("checkpoint_acknowledgements").and_then(Value::as_array).ok_or_else(|| anyhow!("checkpoint acknowledgements missing"))?;
    let seal_ack_hashes = string_array(seal.get("acknowledgement_hashes").ok_or_else(|| anyhow!("seal acknowledgement hashes missing"))?)?;
    if checkpoint_acks.len() != roster.len() || seal_ack_hashes.len() != roster.len() { bail!("checkpoint acknowledgement quorum incomplete"); }
    for (index, ack_record) in checkpoint_acks.iter().enumerate() {
        if expect_str(ack_record, "action_hash")? != seal_ack_hashes[index] { bail!("checkpoint acknowledgement order mismatch"); }
        let ack = entry(ack_record)?;
        if expect_str(ack_record, "author")? != roster[index]
            || expect_str(ack, "acknowledged_by")? != roster[index]
            || expect_str(ack, "checkpoint_hash")? != checkpoint_hash
            || expect_str(ack, "plan_hash")? != plan_hash
            || expect_str(ack, "completion_hash")? != completion_hash
        {
            bail!("checkpoint acknowledgement roster mismatch");
        }
    }

    let freshness = at(root, &["freshness"])?;
    let snapshot_record = at(freshness, &["snapshot"])?;
    let snapshot = entry(snapshot_record)?;
    if expect_str(snapshot_record, "author")? != expect_str(snapshot, "created_by")?
        || expect_str(snapshot, "completion_hash")? != completion_hash
        || expect_str(snapshot, "plan_hash")? != plan_hash
        || expect_str(snapshot, "statement_hash")? != statement_hash
        || expect_str(snapshot, "source_dna_hash")? != source_dna_hash
    {
        bail!("freshness snapshot cites another chain or has a false creator claim");
    }
    let guardian_membership_record = at(freshness, &["snapshot_guardian_membership"])?;
    let guardian_membership = entry(guardian_membership_record)?;
    let snapshot_creator = expect_str(snapshot, "created_by")?;
    if expect_str(snapshot, "authorization_membership_hash")? != expect_str(guardian_membership_record, "action_hash")?
        || expect_str(guardian_membership_record, "author")? != snapshot_creator
        || expect_str(guardian_membership, "agent")? != snapshot_creator
        || expect_str(guardian_membership, "hearth_hash")? != expect_str(completion, "hearth_hash")?
        || expect_str(guardian_membership, "status")? != "Active"
        || !matches!(expect_str(guardian_membership, "role")?, "Founder" | "Elder" | "Adult")
    {
        bail!("freshness snapshot is not bound to an active guardian membership");
    }
    let snapshot_at = integer(snapshot.get("snapshot_at").ok_or_else(|| anyhow!("snapshot_at missing"))?)?;
    let valid_until = integer(snapshot.get("valid_until").ok_or_else(|| anyhow!("valid_until missing"))?)?;
    if valid_until - snapshot_at <= 0 || valid_until - snapshot_at > MAX_SNAPSHOT_TTL_MICROS { bail!("snapshot TTL outside bound"); }
    if now < snapshot_at || now > valid_until { bail!("snapshot is not current"); }
    let observations = freshness.get("observations").and_then(Value::as_array).ok_or_else(|| anyhow!("freshness observations missing"))?;
    let observation_hashes = string_array(snapshot.get("observation_hashes").ok_or_else(|| anyhow!("snapshot observation hashes missing"))?)?;
    if observations.len() != roster.len() || observation_hashes.len() != roster.len() { bail!("freshness observation quorum incomplete"); }
    let report_hashes = string_array(snapshot.get("observed_report_hashes").ok_or_else(|| anyhow!("snapshot reports missing"))?)?;
    let mut latest = 0i128;
    for (index, proof) in observations.iter().enumerate() {
        let observation_record = at(proof, &["observation"])?;
        if expect_str(observation_record, "action_hash")? != observation_hashes[index] { bail!("observation order mismatch"); }
        let observation = entry(observation_record)?;
        let observer = roster[index];
        if expect_str(observation_record, "author")? != observer
            || expect_str(observation, "observed_by")? != observer
            || expect_str(observation, "plan_hash")? != plan_hash
            || expect_str(observation, "statement_hash")? != statement_hash
            || expect_str(observation, "completion_hash")? != completion_hash
            || expect_str(observation, "source_dna_hash")? != source_dna_hash
        {
            bail!("observation roster or migration-chain mismatch");
        }
        let observer_membership_record = at(proof, &["observer_membership"])?;
        let observer_membership = entry(observer_membership_record)?;
        if expect_str(observation, "authorization_membership_hash")? != expect_str(observer_membership_record, "action_hash")?
            || expect_str(observer_membership_record, "author")? != observer
            || expect_str(observer_membership, "agent")? != observer
            || expect_str(observer_membership, "hearth_hash")? != expect_str(completion, "hearth_hash")?
            || expect_str(observer_membership, "status")? != "Active"
        {
            bail!("observation is not bound to the recipient's active membership");
        }
        let reports = string_array(observation.get("observed_report_hashes").ok_or_else(|| anyhow!("observation reports missing"))?)?;
        if reports != report_hashes { bail!("observations disagree on dispute set"); }
        let observed_at = integer(observation.get("observed_at").ok_or_else(|| anyhow!("observed_at missing"))?)?;
        let expires_at = integer(observation.get("expires_at").ok_or_else(|| anyhow!("expires_at missing"))?)?;
        if expires_at - observed_at <= 0 || expires_at - observed_at > MAX_OBSERVATION_TTL_MICROS || expires_at < valid_until { bail!("observation TTL outside bound"); }
        latest = latest.max(observed_at);
    }
    if snapshot_at - latest < CONVERGENCE_MICROS { bail!("freshness convergence interval not met"); }
    let proofs = freshness.get("equivocation_proofs").and_then(Value::as_array).ok_or_else(|| anyhow!("equivocation proofs missing"))?;
    let mut proof_hashes = Vec::with_capacity(proofs.len());
    for proof in proofs {
        let report_record = at(proof, &["report"])?;
        let report_entry = entry(report_record)?;
        let publication_a_record = at(proof, &["publication_a"])?;
        let publication_b_record = at(proof, &["publication_b"])?;
        let publication_a = entry(publication_a_record)?;
        let publication_b = entry(publication_b_record)?;
        let reporter_membership_record = at(proof, &["reporter_membership"])?;
        let reporter_membership = entry(reporter_membership_record)?;
        let report_hash = expect_str(report_record, "action_hash")?;
        proof_hashes.push(report_hash);
        let reporter = expect_str(report_entry, "reported_by")?;
        let equivocator = expect_str(report_entry, "equivocator")?;
        if expect_str(report_record, "author")? != reporter
            || expect_str(report_entry, "plan_hash")? != plan_hash
            || expect_str(report_entry, "statement_hash")? != statement_hash
            || expect_str(report_entry, "completion_hash")? != completion_hash
            || report_entry.get("invalidates_evidence").and_then(Value::as_bool) != Some(true)
            || expect_str(report_entry, "publication_a_hash")? != expect_str(publication_a_record, "action_hash")?
            || expect_str(report_entry, "publication_b_hash")? != expect_str(publication_b_record, "action_hash")?
        {
            bail!("equivocation report does not bind the signed migration evidence records");
        }
        if expect_str(publication_a_record, "author")? != equivocator
            || expect_str(publication_b_record, "author")? != equivocator
            || expect_str(publication_a, "published_by")? != equivocator
            || expect_str(publication_b, "published_by")? != equivocator
        {
            bail!("equivocation publications are not authored by one exact recipient");
        }
        let tuple_a = [
            publication_a.get("core_manifest_digest"),
            publication_a.get("core_bundle_size_bytes"),
            publication_a.get("evidence_schema"),
            publication_a.get("record_profile"),
        ];
        let tuple_b = [
            publication_b.get("core_manifest_digest"),
            publication_b.get("core_bundle_size_bytes"),
            publication_b.get("evidence_schema"),
            publication_b.get("record_profile"),
        ];
        if tuple_a == tuple_b { bail!("equivocation proof contains duplicate rather than conflicting publications"); }
        if expect_str(report_entry, "authorization_membership_hash")? != expect_str(reporter_membership_record, "action_hash")?
            || expect_str(reporter_membership_record, "author")? != reporter
            || expect_str(reporter_membership, "agent")? != reporter
            || expect_str(reporter_membership, "hearth_hash")? != expect_str(completion, "hearth_hash")?
            || expect_str(reporter_membership, "status")? != "Active"
        {
            bail!("equivocation report is not bound to the reporter's active membership");
        }
    }
    proof_hashes.sort_unstable();
    let mut expected_reports = report_hashes.clone();
    expected_reports.sort_unstable();
    if proof_hashes != expected_reports { bail!("equivocation proofs differ from snapshot report set"); }
    if !report_hashes.is_empty() {
        report.evidence_disputed = true;
        report.errors.push("signed freshness snapshot contains an evidence dispute".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn holo_hash_matches_holochain_client_vector() {
        let hash = holo_hash(b"abc", ACTION_PREFIX);
        assert_eq!(
            URL_SAFE_NO_PAD.encode(hash),
            "hCkkvd2BPGNCOXIxce8_7phXm5SWTjuxyz5CcmLIwGjVIxmY4DLe"
        );
    }

    #[test]
    fn canonical_json_sorts_object_keys() {
        let value = serde_json::json!({"z": 1, "a": [true, {"b": "x", "a": null}]});
        let mut output = String::new();
        canonical_json(&value, &mut output);
        assert_eq!(output, r#"{"a":[true,{"a":null,"b":"x"}],"z":1}"#);
    }
}
