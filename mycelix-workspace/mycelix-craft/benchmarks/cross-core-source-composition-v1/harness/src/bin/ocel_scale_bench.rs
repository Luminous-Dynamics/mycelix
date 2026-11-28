#![forbid(unsafe_code)]

//! Reproducible synthetic OCEL profile scale probe.
//!
//! This measures the current whole-document validator/canonicalizer. It is
//! intentionally an executable benchmark, not a CI performance assertion.
//! Large event counts can require substantial memory because the current API
//! materializes JSON values and a canonical JSON receipt.

use craft_water_cross_core_harness::ocel_profile::validate_ocel20_water_profile;
use craft_water_cross_core_harness::temporal_replay::{
    replay_temporal_frontier, TemporalReplayCutV1,
};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

const FIXTURE: &str = include_str!("../../../ocel/ocel-water-service-v1.json");
const MAX_EVENTS: usize = 10_000_000;
const WARMUP_RUNS: usize = 2;

fn set_attribute(event: &mut Value, name: &str, value: Value) -> Result<(), Box<dyn Error>> {
    let attributes = event
        .get_mut("attributes")
        .and_then(Value::as_array_mut)
        .ok_or("event attributes must be an array")?;
    let attribute = attributes
        .iter_mut()
        .find(|attribute| attribute.get("name").and_then(Value::as_str) == Some(name))
        .ok_or_else(|| format!("missing event attribute {name}"))?;
    attribute["value"] = value;
    Ok(())
}

fn set_object_attribute(
    object: &mut Value,
    name: &str,
    value: Value,
) -> Result<(), Box<dyn Error>> {
    let attributes = object
        .get_mut("attributes")
        .and_then(Value::as_array_mut)
        .ok_or("object attributes must be an array")?;
    let attribute = attributes
        .iter_mut()
        .find(|attribute| attribute.get("name").and_then(Value::as_str) == Some(name))
        .ok_or_else(|| format!("missing object attribute {name}"))?;
    attribute["value"] = value;
    Ok(())
}

fn object_template(fixture: &Value, id: &str) -> Result<Value, Box<dyn Error>> {
    fixture
        .get("objects")
        .and_then(Value::as_array)
        .and_then(|objects| objects.iter().find(|object| object["id"] == id))
        .cloned()
        .ok_or_else(|| format!("fixture must contain object {id}").into())
}

fn generate_log(
    path: &Path,
    event_count: usize,
    complaints_per_callout: usize,
) -> Result<(usize, usize), Box<dyn Error>> {
    let fixture: Value = serde_json::from_str(FIXTURE)?;
    let template = fixture
        .get("events")
        .and_then(Value::as_array)
        .and_then(|events| events.iter().find(|event| event["id"] == "event-001"))
        .cloned()
        .ok_or("fixture must contain event-001")?;
    let complaint_template = object_template(&fixture, "complaint-1")?;
    let callout_template = object_template(&fixture, "callout-1")?;
    if complaints_per_callout == 0 {
        return Err("complaints per callout must be greater than zero".into());
    }
    let callout_group_count = event_count.div_ceil(complaints_per_callout);

    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }

    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    writer.write_all(b"{\"objectTypes\":")?;
    serde_json::to_writer(&mut writer, &fixture["objectTypes"])?;
    writer.write_all(b",\"eventTypes\":")?;
    serde_json::to_writer(&mut writer, &fixture["eventTypes"])?;
    writer.write_all(b",\"objects\":[")?;
    let base_objects = fixture["objects"]
        .as_array()
        .ok_or("fixture objects must be an array")?;
    let mut first_object = true;
    for object in base_objects {
        if !first_object {
            writer.write_all(b",")?;
        }
        serde_json::to_writer(&mut writer, object)?;
        first_object = false;
    }

    // Keep the first fixture callout as group zero, then add one callout per
    // additional group. Every generated complaint gets its own object identity.
    for group in 1..callout_group_count {
        let mut callout = callout_template.clone();
        callout["id"] = Value::String(format!("scale-callout-{group:012}"));
        if !first_object {
            writer.write_all(b",")?;
        }
        serde_json::to_writer(&mut writer, &callout)?;
        first_object = false;
    }
    for index in 1..=event_count {
        let group = (index - 1) / complaints_per_callout;
        let mut complaint = complaint_template.clone();
        complaint["id"] = Value::String(format!("scale-complaint-{index:012}"));
        complaint["relationships"][0]["objectId"] = if group == 0 {
            Value::String("callout-1".to_owned())
        } else {
            Value::String(format!("scale-callout-{group:012}"))
        };
        set_object_attribute(
            &mut complaint,
            "fault_fingerprint",
            Value::String(format!("scale-fault-{group:012}")),
        )?;
        if !first_object {
            writer.write_all(b",")?;
        }
        serde_json::to_writer(&mut writer, &complaint)?;
        first_object = false;
    }

    writer.write_all(b"],\"events\":[")?;

    for index in 1..=event_count {
        if index > 1 {
            writer.write_all(b",")?;
        }
        let mut event = template.clone();
        let group = (index - 1) / complaints_per_callout;
        let complaint_id = format!("scale-complaint-{index:012}");
        let callout_id = if group == 0 {
            "callout-1".to_owned()
        } else {
            format!("scale-callout-{group:012}")
        };
        event["id"] = Value::String(format!("scale-event-{index:012}"));
        event["relationships"][0]["objectId"] = Value::String(complaint_id);
        event["relationships"][1]["objectId"] = Value::String(callout_id);
        set_attribute(&mut event, "host_sequence", Value::from(index as u64))?;
        set_attribute(
            &mut event,
            "source_ref",
            Value::String(format!("scale-source-{index:012}")),
        )?;
        let source_record = format!("craft-scale-source-record-v1:{index}");
        let source_digest = Sha256::digest(source_record.as_bytes())
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        set_attribute(
            &mut event,
            "source_digest",
            Value::String(format!("sha256:{source_digest}")),
        )?;
        serde_json::to_writer(&mut writer, &event)?;
    }

    writer.write_all(b"]}\n")?;
    writer.flush()?;
    Ok((event_count, callout_group_count))
}

fn sha256_file(path: &Path) -> Result<String, Box<dyn Error>> {
    let mut reader = BufReader::new(File::open(path)?);
    let mut digest = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let read = reader.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        digest.update(&buffer[..read]);
    }
    Ok(digest
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect())
}

fn high_water_rss_kib() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let status = fs::read_to_string("/proc/self/status").ok()?;
        status.lines().find_map(|line| {
            line.strip_prefix("VmHWM:")?
                .split_whitespace()
                .next()?
                .parse()
                .ok()
        })
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

fn percentile(sorted: &[u128], numerator: usize, denominator: usize) -> u128 {
    let rank = (sorted.len() * numerator).div_ceil(denominator).max(1);
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn run(
    event_count: usize,
    output: PathBuf,
    repeats: usize,
    complaints_per_callout: usize,
) -> Result<(), Box<dyn Error>> {
    if event_count == 0 || event_count > MAX_EVENTS {
        return Err(format!("events must be between 1 and {MAX_EVENTS}").into());
    }
    if !(5..=100).contains(&repeats) {
        return Err("repeats must be between 5 and 100".into());
    }

    let (generated_complaints, callout_group_count) =
        generate_log(&output, event_count, complaints_per_callout)?;
    let input_bytes = fs::metadata(&output)?.len();
    let input_sha256 = sha256_file(&output)?;
    let input = fs::read_to_string(&output)?;
    let warmup_cut = TemporalReplayCutV1 {
        valid_time_as_of: "2026-11-27T23:59:59Z".to_owned(),
        known_through_sequence: event_count as u64,
    };
    for _ in 0..WARMUP_RUNS {
        drop(validate_ocel20_water_profile(&input)?);
        drop(
            replay_temporal_frontier(&input, &warmup_cut)
                .map_err(|error| format!("temporal replay warm-up failed: {error:?}"))?,
        );
    }

    let mut elapsed_ns = Vec::with_capacity(repeats);
    let mut first_receipt = None;
    for _ in 0..repeats {
        let started = Instant::now();
        let receipt = validate_ocel20_water_profile(&input)?;
        elapsed_ns.push(started.elapsed().as_nanos());
        if first_receipt.is_none() {
            first_receipt = Some((
                receipt.canonical_sha256,
                receipt.object_type_count,
                receipt.event_type_count,
                receipt.object_count,
                receipt.event_count,
                receipt.event_to_object_relationship_count,
                receipt.object_to_object_relationship_count,
                receipt.canonical_json.len(),
            ));
        }
    }

    let cut = warmup_cut;
    let mut replay_elapsed_ns = Vec::with_capacity(repeats);
    let mut first_replay = None;
    for _ in 0..repeats {
        let started = Instant::now();
        let receipt = replay_temporal_frontier(&input, &cut)
            .map_err(|error| format!("temporal replay failed: {error:?}"))?;
        replay_elapsed_ns.push(started.elapsed().as_nanos());
        if first_replay.is_none() {
            first_replay = Some((
                receipt.replay_sha256,
                receipt.visible_event_ids.len(),
                receipt.effective_event_ids.len(),
            ));
        }
    }

    elapsed_ns.sort_unstable();
    replay_elapsed_ns.sort_unstable();
    let p50 = percentile(&elapsed_ns, 50, 100);
    let p95 = percentile(&elapsed_ns, 95, 100);
    let replay_p50 = percentile(&replay_elapsed_ns, 50, 100);
    let replay_p95 = percentile(&replay_elapsed_ns, 95, 100);
    let (
        canonical_sha256,
        object_type_count,
        event_type_count,
        object_count,
        event_count,
        e2o_count,
        o2o_count,
        canonical_bytes,
    ) = first_receipt.ok_or("benchmark produced no validation receipt")?;
    let (replay_sha256, visible_event_count, effective_event_count) =
        first_replay.ok_or("benchmark produced no replay receipt")?;
    let seconds = p50 as f64 / 1_000_000_000.0;
    let replay_seconds = replay_p50 as f64 / 1_000_000_000.0;
    let events_per_second = if seconds > 0.0 {
        event_count as f64 / seconds
    } else {
        0.0
    };
    let replay_events_per_second = if replay_seconds > 0.0 {
        visible_event_count as f64 / replay_seconds
    } else {
        0.0
    };

    let report = serde_json::json!({
        "benchmark": "craft_ocel_profile_and_temporal_replay_v1",
        "generator_profile": "unique complaint objects grouped into callouts with configurable fan-out",
        "generated_complaint_object_count": generated_complaints,
        "callout_group_count": callout_group_count,
        "complaints_per_callout": complaints_per_callout,
        "gate": "measurement_only",
        "event_count": event_count,
        "object_count": object_count,
        "event_type_count": event_type_count,
        "object_type_count": object_type_count,
        "event_to_object_relationship_count": e2o_count,
        "object_to_object_relationship_count": o2o_count,
        "input_bytes": input_bytes,
        "canonical_bytes": canonical_bytes,
        "input_sha256": input_sha256,
        "canonical_sha256": canonical_sha256,
        "replay_sha256": replay_sha256,
        "replay_valid_time_as_of": cut.valid_time_as_of,
        "replay_known_through_sequence": cut.known_through_sequence,
        "replay_visible_event_count": visible_event_count,
        "replay_effective_event_count": effective_event_count,
        "warmup_runs": WARMUP_RUNS,
        "measured_repeats": repeats,
        "validation_plus_canonicalization_p50_ns": p50,
        "validation_plus_canonicalization_p95_ns": p95,
        "events_per_second_at_p50": events_per_second,
        "temporal_replay_p50_ns": replay_p50,
        "temporal_replay_p95_ns": replay_p95,
        "temporal_replay_events_per_second_at_p50": replay_events_per_second,
        "peak_rss_kib": high_water_rss_kib(),
        "operating_system": std::env::consts::OS,
        "architecture": std::env::consts::ARCH,
        "claim_boundary": "Synthetic profile validation, canonicalization and replay only; no OCEL general-conformance, source authenticity, owner-core, process-discovery or field-effectiveness claim."
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 || args.len() > 4 {
        eprintln!("usage: ocel-scale-bench EVENTS OUTPUT_PATH [REPEATS] [COMPLAINTS_PER_CALLOUT]");
        std::process::exit(2);
    }

    let event_count = match args[0].parse::<usize>() {
        Ok(value) => value,
        Err(error) => {
            eprintln!("invalid event count: {error}");
            std::process::exit(2);
        }
    };
    let output = PathBuf::from(&args[1]);
    let repeats = match args.get(2).map(|value| value.parse::<usize>()).transpose() {
        Ok(value) => value.unwrap_or(20),
        Err(error) => {
            eprintln!("invalid repeat count: {error}");
            std::process::exit(2);
        }
    };
    let complaints_per_callout = match args.get(3).map(|value| value.parse::<usize>()).transpose() {
        Ok(value) => value.unwrap_or(100),
        Err(error) => {
            eprintln!("invalid complaints-per-callout value: {error}");
            std::process::exit(2);
        }
    };

    if let Err(error) = run(event_count, output, repeats, complaints_per_callout) {
        eprintln!("OCEL scale benchmark failed: {error}");
        std::process::exit(1);
    }
}
