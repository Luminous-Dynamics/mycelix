use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

const VERSION: &str = env!("CARGO_PKG_VERSION");

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn get_path<'a>(value: &'a Value, path: &str) -> Option<&'a Value> {
    let mut current = value;
    for part in path.split('.') {
        current = current.as_object()?.get(part)?;
    }
    Some(current)
}

fn set_path(value: &mut Value, path: &str, replacement: Value) -> Result<(), String> {
    let mut parts = path.split('.').peekable();
    let mut current = value;
    while let Some(part) = parts.next() {
        if parts.peek().is_none() {
            let object = current
                .as_object_mut()
                .ok_or_else(|| format!("path parent is not an object: {path}"))?;
            if !object.contains_key(part) {
                return Err(format!("path does not exist: {path}"));
            }
            object.insert(part.to_owned(), replacement);
            return Ok(());
        }
        current = current
            .as_object_mut()
            .and_then(|object| object.get_mut(part))
            .ok_or_else(|| format!("path does not exist: {path}"))?;
    }
    Err(format!("empty mutation path: {path}"))
}

fn string_at<'a>(value: &'a Value, path: &str) -> Option<&'a str> {
    get_path(value, path)?.as_str()
}

fn integer_at(value: &Value, path: &str) -> Option<i64> {
    get_path(value, path)?.as_i64()
}

fn utc_timestamp_key(value: &str) -> Option<(i64, u32)> {
    let (date, time_with_zone) = value.split_once('T')?;
    let time = time_with_zone.strip_suffix('Z')?;
    if date.len() != 10 {
        return None;
    }
    let mut date_parts = date.split('-');
    let year = date_parts.next()?.parse::<i64>().ok()?;
    let month = date_parts.next()?.parse::<i64>().ok()?;
    let day = date_parts.next()?.parse::<i64>().ok()?;
    if date_parts.next().is_some() || !(1..=12).contains(&month) {
        return None;
    }

    let (clock, fraction) = match time.split_once('.') {
        Some((clock, fraction)) => (clock, Some(fraction)),
        None => (time, None),
    };
    if clock.len() != 8 {
        return None;
    }
    let mut clock_parts = clock.split(':');
    let hour = clock_parts.next()?.parse::<i64>().ok()?;
    let minute = clock_parts.next()?.parse::<i64>().ok()?;
    let second = clock_parts.next()?.parse::<i64>().ok()?;
    if clock_parts.next().is_some()
        || !(0..=23).contains(&hour)
        || !(0..=59).contains(&minute)
        || !(0..=59).contains(&second)
    {
        return None;
    }

    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let month_days = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let max_day = *month_days.get((month - 1) as usize)?;
    if day < 1 || day > max_day {
        return None;
    }

    // Howard Hinnant's civil-date conversion; output is days since 1970-01-01.
    let adjusted_year = year - if month <= 2 { 1 } else { 0 };
    let era = adjusted_year.div_euclid(400);
    let year_of_era = adjusted_year - era * 400;
    let adjusted_month = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    let days = era * 146_097 + day_of_era - 719_468;
    let seconds = days * 86_400 + hour * 3_600 + minute * 60 + second;

    let nanos = match fraction {
        None => 0,
        Some(raw) => {
            if raw.is_empty() || raw.len() > 9 || !raw.bytes().all(|byte| byte.is_ascii_digit()) {
                return None;
            }
            let significant = raw;
            let parsed = significant.parse::<u32>().ok()?;
            parsed * 10_u32.pow((9 - significant.len()) as u32)
        }
    };
    Some((seconds, nanos))
}

fn compare_utc(left: &str, right: &str) -> Option<std::cmp::Ordering> {
    Some(utc_timestamp_key(left)?.cmp(&utc_timestamp_key(right)?))
}

fn target_binding_matches(reference: &Value, context: &Value) -> bool {
    const FIELDS: &[&str] = &[
        "energy_project_id",
        "project_revision_ref",
        "site_id",
        "subject_id",
        "boundary_id",
        "boundary_profile",
        "point_of_connection_id",
    ];
    FIELDS.iter().all(|field| {
        get_path(reference, &format!("project_binding.{field}"))
            == get_path(context, &format!("target_project_binding.{field}"))
    })
}

fn evaluate_reference(reference: &Value, context: &Value) -> (&'static str, &'static str) {
    if string_at(reference, "evidence_class") == Some("unassessed_legacy_locator") {
        return (
            "unassessed_legacy_locator",
            "legacy locator has no canonical authority or receipt",
        );
    }

    match string_at(reference, "lifecycle.state") {
        Some("superseded") => {
            return (
                "historical_only",
                "record is retained but no longer current",
            );
        }
        Some("retracted") | Some("expired") => {
            return ("blocked", "record is retracted or expired");
        }
        Some("disputed") => return ("indeterminate", "record is disputed"),
        Some("current") => {}
        _ => return ("indeterminate", "lifecycle state is missing or unsupported"),
    }

    if !target_binding_matches(reference, context) {
        return (
            "blocked",
            "project, site, subject, boundary or point-of-connection mismatch",
        );
    }

    let proposition_id = match string_at(reference, "claim_binding.proposition_id") {
        Some(value) => value,
        None => return ("indeterminate", "claim proposition identity is missing"),
    };
    let issuer_role = match string_at(reference, "issuer_profile.issuer_role") {
        Some(value) => value,
        None => return ("indeterminate", "issuer role is missing"),
    };
    let required_roles = match get_path(
        context,
        &format!("required_issuer_roles_by_proposition.{proposition_id}"),
    )
    .and_then(Value::as_array)
    {
        Some(roles) => roles,
        None => {
            return (
                "indeterminate",
                "no jurisdiction/claim authority profile is available",
            );
        }
    };
    if !required_roles
        .iter()
        .any(|role| role.as_str() == Some(issuer_role))
    {
        return (
            "blocked",
            "issuer role is not authorized for this proposition type",
        );
    }

    let receipt_time = match string_at(
        reference,
        "canonical_desci_reference.append_receipt.received_at",
    ) {
        Some(value) => value,
        None => {
            return (
                "indeterminate",
                "canonical append receipt time is unavailable",
            );
        }
    };
    let frontier = match string_at(context, "historical_decision_frontier") {
        Some(value) => value,
        None => {
            return (
                "indeterminate",
                "historical decision frontier is unavailable",
            );
        }
    };
    match compare_utc(receipt_time, frontier) {
        Some(std::cmp::Ordering::Greater) => {
            return (
                "blocked_for_historical_frontier",
                "canonical server receipt is after the decision frontier",
            );
        }
        Some(_) => {}
        None => {
            return (
                "indeterminate",
                "UTC receipt/frontier timestamp cannot be compared",
            );
        }
    }
    if string_at(reference, "temporal_scope.known_at") != Some(receipt_time) {
        return (
            "blocked",
            "known_at is not anchored to canonical server receipt time",
        );
    }

    let valid_from = match string_at(reference, "temporal_scope.valid_from") {
        Some(value) => value,
        None => return ("indeterminate", "evidence valid_from is missing"),
    };
    let valid_until = match string_at(reference, "temporal_scope.valid_until") {
        Some(value) => value,
        None => return ("indeterminate", "evidence valid_until is missing"),
    };
    if compare_utc(valid_from, valid_until) == Some(std::cmp::Ordering::Greater) {
        return ("blocked", "evidence validity interval is inverted");
    }
    match compare_utc(valid_from, frontier) {
        Some(std::cmp::Ordering::Greater) => {
            return (
                "blocked_not_valid_at_decision",
                "evidence valid_from is after the decision timestamp",
            );
        }
        Some(_) => {}
        None => return ("indeterminate", "evidence valid_from cannot be compared"),
    }
    match compare_utc(frontier, valid_until) {
        Some(std::cmp::Ordering::Greater) => {
            return (
                "blocked_not_valid_at_decision",
                "evidence valid_until is before the decision timestamp",
            );
        }
        Some(_) => {}
        None => return ("indeterminate", "evidence valid_until cannot be compared"),
    }

    if string_at(
        reference,
        "canonical_desci_reference.evidence_attached_event_id",
    ) != string_at(
        reference,
        "canonical_desci_reference.append_receipt.event_id",
    ) || string_at(reference, "canonical_desci_reference.stream_id")
        != string_at(
            reference,
            "canonical_desci_reference.append_receipt.stream_id",
        )
        || integer_at(reference, "canonical_desci_reference.stream_sequence")
            != integer_at(
                reference,
                "canonical_desci_reference.append_receipt.sequence",
            )
        || string_at(
            reference,
            "canonical_desci_reference.evidence_attached_event_hash",
        ) != string_at(
            reference,
            "canonical_desci_reference.append_receipt.event_hash",
        )
    {
        return (
            "blocked",
            "canonical event and append receipt identities disagree",
        );
    }

    if string_at(reference, "canonical_desci_reference.artifact_content_hash")
        != string_at(reference, "provenance.artifact_bytes_hash")
    {
        return (
            "blocked",
            "artifact content digest and provenance digest disagree",
        );
    }

    if string_at(reference, "verification_receipt.result") != Some("verified")
        || string_at(reference, "verification_receipt.signature_status") != Some("valid")
        || string_at(reference, "verification_receipt.authority_status") != Some("authorized")
        || string_at(reference, "verification_receipt.scope_status") != Some("matches")
        || string_at(reference, "verification_receipt.temporal_status") != Some("valid")
    {
        return (
            "blocked",
            "verification receipt does not satisfy required signature/authority/scope/time checks",
        );
    }

    if string_at(reference, "access_policy.access_decision") != Some("permitted") {
        return ("blocked", "access policy does not permit this evidence use");
    }
    let artifact_availability = string_at(
        reference,
        "canonical_desci_reference.artifact_availability",
    );
    let policy_availability = string_at(reference, "access_policy.availability");
    match (artifact_availability, policy_availability) {
        (Some("unavailable"), _) | (_, Some("unavailable")) => {
            return ("indeterminate", "artifact or authorized resolver is unavailable");
        }
        (Some("public"), Some("public"))
        | (Some("controlled"), Some("controlled"))
        | (Some("embargoed"), Some("embargoed")) => {}
        (Some(_), Some(_)) => {
            return ("blocked", "artifact availability and access-policy availability disagree");
        }
        _ => return ("indeterminate", "artifact availability is unknown or unavailable"),
    }

    (
        "eligible_for_policy_evaluation",
        "declared identity, role, temporal, event, digest, verification and access checks passed",
    )
}

fn evaluate_source_family_group(group: &Value) -> (&'static str, usize, usize) {
    let ids = match group.get("source_family_ids").and_then(Value::as_array) {
        Some(ids) if !ids.is_empty() => ids,
        _ => return ("indeterminate", 0, 0),
    };
    let mut unique = std::collections::BTreeSet::new();
    for id in ids {
        match id.as_str() {
            Some(value) if !value.is_empty() => {
                unique.insert(value.to_owned());
            }
            _ => return ("indeterminate", 0, ids.len()),
        }
    }
    let unique_count = unique.len();
    if unique_count < ids.len() {
        (
            "eligible_but_not_independent_corroboration",
            unique_count,
            ids.len(),
        )
    } else {
        ("independent_source_families", unique_count, ids.len())
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let energy_root = manifest_dir.join("../..");
    let docs = energy_root.join("docs");
    let schema_path = docs.join("project-evidence-reference-v1.schema.json");
    let fixtures_path = docs.join("project-evidence-reference-v1.fixtures.json");

    let schema_bytes = fs::read(&schema_path)?;
    let fixtures_bytes = fs::read(&fixtures_path)?;
    let schema: Value = serde_json::from_slice(&schema_bytes)?;
    let fixtures: Value = serde_json::from_slice(&fixtures_bytes)?;
    let source_commit = env::var("ENERGY_EVIDENCE_SOURCE_COMMIT")?;

    if schema.get("$schema").and_then(Value::as_str)
        != Some("https://json-schema.org/draft/2020-12/schema")
    {
        return Err("schema does not declare Draft 2020-12".into());
    }

    let context = fixtures
        .get("semantic_context")
        .ok_or("fixture has no semantic_context")?;
    let canonical = fixtures
        .get("valid_reference_example")
        .ok_or("fixture has no valid_reference_example")?;
    let legacy = fixtures
        .get("legacy_reference_example")
        .ok_or("fixture has no legacy_reference_example")?;

    let (base_disposition, base_reason) = evaluate_reference(canonical, context);
    if base_disposition != "eligible_for_policy_evaluation" {
        return Err(format!(
            "canonical example did not qualify: {base_disposition}: {base_reason}"
        )
        .into());
    }

    let (legacy_disposition, legacy_reason) = evaluate_reference(legacy, context);
    if legacy_disposition != "unassessed_legacy_locator" {
        return Err(format!(
            "legacy example disposition incorrect: {legacy_disposition}: {legacy_reason}"
        )
        .into());
    }

    let mutations = fixtures
        .get("mutations")
        .and_then(Value::as_array)
        .ok_or("fixture mutations must be an array")?;
    let mut seen = std::collections::BTreeSet::new();
    let mut results = Vec::new();
    for mutation in mutations {
        let case_id = mutation
            .get("case_id")
            .and_then(Value::as_str)
            .ok_or("mutation case_id is missing")?;
        if !seen.insert(case_id.to_owned()) {
            return Err(format!("duplicate mutation case_id: {case_id}").into());
        }
        let path = mutation
            .get("path")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("mutation path missing: {case_id}"))?;
        let replacement = mutation
            .get("replacement")
            .cloned()
            .ok_or_else(|| format!("mutation replacement missing: {case_id}"))?;
        let expected = mutation
            .get("expected_disposition")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("mutation expected disposition missing: {case_id}"))?;

        let mut mutated = canonical.clone();
        set_path(&mut mutated, path, replacement).map_err(|error| format!("{case_id}: {error}"))?;
        let (actual, reason) = evaluate_reference(&mutated, context);
        let passed = actual == expected;
        results.push(json!({
            "case_id": case_id,
            "expected_disposition": expected,
            "actual_disposition": actual,
            "passed": passed,
            "reason": reason
        }));
        if !passed {
            return Err(format!(
                "mutation {case_id} mismatch: expected {expected}, got {actual} ({reason})"
            )
            .into());
        }
    }

    if seen.len() != mutations.len() {
        return Err(format!(
            "mutation execution census mismatch: executed {}, declared {}",
            seen.len(),
            mutations.len()
        )
        .into());
    }

    let source_family_group = fixtures
        .get("source_family_group_example")
        .ok_or("fixture has no source_family_group_example")?;
    let (source_family_disposition, independent_family_count, source_reference_count) =
        evaluate_source_family_group(source_family_group);
    let source_family_expected = source_family_group
        .get("expected_disposition")
        .and_then(Value::as_str)
        .ok_or("source-family example has no expected disposition")?;
    if source_family_disposition != source_family_expected {
        return Err(format!(
            "source-family disposition mismatch: expected {source_family_expected}, got {source_family_disposition}"
        )
        .into());
    }

    let mutation_dispositions: std::collections::BTreeMap<String, String> = results
        .iter()
        .filter_map(|result| {
            Some((
                result.get("case_id")?.as_str()?.to_owned(),
                result.get("actual_disposition")?.as_str()?.to_owned(),
            ))
        })
        .collect();
    let summary_cases = fixtures
        .get("cases")
        .and_then(Value::as_array)
        .ok_or("summary cases must be an array")?;
    let mut summary_results = Vec::new();
    for case in summary_cases {
        let case_id = case
            .get("case_id")
            .and_then(Value::as_str)
            .ok_or("summary case_id is missing")?;
        let expected = case
            .get("expected_disposition")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("summary expected disposition missing: {case_id}"))?;
        let detailed_actual = match case_id {
            "valid-reference-001" => base_disposition,
            "legacy-url-002" => legacy_disposition,
            "wrong-boundary-003" => mutation_dispositions
                .get("wrong-boundary")
                .map(String::as_str)
                .ok_or("wrong-boundary mutation result missing")?,
            "future-known-004" => mutation_dispositions
                .get("future-known")
                .map(String::as_str)
                .ok_or("future-known mutation result missing")?,
            "unauthorized-signer-005" => mutation_dispositions
                .get("unauthorized-role")
                .map(String::as_str)
                .ok_or("unauthorized-role mutation result missing")?,
            "superseded-evidence-006" => mutation_dispositions
                .get("superseded")
                .map(String::as_str)
                .ok_or("superseded mutation result missing")?,
            "copied-source-family-007" => source_family_disposition,
            "embargoed-public-view-008" => mutation_dispositions
                .get("public-embargo-leak")
                .map(String::as_str)
                .ok_or("public-embargo-leak mutation result missing")?,
            _ => return Err(format!("unknown summary case: {case_id}").into()),
        };
        let actual = if detailed_actual.starts_with("blocked") {
            "blocked"
        } else {
            detailed_actual
        };
        let passed = actual == expected;
        if !passed {
            return Err(format!(
                "summary case {case_id} mismatch: expected {expected}, got {actual}"
            )
            .into());
        }
        summary_results.push(json!({
            "case_id": case_id,
            "expected_disposition": expected,
            "actual_disposition": actual,
            "detailed_actual_disposition": detailed_actual,
            "passed": passed
        }));
    }


    let output = json!({
        "receipt_version": "2",
        "source_commit": source_commit,
        "evaluator": format!("energy-evidence-conformance/{VERSION}"),
        "scope": "declared_fixture_semantics_and_source_family_census",
        "canonical_example": {
            "actual_disposition": base_disposition,
            "reason": base_reason
        },
        "legacy_example": {
            "actual_disposition": legacy_disposition,
            "reason": legacy_reason
        },
        "mutation_results": results,
        "mutation_count": seen.len(),
        "source_family_result": {
            "expected_disposition": source_family_expected,
            "actual_disposition": source_family_disposition,
            "independent_family_count": independent_family_count,
            "reference_count": source_reference_count
        },
        "summary_case_results": summary_results,
        "summary_cases_not_semantically_executed": 0,
        "schema_sha256": sha256_hex(&schema_bytes),
        "fixtures_sha256": sha256_hex(&fixtures_bytes),
        "result": "pass"
    });

    let output_path = env::args()
        .skip_while(|arg| arg != "--output")
        .nth(1)
        .ok_or("usage: energy-evidence-conformance --output PATH")?;
    let output_path = Path::new(&output_path);
    if let Some(parent) = output_path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(output_path, serde_json::to_vec_pretty(&output)?)?;
    println!("{}", serde_json::to_string(&output)?);
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("energy-evidence-conformance: FAIL: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load_fixtures() -> Value {
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let fixtures_path = manifest_dir
            .join("../..")
            .join("docs")
            .join("project-evidence-reference-v1.fixtures.json");
        let bytes = fs::read(fixtures_path).expect("fixtures must be readable");
        serde_json::from_slice(&bytes).expect("fixtures must be valid JSON")
    }

    #[test]
    fn canonical_and_legacy_examples_have_distinct_dispositions() {
        let fixtures = load_fixtures();
        let context = &fixtures["semantic_context"];
        assert_eq!(
            evaluate_reference(&fixtures["valid_reference_example"], context).0,
            "eligible_for_policy_evaluation"
        );
        assert_eq!(
            evaluate_reference(&fixtures["legacy_reference_example"], context).0,
            "unassessed_legacy_locator"
        );
    }

    #[test]
    fn every_declared_mutation_is_executed_and_matches_its_disposition() {
        let fixtures = load_fixtures();
        let context = &fixtures["semantic_context"];
        let canonical = &fixtures["valid_reference_example"];
        for mutation in fixtures["mutations"].as_array().expect("mutations array") {
            let mut changed = canonical.clone();
            set_path(
                &mut changed,
                mutation["path"].as_str().expect("mutation path"),
                mutation["replacement"].clone(),
            )
            .expect("mutation path must resolve");
            let actual = evaluate_reference(&changed, context).0;
            let expected = mutation["expected_disposition"]
                .as_str()
                .expect("expected disposition");
            assert_eq!(actual, expected, "mutation case {}", mutation["case_id"]);
        }
    }

    #[test]
    fn validity_window_and_artifact_availability_are_decision_time_gates() {
        let fixtures = load_fixtures();
        let context = &fixtures["semantic_context"];
        let canonical = &fixtures["valid_reference_example"];

        let mut not_yet_valid = canonical.clone();
        set_path(
            &mut not_yet_valid,
            "temporal_scope.valid_from",
            Value::String("2026-08-01T00:00:00Z".to_owned()),
        )
        .expect("valid_from path");
        assert_eq!(
            evaluate_reference(&not_yet_valid, context).0,
            "blocked_not_valid_at_decision"
        );

        let mut expired = canonical.clone();
        set_path(
            &mut expired,
            "temporal_scope.valid_until",
            Value::String("2026-06-30T23:59:59Z".to_owned()),
        )
        .expect("valid_until path");
        assert_eq!(
            evaluate_reference(&expired, context).0,
            "blocked_not_valid_at_decision"
        );

        let mut unavailable = canonical.clone();
        set_path(
            &mut unavailable,
            "canonical_desci_reference.artifact_availability",
            Value::String("unavailable".to_owned()),
        )
        .expect("availability path");
        assert_eq!(evaluate_reference(&unavailable, context).0, "indeterminate");
    }

    #[test]
    fn copied_documents_do_not_count_as_independent_source_families() {
        let fixtures = load_fixtures();
        let (disposition, unique_families, references) =
            evaluate_source_family_group(&fixtures["source_family_group_example"]);
        assert_eq!(
            disposition,
            "eligible_but_not_independent_corroboration"
        );
        assert_eq!(unique_families, 1);
        assert_eq!(references, 2);
    }

    #[test]
    fn utc_frontier_comparison_handles_calendar_rollover() {
        assert_eq!(
            compare_utc("2026-08-01T00:00:00Z", "2026-07-01T00:00:00Z"),
            Some(std::cmp::Ordering::Greater)
        );
        assert_eq!(
            compare_utc("2026-06-03T09:00:00Z", "2026-07-01T00:00:00Z"),
            Some(std::cmp::Ordering::Less)
        );
        assert_eq!(compare_utc("not-a-time", "2026-07-01T00:00:00Z"), None);
    }
}
