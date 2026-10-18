use jsonschema::validator_for;
use serde::Serialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

const DIALECT: &str = "https://json-schema.org/draft/2020-12/schema";
const VALIDATOR: &str = "jsonschema@0.59.0";
const REPORT_VERSION: &str = "1.0.0";
const CLAIM_CEILING: &str = "Local fixture/schema/corpus consistency only. Does not prove source authenticity, user comprehension, UI behavior, accessibility conformance, production synchronization correctness, agronomic safety, completed service, or field impact.";

const CASES: &str = "research/farmer-ux-001-cases-v1.json";
const CASES_SCHEMA: &str = "research/farmer-ux-001-cases-v1.schema.json";
const ROLES: &str = "research/farmer-ux-001-role-task-matrix-v1.json";
const ROLES_SCHEMA: &str = "research/farmer-ux-001-role-task-matrix-v1.schema.json";
const COPY: &str = "research/farmer-ux-001-copy-v1.json";
const COPY_SCHEMA: &str = "research/farmer-ux-001-copy-v1.schema.json";

const REQUIRED_CASES: &[&str] = &[
    "FUX-OFFLINE-001", "FUX-QUOTE-002", "FUX-QUOTE-003", "FUX-SYNC-004",
    "FUX-ROUTE-005", "FUX-NOPROVIDER-006", "FUX-COMPLETION-007",
    "FUX-REWORK-008", "FUX-EVIDENCE-009", "FUX-CONSENT-010",
    "FUX-HUMAN-011", "FUX-PAYMENT-012", "FUX-AUTH-013", "FUX-SCOPE-014",
    "FUX-ROUTE-015", "FUX-PRIVACY-016", "FUX-DIGEST-017",
    "FUX-LANGUAGE-018", "FUX-CAPACITY-019", "FUX-EXPORT-020",
    "FUX-DEFER-021", "FUX-NOSHOW-022",
];
const REQUIRED_ROLES: &[&str] = &[
    "farmer_smallholder_crop", "farmer_mixed_or_livestock",
    "extension_officer_agronomist", "irrigation_water_service_provider",
    "farm_machinery_operator_or_coordinator", "producer_cooperative_or_food_coordinator",
    "solar_energy_and_pump_technician", "construction_repair_trade",
    "circular_economy_operator",
];
const REQUIRED_COPY: &[&str] = &[
    "local_saved", "sync_pending", "sync_confirmed", "sync_conflict",
    "evidence_stale", "evidence_conflict", "evidence_unknown", "modelled_route",
    "no_route_evidence", "no_provider", "quote_incomplete", "quote_expired",
    "booking_confirmed", "provider_reported_complete", "service_accepted",
    "service_disputed", "permission_revoked", "human_mediation",
];
const REQUIRED_INVARIANTS: &[&str] = &[
    "unknown_is_not_zero_or_verified",
    "modelled_is_not_observed",
    "request_is_not_quote",
    "quote_is_not_booking",
    "booking_is_not_completed_work",
    "provider_completion_is_not_farmer_acceptance",
    "locally_acknowledged_event_must_not_be_lost",
    "duplicate_replay_must_not_duplicate_side_effects",
    "conflicting_commitments_must_not_be_silently_overwritten",
    "recommendation_does_not_authorize_payment_procurement_dispatch_or_actuation",
    "human_mediated_channel_must_preserve_request_reference",
];

#[derive(Debug, Serialize)]
struct ArtifactReport {
    name: String,
    instance_path: String,
    schema_path: String,
    instance_sha256: String,
    schema_sha256: String,
    instance_bytes: usize,
    schema_bytes: usize,
    schema_dialect: String,
    schema_status: String,
    schema_errors: Vec<String>,
}
#[derive(Debug, Serialize)]
struct Counts {
    cases: usize,
    roles: usize,
    copy_states: usize,
    shared_invariants: usize,
}
#[derive(Debug, Serialize)]
struct Report {
    report_version: String,
    tool_version: String,
    validator: String,
    status: String,
    artifacts: Vec<ArtifactReport>,
    counts: Counts,
    schema_error_count: usize,
    semantic_error_count: usize,
    errors: Vec<String>,
    claim_ceiling: String,
}
struct Artifact {
    name: &'static str,
    instance_path: &'static str,
    schema_path: &'static str,
    instance_bytes: Vec<u8>,
    schema_bytes: Vec<u8>,
    instance: Value,
    schema: Value,
    schema_errors: Vec<String>,
}

fn sha256(bytes: &[u8]) -> String {
    Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}
fn read_json(root: &Path, relative: &str) -> Result<(Vec<u8>, Value), String> {
    let path = root.join(relative);
    let bytes = fs::read(&path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let value = serde_json::from_slice(&bytes).map_err(|e| format!("cannot parse {}: {e}", path.display()))?;
    Ok((bytes, value))
}
fn validate_schema(name: &str, instance: &Value, schema: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    match schema.get("$schema").and_then(Value::as_str) {
        Some(DIALECT) => {}
        Some(other) => errors.push(format!("{name}: unsupported schema dialect {other:?}; expected {DIALECT}")),
        None => errors.push(format!("{name}: missing $schema dialect identifier")),
    }
    match validator_for(schema) {
        Ok(validator) => {
            for error in validator.iter_errors(instance) {
                errors.push(format!("{name}: schema violation at {}: {error}", error.instance_path()));
            }
        }
        Err(error) => errors.push(format!("{name}: schema compilation failed: {error}")),
    }
    errors.sort();
    errors
}
fn load(root: &Path, name: &'static str, instance_path: &'static str, schema_path: &'static str) -> Result<Artifact, String> {
    let (instance_bytes, instance) = read_json(root, instance_path)?;
    let (schema_bytes, schema) = read_json(root, schema_path)?;
    let schema_errors = validate_schema(name, &instance, &schema);
    Ok(Artifact { name, instance_path, schema_path, instance_bytes, schema_bytes, instance, schema, schema_errors })
}
fn array(doc: &Value, key: &str) -> &[Value] {
    doc.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}
fn ids(doc: &Value, collection: &str, field: &str, label: &str, errors: &mut Vec<String>) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for (idx, item) in array(doc, collection).iter().enumerate() {
        let Some(id) = item.get(field).and_then(Value::as_str) else {
            errors.push(format!("{label}[{idx}]: missing string {field}"));
            continue;
        };
        if !out.insert(id.to_owned()) {
            errors.push(format!("{label}: duplicate {field} {id:?}"));
        }
    }
    out
}
fn require_ids(found: &BTreeSet<String>, required: &[&str], label: &str, errors: &mut Vec<String>) {
    for id in required {
        if !found.contains(*id) {
            errors.push(format!("{label}: required v1 entry is missing: {id}"));
        }
    }
}
fn string_set(value: Option<&Value>) -> BTreeSet<String> {
    value.and_then(Value::as_array).into_iter().flatten()
        .filter_map(Value::as_str).map(str::to_owned).collect()
}
fn find<'a>(doc: &'a Value, collection: &str, key: &str, wanted: &str) -> Option<&'a Value> {
    array(doc, collection).iter().find(|v| v.get(key).and_then(Value::as_str) == Some(wanted))
}
fn check_count(doc: &Value, case_id: &str, field: &str, wanted: i64, errors: &mut Vec<String>) {
    let path = format!("/expected/side_effect_counts/{field}");
    let found = find(doc, "cases", "id", case_id).and_then(|v| v.pointer(&path)).and_then(Value::as_i64);
    if found != Some(wanted) {
        errors.push(format!("{case_id}: expected side_effect_counts.{field}={wanted}, found {found:?}"));
    }
}
fn semantic_checks(cases: &Value, roles: &Value, copy: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    let case_ids = ids(cases, "cases", "id", "cases", &mut errors);
    let role_ids = ids(roles, "roles", "id", "roles", &mut errors);
    let copy_keys = ids(copy, "states", "key", "copy states", &mut errors);
    require_ids(&case_ids, REQUIRED_CASES, "cases", &mut errors);
    require_ids(&role_ids, REQUIRED_ROLES, "roles", &mut errors);
    require_ids(&copy_keys, REQUIRED_COPY, "copy states", &mut errors);

    let invariants = string_set(cases.get("shared_invariants"));
    for invariant in REQUIRED_INVARIANTS {
        if !invariants.contains(*invariant) {
            errors.push(format!("shared_invariants: missing required invariant {invariant}"));
        }
    }

    for (idx, case) in array(cases, "cases").iter().enumerate() {
        let label = case.get("id").and_then(Value::as_str).unwrap_or("<missing-id>");
        let show = string_set(case.pointer("/expected/must_show"));
        let not_show = string_set(case.pointer("/expected/must_not_show"));
        for overlap in show.intersection(&not_show) {
            errors.push(format!("{label}: contradictory visibility contract for {overlap:?}"));
        }
        if show.is_empty() {
            errors.push(format!("cases[{idx}] {label}: must_show must not be empty"));
        }
        if string_set(case.pointer("/expected/invariant_checks")).is_empty() {
            errors.push(format!("cases[{idx}] {label}: invariant_checks must not be empty"));
        }
    }

    let expected: &[(&str, &str, i64)] = &[
        ("FUX-OFFLINE-001", "bookings", 0), ("FUX-OFFLINE-001", "payments", 0),
        ("FUX-QUOTE-002", "bookings", 0), ("FUX-QUOTE-002", "payments", 0),
        ("FUX-QUOTE-003", "bookings", 0), ("FUX-QUOTE-003", "payments", 0),
        ("FUX-SYNC-004", "duplicate_bookings", 0), ("FUX-ROUTE-005", "bookings", 0),
        ("FUX-NOPROVIDER-006", "bookings", 0), ("FUX-NOPROVIDER-006", "payments", 0),
        ("FUX-COMPLETION-007", "accepted_services", 0), ("FUX-COMPLETION-007", "payments", 0),
        ("FUX-REWORK-008", "accepted_services", 0), ("FUX-CONSENT-010", "new_shares_after_revocation", 0),
        ("FUX-PAYMENT-012", "payments", 0), ("FUX-AUTH-013", "service_requests", 0),
        ("FUX-AUTH-013", "bookings", 0), ("FUX-AUTH-013", "payments", 0),
        ("FUX-ROUTE-015", "bookings", 0), ("FUX-PRIVACY-016", "precise_location_disclosures", 0),
        ("FUX-DIGEST-017", "verified_source_claims", 0), ("FUX-CAPACITY-019", "bookings", 0),
        ("FUX-CAPACITY-019", "payments", 0), ("FUX-EXPORT-020", "events_silently_omitted", 0),
        ("FUX-DEFER-021", "unauthorized_actions", 0), ("FUX-NOSHOW-022", "accepted_services", 0),
        ("FUX-NOSHOW-022", "request_denominator_entries", 1),
    ];
    for (case_id, field, wanted) in expected {
        check_count(cases, case_id, field, *wanted, &mut errors);
    }

    for id in ["farmer_smallholder_crop", "extension_officer_agronomist", "irrigation_water_service_provider"] {
        let priority = find(roles, "roles", "id", id).and_then(|v| v.get("priority")).and_then(Value::as_str);
        if priority != Some("P0") {
            errors.push(format!("{id}: expected initial priority P0, found {priority:?}"));
        }
    }
    errors.sort();
    errors.dedup();
    errors
}
fn artifact_report(a: &Artifact) -> ArtifactReport {
    let dialect = a.schema.get("$schema").and_then(Value::as_str).unwrap_or("missing").to_owned();
    ArtifactReport {
        name: a.name.to_owned(),
        instance_path: a.instance_path.to_owned(),
        schema_path: a.schema_path.to_owned(),
        instance_sha256: sha256(&a.instance_bytes),
        schema_sha256: sha256(&a.schema_bytes),
        instance_bytes: a.instance_bytes.len(),
        schema_bytes: a.schema_bytes.len(),
        schema_dialect: dialect,
        schema_status: if a.schema_errors.is_empty() { "PASS" } else { "FAIL" }.to_owned(),
        schema_errors: a.schema_errors.clone(),
    }
}
fn default_root() -> Result<PathBuf, String> {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().and_then(Path::parent).map(Path::to_path_buf)
        .ok_or_else(|| "cannot derive repository root from CARGO_MANIFEST_DIR; use --root".to_owned())
}
fn parse_args() -> Result<(PathBuf, Option<PathBuf>, bool), String> {
    let mut root = None;
    let mut report_path = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--root" => root = Some(PathBuf::from(args.next().ok_or("--root requires a path")?)),
            "--report-json" => report_path = Some(PathBuf::from(args.next().ok_or("--report-json requires a path")?)),
            "--help" | "-h" => return Ok((PathBuf::new(), None, true)),
            _ => return Err(format!("unknown argument {arg:?}; use --help")),
        }
    }
    Ok((root.unwrap_or(default_root()?), report_path, false))
}
fn usage() -> &'static str {
    "Usage: farmer-ux-contract-verifier [--root REPOSITORY_ROOT] [--report-json OUTPUT_PATH]\n\
     Validates local FARMER-UX-001 schemas and pinned v1 corpus invariants.\n\
     It performs no network access and makes no UI or field-impact claim."
}
fn execute(root: &Path) -> Result<Report, String> {
    let mut artifacts = vec![
        load(root, "cases", CASES, CASES_SCHEMA)?,
        load(root, "roles", ROLES, ROLES_SCHEMA)?,
        load(root, "copy", COPY, COPY_SCHEMA)?,
    ];
    let mut schema_errors = Vec::new();
    for artifact in &artifacts { schema_errors.extend(artifact.schema_errors.clone()); }
    schema_errors.sort();
    let semantic_errors = semantic_checks(&artifacts[0].instance, &artifacts[1].instance, &artifacts[2].instance);
    let mut errors = schema_errors.clone();
    errors.extend(semantic_errors.clone());
    errors.sort();
    errors.dedup();

    let counts = Counts {
        cases: array(&artifacts[0].instance, "cases").len(),
        roles: array(&artifacts[1].instance, "roles").len(),
        copy_states: array(&artifacts[2].instance, "states").len(),
        shared_invariants: string_set(artifacts[0].instance.get("shared_invariants")).len(),
    };
    Ok(Report {
        report_version: REPORT_VERSION.to_owned(),
        tool_version: env!("CARGO_PKG_VERSION").to_owned(),
        validator: VALIDATOR.to_owned(),
        status: if errors.is_empty() { "PASS" } else { "FAIL" }.to_owned(),
        artifacts: artifacts.iter_mut().map(artifact_report).collect(),
        counts,
        schema_error_count: schema_errors.len(),
        semantic_error_count: semantic_errors.len(),
        errors,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    })
}
fn main() -> ExitCode {
    let (root, report_path, help) = match parse_args() {
        Ok(args) => args,
        Err(error) => {
            eprintln!("{error}\n{}", usage());
            return ExitCode::from(2);
        }
    };
    if help {
        println!("{}", usage());
        return ExitCode::SUCCESS;
    }
    let report = match execute(&root) {
        Ok(report) => report,
        Err(error) => {
            eprintln!("FARMER-UX-002: {error}");
            return ExitCode::from(2);
        }
    };
    let output = match serde_json::to_string_pretty(&report) {
        Ok(value) => format!("{value}\n"),
        Err(error) => {
            eprintln!("cannot serialize report: {error}");
            return ExitCode::from(2);
        }
    };
    if let Some(path) = report_path {
        if let Err(error) = fs::write(&path, output.as_bytes()) {
            eprintln!("cannot write report {}: {error}", path.display());
            return ExitCode::from(2);
        }
    }
    print!("{output}");
    if report.status == "PASS" { ExitCode::SUCCESS } else { ExitCode::from(1) }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn parse(s: &str) -> Value { serde_json::from_str(s).expect("fixture should parse") }
    fn cases() -> Value { parse(include_str!("../../farmer-ux-001-cases-v1.json")) }
    fn case_schema() -> Value { parse(include_str!("../../farmer-ux-001-cases-v1.schema.json")) }
    fn roles() -> Value { parse(include_str!("../../farmer-ux-001-role-task-matrix-v1.json")) }
    fn role_schema() -> Value { parse(include_str!("../../farmer-ux-001-role-task-matrix-v1.schema.json")) }
    fn copy() -> Value { parse(include_str!("../../farmer-ux-001-copy-v1.json")) }
    fn copy_schema() -> Value { parse(include_str!("../../farmer-ux-001-copy-v1.schema.json")) }

    #[test]
    fn canonical_corpus_passes_schema_and_semantic_checks() {
        let c = cases(); let r = roles(); let p = copy();
        assert!(validate_schema("cases", &c, &case_schema()).is_empty());
        assert!(validate_schema("roles", &r, &role_schema()).is_empty());
        assert!(validate_schema("copy", &p, &copy_schema()).is_empty());
        assert!(semantic_checks(&c, &r, &p).is_empty());
    }
    #[test]
    fn duplicate_case_id_is_rejected() {
        let mut c = cases();
        let first = c["cases"][0].clone();
        c["cases"].as_array_mut().expect("array").push(first);
        assert!(semantic_checks(&c, &roles(), &copy()).iter().any(|e| e.contains("duplicate id")));
    }
    #[test]
    fn contradictory_visibility_is_rejected() {
        let mut c = cases();
        let shown = c["cases"][0]["expected"]["must_show"][0].as_str().expect("string").to_owned();
        c["cases"][0]["expected"]["must_not_show"].as_array_mut().expect("array").push(Value::String(shown));
        assert!(semantic_checks(&c, &roles(), &copy()).iter().any(|e| e.contains("contradictory visibility")));
    }
    #[test]
    fn deleting_a_required_risk_case_is_rejected() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").retain(|case| case["id"].as_str() != Some("FUX-DIGEST-017"));
        assert!(semantic_checks(&c, &roles(), &copy()).iter().any(|e| e.contains("FUX-DIGEST-017")));
    }
    #[test]
    fn expired_quote_cannot_create_a_booking() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").iter_mut()
            .find(|case| case["id"].as_str() == Some("FUX-QUOTE-002")).expect("case")
            ["expected"]["side_effect_counts"]["bookings"] = Value::from(1);
        assert!(semantic_checks(&c, &roles(), &copy()).iter()
            .any(|e| e.contains("FUX-QUOTE-002") && e.contains("side_effect_counts.bookings=0")));
    }
    #[test]
    fn digest_syntax_cannot_become_source_authentication() {
        let mut c = cases();
        c["cases"].as_array_mut().expect("array").iter_mut()
            .find(|case| case["id"].as_str() == Some("FUX-DIGEST-017")).expect("case")
            ["expected"]["side_effect_counts"]["verified_source_claims"] = Value::from(1);
        assert!(semantic_checks(&c, &roles(), &copy()).iter()
            .any(|e| e.contains("FUX-DIGEST-017") && e.contains("side_effect_counts.verified_source_claims=0")));
    }
}
