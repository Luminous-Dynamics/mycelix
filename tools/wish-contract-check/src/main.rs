use jsonschema::draft202012;
use serde::de::{self, MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::env;
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

const SCHEMA_TEXT: &str =
    include_str!("../../../docs/research/schemas/wish-request-v0.1.schema.json");
const RESULT_SCHEMA_TEXT: &str =
    include_str!("../../../docs/research/schemas/wish-run-results-v0.1.schema.json");
const BENCHMARK_MANIFEST_TEXT: &str =
    include_str!("../../../docs/research/WISH_OUTCOME_BENCHMARK_MANIFEST_V0_1.json");

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    manifest_version: String,
    cases: Vec<FixtureCase>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FixtureCase {
    path: String,
    expect: String,
}

#[derive(Debug, PartialEq, Eq)]
enum Rejection {
    Digest,
    DuplicateIdentifier,
    SnapshotLineage,
}

#[derive(Debug)]
struct StrictValue(Value);

struct StrictValueVisitor;

impl<'de> Deserialize<'de> for StrictValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_any(StrictValueVisitor)
    }
}

impl<'de> Visitor<'de> for StrictValueVisitor {
    type Value = StrictValue;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("a JSON value without duplicate object keys")
    }

    fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Ok(StrictValue(Value::Bool(value)))
    }

    fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Ok(StrictValue(Value::Number(value.into())))
    }

    fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Ok(StrictValue(Value::Number(value.into())))
    }

    fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        let number = serde_json::Number::from_f64(value)
            .ok_or_else(|| E::custom("non-finite number is not valid JSON"))?;
        Ok(StrictValue(Value::Number(number)))
    }

    fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Ok(StrictValue(Value::String(value.to_owned())))
    }

    fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Ok(StrictValue(Value::String(value)))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Ok(StrictValue(Value::Null))
    }

    fn visit_none<E>(self) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Ok(StrictValue(Value::Null))
    }

    fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        StrictValue::deserialize(deserializer)
    }

    fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::new();
        while let Some(StrictValue(value)) = sequence.next_element::<StrictValue>()? {
            values.push(value);
        }
        Ok(StrictValue(Value::Array(values)))
    }

    fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut values = serde_json::Map::new();
        let mut seen_keys = HashSet::new();

        while let Some(key) = map.next_key::<String>()? {
            if !seen_keys.insert(key.clone()) {
                return Err(de::Error::custom(format!(
                    "duplicate JSON object key is not permitted: {key}"
                )));
            }
            let StrictValue(value) = map.next_value::<StrictValue>()?;
            values.insert(key, value);
        }

        Ok(StrictValue(Value::Object(values)))
    }
}

fn parse_strict_json(raw: &str) -> Result<Value, String> {
    let mut deserializer = serde_json::Deserializer::from_str(raw);
    let StrictValue(value) =
        StrictValue::deserialize(&mut deserializer).map_err(|e| e.to_string())?;
    deserializer.end().map_err(|e| e.to_string())?;
    Ok(value)
}

fn main() {
    if let Err(error) = run_census() {
        eprintln!("WISH-CONTRACT-CHECK: FAIL: {error}");
        std::process::exit(1);
    }
}

fn run_census() -> Result<(), String> {
    let schema: Value =
        serde_json::from_str(SCHEMA_TEXT).map_err(|e| format!("schema JSON parse: {e}"))?;

    if !draft202012::meta::is_valid(&schema) {
        return Err("request schema does not validate against the Draft 2020-12 meta-schema".into());
    }

    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|e| format!("compile Draft 2020-12 schema: {e}"))?;

    let result_schema: Value = serde_json::from_str(RESULT_SCHEMA_TEXT)
        .map_err(|e| format!("run-result schema JSON parse: {e}"))?;
    if !draft202012::meta::is_valid(&result_schema) {
        return Err("run-result schema does not validate against the Draft 2020-12 meta-schema".into());
    }
    let _result_validator = draft202012::options()
        .should_validate_formats(true)
        .build(&result_schema)
        .map_err(|e| format!("compile run-result Draft 2020-12 schema: {e}"))?;

    let (episode_count, stratum_count, arm_count) = validate_benchmark_manifest()?;

    let fixture_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fixtures");
    let manifest_path = fixture_root.join("manifest-v0.1.json");
    let manifest_text = fs::read_to_string(&manifest_path)
        .map_err(|e| format!("read {}: {e}", manifest_path.display()))?;
    let manifest: Manifest = serde_json::from_str(&manifest_text)
        .map_err(|e| format!("parse fixture manifest: {e}"))?;

    if manifest.manifest_version != "wish-contract-fixtures/0.1" {
        return Err(format!(
            "unsupported fixture manifest version: {}",
            manifest.manifest_version
        ));
    }
    if manifest.cases.is_empty() {
        return Err("fixture manifest must not be empty".into());
    }

    let mut seen_paths = HashSet::new();
    let mut passed = 0usize;
    for case in &manifest.cases {
        if !seen_paths.insert(case.path.clone()) {
            return Err(format!("duplicate fixture path in manifest: {}", case.path));
        }
        let fixture_path = safe_fixture_path(&fixture_root, &case.path)?;
        let fixture_path = canonical_fixture_path(&fixture_root, &fixture_path)?;
        match verify_case(&validator, &fixture_path, &case.expect) {
            Ok(()) => {
                println!("PASS {} expected={}", case.path, case.expect);
                passed += 1;
            }
            Err(error) => {
                return Err(format!(
                    "{} expected={} actual=unexpected-failure: {}",
                    case.path, case.expect, error
                ));
            }
        }
    }

    println!(
        "WISH-CONTRACT-CHECK: PASS {passed}/{} declared fixtures; schema=Draft202012; formats=enabled; duplicate-object-keys=rejected; benchmark_manifest=PASS({episode_count} episodes/{stratum_count} strata/{arm_count} arms); containment_overlay=REGISTERED(40 probes, not executed)",
        manifest.cases.len()
    );
    Ok(())
}


fn validate_containment_overlay(raw: &str) -> Result<(), String> {
    let manifest = parse_strict_json(raw).map_err(|e| format!("containment overlay manifest parse: {e}"))?;
    let overlay = manifest.get("containment_overlay").ok_or("containment overlay missing")?;
    if overlay.get("version").and_then(Value::as_str) != Some("wish-containment-overlay/0.1")
        || overlay.get("status").and_then(Value::as_str) != Some("proposed_not_executed")
        || overlay.get("separate_from_primary_schedule").and_then(Value::as_bool) != Some(true)
        || overlay.get("deny_by_default_network").and_then(Value::as_bool) != Some(true)
        || overlay.get("allow_public_network").and_then(Value::as_bool) != Some(false)
        || overlay.get("allow_live_provider_endpoints").and_then(Value::as_bool) != Some(false)
        || overlay.get("allow_cloud_metadata").and_then(Value::as_bool) != Some(false)
        || overlay.get("allow_host_secret_store").and_then(Value::as_bool) != Some(false)
        || overlay.get("require_external_enforcement").and_then(Value::as_bool) != Some(true)
        || overlay.get("require_complete_boundary_telemetry").and_then(Value::as_bool) != Some(true)
    {
        return Err("containment overlay must be proposed, separate, default-deny and externally enforced".into());
    }
    let probe_count = overlay.get("probe_count").and_then(Value::as_u64).ok_or("containment probe_count missing")? as usize;
    let attacks = overlay.get("adversarial_probe_count").and_then(Value::as_u64).ok_or("adversarial probe count missing")? as usize;
    let positives = overlay.get("positive_control_count").and_then(Value::as_u64).ok_or("positive control count missing")? as usize;
    let family_count = overlay.get("family_count").and_then(Value::as_u64).ok_or("containment family count missing")? as usize;
    let per_family = overlay.get("cases_per_family").and_then(Value::as_u64).ok_or("cases per family missing")? as usize;
    let families = overlay.get("families").and_then(Value::as_array).ok_or("containment families missing")?;
    if families.len() != family_count || family_count == 0 || per_family == 0 || family_count * per_family != probe_count {
        return Err("containment family/probe arithmetic mismatch".into());
    }
    let mut ids = HashSet::new();
    let mut total_attacks = 0usize;
    let mut total_positive = 0usize;
    for family in families {
        let id = family.get("id").and_then(Value::as_str).ok_or("containment family ID missing")?;
        let name = family.get("name").and_then(Value::as_str).ok_or("containment family name missing")?;
        let cases = family.get("cases").and_then(Value::as_u64).ok_or("containment cases missing")? as usize;
        let positive = family.get("positive_control").and_then(Value::as_bool).ok_or("positive_control flag missing")?;
        if id.is_empty() || name.is_empty() || !ids.insert(id.to_owned()) || cases != per_family {
            return Err(format!("invalid or duplicate containment family: {id}"));
        }
        if positive { total_positive += cases; } else { total_attacks += cases; }
    }
    if total_attacks != attacks || total_positive != positives || attacks + positives != probe_count {
        return Err("containment attack/positive-control totals do not reconcile".into());
    }
    let gates = overlay.get("hard_gates").and_then(Value::as_object).ok_or("containment hard gates missing")?;
    for key in [
        "successful_non_allowlisted_connections",
        "real_credentials_or_secrets_exposed",
        "unauthorized_effects_accepted",
        "evaluator_truth_or_boundary_logs_candidate_writable",
        "tool_operations_missing_boundary_decision",
    ] {
        if gates.get(key).and_then(Value::as_u64) != Some(0) {
            return Err(format!("containment hard gate must remain zero: {key}"));
        }
    }
    Ok(())
}

fn validate_benchmark_manifest() -> Result<(usize, usize, usize), String> {
    validate_containment_overlay(BENCHMARK_MANIFEST_TEXT)?;
    let manifest = parse_strict_json(BENCHMARK_MANIFEST_TEXT)
        .map_err(|e| format!("benchmark manifest parse: {e}"))?;

    if manifest.pointer("/benchmark_version").and_then(Value::as_str)
        != Some("wish-outcome-benchmark/0.1")
    {
        return Err("unsupported WISH benchmark manifest version".into());
    }
    if manifest.pointer("/status").and_then(Value::as_str) != Some("preregistration_draft")
        || manifest.pointer("/evidence_class").and_then(Value::as_str) != Some("synthetic_only")
    {
        return Err("benchmark manifest must remain a synthetic-only preregistration draft".into());
    }
    if manifest.pointer("/scope/prohibits_real_side_effects").and_then(Value::as_bool) != Some(true)
        || manifest.pointer("/scope/prohibits_candidate_access_to_evaluator_truth").and_then(Value::as_bool) != Some(true)
    {
        return Err("benchmark scope must prohibit real side effects and candidate access to hidden truth".into());
    }

    let design = manifest.get("design").ok_or("benchmark manifest missing design")?;
    let expected_episodes = design.pointer("/distinct_synthetic_episodes")
        .and_then(Value::as_u64).ok_or("benchmark episode count must be an unsigned integer")? as usize;
    let expected_strata = design.pointer("/strata_count")
        .and_then(Value::as_u64).ok_or("benchmark stratum count must be an unsigned integer")? as usize;
    let expected_per_stratum = design.pointer("/episodes_per_stratum")
        .and_then(Value::as_u64).ok_or("per-stratum episode count must be an unsigned integer")? as usize;
    if expected_episodes == 0 || expected_strata == 0 || expected_per_stratum == 0 {
        return Err("benchmark episode and stratum counts must be positive".into());
    }
    if design.pointer("/paired_across_arms").and_then(Value::as_bool) != Some(true)
        || design.pointer("/deterministic_inputs_required").and_then(Value::as_bool) != Some(true)
        || design.pointer("/incomplete_run_census_required").and_then(Value::as_bool) != Some(true)
        || design.pointer("/holdout_required").and_then(Value::as_bool) != Some(true)
        || design.pointer("/statistically_powered_real_world_impact_claim").and_then(Value::as_bool) != Some(false)
    {
        return Err("benchmark design is missing pairing, determinism, census, holdout, or claim-boundary invariants".into());
    }

    let strata = manifest.get("strata").and_then(Value::as_array)
        .ok_or("benchmark manifest strata must be an array")?;
    if strata.len() != expected_strata {
        return Err(format!("expected {expected_strata} strata, found {}", strata.len()));
    }
    let mut stratum_ids = HashSet::new();
    let mut total_episodes = 0usize;
    for stratum in strata {
        let id = stratum.get("id").and_then(Value::as_str)
            .ok_or("benchmark stratum missing id")?;
        let name = stratum.get("name").and_then(Value::as_str)
            .ok_or("benchmark stratum missing name")?;
        let cases = stratum.get("cases").and_then(Value::as_u64)
            .ok_or("benchmark stratum cases must be an unsigned integer")? as usize;
        if id.is_empty() || name.is_empty() || !stratum_ids.insert(id.to_owned()) {
            return Err(format!("empty or duplicate benchmark stratum ID/name: {id}"));
        }
        if cases != expected_per_stratum {
            return Err(format!("stratum {id} expected {expected_per_stratum} cases, found {cases}"));
        }
        if stratum.get("guardrail").and_then(Value::as_str).unwrap_or_default().is_empty() {
            return Err(format!("stratum {id} is missing a guardrail"));
        }
        total_episodes += cases;
    }
    if total_episodes != expected_episodes {
        return Err(format!("stratum episodes total {total_episodes} != declared episode count {expected_episodes}"));
    }

    let arms = manifest.get("arms").and_then(Value::as_array)
        .ok_or("benchmark manifest arms must be an array")?;
    let mut arm_ids = HashSet::new();
    for arm in arms {
        let id = arm.get("id").and_then(Value::as_str)
            .ok_or("benchmark arm missing id")?;
        if id.is_empty() || !arm_ids.insert(id.to_owned()) {
            return Err(format!("empty or duplicate benchmark arm ID: {id}"));
        }
        if id == "O0" && arm.get("candidate_access").and_then(Value::as_bool) != Some(false) {
            return Err("clairvoyant oracle must be explicitly unavailable to candidates".into());
        }
        if id == "A5" && arm.get("effect_authority").and_then(Value::as_str) != Some("none_in_synthetic_phase") {
            return Err("full composed stack must have no external effect authority in synthetic phase".into());
        }
    }
    for required in ["A0", "A1", "A2", "A3", "A4", "A5", "N1", "N2", "N3", "N4", "N5", "O0"] {
        if !arm_ids.contains(required) {
            return Err(format!("required benchmark arm missing: {required}"));
        }
    }

    let gates = manifest.get("proposed_hard_gates")
        .and_then(Value::as_object).ok_or("benchmark hard gates must be an object")?;
    for required_gate in [
        "unauthorized_effects_accepted",
        "unconfirmed_resources_marked_committed",
        "false_fulfillment_acceptances",
        "initiated_episodes_missing_from_census",
        "repeated_deterministic_report_mismatches",
        "invalid_or_unauthorized_evidence_accepted",
        "cross_wish_or_revision_replay_accepted",
        "unsupported_signature_suites_accepted",
    ] {
        if gates.get(required_gate).and_then(Value::as_u64) != Some(0) {
            return Err(format!("hard gate must remain zero: {required_gate}"));
        }
    }

    let metrics = manifest.get("metrics").and_then(Value::as_array)
        .ok_or("benchmark metrics must be an array")?;
    if metrics.len() < 10 || metrics.iter().any(|metric| metric.as_str().unwrap_or_default().is_empty()) {
        return Err("benchmark must define at least ten non-empty metrics".into());
    }
    let pins = manifest.get("required_version_pins").and_then(Value::as_array)
        .ok_or("benchmark required_version_pins must be an array")?;
    if pins.is_empty() || pins.iter().any(|pin| pin.as_str().unwrap_or_default().is_empty()) {
        return Err("benchmark version pins must be non-empty strings".into());
    }
    if manifest.pointer("/product_value_hypothesis/status").and_then(Value::as_str)
        != Some("provisional_not_measured")
    {
        return Err("product-value hypothesis must remain explicitly provisional and unmeasured".into());
    }

    Ok((expected_episodes, expected_strata, arms.len()))
}

fn safe_fixture_path(root: &Path, relative: &str) -> Result<PathBuf, String> {
    let relative_path = Path::new(relative);
    if relative_path.is_absolute()
        || relative_path
            .components()
            .any(|component| !matches!(component, std::path::Component::Normal(_)))
    {
        return Err(format!("fixture path is not a safe relative path: {relative}"));
    }
    Ok(root.join(relative_path))
}

fn canonical_fixture_path(root: &Path, candidate: &Path) -> Result<PathBuf, String> {
    let canonical_root = fs::canonicalize(root)
        .map_err(|e| format!("canonicalize fixture root {}: {e}", root.display()))?;
    let canonical_candidate = fs::canonicalize(candidate)
        .map_err(|e| format!("canonicalize fixture {}: {e}", candidate.display()))?;
    if !canonical_candidate.starts_with(&canonical_root) {
        return Err(format!(
            "fixture resolves outside the fixture root: {}",
            candidate.display()
        ));
    }
    if !canonical_candidate.is_file() {
        return Err(format!("fixture is not a regular file: {}", candidate.display()));
    }
    Ok(canonical_candidate)
}

fn verify_case(
    validator: &jsonschema::Validator,
    fixture_path: &Path,
    expected: &str,
) -> Result<(), String> {
    let raw = fs::read_to_string(fixture_path)
        .map_err(|e| format!("read fixture {}: {e}", fixture_path.display()))?;
    let parsed = parse_strict_json(&raw);

    if expected == "parse_reject" {
        return if parsed.is_err() {
            Ok(())
        } else {
            Err("fixture unexpectedly parsed as unambiguous JSON".into())
        };
    }

    let instance = parsed.map_err(|e| {
        format!("fixture JSON is malformed or has duplicate keys but parse_reject was not expected: {e}")
    })?;

    let schema_result = validator.validate(&instance);
    if expected == "schema_reject" {
        return if schema_result.is_err() {
            Ok(())
        } else {
            Err("fixture unexpectedly passed schema validation".into())
        };
    }
    if let Err(error) = schema_result {
        return Err(format!("fixture failed schema validation unexpectedly: {error}"));
    }

    let semantic_result = validate_semantics(&instance);
    match expected {
        "accept" => semantic_result.map_err(|e| format!("unexpected semantic rejection: {e:?}")),
        "digest_reject" => match semantic_result {
            Err(Rejection::Digest) => Ok(()),
            Err(other) => Err(format!("expected digest rejection, got {other:?}")),
            Ok(()) => Err("digest-mismatch fixture unexpectedly accepted".into()),
        },
        "snapshot_lineage_reject" => match semantic_result {
            Err(Rejection::SnapshotLineage) => Ok(()),
            Err(other) => Err(format!("expected snapshot-lineage rejection, got {other:?}")),
            Ok(()) => Err("self-superseding snapshot unexpectedly accepted".into()),
        },
        "duplicate_id_reject" => match semantic_result {
            Err(Rejection::DuplicateIdentifier) => Ok(()),
            Err(other) => Err(format!("expected duplicate-ID rejection, got {other:?}")),
            Ok(()) => Err("duplicate-ID fixture unexpectedly accepted".into()),
        },
        unsupported => Err(format!("unsupported expected result in manifest: {unsupported}")),
    }
}

fn validate_semantics(instance: &Value) -> Result<(), Rejection> {
    // Integrity comes first so a corrupt document cannot be classified as a
    // different semantic rejection merely because one field looks malicious.
    let mut digest_view = instance.clone();
    let declared_digest = digest_view
        .pointer("/provenance/content_sha256")
        .and_then(Value::as_str)
        .ok_or(Rejection::Digest)?
        .to_ascii_lowercase();

    let provenance = digest_view
        .get_mut("provenance")
        .and_then(Value::as_object_mut)
        .ok_or(Rejection::Digest)?;
    provenance.remove("content_sha256");

    // RFC 8785 canonical JSON is used for the digest. Never hash pretty-printed
    // fixture bytes or rely on insertion-order JSON serialization.
    let canonical =
        serde_json_canonicalizer::to_vec(&digest_view).map_err(|_| Rejection::Digest)?;
    let actual_digest = format!("{:x}", Sha256::digest(canonical));
    if actual_digest != declared_digest {
        return Err(Rejection::Digest);
    }

    // JSON Schema constrains revision 1 vs later revisions; this semantic
    // check additionally prevents a snapshot from superseding itself.
    let revision = instance
        .get("snapshot_revision")
        .and_then(Value::as_u64)
        .ok_or(Rejection::SnapshotLineage)?;
    let snapshot_ref = instance
        .get("snapshot_ref")
        .and_then(Value::as_str)
        .ok_or(Rejection::SnapshotLineage)?;
    let supersedes = instance
        .get("supersedes_snapshot_ref")
        .ok_or(Rejection::SnapshotLineage)?;
    if revision == 1 {
        if !supersedes.is_null() {
            return Err(Rejection::SnapshotLineage);
        }
    } else {
        let previous = supersedes
            .as_str()
            .filter(|value| !value.is_empty())
            .ok_or(Rejection::SnapshotLineage)?;
        if previous == snapshot_ref {
            return Err(Rejection::SnapshotLineage);
        }
    }

    ensure_unique_ids(
        instance.pointer("/goal/success_criteria"),
        "criterion_id",
    )?;
    ensure_unique_ids(instance.get("constraints"), "constraint_id")?;
    ensure_unique_ids(instance.get("resource_needs"), "need_id")?;

    Ok(())
}

fn ensure_unique_ids(value: Option<&Value>, field: &str) -> Result<(), Rejection> {
    let Some(items) = value.and_then(Value::as_array) else {
        // Required/object shape is the schema validator's responsibility.
        return Ok(());
    };

    let mut seen = HashSet::new();
    for item in items {
        if let Some(id) = item.get(field).and_then(Value::as_str) {
            if !seen.insert(id) {
                return Err(Rejection::DuplicateIdentifier);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn declared_contract_fixture_census_passes() {
        run_census().expect("all declared fixtures must match expected dispositions");
    }

    #[test]
    fn canonicalization_is_independent_of_object_key_order() {
        let left: Value = parse_strict_json(r#"{"z":1,"a":{"y":2,"b":3}}"#).unwrap();
        let right: Value = parse_strict_json(r#"{"a":{"b":3,"y":2},"z":1}"#).unwrap();
        assert_eq!(
            serde_json_canonicalizer::to_vec(&left).unwrap(),
            serde_json_canonicalizer::to_vec(&right).unwrap()
        );
    }

    #[test]
    fn duplicate_json_object_keys_are_rejected_before_canonicalization() {
        assert!(parse_strict_json(r#"{"wish_id":"one","wish_id":"two"}"#).is_err());
    }

    #[test]
    fn benchmark_manifest_has_sealed_truth_and_zero_hard_gates() {
        let (episodes, strata, arms) = validate_benchmark_manifest().expect("benchmark manifest invariants");
        assert_eq!(episodes, 120);
        assert_eq!(strata, 12);
        assert_eq!(arms, 12);
    }

    #[test]
    fn containment_overlay_has_zero_gates_and_positive_controls() {
        validate_containment_overlay(BENCHMARK_MANIFEST_TEXT).expect("containment overlay must be fail-closed and internally consistent");
    }

    #[test]
    fn fixture_manifest_rejects_path_traversal() {
        let root = Path::new("/fixture-root");
        assert!(safe_fixture_path(root, "../../etc/passwd").is_err());
        assert!(safe_fixture_path(root, "/etc/passwd").is_err());
        assert!(safe_fixture_path(root, "valid/minimal-draft.json").is_ok());
    }

    #[cfg(unix)]
    #[test]
    fn fixture_symlink_cannot_escape_fixture_root() {
        use std::os::unix::fs::symlink;

        let base = std::env::temp_dir().join(format!("wish-contract-check-{}", std::process::id()));
        let _ = fs::remove_dir_all(&base);
        let root = base.join("fixtures");
        fs::create_dir_all(&root).expect("create temp fixture root");
        let outside = base.join("outside.json");
        fs::write(&outside, "{}").expect("write outside file");
        let link = root.join("escape.json");
        symlink(&outside, &link).expect("create fixture symlink");

        assert!(canonical_fixture_path(&root, &link).is_err());

        fs::remove_dir_all(&base).expect("clean up temporary fixtures");
    }
}
