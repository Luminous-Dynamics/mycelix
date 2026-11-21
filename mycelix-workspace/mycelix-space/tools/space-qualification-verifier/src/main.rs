use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    fs,
    path::{Component, Path, PathBuf},
    process,
};

#[derive(Default)]
struct Findings {
    failures: BTreeMap<String, String>,
    incomplete: BTreeMap<String, String>,
    checks: BTreeSet<String>,
}

impl Findings {
    fn fail(&mut self, code: &str, detail: impl Into<String>) {
        self.failures.entry(code.to_string()).or_insert_with(|| detail.into());
    }

    fn mark_incomplete(&mut self, code: &str, detail: impl Into<String>) {
        self.incomplete.entry(code.to_string()).or_insert_with(|| detail.into());
    }

    fn check(&mut self, name: &str) {
        self.checks.insert(name.to_string());
    }

    fn status(&self) -> &'static str {
        if !self.failures.is_empty() {
            "FAIL"
        } else if !self.incomplete.is_empty() {
            "INCOMPLETE"
        } else {
            "PASS"
        }
    }

    fn integrity(&self) -> &'static str {
        if self.failures.is_empty() { "PASS" } else { "FAIL" }
    }

    fn as_json(&self, package_id: Option<&str>) -> Value {
        json!({
            "verifier": "mycelix-space-qualification-verifier/0.1.0",
            "status": self.status(),
            "package_integrity": self.integrity(),
            "package_id": package_id,
            "checks": self.checks.iter().cloned().collect::<Vec<_>>(),
            "failure_reason_codes": self.failures.keys().cloned().collect::<Vec<_>>(),
            "incomplete_reason_codes": self.incomplete.keys().cloned().collect::<Vec<_>>(),
            "failures": self.failures.iter().map(|(code, detail)| json!({
                "code": code, "detail": detail
            })).collect::<Vec<_>>(),
            "incomplete": self.incomplete.iter().map(|(code, detail)| json!({
                "code": code, "detail": detail
            })).collect::<Vec<_>>()
        })
    }
}

fn get_str<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn get_array<'a>(value: &'a Value, key: &str) -> Option<&'a [Value]> {
    value.get(key).and_then(Value::as_array).map(Vec::as_slice)
}

fn get_u64(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn is_nonempty(value: Option<&str>) -> bool {
    value.is_some_and(|text| !text.trim().is_empty())
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn string_list(value: &Value, field: &str, findings: &mut Findings) -> Vec<String> {
    let Some(items) = value.as_array() else {
        findings.fail("INVALID_ARRAY", format!("{field} must be an array"));
        return Vec::new();
    };
    let mut output = Vec::with_capacity(items.len());
    for (index, item) in items.iter().enumerate() {
        match item.as_str() {
            Some(text) if !text.trim().is_empty() => output.push(text.to_string()),
            _ => findings.fail(
                "INVALID_ARRAY_ITEM",
                format!("{field}[{index}] must be a non-empty string"),
            ),
        }
    }
    output
}

fn check_unique(values: &[String], field: &str, findings: &mut Findings) {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value) {
            findings.fail(
                "DUPLICATE_ID",
                format!("{field} contains duplicate identity {value}"),
            );
        }
    }
}

fn resolve_artifact_path(
    repo_root: &Path,
    relative: &str,
    findings: &mut Findings,
    artifact_id: &str,
) -> Option<PathBuf> {
    let rel = Path::new(relative);
    if rel.as_os_str().is_empty()
        || rel.is_absolute()
        || rel.components().any(|part| {
            matches!(part, Component::ParentDir | Component::RootDir | Component::Prefix(_))
        })
    {
        findings.fail(
            "UNSAFE_ARTIFACT_PATH",
            format!("artifact {artifact_id} has a non-repository-relative path"),
        );
        return None;
    }

    let root = match fs::canonicalize(repo_root) {
        Ok(path) => path,
        Err(error) => {
            findings.mark_incomplete(
                "REPOSITORY_ROOT_UNAVAILABLE",
                format!("cannot resolve repository root: {error}"),
            );
            return None;
        }
    };
    let joined = root.join(rel);
    let resolved = match fs::canonicalize(&joined) {
        Ok(path) => path,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            findings.mark_incomplete(
                "SOURCE_ARTIFACT_MISSING",
                format!("retained source artifact {artifact_id} is not present: {relative}"),
            );
            return None;
        }
        Err(error) => {
            findings.mark_incomplete(
                "SOURCE_ARTIFACT_UNREADABLE",
                format!("cannot resolve source artifact {artifact_id}: {error}"),
            );
            return None;
        }
    };
    if !resolved.starts_with(&root) {
        findings.fail(
            "ARTIFACT_PATH_ESCAPES_ROOT",
            format!("artifact {artifact_id} resolves outside the declared repository root"),
        );
        return None;
    }
    Some(resolved)
}

fn verify_source_artifacts(
    package: &Value,
    repo_root: &Path,
    findings: &mut Findings,
) -> BTreeMap<String, String> {
    let mut ids = BTreeMap::new();
    let Some(artifacts) = get_array(package, "source_artifacts") else {
        findings.fail("MISSING_SOURCE_ARTIFACTS", "source_artifacts must be an array");
        return ids;
    };

    for (index, artifact) in artifacts.iter().enumerate() {
        let Some(id) = get_str(artifact, "artifact_id").filter(|id| !id.trim().is_empty()) else {
            findings.fail("INVALID_ARTIFACT_ID", format!("source_artifacts[{index}] has no artifact_id"));
            continue;
        };
        if ids.contains_key(id) {
            findings.fail("DUPLICATE_ARTIFACT_ID", format!("duplicate artifact_id {id}"));
            continue;
        }
        let declared_digest = get_str(artifact, "raw_bytes_sha256").unwrap_or("").to_string();
        ids.insert(id.to_string(), declared_digest);

        if !get_str(artifact, "raw_bytes_sha256").is_some_and(is_sha256) {
            findings.fail(
                "INVALID_ARTIFACT_DIGEST",
                format!("artifact {id} must declare a lowercase SHA-256 digest"),
            );
        }
        if let Some(source_sha) = artifact.pointer("/parser/source_sha256").and_then(Value::as_str) {
            if !is_sha256(source_sha) {
                findings.fail(
                    "INVALID_PARSER_SOURCE_DIGEST",
                    format!("artifact {id} parser source_sha256 is malformed"),
                );
            }
        }

        let preservation = get_str(artifact, "preservation").unwrap_or("unknown");
        match preservation {
            "exact_bytes_retained" => {
                let Some(relative) = get_str(artifact, "artifact_path") else {
                    findings.fail(
                        "MISSING_ARTIFACT_PATH",
                        format!("exact_bytes_retained artifact {id} needs artifact_path"),
                    );
                    continue;
                };
                let Some(path) = resolve_artifact_path(repo_root, relative, findings, id) else {
                    continue;
                };
                let bytes = match fs::read(&path) {
                    Ok(bytes) => bytes,
                    Err(error) => {
                        findings.mark_incomplete(
                            "SOURCE_ARTIFACT_UNREADABLE",
                            format!("cannot read artifact {id}: {error}"),
                        );
                        continue;
                    }
                };
                let expected_len = get_u64(artifact, "byte_length");
                if expected_len != Some(bytes.len() as u64) {
                    findings.fail(
                        "ARTIFACT_LENGTH_MISMATCH",
                        format!(
                            "artifact {id} declares byte_length {:?}, actual length is {}",
                            expected_len,
                            bytes.len()
                        ),
                    );
                } else {
                    findings.check("source_artifact_byte_length");
                }

                let expected_digest = get_str(artifact, "raw_bytes_sha256");
                if !expected_digest.is_some_and(is_sha256) {
                    findings.fail(
                        "INVALID_ARTIFACT_DIGEST",
                        format!("artifact {id} must declare a lowercase SHA-256 digest"),
                    );
                } else {
                    let actual = format!("{:x}", Sha256::digest(&bytes));
                    if expected_digest != Some(actual.as_str()) {
                        findings.fail(
                            "ARTIFACT_DIGEST_MISMATCH",
                            format!("artifact {id} SHA-256 does not match retained bytes"),
                        );
                    } else {
                        findings.check("source_artifact_sha256");
                    }
                }
            }
            "digest_only" => findings.mark_incomplete(
                "SOURCE_BYTES_NOT_RETAINED",
                format!("artifact {id} has only a digest; offline byte replay is unavailable"),
            ),
            "unavailable" => findings.mark_incomplete(
                "SOURCE_BYTES_UNAVAILABLE",
                format!("artifact {id} has no retained source bytes"),
            ),
            other => findings.fail(
                "INVALID_PRESERVATION_STATUS",
                format!("artifact {id} has unsupported preservation status {other}"),
            ),
        }

        if let Some(fields) = get_array(artifact, "field_semantics") {
            for (field_index, field) in fields.iter().enumerate() {
                let status = get_str(field, "status").unwrap_or("unknown");
                let normalized = field.get("normalized_value");
                match status {
                    "absent_optional" => {
                        if normalized.is_some_and(|value| !value.is_null()) {
                            findings.fail(
                                "NONVALUE_FIELD_COERCED",
                                format!(
                                    "artifact {id} field_semantics[{field_index}] has status {status} but a normalized value"
                                ),
                            );
                        }
                    }
                    "invalid" | "unsupported" => {
                        if normalized.is_some_and(|value| !value.is_null()) {
                            findings.fail(
                                "NONVALUE_FIELD_COERCED",
                                format!(
                                    "artifact {id} field_semantics[{field_index}] has status {status} but a normalized value"
                                ),
                            );
                        }
                        findings.mark_incomplete(
                            "SOURCE_FIELD_NOT_USABLE",
                            format!("artifact {id} contains a {status} source field"),
                        );
                    }
                    "transformed" => {
                        if !is_nonempty(get_str(field, "transform_id")) {
                            findings.fail(
                                "TRANSFORM_WITHOUT_ID",
                                format!("artifact {id} transformed field lacks transform_id"),
                            );
                        }
                    }
                    "present" => {}
                    other => findings.fail(
                        "INVALID_FIELD_STATUS",
                        format!("artifact {id} has unknown field status {other}"),
                    ),
                }
            }
            findings.check("field_presence_semantics");
        }
        if artifact.get("unknown_fields_preserved").and_then(Value::as_bool) == Some(false) {
            findings.mark_incomplete(
                "UNKNOWN_SOURCE_FIELDS_DROPPED",
                format!("artifact {id} does not preserve unknown source fields"),
            );
        }

        match get_str(artifact, "parse_disposition").unwrap_or("unknown") {
            "parsed" => {}
            "unsupported" | "invalid" | "not_attempted" => findings.mark_incomplete(
                "SOURCE_PARSE_NOT_QUALIFIED",
                format!("artifact {id} parse disposition is not parsed"),
            ),
            other => findings.fail(
                "INVALID_PARSE_DISPOSITION",
                format!("artifact {id} has invalid parse disposition {other}"),
            ),
        }
    }
    findings.check("source_artifact_references");
    ids
}

fn verify_object_and_coverage_ledgers(
    package: &Value,
    artifact_ids: &BTreeMap<String, String>,
    findings: &mut Findings,
) -> (BTreeSet<String>, BTreeSet<String>, usize, usize, usize, usize) {
    let request = package.get("request").unwrap_or(&Value::Null);
    let requested = request
        .get("requested_object_ids")
        .map(|value| string_list(value, "request.requested_object_ids", findings))
        .unwrap_or_else(|| {
            findings.fail("MISSING_REQUESTED_SET", "request.requested_object_ids is missing");
            Vec::new()
        });
    check_unique(&requested, "request.requested_object_ids", findings);
    let requested_set: BTreeSet<String> = requested.iter().cloned().collect();

    let primary = get_str(request, "primary_object_id");
    if !primary.is_some_and(|id| requested_set.contains(id)) {
        findings.fail(
            "PRIMARY_NOT_REQUESTED",
            "primary_object_id must be a member of requested_object_ids",
        );
    }

    let Some(dispositions) = get_array(package, "object_dispositions") else {
        findings.fail("MISSING_OBJECT_DISPOSITIONS", "object_dispositions must be an array");
        return (requested_set, BTreeSet::new(), 0, 0, 0, 0);
    };

    let mut disposition_by_id = BTreeMap::<String, String>::new();
    let mut screened_count = 0usize;
    let mut excluded_count = 0usize;
    let mut unassessed_count = 0usize;
    for (index, disposition) in dispositions.iter().enumerate() {
        let Some(id) = get_str(disposition, "object_id").filter(|id| !id.trim().is_empty()) else {
            findings.fail("INVALID_OBJECT_ID", format!("object_dispositions[{index}] has no object_id"));
            continue;
        };
        let state = get_str(disposition, "disposition").unwrap_or("unknown");
        if disposition_by_id.insert(id.to_string(), state.to_string()).is_some() {
            findings.fail("DUPLICATE_OBJECT_DISPOSITION", format!("object {id} has more than one disposition"));
        }
        if !requested_set.contains(id) {
            findings.fail("UNREQUESTED_OBJECT_DISPOSITION", format!("object {id} was not requested"));
        }

        let artifact_refs = disposition
            .get("source_artifact_ids")
            .map(|value| string_list(value, &format!("object {id} source_artifact_ids"), findings))
            .unwrap_or_default();
        check_unique(&artifact_refs, &format!("object {id} source_artifact_ids"), findings);
        for artifact_id in &artifact_refs {
            if !artifact_ids.contains_key(artifact_id) {
                findings.fail(
                    "UNKNOWN_ARTIFACT_REFERENCE",
                    format!("object {id} references unknown source artifact {artifact_id}"),
                );
            }
        }

        let segments = disposition
            .get("time_segment_ids")
            .map(|value| string_list(value, &format!("object {id} time_segment_ids"), findings))
            .unwrap_or_default();
        check_unique(&segments, &format!("object {id} time_segment_ids"), findings);

        let reasons = disposition
            .get("reason_codes")
            .map(|value| string_list(value, &format!("object {id} reason_codes"), findings))
            .unwrap_or_default();

        match state {
            "screened" => {
                screened_count += 1;
                if segments.is_empty() {
                    findings.fail(
                        "SCREENED_OBJECT_WITHOUT_SEGMENTS",
                        format!("screened object {id} has no declared time segments"),
                    );
                }
            }
            "excluded" => {
                excluded_count += 1;
                if reasons.is_empty() {
                    findings.fail(
                        "EXCLUSION_WITHOUT_REASON",
                        format!("excluded object {id} needs at least one reason code"),
                    );
                }
                if !segments.is_empty() {
                    findings.fail(
                        "EXCLUDED_OBJECT_HAS_SEGMENTS",
                        format!("excluded object {id} must not claim assessed time segments"),
                    );
                }
            }
            "not_assessed" | "rejected" | "ambiguous" => {
                unassessed_count += 1;
                if reasons.is_empty() {
                    findings.fail(
                        "UNASSESSED_WITHOUT_REASON",
                        format!("object {id} with disposition {state} needs a reason code"),
                    );
                }
            }
            other => findings.fail(
                "INVALID_OBJECT_DISPOSITION",
                format!("object {id} has invalid disposition {other}"),
            ),
        }
    }

    for id in &requested_set {
        if !disposition_by_id.contains_key(id) {
            findings.fail("REQUESTED_OBJECT_OMITTED", format!("requested object {id} has no disposition"));
        }
    }
    if disposition_by_id.len() != requested_set.len() {
        findings.fail(
            "OBJECT_SET_CARDINALITY_MISMATCH",
            format!(
                "requested {} objects but received {} unique object dispositions",
                requested_set.len(),
                disposition_by_id.len()
            ),
        );
    }

    let coverage = package.get("coverage").unwrap_or(&Value::Null);
    let Some(segments) = get_array(coverage, "time_segments") else {
        findings.fail("MISSING_TIME_SEGMENTS", "coverage.time_segments must be an array");
        return (requested_set, BTreeSet::new(), screened_count, excluded_count, unassessed_count, 0);
    };
    let mut segment_ids = BTreeSet::new();
    let mut segments_by_object = BTreeMap::<String, BTreeSet<String>>::new();
    let mut segment_unassessed_count = 0usize;
    for (index, segment) in segments.iter().enumerate() {
        let Some(segment_id) = get_str(segment, "segment_id").filter(|id| !id.trim().is_empty()) else {
            findings.fail("INVALID_SEGMENT_ID", format!("coverage.time_segments[{index}] has no segment_id"));
            continue;
        };
        if !segment_ids.insert(segment_id.to_string()) {
            findings.fail("DUPLICATE_SEGMENT_ID", format!("duplicate time segment {segment_id}"));
        }
        let Some(object_id) = get_str(segment, "object_id") else {
            findings.fail("SEGMENT_WITHOUT_OBJECT", format!("time segment {segment_id} has no object_id"));
            continue;
        };
        if !requested_set.contains(object_id) {
            findings.fail(
                "SEGMENT_FOR_UNREQUESTED_OBJECT",
                format!("time segment {segment_id} references unrequested object {object_id}"),
            );
        }
        segments_by_object
            .entry(object_id.to_string())
            .or_default()
            .insert(segment_id.to_string());

        let start = get_str(segment, "start");
        let end = get_str(segment, "end");
        if !is_nonempty(start) || !is_nonempty(end) {
            findings.fail("SEGMENT_WITHOUT_TIME_BOUNDS", format!("time segment {segment_id} needs start and end"));
        }
        match get_str(segment, "disposition").unwrap_or("unknown") {
            "assessed" => {
                if !is_nonempty(get_str(segment, "method_profile_id")) {
                    findings.fail(
                        "ASSESSED_SEGMENT_WITHOUT_METHOD",
                        format!("assessed segment {segment_id} lacks method_profile_id"),
                    );
                }
            }
            "partially_assessed" | "unassessed" => {
                segment_unassessed_count += 1;
                let reason_count = get_array(segment, "reason_codes").map_or(0, |items| items.len());
                if reason_count == 0 {
                    findings.fail(
                        "UNASSESSED_SEGMENT_WITHOUT_REASON",
                        format!("time segment {segment_id} requires a reason code"),
                    );
                }
            }
            "not_applicable" => {
                if disposition_by_id.get(object_id).is_some_and(|state| state == "screened") {
                    findings.fail(
                        "SCREENED_OBJECT_SEGMENT_NOT_APPLICABLE",
                        format!("screened object {object_id} has a time segment marked not_applicable"),
                    );
                }
            }
            other => findings.fail(
                "INVALID_SEGMENT_DISPOSITION",
                format!("time segment {segment_id} has invalid disposition {other}"),
            ),
        }
    }

    for disposition in dispositions {
        let Some(id) = get_str(disposition, "object_id") else { continue };
        let refs = disposition
            .get("time_segment_ids")
            .map(|value| string_list(value, &format!("object {id} time_segment_ids"), findings))
            .unwrap_or_default();
        let reference_set: BTreeSet<String> = refs.into_iter().collect();
        let actual_set = segments_by_object.get(id).cloned().unwrap_or_default();
        if reference_set != actual_set {
            findings.fail(
                "OBJECT_SEGMENT_JOIN_MISMATCH",
                format!("object {id} time_segment_ids do not match the coverage segment ledger"),
            );
        }
    }

    let requested_count = requested_set.len();
    let eligible_count = requested_count.saturating_sub(excluded_count);
    let declared_counts = [
        ("requested_object_count", requested_count),
        ("eligible_object_count", eligible_count),
        ("assessed_object_count", screened_count),
        ("excluded_object_count", excluded_count),
        ("unassessed_object_count", unassessed_count),
    ];
    for (field, expected) in declared_counts {
        if get_u64(coverage, field) != Some(expected as u64) {
            findings.fail(
                "COVERAGE_COUNT_MISMATCH",
                format!("coverage.{field} does not reconcile; expected {expected}"),
            );
        }
    }
    if eligible_count != screened_count + unassessed_count {
        findings.fail(
            "ELIGIBLE_COUNT_DOES_NOT_RECONCILE",
            "eligible_object_count must equal assessed plus unassessed objects",
        );
    }

    match get_str(coverage, "disposition").unwrap_or("unknown") {
        "complete" => {
            if unassessed_count > 0 || segment_unassessed_count > 0 {
                findings.fail(
                    "FALSE_COMPLETE_COVERAGE",
                    "coverage is marked complete while objects or time segments remain unassessed",
                );
            }
            findings.check("complete_object_and_time_coverage");
        }
        "partial" => {
            findings.mark_incomplete(
                "PARTIAL_COVERAGE",
                "coverage ledger explicitly records partial assessment",
            );
        }
        "unknown" => findings.mark_incomplete(
            "UNKNOWN_COVERAGE",
            "coverage completeness is unknown",
        ),
        "rejected" => findings.fail("REJECTED_COVERAGE", "coverage disposition is rejected"),
        other => findings.fail("INVALID_COVERAGE_DISPOSITION", format!("invalid coverage disposition {other}")),
    }

    (
        requested_set,
        segment_ids,
        screened_count,
        excluded_count,
        unassessed_count,
        segment_unassessed_count,
    )
}

#[derive(Clone, Debug)]
struct AssessmentEvidence {
    assessment_type: String,
    status: String,
    pair_id: Option<String>,
    output_artifact_id: Option<String>,
    output_sha256: Option<String>,
}

fn verify_assessments(
    package: &Value,
    artifact_ids: &BTreeMap<String, String>,
    findings: &mut Findings,
) -> (BTreeSet<String>, bool, BTreeMap<String, AssessmentEvidence>) {
    let Some(assessments) = get_array(package, "assessments") else {
        findings.fail("MISSING_ASSESSMENTS", "assessments must be an array");
        return (BTreeSet::new(), false, BTreeMap::new());
    };
    let mut ids = BTreeSet::new();
    let mut completed_screen = false;
    let mut evidence_by_id = BTreeMap::new();
    for (index, assessment) in assessments.iter().enumerate() {
        let Some(id) = get_str(assessment, "assessment_id").filter(|id| !id.trim().is_empty()) else {
            findings.fail("INVALID_ASSESSMENT_ID", format!("assessments[{index}] has no assessment_id"));
            continue;
        };
        if !ids.insert(id.to_string()) {
            findings.fail("DUPLICATE_ASSESSMENT_ID", format!("duplicate assessment_id {id}"));
        }
        if get_str(assessment, "assessment_type") == Some("conjunction_screen")
            && get_str(assessment, "status") == Some("completed")
        {
            completed_screen = true;
        }
        if get_str(assessment, "status") != Some("completed") {
            findings.mark_incomplete(
                "ASSESSMENT_NOT_COMPLETED",
                format!("assessment {id} is not completed"),
            );
        }

        let references = assessment
            .get("input_artifact_ids")
            .map(|value| string_list(value, &format!("assessment {id} input_artifact_ids"), findings))
            .unwrap_or_default();
        for reference in references {
            if !artifact_ids.contains_key(&reference) {
                findings.fail(
                    "UNKNOWN_ASSESSMENT_ARTIFACT",
                    format!("assessment {id} references unknown artifact {reference}"),
                );
            }
        }

        let pair_id = get_str(assessment, "pair_id").map(str::to_string);
        let output_artifact_id = get_str(assessment, "output_artifact_id").map(str::to_string);
        let output_sha256 = get_str(assessment, "output_sha256").map(str::to_string);
        if let Some(output_id) = output_artifact_id.as_deref() {
            if let Some(expected_digest) = artifact_ids.get(output_id) {
                if output_sha256.as_deref() != Some(expected_digest.as_str()) {
                    findings.fail(
                        "ASSESSMENT_OUTPUT_DIGEST_MISMATCH",
                        format!("assessment {id} output_sha256 does not match output artifact {output_id}"),
                    );
                } else {
                    findings.check("assessment_output_artifact_binding");
                }
            } else {
                findings.fail(
                    "UNKNOWN_ASSESSMENT_OUTPUT_ARTIFACT",
                    format!("assessment {id} references unknown output artifact {output_id}"),
                );
            }
        } else if output_sha256.is_some() {
            findings.fail(
                "ASSESSMENT_OUTPUT_WITHOUT_ARTIFACT",
                format!("assessment {id} declares output_sha256 without output_artifact_id"),
            );
        }
        evidence_by_id.insert(
            id.to_string(),
            AssessmentEvidence {
                assessment_type: get_str(assessment, "assessment_type").unwrap_or("unknown").to_string(),
                status: get_str(assessment, "status").unwrap_or("unknown").to_string(),
                pair_id,
                output_artifact_id,
                output_sha256,
            },
        );

        let pc = assessment.get("collision_probability").unwrap_or(&Value::Null);
        match get_str(pc, "status").unwrap_or("unknown") {
            "reported" => {
                let value = pc.get("value").and_then(Value::as_f64);
                if !value.is_some_and(|number| number.is_finite() && (0.0..=1.0).contains(&number)) {
                    findings.fail("INVALID_REPORTED_PC", format!("assessment {id} reports Pc outside [0,1]"));
                }
                if !is_nonempty(get_str(pc, "method_id"))
                    || get_str(pc, "applicability") != Some("inside_profile")
                {
                    findings.fail(
                        "PC_WITHOUT_APPLICABILITY",
                        format!("assessment {id} reports Pc without an applicable method profile"),
                    );
                }
            }
            "not_reported" | "not_calculated" | "invalid" | "unsupported" => {
                if !pc.get("value").is_some_and(Value::is_null) {
                    findings.fail(
                        "ABSENT_PC_COERCED_TO_VALUE",
                        format!("assessment {id} must not attach a numeric value to a non-reported Pc"),
                    );
                }
                if matches!(get_str(pc, "status"), Some("not_calculated" | "invalid" | "unsupported")) {
                    findings.mark_incomplete(
                        "COLLISION_PROBABILITY_NOT_QUALIFIED",
                        format!("assessment {id} has no qualified reported collision probability"),
                    );
                }
            }
            other => findings.fail("INVALID_PC_STATUS", format!("assessment {id} has invalid Pc status {other}")),
        }
    }
    (ids, completed_screen, evidence_by_id)
}

fn verify_pair_coverage(
    package: &Value,
    requested_ids: &BTreeSet<String>,
    artifact_digests: &BTreeMap<String, String>,
    assessments: &BTreeMap<String, AssessmentEvidence>,
    findings: &mut Findings,
) -> bool {
    let request = package.get("request").unwrap_or(&Value::Null);
    let primary = get_str(request, "primary_object_id").unwrap_or("");
    let expected_secondaries: BTreeSet<String> = requested_ids
        .iter()
        .filter(|object_id| object_id.as_str() != primary)
        .cloned()
        .collect();
    let expected_count = expected_secondaries.len();
    let ledger = package.get("pair_coverage").unwrap_or(&Value::Null);
    let Some(pairs) = get_array(ledger, "pairs") else {
        findings.fail("MISSING_PAIR_LEDGER", "pair_coverage.pairs must be an array");
        return false;
    };

    if get_str(ledger, "scope_kind") != Some("primary_vs_each_requested_secondary") {
        findings.fail("INVALID_PAIR_SCOPE", "pair coverage must enumerate the primary against every requested secondary");
    }
    if !get_str(ledger, "pair_manifest_sha256").is_some_and(is_sha256) {
        findings.fail("INVALID_PAIR_MANIFEST_DIGEST", "pair_manifest_sha256 must be a lowercase SHA-256 digest");
    }
    if get_u64(ledger, "requested_pair_count") != Some(expected_count as u64) {
        findings.fail(
            "REQUESTED_PAIR_COUNT_MISMATCH",
            format!("pair ledger must contain {expected_count} primary-secondary pairs"),
        );
    }

    let mut pair_ids = BTreeSet::new();
    let mut secondary_ids = BTreeSet::new();
    let mut retained_count = 0usize;
    let mut excluded_count = 0usize;
    let mut unresolved_count = 0usize;
    let mut unsupported_count = 0usize;
    let mut negative_ready = true;

    for (index, pair) in pairs.iter().enumerate() {
        let Some(pair_id) = get_str(pair, "pair_id").filter(|value| !value.trim().is_empty()) else {
            findings.fail("INVALID_PAIR_ID", format!("pair_coverage.pairs[{index}] has no pair_id"));
            negative_ready = false;
            continue;
        };
        if !pair_ids.insert(pair_id.to_string()) {
            findings.fail("DUPLICATE_PAIR_ID", format!("duplicate pair_id {pair_id}"));
        }

        let primary_id = get_str(pair, "primary_object_id").unwrap_or("");
        let secondary_id = get_str(pair, "secondary_object_id").unwrap_or("");
        if primary_id != primary {
            findings.fail(
                "PAIR_PRIMARY_MISMATCH",
                format!("pair {pair_id} primary does not match request.primary_object_id"),
            );
        }
        if !expected_secondaries.contains(secondary_id) {
            findings.fail(
                "UNREQUESTED_PAIR_SECONDARY",
                format!("pair {pair_id} secondary {secondary_id} is not a requested non-primary object"),
            );
        }
        if !secondary_ids.insert(secondary_id.to_string()) {
            findings.fail(
                "DUPLICATE_PAIR_SECONDARY",
                format!("requested secondary {secondary_id} appears in more than one pair"),
            );
        }

        let source_refs = pair
            .get("source_artifact_ids")
            .map(|value| string_list(value, &format!("pair {pair_id} source_artifact_ids"), findings))
            .unwrap_or_default();
        check_unique(&source_refs, &format!("pair {pair_id} source_artifact_ids"), findings);
        for artifact_id in &source_refs {
            if !artifact_digests.contains_key(artifact_id) {
                findings.fail(
                    "UNKNOWN_PAIR_SOURCE_ARTIFACT",
                    format!("pair {pair_id} references unknown source artifact {artifact_id}"),
                );
            }
        }

        let reason_codes = pair
            .get("reason_codes")
            .map(|value| string_list(value, &format!("pair {pair_id} reason_codes"), findings))
            .unwrap_or_default();
        let witness_id = get_str(pair, "prune_witness_artifact_id");
        let witness_sha = get_str(pair, "prune_witness_sha256");
        let method_profile = get_str(pair, "method_profile_id");
        let narrow_status = get_str(pair, "narrow_phase_status").unwrap_or("unknown");
        let narrow_assessment_id = get_str(pair, "narrow_phase_assessment_id");

        match get_str(pair, "disposition").unwrap_or("unknown") {
            "proven_excluded" => {
                excluded_count += 1;
                if reason_codes.is_empty() || !is_nonempty(method_profile) {
                    findings.fail(
                        "PRUNED_PAIR_WITHOUT_REASON_OR_METHOD",
                        format!("proven-excluded pair {pair_id} needs reason_codes and method_profile_id"),
                    );
                }
                let Some(artifact_id) = witness_id else {
                    findings.fail("PRUNED_PAIR_WITHOUT_WITNESS", format!("pair {pair_id} has no prune-witness artifact"));
                    negative_ready = false;
                    continue;
                };
                match (artifact_digests.get(artifact_id), witness_sha) {
                    (Some(actual), Some(declared)) if actual == declared && is_sha256(declared) => {
                        findings.check("pair_prune_witness_artifact_binding");
                    }
                    (None, _) => findings.fail(
                        "UNKNOWN_PRUNE_WITNESS_ARTIFACT",
                        format!("pair {pair_id} references unknown prune-witness artifact {artifact_id}"),
                    ),
                    _ => findings.fail(
                        "PRUNE_WITNESS_DIGEST_MISMATCH",
                        format!("pair {pair_id} prune-witness digest does not match retained artifact bytes"),
                    ),
                }
                if narrow_status != "not_applicable" || narrow_assessment_id.is_some() {
                    findings.fail(
                        "PRUNED_PAIR_HAS_NARROW_PHASE",
                        format!("proven-excluded pair {pair_id} must not claim a narrow-phase assessment"),
                    );
                    negative_ready = false;
                }
            }
            "retained_candidate" => {
                retained_count += 1;
                if !is_nonempty(method_profile) {
                    findings.fail("RETAINED_PAIR_WITHOUT_METHOD", format!("retained pair {pair_id} needs method_profile_id"));
                }
                if witness_id.is_some() || witness_sha.is_some() {
                    findings.fail(
                        "RETAINED_PAIR_HAS_PRUNE_WITNESS",
                        format!("retained candidate {pair_id} must not carry a prune witness"),
                    );
                }
                match narrow_status {
                    "searched_no_event" | "event_found" => {
                        let Some(assessment_id) = narrow_assessment_id else {
                            findings.fail(
                                "PAIR_NARROW_PHASE_WITHOUT_ASSESSMENT",
                                format!("pair {pair_id} has a completed narrow phase but no assessment reference"),
                            );
                            negative_ready = false;
                            continue;
                        };
                        match assessments.get(assessment_id) {
                            Some(assessment)
                                if assessment.assessment_type == "conjunction_screen"
                                    && assessment.status == "completed"
                                    && assessment.pair_id.as_deref() == Some(pair_id)
                                    && assessment.output_artifact_id.is_some()
                                    && assessment.output_sha256.is_some() =>
                            {
                                findings.check("pair_narrow_phase_assessment_binding");
                                if narrow_status != "searched_no_event" {
                                    negative_ready = false;
                                }
                            }
                            _ => {
                                findings.fail(
                                    "PAIR_NARROW_PHASE_ASSESSMENT_MISMATCH",
                                    format!("pair {pair_id} assessment {assessment_id} is not a completed, output-bound conjunction assessment for this pair"),
                                );
                                negative_ready = false;
                            }
                        }
                    }
                    "not_run" | "inconclusive" => {
                        if narrow_assessment_id.is_some() {
                            findings.fail(
                                "UNFINISHED_PAIR_HAS_ASSESSMENT",
                                format!("pair {pair_id} has an assessment reference despite narrow phase {narrow_status}"),
                            );
                        }
                        findings.mark_incomplete(
                            "PAIR_NARROW_PHASE_NOT_COMPLETE",
                            format!("retained pair {pair_id} has narrow-phase status {narrow_status}"),
                        );
                        negative_ready = false;
                    }
                    other => {
                        findings.fail(
                            "INVALID_PAIR_NARROW_PHASE_STATUS",
                            format!("retained pair {pair_id} has invalid narrow-phase status {other}"),
                        );
                        negative_ready = false;
                    }
                }
            }
            "unresolved" | "unsupported" => {
                if get_str(pair, "disposition") == Some("unresolved") {
                    unresolved_count += 1;
                } else {
                    unsupported_count += 1;
                }
                if reason_codes.is_empty() || witness_id.is_some() || witness_sha.is_some() || narrow_assessment_id.is_some() {
                    findings.fail(
                        "UNRESOLVED_PAIR_SEMANTICS_INVALID",
                        format!("unresolved/unsupported pair {pair_id} needs reasons and cannot claim a witness or assessment"),
                    );
                }
                if !matches!(narrow_status, "not_run" | "inconclusive") {
                    findings.fail(
                        "UNRESOLVED_PAIR_HAS_INVALID_NARROW_PHASE",
                        format!("unresolved/unsupported pair {pair_id} has invalid narrow-phase status {narrow_status}"),
                    );
                }
                findings.mark_incomplete(
                    "PAIR_UNIVERSE_HAS_UNRESOLVED",
                    format!("pair {pair_id} remains unresolved or unsupported"),
                );
                negative_ready = false;
            }
            other => {
                findings.fail("INVALID_PAIR_DISPOSITION", format!("pair {pair_id} has invalid disposition {other}"));
                negative_ready = false;
            }
        }
    }

    if secondary_ids != expected_secondaries || pairs.len() != expected_count {
        findings.fail(
            "PAIR_UNIVERSE_OMITTED_OR_EXTRA",
            format!("pair ledger does not enumerate every requested secondary exactly once; expected {expected_count} pairs"),
        );
        negative_ready = false;
    }

    for (field, expected) in [
        ("retained_candidate_count", retained_count),
        ("proven_excluded_count", excluded_count),
        ("unresolved_count", unresolved_count),
        ("unsupported_count", unsupported_count),
    ] {
        if get_u64(ledger, field) != Some(expected as u64) {
            findings.fail(
                "PAIR_COUNT_MISMATCH",
                format!("pair_coverage.{field} does not reconcile; expected {expected}"),
            );
        }
    }
    if retained_count + excluded_count + unresolved_count + unsupported_count != expected_count {
        findings.fail(
            "PAIR_DISPOSITIONS_DO_NOT_RECONCILE",
            "retained, proven-excluded, unresolved and unsupported counts must sum to requested pairs",
        );
        negative_ready = false;
    }

    match get_str(ledger, "disposition").unwrap_or("unknown") {
        "complete" => {
            if pairs.len() != expected_count || secondary_ids != expected_secondaries {
                findings.fail(
                    "FALSE_COMPLETE_PAIR_COVERAGE",
                    "pair ledger is marked complete but does not enumerate the full requested pair universe",
                );
                negative_ready = false;
            } else {
                findings.check("complete_primary_secondary_pair_coverage");
            }
        }
        "partial" => {
            findings.mark_incomplete("PARTIAL_PAIR_COVERAGE", "pair ledger is partial");
            negative_ready = false;
        }
        "unknown" => {
            findings.mark_incomplete("UNKNOWN_PAIR_COVERAGE", "pair coverage is unknown");
            negative_ready = false;
        }
        "rejected" => {
            findings.fail("REJECTED_PAIR_COVERAGE", "pair coverage is rejected");
            negative_ready = false;
        }
        other => {
            findings.fail("INVALID_PAIR_COVERAGE_DISPOSITION", format!("invalid pair coverage disposition {other}"));
            negative_ready = false;
        }
    }

    negative_ready
}

fn verify_execution_and_authority(
    package: &Value,
    producer_revisions: &mut BTreeSet<(String, String)>,
    findings: &mut Findings,
) {
    let execution = package.get("execution_evidence").unwrap_or(&Value::Null);
    if let Some(revisions) = get_array(execution, "producer_revisions") {
        for (index, revision) in revisions.iter().enumerate() {
            let repo = get_str(revision, "repository").unwrap_or("");
            let sha = get_str(revision, "commit_sha").unwrap_or("");
            if repo.is_empty() || (sha.len() != 40 && sha.len() != 64) || !sha.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase()) {
                findings.fail(
                    "INVALID_PRODUCER_REVISION",
                    format!("producer_revisions[{index}] lacks repository or exact lowercase commit SHA"),
                );
            } else {
                producer_revisions.insert((repo.to_string(), sha.to_string()));
            }
        }
    }

    let mut successful_required_run = false;
    if let Some(runs) = get_array(execution, "workflow_runs") {
        for (index, run) in runs.iter().enumerate() {
            let repo = get_str(run, "repository").unwrap_or("");
            let head = get_str(run, "head_sha").unwrap_or("");
            let matches_producer = producer_revisions.contains(&(repo.to_string(), head.to_string()));
            if !matches_producer {
                findings.fail(
                    "WORKFLOW_HEAD_NOT_BOUND",
                    format!("workflow_runs[{index}] head SHA is not bound to a producer revision in the same repository"),
                );
            }
            let required = run.get("required").and_then(Value::as_bool).unwrap_or(false);
            match get_str(run, "conclusion").unwrap_or("unknown") {
                "success" if required && matches_producer => successful_required_run = true,
                "failure" if required => findings.fail(
                    "REQUIRED_WORKFLOW_FAILED",
                    format!("required workflow run {} failed", get_str(run, "workflow_run_id").unwrap_or("unknown")),
                ),
                "cancelled" if required => findings.mark_incomplete(
                    "REQUIRED_WORKFLOW_CANCELLED",
                    format!("required workflow run {} was cancelled", get_str(run, "workflow_run_id").unwrap_or("unknown")),
                ),
                "queued" | "in_progress" | "skipped" | "blocked" | "stale" | "unknown" if required => findings.mark_incomplete(
                    "REQUIRED_WORKFLOW_NOT_PASSED",
                    format!("required workflow run {} is not a successful exact-head run", get_str(run, "workflow_run_id").unwrap_or("unknown")),
                ),
                "not_applicable" => {}
                "success" | "failure" | "cancelled" | "queued" | "in_progress" | "skipped" | "blocked" | "stale" | "unknown" => {}
                other => findings.fail("INVALID_WORKFLOW_CONCLUSION", format!("workflow_runs[{index}] has invalid conclusion {other}")),
            }
        }
    }

    let replay = execution.get("replay").unwrap_or(&Value::Null);
    match get_str(replay, "status").unwrap_or("unknown") {
        "reproduced" => {
            if !is_nonempty(get_str(replay, "verifier_id"))
                || !get_str(replay, "input_manifest_sha256").is_some_and(is_sha256)
                || !get_str(replay, "output_sha256").is_some_and(is_sha256)
                || !successful_required_run
            {
                findings.fail(
                    "REPLAY_WITHOUT_EXACT_EXECUTION",
                    "reproduced replay requires verifier identity, valid digests and a successful required exact-head workflow run",
                );
            } else {
                findings.check("exact_head_execution_replay");
            }
        }
        "mismatch" => findings.fail("REPLAY_MISMATCH", "independent replay output does not match"),
        "not_run" | "blocked" | "inconclusive" => findings.mark_incomplete(
            "REPLAY_NOT_QUALIFIED",
            format!("replay status is {}", get_str(replay, "status").unwrap_or("unknown")),
        ),
        other => findings.fail("INVALID_REPLAY_STATUS", format!("invalid replay status {other}")),
    }

    let authority = package.get("authority").unwrap_or(&Value::Null);
    if authority.get("actuation_authority").and_then(Value::as_bool) != Some(false) {
        findings.fail(
            "ACTUATION_AUTHORITY_ESCALATION",
            "this package type is not permitted to grant spacecraft actuation authority",
        );
    }
    findings.check("non_actuating_authority_boundary");
}

fn verify_claims(
    package: &Value,
    artifact_ids: &BTreeMap<String, String>,
    assessment_ids: &BTreeSet<String>,
    requested_ids: &BTreeSet<String>,
    segment_ids: &BTreeSet<String>,
    findings: &mut Findings,
) {
    let package_class = get_str(package, "package_class").unwrap_or("unknown");
    let Some(claims) = get_array(package, "claim_matrix") else {
        findings.fail("MISSING_CLAIM_MATRIX", "claim_matrix must be an array");
        return;
    };
    if claims.is_empty() {
        findings.fail("EMPTY_CLAIM_MATRIX", "claim_matrix must contain at least one layer claim");
    }
    for (index, claim) in claims.iter().enumerate() {
        let outcome = get_str(claim, "outcome").unwrap_or("unknown");
        let layer = get_str(claim, "layer").unwrap_or("unknown");
        if package_class == "synthetic_only"
            && outcome == "supported"
            && matches!(layer, "operational_safety" | "physical_truth" | "covariance_realism")
        {
            findings.fail(
                "SYNTHETIC_CLAIM_ESCALATION",
                format!("synthetic package cannot support {layer} claim at claim_matrix[{index}]"),
            );
        }
        let references = claim
            .get("evidence_refs")
            .map(|value| string_list(value, &format!("claim_matrix[{index}].evidence_refs"), findings))
            .unwrap_or_default();
        for reference in references {
            let known = artifact_ids.contains_key(&reference)
                || assessment_ids.contains(&reference)
                || requested_ids.contains(&reference)
                || segment_ids.contains(&reference)
                || matches!(reference.as_str(), "request" | "coverage" | "execution" | "authority");
            if !known {
                findings.fail(
                    "UNKNOWN_CLAIM_EVIDENCE_REFERENCE",
                    format!("claim_matrix[{index}] references unknown evidence {reference}"),
                );
            }
        }
    }
}

fn verify_value(package: &Value, repo_root: &Path) -> Findings {
    let mut findings = Findings::default();
    let package_id = get_str(package, "package_id");
    if get_str(package, "schema_version") != Some("space-qualification-package/v0.1") {
        findings.fail("UNSUPPORTED_SCHEMA_VERSION", "schema_version is not space-qualification-package/v0.1");
    }
    if !is_nonempty(package_id) {
        findings.fail("MISSING_PACKAGE_ID", "package_id must be a non-empty string");
    }
    let package_class = get_str(package, "package_class").unwrap_or("unknown");
    if !matches!(package_class, "synthetic_only" | "public_reference_replay" | "operator_shadow" | "operational_record") {
        findings.fail("INVALID_PACKAGE_CLASS", format!("unsupported package_class {package_class}"));
    }
    let request = package.get("request").unwrap_or(&Value::Null);
    for (field, location) in [
        ("request_sha256", "request.request_sha256"),
        ("configuration_sha256", "request.configuration_sha256"),
    ] {
        if !get_str(request, field).is_some_and(is_sha256) {
            findings.fail("INVALID_REQUEST_DIGEST", format!("{location} must be a lowercase SHA-256 digest"));
        }
    }
    if !package.pointer("/coverage/coverage_manifest_sha256").and_then(Value::as_str).is_some_and(is_sha256) {
        findings.fail("INVALID_COVERAGE_DIGEST", "coverage_manifest_sha256 must be a lowercase SHA-256 digest");
    }
    if package_class == "synthetic_only" {
        findings.mark_incomplete(
            "SYNTHETIC_ONLY",
            "synthetic data can qualify fixture mechanics, not physical or operational claims",
        );
    }

    let artifacts = verify_source_artifacts(package, repo_root, &mut findings);
    let (requested, segments, screened, excluded, unassessed, segment_unassessed) =
        verify_object_and_coverage_ledgers(package, &artifacts, &mut findings);
    let (assessments, completed_screen, assessment_evidence) =
        verify_assessments(package, &artifacts, &mut findings);
    let pair_negative_ready =
        verify_pair_coverage(package, &requested, &artifacts, &assessment_evidence, &mut findings);

    let mut producer_revisions = BTreeSet::new();
    verify_execution_and_authority(package, &mut producer_revisions, &mut findings);

    verify_claims(
        package,
        &artifacts,
        &assessments,
        &requested,
        &segments,
        &mut findings,
    );

    let result = package.get("result").unwrap_or(&Value::Null);
    let result_disposition = get_str(result, "disposition").unwrap_or("unknown");
    let negative = result.get("negative_conclusion").and_then(Value::as_bool).unwrap_or(false);
    let coverage = package.get("coverage").unwrap_or(&Value::Null);
    let coverage_complete = get_str(coverage, "disposition") == Some("complete");
    if result_disposition == "complete" && !coverage_complete {
        findings.fail(
            "COMPLETE_RESULT_WITH_INCOMPLETE_COVERAGE",
            "package result cannot be complete when input/time coverage is not complete",
        );
    }
    if result_disposition == "indeterminate" {
        findings.mark_incomplete(
            "RESULT_INDETERMINATE",
            "package explicitly declares an indeterminate result",
        );
    }
    if negative {
        let all_objects_screened = screened == requested.len() && excluded == 0 && unassessed == 0;
        let all_segments_assessed = segment_unassessed == 0;
        if package_class == "synthetic_only"
            || result_disposition != "complete"
            || !coverage_complete
            || !all_objects_screened
            || !all_segments_assessed
            || !completed_screen
            || !pair_negative_ready
        {
            findings.fail(
                "UNSUPPORTED_NEGATIVE_CONCLUSION",
                "negative event-screening conclusion requires non-synthetic evidence, complete requested-object and time coverage, no exclusions, and a completed conjunction-screen assessment",
            );
        } else {
            findings.check("negative_conclusion_gate");
        }
    }

    if artifacts.is_empty() {
        findings.fail("EMPTY_ARTIFACT_SET", "package must bind at least one source artifact");
    }
    if !package.get("execution_evidence").is_some_and(Value::is_object) {
        findings.fail("MISSING_EXECUTION_EVIDENCE", "execution_evidence is required");
    }
    findings.check("package_cross_field_reconciliation");
    findings.check("claim_reference_integrity");
    findings
}

fn read_package(package_path: &Path) -> Result<Value, String> {
    let bytes = fs::read(package_path).map_err(|error| format!("cannot read package: {error}"))?;
    serde_json::from_slice(&bytes).map_err(|error| format!("package is not valid JSON: {error}"))
}

fn usage() -> &'static str {
    "Usage: space-qualification-verifier [--allow-incomplete] <package.json> <repository-root>"
}

fn main() {
    let mut args = env::args().skip(1);
    let mut allow_incomplete = false;
    let mut positional = Vec::new();
    while let Some(arg) = args.next() {
        if arg == "--allow-incomplete" {
            allow_incomplete = true;
        } else if arg == "--help" || arg == "-h" {
            println!("{}", usage());
            process::exit(0);
        } else {
            positional.push(arg);
        }
    }

    if positional.len() != 2 {
        eprintln!("{}", usage());
        process::exit(64);
    }

    let package_path = PathBuf::from(&positional[0]);
    let repo_root = PathBuf::from(&positional[1]);
    let package = match read_package(&package_path) {
        Ok(value) => value,
        Err(error) => {
            println!("{}", json!({
                "verifier": "mycelix-space-qualification-verifier/0.1.0",
                "status": "FAIL",
                "package_integrity": "FAIL",
                "failure_reason_codes": ["PACKAGE_READ_OR_PARSE_FAILURE"],
                "detail": error
            }));
            process::exit(2);
        }
    };

    let findings = verify_value(&package, &repo_root);
    let report = findings.as_json(get_str(&package, "package_id"));
    println!("{}", serde_json::to_string_pretty(&report).unwrap_or_else(|_| report.to_string()));
    match findings.status() {
        "PASS" => process::exit(0),
        "FAIL" => process::exit(2),
        _ if allow_incomplete => process::exit(0),
        _ => process::exit(3),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../../")
            .canonicalize()
            .expect("repository root must resolve from the test crate")
    }

    fn fixture() -> Value {
        serde_json::from_str(include_str!("../../../contracts/fixtures/synthetic-package-indeterminate-v0.1.json"))
            .expect("synthetic package fixture must parse")
    }

    #[test]
    fn exact_fixture_bytes_verify_but_synthetic_result_remains_incomplete() {
        let report = verify_value(&fixture(), &repo_root());
        assert_eq!(report.integrity(), "PASS");
        assert_eq!(report.status(), "INCOMPLETE");
        assert!(report.incomplete.contains_key("SYNTHETIC_ONLY"));
        assert!(report.incomplete.contains_key("RESULT_INDETERMINATE"));
        assert!(report.incomplete.contains_key("REPLAY_NOT_QUALIFIED"));
        assert!(report.failures.is_empty(), "{:?}", report.failures);
    }

    #[test]
    fn mutated_source_digest_is_rejected() {
        let mut package = fixture();
        package["source_artifacts"][0]["raw_bytes_sha256"] = Value::String("0".repeat(64));
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("ARTIFACT_DIGEST_MISMATCH"));
    }

    #[test]
    fn omitted_requested_object_is_rejected() {
        let mut package = fixture();
        package["object_dispositions"].as_array_mut().unwrap().pop();
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("REQUESTED_OBJECT_OMITTED"));
    }

    #[test]
    fn absent_optional_cannot_be_encoded_as_numeric_zero() {
        let mut package = fixture();
        package["source_artifacts"][0]["field_semantics"][0]["normalized_value"] = json!(0.0);
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("NONVALUE_FIELD_COERCED"));
    }

    #[test]
    fn complete_coverage_cannot_hide_a_partial_time_segment() {
        let mut package = fixture();
        package["coverage"]["time_segments"][0]["disposition"] = Value::String("partially_assessed".to_string());
        package["coverage"]["time_segments"][0]["reason_codes"] = json!(["SEARCH_BOUND_NOT_MET"]);
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("FALSE_COMPLETE_COVERAGE"));
    }

    #[test]
    fn synthetic_fixture_cannot_assert_a_negative_conclusion() {
        let mut package = fixture();
        package["result"]["disposition"] = Value::String("complete".to_string());
        package["result"]["negative_conclusion"] = Value::Bool(true);
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("UNSUPPORTED_NEGATIVE_CONCLUSION"));
    }

    #[test]
    fn omitted_requested_pair_is_rejected_even_when_object_ledger_is_complete() {
        let mut package = fixture();
        package["pair_coverage"]["pairs"].as_array_mut().unwrap().clear();
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("PAIR_UNIVERSE_OMITTED_OR_EXTRA"));
    }

    #[test]
    fn retained_pair_output_must_match_exact_result_artifact() {
        let mut package = fixture();
        package["assessments"][1]["output_sha256"] = Value::String("0".repeat(64));
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("ASSESSMENT_OUTPUT_DIGEST_MISMATCH"));
    }

    #[test]
    fn completed_pair_assessment_must_bind_the_same_pair_id() {
        let mut package = fixture();
        package["assessments"][1]["pair_id"] = Value::String("different-pair".to_string());
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("PAIR_NARROW_PHASE_ASSESSMENT_MISMATCH"));
    }

    #[test]
    fn retained_candidate_without_narrow_phase_blocks_negative_conclusion() {
        let mut package = fixture();
        package["pair_coverage"]["pairs"][0]["narrow_phase_status"] = Value::String("not_run".to_string());
        package["pair_coverage"]["pairs"][0]["narrow_phase_assessment_id"] = Value::Null;
        let report = verify_value(&package, &repo_root());
        assert!(report.incomplete.contains_key("PAIR_NARROW_PHASE_NOT_COMPLETE"));
    }

    #[test]
    fn pair_pruning_requires_a_digest_bound_witness() {
        let mut package = fixture();
        package["pair_coverage"]["retained_candidate_count"] = Value::from(0);
        package["pair_coverage"]["proven_excluded_count"] = Value::from(1);
        package["pair_coverage"]["pairs"][0]["disposition"] = Value::String("proven_excluded".to_string());
        package["pair_coverage"]["pairs"][0]["prune_witness_artifact_id"] = Value::String("synthetic-state-set".to_string());
        package["pair_coverage"]["pairs"][0]["prune_witness_sha256"] = Value::String("0".repeat(64));
        package["pair_coverage"]["pairs"][0]["narrow_phase_status"] = Value::String("not_applicable".to_string());
        package["pair_coverage"]["pairs"][0]["narrow_phase_assessment_id"] = Value::Null;
        package["pair_coverage"]["pairs"][0]["reason_codes"] = json!(["LOWER_BOUND_EXCEEDS_DECLARED_PROFILE"]);
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("PRUNE_WITNESS_DIGEST_MISMATCH"));
    }

    #[test]
    fn path_traversal_is_rejected() {
        let mut package = fixture();
        package["source_artifacts"][0]["artifact_path"] = Value::String("../../../../etc/passwd".to_string());
        let report = verify_value(&package, &repo_root());
        assert_eq!(report.status(), "FAIL");
        assert!(report.failures.contains_key("UNSAFE_ARTIFACT_PATH"));
    }
}
