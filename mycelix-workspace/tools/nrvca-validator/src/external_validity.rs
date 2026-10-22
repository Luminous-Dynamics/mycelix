//! Evidence-qualified benchmark admission and split-integrity checks for NRVCA.
//!
//! This module validates provenance, rights, independent adjudication, and
//! leakage controls around a benchmark corpus. A valid manifest is not evidence
//! that its sources are truthful or that a benchmark result is statistically
//! significant. External evaluation must remain disabled until the preregistered
//! evidence and sample-size gates are met.

use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceDiagnostic {
    pub code: String,
    pub path: String,
    pub message: String,
}

fn push(out: &mut Vec<EvidenceDiagnostic>, code: &str, path: &str, message: impl Into<String>) {
    out.push(EvidenceDiagnostic {
        code: format!("EXTVAL-{code}"),
        path: path.to_owned(),
        message: message.into(),
    });
}

fn string_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str).filter(|s| !s.trim().is_empty())
}

fn valid_sha256(text: &str) -> bool {
    text.len() == 64 && text.bytes().all(|b| b.is_ascii_hexdigit())
}

fn valid_label(text: &str) -> bool {
    ["comparable", "not_comparable", "indeterminate"].contains(&text)
}

fn allowed_primary_split(text: &str) -> bool {
    ["development", "validation", "provider_held_out_test"].contains(&text)
}

fn source_rights(source: &Value) -> (bool, bool) {
    let rights = &source["rights"];
    let status = string_field(rights, "status").unwrap_or("");
    let evaluation = rights["evaluation_allowed"].as_bool() == Some(true);
    let redistribution = rights["redistribution_allowed"].as_bool() == Some(true);
    let evidence = string_field(rights, "evidence_uri").is_some();
    let coherent = match status {
        "open_reuse" => evaluation && redistribution && evidence,
        "permission_granted" => evaluation && evidence,
        "evaluation_only" => evaluation && !redistribution && evidence,
        "restricted" | "prohibited" | "unknown" => !evaluation && !redistribution,
        _ => false,
    };
    (coherent, coherent && evaluation)
}

fn locator_ids(source: &Value) -> BTreeSet<String> {
    source.get("locators").and_then(Value::as_array)
        .into_iter().flatten()
        .filter_map(|item| string_field(item, "locator_id").map(str::to_owned))
        .collect()
}

/// Validate a single acquired source record. The source is eligible only when
/// actual bytes, a valid SHA-256 digest, a reproducible capture/extraction
/// record, exact locators, and coherent rights metadata are present.
pub fn validate_source_record(source: &Value, path: &str) -> Vec<EvidenceDiagnostic> {
    let mut out = Vec::new();
    if !source.is_object() {
        push(&mut out, "SOURCE-TYPE", path, "source record must be an object");
        return out;
    }
    for field in ["source_id", "publisher", "source_uri", "publication_id", "vintage", "captured_at"] {
        if string_field(source, field).is_none() {
            push(&mut out, "SOURCE-FIELD", &format!("{path}/{field}"), "required nonempty source provenance field missing");
        }
    }
    if source["bytes_acquired"].as_bool() != Some(true) {
        push(&mut out, "SOURCE-BYTES", &format!("{path}/bytes_acquired"), "source bytes must be acquired before external evaluation");
    }
    match string_field(source, "sha256") {
        Some(digest) if valid_sha256(digest) => {}
        _ => push(&mut out, "SOURCE-DIGEST", &format!("{path}/sha256"), "a 64-character hexadecimal SHA-256 digest is required"),
    }

    let extraction = &source["extraction"];
    if string_field(extraction, "method").is_none() || string_field(extraction, "tool_version").is_none() {
        push(&mut out, "EXTRACTION-RECIPE", &format!("{path}/extraction"), "extraction method and tool version are required for reproducibility");
    }

    match source.get("locators").and_then(Value::as_array) {
        Some(locators) if !locators.is_empty() => {
            let mut ids = BTreeSet::new();
            for (i, locator) in locators.iter().enumerate() {
                let lp = format!("{path}/locators/{i}");
                let id = string_field(locator, "locator_id");
                let location = string_field(locator, "locator");
                if id.is_none() || location.is_none() {
                    push(&mut out, "LOCATOR-FIELD", &lp, "locator_id and exact locator are required");
                }
                if let Some(id) = id {
                    if !ids.insert(id.to_owned()) {
                        push(&mut out, "LOCATOR-DUPLICATE", &format!("{lp}/locator_id"), "locator IDs must be unique within a source");
                    }
                }
            }
        }
        _ => push(&mut out, "LOCATORS-MISSING", &format!("{path}/locators"), "at least one exact source locator is required"),
    }

    let rights = &source["rights"];
    let status = string_field(rights, "status").unwrap_or("");
    let (coherent, eligible) = source_rights(source);
    if !coherent {
        push(&mut out, "RIGHTS-INCOHERENT", &format!("{path}/rights"), "rights status, evaluation/redistribution flags, and rights evidence must agree");
    }
    if !eligible {
        push(&mut out, "RIGHTS-NOT-EVALUABLE", &format!("{path}/rights"), format!("rights status '{status}' does not authorize benchmark evaluation"));
    }
    out
}

fn validate_side(
    side: &Value,
    side_path: &str,
    source_by_id: &BTreeMap<String, &Value>,
    out: &mut Vec<EvidenceDiagnostic>,
) {
    if !side.is_object() {
        push(out, "PAIR-SIDE-TYPE", side_path, "left/right contract side must be an object");
        return;
    }
    for field in ["source_id", "contract_id", "contract_locator_id", "provider_group_id", "vintage"] {
        if string_field(side, field).is_none() {
            push(out, "PAIR-SIDE-FIELD", &format!("{side_path}/{field}"), "required contract provenance field missing");
        }
    }
    let Some(source_id) = string_field(side, "source_id") else { return; };
    let Some(source) = source_by_id.get(source_id) else {
        push(out, "PAIR-SOURCE-UNRESOLVED", &format!("{side_path}/source_id"), format!("source does not resolve: {source_id}"));
        return;
    };
    if !source_rights(source).1 || source["bytes_acquired"].as_bool() != Some(true)
        || !string_field(source, "sha256").is_some_and(valid_sha256) {
        push(out, "PAIR-SOURCE-INELIGIBLE", &format!("{side_path}/source_id"), "pair references a source that is not pinned and rights-cleared for evaluation");
    }
    if let Some(locator_id) = string_field(side, "contract_locator_id") {
        if !locator_ids(source).contains(locator_id) {
            push(out, "PAIR-LOCATOR-UNRESOLVED", &format!("{side_path}/contract_locator_id"), "contract locator must resolve to a pinned locator in the source record");
        }
    }
}

fn validate_pair_record(
    pair: &Value,
    path: &str,
    source_by_id: &BTreeMap<String, &Value>,
    out: &mut Vec<EvidenceDiagnostic>,
) {
    if !pair.is_object() {
        push(out, "PAIR-TYPE", path, "adjudicated pair must be an object");
        return;
    }
    for field in ["pair_id", "split", "adjudicated_label", "adjudication_rationale"] {
        if string_field(pair, field).is_none() {
            push(out, "PAIR-FIELD", &format!("{path}/{field}"), "required adjudication field missing");
        }
    }
    let split = string_field(pair, "split").unwrap_or("");
    if !allowed_primary_split(split) && split != "temporal_test" {
        push(out, "SPLIT-UNKNOWN", &format!("{path}/split"), "split must be development, validation, provider_held_out_test, or temporal_test");
    }
    let label = string_field(pair, "adjudicated_label").unwrap_or("");
    if !valid_label(label) {
        push(out, "LABEL-UNKNOWN", &format!("{path}/adjudicated_label"), "unsupported three-way adjudicated label");
    }
    let evidence = string_field(pair, "evidence_sufficiency").unwrap_or("");
    if !["sufficient", "insufficient"].contains(&evidence) {
        push(out, "EVIDENCE-SUFFICIENCY", &format!("{path}/evidence_sufficiency"), "evidence_sufficiency must be sufficient or insufficient");
    } else if evidence == "insufficient" && label != "indeterminate" {
        push(out, "INSUFFICIENT-LABEL", &format!("{path}/adjudicated_label"), "insufficient evidence must produce an indeterminate gold label");
    }
    if pair["known_hard_mismatch"].as_bool() == Some(true)
        && (label != "not_comparable" || evidence != "sufficient") {
        push(out, "HARD-MISMATCH-LABEL", path, "a known hard mismatch requires sufficient evidence and a not_comparable gold label");
    }
    if pair["raters_blinded_to_comparator"].as_bool() != Some(true) {
        push(out, "RATER-BLINDING", &format!("{path}/raters_blinded_to_comparator"), "human raters must be blinded to comparator output");
    }

    validate_side(&pair["left"], &format!("{path}/left"), source_by_id, out);
    validate_side(&pair["right"], &format!("{path}/right"), source_by_id, out);

    let Some(raters) = pair.get("raters").and_then(Value::as_array) else {
        push(out, "RATERS-MISSING", &format!("{path}/raters"), "exactly two independent initial raters are required");
        return;
    };
    if raters.len() != 2 {
        push(out, "RATERS-COUNT", &format!("{path}/raters"), "exactly two independent initial raters are required");
        return;
    }
    let rater_ids: Vec<&str> = raters.iter().filter_map(|r| string_field(r, "rater_id")).collect();
    if rater_ids.len() != 2 || rater_ids[0] == rater_ids[1] {
        push(out, "RATER-INDEPENDENCE", &format!("{path}/raters"), "two distinct nonempty rater IDs are required");
    }
    let decisions: Vec<&str> = raters.iter().map(|r| string_field(r, "decision").unwrap_or("")).collect();
    if decisions.iter().any(|d| !valid_label(d)) {
        push(out, "RATER-DECISION", &format!("{path}/raters"), "each rater must record a valid three-way decision");
    }
    if raters.iter().any(|r| r["blinded_to_comparator"].as_bool() != Some(true)) {
        push(out, "RATER-BLINDING", &format!("{path}/raters"), "each rater must be individually marked blinded to comparator output");
    }

    if decisions.len() == 2 && decisions[0] == decisions[1] {
        if decisions[0] != label {
            push(out, "ADJUDICATED-LABEL-MISMATCH", &format!("{path}/adjudicated_label"), "when raters agree, the frozen gold label must match their decision");
        }
        if !pair.get("adjudicator").is_none_or(Value::is_null) {
            push(out, "UNNEEDED-ADJUDICATOR", &format!("{path}/adjudicator"), "agreement cases must not hide an unneeded override; preserve the raters' agreement");
        }
    } else {
        let adjudicator = &pair["adjudicator"];
        let adjudicator_id = string_field(adjudicator, "adjudicator_id").unwrap_or("");
        if adjudicator_id.is_empty()
            || rater_ids.iter().any(|id| *id == adjudicator_id)
            || string_field(adjudicator, "rationale").is_none()
            || string_field(adjudicator, "decision") != Some(label)
            || adjudicator["independent"].as_bool() != Some(true) {
            push(out, "ADJUDICATION-REQUIRED", &format!("{path}/adjudicator"), "rater disagreement requires a distinct independent adjudicator, rationale, and decision matching the gold label");
        }
    }
}

/// Validate one benchmark bundle. A preregistration with no observations is a
/// valid blocked artifact; setting external_evaluation_enabled=true activates
/// stronger sample-size and evidence admission gates.
pub fn validate_benchmark_bundle(bundle: &Value) -> Vec<EvidenceDiagnostic> {
    let mut out = Vec::new();
    if !bundle.is_object() {
        push(&mut out, "BUNDLE-TYPE", "", "bundle must be an object");
        return out;
    }
    if string_field(bundle, "schema_version") != Some("nrvca-external-validity-bundle/0.1") {
        push(&mut out, "SCHEMA-VERSION", "/schema_version", "expected nrvca-external-validity-bundle/0.1");
    }
    if string_field(bundle, "artifact_status") != Some("preregistered_not_executed") {
        push(&mut out, "ARTIFACT-STATUS", "/artifact_status", "new protocol must remain explicitly preregistered_not_executed until real evaluation is run");
    }
    let protocol = &bundle["protocol"];
    if !protocol.is_object() {
        push(&mut out, "PROTOCOL-MISSING", "/protocol", "protocol object is required");
    } else {
        if string_field(protocol, "comparability_contract_version") != Some("nrvca-kpi-dual-axis-comparability-benchmark/0.1") {
            push(&mut out, "CONTRACT-VERSION", "/protocol/comparability_contract_version", "unknown comparator contract version");
        }
        if protocol["adjudication"]["independent_initial_raters"].as_u64() != Some(2)
            || protocol["adjudication"]["third_rater_for_disagreement"].as_bool() != Some(true)
            || protocol["adjudication"]["raters_blinded_to_comparator"].as_bool() != Some(true) {
            push(&mut out, "ADJUDICATION-POLICY", "/protocol/adjudication", "protocol requires two independent blinded raters and a third independent adjudicator for disagreement");
        }
        if protocol["split_policy"]["random_pair_split_allowed"].as_bool() != Some(false)
            || protocol["split_policy"]["provider_group_disjoint_primary_splits"].as_bool() != Some(true)
            || protocol["split_policy"]["separate_temporal_vintage_test"].as_bool() != Some(true) {
            push(&mut out, "SPLIT-POLICY", "/protocol/split_policy", "provider-disjoint primary evaluation and a separate temporal test are mandatory; random pair split is forbidden");
        }
        if protocol["acceptance_targets"]["minimum_hard_mismatch_pairs"].as_u64().unwrap_or(0) < 300
            || protocol["acceptance_targets"]["false_comparable_hard_mismatch_max"].as_u64() != Some(0)
            || protocol["acceptance_targets"]["minimum_exact_three_way_agreement"].as_f64().unwrap_or(0.0) < 0.95
            || protocol["acceptance_targets"]["minimum_coverage_adequate_evidence"].as_f64().unwrap_or(0.0) < 0.80 {
            push(&mut out, "ACCEPTANCE-TARGETS", "/protocol/acceptance_targets", "preregistered targets must include >=300 hard mismatches, zero false-comparable decisions on that set, >=0.95 exact agreement, and >=0.80 coverage");
        }
        if protocol["reporting"]["per_provider_breakdown"].as_bool() != Some(true)
            || protocol["reporting"]["confidence_intervals"].as_bool() != Some(true)
            || protocol["reporting"]["abstention_and_coverage"].as_bool() != Some(true)
            || protocol["reporting"]["adjudicator_disagreement"].as_bool() != Some(true) {
            push(&mut out, "REPORTING-POLICY", "/protocol/reporting", "provider-stratified errors, confidence intervals, abstention/coverage, and adjudicator disagreement must be reported");
        }
    }

    let sources = match bundle.get("sources").and_then(Value::as_array) {
        Some(sources) => sources,
        None => {
            push(&mut out, "SOURCES-TYPE", "/sources", "sources must be an array");
            return out;
        }
    };
    let mut source_by_id = BTreeMap::new();
    let mut digest_to_source: BTreeMap<String, String> = BTreeMap::new();
    for (i, source) in sources.iter().enumerate() {
        let path = format!("/sources/{i}");
        for diagnostic in validate_source_record(source, &path) {
            out.push(diagnostic);
        }
        if let Some(id) = string_field(source, "source_id") {
            if source_by_id.insert(id.to_owned(), source).is_some() {
                push(&mut out, "SOURCE-ID-DUPLICATE", &format!("{path}/source_id"), "source IDs must be unique");
            }
        }
        if let (Some(id), Some(digest)) = (string_field(source, "source_id"), string_field(source, "sha256")) {
            if valid_sha256(digest) {
                if let Some(previous) = digest_to_source.insert(digest.to_ascii_lowercase(), id.to_owned()) {
                    if previous != id {
                        push(&mut out, "SOURCE-DIGEST-DUPLICATE", &format!("{path}/sha256"), "identical source bytes should share one source identity, not masquerade as distinct records");
                    }
                }
            }
        }
    }

    let pairs = match bundle.get("pairs").and_then(Value::as_array) {
        Some(pairs) => pairs,
        None => {
            push(&mut out, "PAIRS-TYPE", "/pairs", "pairs must be an array");
            return out;
        }
    };
    let mut pair_ids = BTreeSet::new();
    let mut pair_identity_to_split = BTreeMap::new();
    let mut provider_to_split: BTreeMap<String, String> = BTreeMap::new();
    let mut contract_to_split: BTreeMap<String, String> = BTreeMap::new();
    let mut digest_to_split: BTreeMap<String, String> = BTreeMap::new();
    let mut hard_mismatches = 0usize;
    for (i, pair) in pairs.iter().enumerate() {
        let path = format!("/pairs/{i}");
        validate_pair_record(pair, &path, &source_by_id, &mut out);
        let Some(pair_id) = string_field(pair, "pair_id") else { continue; };
        if !pair_ids.insert(pair_id.to_owned()) {
            push(&mut out, "PAIR-ID-DUPLICATE", &format!("{path}/pair_id"), "pair IDs must be unique");
        }
        if pair["known_hard_mismatch"].as_bool() == Some(true)
            && pair["adjudicated_label"].as_str() == Some("not_comparable")
            && pair["evidence_sufficiency"].as_str() == Some("sufficient") {
            hard_mismatches += 1;
        }
        let split = string_field(pair, "split").unwrap_or("");
        if allowed_primary_split(split) {
            let left = &pair["left"];
            let right = &pair["right"];
            let groups = [string_field(left, "provider_group_id"), string_field(right, "provider_group_id")];
            for group in groups.into_iter().flatten() {
                if let Some(previous) = provider_to_split.insert(group.to_owned(), split.to_owned()) {
                    if previous != split {
                        push(&mut out, "PROVIDER-LEAKAGE", &path, format!("provider group {group} occurs in both primary splits {previous} and {split}"));
                    }
                }
            }
            for side in [left, right] {
                if let Some(contract_id) = string_field(side, "contract_id") {
                    if let Some(previous) = contract_to_split.insert(contract_id.to_owned(), split.to_owned()) {
                        if previous != split {
                            push(&mut out, "CONTRACT-LEAKAGE", &path, format!("contract ID {contract_id} occurs in both primary splits {previous} and {split}"));
                        }
                    }
                }
                if let Some(source_id) = string_field(side, "source_id") {
                    if let Some(source) = source_by_id.get(source_id) {
                        if let Some(digest) = string_field(source, "sha256") {
                            if let Some(previous) = digest_to_split.insert(digest.to_ascii_lowercase(), split.to_owned()) {
                                if previous != split {
                                    push(&mut out, "DIGEST-LEAKAGE", &path, format!("source digest appears in both primary splits {previous} and {split}"));
                                }
                            }
                        }
                    }
                }
            }
        }
        if split == "temporal_test" {
            if string_field(&pair["left"], "vintage").is_none() || string_field(&pair["right"], "vintage").is_none() {
                push(&mut out, "TEMPORAL-VINTAGE", &path, "both sides of the separate temporal test need explicit vintages");
            }
        }
        let left_key = format!("{}|{}|{}", string_field(&pair["left"], "source_id").unwrap_or(""), string_field(&pair["left"], "contract_id").unwrap_or(""), string_field(&pair["left"], "contract_locator_id").unwrap_or(""));
        let right_key = format!("{}|{}|{}", string_field(&pair["right"], "source_id").unwrap_or(""), string_field(&pair["right"], "contract_id").unwrap_or(""), string_field(&pair["right"], "contract_locator_id").unwrap_or(""));
        let identity = if left_key <= right_key { format!("{left_key}<>|{right_key}") } else { format!("{right_key}<>|{left_key}") };
        if let Some(previous) = pair_identity_to_split.insert(identity, split.to_owned()) {
            push(&mut out, "PAIR-CROSS-SPLIT", &path, format!("same underlying contract pair appears in multiple splits (previous split {previous})"));
        }
    }

    if bundle["external_evaluation_enabled"].as_bool() == Some(true) {
        let minimum = protocol["acceptance_targets"]["minimum_hard_mismatch_pairs"].as_u64().unwrap_or(300) as usize;
        if pairs.is_empty() || sources.is_empty() {
            push(&mut out, "EXECUTION-NO-EVIDENCE", "/external_evaluation_enabled", "external evaluation cannot be enabled without source-pinned pair records");
        }
        if hard_mismatches < minimum {
            push(&mut out, "EXECUTION-HARD-MISMATCH-SAMPLE", "/pairs", format!("found {hard_mismatches} qualified hard-mismatch pairs; minimum is {minimum}"));
        }
        if !out.is_empty() {
            push(&mut out, "EXECUTION-QUALITY-GATES", "/external_evaluation_enabled", "external evaluation enabled with unresolved bundle diagnostics; fail closed and keep execution disabled");
        }
    } else if bundle["execution_state"].as_str() != Some("blocked_no_adjudicated_records") {
        push(&mut out, "EXECUTION-STATE", "/execution_state", "when external evaluation is disabled, execution_state must explain the block");
    }
    out
}
