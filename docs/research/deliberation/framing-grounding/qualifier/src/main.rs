use serde_json::Value;
use std::{collections::BTreeSet, env, fs, process::ExitCode};

const PROTOCOL: &str = "DELIB-FRAME-GROUND-001";
const LANGUAGES: [&str; 5] = ["en-ZA", "af-ZA", "zu-ZA", "xh-ZA", "st-ZA"];
const FIXTURE_COUNT: usize = 12;

fn arr<'a>(v: &'a Value, key: &str) -> &'a [Value] {
    v.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}
fn strv<'a>(v: &'a Value, key: &str) -> Option<&'a str> {
    v.get(key)?.as_str().filter(|s| !s.trim().is_empty())
}
fn fail(errors: &mut Vec<String>, text: impl Into<String>) { errors.push(text.into()); }

fn validate_intake(v: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if strv(v, "protocol_id") != Some(PROTOCOL) { fail(&mut errors, "intake protocol ID mismatch"); }
    if strv(v, "status") != Some("metadata_only_no_external_payloads") {
        fail(&mut errors, "intake must remain metadata-only in v0.1");
    }
    let rules = &v["global_rules"];
    for key in ["raw_external_payloads_committed","human_annotations_collected",
                "training_permitted_by_default","redistribution_permitted_by_default","human_study_authorized"] {
        if rules[key].as_bool() != Some(false) {
            fail(&mut errors, format!("global safety gate must remain false: {key}"));
        }
    }
    let sources = arr(v, "sources");
    if sources.len() < 6 { fail(&mut errors, "external source intake must include at least six audited resources"); }
    let mut ids = BTreeSet::new();
    for source in sources {
        let id = strv(source, "source_id").unwrap_or("");
        if id.is_empty() || !ids.insert(id.to_owned()) { fail(&mut errors, "source IDs must be unique and non-empty"); }
        if strv(source, "payload_status") != Some("metadata_only") {
            fail(&mut errors, format!("{id}: non-metadata payload is not allowed in v0.1"));
        }
        if !source["dataset_license"].is_string() || !source["participant_consent"].is_string() {
            fail(&mut errors, format!("{id}: license and consent states must be explicit"));
        }
        for permission in ["evaluation_use", "training_use", "redistribution_use"] {
            if source[permission].as_str() == Some("limited_declared_scope") {
                if source["dataset_license"].as_str() != Some("verified_for_declared_scope") {
                    fail(&mut errors, format!("{id}: {permission} requires a verified dataset license"));
                }
                if source["participant_consent"].as_str() != Some("verified_for_declared_scope") {
                    fail(&mut errors, format!("{id}: {permission} requires participant consent verified for that scope"));
                }
            }
        }
    }
    let language_rows = arr(v, "language_readiness");
    let mut seen = BTreeSet::new();
    for language in language_rows {
        let tag = strv(language, "language_tag").unwrap_or("");
        if !LANGUAGES.contains(&tag) || !seen.insert(tag.to_owned()) {
            fail(&mut errors, "language readiness tags must be unique and belong to the declared SA target set");
        }
        if strv(language, "required_authoring_mode") != Some("native_authored_or_native_adjudicated") {
            fail(&mut errors, format!("{tag}: native authoring/adjudication requirement missing"));
        }
        let count = language["admitted_items"].as_u64().unwrap_or(0);
        let receipts = language["native_review_receipts"].as_u64().unwrap_or(0);
        let status = strv(language, "status").unwrap_or("");
        if count > 0 && receipts == 0 {
            fail(&mut errors, format!("{tag}: admitted items require native review receipts"));
        }
        if status == "pilot_ready" && (count == 0 || receipts == 0) {
            fail(&mut errors, format!("{tag}: pilot_ready requires admitted items and native review receipts"));
        }
    }
    for required in LANGUAGES {
        if !seen.contains(required) { fail(&mut errors, format!("language readiness row missing: {required}")); }
    }
    errors
}

fn validate_fixtures(v: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if strv(v, "protocol_id") != Some(PROTOCOL)
        || strv(v, "status") != Some("synthetic_controls_only_no_human_annotations") {
        fail(&mut errors, "fixture document identity or status mismatch");
    }
    let controls = arr(v, "controls");
    if controls.len() != FIXTURE_COUNT || v["fixture_count"].as_u64() != Some(FIXTURE_COUNT as u64) {
        fail(&mut errors, "fixture count mismatch");
    }
    let mut ids = BTreeSet::new();
    for c in controls {
        let id = strv(c, "fixture_id").unwrap_or("");
        if id.is_empty() || !ids.insert(id.to_owned()) { fail(&mut errors, "fixture IDs must be unique and non-empty"); }
        if strv(c, "record_kind") != Some("synthetic_contract_fixture")
            || strv(c, "language_tag") != Some("en-ZA")
            || strv(c, "authorship_status") != Some("synthetic_english") {
            fail(&mut errors, format!("{id}: fixture must be explicitly synthetic English"));
        }
        if strv(c, "source_text").is_none() || strv(c, "variant_text").is_none() {
            fail(&mut errors, format!("{id}: source and variant text are required for synthetic controls"));
        }
        if !arr(c, "human_annotations").is_empty() || !arr(c, "model_annotations").is_empty() {
            fail(&mut errors, format!("{id}: synthetic contract controls cannot be passed off as human/model annotations"));
        }
        if strv(c, "adjudication_status") != Some("not_applicable_synthetic") {
            fail(&mut errors, format!("{id}: synthetic control cannot claim human adjudication"));
        }
        if !["meaning_equivalent","disclosed_tradeoff","meaning_changed","underspecified_or_ambiguous"]
            .contains(&strv(c, "synthetic_relation_control").unwrap_or("")) {
            fail(&mut errors, format!("{id}: invalid semantic relation control"));
        }
    }
    errors
}

fn run(args: &[String]) -> Result<(), String> {
    if args.len() != 2 { return Err("usage: qualifier INTAKE_JSON SYNTHETIC_FIXTURES_JSON".into()); }
    let intake: Value = serde_json::from_str(&fs::read_to_string(&args[0]).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let fixtures: Value = serde_json::from_str(&fs::read_to_string(&args[1]).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let a = validate_intake(&intake);
    let b = validate_fixtures(&fixtures);
    if !a.is_empty() || !b.is_empty() {
        return Err(a.into_iter().chain(b).collect::<Vec<_>>().join("; "));
    }
    println!("QUALIFIED: source_records={} languages={} synthetic_controls={} external_payloads=0 human_annotations=0 human_study_authorized=false",
        arr(&intake, "sources").len(), arr(&intake, "language_readiness").len(), arr(&fixtures, "controls").len());
    Ok(())
}
fn main() -> ExitCode {
    match run(&env::args().skip(1).collect::<Vec<_>>()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => { eprintln!("NOT_QUALIFIED: {e}"); ExitCode::FAILURE }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn intake() -> Value {
        serde_json::from_str(include_str!("../../FRAMING_EXTERNAL_INTAKE_V0_1.json")).unwrap()
    }
    fn fixtures() -> Value {
        serde_json::from_str(include_str!("../../FRAMING_CONTRACT_FIXTURES_V0_1.json")).unwrap()
    }
    #[test]
    fn frozen_metadata_intake_passes() { assert!(validate_intake(&intake()).is_empty()); }
    #[test]
    fn raw_external_payload_is_rejected() {
        let mut v=intake();
        v["sources"][0]["payload_status"]=Value::String("downloaded".into());
        assert!(!validate_intake(&v).is_empty());
    }
    #[test]
    fn default_training_permission_is_never_assumed() {
        let mut v=intake();
        v["global_rules"]["training_permitted_by_default"]=Value::Bool(true);
        assert!(!validate_intake(&v).is_empty());
    }
    #[test]
    fn dataset_license_and_paper_license_are_separate_fields() {
        let v=intake();
        assert!(v["sources"][0]["paper_license"].is_string());
        assert_eq!(v["sources"][0]["dataset_license"],"not_located");
    }
    #[test]
    fn native_review_receipt_required_for_non_english_admission() {
        let mut v=intake();
        v["language_readiness"][1]["admitted_items"]=serde_json::json!(1);
        assert!(!validate_intake(&v).is_empty());
    }
    #[test]
    fn all_five_target_languages_are_present() { assert!(validate_intake(&intake()).is_empty()); }
    #[test]
    fn frozen_synthetic_controls_pass() { assert!(validate_fixtures(&fixtures()).is_empty()); }
    #[test]
    fn duplicate_fixture_ids_are_rejected() {
        let mut v=fixtures();
        let first=v["controls"][0].clone();
        v["controls"][1]=first;
        assert!(!validate_fixtures(&v).is_empty());
    }
    #[test]
    fn human_annotations_cannot_be_fabricated_in_synthetic_controls() {
        let mut v=fixtures();
        v["controls"][0]["human_annotations"]=serde_json::json!([{"annotator":"fake"}]);
        assert!(!validate_fixtures(&v).is_empty());
    }
    #[test]
    fn synthetic_fixtures_cannot_claim_adjudication() {
        let mut v=fixtures();
        v["controls"][0]["adjudication_status"]=Value::String("resolved_with_receipt".into());
        assert!(!validate_fixtures(&v).is_empty());
    }
    #[test]
    fn unresolved_language_is_not_promoted_to_native_coverage() {
        let mut v=intake();
        v["language_readiness"][2]["status"]=Value::String("pilot_ready".into());
        assert!(!validate_intake(&v).is_empty()); // Receipt count remains zero.
    }
    #[test]
    fn relation_unknowns_are_rejected() {
        let mut v=fixtures();
        v["controls"][0]["synthetic_relation_control"]=Value::String("neutral".into());
        assert!(!validate_fixtures(&v).is_empty());
    }
    #[test]
    fn pilot_ready_requires_native_admitted_items_and_review_receipts() {
        let mut v=intake();
        v["language_readiness"][1]["status"]=Value::String("pilot_ready".into());
        assert!(!validate_intake(&v).is_empty());
        v["language_readiness"][1]["admitted_items"]=serde_json::json!(4);
        v["language_readiness"][1]["native_review_receipts"]=serde_json::json!(1);
        assert!(validate_intake(&v).is_empty());
    }
    #[test]
    fn evaluation_permission_also_requires_license_and_consent_scope() {
        let mut v=intake();
        v["sources"][0]["evaluation_use"]=Value::String("limited_declared_scope".into());
        v["sources"][0]["dataset_license"]=Value::String("verified_for_declared_scope".into());
        assert!(!validate_intake(&v).is_empty());
        v["sources"][0]["participant_consent"]=Value::String("verified_for_declared_scope".into());
        assert!(validate_intake(&v).is_empty());
    }
    #[test]
    fn contradictory_no_frame_and_frame_labels_are_not_a_human_annotation() {
        // The schema contract keeps this distinction explicit; the synthetic fixture path
        // is intentionally not allowed to contain any human annotation payload.
        let mut v=fixtures();
        v["controls"][0]["human_annotations"]=serde_json::json!([{
            "frame_functions":["define_problem"],"no_frame":true,"uncertain":false,"abstained":false
        }]);
        assert!(!validate_fixtures(&v).is_empty());
    }

}
