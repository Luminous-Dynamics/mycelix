use serde_json::Value;
use std::{env, fs, process};

fn read_json(path: &str) -> Value {
    let bytes = fs::read(path).unwrap_or_else(|err| {
        eprintln!("cannot read {path}: {err}");
        process::exit(2);
    });
    serde_json::from_slice(&bytes).unwrap_or_else(|err| {
        eprintln!("invalid JSON in {path}: {err}");
        process::exit(2);
    })
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 2 {
        eprintln!("usage: evidence-plane-validator <schema.json> <receipt.json>");
        process::exit(2);
    }

    let schema = read_json(&args[0]);
    let receipt = read_json(&args[1]);

    if let Err(err) = jsonschema::meta::validate(&schema) {
        eprintln!("schema is invalid: {err}");
        process::exit(2);
    }

    let validator = jsonschema::draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .unwrap_or_else(|err| {
            eprintln!("cannot construct Draft 2020-12 validator: {err}");
            process::exit(2);
        });

    let errors: Vec<String> = validator
        .iter_errors(&receipt)
        .map(|err| format!("{err} at instance path {}", err.instance_path()))
        .collect();

    if !errors.is_empty() {
        for error in errors {
            eprintln!("INVALID: {error}");
        }
        process::exit(1);
    }

    println!("receipt conforms to the Evidence Plane schema; source truth, hash-to-byte equality, rights and scientific validity are not established by this check");
}

#[cfg(test)]
mod tests {
    use serde_json::{json, Value};
    use std::{fs, path::PathBuf};

    fn evidence_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
    }

    fn schema() -> Value {
        let path = evidence_dir().join("receipt-v0.schema.json");
        serde_json::from_slice(&fs::read(path).expect("read schema")).expect("parse schema JSON")
    }

    fn fixture() -> Value {
        let path = evidence_dir().join("examples/synthetic-planned-receipt-v0.json");
        serde_json::from_slice(&fs::read(path).expect("read fixture")).expect("parse fixture JSON")
    }

    fn errors(instance: &Value) -> Vec<String> {
        let schema = schema();
        let validator = jsonschema::draft202012::options()
            .should_validate_formats(true)
            .build(&schema)
            .expect("schema builds under Draft 2020-12");
        validator.iter_errors(instance).map(|e| e.to_string()).collect()
    }

    #[test]
    fn schema_document_passes_its_meta_schema() {
        let schema = schema();
        assert!(jsonschema::meta::validate(&schema).is_ok());
    }

    #[test]
    fn synthetic_planned_receipt_is_valid() {
        let value = fixture();
        assert!(errors(&value).is_empty(), "{:?}", errors(&value));
    }

    #[test]
    fn planned_receipt_cannot_claim_domain_validation() {
        let mut value = fixture();
        value["claim_ceiling"] = json!("domain_validated");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn planned_receipt_cannot_promote_its_evidence_tier() {
        let mut value = fixture();
        value["evidence_tier"] = json!("S4");
        value["claim_ceiling"] = json!("domain_validated");
        value["run_status"] = json!("domain_validated");
        value["gates"]["domain_validation"] = json!("pass");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn planned_receipt_cannot_mark_an_unknown_gate_as_pass() {
        let mut value = fixture();
        value["gates"]["source_integrity"] = json!("pass");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn missing_gate_is_rejected() {
        let mut value = fixture();
        value["gates"].as_object_mut().expect("gates object").remove("exact_head");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn malformed_sha256_is_rejected() {
        let mut value = fixture();
        value["subject"]["artifact_sha256"] = json!("sha256:looks-plausible-but-is-not-a-64-character-hex-digest");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn invalid_timestamp_is_rejected_when_format_checks_are_enabled() {
        let mut value = fixture();
        value["created_at"] = json!("yesterday, sometime");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn unknown_root_fields_are_rejected() {
        let mut value = fixture();
        value["trust_me"] = json!(true);
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn source_bytes_require_byte_count_and_hash() {
        let mut value = fixture();
        value["sources"] = json!([{
            "source_id":"fixture-source",
            "url":"https://example.org/data.csv",
            "observed_at":"2026-10-22T00:00:00Z",
            "media_type":"text/csv",
            "raw_bytes_ingested":true,
            "byte_count":12,
            "sha256":null,
            "rights_status":"open_license",
            "contact_status":"not_required",
            "temporal_coverage":null,
            "spatial_coverage":null,
            "metadata_conflicts":[]
        }]);
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn s2_receipt_rejects_unresolved_stewardship() {
        let mut value = fixture();
        value["run_status"] = json!("executed");
        value["evidence_tier"] = json!("S2");
        value["claim_ceiling"] = json!("source_reproducible");
        value["sources"] = json!([{
            "source_id":"fixture-source",
            "url":"https://example.org/data.csv",
            "observed_at":"2026-10-22T00:00:00Z",
            "media_type":"text/csv",
            "raw_bytes_ingested":true,
            "byte_count":12,
            "sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "rights_status":"open_license",
            "contact_status":"acknowledged_not_contacted",
            "temporal_coverage":null,
            "spatial_coverage":null,
            "metadata_conflicts":[]
        }]);
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn s4_requires_independent_verification_and_exact_head() {
        let mut value = fixture();
        value["run_status"] = json!("domain_validated");
        value["evidence_tier"] = json!("S4");
        value["claim_ceiling"] = json!("domain_validated");
        value["gates"]["domain_validation"] = json!("pass");
        assert!(!errors(&value).is_empty());
    }
}
