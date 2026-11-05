#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde::Serialize;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Serialize)]
struct CaseResult {
    case_id: String,
    actual_disposition: String,
    expected_disposition: String,
    passed: bool,
    reason_codes: Vec<String>,
}

#[derive(Debug)]
struct Assessment {
    disposition: String,
    reasons: BTreeSet<String>,
}

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
    value.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn validate(label: &str, schema: &Value, instance: &Value) -> Vec<String> {
    if let Err(error) = jsonschema::draft202012::meta::validate(schema) {
        return vec![format!("{label}: invalid Draft 2020-12 schema: {error}")];
    }
    let validator = match draft202012::options().should_validate_formats(true).build(schema) {
        Ok(validator) => validator,
        Err(error) => return vec![format!("{label}: schema compile failed: {error}")],
    };
    validator.iter_errors(instance).map(|e| format!("{label}: {} at {}", e, e.instance_path())).collect()
}

fn has_obligation(contract: &Value, name: &str) -> bool {
    array(contract.get("procedure").unwrap_or(&Value::Null), "required_obligations")
        .iter().any(|v| v.as_str() == Some(name))
}

fn assess(case: &Value) -> Assessment {
    let contract = &case["contract"];
    let mut reasons = BTreeSet::new();
    let mut disposition = "insufficient_evidence".to_owned();

    if string(contract, "applicability") == Some("not_applicable") {
        if string(contract, "not_applicable_rationale").is_some_and(|v| !v.trim().is_empty()) {
            reasons.insert("not_applicable_rationale_present".to_owned());
            return Assessment { disposition: "not_applicable_with_rationale".to_owned(), reasons };
        }
        reasons.insert("not_applicable_without_rationale".to_owned());
        return Assessment { disposition: "indeterminate".to_owned(), reasons };
    }

    let evidence = array(contract, "evidence");
    let evidence_by_id = evidence.iter().filter_map(|e| string(e, "evidence_id").map(|id| (id, e))).collect::<BTreeMap<_, _>>();
    for claim in array(contract, "candidate_claims") {
        for ref_id in array(claim, "evidence_refs").iter().filter_map(Value::as_str) {
            if !evidence_by_id.contains_key(ref_id) {
                reasons.insert("candidate_claim_references_unknown_evidence".to_owned());
                return Assessment { disposition: "indeterminate".to_owned(), reasons };
            }
        }
    }

    let relevant = evidence.iter().filter(|e| e.get("purpose_fit").and_then(Value::as_bool) == Some(true)).collect::<Vec<_>>();
    if let Some(_contradiction) = relevant.iter().find(|e| {
        string(e, "polarity") == Some("contradicts")
            && string(e, "source_authentication") == Some("qualified")
            && string(e, "source_authority") == Some("qualified")
            && string(e, "temporal_fit") != Some("outside_reporting_period")
            && !matches!(string(e, "transformation_state"), Some("altered_or_conflicted" | "derived_without_lineage" | "unknown"))
    }) {
        reasons.insert("qualified_contradictory_evidence_present".to_owned());
        return Assessment { disposition: "contradicted".to_owned(), reasons };
    }

    if has_obligation(contract, "purpose_fit") && relevant.is_empty() {
        reasons.insert("purpose_fit_missing".to_owned());
        disposition = "insufficient_evidence".to_owned();
    }

    if has_obligation(contract, "source_authentication") {
        let auth_qualified = relevant.iter().any(|e| string(e, "source_authentication") == Some("qualified"));
        if !auth_qualified {
            let unqualified = relevant.iter().any(|e| string(e, "source_authentication") == Some("unqualified"));
            let unknown = relevant.iter().any(|e| string(e, "source_authentication") == Some("unknown"));
            if unqualified {
                reasons.insert("source_authentication_unqualified".to_owned());
                disposition = "needs_authorized_review".to_owned();
            } else if unknown {
                reasons.insert("source_authentication_unknown".to_owned());
                disposition = "indeterminate".to_owned();
            } else {
                reasons.insert("source_authentication_missing".to_owned());
                disposition = "insufficient_evidence".to_owned();
            }
        } else {
            reasons.insert("source_authentication_met".to_owned());
        }
    }

    if has_obligation(contract, "issuer_authority") {
        let authority_qualified = relevant.iter().any(|e| string(e, "source_authority") == Some("qualified"));
        if !authority_qualified {
            let unqualified = relevant.iter().any(|e| string(e, "source_authority") == Some("unqualified"));
            let unknown = relevant.iter().any(|e| string(e, "source_authority") == Some("unknown"));
            if unqualified {
                reasons.insert("issuer_authority_unqualified".to_owned());
                disposition = "needs_authorized_review".to_owned();
            } else if unknown {
                reasons.insert("issuer_authority_unknown".to_owned());
                disposition = "indeterminate".to_owned();
            } else {
                reasons.insert("issuer_authority_missing".to_owned());
                disposition = "insufficient_evidence".to_owned();
            }
        } else {
            reasons.insert("issuer_authority_met".to_owned());
        }
    }

    if has_obligation(contract, "population_completeness") {
        let population = &contract["population"];
        let counts_match = population["expected_count"].as_u64().is_some()
            && population["expected_count"] == population["observed_count"];
        if string(population, "scope_status") != Some("qualified") || !counts_match {
            reasons.insert("population_completeness_not_qualified".to_owned());
            disposition = "insufficient_evidence".to_owned();
        } else {
            reasons.insert("population_completeness_met".to_owned());
        }
    }

    if has_obligation(contract, "independent_corroboration") {
        let groups = relevant.iter().filter(|e| {
            string(e, "polarity") == Some("supports")
                && string(e, "source_authentication") == Some("qualified")
                && string(e, "source_authority") == Some("qualified")
        }).filter_map(|e| string(e, "origin_group_ref")).collect::<BTreeSet<_>>();
        let min = contract["procedure"]["min_independent_origins"].as_u64().unwrap_or(1) as usize;
        if groups.len() < min {
            reasons.insert("independent_corroboration_missing".to_owned());
            disposition = "insufficient_evidence".to_owned();
        } else {
            reasons.insert("independent_corroboration_met".to_owned());
        }
    }

    if has_obligation(contract, "temporal_coverage") {
        if !relevant.iter().any(|e| string(e, "temporal_fit") == Some("within_reporting_period")) {
            reasons.insert("temporal_coverage_missing".to_owned());
            disposition = "insufficient_evidence".to_owned();
        } else {
            reasons.insert("temporal_coverage_met".to_owned());
        }
    }

    if has_obligation(contract, "transformation_lineage") {
        let lineage_ok = relevant.iter().any(|e| matches!(
            string(e, "transformation_state"),
            Some("original_preserved" | "derived_with_lineage")
        ));
        let lineage_bad = relevant.iter().any(|e| matches!(
            string(e, "transformation_state"),
            Some("derived_without_lineage" | "altered_or_conflicted" | "unknown")
        ));
        if !lineage_ok || lineage_bad {
            reasons.insert("transformation_lineage_unqualified".to_owned());
            disposition = "indeterminate".to_owned();
        } else {
            reasons.insert("transformation_lineage_met".to_owned());
        }
    }

    if has_obligation(contract, "authorization") {
        let approved = relevant.iter().any(|e| {
            string(e, "evidence_class") == Some("approval_record")
                && string(e, "source_authentication") == Some("qualified")
                && string(e, "source_authority") == Some("qualified")
        });
        if !approved {
            reasons.insert("authorization_not_qualified".to_owned());
            disposition = "needs_authorized_review".to_owned();
        } else {
            reasons.insert("authorization_met".to_owned());
        }
    }

    if reasons.is_empty() && !relevant.is_empty() {
        reasons.insert("purpose_fit_met".to_owned());
    }
    let all_required_met = contract["procedure"]["required_obligations"].as_array().is_some_and(|items| {
        items.iter().filter_map(Value::as_str).all(|name| match name {
            "purpose_fit" => !relevant.is_empty(),
            "source_authentication" => reasons.contains("source_authentication_met"),
            "issuer_authority" => reasons.contains("issuer_authority_met"),
            "population_completeness" => reasons.contains("population_completeness_met"),
            "independent_corroboration" => reasons.contains("independent_corroboration_met"),
            "temporal_coverage" => reasons.contains("temporal_coverage_met"),
            "transformation_lineage" => reasons.contains("transformation_lineage_met"),
            "authorization" => reasons.contains("authorization_met"),
            _ => false,
        })
    });
    if all_required_met {
        disposition = "supported_within_declared_scope".to_owned();
        if relevant.is_empty() {
            disposition = "insufficient_evidence".to_owned();
        }
    }
    Assessment { disposition, reasons }
}

pub fn verify(input_bytes: &[u8], input_schema_bytes: &[u8], oracle_bytes: &[u8], oracle_schema_bytes: &[u8]) -> Value {
    let mut errors = Vec::<String>::new();
    let input: Value = serde_json::from_slice(input_bytes).unwrap_or_else(|e| { errors.push(format!("input JSON: {e}")); Value::Null });
    let input_schema: Value = serde_json::from_slice(input_schema_bytes).unwrap_or_else(|e| { errors.push(format!("input schema JSON: {e}")); Value::Null });
    let oracle: Value = serde_json::from_slice(oracle_bytes).unwrap_or_else(|e| { errors.push(format!("oracle JSON: {e}")); Value::Null });
    let oracle_schema: Value = serde_json::from_slice(oracle_schema_bytes).unwrap_or_else(|e| { errors.push(format!("oracle schema JSON: {e}")); Value::Null });
    if errors.is_empty() {
        errors.extend(validate("candidate input", &input_schema, &input));
        errors.extend(validate("public oracle", &oracle_schema, &oracle));
    }
    let mut rows=Vec::<CaseResult>::new();
    if errors.is_empty() {
        let input_cases=array(&input,"cases");
        let oracle_cases=array(&oracle,"cases");
        let mut oracle_by_id=BTreeMap::<String,&Value>::new();
        for row in oracle_cases {
            if let Some(id)=string(row,"case_id") {
                if oracle_by_id.insert(id.to_owned(),row).is_some(){errors.push(format!("duplicate oracle case: {id}"));}
            }
        }
        let mut seen=BTreeSet::new();
        for case in input_cases {
            let id=string(case,"case_id").unwrap_or("<missing>").to_owned();
            if !seen.insert(id.clone()){errors.push(format!("duplicate candidate case: {id}"));continue;}
            let Some(expected)=oracle_by_id.get(&id) else {errors.push(format!("missing oracle case: {id}"));continue;};
            let actual=assess(case);
            let expected_disposition=string(expected,"expected_disposition").unwrap_or("<missing>").to_owned();
            rows.push(CaseResult{case_id:id,passed:actual.disposition==expected_disposition,actual_disposition:actual.disposition,expected_disposition,reason_codes:actual.reasons.into_iter().collect()});
        }
        for id in oracle_by_id.keys(){if !seen.contains(id){errors.push(format!("oracle case without input: {id}"));}}
    }
    let passed=rows.iter().filter(|row|row.passed).count();
    let mut report=json!({
      "report_schema":"fin-acc-assertion-purpose.report.v1",
      "benchmark_id":"FIN-ACC-ASSERTION-PURPOSE",
      "dataset_version":"1.0.0",
      "status":if errors.is_empty() && !rows.is_empty() && passed==rows.len(){"PASS"}else{"FAIL"},
      "claim_scope":"public_synthetic_assertion_purpose_conformance_not_audit_sufficiency",
      "digests":{
        "candidate_input_sha256":sha256(input_bytes),
        "candidate_schema_sha256":sha256(input_schema_bytes),
        "public_oracle_sha256":sha256(oracle_bytes),
        "oracle_schema_sha256":sha256(oracle_schema_bytes),
        "verifier_source_sha256":sha256(include_bytes!("lib.rs"))
      },
      "case_count":rows.len(),
      "passed_case_count":passed,
      "errors":errors,
      "cases":rows
    });
    let report_schema:Value=serde_json::from_str(include_str!("../report.schema.json")).unwrap_or(Value::Null);
    for error in validate("report", &report_schema, &report) {
        report["status"]=Value::String("FAIL".to_owned());
        if let Some(items)=report["errors"].as_array_mut(){items.push(Value::String(error));}
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(raw:&str)->Value{serde_json::from_str(raw).expect("fixture JSON")}
    #[test]
    fn public_reference_cases_match_dispositions(){
        let input=fixture(include_str!("../../assertion_purpose_cases_v1.0.0.json"));
        let oracle=fixture(include_str!("../../public-reference-oracle/assertion_purpose_oracle_v1.0.0.json"));
        for case in array(&input,"cases"){
            let id=string(case,"case_id").expect("case id");
            let expected=array(&oracle,"cases").iter().find(|r|string(r,"case_id")==Some(id)).expect("oracle case");
            assert_eq!(assess(case).disposition,string(expected,"expected_disposition").expect("expected"),"{id}");
        }
    }
    #[test]
    fn mirrors_do_not_count_as_independent_origins(){
        let input=fixture(include_str!("../../assertion_purpose_cases_v1.0.0.json"));
        let case=array(&input,"cases").iter().find(|c|string(c,"case_id")==Some("AEP-003")).expect("AEP-003");
        assert_eq!(assess(case).disposition,"insufficient_evidence");
    }
    #[test]
    fn contradictory_qualified_evidence_prevents_support(){
        let input=fixture(include_str!("../../assertion_purpose_cases_v1.0.0.json"));
        let case=array(&input,"cases").iter().find(|c|string(c,"case_id")==Some("AEP-004")).expect("AEP-004");
        assert_eq!(assess(case).disposition,"contradicted");
    }
    #[test]
    fn nonexistent_candidate_citations_abstain(){
        let input=fixture(include_str!("../../assertion_purpose_cases_v1.0.0.json"));
        let case=array(&input,"cases").iter().find(|c|string(c,"case_id")==Some("AEP-014")).expect("AEP-014");
        assert_eq!(assess(case).disposition,"indeterminate");
    }
}
