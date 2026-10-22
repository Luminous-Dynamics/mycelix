use serde_json::{json, Value};
use std::collections::BTreeSet;

const LEDGER: &str = include_str!("../../../../fixtures/nrvca-johannesburg-water-source-ledger-v0.1.json");

fn fixture() -> Value {
    serde_json::from_str(LEDGER).expect("water evidence ledger parses")
}

fn non_empty<'a>(value: &'a Value, field: &str) -> Result<&'a str, String> {
    value[field]
        .as_str()
        .filter(|text| !text.trim().is_empty())
        .ok_or_else(|| format!("missing/non-empty field {field}"))
}

fn validate(root: &Value) -> Result<(), String> {
    if root["schema_version"].as_str() != Some("nrvca-service-evidence-ledger/0.1")
        || root["artifact_status"].as_str() != Some("source_discovery_and_reported_claims_only")
    {
        return Err("ledger must retain discovery-only evidence status".to_owned());
    }
    if root["as_of_date"].as_str() != Some("2026-10-22") {
        return Err("ledger reference date changed without review".to_owned());
    }
    if root["policy"]["automatic_dispatch_enabled"].as_bool() != Some(false)
        || root["policy"]["reported_assertion_is_not_operator_measurement"].as_bool() != Some(true)
        || root["policy"]["public_notice_is_not_proof_of_delivered_service"].as_bool() != Some(true)
        || root["policy"]["no_unknown_to_zero_conversion"].as_bool() != Some(true)
    {
        return Err("evidence or decision-right boundary weakened".to_owned());
    }

    let sources = root["source_registry"]
        .as_array()
        .ok_or_else(|| "source_registry must be an array".to_owned())?;
    if sources.len() != 6 {
        return Err(format!("expected six discovered sources, got {}", sources.len()));
    }
    let mut source_ids = BTreeSet::new();
    for source in sources {
        let id = non_empty(source, "source_id")?;
        if !source_ids.insert(id.to_owned()) {
            return Err(format!("duplicate source ID {id}"));
        }
        non_empty(source, "publisher")?;
        non_empty(source, "url")?;
        non_empty(source, "source_class")?;
        if source.get("sha256").is_none() {
            return Err(format!("{id}: digest field must exist even when explicitly null"));
        }
        if source["sha256"].is_null() {
            if source["source_html_bytes_acquired"].as_bool() == Some(true)
                || source["linked_asset_bytes_acquired"].as_bool() == Some(true)
                || source["report_bytes_acquired"].as_bool() == Some(true)
            {
                return Err(format!("{id}: acquired bytes cannot be represented with null digest"));
            }
        }
        if source["source_html_bytes_acquired"].as_bool() == Some(false)
            && source["sha256"].as_str().is_some()
        {
            return Err(format!("{id}: a digest must not be fabricated for unacquired HTML bytes"));
        }
        if source["source_class"].as_str() == Some("official_dashboard_report_index")
            && source["page_fetch_status"].as_str() == Some("403_forbidden_in_this_retrieval")
            && source["report_bytes_acquired"].as_bool() != Some(false)
        {
            return Err(format!("{id}: the blocked dashboard index cannot imply report acquisition"));
        }
        if source["source_id"].as_str() == Some("jhb-water-daily-notices") {
            if source["archive_is_machine_readable"].as_bool() != Some(false)
                || source["linked_asset_bytes_acquired"].as_bool() != Some(false)
                || source["asset_date_semantics"].as_str().is_none()
            {
                return Err("daily notice images must remain discovery-only until acquired and parsed".to_owned());
            }
        }
        if source["source_id"].as_str() == Some("dws-gauteng-water-security-dashboard-index")
            && source["listing_semantics"].as_str().is_none()
        {
            return Err("last visible dashboard entry must not be misrepresented as proof of no later reports".to_owned());
        }
    }

    let assertions = root["reported_assertions"]
        .as_array()
        .ok_or_else(|| "reported_assertions must be an array".to_owned())?;
    if assertions.len() != 3 {
        return Err(format!("expected three source-grounded assertions, got {}", assertions.len()));
    }
    let mut assertion_ids = BTreeSet::new();
    for item in assertions {
        let assertion_id = non_empty(item, "assertion_id")?;
        if !assertion_ids.insert(assertion_id.to_owned()) {
            return Err(format!("duplicate assertion ID {assertion_id}"));
        }
        let kind = non_empty(item, "assertion_type")?;
        if !matches!(
            kind,
            "official_reported_context"
                | "official_reported_operational_state"
                | "official_reported_operational_context"
        ) {
            return Err(format!("{assertion_id}: assertion improperly promoted to a measurement"));
        }
        let evidence_status = non_empty(item, "evidence_status")?;
        if evidence_status.contains("sensor")
            || evidence_status.contains("operator_measured")
            || evidence_status.contains("independently_measured")
        {
            return Err(format!("{assertion_id}: official statements cannot masquerade as telemetry"));
        }
        non_empty(item, "subject")?;
        non_empty(item, "publication_date")?;
        if item["source_ids"].as_array().is_none_or(|ids| ids.is_empty()) {
            return Err(format!("{assertion_id}: source links are required"));
        }
        for source_id in item["source_ids"].as_array().unwrap() {
            let source_id = source_id.as_str().ok_or_else(|| format!("{assertion_id}: non-string source ID"))?;
            if !source_ids.contains(source_id) {
                return Err(format!("{assertion_id}: unknown source ID {source_id}"));
            }
        }
        if item["geography"]["geocoded"].as_bool() != Some(false)
            || !item["geography"]["geometry"].is_null()
        {
            return Err(format!("{assertion_id}: reported geography cannot be silently geocoded"));
        }
        if item["reported_valid_period"].get("start").is_none()
            || item["reported_valid_period"].get("end").is_none()
        {
            return Err(format!("{assertion_id}: unknown event-time bounds must be explicitly null"));
        }
        if item.get("structured_measurement").is_some()
            && !item["structured_measurement"].is_null()
        {
            return Err(format!("{assertion_id}: structured measurement not supported by this evidence tier"));
        }
        if item["assertion_id"] == "bulk-supply-reported-normal-by-2026-02-04" {
            if item["value"].as_i64() != Some(5000)
                || item["unit"].as_str() != Some("million_litres_per_day")
                || item["evidence_status"].as_str() != Some("official_reported_quantity")
                || item["value_semantics"].as_str().is_none()
                || item["outcome_status"].as_str() == Some("customer_service_restored")
            {
                return Err("reported bulk-supply quantity must not become customer-delivery evidence".to_owned());
            }
        }
    }

    let requirements = root["unacquired_evidence_requirements"]
        .as_array()
        .ok_or_else(|| "unacquired_evidence_requirements missing".to_owned())?;
    if requirements.len() < 4 {
        return Err("image, dashboard, telemetry and independent-corroboration gaps must stay explicit".to_owned());
    }
    for requirement in requirements {
        non_empty(requirement, "requirement_id")?;
        non_empty(requirement, "needed")?;
        non_empty(requirement, "qualification_gate")?;
    }
    let ceiling = &root["claim_ceiling"];
    if ceiling["permitted"].as_array().is_none_or(|v| v.is_empty())
        || ceiling["not_permitted"].as_array().is_none_or(|v| v.is_empty())
        || !ceiling["not_permitted"].as_array().unwrap().iter().any(|v| {
            v.as_str().is_some_and(|s| s.contains("current water availability"))
        })
    {
        return Err("claim ceiling must forbid inferring current customer service from discovery sources".to_owned());
    }
    Ok(())
}

#[test]
fn source_ledger_preserves_access_status_and_claim_ceiling() {
    validate(&fixture()).expect("ledger remains discovery-only");
}

#[test]
fn all_assertions_are_linked_and_remain_distinct_from_telemetry() {
    let root = fixture();
    validate(&root).expect("valid ledger");
    assert_eq!(root["reported_assertions"].as_array().unwrap().len(), 3);
    assert!(root["reported_assertions"].as_array().unwrap().iter().all(|item| {
        item["geography"]["geocoded"] == false
            && item["geography"]["geometry"].is_null()
            && item["structured_measurement"].is_null()
    }));
}

#[test]
fn no_digest_is_asserted_for_unacquired_source_bytes() {
    let mut root = fixture();
    root["source_registry"][0]["sha256"] = json!("0123456789abcdef");
    assert!(validate(&root).is_err());

    let mut acquired_without_digest = fixture();
    acquired_without_digest["source_registry"][0]["source_html_bytes_acquired"] = json!(true);
    assert!(validate(&acquired_without_digest).is_err());

    let mut missing_digest_field = fixture();
    missing_digest_field["source_registry"][0].as_object_mut().unwrap().remove("sha256");
    assert!(validate(&missing_digest_field).is_err());
}

#[test]
fn notice_or_press_release_cannot_be_promoted_to_measured_service() {
    let mut root = fixture();
    root["reported_assertions"][0]["assertion_type"] = json!("operator_measured_timeseries");
    assert!(validate(&root).is_err());

    let mut customer_outcome = fixture();
    customer_outcome["reported_assertions"][1]["outcome_status"] = json!("customer_service_restored");
    assert!(validate(&customer_outcome).is_err());

    let mut geo = fixture();
    geo["reported_assertions"][0]["geography"]["geocoded"] = json!(true);
    assert!(validate(&geo).is_err());
}

#[test]
fn missing_source_links_and_duplicate_ids_fail_closed() {
    let mut missing_source = fixture();
    missing_source["reported_assertions"][0]["source_ids"][0] = json!("not-a-source");
    assert!(validate(&missing_source).is_err());

    let mut duplicate_source = fixture();
    duplicate_source["source_registry"][1]["source_id"] = json!("jhb-water-interruptions-page");
    assert!(validate(&duplicate_source).is_err());

    let mut duplicate_assertion = fixture();
    duplicate_assertion["reported_assertions"][1]["assertion_id"] = json!("gauteng-disruption-context-feb-2026");
    assert!(validate(&duplicate_assertion).is_err());
}
