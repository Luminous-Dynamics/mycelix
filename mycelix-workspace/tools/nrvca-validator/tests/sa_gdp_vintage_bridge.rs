use serde_json::{json, Value};
use std::collections::BTreeSet;

const FIXTURE: &str = include_str!("../../../../fixtures/nrvca-sa-gdp-table1-transcription-v0.1.json");
const SECTORS: &[&str] = &[
    "agriculture_forestry_fishing", "mining_quarrying", "manufacturing",
    "electricity_gas_water", "construction", "trade_catering_accommodation",
    "transport_storage_communication", "finance_real_estate_business_services",
    "general_government_services", "personal_services",
];
const VA: &str = "total_value_added_basic_prices";
const TAX: &str = "taxes_less_subsidies";
const GDP: &str = "gdp_market_prices";

fn fixture() -> Value { serde_json::from_str(FIXTURE).expect("fixture parses") }
fn integer(value: &Value, field: &str) -> Result<i64, String> {
    value[field].as_i64().ok_or_else(|| format!("missing or non-integer {field}"))
}

fn validate(root: &Value) -> Result<(), String> {
    if root["schema_version"].as_str() != Some("nrvca-sa-gdp-table1-transcription/0.1")
        || root["artifact_status"].as_str() != Some("discovery_only_unpinned_transcription") {
        return Err("fixture must retain discovery-only unpinned status".to_owned());
    }
    let source = &root["source"];
    if source["publisher"].as_str() != Some("Statistics South Africa")
        || source["publication_id"].as_str() != Some("P0441")
        || source["published_at"].as_str() != Some("2026-09-08T11:00:00+02:00")
        || source["table_id"].as_str() != Some("Table 1")
        || source["printed_page"].as_i64() != Some(8) {
        return Err("official source identity or vintage changed".to_owned());
    }
    if source["source_bytes_acquired"].as_bool() != Some(false) || !source["source_sha256"].is_null()
        || source["extraction_status"].as_str() != Some("not_independently_reparsed_from_source_bytes") {
        return Err("unacquired source bytes cannot carry a digest or claim parser replay".to_owned());
    }
    if source["source_document_url"].as_str() != Some("https://www.statssa.gov.za/publications/P0441/P04412ndQuarter2026.pdf")
        || source["companion_timeseries_xlsx_url"].as_str().is_none()
        || source["license_status"].as_str().is_none_or(str::is_empty) {
        return Err("source locator or license status missing".to_owned());
    }
    if root["claim_ceiling"]["evidence_status"].as_str() != Some("published_official_manual_transcription_unpinned") {
        return Err("claim ceiling changed without review".to_owned());
    }
    if root["units"]["price_basis"].as_str() != Some("constant_2015_prices")
        || root["units"]["currency"].as_str() != Some("ZAR")
        || root["units"]["scale"].as_str() != Some("million") {
        return Err("price basis or unit changed".to_owned());
    }
    let metrics = root["metrics"].as_array().ok_or_else(|| "metrics missing".to_owned())?;
    if metrics.len() != 13 { return Err("Table 1 must retain ten sectors and three aggregates".to_owned()); }
    let mut metric_ids = BTreeSet::new();
    for metric in metrics {
        let id = metric["metric_id"].as_str().ok_or_else(|| "metric ID missing".to_owned())?;
        if id.trim().is_empty() || !metric_ids.insert(id.to_owned()) { return Err(format!("empty/duplicate metric ID: {id}")); }
        if metric["unit"].as_str() != Some("ZAR_million_constant_2015_prices") { return Err(format!("unit mismatch for {id}")); }
    }
    for id in SECTORS.iter().copied().chain([VA, TAX, GDP]) {
        if !metric_ids.contains(id) { return Err(format!("required metric missing: {id}")); }
    }
    let identities = root["reconciliation_contract"].as_array().ok_or_else(|| "identity contract missing".to_owned())?;
    if identities.len() != 2
        || identities[0]["identity_id"].as_str() != Some("industry_value_added_sum")
        || identities[0]["tolerance_absolute"].as_i64() != Some(5)
        || identities[1]["identity_id"].as_str() != Some("gdp_at_market_prices_bridge")
        || identities[1]["tolerance_absolute"].as_i64() != Some(1) {
        return Err("published-rounding tolerances changed from reviewed bounds".to_owned());
    }
    let rows = root["rows"].as_array().ok_or_else(|| "rows missing".to_owned())?;
    if rows.len() != 20 { return Err(format!("expected 20 rows, got {}", rows.len())); }
    let mut keys = BTreeSet::new();
    let mut annual = 0;
    let mut quarterly = 0;
    for row in rows {
        let period = row["period"].as_str().ok_or_else(|| "period missing".to_owned())?;
        let frequency = row["frequency"].as_str().ok_or_else(|| format!("{period}: frequency missing"))?;
        if !keys.insert(format!("{frequency}:{period}")) { return Err(format!("duplicate period {frequency}:{period}")); }
        match frequency {
            "annual" => {
                annual += 1;
                if row["seasonal_adjustment"].as_str() != Some("not_seasonally_adjusted_annual")
                    || row["annualisation"].as_str() != Some("annual_total")
                    || !matches!(period, "2018" | "2019" | "2020" | "2021" | "2022" | "2023" | "2024" | "2025") {
                    return Err(format!("{period}: annual metadata inconsistent"));
                }
            }
            "quarterly" => {
                quarterly += 1;
                if row["seasonal_adjustment"].as_str() != Some("seasonally_adjusted")
                    || row["annualisation"].as_str() != Some("seasonally_adjusted_annualised_rate")
                    || !matches!(period, "2023-Q3" | "2023-Q4" | "2024-Q1" | "2024-Q2" | "2024-Q3" | "2024-Q4" | "2025-Q1" | "2025-Q2" | "2025-Q3" | "2025-Q4" | "2026-Q1" | "2026-Q2") {
                    return Err(format!("{period}: quarterly metadata inconsistent"));
                }
            }
            _ => return Err(format!("{period}: unsupported frequency")),
        }
        if row["source_locator"]["printed_page"].as_i64() != Some(8)
            || row["source_locator"]["table_id"].as_str() != Some("table_1") {
            return Err(format!("{period}: row locator missing"));
        }
        let values = &row["values"];
        let value_object = values.as_object().ok_or_else(|| format!("{period}: values missing"))?;
        if value_object.len() != metrics.len() { return Err(format!("{period}: metric coverage incomplete")); }
        for id in &metric_ids {
            let n = values[id.as_str()].as_i64().ok_or_else(|| format!("{period}: {id} not integer"))?;
            if n < 0 { return Err(format!("{period}: negative level for {id}")); }
        }
        let sector_sum: i64 = SECTORS.iter().map(|id| values[*id].as_i64().unwrap()).sum();
        let va = values[VA].as_i64().unwrap();
        let tax = values[TAX].as_i64().unwrap();
        let gdp = values[GDP].as_i64().unwrap();
        let sector_residual = sector_sum - va;
        let gdp_residual = va + tax - gdp;
        if row["displayed_rounding_residuals"]["industry_sector_sum_minus_total_value_added"].as_i64() != Some(sector_residual)
            || row["displayed_rounding_residuals"]["value_added_plus_taxes_minus_gdp"].as_i64() != Some(gdp_residual) {
            return Err(format!("{period}: residual receipt differs from displayed values"));
        }
        if sector_residual.abs() > 5 { return Err(format!("{period}: sector-sum rounding bound exceeded")); }
        if gdp_residual.abs() > 1 { return Err(format!("{period}: GDP-bridge rounding bound exceeded")); }
    }
    if annual != 8 || quarterly != 12
        || root["declared_row_counts"]["annual"].as_i64() != Some(8)
        || root["declared_row_counts"]["quarterly_saar"].as_i64() != Some(12)
        || root["declared_row_counts"]["total"].as_i64() != Some(20) {
        return Err("frequency-specific row counts do not reconcile".to_owned());
    }
    let annual_2025 = rows.iter().find(|r| r["period"] == "2025" && r["frequency"] == "annual").ok_or_else(|| "annual 2025 missing".to_owned())?;
    if annual_2025["values"][GDP].as_i64() != Some(4_716_601) { return Err("annual 2025 GDP differs from frozen table".to_owned()); }
    let q2 = rows.iter().find(|r| r["period"] == "2026-Q2" && r["frequency"] == "quarterly").ok_or_else(|| "2026 Q2 missing".to_owned())?;
    if q2["values"][GDP].as_i64() != Some(4_754_800) { return Err("2026 Q2 GDP differs from frozen table".to_owned()); }
    Ok(())
}

#[test]
fn source_vintage_frequency_and_unpinned_status_are_explicit() {
    validate(&fixture()).expect("valid discovery-only transcription");
}

#[test]
fn all_rows_reconcile_with_published_rounding_tolerances() {
    let root = fixture();
    let mut max_sector = 0_i64;
    let mut max_gdp = 0_i64;
    for row in root["rows"].as_array().unwrap() {
        let v = &row["values"];
        let sector_sum: i64 = SECTORS.iter().map(|id| v[*id].as_i64().unwrap()).sum();
        max_sector = max_sector.max((sector_sum - v[VA].as_i64().unwrap()).abs());
        max_gdp = max_gdp.max((v[VA].as_i64().unwrap() + v[TAX].as_i64().unwrap() - v[GDP].as_i64().unwrap()).abs());
    }
    assert_eq!(max_sector, 3);
    assert_eq!(max_gdp, 1);
    validate(&root).expect("all rows within rounding bounds");
}

#[test]
fn annual_and_quarterly_levels_cannot_be_mixed_or_silently_rebased() {
    let mut root = fixture();
    let q2 = root["rows"].as_array_mut().unwrap().iter_mut().find(|r| r["period"] == "2026-Q2").unwrap();
    q2["seasonal_adjustment"] = json!("not_seasonally_adjusted_annual");
    assert!(validate(&root).is_err());
    let mut wrong_base = fixture();
    wrong_base["units"]["price_basis"] = json!("constant_2022_prices");
    assert!(validate(&wrong_base).is_err());
}

#[test]
fn unsupported_source_byte_claims_and_relaxed_tolerances_fail_closed() {
    let mut bytes_claim = fixture();
    bytes_claim["source"]["source_bytes_acquired"] = json!(true);
    assert!(validate(&bytes_claim).is_err());
    let mut relaxed = fixture();
    relaxed["reconciliation_contract"][0]["tolerance_absolute"] = json!(5000);
    assert!(validate(&relaxed).is_err());
    let mut outside = fixture();
    let row = outside["rows"].as_array_mut().unwrap().iter_mut().find(|r| r["period"] == "2026-Q2" && r["frequency"] == "quarterly").unwrap();
    let gdp = row["values"][GDP].as_i64().unwrap();
    row["values"][GDP] = json!(gdp + 100);
    assert!(validate(&outside).is_err());
    let mut duplicate = fixture();
    duplicate["rows"][1]["period"] = json!("2018");
    assert!(validate(&duplicate).is_err());
}
