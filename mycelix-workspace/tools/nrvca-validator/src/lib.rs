//! NRVCA semantic checks.
//!
//! This is intentionally an independent semantic layer; it does not claim full JSON
//! Schema Draft 2020-12 conformance, official SNA/SEEA compliance, or factual truth.

pub mod comparability;

use serde_json::Value;
use std::collections::HashSet;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Diagnostic {
    pub code: String,
    pub path: String,
    pub message: String,
}

impl Diagnostic {
    fn structural(code: &str, path: &str, message: impl Into<String>) -> Self {
        Self { code: format!("STRUCT-{code}"), path: path.to_owned(), message: message.into() }
    }

    fn semantic(code: &str, path: &str, message: impl Into<String>) -> Self {
        Self { code: format!("SEM-{code}"), path: path.to_owned(), message: message.into() }
    }
}

fn push_structural(out: &mut Vec<Diagnostic>, code: &str, path: &str, message: impl Into<String>) {
    out.push(Diagnostic::structural(code, path, message));
}

fn push_semantic(out: &mut Vec<Diagnostic>, code: &str, path: &str, message: impl Into<String>) {
    out.push(Diagnostic::semantic(code, path, message));
}

fn field_str<'a>(obj: &'a Value, key: &str) -> Option<&'a str> {
    obj.get(key).and_then(Value::as_str)
}

fn field_num(obj: &Value, key: &str) -> Option<f64> {
    obj.get(key).and_then(Value::as_f64).filter(|n| n.is_finite())
}

fn approx_eq(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance
}

fn valid_iso_date(value: &str) -> bool {
    let b = value.as_bytes();
    b.len() == 10
        && b[4] == b'-'
        && b[7] == b'-'
        && b.iter().enumerate().all(|(i, c)| i == 4 || i == 7 || c.is_ascii_digit())
}

fn reference_ids(value: &Value, path: &str, admitted: &HashSet<String>, out: &mut Vec<Diagnostic>) {
    match value.as_array() {
        Some(refs) => {
            let mut seen = HashSet::new();
            for (i, reference) in refs.iter().enumerate() {
                let Some(id) = reference.as_str() else {
                    push_structural(out, "SOURCE-REF-TYPE", &format!("{path}/{i}"), "source reference must be a string");
                    continue;
                };
                if !seen.insert(id.to_owned()) {
                    push_semantic(out, "DUPLICATE-SOURCE-REF", &format!("{path}/{i}"), format!("duplicate source reference: {id}"));
                }
                if !admitted.contains(id) {
                    push_semantic(out, "UNRESOLVED-SOURCE-REF", &format!("{path}/{i}"), format!("source reference does not resolve: {id}"));
                }
            }
        }
        None => push_structural(out, "SOURCE-REFS-TYPE", path, "source_refs must be an array"),
    }
}

/// Validate structural essentials and cross-field semantics for one NRVCA report.
/// An empty diagnostic vector means only that these checks passed.
pub fn validate_report(report: &Value) -> Vec<Diagnostic> {
    let mut out = Vec::new();
    if !report.is_object() {
        push_structural(&mut out, "ROOT-TYPE", "", "report must be a JSON object");
        return out;
    }

    if field_str(report, "schema_version") != Some("nrvca/0.1") {
        push_structural(&mut out, "SCHEMA-VERSION", "/schema_version", "expected nrvca/0.1");
    }

    let report_status = match field_str(report, "report_status") {
        Some(status) if ["synthetic", "historical_replay", "shadow", "prospective_pilot", "independent_replication"].contains(&status) => Some(status),
        Some(_) => {
            push_structural(&mut out, "REPORT-STATUS", "/report_status", "unsupported report_status");
            None
        }
        None => {
            push_structural(&mut out, "REPORT-STATUS-MISSING", "/report_status", "report_status is required");
            None
        }
    };

    let jurisdiction = report.get("jurisdiction");
    if !jurisdiction.is_some_and(Value::is_object) {
        push_structural(&mut out, "JURISDICTION", "/jurisdiction", "jurisdiction object is required");
    } else if let Some(j) = jurisdiction {
        if !matches!(field_str(j, "level"), Some("national" | "subnational")) {
            push_structural(&mut out, "JURISDICTION-LEVEL", "/jurisdiction/level", "level must be national or subnational");
        }
        if field_str(j, "name").is_none_or(str::is_empty) {
            push_structural(&mut out, "JURISDICTION-NAME", "/jurisdiction/name", "nonempty jurisdiction name is required");
        }
        let iso = field_str(j, "iso_3166_1_alpha3").unwrap_or("");
        if iso.len() != 3 || !iso.bytes().all(|b| b.is_ascii_uppercase()) {
            push_structural(&mut out, "JURISDICTION-CODE", "/jurisdiction/iso_3166_1_alpha3", "expected three uppercase ASCII letters");
        }
    }

    let period = report.get("reference_period");
    if !period.is_some_and(Value::is_object) {
        push_structural(&mut out, "PERIOD", "/reference_period", "reference_period object is required");
    } else if let Some(p) = period {
        let start = field_str(p, "start_date").unwrap_or("");
        let end = field_str(p, "end_date").unwrap_or("");
        if !valid_iso_date(start) {
            push_structural(&mut out, "PERIOD-START", "/reference_period/start_date", "start_date must use YYYY-MM-DD");
        }
        if !valid_iso_date(end) {
            push_structural(&mut out, "PERIOD-END", "/reference_period/end_date", "end_date must use YYYY-MM-DD");
        }
        if valid_iso_date(start) && valid_iso_date(end) && start > end {
            push_semantic(&mut out, "PERIOD-ORDER", "/reference_period", "start_date must not follow end_date");
        }
    }

    let basis = report.get("reporting_basis");
    if !basis.is_some_and(Value::is_object) {
        push_structural(&mut out, "REPORTING-BASIS", "/reporting_basis", "reporting_basis object is required");
    } else if let Some(b) = basis {
        let currency = field_str(b, "currency").unwrap_or("");
        if currency.len() != 3 || !currency.bytes().all(|x| x.is_ascii_uppercase()) {
            push_structural(&mut out, "CURRENCY", "/reporting_basis/currency", "currency must be a three-letter uppercase code");
        }
        if !matches!(field_str(b, "price_basis"), Some("nominal" | "constant_prices" | "chain_volume" | "not_applicable")) {
            push_structural(&mut out, "PRICE-BASIS", "/reporting_basis/price_basis", "unsupported price_basis");
        }
    }

    // Establish the authoritative source IDs before checking claims that reference them.
    let mut source_ids = HashSet::new();
    let mut source_classes = Vec::new();
    match report.get("source_manifest").and_then(Value::as_array) {
        Some(sources) if !sources.is_empty() => {
            for (i, source) in sources.iter().enumerate() {
                let path = format!("/source_manifest/{i}");
                let Some(id) = field_str(source, "source_id") else {
                    push_structural(&mut out, "SOURCE-ID", &format!("{path}/source_id"), "source_id is required");
                    continue;
                };
                if !source_ids.insert(id.to_owned()) {
                    push_semantic(&mut out, "DUPLICATE-SOURCE-ID", &format!("{path}/source_id"), format!("duplicate source_id: {id}"));
                }
                let class = field_str(source, "source_class").unwrap_or("");
                if !["official_statistics", "experimental_statistics", "administrative_record", "operational_measurement", "remote_sensing", "model_output", "synthetic_dataset"].contains(&class) {
                    push_structural(&mut out, "SOURCE-CLASS", &format!("{path}/source_class"), "unsupported source_class");
                }
                source_classes.push(class.to_owned());
                for required in ["title", "publisher", "vintage", "coverage", "license_status", "limitations"] {
                    if source.get(required).is_none() {
                        push_structural(&mut out, "SOURCE-FIELD-MISSING", &format!("{path}/{required}"), format!("required source field missing: {required}"));
                    }
                }
            }
        }
        Some(_) => push_structural(&mut out, "SOURCE-MANIFEST-EMPTY", "/source_manifest", "source_manifest must not be empty"),
        None => push_structural(&mut out, "SOURCE-MANIFEST-TYPE", "/source_manifest", "source_manifest must be an array"),
    }

    if report_status == Some("synthetic") && source_classes.iter().any(|c| c != "synthetic_dataset") {
        push_semantic(&mut out, "SYNTHETIC-SOURCE-STATUS", "/source_manifest", "a synthetic report may only cite synthetic_dataset sources");
    }
    if matches!(report_status, Some("historical_replay" | "shadow" | "prospective_pilot" | "independent_replication"))
        && !source_classes.iter().any(|c| c != "synthetic_dataset")
    {
        push_semantic(&mut out, "EVIDENCE-TIER-PROMOTION", "/report_status", "a non-synthetic evidence tier requires at least one non-synthetic source");
    }

    // Basic SNA-style monetary account coherence and source admissibility.
    let accounts = report.get("conventional_accounts");
    if !accounts.is_some_and(Value::is_object) {
        push_structural(&mut out, "ACCOUNTS", "/conventional_accounts", "conventional_accounts object is required");
    } else if let Some(a) = accounts {
        let top_currency = basis.and_then(|x| field_str(x, "currency")).unwrap_or("");
        let top_price_basis = basis.and_then(|x| field_str(x, "price_basis")).unwrap_or("");
        let top_price_year = basis.and_then(|x| x.get("price_year")).and_then(Value::as_i64);
        for metric_name in ["gdp", "consumption_of_fixed_capital", "net_domestic_product", "gross_capital_formation"] {
            let path = format!("/conventional_accounts/{metric_name}");
            let Some(metric) = a.get(metric_name) else {
                push_structural(&mut out, "ACCOUNT-MISSING", &path, "required conventional account is missing");
                continue;
            };
            if field_num(metric, "value").is_none() {
                push_structural(&mut out, "ACCOUNT-VALUE", &format!("{path}/value"), "account value must be a finite JSON number");
            }
            if field_str(metric, "currency") != Some(top_currency) {
                push_semantic(&mut out, "ACCOUNT-CURRENCY", &format!("{path}/currency"), "account currency must match reporting currency in v0.1");
            }
            if field_str(metric, "price_basis") != Some(top_price_basis) {
                push_semantic(&mut out, "ACCOUNT-PRICE-BASIS", &format!("{path}/price_basis"), "account price basis must match reporting basis in v0.1");
            }
            if metric.get("price_year").and_then(Value::as_i64) != top_price_year {
                push_semantic(&mut out, "ACCOUNT-PRICE-YEAR", &format!("{path}/price_year"), "account price year must match reporting basis in v0.1");
            }
            if field_str(metric, "unit").is_none() || field_str(metric, "definition").is_none() {
                push_structural(&mut out, "ACCOUNT-UNIT-DEFINITION", &path, "account unit and definition are required");
            }
            reference_ids(metric.get("source_refs").unwrap_or(&Value::Null), &format!("{path}/source_refs"), &source_ids, &mut out);
        }
    }

    // Events which are pure financing/revaluation/transfers cannot silently become output.
    let mut event_ids = HashSet::new();
    match report.get("financial_events").and_then(Value::as_array) {
        Some(events) => for (i, event) in events.iter().enumerate() {
            let path = format!("/financial_events/{i}");
            let Some(id) = field_str(event, "event_id") else {
                push_structural(&mut out, "EVENT-ID", &format!("{path}/event_id"), "event_id is required");
                continue;
            };
            if !event_ids.insert(id.to_owned()) {
                push_semantic(&mut out, "DUPLICATE-EVENT", &format!("{path}/event_id"), format!("duplicate event identity: {id}"));
            }
            let class = field_str(event, "event_class").unwrap_or("");
            let treatment = field_str(event, "gdp_treatment").unwrap_or("");
            if !["new_borrowing", "principal_repayment", "writeoff", "asset_revaluation", "liability_revaluation", "secondary_market_asset_transaction", "intersector_transfer", "final_goods_or_service", "other"].contains(&class) {
                push_structural(&mut out, "EVENT-CLASS", &format!("{path}/event_class"), "unsupported event_class");
            }
            if !["excluded_from_current_production", "included_under_declared_profile", "requires_profile_review"].contains(&treatment) {
                push_structural(&mut out, "EVENT-TREATMENT", &format!("{path}/gdp_treatment"), "unsupported gdp_treatment");
            }
            if ["new_borrowing", "principal_repayment", "writeoff", "asset_revaluation", "liability_revaluation", "secondary_market_asset_transaction"].contains(&class)
                && treatment != "excluded_from_current_production"
            {
                push_semantic(&mut out, "FINANCIAL-EVENT-AS-OUTPUT", &format!("{path}/gdp_treatment"), "financing, principal repayment, asset revaluation and secondary-market transfer are not current production by themselves");
            }
            if class == "intersector_transfer" && treatment == "included_under_declared_profile" {
                push_semantic(&mut out, "TRANSFER-AS-OUTPUT", &format!("{path}/gdp_treatment"), "a transfer cannot be treated as new output without an explicit, separately qualified production account");
            }
            if field_num(event, "amount").is_none() || field_num(event, "amount").is_some_and(|n| n < 0.0) {
                push_structural(&mut out, "EVENT-AMOUNT", &format!("{path}/amount"), "event amount must be a finite, nonnegative value; direction is carried by event_class/profile");
            }
            let currency = field_str(event, "currency").unwrap_or("");
            let unit = field_str(event, "unit").unwrap_or("");
            if currency.len() != 3 || !currency.bytes().all(|x| x.is_ascii_uppercase()) || unit.is_empty() {
                push_structural(&mut out, "EVENT-UNIT", &path, "event needs a currency code and nonempty unit");
            } else if unit.len() > 4 && unit.as_bytes().get(3) == Some(&b'_') && unit.as_bytes()[..3].iter().all(|x| x.is_ascii_uppercase()) && !unit.starts_with(format!("{currency}_").as_str()) {
                push_semantic(&mut out, "EVENT-CURRENCY-UNIT", &format!("{path}/unit"), "currency-prefixed unit conflicts with event currency");
            }
            reference_ids(event.get("source_refs").unwrap_or(&Value::Null), &format!("{path}/source_refs"), &source_ids, &mut out);
        },
        None => push_structural(&mut out, "EVENTS-TYPE", "/financial_events", "financial_events must be an array"),
    }

    // Canonical stock-flow values are stored once. Reconciliation operands later point
    // to these fields; financial-account changes optionally cross-reference event IDs.
    let mut stock_ids = HashSet::new();
    match report.get("stock_flow_accounts").and_then(Value::as_array) {
        Some(accounts) if !accounts.is_empty() => for (i, account) in accounts.iter().enumerate() {
            let path = format!("/stock_flow_accounts/{i}");
            let Some(id) = field_str(account, "account_id") else {
                push_structural(&mut out, "STOCK-ID", &format!("{path}/account_id"), "account_id is required");
                continue;
            };
            if !stock_ids.insert(id.to_owned()) {
                push_semantic(&mut out, "DUPLICATE-STOCK-ID", &format!("{path}/account_id"), format!("duplicate account_id: {id}"));
            }
            let class = field_str(account, "account_class").unwrap_or("");
            let unit = field_str(account, "unit").unwrap_or("");
            let currency = account.get("currency");
            if !["financial_stock", "physical_stock", "produced_asset_stock", "natural_asset_stock"].contains(&class) {
                push_structural(&mut out, "STOCK-CLASS", &format!("{path}/account_class"), "unsupported stock account class");
            }
            if unit.is_empty() {
                push_structural(&mut out, "STOCK-UNIT", &format!("{path}/unit"), "stock unit is required");
            }
            if class == "financial_stock" && !unit.starts_with(format!("{}_", currency.and_then(Value::as_str).unwrap_or("")).as_str()) {
                push_semantic(&mut out, "FINANCIAL-STOCK-UNIT", &format!("{path}/unit"), "financial stock unit must be prefixed with its declared currency in v0.1");
            }
            if class == "financial_stock" && currency.and_then(Value::as_str).is_none() {
                push_semantic(&mut out, "FINANCIAL-STOCK-CURRENCY", &format!("{path}/currency"), "financial stock requires an explicit currency");
            }
            if class == "physical_stock" && !currency.is_some_and(Value::is_null) {
                push_semantic(&mut out, "PHYSICAL-STOCK-CURRENCY", &format!("{path}/currency"), "physical stock should not claim a currency unit");
            }
            let opening = field_num(account, "opening_stock");
            let closing = field_num(account, "closing_stock");
            if opening.is_none() || closing.is_none() || opening.is_some_and(|v| v < 0.0) || closing.is_some_and(|v| v < 0.0) {
                push_structural(&mut out, "STOCK-VALUES", &path, "opening and closing stocks must be finite nonnegative values");
            }
            let changes = account.get("changes").and_then(Value::as_array);
            match changes {
                Some(changes) => {
                    let mut change_ids = HashSet::new();
                    let mut total = opening.unwrap_or(0.0);
                    for (j, change) in changes.iter().enumerate() {
                        let cpath = format!("{path}/changes/{j}");
                        let Some(change_id) = field_str(change, "change_id") else {
                            push_structural(&mut out, "CHANGE-ID", &format!("{cpath}/change_id"), "change_id is required");
                            continue;
                        };
                        if !change_ids.insert(change_id.to_owned()) {
                            push_semantic(&mut out, "DUPLICATE-CHANGE-ID", &format!("{cpath}/change_id"), format!("duplicate change_id: {change_id}"));
                        }
                        let kind = field_str(change, "change_class").unwrap_or("");
                        if !["borrowing", "principal_repayment", "writeoff", "revaluation", "liability_revaluation", "physical_inflow", "physical_use", "physical_outflow", "other_change"].contains(&kind) {
                            push_structural(&mut out, "CHANGE-CLASS", &format!("{cpath}/change_class"), "unsupported stock change class");
                        }
                        if class == "financial_stock" && kind == "revaluation" {
                            push_semantic(&mut out, "FINANCIAL-REVALUATION-CLASS", &format!("{cpath}/change_class"), "financial liability revaluation must use liability_revaluation explicitly");
                        }
                        if class != "financial_stock" && kind == "liability_revaluation" {
                            push_semantic(&mut out, "NONFINANCIAL-LIABILITY-REVALUATION", &format!("{cpath}/change_class"), "liability_revaluation is only valid for a financial stock");
                        }
                        if class == "financial_stock" && ["physical_inflow", "physical_use", "physical_outflow"].contains(&kind) {
                            push_semantic(&mut out, "FINANCIAL-PHYSICAL-CHANGE", &format!("{cpath}/change_class"), "physical resource change cannot be booked directly to a financial stock");
                        }
                        if class == "physical_stock" && ["borrowing", "principal_repayment", "writeoff"].contains(&kind) {
                            push_semantic(&mut out, "PHYSICAL-FINANCIAL-CHANGE", &format!("{cpath}/change_class"), "financial financing change cannot be booked directly to a physical stock");
                        }
                        let signed = field_num(change, "signed_change");
                        if signed.is_none() {
                            push_structural(&mut out, "SIGNED-CHANGE", &format!("{cpath}/signed_change"), "signed_change must be a finite number");
                            continue;
                        }
                        let signed = signed.unwrap();
                        if ["borrowing", "physical_inflow"].contains(&kind) && signed < 0.0 {
                            push_semantic(&mut out, "CHANGE-SIGN", &format!("{cpath}/signed_change"), "inflow changes must be nonnegative");
                        }
                        if ["principal_repayment", "writeoff", "physical_use", "physical_outflow"].contains(&kind) && signed > 0.0 {
                            push_semantic(&mut out, "CHANGE-SIGN", &format!("{cpath}/signed_change"), "outflow/decrease changes must be nonpositive");
                        }
                        total += signed;
                        if let Some(event_id) = field_str(change, "source_event_id") {
                            let found = report.get("financial_events").and_then(Value::as_array).into_iter().flatten()
                                .find(|e| field_str(e, "event_id") == Some(event_id));
                            match found {
                                Some(event) => {
                                    if !approx_eq(field_num(event, "amount").unwrap_or(f64::NAN), signed.abs(), 1e-9) {
                                        push_semantic(&mut out, "STOCK-EVENT-AMOUNT", &format!("{cpath}/source_event_id"), "linked financial event amount must match absolute signed stock change");
                                    }
                                    let expected = match kind {
                                        "borrowing" => Some("new_borrowing"),
                                        "principal_repayment" => Some("principal_repayment"),
                                        "writeoff" => Some("writeoff"),
                                        "revaluation" => Some("asset_revaluation"),
                                        "liability_revaluation" => Some("liability_revaluation"),
                                        _ => None,
                                    };
                                    if expected.is_none_or(|e| field_str(event, "event_class") != Some(e)) {
                                        push_semantic(&mut out, "STOCK-EVENT-CLASS", &format!("{cpath}/source_event_id"), "linked event class does not match stock change class");
                                    }
                                    if field_str(event, "unit") != Some(unit) {
                                        push_semantic(&mut out, "STOCK-EVENT-UNIT", &format!("{cpath}/source_event_id"), "linked event unit must match stock account unit");
                                    }
                                    if event.get("source_refs").and_then(Value::as_array).is_some_and(|r| !r.iter().filter_map(Value::as_str).any(|s| source_ids.contains(s))) {
                                        push_semantic(&mut out, "STOCK-EVENT-SOURCE", &format!("{cpath}/source_event_id"), "linked event has no source reference admitted by the manifest");
                                    }
                                }
                                None => push_semantic(&mut out, "STOCK-EVENT-MISSING", &format!("{cpath}/source_event_id"), format!("linked event not found: {event_id}")),
                            }
                        }
                        reference_ids(change.get("source_refs").unwrap_or(&Value::Null), &format!("{cpath}/source_refs"), &source_ids, &mut out);
                    }
                    if let Some(closing) = closing {
                        if !approx_eq(total, closing, 1e-6) {
                            push_semantic(&mut out, "STOCK-FLOW-DOES-NOT-CLOSE", &path, format!("opening stock plus signed changes is {total}, but closing stock is {closing}"));
                        }
                    }
                }
                None => push_structural(&mut out, "STOCK-CHANGES-TYPE", &format!("{path}/changes"), "changes must be an array"),
            }
            reference_ids(account.get("source_refs").unwrap_or(&Value::Null), &format!("{path}/source_refs"), &source_ids, &mut out);
        },
        Some(_) => push_structural(&mut out, "STOCK-ACCOUNTS-EMPTY", "/stock_flow_accounts", "stock_flow_accounts must not be empty"),
        None => push_structural(&mut out, "STOCK-ACCOUNTS-TYPE", "/stock_flow_accounts", "stock_flow_accounts must be an array"),
    }

    // Outcome coordinates remain typed and multi-dimensional; missingness is not zero.
    let mut observed_unknown_ids = HashSet::new();
    match report.get("outcome_vector").and_then(Value::as_array) {
        Some(metrics) if !metrics.is_empty() => for (i, metric) in metrics.iter().enumerate() {
            let path = format!("/outcome_vector/{i}");
            let id = field_str(metric, "indicator_id").unwrap_or("");
            if id.is_empty() {
                push_structural(&mut out, "INDICATOR-ID", &format!("{path}/indicator_id"), "indicator_id is required");
            }
            let status = field_str(metric, "evidence_status").unwrap_or("");
            if !["official_statistic", "observed", "derived", "estimated", "modelled", "assumption", "synthetic", "unknown"].contains(&status) {
                push_structural(&mut out, "EVIDENCE-STATUS", &format!("{path}/evidence_status"), "unsupported evidence_status");
            }
            if field_str(metric, "dimension").is_none() || field_str(metric, "definition").is_none() || field_str(metric, "unit").is_none() {
                push_structural(&mut out, "INDICATOR-METADATA", &path, "dimension, definition, and unit are required");
            }
            let value = metric.get("value");
            if value.is_none() {
                push_structural(&mut out, "INDICATOR-VALUE-MISSING", &format!("{path}/value"), "value field is required; use null only with evidence_status=unknown");
            }
            let is_null = value.is_none_or(Value::is_null);
            let is_numeric = value.and_then(Value::as_f64).is_some_and(f64::is_finite);
            if !is_null && !is_numeric {
                push_structural(&mut out, "INDICATOR-VALUE", &format!("{path}/value"), "indicator value must be a finite number or null");
            }
            if status == "unknown" || is_null {
                observed_unknown_ids.insert(id.to_owned());
            }
            if (status == "unknown") != is_null {
                push_semantic(&mut out, "UNKNOWN-ZERO-SEPARATION", &format!("{path}/value"), "unknown evidence must have a null value, and null values must use evidence_status=unknown");
            }
            for required in ["currency", "price_basis", "price_year", "uncertainty", "coverage"] {
                if metric.get(required).is_none() {
                    push_structural(&mut out, "INDICATOR-FIELD-MISSING", &format!("{path}/{required}"), format!("required indicator field missing: {required}"));
                }
            }
            reference_ids(metric.get("source_refs").unwrap_or(&Value::Null), &format!("{path}/source_refs"), &source_ids, &mut out);
        },
        Some(_) => push_structural(&mut out, "OUTCOME-VECTOR-EMPTY", "/outcome_vector", "outcome_vector must not be empty"),
        None => push_structural(&mut out, "OUTCOME-VECTOR-TYPE", "/outcome_vector", "outcome_vector must be an array"),
    }

    let registered_unknowns: HashSet<String> = report
        .get("data_quality")
        .and_then(|x| x.get("unknown_indicator_ids"))
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect();
    if report.get("data_quality").and_then(Value::as_object).is_none() {
        push_structural(&mut out, "DATA-QUALITY", "/data_quality", "data_quality object is required");
    }
    for id in &observed_unknown_ids {
        if !registered_unknowns.contains(id) {
            push_semantic(&mut out, "UNKNOWN-REGISTER", "/data_quality/unknown_indicator_ids", format!("unknown indicator missing from register: {id}"));
        }
    }
    for id in &registered_unknowns {
        if !observed_unknown_ids.contains(id) {
            push_semantic(&mut out, "UNKNOWN-REGISTER-STALE", "/data_quality/unknown_indicator_ids", format!("unknown register contains no unknown-valued indicator: {id}"));
        }
    }

    // Independently recompute every signed identity from named operands.
    match report.get("reconciliations").and_then(Value::as_array) {
        Some(checks) if !checks.is_empty() => for (i, check) in checks.iter().enumerate() {
            let path = format!("/reconciliations/{i}");
            let lhs = field_num(check, "lhs");
            let rhs = field_num(check, "rhs");
            let residual = field_num(check, "residual");
            let tolerance = field_num(check, "tolerance");
            let unit = field_str(check, "unit").unwrap_or("");
            let lhs_pointer = field_str(check, "lhs_pointer").unwrap_or("");
            if lhs_pointer.is_empty() {
                push_structural(&mut out, "RECONCILIATION-LHS-POINTER", &format!("{path}/lhs_pointer"), "lhs_pointer must bind the claimed LHS to a canonical report value");
            } else {
                match report.pointer(lhs_pointer).and_then(Value::as_f64) {
                    Some(bound_lhs) if lhs.is_some_and(|v| approx_eq(v, bound_lhs, tolerance.unwrap_or(0.0))) => {}
                    _ => push_semantic(&mut out, "RECONCILIATION-LHS-BINDING", &format!("{path}/lhs_pointer"), "lhs must equal the canonical value referenced by lhs_pointer"),
                }
            }
            if lhs.is_none() || rhs.is_none() || residual.is_none() || tolerance.is_none() || tolerance.is_some_and(|t| t < 0.0) || unit.is_empty() {
                push_structural(&mut out, "RECONCILIATION-METADATA", &path, "lhs, rhs, residual, nonnegative tolerance and unit are required");
                continue;
            }
            let (lhs, rhs, residual, tolerance) = (lhs.unwrap(), rhs.unwrap(), residual.unwrap(), tolerance.unwrap());
            let mut recomputed = 0.0;
            let mut operands_valid = true;
            match check.get("operands").and_then(Value::as_array) {
                Some(operands) if operands.len() >= 2 => for (j, operand) in operands.iter().enumerate() {
                    let opath = format!("{path}/operands/{j}");
                    match (field_num(operand, "value"), field_num(operand, "coefficient"), field_str(operand, "unit")) {
                        (Some(value), Some(coefficient), Some(operand_unit)) => {
                            let value_pointer = field_str(operand, "value_pointer").unwrap_or("");
                            if value_pointer.is_empty() {
                                operands_valid = false;
                                push_structural(&mut out, "RECONCILIATION-VALUE-POINTER", &format!("{opath}/value_pointer"), "each operand must bind to a canonical report value");
                            } else {
                                match report.pointer(value_pointer).and_then(Value::as_f64) {
                                    Some(bound_value) if approx_eq(value, bound_value, tolerance) => {}
                                    _ => {
                                        operands_valid = false;
                                        push_semantic(&mut out, "RECONCILIATION-VALUE-BINDING", &format!("{opath}/value_pointer"), "operand value differs from the canonical field referenced by value_pointer");
                                    }
                                }
                            }
                            if operand_unit != unit {
                                operands_valid = false;
                                push_semantic(&mut out, "RECONCILIATION-UNIT", &format!("{opath}/unit"), "all operands must use the reconciliation's declared compatible unit");
                            }
                            if field_str(operand, "term").is_none() {
                                operands_valid = false;
                                push_structural(&mut out, "RECONCILIATION-TERM", &format!("{opath}/term"), "operand term name is required");
                            }
                            recomputed += value * coefficient;
                        }
                        _ => {
                            operands_valid = false;
                            push_structural(&mut out, "RECONCILIATION-OPERAND", &opath, "each operand needs numeric value/coefficient and a unit");
                        }
                    }
                },
                _ => {
                    operands_valid = false;
                    push_structural(&mut out, "RECONCILIATION-OPERANDS", &format!("{path}/operands"), "at least two machine-readable operands are required");
                }
            }
            if !approx_eq(recomputed, rhs, tolerance) {
                push_semantic(&mut out, "RECONCILIATION-RHS", &format!("{path}/rhs"), format!("recomputed RHS {recomputed} differs from declared rhs {rhs}"));
            }
            if !approx_eq(lhs - rhs, residual, tolerance) {
                push_semantic(&mut out, "RECONCILIATION-RESIDUAL", &format!("{path}/residual"), "residual does not equal lhs minus rhs");
            }
            if field_str(check, "disposition") == Some("pass")
                && (!approx_eq(lhs, rhs, tolerance) || !approx_eq(residual, 0.0, tolerance) || !operands_valid)
            {
                push_semantic(&mut out, "FALSE-PASS", &format!("{path}/disposition"), "a reconciliation marked pass must close within its declared tolerance");
            }
            if field_str(check, "disposition").is_none() {
                push_structural(&mut out, "RECONCILIATION-DISPOSITION", &format!("{path}/disposition"), "disposition is required");
            }
            if field_str(check, "class") == Some("official_account_bridge")
                && field_str(check, "equation").is_some_and(|e| e.to_ascii_lowercase().contains("ndp"))
            {
                let expected = [
                    ("GDP", "/conventional_accounts/gdp/value", 1.0),
                    ("consumption_of_fixed_capital", "/conventional_accounts/consumption_of_fixed_capital/value", -1.0),
                ];
                let operands = check.get("operands").and_then(Value::as_array);
                if let Some(operands) = operands {
                    let matches = operands.len() == expected.len() && expected.iter().all(|(term, pointer, coefficient)| {
                        operands.iter().any(|o| field_str(o, "term") == Some(*term)
                            && field_str(o, "value_pointer") == Some(*pointer)
                            && field_num(o, "coefficient").is_some_and(|c| approx_eq(c, *coefficient, 1e-12)))
                    });
                    if !matches {
                        push_semantic(&mut out, "NDP-BRIDGE-TERMS", &format!("{path}/operands"), "NDP must be derived specifically from GDP minus consumption of fixed capital");
                    }
                }
                if let Some(ndp) = report.pointer("/conventional_accounts/net_domestic_product/value").and_then(Value::as_f64) {
                    if !approx_eq(lhs, ndp, tolerance) || !approx_eq(rhs, ndp, tolerance) {
                        push_semantic(&mut out, "ACCOUNT-BRIDGE-LHS", &format!("{path}/lhs"), "NDP bridge result must match reported net_domestic_product");
                    }
                }
            }
        },
        Some(_) => push_structural(&mut out, "RECONCILIATIONS-EMPTY", "/reconciliations", "at least one reconciliation is required"),
        None => push_structural(&mut out, "RECONCILIATIONS-TYPE", "/reconciliations", "reconciliations must be an array"),
    }

    // The base profile deliberately refuses an unqualified universal welfare scalar.
    let agg = report.get("aggregation_policy");
    if agg.and_then(|x| x.get("universal_scalar_produced")).and_then(Value::as_bool) != Some(false) {
        push_structural(&mut out, "UNIVERSAL-SCALAR", "/aggregation_policy/universal_scalar_produced", "v0.1 must set universal_scalar_produced=false");
    }
    if agg.and_then(|x| x.get("cross_unit_addition_permitted")).and_then(Value::as_bool) != Some(false) {
        push_structural(&mut out, "CROSS-UNIT-ADDITION", "/aggregation_policy/cross_unit_addition_permitted", "v0.1 must set cross_unit_addition_permitted=false");
    }

    if !report.get("claim_ceiling").is_some_and(Value::is_array) {
        push_structural(&mut out, "CLAIM-CEILING", "/claim_ceiling", "claim_ceiling array is required");
    }

    // Machine-checkable claims carry explicit type/status/evidence references. Do not
    // infer epistemic status by parsing disclaimer prose: negation makes that brittle.
    let mut claim_ids = HashSet::new();
    match report.get("claims").and_then(Value::as_array) {
        Some(claims) if !claims.is_empty() => for (i, claim) in claims.iter().enumerate() {
            let path = format!("/claims/{i}");
            let Some(id) = field_str(claim, "claim_id") else {
                push_structural(&mut out, "CLAIM-ID", &format!("{path}/claim_id"), "claim_id is required");
                continue;
            };
            if !claim_ids.insert(id.to_owned()) {
                push_semantic(&mut out, "DUPLICATE-CLAIM-ID", &format!("{path}/claim_id"), format!("duplicate claim_id: {id}"));
            }
            let claim_type = field_str(claim, "claim_type").unwrap_or("");
            let evidence_status = field_str(claim, "evidence_status").unwrap_or("");
            if !["accounting_reconciliation", "official_measurement", "observed_measurement", "derived_indicator", "forecast", "modeled_scenario", "causal_effect", "policy_assessment"].contains(&claim_type) {
                push_structural(&mut out, "CLAIM-TYPE", &format!("{path}/claim_type"), "unsupported claim_type");
            }
            if !["synthetic", "official_statistic", "observed", "derived", "estimated", "modelled", "causally_evaluated", "independently_replicated", "unknown"].contains(&evidence_status) {
                push_structural(&mut out, "CLAIM-EVIDENCE-STATUS", &format!("{path}/evidence_status"), "unsupported claim evidence_status");
            }
            if field_str(claim, "statement").is_none_or(str::is_empty) || field_str(claim, "scope").is_none_or(str::is_empty) {
                push_structural(&mut out, "CLAIM-SCOPE-STATEMENT", &path, "nonempty statement and scope are required");
            }
            if report_status == Some("synthetic") && !["synthetic", "unknown"].contains(&evidence_status) {
                push_semantic(&mut out, "SYNTHETIC-CLAIM-PROMOTION", &format!("{path}/evidence_status"), "synthetic report cannot promote a claim to real observation, estimate, or causal evaluation");
            }
            match claim_type {
                "official_measurement" if evidence_status != "official_statistic" =>
                    push_semantic(&mut out, "CLAIM-TYPE-STATUS", &format!("{path}/evidence_status"), "official_measurement requires official_statistic evidence status"),
                "observed_measurement" if !["observed", "official_statistic", "independently_replicated"].contains(&evidence_status) =>
                    push_semantic(&mut out, "CLAIM-TYPE-STATUS", &format!("{path}/evidence_status"), "observed_measurement requires observation or official-statistic evidence"),
                "causal_effect" if !["causally_evaluated", "independently_replicated"].contains(&evidence_status) =>
                    push_semantic(&mut out, "CAUSAL-CLAIM-UNQUALIFIED", &format!("{path}/evidence_status"), "causal_effect requires a qualified causal evaluation or independent replication"),
                "forecast" if !["estimated", "modelled", "independently_replicated"].contains(&evidence_status) =>
                    push_semantic(&mut out, "CLAIM-TYPE-STATUS", &format!("{path}/evidence_status"), "forecast requires estimated, modelled, or independently replicated status"),
                _ => {}
            }
            reference_ids(claim.get("source_refs").unwrap_or(&Value::Null), &format!("{path}/source_refs"), &source_ids, &mut out);
        },
        Some(_) => push_structural(&mut out, "CLAIMS-EMPTY", "/claims", "at least one typed claim is required"),
        None => push_structural(&mut out, "CLAIMS-TYPE", "/claims", "claims must be an array"),
    }

    out
}

/// Check the subset of the v0.1 contract that this validator relies on.
/// This is deliberately not advertised as a full JSON Schema implementation.
pub fn validate_schema_contract(schema: &Value) -> Vec<Diagnostic> {
    let mut out = Vec::new();
    if !schema.is_object() {
        push_structural(&mut out, "SCHEMA-ROOT", "", "schema document must be an object");
        return out;
    }
    if schema.pointer("/properties/schema_version/const").and_then(Value::as_str) != Some("nrvca/0.1") {
        push_structural(&mut out, "SCHEMA-VERSION", "/properties/schema_version/const", "schema must pin nrvca/0.1");
    }
    for (name, required) in [
        ("outcome_vector", vec!["currency", "price_basis", "price_year", "uncertainty", "source_refs"]),
        ("reconciliations", vec!["operands", "lhs_pointer"]),
        ("claims", vec!["claim_id", "claim_type", "evidence_status", "statement", "scope", "source_refs"]),
    ] {
        let path = format!("/properties/{name}/items/required");
        let found: HashSet<&str> = schema.pointer(&path).and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str).collect();
        for field in required {
            if !found.contains(field) {
                push_structural(&mut out, "SCHEMA-REQUIRED-FIELD", &path, format!("{name} must require {field}"));
            }
        }
    }

    let operand_required: HashSet<&str> = schema.pointer("/properties/reconciliations/items/properties/operands/items/required")
        .and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str).collect();
    if !operand_required.contains("value_pointer") {
        push_structural(&mut out, "SCHEMA-REQUIRED-FIELD", "/properties/reconciliations/items/properties/operands/items/required", "reconciliation operands must require value_pointer");
    }
    let stock_required: HashSet<&str> = schema.pointer("/required").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_str).collect();
    if !stock_required.contains("stock_flow_accounts") {
        push_structural(&mut out, "SCHEMA-REQUIRED-FIELD", "/required", "report must require stock_flow_accounts");
    }
    out
}


/// Check required fields in the benchmark preregistration schema.
/// This remains a narrow schema-contract check, not a full JSON Schema engine.
pub fn validate_benchmark_schema_contract(schema: &Value) -> Vec<Diagnostic> {
    let mut out = Vec::new();
    if !schema.is_object() {
        push_structural(&mut out, "BENCH-SCHEMA-ROOT", "", "benchmark schema must be a JSON object");
        return out;
    }
    if schema.pointer("/properties/schema_version/const").and_then(Value::as_str) != Some("nrvca-benchmark/0.1") {
        push_structural(&mut out, "BENCH-SCHEMA-VERSION", "/properties/schema_version/const", "schema must pin nrvca-benchmark/0.1");
    }
    let top_required: HashSet<&str> = schema.pointer("/required").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_str).collect();
    for field in ["source_manifest", "source_ids", "execution_state", "arms", "metrics", "failure_policy"] {
        if !top_required.contains(field) {
            push_structural(&mut out, "BENCH-SCHEMA-REQUIRED", "/required", format!("benchmark schema must require {field}"));
        }
    }
    let source_required: HashSet<&str> = schema.pointer("/properties/source_manifest/items/required").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_str).collect();
    for field in ["source_id", "content_sha256", "license_status", "ingest_state", "digest_scope"] {
        if !source_required.contains(field) {
            push_structural(&mut out, "BENCH-SCHEMA-SOURCE-FIELD", "/properties/source_manifest/items/required", format!("source manifest must require {field}"));
        }
    }
    let failure_required: HashSet<&str> = schema.pointer("/properties/failure_policy/required").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_str).collect();
    for field in ["retain_failed_runs", "retain_unfavourable_results", "excluded_case_ledger", "no_posthoc_threshold_changes"] {
        if !failure_required.contains(field) {
            push_structural(&mut out, "BENCH-SCHEMA-FAILURE-POLICY", "/properties/failure_policy/required", format!("failure policy must require {field}"));
        }
    }
    out
}

/// Validate a preregistration's critical cross-field semantics and fail-closed execution gate.
/// A PASS means the preregistration is internally coherent; it does not mean a benchmark ran.
pub fn validate_benchmark_preregistration(record: &Value) -> Vec<Diagnostic> {
    let mut out = Vec::new();
    if !record.is_object() {
        push_structural(&mut out, "BENCH-ROOT", "", "benchmark preregistration must be a JSON object");
        return out;
    }
    if field_str(record, "schema_version") != Some("nrvca-benchmark/0.1") {
        push_structural(&mut out, "BENCH-VERSION", "/schema_version", "expected nrvca-benchmark/0.1");
    }
    let status = field_str(record, "status").unwrap_or("");
    if !["design_only", "preregistered", "in_progress", "completed", "invalidated"].contains(&status) {
        push_structural(&mut out, "BENCH-STATUS", "/status", "unsupported benchmark status");
    }
    let execution = field_str(record, "execution_state").unwrap_or("");
    if !["blocked_missing_sources", "blocked_rights_review", "ready_for_execution", "executed", "invalidated"].contains(&execution) {
        push_structural(&mut out, "BENCH-EXECUTION-STATE", "/execution_state", "unsupported execution_state");
    }
    if status == "design_only" && !["blocked_missing_sources", "blocked_rights_review"].contains(&execution) {
        push_semantic(&mut out, "BENCH-DESIGN-EXECUTION", "/execution_state", "design_only preregistration must remain blocked until evidence and rights are qualified");
    }

    let policy = record.get("source_policy");
    if policy.and_then(|x| x.get("content_digest_required_before_execution")).and_then(Value::as_bool) != Some(true) {
        push_semantic(&mut out, "BENCH-DIGEST-POLICY", "/source_policy/content_digest_required_before_execution", "source bytes must be content-addressed before execution");
    }
    if policy.and_then(|x| x.get("rights_review_required")).and_then(Value::as_bool) != Some(true) {
        push_semantic(&mut out, "BENCH-RIGHTS-POLICY", "/source_policy/rights_review_required", "source reuse rights must be reviewed before execution");
    }
    if policy.and_then(|x| x.get("time_alignment_policy")).and_then(Value::as_str).is_none() {
        push_structural(&mut out, "BENCH-TIME-POLICY", "/source_policy/time_alignment_policy", "time alignment policy is required");
    }

    let mut declared_ids = HashSet::new();
    match record.get("source_ids").and_then(Value::as_array) {
        Some(ids) if !ids.is_empty() => for (i, id) in ids.iter().enumerate() {
            match id.as_str() {
                Some(id) => {
                    if !declared_ids.insert(id.to_owned()) {
                        push_semantic(&mut out, "BENCH-DUPLICATE-SOURCE-ID", &format!("/source_ids/{i}"), format!("duplicate declared source ID: {id}"));
                    }
                }
                None => push_structural(&mut out, "BENCH-SOURCE-ID", &format!("/source_ids/{i}"), "source ID must be a string"),
            }
        },
        Some(_) => push_structural(&mut out, "BENCH-SOURCE-IDS-EMPTY", "/source_ids", "source_ids must not be empty"),
        None => push_structural(&mut out, "BENCH-SOURCE-IDS-TYPE", "/source_ids", "source_ids must be an array"),
    }

    let mut manifest_ids = HashSet::new();
    let mut digest_missing = false;
    let mut rights_pending = false;
    let mut source_uningested = false;
    match record.get("source_manifest").and_then(Value::as_array) {
        Some(entries) if !entries.is_empty() => for (i, entry) in entries.iter().enumerate() {
            let path = format!("/source_manifest/{i}");
            let Some(id) = field_str(entry, "source_id") else {
                push_structural(&mut out, "BENCH-MANIFEST-ID", &format!("{path}/source_id"), "manifest source_id is required");
                continue;
            };
            if !manifest_ids.insert(id.to_owned()) {
                push_semantic(&mut out, "BENCH-DUPLICATE-MANIFEST-ID", &format!("{path}/source_id"), format!("duplicate manifest source ID: {id}"));
            }
            if !declared_ids.contains(id) {
                push_semantic(&mut out, "BENCH-MANIFEST-UNDECLARED", &format!("{path}/source_id"), format!("manifest source is not in source_ids: {id}"));
            }
            for required in ["publisher", "dataset_id", "source_url", "source_status", "data_vintage", "reference_period", "digest_scope", "license_status", "ingest_state"] {
                if entry.get(required).is_none() {
                    push_structural(&mut out, "BENCH-MANIFEST-FIELD", &format!("{path}/{required}"), format!("required source manifest field missing: {required}"));
                }
            }
            let digest = entry.get("content_sha256");
            match digest {
                Some(Value::String(hash)) if hash.len() == 64 && hash.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase()) => {}
                Some(Value::Null) | None => digest_missing = true,
                _ => {
                    digest_missing = true;
                    push_semantic(&mut out, "BENCH-SOURCE-DIGEST-FORMAT", &format!("{path}/content_sha256"), "source digest must be lowercase SHA-256 hex or null while unpinned");
                }
            }
            if !["reviewed_permitted", "not_applicable_synthetic"].contains(&field_str(entry, "license_status").unwrap_or("")) {
                rights_pending = true;
            }
            if !["ingested", "independently_verified"].contains(&field_str(entry, "ingest_state").unwrap_or("")) {
                source_uningested = true;
            }
        },
        Some(_) => push_structural(&mut out, "BENCH-MANIFEST-EMPTY", "/source_manifest", "source_manifest must not be empty"),
        None => push_structural(&mut out, "BENCH-MANIFEST-TYPE", "/source_manifest", "source_manifest must be an array"),
    }
    for id in &declared_ids {
        if !manifest_ids.contains(id) {
            push_semantic(&mut out, "BENCH-MANIFEST-MISSING-ID", "/source_manifest", format!("declared source is missing from manifest: {id}"));
        }
    }

    if matches!(execution, "ready_for_execution" | "executed") {
        if digest_missing {
            push_semantic(&mut out, "BENCH-EXECUTION-MISSING-DIGEST", "/execution_state", "execution requires exact source content digests");
        }
        if rights_pending {
            push_semantic(&mut out, "BENCH-EXECUTION-RIGHTS", "/execution_state", "execution requires reviewed source reuse rights");
        }
        if source_uningested {
            push_semantic(&mut out, "BENCH-EXECUTION-INGEST", "/execution_state", "execution requires pinned source bytes to be ingested");
        }
    } else if digest_missing && execution != "blocked_missing_sources" {
        push_semantic(&mut out, "BENCH-UNBLOCKED-MISSING-DIGEST", "/execution_state", "missing source digests must keep execution blocked_missing_sources");
    } else if !digest_missing && rights_pending && execution != "blocked_rights_review" {
        push_semantic(&mut out, "BENCH-UNBLOCKED-RIGHTS", "/execution_state", "unreviewed source rights must keep execution blocked_rights_review");
    }

    let arms = record.get("arms").and_then(Value::as_array);
    let mut arm_ids = HashSet::new();
    let mut has_candidate = false;
    let mut has_baseline = false;
    match arms {
        Some(arms) if arms.len() >= 2 => for (i, arm) in arms.iter().enumerate() {
            let path = format!("/arms/{i}");
            let Some(id) = field_str(arm, "arm_id") else {
                push_structural(&mut out, "BENCH-ARM-ID", &format!("{path}/arm_id"), "arm_id is required");
                continue;
            };
            if !arm_ids.insert(id.to_owned()) {
                push_semantic(&mut out, "BENCH-DUPLICATE-ARM", &format!("{path}/arm_id"), format!("duplicate arm ID: {id}"));
            }
            let role = field_str(arm, "role").unwrap_or("");
            has_candidate |= role == "candidate";
            has_baseline |= ["incumbent", "simple_baseline", "strong_baseline"].contains(&role);
            for required in ["method", "description", "resource_budget_policy"] {
                if field_str(arm, required).is_none_or(str::is_empty) {
                    push_structural(&mut out, "BENCH-ARM-FIELD", &format!("{path}/{required}"), format!("nonempty arm field required: {required}"));
                }
            }
        },
        Some(_) => push_structural(&mut out, "BENCH-ARMS-COUNT", "/arms", "at least two benchmark arms are required"),
        None => push_structural(&mut out, "BENCH-ARMS-TYPE", "/arms", "arms must be an array"),
    }
    if !has_candidate {
        push_semantic(&mut out, "BENCH-NO-CANDIDATE", "/arms", "at least one candidate arm is required");
    }
    if !has_baseline {
        push_semantic(&mut out, "BENCH-NO-BASELINE", "/arms", "at least one incumbent or baseline arm is required");
    }

    let metrics = record.get("metrics").and_then(Value::as_array);
    let mut metric_ids = HashSet::new();
    let mut has_primary = false;
    match metrics {
        Some(metrics) if !metrics.is_empty() => for (i, metric) in metrics.iter().enumerate() {
            let path = format!("/metrics/{i}");
            let Some(id) = field_str(metric, "metric_id") else {
                push_structural(&mut out, "BENCH-METRIC-ID", &format!("{path}/metric_id"), "metric_id is required");
                continue;
            };
            if !metric_ids.insert(id.to_owned()) {
                push_semantic(&mut out, "BENCH-DUPLICATE-METRIC", &format!("{path}/metric_id"), format!("duplicate metric ID: {id}"));
            }
            has_primary |= metric.get("primary").and_then(Value::as_bool) == Some(true);
            if field_str(metric, "definition").is_none_or(str::is_empty) || field_str(metric, "unit").is_none_or(str::is_empty) {
                push_structural(&mut out, "BENCH-METRIC-META", &path, "metric definition and unit are required");
            }
            if field_str(metric, "direction").is_none() || field_str(metric, "acceptance_rule").is_none_or(str::is_empty) {
                push_structural(&mut out, "BENCH-METRIC-RULE", &path, "metric direction and acceptance rule are required");
            }
        },
        Some(_) => push_structural(&mut out, "BENCH-METRICS-EMPTY", "/metrics", "at least one metric is required"),
        None => push_structural(&mut out, "BENCH-METRICS-TYPE", "/metrics", "metrics must be an array"),
    }
    if !has_primary {
        push_semantic(&mut out, "BENCH-NO-PRIMARY-METRIC", "/metrics", "at least one primary metric must be preregistered");
    }

    if record.pointer("/comparison_protocol/preregister_before_results").and_then(Value::as_bool) != Some(true) {
        push_semantic(&mut out, "BENCH-NOT-PREREGISTERED", "/comparison_protocol/preregister_before_results", "protocol must be frozen before results are inspected");
    }
    let failures = record.get("failure_policy");
    for field in ["retain_failed_runs", "retain_unfavourable_results", "excluded_case_ledger", "no_posthoc_threshold_changes"] {
        if failures.and_then(|f| f.get(field)).and_then(Value::as_bool) != Some(true) {
            push_semantic(&mut out, "BENCH-FAILURE-POLICY", &format!("/failure_policy/{field}"), format!("{field} must be true"));
        }
    }
    out
}

fn decode_pointer(path: &str) -> Result<Vec<String>, String> {
    if path.is_empty() {
        return Ok(Vec::new());
    }
    if !path.starts_with('/') {
        return Err(format!("JSON pointer must be empty or start with '/': {path}"));
    }
    path.split('/').skip(1).map(|part| {
        let mut decoded = String::new();
        let mut chars = part.chars();
        while let Some(ch) = chars.next() {
            if ch == '~' {
                match chars.next() {
                    Some('0') => decoded.push('~'),
                    Some('1') => decoded.push('/'),
                    _ => return Err(format!("invalid JSON pointer escape in {path}")),
                }
            } else {
                decoded.push(ch);
            }
        }
        Ok(decoded)
    }).collect()
}

fn get_pointer<'a>(root: &'a Value, path: &str) -> Result<&'a Value, String> {
    let mut current = root;
    for segment in decode_pointer(path)? {
        current = match current {
            Value::Object(map) => map.get(&segment).ok_or_else(|| format!("missing object path {path}"))?,
            Value::Array(array) => {
                let index: usize = segment.parse().map_err(|_| format!("invalid array index in {path}"))?;
                array.get(index).ok_or_else(|| format!("array index out of range in {path}"))?
            }
            _ => return Err(format!("path traverses scalar in {path}")),
        };
    }
    Ok(current)
}

fn parent_mut<'a>(root: &'a mut Value, segments: &[String], full_path: &str) -> Result<&'a mut Value, String> {
    if segments.is_empty() {
        return Ok(root);
    }
    match root {
        Value::Object(map) => {
            let child = map.get_mut(&segments[0]).ok_or_else(|| format!("missing object path {full_path}"))?;
            parent_mut(child, &segments[1..], full_path)
        }
        Value::Array(array) => {
            let index: usize = segments[0].parse().map_err(|_| format!("invalid array index in {full_path}"))?;
            let child = array.get_mut(index).ok_or_else(|| format!("array index out of range in {full_path}"))?;
            parent_mut(child, &segments[1..], full_path)
        }
        _ => Err(format!("path traverses scalar in {full_path}")),
    }
}

fn add_pointer(root: &mut Value, path: &str, value: Value) -> Result<(), String> {
    let segments = decode_pointer(path)?;
    if segments.is_empty() {
        *root = value;
        return Ok(());
    }
    let (last, parents) = segments.split_last().expect("nonempty segments");
    let parent = parent_mut(root, parents, path)?;
    match parent {
        Value::Object(map) => { map.insert(last.clone(), value); Ok(()) }
        Value::Array(array) if last == "-" => { array.push(value); Ok(()) }
        Value::Array(array) => {
            let index: usize = last.parse().map_err(|_| format!("invalid array index in {path}"))?;
            if index > array.len() { return Err(format!("array insert index out of range in {path}")); }
            array.insert(index, value);
            Ok(())
        }
        _ => Err(format!("cannot add at non-container path {path}")),
    }
}

fn replace_pointer(root: &mut Value, path: &str, value: Value) -> Result<(), String> {
    let segments = decode_pointer(path)?;
    if segments.is_empty() {
        *root = value;
        return Ok(());
    }
    let (last, parents) = segments.split_last().expect("nonempty segments");
    let parent = parent_mut(root, parents, path)?;
    match parent {
        Value::Object(map) => {
            if !map.contains_key(last) { return Err(format!("replace path does not exist: {path}")); }
            map.insert(last.clone(), value);
            Ok(())
        }
        Value::Array(array) => {
            let index: usize = last.parse().map_err(|_| format!("invalid array index in {path}"))?;
            let target = array.get_mut(index).ok_or_else(|| format!("replace index out of range in {path}"))?;
            *target = value;
            Ok(())
        }
        _ => Err(format!("cannot replace at non-container path {path}")),
    }
}

fn remove_pointer(root: &mut Value, path: &str) -> Result<(), String> {
    let segments = decode_pointer(path)?;
    if segments.is_empty() { return Err("cannot remove document root".into()); }
    let (last, parents) = segments.split_last().expect("nonempty segments");
    let parent = parent_mut(root, parents, path)?;
    match parent {
        Value::Object(map) => {
            map.remove(last).ok_or_else(|| format!("remove path does not exist: {path}"))?;
            Ok(())
        }
        Value::Array(array) => {
            let index: usize = last.parse().map_err(|_| format!("invalid array index in {path}"))?;
            if index >= array.len() { return Err(format!("remove index out of range in {path}")); }
            array.remove(index);
            Ok(())
        }
        _ => Err(format!("cannot remove at non-container path {path}")),
    }
}

/// Apply the restricted JSON Patch operations used by the checked-in corpus.
pub fn apply_patch(root: &mut Value, patch: &Value) -> Result<(), String> {
    let operations = patch.as_array().ok_or_else(|| "patch must be an array".to_owned())?;
    for operation in operations {
        let op = field_str(operation, "op").ok_or_else(|| "patch operation missing op".to_owned())?;
        let path = field_str(operation, "path").ok_or_else(|| "patch operation missing path".to_owned())?;
        match op {
            "replace" => replace_pointer(root, path, operation.get("value").cloned().ok_or_else(|| "replace missing value".to_owned())?)?,
            "add" => add_pointer(root, path, operation.get("value").cloned().ok_or_else(|| "add missing value".to_owned())?)?,
            "remove" => remove_pointer(root, path)?,
            "copy" => {
                let from = field_str(operation, "from").ok_or_else(|| "copy missing from".to_owned())?;
                let value = get_pointer(root, from)?.clone();
                add_pointer(root, path, value)?;
            }
            other => return Err(format!("unsupported patch operation: {other}")),
        }
    }
    Ok(())
}

#[derive(Debug)]
pub struct CorpusResult {
    pub case_id: String,
    pub expected_disposition: String,
    pub diagnostic_codes: Vec<String>,
    pub passed: bool,
    pub detail: String,
}

/// Run every declared adversarial mutation. The corpus is only qualified if each case
/// is rejected in its declared diagnostic class; merely having cases listed is not a pass.
pub fn run_adversarial_corpus(base: &Value, corpus: &Value) -> Result<Vec<CorpusResult>, String> {
    let cases = corpus.get("cases").and_then(Value::as_array).ok_or_else(|| "corpus.cases must be an array".to_owned())?;
    let baseline_errors = validate_report(base);
    if !baseline_errors.is_empty() {
        return Err(format!("golden baseline is invalid: {:?}", baseline_errors));
    }
    let mut results = Vec::new();
    for case in cases {
        let id = field_str(case, "case_id").ok_or_else(|| "case is missing case_id".to_owned())?.to_owned();
        let expected = field_str(case, "expected_disposition").ok_or_else(|| format!("{id}: missing expected_disposition"))?.to_owned();
        let patch = case.get("patch").ok_or_else(|| format!("{id}: missing patch"))?;
        let mut mutated = base.clone();
        apply_patch(&mut mutated, patch).map_err(|e| format!("{id}: mutation could not be applied: {e}"))?;
        let diagnostics = validate_report(&mutated);
        let codes: Vec<String> = diagnostics.iter().map(|d| d.code.clone()).collect();
        let correct_class = match expected.as_str() {
            "reject_structural" => codes.iter().any(|c| c.starts_with("STRUCT-")),
            "reject_semantic" => codes.iter().any(|c| c.starts_with("SEM-")),
            other => return Err(format!("{id}: unsupported expected disposition {other}")),
        };
        let passed = !diagnostics.is_empty() && correct_class;
        let detail = if diagnostics.is_empty() {
            "MUTATION SURVIVED: validator accepted adversarial input".to_owned()
        } else if !correct_class {
            format!("rejected, but not in expected class {expected}")
        } else {
            format!("rejected with {} diagnostic(s)", diagnostics.len())
        };
        results.push(CorpusResult { case_id: id, expected_disposition: expected, diagnostic_codes: codes, passed, detail });
    }
    Ok(results)
}


#[cfg(test)]
mod tests {
    use super::*;

    const GOLDEN: &str = include_str!("../../../../fixtures/nrvca-synthetic-golden-v0.1.json");
    const CORPUS: &str = include_str!("../../../../fixtures/nrvca-adversarial-corpus-v0.1.json");
    const SCHEMA: &str = include_str!("../../../../schemas/nrvca-result-v0.1.schema.json");
    const BENCHMARK: &str = include_str!("../../../../fixtures/nrvca-sa-bridge-preregistration-v0.1.json");
    const BENCH_SCHEMA: &str = include_str!("../../../../schemas/nrvca-benchmark-preregistration-v0.1.schema.json");

    #[test]
    fn schema_contract_has_required_fields() {
        let schema: Value = serde_json::from_str(SCHEMA).expect("schema parses as JSON");
        let diagnostics = validate_schema_contract(&schema);
        assert!(diagnostics.is_empty(), "schema contract diagnostics: {diagnostics:?}");
    }

    #[test]
    fn synthetic_golden_fixture_is_semantically_valid() {
        let report: Value = serde_json::from_str(GOLDEN).expect("golden fixture parses as JSON");
        let diagnostics = validate_report(&report);
        assert!(diagnostics.is_empty(), "golden fixture diagnostics: {diagnostics:?}");
    }

    #[test]
    fn all_adversarial_mutations_are_rejected_in_the_expected_class() {
        let report: Value = serde_json::from_str(GOLDEN).expect("golden fixture parses as JSON");
        let corpus: Value = serde_json::from_str(CORPUS).expect("adversarial corpus parses as JSON");
        let results = run_adversarial_corpus(&report, &corpus).expect("corpus runner executes");
        assert_eq!(results.len(), 20, "unexpected corpus size");
        for result in &results {
            assert!(result.passed, "{}: {}", result.case_id, result.detail);
        }
    }

    #[test]
    fn benchmark_schema_contract_requires_source_digest_gates() {
        let schema: Value = serde_json::from_str(BENCH_SCHEMA).expect("benchmark schema parses");
        let diagnostics = validate_benchmark_schema_contract(&schema);
        assert!(diagnostics.is_empty(), "benchmark schema diagnostics: {diagnostics:?}");
    }

    #[test]
    fn source_benchmark_preregistration_is_valid_but_execution_blocked() {
        let record: Value = serde_json::from_str(BENCHMARK).expect("benchmark preregistration parses");
        let diagnostics = validate_benchmark_preregistration(&record);
        assert!(diagnostics.is_empty(), "benchmark preregistration diagnostics: {diagnostics:?}");
        assert_eq!(record["execution_state"], "blocked_missing_sources");
    }

    #[test]
    fn benchmark_cannot_execute_without_pinned_source_bytes() {
        let mut record: Value = serde_json::from_str(BENCHMARK).expect("benchmark preregistration parses");
        record["status"] = serde_json::json!("preregistered");
        record["execution_state"] = serde_json::json!("ready_for_execution");
        let diagnostics = validate_benchmark_preregistration(&record);
        assert!(diagnostics.iter().any(|d| d.code == "SEM-BENCH-EXECUTION-MISSING-DIGEST"));
        assert!(diagnostics.iter().any(|d| d.code == "SEM-BENCH-EXECUTION-RIGHTS"));
    }

    #[test]
    fn duplicate_event_identity_is_rejected_even_when_values_match() {
        let mut report: Value = serde_json::from_str(GOLDEN).expect("golden fixture parses as JSON");
        let mut duplicate = report["financial_events"][0].clone();
        duplicate["amount"] = serde_json::json!(80.0);
        report["financial_events"].as_array_mut().expect("events array").push(duplicate);
        let diagnostics = validate_report(&report);
        assert!(diagnostics.iter().any(|d| d.code == "SEM-DUPLICATE-EVENT"));
    }

    #[test]
    fn unknown_value_cannot_be_coerced_to_zero() {
        let mut report: Value = serde_json::from_str(GOLDEN).expect("golden fixture parses as JSON");
        report["outcome_vector"][5]["value"] = serde_json::json!(0.0);
        let diagnostics = validate_report(&report);
        assert!(diagnostics.iter().any(|d| d.code == "SEM-UNKNOWN-ZERO-SEPARATION"));
    }

    #[test]
    fn balanced_but_detached_reconciliation_operand_is_rejected() {
        let mut report: Value = serde_json::from_str(GOLDEN).expect("golden fixture parses as JSON");
        // Forge a self-consistent equation by changing both the reported stock and
        // closing side, while leaving the canonical signed-change field untouched.
        report["reconciliations"][1]["operands"][1]["value"] = serde_json::json!(81.0);
        report["reconciliations"][1]["rhs"] = serde_json::json!(246.0);
        report["reconciliations"][1]["lhs"] = serde_json::json!(246.0);
        report["reconciliations"][1]["residual"] = serde_json::json!(0.0);
        report["stock_flow_accounts"][0]["closing_stock"] = serde_json::json!(246.0);
        let diagnostics = validate_report(&report);
        assert!(diagnostics.iter().any(|d| d.code == "SEM-RECONCILIATION-VALUE-BINDING"));
        assert!(diagnostics.iter().any(|d| d.code == "SEM-STOCK-FLOW-DOES-NOT-CLOSE"));
    }

    #[test]
    fn financial_stock_change_must_match_linked_event() {
        let mut report: Value = serde_json::from_str(GOLDEN).expect("golden fixture parses as JSON");
        report["stock_flow_accounts"][0]["changes"][0]["signed_change"] = serde_json::json!(81.0);
        let diagnostics = validate_report(&report);
        assert!(diagnostics.iter().any(|d| d.code == "SEM-STOCK-EVENT-AMOUNT"));
    }
}
