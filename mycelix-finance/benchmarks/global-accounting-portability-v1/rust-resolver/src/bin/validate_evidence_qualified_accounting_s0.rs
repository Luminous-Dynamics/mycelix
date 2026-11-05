use jsonschema::draft202012;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    error::Error,
    fs,
    path::PathBuf,
};

fn main() {
    if let Err(error) = run() {
        eprintln!("evidence-qualified-accounting-s0: {error}");
        std::process::exit(2);
    }
}

fn read_json(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Result<(), String> {
    draft202012::meta::validate(schema)
        .map_err(|error| format!("{label}: invalid Draft 2020-12 schema: {error}"))?;
    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("{label}: could not compile schema: {error}"))?;
    let errors = validator
        .iter_errors(instance)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect::<Vec<_>>();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!("{label}: {}", errors.join("; ")))
    }
}

fn array<'a>(value: &'a Value, key: &str) -> Result<&'a [Value], String> {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| format!("missing or invalid array: {key}"))
}

fn string<'a>(value: &'a Value, key: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("missing or invalid string: {key}"))
}

fn optional_string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn case_by_id<'a>(bundle: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(bundle, "cases")?
        .iter()
        .find(|case| optional_string(case, "case_id") == Some(case_id))
        .ok_or_else(|| format!("missing candidate case {case_id}"))
}

fn document_by_ref<'a>(case: &'a Value, document_ref: &str) -> Result<&'a Value, String> {
    array(case, "documents")?
        .iter()
        .find(|document| optional_string(document, "document_ref") == Some(document_ref))
        .ok_or_else(|| format!("missing document {document_ref}"))
}

fn is_known_fact(case: &Value, fact: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let recorded_at = string(fact, "recorded_at")?;
    let document_ref = string(fact, "document_ref")?;
    let document = document_by_ref(case, document_ref)?;
    let received_at = string(document, "received_at")?;
    let period = case.get("accounting_period").ok_or("missing accounting period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    let valid_from = string(fact, "valid_from")?;
    let valid_to = fact.get("valid_to").and_then(Value::as_str);
    let valid_time_overlaps = valid_from <= period_end
        && valid_to.is_none_or(|end| end >= period_start);
    Ok(recorded_at <= frontier && received_at <= frontier && valid_time_overlaps)
}

fn is_eligible_assertion(case: &Value, fact: &Value) -> Result<bool, String> {
    Ok(is_known_fact(case, fact)?
        && optional_string(fact, "assertion_state") == Some("asserted")
        && !fact.get("value").is_none_or(Value::is_null))
}

fn facts_of_type<'a>(case: &'a Value, fact_type: &str) -> Result<Vec<&'a Value>, String> {
    let mut found = Vec::new();
    for fact in array(case, "facts")? {
        if optional_string(fact, "fact_type") == Some(fact_type)
            && is_eligible_assertion(case, fact)?
        {
            found.push(fact);
        }
    }
    Ok(found)
}

fn future_evidence(case: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    for document in array(case, "documents")? {
        if string(document, "received_at")? > frontier {
            return Ok(true);
        }
    }
    for fact in array(case, "facts")? {
        if string(fact, "recorded_at")? > frontier {
            return Ok(true);
        }
    }
    Ok(false)
}

fn source_integrity_failed(case: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    Ok(array(case, "documents")?.iter().any(|document| {
        optional_string(document, "integrity_state") == Some("synthetic_mismatch")
            && optional_string(document, "received_at").is_some_and(|received| received <= frontier)
    }))
}

fn credit_note_parent_missing(case: &Value) -> Result<bool, String> {
    let documents = array(case, "documents")?;
    for document in documents {
        if optional_string(document, "document_type") != Some("credit_note") {
            continue;
        }
        let document_ref = string(document, "document_ref")?;
        for fact in array(case, "facts")? {
            if optional_string(fact, "fact_type") != Some("original_invoice_id")
                || optional_string(fact, "document_ref") != Some(document_ref)
                || !is_eligible_assertion(case, fact)?
            {
                continue;
            }
            let original_id = fact
                .get("value")
                .and_then(Value::as_str)
                .ok_or_else(|| format!("{document_ref}: original invoice ID is not a string"))?;
            let frontier = string(case, "knowledge_at")?;
            let found = documents.iter().any(|candidate| {
                optional_string(candidate, "external_document_id") == Some(original_id)
                    && optional_string(candidate, "received_at").is_some_and(|received| received <= frontier)
            });
            if !found {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn contradictory_entity_evidence(case: &Value) -> Result<bool, String> {
    let values = facts_of_type(case, "reporting_entity_ref")?
        .iter()
        .filter_map(|fact| fact.get("value").and_then(Value::as_str))
        .collect::<BTreeSet<_>>();
    Ok(values.len() > 1)
}

fn contradictory_amount_evidence(case: &Value) -> Result<bool, String> {
    let invoices = facts_of_type(case, "invoice_total_minor")?;
    let purchase_orders = facts_of_type(case, "purchase_order_total_minor")?;
    for invoice in &invoices {
        let invoice_amount = invoice.get("value").and_then(Value::as_i64);
        let invoice_currency = optional_string(invoice, "currency");
        for purchase_order in &purchase_orders {
            if invoice_currency == optional_string(purchase_order, "currency")
                && invoice_amount.is_some()
                && invoice_amount != purchase_order.get("value").and_then(Value::as_i64)
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn fx_rate_missing(case: &Value) -> Result<bool, String> {
    let presentation_currency = string(case, "presentation_currency")?;
    for invoice in facts_of_type(case, "invoice_total_minor")? {
        let invoice_currency = optional_string(invoice, "currency").unwrap_or_default();
        if invoice_currency != presentation_currency
            && facts_of_type(case, "fx_rate_to_presentation")?.is_empty()
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn service_start_missing(case: &Value) -> Result<bool, String> {
    let invoice_document_exists = array(case, "documents")?.iter()
        .any(|document| optional_string(document, "document_type") == Some("invoice"));
    if !invoice_document_exists {
        return Ok(false);
    }
    Ok(facts_of_type(case, "invoice_total_minor")?.is_empty()
        || facts_of_type(case, "service_period_start")?.is_empty())
}

fn account_mapping_missing(case: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let mapping_document_exists = array(case, "documents")?
        .iter()
        .any(|document| optional_string(document, "document_type") == Some("account_mapping")
            && optional_string(document, "received_at").is_some_and(|received| received <= frontier));
    if !mapping_document_exists {
        return Ok(false);
    }
    Ok(facts_of_type(case, "account_mapping_ref")?.is_empty())
}

fn derive_expectation(case: &Value) -> Result<(&'static str, &'static str), String> {
    if source_integrity_failed(case)? {
        return Ok(("abstain", "source_integrity_failed"));
    }
    if credit_note_parent_missing(case)? {
        return Ok(("escalate", "credit_note_parent_missing"));
    }
    if contradictory_amount_evidence(case)? {
        return Ok(("escalate", "contradictory_amount_evidence"));
    }
    if contradictory_entity_evidence(case)? {
        return Ok(("clarify", "contradictory_entity_evidence"));
    }
    if future_evidence(case)? {
        return Ok(("clarify", "evidence_after_knowledge_frontier"));
    }
    if fx_rate_missing(case)? {
        return Ok(("clarify", "fx_rate_missing"));
    }
    if service_start_missing(case)? {
        return Ok(("clarify", "required_fact_missing"));
    }
    if account_mapping_missing(case)? {
        return Ok(("clarify", "account_mapping_missing"));
    }
    Ok(("deliver", "no_blocker_detected"))
}

fn validate_case_references(case: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let knowledge_at = string(case, "knowledge_at")?;
    let mut document_refs = BTreeSet::new();
    let mut document_identity = BTreeMap::<(String, String, String), String>::new();

    for document in array(case, "documents")? {
        let document_ref = string(document, "document_ref")?.to_owned();
        if !document_refs.insert(document_ref.clone()) {
            return Err(format!("{case_id}: duplicate document_ref {document_ref}"));
        }
        let identity = (
            string(document, "namespace")?.to_owned(),
            string(document, "external_document_id")?.to_owned(),
            string(document, "revision")?.to_owned(),
        );
        let duplicate_of = optional_string(document, "duplicate_of_document_ref");
        if let Some(canonical_ref) = document_identity.get(&identity) {
            if duplicate_of != Some(canonical_ref.as_str()) {
                return Err(format!(
                    "{case_id}: duplicate source identity {} must point to canonical document {canonical_ref}",
                    identity.1
                ));
            }
        } else {
            if duplicate_of.is_some() {
                return Err(format!(
                    "{case_id}: {} points to a duplicate without an earlier canonical source",
                    document_ref
                ));
            }
            document_identity.insert(identity, document_ref);
        }
    }

    let mut fact_refs = BTreeSet::new();
    for fact in array(case, "facts")? {
        let fact_ref = string(fact, "fact_ref")?;
        if !fact_refs.insert(fact_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate fact_ref {fact_ref}"));
        }
        let source_document = string(fact, "document_ref")?;
        if !document_refs.contains(source_document) {
            return Err(format!("{case_id}/{fact_ref}: unknown source document {source_document}"));
        }
        let state = string(fact, "assertion_state")?;
        let value_is_null = fact.get("value").is_some_and(Value::is_null);
        if state == "asserted" && value_is_null {
            return Err(format!("{case_id}/{fact_ref}: asserted fact has null value"));
        }
        if state != "asserted" && !value_is_null {
            return Err(format!("{case_id}/{fact_ref}: non-asserted fact must have null value"));
        }
        if string(fact, "recorded_at")? > knowledge_at && is_eligible_assertion(case, fact)? {
            return Err(format!("{case_id}/{fact_ref}: future fact is marked eligible"));
        }
    }
    Ok(())
}

fn validate_oracle_references(
    case: &Value,
    expectation: &Value,
) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let known_fact_refs = array(case, "facts")?
        .iter()
        .filter_map(|fact| {
            is_eligible_assertion(case, fact)
                .ok()
                .filter(|eligible| *eligible)
                .and_then(|_| optional_string(fact, "fact_ref"))
        })
        .map(ToOwned::to_owned)
        .collect::<BTreeSet<_>>();

    let mut required = BTreeSet::new();
    for value in array(expectation, "required_source_fact_refs")? {
        let fact_ref = value.as_str().ok_or("oracle source fact ref is not a string")?;
        if !known_fact_refs.contains(fact_ref) {
            return Err(format!("{case_id}: oracle requires unavailable or non-asserted source fact {fact_ref}"));
        }
        required.insert(fact_ref.to_owned());
    }

    let mut journal_entry_refs = BTreeSet::new();
    let mut line_refs = BTreeSet::new();
    let mut debit_totals = BTreeMap::<(String, String, String, String), i128>::new();
    let mut credit_totals = BTreeMap::<(String, String, String, String), i128>::new();
    let mut journal_fact_refs = BTreeSet::new();

    for entry in array(expectation, "expected_journal_entries")? {
        let entry_ref = string(entry, "entry_ref")?.to_owned();
        if !journal_entry_refs.insert(entry_ref.clone()) {
            return Err(format!("{case_id}: duplicate journal entry {entry_ref}"));
        }
        for line in array(entry, "lines")? {
            let line_ref = string(line, "line_ref")?;
            if !line_refs.insert(line_ref.to_owned()) {
                return Err(format!("{case_id}: duplicate journal line {line_ref}"));
            }
            let currency = string(line, "currency")?.to_owned();
            let entity = string(line, "entity_ref")?.to_owned();
            if entity != string(case, "reporting_entity_ref")? {
                return Err(format!("{case_id}/{line_ref}: journal entity does not match reporting entity"));
            }
            let period = string(line, "period_ref")?.to_owned();
            let key = (entry_ref.clone(), currency, entity, period);
            let amount = line
                .get("amount_minor")
                .and_then(Value::as_i64)
                .ok_or_else(|| format!("{case_id}/{line_ref}: amount_minor is not an integer"))?
                as i128;
            match string(line, "side")? {
                "debit" => *debit_totals.entry(key).or_default() += amount,
                "credit" => *credit_totals.entry(key).or_default() += amount,
                other => return Err(format!("{case_id}/{line_ref}: invalid journal side {other}")),
            }
            for fact in array(line, "source_fact_refs")? {
                let fact_ref = fact.as_str().ok_or("journal source fact ref is not a string")?;
                if !known_fact_refs.contains(fact_ref) {
                    return Err(format!("{case_id}/{line_ref}: journal cites unavailable source fact {fact_ref}"));
                }
                journal_fact_refs.insert(fact_ref.to_owned());
            }
        }
    }

    for (key, debit) in &debit_totals {
        let credit = credit_totals.get(key).copied().unwrap_or_default();
        if *debit != credit {
            return Err(format!("{case_id}: unbalanced journal {:?}: debits {debit}, credits {credit}", key));
        }
    }
    for (key, credit) in &credit_totals {
        let debit = debit_totals.get(key).copied().unwrap_or_default();
        if *debit != *credit {
            return Err(format!("{case_id}: unbalanced journal {:?}: debits {debit}, credits {credit}", key));
        }
    }

    if !array(expectation, "expected_journal_entries")?.is_empty() && journal_fact_refs != required {
        return Err(format!(
            "{case_id}: required-source refs and journal-source refs differ; required={required:?}, journal={journal_fact_refs:?}"
        ));
    }

    let mut statement_refs = BTreeSet::new();
    for statement in array(expectation, "expected_statement_lines")? {
        let statement_ref = string(statement, "statement_line_ref")?;
        if !statement_refs.insert(statement_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate statement line {statement_ref}"));
        }
        if string(statement, "entity_ref")? != string(case, "reporting_entity_ref")? {
            return Err(format!("{case_id}/{statement_ref}: statement entity mismatch"));
        }
        let linked_entries = array(statement, "source_journal_entry_refs")?;
        if linked_entries.is_empty() {
            return Err(format!("{case_id}/{statement_ref}: statement line has no journal lineage"));
        }
        let linked = linked_entries
            .iter()
            .map(|value| value.as_str().ok_or("statement journal ref is not a string"))
            .collect::<Result<BTreeSet<_>, _>>()?;
        if linked.iter().any(|reference| !journal_entry_refs.contains(*reference)) {
            return Err(format!("{case_id}/{statement_ref}: statement references unknown journal entry"));
        }
        let statement_amount = statement
            .get("amount_minor")
            .and_then(Value::as_i64)
            .ok_or_else(|| format!("{case_id}/{statement_ref}: invalid statement amount"))?;
        let account = string(statement, "account_ref")?;
        let currency = string(statement, "currency")?;
        let period = string(statement, "period_ref")?;
        let normal_balance = string(statement, "normal_balance")?;
        if !matches!(normal_balance, "debit" | "credit") {
            return Err(format!("{case_id}/{statement_ref}: invalid account normal balance"));
        }
        let mut reconstructed: i128 = 0;
        for entry in array(expectation, "expected_journal_entries")? {
            if linked.contains(string(entry, "entry_ref")?) {
                for line in array(entry, "lines")? {
                    if string(line, "account_ref")? == account
                        && string(line, "currency")? == currency
                        && string(line, "period_ref")? == period
                        && string(line, "side")? == normal_balance
                    {
                        reconstructed += line
                            .get("amount_minor")
                            .and_then(Value::as_i64)
                            .ok_or("invalid journal amount")? as i128;
                    }
                }
            }
        }
        if reconstructed != statement_amount as i128 {
            return Err(format!(
                "{case_id}/{statement_ref}: statement amount {statement_amount} does not reconstruct from journal {normal_balance} balance {reconstructed}"
            ));
        }
    }

    if !array(expectation, "expected_journal_entries")?.is_empty() {
        let mut invoice_totals = BTreeMap::<String, i128>::new();
        for fact in facts_of_type(case, "invoice_total_minor")? {
            let currency = optional_string(fact, "currency").unwrap_or_default().to_owned();
            let amount = fact.get("value").and_then(Value::as_i64)
                .ok_or_else(|| format!("{case_id}: invoice total is not an integer"))?;
            *invoice_totals.entry(currency).or_default() += amount as i128;
        }
        let mut journal_totals = BTreeMap::<String, i128>::new();
        for (key, debit) in &debit_totals {
            *journal_totals.entry(key.1.clone()).or_default() += debit;
        }
        if invoice_totals != journal_totals {
            return Err(format!("{case_id}: total journal debits {journal_totals:?} do not equal evidenced invoice totals {invoice_totals:?}"));
        }
    }

    Ok(())
}

fn validate_oracle(bundle: &Value, oracle: &Value) -> Result<(), String> {
    let cases = array(bundle, "cases")?;
    let expectations = array(oracle, "expectations")?;
    if cases.len() != expectations.len() {
        return Err(format!("candidate count {} differs from oracle count {}", cases.len(), expectations.len()));
    }
    let mut seen = BTreeSet::new();
    for expectation in expectations {
        let case_id = string(expectation, "case_id")?;
        if !seen.insert(case_id.to_owned()) {
            return Err(format!("duplicate oracle case {case_id}"));
        }
        let case = case_by_id(bundle, case_id)?;
        let (disposition, reason) = derive_expectation(case)?;
        if string(expectation, "expected_disposition")? != disposition
            || string(expectation, "primary_reason_code")? != reason
        {
            return Err(format!(
                "{case_id}: oracle says {}/{} but deterministic candidate analysis says {disposition}/{reason}",
                string(expectation, "expected_disposition")?,
                string(expectation, "primary_reason_code")?
            ));
        }
        if string(expectation, "expected_disposition")? == "clarify" {
            if array(expectation, "expected_questions")?.is_empty() {
                return Err(format!("{case_id}: clarify disposition has no question"));
            }
            for question in array(expectation, "expected_questions")? {
                let blocking_type = string(question, "blocking_fact_type")?;
                let question_matches = array(case, "facts")?.iter().any(|fact| {
                    optional_string(fact, "fact_type") == Some(blocking_type)
                        && (optional_string(fact, "assertion_state") != Some("asserted")
                            || !is_known_fact(case, fact).unwrap_or(false))
                }) || (blocking_type == "reporting_entity_ref" && contradictory_entity_evidence(case).unwrap_or(false));
                if !question_matches {
                    return Err(format!("{case_id}: question {} does not target an observed blocker {}", string(question, "question_id")?, blocking_type));
                }
            }
        }
        if expectation.get("expected_no_posting").and_then(Value::as_bool) != Some(true) {
            return Err(format!("{case_id}: benchmark oracle must never post to a live ledger"));
        }
        validate_oracle_references(case, expectation)?;
    }
    Ok(())
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../evidence-qualified-accounting-v1");
    let mut data_dir = root;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a directory")?),
            "--help" | "-h" => {
                println!("Validates synthetic evidence-qualified accounting tasks, clarification oracle, bitemporal source facts, journal balance and statement reconstruction. No ledger posting occurs.");
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}").into()),
        }
    }

    let task_schema: Value = read_json(data_dir.join("evidence_qualified_accounting_task_bundle_v1.0.0.schema.json"))?;
    let oracle_schema: Value = read_json(data_dir.join("evidence_qualified_accounting_oracle_v1.0.0.schema.json"))?;
    let candidate_path = data_dir.join("fixtures/evidence_qualified_accounting_candidates_v1.0.0.json");
    let oracle_path = data_dir.join("fixtures/evidence_qualified_accounting_oracle_v1.0.0.json");
    let candidate_bytes = fs::read(&candidate_path)?;
    let oracle_bytes = fs::read(&oracle_path)?;
    let candidates: Value = serde_json::from_slice(&candidate_bytes)?;
    let oracle: Value = serde_json::from_slice(&oracle_bytes)?;

    validate_instance("candidate task bundle", &task_schema, &candidates)?;
    validate_instance("accounting oracle", &oracle_schema, &oracle)?;

    let mut case_ids = BTreeSet::new();
    for case in array(&candidates, "cases")? {
        let case_id = string(case, "case_id")?;
        if !case_ids.insert(case_id.to_owned()) {
            return Err(format!("duplicate case ID {case_id}").into());
        }
        validate_case_references(case)?;
    }
    validate_oracle(&candidates, &oracle)?;

    let candidate_digest = hex::encode(Sha256::digest(&candidate_bytes));
    let oracle_digest = hex::encode(Sha256::digest(&oracle_bytes));
    println!(
        "PASS: {} evidence-qualified accounting cases; exact dispositions, source references, double-entry journals, statement reconstruction, temporal frontiers and no-posting constraints validated. candidate_sha256={} oracle_sha256={}. Synthetic benchmark only; no professional or legal conclusion is produced.",
        case_ids.len(),
        candidate_digest,
        oracle_digest
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../evidence-qualified-accounting-v1/fixtures");
        (
            read_json(root.join("evidence_qualified_accounting_candidates_v1.0.0.json")).unwrap(),
            read_json(root.join("evidence_qualified_accounting_oracle_v1.0.0.json")).unwrap(),
        )
    }

    #[test]
    fn all_eleven_public_oracle_rows_match_deterministic_case_analysis() {
        let (candidates, oracle) = fixtures();
        assert_eq!(array(&candidates, "cases").unwrap().len(), 11);
        validate_oracle(&candidates, &oracle).unwrap();
    }

    #[test]
    fn duplicate_invoice_is_linked_to_one_canonical_source_identity() {
        let (candidates, _) = fixtures();
        let case = case_by_id(&candidates, "ACCT-004-DUPLICATE-INVOICE").unwrap();
        validate_case_references(case).unwrap();
        assert_eq!(
            array(case, "documents").unwrap().iter()
                .filter(|document| optional_string(document, "external_document_id") == Some("INV-4004"))
                .count(),
            2
        );
        assert_eq!(derive_expectation(case).unwrap(), ("deliver", "no_blocker_detected"));
    }

    #[test]
    fn future_recorded_acceptance_is_excluded_at_the_requested_frontier() {
        let (candidates, _) = fixtures();
        let case = case_by_id(&candidates, "ACCT-007-FUTURE-RECORDED-ACCEPTANCE").unwrap();
        assert!(future_evidence(case).unwrap());
        assert!(facts_of_type(case, "acceptance_date").unwrap().is_empty());
        assert_eq!(derive_expectation(case).unwrap(), ("clarify", "evidence_after_knowledge_frontier"));
    }

    #[test]
    fn source_integrity_mismatch_forces_abstention() {
        let (candidates, _) = fixtures();
        let case = case_by_id(&candidates, "ACCT-009-SOURCE-INTEGRITY-MISMATCH").unwrap();
        assert_eq!(derive_expectation(case).unwrap(), ("abstain", "source_integrity_failed"));
    }

    #[test]
    fn conflicting_purchase_order_and_invoice_amounts_escalate() {
        let (candidates, _) = fixtures();
        let case = case_by_id(&candidates, "ACCT-010-CONTRADICTORY-AMOUNT").unwrap();
        assert_eq!(derive_expectation(case).unwrap(), ("escalate", "contradictory_amount_evidence"));
    }

    #[test]
    fn missing_fx_rate_never_produces_a_usd_journal() {
        let (candidates, oracle) = fixtures();
        let case = case_by_id(&candidates, "ACCT-005-MISSING-FX-RATE").unwrap();
        assert_eq!(derive_expectation(case).unwrap(), ("clarify", "fx_rate_missing"));
        let expectation = array(&oracle, "expectations").unwrap().iter()
            .find(|item| optional_string(item, "case_id") == Some("ACCT-005-MISSING-FX-RATE")).unwrap();
        assert!(array(expectation, "expected_journal_entries").unwrap().is_empty());
    }

    #[test]
    fn period_cutoff_allocation_reconstructs_october_and_november_separately() {
        let (candidates, oracle) = fixtures();
        let case = case_by_id(&candidates, "ACCT-008-PERIOD-CUTOFF-ALLOCATION").unwrap();
        let expectation = array(&oracle, "expectations").unwrap().iter()
            .find(|item| optional_string(item, "case_id") == Some("ACCT-008-PERIOD-CUTOFF-ALLOCATION")).unwrap();
        validate_oracle_references(case, expectation).unwrap();
        assert_eq!(array(expectation, "expected_journal_entries").unwrap().len(), 2);
        assert_eq!(array(expectation, "expected_statement_lines").unwrap().len(), 2);
    }

    #[test]
    fn every_non_deliver_disposition_has_no_journal_or_statement_output() {
        let (candidates, oracle) = fixtures();
        for expectation in array(&oracle, "expectations").unwrap() {
            if optional_string(expectation, "expected_disposition") != Some("deliver") {
                assert!(array(expectation, "expected_journal_entries").unwrap().is_empty());
                assert!(array(expectation, "expected_statement_lines").unwrap().is_empty());
                assert_eq!(expectation["expected_no_posting"], true);
            }
        }
        assert_eq!(array(&candidates, "cases").unwrap().len(), 11);
    }

    #[test]
    fn future_source_integrity_result_cannot_retroactively_force_abstention() {
        let (mut candidates, _) = fixtures();
        let case = candidates["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("ACCT-009-SOURCE-INTEGRITY-MISMATCH")).unwrap();
        case["documents"][0]["received_at"] = Value::String("2026-11-05T10:45:00Z".to_owned());
        assert!(!source_integrity_failed(case).unwrap());
        assert_eq!(derive_expectation(case).unwrap(), ("clarify", "evidence_after_knowledge_frontier"));
    }

    #[test]
    fn future_fx_fact_does_not_satisfy_an_earlier_conversion_requirement() {
        let (mut candidates, _) = fixtures();
        let case = candidates["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("ACCT-005-MISSING-FX-RATE")).unwrap();
        let fx = case["facts"].as_array_mut().unwrap().iter_mut()
            .find(|fact| optional_string(fact, "fact_ref") == Some("F-005-FX-UNKNOWN")).unwrap();
        fx["assertion_state"] = Value::String("asserted".to_owned());
        fx["value"] = Value::Number(serde_json::Number::from_f64(1.1).unwrap());
        fx["recorded_at"] = Value::String("2026-11-05T10:45:00Z".to_owned());
        assert!(facts_of_type(case, "fx_rate_to_presentation").unwrap().is_empty());
        assert_eq!(derive_expectation(case).unwrap(), ("clarify", "evidence_after_knowledge_frontier"));
    }

    #[test]
    fn fact_outside_the_accounting_valid_time_is_not_eligible() {
        let (mut candidates, _) = fixtures();
        let case = candidates["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        let amount = case["facts"].as_array_mut().unwrap().iter_mut()
            .find(|fact| optional_string(fact, "fact_ref") == Some("F-001-AMOUNT")).unwrap();
        amount["valid_to"] = Value::String("2026-09-30".to_owned());
        assert!(facts_of_type(case, "invoice_total_minor").unwrap().is_empty());
        assert_eq!(derive_expectation(case).unwrap(), ("clarify", "required_fact_missing"));
    }


    #[test]
    fn per_entry_balance_cannot_be_forged_by_offsetting_two_entries() {
        let (candidates, mut oracle) = fixtures();
        let case = case_by_id(&candidates, "ACCT-008-PERIOD-CUTOFF-ALLOCATION").unwrap();
        let expectation = oracle["expectations"].as_array_mut().unwrap().iter_mut()
            .find(|item| optional_string(item, "case_id") == Some("ACCT-008-PERIOD-CUTOFF-ALLOCATION")).unwrap();
        let entries = expectation["expected_journal_entries"].as_array_mut().unwrap();
        entries[0]["lines"][0]["amount_minor"] = Value::from(170100);
        entries[1]["lines"][1]["amount_minor"] = Value::from(140100);
        for line in entries[1]["lines"].as_array_mut().unwrap() {
            line["period_ref"] = Value::String("2026-10".to_owned());
        }
        assert!(validate_oracle_references(case, expectation).is_err());
    }

    #[test]
    fn future_parent_invoice_does_not_retroactively_resolve_credit_note() {
        let (mut candidates, _) = fixtures();
        let case = candidates["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("ACCT-006-CREDIT-NOTE-PARENT-MISSING")).unwrap();
        case["documents"].as_array_mut().unwrap().push(serde_json::json!({
            "document_ref":"DOC-006-FUTURE-PARENT",
            "external_document_id":"INV-NOT-IN-SOURCE-SET",
            "namespace":"tenant-acme/ap",
            "revision":"r1",
            "document_type":"invoice",
            "source_system":"synthetic_erp",
            "received_at":"2026-11-05T10:45:00Z",
            "integrity_state":"synthetic_match",
            "duplicate_of_document_ref":null
        }));
        assert!(credit_note_parent_missing(case).unwrap());
        assert_eq!(derive_expectation(case).unwrap(), ("escalate", "credit_note_parent_missing"));
    }

    #[test]
    fn complete_invoice_and_duplicate_invoice_each_create_one_balanced_entry() {
        let (candidates, oracle) = fixtures();
        for id in ["ACCT-001-COMPLETE-INVOICE", "ACCT-004-DUPLICATE-INVOICE"] {
            let case = case_by_id(&candidates, id).unwrap();
            let expectation = array(&oracle, "expectations").unwrap().iter()
                .find(|item| optional_string(item, "case_id") == Some(id)).unwrap();
            validate_oracle_references(case, expectation).unwrap();
            assert_eq!(array(expectation, "expected_journal_entries").unwrap().len(), 1);
        }
    }
}
