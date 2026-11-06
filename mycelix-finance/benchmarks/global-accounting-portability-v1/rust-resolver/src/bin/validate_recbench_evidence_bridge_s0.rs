use jsonschema::draft202012;
use serde_json::{json, Value};
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
        eprintln!("recbench-evidence-bridge-s0: {error}");
        std::process::exit(2);
    }
}

fn read_bytes(path: PathBuf) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(fs::read(path)?)
}

fn read_json(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&read_bytes(path)?)?)
}

fn digest(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
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

fn find_ref<'a>(
    values: &'a [Value],
    field: &str,
    reference: &str,
) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, field) == Some(reference))
}

fn source_by_ref<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_ref(array(case, "sources")?, "source_ref", reference)
        .ok_or_else(|| format!("unknown source ref {reference}"))
}

fn assertion_by_ref<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_ref(array(case, "assertions")?, "assertion_ref", reference)
        .ok_or_else(|| format!("unknown assertion ref {reference}"))
}

fn event_by_ref<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_ref(array(case, "events")?, "event_ref", reference)
        .ok_or_else(|| format!("unknown event ref {reference}"))
}

fn optional_integer(value: &Value, key: &str) -> Option<i64> {
    value.get(key).and_then(Value::as_i64)
}

fn frontier<'a>(case: &'a Value) -> Result<&'a str, String> {
    string(case, "knowledge_at")
}

fn source_available(case: &Value, source: &Value) -> Result<bool, String> {
    Ok(string(source, "received_at")? <= frontier(case)?)
}

fn assertion_available(case: &Value, assertion: &Value) -> Result<bool, String> {
    let source = source_by_ref(case, string(assertion, "source_ref")?)?;
    let period = case.get("period").ok_or("missing case period")?;
    let valid_date = string(assertion, "valid_date")?;
    let within_period = valid_date >= string(period, "start")?
        && valid_date <= string(period, "end")?;
    Ok(source_available(case, source)?
        && string(assertion, "recorded_at")? <= frontier(case)?
        && within_period
        && optional_string(assertion, "evidence_state") == Some("asserted")
        && !assertion.get("value").is_none_or(Value::is_null))
}

fn assertions_for_source_type<'a>(
    case: &'a Value,
    source_ref: &str,
    fact_type: &str,
) -> Result<Vec<&'a Value>, String> {
    Ok(array(case, "assertions")?
        .iter()
        .filter(|assertion| {
            optional_string(assertion, "source_ref") == Some(source_ref)
                && optional_string(assertion, "fact_type") == Some(fact_type)
        })
        .collect())
}

fn event_for_source<'a>(case: &'a Value, source_ref: &str) -> Result<&'a Value, String> {
    let events = array(case, "events")?
        .iter()
        .filter(|event| {
            array(event, "source_refs")
                .is_ok_and(|refs| refs.iter().any(|reference| reference.as_str() == Some(source_ref)))
        })
        .collect::<Vec<_>>();
    if events.len() != 1 {
        return Err(format!(
            "{}: expected exactly one event for source {}, found {}",
            string(case, "case_id")?,
            source_ref,
            events.len()
        ));
    }
    Ok(events[0])
}

fn event_available(case: &Value, event: &Value) -> Result<bool, String> {
    if string(event, "recorded_at")? > frontier(case)? {
        return Ok(false);
    }
    for reference in array(event, "source_refs")? {
        let source_ref = reference.as_str().ok_or("event source ref must be a string")?;
        if !source_available(case, source_by_ref(case, source_ref)?)? {
            return Ok(false);
        }
    }
    for reference in array(event, "assertion_refs")? {
        let assertion_ref = reference.as_str().ok_or("event assertion ref must be a string")?;
        if !assertion_available(case, assertion_by_ref(case, assertion_ref)?)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn validate_source_and_assertion_references(case: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let mut source_refs = BTreeSet::new();
    let mut exact_identities = BTreeSet::new();

    for source in array(case, "sources")? {
        let source_ref = string(source, "source_ref")?;
        if !source_refs.insert(source_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate source_ref {source_ref}"));
        }
        let identity = (
            string(source, "tenant_namespace")?.to_owned(),
            string(source, "external_id")?.to_owned(),
            string(source, "revision")?.to_owned(),
            string(source, "source_kind")?.to_owned(),
        );
        if !exact_identities.insert(identity) {
            return Err(format!("{case_id}: duplicate exact source identity"));
        }
    }

    let mut assertion_refs = BTreeSet::new();
    for assertion in array(case, "assertions")? {
        let assertion_ref = string(assertion, "assertion_ref")?;
        if !assertion_refs.insert(assertion_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate assertion_ref {assertion_ref}"));
        }
        source_by_ref(case, string(assertion, "source_ref")?)?;
        match optional_string(assertion, "evidence_state") {
            Some("asserted") if assertion.get("value").is_none_or(Value::is_null) => {
                return Err(format!("{case_id}/{assertion_ref}: asserted fact has null value"));
            }
            Some("unknown" | "invalidated") if !assertion.get("value").is_none_or(Value::is_null) => {
                return Err(format!("{case_id}/{assertion_ref}: unknown/invalidated fact must have null value"));
            }
            _ => {}
        }
    }

    let mut event_refs = BTreeSet::new();
    for event in array(case, "events")? {
        let event_ref = string(event, "event_ref")?;
        if !event_refs.insert(event_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate event_ref {event_ref}"));
        }
        let source_refs = array(event, "source_refs")?;
        let assertion_refs = array(event, "assertion_refs")?;
        if source_refs.is_empty() || assertion_refs.is_empty() {
            return Err(format!("{case_id}/{event_ref}: event lacks source or assertion lineage"));
        }
        for reference in source_refs {
            source_by_ref(case, reference.as_str().ok_or("source ref must be string")?)?;
        }
        for reference in assertion_refs {
            assertion_by_ref(case, reference.as_str().ok_or("assertion ref must be string")?)?;
        }
    }

    Ok(())
}

fn validate_event_assertion_consistency(case: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    for event in array(case, "events")? {
        let event_ref = string(event, "event_ref")?;
        let event_kind = string(event, "event_kind")?;
        let source_refs = array(event, "source_refs")?
            .iter()
            .map(|value| value.as_str().ok_or("event source ref must be string"))
            .collect::<Result<Vec<_>, _>>()?;
        if source_refs.len() != 1 {
            return Err(format!("{case_id}/{event_ref}: S0 events must bind exactly one source object"));
        }
        let source_ref = source_refs[0];
        let expected_type = match event_kind {
            "ledger_charge" | "ledger_refund" | "ledger_dispute" | "bank_deposit" | "reversal" => "amount_minor",
            "settlement_line" | "payout_created" => "net_minor",
            "bank_fee" => "fee_minor",
            other => return Err(format!("{case_id}/{event_ref}: unsupported S0 event kind {other}")),
        };
        let amount_assertions = assertions_for_source_type(case, source_ref, expected_type)?;
        if amount_assertions.len() != 1 {
            return Err(format!(
                "{case_id}/{event_ref}: expected one {expected_type} assertion for source {source_ref}, found {}",
                amount_assertions.len()
            ));
        }
        let assertion = amount_assertions[0];
        let assertion_amount = optional_integer(assertion, "value")
            .ok_or_else(|| format!("{case_id}/{event_ref}: amount assertion must be integer minor units"))?;
        let event_amount = optional_integer(event, "amount_minor")
            .ok_or_else(|| format!("{case_id}/{event_ref}: event amount must be integer minor units"))?;
        if assertion_amount != event_amount
            || optional_string(assertion, "currency") != optional_string(event, "currency")
        {
            return Err(format!("{case_id}/{event_ref}: event amount/currency disagrees with source assertion"));
        }

        if event_kind == "settlement_line" {
            let gross = assertions_for_source_type(case, source_ref, "gross_minor")?;
            let fee = assertions_for_source_type(case, source_ref, "fee_minor")?;
            if gross.len() == 1 && fee.len() == 1 {
                let gross_value = optional_integer(gross[0], "value").ok_or("gross must be integer minor units")?;
                let fee_value = optional_integer(fee[0], "value").ok_or("fee must be integer minor units")?;
                if gross_value.checked_sub(fee_value) != Some(assertion_amount) {
                    return Err(format!("{case_id}/{event_ref}: gross minus fee does not equal net"));
                }
                if optional_string(gross[0], "currency") != optional_string(fee[0], "currency")
                    || optional_string(fee[0], "currency") != optional_string(assertion, "currency")
                {
                    return Err(format!("{case_id}/{event_ref}: gross/fee/net currencies differ"));
                }
            }
        }
    }
    Ok(())
}

fn duplicate_identity_family_count(case: &Value) -> Result<usize, String> {
    let mut groups = BTreeMap::<(String, String, String), usize>::new();
    for source in array(case, "sources")? {
        let key = (
            string(source, "tenant_namespace")?.to_owned(),
            string(source, "external_id")?.to_owned(),
            string(source, "source_kind")?.to_owned(),
        );
        *groups.entry(key).or_default() += 1;
    }
    Ok(groups.values().filter(|count| **count > 1).count())
}

fn source_kind<'a>(case: &'a Value, source_ref: &str) -> Result<&'a str, String> {
    string(source_by_ref(case, source_ref)?, "source_kind")
}

fn dataset_side<'a>(case: &'a Value, source_ref: &str) -> Result<&'a str, String> {
    string(source_by_ref(case, source_ref)?, "dataset_side")
}

fn amount_and_currency_for_source(case: &Value, source_ref: &str) -> Result<(i64, String), String> {
    let event = event_for_source(case, source_ref)?;
    Ok((
        optional_integer(event, "amount_minor")
            .ok_or_else(|| format!("missing event amount for {source_ref}"))?,
        string(event, "currency")?.to_owned(),
    ))
}

fn amount_fact_for_source(
    case: &Value,
    source_ref: &str,
    fact_type: &str,
) -> Result<Option<(i64, String)>, String> {
    let facts = assertions_for_source_type(case, source_ref, fact_type)?;
    if facts.is_empty() {
        return Ok(None);
    }
    if facts.len() != 1 {
        return Err(format!("source {source_ref} has multiple {fact_type} assertions"));
    }
    let fact = facts[0];
    Ok(Some((
        optional_integer(fact, "value").ok_or("amount fact must be integer")?,
        string(fact, "currency")?.to_owned(),
    )))
}

fn validate_link(case: &Value, link: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let kind = string(link, "match_kind")?;
    let refs = array(link, "source_refs")?
        .iter()
        .map(|value| value.as_str().ok_or("match source ref must be string"))
        .collect::<Result<Vec<_>, _>>()?;
    let unique = refs.iter().copied().collect::<BTreeSet<_>>();
    if unique.len() != refs.len() {
        return Err(format!("{case_id}: match contains duplicate source refs"));
    }
    let mut sides = BTreeMap::<&str, Vec<&str>>::new();
    let mut tenant_namespaces = BTreeSet::<String>::new();
    for reference in &refs {
        let source = source_by_ref(case, reference)?;
        if !source_available(case, source)? {
            return Err(format!("{case_id}: expected link uses source {reference} unavailable at frontier"));
        }
        tenant_namespaces.insert(string(source, "tenant_namespace")?.to_owned());
        sides.entry(dataset_side(case, reference)?).or_default().push(source_kind(case, reference)?);
    }
    if tenant_namespaces.len() != 1 {
        return Err(format!("{case_id}: one reconciliation match cannot span tenant namespaces"));
    }

    let expected_amount = optional_integer(link, "amount_minor")
        .ok_or("oracle link amount must be integer minor units")?;
    let expected_currency = string(link, "currency")?;

    match kind {
        "ledger_to_settlement" => {
            let ledger = sides.get("ledger").ok_or("ledger-to-settlement link has no ledger side")?;
            let settlement = sides.get("settlement").ok_or("ledger-to-settlement link has no settlement side")?;
            if ledger.len() != 1 || settlement.len() != 1 || refs.len() != 2
                || !matches!(ledger[0], "charge" | "refund" | "dispute")
                || settlement[0] != "settlement_line"
            {
                return Err(format!("{case_id}: ledger-to-settlement link cardinality/type mismatch"));
            }
            let ledger_source = refs.iter().find(|reference| dataset_side(case, reference).ok() == Some("ledger")).unwrap();
            let settlement_source = refs.iter().find(|reference| dataset_side(case, reference).ok() == Some("settlement")).unwrap();
            let (ledger_value, ledger_currency) = amount_and_currency_for_source(case, ledger_source)?;
            let settlement_gross = amount_fact_for_source(case, settlement_source, "gross_minor")?;
            let settlement_value = settlement_gross
                .unwrap_or(amount_and_currency_for_source(case, settlement_source)?);
            if ledger_value != expected_amount || ledger_currency != expected_currency
                || settlement_value.0 != ledger_value || settlement_value.1 != ledger_currency
            {
                return Err(format!("{case_id}: ledger and settlement gross evidence do not reconcile"));
            }
        }
        "payout_to_bank" => {
            let payout = sides.get("payout").ok_or("payout-to-bank link has no payout side")?;
            let bank = sides.get("bank_statement").ok_or("payout-to-bank link has no bank side")?;
            if payout.len() != 1 || bank.len() != 1 || refs.len() != 2
                || payout[0] != "payout" || bank[0] != "bank_deposit"
            {
                return Err(format!("{case_id}: payout-to-bank link cardinality/type mismatch"));
            }
            let values = refs.iter()
                .map(|reference| amount_and_currency_for_source(case, reference))
                .collect::<Result<Vec<_>, _>>()?;
            if values.iter().any(|value| value.0 != expected_amount || value.1 != expected_currency) {
                return Err(format!("{case_id}: payout and bank deposit do not match"));
            }
        }
        "deposit_unbundle" => {
            let payouts = sides.get("payout").ok_or("deposit unbundle has no payout sources")?;
            let banks = sides.get("bank_statement").ok_or("deposit unbundle has no bank source")?;
            if payouts.is_empty() || banks.len() != 1 || banks[0] != "bank_deposit"
                || payouts.iter().any(|kind| *kind != "payout")
            {
                return Err(format!("{case_id}: deposit-unbundle group cardinality/type mismatch"));
            }
            let bank_ref = refs.iter().find(|reference| dataset_side(case, reference).ok() == Some("bank_statement")).unwrap();
            let (bank_amount, bank_currency) = amount_and_currency_for_source(case, bank_ref)?;
            let mut sum = 0i64;
            for payout_ref in refs.iter().filter(|reference| dataset_side(case, reference).ok() == Some("payout")) {
                let (amount, currency) = amount_and_currency_for_source(case, payout_ref)?;
                if currency != bank_currency {
                    return Err(format!("{case_id}: payout and bank deposit currencies differ"));
                }
                sum = sum.checked_add(amount).ok_or("payout sum overflow")?;
            }
            if sum != bank_amount || sum != expected_amount || bank_currency != expected_currency {
                return Err(format!("{case_id}: payout group sum does not equal bank deposit"));
            }
        }
        "partial_application" => {
            let ledgers = sides.get("ledger").ok_or("partial application has no ledger source")?;
            let settlements = sides.get("settlement").ok_or("partial application has no settlement sources")?;
            if ledgers.len() != 1 || settlements.len() < 2 || refs.len() != ledgers.len() + settlements.len()
                || ledgers[0] != "charge" || settlements.iter().any(|kind| *kind != "settlement_line")
            {
                return Err(format!("{case_id}: partial-application group cardinality/type mismatch"));
            }
            let ledger_ref = refs.iter().find(|reference| dataset_side(case, reference).ok() == Some("ledger")).unwrap();
            let (ledger_amount, ledger_currency) = amount_and_currency_for_source(case, ledger_ref)?;
            let mut sum = 0i64;
            for settlement_ref in refs.iter().filter(|reference| dataset_side(case, reference).ok() == Some("settlement")) {
                let (amount, currency) = amount_and_currency_for_source(case, settlement_ref)?;
                if currency != ledger_currency {
                    return Err(format!("{case_id}: partial captures have currency mismatch"));
                }
                sum = sum.checked_add(amount).ok_or("partial settlement sum overflow")?;
            }
            if sum != ledger_amount || sum != expected_amount || ledger_currency != expected_currency {
                return Err(format!("{case_id}: settlement captures do not fully cover the ledger amount"));
            }
        }
        "reversal_pair" => {
            let settlement_kinds = sides.get("settlement").ok_or("reversal pair has no settlement-side sources")?;
            let settlement_line_count = settlement_kinds.iter().filter(|kind| **kind == "settlement_line").count();
            let reversal_count = settlement_kinds.iter().filter(|kind| **kind == "reversal").count();
            if refs.len() != 2 || settlement_line_count != 1 || reversal_count != 1 {
                return Err(format!("{case_id}: reversal pair requires exactly one settlement line and one reversal"));
            }
            let reversal_ref = refs.iter().find(|reference| source_kind(case, reference).ok() == Some("reversal"))
                .ok_or("reversal pair has no reversal source")?;
            let settlement_ref = refs.iter().find(|reference| source_kind(case, reference).ok() == Some("settlement_line"))
                .ok_or("reversal pair has no settlement source")?;
            let (settlement_amount, currency) = amount_and_currency_for_source(case, settlement_ref)?;
            let (reversal_amount, reversal_currency) = amount_and_currency_for_source(case, reversal_ref)?;
            if currency != reversal_currency || currency != expected_currency
                || settlement_amount.checked_add(reversal_amount) != Some(expected_amount)
                || expected_amount != 0
            {
                return Err(format!("{case_id}: reversal does not conserve the original settlement"));
            }
        }
        other => return Err(format!("{case_id}: unsupported expected match kind {other}")),
    }

    Ok(())
}

fn validate_oracle_case(case: &Value, expectation: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    if string(expectation, "case_id")? != case_id {
        return Err(format!("{case_id}: candidate/oracle ID mismatch"));
    }
    for link in array(expectation, "expected_links")? {
        validate_link(case, link)?;
    }
    let mut unresolved = BTreeSet::new();
    for (field, label) in [
        ("expected_review_source_refs", "review"),
        ("expected_unmatched_source_refs", "unmatched"),
        ("expected_excluded_source_refs", "excluded"),
    ] {
        for reference in array(expectation, field)? {
            let source_ref = reference.as_str().ok_or("oracle source ref must be a string")?;
            if !unresolved.insert(source_ref.to_owned()) {
                return Err(format!("{case_id}: source {source_ref} appears in multiple terminal categories"));
            }
            let source = source_by_ref(case, source_ref)?;
            let available = source_available(case, source)?;
            match label {
                "review" | "unmatched" if !available => {
                    return Err(format!("{case_id}: {label} source {source_ref} is not available at frontier"));
                }
                "excluded" if available => {
                    return Err(format!("{case_id}: excluded source {source_ref} is available at frontier"));
                }
                _ => {}
            }
        }
    }
    let unresolved_count = unresolved.len();
    if optional_boolean(expectation, "forbid_auto_close") == Some(true) && unresolved_count == 0 {
        return Err(format!("{case_id}: forbid_auto_close is true without any unresolved/excluded source"));
    }
    if optional_boolean(expectation, "forbid_auto_close") == Some(false) && unresolved_count > 0 {
        return Err(format!("{case_id}: unresolved source exists but oracle does not forbid auto-close"));
    }
    Ok(())
}

fn optional_boolean(value: &Value, key: &str) -> Option<bool> {
    value.get(key).and_then(Value::as_bool)
}

fn analyze_case(case: &Value, expectation: &Value) -> Result<Value, String> {
    validate_source_and_assertion_references(case)?;
    validate_event_assertion_consistency(case)?;
    validate_oracle_case(case, expectation)?;

    let mut available_assertions = 0usize;
    let mut future_assertions = 0usize;
    for assertion in array(case, "assertions")? {
        if assertion_available(case, assertion)? {
            available_assertions += 1;
        } else {
            let source = source_by_ref(case, string(assertion, "source_ref")?)?;
            if string(assertion, "recorded_at")? > frontier(case)?
                || string(source, "received_at")? > frontier(case)?
            {
                future_assertions += 1;
            }
        }
    }

    let mut available_events = 0usize;
    let mut future_events = 0usize;
    for event in array(case, "events")? {
        if event_available(case, event)? {
            available_events += 1;
        } else if string(event, "recorded_at")? > frontier(case)?
            || array(event, "source_refs")?.iter().any(|reference| {
                reference.as_str()
                    .and_then(|source_ref| source_by_ref(case, source_ref).ok())
                    .is_some_and(|source| optional_string(source, "received_at")
                        .is_some_and(|received| Some(received) > Some(frontier(case).unwrap_or(""))))
            })
        {
            future_events += 1;
        }
    }

    let duplicate_families = duplicate_identity_family_count(case)?;
    let review_count = array(expectation, "expected_review_source_refs")?.len();
    let unmatched_count = array(expectation, "expected_unmatched_source_refs")?.len();
    let excluded_count = array(expectation, "expected_excluded_source_refs")?.len();
    let forbid_auto_close = optional_boolean(expectation, "forbid_auto_close")
        .ok_or("oracle forbid_auto_close must be boolean")?;

    let mut findings = BTreeSet::<&str>::new();
    if future_assertions > 0 || future_events > 0 {
        findings.insert("future_source_or_fact_excluded");
    }
    if duplicate_families > 0 {
        findings.insert("duplicate_source_identity_requires_review");
    }
    if forbid_auto_close && review_count > 0 {
        findings.insert("ambiguous_case_forbids_auto_close");
    }
    if unmatched_count > 0 {
        findings.insert("unmatched_source_at_frontier");
    }
    if !array(expectation, "expected_links")?.is_empty() {
        findings.insert("event_oracle_link_validated");
    }

    Ok(json!({
        "case_id": string(case, "case_id")?,
        "source_object_count": array(case, "sources")?.len(),
        "assertion_count": array(case, "assertions")?.len(),
        "available_assertion_count": available_assertions,
        "future_assertion_count": future_assertions,
        "event_count": array(case, "events")?.len(),
        "available_event_count": available_events,
        "future_event_count": future_events,
        "duplicate_source_identity_family_count": duplicate_families,
        "expected_link_count": array(expectation, "expected_links")?.len(),
        "expected_review_source_count": review_count,
        "expected_unmatched_source_count": unmatched_count,
        "expected_excluded_source_count": excluded_count,
        "forbid_auto_close": forbid_auto_close,
        "finding_codes": findings.into_iter().collect::<Vec<_>>()
    }))
}

fn build_report(
    candidates: &Value,
    oracle: &Value,
    candidate_digest: String,
    oracle_digest: String,
) -> Result<Value, String> {
    let cases = array(candidates, "cases")?;
    let expectations = array(oracle, "expectations")?;
    if cases.len() != expectations.len() {
        return Err("candidate and oracle case counts differ".to_owned());
    }
    let mut expected_by_id = BTreeMap::<String, &Value>::new();
    for expectation in expectations {
        let case_id = string(expectation, "case_id")?.to_owned();
        if expected_by_id.insert(case_id.clone(), expectation).is_some() {
            return Err(format!("duplicate oracle case ID {case_id}"));
        }
    }
    let mut candidate_ids = BTreeSet::new();
    let mut totals = BTreeMap::<String, usize>::new();
    let fields = [
        "source_object_count",
        "assertion_count",
        "available_assertion_count",
        "future_assertion_count",
        "event_count",
        "available_event_count",
        "future_event_count",
        "duplicate_source_identity_family_count",
        "expected_link_count",
        "expected_review_source_count",
        "expected_unmatched_source_count",
        "expected_excluded_source_count",
    ];
    let mut case_results = Vec::new();
    let mut forbid_count = 0usize;

    for case in cases {
        let case_id = string(case, "case_id")?;
        if !candidate_ids.insert(case_id.to_owned()) {
            return Err(format!("duplicate candidate case ID {case_id}"));
        }
        let expectation = expected_by_id.get(case_id)
            .ok_or_else(|| format!("missing oracle case {case_id}"))?;
        let result = analyze_case(case, expectation)?;
        if result["forbid_auto_close"].as_bool() == Some(true) {
            forbid_count += 1;
        }
        for field in fields {
            *totals.entry(field.to_owned()).or_default() +=
                result[field].as_u64().unwrap_or(0) as usize;
        }
        case_results.push(result);
    }
    if candidate_ids != expected_by_id.keys().cloned().collect() {
        return Err("candidate/oracle case ID sets differ".to_owned());
    }
    totals.insert("forbid_auto_close_case_count".to_owned(), forbid_count);

    Ok(json!({
        "report_id": "MYCELIX-RECBENCH-BRIDGE-S0-VALIDATION",
        "schema_version": "1.0.0",
        "claim_ceiling": "synthetic_bridge_structure_and_oracle_integrity_not_recbench_performance",
        "candidate_sha256": candidate_digest,
        "oracle_sha256": oracle_digest,
        "case_count": case_results.len(),
        "totals": totals,
        "case_results": case_results
    }))
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../recbench-evidence-bridge-v1");
    let mut data_dir = root;
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a directory")?),
            "--output" => output_path = Some(PathBuf::from(args.next().ok_or("--output requires a file path")?)),
            "--help" | "-h" => {
                println!(
                    "Validates synthetic RecBench evidence-bridge cases, oracle links and source frontier rules.\n\n\
                     --data-dir PATH  Bridge artifact directory\n\
                     --output PATH    Write report JSON to PATH\n\
                     --help           Show usage"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let candidate_schema: Value = read_json(data_dir.join("recbench_bridge_task_bundle_v1.0.0.schema.json"))?;
    let oracle_schema: Value = read_json(data_dir.join("recbench_bridge_oracle_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("recbench_bridge_validation_report_v1.0.0.schema.json"))?;
    let snapshot_schema: Value = read_json(data_dir.join("recbench_source_snapshot_v1.0.0.schema.json"))?;
    let snapshot: Value = read_json(data_dir.join("fixtures/recbench_source_snapshot_v1.0.0.json"))?;
    let candidate_path = data_dir.join("fixtures/recbench_bridge_candidates_v1.0.0.json");
    let oracle_path = data_dir.join("fixtures/recbench_bridge_oracle_v1.0.0.json");
    let candidate_bytes = read_bytes(candidate_path)?;
    let oracle_bytes = read_bytes(oracle_path)?;
    let candidates: Value = serde_json::from_slice(&candidate_bytes)?;
    let oracle: Value = serde_json::from_slice(&oracle_bytes)?;

    validate_instance("RecBench source snapshot", &snapshot_schema, &snapshot)?;
    validate_instance("RecBench bridge candidates", &candidate_schema, &candidates)?;
    validate_instance("RecBench bridge oracle", &oracle_schema, &oracle)?;

    let report = build_report(
        &candidates,
        &oracle,
        digest(&candidate_bytes),
        digest(&oracle_bytes),
    )?;
    validate_instance("RecBench bridge validation report", &report_schema, &report)?;
    let output = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &output)?;
            println!("Wrote RecBench bridge validation report for {} cases.", report["case_count"]);
        }
        None => println!("{}", String::from_utf8(output)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value, Value, Value) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../recbench-evidence-bridge-v1");
        (
            read_json(root.join("fixtures/recbench_bridge_candidates_v1.0.0.json")).unwrap(),
            read_json(root.join("fixtures/recbench_bridge_oracle_v1.0.0.json")).unwrap(),
            read_json(root.join("recbench_bridge_task_bundle_v1.0.0.schema.json")).unwrap(),
            read_json(root.join("recbench_bridge_oracle_v1.0.0.schema.json")).unwrap(),
            read_json(root.join("recbench_source_snapshot_v1.0.0.schema.json")).unwrap(),
        )
    }

    fn case<'a>(bundle: &'a Value, case_id: &str) -> &'a Value {
        array(bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some(case_id)).unwrap()
    }

    fn expectation<'a>(oracle: &'a Value, case_id: &str) -> &'a Value {
        array(oracle, "expectations").unwrap().iter()
            .find(|item| optional_string(item, "case_id") == Some(case_id)).unwrap()
    }

    #[test]
    fn all_ten_candidate_cases_and_oracle_rows_validate() {
        let (candidates, oracle, candidate_schema, oracle_schema, snapshot_schema) = fixtures();
        validate_instance("candidate", &candidate_schema, &candidates).unwrap();
        validate_instance("oracle", &oracle_schema, &oracle).unwrap();
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../recbench-evidence-bridge-v1");
        let snapshot = read_json(root.join("fixtures/recbench_source_snapshot_v1.0.0.json")).unwrap();
        validate_instance("snapshot", &snapshot_schema, &snapshot).unwrap();
        assert_eq!(array(&candidates, "cases").unwrap().len(), 10);
        assert_eq!(array(&oracle, "expectations").unwrap().len(), 10);
        let report = build_report(&candidates, &oracle, "a".repeat(64), "b".repeat(64)).unwrap();
        assert_eq!(report["case_count"], 10);
        assert_eq!(report["totals"]["source_object_count"], 32);
        assert_eq!(report["totals"]["assertion_count"], 38);
        assert_eq!(report["totals"]["event_count"], 32);
        assert_eq!(report["totals"]["expected_link_count"], 11);
        assert_eq!(report["totals"]["expected_excluded_source_count"], 1);
    }

    #[test]
    fn fee_netting_requires_gross_minus_fee_equals_net() {
        let (bundle, _, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-002-FEE-NETTING");
        validate_event_assertion_consistency(c).unwrap();
        let net = assertions_for_source_type(c, "SRC-002-S", "net_minor").unwrap()[0];
        assert_eq!(net["value"], 48500);
    }

    #[test]
    fn fx_case_keeps_eur_settlement_and_usd_payout_separate() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-003-FX-PAYOUT");
        let e = expectation(&oracle, "RBC-003-FX-PAYOUT");
        validate_oracle_case(c, e).unwrap();
        assert_eq!(amount_and_currency_for_source(c, "SRC-003-S").unwrap(), (9900, "EUR".to_owned()));
        assert_eq!(amount_and_currency_for_source(c, "SRC-003-P").unwrap(), (10890, "USD".to_owned()));
    }

    #[test]
    fn utc_day_boundary_is_not_a_source_identity_mismatch() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-004-TIMEZONE-DAY-BOUNDARY");
        let e = expectation(&oracle, "RBC-004-TIMEZONE-DAY-BOUNDARY");
        validate_oracle_case(c, e).unwrap();
        assert_ne!(
            string(assertion_by_ref(c, "AS-004-L-AMT").unwrap(), "valid_date").unwrap(),
            string(assertion_by_ref(c, "AS-004-S-NET").unwrap(), "valid_date").unwrap()
        );
    }

    #[test]
    fn two_partial_captures_must_sum_exactly_to_the_ledger_amount() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-005-PARTIAL-CAPTURE");
        let e = expectation(&oracle, "RBC-005-PARTIAL-CAPTURE");
        validate_oracle_case(c, e).unwrap();
        let links = array(e, "expected_links").unwrap();
        assert_eq!(links[0]["match_kind"], "partial_application");
    }

    #[test]
    fn duplicate_feed_family_is_reviewed_not_auto_closed() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-006-DUPLICATE-SETTLEMENT-FEED");
        let e = expectation(&oracle, "RBC-006-DUPLICATE-SETTLEMENT-FEED");
        assert_eq!(duplicate_identity_family_count(c).unwrap(), 1);
        validate_oracle_case(c, e).unwrap();
        assert_eq!(e["forbid_auto_close"], true);
    }


    #[test]
    fn cross_tenant_source_group_is_rejected() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-001-EXACT-THREE-WAY");
        let mut e = expectation(&oracle, "RBC-001-EXACT-THREE-WAY").clone();
        e["expected_links"][0]["source_refs"] = json!(["SRC-001-L", "SRC-001-S"]);
        let mut changed = c.clone();
        changed["sources"].as_array_mut().unwrap()
            .iter_mut().find(|source| optional_string(source, "source_ref") == Some("SRC-001-S")).unwrap()["tenant_namespace"] = json!("tenant-other/processor");
        assert!(validate_link(&changed, &e["expected_links"][0]).is_err());
    }

    #[test]
    fn reversal_pair_uses_typed_source_kinds_on_the_settlement_side() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-007-REVERSAL");
        let e = expectation(&oracle, "RBC-007-REVERSAL");
        validate_link(c, &e["expected_links"][1]).unwrap();
    }

    #[test]
    fn reversal_pair_is_zero_sum_but_preserves_both_sources() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-007-REVERSAL");
        let e = expectation(&oracle, "RBC-007-REVERSAL");
        validate_oracle_case(c, e).unwrap();
        assert_eq!(amount_and_currency_for_source(c, "SRC-007-S").unwrap().0
            + amount_and_currency_for_source(c, "SRC-007-R").unwrap().0, 0);
    }

    #[test]
    fn unique_subset_sum_must_equal_the_bank_deposit() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-008-DEPOSIT-UNBUNDLE");
        let e = expectation(&oracle, "RBC-008-DEPOSIT-UNBUNDLE");
        validate_oracle_case(c, e).unwrap();
    }

    #[test]
    fn ambiguous_amount_only_pairs_are_review_only() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-009-PROVABLY-AMBIGUOUS");
        let e = expectation(&oracle, "RBC-009-PROVABLY-AMBIGUOUS");
        assert!(array(e, "expected_links").unwrap().is_empty());
        assert_eq!(array(e, "expected_review_source_refs").unwrap().len(), 4);
        validate_oracle_case(c, e).unwrap();
    }

    #[test]
    fn future_payout_is_excluded_from_historical_bank_matching() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-010-UNMATCHED-BANK-MOVEMENT");
        let e = expectation(&oracle, "RBC-010-UNMATCHED-BANK-MOVEMENT");
        assert!(!source_available(c, source_by_ref(c, "SRC-010-P-FUTURE").unwrap()).unwrap());
        assert!(event_available(c, event_for_source(c, "SRC-010-P-FUTURE").unwrap()).unwrap() == false);
        validate_oracle_case(c, e).unwrap();
        assert_eq!(array(e, "expected_unmatched_source_refs").unwrap()[0], "SRC-010-B");
        assert_eq!(array(e, "expected_excluded_source_refs").unwrap()[0], "SRC-010-P-FUTURE");
    }

    #[test]
    fn future_payout_cannot_be_added_to_a_link_at_the_earlier_frontier() {
        let (mut bundle, _, _, _, _) = fixtures();
        let c = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("RBC-010-UNMATCHED-BANK-MOVEMENT")).unwrap();
        let e = json!({
            "case_id":"RBC-010-UNMATCHED-BANK-MOVEMENT",
            "expected_links":[{
                "match_kind":"payout_to_bank",
                "source_refs":["SRC-010-P-FUTURE","SRC-010-B"],
                "amount_minor":3333,"currency":"USD"
            }],
            "expected_unmatched_source_refs":[],
            "expected_review_source_refs":[],
            "expected_excluded_source_refs":[],
            "forbid_auto_close":false,
            "reason_codes":[]
        });
        assert!(validate_oracle_case(c, &e).is_err());
    }

    #[test]
    fn changed_settlement_event_amount_fails_source_assertion_consistency() {
        let (mut bundle, _, _, _, _) = fixtures();
        let c = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("RBC-002-FEE-NETTING")).unwrap();
        c["events"].as_array_mut().unwrap()
            .iter_mut().find(|event| optional_string(event, "event_ref") == Some("EV-002-S")).unwrap()["amount_minor"] = json!(48000);
        assert!(validate_event_assertion_consistency(c).is_err());
    }

    #[test]
    fn fee_arithmetic_mismatch_is_rejected() {
        let (mut bundle, _, _, _, _) = fixtures();
        let c = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("RBC-002-FEE-NETTING")).unwrap();
        c["assertions"].as_array_mut().unwrap()
            .iter_mut().find(|fact| optional_string(fact, "assertion_ref") == Some("AS-002-S-FEE")).unwrap()["value"] = json!(1000);
        assert!(validate_event_assertion_consistency(c).is_err());
    }

    #[test]
    fn duplicate_exact_source_identity_is_rejected() {
        let (mut bundle, _, _, _, _) = fixtures();
        let c = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("RBC-006-DUPLICATE-SETTLEMENT-FEED")).unwrap();
        c["sources"][2]["revision"] = json!("r1");
        assert!(validate_source_and_assertion_references(c).is_err());
    }

    #[test]
    fn expected_link_cannot_use_a_source_from_another_tenant_case() {
        let (bundle, oracle, _, _, _) = fixtures();
        let c = case(&bundle, "RBC-001-EXACT-THREE-WAY");
        let mut e = expectation(&oracle, "RBC-001-EXACT-THREE-WAY").clone();
        e["expected_links"][0]["source_refs"][0] = json!("SRC-999-NOT-IN-CASE");
        assert!(validate_oracle_case(c, &e).is_err());
    }
}
