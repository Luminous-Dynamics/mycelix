use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    error::Error,
    fs,
    io::Read,
    path::{Component, Path, PathBuf},
};

fn main() {
    if let Err(error) = run() {
        eprintln!("recbench-bridge-s0: {error}");
        std::process::exit(2);
    }
}

fn read_bytes(path: &Path) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(fs::read(path)?)
}

fn read_json(path: &Path) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&read_bytes(path)?)?)
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

fn integer(value: &Value, key: &str) -> Result<i64, String> {
    value
        .get(key)
        .and_then(Value::as_i64)
        .ok_or_else(|| format!("missing or invalid integer: {key}"))
}

fn nullable_integer(value: &Value, key: &str) -> Result<Option<i64>, String> {
    match value.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(value) => value
            .as_i64()
            .map(Some)
            .ok_or_else(|| format!("invalid integer or null: {key}")),
    }
}

fn absolute(value: i64) -> Result<i64, String> {
    value
        .checked_abs()
        .ok_or_else(|| "i64::MIN cannot be converted to a positive minor-unit amount".to_owned())
}

fn record_by_ref<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    array(case, "records")?
        .iter()
        .find(|record| optional_string(record, "record_ref") == Some(reference))
        .ok_or_else(|| format!("unknown record reference {reference}"))
}

fn case_by_id<'a>(bundle: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(bundle, "cases")?
        .iter()
        .find(|case| optional_string(case, "case_id") == Some(case_id))
        .ok_or_else(|| format!("unknown candidate case {case_id}"))
}

fn expectation_by_id<'a>(oracle: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(oracle, "expectations")?
        .iter()
        .find(|expectation| optional_string(expectation, "case_id") == Some(case_id))
        .ok_or_else(|| format!("unknown oracle expectation {case_id}"))
}

fn record_kind(record: &Value) -> Result<&'static str, String> {
    match string(record, "record_kind")? {
        "ledger_transaction" => Ok("ledger_transaction"),
        "settlement_line" => Ok("settlement_line"),
        "payout" => Ok("payout"),
        "bank_statement" => Ok("bank_statement"),
        other => Err(format!("unknown record kind {other}")),
    }
}

fn is_future(case: &Value, record: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    Ok(string(record, "received_at")? > frontier
        || string(record, "recorded_at")? > frontier)
}

fn validate_record_shape(case: &Value, record: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let record_ref = string(record, "record_ref")?;
    let kind = record_kind(record)?;
    let table = string(record, "source_table")?;
    let expected_table = match kind {
        "ledger_transaction" => "ledger",
        "settlement_line" => "settlement_lines",
        "payout" => "payouts",
        "bank_statement" => "bank_statements",
        _ => unreachable!(),
    };
    if table != expected_table {
        return Err(format!("{case_id}/{record_ref}: record kind {kind} cannot use table {table}"));
    }

    let amount = integer(record, "amount_minor")?;
    let direction = string(record, "direction")?;
    let expected_direction = match amount.cmp(&0) {
        std::cmp::Ordering::Greater => "credit",
        std::cmp::Ordering::Less => "debit",
        std::cmp::Ordering::Equal => "neutral",
    };
    if direction != expected_direction {
        return Err(format!(
            "{case_id}/{record_ref}: signed amount {amount} conflicts with direction {direction}"
        ));
    }

    let external_id = optional_string(record, "external_id");
    if external_id.is_none_or(str::is_empty) {
        return Err(format!("{case_id}/{record_ref}: source external_id must be non-empty"));
    }

    match kind {
        "payout" => {
            let gross = nullable_integer(record, "gross_amount_minor")?
                .ok_or_else(|| format!("{case_id}/{record_ref}: payout gross amount missing"))?;
            let fee = nullable_integer(record, "fee_amount_minor")?
                .ok_or_else(|| format!("{case_id}/{record_ref}: payout fee amount missing"))?;
            let net = nullable_integer(record, "net_amount_minor")?
                .ok_or_else(|| format!("{case_id}/{record_ref}: payout net amount missing"))?;
            if gross.checked_add(fee) != Some(net) || amount != net {
                return Err(format!(
                    "{case_id}/{record_ref}: gross {gross} + fee {fee} must equal net/amount {net}/{amount}"
                ));
            }
            if optional_string(record, "line_role").is_some() {
                return Err(format!("{case_id}/{record_ref}: payout cannot carry a settlement line role"));
            }
        }
        "settlement_line" => {
            if optional_string(record, "line_role").is_none() {
                return Err(format!("{case_id}/{record_ref}: settlement line requires line_role"));
            }
            for field in ["gross_amount_minor", "fee_amount_minor", "net_amount_minor"] {
                if nullable_integer(record, field)?.is_some() {
                    return Err(format!("{case_id}/{record_ref}: settlement line cannot carry payout field {field}"));
                }
            }
        }
        _ => {
            if optional_string(record, "line_role").is_some() {
                return Err(format!("{case_id}/{record_ref}: non-settlement record cannot carry line_role"));
            }
            for field in ["gross_amount_minor", "fee_amount_minor", "net_amount_minor"] {
                if nullable_integer(record, field)?.is_some() {
                    return Err(format!("{case_id}/{record_ref}: non-payout record cannot carry {field}"));
                }
            }
        }
    }
    Ok(())
}

fn duplicate_signature(record: &Value) -> Result<Value, String> {
    let mut signature = record.clone();
    let object = signature
        .as_object_mut()
        .ok_or("record must be an object for duplicate comparison")?;
    object.remove("record_ref");
    object.remove("source_row_id");
    Ok(signature)
}

fn detect_duplicate_feed_rows(case: &Value) -> Result<Vec<(String, String)>, String> {
    let mut groups = BTreeMap::<(String, String, String, String), Vec<&Value>>::new();
    for record in array(case, "records")? {
        let key = (
            string(record, "tenant_namespace")?.to_owned(),
            string(record, "source_table")?.to_owned(),
            string(record, "external_id")?.to_owned(),
            string(record, "revision")?.to_owned(),
        );
        groups.entry(key).or_default().push(record);
    }
    let mut duplicates = Vec::new();
    for (identity, mut group) in groups {
        if group.len() <= 1 {
            continue;
        }
        group.sort_by(|left, right| {
            optional_string(left, "source_row_id").cmp(&optional_string(right, "source_row_id"))
        });
        let canonical = group[0];
        let canonical_signature = duplicate_signature(canonical)?;
        for duplicate in group.iter().skip(1) {
            if duplicate_signature(duplicate)? != canonical_signature {
                return Err(format!(
                    "{}: conflicting rows share source identity {}/{}/{}/{}",
                    string(case, "case_id")?,
                    identity.0,
                    identity.1,
                    identity.2,
                    identity.3
                ));
            }
            duplicates.push((
                string(duplicate, "record_ref")?.to_owned(),
                string(canonical, "record_ref")?.to_owned(),
            ));
        }
    }
    Ok(duplicates)
}

fn validate_fx_quote(case: &Value, quote: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let quote_ref = string(quote, "fx_quote_ref")?;
    let numerator = integer(quote, "rate_numerator")?;
    let denominator = integer(quote, "rate_denominator")?;
    if numerator <= 0 || denominator <= 0 {
        return Err(format!("{case_id}/{quote_ref}: FX rational must be positive"));
    }
    let source_amount = integer(quote, "source_amount_minor")?;
    let product = (source_amount as i128)
        .checked_mul(numerator as i128)
        .ok_or_else(|| format!("{case_id}/{quote_ref}: FX multiplication overflow"))?;
    let denominator = denominator as i128;
    let quotient = product / denominator;
    let remainder = product % denominator;
    let converted = match string(quote, "rounding_mode")? {
        "exact" if remainder == 0 => quotient,
        "exact" => {
            return Err(format!("{case_id}/{quote_ref}: exact FX quote produces a fractional minor unit"));
        }
        "floor" => quotient,
        "ceiling" => quotient + i128::from(remainder != 0),
        "half_up" => quotient + i128::from(remainder * 2 >= denominator),
        "half_even" => {
            if remainder * 2 > denominator {
                quotient + 1
            } else if remainder * 2 < denominator {
                quotient
            } else if quotient % 2 == 0 {
                quotient
            } else {
                quotient + 1
            }
        }
        mode => return Err(format!("{case_id}/{quote_ref}: unsupported rounding mode {mode}")),
    };
    let converted = i64::try_from(converted)
        .map_err(|_| format!("{case_id}/{quote_ref}: converted amount overflows i64"))?;
    if converted != integer(quote, "target_amount_minor")? {
        return Err(format!("{case_id}/{quote_ref}: FX conversion result does not match target amount"));
    }
    if string(quote, "base_currency")? == string(quote, "quote_currency")? {
        return Err(format!("{case_id}/{quote_ref}: FX currencies must differ"));
    }
    for reference in array(quote, "source_record_refs")? {
        let reference = reference.as_str().ok_or("FX source record ref must be a string")?;
        record_by_ref(case, reference)?;
    }
    Ok(())
}

fn validate_ledger_settlement_match(
    case: &Value,
    match_item: &Value,
) -> Result<usize, String> {
    let case_id = string(case, "case_id")?;
    let match_ref = string(match_item, "match_ref")?;
    let ledger_refs = array(match_item, "ledger_record_refs")?;
    let settlement_refs = array(match_item, "settlement_record_refs")?;
    if ledger_refs.is_empty() || settlement_refs.is_empty() {
        return Err(format!("{case_id}/{match_ref}: ledger-to-settlement match needs both sides"));
    }
    if !array(match_item, "payout_record_refs")?.is_empty()
        || !array(match_item, "bank_record_refs")?.is_empty()
    {
        return Err(format!("{case_id}/{match_ref}: ledger-to-settlement match has unrelated record kinds"));
    }

    let fx_quote_ref = optional_string(match_item, "fx_quote_ref");
    let fx_quote = if let Some(reference) = fx_quote_ref {
        Some(
            array(case, "fx_quotes")?
                .iter()
                .find(|quote| optional_string(quote, "fx_quote_ref") == Some(reference))
                .ok_or_else(|| format!("{case_id}/{match_ref}: missing FX quote {reference}"))?,
        )
    } else {
        None
    };

    let mut allocation_total = 0i128;
    let mut ledger_allocations = BTreeMap::<String, i128>::new();
    let mut settlement_allocations = BTreeMap::<String, i128>::new();
    let mut allocation_count = 0usize;

    for allocation in array(match_item, "allocations")? {
        let from_ref = string(allocation, "from_record_ref")?;
        let to_ref = string(allocation, "to_record_ref")?;
        let from = record_by_ref(case, from_ref)?;
        let to = record_by_ref(case, to_ref)?;
        if record_kind(from)? != "ledger_transaction"
            || record_kind(to)? != "settlement_line"
        {
            return Err(format!("{case_id}/{match_ref}: allocation has wrong record kinds"));
        }
        if is_future(case, from)? || is_future(case, to)? {
            return Err(format!("{case_id}/{match_ref}: future source record cannot be automatically matched"));
        }
        if string(from, "tenant_namespace")? != string(to, "tenant_namespace")? {
            return Err(format!("{case_id}/{match_ref}: cross-tenant match is prohibited"));
        }
        let amount = integer(allocation, "amount_minor")?;
        if amount <= 0 {
            return Err(format!("{case_id}/{match_ref}: allocation amount must be positive"));
        }
        if string(allocation, "currency")? != string(to, "currency")? {
            return Err(format!("{case_id}/{match_ref}: allocation currency must be settlement currency"));
        }
        if let Some(quote) = fx_quote {
            if string(from, "currency")? != string(quote, "base_currency")?
                || string(to, "currency")? != string(quote, "quote_currency")?
                || absolute(integer(from, "amount_minor")?)? != integer(quote, "source_amount_minor")?
                || absolute(integer(to, "amount_minor")?)? != integer(quote, "target_amount_minor")?
            {
                return Err(format!("{case_id}/{match_ref}: FX quote does not bind to source and settlement records"));
            }
            if amount != integer(quote, "target_amount_minor")? {
                return Err(format!("{case_id}/{match_ref}: FX allocation does not equal converted target amount"));
            }
        } else {
            if string(from, "currency")? != string(to, "currency")?
                || string(from, "currency")? != string(match_item, "currency")?
            {
                return Err(format!("{case_id}/{match_ref}: cross-currency match requires an explicit FX quote"));
            }
            if amount > absolute(integer(from, "amount_minor")?)?
                || amount > absolute(integer(to, "amount_minor")?)?
            {
                return Err(format!("{case_id}/{match_ref}: allocation exceeds a source or settlement amount"));
            }
        }
        *ledger_allocations.entry(from_ref.to_owned()).or_default() += amount as i128;
        *settlement_allocations.entry(to_ref.to_owned()).or_default() += amount as i128;
        allocation_total += amount as i128;
        allocation_count += 1;
    }

    if allocation_count == 0 {
        return Err(format!("{case_id}/{match_ref}: ledger match has no explicit allocations"));
    }
    for reference in ledger_refs {
        let reference = reference.as_str().ok_or("ledger record ref must be string")?;
        let record = record_by_ref(case, reference)?;
        if record_kind(record)? != "ledger_transaction" {
            return Err(format!("{case_id}/{match_ref}: ledger side contains non-ledger record"));
        }
        if let Some(quote) = fx_quote {
            if ledger_refs.len() != 1 || settlement_refs.len() != 1 {
                return Err(format!("{case_id}/{match_ref}: S0 FX match must be one-to-one"));
            }
            if absolute(integer(record, "amount_minor")?)? != integer(quote, "source_amount_minor")? {
                return Err(format!("{case_id}/{match_ref}: FX source amount mismatch"));
            }
            let target_ref = settlement_refs[0].as_str().ok_or("settlement ref must be string")?;
            if ledger_allocations.get(reference).copied().unwrap_or(0)
                != integer(quote, "target_amount_minor")? as i128
                || settlement_allocations.get(target_ref).copied().unwrap_or(0)
                    != integer(quote, "target_amount_minor")? as i128
            {
                return Err(format!("{case_id}/{match_ref}: FX allocation does not fully consume source/target"));
            }
        } else if ledger_allocations.get(reference).copied().unwrap_or(0)
            != absolute(integer(record, "amount_minor")?)? as i128
        {
            return Err(format!("{case_id}/{match_ref}: ledger allocation does not equal full record amount"));
        }
    }
    for reference in settlement_refs {
        let reference = reference.as_str().ok_or("settlement ref must be string")?;
        let record = record_by_ref(case, reference)?;
        if record_kind(record)? != "settlement_line" {
            return Err(format!("{case_id}/{match_ref}: settlement side contains non-settlement record"));
        }
        if settlement_allocations.get(reference).copied().unwrap_or(0)
            != absolute(integer(record, "amount_minor")?)? as i128
        {
            return Err(format!("{case_id}/{match_ref}: settlement line is not allocated exactly once"));
        }
    }
    if allocation_total != integer(match_item, "matched_amount_minor")? as i128 {
        return Err(format!("{case_id}/{match_ref}: matched amount does not equal allocation sum"));
    }
    if string(match_item, "currency")? != string(settlement_refs.first().and_then(Value::as_str).map(|r| record_by_ref(case, r).ok()).flatten().ok_or("missing settlement record")?, "currency")? {
        return Err(format!("{case_id}/{match_ref}: match currency must equal settlement currency"));
    }
    Ok(allocation_count)
}

fn validate_payout_bank_match(case: &Value, match_item: &Value) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let match_ref = string(match_item, "match_ref")?;
    let payout_refs = array(match_item, "payout_record_refs")?;
    let bank_refs = array(match_item, "bank_record_refs")?;
    let subset = string(match_item, "match_kind")? == "payout_subset_to_bank";
    if payout_refs.is_empty() || bank_refs.len() != 1 {
        return Err(format!("{case_id}/{match_ref}: payout-to-bank match needs payouts and exactly one bank line"));
    }
    if (subset && payout_refs.len() < 2) || (!subset && payout_refs.len() != 1) {
        return Err(format!("{case_id}/{match_ref}: payout match cardinality conflicts with match_kind"));
    }
    if !array(match_item, "ledger_record_refs")?.is_empty()
        || !array(match_item, "settlement_record_refs")?.is_empty()
    {
        return Err(format!("{case_id}/{match_ref}: payout-to-bank match contains unrelated record kinds"));
    }
    let bank = record_by_ref(case, bank_refs[0].as_str().ok_or("bank ref must be string")?)?;
    if record_kind(bank)? != "bank_statement" || is_future(case, bank)? {
        return Err(format!("{case_id}/{match_ref}: bank record is invalid or future"));
    }
    let mut total = 0i128;
    let mut namespace: Option<&str> = None;
    for reference in payout_refs {
        let reference = reference.as_str().ok_or("payout ref must be string")?;
        let payout = record_by_ref(case, reference)?;
        if record_kind(payout)? != "payout" || is_future(case, payout)? {
            return Err(format!("{case_id}/{match_ref}: payout record is invalid or future"));
        }
        if string(payout, "currency")? != string(bank, "currency")? {
            return Err(format!("{case_id}/{match_ref}: payout and bank currencies differ"));
        }
        if string(payout, "tenant_namespace")? != string(bank, "tenant_namespace")? {
            return Err(format!("{case_id}/{match_ref}: cross-tenant payout/bank match is prohibited"));
        }
        if namespace.is_some_and(|existing| existing != string(payout, "tenant_namespace").unwrap_or("")) {
            return Err(format!("{case_id}/{match_ref}: payout group crosses tenant namespaces"));
        }
        namespace = Some(string(payout, "tenant_namespace")?);
        total += integer(payout, "amount_minor")? as i128;
    }
    if total != integer(bank, "amount_minor")? as i128
        || total != integer(match_item, "matched_amount_minor")? as i128
        || string(match_item, "currency")? != string(bank, "currency")?
    {
        return Err(format!("{case_id}/{match_ref}: payout group does not exactly reconcile to bank amount"));
    }
    Ok(())
}

fn accepted_match_refs(expectation: &Value) -> Result<BTreeSet<String>, String> {
    let mut refs = BTreeSet::new();
    for item in array(expectation, "matches")? {
        for field in [
            "ledger_record_refs",
            "settlement_record_refs",
            "payout_record_refs",
            "bank_record_refs",
        ] {
            for reference in array(item, field)? {
                refs.insert(
                    reference
                        .as_str()
                        .ok_or("match record ref must be string")?
                        .to_owned(),
                );
            }
        }
    }
    Ok(refs)
}

fn candidate_ambiguous_pairs(case: &Value) -> Result<Vec<(String, String)>, String> {
    let ledgers = array(case, "records")?
        .iter()
        .filter(|record| {
            optional_string(record, "record_kind") == Some("ledger_transaction")
                && !is_future(case, record).unwrap_or(true)
                && value_is_null(record, "source_reference")
        })
        .collect::<Vec<_>>();
    let settlements = array(case, "records")?
        .iter()
        .filter(|record| {
            optional_string(record, "record_kind") == Some("settlement_line")
                && !is_future(case, record).unwrap_or(true)
                && value_is_null(record, "source_reference")
        })
        .collect::<Vec<_>>();
    let mut pairs = Vec::new();
    for ledger in ledgers {
        for settlement in &settlements {
            if integer(ledger, "amount_minor")? == integer(settlement, "amount_minor")?
                && optional_string(ledger, "currency") == optional_string(settlement, "currency")
                && optional_string(ledger, "value_date") == optional_string(settlement, "value_date")
                && optional_string(ledger, "counterparty") == optional_string(settlement, "counterparty")
                && optional_string(ledger, "tenant_namespace") == optional_string(settlement, "tenant_namespace")
            {
                pairs.push((
                    string(ledger, "record_ref")?.to_owned(),
                    string(settlement, "record_ref")?.to_owned(),
                ));
            }
        }
    }
    Ok(pairs)
}

fn validate_duplicates(case: &Value, expectation: &Value) -> Result<usize, String> {
    let actual = detect_duplicate_feed_rows(case)?;
    let expected = array(expectation, "duplicates")?;
    if actual.len() != expected.len() {
        return Err(format!(
            "{}: duplicate oracle count {} differs from detected count {}",
            string(case, "case_id")?,
            expected.len(),
            actual.len()
        ));
    }
    let mut expected_pairs = BTreeSet::new();
    for duplicate in expected {
        let pair = (
            string(duplicate, "duplicate_record_ref")?.to_owned(),
            string(duplicate, "canonical_record_ref")?.to_owned(),
        );
        if !expected_pairs.insert(pair.clone()) {
            return Err(format!("{}: duplicate oracle repeats a pair", string(case, "case_id")?));
        }
        let duplicate_record = record_by_ref(case, &pair.0)?;
        let canonical_record = record_by_ref(case, &pair.1)?;
        if duplicate_signature(duplicate_record)? != duplicate_signature(canonical_record)? {
            return Err(format!("{}: duplicate oracle pair is not byte-semantically equivalent", string(case, "case_id")?));
        }
    }
    let actual_set = actual.into_iter().collect::<BTreeSet<_>>();
    if actual_set != expected_pairs {
        return Err(format!("{}: expected duplicate set differs from detected duplicate set", string(case, "case_id")?));
    }
    Ok(expected.len())
}

fn validate_reversals(case: &Value, expectation: &Value) -> Result<usize, String> {
    for reversal in array(expectation, "reversals")? {
        let reversal_ref = string(reversal, "reversal_record_ref")?;
        let original_ref = string(reversal, "original_record_ref")?;
        let reversal_record = record_by_ref(case, reversal_ref)?;
        let original_record = record_by_ref(case, original_ref)?;
        if record_kind(reversal_record)? != record_kind(original_record)? {
            return Err(format!("{}: reversal and original record kinds differ", string(case, "case_id")?));
        }
        if optional_string(reversal_record, "original_external_id")
            != optional_string(original_record, "external_id")
        {
            return Err(format!("{}: explicit reversal does not identify its original external ID", string(case, "case_id")?));
        }
        if string(reversal_record, "tenant_namespace")? != string(original_record, "tenant_namespace")?
            || string(reversal_record, "currency")? != string(original_record, "currency")?
        {
            return Err(format!("{}: reversal crosses tenant or currency scope", string(case, "case_id")?));
        }
        if integer(reversal_record, "amount_minor")? != -integer(original_record, "amount_minor")? {
            return Err(format!("{}: reversal amount must be the exact opposite of original", string(case, "case_id")?));
        }
    }
    Ok(array(expectation, "reversals")?.len())
}

fn value_is_null(record: &Value, field: &str) -> bool {
    record.get(field).is_none_or(Value::is_null)
}

fn validate_exceptions(
    case: &Value,
    expectation: &Value,
    matched_refs: &BTreeSet<String>,
) -> Result<(usize, usize, usize), String> {
    let mut future_count = 0usize;
    let mut ambiguous_count = 0usize;
    for record in array(case, "records")? {
        if is_future(case, record)? {
            future_count += 1;
        }
    }
    let possible_ambiguous_pairs = candidate_ambiguous_pairs(case)?;
    for exception in array(expectation, "exceptions")? {
        let case_id = string(case, "case_id")?;
        let reason = string(exception, "reason_code")?;
        let references = array(exception, "record_refs")?
            .iter()
            .map(|reference| {
                let reference = reference.as_str().ok_or("exception record ref must be string")?;
                record_by_ref(case, reference)?;
                Ok::<String, String>(reference.to_owned())
            })
            .collect::<Result<Vec<_>, _>>()?;
        if references.iter().any(|reference| matched_refs.contains(reference)) {
            return Err(format!("{case_id}: exception record was also accepted in a match"));
        }
        match reason {
            "unmatched_bank_movement" => {
                if references.len() != 1 {
                    return Err(format!("{case_id}: unmatched bank exception must have one record"));
                }
                let record = record_by_ref(case, &references[0])?;
                if record_kind(record)? != "bank_statement" || is_future(case, record)? {
                    return Err(format!("{case_id}: unmatched-bank exception must be an eligible bank record"));
                }
                if nullable_integer(exception, "expected_residual_minor")?
                    != Some(absolute(integer(record, "amount_minor")?)?)
                {
                    return Err(format!("{case_id}: unmatched bank residual differs from signed record amount"));
                }
            }
            "future_evidence_not_available" => {
                if references.is_empty()
                    || references.iter().any(|reference| {
                        record_by_ref(case, reference)
                            .ok()
                            .is_none_or(|record| !is_future(case, record).unwrap_or(false))
                    })
                {
                    return Err(format!("{case_id}: future-evidence exception contains a non-future record"));
                }
                if string(exception, "action")? != "exclude_future_evidence" {
                    return Err(format!("{case_id}: future evidence must be excluded, not matched/review-closed"));
                }
            }
            "ambiguous_match" => {
                ambiguous_count += 1;
                if references.len() < 4 {
                    return Err(format!("{case_id}: ambiguous component needs at least two records on both sides"));
                }
                if possible_ambiguous_pairs.len() < 4 {
                    return Err(format!("{case_id}: candidate facts do not prove a 2x2 ambiguous assignment"));
                }
                let reference_set = references.iter().cloned().collect::<BTreeSet<_>>();
                if possible_ambiguous_pairs.iter().any(|(left, right)| {
                    !reference_set.contains(left) || !reference_set.contains(right)
                }) {
                    return Err(format!("{case_id}: ambiguous exception omits one of the candidate pair nodes"));
                }
                if string(exception, "action")? != "route_to_review" {
                    return Err(format!("{case_id}: provably ambiguous component must route to review"));
                }
            }
            "unmatched_settlement_movement" => {
                if references.is_empty()
                    || references.iter().any(|reference| {
                        record_by_ref(case, reference)
                            .ok()
                            .is_none_or(|record| optional_string(record, "record_kind") != Some("settlement_line"))
                    })
                {
                    return Err(format!("{case_id}: unmatched settlement exception contains non-settlement record"));
                }
            }
            other => return Err(format!("{case_id}: unknown exception reason {other}")),
        }
    }
    Ok((array(expectation, "exceptions")?.len(), future_count, ambiguous_count))
}

fn validate_case(case: &Value, expectation: &Value) -> Result<Value, String> {
    let case_id = string(case, "case_id")?;
    if string(expectation, "case_id")? != case_id {
        return Err(format!("{case_id}: candidate and oracle IDs differ"));
    }
    let mut record_refs = BTreeSet::new();
    for record in array(case, "records")? {
        let reference = string(record, "record_ref")?;
        if !record_refs.insert(reference.to_owned()) {
            return Err(format!("{case_id}: duplicate record_ref {reference}"));
        }
        validate_record_shape(case, record)?;
    }
    let mut fx_refs = BTreeSet::new();
    for quote in array(case, "fx_quotes")? {
        let reference = string(quote, "fx_quote_ref")?;
        if !fx_refs.insert(reference.to_owned()) {
            return Err(format!("{case_id}: duplicate FX quote ref {reference}"));
        }
        validate_fx_quote(case, quote)?;
    }

    let duplicate_count = validate_duplicates(case, expectation)?;
    let duplicate_refs = array(expectation, "duplicates")?
        .iter()
        .map(|item| string(item, "duplicate_record_ref").map(ToOwned::to_owned))
        .collect::<Result<BTreeSet<_>, _>>()?;
    let mut match_refs = BTreeSet::new();
    let mut matched_refs = BTreeSet::new();
    let mut pair_count = 0usize;
    let mut group_count = 0usize;
    let mut allocation_count = 0usize;

    for match_item in array(expectation, "matches")? {
        let match_ref = string(match_item, "match_ref")?;
        if !match_refs.insert(match_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate match ref {match_ref}"));
        }
        for field in [
            "ledger_record_refs",
            "settlement_record_refs",
            "payout_record_refs",
            "bank_record_refs",
        ] {
            for reference in array(match_item, field)? {
                let reference = reference.as_str().ok_or("match record ref must be string")?;
                record_by_ref(case, reference)?;
                if duplicate_refs.contains(reference) {
                    return Err(format!("{case_id}/{match_ref}: duplicate feed row cannot be accepted as a canonical match"));
                }
                if !matched_refs.insert(reference.to_owned()) {
                    return Err(format!("{case_id}: record {reference} is consumed by multiple matches"));
                }
            }
        }
        match string(match_item, "match_kind")? {
            "ledger_to_settlement" => {
                allocation_count += validate_ledger_settlement_match(case, match_item)?;
                pair_count += 1;
            }
            "payout_to_bank" | "payout_subset_to_bank" => {
                validate_payout_bank_match(case, match_item)?;
                group_count += 1;
            }
            other => return Err(format!("{case_id}/{match_ref}: unsupported match kind {other}")),
        }
    }

    let reversal_count = validate_reversals(case, expectation)?;
    let (exception_count, future_count, ambiguous_count) =
        validate_exceptions(case, expectation, &matched_refs)?;

    // Every duplicate row must be excluded from accepted match membership.
    for reference in &duplicate_refs {
        if matched_refs.contains(reference) {
            return Err(format!("{case_id}: duplicate source row {reference} was counted in a match"));
        }
    }

    // Settlement rows grouped to a payout must sum to the payout's net.
    let mut payout_lines = BTreeMap::<String, i128>::new();
    for record in array(case, "records")? {
        if optional_string(record, "record_kind") == Some("settlement_line") && !is_future(case, record)? {
            if let Some(payout_external_id) = optional_string(record, "payout_external_id") {
                *payout_lines.entry(payout_external_id.to_owned()).or_default() +=
                    integer(record, "amount_minor")? as i128;
            }
        }
    }
    for payout in array(case, "records")?
        .iter()
        .filter(|record| optional_string(record, "record_kind") == Some("payout"))
    {
        let payout_id = string(payout, "external_id")?;
        if let Some(lines_total) = payout_lines.get(payout_id) {
            if *lines_total != integer(payout, "amount_minor")? as i128 {
                return Err(format!("{case_id}: eligible settlement lines do not equal payout net for {payout_id}"));
            }
        }
    }

    let checks = vec![
        "record_identity_and_signs",
        "payout_gross_fee_net",
        "ledger_settlement_allocations",
        "fx_exact_conversion",
        "payout_bank_arithmetic",
        "duplicate_feed_canonicalization",
        "reversal_lineage",
        "future_evidence_exclusion",
        "ambiguous_component_not_auto_matched",
        "oracle_reference_integrity",
    ];
    Ok(json!({
        "case_id": case_id,
        "record_count": array(case, "records")?.len(),
        "pair_match_count": pair_count,
        "group_match_count": group_count,
        "allocation_count": allocation_count,
        "duplicate_count": duplicate_count,
        "reversal_count": reversal_count,
        "exception_count": exception_count,
        "future_record_count": future_count,
        "ambiguous_component_count": ambiguous_count,
        "checks": checks
    }))
}

fn validate_dataset_binding(
    candidate: &Value,
    oracle: &Value,
    snapshot: &Value,
) -> Result<(), String> {
    let candidate_binding = candidate.get("dataset_binding").ok_or("candidate dataset binding missing")?;
    let oracle_binding = oracle.get("dataset_binding").ok_or("oracle dataset binding missing")?;
    let expected = json!({
        "snapshot_id": string(snapshot, "snapshot_id")?,
        "tier": string(&snapshot["manifest"], "tier")?,
        "generator_version": string(&snapshot["manifest"], "generator_version")?,
        "seed": integer(&snapshot["manifest"], "seed")?,
        "config_sha256": string(&snapshot["manifest"], "config_sha256")?,
        "oracle_files_excluded": true
    });
    if candidate_binding != &expected || oracle_binding != &expected {
        return Err("candidate/oracle dataset binding does not equal pinned RecBench small-tier snapshot".to_owned());
    }
    Ok(())
}

fn build_report(
    candidate: &Value,
    oracle: &Value,
    source_snapshot: &Value,
    candidate_digest: String,
    oracle_digest: String,
    snapshot_digest: String,
    verification_status: &str,
    verified_file_count: usize,
    oracle_only_opened: bool,
) -> Result<Value, String> {
    validate_dataset_binding(candidate, oracle, source_snapshot)?;
    let candidate_cases = array(candidate, "cases")?;
    let expectations = array(oracle, "expectations")?;
    if candidate_cases.len() != 10 || expectations.len() != 10 {
        return Err("S0 bridge requires exactly ten candidate cases and ten oracle rows".to_owned());
    }
    let mut candidate_ids = BTreeSet::new();
    let mut oracle_ids = BTreeSet::new();
    let mut case_results = Vec::new();
    let mut totals = BTreeMap::<String, usize>::new();

    for case in candidate_cases {
        let case_id = string(case, "case_id")?;
        if !candidate_ids.insert(case_id.to_owned()) {
            return Err(format!("duplicate candidate case ID {case_id}"));
        }
        let expectation = expectation_by_id(oracle, case_id)?;
        oracle_ids.insert(string(expectation, "case_id")?.to_owned());
        let result = validate_case(case, expectation)?;
        for field in [
            "record_count",
            "pair_match_count",
            "group_match_count",
            "allocation_count",
            "duplicate_count",
            "reversal_count",
            "exception_count",
            "future_record_count",
            "ambiguous_component_count",
        ] {
            *totals.entry(field.to_owned()).or_default() +=
                result[field].as_u64().unwrap_or(0) as usize;
        }
        case_results.push(result);
    }
    if candidate_ids != oracle_ids {
        return Err("candidate and oracle case ID sets differ".to_owned());
    }

    Ok(json!({
        "report_id": "MYCELIX-RECBENCH-BRIDGE-S0-VALIDATION",
        "schema_version": "1.0.0",
        "claim_ceiling": "synthetic_bridge_oracle_validation_not_recbench_agent_performance",
        "candidate_sha256": candidate_digest,
        "oracle_sha256": oracle_digest,
        "source_snapshot_sha256": snapshot_digest,
        "case_count": case_results.len(),
        "dataset_byte_verification": {
            "status": verification_status,
            "verified_file_count": verified_file_count,
            "oracle_only_files_opened": oracle_only_opened
        },
        "totals": totals,
        "case_results": case_results
    }))
}

fn hash_file(path: &Path) -> Result<String, Box<dyn Error>> {
    let mut file = fs::File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; 1024 * 1024];
    loop {
        let read = file.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
    }
    Ok(hex::encode(hasher.finalize()))
}

fn safe_relative_path(raw: &str) -> Result<PathBuf, String> {
    let path = Path::new(raw);
    if path.is_absolute()
        || path.components().any(|component| {
            !matches!(component, Component::Normal(_))
        })
    {
        return Err(format!("unsafe dataset relative path {raw}"));
    }
    Ok(path.to_path_buf())
}

fn verify_recbench_dataset(
    dataset_dir: &Path,
    snapshot: &Value,
    verify_all_files: bool,
) -> Result<(String, usize, bool), Box<dyn Error>> {
    let manifest_path = dataset_dir.join("manifest.json");
    let actual: Value = read_json(&manifest_path)?;
    let expected = snapshot.get("manifest").ok_or("pinned RecBench manifest missing")?;
    for field in [
        "benchmark",
        "generator_version",
        "tier",
        "seed",
        "config_sha256",
        "start_date",
        "ledger_days",
        "row_counts",
    ] {
        if actual.get(field) != expected.get(field) {
            return Err(format!("RecBench manifest field {field} differs from pinned snapshot").into());
        }
    }
    let actual_files = array(&actual, "files")?;
    let expected_files = array(expected, "files")?;
    if actual_files.len() != expected_files.len() {
        return Err("RecBench manifest file count differs from pinned snapshot".into());
    }
    let input_paths = array(snapshot, "agent_input_paths")?
        .iter()
        .map(|value| value.as_str().ok_or("agent_input_paths must contain strings"))
        .collect::<Result<BTreeSet<_>, _>>()?;
    let mut verified = 0usize;
    for expected_file in expected_files {
        let relative = string(expected_file, "path")?;
        let actual_file = actual_files
            .iter()
            .find(|file| optional_string(file, "path") == Some(relative))
            .ok_or_else(|| format!("RecBench manifest is missing file {relative}"))?;
        if actual_file.get("rows") != expected_file.get("rows")
            || actual_file.get("sha256") != expected_file.get("sha256")
        {
            return Err(format!("RecBench file manifest commitment differs for {relative}").into());
        }
        let is_input = input_paths.contains(relative);
        if !is_input && !verify_all_files {
            continue;
        }
        let relative_path = safe_relative_path(relative)?;
        let full_path = dataset_dir.join(relative_path);
        let actual_digest = hash_file(&full_path)?;
        if actual_digest != string(expected_file, "sha256")? {
            return Err(format!("RecBench file SHA-256 mismatch for {relative}").into());
        }
        verified += 1;
    }
    let status = if verify_all_files {
        "all_files_verified"
    } else {
        "agent_inputs_verified"
    };
    Ok((status.to_owned(), verified, verify_all_files))
}

fn run() -> Result<(), Box<dyn Error>> {
    let data_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../recbench-extension-v1");
    let mut dataset_dir: Option<PathBuf> = None;
    let mut output_path: Option<PathBuf> = None;
    let mut verify_all_files = false;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--dataset-dir" => {
                dataset_dir = Some(PathBuf::from(args.next().ok_or("--dataset-dir needs a path")?));
            }
            "--verify-all-files" => verify_all_files = true,
            "--output" => output_path = Some(PathBuf::from(args.next().ok_or("--output needs a path")?)),
            "--help" | "-h" => {
                println!(
                    "Validates the synthetic RecBench bridge and optionally verifies an external RecBench small tier.\n\n\
                     --dataset-dir PATH   Path to the RecBench small/ directory containing manifest.json\n\
                     --verify-all-files   Also hash oracle-only truth/label files (default verifies only four agent input tables)\n\
                     --output PATH        Write validation report JSON to PATH instead of stdout\n\
                     --help               Show usage\n\n\
                     This S0 tool does not parse Parquet or execute a RecBench matcher. It pins/verifies the external inputs and validates Mycelix bridge controls."
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument {other}; use --help").into()),
        }
    }
    if verify_all_files && dataset_dir.is_none() {
        return Err("--verify-all-files requires --dataset-dir".into());
    }

    let source_schema: Value = read_json(&data_dir.join("recbench_source_evidence_v1.0.0.schema.json"))?;
    let candidate_schema: Value = read_json(&data_dir.join("recbench_bridge_candidates_v1.0.0.schema.json"))?;
    let oracle_schema: Value = read_json(&data_dir.join("recbench_bridge_oracle_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(&data_dir.join("recbench_bridge_validation_report_v1.0.0.schema.json"))?;
    let source_path = data_dir.join("fixtures/recbench_source_snapshot_v1.0.0.json");
    let candidate_path = data_dir.join("fixtures/recbench_bridge_candidates_s0_v1.0.0.json");
    let oracle_path = data_dir.join("fixtures/recbench_bridge_oracle_s0_v1.0.0.json");
    let source_bytes = read_bytes(&source_path)?;
    let candidate_bytes = read_bytes(&candidate_path)?;
    let oracle_bytes = read_bytes(&oracle_path)?;
    let source_snapshot: Value = serde_json::from_slice(&source_bytes)?;
    let candidate: Value = serde_json::from_slice(&candidate_bytes)?;
    let oracle: Value = serde_json::from_slice(&oracle_bytes)?;
    validate_instance("RecBench source evidence", &source_schema, &source_snapshot)?;
    validate_instance("RecBench bridge candidates", &candidate_schema, &candidate)?;
    validate_instance("RecBench bridge oracle", &oracle_schema, &oracle)?;

    let (verification_status, verified_count, oracle_only_opened) =
        if let Some(path) = dataset_dir.as_deref() {
            let (status, count, opened) = verify_recbench_dataset(path, &source_snapshot, verify_all_files)?;
            (status, count, opened)
        } else {
            ("not_requested".to_owned(), 0, false)
        };

    let report = build_report(
        &candidate,
        &oracle,
        &source_snapshot,
        hex::encode(Sha256::digest(&candidate_bytes)),
        hex::encode(Sha256::digest(&oracle_bytes)),
        hex::encode(Sha256::digest(&source_bytes)),
        &verification_status,
        verified_count,
        oracle_only_opened,
    )?;
    validate_instance("RecBench bridge validation report", &report_schema, &report)?;
    let bytes = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &bytes)?;
            println!("Wrote RecBench bridge validation report for {} cases.", report["case_count"]);
        }
        None => println!("{}", String::from_utf8(bytes)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../recbench-extension-v1");
        (
            read_json(&root.join("fixtures/recbench_source_snapshot_v1.0.0.json")).unwrap(),
            read_json(&root.join("fixtures/recbench_bridge_candidates_s0_v1.0.0.json")).unwrap(),
            read_json(&root.join("fixtures/recbench_bridge_oracle_s0_v1.0.0.json")).unwrap(),
        )
    }

    fn validate_all(snapshot: &Value, candidates: &Value, oracle: &Value) -> Result<Value, String> {
        build_report(
            candidates,
            oracle,
            snapshot,
            "a".repeat(64),
            "b".repeat(64),
            "c".repeat(64),
            "not_requested",
            0,
            false,
        )
    }

    fn case_mut<'a>(bundle: &'a mut Value, case_id: &str) -> &'a mut Value {
        bundle["cases"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|case| optional_string(case, "case_id") == Some(case_id))
            .unwrap()
    }

    #[test]
    fn all_ten_bridge_cases_validate() {
        let (snapshot, candidates, oracle) = fixtures();
        let report = validate_all(&snapshot, &candidates, &oracle).unwrap();
        assert_eq!(report["case_count"], 10);
        assert_eq!(report["totals"]["record_count"], 49);
        assert_eq!(report["totals"]["candidate_pair_match_count"], 14);
        assert_eq!(report["totals"]["group_match_count"], 5);
        assert_eq!(report["totals"]["allocation_count"], 14);
        assert_eq!(report["totals"]["duplicate_count"], 1);
        assert_eq!(report["totals"]["reversal_count"], 2);
        assert_eq!(report["totals"]["exception_count"], 3);
        assert_eq!(report["totals"]["future_record_count"], 1);
        assert_eq!(report["totals"]["ambiguous_component_count"], 1);
    }

    #[test]
    fn payout_gross_fee_net_must_reconcile() {
        let (snapshot, mut candidates, oracle) = fixtures();
        let case = case_mut(&mut candidates, "RB-002-GROSS-FEE-NET");
        let payout = case["records"].as_array_mut().unwrap()
            .iter_mut().find(|record| optional_string(record, "record_ref") == Some("P002")).unwrap();
        payout["net_amount_minor"] = Value::from(97000);
        payout["amount_minor"] = Value::from(97000);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn partial_capture_allocations_must_equal_the_full_ledger_and_settlement_amounts() {
        let (snapshot, candidates, mut oracle) = fixtures();
        let expectation = oracle["expectations"].as_array_mut().unwrap()
            .iter_mut().find(|row| optional_string(row, "case_id") == Some("RB-005-PARTIAL-CAPTURE")).unwrap();
        let allocation = expectation["matches"].as_array_mut().unwrap()[0]["allocations"].as_array_mut().unwrap();
        allocation[1]["amount_minor"] = Value::from(11000);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn cross_tenant_ledger_settlement_match_is_rejected() {
        let (snapshot, mut candidates, oracle) = fixtures();
        let case = case_mut(&mut candidates, "RB-001-EXACT-TRIPLE");
        let settlement = case["records"].as_array_mut().unwrap()
            .iter_mut().find(|record| optional_string(record, "record_ref") == Some("S001")).unwrap();
        settlement["tenant_namespace"] = Value::String("other-tenant/processor".to_owned());
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn fx_quote_must_reconcile_source_and_target_minor_units() {
        let (snapshot, mut candidates, oracle) = fixtures();
        let case = case_mut(&mut candidates, "RB-003-FX-SETTLEMENT");
        case["fx_quotes"][0]["target_amount_minor"] = Value::from(109999);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn duplicate_feed_row_is_not_counted_twice() {
        let (snapshot, candidates, mut oracle) = fixtures();
        let expectation = oracle["expectations"].as_array_mut().unwrap()
            .iter_mut().find(|row| optional_string(row, "case_id") == Some("RB-006-DUPLICATE-SETTLEMENT-FEED")).unwrap();
        expectation["matches"][0]["settlement_record_refs"] = json!(["S006", "S006-DUP"]);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn reversal_must_reference_an_original_of_equal_and_opposite_amount() {
        let (snapshot, mut candidates, oracle) = fixtures();
        let case = case_mut(&mut candidates, "RB-007-REVERSAL-LIFECYCLE");
        let reversal = case["records"].as_array_mut().unwrap()
            .iter_mut().find(|record| optional_string(record, "record_ref") == Some("L007-REV")).unwrap();
        reversal["original_external_id"] = Value::String("NOT-CHG-7001".to_owned());
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn future_evidence_cannot_be_used_to_close_an_earlier_bank_movement() {
        let (snapshot, mut candidates, mut oracle) = fixtures();
        let expectation = oracle["expectations"].as_array_mut().unwrap()
            .iter_mut().find(|row| optional_string(row, "case_id") == Some("RB-008-UNMATCHED-BANK-AND-FUTURE-SOURCE")).unwrap();
        expectation["matches"] = json!([{
            "match_ref":"M-FUTURE",
            "match_kind":"ledger_to_settlement",
            "ledger_record_refs":[],
            "settlement_record_refs":["S008-FUTURE"],
            "payout_record_refs":[],
            "bank_record_refs":["B008"],
            "currency":"USD",
            "matched_amount_minor":15900,
            "fx_quote_ref":null,
            "allocations":[]
        }]);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn ambiguous_component_cannot_be_auto_matched() {
        let (snapshot, candidates, mut oracle) = fixtures();
        let expectation = oracle["expectations"].as_array_mut().unwrap()
            .iter_mut().find(|row| optional_string(row, "case_id") == Some("RB-010-PROVABLY-AMBIGUOUS-PAIRING")).unwrap();
        expectation["matches"] = json!([{
            "match_ref":"M-AMBIGUOUS-UNSAFE",
            "match_kind":"ledger_to_settlement",
            "ledger_record_refs":["L010-A"],
            "settlement_record_refs":["S010-A"],
            "payout_record_refs":[],
            "bank_record_refs":[],
            "currency":"USD",
            "matched_amount_minor":25000,
            "fx_quote_ref":null,
            "allocations":[{"from_record_ref":"L010-A","to_record_ref":"S010-A","amount_minor":25000,"currency":"USD"}]
        }]);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn subset_sum_group_must_equal_bank_deposit_exactly() {
        let (snapshot, candidates, mut oracle) = fixtures();
        let expectation = oracle["expectations"].as_array_mut().unwrap()
            .iter_mut().find(|row| optional_string(row, "case_id") == Some("RB-009-DEPOSIT-SUBSET-SUM")).unwrap();
        expectation["matches"].as_array_mut().unwrap()[2]["matched_amount_minor"] = Value::from(99999);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn direction_must_agree_with_signed_minor_units() {
        let (snapshot, mut candidates, oracle) = fixtures();
        let case = case_mut(&mut candidates, "RB-002-GROSS-FEE-NET");
        let fee = case["records"].as_array_mut().unwrap()
            .iter_mut().find(|record| optional_string(record, "record_ref") == Some("S002-FEE")).unwrap();
        fee["direction"] = Value::String("credit".to_owned());
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn oracle_case_ids_must_match_candidate_case_ids() {
        let (snapshot, candidates, mut oracle) = fixtures();
        oracle["expectations"].as_array_mut().unwrap()[0]["case_id"] =
            Value::String("RB-UNKNOWN".to_owned());
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }

    #[test]
    fn pinned_source_file_manifest_contains_four_agent_inputs_and_seven_oracle_only_files() {
        let (snapshot, _, _) = fixtures();
        assert_eq!(array(&snapshot, "agent_input_paths").unwrap().len(), 4);
        assert_eq!(array(&snapshot, "oracle_only_paths").unwrap().len(), 7);
        assert_eq!(array(&snapshot["manifest"], "files").unwrap().len(), 11);
        assert_eq!(snapshot["manifest_sha256_status"], "not_computed_from_raw_manifest_bytes");
    }

    #[test]
    fn unsafe_relative_dataset_path_is_rejected() {
        assert!(safe_relative_path("../truth/file.parquet").is_err());
        assert!(safe_relative_path("/etc/passwd").is_err());
        assert!(safe_relative_path("ledger/part-0000.parquet").is_ok());
    }

    #[test]
    fn duplicate_source_identity_with_different_payload_is_conflict_not_deduplication() {
        let (snapshot, mut candidates, oracle) = fixtures();
        let case = case_mut(&mut candidates, "RB-006-DUPLICATE-SETTLEMENT-FEED");
        let duplicate = case["records"].as_array_mut().unwrap()
            .iter_mut().find(|record| optional_string(record, "record_ref") == Some("S006-DUP")).unwrap();
        duplicate["amount_minor"] = Value::from(51000);
        assert!(validate_all(&snapshot, &candidates, &oracle).is_err());
    }
}
