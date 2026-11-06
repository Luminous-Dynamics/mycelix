#![forbid(unsafe_code)]

//! Effects-disabled multi-hop payment recovery graph evaluator.
//!
//! Candidate transaction continuity is not proof of ownership of fungible funds.
//! Notifications and holds are not returns. Only unique, independently
//! reconciled successor returns change per-root recovery and residual loss.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/payment-recovery-graph-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/payment-recovery-graph-corpus-v1.schema.json");

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn valid_date(value: &str) -> bool {
    let mut parts = value.split('-');
    let Some(year) = parts.next().and_then(|part| part.parse::<i32>().ok()) else {
        return false;
    };
    let Some(month) = parts.next().and_then(|part| part.parse::<u32>().ok()) else {
        return false;
    };
    let Some(day) = parts.next().and_then(|part| part.parse::<u32>().ok()) else {
        return false;
    };
    if parts.next().is_some() || year < 1970 || !(1..=12).contains(&month) {
        return false;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let last_day = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    (1..=last_day).contains(&day)
}

fn optional_balance(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn valid_timestamp(value: &str) -> bool {
    value.len() >= 20
        && valid_date(&value[..10])
        && value.as_bytes().get(10) == Some(&b'T')
        && (value.ends_with('Z') || value.contains('+') || value.rfind('-').is_some_and(|i| i > 10))
}

fn schema_errors(schema: &Value, corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against the JSON Schema meta-schema".into());
        return errors;
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => validator,
        Err(error) => {
            errors.push(format!("cannot compile Draft 2020-12 schema: {error}"));
            return errors;
        }
    };
    for error in validator.iter_errors(corpus) {
        errors.push(format!(
            "schema error at {}: {error}",
            error.instance_path()
        ));
    }
    errors
}

fn unique_ids(values: &[Value], field: &str, path: &str, errors: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    for (index, item) in values.iter().enumerate() {
        match str_at(item, field) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{path}[{index}].{field}: duplicate identity {id}")),
            None => errors.push(format!("{path}[{index}].{field}: missing identity")),
        }
    }
}

fn string_set(values: &[Value]) -> BTreeSet<String> {
    values
        .iter()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect()
}

fn ancestry_roots(
    parent_ids: &[Value],
    roots: &BTreeMap<String, Value>,
    transactions: &BTreeMap<String, Value>,
    visiting: &mut BTreeSet<String>,
    cache: &mut BTreeMap<String, BTreeSet<String>>,
) -> Result<BTreeSet<String>, String> {
    let mut result = BTreeSet::new();
    for parent_value in parent_ids {
        let parent_id = parent_value
            .as_str()
            .ok_or_else(|| "parent identity is not a string".to_owned())?;
        if roots.contains_key(parent_id) {
            result.insert(parent_id.to_owned());
            continue;
        }
        if let Some(cached) = cache.get(parent_id) {
            result.extend(cached.iter().cloned());
            continue;
        }
        if !visiting.insert(parent_id.to_owned()) {
            return Err(format!("graph cycle detected at transaction {parent_id}"));
        }
        let parent = transactions
            .get(parent_id)
            .ok_or_else(|| format!("unresolved parent transaction {parent_id}"))?;
        let parent_ids = array_at(parent, "parent_transaction_ids")
            .ok_or_else(|| format!("{parent_id}: parent_transaction_ids missing"))?;
        let found = ancestry_roots(parent_ids, roots, transactions, visiting, cache)?;
        visiting.remove(parent_id);
        cache.insert(parent_id.to_owned(), found.clone());
        result.extend(found);
    }
    Ok(result)
}

fn scenario_evaluation(case: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    let case_id = str_at(case, "scenario_id").unwrap_or("<missing>");
    let roots_vec = array_at(case, "roots").cloned().unwrap_or_default();
    let tx_vec = array_at(case, "transactions").cloned().unwrap_or_default();
    let snapshots = array_at(case, "account_snapshots")
        .cloned()
        .unwrap_or_default();
    let notifications = array_at(case, "notifications").cloned().unwrap_or_default();
    let holds = array_at(case, "holds").cloned().unwrap_or_default();
    let returns = array_at(case, "returns").cloned().unwrap_or_default();
    let reversals = array_at(case, "return_reversals")
        .cloned()
        .unwrap_or_default();

    if roots_vec.is_empty() {
        errors.push(format!(
            "{case_id}: at least one root transaction is required"
        ));
    }
    let mut roots = BTreeMap::new();
    for root in &roots_vec {
        if let Some(id) = str_at(root, "root_transaction_id") {
            roots.insert(id.to_owned(), root.clone());
        }
    }
    if roots.len() != roots_vec.len() {
        errors.push(format!("{case_id}: duplicate or missing root identity"));
    }
    let mut transactions = BTreeMap::new();
    for tx in &tx_vec {
        if let Some(id) = str_at(tx, "transaction_id") {
            transactions.insert(id.to_owned(), tx.clone());
        }
    }
    if transactions.len() != tx_vec.len() {
        errors.push(format!(
            "{case_id}: duplicate or missing descendant transaction identity"
        ));
    }
    for id in roots.keys() {
        if transactions.contains_key(id) {
            errors.push(format!(
                "{case_id}: root/descendant identity collision for {id}"
            ));
        }
    }

    let completeness = str_at(case, "graph_completeness").unwrap_or("unknown");
    let unresolved = array_at(case, "unresolved_external_parent_ids")
        .cloned()
        .unwrap_or_default();
    if completeness == "complete" && !unresolved.is_empty() {
        errors.push(format!(
            "{case_id}: complete graph has unresolved external parents"
        ));
    }
    if matches!(completeness, "partial" | "unknown") && unresolved.is_empty() {
        errors.push(format!(
            "{case_id}: partial/unknown graph must expose unresolved external evidence"
        ));
    }

    for (root_id, root) in &roots {
        let amount = u64_at(root, "amount_minor_units").unwrap_or(0);
        let currency = str_at(root, "currency").unwrap_or("");
        let timestamp = str_at(root, "observed_at").unwrap_or("");
        if amount == 0 || currency.len() != 3 || !valid_timestamp(timestamp) {
            errors.push(format!(
                "{case_id}: root {root_id} has invalid amount, currency or time"
            ));
        }
        if !array_at(root, "source_evidence_refs").is_some_and(|refs| !refs.is_empty()) {
            errors.push(format!("{case_id}: root {root_id} lacks source evidence"));
        }
    }

    let mut cache = BTreeMap::new();
    for (tx_id, tx) in &transactions {
        let source_account = str_at(tx, "source_account_id").unwrap_or("");
        let currency = str_at(tx, "currency").unwrap_or("");
        let amount = u64_at(tx, "amount_minor_units").unwrap_or(0);
        let observed = str_at(tx, "observed_at").unwrap_or("");
        if source_account.is_empty() || amount == 0 || !valid_timestamp(observed) {
            errors.push(format!(
                "{case_id}: transaction {tx_id} has invalid source, amount or time"
            ));
        }
        if str_at(tx, "relationship_claim")
            != Some("candidate_flow_continuity_not_proven_fund_origin")
        {
            errors.push(format!(
                "{case_id}: transaction {tx_id} overstates candidate provenance"
            ));
        }
        if !array_at(tx, "link_evidence_refs").is_some_and(|refs| !refs.is_empty()) {
            errors.push(format!(
                "{case_id}: transaction {tx_id} lacks link evidence"
            ));
        }

        let parents = array_at(tx, "parent_transaction_ids")
            .cloned()
            .unwrap_or_default();
        let mut visiting = BTreeSet::new();
        match ancestry_roots(&parents, &roots, &transactions, &mut visiting, &mut cache) {
            Ok(ancestral) => {
                let declared = string_set(
                    array_at(tx, "candidate_root_ids")
                        .map(Vec::as_slice)
                        .unwrap_or(&[]),
                );
                if ancestral != declared || ancestral.is_empty() {
                    errors.push(format!(
                        "{case_id}: transaction {tx_id} candidate roots disagree with graph reachability"
                    ));
                }
            }
            Err(error) => errors.push(format!("{case_id}: transaction {tx_id}: {error}")),
        }

        for parent_value in &parents {
            let Some(parent_id) = parent_value.as_str() else {
                errors.push(format!(
                    "{case_id}: transaction {tx_id} has non-string parent"
                ));
                continue;
            };
            let (destination, parent_currency, parent_time) =
                if let Some(root) = roots.get(parent_id) {
                    (
                        str_at(root, "recipient_account_id").unwrap_or(""),
                        str_at(root, "currency").unwrap_or(""),
                        str_at(root, "observed_at").unwrap_or(""),
                    )
                } else if let Some(parent) = transactions.get(parent_id) {
                    (
                        str_at(parent, "destination_account_id").unwrap_or(""),
                        str_at(parent, "currency").unwrap_or(""),
                        str_at(parent, "observed_at").unwrap_or(""),
                    )
                } else {
                    errors.push(format!(
                        "{case_id}: transaction {tx_id} has unresolved parent {parent_id}"
                    ));
                    continue;
                };
            if destination != source_account {
                errors.push(format!(
                    "{case_id}: transaction {tx_id} breaks account continuity from {parent_id}"
                ));
            }
            if parent_currency != currency {
                errors.push(format!(
                    "{case_id}: transaction {tx_id} changes currency without an FX contract"
                ));
            }
            if parent_time > observed {
                errors.push(format!(
                    "{case_id}: transaction {tx_id} predates parent {parent_id}"
                ));
            }
        }
    }

    let mut snapshot_keys = BTreeSet::new();
    let mut snapshot_map = BTreeMap::<String, Value>::new();
    for snapshot in &snapshots {
        let account = str_at(snapshot, "account_id").unwrap_or("");
        let currency = str_at(snapshot, "currency").unwrap_or("");
        let key = format!("{account}:{currency}");
        if account.is_empty() || currency.len() != 3 || !snapshot_keys.insert(key.clone()) {
            errors.push(format!("{case_id}: invalid or duplicate account/currency snapshot {key}"));
        } else {
            snapshot_map.insert(key, snapshot.clone());
        }
    }
    for snapshot in &snapshots {
        let account = str_at(snapshot, "account_id").unwrap_or("");
        let coverage = str_at(snapshot, "coverage_state").unwrap_or("unknown");
        if coverage == "unknown" {
            for field in [
                "opening_balance_minor_units",
                "root_inflows_minor_units",
                "other_inflows_minor_units",
                "observed_outflows_minor_units",
                "other_outflows_minor_units",
                "closing_balance_minor_units",
                "available_balance_minor_units",
            ] {
                if !snapshot.get(field).is_none_or(Value::is_null) {
                    errors.push(format!(
                        "{case_id}: unknown snapshot {account} invents {field}"
                    ));
                }
            }
        } else if coverage == "complete" {
            let fields = [
                "opening_balance_minor_units",
                "root_inflows_minor_units",
                "other_inflows_minor_units",
                "observed_outflows_minor_units",
                "other_outflows_minor_units",
                "closing_balance_minor_units",
            ];
            let values: Option<Vec<u64>> = fields
                .iter()
                .map(|field| snapshot.get(*field).and_then(Value::as_u64))
                .collect();
            match values {
                Some(v) => {
                    let credits = v[0].checked_add(v[1]).and_then(|x| x.checked_add(v[2]));
                    let debits_and_close = v[3].checked_add(v[4]).and_then(|x| x.checked_add(v[5]));
                    if credits.is_none() || credits != debits_and_close {
                        errors.push(format!(
                            "{case_id}: complete account snapshot {account} fails conservation"
                        ));
                    }
                    let graph_root_inflows = roots
                        .values()
                        .filter(|root| str_at(root, "recipient_account_id") == Some(account))
                        .try_fold(0_u64, |total, root| {
                            total.checked_add(u64_at(root, "amount_minor_units")?)
                        });
                    let graph_observed_outflows = transactions
                        .values()
                        .filter(|tx| str_at(tx, "source_account_id") == Some(account))
                        .try_fold(0_u64, |total, tx| {
                            total.checked_add(u64_at(tx, "amount_minor_units")?)
                        });
                    if graph_root_inflows != Some(v[1]) || graph_observed_outflows != Some(v[3]) {
                        errors.push(format!(
                            "{case_id}: complete snapshot {account} inflow/outflow totals differ from observed root and graph transactions"
                        ));
                    }
                    if optional_balance(snapshot, "available_balance_minor_units")
                        .is_some_and(|available| available > v[5])
                    {
                        errors.push(format!(
                            "{case_id}: available balance exceeds closing balance for {account}"
                        ));
                    }
                }
                None => errors.push(format!(
                    "{case_id}: complete account snapshot {account} has missing values"
                )),
            }
        } else if coverage != "partial" {
            errors.push(format!(
                "{case_id}: account {account} has invalid coverage state"
            ));
        }
    }

    unique_ids(
        &notifications,
        "notification_id",
        "notifications",
        &mut errors,
    );
    let mut notification_map = BTreeMap::new();
    for notification in &notifications {
        let Some(id) = str_at(notification, "notification_id") else {
            continue;
        };
        notification_map.insert(id.to_owned(), notification.clone());
        let tx_id = str_at(notification, "transaction_id").unwrap_or("");
        let account = str_at(notification, "target_account_id").unwrap_or("");
        let psp = str_at(notification, "psp_id").unwrap_or("");
        let root_id = str_at(notification, "root_transaction_id").unwrap_or("");
        match (transactions.get(tx_id), roots.get(root_id)) {
            (Some(tx), Some(_root))
                if str_at(tx, "destination_account_id") == Some(account)
                    && str_at(tx, "destination_psp_id") == Some(psp)
                    && string_set(
                        array_at(tx, "candidate_root_ids")
                            .map(Vec::as_slice)
                            .unwrap_or(&[]),
                    )
                    .contains(root_id) => {}
            (Some(_), Some(_)) => errors.push(format!(
                "{case_id}: notification {id} target or root binding differs from candidate graph"
            )),
            _ => errors.push(format!(
                "{case_id}: notification {id} references unknown transaction or root"
            )),
        }
    }

    unique_ids(&holds, "hold_id", "holds", &mut errors);
    let mut active_hold_totals = BTreeMap::<(String, String), (u64, u64)>::new();
    for hold in &holds {
        let id = str_at(hold, "hold_id").unwrap_or("");
        let notification_id = str_at(hold, "notification_id").unwrap_or("");
        let account = str_at(hold, "account_id").unwrap_or("");
        let currency = str_at(hold, "currency").unwrap_or("");
        let requested = u64_at(hold, "requested_minor_units").unwrap_or(0);
        let held = u64_at(hold, "held_minor_units").unwrap_or(u64::MAX);
        let available = hold
            .get("available_balance_minor_units")
            .and_then(Value::as_u64);
        let state = str_at(hold, "hold_state").unwrap_or("");
        match notification_map.get(notification_id) {
            Some(n)
                if str_at(n, "state") == Some("accepted")
                    && str_at(n, "authority_state") == Some("valid")
                    && str_at(n, "target_account_id") == Some(account) =>
            {
                let notification_root = str_at(n, "root_transaction_id").unwrap_or("");
                if roots
                    .get(notification_root)
                    .is_none_or(|root| requested > u64_at(root, "amount_minor_units").unwrap_or(0))
                {
                    errors.push(format!(
                        "{case_id}: hold {id} request exceeds or lacks its exact root transaction"
                    ));
                }
                let notification_tx_id = str_at(n, "transaction_id").unwrap_or("");
                if transactions
                    .get(notification_tx_id)
                    .is_none_or(|tx| str_at(tx, "currency") != Some(currency))
                {
                    errors.push(format!(
                        "{case_id}: hold {id} currency differs from its notified transaction"
                    ));
                }
                let snapshot_key = format!("{account}:{currency}");
                match snapshot_map.get(&snapshot_key) {
                            Some(snapshot)
                                if str_at(snapshot, "psp_id") == str_at(n, "psp_id")
                                    && optional_balance(snapshot, "available_balance_minor_units")
                                        == available => {}
                            _ => errors.push(format!(
                                "{case_id}: hold {id} is not backed by the matching account/PSP/currency balance snapshot"
                            )),
                        }
            }
            _ => errors.push(format!(
                "{case_id}: hold {id} lacks accepted notification or target scope"
            )),
        }
        if str_at(hold, "authority_state") != Some("valid") || requested == 0 || held > requested {
            errors.push(format!(
                "{case_id}: hold {id} lacks authority or exceeds requested value"
            ));
        }
        match available {
            Some(balance) if held <= balance => {}
            Some(_) => errors.push(format!(
                "{case_id}: hold {id} exceeds observed available balance"
            )),
            None if held > 0 => errors.push(format!(
                "{case_id}: hold {id} claims value without known local balance"
            )),
            None => {}
        }
        if matches!(state, "applied" | "partial") && held == 0 {
            errors.push(format!("{case_id}: active hold {id} has zero held value"));
        }
        if !array_at(hold, "source_evidence_refs").is_some_and(|refs| !refs.is_empty()) {
            errors.push(format!("{case_id}: hold {id} lacks source evidence"));
        }
        if matches!(state, "applied" | "partial")
            && str_at(hold, "authority_state") == Some("valid")
        {
            if let Some(balance) = available {
                let key = (account.to_owned(), currency.to_owned());
                let entry = active_hold_totals.entry(key).or_insert((0, balance));
                if entry.1 != balance {
                    errors.push(format!(
                        "{case_id}: active holds on {account}/{currency} cite inconsistent available-balance snapshots"
                    ));
                }
                entry.0 = match entry.0.checked_add(held) {
                    Some(total) => total,
                    None => {
                        errors.push(format!(
                            "{case_id}: active hold aggregate overflow for {account}/{currency}"
                        ));
                        u64::MAX
                    }
                };
            }
        }
    }
    for ((account, currency), (held_total, available_total)) in active_hold_totals {
        if held_total > available_total {
            errors.push(format!(
                "{case_id}: concurrent active holds reserve {held_total} {currency} on {account}, exceeding shared available balance {available_total}"
            ));
        }
    }

    unique_ids(&returns, "return_id", "returns", &mut errors);
    let mut return_map = BTreeMap::new();
    let mut economic_refs = BTreeSet::new();
    let mut receipt_refs = BTreeSet::new();
    let mut reconciliation_refs = BTreeSet::new();
    let mut gross_by_root = BTreeMap::<String, u64>::new();
    for returned in &returns {
        let id = str_at(returned, "return_id").unwrap_or("");
        let root_id = str_at(returned, "root_transaction_id").unwrap_or("");
        let amount = u64_at(returned, "amount_minor_units").unwrap_or(0);
        let currency = str_at(returned, "currency").unwrap_or("");
        let economic_ref = str_at(returned, "economic_reference").unwrap_or("");
        if !economic_refs.insert(economic_ref.to_owned()) {
            errors.push(format!(
                "{case_id}: duplicate economic return reference {economic_ref}"
            ));
        }
        if let Some(receipt) = str_at(returned, "independent_return_receipt")
            && !receipt_refs.insert(receipt.to_owned())
        {
            errors.push(format!(
                "{case_id}: duplicate independent return receipt {receipt}"
            ));
        }
        if let Some(reconciliation) = str_at(returned, "independent_ledger_reconciliation")
            && !reconciliation_refs.insert(reconciliation.to_owned())
        {
            errors.push(format!(
                "{case_id}: duplicate independent ledger reconciliation {reconciliation}"
            ));
        }
        if amount == 0
            || str_at(returned, "authority_state") != Some("valid")
            || returned.get("reconciled").and_then(Value::as_bool) != Some(true)
            || str_at(returned, "independent_return_receipt").is_none()
            || str_at(returned, "independent_ledger_reconciliation").is_none()
        {
            errors.push(format!(
                "{case_id}: return {id} lacks valid authority or independent evidence"
            ));
        }
        match roots.get(root_id) {
            Some(root) if str_at(root, "currency") == Some(currency) => {}
            Some(_) => errors.push(format!(
                "{case_id}: return {id} currency differs from its root"
            )),
            None => errors.push(format!(
                "{case_id}: return {id} references unknown root {root_id}"
            )),
        }
        let total = gross_by_root.entry(root_id.to_owned()).or_default();
        *total = match total.checked_add(amount) {
            Some(value) => value,
            None => {
                errors.push(format!(
                    "{case_id}: return arithmetic overflow for root {root_id}"
                ));
                u64::MAX
            }
        };
        return_map.insert(id.to_owned(), returned.clone());
    }

    unique_ids(&reversals, "reversal_id", "return_reversals", &mut errors);
    let mut reversed_by_return = BTreeMap::<String, u64>::new();
    for reversal in &reversals {
        let id = str_at(reversal, "reversal_id").unwrap_or("");
        let return_id = str_at(reversal, "return_id").unwrap_or("");
        let amount = u64_at(reversal, "amount_minor_units").unwrap_or(0);
        let Some(original) = return_map.get(return_id) else {
            errors.push(format!(
                "{case_id}: reversal {id} references unknown return {return_id}"
            ));
            continue;
        };
        if amount == 0
            || str_at(reversal, "authority_state") != Some("valid")
            || reversal.get("reconciled").and_then(Value::as_bool) != Some(true)
            || str_at(reversal, "currency") != str_at(original, "currency")
            || !array_at(reversal, "evidence_refs").is_some_and(|refs| !refs.is_empty())
        {
            errors.push(format!(
                "{case_id}: reversal {id} lacks authority, reconciliation or evidence"
            ));
        }
        let total = reversed_by_return.entry(return_id.to_owned()).or_default();
        *total = match total.checked_add(amount) {
            Some(value) => value,
            None => {
                errors.push(format!(
                    "{case_id}: reversal arithmetic overflow for {return_id}"
                ));
                u64::MAX
            }
        };
        if *total > u64_at(original, "amount_minor_units").unwrap_or(0) {
            errors.push(format!(
                "{case_id}: reversals exceed original return {return_id}"
            ));
        }
    }

    let mut net_by_root = BTreeMap::<String, u64>::new();
    for (root_id, gross) in &gross_by_root {
        let mut reversed = 0_u64;
        for (return_id, original) in &return_map {
            if str_at(original, "root_transaction_id") == Some(root_id) {
                reversed = match reversed
                    .checked_add(reversed_by_return.get(return_id).copied().unwrap_or(0))
                {
                    Some(value) => value,
                    None => {
                        errors.push(format!(
                            "{case_id}: reversal total overflow for root {root_id}"
                        ));
                        u64::MAX
                    }
                };
            }
        }
        if reversed > *gross {
            errors.push(format!(
                "{case_id}: reversals exceed gross returns for root {root_id}"
            ));
            continue;
        }
        let net = gross - reversed;
        let original = roots
            .get(root_id)
            .and_then(|root| u64_at(root, "amount_minor_units"))
            .unwrap_or(0);
        if net > original {
            errors.push(format!(
                "{case_id}: net return exceeds original root {root_id}"
            ));
        }
        net_by_root.insert(root_id.clone(), net);
    }
    for id in roots.keys() {
        net_by_root.entry(id.clone()).or_insert(0);
    }

    let mut expected = BTreeMap::new();
    for item in array_at(case, "expected_net_return_by_root")
        .cloned()
        .unwrap_or_default()
    {
        let id = str_at(&item, "root_transaction_id").unwrap_or("");
        let amount = u64_at(&item, "net_returned_minor_units").unwrap_or(u64::MAX);
        if expected.insert(id.to_owned(), amount).is_some() {
            errors.push(format!("{case_id}: duplicate expected root return {id}"));
        }
    }
    if expected.len() != roots.len() {
        errors.push(format!(
            "{case_id}: expected return table must cover each root once"
        ));
    }
    for (id, amount) in &net_by_root {
        if expected.get(id) != Some(amount) {
            errors.push(format!(
                "{case_id}: expected net return differs for root {id}"
            ));
        }
    }

    if str_at(case, "remedy_classification") == Some("mistaken_payee")
        && (str_at(case, "recovery_admission") != Some("not_admitted")
            || !tx_vec.is_empty()
            || !notifications.is_empty()
            || !holds.is_empty()
            || !returns.is_empty()
            || !reversals.is_empty())
    {
        errors.push(format!(
            "{case_id}: mistaken-payee case incorrectly entered fraud MED graph"
        ));
    }

    if u64_at(case, "expected_unique_descendant_transaction_count")
        != Some(transactions.len() as u64)
    {
        errors.push(format!(
            "{case_id}: unique descendant transaction count mismatch"
        ));
    }

    if errors.is_empty() {
        let accounting: Vec<Value> = roots
            .iter()
            .map(|(id, root)| {
                let original = u64_at(root, "amount_minor_units").unwrap_or(0);
                let net = net_by_root.get(id).copied().unwrap_or(0);
                json!({
                    "root_transaction_id": id,
                    "currency": root["currency"],
                    "original_minor_units": original,
                    "net_reconciled_return_minor_units": net,
                    "residual_loss_minor_units": original - net
                })
            })
            .collect();
        Ok(json!({
            "scenario_id": case_id,
            "unique_descendant_transaction_count": transactions.len(),
            "notification_count": notifications.len(),
            "hold_record_count": holds.len(),
            "verified_return_event_count": returns.len(),
            "return_reversal_event_count": reversals.len(),
            "root_accounting": accounting,
            "graph_completeness": completeness,
            "candidate_edges_prove_fund_origin": false,
            "holds_are_recovered_value": false
        }))
    } else {
        Err(errors)
    }
}

fn semantic_evaluation(corpus: &Value) -> (Vec<String>, Vec<Value>) {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some("mycelix.payment-recovery-graph/v1")
        || str_at(corpus, "corpus_id") != Some("PAYMENT-RECOVERY-GRAPH-SYNTHETIC-V1")
    {
        errors.push("unexpected graph corpus identity".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only")
        || str_at(corpus, "jurisdiction") != Some("BR")
        || str_at(corpus, "scheme") != Some("PIX_MED_V4_2_COMPARATOR")
        || corpus
            .get("external_effects_enabled")
            .and_then(Value::as_bool)
            != Some(false)
    {
        errors.push(
            "graph corpus must remain synthetic, Brazil Pix comparator and effects-disabled".into(),
        );
    }
    let as_of = str_at(corpus, "as_of").unwrap_or("");
    if !valid_date(as_of) {
        errors.push("as_of must be a pinned valid date".into());
    }

    let sources = array_at(corpus, "source_register")
        .cloned()
        .unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    unique_ids(&sources, "id", "source_register", &mut errors);
    unique_ids(&profiles, "profile_id", "profiles", &mut errors);
    unique_ids(&scenarios, "scenario_id", "scenarios", &mut errors);
    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|item| str_at(item, "id").map(str::to_owned))
        .collect();

    for (index, source) in sources.iter().enumerate() {
        let observed = str_at(source, "observed_on").unwrap_or("");
        if !valid_date(observed) || observed > as_of {
            errors.push(format!(
                "source_register[{index}] has a future/invalid observation date"
            ));
        }
        if let Some(published) = str_at(source, "published_on")
            && (!valid_date(published) || published > as_of)
        {
            errors.push(format!(
                "source_register[{index}] has a future/invalid publication date"
            ));
        }
    }
    for (index, profile) in profiles.iter().enumerate() {
        let refs = array_at(profile, "source_refs")
            .cloned()
            .unwrap_or_default();
        if refs.is_empty() {
            errors.push(format!("profiles[{index}] has no source references"));
        }
        for reference in refs {
            if !reference.as_str().is_some_and(|id| source_ids.contains(id)) {
                errors.push(format!("profiles[{index}] has unresolved source reference"));
            }
        }
        let observed = str_at(profile, "observed_on").unwrap_or("");
        if !valid_date(observed) || observed > as_of {
            errors.push(format!(
                "profiles[{index}] has future/invalid observation date"
            ));
        }
        if str_at(profile, "effective_date_state") == Some("future_effective_not_current_on_as_of")
            && !str_at(profile, "effective_from")
                .is_some_and(|date| valid_date(date) && date > as_of)
        {
            errors.push(format!(
                "profiles[{index}] future-effective date is not actually in the future"
            ));
        }
    }

    let mut summaries = Vec::new();
    for case in &scenarios {
        match scenario_evaluation(case) {
            Ok(summary) => summaries.push(summary),
            Err(mut case_errors) => errors.append(&mut case_errors),
        }
    }
    (errors, summaries)
}

fn evaluate(
    corpus_text: &str,
    schema_text: &str,
) -> Result<(Vec<String>, Value, Value, Vec<Value>), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let (mut errors, summaries) = semantic_evaluation(&corpus);
    errors.extend(schema_errors(&schema, &corpus));
    Ok((errors, corpus, schema, summaries))
}

fn mutate_and_reject(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let (mut errors, _) = semantic_evaluation(&changed);
    errors.extend(schema_errors(schema, &changed));
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let (baseline, _) = semantic_evaluation(corpus);
    if !baseline.is_empty() || !schema_errors(schema, corpus).is_empty() {
        return Err(format!(
            "baseline corpus must pass before mutations: {baseline:?}"
        ));
    }
    let mut count = 0usize;

    mutate_and_reject("external effects enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject(
        "candidate edge promoted to fund ownership",
        corpus,
        schema,
        |value| {
            value["scenarios"][0]["transactions"][0]["relationship_claim"] =
                json!("proven_fund_origin");
        },
    )?;
    count += 1;

    mutate_and_reject("convergent graph double counted", corpus, schema, |value| {
        value["scenarios"][3]["expected_unique_descendant_transaction_count"] = json!(6);
    })?;
    count += 1;

    mutate_and_reject("hold promoted to returned value", corpus, schema, |value| {
        value["scenarios"][6]["expected_net_return_by_root"][0]["net_returned_minor_units"] =
            json!(2_000);
    })?;
    count += 1;

    mutate_and_reject(
        "hold exceeds local available balance",
        corpus,
        schema,
        |value| {
            value["scenarios"][6]["holds"][0]["held_minor_units"] = json!(3_000);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "concurrent root holds exceed shared account balance",
        corpus,
        schema,
        |value| {
            value["scenarios"][24]["holds"][1]["held_minor_units"] = json!(4_000);
        },
    )?;
    count += 1;

    mutate_and_reject("duplicate transaction identity", corpus, schema, |value| {
        let duplicate = value["scenarios"][1]["transactions"][0].clone();
        value["scenarios"][1]["transactions"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
    })?;
    count += 1;

    mutate_and_reject("return greater than root amount", corpus, schema, |value| {
        value["scenarios"][12]["returns"][0]["amount_minor_units"] = json!(10_001);
    })?;
    count += 1;

    mutate_and_reject(
        "duplicate economic return identity",
        corpus,
        schema,
        |value| {
            let duplicate = value["scenarios"][13]["returns"][0].clone();
            value["scenarios"][13]["returns"]
                .as_array_mut()
                .unwrap()
                .push(duplicate);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "duplicate reconciled receipt hidden by new return id",
        corpus,
        schema,
        |value| {
            value["scenarios"][13]["returns"][1]["independent_return_receipt"] =
                value["scenarios"][13]["returns"][0]["independent_return_receipt"].clone();
        },
    )?;
    count += 1;

    mutate_and_reject(
        "unknown balance replaced by zero",
        corpus,
        schema,
        |value| {
            value["scenarios"][18]["account_snapshots"][0]["available_balance_minor_units"] =
                json!(0);
        },
    )?;
    count += 1;

    mutate_and_reject(
        "mistaken-payee entered fraud MED graph",
        corpus,
        schema,
        |value| {
            value["scenarios"][23]["recovery_admission"] = json!("pix_med_fraud_graph");
        },
    )?;
    count += 1;

    mutate_and_reject("graph cycle", corpus, schema, |value| {
        value["scenarios"][1]["transactions"][0]["parent_transaction_ids"]
            .as_array_mut()
            .unwrap()
            .push(json!("TX-2"));
    })?;
    count += 1;

    mutate_and_reject("partial graph labelled complete", corpus, schema, |value| {
        value["scenarios"][15]["graph_completeness"] = json!("complete");
    })?;
    count += 1;

    mutate_and_reject(
        "return reversal exceeds original return",
        corpus,
        schema,
        |value| {
            value["scenarios"][14]["return_reversals"][0]["amount_minor_units"] = json!(8_001);
        },
    )?;
    count += 1;

    Ok(count)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = DEFAULT_CORPUS.to_owned();
    let mut schema_text = DEFAULT_SCHEMA.to_owned();
    let mut self_test = false;
    let mut args = std::env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--corpus" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--corpus requires a path",
                    )
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--schema requires a path",
                    )
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!(
                    "payment-recovery-graph-evaluator [--corpus PATH] [--schema PATH] [--self-test]"
                );
                return Ok(());
            }
            other => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("unknown argument: {other}"),
                )
                .into());
            }
        }
    }

    let (errors, corpus, schema, summaries) = evaluate(&corpus_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let mutation_count = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let report = json!({
        "evaluator": "mycelix.payment-recovery-graph-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic graph reachability, account conservation, authority-scoped holds and unique per-root return accounting only",
        "scenario_count": array_at(&corpus, "scenarios").map_or(0, Vec::len),
        "negative_controls": mutation_count,
        "scenario_summaries": summaries,
        "errors": errors
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if status != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_in_schema_and_graph_corpus_validate() {
        let (errors, corpus, _, summaries) =
            evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 3);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 2);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 25);
        assert_eq!(summaries.len(), 25);
    }

    #[test]
    fn concurrent_active_holds_cannot_overreserve_shared_account_balance() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let (baseline_errors, _) = semantic_evaluation(&corpus);
        assert!(baseline_errors.is_empty(), "{baseline_errors:?}");

        let mut changed = corpus;
        changed["scenarios"][24]["holds"][1]["held_minor_units"] = json!(4_000);
        let (errors, _) = semantic_evaluation(&changed);
        assert!(
            errors.iter().any(|error| error.contains("concurrent active holds reserve")),
            "{errors:?}"
        );
    }

    #[test]
    fn net_returns_and_reversals_are_accounted_per_root() {
        let (_, _, _, summaries) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert_eq!(
            summaries[11]["root_accounting"][0]["net_reconciled_return_minor_units"],
            json!(3_000)
        );
        assert_eq!(
            summaries[14]["root_accounting"][0]["net_reconciled_return_minor_units"],
            json!(6_000)
        );
        assert_eq!(
            summaries[13]["root_accounting"][0]["residual_loss_minor_units"],
            json!(4_000)
        );
    }

    #[test]
    fn deterministic_mutation_campaign_rejects_graph_and_money_errors() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 15);
    }
}
