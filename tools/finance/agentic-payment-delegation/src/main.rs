#![forbid(unsafe_code)]

//! Effects-disabled payment-delegation reference evaluator.
//!
//! QIE and exact Intent/ActionRequest values are opaque references owned by
//! their existing ancestry. This crate checks payment-specific scope,
//! price, reservation, revocation and sub-delegation invariants only.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/agentic-payment-delegation-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/agentic-payment-delegation-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.agentic-payment-delegation/v1";
const EXPECTED_CORPUS: &str = "AGENTIC-PAYMENT-DELEGATION-SYNTHETIC-V1";

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn contains(values: &Value, needle: &str) -> bool {
    values
        .as_array()
        .is_some_and(|items| items.iter().any(|item| item.as_str() == Some(needle)))
}

fn valid_digest(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64 && hex.chars().all(|ch| ch.is_ascii_hexdigit() && !ch.is_ascii_uppercase())
    })
}

fn schema_errors(schema: &Value, corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against the Draft 2020-12 meta-schema".into());
        return errors;
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(value) => value,
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

fn check_unique(values: &[Value], field: &str, path: &str, errors: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    for (index, value) in values.iter().enumerate() {
        match str_at(value, field) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{path}[{index}].{field}: duplicate {id}")),
            None => errors.push(format!("{path}[{index}].{field}: missing string")),
        }
    }
}

fn source_ids(corpus: &Value) -> BTreeSet<String> {
    array_at(corpus, "source_register")
        .into_iter()
        .flatten()
        .filter_map(|source| str_at(source, "id").map(str::to_owned))
        .collect()
}

fn profile_map(corpus: &Value) -> BTreeMap<String, Value> {
    array_at(corpus, "profiles")
        .into_iter()
        .flatten()
        .filter_map(|profile| {
            str_at(profile, "profile_id").map(|id| (id.to_owned(), profile.clone()))
        })
        .collect()
}

fn source_dates(corpus: &Value, as_of: &str, errors: &mut Vec<String>) {
    for (index, source) in array_at(corpus, "source_register")
        .into_iter()
        .flatten()
        .enumerate()
    {
        let Some(observed) = str_at(source, "observed_on") else {
            errors.push(format!("source_register[{index}].observed_on: missing"));
            continue;
        };
        if observed > as_of {
            errors.push(format!(
                "source_register[{index}].observed_on: later than corpus as_of"
            ));
        }
        if let Some(published) = source.get("published_on").and_then(Value::as_str)
            && published > as_of
        {
            errors.push(format!(
                "source_register[{index}].published_on: later than corpus as_of"
            ));
        }
    }
}

fn refs_resolve(values: &Value, known: &BTreeSet<String>) -> bool {
    values.as_array().is_some_and(|items| {
        !items.is_empty()
            && items
                .iter()
                .all(|item| item.as_str().is_some_and(|id| known.contains(id)))
    })
}

fn scope_value<'a>(scope: &'a Value, key: &str) -> Option<&'a Value> {
    scope.get(key)
}

fn set_is_subset(child: &Value, parent: &Value) -> bool {
    let Some(child_items) = child.as_array() else {
        return false;
    };
    let Some(parent_items) = parent.as_array() else {
        return false;
    };
    let parent_set: BTreeSet<&str> = parent_items.iter().filter_map(Value::as_str).collect();
    child_items
        .iter()
        .filter_map(Value::as_str)
        .all(|item| parent_set.contains(item))
}

fn scope_narrows(child: &Value, parent: &Value) -> bool {
    let numeric = [
        "max_single_debit_minor_units",
        "max_total_debit_minor_units",
        "max_fee_minor_units",
        "max_fx_cost_minor_units",
    ];
    numeric.iter().all(|key| {
        u64_at(child, key)
            .zip(u64_at(parent, key))
            .is_some_and(|(child_value, parent_value)| child_value <= parent_value)
    }) && [
        "allowed_action_classes",
        "allowed_purposes",
        "allowed_currencies",
        "allowed_beneficiary_refs",
    ]
    .iter()
    .all(|key| {
        scope_value(child, key)
            .zip(scope_value(parent, key))
            .is_some_and(|(child_values, parent_values)| {
                set_is_subset(child_values, parent_values)
            })
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Decision {
    Eligible,
    HumanConfirmationRequired,
    TermsRequalificationRequired,
    Rejected,
}

#[derive(Clone, Debug)]
struct LeafScope {
    agent_ref: String,
    scope: Value,
    is_delegated: bool,
}

fn leaf_scope(mandate: &Value, chain: &[Value], profile: &Value) -> Result<LeafScope, String> {
    let max_depth = u64_at(mandate, "max_delegation_depth").unwrap_or(0)
        .min(u64_at(profile, "max_delegation_depth").unwrap_or(0));
    if chain.len() as u64 > max_depth {
        return Err("delegation depth exceeds mandate or profile maximum".into());
    }
    let mut parent_agent = str_at(mandate, "agent_ref")
        .ok_or_else(|| "mandate agent_ref missing".to_owned())?
        .to_owned();
    let mut parent_scope = mandate.clone();
    let mut seen = BTreeSet::new();
    seen.insert(parent_agent.clone());

    for (index, edge) in chain.iter().enumerate() {
        let expected_depth = index as u64 + 1;
        if str_at(edge, "parent_agent_ref") != Some(parent_agent.as_str()) {
            return Err(format!("delegation_chain[{index}]: parent agent mismatch"));
        }
        if u64_at(edge, "depth") != Some(expected_depth) {
            return Err(format!("delegation_chain[{index}]: non-contiguous depth"));
        }
        let child_agent = str_at(edge, "child_agent_ref")
            .ok_or_else(|| format!("delegation_chain[{index}]: child agent missing"))?
            .to_owned();
        if !seen.insert(child_agent.clone()) {
            return Err(format!("delegation_chain[{index}]: delegation cycle or duplicate agent"));
        }
        let child_scope = edge
            .get("scope")
            .ok_or_else(|| format!("delegation_chain[{index}]: scope missing"))?;
        if !scope_narrows(child_scope, &parent_scope) {
            return Err(format!(
                "delegation_chain[{index}]: child scope widens parent authority"
            ));
        }
        parent_agent = child_agent;
        parent_scope = child_scope.clone();
    }

    if chain.is_empty() {
        Ok(LeafScope {
            agent_ref: str_at(mandate, "agent_ref").unwrap_or_default().to_owned(),
            scope: mandate.clone(),
            is_delegated: false,
        })
    } else {
        Ok(LeafScope {
            agent_ref: parent_agent,
            scope: parent_scope,
            is_delegated: true,
        })
    }
}

fn action_decision(
    action: &Value,
    mandate: &Value,
    profile: &Value,
    leaf: &LeafScope,
    source_ids: &BTreeSet<String>,
    human_confirmed: bool,
) -> Decision {
    let Some(agent_ref) = str_at(action, "agent_ref") else {
        return Decision::Rejected;
    };
    if agent_ref != leaf.agent_ref {
        return Decision::Rejected;
    }
    if str_at(action, "qualified_intent_envelope_ref")
        != str_at(mandate, "qualified_intent_envelope_ref")
        || str_at(action, "qualified_intent_envelope_digest")
            != str_at(mandate, "qualified_intent_envelope_digest")
    {
        return Decision::Rejected;
    }
    let Some(digest) = str_at(action, "exact_action_request_digest") else {
        return Decision::Rejected;
    };
    if !valid_digest(digest) {
        return Decision::Rejected;
    }

    let proposed_at = str_at(action, "proposed_at").unwrap_or("");
    if proposed_at < str_at(mandate, "not_before").unwrap_or("")
        || proposed_at >= str_at(mandate, "expires_at").unwrap_or("")
        || proposed_at < str_at(mandate, "budget_period_start").unwrap_or("")
        || proposed_at >= str_at(mandate, "budget_period_end").unwrap_or("")
    {
        return Decision::Rejected;
    }

    for (field, value) in [
        ("allowed_action_classes", str_at(action, "action_class")),
        ("allowed_purposes", str_at(action, "purpose")),
        ("allowed_currencies", str_at(action, "currency")),
    ] {
        let Some(value) = value else {
            return Decision::Rejected;
        };
        if !contains(&mandate[field], value) || !contains(&leaf.scope[field], value) {
            return Decision::Rejected;
        }
    }

    let beneficiary = str_at(action, "beneficiary_ref").unwrap_or("");
    let mandate_beneficiary_allowed = contains(&mandate["allowed_beneficiary_refs"], beneficiary);
    let leaf_beneficiary_allowed = contains(&leaf.scope["allowed_beneficiary_refs"], beneficiary);
    let is_new = action.get("is_new_beneficiary").and_then(Value::as_bool) == Some(true);
    let new_beneficiary_exception = is_new
        && !leaf.is_delegated
        && mandate.get("allow_new_beneficiary_with_confirmation").and_then(Value::as_bool)
            == Some(true);
    if !(mandate_beneficiary_allowed && leaf_beneficiary_allowed) && !new_beneficiary_exception {
        return Decision::Rejected;
    }

    let amount = u64_at(action, "amount_minor_units").unwrap_or(0);
    if amount == 0 {
        return Decision::Rejected;
    }

    let fee_state = str_at(action, "fee_state").unwrap_or("unknown");
    let fee_value = if fee_state == "known" {
        u64_at(action, "fee_minor_units")
    } else {
        None
    };
    let Some(fee) = fee_value else {
        return Decision::TermsRequalificationRequired;
    };
    let fx_value = if matches!(
        str_at(action, "fx_state"),
        Some("not_required" | "known")
    ) {
        u64_at(action, "fx_cost_minor_units")
    } else {
        None
    };
    let Some(fx_cost) = fx_value else {
        return Decision::TermsRequalificationRequired;
    };
    if fee > u64_at(mandate, "max_fee_minor_units").unwrap_or(0)
        || fee > u64_at(leaf.scope, "max_fee_minor_units").unwrap_or(0)
        || fx_cost > u64_at(mandate, "max_fx_cost_minor_units").unwrap_or(0)
        || fx_cost > u64_at(leaf.scope, "max_fx_cost_minor_units").unwrap_or(0)
    {
        return Decision::Rejected;
    }

    let budget_currency = str_at(mandate, "budget_currency").unwrap_or("");
    let action_currency = str_at(action, "currency").unwrap_or("");
    let Some(budget_equivalent) = u64_at(action, "budget_equivalent_principal_minor_units") else {
        return Decision::TermsRequalificationRequired;
    };
    if action_currency == budget_currency {
        if str_at(action, "fx_state") != Some("not_required")
            || fx_cost != 0
            || budget_equivalent != amount
            || !action["fx_quote"].is_null()
        {
            return Decision::Rejected;
        }
    } else {
        if str_at(action, "fx_state") != Some("known") {
            return Decision::TermsRequalificationRequired;
        }
        let Some(quote) = action.get("fx_quote").filter(|value| value.is_object()) else {
            return Decision::TermsRequalificationRequired;
        };
        let quote_source = str_at(quote, "source_ref").unwrap_or("");
        if !source_ids.contains(quote_source)
            || str_at(quote, "base_currency") != Some(action_currency)
            || str_at(quote, "quote_currency") != Some(budget_currency)
            || u64_at(quote, "base_amount_minor_units") != Some(amount)
            || u64_at(quote, "quote_amount_minor_units") != Some(budget_equivalent)
            || str_at(quote, "observed_at").unwrap_or("") > proposed_at
            || proposed_at >= str_at(quote, "expires_at").unwrap_or("")
        {
            return Decision::Rejected;
        }
    }

    let Some(all_in) = budget_equivalent.checked_add(fee).and_then(|v| v.checked_add(fx_cost)) else {
        return Decision::Rejected;
    };
    if u64_at(action, "all_in_budget_minor_units") != Some(all_in) {
        return Decision::Rejected;
    }
    let max_single = u64_at(mandate, "max_single_debit_minor_units")
        .unwrap_or(0)
        .min(u64_at(leaf.scope, "max_single_debit_minor_units").unwrap_or(0));
    if all_in > max_single {
        return Decision::Rejected;
    }

    let profile_threshold = u64_at(profile, "human_confirmation_threshold_minor_units")
        .unwrap_or(u64::MAX);
    let mandate_threshold = u64_at(mandate, "human_confirmation_threshold_minor_units")
        .unwrap_or(u64::MAX);
    let threshold = profile_threshold.min(mandate_threshold);
    let new_beneficiary_requires_confirmation = is_new
        && (profile
            .get("requires_human_confirmation_for_new_beneficiary")
            .and_then(Value::as_bool)
            == Some(true)
            || mandate
                .get("require_human_confirmation_for_new_beneficiary")
                .and_then(Value::as_bool)
                == Some(true));
    let profile_requires_human_presence = profile
        .get("allows_human_not_present")
        .and_then(Value::as_bool)
        == Some(false);
    let confirmation_required =
        new_beneficiary_requires_confirmation || profile_requires_human_presence || all_in >= threshold;

    if confirmation_required && !human_confirmed {
        Decision::HumanConfirmationRequired
    } else {
        Decision::Eligible
    }
}

#[derive(Clone, Debug)]
struct Reservation {
    action_id: String,
    agent_ref: String,
    amount: u64,
    state: String,
}

fn action_map(scenario: &Value) -> BTreeMap<String, Value> {
    array_at(scenario, "actions")
        .into_iter()
        .flatten()
        .filter_map(|action| str_at(action, "action_id").map(|id| (id.to_owned(), action.clone())))
        .collect()
}

fn semantic_scenario(
    scenario: &Value,
    profile: &Value,
    source_ids: &BTreeSet<String>,
    scenario_index: usize,
) -> Vec<String> {
    let mut errors = Vec::new();
    let prefix = format!("scenarios[{scenario_index}]");
    let mandate = &scenario["mandate"];
    let principal = str_at(mandate, "principal_ref").unwrap_or("");
    let root_agent = str_at(mandate, "agent_ref").unwrap_or("");
    let mandate_id = str_at(mandate, "mandate_id").unwrap_or("");
    let qie_digest = str_at(mandate, "qualified_intent_envelope_digest").unwrap_or("");
    let approval = &mandate["approval"];

    if mandate_id.is_empty() || principal.is_empty() || root_agent.is_empty() || principal == root_agent {
        errors.push(format!("{prefix}: invalid mandate/principal/agent identity"));
    }
    if !valid_digest(qie_digest)
        || str_at(approval, "qualified_intent_envelope_digest") != Some(qie_digest)
        || str_at(approval, "approval_state") != Some("qualified")
        || str_at(approval, "receipt_ref").is_none()
        || str_at(approval, "scope_digest").is_none()
    {
        errors.push(format!("{prefix}: mandate lacks qualified approval/QIE binding"));
    }
    if str_at(approval, "approval_kind") == Some("principal_approval")
        && str_at(approval, "approver_ref") != Some(principal)
    {
        errors.push(format!("{prefix}: principal approval is not from the mandator"));
    }
    if str_at(approval, "approval_kind") == Some("organization_policy_approval")
        && !str_at(approval, "approver_ref").unwrap_or("").starts_with("organization-policy:")
    {
        errors.push(format!("{prefix}: organization policy approval lacks registered policy identity"));
    }
    if mandate.get("external_effects_enabled").and_then(Value::as_bool) != Some(false) {
        errors.push(format!("{prefix}: mandate external effects must remain disabled"));
    }
    if str_at(mandate, "created_at").unwrap_or("") > str_at(mandate, "not_before").unwrap_or("")
        || str_at(mandate, "not_before").unwrap_or("") >= str_at(mandate, "expires_at").unwrap_or("")
        || str_at(mandate, "budget_period_start").unwrap_or("")
            >= str_at(mandate, "budget_period_end").unwrap_or("")
    {
        errors.push(format!("{prefix}: invalid mandate validity or budget period"));
    }
    let profile_id = str_at(mandate, "profile_id").unwrap_or("");
    if str_at(profile, "profile_id") != Some(profile_id) {
        errors.push(format!("{prefix}: mandate profile mismatch"));
    }
    if profile.get("requires_principal_approval").and_then(Value::as_bool) == Some(true)
        && str_at(approval, "approval_state") != Some("qualified")
    {
        errors.push(format!("{prefix}: profile requires qualified principal approval"));
    }

    let chain = array_at(scenario, "delegation_chain").cloned().unwrap_or_default();
    let leaf = match leaf_scope(mandate, &chain, profile) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("{prefix}: {error}"));
            LeafScope {
                agent_ref: root_agent.to_owned(),
                scope: mandate.clone(),
                is_delegated: !chain.is_empty(),
            }
        }
    };

    let actions = action_map(scenario);
    if actions.is_empty() {
        errors.push(format!("{prefix}: no candidate actions"));
        return errors;
    }
    let mut action_states: BTreeMap<String, String> = actions
        .keys()
        .map(|id| (id.clone(), "proposed".into()))
        .collect();
    let mut proposal_seen = BTreeSet::new();
    let mut human_confirmed = BTreeSet::new();
    let mut human_required_seen = BTreeSet::new();
    let mut rejected_seen = BTreeSet::new();
    let mut reservations: BTreeMap<String, Reservation> = BTreeMap::new();
    let mut action_reservation: BTreeMap<String, String> = BTreeMap::new();
    let mut event_ids = BTreeSet::new();
    let mut return_refs = BTreeSet::new();
    let mut used = 0_u64;
    let mut period_used = 0_u64;
    let mut outstanding = 0_u64;
    let mut reservation_count = 0_u64;
    let mut agent_used: BTreeMap<String, u64> = BTreeMap::new();
    let mut active = false;
    let mut revocation_pending = false;
    let mut revoked = false;
    let mut previous_sequence = 0_u64;
    let mut previous_time = String::new();

    let events = array_at(scenario, "events").cloned().unwrap_or_default();
    for (event_index, event) in events.iter().enumerate() {
        let event_path = format!("{prefix}.events[{event_index}]");
        let event_id = str_at(event, "event_id").unwrap_or("<missing>");
        if !event_ids.insert(event_id.to_owned()) {
            errors.push(format!("{event_path}: duplicate event_id"));
        }
        let sequence = u64_at(event, "sequence").unwrap_or(0);
        let observed_at = str_at(event, "observed_at").unwrap_or("");
        if sequence <= previous_sequence {
            errors.push(format!("{event_path}: sequence must be strictly increasing"));
        }
        if !previous_time.is_empty() && observed_at < previous_time.as_str() {
            errors.push(format!("{event_path}: event time moved backwards"));
        }
        previous_sequence = sequence;
        previous_time = observed_at.to_owned();

        let kind = str_at(event, "event_type").unwrap_or("");
        let actor_role = str_at(event, "actor_role").unwrap_or("");
        let actor_ref = str_at(event, "actor_ref").unwrap_or("");
        let authority = str_at(event, "authority_state").unwrap_or("");
        let action_id = str_at(event, "action_id");
        let reservation_id = str_at(event, "reservation_id");
        let amount = u64_at(event, "amount_minor_units").unwrap_or(u64::MAX);
        let digest = str_at(event, "exact_action_request_digest");

        if kind == "mandate_activated" {
            if active || sequence != 1 || actor_role != "principal" || actor_ref != principal
                || authority != "valid"
                || !contains(&event["evidence_types"], "principal_approval_receipt")
            {
                errors.push(format!("{event_path}: mandate activation lacks principal approval evidence"));
            }
            active = true;
            continue;
        }
        if !active {
            errors.push(format!("{event_path}: event precedes mandate activation"));
        }

        if kind == "mandate_revocation_requested" {
            if actor_role != "principal" || actor_ref != principal || authority != "valid"
                || !contains(&event["evidence_types"], "authenticated_revocation_request")
            {
                errors.push(format!("{event_path}: revocation request lacks authenticated principal evidence"));
            }
            revocation_pending = true;
            continue;
        }
        if kind == "mandate_revoked" {
            if actor_role != "authority_service" || authority != "valid"
                || !contains(&event["evidence_types"], "qualified_revocation_receipt")
            {
                errors.push(format!("{event_path}: effective revocation lacks authority-service evidence"));
            }
            revoked = true;
            revocation_pending = true;
            continue;
        }

        let Some(action) = action_id.and_then(|id| actions.get(id)) else {
            errors.push(format!("{event_path}: action-scoped event references unknown action"));
            continue;
        };
        let action_key = action_id.unwrap_or("");
        let expected_digest = str_at(action, "exact_action_request_digest");
        if kind != "action_proposed" && !proposal_seen.contains(action_key) {
            errors.push(format!("{event_path}: action event precedes the action proposal"));
        }
        if kind != "reservation_created" && kind != "payment_returned" && amount != 0 {
            errors.push(format!("{event_path}: non-reservation/non-return event carries an amount"));
        }
        if !matches!(kind, "payment_returned" | "duplicate_return_callback_ignored")
            && str_at(event, "return_reference").is_some()
        {
            errors.push(format!("{event_path}: non-return event carries a return reference"));
        }
        if kind != "human_confirmation_accepted" && str_at(event, "presentation_digest").is_some() {
            errors.push(format!("{event_path}: presentation digest appears outside a confirmation event"));
        }

        if matches!(
            kind,
            "action_proposed"
                | "human_confirmation_required"
                | "human_confirmation_accepted"
                | "action_rejected"
                | "reservation_created"
                | "reservation_rejected"
                | "duplicate_reservation_rejected"
                | "payment_dispatched"
                | "provider_outcome_unknown"
                | "payment_effect_confirmed"
                | "payment_effect_reconciled_no_effect"
                | "payment_returned"
                | "duplicate_return_callback_ignored"
        ) && digest != expected_digest
        {
            errors.push(format!("{event_path}: event is not bound to the exact ActionRequest digest"));
        }

        match kind {
            "action_proposed" => {
                if !proposal_seen.insert(action_key.to_owned()) {
                    errors.push(format!("{event_path}: duplicate action proposal"));
                }
                if actor_role != "agent" || actor_ref != str_at(action, "agent_ref").unwrap_or("")
                    || authority != "valid"
                    || !contains(&event["evidence_types"], "agent_proposal")
                {
                    errors.push(format!("{event_path}: action proposal lacks correct agent identity/evidence"));
                }
            }
            "human_confirmation_required" => {
                if actor_role != "policy_evaluator"
                    || authority != "valid"
                    || !contains(&event["evidence_types"], "confirmation_threshold_or_new_beneficiary")
                {
                    errors.push(format!("{event_path}: human-confirmation requirement lacks policy-evaluator evidence"));
                }
                human_required_seen.insert(action_key.to_owned());
                action_states.insert(action_key.to_owned(), "confirmation_required".into());
            }
            "terms_requalification_required" => {
                if actor_role != "policy_evaluator"
                    || authority != "valid"
                    || !contains(&event["evidence_types"], "terms_requalification_reason")
                {
                    errors.push(format!("{event_path}: terms requalification lacks policy-evaluator evidence"));
                }
                human_required_seen.insert(action_key.to_owned());
                action_states.insert(action_key.to_owned(), "terms_requalification_required".into());
            }
            "human_confirmation_accepted" => {
                if actor_role != "principal" || actor_ref != principal || authority != "valid"
                    || str_at(event, "presentation_digest") != expected_digest
                    || !contains(&event["evidence_types"], "user_confirmation_receipt")
                    || !contains(&event["evidence_types"], "material_terms_presented")
                {
                    errors.push(format!("{event_path}: confirmation is not principal-bound to presented exact terms"));
                } else {
                    human_confirmed.insert(action_key.to_owned());
                }
            }
            "action_rejected" => {
                if actor_role != "policy_evaluator" || authority != "valid"
                    || str_at(event, "reason_code").is_none()
                {
                    errors.push(format!("{event_path}: policy rejection lacks evaluator/reason"));
                }
                rejected_seen.insert(action_key.to_owned());
                action_states.insert(action_key.to_owned(), "rejected".into());
            }
            "reservation_rejected" => {
                if actor_role != "budget_coordinator" || authority != "valid"
                    || str_at(event, "reason_code").is_none()
                {
                    errors.push(format!("{event_path}: reservation rejection lacks coordinator/reason"));
                }
                let decision = action_decision(
                    action,
                    mandate,
                    profile,
                    &leaf,
                    source_ids,
                    human_confirmed.contains(action_key),
                );
                if decision != Decision::Eligible || revocation_pending || revoked {
                    errors.push(format!("{event_path}: reservation rejected for a reason other than an eligible action's budget/concurrency limit"));
                } else {
                    let Some(cost) = u64_at(action, "all_in_budget_minor_units") else {
                        errors.push(format!("{event_path}: reservation rejection has unknown cost"));
                        continue;
                    };
                    let current_active = outstanding;
                    let can_fit = reservation_count < u64_at(mandate, "max_transaction_count").unwrap_or(0)
                        && current_active < u64_at(mandate, "max_concurrent_reservations").unwrap_or(0)
                        && used.checked_add(cost).is_some_and(|value| {
                            value <= u64_at(mandate, "max_total_debit_minor_units").unwrap_or(0)
                        })
                        && period_used.checked_add(cost).is_some_and(|value| {
                            value <= u64_at(mandate, "max_period_debit_minor_units").unwrap_or(0)
                        });
                    let agent_current = agent_used.get(str_at(action, "agent_ref").unwrap_or(""))
                        .copied().unwrap_or(0);
                    let agent_limit = if leaf.is_delegated {
                        u64_at(&leaf.scope, "max_total_debit_minor_units").unwrap_or(0)
                    } else {
                        u64_at(mandate, "max_total_debit_minor_units").unwrap_or(0)
                    };
                    if can_fit && agent_current.checked_add(cost).is_some_and(|value| value <= agent_limit) {
                        errors.push(format!("{event_path}: false budget/concurrency rejection"));
                    }
                }
                rejected_seen.insert(action_key.to_owned());
                action_states.insert(action_key.to_owned(), "rejected".into());
            }
            "reservation_created" => {
                if actor_role != "budget_coordinator" || actor_ref != "system:budget-coordinator"
                    || authority != "valid"
                    || !contains(&event["evidence_types"], "atomic_budget_reservation_receipt")
                {
                    errors.push(format!("{event_path}: reservation lacks budget-coordinator authority/evidence"));
                }
                if revocation_pending || revoked {
                    errors.push(format!("{event_path}: reservation after revocation request/effect"));
                }
                if !proposal_seen.contains(action_key) {
                    errors.push(format!("{event_path}: reservation precedes action proposal"));
                }
                let decision = action_decision(
                    action,
                    mandate,
                    profile,
                    &leaf,
                    source_ids,
                    human_confirmed.contains(action_key),
                );
                if decision != Decision::Eligible {
                    errors.push(format!("{event_path}: candidate is outside mandate or lacks required human confirmation/known terms"));
                }
                if !human_required_seen.contains(action_key) {
                    let amount_for_threshold = u64_at(action, "all_in_budget_minor_units").unwrap_or(0);
                    let needs = action.get("is_new_beneficiary").and_then(Value::as_bool) == Some(true)
                        && profile.get("requires_human_confirmation_for_new_beneficiary").and_then(Value::as_bool) == Some(true)
                        || amount_for_threshold >= u64_at(profile, "human_confirmation_threshold_minor_units").unwrap_or(u64::MAX).min(
                            u64_at(mandate, "human_confirmation_threshold_minor_units").unwrap_or(u64::MAX)
                        );
                    if needs && !human_confirmed.contains(action_key) {
                        errors.push(format!("{event_path}: threshold/new-beneficiary action was not confirmed"));
                    }
                }
                let Some(cost) = u64_at(action, "all_in_budget_minor_units") else {
                    errors.push(format!("{event_path}: reservation amount unknown"));
                    continue;
                };
                if amount != cost {
                    errors.push(format!("{event_path}: reserved amount differs from all-in budget debit"));
                }
                let Some(res_id) = reservation_id else {
                    errors.push(format!("{event_path}: reservation identity missing"));
                    continue;
                };
                if reservations.contains_key(res_id) || action_reservation.contains_key(action_key) {
                    errors.push(format!("{event_path}: duplicate reservation or second reservation for one action"));
                    continue;
                }
                let agent_ref = str_at(action, "agent_ref").unwrap_or("").to_owned();
                let active_ok = outstanding < u64_at(mandate, "max_concurrent_reservations").unwrap_or(0);
                let count_ok = reservation_count < u64_at(mandate, "max_transaction_count").unwrap_or(0);
                let total_ok = used.checked_add(cost).is_some_and(|value| {
                    value <= u64_at(mandate, "max_total_debit_minor_units").unwrap_or(0)
                });
                let period_ok = period_used.checked_add(cost).is_some_and(|value| {
                    value <= u64_at(mandate, "max_period_debit_minor_units").unwrap_or(0)
                });
                let agent_current = agent_used.get(&agent_ref).copied().unwrap_or(0);
                let agent_limit = if leaf.is_delegated {
                    u64_at(&leaf.scope, "max_total_debit_minor_units").unwrap_or(0)
                } else {
                    u64_at(mandate, "max_total_debit_minor_units").unwrap_or(0)
                };
                let agent_ok = agent_current.checked_add(cost).is_some_and(|value| value <= agent_limit);
                if !active_ok || !count_ok || !total_ok || !period_ok || !agent_ok {
                    errors.push(format!("{event_path}: reservation exceeds a total, period, count, agent or concurrency budget"));
                    continue;
                }
                used = used.checked_add(cost).unwrap_or(u64::MAX);
                period_used = period_used.checked_add(cost).unwrap_or(u64::MAX);
                agent_used.insert(agent_ref.clone(), agent_current.checked_add(cost).unwrap_or(u64::MAX));
                outstanding += 1;
                reservation_count += 1;
                reservations.insert(
                    res_id.to_owned(),
                    Reservation {
                        action_id: action_key.to_owned(),
                        agent_ref,
                        amount: cost,
                        state: "reserved".into(),
                    },
                );
                action_reservation.insert(action_key.to_owned(), res_id.to_owned());
                action_states.insert(action_key.to_owned(), "reserved".into());
            }
            "duplicate_reservation_rejected" => {
                let Some(res_id) = reservation_id else {
                    errors.push(format!("{event_path}: duplicate reservation reference missing"));
                    continue;
                };
                if actor_role != "budget_coordinator"
                    || authority != "valid"
                    || reservations.get(res_id).is_none_or(|reservation| reservation.action_id != action_key)
                    || !contains(&event["evidence_types"], "duplicate_reservation_receipt")
                {
                    errors.push(format!("{event_path}: duplicate reservation rejection is not bound to the original reservation"));
                }
            }
            "payment_dispatched" => {
                let Some(res_id) = reservation_id else {
                    errors.push(format!("{event_path}: dispatch reservation reference missing"));
                    continue;
                };
                if actor_role != "payment_dispatcher" || authority != "valid" {
                    errors.push(format!("{event_path}: dispatch actor/authority invalid"));
                }
                if revocation_pending || revoked {
                    errors.push(format!("{event_path}: dispatch after revocation request/effect"));
                }
                match reservations.get_mut(res_id) {
                    Some(reservation)
                        if reservation.action_id == action_key && reservation.state == "reserved" =>
                    {
                        reservation.state = "dispatched".into();
                        action_states.insert(action_key.to_owned(), "dispatched".into());
                    }
                    _ => errors.push(format!("{event_path}: dispatch lacks one active prior reservation")),
                }
            }
            "provider_outcome_unknown" => {
                let Some(res_id) = reservation_id else {
                    errors.push(format!("{event_path}: unknown outcome reservation reference missing"));
                    continue;
                };
                if actor_role != "payment_provider" || authority != "unknown" {
                    errors.push(format!("{event_path}: unknown outcome must be provider-sourced and unknown"));
                }
                match reservations.get_mut(res_id) {
                    Some(reservation)
                        if reservation.action_id == action_key && reservation.state == "dispatched" =>
                    {
                        reservation.state = "outcome_unknown".into();
                        action_states.insert(action_key.to_owned(), "outcome_unknown".into());
                    }
                    _ => errors.push(format!("{event_path}: unknown outcome lacks prior dispatch")),
                }
            }
            "payment_effect_confirmed" => {
                let Some(res_id) = reservation_id else {
                    errors.push(format!("{event_path}: effect confirmation reservation reference missing"));
                    continue;
                };
                if actor_role != "reconciliation_verifier"
                    || authority != "valid"
                    || !contains(&event["evidence_types"], "independent_recipient_available_receipt")
                    || !contains(&event["evidence_types"], "independent_account_reconciliation")
                {
                    errors.push(format!("{event_path}: payment effect lacks independent evidence"));
                }
                match reservations.get_mut(res_id) {
                    Some(reservation)
                        if reservation.action_id == action_key
                            && matches!(reservation.state.as_str(), "dispatched" | "outcome_unknown") =>
                    {
                        reservation.state = "consumed".into();
                        outstanding = outstanding.saturating_sub(1);
                        action_states.insert(action_key.to_owned(), "consumed".into());
                    }
                    _ => errors.push(format!("{event_path}: effect confirmation lacks dispatched/unknown reservation")),
                }
            }
            "payment_effect_reconciled_no_effect" => {
                let Some(res_id) = reservation_id else {
                    errors.push(format!("{event_path}: no-effect reservation reference missing"));
                    continue;
                };
                if actor_role != "reconciliation_verifier"
                    || authority != "valid"
                    || !contains(&event["evidence_types"], "independent_no_effect_reconciliation")
                {
                    errors.push(format!("{event_path}: reservation release lacks independent no-effect reconciliation"));
                }
                match reservations.get_mut(res_id) {
                    Some(reservation)
                        if reservation.action_id == action_key
                            && matches!(reservation.state.as_str(), "reserved" | "dispatched" | "outcome_unknown") =>
                    {
                        reservation.state = "released".into();
                        used = match used.checked_sub(reservation.amount) {
                            Some(value) => value,
                            None => {
                                errors.push(format!("{event_path}: budget release underflow"));
                                0
                            }
                        };
                        period_used = match period_used.checked_sub(reservation.amount) {
                            Some(value) => value,
                            None => {
                                errors.push(format!("{event_path}: period budget release underflow"));
                                0
                            }
                        };
                        let agent_value = agent_used.get(&reservation.agent_ref).copied().unwrap_or(0);
                        agent_used.insert(
                            reservation.agent_ref.clone(),
                            agent_value.checked_sub(reservation.amount).unwrap_or(0),
                        );
                        outstanding = outstanding.saturating_sub(1);
                        action_states.insert(action_key.to_owned(), "released".into());
                    }
                    _ => errors.push(format!("{event_path}: no-effect reconciliation lacks active reservation")),
                }
            }
            "payment_returned" => {
                let Some(res_id) = reservation_id else {
                    errors.push(format!("{event_path}: return reservation reference missing"));
                    continue;
                };
                let Some(reference) = str_at(event, "return_reference") else {
                    errors.push(format!("{event_path}: return reference missing"));
                    continue;
                };
                if !return_refs.insert(reference.to_owned()) {
                    errors.push(format!("{event_path}: duplicate return reference"));
                }
                if actor_role != "reconciliation_verifier"
                    || authority != "valid"
                    || !contains(&event["evidence_types"], "independent_return_receipt")
                    || !contains(&event["evidence_types"], "independent_ledger_reconciliation")
                {
                    errors.push(format!("{event_path}: return lacks independent receipt/reconciliation"));
                }
                match reservations.get_mut(res_id) {
                    Some(reservation)
                        if reservation.action_id == action_key
                            && reservation.state == "consumed"
                            && amount > 0
                            && amount <= reservation.amount =>
                    {
                        reservation.state = "returned".into();
                        action_states.insert(action_key.to_owned(), "returned_without_replenishment".into());
                    }
                    _ => errors.push(format!("{event_path}: return lacks consumed original payment or exceeds its amount")),
                }
            }
            "duplicate_return_callback_ignored" => {
                if actor_role != "reconciliation_verifier"
                    || authority != "valid"
                    || !str_at(event, "return_reference").is_some_and(|reference| return_refs.contains(reference))
                    || !contains(&event["evidence_types"], "duplicate_return_callback_receipt")
                {
                    errors.push(format!("{event_path}: duplicate return callback is not linked to a prior return"));
                }
            }
            _ => errors.push(format!("{event_path}: unsupported event type {kind}")),
        }
    }

    if !active {
        errors.push(format!("{prefix}: mandate never activated"));
    }
    for action_id in actions.keys() {
        if !proposal_seen.contains(action_id) {
            errors.push(format!("{prefix}: action {action_id} lacks one proposal event"));
        }
        let Some(action) = actions.get(action_id) else {
            continue;
        };
        let state = action_states.get(action_id).map(String::as_str).unwrap_or("proposed");
        let expected = str_at(action, "expected_disposition").unwrap_or("");
        if state != expected {
            errors.push(format!(
                "{prefix}: action {action_id} expected disposition {expected}, computed {state}"
            ));
        }
        let decision = action_decision(
            action,
            mandate,
            profile,
            &leaf,
            source_ids,
            human_confirmed.contains(action_id),
        );
        if decision == Decision::Rejected && !rejected_seen.contains(action_id) {
            errors.push(format!("{prefix}: rejected candidate {action_id} lacks explicit rejection event"));
        }
        if matches!(decision, Decision::HumanConfirmationRequired | Decision::TermsRequalificationRequired)
            && !human_required_seen.contains(action_id)
            && !action_reservation.contains_key(action_id)
        {
            errors.push(format!("{prefix}: action {action_id} requires confirmation/requalification but no pause event exists"));
        }
        if decision == Decision::Eligible
            && !action_reservation.contains_key(action_id)
            && !rejected_seen.contains(action_id)
            && !human_required_seen.contains(action_id)
        {
            errors.push(format!("{prefix}: eligible action {action_id} has no reservation or explicit disposition"));
        }
    }

    let computed_outstanding = reservations
        .values()
        .filter(|reservation| matches!(reservation.state.as_str(), "reserved" | "dispatched" | "outcome_unknown"))
        .count() as u64;
    if computed_outstanding != outstanding {
        errors.push(format!("{prefix}: outstanding reservation accounting diverged"));
    }
    if u64_at(scenario, "expected_budget_used_minor_units") != Some(used) {
        errors.push(format!("{prefix}: expected budget-used total differs from reservation accounting"));
    }
    if u64_at(scenario, "expected_outstanding_reservations") != Some(outstanding) {
        errors.push(format!("{prefix}: expected outstanding-reservation count differs"));
    }
    errors
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some(EXPECTED_SCHEMA) {
        errors.push("corpus schema identity mismatch".into());
    }
    if str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS) {
        errors.push("corpus identity mismatch".into());
    }
    if str_at(corpus, "status") != Some("synthetic_reference_only") {
        errors.push("corpus must remain synthetic_reference_only".into());
    }
    if corpus.get("external_effects_enabled").and_then(Value::as_bool) != Some(false) {
        errors.push("external_effects_enabled must be false".into());
    }
    let as_of = str_at(corpus, "as_of").unwrap_or("");
    if as_of.is_empty() {
        errors.push("as_of is missing".into());
    }
    let sources = array_at(corpus, "source_register").cloned().unwrap_or_default();
    let profiles = array_at(corpus, "profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    check_unique(&sources, "id", "source_register", &mut errors);
    check_unique(&profiles, "profile_id", "profiles", &mut errors);
    check_unique(&scenarios, "scenario_id", "scenarios", &mut errors);
    source_dates(corpus, as_of, &mut errors);

    let source_ids = source_ids(corpus);
    let profile_lookup = profile_map(corpus);
    for (index, profile) in profiles.iter().enumerate() {
        if !refs_resolve(&profile["source_refs"], &source_ids) {
            errors.push(format!("profiles[{index}]: source reference missing or unresolved"));
        }
        let status = str_at(profile, "status").unwrap_or("");
        let kind = str_at(profile, "profile_kind").unwrap_or("");
        if status != "synthetic_reference_only" || kind != "synthetic_research_policy" {
            errors.push(format!(
                "profiles[{index}]: comparator or non-synthetic profile cannot authorize this synthetic mandate corpus"
            ));
        }
    }
    for (index, scenario) in scenarios.iter().enumerate() {
        let profile_id = str_at(&scenario["mandate"], "profile_id").unwrap_or("");
        let Some(profile) = profile_lookup.get(profile_id) else {
            errors.push(format!("scenarios[{index}]: unknown mandate profile"));
            continue;
        };
        errors.extend(semantic_scenario(scenario, profile, &source_ids, index));
    }
    errors
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Vec<String>, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let mut errors = semantic_errors(&corpus);
    errors.extend(schema_errors(&schema, &corpus));
    Ok((errors, corpus, schema))
}

fn mutate_and_reject(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let mut errors = semantic_errors(&changed);
    errors.extend(schema_errors(schema, &changed));
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let mut baseline = semantic_errors(corpus);
    baseline.extend(schema_errors(schema, corpus));
    if !baseline.is_empty() {
        return Err(format!("baseline corpus failed before mutations: {baseline:?}"));
    }
    let mut count = 0usize;

    mutate_and_reject("external effects enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("agent self-approves mandate", corpus, schema, |value| {
        value["scenarios"][0]["mandate"]["approval"]["approver_ref"] =
            value["scenarios"][0]["mandate"]["agent_ref"].clone();
    })?;
    count += 1;

    mutate_and_reject("QIE digest detached from approval", corpus, schema, |value| {
        value["scenarios"][0]["mandate"]["qualified_intent_envelope_digest"] = json!(
            "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
        );
    })?;
    count += 1;

    mutate_and_reject("single-action amount overrun", corpus, schema, |value| {
        value["scenarios"][0]["actions"][0]["amount_minor_units"] = json!(11000);
        value["scenarios"][0]["actions"][0]["budget_equivalent_principal_minor_units"] = json!(11000);
        value["scenarios"][0]["actions"][0]["all_in_budget_minor_units"] = json!(11010);
        value["scenarios"][0]["events"][2]["amount_minor_units"] = json!(11010);
    })?;
    count += 1;

    mutate_and_reject("purpose widened by action", corpus, schema, |value| {
        value["scenarios"][0]["actions"][0]["purpose"] = json!("personal_transfer");
    })?;
    count += 1;

    mutate_and_reject("beneficiary substituted", corpus, schema, |value| {
        value["scenarios"][0]["actions"][0]["beneficiary_ref"] = json!("merchant-evil");
    })?;
    count += 1;

    mutate_and_reject("unknown fee treated as reservable", corpus, schema, |value| {
        value["scenarios"][0]["actions"][0]["fee_state"] = json!("unknown");
        value["scenarios"][0]["actions"][0]["fee_minor_units"] = Value::Null;
        value["scenarios"][0]["actions"][0]["all_in_budget_minor_units"] = Value::Null;
    })?;
    count += 1;

    mutate_and_reject("all-in fee arithmetic changed", corpus, schema, |value| {
        value["scenarios"][0]["actions"][0]["fee_minor_units"] = json!(100);
    })?;
    count += 1;

    mutate_and_reject("duplicate reservation replay", corpus, schema, |value| {
        let mut duplicate = value["scenarios"][0]["events"][2].clone();
        duplicate["event_id"] = json!("EV-MUTATION-DUPLICATE-RESERVATION");
        duplicate["sequence"] = json!(6);
        duplicate["observed_at"] = json!("2026-11-06T10:00:06Z");
        value["scenarios"][0]["events"].as_array_mut().unwrap().push(duplicate);
    })?;
    count += 1;

    mutate_and_reject("budget cap lowered below existing reservations", corpus, schema, |value| {
        value["scenarios"][0]["mandate"]["max_total_debit_minor_units"] = json!(4000);
    })?;
    count += 1;

    mutate_and_reject("mandate expires before action", corpus, schema, |value| {
        value["scenarios"][0]["mandate"]["expires_at"] = json!("2026-11-06T10:00:01Z");
    })?;
    count += 1;

    mutate_and_reject("human confirmation presentation mismatch", corpus, schema, |value| {
        value["scenarios"][10]["events"][2]["presentation_digest"] =
            json!("sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
    })?;
    count += 1;

    mutate_and_reject("agent self-confirms high-value action", corpus, schema, |value| {
        value["scenarios"][10]["events"][2]["actor_role"] = json!("agent");
        value["scenarios"][10]["events"][2]["actor_ref"] = json!("principal:agent-001");
    })?;
    count += 1;

    mutate_and_reject("effective revocation before reservation", corpus, schema, |value| {
        value["scenarios"][0]["events"].as_array_mut().unwrap().insert(
            2,
            json!({
                "event_id":"EV-MUTATION-REVOKED",
                "sequence":3,
                "event_type":"mandate_revoked",
                "observed_at":"2026-11-06T10:00:03Z",
                "actor_role":"authority_service",
                "actor_ref":"system:authority-service",
                "authority_state":"valid",
                "action_id":null,
                "reservation_id":null,
                "amount_minor_units":0,
                "exact_action_request_digest":null,
                "presentation_digest":null,
                "evidence_types":["qualified_revocation_receipt"],
                "reason_code":null,
                "return_reference":null
            })
        );
        for (index, event) in value["scenarios"][0]["events"].as_array_mut().unwrap().iter_mut().enumerate() {
            event["sequence"] = json!((index + 1) as u64);
            event["observed_at"] = json!(format!("2026-11-06T10:00:{:02}Z", index + 1));
        }
    })?;
    count += 1;

    mutate_and_reject("no-effect release without reconciliation evidence", corpus, schema, |value| {
        value["scenarios"][16]["events"][5]["evidence_types"] = json!(["provider_timeout_receipt"]);
    })?;
    count += 1;

    mutate_and_reject("return replenishes authorization budget", corpus, schema, |value| {
        value["scenarios"][17]["expected_budget_used_minor_units"] = json!(0);
    })?;
    count += 1;

    mutate_and_reject("sub-agent widens parent single-spend scope", corpus, schema, |value| {
        value["scenarios"][21]["delegation_chain"][0]["scope"]["max_single_debit_minor_units"] =
            json!(20000);
    })?;
    count += 1;

    mutate_and_reject("sub-agent adds an unapproved beneficiary", corpus, schema, |value| {
        value["scenarios"][21]["delegation_chain"][0]["scope"]["allowed_beneficiary_refs"]
            .as_array_mut().unwrap().push(json!("merchant-evil"));
    })?;
    count += 1;

    mutate_and_reject("action digest detached from exact ActionRequest", corpus, schema, |value| {
        value["scenarios"][0]["events"][2]["exact_action_request_digest"] =
            json!("sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff");
    })?;
    count += 1;

    mutate_and_reject("unknown FX quote treated as known", corpus, schema, |value| {
        value["scenarios"][22]["actions"][0]["fx_quote"]["expires_at"] =
            json!("2026-11-06T09:00:00Z");
    })?;
    count += 1;

    mutate_and_reject("concurrent reservation bypass", corpus, schema, |value| {
        value["scenarios"][14]["mandate"]["max_concurrent_reservations"] = json!(2);
        value["scenarios"][14]["events"][6]["event_type"] = json!("reservation_created");
        value["scenarios"][14]["events"][6]["reservation_id"] = json!("RSV-MUTATION-SECOND");
        value["scenarios"][14]["events"][6]["amount_minor_units"] = json!(5010);
        value["scenarios"][14]["events"][6]["evidence_types"] =
            json!(["atomic_budget_reservation_receipt"]);
    })?;
    count += 1;

    mutate_and_reject("sub-agent depth exceeds parent", corpus, schema, |value| {
        value["scenarios"][21]["delegation_chain"][0]["depth"] = json!(2);
    })?;
    count += 1;

    mutate_and_reject("provider timeout releases budget", corpus, schema, |value| {
        value["scenarios"][15]["events"][4]["event_type"] =
            json!("payment_effect_reconciled_no_effect");
        value["scenarios"][15]["events"][4]["actor_role"] = json!("reconciliation_verifier");
        value["scenarios"][15]["events"][4]["authority_state"] = json!("valid");
        value["scenarios"][15]["events"][4]["evidence_types"] =
            json!(["provider_timeout_receipt"]);
        value["scenarios"][15]["expected_budget_used_minor_units"] = json!(0);
        value["scenarios"][15]["expected_outstanding_reservations"] = json!(0);
        value["scenarios"][15]["actions"][0]["expected_disposition"] = json!("released");
    })?;
    count += 1;

    mutate_and_reject("human confirmation cannot cure unknown fees", corpus, schema, |value| {
        value["scenarios"][6]["events"][2]["event_type"] = json!("human_confirmation_required");
        value["scenarios"][6]["events"][2]["evidence_types"] =
            json!(["confirmation_threshold_or_new_beneficiary"]);
        value["scenarios"][6]["actions"][0]["expected_disposition"] = json!("confirmation_required");
    })?;
    count += 1;

    mutate_and_reject("comparator profile cannot authorize policy", corpus, schema, |value| {
        value["scenarios"][0]["mandate"]["profile_id"] =
            json!("AP2-FIDO-HNP-COMPARATOR-V02");
    })?;
    count += 1;

    mutate_and_reject("human-present profile cannot silently run unattended", corpus, schema, |value| {
        value["profiles"][0]["allows_human_not_present"] = json!(false);
    })?;
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
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--corpus requires a path")
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--schema requires a path")
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!("agentic-payment-delegation-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
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
    let (errors, corpus, schema) = evaluate(&corpus_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let negative_controls = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let report = json!({
        "evaluator": "mycelix.agentic-payment-delegation-evaluator.v1",
        "status": if errors.is_empty() { "PASS" } else { "FAIL" },
        "external_effects_enabled": false,
        "claim_ceiling": "synthetic mandate, budget, revocation, delegation and reservation conformance only",
        "scenario_count": array_at(&corpus, "scenarios").map_or(0, Vec::len),
        "negative_controls": negative_controls,
        "errors": errors
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !errors.is_empty() {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_in_schema_and_corpus_validate() {
        let (errors, corpus, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 10);
        assert_eq!(array_at(&corpus, "profiles").unwrap().len(), 4);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 30);
    }

    #[test]
    fn deterministic_negative_mutation_campaign_rejects_authority_and_budget_attacks() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 26);
    }
}
