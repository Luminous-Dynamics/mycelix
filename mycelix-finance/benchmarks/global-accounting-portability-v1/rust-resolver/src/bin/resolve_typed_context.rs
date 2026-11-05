use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

type AppResult<T> = Result<T, String>;

fn main() {
    if let Err(error) = run_cli() {
        eprintln!("mycelix-typed-context-resolver: {error}");
        std::process::exit(2);
    }
}

fn run_cli() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let typed_dir = root.join("typed-context-v1");
    let context_dir = root.join("../applicability-context");
    let mut bundle_path = context_dir.join("fixtures/jurisdiction_context_candidates_v1.0.0.json");
    let mut context_schema_path = context_dir.join("jurisdiction_context_bundle_v1.0.0.schema.json");
    let mut request_path = typed_dir.join("fixtures/typed_context_requests_v1.0.0.json");
    let mut request_schema_path = typed_dir.join("typed_context_request_v1.0.0.schema.json");
    let mut report_schema_path = typed_dir.join("typed_context_report_v1.0.0.schema.json");
    let mut request_index = 0usize;
    let mut output_path: Option<PathBuf> = None;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--context-bundle" => bundle_path = PathBuf::from(next_arg(&mut args, "--context-bundle")?),
            "--context-schema" => context_schema_path = PathBuf::from(next_arg(&mut args, "--context-schema")?),
            "--request" => request_path = PathBuf::from(next_arg(&mut args, "--request")?),
            "--request-schema" => request_schema_path = PathBuf::from(next_arg(&mut args, "--request-schema")?),
            "--report-schema" => report_schema_path = PathBuf::from(next_arg(&mut args, "--report-schema")?),
            "--request-index" => {
                request_index = next_arg(&mut args, "--request-index")?.parse()?;
            }
            "--out" => output_path = Some(PathBuf::from(next_arg(&mut args, "--out")?)),
            "--help" | "-h" => {
                print_help();
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}; use --help").into()),
        }
    }

    let bundle_bytes = fs::read(&bundle_path)?;
    let context_schema_bytes = fs::read(&context_schema_path)?;
    let request_file_bytes = fs::read(&request_path)?;
    let request_schema_bytes = fs::read(&request_schema_path)?;
    let report_schema_bytes = fs::read(&report_schema_path)?;

    let bundle: Value = serde_json::from_slice(&bundle_bytes)?;
    let context_schema: Value = serde_json::from_slice(&context_schema_bytes)?;
    let request_file: Value = serde_json::from_slice(&request_file_bytes)?;
    let request_schema: Value = serde_json::from_slice(&request_schema_bytes)?;
    let report_schema: Value = serde_json::from_slice(&report_schema_bytes)?;

    validate_instance("context bundle", &context_schema, &bundle)?;
    validate_context_integrity(&bundle)?;
    validate_schema("request schema", &request_schema)?;
    validate_schema("report schema", &report_schema)?;
    let request = select_request(&request_file, request_index)?;
    validate_instance("typed context request", &request_schema, request)?;

    let report = resolve(
        &bundle,
        &bundle_bytes,
        &context_schema,
        &context_schema_bytes,
        request,
    )?;
    validate_instance("typed context report", &report_schema, &report)?;
    let rendered = serde_json::to_string_pretty(&report)?;
    match output_path {
        Some(path) => fs::write(path, format!("{rendered}\n"))?,
        None => println!("{rendered}"),
    }
    Ok(())
}

fn next_arg(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, Box<dyn Error>> {
    args.next().ok_or_else(|| format!("{flag} requires a value").into())
}

fn print_help() {
    println!(
        "Mycelix Global Typed Context Resolver v1.0.0\n\n\
         --context-bundle PATH  Exact context-bundle JSON bytes\n\
         --context-schema PATH  Draft 2020-12 context-bundle schema\n\
         --request PATH         Request object or fixture bundle containing requests[]\n\
         --request-index N      Select request N from a request bundle (default: 0)\n\
         --request-schema PATH  Draft 2020-12 request schema\n\
         --report-schema PATH   Draft 2020-12 report schema\n\
         --out PATH             Write JSON report to PATH (default: stdout)\n\
         --help                 Show this help"
    );
}

fn select_request(file: &Value, index: usize) -> AppResult<&Value> {
    match file.get("requests").and_then(Value::as_array) {
        Some(requests) => requests
            .get(index)
            .ok_or_else(|| format!("request index {index} is outside {} fixture requests", requests.len())),
        None if index == 0 => Ok(file),
        None => Err("--request-index must be zero when --request contains one request object".to_owned()),
    }
}

fn validate_schema(label: &str, schema: &Value) -> AppResult<()> {
    draft202012::meta::validate(schema)
        .map_err(|error| format!("{label}: invalid Draft 2020-12 schema: {error}"))?;
    draft202012::options()
        .build(schema)
        .map(|_| ())
        .map_err(|error| format!("{label}: could not compile Draft 2020-12 schema: {error}"))
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> AppResult<()> {
    validate_schema(label, schema)?;
    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("{label}: could not compile Draft 2020-12 schema: {error}"))?;
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

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    hex::encode(digest)
}

fn array<'a>(value: &'a Value, key: &str) -> AppResult<&'a [Value]> {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| format!("missing or invalid array: {key}"))
}

fn string<'a>(value: &'a Value, key: &str) -> AppResult<&'a str> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("missing or invalid string: {key}"))
}

fn nullable_string<'a>(value: &'a Value, key: &str) -> AppResult<Option<&'a str>> {
    match value.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(text)) => Ok(Some(text)),
        _ => Err(format!("missing or invalid nullable string: {key}")),
    }
}

fn validate_context_integrity(bundle: &Value) -> AppResult<()> {
    let mut contexts = BTreeSet::new();
    for context in array(bundle, "contexts")? {
        let context_id = string(context, "context_id")?;
        if !contexts.insert(context_id.to_owned()) {
            return Err(format!("duplicate context_id: {context_id}"));
        }

        let mut parties = BTreeMap::<String, BTreeSet<String>>::new();
        for party in array(context, "parties")? {
            let party_ref = string(party, "party_ref")?.to_owned();
            if parties.contains_key(&party_ref) {
                return Err(format!("{context_id}: duplicate party_ref {party_ref}"));
            }
            let roles = array(party, "transaction_roles")?
                .iter()
                .map(|value| value.as_str().map(str::to_owned)
                    .ok_or_else(|| format!("{context_id}: party role must be a string")))
                .collect::<Result<BTreeSet<_>, _>>()?;
            parties.insert(party_ref, roles);
        }

        let mut scopes = BTreeMap::<String, &Value>::new();
        let mut scope_identities = BTreeSet::new();
        for scope in array(context, "scope_registry")? {
            let id = string(scope, "scope_id")?.to_owned();
            if scopes.insert(id.clone(), scope).is_some() {
                return Err(format!("{context_id}: duplicate scope_id {id}"));
            }
            let identity = (
                string(scope, "namespace_uri")?.to_owned(),
                string(scope, "code")?.to_owned(),
                string(scope, "scope_level")?.to_owned(),
                scope.get("parent_scope_ref").and_then(Value::as_str).map(str::to_owned),
            );
            if !scope_identities.insert(identity) {
                return Err(format!("{context_id}: duplicate canonical scope identity for {id}"));
            }
        }
        for (id, scope) in &scopes {
            if let Some(parent) = scope.get("parent_scope_ref").and_then(Value::as_str) {
                if !scopes.contains_key(parent) {
                    return Err(format!("{context_id}: scope {id} references unknown parent {parent}"));
                }
            }
        }
        for id in scopes.keys() {
            let mut visited = BTreeSet::new();
            let mut current = Some(id.as_str());
            while let Some(scope_id) = current {
                if !visited.insert(scope_id.to_owned()) {
                    return Err(format!("{context_id}: jurisdiction scope hierarchy cycle at {scope_id}"));
                }
                current = scopes.get(scope_id)
                    .and_then(|scope| scope.get("parent_scope_ref"))
                    .and_then(Value::as_str);
            }
        }

        let mut facts = BTreeMap::<String, &Value>::new();
        for fact in array(context, "jurisdiction_facts")? {
            let id = string(fact, "fact_id")?.to_owned();
            if facts.insert(id.clone(), fact).is_some() {
                return Err(format!("{context_id}: duplicate fact_id {id}"));
            }
        }
        for (fact_id, fact) in &facts {
            let kind = string(fact, "subject_kind")?;
            let subject = string(fact, "subject_ref")?;
            if kind == "party" {
                let role = string(fact, "subject_party_role")?;
                if !parties.get(subject).is_some_and(|roles| roles.contains(role)) {
                    return Err(format!("{context_id}: invalid party subject/role on {fact_id}"));
                }
            } else if fact.get("subject_party_role").is_some() {
                return Err(format!("{context_id}: non-party fact {fact_id} has subject_party_role"));
            }
            if kind == "transaction" && subject != string(context, "transaction_ref")? {
                return Err(format!("{context_id}: transaction fact {fact_id} references a different transaction"));
            }

            let role = string(fact, "jurisdiction_role")?;
            if role == "place_of_supply" && kind != "transaction" {
                return Err(format!("{context_id}: place_of_supply fact {fact_id} is not transaction-scoped"));
            }
            if role == "establishment_location" && kind != "establishment" {
                return Err(format!("{context_id}: establishment_location fact {fact_id} has wrong subject kind"));
            }
            if matches!(role, "ship_from" | "ship_to") && kind != "shipment" {
                return Err(format!("{context_id}: shipment fact {fact_id} has wrong subject kind"));
            }

            if let Some(bindings) = fact.get("party_role_bindings").and_then(Value::as_array) {
                let mut unique_bindings = BTreeSet::new();
                for binding in bindings {
                    let party_ref = string(binding, "party_ref")?;
                    let party_role = string(binding, "transaction_role")?;
                    if !parties.get(party_ref).is_some_and(|roles| roles.contains(party_role)) {
                        return Err(format!("{context_id}: invalid party-role binding on {fact_id}"));
                    }
                    if !unique_bindings.insert((party_ref.to_owned(), party_role.to_owned())) {
                        return Err(format!("{context_id}: duplicate party-role binding on {fact_id}"));
                    }
                }
            }
            if let Some(scope_ref) = fact.get("scope_ref").and_then(Value::as_str) {
                if !scopes.contains_key(scope_ref) {
                    return Err(format!("{context_id}: fact {fact_id} references unknown scope {scope_ref}"));
                }
            }
            let valid_from = string(fact, "valid_from")?;
            if let Some(valid_to) = fact.get("valid_to").and_then(Value::as_str) {
                if valid_to <= valid_from {
                    return Err(format!("{context_id}: invalid valid-time interval on {fact_id}"));
                }
            }
            if string(fact, "assertion_state")? == "evidence_supported"
                && array(fact, "source_evidence_refs")?.is_empty()
            {
                return Err(format!("{context_id}: evidence-supported fact {fact_id} has no source evidence refs"));
            }
        }
        for (fact_id, fact) in &facts {
            for target in fact.get("conflicts_with_fact_refs").and_then(Value::as_array).into_iter().flatten() {
                let target_id = target.as_str().ok_or_else(|| format!("{context_id}: non-string conflict ref on {fact_id}"))?;
                let target_fact = facts.get(target_id)
                    .ok_or_else(|| format!("{context_id}: dangling conflict ref {fact_id} -> {target_id}"))?;
                let reciprocal = target_fact.get("conflicts_with_fact_refs")
                    .and_then(Value::as_array)
                    .is_some_and(|refs| refs.iter().any(|item| item.as_str() == Some(fact_id)));
                if !reciprocal {
                    return Err(format!("{context_id}: non-reciprocal conflict ref {fact_id} -> {target_id}"));
                }
            }
        }
    }
    Ok(())
}

fn binding_set(value: &Value, key: &str) -> AppResult<BTreeSet<(String, String)>> {
    let mut result = BTreeSet::new();
    let Some(values) = value.get(key) else {
        return Ok(result);
    };
    let values = values.as_array().ok_or_else(|| format!("{key} must be an array when present"))?;
    for binding in values {
        let pair = (
            string(binding, "party_ref")?.to_owned(),
            string(binding, "transaction_role")?.to_owned(),
        );
        if !result.insert(pair) {
            return Err(format!("duplicate party-role binding in {key}"));
        }
    }
    Ok(result)
}

fn scope_identity(bundle_context: &Value, scope_ref: &str) -> AppResult<Value> {
    let scope = array(bundle_context, "scope_registry")?
        .iter()
        .find(|scope| scope.get("scope_id").and_then(Value::as_str) == Some(scope_ref))
        .ok_or_else(|| format!("unknown scope_ref: {scope_ref}"))?;
    Ok(json!({
        "scope_id": string(scope, "scope_id")?,
        "namespace_uri": string(scope, "namespace_uri")?,
        "code": string(scope, "code")?,
        "scope_level": string(scope, "scope_level")?,
        "parent_scope_ref": scope.get("parent_scope_ref").cloned().unwrap_or(Value::Null)
    }))
}

fn semantic_subject_match(fact: &Value, requirement: &Value) -> AppResult<bool> {
    if fact.get("subject_kind").and_then(Value::as_str)
        != requirement.get("subject_kind").and_then(Value::as_str)
        || fact.get("subject_ref").and_then(Value::as_str)
        != requirement.get("subject_ref").and_then(Value::as_str)
        || fact.get("jurisdiction_role").and_then(Value::as_str)
        != requirement.get("jurisdiction_role").and_then(Value::as_str)
    {
        return Ok(false);
    }

    let fact_party_role = nullable_string(fact, "subject_party_role")?;
    let required_party_role = nullable_string(requirement, "subject_party_role")?;
    if fact_party_role != required_party_role {
        return Ok(false);
    }
    Ok(binding_set(fact, "party_role_bindings")? == binding_set(requirement, "party_role_bindings")?)
}

fn temporal_match(fact: &Value, valid_at: &str, knowledge_at: &str) -> AppResult<bool> {
    let recorded_at = string(fact, "recorded_at")?;
    let valid_from = string(fact, "valid_from")?;
    let valid_to = nullable_string(fact, "valid_to")?;
    Ok(recorded_at <= knowledge_at
        && valid_from <= valid_at
        && valid_to.map(|end| valid_at < end).unwrap_or(true))
}

#[derive(Debug)]
struct RequirementResult {
    json: Value,
    disposition: String,
    reasons: BTreeSet<String>,
}

fn evaluate_requirement(
    context: &Value,
    requirement: &Value,
    valid_at: &str,
    knowledge_at: &str,
) -> AppResult<RequirementResult> {
    let requirement_id = string(requirement, "requirement_id")?.to_owned();
    let mut same_subject_role = Vec::<&Value>::new();
    let mut exact_scope = Vec::<&Value>::new();
    let mut unknown_scope = Vec::<&Value>::new();

    for fact in array(context, "jurisdiction_facts")? {
        if !semantic_subject_match(fact, requirement)? {
            continue;
        }
        same_subject_role.push(fact);
        match fact.get("scope_ref").and_then(Value::as_str) {
            Some(scope_ref) => {
                let identity = scope_identity(context, scope_ref)?;
                if identity == requirement["scope_identity"] {
                    exact_scope.push(fact);
                }
            }
            None if fact.get("assertion_state").and_then(Value::as_str) == Some("unknown") => {
                unknown_scope.push(fact);
            }
            None => {}
        }
    }

    let mut candidate_ids = BTreeSet::new();
    let mut matched_ids = BTreeSet::new();
    let mut reasons = BTreeSet::new();
    let mut eligible = Vec::<&Value>::new();
    let mut visible_review = false;
    let mut exact_visible = Vec::<&Value>::new();

    for fact in &exact_scope {
        let fact_id = string(fact, "fact_id")?.to_owned();
        candidate_ids.insert(fact_id.clone());
        if !temporal_match(fact, valid_at, knowledge_at)? {
            reasons.insert("matching_fact_not_visible_at_requested_frontier".to_owned());
            continue;
        }
        exact_visible.push(fact);

        let state = string(fact, "assertion_state")?;
        if state == "disputed" {
            visible_review = true;
            reasons.insert("matching_fact_disputed".to_owned());
            continue;
        }
        if state != "evidence_supported" {
            visible_review = true;
            reasons.insert("matching_fact_not_evidence_supported".to_owned());
            continue;
        }

        let fact_origin = string(fact, "origin")?;
        if let Some(required_origin) = nullable_string(requirement, "required_origin")? {
            if fact_origin != required_origin {
                visible_review = true;
                reasons.insert("fact_origin_mismatch".to_owned());
                continue;
            }
        }
        if let Some(required_derivation) = nullable_string(requirement, "required_derivation_ref")? {
            if fact.get("derivation_ref").and_then(Value::as_str) != Some(required_derivation) {
                visible_review = true;
                reasons.insert("fact_derivation_ref_mismatch".to_owned());
                continue;
            }
        }
        if requirement["require_evidence_refs"] == Value::Bool(true)
            && fact.get("source_evidence_refs").and_then(Value::as_array).is_none_or(Vec::is_empty)
        {
            visible_review = true;
            reasons.insert("required_evidence_refs_missing".to_owned());
            continue;
        }
        if fact.get("conflicts_with_fact_refs").and_then(Value::as_array).is_some_and(|refs| !refs.is_empty()) {
            visible_review = true;
            reasons.insert("matching_fact_has_conflict_links".to_owned());
            continue;
        }
        eligible.push(fact);
    }

    let disposition;
    if !unknown_scope.is_empty() && !eligible.is_empty() {
        visible_review = true;
        reasons.insert("competing_unknown_fact_present".to_owned());
    }
    if !eligible.is_empty() {
        let eligible_ids = eligible.iter()
            .map(|fact| string(fact, "fact_id").map(str::to_owned))
            .collect::<Result<BTreeSet<_>, _>>()?;
        for candidate in &same_subject_role {
            let conflicts = candidate.get("conflicts_with_fact_refs")
                .and_then(Value::as_array)
                .is_some_and(|refs| refs.iter().any(|reference| {
                    reference.as_str().is_some_and(|id| eligible_ids.contains(id))
                }));
            if conflicts {
                visible_review = true;
                reasons.insert("competing_conflicting_fact_present".to_owned());
                candidate_ids.insert(string(candidate, "fact_id")?.to_owned());
            }
        }
    }

    if eligible.len() > 1 {
        disposition = "needs_review";
        reasons.insert("multiple_exact_facts_ambiguous".to_owned());
    } else if visible_review {
        disposition = "needs_review";
    } else if eligible.len() == 1 {
        disposition = "satisfied";
        matched_ids.insert(string(eligible[0], "fact_id")?.to_owned());
        reasons.insert("exact_typed_fact_satisfied".to_owned());
    } else if !exact_visible.is_empty() {
        disposition = "unsupported";
        if reasons.is_empty() {
            reasons.insert("exact_fact_failed_required_origin_or_derivation".to_owned());
        }
    } else if !exact_scope.is_empty() {
        disposition = "indeterminate";
        reasons.insert("matching_fact_not_visible_at_requested_frontier".to_owned());
    } else if !unknown_scope.is_empty() {
        disposition = "indeterminate";
        reasons.insert("required_fact_unknown".to_owned());
        for fact in unknown_scope {
            candidate_ids.insert(string(fact, "fact_id")?.to_owned());
        }
    } else if !same_subject_role.is_empty() {
        disposition = "unsupported";
        reasons.insert("jurisdiction_scope_mismatch".to_owned());
        for fact in same_subject_role {
            candidate_ids.insert(string(fact, "fact_id")?.to_owned());
        }
    } else {
        disposition = "indeterminate";
        reasons.insert("required_typed_fact_missing".to_owned());
    }

    Ok(RequirementResult {
        json: json!({
            "requirement_id": requirement_id,
            "disposition": disposition,
            "matched_fact_ids": matched_ids.into_iter().collect::<Vec<_>>(),
            "candidate_fact_ids": candidate_ids.into_iter().collect::<Vec<_>>(),
            "reason_codes": reasons.iter().cloned().collect::<Vec<_>>()
        }),
        disposition: disposition.to_owned(),
        reasons,
    })
}

fn resolve(
    bundle: &Value,
    bundle_bytes: &[u8],
    context_schema: &Value,
    context_schema_bytes: &[u8],
    request: &Value,
) -> AppResult<Value> {
    let bundle_sha = sha256_hex(bundle_bytes);
    let schema_sha = sha256_hex(context_schema_bytes);
    if string(request, "context_bundle_sha256")? != bundle_sha {
        return Err("context bundle SHA-256 mismatch; frozen request does not bind these exact bytes".to_owned());
    }
    if string(request, "context_schema_sha256")? != schema_sha {
        return Err("context schema SHA-256 mismatch; frozen request does not bind this exact schema".to_owned());
    }
    if string(request, "context_schema_id")? != string(context_schema, "$id")? {
        return Err("context schema $id mismatch".to_owned());
    }
    if string(request, "context_bundle_id")? != string(bundle, "bundle_id")? {
        return Err("context bundle_id mismatch".to_owned());
    }

    let context_id = string(request, "context_id")?;
    let context = array(bundle, "contexts")?
        .iter()
        .find(|context| context.get("context_id").and_then(Value::as_str) == Some(context_id))
        .ok_or_else(|| format!("unknown context_id: {context_id}"))?;
    let valid_at = string(request, "valid_at")?;
    let knowledge_at = string(request, "knowledge_at")?;
    let context_frontier = string(context, "knowledge_at")?;
    let context_period_start = string(context.get("reporting_period").ok_or("missing reporting_period")?, "start")?;
    let context_period_end = string(context.get("reporting_period").ok_or("missing reporting_period")?, "end")?;
    let frontier_insufficient = knowledge_at > context_frontier;
    let valid_outside_period = valid_at < context_period_start || valid_at > context_period_end;

    let requirements = array(request, "requirements")?;
    let mut requirement_ids = BTreeSet::new();
    for requirement in requirements {
        let requirement_id = string(requirement, "requirement_id")?;
        if !requirement_ids.insert(requirement_id.to_owned()) {
            return Err(format!("duplicate requirement_id: {requirement_id}"));
        }
    }

    let mut results = Vec::new();
    let mut overall_reasons = BTreeSet::new();
    let mut counts = BTreeMap::<String, usize>::new();
    for requirement in requirements {
        let mut result = evaluate_requirement(context, requirement, valid_at, knowledge_at)?;
        if frontier_insufficient {
            result.disposition = "indeterminate".to_owned();
            result.reasons.clear();
            result.reasons.insert("context_snapshot_ends_before_requested_knowledge_frontier".to_owned());
            result.json["disposition"] = json!("indeterminate");
            result.json["matched_fact_ids"] = json!([]);
            result.json["reason_codes"] = json!(result.reasons.iter().cloned().collect::<Vec<_>>());
        }
        if valid_outside_period {
            result.disposition = "indeterminate".to_owned();
            result.reasons.clear();
            result.reasons.insert("valid_date_outside_context_reporting_period".to_owned());
            result.json["disposition"] = json!("indeterminate");
            result.json["matched_fact_ids"] = json!([]);
            result.json["reason_codes"] = json!(result.reasons.iter().cloned().collect::<Vec<_>>());
        }
        *counts.entry(result.disposition.clone()).or_default() += 1;
        overall_reasons.extend(result.reasons.iter().cloned());
        results.push(result.json);
    }

    let disposition = if counts.get("needs_review").copied().unwrap_or(0) > 0 {
        "needs_review"
    } else if counts.get("unsupported").copied().unwrap_or(0) > 0 {
        "unsupported"
    } else if counts.get("indeterminate").copied().unwrap_or(0) > 0 {
        if counts.get("satisfied").copied().unwrap_or(0) > 0 { "partial" } else { "indeterminate" }
    } else {
        "requirements_satisfied"
    };
    if disposition == "requirements_satisfied" {
        overall_reasons.insert("all_exact_typed_facts_satisfied".to_owned());
    }

    Ok(json!({
        "report_schema_version": "1.0.0",
        "request_id": string(request, "request_id")?,
        "context_bundle_id": string(bundle, "bundle_id")?,
        "context_id": context_id,
        "context_bundle_sha256": bundle_sha,
        "context_schema_sha256": schema_sha,
        "valid_at": valid_at,
        "knowledge_at": knowledge_at,
        "disposition": disposition,
        "claim_ceiling": "typed_context_fact_matching_only_not_legal_tax_or_accounting_applicability",
        "requirement_results": results,
        "blocking_reasons": overall_reasons.into_iter().collect::<Vec<_>>(),
        "network_access_during_resolution": false,
        "policy_execution_during_resolution": false
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixtures() -> (Vec<u8>, Vec<u8>, Value, Value, Value, Value) {
        let context_bytes = include_bytes!("../../../applicability-context/fixtures/jurisdiction_context_candidates_v1.0.0.json").to_vec();
        let context_schema_bytes = include_bytes!("../../../applicability-context/jurisdiction_context_bundle_v1.0.0.schema.json").to_vec();
        let request_schema: Value = serde_json::from_str(include_str!("../../typed-context-v1/typed_context_request_v1.0.0.schema.json")).unwrap();
        let report_schema: Value = serde_json::from_str(include_str!("../../typed-context-v1/typed_context_report_v1.0.0.schema.json")).unwrap();
        let request_bundle: Value = serde_json::from_str(include_str!("../../typed-context-v1/fixtures/typed_context_requests_v1.0.0.json")).unwrap();
        let oracle: Value = serde_json::from_str(include_str!("../../typed-context-v1/fixtures/typed_context_oracle_v1.0.0.json")).unwrap();
        (context_bytes, context_schema_bytes, request_schema, report_schema, request_bundle, oracle)
    }

    fn oracle_schema() -> Value {
        serde_json::from_str(include_str!(
            "../../typed-context-v1/typed_context_oracle_v1.0.0.schema.json"
        )).unwrap()
    }

    fn request_for<'a>(bundle: &'a Value, id: &str) -> &'a Value {
        array(bundle, "requests").unwrap().iter()
            .find(|request| request["request_id"] == id).unwrap()
    }

    fn parsed_context(bytes: &[u8]) -> Value {
        serde_json::from_slice(bytes).unwrap()
    }

    fn parsed_schema(bytes: &[u8]) -> Value {
        serde_json::from_slice(bytes).unwrap()
    }

    fn resolve_fixture(id: &str) -> Value {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let bundle = parsed_context(&bundle_bytes);
        let schema = parsed_schema(&schema_bytes);
        resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, request_for(&requests, id)).unwrap()
    }

    #[test]
    fn public_request_and_report_schemas_validate() {
        let (bundle_bytes, schema_bytes, request_schema, report_schema, requests, _) = fixtures();
        let bundle = parsed_context(&bundle_bytes);
        let context_schema = parsed_schema(&schema_bytes);
        validate_instance("context bundle", &context_schema, &bundle).unwrap();
        validate_context_integrity(&bundle).unwrap();
        validate_schema("request schema", &request_schema).unwrap();
        validate_schema("report schema", &report_schema).unwrap();
        for request in array(&requests, "requests").unwrap() {
            validate_instance("request", &request_schema, request).unwrap();
            let report = resolve(&bundle, &bundle_bytes, &context_schema, &schema_bytes, request).unwrap();
            validate_instance("report", &report_schema, &report).unwrap();
        }
    }

    #[test]
    fn public_oracle_schema_and_all_expected_dispositions_match() {
        let (_, _, request_schema, _, requests, oracle) = fixtures();
        let oracle_schema = oracle_schema();
        validate_instance("public typed-context oracle", &oracle_schema, &oracle).unwrap();

        let expected = array(&oracle, "request_expectations").unwrap();
        assert_eq!(
            expected.len(),
            array(&requests, "requests").unwrap().len(),
            "every public candidate request must have exactly one oracle expectation"
        );
        let mut seen = BTreeSet::new();
        for expectation in expected {
            let request_id = string(expectation, "request_id").unwrap();
            assert!(seen.insert(request_id.to_owned()), "duplicate oracle request id");
            let request = request_for(&requests, request_id);
            let report = resolve_fixture(request_id);
            assert_eq!(
                report["disposition"],
                expectation["expected_disposition"],
                "unexpected disposition for {request_id}"
            );
            let required_reason = string(expectation, "required_reason_code").unwrap();
            assert!(
                report["blocking_reasons"].to_string().contains(required_reason),
                "missing oracle reason {required_reason} for {request_id}"
            );
            validate_instance("oracle-matched report", &report_schema_for_test(), &report).unwrap();
            let _ = (request, request_schema);
        }
    }

    fn report_schema_for_test() -> Value {
        serde_json::from_str(include_str!(
            "../../typed-context-v1/typed_context_report_v1.0.0.schema.json"
        )).unwrap()
    }

    #[test]
    fn seller_establishment_shipment_and_transaction_facts_satisfy_exact_requirements() {
        let report = resolve_fixture("TCTX-ROLE-A-COMPLETE");
        assert_eq!(report["disposition"], "requirements_satisfied");
        assert_eq!(report["requirement_results"].as_array().unwrap().len(), 6);
    }

    #[test]
    fn role_swapped_context_does_not_satisfy_same_typed_scope_requirements() {
        let report = resolve_fixture("TCTX-ROLE-B-ROLE-SWAP");
        assert_eq!(report["disposition"], "unsupported");
        assert!(report["blocking_reasons"].to_string().contains("jurisdiction_scope_mismatch"));
    }

    #[test]
    fn unknown_place_of_supply_is_indeterminate() {
        let report = resolve_fixture("TCTX-UNKNOWN-PLACE");
        assert_eq!(report["disposition"], "indeterminate");
        assert!(report["blocking_reasons"].to_string().contains("required_fact_unknown"));
    }

    #[test]
    fn disputed_seller_registration_requires_review() {
        let report = resolve_fixture("TCTX-DISPUTED-REGISTRATION");
        assert_eq!(report["disposition"], "needs_review");
        assert!(report["blocking_reasons"].to_string().contains("matching_fact_disputed"));
    }

    #[test]
    fn exact_second_establishment_is_selected_without_merging_establishments() {
        let report = resolve_fixture("TCTX-MULTI-ESTABLISHMENT");
        assert_eq!(report["disposition"], "requirements_satisfied");
        assert_eq!(report["requirement_results"][0]["matched_fact_ids"][0], "CTX-ROLE-A-SELLER-ESTABLISHMENT-02");
    }

    #[test]
    fn country_level_scope_does_not_satisfy_subnational_establishment_fact() {
        let report = resolve_fixture("TCTX-COUNTRY-NOT-SUBNATIONAL");
        assert_eq!(report["disposition"], "unsupported");
        assert!(report["blocking_reasons"].to_string().contains("jurisdiction_scope_mismatch"));
    }

    #[test]
    fn facts_not_visible_at_historical_knowledge_frontier_are_excluded() {
        let report = resolve_fixture("TCTX-KNOWLEDGE-FRONTIER");
        assert_eq!(report["disposition"], "indeterminate");
        assert!(report["blocking_reasons"].to_string().contains("matching_fact_not_visible_at_requested_frontier"));
    }

    #[test]
    fn modified_context_bytes_fail_the_exact_bundle_digest_check() {
        let (mut bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let mut bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        bundle["contexts"][0]["transaction_ref"] = json!("SYN-ALTERED-TRANSACTION");
        bundle_bytes = serde_json::to_vec(&bundle).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let error = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, request_for(&requests, "TCTX-ROLE-A-COMPLETE")).unwrap_err();
        assert!(error.contains("context bundle SHA-256 mismatch"));
    }

    #[test]
    fn modified_context_schema_bytes_fail_the_exact_schema_digest_check() {
        let (bundle_bytes, mut schema_bytes, _, _, requests, _) = fixtures();
        schema_bytes.push(b' ');
        let bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let error = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, request_for(&requests, "TCTX-ROLE-A-COMPLETE")).unwrap_err();
        assert!(error.contains("context schema SHA-256 mismatch"));
    }

    #[test]
    fn duplicate_exact_facts_are_ambiguous_not_latest_wins() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let mut bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let duplicate = bundle["contexts"][0]["jurisdiction_facts"].as_array().unwrap()
            .iter().find(|fact| fact["fact_id"] == "CTX-ROLE-A-SELLER-REGISTRATION").unwrap().clone();
        let mut duplicate = duplicate;
        duplicate["fact_id"] = json!("CTX-ROLE-A-SELLER-REGISTRATION-DUPLICATE");
        bundle["contexts"][0]["jurisdiction_facts"].as_array_mut().unwrap().push(duplicate);
        let changed_bytes = serde_json::to_vec(&bundle).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let mut request = request_for(&requests, "TCTX-ROLE-A-COMPLETE").clone();
        request["context_bundle_sha256"] = json!(sha256_hex(&changed_bytes));
        let report = resolve(&bundle, &changed_bytes, &schema, &schema_bytes, &request).unwrap();
        assert_eq!(report["disposition"], "needs_review");
        assert!(report["blocking_reasons"].to_string().contains("multiple_exact_facts_ambiguous"));
    }

    #[test]
    fn namespace_substitution_does_not_match_same_scope_code() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let mut request = request_for(&requests, "TCTX-ROLE-A-COMPLETE").clone();
        request["requirements"][2]["scope_identity"]["namespace_uri"] =
            json!("urn:mycelix:synthetic-alternate-scope-namespace:v1");
        let report = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, &request).unwrap();
        assert_eq!(report["requirement_results"][2]["disposition"], "unsupported");
        assert!(report["requirement_results"][2]["reason_codes"].to_string().contains("jurisdiction_scope_mismatch"));
    }

    #[test]
    fn party_substitution_does_not_satisfy_seller_registration_requirement() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let mut request = request_for(&requests, "TCTX-ROLE-A-COMPLETE").clone();
        request["requirements"][2]["subject_ref"] = json!("SYN-PARTY-BUYER");
        request["requirements"][2]["subject_party_role"] = json!("buyer");
        let report = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, &request).unwrap();
        assert_eq!(report["requirement_results"][2]["disposition"], "unsupported");
        assert!(report["requirement_results"][2]["reason_codes"].to_string().contains("jurisdiction_scope_mismatch"));
    }

    #[test]
    fn reordered_fact_and_scope_arrays_preserve_semantic_result() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let original_bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let original_request = request_for(&requests, "TCTX-ROLE-A-COMPLETE").clone();
        let original = resolve(&original_bundle, &bundle_bytes, &schema, &schema_bytes, &original_request).unwrap();

        let mut reordered = original_bundle;
        let context = reordered["contexts"].as_array_mut().unwrap()
            .iter_mut().find(|item| item["context_id"] == "CTX-ROLE-A").unwrap();
        context["jurisdiction_facts"].as_array_mut().unwrap().reverse();
        context["scope_registry"].as_array_mut().unwrap().reverse();
        let reordered_bytes = serde_json::to_vec(&reordered).unwrap();
        let mut request = original_request;
        request["context_bundle_sha256"] = json!(sha256_hex(&reordered_bytes));
        let actual = resolve(&reordered, &reordered_bytes, &schema, &schema_bytes, &request).unwrap();

        assert_eq!(original["disposition"], actual["disposition"]);
        assert_eq!(original["requirement_results"], actual["requirement_results"]);
    }

    #[test]
    fn identical_frozen_inputs_produce_byte_identical_report_json() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let request = request_for(&requests, "TCTX-ROLE-A-COMPLETE");
        let first = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, request).unwrap();
        let second = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, request).unwrap();
        assert_eq!(serde_json::to_vec(&first).unwrap(), serde_json::to_vec(&second).unwrap());
    }

    #[test]
    fn duplicate_requirement_ids_are_rejected() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let mut request = request_for(&requests, "TCTX-ROLE-A-COMPLETE").clone();
        let first_id = request["requirements"][0]["requirement_id"].clone();
        request["requirements"][1]["requirement_id"] = first_id;
        let error = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, &request).unwrap_err();
        assert!(error.contains("duplicate requirement_id"));
    }

    #[test]
    fn explicit_conflict_link_to_an_otherwise_matching_fact_requires_review() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let mut bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let context = bundle["contexts"].as_array_mut().unwrap()
            .iter_mut().find(|context| context["context_id"] == "CTX-ROLE-A").unwrap();
        let facts = context["jurisdiction_facts"].as_array_mut().unwrap();
        let target = facts.iter_mut().find(|fact| fact["fact_id"] == "CTX-ROLE-A-SELLER-REGISTRATION").unwrap();
        target["conflicts_with_fact_refs"] = json!(["CTX-ROLE-A-SELLER-REGISTRATION-CONFLICT"]);
        let mut conflict = target.clone();
        conflict["fact_id"] = json!("CTX-ROLE-A-SELLER-REGISTRATION-CONFLICT");
        conflict["scope_ref"] = json!("SYN-SCOPE-B");
        conflict["conflicts_with_fact_refs"] = json!(["CTX-ROLE-A-SELLER-REGISTRATION"]);
        facts.push(conflict);

        let changed_bytes = serde_json::to_vec(&bundle).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let mut request = request_for(&requests, "TCTX-ROLE-A-COMPLETE").clone();
        request["context_bundle_sha256"] = json!(sha256_hex(&changed_bytes));
        let report = resolve(&bundle, &changed_bytes, &schema, &schema_bytes, &request).unwrap();
        assert_eq!(report["disposition"], "needs_review");
        assert!(report["blocking_reasons"].to_string().contains("competing_conflicting_fact_present"));
    }

    #[test]
    fn request_after_bundle_knowledge_frontier_is_indeterminate() {
        let (bundle_bytes, schema_bytes, _, _, requests, _) = fixtures();
        let bundle: Value = serde_json::from_slice(&bundle_bytes).unwrap();
        let schema: Value = serde_json::from_slice(&schema_bytes).unwrap();
        let mut request = request_for(&requests, "TCTX-ROLE-A-COMPLETE").clone();
        request["knowledge_at"] = json!("2026-11-05T10:00:00Z");
        let report = resolve(&bundle, &bundle_bytes, &schema, &schema_bytes, &request).unwrap();
        assert_eq!(report["disposition"], "indeterminate");
        assert!(report["blocking_reasons"].to_string().contains("context_snapshot_ends_before_requested_knowledge_frontier"));
    }
}
