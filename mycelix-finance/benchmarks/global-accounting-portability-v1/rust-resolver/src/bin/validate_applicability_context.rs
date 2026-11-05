use jsonschema::draft202012;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("mycelix-applicability-context-validator: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut context_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../applicability-context");
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--context-dir" => {
                context_dir = PathBuf::from(
                    args.next().ok_or("--context-dir requires a directory")?,
                );
            }
            "--help" | "-h" => {
                println!(
                    "Mycelix typed applicability-context validator\n\n\
                     --context-dir PATH  Context artifact directory (default: sibling applicability-context)\n\
                     --help              Show this help"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let schema: Value = serde_json::from_slice(&fs::read(
        context_dir.join("jurisdiction_context_bundle_v1.0.0.schema.json"),
    )?)?;
    let candidates: Value = serde_json::from_slice(&fs::read(
        context_dir.join("fixtures/jurisdiction_context_candidates_v1.0.0.json"),
    )?)?;
    let oracle_schema: Value = serde_json::from_slice(&fs::read(
        context_dir.join("jurisdiction_context_oracle_v1.0.0.schema.json"),
    )?)?;
    let oracle: Value = serde_json::from_slice(&fs::read(
        context_dir.join("fixtures/jurisdiction_context_oracle_v1.0.0.json"),
    )?)?;

    validate_instance("context bundle", &schema, &candidates)?;
    validate_instance("context oracle", &oracle_schema, &oracle)?;
    validate_context_bundle(&candidates)?;
    validate_oracle(&candidates, &oracle)?;

    let context_count = array(&candidates, "contexts")?.len();
    let assertion_count = array(&oracle, "assertions")?.len();
    println!(
        "PASS: {context_count} synthetic contexts; {assertion_count} public reference assertions; \
         schema and context-integrity checks complete. No tax/legal applicability conclusion is produced."
    );
    Ok(())
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Result<(), String> {
    draft202012::meta::validate(schema)
        .map_err(|error| format!("{label}: invalid Draft 2020-12 schema: {error}"))?;
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

fn insert_unique(set: &mut BTreeSet<String>, value: &str, label: &str) -> Result<(), String> {
    if set.insert(value.to_owned()) {
        Ok(())
    } else {
        Err(format!("duplicate {label}: {value}"))
    }
}

fn validate_context_bundle(bundle: &Value) -> Result<(), String> {
    let mut context_ids = BTreeSet::new();
    for context in array(bundle, "contexts")? {
        let context_id = string(context, "context_id")?;
        insert_unique(&mut context_ids, context_id, "context_id")?;
        let mut party_roles = BTreeMap::<String, BTreeSet<String>>::new();
        for party in array(context, "parties")? {
            let party_ref = string(party, "party_ref")?.to_owned();
            if party_roles.contains_key(&party_ref) {
                return Err(format!("{context_id}: duplicate party_ref {party_ref}"));
            }
            let roles = array(party, "transaction_roles")?
                .iter()
                .map(|role| {
                    role.as_str()
                        .map(str::to_owned)
                        .ok_or_else(|| format!("{context_id}: non-string party role"))
                })
                .collect::<Result<BTreeSet<_>, _>>()?;
            party_roles.insert(party_ref, roles);
        }

        let mut scopes = BTreeMap::<String, &Value>::new();
        let mut canonical_scopes = BTreeSet::new();
        for scope in array(context, "scope_registry")? {
            let scope_id = string(scope, "scope_id")?.to_owned();
            if scopes.insert(scope_id.clone(), scope).is_some() {
                return Err(format!("{context_id}: duplicate scope_id {scope_id}"));
            }
            let identity = format!(
                "{}|{}|{}|{}",
                string(scope, "namespace_uri")?,
                string(scope, "code")?,
                string(scope, "scope_level")?,
                scope.get("parent_scope_ref").and_then(Value::as_str).unwrap_or(""),
            );
            insert_unique(&mut canonical_scopes, &identity, "canonical jurisdiction scope")?;
        }
        for (scope_id, scope) in &scopes {
            if let Some(parent) = scope.get("parent_scope_ref").and_then(Value::as_str) {
                if !scopes.contains_key(parent) {
                    return Err(format!("{context_id}: {scope_id} references missing parent scope {parent}"));
                }
                if parent == scope_id.as_str() {
                    return Err(format!("{context_id}: scope {scope_id} is its own parent"));
                }
            }
        }

        let facts = array(context, "jurisdiction_facts")?;
        let mut fact_map = BTreeMap::<String, &Value>::new();
        for fact in facts {
            let fact_id = string(fact, "fact_id")?.to_owned();
            if fact_map.insert(fact_id.clone(), fact).is_some() {
                return Err(format!("{context_id}: duplicate fact_id {fact_id}"));
            }
        }
        for (fact_id, fact) in &fact_map {
            let subject_kind = string(fact, "subject_kind")?;
            let subject_ref = string(fact, "subject_ref")?;
            if subject_kind == "party" {
                let subject_role = string(fact, "subject_party_role")?;
                let roles = party_roles.get(subject_ref).ok_or_else(|| {
                    format!("{context_id}: fact {fact_id} has unknown party subject {subject_ref}")
                })?;
                if !roles.contains(subject_role) {
                    return Err(format!(
                        "{context_id}: fact {fact_id} subject role {subject_role} is not bound to {subject_ref}"
                    ));
                }
            } else if fact.get("subject_party_role").is_some() {
                return Err(format!(
                    "{context_id}: non-party fact {fact_id} must not declare subject_party_role"
                ));
            }

            if subject_kind == "transaction"
                && subject_ref != string(context, "transaction_ref")?
            {
                return Err(format!(
                    "{context_id}: transaction fact {fact_id} references a different transaction"
                ));
            }

            let jurisdiction_role = string(fact, "jurisdiction_role")?;
            if jurisdiction_role == "place_of_supply" && subject_kind != "transaction" {
                return Err(format!(
                    "{context_id}: place_of_supply fact {fact_id} must be transaction-scoped"
                ));
            }

            if let Some(bindings) = fact.get("party_role_bindings").and_then(Value::as_array) {
                for binding in bindings {
                    let party_ref = string(binding, "party_ref")?;
                    let transaction_role = string(binding, "transaction_role")?;
                    let roles = party_roles.get(party_ref).ok_or_else(|| {
                        format!("{context_id}: fact {fact_id} binds unknown party {party_ref}")
                    })?;
                    if !roles.contains(transaction_role) {
                        return Err(format!(
                            "{context_id}: fact {fact_id} binds {party_ref} to undeclared transaction role {transaction_role}"
                        ));
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
                    return Err(format!("{context_id}: fact {fact_id} has an empty/reversed valid-time interval"));
                }
            }
            if string(fact, "assertion_state")? == "evidence_supported"
                && array(fact, "source_evidence_refs")?.is_empty()
            {
                return Err(format!("{context_id}: evidence-supported fact {fact_id} has no evidence reference"));
            }
        }

        for (fact_id, fact) in &fact_map {
            for relation in ["conflicts_with_fact_refs", "supersedes_fact_refs"] {
                if let Some(values) = fact.get(relation).and_then(Value::as_array) {
                    for target in values {
                        let target_id = target
                            .as_str()
                            .ok_or_else(|| format!("{context_id}: non-string {relation} target"))?;
                        if !fact_map.contains_key(target_id) {
                            return Err(format!(
                                "{context_id}: fact {fact_id} has dangling {relation} reference {target_id}"
                            ));
                        }
                        if relation == "conflicts_with_fact_refs" {
                            let reciprocal = fact_map
                                .get(target_id)
                                .and_then(|other| other.get(relation))
                                .and_then(Value::as_array)
                                .is_some_and(|refs| refs.iter().any(|item| item.as_str() == Some(fact_id)));
                            if !reciprocal {
                                return Err(format!(
                                    "{context_id}: conflict relation {fact_id}/{target_id} is not reciprocal"
                                ));
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

fn context_by_id<'a>(bundle: &'a Value, context_id: &str) -> Result<&'a Value, String> {
    array(bundle, "contexts")?
        .iter()
        .find(|context| {
            context.get("context_id").and_then(Value::as_str) == Some(context_id)
        })
        .ok_or_else(|| format!("missing candidate context {context_id}"))
}

fn scope_set_for_role(context: &Value, role: &str) -> Result<BTreeSet<String>, String> {
    Ok(array(context, "jurisdiction_facts")?
        .iter()
        .filter(|fact| fact.get("jurisdiction_role").and_then(Value::as_str) == Some(role))
        .filter_map(|fact| fact.get("scope_ref").and_then(Value::as_str).map(str::to_owned))
        .collect())
}

fn validate_oracle(candidates: &Value, oracle: &Value) -> Result<(), String> {
    let candidate_ids = array(candidates, "contexts")?
        .iter()
        .map(|context| string(context, "context_id").map(str::to_owned))
        .collect::<Result<BTreeSet<_>, _>>()?;
    let mut assertion_ids = BTreeSet::new();

    for assertion in array(oracle, "assertions")? {
        let assertion_id = string(assertion, "assertion_id")?;
        insert_unique(&mut assertion_ids, assertion_id, "oracle assertion id")?;
        let context_ids = array(assertion, "context_ids")?
            .iter()
            .map(|value| {
                value
                    .as_str()
                    .ok_or_else(|| "oracle context_ids must be strings".to_owned())
            })
            .collect::<Result<Vec<_>, _>>()?;
        for context_id in &context_ids {
            if !candidate_ids.contains(*context_id) {
                return Err(format!("oracle references missing candidate context {context_id}"));
            }
        }

        match string(assertion, "expected_behavior")? {
            "typed_contexts_not_equivalent" => {
                if context_ids.len() != 2 {
                    return Err(format!("{assertion_id}: role-swap assertion requires two contexts"));
                }
                let a = context_by_id(candidates, context_ids[0])?;
                let b = context_by_id(candidates, context_ids[1])?;
                let all_scopes = |context: &Value| -> Result<BTreeSet<String>, String> {
                    Ok(array(context, "jurisdiction_facts")?
                        .iter()
                        .filter_map(|fact| {
                            fact.get("scope_ref").and_then(Value::as_str).map(str::to_owned)
                        })
                        .collect())
                };
                if all_scopes(a)? != all_scopes(b)? {
                    return Err(format!("{assertion_id}: flattened scope sets must be equal"));
                }
                let mut typed_difference = false;
                for role in ["establishment_location", "ship_from", "ship_to"] {
                    if scope_set_for_role(a, role)? != scope_set_for_role(b, role)? {
                        typed_difference = true;
                    }
                }
                if !typed_difference {
                    return Err(format!("{assertion_id}: no jurisdiction-role binding differs"));
                }
            }
            "indeterminate_no_place_of_supply" => {
                for context_id in &context_ids {
                    let context = context_by_id(candidates, context_id)?;
                    let place = array(context, "jurisdiction_facts")?
                        .iter()
                        .find(|fact| {
                            fact.get("jurisdiction_role").and_then(Value::as_str)
                                == Some("place_of_supply")
                        })
                        .ok_or_else(|| format!("{assertion_id}: missing place-of-supply fact"))?;
                    if place.get("assertion_state").and_then(Value::as_str) != Some("unknown")
                        || place.get("origin").and_then(Value::as_str) != Some("unresolved")
                        || !place.get("scope_ref").is_some_and(Value::is_null)
                        || place.get("subject_kind").and_then(Value::as_str) != Some("transaction")
                    {
                        return Err(format!(
                            "{assertion_id}: place of supply must remain unknown and transaction-scoped"
                        ));
                    }
                }
            }
            "needs_review_conflicting_registration" => {
                for context_id in &context_ids {
                    let context = context_by_id(candidates, context_id)?;
                    let registrations = array(context, "jurisdiction_facts")?
                        .iter()
                        .filter(|fact| {
                            fact.get("subject_ref").and_then(Value::as_str)
                                == Some("SYN-PARTY-SELLER")
                                && fact.get("jurisdiction_role").and_then(Value::as_str)
                                    == Some("tax_registration")
                        })
                        .collect::<Vec<_>>();
                    if registrations.len() < 2
                        || registrations.iter().any(|fact| {
                            fact.get("assertion_state").and_then(Value::as_str) != Some("disputed")
                                || fact
                                    .get("conflicts_with_fact_refs")
                                    .and_then(Value::as_array)
                                    .is_none_or(Vec::is_empty)
                        })
                    {
                        return Err(format!(
                            "{assertion_id}: expected mutually linked disputed seller-registration facts"
                        ));
                    }
                }
            }
            "subjects_remain_distinct_across_namespaces" => {
                for context_id in &context_ids {
                    let context = context_by_id(candidates, context_id)?;
                    let mut identifiers =
                        BTreeMap::<String, BTreeSet<(String, String)>>::new();
                    for party in array(context, "parties")? {
                        let party_ref = string(party, "party_ref")?.to_owned();
                        if let Some(bindings) =
                            party.get("identity_bindings").and_then(Value::as_array)
                        {
                            for binding in bindings {
                                let identifier = string(binding, "identifier")?.to_owned();
                                let namespace = string(binding, "namespace_uri")?.to_owned();
                                identifiers
                                    .entry(identifier)
                                    .or_default()
                                    .insert((party_ref.clone(), namespace));
                            }
                        }
                    }
                    let separated = identifiers.values().any(|bindings| {
                        let party_refs = bindings
                            .iter()
                            .map(|(party_ref, _)| party_ref)
                            .collect::<BTreeSet<_>>();
                        let namespaces = bindings
                            .iter()
                            .map(|(_, namespace)| namespace)
                            .collect::<BTreeSet<_>>();
                        party_refs.len() >= 2 && namespaces.len() >= 2
                    });
                    if !separated {
                        return Err(format!(
                            "{assertion_id}: no same-string identifier collision across issuer namespaces"
                        ));
                    }
                }
            }
            other => return Err(format!("{assertion_id}: unsupported expected behavior {other}")),
        }
    }
    Ok(())
}

fn fact_eligible_at(fact: &Value, valid_at: &str, knowledge_at: &str) -> bool {
    let state = fact.get("assertion_state").and_then(Value::as_str);
    let origin = fact.get("origin").and_then(Value::as_str);
    let recorded_at = fact.get("recorded_at").and_then(Value::as_str);
    let valid_from = fact.get("valid_from").and_then(Value::as_str);
    let valid_to = fact.get("valid_to").and_then(Value::as_str);
    let scope_ref = fact.get("scope_ref").and_then(Value::as_str);
    let evidence_present = fact
        .get("source_evidence_refs")
        .and_then(Value::as_array)
        .is_some_and(|refs| !refs.is_empty());
    let conflict_free = fact
        .get("conflicts_with_fact_refs")
        .and_then(Value::as_array)
        .is_none_or(Vec::is_empty);
    state == Some("evidence_supported")
        && origin != Some("unresolved")
        && scope_ref.is_some_and(|scope| !scope.is_empty())
        && evidence_present
        && conflict_free
        && recorded_at.is_some_and(|value| value <= knowledge_at)
        && valid_from.is_some_and(|value| value <= valid_at)
        && valid_to.is_none_or(|value| valid_at < value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixtures() -> (Value, Value, Value, Value) {
        let context_schema = serde_json::from_str(include_str!(
            "../../../applicability-context/jurisdiction_context_bundle_v1.0.0.schema.json"
        ))
        .unwrap();
        let candidates = serde_json::from_str(include_str!(
            "../../../applicability-context/fixtures/jurisdiction_context_candidates_v1.0.0.json"
        ))
        .unwrap();
        let oracle_schema = serde_json::from_str(include_str!(
            "../../../applicability-context/jurisdiction_context_oracle_v1.0.0.schema.json"
        ))
        .unwrap();
        let oracle = serde_json::from_str(include_str!(
            "../../../applicability-context/fixtures/jurisdiction_context_oracle_v1.0.0.json"
        ))
        .unwrap();
        (context_schema, candidates, oracle_schema, oracle)
    }

    fn context<'a>(bundle: &'a Value, id: &str) -> &'a Value {
        array(bundle, "contexts")
            .unwrap()
            .iter()
            .find(|item| item["context_id"] == id)
            .unwrap()
    }

    fn fact<'a>(context: &'a Value, role: &str) -> &'a Value {
        array(context, "jurisdiction_facts")
            .unwrap()
            .iter()
            .find(|item| item["jurisdiction_role"] == role)
            .unwrap()
    }

    fn fact_scope_set(context: &Value) -> BTreeSet<String> {
        array(context, "jurisdiction_facts")
            .unwrap()
            .iter()
            .filter_map(|item| item["scope_ref"].as_str().map(str::to_owned))
            .collect()
    }

    #[test]
    fn public_schemas_and_fixtures_validate() {
        let (schema, candidates, oracle_schema, oracle) = fixtures();
        validate_instance("context bundle", &schema, &candidates).unwrap();
        validate_instance("context oracle", &oracle_schema, &oracle).unwrap();
        validate_context_bundle(&candidates).unwrap();
        validate_oracle(&candidates, &oracle).unwrap();
    }

    #[test]
    fn role_swap_keeps_flat_scope_set_but_changes_typed_bindings() {
        let (_, candidates, _, _) = fixtures();
        let a = context(&candidates, "CTX-ROLE-A");
        let b = context(&candidates, "CTX-ROLE-B");
        assert_eq!(fact_scope_set(a), fact_scope_set(b));
        assert_eq!(fact(a, "establishment_location")["scope_ref"], json!("SYN-SCOPE-A"));
        assert_eq!(fact(b, "establishment_location")["scope_ref"], json!("SYN-SCOPE-B"));
        assert_eq!(fact(a, "ship_from")["scope_ref"], json!("SYN-SCOPE-B"));
        assert_eq!(fact(b, "ship_from")["scope_ref"], json!("SYN-SCOPE-A"));
    }

    #[test]
    fn place_of_supply_is_a_transaction_subject_not_a_party_attribute() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "CTX-ROLE-A");
        let place = fact(ctx, "place_of_supply");
        assert_eq!(place["subject_kind"], "transaction");
        assert_eq!(place["subject_ref"], ctx["transaction_ref"]);
        assert!(place.get("subject_party_role").is_none());
    }

    #[test]
    fn multiple_establishments_remain_distinct_subjects_of_the_same_party() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "CTX-ROLE-A");
        let establishments = array(ctx, "jurisdiction_facts")
            .unwrap()
            .iter()
            .filter(|item| item["jurisdiction_role"] == "establishment_location")
            .collect::<Vec<_>>();
        assert_eq!(establishments.len(), 2);
        assert_ne!(establishments[0]["subject_ref"], establishments[1]["subject_ref"]);
        assert!(establishments.iter().all(|item| {
            item["subject_kind"] == "establishment"
                && item["party_role_bindings"][0]["party_ref"] == "SYN-PARTY-SELLER"
        }));
    }

    #[test]
    fn unknown_place_of_supply_is_not_eligible() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "CTX-PLACE-UNKNOWN");
        let place = fact(ctx, "place_of_supply");
        assert_eq!(place["assertion_state"], "unknown");
        assert_eq!(place["scope_ref"], Value::Null);
        assert!(!fact_eligible_at(place, "2026-10-31", "2026-11-05T09:30:00Z"));
    }

    #[test]
    fn disputed_registration_claims_are_not_eligible() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "CTX-REGISTRATION-DISPUTED");
        let registrations = array(ctx, "jurisdiction_facts")
            .unwrap()
            .iter()
            .filter(|item| item["jurisdiction_role"] == "tax_registration")
            .collect::<Vec<_>>();
        assert_eq!(registrations.len(), 2);
        assert!(registrations.iter().all(|item| {
            item["assertion_state"] == "disputed"
                && !fact_eligible_at(item, "2026-10-31", "2026-11-05T09:30:00Z")
        }));
    }

    #[test]
    fn same_identifier_in_different_issuer_namespaces_is_not_an_identity_key() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "CTX-ROLE-A");
        let matches = array(ctx, "parties")
            .unwrap()
            .iter()
            .filter_map(|party| {
                let bindings = party.get("identity_bindings")?.as_array()?;
                bindings
                    .iter()
                    .find(|binding| binding["identifier"] == "000123")
                    .map(|binding| {
                        (
                            party["party_ref"].as_str().unwrap().to_owned(),
                            binding["namespace_uri"].as_str().unwrap().to_owned(),
                        )
                    })
            })
            .collect::<Vec<_>>();
        assert_eq!(matches.len(), 2);
        assert_ne!(matches[0].0, matches[1].0);
        assert_ne!(matches[0].1, matches[1].1);
    }

    #[test]
    fn fact_recorded_after_knowledge_frontier_is_not_eligible() {
        let mut candidate = json!({
            "assertion_state":"evidence_supported",
            "origin":"source_asserted",
            "scope_ref":"SYN-SCOPE-A",
            "source_evidence_refs":["SYN-EVIDENCE-UNIT-TEST"],
            "recorded_at":"2026-11-05T10:00:00Z",
            "valid_from":"2026-01-01",
            "valid_to":null
        });
        assert!(!fact_eligible_at(&candidate, "2026-11-05", "2026-11-05T09:30:00Z"));
        candidate["recorded_at"] = json!("2026-11-05T09:00:00Z");
        assert!(fact_eligible_at(&candidate, "2026-11-05", "2026-11-05T09:30:00Z"));
    }

    #[test]
    fn valid_time_end_is_exclusive_and_start_is_inclusive() {
        let candidate = json!({
            "assertion_state":"evidence_supported",
            "origin":"source_asserted",
            "scope_ref":"SYN-SCOPE-A",
            "source_evidence_refs":["SYN-EVIDENCE-UNIT-TEST"],
            "recorded_at":"2026-11-05T09:00:00Z",
            "valid_from":"2026-01-01",
            "valid_to":"2026-11-05"
        });
        assert!(fact_eligible_at(&candidate, "2026-01-01", "2026-11-05T09:30:00Z"));
        assert!(!fact_eligible_at(&candidate, "2026-11-05", "2026-11-05T09:30:00Z"));
    }

    #[test]
    fn a_conflicted_supported_fact_is_not_eligible() {
        let candidate = json!({
            "assertion_state":"evidence_supported",
            "origin":"source_asserted",
            "scope_ref":"SYN-SCOPE-A",
            "source_evidence_refs":["SYN-EVIDENCE-UNIT-TEST"],
            "conflicts_with_fact_refs":["OTHER-FACT"],
            "recorded_at":"2026-11-05T09:00:00Z",
            "valid_from":"2026-01-01",
            "valid_to":null
        });
        assert!(!fact_eligible_at(&candidate, "2026-11-05", "2026-11-05T09:30:00Z"));
    }

    #[test]
    fn missing_scope_or_unknown_fact_never_becomes_eligible() {
        let candidate = json!({
            "assertion_state":"unknown",
            "origin":"unresolved",
            "scope_ref":null,
            "recorded_at":"2026-11-05T09:00:00Z",
            "valid_from":"2026-01-01",
            "valid_to":null
        });
        assert!(!fact_eligible_at(&candidate, "2026-11-05", "2026-11-05T09:30:00Z"));
    }
}
