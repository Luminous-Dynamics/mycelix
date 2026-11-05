use jsonschema::draft202012;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("mycelix-finance-data-flow-validator: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut data_dir = root.join("../data-governance-v1");
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => {
                data_dir = PathBuf::from(
                    args.next().ok_or("--data-dir requires a directory")?,
                );
            }
            "--help" | "-h" => {
                println!(
                    "Mycelix Finance Data-Flow Context Validator v1.0.0\n\n\
                     --data-dir PATH  Context artifact directory (default: sibling data-governance-v1)\n\
                     --help           Show this help"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let schema: Value = serde_json::from_slice(&fs::read(
        data_dir.join("finance_data_flow_context_v1.0.0.schema.json"),
    )?)?;
    let candidates: Value = serde_json::from_slice(&fs::read(
        data_dir.join("fixtures/finance_data_flow_candidates_v1.0.0.json"),
    )?)?;
    let oracle_schema: Value = serde_json::from_slice(&fs::read(
        data_dir.join("finance_data_flow_oracle_v1.0.0.schema.json"),
    )?)?;
    let oracle: Value = serde_json::from_slice(&fs::read(
        data_dir.join("fixtures/finance_data_flow_oracle_v1.0.0.json"),
    )?)?;

    validate_instance("data-flow context bundle", &schema, &candidates)?;
    validate_instance("data-flow public oracle", &oracle_schema, &oracle)?;
    validate_bundle_integrity(&candidates)?;
    validate_oracle(&candidates, &oracle)?;

    println!(
        "PASS: {} synthetic data-flow contexts; {} public oracle expectations; \
         schema and cross-reference checks complete. No legal transfer or privacy conclusion is produced.",
        array(&candidates, "contexts")?.len(),
        array(&oracle, "expectations")?.len()
    );
    Ok(())
}

fn validate_schema(label: &str, schema: &Value) -> Result<(), String> {
    draft202012::meta::validate(schema)
        .map_err(|error| format!("{label}: invalid Draft 2020-12 schema: {error}"))?;
    draft202012::options()
        .build(schema)
        .map(|_| ())
        .map_err(|error| format!("{label}: could not compile Draft 2020-12 schema: {error}"))
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Result<(), String> {
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

fn validate_bundle_integrity(bundle: &Value) -> Result<(), String> {
    let mut context_ids = BTreeSet::new();
    for context in array(bundle, "contexts")? {
        let context_id = string(context, "context_id")?;
        insert_unique(&mut context_ids, context_id, "context_id")?;

        let mut scopes = BTreeMap::<String, &Value>::new();
        for scope in array(context, "scope_registry")? {
            let scope_id = string(scope, "scope_id")?.to_owned();
            if scopes.insert(scope_id.clone(), scope).is_some() {
                return Err(format!("{context_id}: duplicate scope_id {scope_id}"));
            }
        }
        for (scope_id, scope) in &scopes {
            if let Some(parent) = scope.get("parent_scope_ref").and_then(Value::as_str) {
                if !scopes.contains_key(parent) {
                    return Err(format!("{context_id}: scope {scope_id} references unknown parent {parent}"));
                }
            }
        }
        for scope_id in scopes.keys() {
            let mut visited = BTreeSet::new();
            let mut current = Some(scope_id.as_str());
            while let Some(id) = current {
                if !visited.insert(id.to_owned()) {
                    return Err(format!("{context_id}: jurisdiction scope cycle at {id}"));
                }
                current = scopes
                    .get(id)
                    .and_then(|scope| scope.get("parent_scope_ref"))
                    .and_then(Value::as_str);
            }
        }

        let mut actors = BTreeMap::<String, &Value>::new();
        for actor in array(context, "actors")? {
            let actor_ref = string(actor, "actor_ref")?.to_owned();
            if actors.insert(actor_ref.clone(), actor).is_some() {
                return Err(format!("{context_id}: duplicate actor_ref {actor_ref}"));
            }
            if let Some(home_scope) = actor.get("home_scope_ref").and_then(Value::as_str) {
                if !scopes.contains_key(home_scope) {
                    return Err(format!("{context_id}: actor {actor_ref} references unknown home scope {home_scope}"));
                }
            }
        }

        let mut assets = BTreeSet::new();
        for asset in array(context, "data_assets")? {
            let asset_ref = string(asset, "asset_ref")?;
            insert_unique(&mut assets, asset_ref, "asset_ref")?;
            if asset.get("synthetic_only").and_then(Value::as_bool) != Some(true) {
                return Err(format!("{context_id}: data asset {asset_ref} is not marked synthetic_only"));
            }
        }

        let mut purposes = BTreeSet::new();
        for purpose in array(context, "purposes")? {
            insert_unique(&mut purposes, string(purpose, "purpose_ref")?, "purpose_ref")?;
            let valid_from = string(purpose, "valid_from")?;
            if let Some(valid_to) = purpose.get("valid_to").and_then(Value::as_str) {
                if valid_to <= valid_from {
                    return Err(format!("{context_id}: purpose has an empty/reversed valid interval"));
                }
            }
        }

        let mut profiles = BTreeMap::<String, &Value>::new();
        for profile in array(context, "legal_profiles")? {
            let profile_ref = string(profile, "profile_ref")?.to_owned();
            if profiles.insert(profile_ref.clone(), profile).is_some() {
                return Err(format!("{context_id}: duplicate legal profile {profile_ref}"));
            }
            let scope_ref = string(profile, "jurisdiction_scope_ref")?;
            if !scopes.contains_key(scope_ref) {
                return Err(format!("{context_id}: legal profile {profile_ref} references unknown jurisdiction scope {scope_ref}"));
            }
        }

        let mut mechanisms = BTreeMap::<String, &Value>::new();
        for mechanism in array(context, "transfer_mechanisms")? {
            let mechanism_ref = string(mechanism, "mechanism_ref")?.to_owned();
            if mechanisms.insert(mechanism_ref.clone(), mechanism).is_some() {
                return Err(format!("{context_id}: duplicate transfer mechanism {mechanism_ref}"));
            }
            let profile_ref = string(mechanism, "profile_ref")?;
            if !profiles.contains_key(profile_ref) {
                return Err(format!("{context_id}: transfer mechanism {mechanism_ref} references unknown legal profile {profile_ref}"));
            }
            let destination = string(mechanism, "destination_scope_ref")?;
            if !scopes.contains_key(destination) {
                return Err(format!("{context_id}: transfer mechanism {mechanism_ref} references unknown destination scope {destination}"));
            }
        }

        let mut flows = BTreeMap::<String, &Value>::new();
        for flow in array(context, "flows")? {
            let flow_ref = string(flow, "flow_ref")?.to_owned();
            if flows.insert(flow_ref.clone(), flow).is_some() {
                return Err(format!("{context_id}: duplicate flow_ref {flow_ref}"));
            }
            let asset_ref = string(flow, "asset_ref")?;
            if !assets.contains(asset_ref) {
                return Err(format!("{context_id}: flow {flow_ref} references unknown data asset {asset_ref}"));
            }
            for actor_key in ["source_actor_ref", "recipient_actor_ref"] {
                let actor_ref = string(flow, actor_key)?;
                if !actors.contains_key(actor_ref) {
                    return Err(format!("{context_id}: flow {flow_ref} references unknown actor {actor_ref}"));
                }
            }
            let purpose_ref = string(flow, "purpose_ref")?;
            if !purposes.contains(purpose_ref) {
                return Err(format!("{context_id}: flow {flow_ref} references unknown purpose {purpose_ref}"));
            }
            for profile_ref in array(flow, "legal_profile_refs")? {
                let profile_ref = profile_ref.as_str().ok_or("legal_profile_refs entries must be strings")?;
                if !profiles.contains_key(profile_ref) {
                    return Err(format!("{context_id}: flow {flow_ref} references unknown legal profile {profile_ref}"));
                }
            }
            for mechanism_ref in array(flow, "transfer_mechanism_refs")? {
                let mechanism_ref = mechanism_ref.as_str().ok_or("transfer_mechanism_refs entries must be strings")?;
                if !mechanisms.contains_key(mechanism_ref) {
                    return Err(format!("{context_id}: flow {flow_ref} references unknown transfer mechanism {mechanism_ref}"));
                }
            }

            let locations = array(flow, "location_observations")?;
            for location in locations {
                if let Some(scope_ref) = location.get("scope_ref").and_then(Value::as_str) {
                    if !scopes.contains_key(scope_ref) {
                        return Err(format!("{context_id}: flow {flow_ref} location references unknown scope {scope_ref}"));
                    }
                }
                if location.get("assertion_state").and_then(Value::as_str) == Some("unknown")
                    && !location.get("scope_ref").is_some_and(Value::is_null)
                {
                    return Err(format!("{context_id}: unknown location in {flow_ref} must not contain a scope"));
                }
            }
            let operation = string(flow, "operation")?;
            if operation == "support_access"
                && !locations.iter().any(|location| {
                    location.get("facet").and_then(Value::as_str) == Some("support_access")
                })
            {
                return Err(format!("{context_id}: support_access flow {flow_ref} lacks a support_access location facet"));
            }
            if operation == "onward_transfer"
                && !locations.iter().any(|location| {
                    location.get("facet").and_then(Value::as_str) == Some("onward_recipient")
                })
            {
                return Err(format!("{context_id}: onward_transfer flow {flow_ref} lacks an onward_recipient location facet"));
            }
        }

        let mut unique_claims = BTreeSet::new();
        for claim in array(context, "actor_role_claims")? {
            let claim_ref = string(claim, "claim_ref")?;
            insert_unique(&mut unique_claims, claim_ref, "actor role claim id")?;
            let actor_ref = string(claim, "actor_ref")?;
            let flow_ref = string(claim, "flow_ref")?;
            if !actors.contains_key(actor_ref) {
                return Err(format!("{context_id}: actor-role claim {claim_ref} references unknown actor {actor_ref}"));
            }
            if !flows.contains_key(flow_ref) {
                return Err(format!("{context_id}: actor-role claim {claim_ref} references unknown flow {flow_ref}"));
            }
        }
        let claims = array(context, "actor_role_claims")?
            .iter()
            .map(|claim| {
                string(claim, "claim_ref").map(|id| (id.to_owned(), claim))
            })
            .collect::<Result<BTreeMap<_, _>, _>>()?;
        for (claim_ref, claim) in &claims {
            if string(claim, "assertion_state")? == "synthetic_disputed" {
                let conflicts = array(claim, "conflicts_with_claim_refs")?;
                if conflicts.is_empty() {
                    return Err(format!("{context_id}: disputed actor-role claim {claim_ref} has no conflict reference"));
                }
                for conflict in conflicts {
                    let conflict_ref = conflict.as_str().ok_or("conflict claim refs must be strings")?;
                    let other = claims.get(conflict_ref)
                        .ok_or_else(|| format!("{context_id}: actor-role claim {claim_ref} references unknown conflict {conflict_ref}"))?;
                    let reciprocal = other.get("conflicts_with_claim_refs")
                        .and_then(Value::as_array)
                        .is_some_and(|refs| refs.iter().any(|value| value.as_str() == Some(claim_ref)));
                    if !reciprocal {
                        return Err(format!("{context_id}: actor-role conflict {claim_ref}/{conflict_ref} is not reciprocal"));
                    }
                }
            }
        }

        for policy in array(context, "retention_policies")? {
            let retention_ref = string(policy, "retention_ref")?;
            let asset_ref = string(policy, "asset_ref")?;
            let purpose_ref = string(policy, "purpose_ref")?;
            if !assets.contains(asset_ref) || !purposes.contains(purpose_ref) {
                return Err(format!("{context_id}: retention policy {retention_ref} has dangling asset/purpose reference"));
            }
            if policy.get("legal_hold_state").and_then(Value::as_str) == Some("active")
                && policy.get("deletion_state").and_then(Value::as_str) == Some("executed")
            {
                return Err(format!("{context_id}: retention policy {retention_ref} executes deletion while legal hold is active"));
            }
        }
    }
    Ok(())
}

fn validate_oracle(candidates: &Value, oracle: &Value) -> Result<(), String> {
    let context_ids = array(candidates, "contexts")?
        .iter()
        .map(|context| string(context, "context_id").map(str::to_owned))
        .collect::<Result<BTreeSet<_>, _>>()?;
    let mut seen = BTreeSet::new();
    for expectation in array(oracle, "expectations")? {
        let context_id = string(expectation, "context_id")?;
        insert_unique(&mut seen, context_id, "oracle context expectation")?;
        if !context_ids.contains(context_id) {
            return Err(format!("oracle expectation references unknown context {context_id}"));
        }
    }
    if seen != context_ids {
        return Err("public oracle must contain exactly one expectation per candidate context".to_owned());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixtures() -> (Value, Value, Value, Value) {
        let schema = serde_json::from_str(include_str!(
            "../../../data-governance-v1/finance_data_flow_context_v1.0.0.schema.json"
        )).unwrap();
        let candidates = serde_json::from_str(include_str!(
            "../../../data-governance-v1/fixtures/finance_data_flow_candidates_v1.0.0.json"
        )).unwrap();
        let oracle_schema = serde_json::from_str(include_str!(
            "../../../data-governance-v1/finance_data_flow_oracle_v1.0.0.schema.json"
        )).unwrap();
        let oracle = serde_json::from_str(include_str!(
            "../../../data-governance-v1/fixtures/finance_data_flow_oracle_v1.0.0.json"
        )).unwrap();
        (schema, candidates, oracle_schema, oracle)
    }

    fn context<'a>(bundle: &'a Value, id: &str) -> &'a Value {
        array(bundle, "contexts").unwrap()
            .iter()
            .find(|context| context["context_id"] == id)
            .unwrap()
    }

    #[test]
    fn ten_candidate_contexts_and_public_oracle_validate() {
        let (schema, candidates, oracle_schema, oracle) = fixtures();
        validate_schema("context schema", &schema).unwrap();
        validate_instance("candidate bundle", &schema, &candidates).unwrap();
        validate_schema("oracle schema", &oracle_schema).unwrap();
        validate_instance("public oracle", &oracle_schema, &oracle).unwrap();
        validate_bundle_integrity(&candidates).unwrap();
        validate_oracle(&candidates, &oracle).unwrap();
        assert_eq!(array(&candidates, "contexts").unwrap().len(), 10);
        assert_eq!(array(&oracle, "expectations").unwrap().len(), 10);
    }

    #[test]
    fn foreign_support_access_is_distinct_from_local_storage() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-001-FOREIGN-SUPPORT");
        let support = array(ctx, "flows").unwrap().iter()
            .find(|flow| flow["operation"] == "support_access").unwrap();
        let locations = array(support, "location_observations").unwrap();
        assert!(locations.iter().any(|item| item["facet"] == "primary_storage" && item["scope_ref"] == "SYN-DG-SA"));
        assert!(locations.iter().any(|item| item["facet"] == "support_access" && item["scope_ref"] == "SYN-DG-FOREIGN-X"));
    }

    #[test]
    fn foreign_processor_storage_and_processing_are_separate_facets() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-002-FOREIGN-PROCESSOR");
        let flow = &array(ctx, "flows").unwrap()[0];
        let locations = array(flow, "location_observations").unwrap();
        assert!(locations.iter().any(|item| item["facet"] == "collection_source" && item["scope_ref"] == "SYN-DG-SA"));
        assert!(locations.iter().any(|item| item["facet"] == "primary_storage" && item["scope_ref"] == "SYN-DG-FOREIGN-X"));
        assert!(locations.iter().any(|item| item["facet"] == "active_processing" && item["scope_ref"] == "SYN-DG-FOREIGN-X"));
    }

    #[test]
    fn eu_and_uk_data_transfer_profiles_are_not_conflated() {
        let (_, candidates, _, _) = fixtures();
        let eu = context(&candidates, "DG-004-EU-ONWARD");
        let uk = context(&candidates, "DG-005-UK-REGIME-VERSION");
        assert_eq!(array(eu, "legal_profiles").unwrap().len(), 1);
        assert_eq!(eu["legal_profiles"][0]["profile_ref"], "SYN-PROFILE-EU-GDPR");
        assert_eq!(array(uk, "legal_profiles").unwrap().len(), 1);
        assert_eq!(uk["legal_profiles"][0]["profile_ref"], "SYN-PROFILE-UK-GDPR-2026");
        assert_eq!(uk["legal_profiles"][0]["effective_from"], "2026-02-05");
        assert_ne!(eu["legal_profiles"][0]["instrument_version"], uk["legal_profiles"][0]["instrument_version"]);
    }

    #[test]
    fn accounting_profile_alone_has_no_transfer_mechanism_authority() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-006-ACCOUNTING-ONLY");
        let flow = &array(ctx, "flows").unwrap()[0];
        assert_eq!(array(flow, "legal_profile_refs").unwrap(), &[json!("SYN-PROFILE-ACCOUNTING")]);
        assert!(array(flow, "transfer_mechanism_refs").unwrap().is_empty());
    }

    #[test]
    fn active_legal_hold_never_executes_deletion() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-007-RETENTION-LEGAL-HOLD");
        let retention = &array(ctx, "retention_policies").unwrap()[0];
        assert_eq!(retention["legal_hold_state"], "active");
        assert_eq!(retention["deletion_state"], "requested");
        assert_ne!(retention["deletion_state"], "executed");
    }

    #[test]
    fn secondary_purpose_expansion_is_unreviewed() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-008-UNAPPROVED-PURPOSE");
        let marketing = array(ctx, "purposes").unwrap().iter()
            .find(|purpose| purpose["purpose_ref"] == "SYN-DG-PURPOSE-MARKETING").unwrap();
        assert_eq!(marketing["status"], "synthetic_unreviewed");
        let flow = &array(ctx, "flows").unwrap()[0];
        assert_eq!(flow["purpose_ref"], "SYN-DG-PURPOSE-MARKETING");
    }

    #[test]
    fn unknown_remote_access_has_no_implied_local_scope() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-009-UNKNOWN-REMOTE-ACCESS");
        let flow = array(ctx, "flows").unwrap().iter()
            .find(|flow| flow["operation"] == "support_access").unwrap();
        let access = array(flow, "location_observations").unwrap().iter()
            .find(|location| location["facet"] == "support_access").unwrap();
        assert_eq!(access["assertion_state"], "unknown");
        assert!(access["scope_ref"].is_null());
    }

    #[test]
    fn eu_onward_transfer_contains_a_destination_outside_the_candidate_mechanism_scope() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-004-EU-ONWARD");
        let flow = &array(ctx, "flows").unwrap()[0];
        let mechanism = &array(ctx, "transfer_mechanisms").unwrap()[0];
        let destinations = array(flow, "location_observations").unwrap().iter()
            .filter(|location| location["facet"] == "onward_recipient")
            .map(|location| location["scope_ref"].as_str().unwrap())
            .collect::<BTreeSet<_>>();
        assert!(destinations.contains("SYN-DG-FOREIGN-X"));
        assert!(destinations.contains("SYN-DG-FOREIGN-Y"));
        assert_eq!(mechanism["destination_scope_ref"], "SYN-DG-FOREIGN-Y");
        assert_eq!(mechanism["status"], "synthetic_review_required");
    }

    #[test]
    fn disputed_actor_role_claims_are_flow_bound_and_reciprocal() {
        let (_, candidates, _, _) = fixtures();
        let ctx = context(&candidates, "DG-010-ACTOR-ROLE-CONFLICT");
        let claims = array(ctx, "actor_role_claims").unwrap();
        assert_eq!(claims.len(), 2);
        assert_eq!(claims[0]["flow_ref"], claims[1]["flow_ref"]);
        assert_eq!(claims[0]["actor_ref"], claims[1]["actor_ref"]);
        assert!(claims.iter().all(|claim| claim["assertion_state"] == "synthetic_disputed"));
        assert!(claims[0]["conflicts_with_claim_refs"][0] == claims[1]["claim_ref"]);
        assert!(claims[1]["conflicts_with_claim_refs"][0] == claims[0]["claim_ref"]);
    }
}
