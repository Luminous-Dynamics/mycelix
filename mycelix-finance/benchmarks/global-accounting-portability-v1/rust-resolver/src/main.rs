use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

type AppResult<T> = Result<T, String>;

fn main() {
    if let Err(error) = run_cli() {
        eprintln!("mycelix-global-profile-resolver: {error}");
        std::process::exit(2);
    }
}

fn run_cli() -> Result<(), Box<dyn Error>> {
    let mut registry_path = PathBuf::from("../profile_reference_v1.0.0.json");
    let mut registry_schema_path = PathBuf::from("../profile_reference_v1.0.0.schema.json");
    let mut request_path = PathBuf::from("fixtures/synthetic-qualified-request.json");
    let mut request_schema_path = PathBuf::from("resolve_request.schema.json");
    let mut report_schema_path = PathBuf::from("report.schema.json");
    let mut output_path: Option<PathBuf> = None;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--registry" => registry_path = PathBuf::from(next_arg(&mut args, "--registry")?),
            "--registry-schema" => {
                registry_schema_path = PathBuf::from(next_arg(&mut args, "--registry-schema")?)
            }
            "--request" => request_path = PathBuf::from(next_arg(&mut args, "--request")?),
            "--request-schema" => {
                request_schema_path = PathBuf::from(next_arg(&mut args, "--request-schema")?)
            }
            "--report-schema" => {
                report_schema_path = PathBuf::from(next_arg(&mut args, "--report-schema")?)
            }
            "--out" => output_path = Some(PathBuf::from(next_arg(&mut args, "--out")?)),
            "--help" | "-h" => {
                print_help();
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}; use --help").into()),
        }
    }

    let registry_bytes = fs::read(&registry_path)?;
    let registry_schema_bytes = fs::read(&registry_schema_path)?;
    let request_bytes = fs::read(&request_path)?;
    let request_schema_bytes = fs::read(&request_schema_path)?;
    let report_schema_bytes = fs::read(&report_schema_path)?;

    let registry: Value = serde_json::from_slice(&registry_bytes)?;
    let registry_schema: Value = serde_json::from_slice(&registry_schema_bytes)?;
    let request: Value = serde_json::from_slice(&request_bytes)?;
    let request_schema: Value = serde_json::from_slice(&request_schema_bytes)?;
    let report_schema: Value = serde_json::from_slice(&report_schema_bytes)?;

    validate_instance("profile registry", &registry_schema, &registry)?;
    validate_instance("resolution request", &request_schema, &request)?;
    validate_registry_semantics(&registry)?;

    let mut report = resolve(&registry, &request)?;
    report["digests"] = json!({
        "registry_sha256": sha256_hex(&registry_bytes),
        "registry_schema_sha256": sha256_hex(&registry_schema_bytes),
        "request_sha256": sha256_hex(&request_bytes),
        "request_schema_sha256": sha256_hex(&request_schema_bytes),
        "resolver_source_sha256": resolver_source_digest()
    });
    report["network_access_during_resolution"] = Value::Bool(false);

    validate_instance("resolution report", &report_schema, &report)?;
    let rendered = serde_json::to_string_pretty(&report)?;
    match output_path {
        Some(path) => fs::write(path, format!("{rendered}\n"))?,
        None => println!("{rendered}"),
    }
    Ok(())
}

fn next_arg(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, Box<dyn Error>> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a value").into())
}

fn print_help() {
    println!(
        "Mycelix Global Profile Resolver\n\n\
         --registry PATH         Profile reference catalogue\n\
         --registry-schema PATH  Draft 2020-12 catalogue schema\n\
         --request PATH          Resolution context request\n\
         --request-schema PATH   Resolution request schema\n\
         --report-schema PATH    Resolution report schema\n\
         --out PATH              Write JSON report to PATH (default: stdout)\n\
         --help                  Show this help"
    );
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> AppResult<()> {
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

fn string_field<'a>(value: &'a Value, key: &str) -> AppResult<&'a str> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("missing or invalid string field: {key}"))
}

fn string_array(value: &Value, key: &str) -> AppResult<Vec<String>> {
    let array = value
        .get(key)
        .and_then(Value::as_array)
        .ok_or_else(|| format!("missing or invalid array field: {key}"))?;
    let mut items = Vec::with_capacity(array.len());
    for item in array {
        let text = item
            .as_str()
            .ok_or_else(|| format!("{key} contains a non-string value"))?;
        items.push(text.to_owned());
    }
    Ok(items)
}

fn sorted_unique(values: Vec<String>, label: &str) -> AppResult<Vec<String>> {
    let set = values.iter().cloned().collect::<BTreeSet<_>>();
    if set.len() != values.len() {
        return Err(format!("{label} contains duplicate values"));
    }
    Ok(set.into_iter().collect())
}

fn array_of_objects<'a>(value: &'a Value, key: &str) -> AppResult<&'a Vec<Value>> {
    value
        .get(key)
        .and_then(Value::as_array)
        .ok_or_else(|| format!("missing or invalid array: {key}"))
}

fn validate_registry_semantics(registry: &Value) -> AppResult<()> {
    let mut profiles = BTreeMap::<String, Value>::new();
    for profile in array_of_objects(registry, "profile_references")? {
        let id = string_field(profile, "profile_id")?.to_owned();
        if profiles.insert(id.clone(), profile.clone()).is_some() {
            return Err(format!("duplicate profile_id: {id}"));
        }
    }

    let mut intersections = BTreeSet::new();
    for intersection in array_of_objects(registry, "qualification_intersections")? {
        let id = string_field(intersection, "intersection_id")?.to_owned();
        if !intersections.insert(id.clone()) {
            return Err(format!("duplicate intersection_id: {id}"));
        }
        let refs = sorted_unique(
            string_array(intersection, "profile_refs")?,
            "intersection profile_refs",
        )?;
        let dimensions = sorted_unique(
            string_array(intersection, "supported_dimensions")?,
            "intersection supported_dimensions",
        )?;
        let binding_rows = array_of_objects(intersection, "profile_bindings")?;
        let mut bindings = BTreeMap::<String, Value>::new();
        for binding in binding_rows {
            let profile_id = string_field(binding, "profile_id")?.to_owned();
            if bindings
                .insert(profile_id.clone(), binding.clone())
                .is_some()
            {
                return Err(format!(
                    "intersection {id} has duplicate profile binding: {profile_id}"
                ));
            }
        }
        if bindings.len() != refs.len() {
            return Err(format!(
                "intersection {id} profile binding count does not match profile_refs"
            ));
        }
        let mut actual_dimensions = BTreeSet::new();
        for profile_id in &refs {
            let profile = profiles.get(profile_id).ok_or_else(|| {
                format!("intersection {id} references unknown profile {profile_id}")
            })?;
            let binding = bindings.get(profile_id).ok_or_else(|| {
                format!("intersection {id} has no content binding for profile {profile_id}")
            })?;
            let expected_version = string_field(profile, "version")?;
            let bound_version = string_field(binding, "version")?;
            if bound_version != expected_version {
                return Err(format!(
                    "intersection {id} profile version binding mismatch: {profile_id}"
                ));
            }
            let expected_digest = profile_sha256(profile)?;
            let bound_digest = string_field(binding, "profile_sha256")?;
            if bound_digest != expected_digest {
                return Err(format!(
                    "intersection {id} profile content digest mismatch: {profile_id}"
                ));
            }
            actual_dimensions.insert(string_field(profile, "dimension")?.to_owned());
        }
        if bindings.keys().any(|profile_id| !refs.contains(profile_id)) {
            return Err(format!(
                "intersection {id} has a profile binding outside profile_refs"
            ));
        }
        for dimension in dimensions {
            if !actual_dimensions.contains(&dimension) {
                return Err(format!(
                    "intersection {id} claims unsupported dimension {dimension}"
                ));
            }
        }
        let scope_kind = string_field(intersection, "scope_kind")?;
        let claim_ceiling = string_field(intersection, "claim_ceiling")?;
        if scope_kind == "synthetic_fixture_only" && claim_ceiling != "synthetic_fixture_only" {
            return Err(format!(
                "synthetic intersection {id} has an invalid claim ceiling"
            ));
        }
        if scope_kind == "real_world" && claim_ceiling != "declared_scope_only" {
            return Err(format!(
                "real-world intersection {id} has an invalid claim ceiling"
            ));
        }
        let valid_from = string_field(intersection, "valid_from")?;
        if let Some(valid_to) = intersection.get("valid_to").and_then(Value::as_str)
            && valid_to < valid_from
        {
            return Err(format!("intersection {id} has valid_to before valid_from"));
        }
        let recorded_at = string_field(intersection, "recorded_at")?;
        let approved_at = string_field(intersection, "approved_at")?;
        if approved_at < recorded_at {
            return Err(format!(
                "intersection {id} is approved before it was recorded"
            ));
        }
        let evidence_refs = string_array(intersection, "evidence_refs")?
            .into_iter()
            .collect::<BTreeSet<_>>();
        let reviewer_refs = string_array(intersection, "reviewer_refs")?
            .into_iter()
            .collect::<BTreeSet<_>>();
        let intersection_approved_at = string_field(intersection, "approved_at")?;
        let mut early_application_profiles = BTreeSet::<String>::new();
        for qualification in array_of_objects(intersection, "early_application_qualifications")? {
            let profile_id = string_field(qualification, "profile_id")?;
            if !refs.iter().any(|selected| selected.as_str() == profile_id) {
                return Err(format!(
                    "early-application qualification {id} references a profile outside profile_refs: {profile_id}"
                ));
            }
            if !early_application_profiles.insert(profile_id.to_owned()) {
                return Err(format!(
                    "intersection {id} has duplicate early-application qualification for profile: {profile_id}"
                ));
            }
            let profile = profiles.get(profile_id).ok_or_else(|| {
                format!(
                    "early-application qualification {id} references unknown profile {profile_id}"
                )
            })?;
            if profile
                .get("early_application_allowed")
                .and_then(Value::as_bool)
                != Some(true)
            {
                return Err(format!(
                    "early-application qualification {id} references a profile that does not permit early application: {profile_id}"
                ));
            }
            let evidence_ref = string_field(qualification, "evidence_ref")?;
            if !evidence_refs.contains(evidence_ref) {
                return Err(format!(
                    "early-application qualification {id} evidence reference is not bound by the intersection: {evidence_ref}"
                ));
            }
            let reviewer_ref = string_field(qualification, "reviewer_ref")?;
            if !reviewer_refs.contains(reviewer_ref) {
                return Err(format!(
                    "early-application qualification {id} reviewer is not bound by the intersection: {reviewer_ref}"
                ));
            }
            if string_field(qualification, "approved_at")? > intersection_approved_at {
                return Err(format!(
                    "early-application qualification {id} is approved after the parent intersection"
                ));
            }
        }
        if string_field(intersection, "status")? == "qualified_for_declared_scope" {
            let reviewers = string_array(intersection, "reviewer_refs")?;
            let evidence = string_array(intersection, "evidence_refs")?;
            if reviewers.is_empty() || evidence.is_empty() {
                return Err(format!(
                    "qualified intersection {id} lacks review/evidence references"
                ));
            }
        }
    }
    Ok(())
}

fn resolve(registry: &Value, request: &Value) -> AppResult<Value> {
    let request_id = string_field(request, "request_id")?.to_owned();
    let intended_use = string_field(request, "intended_use")?.to_owned();
    let entity_scope = string_field(request, "entity_scope")?.to_owned();
    let period_start = string_field(request, "reporting_period_start")?.to_owned();
    let period_end = string_field(request, "reporting_period_end")?.to_owned();
    let knowledge_at = string_field(request, "knowledge_at")?.to_owned();
    let jurisdiction_scopes = sorted_unique(
        string_array(request, "jurisdiction_scopes")?,
        "jurisdiction_scopes",
    )?;
    let required_dimensions = sorted_unique(
        string_array(request, "required_dimensions")?,
        "required_dimensions",
    )?;
    let selected_profile_ids = sorted_unique(
        string_array(request, "selected_profile_ids")?,
        "selected_profile_ids",
    )?;
    let allow_synthetic = request
        .get("allow_synthetic")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let invalid_reporting_period = period_end < period_start;

    let mut profiles = BTreeMap::<String, Value>::new();
    for profile in array_of_objects(registry, "profile_references")? {
        let id = string_field(profile, "profile_id")?.to_owned();
        profiles.insert(id, profile.clone());
    }

    let mut selected_profiles = Vec::new();
    let mut resolved_dimensions = BTreeSet::new();
    let mut unsupported_profile_ids = BTreeSet::new();
    let mut blockers = BTreeSet::new();
    let mut unknown_profile_ids = BTreeSet::new();
    let mut review_required = false;

    for profile_id in &selected_profile_ids {
        match profiles.get(profile_id) {
            None => {
                unknown_profile_ids.insert(profile_id.clone());
                unsupported_profile_ids.insert(profile_id.clone());
                blockers.insert(format!("unknown_profile:{profile_id}"));
            }
            Some(profile) => {
                let dimension = string_field(profile, "dimension")?.to_owned();
                let status = string_field(profile, "status")?.to_owned();
                resolved_dimensions.insert(dimension.clone());
                selected_profiles.push(json!({
                    "profile_id": profile_id,
                    "version": profile.get("version").and_then(Value::as_str).unwrap_or("unknown"),
                    "profile_sha256": profile_sha256(profile)?,
                    "dimension": dimension,
                    "status": status
                }));
                match status.as_str() {
                    "qualified_for_declared_scope" => {}
                    "synthetic_only" if allow_synthetic => {}
                    "review_required" => {
                        review_required = true;
                        blockers.insert(format!("profile_requires_review:{profile_id}"));
                    }
                    "synthetic_only" => {
                        unsupported_profile_ids.insert(profile_id.clone());
                        blockers.insert(format!("synthetic_permission_required:{profile_id}"));
                    }
                    other => {
                        unsupported_profile_ids.insert(profile_id.clone());
                        blockers.insert(format!("profile_not_qualified:{profile_id}:{other}"));
                    }
                }
            }
        }
    }
    selected_profiles.sort_by(|a, b| {
        a.get("profile_id")
            .and_then(Value::as_str)
            .cmp(&b.get("profile_id").and_then(Value::as_str))
    });

    let resolved_dimension_set = resolved_dimensions.clone();
    let missing_dimensions = required_dimensions
        .iter()
        .filter(|dimension| !resolved_dimension_set.contains(*dimension))
        .cloned()
        .collect::<Vec<_>>();
    if !missing_dimensions.is_empty() {
        blockers.insert("required_dimensions_missing".to_owned());
    }
    if invalid_reporting_period {
        blockers.insert("reporting_period_end_precedes_start".to_owned());
    }

    let requested_profile_set = selected_profile_ids
        .iter()
        .cloned()
        .collect::<BTreeSet<_>>();
    let requested_jurisdiction_set = jurisdiction_scopes.iter().cloned().collect::<BTreeSet<_>>();
    let mut exact_intersections = Vec::<Value>::new();
    for intersection in array_of_objects(registry, "qualification_intersections")? {
        let intersection_profiles = string_array(intersection, "profile_refs")?
            .into_iter()
            .collect::<BTreeSet<_>>();
        let intersection_jurisdictions = string_array(intersection, "jurisdiction_scopes")?
            .into_iter()
            .collect::<BTreeSet<_>>();
        let same_entity =
            intersection.get("entity_scope").and_then(Value::as_str) == Some(entity_scope.as_str());
        if intersection_profiles == requested_profile_set
            && intersection_jurisdictions == requested_jurisdiction_set
            && same_entity
        {
            exact_intersections.push(intersection.clone());
        }
    }

    let mut matched_intersection_id: Option<String> = None;
    let mut scope_kind = "none".to_owned();
    let mut claim_ceiling = "no_qualification_claim".to_owned();
    let mut disposition;

    if invalid_reporting_period || !missing_dimensions.is_empty() {
        disposition = "indeterminate";
    } else if !unknown_profile_ids.is_empty() {
        disposition = "unsupported_profile";
    } else if exact_intersections.len() > 1 {
        disposition = "needs_review";
        blockers.insert("multiple_exact_intersections_ambiguous".to_owned());
    } else if exact_intersections.is_empty() {
        disposition = if review_required {
            "needs_review"
        } else {
            "unsupported_profile"
        };
        blockers.insert("no_exact_qualified_intersection".to_owned());
    } else {
        let intersection = &exact_intersections[0];
        let intersection_id = string_field(intersection, "intersection_id")?.to_owned();
        matched_intersection_id = Some(intersection_id.clone());
        scope_kind = string_field(intersection, "scope_kind")?.to_owned();
        let status = string_field(intersection, "status")?;
        let valid_from = string_field(intersection, "valid_from")?;
        let valid_to = intersection.get("valid_to").and_then(Value::as_str);
        let recorded_at = string_field(intersection, "recorded_at")?;
        let approved_at = string_field(intersection, "approved_at")?;
        let allowed_uses = string_array(intersection, "allowed_intended_uses")?;
        let supported_dimensions = string_array(intersection, "supported_dimensions")?
            .into_iter()
            .collect::<BTreeSet<_>>();
        let all_dimensions_supported = required_dimensions
            .iter()
            .all(|dimension| supported_dimensions.contains(dimension));
        let all_profile_statuses_valid = selected_profile_ids.iter().all(|profile_id| {
            profiles
                .get(profile_id)
                .and_then(|profile| profile.get("status"))
                .and_then(Value::as_str)
                == Some(if scope_kind == "synthetic_fixture_only" {
                    "synthetic_only"
                } else {
                    "qualified_for_declared_scope"
                })
        });
        let period_in_scope = period_start.as_str() >= valid_from
            && valid_to
                .map(|end| period_end.as_str() <= end)
                .unwrap_or(true);
        let known_and_approved =
            knowledge_at.as_str() >= recorded_at && knowledge_at.as_str() >= approved_at;
        let early_qualifications =
            array_of_objects(intersection, "early_application_qualifications")?;
        let mut profile_effective = true;
        let mut early_application_unknown = false;
        for profile_id in &selected_profile_ids {
            if let Some(profile) = profiles.get(profile_id)
                && let Some(profile_valid_from) = profile.get("valid_from").and_then(Value::as_str)
                && period_start.as_str() < profile_valid_from
            {
                let permits_early = profile
                    .get("early_application_allowed")
                    .and_then(Value::as_bool)
                    == Some(true);
                let qualification = early_qualifications.iter().find(|item| {
                    item.get("profile_id").and_then(Value::as_str) == Some(profile_id.as_str())
                });
                match (permits_early, qualification) {
                    (true, Some(item)) => {
                        let approved_at = string_field(item, "approved_at")?;
                        if knowledge_at.as_str() < approved_at {
                            early_application_unknown = true;
                            blockers.insert(format!(
                                "early_application_not_approved_at_knowledge_frontier:{profile_id}"
                            ));
                        }
                    }
                    _ => {
                        profile_effective = false;
                        blockers.insert(format!(
                            "profile_not_effective_for_reporting_period:{profile_id}"
                        ));
                    }
                }
            }
        }

        match status {
            "candidate" | "review_required" => {
                disposition = "needs_review";
                blockers.insert(format!("intersection_not_qualified:{status}"));
            }
            "suspended" | "retired" => {
                disposition = "unsupported_profile";
                blockers.insert(format!("intersection_unavailable:{status}"));
            }
            "qualified_for_declared_scope" => {
                if scope_kind == "synthetic_fixture_only" && !allow_synthetic {
                    disposition = "unsupported_profile";
                    blockers.insert("synthetic_permission_required_for_intersection".to_owned());
                } else if !all_profile_statuses_valid {
                    disposition = "unsupported_profile";
                    blockers.insert(
                        "selected_profile_status_incompatible_with_intersection_scope".to_owned(),
                    );
                } else if !period_in_scope {
                    disposition = "unsupported_profile";
                    blockers.insert("reporting_period_outside_intersection_validity".to_owned());
                } else if !profile_effective {
                    disposition = "unsupported_profile";
                    blockers
                        .insert("selected_profile_not_effective_for_reporting_period".to_owned());
                } else if early_application_unknown || !known_and_approved {
                    disposition = "indeterminate";
                    blockers.insert(
                        "intersection_or_early_application_not_yet_known_or_approved_at_frontier"
                            .to_owned(),
                    );
                } else if !allowed_uses.iter().any(|use_id| use_id == &intended_use) {
                    disposition = "unsupported_profile";
                    blockers.insert("intended_use_not_authorized_by_intersection".to_owned());
                } else if !all_dimensions_supported {
                    disposition = "partial_or_profile_scoped";
                    blockers
                        .insert("intersection_does_not_cover_all_required_dimensions".to_owned());
                } else {
                    disposition = "qualified_for_declared_scope";
                    claim_ceiling = string_field(intersection, "claim_ceiling")?.to_owned();
                }
            }
            other => {
                disposition = "unsupported_profile";
                blockers.insert(format!("unknown_intersection_status:{other}"));
            }
        }
    }

    Ok(json!({
        "report_schema": "global-profile-resolution-report.v1.0.0",
        "request_id": request_id,
        "disposition": disposition,
        "intended_use": intended_use,
        "jurisdiction_scopes": jurisdiction_scopes,
        "entity_scope": entity_scope,
        "reporting_period_start": period_start,
        "reporting_period_end": period_end,
        "knowledge_at": knowledge_at,
        "required_dimensions": required_dimensions,
        "resolved_dimensions": resolved_dimensions.into_iter().collect::<Vec<_>>(),
        "missing_dimensions": missing_dimensions,
        "selected_profiles": selected_profiles,
        "unsupported_profile_ids": unsupported_profile_ids.into_iter().collect::<Vec<_>>(),
        "matched_intersection_id": matched_intersection_id,
        "scope_kind": scope_kind,
        "claim_ceiling": claim_ceiling,
        "blocking_reasons": blockers.into_iter().collect::<Vec<_>>()
    }))
}

fn sha256_hex(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn append_canonical_json(value: &Value, output: &mut String) -> AppResult<()> {
    match value {
        Value::Null => output.push_str("null"),
        Value::Bool(value) => output.push_str(if *value { "true" } else { "false" }),
        Value::Number(value) => output.push_str(&value.to_string()),
        Value::String(value) => output.push_str(
            &serde_json::to_string(value)
                .map_err(|error| format!("could not encode canonical JSON string: {error}"))?,
        ),
        Value::Array(values) => {
            output.push('[');
            for (index, item) in values.iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                append_canonical_json(item, output)?;
            }
            output.push(']');
        }
        Value::Object(values) => {
            let mut keys = values.keys().collect::<Vec<_>>();
            keys.sort_by(|left, right| left.as_bytes().cmp(right.as_bytes()));
            output.push('{');
            for (index, key) in keys.iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                output
                    .push_str(&serde_json::to_string(*key).map_err(|error| {
                        format!("could not encode canonical JSON key: {error}")
                    })?);
                output.push(':');
                let child = values
                    .get(*key)
                    .ok_or_else(|| "canonical JSON object key disappeared".to_owned())?;
                append_canonical_json(child, output)?;
            }
            output.push('}');
        }
    }
    Ok(())
}

fn canonical_profile_bytes(profile: &Value) -> AppResult<Vec<u8>> {
    let mut canonical = String::new();
    append_canonical_json(profile, &mut canonical)?;
    Ok(canonical.into_bytes())
}

fn profile_sha256(profile: &Value) -> AppResult<String> {
    let bytes = canonical_profile_bytes(profile)?;
    Ok(sha256_hex(&bytes))
}

fn resolver_source_digest() -> String {
    let mut hasher = Sha256::new();
    for (path, bytes) in [
        ("Cargo.toml", include_bytes!("../Cargo.toml").as_slice()),
        ("README.md", include_bytes!("../README.md").as_slice()),
        (
            "resolve_request.schema.json",
            include_bytes!("../resolve_request.schema.json").as_slice(),
        ),
        (
            "report.schema.json",
            include_bytes!("../report.schema.json").as_slice(),
        ),
        ("src/main.rs", include_bytes!("main.rs").as_slice()),
    ] {
        hasher.update(path.as_bytes());
        hasher.update([0]);
        hasher.update(bytes);
        hasher.update([0]);
    }
    hex::encode(hasher.finalize())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn registry() -> Value {
        serde_json::from_str(include_str!("../../profile_reference_v1.0.0.json"))
            .expect("registry JSON")
    }

    fn registry_schema() -> Value {
        serde_json::from_str(include_str!("../../profile_reference_v1.0.0.schema.json"))
            .expect("registry schema JSON")
    }

    fn request_schema() -> Value {
        serde_json::from_str(include_str!("../resolve_request.schema.json"))
            .expect("request schema JSON")
    }

    fn report_schema() -> Value {
        serde_json::from_str(include_str!("../report.schema.json")).expect("report schema JSON")
    }

    fn request(name: &str) -> Value {
        let input = match name {
            "qualified" => include_str!("../fixtures/synthetic-qualified-request.json"),
            "disabled" => include_str!("../fixtures/synthetic-disabled-request.json"),
            "reference" => include_str!("../fixtures/reference-only-request.json"),
            "missing" => include_str!("../fixtures/missing-dimension-request.json"),
            _ => panic!("unknown fixture"),
        };
        serde_json::from_str(input).expect("request JSON")
    }

    // Test mutations must refresh the exact selected-profile binding; otherwise
    // the digest guard correctly rejects the registry before temporal rules run.
    fn refresh_profile_binding(registry: &mut Value, profile_id: &str) {
        let profile = registry["profile_references"]
            .as_array()
            .expect("profile references")
            .iter()
            .find(|item| item["profile_id"] == profile_id)
            .expect("profile exists")
            .clone();
        let version = profile["version"]
            .as_str()
            .expect("profile version")
            .to_owned();
        let digest = profile_sha256(&profile).expect("profile digest");
        let binding = registry["qualification_intersections"][0]["profile_bindings"]
            .as_array_mut()
            .expect("profile bindings")
            .iter_mut()
            .find(|item| item["profile_id"] == profile_id)
            .expect("profile binding exists");
        binding["version"] = json!(version);
        binding["profile_sha256"] = json!(digest);
    }

    fn checked_resolve(registry: &Value, request: &Value) -> Value {
        validate_instance("registry", &registry_schema(), registry).expect("registry schema");
        validate_instance("request", &request_schema(), request).expect("request schema");
        validate_registry_semantics(registry).expect("registry semantics");
        let mut report = resolve(registry, request).expect("resolver");
        report["digests"] = json!({
            "registry_sha256": "a".repeat(64),
            "registry_schema_sha256": "b".repeat(64),
            "request_sha256": "c".repeat(64),
            "request_schema_sha256": "d".repeat(64),
            "resolver_source_sha256": "e".repeat(64)
        });
        report["network_access_during_resolution"] = Value::Bool(false);
        validate_instance("report", &report_schema(), &report).expect("report schema");
        report
    }

    #[test]
    fn all_three_schemas_are_valid_draft_2020_12() {
        validate_instance("registry", &registry_schema(), &registry()).unwrap();
        validate_instance("request", &request_schema(), &request("qualified")).unwrap();
        let report = checked_resolve(&registry(), &request("qualified"));
        validate_instance("report", &report_schema(), &report).unwrap();
    }

    #[test]
    fn exact_synthetic_intersection_qualifies_only_in_explicit_synthetic_mode() {
        let report = checked_resolve(&registry(), &request("qualified"));
        assert_eq!(report["disposition"], "qualified_for_declared_scope");
        assert_eq!(report["claim_ceiling"], "synthetic_fixture_only");
        assert_eq!(
            report["matched_intersection_id"],
            "SYNTH-PORTABILITY-INTERSECTION-001"
        );
    }

    #[test]
    fn synthetic_profiles_without_explicit_permission_are_unsupported() {
        let report = checked_resolve(&registry(), &request("disabled"));
        assert_eq!(report["disposition"], "unsupported_profile");
        assert_eq!(report["claim_ceiling"], "no_qualification_claim");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("synthetic_permission_required")
        );
    }

    #[test]
    fn reference_only_profiles_do_not_become_qualified_by_coexistence() {
        let report = checked_resolve(&registry(), &request("reference"));
        assert_eq!(report["disposition"], "unsupported_profile");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("no_exact_qualified_intersection")
        );
        assert!(
            !report["unsupported_profile_ids"]
                .as_array()
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn missing_required_dimension_is_indeterminate() {
        let report = checked_resolve(&registry(), &request("missing"));
        assert_eq!(report["disposition"], "indeterminate");
        assert_eq!(report["missing_dimensions"][0], "tax_profile");
    }

    #[test]
    fn profile_array_order_does_not_change_result() {
        let registry = registry();
        let mut request = request("qualified");
        let before = checked_resolve(&registry, &request);
        request["selected_profile_ids"]
            .as_array_mut()
            .unwrap()
            .reverse();
        request["required_dimensions"]
            .as_array_mut()
            .unwrap()
            .reverse();
        request["jurisdiction_scopes"]
            .as_array_mut()
            .unwrap()
            .reverse();
        let after = checked_resolve(&registry, &request);
        assert_eq!(before, after);
    }

    #[test]
    fn a_superset_profile_set_never_matches_an_exact_intersection() {
        let registry = registry();
        let mut request = request("qualified");
        request["selected_profile_ids"]
            .as_array_mut()
            .unwrap()
            .push(json!("SYN-EXTRA-PROFILE-A"));
        let report = checked_resolve(&registry, &request);
        assert_ne!(report["disposition"], "qualified_for_declared_scope");
        assert_eq!(report["matched_intersection_id"], Value::Null);
    }

    #[test]
    fn entity_scope_mismatch_never_matches() {
        let registry = registry();
        let mut request = request("qualified");
        request["entity_scope"] = json!("different_synthetic_entity");
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "unsupported_profile");
        assert_eq!(report["matched_intersection_id"], Value::Null);
    }

    #[test]
    fn period_outside_intersection_validity_is_unsupported() {
        let registry = registry();
        let mut request = request("qualified");
        request["reporting_period_start"] = json!("2025-12-01");
        request["reporting_period_end"] = json!("2025-12-31");
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "unsupported_profile");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("outside_intersection_validity")
        );
    }

    #[test]
    fn duplicate_early_application_qualifications_fail_closed() {
        let mut registry = registry();
        let intersection = &mut registry["qualification_intersections"][0];
        intersection["evidence_refs"]
            .as_array_mut()
            .unwrap()
            .push(json!("SYN-EARLY-APPLICATION-APPROVAL"));
        let qualification = json!({
            "profile_id": "SYN-FRAMEWORK-A",
            "evidence_ref": "SYN-EARLY-APPLICATION-APPROVAL",
            "reviewer_ref": "SYNTHETIC-FIXTURE-MAINTAINER",
            "approved_at": "2026-11-05T01:25:00Z"
        });
        intersection["early_application_qualifications"] =
            json!([qualification.clone(), qualification]);
        assert!(validate_registry_semantics(&registry).is_err());
    }

    #[test]
    fn early_application_qualification_for_unselected_profile_fails_closed() {
        let mut registry = registry();
        let intersection = &mut registry["qualification_intersections"][0];
        intersection["evidence_refs"]
            .as_array_mut()
            .unwrap()
            .push(json!("SYN-EARLY-APPLICATION-APPROVAL"));
        intersection["early_application_qualifications"] = json!([{
            "profile_id": "IFRS-SMES-2025",
            "evidence_ref": "SYN-EARLY-APPLICATION-APPROVAL",
            "reviewer_ref": "SYNTHETIC-FIXTURE-MAINTAINER",
            "approved_at": "2026-11-05T01:25:00Z"
        }]);
        assert!(validate_registry_semantics(&registry).is_err());
    }

    #[test]
    fn profile_digest_is_independent_of_json_object_key_order() {
        let first: Value = serde_json::from_str(
            r#"{"profile_id":"SYN-PROFILE","version":"v1","dimension":"accounting_framework","status":"synthetic_only","authority_class":"synthetic_test_fixture","source_url":"https://example.invalid/profile","valid_from":null,"notes":"synthetic"}"#
        ).unwrap();
        let second: Value = serde_json::from_str(
            r#"{"notes":"synthetic","valid_from":null,"source_url":"https://example.invalid/profile","authority_class":"synthetic_test_fixture","status":"synthetic_only","dimension":"accounting_framework","version":"v1","profile_id":"SYN-PROFILE"}"#
        ).unwrap();
        assert_eq!(
            profile_sha256(&first).unwrap(),
            profile_sha256(&second).unwrap()
        );
    }

    #[test]
    fn fractional_second_knowledge_frontiers_are_rejected_by_request_schema() {
        let mut request = request("qualified");
        request["knowledge_at"] = json!("2026-11-05T02:00:00.000Z");
        assert!(validate_instance("request", &request_schema(), &request).is_err());
    }

    #[test]
    fn canonical_profile_digest_preserves_array_order() {
        let first: Value = serde_json::from_str(
            r#"{"profile_id":"SYN-PROFILE","version":"v1","dimension":"accounting_framework","status":"synthetic_only","authority_class":"synthetic_test_fixture","source_url":"https://example.invalid/profile","valid_from":null,"notes":"synthetic","labels":["one","two"]}"#
        ).unwrap();
        let reversed: Value = serde_json::from_str(
            r#"{"profile_id":"SYN-PROFILE","version":"v1","dimension":"accounting_framework","status":"synthetic_only","authority_class":"synthetic_test_fixture","source_url":"https://example.invalid/profile","valid_from":null,"notes":"synthetic","labels":["two","one"]}"#
        ).unwrap();
        assert_ne!(
            profile_sha256(&first).unwrap(),
            profile_sha256(&reversed).unwrap()
        );
    }

    #[test]
    fn profile_content_mutation_invalidates_approved_intersection() {
        let mut registry = registry();
        let profile = registry["profile_references"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["profile_id"] == "SYN-FRAMEWORK-A")
            .unwrap();
        profile["notes"] = json!("mutated content under unchanged profile ID and version");
        assert!(validate_registry_semantics(&registry).is_err());
    }

    #[test]
    fn profile_version_mutation_invalidates_approved_intersection() {
        let mut registry = registry();
        let profile = registry["profile_references"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["profile_id"] == "SYN-FRAMEWORK-A")
            .unwrap();
        profile["version"] = json!("synthetic-framework-a-revised");
        assert!(validate_registry_semantics(&registry).is_err());
    }

    #[test]
    fn missing_profile_content_binding_fails_closed() {
        let mut registry = registry();
        registry["qualification_intersections"][0]["profile_bindings"]
            .as_array_mut()
            .unwrap()
            .pop();
        assert!(validate_registry_semantics(&registry).is_err());
    }

    #[test]
    fn unknown_profile_id_is_unsupported() {
        let registry = registry();
        let mut request = request("qualified");
        request["selected_profile_ids"]
            .as_array_mut()
            .unwrap()
            .push(json!("UNKNOWN-PROFILE"));
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "unsupported_profile");
        assert!(
            report["unsupported_profile_ids"]
                .to_string()
                .contains("UNKNOWN-PROFILE")
        );
    }

    #[test]
    fn partial_dimension_coverage_never_claims_full_qualification() {
        let mut registry = registry();
        registry["qualification_intersections"][0]["supported_dimensions"]
            .as_array_mut()
            .unwrap()
            .retain(|dimension| dimension != "timezone_database");
        let report = checked_resolve(&registry, &request("qualified"));
        assert_eq!(report["disposition"], "partial_or_profile_scoped");
        assert_eq!(report["claim_ceiling"], "no_qualification_claim");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("does_not_cover_all_required_dimensions")
        );
    }

    #[test]
    fn multiple_exact_intersections_require_review_instead_of_arbitrary_selection() {
        let mut registry = registry();
        let mut duplicate = registry["qualification_intersections"][0].clone();
        duplicate["intersection_id"] = json!("SYNTH-PORTABILITY-INTERSECTION-002");
        registry["qualification_intersections"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
        let report = checked_resolve(&registry, &request("qualified"));
        assert_eq!(report["disposition"], "needs_review");
        assert_eq!(report["matched_intersection_id"], Value::Null);
    }

    #[test]
    fn jurisdiction_mismatch_does_not_fall_back_to_a_country_default() {
        let registry = registry();
        let mut request = request("qualified");
        request["jurisdiction_scopes"] = json!(["SYNTH-JUR-B"]);
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "unsupported_profile");
        assert_eq!(report["matched_intersection_id"], Value::Null);
    }

    #[test]
    fn intended_use_must_be_explicitly_allowed_by_the_intersection() {
        let registry = registry();
        let mut request = request("qualified");
        request["intended_use"] = json!("tax_calculation");
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "unsupported_profile");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("intended_use_not_authorized")
        );
    }

    #[test]
    fn early_application_is_rejected_when_profile_does_not_allow_it() {
        let mut registry = registry();
        let profile = registry["profile_references"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["profile_id"] == "SYN-FRAMEWORK-A")
            .unwrap();
        profile["valid_from"] = json!("2027-01-01");
        profile["early_application_allowed"] = json!(false);
        refresh_profile_binding(&mut registry, "SYN-FRAMEWORK-A");
        let report = checked_resolve(&registry, &request("qualified"));
        assert_eq!(report["disposition"], "unsupported_profile");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("profile_not_effective_for_reporting_period")
        );
    }

    #[test]
    fn reversed_reporting_period_is_indeterminate() {
        let registry = registry();
        let mut request = request("qualified");
        request["reporting_period_start"] = json!("2026-11-30");
        request["reporting_period_end"] = json!("2026-11-01");
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "indeterminate");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("reporting_period_end_precedes_start")
        );
    }

    #[test]
    fn historical_knowledge_frontier_cannot_see_future_approval() {
        let registry = registry();
        let mut request = request("qualified");
        request["knowledge_at"] = json!("2026-11-04T00:00:00Z");
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "indeterminate");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("not_yet_known_or_approved")
        );
    }

    #[test]
    fn no_intersection_is_not_inferred_from_individual_synthetic_profiles() {
        let mut registry = registry();
        registry["qualification_intersections"] = json!([]);
        let report = checked_resolve(&registry, &request("qualified"));
        assert_eq!(report["disposition"], "unsupported_profile");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("no_exact_qualified_intersection")
        );
    }

    #[test]
    fn duplicate_registry_profile_ids_fail_closed() {
        let mut registry = registry();
        let duplicate = registry["profile_references"][0].clone();
        registry["profile_references"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
        assert!(validate_registry_semantics(&registry).is_err());
    }

    #[test]
    fn duplicate_intersection_ids_fail_closed() {
        let mut registry = registry();
        let duplicate = registry["qualification_intersections"][0].clone();
        registry["qualification_intersections"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
        assert!(validate_registry_semantics(&registry).is_err());
    }

    #[test]
    fn future_effective_profile_requires_explicit_early_application_evidence() {
        let mut registry = registry();
        let profile = registry["profile_references"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["profile_id"] == "SYN-FRAMEWORK-A")
            .unwrap();
        profile["valid_from"] = json!("2027-01-01");
        profile["early_application_allowed"] = json!(true);
        refresh_profile_binding(&mut registry, "SYN-FRAMEWORK-A");

        let request = request("qualified");
        let without_evidence = checked_resolve(&registry, &request);
        assert_eq!(without_evidence["disposition"], "unsupported_profile");
        assert!(
            without_evidence["blocking_reasons"]
                .to_string()
                .contains("profile_not_effective_for_reporting_period")
        );

        let intersection = &mut registry["qualification_intersections"][0];
        intersection["evidence_refs"]
            .as_array_mut()
            .unwrap()
            .push(json!("SYN-EARLY-APPLICATION-APPROVAL"));
        intersection["early_application_qualifications"] = json!([{
            "profile_id": "SYN-FRAMEWORK-A",
            "evidence_ref": "SYN-EARLY-APPLICATION-APPROVAL",
            "reviewer_ref": "SYNTHETIC-FIXTURE-MAINTAINER",
            "approved_at": "2026-11-05T01:25:00Z"
        }]);
        let with_evidence = checked_resolve(&registry, &request);
        assert_eq!(with_evidence["disposition"], "qualified_for_declared_scope");
        assert_eq!(with_evidence["claim_ceiling"], "synthetic_fixture_only");
    }

    #[test]
    fn future_early_application_approval_is_not_visible_before_its_frontier() {
        let mut registry = registry();
        let profile = registry["profile_references"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["profile_id"] == "SYN-FRAMEWORK-A")
            .unwrap();
        profile["valid_from"] = json!("2027-01-01");
        profile["early_application_allowed"] = json!(true);
        refresh_profile_binding(&mut registry, "SYN-FRAMEWORK-A");
        let intersection = &mut registry["qualification_intersections"][0];
        intersection["evidence_refs"]
            .as_array_mut()
            .unwrap()
            .push(json!("SYN-EARLY-APPLICATION-APPROVAL"));
        intersection["early_application_qualifications"] = json!([{
            "profile_id": "SYN-FRAMEWORK-A",
            "evidence_ref": "SYN-EARLY-APPLICATION-APPROVAL",
            "reviewer_ref": "SYNTHETIC-FIXTURE-MAINTAINER",
            "approved_at": "2026-11-05T01:25:00Z"
        }]);
        let mut request = request("qualified");
        request["knowledge_at"] = json!("2026-11-05T01:00:00Z");
        let report = checked_resolve(&registry, &request);
        assert_eq!(report["disposition"], "indeterminate");
        assert!(
            report["blocking_reasons"]
                .to_string()
                .contains("early_application_not_approved_at_knowledge_frontier")
        );
    }

    #[test]
    fn rotations_and_reversals_preserve_set_resolution() {
        let registry = registry();
        let original_request = request("qualified");
        let expected = checked_resolve(&registry, &original_request);
        for shift in 0..8 {
            let mut candidate = original_request.clone();
            let values = candidate["selected_profile_ids"].as_array_mut().unwrap();
            values.rotate_left(shift);
            let dimensions = candidate["required_dimensions"].as_array_mut().unwrap();
            dimensions.rotate_left(shift);
            let actual = checked_resolve(&registry, &candidate);
            assert_eq!(expected, actual);
        }
    }
}
