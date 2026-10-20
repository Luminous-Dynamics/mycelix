//! Independent semantic checks for synthetic furnishing catalog profiles.
//!
//! This is not a JSON Schema engine and does not prove product properties. It
//! checks critical cross-field invariants that JSON Schema alone cannot express.

use serde_json::Value;
use std::collections::BTreeSet;

const ROOT_FIXTURE_STATUS: &str = "synthetic_only_not_marketplace_data";
const SYNTHETIC_PROFILE_STATUS: &str = "synthetic_fixture";
const LAYOUT_LICENSE: &str = "redisplay_and_layout_allowed";

fn field<'a>(value: &'a Value, key: &str) -> Option<&'a Value> {
    value.as_object()?.get(key)
}

fn text<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    field(value, key)?.as_str()
}

fn array<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    field(value, key)?.as_array()
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|b| b.is_ascii_hexdigit())
}

fn positive_finite(value: Option<&Value>) -> bool {
    value
        .and_then(Value::as_f64)
        .is_some_and(|number| number.is_finite() && number > 0.0)
}

fn string_set(items: Option<&Vec<Value>>, label: &str, errors: &mut Vec<String>) -> BTreeSet<String> {
    let mut result = BTreeSet::new();
    let Some(items) = items else {
        errors.push(format!("{label}: missing array"));
        return result;
    };
    for (index, item) in items.iter().enumerate() {
        let Some(value) = item.as_str() else {
            errors.push(format!("{label}[{index}]: expected string"));
            continue;
        };
        if value.trim().is_empty() {
            errors.push(format!("{label}[{index}]: blank identifier"));
            continue;
        }
        if !result.insert(value.to_owned()) {
            errors.push(format!("{label}: duplicate identifier {value:?}"));
        }
    }
    result
}

fn validate_profile(profile: &Value, label: &str, errors: &mut Vec<String>) {
    for required in [
        "schema_version",
        "profile_id",
        "status",
        "catalog_item",
        "marketplace_offer",
        "design_compatibility",
        "evidence",
        "claim_ceiling",
    ] {
        if field(profile, required).is_none() {
            errors.push(format!("{label}: missing required field {required}"));
        }
    }

    if text(profile, "schema_version") != Some("0.1.0") {
        errors.push(format!("{label}: unsupported schema_version"));
    }
    if text(profile, "status") != Some(SYNTHETIC_PROFILE_STATUS) {
        errors.push(format!("{label}: this fixture corpus only accepts synthetic_fixture status"));
    }

    let Some(item) = field(profile, "catalog_item") else {
        return;
    };
    let Some(geometry) = field(item, "geometry") else {
        errors.push(format!("{label}: missing catalog_item.geometry"));
        return;
    };

    // Geometry field semantics.
    let dimension_state = text(geometry, "dimension_state").unwrap_or("");
    let dimensions = ["width", "depth", "height"];
    if dimension_state == "unknown" {
        for name in dimensions {
            if field(geometry, name).is_none_or(|value| !value.is_null()) {
                errors.push(format!("{label}: unknown dimensions must be null ({name})"));
            }
        }
        if field(geometry, "geometry_digest_sha256").is_some_and(|value| !value.is_null()) {
            errors.push(format!("{label}: unknown dimensions cannot carry a geometry digest"));
        }
        if array(geometry, "geometry_evidence_refs").is_some_and(|refs| !refs.is_empty()) {
            errors.push(format!("{label}: unknown dimensions cannot claim geometry evidence references"));
        }
    } else {
        for name in dimensions {
            if !positive_finite(field(geometry, name)) {
                errors.push(format!("{label}: {dimension_state} geometry needs a positive finite {name}"));
            }
        }
        match text(geometry, "geometry_digest_sha256") {
            Some(digest) if is_sha256(digest) => {}
            _ => errors.push(format!("{label}: measured/specified geometry needs a SHA-256 digest")),
        }
        if array(geometry, "geometry_evidence_refs").is_none_or(Vec::is_empty) {
            errors.push(format!("{label}: known dimensions need at least one geometry evidence reference"));
        }
    }

    let geometry_refs = string_set(
        array(geometry, "geometry_evidence_refs"),
        &format!("{label}.geometry_evidence_refs"),
        errors,
    );
    let geometry_license = text(geometry, "geometry_use_license").unwrap_or("unknown");

    // Evidence identity and scoped references.
    let evidence = array(profile, "evidence");
    let mut evidence_by_id = std::collections::BTreeMap::<String, &Value>::new();
    if let Some(items) = evidence {
        for (index, record) in items.iter().enumerate() {
            let Some(id) = text(record, "evidence_ref") else {
                errors.push(format!("{label}.evidence[{index}]: missing evidence_ref"));
                continue;
            };
            if evidence_by_id.insert(id.to_owned(), record).is_some() {
                errors.push(format!("{label}: duplicate evidence_ref {id:?}"));
            }
            if let Some(digest) = field(record, "source_digest_sha256").and_then(Value::as_str) {
                if !is_sha256(digest) {
                    errors.push(format!("{label}.evidence[{index}]: malformed source SHA-256"));
                }
            }
        }
    } else {
        errors.push(format!("{label}: evidence must be an array"));
    }

    for reference in &geometry_refs {
        let Some(record) = evidence_by_id.get(reference) else {
            errors.push(format!("{label}: unresolved geometry evidence reference {reference:?}"));
            continue;
        };
        if text(record, "claim_kind") != Some("geometry") {
            errors.push(format!("{label}: geometry reference {reference:?} is not geometry evidence"));
        }
        if text(record, "validity_state") != Some("valid_for_scoped_claim") {
            errors.push(format!("{label}: geometry evidence {reference:?} is not valid for its scoped claim"));
        }
        let item_digest = text(geometry, "geometry_digest_sha256");
        let evidence_digest = text(record, "source_digest_sha256");
        if item_digest.is_none() || item_digest != evidence_digest {
            errors.push(format!("{label}: geometry/evidence digest mismatch for {reference:?}"));
        }
    }

    if let Some(compatibility_record) = field(profile, "design_compatibility") {
        if let Some(results) = array(compatibility_record, "hard_constraint_results") {
            for (index, result) in results.iter().enumerate() {
                let Some(refs) = array(result, "evidence_refs") else {
                    continue;
                };
                for reference in refs.iter().filter_map(Value::as_str) {
                    if !evidence_by_id.contains_key(reference) {
                        errors.push(format!("{label}.hard_constraint_results[{index}]: unresolved evidence reference {reference:?}"));
                    }
                }
            }
        }
    }

    // A model cannot report a positive compatibility result when geometry
    // is missing, unlicensed for layout, unsupported by scoped evidence, or
    // contradicted by any hard constraint.
    let compatibility = field(profile, "design_compatibility")
        .and_then(|value| text(value, "compatibility_state"))
        .unwrap_or("not_checked");
    if compatibility == "compatible_for_declared_checks" {
        if dimension_state == "unknown" {
            errors.push(format!("{label}: unknown geometry cannot be marked compatible"));
        }
        if geometry_license != LAYOUT_LICENSE {
            errors.push(format!("{label}: geometry-use rights do not permit layout compatibility"));
        }
        if geometry_refs.is_empty() {
            errors.push(format!("{label}: compatibility requires scoped geometry evidence"));
        }
        let results = field(profile, "design_compatibility").and_then(|value| array(value, "hard_constraint_results"));
        if results.is_none_or(Vec::is_empty) {
            errors.push(format!("{label}: compatibility needs explicit hard-constraint results"));
        } else if results.is_some_and(|items| items.iter().any(|value| text(value, "outcome") != Some("pass"))) {
            errors.push(format!("{label}: compatible result contradicts a failed or indeterminate hard constraint"));
        }
    }

    // In a synthetic fixture, commerce terms must not masquerade as live,
    // quoted, or confirmed inventory state.
    if let Some(offer) = field(profile, "marketplace_offer") {
        if text(offer, "observed_state") == Some("fixture") {
            for (key, forbidden, description) in [
                ("price_state", "quoted", "live quote"),
                ("availability_state", "available_confirmed", "confirmed live availability"),
                ("delivery_state", "delivery_quoted", "delivery quote"),
            ] {
                if text(offer, key) == Some(forbidden) {
                    errors.push(format!("{label}: fixture cannot claim {description}"));
                }
            }
        }
        if text(offer, "availability_state") == Some("available_confirmed")
            && text(offer, "observed_state") != Some("live_read")
        {
            errors.push(format!("{label}: confirmed availability requires a live source read"));
        }
        if text(offer, "price_state") == Some("quoted")
            && text(offer, "observed_state") != Some("live_read")
        {
            errors.push(format!("{label}: quoted price requires a live source read"));
        }
        if text(offer, "delivery_state") == Some("delivery_quoted")
            && text(offer, "observed_state") != Some("live_read")
        {
            errors.push(format!("{label}: delivery quote requires a live source read"));
        }
    }
}

pub fn validate_fixture(document: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if text(document, "fixture_status") != Some(ROOT_FIXTURE_STATUS) {
        errors.push(format!("root.fixture_status must equal {ROOT_FIXTURE_STATUS:?}"));
    }
    if text(document, "fixture_status").is_some_and(|status| status != ROOT_FIXTURE_STATUS) {
        errors.push("refusing to treat this corpus as live marketplace data".to_owned());
    }

    let Some(profiles) = array(document, "profiles") else {
        errors.push("root.profiles must be an array".to_owned());
        return errors;
    };
    if profiles.is_empty() {
        errors.push("root.profiles must not be empty".to_owned());
    }

    let mut ids = BTreeSet::new();
    for (index, profile) in profiles.iter().enumerate() {
        let label = text(profile, "profile_id")
            .map(str::to_owned)
            .unwrap_or_else(|| format!("profile[{index}]"));
        if !ids.insert(label.clone()) {
            errors.push(format!("duplicate profile_id {label:?}"));
        }
        validate_profile(profile, &label, &mut errors);
    }
    errors
}

#[cfg(test)]
mod tests {
    use super::validate_fixture;
    use serde_json::Value;
    use std::path::PathBuf;

    fn baseline() -> Value {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../contracts/fixtures/furnishing-catalog-profile-v0.1.json");
        let bytes = std::fs::read(&path).expect("synthetic fixture should exist");
        serde_json::from_slice(&bytes).expect("synthetic fixture should be valid JSON")
    }

    fn errors_after(mut document: Value, mutate: impl FnOnce(&mut Value)) -> Vec<String> {
        mutate(&mut document);
        validate_fixture(&document)
    }

    #[test]
    fn frozen_synthetic_fixture_passes_semantic_audit() {
        let document = baseline();
        let errors = validate_fixture(&document);
        assert!(errors.is_empty(), "unexpected audit errors: {errors:#?}");
    }

    #[test]
    fn duplicate_profile_identity_is_rejected() {
        let errors = errors_after(baseline(), |doc| {
            let first = doc["profiles"][0].clone();
            doc["profiles"].as_array_mut().unwrap().push(first);
        });
        assert!(errors.iter().any(|error| error.contains("duplicate profile_id")));
    }

    #[test]
    fn unknown_dimensions_cannot_be_promoted_to_compatible() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][2]["design_compatibility"]["compatibility_state"] =
                Value::String("compatible_for_declared_checks".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("unknown geometry cannot be marked compatible")));
    }

    #[test]
    fn unlicensed_geometry_cannot_be_promoted_to_compatible() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][1]["design_compatibility"]["compatibility_state"] =
                Value::String("compatible_for_declared_checks".to_owned());
            for result in doc["profiles"][1]["design_compatibility"]["hard_constraint_results"]
                .as_array_mut().unwrap()
            {
                result["outcome"] = Value::String("pass".to_owned());
            }
        });
        assert!(errors.iter().any(|error| error.contains("geometry-use rights do not permit layout compatibility")));
    }

    #[test]
    fn geometry_digest_substitution_is_rejected() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][0]["evidence"][0]["source_digest_sha256"] =
                Value::String("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("geometry/evidence digest mismatch")));
    }

    #[test]
    fn synthetic_offer_cannot_claim_confirmed_live_stock() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][0]["marketplace_offer"]["availability_state"] =
                Value::String("available_confirmed".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("confirmed availability requires a live source read")));
    }

    #[test]
    fn synthetic_offer_cannot_claim_a_live_quote() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][0]["marketplace_offer"]["price_state"] =
                Value::String("quoted".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("quoted price requires a live source read")));
    }

    #[test]
    fn missing_geometry_evidence_is_rejected() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][0]["catalog_item"]["geometry"]["geometry_evidence_refs"][0] =
                Value::String("missing-evidence".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("unresolved geometry evidence reference")));
    }

    #[test]
    fn failed_hard_constraint_cannot_coexist_with_compatibility() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][0]["design_compatibility"]["hard_constraint_results"][0]["outcome"] =
                Value::String("fail".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("compatible result contradicts a failed or indeterminate hard constraint")));
    }

    #[test]
    fn malformed_geometry_digest_is_rejected() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][0]["catalog_item"]["geometry"]["geometry_digest_sha256"] =
                Value::String("not-a-digest".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("needs a SHA-256 digest")));
    }

    #[test]
    fn fixture_cannot_be_relabelled_as_live_data() {
        let errors = errors_after(baseline(), |doc| {
            doc["fixture_status"] = Value::String("live_marketplace_data".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("refusing to treat this corpus as live marketplace data")));
    }

    #[test]
    fn unresolved_constraint_evidence_is_rejected() {
        let errors = errors_after(baseline(), |doc| {
            doc["profiles"][0]["design_compatibility"]["hard_constraint_results"][0]["evidence_refs"][0] =
                Value::String("evidence-never-defined".to_owned());
        });
        assert!(errors.iter().any(|error| error.contains("unresolved evidence reference")));
    }
}


/// Validate an adapter request that is explicitly discovery-only. Every
/// externally visible or commercial side effect must remain false until a
/// distinct, resident-authorized purchase handoff is designed and qualified.
pub fn validate_discovery_request(document: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if text(document, "schema_version") != Some("0.1.0") {
        errors.push("request: unsupported schema_version".to_owned());
    }
    if text(document, "mode") != Some("discovery_only") {
        errors.push("request: only discovery_only mode is accepted".to_owned());
    }

    let Some(snapshot) = field(document, "design_snapshot") else {
        errors.push("request: missing design_snapshot".to_owned());
        return errors;
    };
    if text(snapshot, "dimensions_unit") != Some("m") {
        errors.push("request: design dimensions must be expressed in metres".to_owned());
    }
    if text(snapshot, "spatial_evaluation_location") != Some("local_or_trusted_private_client") {
        errors.push("request: spatial evaluation must remain local or in a trusted private client".to_owned());
    }
    if field(snapshot, "full_geometry_transmission").and_then(Value::as_bool) != Some(false) {
        errors.push("request: full geometry transmission must be false".to_owned());
    }
    match text(snapshot, "design_context_digest_sha256") {
        Some(digest) if is_sha256(digest) => {}
        _ => errors.push("request: malformed design_context_digest_sha256".to_owned()),
    }

    let Some(data_minimization) = field(document, "data_minimization") else {
        errors.push("request: missing data_minimization".to_owned());
        return errors;
    };
    for forbidden in [
        "exact_address_transmitted",
        "occupant_profile_transmitted",
        "floor_plan_transmitted",
        "unrelated_household_preferences_transmitted",
        "design_context_digest_transmitted",
        "query_digest_transmitted",
    ] {
        if field(data_minimization, forbidden).and_then(Value::as_bool) != Some(false) {
            errors.push(format!("request: {forbidden} must be false"));
        }
    }
    const ALLOWED_QUERY_FIELDS: &[&str] = &[
        "category",
        "bounding_dimensions_m",
        "condition_preference",
        "coarse_delivery_region",
        "evidence_completeness",
        "repairability_claim_state",
        "geometry_use_rights",
    ];
    if let Some(fields) = array(data_minimization, "fields_permitted_for_query") {
        let mut seen = BTreeSet::new();
        for value in fields {
            let Some(name) = value.as_str() else {
                errors.push("request: permitted query fields must be strings".to_owned());
                continue;
            };
            if !ALLOWED_QUERY_FIELDS.contains(&name) {
                errors.push(format!("request: query field {name:?} exceeds the allowlist"));
            }
            if !seen.insert(name) {
                errors.push(format!("request: duplicate query field {name:?}"));
            }
        }
    } else {
        errors.push("request: fields_permitted_for_query must be an array".to_owned());
    }

    let Some(effects) = field(document, "effects") else {
        errors.push("request: missing effects".to_owned());
        return errors;
    };
    for forbidden in [
        "create_order",
        "reserve_stock",
        "create_payment",
        "contact_seller",
        "upload_floor_plan",
        "transmit_exact_address",
        "commit_transaction",
    ] {
        if field(effects, forbidden).and_then(Value::as_bool) != Some(false) {
            errors.push(format!("request: effect {forbidden} must be false in discovery-only mode"));
        }
    }

    if let Some(authorization) = field(document, "authorization") {
        if field(authorization, "resident_selection_required").and_then(Value::as_bool) != Some(true) {
            errors.push("request: resident_selection_required must be true".to_owned());
        }
        if text(authorization, "purchase_authorization_state") != Some("not_requested") {
            errors.push("request: purchase authorization must not be requested during discovery".to_owned());
        }
    } else {
        errors.push("request: missing authorization".to_owned());
    }

    if let Some(query) = field(document, "query") {
        match text(query, "query_digest_sha256") {
            Some(digest) if is_sha256(digest) => {}
            _ => errors.push("request: malformed query_digest_sha256".to_owned()),
        }
        if let Some(dimensions) = field(query, "minimum_dimensions_m") {
            for name in ["width", "depth", "height"] {
                if !positive_finite(field(dimensions, name)) {
                    errors.push(format!("request: minimum_dimensions_m.{name} must be positive and finite"));
                }
            }
        } else {
            errors.push("request: missing minimum_dimensions_m".to_owned());
        }
    } else {
        errors.push("request: missing query".to_owned());
    }

    errors
}

#[cfg(test)]
mod discovery_request_tests;
