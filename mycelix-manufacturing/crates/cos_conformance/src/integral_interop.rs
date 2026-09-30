//! Integral interoperability semantic projection.
//!
//! ReferenceModelOnly. This module makes the D6X dependency boundary explicit:
//! the full OAD record may contain certification, lifecycle, provenance, and
//! archival metadata, while D6X consumes only the named production semantics
//! selected by the closure profile.

use serde_json::{json, Value};

use crate::canonical_derivation_receipt::canonical_sha256;

pub const INTEGRAL_INTEROP_PROFILE: &str = "integral-interop-1";
pub const OAD_SEMANTIC_PROJECTION_VERSION: &str = "integral-interop-1-design-semantic-v1";

fn required<'a>(root: &'a Value, path: &str) -> Result<&'a Value, String> {
    let mut current = root;
    for segment in path.split('.') {
        current = current
            .get(segment)
            .ok_or_else(|| format!("missing required Integral field: {path}"))?;
        if current.is_null() {
            return Err(format!("required Integral field is null: {path}"));
        }
    }
    Ok(current)
}

/// Validate the exact structural inputs selected by the current semantic profile.
pub fn validate_selected_oad_design_semantics(value: &Value) -> Result<(), String> {
    let design_id = required(value, "design_version.id")?;
    if !design_id.is_string() {
        return Err("design_version.id must be a string".into());
    }

    let spec_id = required(value, "design_version.spec_id")?;
    if !spec_id.is_string() {
        return Err("design_version.spec_id must be a string".into());
    }

    if !required(value, "design_version.materials")?.is_array() {
        return Err("design_version.materials must be an array".into());
    }

    if !required(
        value,
        "design_version.parameters.bill_of_materials_kg",
    )?
    .is_object()
    {
        return Err("bill_of_materials_kg must be an object".into());
    }

    let steps = required(
        value,
        "design_version.parameters.production_steps",
    )?
    .as_array()
    .ok_or_else(|| "production_steps must be an array".to_string())?;

    for (index, step) in steps.iter().enumerate() {
        if !step.is_object() {
            return Err(format!("production step {index} must be an object"));
        }
        for field in [
            "name",
            "estimated_hours",
            "skill_tier",
            "tools_required",
            "sequence_index",
            "safety_notes",
        ] {
            let field_value = step
                .get(field)
                .ok_or_else(|| format!("missing required production_steps[{index}].{field}"))?;
            if field_value.is_null() {
                return Err(format!(
                    "required production_steps[{index}].{field} is null"
                ));
            }
        }
    }

    Ok(())
}

/// Extract the OAD fields that are semantic dependencies for the current
/// Integral OAD -> COS -> material reference closure.
pub fn selected_oad_design_semantic_projection(value: &Value) -> Value {
    json!({
        "design_version_id": value["design_version"]["id"],
        "spec_id": value["design_version"]["spec_id"],
        "materials": value["design_version"]["materials"],
        "bill_of_materials_kg": value["design_version"]["parameters"]["bill_of_materials_kg"],
        "production_steps": value["design_version"]["parameters"]["production_steps"],
    })
}

/// Commitment for the selected OAD semantic projection consumed by D6X.
pub fn selected_oad_design_semantic_commitment_checked(value: &Value) -> Result<String, String> {
    validate_selected_oad_design_semantics(value)?;
    Ok(canonical_sha256(
        OAD_SEMANTIC_PROJECTION_VERSION,
        &selected_oad_design_semantic_projection(value),
    ))
}

pub fn selected_oad_design_semantic_commitment(value: &Value) -> String {
    selected_oad_design_semantic_commitment_checked(value)
        .expect("selected Integral semantic projection must be structurally valid")
}

#[cfg(test)]
mod tests {
    use super::*;
    
    fn fixture() -> Value {
        serde_json::from_str(include_str!(
            "../testdata/integral_interop_1_design.json"
        ))
        .expect("Integral fixture must be valid JSON")
    }

    #[test]
    fn certification_metadata_is_outside_selected_projection() {
        let baseline = fixture();
        let mut changed = baseline.clone();
        changed["certification"]["documentation_bundle_uri"] =
            Value::from("urn:integral:bundle:changed");

        assert_ne!(
            canonical_sha256(INTEGRAL_INTEROP_PROFILE, &baseline),
            canonical_sha256(INTEGRAL_INTEROP_PROFILE, &changed)
        );
        assert_eq!(
            selected_oad_design_semantic_commitment(&baseline),
            selected_oad_design_semantic_commitment(&changed)
        );
    }

    #[test]
    fn selected_production_semantics_change_selected_commitment() {
        let baseline = fixture();
        let mut changed = baseline.clone();
        changed["design_version"]["parameters"]["production_steps"][1]["estimated_hours"] =
            Value::from(3);

        assert_ne!(
            selected_oad_design_semantic_commitment(&baseline),
            selected_oad_design_semantic_commitment(&changed)
        );
    }

    #[test]
    fn missing_required_field_fails_closed() {
        let mut changed = fixture();
        changed["design_version"].as_object_mut().unwrap().remove("id");
        assert!(selected_oad_design_semantic_commitment_checked(&changed).is_err());
    }

    #[test]
    fn explicit_null_fails_closed() {
        let mut changed = fixture();
        changed["design_version"]["materials"] = Value::Null;
        assert!(selected_oad_design_semantic_commitment_checked(&changed).is_err());
    }
}
