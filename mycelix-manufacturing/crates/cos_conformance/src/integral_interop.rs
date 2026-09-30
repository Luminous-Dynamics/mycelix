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

    let materials = required(value, "design_version.materials")?
        .as_array()
        .ok_or_else(|| "design_version.materials must be an array".to_string())?;
    for (index, material) in materials.iter().enumerate() {
        if !material.is_string() {
            return Err(format!("design_version.materials[{index}] must be a string"));
        }
    }

    let bill_of_materials = required(
        value,
        "design_version.parameters.bill_of_materials_kg",
    )?
    .as_object()
    .ok_or_else(|| "bill_of_materials_kg must be an object".to_string())?;
    for (material, quantity) in bill_of_materials {
        if quantity.as_i64().is_none() && quantity.as_u64().is_none() {
            return Err(format!(
                "bill_of_materials_kg[{material}] must be an integral JSON number"
            ));
        }
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

        let name = step
            .get("name")
            .ok_or_else(|| format!("missing required production_steps[{index}].name"))?;
        if !name.is_string() {
            return Err(format!(
                "production_steps[{index}].name must be a string"
            ));
        }

        let estimated_hours = step.get("estimated_hours").ok_or_else(|| {
            format!("missing required production_steps[{index}].estimated_hours")
        })?;
        if estimated_hours.as_i64().is_none() && estimated_hours.as_u64().is_none() {
            return Err(format!(
                "production_steps[{index}].estimated_hours must be an integral JSON number"
            ));
        }

        let skill_tier = step
            .get("skill_tier")
            .ok_or_else(|| format!("missing required production_steps[{index}].skill_tier"))?;
        if !skill_tier.is_string() {
            return Err(format!(
                "production_steps[{index}].skill_tier must be a string"
            ));
        }

        let tools_required = step.get("tools_required").ok_or_else(|| {
            format!("missing required production_steps[{index}].tools_required")
        })?;
        let tools = tools_required.as_array().ok_or_else(|| {
            format!(
                "production_steps[{index}].tools_required must be an array"
            )
        })?;
        for (tool_index, tool) in tools.iter().enumerate() {
            if !tool.is_string() {
                return Err(format!(
                    "production_steps[{index}].tools_required[{tool_index}] must be a string"
                ));
            }
        }

        let sequence_index = step.get("sequence_index").ok_or_else(|| {
            format!("missing required production_steps[{index}].sequence_index")
        })?;
        if sequence_index.as_i64().is_none() && sequence_index.as_u64().is_none() {
            return Err(format!(
                "production_steps[{index}].sequence_index must be an integral JSON number"
            ));
        }

        let safety_notes = step.get("safety_notes").ok_or_else(|| {
            format!("missing required production_steps[{index}].safety_notes")
        })?;
        if !safety_notes.is_string() {
            return Err(format!(
                "production_steps[{index}].safety_notes must be a string"
            ));
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
    fn wrong_selected_types_fail_closed() {
        let cases = [
            ("materials", serde_json::json!({"materials": "stainless-steel"})),
            (
                "bill_of_materials_kg",
                serde_json::json!({"bill_of_materials_kg": ["bad"]}),
            ),
        ];

        for (field, replacement) in cases {
            let mut changed = fixture();
            if field == "materials" {
                changed["design_version"]["materials"] = replacement["materials"].clone();
            } else {
                changed["design_version"]["parameters"]["bill_of_materials_kg"] =
                    replacement["bill_of_materials_kg"].clone();
            }
            assert!(
                selected_oad_design_semantic_commitment_checked(&changed).is_err(),
                "{field} wrong type must fail closed"
            );
        }

        let production_cases = [
            ("name", serde_json::json!(42)),
            ("estimated_hours", serde_json::json!(1.5)),
            ("skill_tier", serde_json::json!(false)),
            ("tools_required", serde_json::json!("press")),
            ("sequence_index", serde_json::json!(1.5)),
            ("safety_notes", serde_json::json!(["guarded press"])),
        ];

        for (field, replacement) in production_cases {
            let mut changed = fixture();
            changed["design_version"]["parameters"]["production_steps"][0][field] =
                replacement;
            assert!(
                selected_oad_design_semantic_commitment_checked(&changed).is_err(),
                "production_steps[0].{field} wrong type must fail closed"
            );
        }
    }

    #[test]
    fn integral_numeric_boundaries_remain_valid() {
        let mut changed = fixture();
        changed["design_version"]["parameters"]["bill_of_materials_kg"]["silicone"] =
            serde_json::json!(-1);
        changed["design_version"]["parameters"]["production_steps"][0]["estimated_hours"] =
            serde_json::json!(0);
        changed["design_version"]["parameters"]["production_steps"][0]["sequence_index"] =
            serde_json::json!(-1);

        assert!(selected_oad_design_semantic_commitment_checked(&changed).is_ok());
    }

    #[test]
    fn explicit_null_fails_closed() {
        let mut changed = fixture();
        changed["design_version"]["materials"] = Value::Null;
        assert!(selected_oad_design_semantic_commitment_checked(&changed).is_err());
    }
}
