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

/// Extract the OAD fields that are semantic dependencies for the current
/// Integral OAD -> COS -> material reference closure.
///
/// This is intentionally narrower than the complete DesignVersion envelope.
/// Certification and archival metadata remain independently addressable D6S
/// material and must not become D6X dependencies unless a future closure
/// profile explicitly selects them.
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
pub fn selected_oad_design_semantic_commitment(value: &Value) -> String {
    canonical_sha256(
        OAD_SEMANTIC_PROJECTION_VERSION,
        &selected_oad_design_semantic_projection(value),
    )
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
            canonical_sha256(INTEGRAL_INTEROP_PROFILE, &changed),
            "full external object identity must observe certification metadata"
        );
        assert_eq!(
            selected_oad_design_semantic_commitment(&baseline),
            selected_oad_design_semantic_commitment(&changed),
            "selected D6X semantics must ignore unselected certification metadata"
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
}
