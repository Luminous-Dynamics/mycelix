//! Validate JSON Schema documents and optionally validate JSON instances against them.
//!
//! Uses the Rust jsonschema validator with default features disabled so schema
//! validation is offline by default; externally hosted $ref targets are not needed
//! by the Terra Preta contracts and must not silently become network dependencies.

use serde_json::Value;
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

#[cfg(test)]
const SCHEMA_FIXTURES: &[(&str, &str)] = &[
    (
        "batch-record",
        include_str!("../../../batch-record.schema.json"),
    ),
    (
        "contrast-record",
        include_str!("../../../contrast-record.schema.json"),
    ),
    (
        "split-manifest",
        include_str!("../../../split-manifest.schema.json"),
    ),
    (
        "biochar-audit-report",
        include_str!("../../biochar-audit/audit-report.schema.json"),
    ),
    (
        "prediction-audit-report",
        include_str!("../../prediction-audit/prediction-report.schema.json"),
    ),
    (
        "split-audit-report",
        include_str!("../../split-audit/split-audit-report.schema.json"),
    ),
    (
        "source-register",
        include_str!("../../../source-register.schema.json"),
    ),
    (
        "dependency-audit-report",
        include_str!("../../dependency-audit/dependency-audit-report.schema.json"),
    ),
    (
        "water-retention-curve-input",
        include_str!("../../water-retention-audit/curve-input.schema.json"),
    ),
    (
        "water-retention-audit-report",
        include_str!("../../water-retention-audit/water-retention-audit-report.schema.json"),
    ),
    (
        "nutrient-budget-input",
        include_str!("../../nutrient-budget-audit/nutrient-budget-input.schema.json"),
    ),
    (
        "nutrient-budget-audit-report",
        include_str!("../../nutrient-budget-audit/nutrient-budget-audit-report.schema.json"),
    ),
];

#[cfg(test)]
const SOURCE_REGISTER: &str = include_str!("../../../source-register.v0.1.json");

fn validate_schema(schema_bytes: &[u8], label: &str) -> Result<Value, Box<dyn Error>> {
    let schema: Value = serde_json::from_slice(schema_bytes)
        .map_err(|error| format!("{label}: invalid JSON: {error}"))?;
    jsonschema::meta::validate(&schema)
        .map_err(|error| format!("{label}: invalid JSON Schema: {error}"))?;
    jsonschema::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|error| format!("{label}: cannot build validator: {error}"))?;
    Ok(schema)
}

fn validate_instance(
    schema: &Value,
    instance_bytes: &[u8],
    schema_label: &str,
    instance_label: &str,
) -> Result<(), Box<dyn Error>> {
    let instance: Value = serde_json::from_slice(instance_bytes)
        .map_err(|error| format!("{instance_label}: invalid JSON: {error}"))?;
    let validator = jsonschema::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("{schema_label}: cannot build validator: {error}"))?;
    let errors: Vec<String> = validator
        .iter_errors(&instance)
        .map(|error| format!("{}: {}", error.instance_path(), error))
        .collect();
    if !errors.is_empty() {
        return Err(format!(
            "{instance_label} does not conform to {schema_label}: {}",
            errors.join("; ")
        )
        .into());
    }
    Ok(())
}

fn run() -> Result<(), Box<dyn Error>> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 2 && arguments.len() != 3 {
        return Err(format!(
            "usage: {} <schema.json> [instance.json]",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("terra-preta-schema-audit")
        )
        .into());
    }

    let schema_path = Path::new(&arguments[1]);
    let schema_bytes = fs::read(schema_path)?;
    let schema = validate_schema(&schema_bytes, &schema_path.display().to_string())?;
    println!("schema is valid JSON and conforms to its declared JSON Schema draft");

    if let Some(instance_arg) = arguments.get(2) {
        let instance_path = Path::new(instance_arg);
        let instance_bytes = fs::read(instance_path)?;
        validate_instance(
            &schema,
            &instance_bytes,
            &schema_path.display().to_string(),
            &instance_path.display().to_string(),
        )?;
        println!("instance conforms to schema");
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("schema audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn all_committed_schemas_parse_and_validate_against_their_meta_schema() {
        for (name, text) in SCHEMA_FIXTURES {
            let schema: Value =
                serde_json::from_str(text).unwrap_or_else(|error| panic!("{name}: {error}"));
            jsonschema::meta::validate(&schema)
                .unwrap_or_else(|error| panic!("{name} is not a valid JSON Schema: {error}"));
            jsonschema::validator_for(&schema)
                .unwrap_or_else(|error| panic!("{name} cannot build validator: {error}"));
        }
    }

    fn split_report_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[5].1).unwrap()
    }

    fn prediction_report_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[4].1).unwrap()
    }

    fn valid_prediction_report() -> Value {
        json!({
            "audit_version": "0.1.0",
            "input_label": "held-out-predictions.csv",
            "input_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "observations": 6,
            "independent_groups": 3,
            "bootstrap_seed": 123,
            "nominal_interval_coverage": 0.9,
            "candidate": {
                "observations": 6,
                "mae": 1.0,
                "rmse": 1.2,
                "bias_prediction_minus_actual": -0.2,
                "empirical_interval_coverage": 1.0,
                "mean_interval_width": 6.0,
                "mean_interval_score": 6.0
            },
            "baseline": {
                "observations": 6,
                "mae": 2.0,
                "rmse": 2.2,
                "bias_prediction_minus_actual": -0.5,
                "empirical_interval_coverage": null,
                "mean_interval_width": null,
                "mean_interval_score": null
            },
            "relative_mae_improvement": 0.5,
            "improvement_cluster_bootstrap_95pct": {
                "confidence_level": 0.95,
                "replicates_requested": 2000,
                "replicates_valid": 2000,
                "relative_mae_improvement_lower": 0.1,
                "relative_mae_improvement_upper": 0.8,
                "excludes_zero": true
            },
            "warnings": ["synthetic fixture; not efficacy evidence"],
            "interpretation_boundary": "schema conformance test only"
        })
    }

    fn valid_split_report() -> Value {
        json!({
            "audit_version": "0.1.0",
            "status": "passed",
            "input_label": "fixture.csv",
            "input_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "grouping_key": "source_study_id",
            "row_count": 5,
            "distinct_group_count": 4,
            "group_assignment_sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "split_counts": {
                "train": {"rows": 2, "groups": 1},
                "validation": {"rows": 1, "groups": 1},
                "test": {"rows": 1, "groups": 1},
                "excluded": {"rows": 1, "groups": 1}
            },
            "warnings": ["structural checks cannot prove scientific independence"],
            "interpretation_boundary": "synthetic schema fixture only"
        })
    }

    #[test]
    fn source_register_conforms_to_its_schema() {
        let schema: Value = serde_json::from_str(SCHEMA_FIXTURES[6].1).unwrap();
        let validator = jsonschema::options()
            .should_validate_formats(true)
            .build(&schema)
            .unwrap();
        let register: Value = serde_json::from_str(SOURCE_REGISTER).unwrap();
        assert!(validator.is_valid(&register));
    }

    #[test]
    fn rejects_source_register_with_invalid_date_format() {
        let schema: Value = serde_json::from_str(SCHEMA_FIXTURES[6].1).unwrap();
        let validator = jsonschema::options()
            .should_validate_formats(true)
            .build(&schema)
            .unwrap();
        let mut register: Value = serde_json::from_str(SOURCE_REGISTER).unwrap();
        register["reviewed_on"] = json!("29-10-2026");
        assert!(!validator.is_valid(&register));
    }

    fn dependency_report_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[7].1).unwrap()
    }

    fn valid_dependency_report() -> Value {
        json!({
            "audit_version": "0.1.0",
            "status": "passed",
            "input_label": "row-manifest.csv",
            "input_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "row_count": 5,
            "component_count": 4,
            "dependency_columns": ["study_id", "site_id"],
            "component_assignment_sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
            "split_counts": {
                "train": {"rows": 2, "components": 1},
                "validation": {"rows": 1, "components": 1},
                "test": {"rows": 1, "components": 1},
                "excluded": {"rows": 1, "components": 1}
            },
            "dependency_key_stats": {
                "study_id": {"rows_with_value": 5, "rows_missing_value": 0, "distinct_values": 4},
                "site_id": {"rows_with_value": 4, "rows_missing_value": 1, "distinct_values": 4}
            },
            "warnings": ["structural check only"],
            "interpretation_boundary": "schema test only"
        })
    }

    #[test]
    fn accepts_valid_dependency_audit_report() {
        let validator = jsonschema::validator_for(&dependency_report_schema()).unwrap();
        assert!(validator.is_valid(&valid_dependency_report()));
    }

    #[test]
    fn rejects_dependency_report_with_too_few_components() {
        let validator = jsonschema::validator_for(&dependency_report_schema()).unwrap();
        let mut report = valid_dependency_report();
        report["component_count"] = json!(2);
        assert!(!validator.is_valid(&report));
    }

    #[test]
    fn accepts_prediction_report_with_nominal_level_and_interval_score() {
        let validator = jsonschema::validator_for(&prediction_report_schema()).unwrap();
        assert!(validator.is_valid(&valid_prediction_report()));
    }

    #[test]
    fn rejects_negative_prediction_interval_score() {
        let validator = jsonschema::validator_for(&prediction_report_schema()).unwrap();
        let mut report = valid_prediction_report();
        report["candidate"]["mean_interval_score"] = json!(-1.0);
        assert!(!validator.is_valid(&report));
    }

    #[test]
    fn rejects_wrong_nominal_interval_coverage() {
        let validator = jsonschema::validator_for(&prediction_report_schema()).unwrap();
        let mut report = valid_prediction_report();
        report["nominal_interval_coverage"] = json!(0.8);
        assert!(!validator.is_valid(&report));
    }

    #[test]
    fn accepts_valid_split_audit_report_instance() {
        let schema = split_report_schema();
        let validator = jsonschema::validator_for(&schema).unwrap();
        assert!(validator.is_valid(&valid_split_report()));
    }

    #[test]
    fn rejects_report_with_invalid_status() {
        let schema = split_report_schema();
        let validator = jsonschema::validator_for(&schema).unwrap();
        let mut report = valid_split_report();
        report["status"] = json!("warning");
        assert!(!validator.is_valid(&report));
    }

    fn water_retention_input_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[8].1).unwrap()
    }

    fn water_retention_report_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[9].1).unwrap()
    }

    const WATER_RETENTION_INPUT_FIXTURE: &str =
        include_str!("../../water-retention-audit/fixtures/synthetic-curve-input.json");

    #[test]
    fn accepts_synthetic_water_retention_curve_input() {
        let validator = jsonschema::validator_for(&water_retention_input_schema()).unwrap();
        let instance: Value = serde_json::from_str(WATER_RETENTION_INPUT_FIXTURE).unwrap();
        assert!(validator.is_valid(&instance));
    }

    #[test]
    fn rejects_gravimetric_curve_input_without_density_conversion_metadata() {
        let validator = jsonschema::validator_for(&water_retention_input_schema()).unwrap();
        let mut instance: Value = serde_json::from_str(WATER_RETENTION_INPUT_FIXTURE).unwrap();
        instance["measurement_basis"] = json!("gravimetric_kg_per_kg_dry_soil");
        assert!(!validator.is_valid(&instance));
    }

    fn valid_water_retention_report() -> Value {
        json!({
            "audit_version": "0.1.0",
            "status": "passed",
            "sample_id": "sample-01",
            "curve_id": "curve-01",
            "input_label": "fixture.json",
            "input_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "measurement_basis": "volumetric_fraction",
            "bulk_density_kg_m3": null,
            "water_density_kg_m3": null,
            "method": {"name": "pressure-plate-fixture", "standard_or_protocol": "synthetic-only"},
            "measurement_count": 3,
            "normalized_points": [
                {"matric_potential_kpa": -10.0, "suction_kpa": 10.0, "water_content_m3_m3": 0.30},
                {"matric_potential_kpa": -100.0, "suction_kpa": 100.0, "water_content_m3_m3": 0.22},
                {"matric_potential_kpa": -1500.0, "suction_kpa": 1500.0, "water_content_m3_m3": 0.12}
            ],
            "monotonicity_tolerance_m3_m3": 0.0,
            "field_capacity": {
                "target_matric_potential_kpa": -33.0,
                "water_content_m3_m3": 0.2585,
                "estimate_method": "log10_suction_linear_interpolation",
                "less_suction_bracket_matric_potential_kpa": -10.0,
                "greater_suction_bracket_matric_potential_kpa": -100.0,
                "interpolation_fraction": 0.5185
            },
            "permanent_wilting_point": {
                "target_matric_potential_kpa": -1500.0,
                "water_content_m3_m3": 0.12,
                "estimate_method": "observed",
                "less_suction_bracket_matric_potential_kpa": null,
                "greater_suction_bracket_matric_potential_kpa": null,
                "interpolation_fraction": null
            },
            "available_water_fraction_m3_m3": 0.1385,
            "layer_depth_mm": 200.0,
            "available_water_storage_mm": 27.7,
            "warnings": ["synthetic fixture only"],
            "uncertainty_status": "not_propagated",
            "interpretation_boundary": "schema conformance test only"
        })
    }

    #[test]
    fn accepts_valid_water_retention_audit_report() {
        let validator = jsonschema::validator_for(&water_retention_report_schema()).unwrap();
        assert!(validator.is_valid(&valid_water_retention_report()));
    }

    #[test]
    fn rejects_negative_available_water_fraction() {
        let validator = jsonschema::validator_for(&water_retention_report_schema()).unwrap();
        let mut report = valid_water_retention_report();
        report["available_water_fraction_m3_m3"] = json!(-0.01);
        assert!(!validator.is_valid(&report));
    }

    #[test]
    fn rejects_claim_of_uncertainty_propagation_in_v01_report() {
        let validator = jsonschema::validator_for(&water_retention_report_schema()).unwrap();
        let mut report = valid_water_retention_report();
        report["uncertainty_status"] = json!("propagated");
        assert!(!validator.is_valid(&report));
    }

    fn nutrient_budget_input_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[10].1).unwrap()
    }

    fn nutrient_budget_report_schema() -> Value {
        serde_json::from_str(SCHEMA_FIXTURES[11].1).unwrap()
    }

    const NUTRIENT_BUDGET_INPUT_FIXTURE: &str =
        include_str!("../../nutrient-budget-audit/fixtures/synthetic-n-budget.json");

    #[test]
    fn accepts_synthetic_nutrient_budget_input() {
        let validator = jsonschema::validator_for(&nutrient_budget_input_schema()).unwrap();
        let instance: Value = serde_json::from_str(NUTRIENT_BUDGET_INPUT_FIXTURE).unwrap();
        assert!(validator.is_valid(&instance));
    }

    #[test]
    fn rejects_non_elemental_nutrient_budget_basis() {
        let validator = jsonschema::validator_for(&nutrient_budget_input_schema()).unwrap();
        let mut instance: Value = serde_json::from_str(NUTRIENT_BUDGET_INPUT_FIXTURE).unwrap();
        instance["unit_basis"] = json!("kg_P2O5_per_ha");
        assert!(!validator.is_valid(&instance));
    }

    fn valid_nutrient_budget_report() -> Value {
        json!({
            "audit_version": "0.1.0",
            "status": "within_conservative_uncertainty_bound",
            "record_id": "synthetic-n-budget-01",
            "element": "N",
            "input_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "study_id": "synthetic-study",
            "site_id": "synthetic-site",
            "experimental_unit_id": "plot-01",
            "treatment_arm_id": "treatment-a",
            "comparator_arm_id": "local-practice",
            "period_start_date": "2026-01-01",
            "period_end_date": "2026-12-31",
            "control_volume": {
                "area_ha": 0.25,
                "top_depth_cm": 0,
                "bottom_depth_cm": 30,
                "boundary_description": "synthetic soil + standing crop"
            },
            "input_total_kg_element_per_ha": 50,
            "output_total_kg_element_per_ha": 28,
            "storage_start_total_kg_element_per_ha": 100,
            "storage_end_total_kg_element_per_ha": 122,
            "storage_change_kg_element_per_ha": 22,
            "residual_kg_element_per_ha": 0,
            "conservative_linear_standard_uncertainty_bound_kg_element_per_ha": 5.1,
            "plot_scaled": {
                "input_kg_element": 12.5,
                "output_kg_element": 7,
                "storage_change_kg_element": 5.5,
                "residual_kg_element": 0,
                "uncertainty_bound_kg_element": 1.275
            },
            "flow_counts": {
                "inputs": 2,
                "outputs": 3,
                "storage_pools": 2,
                "availability_indicators": 1
            },
            "availability_indicators": [
                {
                    "indicator_id": "mineral-n",
                    "name": "extractable mineral nitrogen",
                    "value": 18,
                    "unit": "mg N kg-1 dry soil",
                    "depth_top_cm": 0,
                    "depth_bottom_cm": 30,
                    "method": "synthetic extraction fixture",
                    "evidence_ref": "synthetic-fixture"
                }
            ],
            "warnings": ["synthetic fixture only"],
            "interpretation_boundary": "arithmetic schema fixture only"
        })
    }

    #[test]
    fn accepts_valid_nutrient_budget_report() {
        let validator = jsonschema::validator_for(&nutrient_budget_report_schema()).unwrap();
        assert!(validator.is_valid(&valid_nutrient_budget_report()));
    }

    #[test]
    fn rejects_unknown_nutrient_budget_report_status() {
        let validator = jsonschema::validator_for(&nutrient_budget_report_schema()).unwrap();
        let mut report = valid_nutrient_budget_report();
        report["status"] = json!("balanced");
        assert!(!validator.is_valid(&report));
    }

    #[test]
    fn rejects_negative_nutrient_budget_residual_components_that_must_be_nonnegative() {
        let validator = jsonschema::validator_for(&nutrient_budget_report_schema()).unwrap();
        let mut report = valid_nutrient_budget_report();
        report["input_total_kg_element_per_ha"] = json!(-1.0);
        assert!(!validator.is_valid(&report));
    }

    #[test]
    fn rejects_report_with_placeholder_digest() {
        let schema = split_report_schema();
        let validator = jsonschema::validator_for(&schema).unwrap();
        let mut report = valid_split_report();
        report["input_sha256"] = json!("not-a-sha256");
        assert!(!validator.is_valid(&report));
    }
}
