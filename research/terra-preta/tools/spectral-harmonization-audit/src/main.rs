//! Synthetic-only spectral source harmonization contract auditor.
//!
//! Validates source identity, explicit transform chains, target comparability
//! and physical-sample split grouping. It does not read spectra, apply
//! transformations or establish real-world analytical method equivalence.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_metadata_and_transforms_not_source_data";
const BOUNDARY: &str = "Synthetic metadata and transformation-chain audit only. No spectra are read or transformed. A matching target definition is not empirical method-equivalence proof; rights, source bytes and calibration/bridge evidence require separate review.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Registry {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    registry_id: String,
    sources: Vec<Source>,
    target_comparisons: Vec<TargetComparison>,
    sample_rows: Vec<SampleRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Source {
    source_dataset_id: String,
    source_release_id: String,
    source_uri: String,
    source_artifact_sha256: String,
    manifest_sha256: String,
    rights_status: String,
    instrument_id: String,
    measurement_mode: String,
    native_spectral_definition: SpectralState,
    analysis_view: SpectralState,
    transform_chain: Vec<Transform>,
    target_definitions: Vec<Target>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SpectralState {
    representation: String,
    axis_unit: String,
    axis_min: f64,
    axis_max: f64,
    point_count: usize,
    axis_direction: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct Transform {
    sequence: usize,
    operation_id: String,
    operation: String,
    input_representation: String,
    output_representation: String,
    input_axis_unit: String,
    input_axis_min: f64,
    input_axis_max: f64,
    output_axis_unit: String,
    output_axis_min: f64,
    output_axis_max: f64,
    output_point_count: usize,
    parameters: serde_json::Value,
    code_sha256: String,
    config_sha256: String,
    software_version: String,
    lossy: bool,
    extrapolation_allowed: bool,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Target {
    target_id: String,
    property_name: String,
    definition: String,
    value_semantics: String,
    unit: String,
    method_id: String,
    extraction_basis: String,
    depth_min_m: f64,
    depth_max_m: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TargetComparison {
    comparison_id: String,
    left_source_dataset_id: String,
    right_source_dataset_id: String,
    left_target_id: String,
    right_target_id: String,
    comparison_status: String,
    rationale: String,
    bridge_evidence: Option<BridgeEvidence>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BridgeEvidence {
    validation_dataset_sha256: String,
    bridge_model_sha256: String,
    paired_sample_count: usize,
    independent_validation_group_count: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SampleRow {
    row_id: String,
    source_dataset_id: String,
    physical_sample_group_id: String,
    split_group_id: String,
    split: String,
    instrument_id: String,
    replicate_id: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    registry_id: String,
    input_sha256: String,
    source_count: usize,
    transform_count: usize,
    target_comparison_count: usize,
    directly_comparable_count: usize,
    bridge_required_count: usize,
    non_comparable_count: usize,
    unresolved_count: usize,
    sample_row_count: usize,
    physical_sample_group_count: usize,
    split_group_count: usize,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    source_and_manifest_digests_valid: bool,
    transform_chain_continuity_pass: bool,
    no_double_reflectance_to_absorbance_pass: bool,
    no_axis_extrapolation_pass: bool,
    direct_target_semantics_match_pass: bool,
    sample_group_split_integrity_pass: bool,
    no_aggregate_equivalence_claim: bool,
}

fn valid_sha256(s: &str) -> bool {
    s.len() == 64
        && s.bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn validate_spectral_state(s: &SpectralState, context: &str) -> Result<(), String> {
    finite("axis_min", s.axis_min)?;
    finite("axis_max", s.axis_max)?;
    if s.axis_min <= 0.0
        || s.axis_max <= s.axis_min
        || s.point_count < 2
        || !matches!(s.axis_unit.as_str(), "nm" | "cm-1")
        || !matches!(s.axis_direction.as_str(), "ascending" | "descending")
        || !matches!(
            s.representation.as_str(),
            "reflectance" | "absorbance" | "transmittance" | "derivative" | "normalized" | "other"
        )
    {
        return Err(format!("{context} has invalid spectral axis or representation"));
    }
    Ok(())
}

fn validate_transform_chain(source: &Source) -> Result<usize, String> {
    let native = &source.native_spectral_definition;
    let final_view = &source.analysis_view;
    validate_spectral_state(native, "native spectrum")?;
    validate_spectral_state(final_view, "analysis view")?;

    if source.measurement_mode != native.representation {
        return Err(format!(
            "{} measurement_mode does not match native spectral representation",
            source.source_dataset_id
        ));
    }

    let mut representation = native.representation.as_str();
    let mut axis_unit = native.axis_unit.as_str();
    let mut axis_min = native.axis_min;
    let mut axis_max = native.axis_max;
    let mut points = native.point_count;
    let mut seen_ids = BTreeSet::new();
    let mut reflectance_to_absorbance_count = 0usize;

    for (index, transform) in source.transform_chain.iter().enumerate() {
        if transform.sequence != index + 1
            || transform.operation_id.trim().is_empty()
            || !seen_ids.insert(transform.operation_id.clone())
            || transform.software_version.trim().is_empty()
            || !valid_sha256(&transform.code_sha256)
            || !valid_sha256(&transform.config_sha256)
            || transform.extrapolation_allowed
            || !transform.parameters.is_object()
            || transform.parameters.as_object().is_some_and(|x| x.is_empty())
        {
            return Err(format!(
                "{} transform {} has invalid identity, sequence, digest or extrapolation flag",
                source.source_dataset_id, transform.operation_id
            ));
        }
        for (name, value) in [
            ("input_axis_min", transform.input_axis_min),
            ("input_axis_max", transform.input_axis_max),
            ("output_axis_min", transform.output_axis_min),
            ("output_axis_max", transform.output_axis_max),
        ] {
            finite(name, value)?;
        }
        if transform.input_representation != representation
            || transform.input_axis_unit != axis_unit
            || (transform.input_axis_min - axis_min).abs() > 1e-9
            || (transform.input_axis_max - axis_max).abs() > 1e-9
            || transform.output_axis_unit != axis_unit
            || transform.output_axis_min < native.axis_min
            || transform.output_axis_max > native.axis_max
            || transform.output_axis_min <= 0.0
            || transform.output_axis_min >= transform.output_axis_max
            || transform.output_point_count < 2
        {
            return Err(format!(
                "{} transform {} breaks chain continuity or leaves native axis support",
                source.source_dataset_id, transform.operation_id
            ));
        }

        match transform.operation.as_str() {
            "reflectance_to_absorbance" => {
                reflectance_to_absorbance_count += 1;
                if representation != "reflectance"
                    || transform.output_representation != "absorbance"
                    || reflectance_to_absorbance_count > 1
                {
                    return Err(format!(
                        "{} contains an invalid or repeated reflectance-to-absorbance transform",
                        source.source_dataset_id
                    ));
                }
            }
            "resample_axis" | "smoothing" => {
                if transform.output_representation != representation {
                    return Err(format!(
                        "{} operation must preserve spectral representation",
                        transform.operation
                    ));
                }
            }
            "first_derivative" | "second_derivative" => {
                if representation != "absorbance"
                    || transform.output_representation != "derivative"
                {
                    return Err("derivatives require absorbance input and derivative output".into());
                }
            }
            "snv" | "msc" => {
                if representation != "absorbance"
                    || transform.output_representation != "normalized"
                {
                    return Err("SNV/MSC require absorbance input and normalized output".into());
                }
            }
            _ => return Err(format!("unsupported transform operation {}", transform.operation)),
        }

        representation = &transform.output_representation;
        axis_unit = &transform.output_axis_unit;
        axis_min = transform.output_axis_min;
        axis_max = transform.output_axis_max;
        points = transform.output_point_count;
    }

    if representation != final_view.representation
        || axis_unit != final_view.axis_unit
        || (axis_min - final_view.axis_min).abs() > 1e-9
        || (axis_max - final_view.axis_max).abs() > 1e-9
        || points != final_view.point_count
        || final_view.axis_min < native.axis_min
        || final_view.axis_max > native.axis_max
    {
        return Err(format!(
            "{} transform chain does not produce the declared analysis view",
            source.source_dataset_id
        ));
    }

    if native.representation == "absorbance" && reflectance_to_absorbance_count > 0 {
        return Err(format!(
            "{} attempts to transform already-absorbance data again",
            source.source_dataset_id
        ));
    }
    Ok(source.transform_chain.len())
}

fn target<'a>(source: &'a Source, target_id: &str) -> Option<&'a Target> {
    source.target_definitions.iter().find(|x| x.target_id == target_id)
}

fn target_match(left: &Target, right: &Target) -> bool {
    left.property_name == right.property_name
        && left.definition == right.definition
        && left.unit == right.unit
        && left.method_id == right.method_id
        && left.extraction_basis == right.extraction_basis
        && (left.depth_min_m - right.depth_min_m).abs() <= 1e-9
        && (left.depth_max_m - right.depth_max_m).abs() <= 1e-9
        && left.value_semantics == "direct_measurement"
        && right.value_semantics == "direct_measurement"
}

fn validate(reg: &Registry) -> Result<usize, String> {
    if reg.schema_version != "soil-spectral-harmonization-v0.1"
        || reg.fixture_status != FIXTURE_STATUS
        || reg.evidence_class != "synthetic"
        || reg.registry_id.trim().is_empty()
        || reg.sources.len() < 2
        || reg.target_comparisons.is_empty()
        || reg.sample_rows.len() < 2
    {
        return Err("unsupported registry version, evidence class or missing registry data".into());
    }

    let mut source_map = BTreeMap::new();
    let mut transforms = 0usize;
    for source in &reg.sources {
        if source.source_dataset_id.trim().is_empty()
            || source.source_release_id.trim().is_empty()
            || source.source_uri.trim().is_empty()
            || source.instrument_id.trim().is_empty()
            || !valid_sha256(&source.source_artifact_sha256)
            || !valid_sha256(&source.manifest_sha256)
            || !matches!(
                source.rights_status.as_str(),
                "verified_open" | "verified_restricted" | "agreement_required" | "pending_review" | "unknown"
            )
        {
            return Err(format!("{} has incomplete source identity/provenance", source.source_dataset_id));
        }
        if source_map.insert(source.source_dataset_id.as_str(), source).is_some() {
            return Err("source dataset IDs must be unique".into());
        }
        let mut target_ids = BTreeSet::new();
        for t in &source.target_definitions {
            finite("target depth_min_m", t.depth_min_m)?;
            finite("target depth_max_m", t.depth_max_m)?;
            if t.target_id.trim().is_empty()
                || !target_ids.insert(t.target_id.clone())
                || t.property_name.trim().is_empty()
                || t.definition.trim().is_empty()
                || t.unit.trim().is_empty()
                || t.method_id.trim().is_empty()
                || t.extraction_basis.trim().is_empty()
                || t.depth_min_m < 0.0
                || t.depth_max_m <= t.depth_min_m
                || !matches!(
                    t.value_semantics.as_str(),
                    "direct_measurement" | "derived_estimate" | "model_prediction"
                )
            {
                return Err(format!("{} contains invalid target semantics", source.source_dataset_id));
            }
        }
        transforms += validate_transform_chain(source)?;
    }

    let mut comparison_ids = BTreeSet::new();
    for edge in &reg.target_comparisons {
        if edge.comparison_id.trim().is_empty()
            || !comparison_ids.insert(edge.comparison_id.clone())
            || edge.rationale.trim().is_empty()
        {
            return Err("target comparison IDs/rationale must be present and unique".into());
        }
        let left = source_map
            .get(edge.left_source_dataset_id.as_str())
            .ok_or_else(|| format!("{} references unknown left source", edge.comparison_id))?;
        let right = source_map
            .get(edge.right_source_dataset_id.as_str())
            .ok_or_else(|| format!("{} references unknown right source", edge.comparison_id))?;
        let left_target = target(left, &edge.left_target_id)
            .ok_or_else(|| format!("{} references unknown left target", edge.comparison_id))?;
        let right_target = target(right, &edge.right_target_id)
            .ok_or_else(|| format!("{} references unknown right target", edge.comparison_id))?;

        match edge.comparison_status.as_str() {
            "directly_comparable" => {
                if edge.bridge_evidence.is_some() || !target_match(left_target, right_target) {
                    return Err(format!(
                        "{} claims direct comparability without identical target method/unit/depth semantics",
                        edge.comparison_id
                    ));
                }
            }
            "bridge_required" => {
                let evidence = edge.bridge_evidence.as_ref()
                    .ok_or_else(|| format!("{} bridge-required edge lacks bridge evidence", edge.comparison_id))?;
                if !valid_sha256(&evidence.validation_dataset_sha256)
                    || !valid_sha256(&evidence.bridge_model_sha256)
                    || evidence.paired_sample_count == 0
                    || evidence.independent_validation_group_count == 0
                {
                    return Err(format!("{} has invalid bridge evidence identity/coverage", edge.comparison_id));
                }
            }
            "non_comparable" | "unresolved" => {
                if edge.bridge_evidence.is_some() {
                    return Err(format!("{} non-qualified edge must not carry bridge evidence", edge.comparison_id));
                }
            }
            _ => return Err(format!("{} has unsupported comparison status", edge.comparison_id)),
        }
    }

    let mut row_ids = BTreeSet::new();
    let mut physical_splits: BTreeMap<String, (String, String)> = BTreeMap::new();
    let mut split_group_splits: BTreeMap<String, String> = BTreeMap::new();
    let mut split_groups = BTreeSet::new();
    for row in &reg.sample_rows {
        if row.row_id.trim().is_empty()
            || !row_ids.insert(row.row_id.clone())
            || !source_map.contains_key(row.source_dataset_id.as_str())
            || row.physical_sample_group_id.trim().is_empty()
            || row.split_group_id.trim().is_empty()
            || row.instrument_id.trim().is_empty()
            || row.replicate_id.trim().is_empty()
            || !matches!(row.split.as_str(), "train" | "validation" | "test")
        {
            return Err("sample row has invalid identity/source/split metadata".into());
        }
        let source = source_map
            .get(row.source_dataset_id.as_str())
            .expect("source existence checked");
        if source.instrument_id != row.instrument_id {
            return Err(format!("row {} instrument differs from source instrument", row.row_id));
        }
        split_groups.insert(row.split_group_id.clone());
        if let Some(split) = split_group_splits.get(&row.split_group_id) {
            if split != &row.split {
                return Err(format!("split group {} spans multiple dataset splits", row.split_group_id));
            }
        } else {
            split_group_splits.insert(row.split_group_id.clone(), row.split.clone());
        }
        if let Some((group, split)) = physical_splits.get(&row.physical_sample_group_id) {
            if group != &row.split_group_id || split != &row.split {
                return Err(format!(
                    "physical sample {} leaks across split groups or splits",
                    row.physical_sample_group_id
                ));
            }
        } else {
            physical_splits.insert(
                row.physical_sample_group_id.clone(),
                (row.split_group_id.clone(), row.split.clone()),
            );
        }
    }
    Ok(transforms)
}

fn build_report(reg: &Registry, digest: String, transform_count: usize) -> Report {
    let mut direct = 0usize;
    let mut bridge = 0usize;
    let mut non_comparable = 0usize;
    let mut unresolved = 0usize;
    for edge in &reg.target_comparisons {
        match edge.comparison_status.as_str() {
            "directly_comparable" => direct += 1,
            "bridge_required" => bridge += 1,
            "non_comparable" => non_comparable += 1,
            "unresolved" => unresolved += 1,
            _ => {}
        }
    }
    let physical: BTreeSet<&str> = reg.sample_rows.iter()
        .map(|x| x.physical_sample_group_id.as_str())
        .collect();
    let split: BTreeSet<&str> = reg.sample_rows.iter()
        .map(|x| x.split_group_id.as_str())
        .collect();
    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        registry_id: reg.registry_id.clone(),
        input_sha256: digest,
        source_count: reg.sources.len(),
        transform_count,
        target_comparison_count: reg.target_comparisons.len(),
        directly_comparable_count: direct,
        bridge_required_count: bridge,
        non_comparable_count: non_comparable,
        unresolved_count: unresolved,
        sample_row_count: reg.sample_rows.len(),
        physical_sample_group_count: physical.len(),
        split_group_count: split.len(),
        controls: Controls {
            source_and_manifest_digests_valid: true,
            transform_chain_continuity_pass: true,
            no_double_reflectance_to_absorbance_pass: true,
            no_axis_extrapolation_pass: true,
            direct_target_semantics_match_pass: true,
            sample_group_split_integrity_pass: true,
            no_aggregate_equivalence_claim: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into());
    }
    let bytes = fs::read(input_path)?;
    let registry: Registry = serde_json::from_slice(&bytes)?;
    let transform_count = validate(&registry).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&registry, digest, transform_count);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new().write(true).create_new(true).open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: spectral-harmonization-audit <registry.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("spectral harmonization audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn registry() -> Registry {
        serde_json::from_str(include_str!("../fixtures/synthetic-harmonization-registry.json"))
            .expect("synthetic registry parses")
    }

    #[test]
    fn valid_synthetic_registry_preserves_source_and_target_distinctions() {
        let r = registry();
        let transforms = validate(&r).expect("registry qualifies structurally");
        assert_eq!(transforms, 2);
        let report = build_report(&r, "0".repeat(64), transforms);
        assert_eq!(report.source_count, 2);
        assert_eq!(report.directly_comparable_count, 1);
        assert_eq!(report.non_comparable_count, 1);
        assert_eq!(report.physical_sample_group_count, 3);
    }

    #[test]
    fn already_absorbance_source_cannot_be_converted_again() {
        let mut r = registry();
        let reflectance_transform = r.sources[0].transform_chain[0].clone();
        r.sources[1].transform_chain.push(reflectance_transform);
        assert!(validate(&r).is_err());
    }

    #[test]
    fn resampling_cannot_extrapolate_beyond_native_spectrum() {
        let mut r = registry();
        r.sources[0].transform_chain[1].output_axis_max = 2600.0;
        r.sources[0].analysis_view.axis_max = 2600.0;
        assert!(validate(&r).is_err());
    }

    #[test]
    fn direct_target_edge_rejects_method_mismatch() {
        let mut r = registry();
        r.sources[1].target_definitions[1].method_id = "BRAY1-V1".into();
        r.sources[1].target_definitions[1].extraction_basis = "Bray-1 extraction".into();
        r.target_comparisons[1].comparison_status = "directly_comparable".into();
        assert!(validate(&r).is_err());
    }

    #[test]
    fn one_physical_sample_cannot_cross_train_and_test() {
        let mut r = registry();
        r.sample_rows[2].split = "test".into();
        r.sample_rows[2].split_group_id = "group-99".into();
        assert!(validate(&r).is_err());
    }

    #[test]
    fn bridge_required_edges_need_independent_bridge_evidence() {
        let mut r = registry();
        r.target_comparisons[1].comparison_status = "bridge_required".into();
        assert!(validate(&r).is_err());
    }

    #[test]
    fn a_split_group_cannot_span_train_and_validation() {
        let mut r = registry();
        r.sample_rows[3].split_group_id = "group-01".into();
        assert!(validate(&r).is_err());
    }
}
