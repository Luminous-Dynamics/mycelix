use chrono::{DateTime, FixedOffset};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashSet};
use std::env;
use std::error::Error;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const DISSOLVED_MM_L_TO_KG_HA: f64 = 0.01;
const MG_PER_KG_TO_KG: f64 = 1_000_000.0;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Interval {
    start: String,
    end: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ControlVolume {
    area_ha: f64,
    top_depth_cm: f64,
    bottom_depth_cm: f64,
    boundary_description: String,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum MeasurementStatus {
    Measured,
    Estimated,
    Modelled,
}

impl MeasurementStatus {
    fn measured(self) -> bool {
        matches!(self, Self::Measured)
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum InputCategory {
    Precipitation,
    Irrigation,
    Runon,
    LateralInflow,
    CapillaryRise,
    OtherDeclared,
}

impl InputCategory {
    fn as_str(self) -> &'static str {
        match self {
            Self::Precipitation => "precipitation",
            Self::Irrigation => "irrigation",
            Self::Runon => "runon",
            Self::LateralInflow => "lateral_inflow",
            Self::CapillaryRise => "capillary_rise",
            Self::OtherDeclared => "other_declared",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum OutputCategory {
    Evapotranspiration,
    Runoff,
    LowerBoundaryDrainage,
    LateralOutflow,
    OtherDeclared,
}

impl OutputCategory {
    fn as_str(self) -> &'static str {
        match self {
            Self::Evapotranspiration => "evapotranspiration",
            Self::Runoff => "runoff",
            Self::LowerBoundaryDrainage => "lower_boundary_drainage",
            Self::LateralOutflow => "lateral_outflow",
            Self::OtherDeclared => "other_declared",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum WaterDepthStatus {
    Measured,
    Estimated,
    Modelled,
    Missing,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SamplingDesign {
    FlowWeightedComposite,
    EventIntegratedTimeSeries,
    GrabSample,
    SpotSample,
}

impl SamplingDesign {
    fn integrated(self) -> bool {
        matches!(
            self,
            Self::FlowWeightedComposite | Self::EventIntegratedTimeSeries
        )
    }
    fn spot(self) -> bool {
        matches!(self, Self::GrabSample | Self::SpotSample)
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum FluxStatus {
    MeasuredQualified,
    Estimated,
    ConcentrationOnly,
    Unqualified,
}

impl FluxStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::MeasuredQualified => "measured_qualified",
            Self::Estimated => "estimated",
            Self::ConcentrationOnly => "concentration_only",
            Self::Unqualified => "unqualified",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ParticulateFluxStatus {
    MeasuredQualified,
    Estimated,
    Unqualified,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Element {
    N,
    P,
    K,
}

impl Element {
    fn as_str(self) -> &'static str {
        match self {
            Self::N => "N",
            Self::P => "P",
            Self::K => "K",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum DissolvedAnalyte {
    NitrateN,
    AmmoniumN,
    DissolvedOrganicN,
    TotalDissolvedN,
    DissolvedReactiveP,
    DissolvedOrganicP,
    DissolvedK,
    OtherDeclared,
}

impl DissolvedAnalyte {
    fn as_str(self) -> &'static str {
        match self {
            Self::NitrateN => "nitrate_n",
            Self::AmmoniumN => "ammonium_n",
            Self::DissolvedOrganicN => "dissolved_organic_n",
            Self::TotalDissolvedN => "total_dissolved_n",
            Self::DissolvedReactiveP => "dissolved_reactive_p",
            Self::DissolvedOrganicP => "dissolved_organic_p",
            Self::DissolvedK => "dissolved_k",
            Self::OtherDeclared => "other_declared",
        }
    }
    fn matches_element(self, element: Element) -> bool {
        match self {
            Self::NitrateN | Self::AmmoniumN | Self::DissolvedOrganicN | Self::TotalDissolvedN => {
                matches!(element, Element::N)
            }
            Self::DissolvedReactiveP | Self::DissolvedOrganicP => matches!(element, Element::P),
            Self::DissolvedK => matches!(element, Element::K),
            Self::OtherDeclared => true,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ParticulateAnalyte {
    ParticulateN,
    ParticulateP,
    ParticulateK,
    OtherDeclared,
}

impl ParticulateAnalyte {
    fn as_str(self) -> &'static str {
        match self {
            Self::ParticulateN => "particulate_n",
            Self::ParticulateP => "particulate_p",
            Self::ParticulateK => "particulate_k",
            Self::OtherDeclared => "other_declared",
        }
    }
    fn matches_element(self, element: Element) -> bool {
        match self {
            Self::ParticulateN => matches!(element, Element::N),
            Self::ParticulateP => matches!(element, Element::P),
            Self::ParticulateK => matches!(element, Element::K),
            Self::OtherDeclared => true,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum CoverageStatus {
    ReviewedComplete,
    ReviewedIncomplete,
    Unknown,
}

impl CoverageStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::ReviewedComplete => "reviewed_complete",
            Self::ReviewedIncomplete => "reviewed_incomplete",
            Self::Unknown => "unknown",
        }
    }
    fn complete(self) -> bool {
        matches!(self, Self::ReviewedComplete)
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum PreferentialFlowClass {
    Corroborated,
    SensorSignatureOnly,
    Indeterminate,
    NotDetected,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ReferenceMethod {
    DyeTracer,
    ConservativeSoluteTracer,
    StableIsotope,
    Geophysical,
    WaterTable,
    None,
    OtherDeclared,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SedimentStatus {
    Measured,
    Estimated,
    Missing,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct WaterInput {
    flow_id: String,
    category: InputCategory,
    interval: Interval,
    volume_depth_mm: f64,
    measurement_status: MeasurementStatus,
    standard_uncertainty_mm: Option<f64>,
    method: String,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct WaterOutput {
    flow_id: String,
    category: OutputCategory,
    interval: Interval,
    volume_depth_mm: f64,
    measurement_status: MeasurementStatus,
    standard_uncertainty_mm: Option<f64>,
    method: String,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StorageChange {
    change_mm: f64,
    measurement_status: MeasurementStatus,
    standard_uncertainty_mm: Option<f64>,
    method: String,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MoistureObservation {
    sensor_id: String,
    location_id: String,
    timestamp: String,
    depth_cm: f64,
    volumetric_water_content_m3_m3: f64,
    matric_potential_kpa: Option<f64>,
    standard_uncertainty_m3_m3: Option<f64>,
    method: String,
    calibration_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DissolvedExport {
    flux_id: String,
    pathway: String,
    element: Element,
    analyte: DissolvedAnalyte,
    interval: Interval,
    concentration_mg_element_per_l: f64,
    concentration_standard_uncertainty_mg_element_per_l: Option<f64>,
    concentration_sampling_design: SamplingDesign,
    water_depth_mm: Option<f64>,
    water_depth_status: WaterDepthStatus,
    water_depth_standard_uncertainty_mm: Option<f64>,
    collection_coverage_fraction: f64,
    mass_flux_status: FluxStatus,
    method: String,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ParticulateExport {
    flux_id: String,
    pathway: String,
    element: Element,
    analyte: ParticulateAnalyte,
    interval: Interval,
    sediment_export_kg_dry_per_ha: Option<f64>,
    sediment_measurement_status: SedimentStatus,
    sediment_standard_uncertainty_kg_dry_per_ha: Option<f64>,
    concentration_mg_element_per_kg_dry_sediment: Option<f64>,
    concentration_standard_uncertainty_mg_element_per_kg_dry_sediment: Option<f64>,
    collection_coverage_fraction: f64,
    mass_flux_status: ParticulateFluxStatus,
    method: String,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PreferentialFlowAssessment {
    assessment_id: String,
    classification: PreferentialFlowClass,
    sensor_signature: String,
    sensor_ids: Vec<String>,
    reference_method: ReferenceMethod,
    reference_evidence_ref: Option<String>,
    method: String,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct UnmeasuredPathway {
    pathway: String,
    reason: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoverageReview {
    coverage_status: CoverageStatus,
    review_ref: String,
    unmeasured_pathways: Vec<UnmeasuredPathway>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct EventInput {
    schema_version: String,
    event_id: String,
    study_id: String,
    site_id: String,
    experimental_unit_id: String,
    treatment_arm_id: String,
    comparator_arm_id: String,
    event_interval: Interval,
    control_volume: ControlVolume,
    water_inputs: Vec<WaterInput>,
    water_outputs: Vec<WaterOutput>,
    soil_water_storage_change: StorageChange,
    soil_moisture_observations: Vec<MoistureObservation>,
    dissolved_exports: Vec<DissolvedExport>,
    particulate_exports: Vec<ParticulateExport>,
    preferential_flow_assessments: Vec<PreferentialFlowAssessment>,
    water_balance_review: CoverageReview,
    solute_flux_review: CoverageReview,
    source_sha256: String,
    source_evidence_ref: Option<String>,
    #[serde(default)]
    warnings: Vec<String>,
}

#[derive(Clone, Debug, Serialize)]
struct PathwaySummary {
    category: String,
    count: usize,
    volume_depth_mm: f64,
}

#[derive(Clone, Debug, Serialize)]
struct ExportAudit {
    flux_id: String,
    pathway: String,
    element: String,
    analyte: String,
    mass_status: String,
    mass_kg_element_per_ha: Option<f64>,
    plot_mass_kg_element: Option<f64>,
    first_order_linear_uncertainty_bound_kg_element_per_ha: Option<f64>,
    note: String,
}

#[derive(Clone, Debug, Serialize)]
struct WaterBalanceAudit {
    status: String,
    input_total_mm: f64,
    output_total_mm: f64,
    storage_change_mm: f64,
    residual_mm: f64,
    conservative_linear_standard_uncertainty_bound_mm: Option<f64>,
    input_pathways: Vec<PathwaySummary>,
    output_pathways: Vec<PathwaySummary>,
    unmeasured_pathways: Vec<UnmeasuredPathway>,
}

#[derive(Clone, Debug, Serialize)]
struct CoverageAudit {
    status: String,
    unmeasured_pathways: Vec<UnmeasuredPathway>,
}

#[derive(Clone, Debug, Serialize)]
struct FlowCounts {
    water_inputs: usize,
    water_outputs: usize,
    soil_moisture_observations: usize,
    dissolved_exports: usize,
    particulate_exports: usize,
    preferential_flow_assessments: usize,
}

#[derive(Clone, Debug, Serialize)]
struct EventAuditReport {
    audit_version: &'static str,
    event_id: String,
    input_sha256: String,
    source_sha256: String,
    control_volume_area_ha: f64,
    water_balance: WaterBalanceAudit,
    solute_coverage: CoverageAudit,
    dissolved_export_audits: Vec<ExportAudit>,
    particulate_export_audits: Vec<ExportAudit>,
    flow_counts: FlowCounts,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

fn nonblank(value: &str, context: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{context} must not be blank"));
    }
    Ok(())
}

fn finite_nonnegative(value: f64, context: &str) -> Result<(), String> {
    if !value.is_finite() || value < 0.0 {
        return Err(format!("{context} must be finite and non-negative"));
    }
    Ok(())
}

fn validate_optional(value: Option<f64>, context: &str) -> Result<(), String> {
    if let Some(number) = value {
        finite_nonnegative(number, context)?;
    }
    Ok(())
}

fn checked_product(a: f64, b: f64, context: &str) -> Result<f64, String> {
    let result = a * b;
    if !result.is_finite() {
        return Err(format!("{context} overflowed or became non-finite"));
    }
    Ok(result)
}

fn parse_timestamp(value: &str, context: &str) -> Result<DateTime<FixedOffset>, String> {
    DateTime::parse_from_rfc3339(value)
        .map_err(|error| format!("{context} is not valid RFC3339: {error}"))
}

fn parse_interval(
    interval: &Interval,
    context: &str,
) -> Result<(DateTime<FixedOffset>, DateTime<FixedOffset>), String> {
    let start = parse_timestamp(&interval.start, &format!("{context} start"))?;
    let end = parse_timestamp(&interval.end, &format!("{context} end"))?;
    if start >= end {
        return Err(format!("{context} end must be after start"));
    }
    Ok((start, end))
}

fn check_child(parent: &Interval, child: &Interval, context: &str) -> Result<(), String> {
    let (parent_start, parent_end) = parse_interval(parent, "event interval")?;
    let (child_start, child_end) = parse_interval(child, context)?;
    if child_start < parent_start || child_end > parent_end {
        return Err(format!(
            "{context} must be contained within the event interval"
        ));
    }
    Ok(())
}

fn check_review(review: &CoverageReview, context: &str) -> Result<(), String> {
    nonblank(&review.review_ref, &format!("{context} review_ref"))?;
    if review.coverage_status.complete() && !review.unmeasured_pathways.is_empty() {
        return Err(format!(
            "{context} declares complete coverage while listing unmeasured pathways"
        ));
    }
    if matches!(review.coverage_status, CoverageStatus::ReviewedIncomplete)
        && review.unmeasured_pathways.is_empty()
    {
        return Err(format!(
            "{context} declares incomplete coverage without naming unmeasured pathways"
        ));
    }
    for pathway in &review.unmeasured_pathways {
        nonblank(&pathway.pathway, &format!("{context} unmeasured pathway"))?;
        nonblank(
            &pathway.reason,
            &format!("{context} unmeasured pathway reason"),
        )?;
    }
    Ok(())
}

fn validate_input(input: &EventInput) -> Result<(), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!(
            "unsupported schema version {:?}",
            input.schema_version
        ));
    }
    for (value, label) in [
        (&input.event_id, "event_id"),
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.experimental_unit_id, "experimental_unit_id"),
        (&input.treatment_arm_id, "treatment_arm_id"),
        (&input.comparator_arm_id, "comparator_arm_id"),
        (
            &input.control_volume.boundary_description,
            "boundary_description",
        ),
    ] {
        nonblank(value, label)?;
    }

    if input.source_sha256.len() != 64
        || !input
            .source_sha256
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("source_sha256 must contain exactly 64 hexadecimal characters".to_owned());
    }
    if let Some(reference) = &input.source_evidence_ref {
        nonblank(reference, "source_evidence_ref")?;
    }

    let (event_start, event_end) = parse_interval(&input.event_interval, "event interval")?;
    finite_nonnegative(input.control_volume.area_ha, "control-volume area")?;
    if input.control_volume.area_ha <= 0.0 {
        return Err("control-volume area must be greater than zero".to_owned());
    }
    finite_nonnegative(
        input.control_volume.top_depth_cm,
        "control-volume top depth",
    )?;
    finite_nonnegative(
        input.control_volume.bottom_depth_cm,
        "control-volume bottom depth",
    )?;
    if input.control_volume.bottom_depth_cm <= input.control_volume.top_depth_cm {
        return Err("control-volume bottom depth must exceed top depth".to_owned());
    }

    let mut water_ids = HashSet::new();
    for flow in &input.water_inputs {
        nonblank(&flow.flow_id, "water input flow_id")?;
        if !water_ids.insert(flow.flow_id.as_str()) {
            return Err(format!("duplicate water flow_id {:?}", flow.flow_id));
        }
        check_child(
            &input.event_interval,
            &flow.interval,
            "water input interval",
        )?;
        finite_nonnegative(flow.volume_depth_mm, "water input depth")?;
        validate_optional(flow.standard_uncertainty_mm, "water input uncertainty")?;
        nonblank(&flow.method, "water input method")?;
        nonblank(&flow.evidence_ref, "water input evidence_ref")?;
        let _ = flow.category.as_str();
    }
    for flow in &input.water_outputs {
        nonblank(&flow.flow_id, "water output flow_id")?;
        if !water_ids.insert(flow.flow_id.as_str()) {
            return Err(format!("duplicate water flow_id {:?}", flow.flow_id));
        }
        check_child(
            &input.event_interval,
            &flow.interval,
            "water output interval",
        )?;
        finite_nonnegative(flow.volume_depth_mm, "water output depth")?;
        validate_optional(flow.standard_uncertainty_mm, "water output uncertainty")?;
        nonblank(&flow.method, "water output method")?;
        nonblank(&flow.evidence_ref, "water output evidence_ref")?;
        let _ = flow.category.as_str();
    }

    if !input.soil_water_storage_change.change_mm.is_finite() {
        return Err("storage change must be finite".to_owned());
    }
    validate_optional(
        input.soil_water_storage_change.standard_uncertainty_mm,
        "storage-change uncertainty",
    )?;
    nonblank(
        &input.soil_water_storage_change.method,
        "storage-change method",
    )?;
    nonblank(
        &input.soil_water_storage_change.evidence_ref,
        "storage-change evidence_ref",
    )?;

    let mut sensors = BTreeMap::<&str, HashSet<u64>>::new();
    for observation in &input.soil_moisture_observations {
        nonblank(&observation.sensor_id, "sensor_id")?;
        nonblank(&observation.location_id, "sensor location_id")?;
        nonblank(&observation.method, "sensor method")?;
        nonblank(&observation.calibration_ref, "sensor calibration_ref")?;
        let timestamp = parse_timestamp(&observation.timestamp, "sensor timestamp")?;
        if timestamp < event_start || timestamp > event_end {
            return Err("sensor timestamp lies outside event interval".to_owned());
        }
        finite_nonnegative(observation.depth_cm, "sensor depth")?;
        if observation.depth_cm < input.control_volume.top_depth_cm
            || observation.depth_cm > input.control_volume.bottom_depth_cm
        {
            return Err("sensor depth lies outside the control volume".to_owned());
        }
        if !observation.volumetric_water_content_m3_m3.is_finite()
            || !(0.0..=1.0).contains(&observation.volumetric_water_content_m3_m3)
        {
            return Err("volumetric water content must lie in [0, 1]".to_owned());
        }
        validate_optional(observation.standard_uncertainty_m3_m3, "sensor uncertainty")?;
        if observation
            .matric_potential_kpa
            .is_some_and(|value| !value.is_finite())
        {
            return Err("matric potential must be finite".to_owned());
        }
        sensors
            .entry(observation.sensor_id.as_str())
            .or_default()
            .insert(observation.depth_cm.to_bits());
    }

    let mut flux_ids = HashSet::new();
    for export in &input.dissolved_exports {
        nonblank(&export.flux_id, "dissolved flux_id")?;
        if !flux_ids.insert(export.flux_id.as_str()) {
            return Err(format!("duplicate nutrient flux_id {:?}", export.flux_id));
        }
        check_child(
            &input.event_interval,
            &export.interval,
            "dissolved export interval",
        )?;
        nonblank(&export.method, "dissolved export method")?;
        nonblank(&export.evidence_ref, "dissolved export evidence_ref")?;
        finite_nonnegative(
            export.concentration_mg_element_per_l,
            "dissolved concentration",
        )?;
        validate_optional(
            export.concentration_standard_uncertainty_mg_element_per_l,
            "concentration uncertainty",
        )?;
        validate_optional(export.water_depth_mm, "dissolved water depth")?;
        validate_optional(
            export.water_depth_standard_uncertainty_mm,
            "dissolved water-depth uncertainty",
        )?;
        finite_nonnegative(
            export.collection_coverage_fraction,
            "dissolved collection coverage",
        )?;
        if export.collection_coverage_fraction > 1.0 {
            return Err("dissolved collection coverage exceeds one".to_owned());
        }
        if !export.analyte.matches_element(export.element) {
            return Err(format!(
                "dissolved analyte {} conflicts with element {}",
                export.analyte.as_str(),
                export.element.as_str()
            ));
        }
        match export.mass_flux_status {
            FluxStatus::MeasuredQualified => {
                if !matches!(export.water_depth_status, WaterDepthStatus::Measured)
                    || export.water_depth_mm.unwrap_or(0.0) <= 0.0
                    || !export.concentration_sampling_design.integrated()
                    || export.collection_coverage_fraction != 1.0
                    || export.water_depth_standard_uncertainty_mm.is_none()
                    || export
                        .concentration_standard_uncertainty_mg_element_per_l
                        .is_none()
                {
                    return Err(format!(
                        "dissolved flux {:?} lacks measured integrated water/concentration evidence, complete coverage or uncertainty",
                        export.flux_id
                    ));
                }
            }
            FluxStatus::Estimated => {
                if matches!(export.water_depth_status, WaterDepthStatus::Missing)
                    || export.water_depth_mm.unwrap_or(0.0) <= 0.0
                    || !export.concentration_sampling_design.integrated()
                {
                    return Err(format!(
                        "estimated dissolved flux {:?} lacks a non-missing water-volume basis or integrated concentration",
                        export.flux_id
                    ));
                }
            }
            FluxStatus::ConcentrationOnly => {
                if export.water_depth_mm.is_some()
                    || !matches!(export.water_depth_status, WaterDepthStatus::Missing)
                    || !export.concentration_sampling_design.spot()
                {
                    return Err(format!(
                        "concentration-only flux {:?} must not claim integrated water volume",
                        export.flux_id
                    ));
                }
            }
            FluxStatus::Unqualified => {}
        }
    }

    for export in &input.particulate_exports {
        nonblank(&export.flux_id, "particulate flux_id")?;
        if !flux_ids.insert(export.flux_id.as_str()) {
            return Err(format!("duplicate nutrient flux_id {:?}", export.flux_id));
        }
        check_child(
            &input.event_interval,
            &export.interval,
            "particulate export interval",
        )?;
        nonblank(&export.method, "particulate method")?;
        nonblank(&export.evidence_ref, "particulate evidence_ref")?;
        finite_nonnegative(
            export.collection_coverage_fraction,
            "particulate collection coverage",
        )?;
        if export.collection_coverage_fraction > 1.0 {
            return Err("particulate collection coverage exceeds one".to_owned());
        }
        validate_optional(export.sediment_export_kg_dry_per_ha, "sediment export")?;
        validate_optional(
            export.sediment_standard_uncertainty_kg_dry_per_ha,
            "sediment uncertainty",
        )?;
        validate_optional(
            export.concentration_mg_element_per_kg_dry_sediment,
            "sediment concentration",
        )?;
        validate_optional(
            export.concentration_standard_uncertainty_mg_element_per_kg_dry_sediment,
            "sediment concentration uncertainty",
        )?;
        if !export.analyte.matches_element(export.element) {
            return Err(format!(
                "particulate analyte {} conflicts with element {}",
                export.analyte.as_str(),
                export.element.as_str()
            ));
        }
        match export.mass_flux_status {
            ParticulateFluxStatus::MeasuredQualified => {
                if !matches!(export.sediment_measurement_status, SedimentStatus::Measured)
                    || export.sediment_export_kg_dry_per_ha.is_none()
                    || export
                        .concentration_mg_element_per_kg_dry_sediment
                        .is_none()
                    || export.sediment_standard_uncertainty_kg_dry_per_ha.is_none()
                    || export
                        .concentration_standard_uncertainty_mg_element_per_kg_dry_sediment
                        .is_none()
                    || export.collection_coverage_fraction != 1.0
                {
                    return Err(format!(
                        "particulate flux {:?} lacks complete measured sediment mass, concentration, coverage or uncertainty",
                        export.flux_id
                    ));
                }
            }
            ParticulateFluxStatus::Estimated => {
                if matches!(export.sediment_measurement_status, SedimentStatus::Missing)
                    || export.sediment_export_kg_dry_per_ha.is_none()
                    || export
                        .concentration_mg_element_per_kg_dry_sediment
                        .is_none()
                {
                    return Err(format!(
                        "estimated particulate flux {:?} lacks sediment mass or concentration",
                        export.flux_id
                    ));
                }
            }
            ParticulateFluxStatus::Unqualified => {}
        }
    }

    let mut assessment_ids = HashSet::new();
    for assessment in &input.preferential_flow_assessments {
        nonblank(&assessment.assessment_id, "preferential-flow assessment_id")?;
        if !assessment_ids.insert(assessment.assessment_id.as_str()) {
            return Err("duplicate preferential-flow assessment_id".to_owned());
        }
        nonblank(
            &assessment.sensor_signature,
            "preferential-flow sensor signature",
        )?;
        nonblank(&assessment.method, "preferential-flow method")?;
        nonblank(&assessment.evidence_ref, "preferential-flow evidence_ref")?;
        if assessment.sensor_ids.len() < 2 {
            return Err("preferential-flow assessment needs at least two sensors".to_owned());
        }
        let mut unique_ids = HashSet::new();
        let mut unique_depths = HashSet::new();
        for id in &assessment.sensor_ids {
            nonblank(id, "preferential-flow sensor ID")?;
            if !unique_ids.insert(id.as_str()) {
                return Err("duplicate sensor in preferential-flow assessment".to_owned());
            }
            let Some(depths) = sensors.get(id.as_str()) else {
                return Err(format!(
                    "preferential-flow sensor {:?} is absent from event observations",
                    id
                ));
            };
            unique_depths.extend(depths.iter().copied());
        }
        if unique_depths.len() < 2 {
            return Err(
                "preferential-flow assessment must reference at least two distinct observed depths"
                    .to_owned(),
            );
        }
        if matches!(
            assessment.classification,
            PreferentialFlowClass::Corroborated
        ) && (matches!(assessment.reference_method, ReferenceMethod::None)
            || assessment
                .reference_evidence_ref
                .as_deref()
                .map_or(true, str::is_empty))
        {
            return Err(
                "corroborated preferential flow requires independent reference method and evidence"
                    .to_owned(),
            );
        }
    }

    check_review(&input.water_balance_review, "water balance")?;
    check_review(&input.solute_flux_review, "solute flux")?;
    Ok(())
}

fn sum_checked(values: impl Iterator<Item = f64>, context: &str) -> Result<f64, String> {
    let mut total = 0.0;
    for value in values {
        if !value.is_finite() {
            return Err(format!("{context} contains non-finite value"));
        }
        total += value;
        if !total.is_finite() {
            return Err(format!("{context} sum overflowed"));
        }
    }
    Ok(total)
}

fn uncertainty_bound(values: impl Iterator<Item = Option<f64>>) -> Result<Option<f64>, String> {
    let mut total = 0.0;
    for value in values {
        let Some(value) = value else {
            return Ok(None);
        };
        finite_nonnegative(value, "standard uncertainty")?;
        total += value;
        if !total.is_finite() {
            return Err("linear uncertainty sum overflowed".to_owned());
        }
    }
    Ok(Some(total))
}

fn summarize_inputs(flows: &[WaterInput]) -> Result<Vec<PathwaySummary>, String> {
    let mut values = BTreeMap::<&'static str, (usize, f64)>::new();
    for flow in flows {
        let item = values.entry(flow.category.as_str()).or_insert((0, 0.0));
        item.0 += 1;
        item.1 += flow.volume_depth_mm;
        if !item.1.is_finite() {
            return Err("water input category sum overflowed".to_owned());
        }
    }
    Ok(values
        .into_iter()
        .map(|(category, (count, volume_depth_mm))| PathwaySummary {
            category: category.to_owned(),
            count,
            volume_depth_mm,
        })
        .collect())
}

fn summarize_outputs(flows: &[WaterOutput]) -> Result<Vec<PathwaySummary>, String> {
    let mut values = BTreeMap::<&'static str, (usize, f64)>::new();
    for flow in flows {
        let item = values.entry(flow.category.as_str()).or_insert((0, 0.0));
        item.0 += 1;
        item.1 += flow.volume_depth_mm;
        if !item.1.is_finite() {
            return Err("water output category sum overflowed".to_owned());
        }
    }
    Ok(values
        .into_iter()
        .map(|(category, (count, volume_depth_mm))| PathwaySummary {
            category: category.to_owned(),
            count,
            volume_depth_mm,
        })
        .collect())
}

fn audit_dissolved(export: &DissolvedExport, area_ha: f64) -> Result<ExportAudit, String> {
    let partial = export.collection_coverage_fraction < 1.0;
    let mass = match export.mass_flux_status {
        FluxStatus::MeasuredQualified => {
            let product = checked_product(
                export.concentration_mg_element_per_l,
                export.water_depth_mm.unwrap_or_default(),
                "concentration × water depth",
            )?;
            Some(checked_product(
                product,
                DISSOLVED_MM_L_TO_KG_HA,
                "dissolved mass conversion",
            )?)
        }
        FluxStatus::Estimated if !partial => {
            let product = checked_product(
                export.concentration_mg_element_per_l,
                export.water_depth_mm.unwrap_or_default(),
                "estimated concentration × water depth",
            )?;
            Some(checked_product(
                product,
                DISSOLVED_MM_L_TO_KG_HA,
                "estimated dissolved mass conversion",
            )?)
        }
        FluxStatus::Estimated | FluxStatus::ConcentrationOnly | FluxStatus::Unqualified => None,
    };
    let mass_status = if matches!(export.mass_flux_status, FluxStatus::Estimated) && partial {
        "not_computed_partial_collection_coverage"
    } else {
        export.mass_flux_status.as_str()
    };
    let uncertainty = match (
        export.concentration_standard_uncertainty_mg_element_per_l,
        export.water_depth_standard_uncertainty_mm,
        export.water_depth_mm,
        mass,
    ) {
        (Some(cu), Some(wu), Some(water), Some(_)) => {
            let first = checked_product(
                export.concentration_mg_element_per_l,
                wu,
                "dissolved uncertainty term",
            )?;
            let second = checked_product(water, cu, "dissolved uncertainty term")?;
            let sum = first + second;
            if !sum.is_finite() {
                return Err("dissolved uncertainty sum overflowed".to_owned());
            }
            Some(checked_product(
                sum,
                DISSOLVED_MM_L_TO_KG_HA,
                "dissolved uncertainty conversion",
            )?)
        }
        _ => None,
    };
    let plot_mass = match mass {
        Some(x) => Some(checked_product(x, area_ha, "dissolved plot mass")?),
        None => None,
    };
    let note = match mass_status {
        "measured_qualified" => {
            "Measured integrated dissolved mass from elemental concentration × measured event water depth; uncertainty is a first-order linear bound, not a confidence interval."
        }
        "estimated" => {
            "Estimated integrated dissolved mass; not a measured export or field-validated prediction."
        }
        "not_computed_partial_collection_coverage" => {
            "Partial collection is not scaled to an event total."
        }
        "concentration_only" => {
            "Concentration retained; mass export is not computed without integrated water volume."
        }
        _ => "Unqualified dissolved record; mass export is not computed.",
    };
    Ok(ExportAudit {
        flux_id: export.flux_id.clone(),
        pathway: export.pathway.clone(),
        element: export.element.as_str().to_owned(),
        analyte: export.analyte.as_str().to_owned(),
        mass_status: mass_status.to_owned(),
        mass_kg_element_per_ha: mass,
        plot_mass_kg_element: plot_mass,
        first_order_linear_uncertainty_bound_kg_element_per_ha: uncertainty,
        note: note.to_owned(),
    })
}

fn audit_particulate(export: &ParticulateExport, area_ha: f64) -> Result<ExportAudit, String> {
    let partial = export.collection_coverage_fraction < 1.0;
    let mass = match export.mass_flux_status {
        ParticulateFluxStatus::MeasuredQualified => {
            let product = checked_product(
                export.sediment_export_kg_dry_per_ha.unwrap_or_default(),
                export
                    .concentration_mg_element_per_kg_dry_sediment
                    .unwrap_or_default(),
                "sediment mass × concentration",
            )?;
            Some(product / MG_PER_KG_TO_KG)
        }
        ParticulateFluxStatus::Estimated if !partial => {
            let product = checked_product(
                export.sediment_export_kg_dry_per_ha.unwrap_or_default(),
                export
                    .concentration_mg_element_per_kg_dry_sediment
                    .unwrap_or_default(),
                "estimated sediment mass × concentration",
            )?;
            Some(product / MG_PER_KG_TO_KG)
        }
        ParticulateFluxStatus::Estimated | ParticulateFluxStatus::Unqualified => None,
    };
    let mass_status =
        if matches!(export.mass_flux_status, ParticulateFluxStatus::Estimated) && partial {
            "not_computed_partial_collection_coverage"
        } else {
            match export.mass_flux_status {
                ParticulateFluxStatus::MeasuredQualified => "measured_qualified",
                ParticulateFluxStatus::Estimated => "estimated",
                ParticulateFluxStatus::Unqualified => "unqualified",
            }
        };
    let uncertainty = match (
        export.sediment_standard_uncertainty_kg_dry_per_ha,
        export.concentration_standard_uncertainty_mg_element_per_kg_dry_sediment,
        export.sediment_export_kg_dry_per_ha,
        export.concentration_mg_element_per_kg_dry_sediment,
        mass,
    ) {
        (Some(su), Some(cu), Some(s), Some(c), Some(_)) => {
            let first = checked_product(c, su, "particulate uncertainty term")?;
            let second = checked_product(s, cu, "particulate uncertainty term")?;
            let sum = first + second;
            if !sum.is_finite() {
                return Err("particulate uncertainty sum overflowed".to_owned());
            }
            Some(sum / MG_PER_KG_TO_KG)
        }
        _ => None,
    };
    let plot_mass = match mass {
        Some(x) => Some(checked_product(x, area_ha, "particulate plot mass")?),
        None => None,
    };
    let note = match mass_status {
        "measured_qualified" => {
            "Measured particulate elemental mass from dry sediment export × elemental concentration; uncertainty is a first-order linear bound, not a confidence interval."
        }
        "estimated" => {
            "Estimated particulate mass; not a measured export or field-validated prediction."
        }
        "not_computed_partial_collection_coverage" => {
            "Partial sediment collection is not scaled to an event total."
        }
        _ => "Unqualified particulate record; mass export is not computed.",
    };
    Ok(ExportAudit {
        flux_id: export.flux_id.clone(),
        pathway: export.pathway.clone(),
        element: export.element.as_str().to_owned(),
        analyte: export.analyte.as_str().to_owned(),
        mass_status: mass_status.to_owned(),
        mass_kg_element_per_ha: mass,
        plot_mass_kg_element: plot_mass,
        first_order_linear_uncertainty_bound_kg_element_per_ha: uncertainty,
        note: note.to_owned(),
    })
}

fn audit(bytes: &[u8]) -> Result<EventAuditReport, String> {
    let input: EventInput =
        serde_json::from_slice(bytes).map_err(|error| format!("invalid event JSON: {error}"))?;
    validate_input(&input)?;
    let input_total = sum_checked(
        input.water_inputs.iter().map(|flow| flow.volume_depth_mm),
        "water inputs",
    )?;
    let output_total = sum_checked(
        input.water_outputs.iter().map(|flow| flow.volume_depth_mm),
        "water outputs",
    )?;
    let storage_change = input.soil_water_storage_change.change_mm;
    let residual = input_total - output_total - storage_change;
    if !residual.is_finite() {
        return Err("water-balance residual is non-finite".to_owned());
    }
    let uncertainty = uncertainty_bound(
        input
            .water_inputs
            .iter()
            .map(|flow| flow.standard_uncertainty_mm)
            .chain(
                input
                    .water_outputs
                    .iter()
                    .map(|flow| flow.standard_uncertainty_mm),
            )
            .chain(std::iter::once(
                input.soil_water_storage_change.standard_uncertainty_mm,
            )),
    )?;
    let boundary_terms_measured = input
        .water_inputs
        .iter()
        .all(|flow| flow.measurement_status.measured())
        && input
            .water_outputs
            .iter()
            .all(|flow| flow.measurement_status.measured())
        && input
            .soil_water_storage_change
            .measurement_status
            .measured();
    let water_status = if !input.water_balance_review.coverage_status.complete() {
        "unqualified_incomplete_coverage"
    } else if !boundary_terms_measured {
        "unqualified_estimated_or_modelled_terms"
    } else if uncertainty.is_none() {
        "unqualified_missing_uncertainty"
    } else if residual.abs() <= uncertainty.unwrap_or_default() {
        "within_conservative_linear_uncertainty_bound"
    } else {
        "outside_conservative_linear_uncertainty_bound"
    };

    let dissolved = input
        .dissolved_exports
        .iter()
        .map(|export| audit_dissolved(export, input.control_volume.area_ha))
        .collect::<Result<Vec<_>, _>>()?;
    let particulate = input
        .particulate_exports
        .iter()
        .map(|export| audit_particulate(export, input.control_volume.area_ha))
        .collect::<Result<Vec<_>, _>>()?;

    let mut warnings = input.warnings.clone();
    warnings.push("The conservative linear uncertainty sum is not a confidence interval and does not model covariance or systematic bias.".to_owned());
    warnings.push(
        "Dissolved mass: kg element/ha = concentration (mg/L) × water depth (mm) × 0.01."
            .to_owned(),
    );
    warnings.push("Particulate mass: kg element/ha = dry sediment export (kg/ha) × concentration (mg/kg) ÷ 1,000,000.".to_owned());
    warnings.push("Audit success does not establish physical collection completeness, causal efficacy, material safety or regulatory approval.".to_owned());
    if !input.water_balance_review.coverage_status.complete() {
        warnings.push(
            "Water-balance coverage is incomplete or unknown; residual is not qualified."
                .to_owned(),
        );
    }
    if !boundary_terms_measured {
        warnings.push("At least one water-boundary or storage-change term is estimated/modelled; closure is not classified as measured.".to_owned());
    }
    if uncertainty.is_none() {
        warnings.push(
            "At least one boundary-water/storage uncertainty is missing; closure is unqualified."
                .to_owned(),
        );
    }
    if !input.solute_flux_review.coverage_status.complete() {
        warnings.push("Solute coverage is incomplete or unknown; individual flux records do not constitute complete event nutrient loss.".to_owned());
    }
    if dissolved
        .iter()
        .any(|x| x.mass_status == "concentration_only")
    {
        warnings.push(
            "Concentration-only observations were retained without conversion to mass export."
                .to_owned(),
        );
    }
    if dissolved.iter().chain(particulate.iter()).any(|x| {
        x.mass_status == "estimated" || x.mass_status == "not_computed_partial_collection_coverage"
    }) {
        warnings.push("Estimated or partially collected exports remain distinct from complete measured-qualified export.".to_owned());
    }
    if input.preferential_flow_assessments.iter().any(|x| {
        matches!(
            x.classification,
            PreferentialFlowClass::SensorSignatureOnly | PreferentialFlowClass::Indeterminate
        )
    }) {
        warnings.push("Sensor signatures or indeterminate preferential-flow assessments are not treated as corroborated detections.".to_owned());
    }

    Ok(EventAuditReport {
        audit_version: AUDIT_VERSION,
        event_id: input.event_id,
        input_sha256: format!("{:x}", Sha256::digest(bytes)),
        source_sha256: input.source_sha256,
        control_volume_area_ha: input.control_volume.area_ha,
        water_balance: WaterBalanceAudit {
            status: water_status.to_owned(),
            input_total_mm: input_total,
            output_total_mm: output_total,
            storage_change_mm: storage_change,
            residual_mm: residual,
            conservative_linear_standard_uncertainty_bound_mm: uncertainty,
            input_pathways: summarize_inputs(&input.water_inputs)?,
            output_pathways: summarize_outputs(&input.water_outputs)?,
            unmeasured_pathways: input.water_balance_review.unmeasured_pathways.clone(),
        },
        solute_coverage: CoverageAudit {
            status: input.solute_flux_review.coverage_status.as_str().to_owned(),
            unmeasured_pathways: input.solute_flux_review.unmeasured_pathways.clone(),
        },
        dissolved_export_audits: dissolved,
        particulate_export_audits: particulate,
        flow_counts: FlowCounts {
            water_inputs: input.water_inputs.len(),
            water_outputs: input.water_outputs.len(),
            soil_moisture_observations: input.soil_moisture_observations.len(),
            dissolved_exports: input.dissolved_exports.len(),
            particulate_exports: input.particulate_exports.len(),
            preferential_flow_assessments: input.preferential_flow_assessments.len(),
        },
        warnings,
        interpretation_boundary: "Event accounting and unit conversion only; not proof of flux representativeness, causal effect, agronomic benefit, safety or regulatory approval.",
    })
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <event-record.json> <audit-report.json>",
            args.first()
                .map(String::as_str)
                .unwrap_or("terra-preta-coupled-event-audit")
        )
        .into());
    }
    let input_path = Path::new(&args[1]);
    let output_path = Path::new(&args[2]);
    if output_path.exists() {
        return Err("refusing to overwrite existing report".into());
    }
    let canonical_input = fs::canonicalize(input_path)?;
    if output_path == canonical_input.as_path() {
        return Err("input and output paths must differ".into());
    }
    let bytes = fs::read(input_path)?;
    let report =
        audit(&bytes).map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let payload = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    file.write_all(&payload)?;
    println!(
        "event={} water_balance={} residual_mm={} report={}",
        report.event_id,
        report.water_balance.status,
        report.water_balance.residual_mm,
        output_path.display()
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("coupled event audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    fn fixture_bytes() -> Vec<u8> {
        include_bytes!("../fixtures/synthetic-event.json").to_vec()
    }

    fn audited(bytes: &[u8]) -> EventAuditReport {
        audit(bytes).expect("synthetic fixture should audit")
    }

    #[test]
    fn computes_water_balance_and_dissolved_export() {
        let report = audited(&fixture_bytes());
        assert_eq!(report.water_balance.input_total_mm, 18.0);
        assert_eq!(report.water_balance.output_total_mm, 5.5);
        assert_eq!(report.water_balance.storage_change_mm, 12.5);
        assert!(report.water_balance.residual_mm.abs() < 1e-12);
        assert_eq!(
            report
                .water_balance
                .conservative_linear_standard_uncertainty_bound_mm,
            Some(1.7)
        );
        assert_eq!(
            report.water_balance.status,
            "within_conservative_linear_uncertainty_bound"
        );
        assert!(
            (report.dissolved_export_audits[0]
                .mass_kg_element_per_ha
                .unwrap()
                - 0.4)
                .abs()
                < 1e-12
        );
        assert!(
            (report.dissolved_export_audits[0]
                .plot_mass_kg_element
                .unwrap()
                - 0.1)
                .abs()
                < 1e-12
        );
        assert!(
            (report.dissolved_export_audits[0]
                .first_order_linear_uncertainty_bound_kg_element_per_ha
                .unwrap()
                - 0.018)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn computes_particulate_export_from_sediment_mass() {
        let report = audited(&fixture_bytes());
        assert!(
            (report.particulate_export_audits[0]
                .mass_kg_element_per_ha
                .unwrap()
                - 0.04)
                .abs()
                < 1e-12
        );
        assert!(
            (report.particulate_export_audits[0]
                .plot_mass_kg_element
                .unwrap()
                - 0.01)
                .abs()
                < 1e-12
        );
        assert!(
            (report.particulate_export_audits[0]
                .first_order_linear_uncertainty_bound_kg_element_per_ha
                .unwrap()
                - 0.0036)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn concentration_only_is_never_converted_to_mass() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["dissolved_exports"][0]["mass_flux_status"] = json!("concentration_only");
        value["dissolved_exports"][0]["water_depth_mm"] = Value::Null;
        value["dissolved_exports"][0]["water_depth_status"] = json!("missing");
        value["dissolved_exports"][0]["water_depth_standard_uncertainty_mm"] = Value::Null;
        value["dissolved_exports"][0]["concentration_sampling_design"] = json!("grab_sample");
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = audited(&bytes);
        assert_eq!(
            report.dissolved_export_audits[0].mass_kg_element_per_ha,
            None
        );
        assert_eq!(
            report.dissolved_export_audits[0].mass_status,
            "concentration_only"
        );
    }

    #[test]
    fn partial_estimated_collection_is_not_scaled_to_event_total() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["dissolved_exports"][0]["mass_flux_status"] = json!("estimated");
        value["dissolved_exports"][0]["water_depth_status"] = json!("estimated");
        value["dissolved_exports"][0]["collection_coverage_fraction"] = json!(0.7);
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = audited(&bytes);
        assert_eq!(
            report.dissolved_export_audits[0].mass_kg_element_per_ha,
            None
        );
        assert_eq!(
            report.dissolved_export_audits[0].mass_status,
            "not_computed_partial_collection_coverage"
        );
    }

    #[test]
    fn estimated_flux_can_use_measured_water_depth_without_becoming_measured() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["dissolved_exports"][0]["mass_flux_status"] = json!("estimated");
        value["dissolved_exports"][0]["water_depth_status"] = json!("measured");
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = audited(&bytes);
        assert_eq!(report.dissolved_export_audits[0].mass_status, "estimated");
        assert!(
            report.dissolved_export_audits[0]
                .mass_kg_element_per_ha
                .is_some()
        );
    }

    #[test]
    fn estimated_water_terms_cannot_receive_measured_closure_status() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["water_inputs"][0]["measurement_status"] = json!("estimated");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            audited(&bytes).water_balance.status,
            "unqualified_estimated_or_modelled_terms"
        );
    }

    #[test]
    fn incomplete_water_coverage_blocks_closure_even_at_zero_residual() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["water_balance_review"]["coverage_status"] = json!("reviewed_incomplete");
        value["water_balance_review"]["unmeasured_pathways"] = json!([
            {"pathway":"deep lateral inflow","reason":"not monitored"}
        ]);
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = audited(&bytes);
        assert!(report.water_balance.residual_mm.abs() < 1e-12);
        assert_eq!(
            report.water_balance.status,
            "unqualified_incomplete_coverage"
        );
    }

    #[test]
    fn incomplete_solute_coverage_is_not_hidden_by_individual_flux() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["solute_flux_review"]["coverage_status"] = json!("reviewed_incomplete");
        value["solute_flux_review"]["unmeasured_pathways"] = json!([
            {"pathway":"dissolved organic nitrogen","reason":"not analyzed"}
        ]);
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = audited(&bytes);
        assert_eq!(report.solute_coverage.status, "reviewed_incomplete");
        assert!(
            report.dissolved_export_audits[0]
                .mass_kg_element_per_ha
                .is_some()
        );
    }

    #[test]
    fn estimated_storage_change_cannot_receive_measured_closure_status() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["soil_water_storage_change"]["measurement_status"] = json!("modelled");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            audited(&bytes).water_balance.status,
            "unqualified_estimated_or_modelled_terms"
        );
    }

    #[test]
    fn missing_uncertainty_blocks_water_closure() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["water_outputs"][0]["standard_uncertainty_mm"] = Value::Null;
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = audited(&bytes);
        assert_eq!(
            report.water_balance.status,
            "unqualified_missing_uncertainty"
        );
        assert_eq!(
            report
                .water_balance
                .conservative_linear_standard_uncertainty_bound_mm,
            None
        );
    }

    #[test]
    fn partial_estimated_particulate_collection_is_not_scaled() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["particulate_exports"][0]["mass_flux_status"] = json!("estimated");
        value["particulate_exports"][0]["sediment_measurement_status"] = json!("estimated");
        value["particulate_exports"][0]["collection_coverage_fraction"] = json!(0.6);
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = audited(&bytes);
        assert_eq!(
            report.particulate_export_audits[0].mass_kg_element_per_ha,
            None
        );
        assert_eq!(
            report.particulate_export_audits[0].mass_status,
            "not_computed_partial_collection_coverage"
        );
    }

    #[test]
    fn rejects_corroborated_preferential_flow_without_reference() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["preferential_flow_assessments"][0]["classification"] = json!("corroborated");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(
            audit(&bytes)
                .unwrap_err()
                .contains("requires independent reference")
        );
    }

    #[test]
    fn rejects_out_of_event_flux_interval() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["dissolved_exports"][0]["interval"]["end"] = json!("2026-02-01T13:00:00Z");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(
            audit(&bytes)
                .unwrap_err()
                .contains("contained within the event interval")
        );
    }

    #[test]
    fn rejects_element_analyte_mismatch() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["dissolved_exports"][0]["element"] = json!("P");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(
            audit(&bytes)
                .unwrap_err()
                .contains("conflicts with element")
        );
    }

    #[test]
    fn rejects_preferential_flow_sensor_not_in_event() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["preferential_flow_assessments"][0]["sensor_ids"][0] = json!("missing-sensor");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(
            audit(&bytes)
                .unwrap_err()
                .contains("absent from event observations")
        );
    }

    #[test]
    fn rejects_preferential_flow_with_one_observed_depth() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["soil_moisture_observations"][2]["depth_cm"] = json!(10.0);
        value["soil_moisture_observations"][3]["depth_cm"] = json!(10.0);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(
            audit(&bytes)
                .unwrap_err()
                .contains("two or more distinct observed depths")
        );
    }

    #[test]
    fn rejects_invalid_event_interval() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["event_interval"]["end"] = json!("2026-02-01T07:00:00Z");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(
            audit(&bytes)
                .unwrap_err()
                .contains("end must be after start")
        );
    }

    #[test]
    fn rejects_overflowing_flux_conversion() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["dissolved_exports"][0]["concentration_mg_element_per_l"] = json!(1.0e308);
        value["dissolved_exports"][0]["water_depth_mm"] = json!(1.0e308);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(
            audit(&bytes)
                .unwrap_err()
                .contains("overflowed or became non-finite")
        );
    }

    #[test]
    fn input_digest_binds_exact_event_bytes() {
        let bytes = fixture_bytes();
        let report = audited(&bytes);
        assert_eq!(report.input_sha256, format!("{:x}", Sha256::digest(&bytes)));
        assert_eq!(
            report.source_sha256,
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        );
    }

    #[test]
    fn rejects_malformed_source_digest() {
        let mut value: Value = serde_json::from_slice(&fixture_bytes()).unwrap();
        value["source_sha256"] = json!("abcdef");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(audit(&bytes).unwrap_err().contains("64 hexadecimal"));
    }
}
