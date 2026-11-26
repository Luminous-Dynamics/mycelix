#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt::{Display, Formatter};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DatasetRole {
    ObservedMeasurements,
    DerivedMap,
    NumericalReference,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HydraulicTarget {
    SaturatedConductivity,
    UnsaturatedConductivity,
    WaterRetention,
    WaterDiffusivity,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProtocolClass {
    FieldInfiltrometer,
    FieldPermeameter,
    LaboratoryConstantHead,
    LaboratoryFallingHead,
    LaboratoryPressurePlate,
    DewPoint,
    NumericalSolver,
    Unknown,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDomain {
    InSitu,
    UndisturbedCore,
    RepackedCore,
    PointRetention,
    MapCell,
    DigitalImage,
    NumericalDomain,
    Unknown,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DatasetManifest {
    pub dataset_id: String,
    pub title: String,
    pub version: String,
    pub doi: Option<String>,
    pub record_url: String,
    pub license: Option<String>,
    pub role: DatasetRole,
    pub row_count: Option<usize>,
    pub independent_study_count: Option<usize>,
    pub independent_site_count: Option<usize>,
    pub method_metadata_available: bool,
    pub spatial_metadata_available: bool,
    pub notes: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MeasurementSupport {
    pub domain: SupportDomain,
    pub protocol: ProtocolClass,
    pub original_method_label: String,
    pub spatial_support_m: Option<f64>,
    pub depth_top_m: Option<f64>,
    pub depth_bottom_m: Option<f64>,
    pub sample_volume_cm3: Option<f64>,
    pub pressure_head_kpa: Option<f64>,
    pub observation_duration_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct HydraulicObservation {
    pub record_id: String,
    pub source_dataset_id: String,
    // Canonical group IDs must be established by documented crosswalks.
    // They must not be guessed from rounded coordinates or row ordering.
    pub study_group_id: Option<String>,
    pub site_group_id: Option<String>,
    pub physical_sample_group_id: Option<String>,
    pub target: HydraulicTarget,
    pub value: f64,
    pub original_unit: String,
    pub standard_uncertainty: Option<f64>,
    pub support: MeasurementSupport,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GroupingLevel {
    PhysicalSample,
    Site,
    Study,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FoldAssignment {
    pub record_id: String,
    pub canonical_group_id: String,
    pub fold: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ContractError {
    EmptyField(&'static str),
    MissingIdentifier {
        record_id: String,
        level: GroupingLevel,
    },
    InvalidNumber(&'static str),
    InvalidRange(&'static str),
    DuplicateRecordId(String),
    UnknownDataset(String),
    WrongDatasetRole {
        dataset_id: String,
        expected: DatasetRole,
    },
    InvalidFoldCount,
    InsufficientGroups {
        groups: usize,
        folds: usize,
    },
    InvalidFoldIndex,
    EmptyFold(usize),
    Leakage {
        group_id: String,
        first_fold: usize,
        second_fold: usize,
    },
}

impl Display for ContractError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyField(name) => write!(formatter, "required field {name} is empty"),
            Self::MissingIdentifier { record_id, level } => {
                write!(
                    formatter,
                    "record {record_id} lacks {level:?} grouping identifier"
                )
            }
            Self::InvalidNumber(name) => write!(formatter, "{name} must be finite and positive"),
            Self::InvalidRange(name) => write!(formatter, "{name} is outside its valid range"),
            Self::DuplicateRecordId(record_id) => {
                write!(formatter, "duplicate observation record ID {record_id}")
            }
            Self::UnknownDataset(dataset_id) => write!(formatter, "unknown dataset {dataset_id}"),
            Self::WrongDatasetRole {
                dataset_id,
                expected,
            } => write!(
                formatter,
                "dataset {dataset_id} does not have required role {expected:?}"
            ),
            Self::InvalidFoldCount => write!(formatter, "fold count must be at least two"),
            Self::InsufficientGroups { groups, folds } => write!(
                formatter,
                "cannot populate {folds} folds from only {groups} independent groups"
            ),
            Self::InvalidFoldIndex => write!(formatter, "fold index is outside the declared range"),
            Self::EmptyFold(fold) => write!(formatter, "fold {fold} has no assigned groups"),
            Self::Leakage {
                group_id,
                first_fold,
                second_fold,
            } => write!(
                formatter,
                "group {group_id} leaks across folds {first_fold} and {second_fold}"
            ),
        }
    }
}

impl Error for ContractError {}

fn nonempty(value: &str, name: &'static str) -> Result<(), ContractError> {
    if value.trim().is_empty() {
        Err(ContractError::EmptyField(name))
    } else {
        Ok(())
    }
}

fn finite_positive(value: f64, name: &'static str) -> Result<(), ContractError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(ContractError::InvalidNumber(name))
    }
}

fn finite_nonnegative(value: f64, name: &'static str) -> Result<(), ContractError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(ContractError::InvalidRange(name))
    }
}

pub fn validate_manifest(manifest: &DatasetManifest) -> Result<(), ContractError> {
    nonempty(&manifest.dataset_id, "dataset_id")?;
    nonempty(&manifest.title, "title")?;
    nonempty(&manifest.version, "version")?;
    nonempty(&manifest.record_url, "record_url")?;
    nonempty(&manifest.notes, "notes")?;

    if manifest
        .doi
        .as_ref()
        .is_some_and(|doi| doi.trim().is_empty())
    {
        return Err(ContractError::EmptyField("doi"));
    }
    if manifest
        .license
        .as_ref()
        .is_some_and(|license| license.trim().is_empty())
    {
        return Err(ContractError::EmptyField("license"));
    }
    if manifest.row_count == Some(0)
        || manifest.independent_study_count == Some(0)
        || manifest.independent_site_count == Some(0)
    {
        return Err(ContractError::InvalidRange("dataset counts"));
    }
    Ok(())
}

pub fn validate_observation(
    observation: &HydraulicObservation,
    known_manifests: &[DatasetManifest],
) -> Result<(), ContractError> {
    nonempty(&observation.record_id, "record_id")?;
    nonempty(&observation.source_dataset_id, "source_dataset_id")?;
    nonempty(&observation.original_unit, "original_unit")?;
    nonempty(
        &observation.support.original_method_label,
        "original_method_label",
    )?;

    let manifest = known_manifests
        .iter()
        .find(|manifest| manifest.dataset_id == observation.source_dataset_id)
        .ok_or_else(|| ContractError::UnknownDataset(observation.source_dataset_id.clone()))?;
    if manifest.role != DatasetRole::ObservedMeasurements {
        return Err(ContractError::WrongDatasetRole {
            dataset_id: manifest.dataset_id.clone(),
            expected: DatasetRole::ObservedMeasurements,
        });
    }

    match observation.target {
        HydraulicTarget::SaturatedConductivity
        | HydraulicTarget::UnsaturatedConductivity
        | HydraulicTarget::WaterDiffusivity => {
            finite_positive(observation.value, "hydraulic target value")?;
        }
        HydraulicTarget::WaterRetention => {
            finite_nonnegative(observation.value, "water-retention value")?;
            let unit = observation.original_unit.trim().to_ascii_lowercase();
            if matches!(unit.as_str(), "m3/m3" | "cm3/cm3" | "m^3/m^3" | "cm^3/cm^3") {
                if observation.value > 1.0 {
                    return Err(ContractError::InvalidRange(
                        "volumetric water content expressed as a fraction",
                    ));
                }
            } else if matches!(unit.as_str(), "%" | "percent" | "percentage")
                && observation.value > 100.0
            {
                return Err(ContractError::InvalidRange("water retention percentage"));
            }
        }
    }

    if let Some(uncertainty) = observation.standard_uncertainty {
        finite_nonnegative(uncertainty, "standard_uncertainty")?;
    }

    for (value, name) in [
        (observation.support.spatial_support_m, "spatial_support_m"),
        (observation.support.sample_volume_cm3, "sample_volume_cm3"),
        (
            observation.support.observation_duration_s,
            "observation_duration_s",
        ),
    ] {
        if let Some(value) = value {
            finite_positive(value, name)?;
        }
    }

    if let (Some(top), Some(bottom)) = (
        observation.support.depth_top_m,
        observation.support.depth_bottom_m,
    ) {
        if !top.is_finite() || !bottom.is_finite() || top < 0.0 || bottom <= top {
            return Err(ContractError::InvalidRange("depth interval"));
        }
    }

    if let Some(head) = observation.support.pressure_head_kpa {
        if !head.is_finite() {
            return Err(ContractError::InvalidNumber("pressure_head_kpa"));
        }
    }

    Ok(())
}

fn canonical_group_id(
    observation: &HydraulicObservation,
    level: GroupingLevel,
) -> Result<&str, ContractError> {
    let selected = match level {
        GroupingLevel::PhysicalSample => observation.physical_sample_group_id.as_deref(),
        GroupingLevel::Site => observation.site_group_id.as_deref(),
        GroupingLevel::Study => observation.study_group_id.as_deref(),
    };
    selected
        .filter(|identifier| !identifier.trim().is_empty())
        .ok_or_else(|| ContractError::MissingIdentifier {
            record_id: observation.record_id.clone(),
            level,
        })
}

fn stable_hash(value: &str) -> u64 {
    // FNV-1a is used only for deterministic fold assignment, not security.
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in value.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

pub fn grouped_folds(
    observations: &[HydraulicObservation],
    level: GroupingLevel,
    fold_count: usize,
) -> Result<Vec<FoldAssignment>, ContractError> {
    if fold_count < 2 {
        return Err(ContractError::InvalidFoldCount);
    }

    // Build groups before assigning folds. Sorting by descending row count,
    // then by stable hash and canonical ID, makes the greedy assignment
    // deterministic regardless of incoming row order.
    let mut grouped_record_ids = BTreeMap::<String, Vec<String>>::new();
    let mut record_ids = BTreeSet::new();
    for observation in observations {
        if !record_ids.insert(observation.record_id.clone()) {
            return Err(ContractError::DuplicateRecordId(
                observation.record_id.clone(),
            ));
        }
        let group_id = canonical_group_id(observation, level)?;
        grouped_record_ids
            .entry(group_id.to_string())
            .or_default()
            .push(observation.record_id.clone());
    }

    let independent_group_count = grouped_record_ids.len();
    if independent_group_count < fold_count {
        return Err(ContractError::InsufficientGroups {
            groups: independent_group_count,
            folds: fold_count,
        });
    }

    let mut groups: Vec<(String, Vec<String>)> = grouped_record_ids.into_iter().collect();
    groups.sort_by(|left, right| {
        right
            .1
            .len()
            .cmp(&left.1.len())
            .then_with(|| stable_hash(&left.0).cmp(&stable_hash(&right.0)))
            .then_with(|| left.0.cmp(&right.0))
    });

    let mut record_loads = vec![0usize; fold_count];
    let mut group_loads = vec![0usize; fold_count];
    let mut group_to_fold = BTreeMap::<String, usize>::new();

    for (group_id, record_ids_for_group) in groups {
        let selected_fold = (0..fold_count)
            .min_by_key(|fold| (record_loads[*fold], group_loads[*fold], *fold))
            .ok_or(ContractError::InvalidFoldCount)?;
        record_loads[selected_fold] += record_ids_for_group.len();
        group_loads[selected_fold] += 1;
        group_to_fold.insert(group_id, selected_fold);
    }

    let assignments: Vec<FoldAssignment> = observations
        .iter()
        .map(|observation| {
            let group_id = canonical_group_id(observation, level)?;
            let fold = group_to_fold.get(group_id).copied().ok_or_else(|| {
                ContractError::MissingIdentifier {
                    record_id: observation.record_id.clone(),
                    level,
                }
            })?;
            Ok(FoldAssignment {
                record_id: observation.record_id.clone(),
                canonical_group_id: group_id.to_string(),
                fold,
            })
        })
        .collect::<Result<_, ContractError>>()?;

    validate_no_group_leakage(&assignments, fold_count)?;
    Ok(assignments)
}

pub fn validate_no_group_leakage(
    assignments: &[FoldAssignment],
    fold_count: usize,
) -> Result<(), ContractError> {
    if fold_count < 2 {
        return Err(ContractError::InvalidFoldCount);
    }
    let mut group_folds = BTreeMap::<&str, usize>::new();
    let mut record_ids = BTreeSet::new();
    let mut populated_folds = vec![false; fold_count];
    for assignment in assignments {
        if assignment.fold >= fold_count {
            return Err(ContractError::InvalidFoldIndex);
        }
        populated_folds[assignment.fold] = true;
        if !record_ids.insert(assignment.record_id.as_str()) {
            return Err(ContractError::DuplicateRecordId(
                assignment.record_id.clone(),
            ));
        }
        match group_folds.get(assignment.canonical_group_id.as_str()) {
            Some(first_fold) if *first_fold != assignment.fold => {
                return Err(ContractError::Leakage {
                    group_id: assignment.canonical_group_id.clone(),
                    first_fold: *first_fold,
                    second_fold: assignment.fold,
                });
            }
            Some(_) => {}
            None => {
                group_folds.insert(&assignment.canonical_group_id, assignment.fold);
            }
        }
    }
    if let Some(empty_fold) = populated_folds.iter().position(|populated| !populated) {
        return Err(ContractError::EmptyFold(empty_fold));
    }
    Ok(())
}

pub fn observed_measurement_registry() -> Vec<DatasetManifest> {
    vec![
        DatasetManifest {
            dataset_id: "soilksatdb-v0.3".into(),
            title: "SoilKsatDB global saturated hydraulic conductivity measurements".into(),
            version: "Zenodo record 4384592, v0.3".into(),
            doi: Some("10.5281/zenodo.4384592".into()),
            record_url: "https://zenodo.org/records/4384592".into(),
            license: None,
            role: DatasetRole::ObservedMeasurements,
            row_count: Some(13_267),
            independent_study_count: None,
            independent_site_count: Some(1_910),
            method_metadata_available: true,
            spatial_metadata_available: true,
            notes: "Current Zenodo v0.3 record reports 13,267 measurements at 1,910 sites. The 2021 ESSD paper reports 13,258 values at 1,908 locations, and 13,286 when both field and laboratory measurements reported for the same location are counted; its method split is 4,131 field and 9,155 laboratory. Reconcile exact file rows and release version before split construction; do not treat rows as independent soils.".into(),
        },
        DatasetManifest {
            dataset_id: "unsoda-2.0".into(),
            title: "UNSODA 2.0 unsaturated soil hydraulic database".into(),
            version: "UNSODA 2.0".into(),
            doi: Some("10.3886/E250569V1".into()),
            record_url: "https://www.datalumos.org/datalumos/project/250569/version/V1/view".into(),
            license: Some("https://www.usa.gov/publicdomain/label/1.0/".into()),
            role: DatasetRole::ObservedMeasurements,
            row_count: Some(790),
            independent_study_count: None,
            independent_site_count: None,
            method_metadata_available: true,
            spatial_metadata_available: false,
            notes: "USDA/National Agricultural Library UNSODA 2.0 archive distributed by ICPSR V1 (2026-06-28), DOI 10.3886/E250569V1. The original database contains measurements of retention, unsaturated conductivity, water diffusivity and pedological information for about 790 soil samples; the 790 sample count is not a study count.".into(),
        },
        DatasetManifest {
            dataset_id: "cg-ksat-7-1278".into(),
            title: "CG/KSAT/7/1278 coarse-grained soil hydraulic conductivity database".into(),
            version: "Mendeley Data v1, 2026-09-01".into(),
            doi: Some("10.17632/fp8n58ny46.1".into()),
            record_url: "https://data.mendeley.com/datasets/fp8n58ny46/1".into(),
            license: Some("CC BY 4.0".into()),
            role: DatasetRole::ObservedMeasurements,
            row_count: Some(1_278),
            independent_study_count: Some(53),
            independent_site_count: None,
            method_metadata_available: false,
            spatial_metadata_available: false,
            notes: "Coarse-grained saturated conductivity in mm/s with particle-size and void-ratio covariates; some covariates are missing. Study IDs must be preserved when supplied by the original workbook.".into(),
        },
        DatasetManifest {
            dataset_id: "fg-ksat-6-1358".into(),
            title: "FG/KSAT/6/1358 fine-grained soil hydraulic conductivity database".into(),
            version: "Mendeley Data v2, 2026-10-25".into(),
            doi: Some("10.17632/mvpctdnmxw.2".into()),
            record_url: "https://data.mendeley.com/datasets/mvpctdnmxw/2".into(),
            license: Some("CC BY 4.0".into()),
            role: DatasetRole::ObservedMeasurements,
            row_count: Some(1_358),
            independent_study_count: Some(33),
            independent_site_count: None,
            method_metadata_available: false,
            spatial_metadata_available: false,
            notes: "1,358 fine-grained saturated conductivity observations compiled from 33 published studies. Preserve study IDs from the workbook and verify latest version before split admission.".into(),
        },
        DatasetManifest {
            dataset_id: "euptfv2-lucas-2026".into(),
            title: "EUPTFv2 and LUCAS European topsoil hydraulic property map".into(),
            version: "ESDAC August 2026 release".into(),
            doi: None,
            record_url: "https://esdac.jrc.ec.europa.eu/content/soil-hydraulic-properties-europe-using-euptfv2-and-lucas".into(),
            license: None,
            role: DatasetRole::DerivedMap,
            row_count: None,
            independent_study_count: None,
            independent_site_count: None,
            method_metadata_available: true,
            spatial_metadata_available: true,
            notes: "500 m EU/UK 0-20 cm product with Ksat, retention points and uncertainty maps; derived PTF output, not independent validation truth.".into(),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source_registry() -> Vec<DatasetManifest> {
        observed_measurement_registry()
    }

    fn observation(
        record_id: &str,
        sample_id: &str,
        site_id: &str,
        study_id: &str,
    ) -> HydraulicObservation {
        HydraulicObservation {
            record_id: record_id.into(),
            source_dataset_id: "soilksatdb-v0.3".into(),
            study_group_id: Some(study_id.into()),
            site_group_id: Some(site_id.into()),
            physical_sample_group_id: Some(sample_id.into()),
            target: HydraulicTarget::SaturatedConductivity,
            value: 1.0e-5,
            original_unit: "m/s".into(),
            standard_uncertainty: Some(1.0e-6),
            support: MeasurementSupport {
                domain: SupportDomain::UndisturbedCore,
                protocol: ProtocolClass::LaboratoryConstantHead,
                original_method_label: "constant head permeameter".into(),
                spatial_support_m: Some(0.05),
                depth_top_m: Some(0.05),
                depth_bottom_m: Some(0.10),
                sample_volume_cm3: Some(100.0),
                pressure_head_kpa: None,
                observation_duration_s: Some(1800.0),
            },
        }
    }

    #[test]
    fn source_registry_validates_and_keeps_derived_map_out_of_ground_truth_role() {
        let registry = source_registry();
        for manifest in &registry {
            validate_manifest(manifest).unwrap();
        }
        let map = registry
            .iter()
            .find(|manifest| manifest.dataset_id == "euptfv2-lucas-2026")
            .unwrap();
        assert_eq!(map.role, DatasetRole::DerivedMap);
        assert_eq!(map.row_count, None);
    }

    #[test]
    fn registry_distinguishes_current_release_counts_from_paper_counts() {
        let registry = source_registry();
        let soilksat = registry
            .iter()
            .find(|manifest| manifest.dataset_id == "soilksatdb-v0.3")
            .unwrap();
        assert_eq!(soilksat.row_count, Some(13_267));
        assert_eq!(soilksat.independent_site_count, Some(1_910));
        assert!(soilksat.notes.contains("13,286"));
        assert!(soilksat.notes.contains("4,131 field and 9,155 laboratory"));

        let unsoda = registry
            .iter()
            .find(|manifest| manifest.dataset_id == "unsoda-2.0")
            .unwrap();
        assert_eq!(unsoda.doi.as_deref(), Some("10.3886/E250569V1"));

        let fine = registry
            .iter()
            .find(|manifest| manifest.dataset_id == "fg-ksat-6-1358")
            .unwrap();
        assert_eq!(fine.independent_study_count, Some(33));
    }

    #[test]
    fn observation_retains_protocol_label_and_support_domain() {
        let record = observation("r1", "sample-1", "site-1", "study-1");
        validate_observation(&record, &source_registry()).unwrap();
        assert_eq!(
            record.support.protocol,
            ProtocolClass::LaboratoryConstantHead
        );
        assert_eq!(
            record.support.original_method_label,
            "constant head permeameter"
        );
        assert_eq!(record.support.domain, SupportDomain::UndisturbedCore);
    }

    #[test]
    fn grouped_sample_fold_keeps_repeats_together() {
        let records = vec![
            observation("r1", "sample-1", "site-1", "study-1"),
            observation("r2", "sample-1", "site-1", "study-1"),
            observation("r3", "sample-2", "site-1", "study-1"),
            observation("r4", "sample-3", "site-2", "study-2"),
        ];
        let assignments = grouped_folds(&records, GroupingLevel::PhysicalSample, 3).unwrap();
        assert_eq!(assignments[0].fold, assignments[1].fold);
        assert_ne!(
            assignments[0].canonical_group_id,
            assignments[2].canonical_group_id
        );
        validate_no_group_leakage(&assignments, 3).unwrap();
    }

    #[test]
    fn grouped_site_fold_keeps_all_samples_at_a_site_together() {
        let records = vec![
            observation("r1", "sample-1", "site-a", "study-1"),
            observation("r2", "sample-2", "site-a", "study-1"),
            observation("r3", "sample-3", "site-b", "study-2"),
        ];
        let assignments = grouped_folds(&records, GroupingLevel::Site, 2).unwrap();
        assert_eq!(assignments[0].fold, assignments[1].fold);
        assert_ne!(
            assignments[0].canonical_group_id,
            assignments[2].canonical_group_id
        );
    }

    #[test]
    fn grouped_study_fold_prevents_source_paper_leakage() {
        let records = vec![
            observation("r1", "sample-1", "site-a", "paper-a"),
            observation("r2", "sample-2", "site-b", "paper-a"),
            observation("r3", "sample-3", "site-c", "paper-b"),
        ];
        let assignments = grouped_folds(&records, GroupingLevel::Study, 2).unwrap();
        assert_eq!(assignments[0].fold, assignments[1].fold);
        assert_ne!(
            assignments[0].canonical_group_id,
            assignments[2].canonical_group_id
        );
    }

    #[test]
    fn grouped_folds_balance_record_loads_without_splitting_groups() {
        let mut records = Vec::new();
        for index in 0..4 {
            records.push(observation(
                &format!("large-{index}"),
                "group-large",
                "site-large",
                "study-large",
            ));
        }
        for index in 0..3 {
            records.push(observation(
                &format!("medium-{index}"),
                "group-medium",
                "site-medium",
                "study-medium",
            ));
        }
        for index in 0..2 {
            records.push(observation(
                &format!("small-{index}"),
                "group-small",
                "site-small",
                "study-small",
            ));
        }
        records.push(observation(
            "single",
            "group-single",
            "site-single",
            "study-single",
        ));

        let assignments = grouped_folds(&records, GroupingLevel::PhysicalSample, 3).unwrap();
        let mut loads = [0usize; 3];
        let mut group_folds = BTreeMap::<String, usize>::new();
        for assignment in &assignments {
            loads[assignment.fold] += 1;
            if let Some(previous) =
                group_folds.insert(assignment.canonical_group_id.clone(), assignment.fold)
            {
                assert_eq!(previous, assignment.fold);
            }
        }
        let minimum = *loads.iter().min().unwrap();
        let maximum = *loads.iter().max().unwrap();
        assert_eq!(loads, [4, 3, 3]);
        assert!(maximum - minimum <= 1);

        let mut reversed = records.clone();
        reversed.reverse();
        let reversed_assignments =
            grouped_folds(&reversed, GroupingLevel::PhysicalSample, 3).unwrap();
        let baseline_map: BTreeMap<_, _> = assignments
            .iter()
            .map(|item| (item.canonical_group_id.clone(), item.fold))
            .collect();
        let reversed_map: BTreeMap<_, _> = reversed_assignments
            .iter()
            .map(|item| (item.canonical_group_id.clone(), item.fold))
            .collect();
        assert_eq!(baseline_map, reversed_map);
    }

    #[test]
    fn grouped_folds_reject_fewer_independent_groups_than_folds() {
        let records = vec![
            observation("r1", "same-sample", "same-site", "same-study"),
            observation("r2", "same-sample", "same-site", "same-study"),
        ];
        assert!(matches!(
            grouped_folds(&records, GroupingLevel::PhysicalSample, 2),
            Err(ContractError::InsufficientGroups {
                groups: 1,
                folds: 2
            })
        ));
    }

    #[test]
    fn manually_empty_fold_is_rejected() {
        let assignments = vec![
            FoldAssignment {
                record_id: "r1".into(),
                canonical_group_id: "sample-1".into(),
                fold: 0,
            },
            FoldAssignment {
                record_id: "r2".into(),
                canonical_group_id: "sample-2".into(),
                fold: 0,
            },
        ];
        assert!(matches!(
            validate_no_group_leakage(&assignments, 2),
            Err(ContractError::EmptyFold(1))
        ));
    }

    #[test]
    fn missing_group_identifier_fails_closed() {
        let mut record = observation("r1", "sample-1", "site-1", "study-1");
        record.site_group_id = None;
        let error = grouped_folds(&[record], GroupingLevel::Site, 3).unwrap_err();
        assert!(matches!(
            error,
            ContractError::MissingIdentifier {
                level: GroupingLevel::Site,
                ..
            }
        ));
    }

    #[test]
    fn global_cross_dataset_sample_id_keeps_alias_rows_together() {
        let mut first = observation("r1", "canonical-core-42", "site-9", "study-1");
        first.source_dataset_id = "soilksatdb-v0.3".into();
        let mut second = observation("r2", "canonical-core-42", "site-9", "study-2");
        second.source_dataset_id = "unsoda-2.0".into();
        let third = observation("r3", "independent-core-99", "site-10", "study-3");
        let assignments =
            grouped_folds(&[first, second, third], GroupingLevel::PhysicalSample, 2).unwrap();
        assert_eq!(assignments[0].fold, assignments[1].fold);
        assert_ne!(assignments[0].fold, assignments[2].fold);
        assert_eq!(
            assignments[0].canonical_group_id,
            assignments[1].canonical_group_id
        );
    }

    #[test]
    fn map_products_cannot_be_admitted_as_independent_observations() {
        let mut record = observation("r1", "sample-1", "site-1", "study-1");
        record.source_dataset_id = "euptfv2-lucas-2026".into();
        let error = validate_observation(&record, &source_registry()).unwrap_err();
        assert!(matches!(
            error,
            ContractError::WrongDatasetRole {
                expected: DatasetRole::ObservedMeasurements,
                ..
            }
        ));
    }

    #[test]
    fn retention_ranges_follow_explicit_units_without_guessing_unknown_units() {
        let mut record = observation("r1", "sample-1", "site-1", "study-1");
        record.target = HydraulicTarget::WaterRetention;
        record.original_unit = "m3/m3".into();
        record.value = 0.42;
        validate_observation(&record, &source_registry()).unwrap();
        record.value = 1.02;
        assert!(validate_observation(&record, &source_registry()).is_err());

        record.original_unit = "%".into();
        record.value = 42.0;
        validate_observation(&record, &source_registry()).unwrap();
        record.value = 102.0;
        assert!(validate_observation(&record, &source_registry()).is_err());

        record.original_unit = "kg/kg".into();
        record.value = 1.2;
        validate_observation(&record, &source_registry()).unwrap();
    }

    #[test]
    fn duplicate_record_ids_are_rejected_before_folding() {
        let records = vec![
            observation("same-id", "sample-1", "site-1", "study-1"),
            observation("same-id", "sample-2", "site-2", "study-2"),
        ];
        assert!(matches!(
            grouped_folds(&records, GroupingLevel::PhysicalSample, 3),
            Err(ContractError::DuplicateRecordId(_))
        ));
    }

    #[test]
    fn manually_corrupted_assignments_reveal_cross_fold_leakage() {
        let assignments = vec![
            FoldAssignment {
                record_id: "r1".into(),
                canonical_group_id: "sample-1".into(),
                fold: 0,
            },
            FoldAssignment {
                record_id: "r2".into(),
                canonical_group_id: "sample-1".into(),
                fold: 1,
            },
        ];
        assert!(matches!(
            validate_no_group_leakage(&assignments, 2),
            Err(ContractError::Leakage { .. })
        ));
    }

    #[test]
    fn invalid_support_units_and_nonfinite_uncertainty_are_rejected() {
        let mut record = observation("r1", "sample-1", "site-1", "study-1");
        record.original_unit.clear();
        assert!(validate_observation(&record, &source_registry()).is_err());

        let mut record = observation("r2", "sample-2", "site-2", "study-2");
        record.standard_uncertainty = Some(f64::NAN);
        assert!(validate_observation(&record, &source_registry()).is_err());
    }

    #[test]
    fn map_support_can_be_recorded_but_not_confused_with_core_support() {
        let mut record = observation("r1", "sample-1", "site-1", "study-1");
        record.support.domain = SupportDomain::MapCell;
        record.support.protocol = ProtocolClass::Unknown;
        record.support.spatial_support_m = Some(500.0);
        validate_observation(&record, &source_registry()).unwrap();
        assert_ne!(record.support.domain, SupportDomain::UndisturbedCore);
        assert_eq!(record.support.protocol, ProtocolClass::Unknown);
    }
}
