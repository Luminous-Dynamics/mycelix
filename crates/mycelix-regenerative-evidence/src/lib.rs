// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Dependency-light soil evidence bindings for Mycelix regenerative systems.
//!
//! REGEN-010 deliberately binds regenerative site/plot identity to existing PEF
//! environmental evidence. It does not duplicate generic measurement, unit,
//! uncertainty, spatial, temporal, or provenance fields and it creates no
//! agronomic recommendation, governance, market, carbon, or physical-action authority.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use mycelix_core_types::{
    EnvironmentalObservation, EvidenceClass, LineagedObservation, MAX_ID_BYTES,
};
use mycelix_regenerative_core::{RegenerativeSiteId, SoilPlotId};
use std::{collections::HashSet, fmt};

#[cfg(feature = "serde")]
use serde::{Deserialize, Deserializer, Serialize, de::Error as _};

/// Current REGEN soil-evidence schema version.
pub const SOIL_EVIDENCE_SCHEMA_VERSION: u16 = 1;
/// Maximum number of PEF observation bindings in one soil profile.
pub const MAX_SOIL_OBSERVATION_BINDINGS: usize = 256;
/// Maximum UTF-8 byte length of one sampling/method/group reference.
pub const MAX_CONTEXT_REF_BYTES: usize = 512;

/// Domain role played by one PEF observation inside a soil evidence profile.
///
/// The exact measured or modeled phenomenon remains owned by the referenced
/// [`EnvironmentalObservation::phenomenon`] string. This role is deliberately
/// broader and expresses how the evidence is being used in the soil profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum SoilObservationRole {
    /// Soil acidity/alkalinity evidence such as pH.
    Acidity,
    /// Soil carbon evidence.
    Carbon,
    /// Soil nutrient evidence.
    Nutrient,
    /// Soil physical-property evidence such as bulk density or texture.
    Physical,
    /// Soil water/hydrologic evidence.
    Hydrologic,
    /// Soil biological evidence.
    Biological,
    /// Soil contaminant or undesirable-material evidence.
    Contaminant,
    /// Other soil assessment evidence that does not fit the frozen categories.
    OtherAssessment,
}

/// Optional vertical support of a sample, expressed as millimetres below the
/// declared local surface reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DepthIntervalMm {
    /// Inclusive/nominal top depth in millimetres below the declared surface.
    pub top_mm: u32,
    /// Exclusive/nominal bottom depth in millimetres below the declared surface.
    pub bottom_mm: u32,
}

impl DepthIntervalMm {
    /// Construct a non-empty depth interval.
    pub fn new(top_mm: u32, bottom_mm: u32) -> Result<Self, SoilEvidenceError> {
        let depth = Self { top_mm, bottom_mm };
        depth.validate()?;
        Ok(depth)
    }

    /// Validate interval ordering.
    pub fn validate(&self) -> Result<(), SoilEvidenceError> {
        if self.top_mm >= self.bottom_mm {
            return Err(SoilEvidenceError::InvalidDepthInterval {
                top_mm: self.top_mm,
                bottom_mm: self.bottom_mm,
            });
        }
        Ok(())
    }
}

/// Declared sampling support for a soil evidence binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum SampleSupport {
    /// Sampling support is unknown or deliberately unspecified.
    Unspecified,
    /// One declared point/sample location.
    Point,
    /// A composite formed from multiple constituent samples.
    Composite {
        /// Number of declared constituent samples represented by the composite.
        constituent_count: u16,
    },
}

impl SampleSupport {
    fn validate(&self) -> Result<(), SoilEvidenceError> {
        if let Self::Composite { constituent_count } = self
            && *constituent_count < 2
        {
            return Err(SoilEvidenceError::InvalidCompositeSampleCount(
                *constituent_count,
            ));
        }
        Ok(())
    }
}

/// One soil-domain use of an existing PEF observation.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SoilObservationBinding {
    /// Soil-domain role assigned to the evidence.
    pub role: SoilObservationRole,
    /// Exact opaque PEF observation identifier. No REGEN normalization is applied.
    pub observation_id: String,
    /// Exact PEF phenomenon expected when the observation resolves.
    pub expected_phenomenon: String,
    /// Optional exact evidence-class expectation.
    pub expected_class: Option<EvidenceClass>,
    /// Optional declared sample depth support. `None` does not mean surface soil.
    pub depth: Option<DepthIntervalMm>,
    /// Declared point/composite/unspecified sampling support.
    pub sample_support: SampleSupport,
    /// Optional reference tying several observations to the same physical sample/group.
    pub sample_group_ref: Option<String>,
    /// Optional sampling-method reference; presence does not prove compliance.
    pub sampling_method_ref: Option<String>,
    /// Optional laboratory/analytical-method reference; presence does not prove competence.
    pub laboratory_method_ref: Option<String>,
}

impl SoilObservationBinding {
    /// Validate the binding without resolving external evidence.
    pub fn validate(&self) -> Result<(), SoilEvidenceError> {
        require_text("binding.observation_id", &self.observation_id, MAX_ID_BYTES)?;
        require_text(
            "binding.expected_phenomenon",
            &self.expected_phenomenon,
            MAX_ID_BYTES,
        )?;
        if let Some(depth) = self.depth {
            depth.validate()?;
        }
        self.sample_support.validate()?;
        for (field, value) in [
            ("binding.sample_group_ref", self.sample_group_ref.as_deref()),
            (
                "binding.sampling_method_ref",
                self.sampling_method_ref.as_deref(),
            ),
            (
                "binding.laboratory_method_ref",
                self.laboratory_method_ref.as_deref(),
            ),
        ] {
            if let Some(value) = value {
                require_text(field, value, MAX_CONTEXT_REF_BYTES)?;
            }
        }
        Ok(())
    }
}

/// Structural soil evidence profile for one regenerative site/plot.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct SoilObservationProfile {
    /// Soil evidence schema version.
    pub schema_version: u16,
    /// Regenerative site identity.
    pub site_id: RegenerativeSiteId,
    /// Soil plot identity.
    pub plot_id: SoilPlotId,
    /// PEF evidence bindings used by this profile.
    pub bindings: Vec<SoilObservationBinding>,
}

impl SoilObservationProfile {
    /// Construct and structurally validate a soil evidence profile.
    pub fn new(
        site_id: RegenerativeSiteId,
        plot_id: SoilPlotId,
        bindings: Vec<SoilObservationBinding>,
    ) -> Result<Self, SoilEvidenceError> {
        let profile = Self {
            schema_version: SOIL_EVIDENCE_SCHEMA_VERSION,
            site_id,
            plot_id,
            bindings,
        };
        profile.validate()?;
        Ok(profile)
    }

    /// Validate schema shape, typed identities, context references, and duplicate use.
    pub fn validate(&self) -> Result<(), SoilEvidenceError> {
        if self.schema_version != SOIL_EVIDENCE_SCHEMA_VERSION {
            return Err(SoilEvidenceError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        if self.bindings.is_empty() {
            return Err(SoilEvidenceError::MissingBindings);
        }
        if self.bindings.len() > MAX_SOIL_OBSERVATION_BINDINGS {
            return Err(SoilEvidenceError::TooManyBindings {
                actual: self.bindings.len(),
                max: MAX_SOIL_OBSERVATION_BINDINGS,
            });
        }

        let mut observation_ids = HashSet::with_capacity(self.bindings.len());
        for binding in &self.bindings {
            binding.validate()?;
            if !observation_ids.insert(binding.observation_id.as_str()) {
                return Err(SoilEvidenceError::DuplicateObservationId(
                    binding.observation_id.clone(),
                ));
            }
        }
        Ok(())
    }

    /// Resolve and validate every bound PEF observation using a caller-provided resolver.
    ///
    /// The resolver may be backed by memory, a database, Holochain, a file, or another
    /// application boundary. This crate performs no hidden I/O or network lookup.
    pub fn validate_resolved<'a, F>(&self, mut resolve: F) -> Result<(), SoilEvidenceError>
    where
        F: FnMut(&str) -> Option<ResolvedEvidence<'a>>,
    {
        self.validate()?;
        for binding in &self.bindings {
            let resolved = resolve(&binding.observation_id).ok_or_else(|| {
                SoilEvidenceError::MissingResolvedEvidence(binding.observation_id.clone())
            })?;
            validate_resolved_binding(binding, resolved)?;
        }
        Ok(())
    }
}

#[cfg(feature = "serde")]
#[derive(Deserialize)]
struct SoilObservationProfileWire {
    schema_version: u16,
    site_id: RegenerativeSiteId,
    plot_id: SoilPlotId,
    bindings: Vec<SoilObservationBinding>,
}

#[cfg(feature = "serde")]
impl<'de> Deserialize<'de> for SoilObservationProfile {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = SoilObservationProfileWire::deserialize(deserializer)?;
        let profile = Self {
            schema_version: wire.schema_version,
            site_id: wire.site_id,
            plot_id: wire.plot_id,
            bindings: wire.bindings,
        };
        profile.validate().map_err(D::Error::custom)?;
        Ok(profile)
    }
}

/// Resolved PEF evidence supplied to soil-profile validation.
///
/// Computed evidence must be supplied as a validated [`LineagedObservation`],
/// preventing a bare Derived/Inferred/Forecast/Scenario payload from laundering
/// away its required provenance graph.
#[derive(Debug, Clone, Copy)]
pub enum ResolvedEvidence<'a> {
    /// Raw Reported/Observed evidence.
    Raw(&'a EnvironmentalObservation),
    /// Computed Derived/Inferred/Forecast/Scenario evidence with PEF lineage.
    Lineaged(&'a LineagedObservation),
}

fn validate_resolved_binding(
    binding: &SoilObservationBinding,
    resolved: ResolvedEvidence<'_>,
) -> Result<(), SoilEvidenceError> {
    let observation = match resolved {
        ResolvedEvidence::Raw(observation) => {
            observation
                .validate()
                .map_err(|error| SoilEvidenceError::InvalidResolvedEvidence {
                    observation_id: binding.observation_id.clone(),
                    reason: error.to_string(),
                })?;
            if !matches!(
                observation.class,
                EvidenceClass::Reported | EvidenceClass::Observed
            ) {
                return Err(SoilEvidenceError::ComputedEvidenceRequiresLineage {
                    observation_id: binding.observation_id.clone(),
                    actual: observation.class,
                });
            }
            observation
        }
        ResolvedEvidence::Lineaged(product) => {
            product
                .validate()
                .map_err(|error| SoilEvidenceError::InvalidResolvedEvidence {
                    observation_id: binding.observation_id.clone(),
                    reason: error.to_string(),
                })?;
            &product.observation
        }
    };

    if observation.id != binding.observation_id {
        return Err(SoilEvidenceError::ObservationIdMismatch {
            requested: binding.observation_id.clone(),
            resolved: observation.id.clone(),
        });
    }
    if observation.phenomenon != binding.expected_phenomenon {
        return Err(SoilEvidenceError::PhenomenonMismatch {
            observation_id: binding.observation_id.clone(),
            expected: binding.expected_phenomenon.clone(),
            actual: observation.phenomenon.clone(),
        });
    }
    if let Some(expected) = binding.expected_class
        && observation.class != expected
    {
        return Err(SoilEvidenceError::EvidenceClassMismatch {
            observation_id: binding.observation_id.clone(),
            expected,
            actual: observation.class,
        });
    }
    Ok(())
}

/// Soil evidence validation failures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SoilEvidenceError {
    /// Unsupported top-level schema version.
    UnsupportedSchemaVersion(u16),
    /// A soil profile must bind at least one observation.
    MissingBindings,
    /// Binding count exceeds the bounded profile size.
    TooManyBindings {
        /// Actual number of bindings.
        actual: usize,
        /// Maximum allowed number of bindings.
        max: usize,
    },
    /// Required text field is empty or whitespace-only.
    EmptyField(&'static str),
    /// Text field exceeds its byte bound.
    FieldTooLong {
        /// Field name.
        field: &'static str,
        /// Actual UTF-8 byte length.
        actual: usize,
        /// Maximum UTF-8 byte length.
        max: usize,
    },
    /// Depth interval is empty or inverted.
    InvalidDepthInterval {
        /// Top depth in millimetres.
        top_mm: u32,
        /// Bottom depth in millimetres.
        bottom_mm: u32,
    },
    /// A composite must contain at least two constituent samples.
    InvalidCompositeSampleCount(u16),
    /// One PEF observation cannot be relabeled multiple times in one profile.
    DuplicateObservationId(String),
    /// Caller resolver did not return required evidence.
    MissingResolvedEvidence(String),
    /// Resolved PEF evidence failed its owning validator.
    InvalidResolvedEvidence {
        /// Requested PEF observation identity.
        observation_id: String,
        /// Validation failure rendered by the owning PEF type.
        reason: String,
    },
    /// Computed evidence was supplied without PEF lineage.
    ComputedEvidenceRequiresLineage {
        /// Requested PEF observation identity.
        observation_id: String,
        /// Computed evidence class that requires lineage.
        actual: EvidenceClass,
    },
    /// Resolver returned evidence under a different exact PEF ID.
    ObservationIdMismatch {
        /// Exact requested ID.
        requested: String,
        /// Exact resolved ID.
        resolved: String,
    },
    /// Resolver returned the right ID with a different phenomenon.
    PhenomenonMismatch {
        /// PEF observation identity.
        observation_id: String,
        /// Exact expected PEF phenomenon.
        expected: String,
        /// Exact resolved PEF phenomenon.
        actual: String,
    },
    /// Resolved PEF class did not match an explicit binding expectation.
    EvidenceClassMismatch {
        /// PEF observation identity.
        observation_id: String,
        /// Expected evidence class.
        expected: EvidenceClass,
        /// Resolved evidence class.
        actual: EvidenceClass,
    },
}

impl fmt::Display for SoilEvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion(version) => {
                write!(f, "unsupported soil evidence schema version {version}")
            }
            Self::MissingBindings => {
                f.write_str("soil evidence profile requires at least one binding")
            }
            Self::TooManyBindings { actual, max } => {
                write!(
                    f,
                    "soil evidence profile has {actual} bindings; maximum is {max}"
                )
            }
            Self::EmptyField(field) => write!(f, "{field} cannot be empty"),
            Self::FieldTooLong { field, actual, max } => {
                write!(f, "{field} is {actual} bytes; maximum is {max}")
            }
            Self::InvalidDepthInterval { top_mm, bottom_mm } => write!(
                f,
                "soil depth interval must be non-empty and ordered; got {top_mm}..{bottom_mm} mm"
            ),
            Self::InvalidCompositeSampleCount(count) => {
                write!(
                    f,
                    "composite sample requires at least two constituents; got {count}"
                )
            }
            Self::DuplicateObservationId(id) => {
                write!(f, "PEF observation {id} is bound more than once")
            }
            Self::MissingResolvedEvidence(id) => {
                write!(f, "missing resolved PEF evidence for {id}")
            }
            Self::InvalidResolvedEvidence {
                observation_id,
                reason,
            } => {
                write!(
                    f,
                    "invalid resolved PEF evidence {observation_id}: {reason}"
                )
            }
            Self::ComputedEvidenceRequiresLineage {
                observation_id,
                actual,
            } => write!(
                f,
                "computed PEF evidence {observation_id} with class {actual:?} requires LineagedObservation"
            ),
            Self::ObservationIdMismatch {
                requested,
                resolved,
            } => write!(
                f,
                "resolved PEF observation id {resolved} does not exactly match requested {requested}"
            ),
            Self::PhenomenonMismatch {
                observation_id,
                expected,
                actual,
            } => write!(
                f,
                "PEF observation {observation_id} phenomenon {actual} does not match expected {expected}"
            ),
            Self::EvidenceClassMismatch {
                observation_id,
                expected,
                actual,
            } => write!(
                f,
                "PEF observation {observation_id} class {actual:?} does not match expected {expected:?}"
            ),
        }
    }
}

impl std::error::Error for SoilEvidenceError {}

fn require_text(field: &'static str, value: &str, max: usize) -> Result<(), SoilEvidenceError> {
    if value.trim().is_empty() {
        return Err(SoilEvidenceError::EmptyField(field));
    }
    if value.len() > max {
        return Err(SoilEvidenceError::FieldTooLong {
            field,
            actual: value.len(),
            max,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_core_types::{
        EvidenceLineage, ExternalEvidenceRef, GeoPoint, LineageRef, LineageRoot, LineageSource,
        LineageStep, Measurement, ProducerIdentity, ProducerKind, SpatialExtent, TemporalExtent,
        Uncertainty,
    };

    fn evidence() -> ExternalEvidenceRef {
        ExternalEvidenceRef {
            source_system: "soil-lab".into(),
            resource_id: "report/42".into(),
            content_digest: Some("sha256:feedbeef".into()),
            retrieved_at: Some(1_789_000_000),
            license: None,
        }
    }

    fn observation(id: &str, phenomenon: &str, class: EvidenceClass) -> EnvironmentalObservation {
        EnvironmentalObservation::new(
            id,
            phenomenon,
            class,
            Some(Measurement::new(6.4, "1").unwrap()),
            SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            TemporalExtent::instant(1_789_000_000),
            Uncertainty::Unspecified,
            vec![evidence()],
        )
        .unwrap()
    }

    fn lineage(output_id: &str) -> EvidenceLineage {
        EvidenceLineage::new(
            output_id,
            vec![LineageRoot {
                id: "raw".into(),
                source: LineageSource::Observation("soil:raw:carbon".into()),
            }],
            vec![LineageStep {
                id: "derive".into(),
                operation: "dry_matter_conversion".into(),
                producer: ProducerIdentity {
                    kind: ProducerKind::DeterministicTransform,
                    implementation: "mycelix://regen/test".into(),
                    version: Some("1".into()),
                    code_digest: Some("sha256:code".into()),
                    configuration_digest: Some("sha256:config".into()),
                    environment_digest: Some("sha256:env".into()),
                },
                inputs: vec![LineageRef::Root("raw".into())],
                output_digest: None,
                completed_at: Some(1_789_000_001),
            }],
            "derive",
        )
        .unwrap()
    }

    fn binding(id: &str, phenomenon: &str, class: EvidenceClass) -> SoilObservationBinding {
        SoilObservationBinding {
            role: SoilObservationRole::Acidity,
            observation_id: id.into(),
            expected_phenomenon: phenomenon.into(),
            expected_class: Some(class),
            depth: Some(DepthIntervalMm::new(0, 150).unwrap()),
            sample_support: SampleSupport::Composite {
                constituent_count: 5,
            },
            sample_group_ref: Some("sample-group:plot-a:2026-09-15".into()),
            sampling_method_ref: Some("method:soil-grid-v1".into()),
            laboratory_method_ref: Some("lab-method:ph-water-v1".into()),
        }
    }

    fn profile(binding: SoilObservationBinding) -> SoilObservationProfile {
        SoilObservationProfile::new(
            RegenerativeSiteId::new("jhb-community-1").unwrap(),
            SoilPlotId::new("plot-a").unwrap(),
            vec![binding],
        )
        .unwrap()
    }

    #[test]
    fn structural_profile_preserves_typed_site_plot_and_binding() {
        let profile = profile(binding("soil:ph:1", "soil_ph", EvidenceClass::Observed));
        assert_eq!(profile.site_id.local_token(), "jhb-community-1");
        assert_eq!(profile.plot_id.local_token(), "plot-a");
        assert_eq!(profile.bindings.len(), 1);
        profile.validate().unwrap();
    }

    #[test]
    fn missing_bindings_fail_closed() {
        let result = SoilObservationProfile::new(
            RegenerativeSiteId::new("site").unwrap(),
            SoilPlotId::new("plot").unwrap(),
            vec![],
        );
        assert_eq!(result, Err(SoilEvidenceError::MissingBindings));
    }

    #[test]
    fn duplicate_observation_id_cannot_be_relabeled() {
        let first = binding("soil:shared:1", "soil_ph", EvidenceClass::Observed);
        let mut second = first.clone();
        second.role = SoilObservationRole::Carbon;
        let result = SoilObservationProfile::new(
            RegenerativeSiteId::new("site").unwrap(),
            SoilPlotId::new("plot").unwrap(),
            vec![first, second],
        );
        assert!(matches!(
            result,
            Err(SoilEvidenceError::DuplicateObservationId(_))
        ));
    }

    #[test]
    fn invalid_depth_and_composite_count_are_rejected() {
        assert!(DepthIntervalMm::new(150, 150).is_err());
        let mut invalid = binding("soil:ph:1", "soil_ph", EvidenceClass::Observed);
        invalid.sample_support = SampleSupport::Composite {
            constituent_count: 1,
        };
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn observed_and_reported_raw_evidence_are_admitted() {
        for class in [EvidenceClass::Observed, EvidenceClass::Reported] {
            let id = format!("soil:raw:{class:?}");
            let obs = observation(&id, "soil_ph", class);
            let profile = profile(binding(&id, "soil_ph", class));
            profile
                .validate_resolved(|requested| {
                    (requested == id).then_some(ResolvedEvidence::Raw(&obs))
                })
                .unwrap();
        }
    }

    #[test]
    fn bare_computed_evidence_requires_lineage() {
        let obs = observation("soil:derived:1", "soil_carbon", EvidenceClass::Derived);
        let profile = profile(binding(
            "soil:derived:1",
            "soil_carbon",
            EvidenceClass::Derived,
        ));
        assert!(matches!(
            profile.validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs))),
            Err(SoilEvidenceError::ComputedEvidenceRequiresLineage { .. })
        ));
    }

    #[test]
    fn lineaged_derived_evidence_is_admitted() {
        let obs = observation("soil:derived:1", "soil_carbon", EvidenceClass::Derived);
        let product = LineagedObservation::new(obs, lineage("soil:derived:1")).unwrap();
        let profile = profile(binding(
            "soil:derived:1",
            "soil_carbon",
            EvidenceClass::Derived,
        ));
        profile
            .validate_resolved(|_| Some(ResolvedEvidence::Lineaged(&product)))
            .unwrap();
    }

    #[test]
    fn scenario_class_is_preserved_and_requires_lineage() {
        let obs = observation("soil:scenario:1", "soil_ph", EvidenceClass::Scenario);
        let raw_profile = profile(binding(
            "soil:scenario:1",
            "soil_ph",
            EvidenceClass::Scenario,
        ));
        assert!(matches!(
            raw_profile.validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs))),
            Err(SoilEvidenceError::ComputedEvidenceRequiresLineage { .. })
        ));

        let product = LineagedObservation::new(obs, lineage("soil:scenario:1")).unwrap();
        raw_profile
            .validate_resolved(|_| Some(ResolvedEvidence::Lineaged(&product)))
            .unwrap();
    }

    #[test]
    fn resolver_cannot_substitute_observation_id() {
        let obs = observation("soil:other:1", "soil_ph", EvidenceClass::Observed);
        let profile = profile(binding("soil:wanted:1", "soil_ph", EvidenceClass::Observed));
        assert!(matches!(
            profile.validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs))),
            Err(SoilEvidenceError::ObservationIdMismatch { .. })
        ));
    }

    #[test]
    fn resolver_cannot_substitute_phenomenon() {
        let obs = observation("soil:ph:1", "soil_carbon", EvidenceClass::Observed);
        let profile = profile(binding("soil:ph:1", "soil_ph", EvidenceClass::Observed));
        assert!(matches!(
            profile.validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs))),
            Err(SoilEvidenceError::PhenomenonMismatch { .. })
        ));
    }

    #[test]
    fn explicit_evidence_class_expectation_is_enforced() {
        let obs = observation("soil:ph:1", "soil_ph", EvidenceClass::Reported);
        let profile = profile(binding("soil:ph:1", "soil_ph", EvidenceClass::Observed));
        assert!(matches!(
            profile.validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs))),
            Err(SoilEvidenceError::EvidenceClassMismatch { .. })
        ));
    }

    #[test]
    fn missing_resolution_is_distinct_from_invalid_evidence() {
        let profile = profile(binding("soil:ph:1", "soil_ph", EvidenceClass::Observed));
        assert!(matches!(
            profile.validate_resolved(|_| None),
            Err(SoilEvidenceError::MissingResolvedEvidence(_))
        ));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_and_revalidates_profile() {
        let profile = profile(binding("soil:ph:1", "soil_ph", EvidenceClass::Observed));
        let encoded = serde_json::to_string(&profile).unwrap();
        let decoded: SoilObservationProfile = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, profile);

        let malformed = encoded.replace("\"bottom_mm\":150", "\"bottom_mm\":0");
        assert!(serde_json::from_str::<SoilObservationProfile>(&malformed).is_err());
    }
}
