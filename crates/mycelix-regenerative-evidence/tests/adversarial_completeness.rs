// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! REGEN-010C adversarial completeness over the qualified REGEN-010B API.
//!
//! This campaign adds no product semantics. It closes explicitly preregistered
//! evidence-boundary cases against the already-qualified soil evidence waist.

use mycelix_core_types::{
    EnvironmentalObservation, EvidenceClass, EvidenceLineage, ExternalEvidenceRef, GeoPoint,
    LineageRef, LineageRoot, LineageSource, LineageStep, LineagedObservation, Measurement,
    ProducerIdentity, ProducerKind, SpatialExtent, TemporalExtent, Uncertainty,
};
use mycelix_regenerative_core::{RegenerativeSiteId, SoilPlotId};
use mycelix_regenerative_evidence::{
    DepthIntervalMm, ResolvedEvidence, SampleSupport, SoilEvidenceError, SoilObservationBinding,
    SoilObservationProfile, SoilObservationRole,
};

fn source_evidence() -> ExternalEvidenceRef {
    ExternalEvidenceRef {
        source_system: "regen-010c-fixture".into(),
        resource_id: "fixture/source/1".into(),
        content_digest: Some("sha256:fixture-source".into()),
        retrieved_at: Some(1_789_000_000),
        license: None,
    }
}

fn observation(
    id: &str,
    phenomenon: &str,
    class: EvidenceClass,
    measurement: Option<Measurement>,
    spatial: SpatialExtent,
    temporal: TemporalExtent,
) -> EnvironmentalObservation {
    EnvironmentalObservation::new(
        id,
        phenomenon,
        class,
        measurement,
        spatial,
        temporal,
        Uncertainty::Unspecified,
        vec![source_evidence()],
    )
    .unwrap()
}

fn producer(complete_capsule: bool) -> ProducerIdentity {
    ProducerIdentity {
        kind: ProducerKind::DeterministicTransform,
        implementation: "mycelix://regen/010c-fixture".into(),
        version: Some("1".into()),
        code_digest: complete_capsule.then_some("sha256:code".into()),
        configuration_digest: complete_capsule.then_some("sha256:config".into()),
        environment_digest: complete_capsule.then_some("sha256:env".into()),
    }
}

fn lineage(output_id: &str, complete_capsule: bool) -> EvidenceLineage {
    EvidenceLineage::new(
        output_id,
        vec![LineageRoot {
            id: "raw".into(),
            source: LineageSource::Observation("regen:raw:source".into()),
        }],
        vec![LineageStep {
            id: "compute".into(),
            operation: "regen_010c_fixture_transform".into(),
            producer: producer(complete_capsule),
            inputs: vec![LineageRef::Root("raw".into())],
            output_digest: None,
            completed_at: Some(1_789_000_001),
        }],
        "compute",
    )
    .unwrap()
}

fn binding(id: &str, phenomenon: &str, class: EvidenceClass) -> SoilObservationBinding {
    SoilObservationBinding {
        role: SoilObservationRole::OtherAssessment,
        observation_id: id.into(),
        expected_phenomenon: phenomenon.into(),
        expected_class: Some(class),
        depth: Some(DepthIntervalMm::new(0, 150).unwrap()),
        sample_support: SampleSupport::Point,
        sample_group_ref: Some("sample-group:regen-010c".into()),
        sampling_method_ref: Some("method:regen-010c".into()),
        laboratory_method_ref: None,
    }
}

fn profile(binding: SoilObservationBinding) -> SoilObservationProfile {
    SoilObservationProfile::new(
        RegenerativeSiteId::new("regen-010c-site").unwrap(),
        SoilPlotId::new("plot-a").unwrap(),
        vec![binding],
    )
    .unwrap()
}

#[test]
fn every_bare_computed_class_requires_lineage() {
    for class in [
        EvidenceClass::Derived,
        EvidenceClass::Inferred,
        EvidenceClass::Forecast,
        EvidenceClass::Scenario,
    ] {
        let id = format!("regen:010c:bare:{class:?}");
        let obs = observation(
            &id,
            "soil_state",
            class,
            Some(Measurement::new(1.0, "1").unwrap()),
            SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            TemporalExtent::instant(1_789_000_000),
        );
        let profile = profile(binding(&id, "soil_state", class));
        let result = profile.validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs)));
        assert!(matches!(
            result,
            Err(SoilEvidenceError::ComputedEvidenceRequiresLineage { actual, .. }) if actual == class
        ));
    }
}

#[test]
fn every_computed_class_is_admitted_when_validly_lineaged() {
    for class in [
        EvidenceClass::Derived,
        EvidenceClass::Inferred,
        EvidenceClass::Forecast,
        EvidenceClass::Scenario,
    ] {
        let id = format!("regen:010c:lineaged:{class:?}");
        let obs = observation(
            &id,
            "soil_state",
            class,
            Some(Measurement::new(1.0, "1").unwrap()),
            SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            TemporalExtent::instant(1_789_000_000),
        );
        let product = LineagedObservation::new(obs, lineage(&id, true)).unwrap();
        let profile = profile(binding(&id, "soil_state", class));
        profile
            .validate_resolved(|_| Some(ResolvedEvidence::Lineaged(&product)))
            .unwrap();
    }
}

#[test]
fn unspecified_observation_time_is_preserved_without_ingestion_substitution() {
    let id = "regen:010c:unknown-time";
    let obs = observation(
        id,
        "soil_state",
        EvidenceClass::Observed,
        Some(Measurement::new(1.0, "1").unwrap()),
        SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
        TemporalExtent::Unspecified,
    );
    let profile = profile(binding(id, "soil_state", EvidenceClass::Observed));
    profile
        .validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs)))
        .unwrap();
    assert_eq!(obs.temporal, TemporalExtent::Unspecified);
}

#[test]
fn absent_scalar_measurement_is_preserved_as_none() {
    let id = "regen:010c:no-scalar";
    let obs = observation(
        id,
        "soil_state_event",
        EvidenceClass::Observed,
        None,
        SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
        TemporalExtent::instant(1_789_000_000),
    );
    let profile = profile(binding(id, "soil_state_event", EvidenceClass::Observed));
    profile
        .validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs)))
        .unwrap();
    assert!(obs.measurement.is_none());
}

#[test]
fn spatial_support_is_not_interpreted_as_plot_containment() {
    let id = "regen:010c:spatial-no-containment";
    let spatial = SpatialExtent::Point(GeoPoint::new(80.0, 170.0).unwrap());
    let obs = observation(
        id,
        "soil_state",
        EvidenceClass::Observed,
        Some(Measurement::new(1.0, "1").unwrap()),
        spatial.clone(),
        TemporalExtent::instant(1_789_000_000),
    );
    let profile = profile(binding(id, "soil_state", EvidenceClass::Observed));

    // REGEN-010 carries no plot geometry. Structural/resolved validation therefore
    // preserves PEF spatial support but cannot manufacture a containment theorem.
    profile
        .validate_resolved(|_| Some(ResolvedEvidence::Raw(&obs)))
        .unwrap();
    assert_eq!(obs.spatial, spatial);
}

#[test]
fn valid_lineage_does_not_imply_complete_reproducibility_capsule() {
    let id = "regen:010c:incomplete-capsule";
    let obs = observation(
        id,
        "soil_latent_state",
        EvidenceClass::Inferred,
        Some(Measurement::new(0.5, "1").unwrap()),
        SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
        TemporalExtent::instant(1_789_000_000),
    );
    let product = LineagedObservation::new(obs, lineage(id, false)).unwrap();
    assert!(!product.lineage.steps[0].producer.has_complete_capsule());

    let profile = profile(binding(id, "soil_latent_state", EvidenceClass::Inferred));
    profile
        .validate_resolved(|_| Some(ResolvedEvidence::Lineaged(&product)))
        .unwrap();
}
