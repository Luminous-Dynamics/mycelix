// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Independent matrix for domain-neutral PEF admission.

use mycelix_core_types::{
    EnvironmentalObservation, EvidenceClass, EvidenceLineage, ExternalEvidenceRef, GeoPoint,
    LineageRef, LineageRoot, LineageSource, LineageStep, LineagedObservation, Measurement,
    ProducerIdentity, ProducerKind, SpatialExtent, TemporalExtent, Uncertainty,
};
use mycelix_regenerative_admission::{
    EvidenceAdmissionError, EvidenceCandidate, EvidenceExpectation, admit_pef_evidence,
};

fn source_evidence() -> ExternalEvidenceRef {
    ExternalEvidenceRef {
        source_system: "regen-019-fixture".into(),
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
        implementation: "mycelix://regen/019-fixture".into(),
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
            operation: "regen_019_fixture_transform".into(),
            producer: producer(complete_capsule),
            inputs: vec![LineageRef::Root("raw".into())],
            output_digest: None,
            completed_at: Some(1_789_000_001),
        }],
        "compute",
    )
    .unwrap()
}

fn expectation<'a>(
    id: &'a str,
    phenomenon: &'a str,
    class: EvidenceClass,
) -> EvidenceExpectation<'a> {
    EvidenceExpectation::new(id, phenomenon, Some(class)).unwrap()
}

#[test]
fn raw_reported_and_observed_are_admitted() {
    for class in [EvidenceClass::Reported, EvidenceClass::Observed] {
        let id = format!("regen:019:raw:{class:?}");
        let obs = observation(
            &id,
            "environmental_state",
            class,
            Some(Measurement::new(1.0, "1").unwrap()),
            SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            TemporalExtent::instant(1_789_000_000),
        );
        let expected = expectation(&id, "environmental_state", class);
        let admitted = admit_pef_evidence(&expected, EvidenceCandidate::Raw(&obs)).unwrap();
        assert_eq!(admitted.id, id);
        assert_eq!(admitted.class, class);
    }
}

#[test]
fn every_bare_computed_class_requires_lineage() {
    for class in [
        EvidenceClass::Derived,
        EvidenceClass::Inferred,
        EvidenceClass::Forecast,
        EvidenceClass::Scenario,
    ] {
        let id = format!("regen:019:bare:{class:?}");
        let obs = observation(
            &id,
            "environmental_state",
            class,
            Some(Measurement::new(1.0, "1").unwrap()),
            SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            TemporalExtent::instant(1_789_000_000),
        );
        let expected = expectation(&id, "environmental_state", class);
        let result = admit_pef_evidence(&expected, EvidenceCandidate::Raw(&obs));
        assert!(matches!(
            result,
            Err(EvidenceAdmissionError::ComputedEvidenceRequiresLineage { actual, .. }) if actual == class
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
        let id = format!("regen:019:lineaged:{class:?}");
        let obs = observation(
            &id,
            "environmental_state",
            class,
            Some(Measurement::new(1.0, "1").unwrap()),
            SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
            TemporalExtent::instant(1_789_000_000),
        );
        let product = LineagedObservation::new(obs, lineage(&id, true)).unwrap();
        let expected = expectation(&id, "environmental_state", class);
        let admitted =
            admit_pef_evidence(&expected, EvidenceCandidate::Lineaged(&product)).unwrap();
        assert_eq!(admitted.class, class);
    }
}

#[test]
fn lineaged_raw_classes_are_rejected_by_pef_owner() {
    for class in [EvidenceClass::Reported, EvidenceClass::Observed] {
        let id = format!("regen:019:laundered:{class:?}");
        let product = LineagedObservation {
            observation: observation(
                &id,
                "environmental_state",
                class,
                Some(Measurement::new(1.0, "1").unwrap()),
                SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
                TemporalExtent::instant(1_789_000_000),
            ),
            lineage: lineage(&id, true),
        };
        let expected = expectation(&id, "environmental_state", class);
        assert!(matches!(
            admit_pef_evidence(&expected, EvidenceCandidate::Lineaged(&product)),
            Err(EvidenceAdmissionError::InvalidEvidence { .. })
        ));
    }
}

#[test]
fn exact_id_phenomenon_and_class_expectations_are_enforced() {
    let obs = observation(
        "regen:019:actual",
        "actual_phenomenon",
        EvidenceClass::Reported,
        Some(Measurement::new(1.0, "1").unwrap()),
        SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
        TemporalExtent::instant(1_789_000_000),
    );

    let wrong_id = expectation(
        "regen:019:wanted",
        "actual_phenomenon",
        EvidenceClass::Reported,
    );
    assert!(matches!(
        admit_pef_evidence(&wrong_id, EvidenceCandidate::Raw(&obs)),
        Err(EvidenceAdmissionError::ObservationIdMismatch { .. })
    ));

    let wrong_phenomenon = expectation(
        "regen:019:actual",
        "different_phenomenon",
        EvidenceClass::Reported,
    );
    assert!(matches!(
        admit_pef_evidence(&wrong_phenomenon, EvidenceCandidate::Raw(&obs)),
        Err(EvidenceAdmissionError::PhenomenonMismatch { .. })
    ));

    let wrong_class = expectation(
        "regen:019:actual",
        "actual_phenomenon",
        EvidenceClass::Observed,
    );
    assert!(matches!(
        admit_pef_evidence(&wrong_class, EvidenceCandidate::Raw(&obs)),
        Err(EvidenceAdmissionError::EvidenceClassMismatch { .. })
    ));
}

#[test]
fn invalid_nested_observation_is_rejected_before_expectation_matching() {
    let mut obs = observation(
        "regen:019:invalid",
        "environmental_state",
        EvidenceClass::Observed,
        Some(Measurement::new(1.0, "1").unwrap()),
        SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
        TemporalExtent::instant(1_789_000_000),
    );
    obs.evidence.clear();
    let expected = expectation(
        "regen:019:invalid",
        "environmental_state",
        EvidenceClass::Observed,
    );
    assert!(matches!(
        admit_pef_evidence(&expected, EvidenceCandidate::Raw(&obs)),
        Err(EvidenceAdmissionError::InvalidEvidence { .. })
    ));
}

#[test]
fn unknown_time_absent_scalar_and_spatial_support_are_preserved() {
    let id = "regen:019:preservation";
    let spatial = SpatialExtent::Point(GeoPoint::new(80.0, 170.0).unwrap());
    let obs = observation(
        id,
        "environmental_event",
        EvidenceClass::Observed,
        None,
        spatial.clone(),
        TemporalExtent::Unspecified,
    );
    let expected = expectation(id, "environmental_event", EvidenceClass::Observed);
    let admitted = admit_pef_evidence(&expected, EvidenceCandidate::Raw(&obs)).unwrap();
    assert!(admitted.measurement.is_none());
    assert_eq!(admitted.temporal, TemporalExtent::Unspecified);
    assert_eq!(admitted.spatial, spatial);
}

#[test]
fn valid_lineage_does_not_imply_complete_reproducibility_capsule() {
    let id = "regen:019:incomplete-capsule";
    let obs = observation(
        id,
        "latent_state",
        EvidenceClass::Inferred,
        Some(Measurement::new(0.5, "1").unwrap()),
        SpatialExtent::Point(GeoPoint::new(-26.2, 28.0).unwrap()),
        TemporalExtent::instant(1_789_000_000),
    );
    let product = LineagedObservation::new(obs, lineage(id, false)).unwrap();
    assert!(!product.lineage.steps[0].producer.has_complete_capsule());
    let expected = expectation(id, "latent_state", EvidenceClass::Inferred);
    admit_pef_evidence(&expected, EvidenceCandidate::Lineaged(&product)).unwrap();
}

#[test]
fn expectation_fields_are_bounded_and_nonempty() {
    assert!(matches!(
        EvidenceExpectation::new("", "phenomenon", None),
        Err(EvidenceAdmissionError::EmptyExpectation(_))
    ));
    assert!(matches!(
        EvidenceExpectation::new("id", "   ", None),
        Err(EvidenceAdmissionError::EmptyExpectation(_))
    ));

    let oversized = "x".repeat(mycelix_core_types::MAX_ID_BYTES + 1);
    assert!(matches!(
        EvidenceExpectation::new(&oversized, "phenomenon", None),
        Err(EvidenceAdmissionError::ExpectationTooLong { .. })
    ));
}
