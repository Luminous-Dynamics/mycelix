use mycelix_core_types::{
    EnvironmentalObservation, EvidenceClass, EvidenceLineage, ExternalEvidenceRef, GeoPoint,
    LineageRef, LineageRoot, LineageSource, LineageStep, LineagedObservation, Measurement,
    ProducerIdentity, ProducerKind, SpatialExtent, TemporalExtent, Uncertainty,
};
use mycelix_regenerative_core::{RegenerativeSiteId, SoilPlotId};
use mycelix_regenerative_evidence::{
    ResolvedEvidence, SampleSupport, SoilEvidenceError, SoilObservationBinding,
    SoilObservationProfile, SoilObservationRole,
};

fn evidence() -> ExternalEvidenceRef {
    ExternalEvidenceRef {
        source_system: "soil-lab".into(),
        resource_id: "report/firewall".into(),
        content_digest: Some("sha256:firewall".into()),
        retrieved_at: None,
        license: None,
    }
}

fn observation(id: &str, class: EvidenceClass) -> EnvironmentalObservation {
    EnvironmentalObservation::new(
        id,
        "soil_ph",
        class,
        Some(Measurement::new(6.5, "1").unwrap()),
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
            source: LineageSource::Observation("soil:raw:source".into()),
        }],
        vec![LineageStep {
            id: "compute".into(),
            operation: "test_transform".into(),
            producer: ProducerIdentity {
                kind: ProducerKind::DeterministicTransform,
                implementation: "mycelix://regen/lineage-firewall-test".into(),
                version: Some("1".into()),
                code_digest: Some("sha256:code".into()),
                configuration_digest: Some("sha256:config".into()),
                environment_digest: Some("sha256:environment".into()),
            },
            inputs: vec![LineageRef::Root("raw".into())],
            output_digest: None,
            completed_at: None,
        }],
        "compute",
    )
    .unwrap()
}

fn profile(id: &str, class: EvidenceClass) -> SoilObservationProfile {
    SoilObservationProfile::new(
        RegenerativeSiteId::new("firewall-site").unwrap(),
        SoilPlotId::new("firewall-plot").unwrap(),
        vec![SoilObservationBinding {
            role: SoilObservationRole::Acidity,
            observation_id: id.into(),
            expected_phenomenon: "soil_ph".into(),
            expected_class: Some(class),
            depth: None,
            sample_support: SampleSupport::Unspecified,
            sample_group_ref: None,
            sampling_method_ref: None,
            laboratory_method_ref: None,
        }],
    )
    .unwrap()
}

#[test]
fn lineaged_raw_classes_are_rejected_by_regen_resolution() {
    for (id, class) in [
        ("soil:laundered:observed", EvidenceClass::Observed),
        ("soil:laundered:reported", EvidenceClass::Reported),
    ] {
        // Construct the public PEF wrapper directly to prove REGEN invokes the
        // owning validator rather than trusting the wrapper's type name alone.
        let product = LineagedObservation {
            observation: observation(id, class),
            lineage: lineage(id),
        };
        let profile = profile(id, class);

        assert!(matches!(
            profile.validate_resolved(|_| Some(ResolvedEvidence::Lineaged(&product))),
            Err(SoilEvidenceError::InvalidResolvedEvidence { .. })
        ));
    }
}

#[test]
fn invalid_nested_pef_payload_is_rejected_before_regen_interpretation() {
    let id = "soil:invalid:nested";
    let mut invalid = observation(id, EvidenceClass::Observed);
    invalid.evidence.clear();
    let profile = profile(id, EvidenceClass::Observed);

    assert!(matches!(
        profile.validate_resolved(|_| Some(ResolvedEvidence::Raw(&invalid))),
        Err(SoilEvidenceError::InvalidResolvedEvidence { .. })
    ));
}
