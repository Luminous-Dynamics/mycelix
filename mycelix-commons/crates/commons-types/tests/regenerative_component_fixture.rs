// SPDX-License-Identifier: AGPL-3.0-or-later
use commons_types::{
    ComponentOriginV1, MaritimeEvidenceEnvelope, MaritimeEvidenceKind,
    RegenerativeComponentEvidenceV1,
};

const FIXTURE: &str = include_str!(
    "../fixtures/maritime-evidence-v1-regenerative-component.json"
);
const EXPECTED_EVENT_DIGEST: &str =
    "8257e77f17651e184109856707fd971d16a727835dff5f3a97a60b79aa85c698";

#[test]
fn regenerative_component_fixture_is_canonical_publishable_and_digest_stable() {
    let envelope: MaritimeEvidenceEnvelope = serde_json::from_str(FIXTURE.trim()).unwrap();
    assert!(envelope.validate().is_ok());
    assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
    assert_eq!(envelope.content_digest().unwrap(), EXPECTED_EVENT_DIGEST);

    let component: RegenerativeComponentEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    assert!(component.validate().is_ok());
    assert_eq!(component.component_id, "pump-impeller-0042");
    assert_eq!(component.origin, ComponentOriginV1::Remanufactured);
    assert_eq!(component.recycled_input_refs, vec!["recovered:impeller-0031"]);

    let rebuilt = component
        .to_maritime_envelope(
            "manta-civil-demo-01",
            3,
            17,
            1_788_900_000_000_000,
            "evidence:qualified-forge-event-17",
        )
        .unwrap();
    assert_eq!(rebuilt, envelope);
    assert_eq!(rebuilt.content_digest().unwrap(), EXPECTED_EVENT_DIGEST);
}

#[test]
fn component_provenance_does_not_claim_authentication_or_manufacturing_recipe() {
    let envelope: MaritimeEvidenceEnvelope = serde_json::from_str(FIXTURE.trim()).unwrap();
    let component: RegenerativeComponentEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();

    // These are opaque references. Their presence does not re-verify the producer,
    // process, metrology equipment, or outer Commons signer in this type.
    assert!(!component.producer_evidence_binding.is_empty());
    assert!(!component.qualification_binding.is_empty());

    for forbidden in [
        "temperature_setpoint",
        "pressure_setpoint",
        "toolpath",
        "enrichment",
        "fuel_fabrication",
        "actuator_command",
    ] {
        assert!(!envelope.payload_json.contains(forbidden));
    }
}
