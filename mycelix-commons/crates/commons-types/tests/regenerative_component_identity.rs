use commons_types::{MaritimeEvidenceEnvelope, RegenerativeComponentEvidenceV1};

const FIXTURE: &str = include_str!("../fixtures/maritime-evidence-v1-regenerative-component.json");

#[test]
fn component_identity_is_distinct_from_maritime_continuity_identity() {
    let envelope: MaritimeEvidenceEnvelope = serde_json::from_str(FIXTURE.trim()).unwrap();
    envelope.validate().unwrap();

    let component: RegenerativeComponentEvidenceV1 =
        serde_json::from_str(&envelope.payload_json).unwrap();
    component.validate().unwrap();

    assert_eq!(
        component.content_digest().unwrap(),
        "4740700979f5a23053eafb477a1e3523e6c04801ead31f7e155440137dfd9c23"
    );
    assert_eq!(
        envelope.content_digest().unwrap(),
        "8257e77f17651e184109856707fd971d16a727835dff5f3a97a60b79aa85c698"
    );
    assert_ne!(component.content_digest().unwrap(), envelope.content_digest().unwrap());
}
