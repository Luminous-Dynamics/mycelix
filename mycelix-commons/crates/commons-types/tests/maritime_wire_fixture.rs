// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, verify_maritime_successor,
};

const ROOT_FIXTURE: &str = include_str!("../fixtures/maritime-evidence-v1-root.json");
const CHAINED_FIXTURE: &str = include_str!("../fixtures/maritime-evidence-v1-chained.json");
const ROOT_DIGEST: &str = "2ec31adc7f4ca9200d6c3464171ca639b3075da93e4118bfd7b4688232dfdbc4";
const CHAINED_DIGEST: &str =
    "95b18b682028d2a463633f32ff4aecbdf415d988b83de60f22f5526166d6bdfe";

#[test]
fn canonical_maritime_root_fixture_round_trips_and_pins_digest() {
    let envelope: MaritimeEvidenceEnvelope = serde_json::from_str(ROOT_FIXTURE).unwrap();

    assert_eq!(envelope.schema_version, 1);
    assert_eq!(envelope.platform_id, "auv-01");
    assert_eq!(envelope.generation, 7);
    assert_eq!(envelope.sequence, 42);
    assert_eq!(envelope.observed_at_us, 1_700_000_000_000_042);
    assert_eq!(envelope.kind, MaritimeEvidenceKind::HealthObservation);
    assert_eq!(
        envelope.payload_json,
        r#"{"severity":"healthy","envelope":"normal"}"#
    );
    assert_eq!(
        envelope.position_evidence_refs,
        vec!["mycelix-position:measurement:fixture-001".to_string()]
    );
    assert!(envelope.previous_event_digest.is_none());
    assert_eq!(envelope.validate(), Ok(()));
    assert_eq!(envelope.content_digest().unwrap(), ROOT_DIGEST);

    // Serialization must preserve the v1 field/enum vocabulary expected by
    // independent consumers. Whitespace/order are not part of the digest.
    let reencoded = serde_json::to_string(&envelope).unwrap();
    let reparsed: MaritimeEvidenceEnvelope = serde_json::from_str(&reencoded).unwrap();
    assert_eq!(reparsed, envelope);
    assert_eq!(reparsed.content_digest().unwrap(), ROOT_DIGEST);
}

#[test]
fn v1_wire_rejects_unknown_fields_instead_of_ignoring_future_semantics() {
    let mut value: serde_json::Value = serde_json::from_str(ROOT_FIXTURE).unwrap();
    value
        .as_object_mut()
        .unwrap()
        .insert("authority_override".into(), serde_json::Value::Bool(true));
    let encoded = serde_json::to_string(&value).unwrap();

    assert!(serde_json::from_str::<MaritimeEvidenceEnvelope>(&encoded).is_err());
}

#[test]
fn canonical_chained_fixture_binds_exact_root_and_pins_digest() {
    let root: MaritimeEvidenceEnvelope = serde_json::from_str(ROOT_FIXTURE).unwrap();
    let chained: MaritimeEvidenceEnvelope = serde_json::from_str(CHAINED_FIXTURE).unwrap();

    assert_eq!(chained.sequence, root.sequence + 1);
    assert_eq!(chained.kind, MaritimeEvidenceKind::CommunicationsState);
    assert_eq!(
        chained.previous_event_digest.as_deref(),
        Some(ROOT_DIGEST)
    );
    assert_eq!(verify_maritime_successor(&root, &chained), Ok(()));
    assert_eq!(chained.content_digest().unwrap(), CHAINED_DIGEST);

    // The fixture must not merely contain a plausible predecessor string: changing
    // any claim in the predecessor makes the same chained record invalid.
    let mut tampered_root = root;
    tampered_root.payload_json = r#"{"severity":"degraded","envelope":"safe_transit"}"#.into();
    assert!(verify_maritime_successor(&tampered_root, &chained).is_err());
}
