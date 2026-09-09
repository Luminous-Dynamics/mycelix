// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{MaritimeEvidenceEnvelope, MaritimeEvidenceKind};

const FIXTURE: &str = include_str!("../fixtures/maritime-evidence-v1-root.json");
const EXPECTED_DIGEST: &str = "2ec31adc7f4ca9200d6c3464171ca639b3075da93e4118bfd7b4688232dfdbc4";

#[test]
fn canonical_maritime_fixture_round_trips_and_pins_digest() {
    let envelope: MaritimeEvidenceEnvelope = serde_json::from_str(FIXTURE).unwrap();

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
    assert_eq!(envelope.content_digest().unwrap(), EXPECTED_DIGEST);

    // Serialization must preserve the v1 field/enum vocabulary expected by
    // independent consumers. Whitespace/order are not part of the digest.
    let reencoded = serde_json::to_string(&envelope).unwrap();
    let reparsed: MaritimeEvidenceEnvelope = serde_json::from_str(&reencoded).unwrap();
    assert_eq!(reparsed, envelope);
    assert_eq!(reparsed.content_digest().unwrap(), EXPECTED_DIGEST);
}
