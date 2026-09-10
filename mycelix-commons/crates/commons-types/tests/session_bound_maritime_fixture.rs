// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{MaritimeEvidenceEnvelope, MaritimeEvidenceKind};
use hdi::prelude::{AgentPubKey, Timestamp};

const SESSION_BOUND_ROOT_FIXTURE: &str =
    include_str!("../fixtures/maritime-evidence-v1-session-bound-root.json");
const SYMTHAEA_OBSERVATION_BINDING: &str =
    "symthaea-maritime-session-bound-observation-v1:blake3-256:c258e96264cee15f3e9fd95b36e049884a6a42e67ae968a36a5b9868fc2e52d0";
const MYCELIX_CONTENT_DIGEST: &str =
    "5de90cb56c9a45e8a304587bb9f17dd9f52d928ebbd20c7e37b1f195a5b829c9";

#[test]
fn session_bound_root_is_publishable_and_pins_mycelix_continuity_digest() {
    let envelope: MaritimeEvidenceEnvelope =
        serde_json::from_str(SESSION_BOUND_ROOT_FIXTURE).unwrap();

    assert_eq!(envelope.schema_version, 1);
    assert_eq!(envelope.platform_id, "auv-01");
    assert_eq!(envelope.generation, 7);
    assert_eq!(envelope.sequence, 0);
    assert_eq!(envelope.observed_at_us, 1_700_000_030_000_000);
    assert_eq!(envelope.kind, MaritimeEvidenceKind::HealthObservation);
    assert_eq!(envelope.payload_json, r#"{"severity":"healthy"}"#);
    assert_eq!(envelope.evidence_binding, SYMTHAEA_OBSERVATION_BINDING);
    assert_eq!(
        envelope.position_evidence_refs,
        vec!["mycelix-position:measurement:001".to_string()]
    );
    assert!(envelope.previous_event_digest.is_none());

    assert_eq!(envelope.validate(), Ok(()));
    assert_eq!(envelope.content_digest().unwrap(), MYCELIX_CONTENT_DIGEST);

    let bridge_event = envelope
        .to_commons_event(
            AgentPubKey::from_raw_36(vec![0u8; 36]),
            Timestamp::from_micros(1_700_000_030_000_000),
        )
        .unwrap();
    assert!(bridge_event.payload.len() <= 8 * 1024);

    let reparsed = MaritimeEvidenceEnvelope::from_commons_event(&bridge_event).unwrap();
    assert_eq!(reparsed, envelope);
    assert_eq!(reparsed.content_digest().unwrap(), MYCELIX_CONTENT_DIGEST);
}
