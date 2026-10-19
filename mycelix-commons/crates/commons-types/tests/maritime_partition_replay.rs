// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, MaritimeStreamDisposition, MaritimeStreamHead,
};

const ROOT_FIXTURE: &str =
    include_str!("../fixtures/maritime-evidence-v1-session-bound-root.json");

fn successor_after(
    previous: &MaritimeEvidenceEnvelope,
    generation: u64,
    sequence: u64,
    observed_at_us: u64,
    evidence_binding: &str,
) -> MaritimeEvidenceEnvelope {
    let mut next = MaritimeEvidenceEnvelope::new(
        previous.platform_id.clone(),
        generation,
        sequence,
        observed_at_us,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"healthy"}"#,
        evidence_binding,
    );
    next.position_evidence_refs = vec!["mycelix-position:measurement:002".into()];
    next.chain_after(previous).unwrap()
}

#[test]
fn restart_partition_replay_and_recovery_preserve_linear_head() {
    let root: MaritimeEvidenceEnvelope = serde_json::from_str(ROOT_FIXTURE).unwrap();
    let initial = MaritimeStreamHead::from_root(&root).unwrap();

    // Simulate persisting only the compact cursor before a communications partition.
    let mut head = MaritimeStreamHead::from_retained(
        initial.platform_id.clone(),
        initial.generation,
        initial.sequence,
        initial.digest.clone(),
    )
    .unwrap();
    assert_eq!(head, initial);

    // Re-delivery of the last known record after reconnect is idempotent.
    let retained_root = head.clone();
    assert_eq!(
        head.ingest(&root).unwrap(),
        MaritimeStreamDisposition::Duplicate
    );
    assert_eq!(head, retained_root);

    // A direct successor carrying a new upstream association is the only event that advances.
    let first_reconnected = successor_after(
        &root,
        7,
        1,
        root.observed_at_us + 1_000_000,
        "symthaea-maritime-session-bound-observation-v1:blake3-256:1111111111111111111111111111111111111111111111111111111111111111",
    );
    assert_eq!(
        head.ingest(&first_reconnected).unwrap(),
        MaritimeStreamDisposition::Advance
    );
    let after_first_advance = head.clone();

    // Exact duplicate and stale pre-partition replay leave the retained head unchanged.
    assert_eq!(
        head.ingest(&first_reconnected).unwrap(),
        MaritimeStreamDisposition::Duplicate
    );
    assert_eq!(head, after_first_advance);
    assert_eq!(
        head.ingest(&root).unwrap(),
        MaritimeStreamDisposition::StaleReplay
    );
    assert_eq!(head, after_first_advance);

    // A skipped sequence is not silently accepted even if it points at the current head.
    let mut gap = MaritimeEvidenceEnvelope::new(
        root.platform_id.clone(),
        7,
        3,
        first_reconnected.observed_at_us + 2_000_000,
        MaritimeEvidenceKind::CommunicationsState,
        r#"{"link":"restored"}"#,
        "symthaea-maritime-session-bound-observation-v1:blake3-256:2222222222222222222222222222222222222222222222222222222222222222",
    );
    gap.previous_event_digest = Some(head.digest.clone());
    assert_eq!(head.ingest(&gap).unwrap(), MaritimeStreamDisposition::Gap);
    assert_eq!(head, after_first_advance);

    // A direct-next record with the wrong predecessor is a fork, not a continuation.
    let mut fork = MaritimeEvidenceEnvelope::new(
        root.platform_id.clone(),
        7,
        2,
        first_reconnected.observed_at_us + 3_000_000,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"degraded"}"#,
        "symthaea-maritime-session-bound-observation-v1:blake3-256:3333333333333333333333333333333333333333333333333333333333333333",
    );
    fork.previous_event_digest = Some("00".repeat(32));
    assert_eq!(head.ingest(&fork).unwrap(), MaritimeStreamDisposition::Fork);
    assert_eq!(head, after_first_advance);

    // Reconnecting in an older software/evidence generation cannot roll the cursor backward.
    let mut regression = MaritimeEvidenceEnvelope::new(
        root.platform_id.clone(),
        6,
        2,
        first_reconnected.observed_at_us + 4_000_000,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"healthy"}"#,
        "symthaea-maritime-session-bound-observation-v1:blake3-256:4444444444444444444444444444444444444444444444444444444444444444",
    );
    regression.previous_event_digest = Some(head.digest.clone());
    assert_eq!(
        head.ingest(&regression).unwrap(),
        MaritimeStreamDisposition::GenerationRegression
    );
    assert_eq!(head, after_first_advance);

    // Recovery resumes only with the exact direct successor. Advancing to a newer generation is
    // allowed and retained, making a subsequent rollback detectable after another restart.
    let recovered = successor_after(
        &first_reconnected,
        8,
        2,
        first_reconnected.observed_at_us + 5_000_000,
        "symthaea-maritime-session-bound-observation-v1:blake3-256:5555555555555555555555555555555555555555555555555555555555555555",
    );
    assert_eq!(
        head.ingest(&recovered).unwrap(),
        MaritimeStreamDisposition::Advance
    );
    assert_eq!(head.sequence, 2);
    assert_eq!(head.generation, 8);
    assert_eq!(head.digest, recovered.content_digest().unwrap());
}
