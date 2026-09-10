// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use commons_types::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, MaritimeStreamDisposition, MaritimeStreamHead,
};

fn root() -> MaritimeEvidenceEnvelope {
    MaritimeEvidenceEnvelope::new(
        "auv-01",
        7,
        0,
        1_700_000_030_000_000,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"healthy"}"#,
        "symthaea-maritime-session-bound-observation-v1:blake3-256:c258e96264cee15f3e9fd95b36e049884a6a42e67ae968a36a5b9868fc2e52d0",
    )
}

fn direct_successor(previous: &MaritimeEvidenceEnvelope) -> MaritimeEvidenceEnvelope {
    MaritimeEvidenceEnvelope::new(
        previous.platform_id.clone(),
        previous.generation,
        previous.sequence + 1,
        previous.observed_at_us + 1_000_000,
        MaritimeEvidenceKind::CommunicationsState,
        r#"{"link":"store_forward"}"#,
        previous.evidence_binding.clone(),
    )
    .chain_after(previous)
    .unwrap()
}

#[test]
fn replay_gap_fork_regression_and_wrong_platform_never_advance_retained_head() {
    let root = root();
    let next = direct_successor(&root);
    let mut head = MaritimeStreamHead::from_root(&root).unwrap();

    assert_eq!(head.ingest(&next).unwrap(), MaritimeStreamDisposition::Advance);
    let retained_after_advance = head.clone();

    assert_eq!(head.ingest(&next).unwrap(), MaritimeStreamDisposition::Duplicate);
    assert_eq!(head, retained_after_advance);

    assert_eq!(head.ingest(&root).unwrap(), MaritimeStreamDisposition::StaleReplay);
    assert_eq!(head, retained_after_advance);

    let mut gap = MaritimeEvidenceEnvelope::new(
        "auv-01",
        7,
        head.sequence() + 2,
        next.observed_at_us + 2_000_000,
        MaritimeEvidenceKind::CommunicationsState,
        r#"{"link":"restored"}"#,
        next.evidence_binding.clone(),
    );
    gap.previous_event_digest = Some(head.digest().to_owned());
    assert_eq!(head.ingest(&gap).unwrap(), MaritimeStreamDisposition::Gap);
    assert_eq!(head, retained_after_advance);

    let mut fork = MaritimeEvidenceEnvelope::new(
        "auv-01",
        7,
        head.sequence() + 1,
        next.observed_at_us + 3_000_000,
        MaritimeEvidenceKind::CommunicationsState,
        r#"{"link":"conflicting"}"#,
        next.evidence_binding.clone(),
    );
    fork.previous_event_digest = Some("00".repeat(32));
    assert_eq!(head.ingest(&fork).unwrap(), MaritimeStreamDisposition::Fork);
    assert_eq!(head, retained_after_advance);

    let mut regression = MaritimeEvidenceEnvelope::new(
        "auv-01",
        6,
        head.sequence() + 1,
        next.observed_at_us + 4_000_000,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"healthy"}"#,
        next.evidence_binding.clone(),
    );
    regression.previous_event_digest = Some(head.digest().to_owned());
    assert_eq!(
        head.ingest(&regression).unwrap(),
        MaritimeStreamDisposition::GenerationRegression
    );
    assert_eq!(head, retained_after_advance);

    let mut wrong_platform = MaritimeEvidenceEnvelope::new(
        "auv-02",
        7,
        head.sequence() + 1,
        next.observed_at_us + 5_000_000,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"healthy"}"#,
        next.evidence_binding.clone(),
    );
    wrong_platform.previous_event_digest = Some(head.digest().to_owned());
    assert_eq!(
        head.ingest(&wrong_platform).unwrap(),
        MaritimeStreamDisposition::WrongPlatform
    );
    assert_eq!(head, retained_after_advance);
}

#[test]
fn restart_preserves_replay_and_generation_rollback_protection() {
    let root = root();
    let next = direct_successor(&root);
    let mut head = MaritimeStreamHead::from_root(&root).unwrap();
    assert_eq!(head.ingest(&next).unwrap(), MaritimeStreamDisposition::Advance);

    let mut restarted = MaritimeStreamHead::from_retained(
        head.platform_id().to_owned(),
        head.generation(),
        head.sequence(),
        head.digest().to_owned(),
    )
    .unwrap();
    let retained = restarted.clone();

    assert_eq!(
        restarted.ingest(&next).unwrap(),
        MaritimeStreamDisposition::Duplicate
    );
    assert_eq!(restarted, retained);
    assert_eq!(
        restarted.ingest(&root).unwrap(),
        MaritimeStreamDisposition::StaleReplay
    );
    assert_eq!(restarted, retained);

    let mut regressed = MaritimeEvidenceEnvelope::new(
        "auv-01",
        6,
        restarted.sequence() + 1,
        next.observed_at_us + 1_000_000,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"healthy"}"#,
        next.evidence_binding,
    );
    regressed.previous_event_digest = Some(restarted.digest().to_owned());
    assert_eq!(
        restarted.ingest(&regressed).unwrap(),
        MaritimeStreamDisposition::GenerationRegression
    );
    assert_eq!(restarted, retained);
}

#[test]
fn terminal_sequence_cannot_wrap_into_a_false_successor() {
    let terminal = MaritimeEvidenceEnvelope::new(
        "auv-01",
        7,
        u64::MAX,
        1_700_000_030_000_000,
        MaritimeEvidenceKind::HealthObservation,
        r#"{"severity":"healthy"}"#,
        "terminal-sequence-evidence",
    );
    let mut head = MaritimeStreamHead::from_root(&terminal).unwrap();
    let retained = head.clone();

    let wrapped = MaritimeEvidenceEnvelope::new(
        "auv-01",
        8,
        0,
        1_700_000_031_000_000,
        MaritimeEvidenceKind::RecoveryEvent,
        r#"{"state":"restarted"}"#,
        "new-lineage-evidence",
    );
    assert_eq!(
        head.ingest(&wrapped).unwrap(),
        MaritimeStreamDisposition::StaleReplay
    );
    assert_eq!(head, retained);
    assert!(wrapped.chain_after(&terminal).is_err());
}
