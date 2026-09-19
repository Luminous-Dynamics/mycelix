use hearth_care_ledger_contract::*;

fn occurrence(
    schedule: &str,
    member: &str,
    start: i64,
    end: i64,
    minutes: Option<u32>,
) -> CareOccurrence {
    CareOccurrence::new(
        ScheduleId(schedule.into()),
        MemberId(member.into()),
        OccurrenceWindow {
            start_micros: start,
            end_micros: end,
        },
        minutes,
    )
    .unwrap()
}

fn occ_record(record: &str, occurrence: CareOccurrence) -> OccurrenceRecord {
    OccurrenceRecord {
        record_id: RecordId(record.into()),
        occurrence,
    }
}

fn completion(
    record: &str,
    id: &OccurrenceId,
    performer: &str,
    recorder: &str,
    actual: Option<u32>,
) -> CompletionRecord {
    CompletionRecord {
        record_id: RecordId(record.into()),
        completion: CareCompletion {
            schema_version: CARE_LEDGER_SCHEMA_VERSION,
            occurrence_id: id.clone(),
            performed_by: MemberId(performer.into()),
            recorded_by: MemberId(recorder.into()),
            completed_at_micros: 50,
            actual_minutes: actual,
            evidence_refs: vec!["evidence:1".into()],
        },
    }
}

#[test]
fn occurrence_identity_is_deterministic_and_window_bound() {
    let schedule = ScheduleId("schedule:abc".into());
    let a = derive_occurrence_id(
        &schedule,
        &OccurrenceWindow {
            start_micros: 10,
            end_micros: 20,
        },
    )
    .unwrap();
    let b = derive_occurrence_id(
        &schedule,
        &OccurrenceWindow {
            start_micros: 10,
            end_micros: 20,
        },
    )
    .unwrap();
    let c = derive_occurrence_id(
        &schedule,
        &OccurrenceWindow {
            start_micros: 20,
            end_micros: 30,
        },
    )
    .unwrap();
    assert_eq!(a, b);
    assert_ne!(a, c);
}

#[test]
fn paused_template_cannot_materialize() {
    let proposed = occurrence("s", "m", 10, 20, Some(15));
    assert!(matches!(
        decide_materialization(TemplateState::Paused, proposed, &[]).unwrap(),
        MaterializeDecision::InhibitedTemplateState(TemplateState::Paused)
    ));
}

#[test]
fn identical_duplicate_occurrence_uses_stable_record_tie_break() {
    let item = occurrence("s", "m", 10, 20, Some(15));
    let records = vec![
        occ_record("record:z", item.clone()),
        occ_record("record:a", item.clone()),
    ];
    let canonical = canonicalize_occurrences(&records).unwrap();
    assert_eq!(canonical[&item.id].record_id, RecordId("record:a".into()));
}

#[test]
fn semantic_collision_on_same_occurrence_id_fails_closed() {
    let a = occurrence("s", "m", 10, 20, Some(15));
    let mut b = a.clone();
    b.assigned_to = MemberId("other".into());
    let err = canonicalize_occurrences(&[occ_record("a", a), occ_record("b", b)]).unwrap_err();
    assert!(matches!(err, LedgerError::OccurrenceConflict { .. }));
}

#[test]
fn agreeing_duplicate_completions_do_not_double_count() {
    let item = occurrence("s", "m", 10, 20, Some(30));
    let digest = build_digest_v2(
        &[occ_record("occ:a", item.clone())],
        &[
            completion("complete:z", &item.id, "m", "guardian:a", Some(25)),
            completion("complete:a", &item.id, "m", "guardian:b", Some(25)),
        ],
    )
    .unwrap();
    let stats = &digest.by_member[&MemberId("m".into())];
    assert_eq!(stats.tasks_completed, 1);
    assert_eq!(stats.known_actual_minutes, 25);
    assert_eq!(stats.estimated_minutes_for_completed_tasks, 30);
    assert_eq!(digest.agreeing_duplicate_completion_count, 1);
}

#[test]
fn different_recorders_can_attest_the_same_work_without_conflict() {
    let item = occurrence("s", "child", 10, 20, Some(30));
    let canonical = canonicalize_completions(&[
        completion("record:a", &item.id, "child", "guardian:a", Some(20)),
        completion("record:b", &item.id, "child", "guardian:b", Some(20)),
    ])
    .unwrap();
    assert!(!canonical[&item.id].semantic_conflict);
}

#[test]
fn conflicting_performer_evidence_is_excluded_from_workload() {
    let item = occurrence("s", "m", 10, 20, Some(30));
    let digest = build_digest_v2(
        &[occ_record("occ:a", item.clone())],
        &[
            completion("complete:a", &item.id, "m", "guardian", Some(25)),
            completion("complete:b", &item.id, "other", "guardian", Some(25)),
        ],
    )
    .unwrap();
    assert!(digest.by_member.is_empty());
    assert_eq!(digest.conflicted_completion_count, 1);
}

#[test]
fn recorder_is_not_credited_with_performed_work() {
    let item = occurrence("s", "child", 10, 20, Some(35));
    let digest = build_digest_v2(
        &[occ_record("occ:a", item.clone())],
        &[completion(
            "complete:a",
            &item.id,
            "child",
            "guardian",
            Some(31),
        )],
    )
    .unwrap();
    assert!(digest.by_member.contains_key(&MemberId("child".into())));
    assert!(!digest.by_member.contains_key(&MemberId("guardian".into())));
}

#[test]
fn unknown_actual_duration_stays_unknown_and_estimate_stays_separate() {
    let item = occurrence("s", "m", 10, 20, Some(35));
    let digest = build_digest_v2(
        &[occ_record("occ:a", item.clone())],
        &[completion("complete:a", &item.id, "m", "m", None)],
    )
    .unwrap();
    let stats = &digest.by_member[&MemberId("m".into())];
    assert_eq!(stats.tasks_completed, 1);
    assert_eq!(stats.known_actual_minutes, 0);
    assert_eq!(stats.unknown_actual_duration_count, 1);
    assert_eq!(stats.estimated_minutes_for_completed_tasks, 35);
}

#[test]
fn known_actual_minutes_sum_exactly() {
    let a = occurrence("s:a", "m", 10, 20, Some(30));
    let b = occurrence("s:b", "m", 30, 40, None);
    let digest = build_digest_v2(
        &[
            occ_record("occ:a", a.clone()),
            occ_record("occ:b", b.clone()),
        ],
        &[
            completion("complete:a", &a.id, "m", "m", Some(17)),
            completion("complete:b", &b.id, "m", "m", Some(23)),
        ],
    )
    .unwrap();
    let stats = &digest.by_member[&MemberId("m".into())];
    assert_eq!(stats.tasks_completed, 2);
    assert_eq!(stats.known_actual_minutes, 40);
    assert_eq!(stats.completed_tasks_without_estimate, 1);
}

#[test]
fn legacy_effort_is_labeled_estimated_not_actual() {
    let legacy = import_legacy_care_summary(3, 300);
    assert_eq!(legacy.estimated_minutes, 180);
    assert_eq!(legacy.provenance, EffortProvenance::LegacyOneHourPerTask);
}

#[test]
fn completion_does_not_mutate_template_state() {
    let state = TemplateState::Active;
    let item = occurrence("daily:dog", "m", 10, 20, Some(10));
    let _ = build_digest_v2(
        &[occ_record("occ:a", item.clone())],
        &[completion("complete:a", &item.id, "m", "m", Some(9))],
    )
    .unwrap();
    assert_eq!(state, TemplateState::Active);
}

#[test]
fn orphan_completion_is_reported_not_counted() {
    let missing = OccurrenceId("care-occ-v1|1|s|10|20".into());
    let digest = build_digest_v2(
        &[],
        &[completion("complete:a", &missing, "m", "m", Some(10))],
    )
    .unwrap();
    assert!(digest.by_member.is_empty());
    assert_eq!(digest.orphan_completion_count, 1);
}
