use hearth_care_completion_authority::assess_completion_authority;
use hearth_care_digest_v4::{build_digest_v4, CareDigestV4Item};
use hearth_care_ledger_contract::{
    CareCompletion, CareOccurrence, CompletionRecord, MemberId, OccurrenceAssignmentBinding,
    OccurrenceAssignmentBindingRecord, OccurrenceWindow, RecordId, ScheduleId,
    CARE_LEDGER_SCHEMA_VERSION,
};
use hearth_care_occurrence_admission::{CurrentAssignmentAuthority, CurrentRecurrenceAuthority};
use hearth_care_occurrence_authority::{assess_occurrence_authority, OccurrenceAuthoritySubject};
use hearth_care_occurrence_time_evidence::{
    CareOccurrenceRecurrenceEvidence, OccurrenceRecurrenceEvidenceRecord,
    OCCURRENCE_TIME_EVIDENCE_SCHEMA_VERSION,
};
use hearth_care_recurrence::{
    CivilResolution, ExpansionEngineEvidence, InstanceDisposition, LocalDate, LocalDateTime,
    LocalTime, TimeZoneId,
};
use hearth_household_planner::{
    AssignmentPolicy, CareOccurrence as PlannerCareOccurrence, PlannerPresence, PlanningPolicy,
    TimeWindow,
};
use hearth_planner_evidence::MemberPlanningFacts;
use hearth_planner_evidence_v4::{
    plan_household_work_v4, CareHistoryEvidenceV4, PlanningInputV4,
    PLANNER_EVIDENCE_V4_SCHEMA_VERSION,
};
use std::collections::BTreeSet;

fn main() {
    assert!(probe());
}

fn probe() -> bool {
    let occurrence = CareOccurrence::new(
        ScheduleId("schedule:1".into()),
        MemberId("alice".into()),
        OccurrenceWindow {
            start_micros: 1_000_000,
            end_micros: 1_801_000_000,
        },
        Some(20),
    )
    .expect("probe occurrence");
    let subject = OccurrenceAuthoritySubject {
        occurrence_ref: "occurrence-entry:1".into(),
        occurrence: occurrence.clone(),
    };

    let assignment_record = OccurrenceAssignmentBindingRecord {
        record_id: RecordId("assignment-binding:1".into()),
        binding: OccurrenceAssignmentBinding::from_occurrence(
            &occurrence,
            "assignment:a1".into(),
        )
        .expect("probe assignment binding"),
    };

    let local = LocalDateTime {
        date: LocalDate {
            year: 2026,
            month: 9,
            day: 21,
        },
        time: LocalTime {
            hour: 18,
            minute: 0,
            second: 0,
        },
    };
    let time_record = OccurrenceRecurrenceEvidenceRecord {
        record_ref: "time-evidence:1".into(),
        evidence: CareOccurrenceRecurrenceEvidence {
            schema_version: OCCURRENCE_TIME_EVIDENCE_SCHEMA_VERSION,
            occurrence_ref: subject.occurrence_ref.clone(),
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: "recurrence:r1".into(),
            instance_key: "instance:1".into(),
            timezone: TimeZoneId("Africa/Johannesburg".into()),
            engine: ExpansionEngineEvidence {
                engine_id: "qualification-probe".into(),
                engine_version: "1".into(),
                tzdb_ref: "iana:2026c".into(),
            },
            original_start_local: local.clone(),
            requested_start_local: local.clone(),
            effective_start_local: local,
            window_start_utc_micros: occurrence.window.start_micros,
            window_end_utc_micros: occurrence.window.end_micros,
            resolution: CivilResolution::Exact,
            disposition: InstanceDisposition::Scheduled,
        },
    };

    let occurrence_authority = assess_occurrence_authority(
        &subject,
        &CurrentAssignmentAuthority {
            schedule_ref: "schedule:1".into(),
            assignment_state_ref: "assignment:a1".into(),
            assignee: "alice".into(),
        },
        &CurrentRecurrenceAuthority {
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: "recurrence:r1".into(),
        },
        &[assignment_record],
        &[time_record],
    )
    .expect("probe occurrence authority");
    if !occurrence_authority.is_qualified() {
        return false;
    }

    let completion = CompletionRecord {
        record_id: RecordId("completion:1".into()),
        completion: CareCompletion {
            schema_version: CARE_LEDGER_SCHEMA_VERSION,
            occurrence_id: occurrence.id.clone(),
            performed_by: MemberId("alice".into()),
            recorded_by: MemberId("alice".into()),
            completed_at_micros: occurrence.window.end_micros + 1,
            actual_minutes: Some(25),
            evidence_refs: vec![],
        },
    };
    let completion_authority = assess_completion_authority(
        &subject,
        &occurrence_authority,
        &[completion],
    )
    .expect("probe completion authority");
    if !completion_authority.is_qualified() {
        return false;
    }

    let digest = build_digest_v4(&[CareDigestV4Item {
        subject: subject.clone(),
        occurrence_authority,
        completion_authority,
    }])
    .expect("probe digest v4");
    let history = CareHistoryEvidenceV4::new("probe:digest-v4".into(), 0, 2_000_000_000, digest)
        .expect("probe history evidence");

    let member = MemberPlanningFacts {
        member_id: "alice".into(),
        display_name: "Alice".into(),
        active: true,
        presence: PlannerPresence::Home,
        availability: vec![TimeWindow {
            start_micros: 0,
            end_micros: 2_000_000_000,
        }],
        max_planned_minutes: 600,
        existing_planned_minutes: 0,
        capabilities: BTreeSet::new(),
    };
    let care = PlannerCareOccurrence {
        occurrence_id: "planner:next".into(),
        source_schedule_ref: "schedule:next".into(),
        title: "Next care task".into(),
        category: "chore".into(),
        current_assignee: None,
        assignment_policy: AssignmentPolicy::Pool,
        estimated_minutes: Some(30),
        window: Some(TimeWindow {
            start_micros: 0,
            end_micros: 2_000_000_000,
        }),
        required_capabilities: BTreeSet::new(),
        eligible_members: None,
        priority: 50,
    };
    let result = plan_household_work_v4(&PlanningInputV4 {
        schema_version: PLANNER_EVIDENCE_V4_SCHEMA_VERSION,
        hearth_id: "hearth:qualification".into(),
        horizon: TimeWindow {
            start_micros: 0,
            end_micros: 2_000_000_000,
        },
        members: vec![member],
        care: vec![care],
        rhythms: vec![],
        policy: PlanningPolicy::default(),
        history,
    })
    .expect("probe planner v4");

    result.history_provenance.qualified_occurrence_count == 1
        && result.history_provenance.qualified_completion_count == 1
        && result
            .plan
            .fairness
            .member_loads
            .iter()
            .any(|load| load.member_id == "alice" && load.verified_recent_care_minutes == 25)
}
