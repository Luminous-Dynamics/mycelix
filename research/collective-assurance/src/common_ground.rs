//! Common Ground-style joint-action, referential repair and task-state evidence.
//!
//! This contract validates observable coordination traces. It does not infer
//! private mental states or treat lexical repetition/acknowledgement as proof
//! of shared understanding. Evidence comes from task-state exposure receipts,
//! delivered messages, clarification/repair events and environment actions.

use std::collections::{BTreeMap, BTreeSet};

use super::{QualificationReport, QualificationStatus};

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}
fn fail(report: &mut QualificationReport, reason: impl Into<String>) {
    report.fail(reason);
}
fn indeterminate(report: &mut QualificationReport, reason: impl Into<String>) {
    report.indeterminate(reason);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommonGroundViewMode {
    SharedWorkerState,
    PrivateWorkerState,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RequiredPlacement {
    pub object_id: String,
    pub target_slot_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CommonGroundTrialContract {
    pub trial_id: String,
    pub benchmark_version: String,
    pub task_digest: String,
    pub oracle_spec_digest: String,
    pub expected_final_state_digest: String,
    pub oracle_attestation_digest: String,
    pub helper_id: String,
    pub worker_id: String,
    pub worker_state_digest: String,
    pub worker_state_version: u64,
    pub view_mode: CommonGroundViewMode,
    pub required_placements: Vec<RequiredPlacement>,
    pub time_budget_ms: u64,
    pub turn_budget: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CommonGroundEventKind {
    TaskStateVisible {
        viewer_id: String,
        state_owner_id: String,
        state_digest: String,
        state_version: u64,
        exposure_receipt_digest: String,
    },
    MessageDelivered {
        message_id: String,
        sender_id: String,
        recipient_id: String,
        payload_digest: String,
        delivery_receipt_digest: String,
    },
    ClarificationRequested {
        request_id: String,
        requester_id: String,
        message_id: String,
        ambiguous_referent_digest: String,
    },
    ClarificationResolved {
        request_id: String,
        resolver_id: String,
        resolved_referent_id: String,
        object_id: String,
        rationale_digest: String,
    },
    ReferentConfirmed {
        request_id: String,
        confirmer_id: String,
    },
    JointActionExecuted {
        action_id: String,
        initiator_id: String,
        confirmer_id: String,
        object_id: String,
        target_slot_id: String,
        clarification_request_id: Option<String>,
        environment_accepted: bool,
        effect_receipt_digest: String,
    },
    RepairStarted {
        repair_id: String,
        actor_id: String,
        trigger_event_id: String,
    },
    RepairCompleted {
        repair_id: String,
        outcome_digest: String,
        repair_duration_ms: u64,
    },
    VerbalAcknowledgement {
        actor_id: String,
        message_id: String,
    },
    TrialCompleted {
        final_state_digest: String,
        environment_completion_receipt_digest: String,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CommonGroundEvent {
    pub sequence: u64,
    pub event_id: String,
    pub timestamp_ms: u64,
    pub kind: CommonGroundEventKind,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommonGroundOutcomeStatus {
    Success,
    Incomplete,
    Incorrect,
    NotEvaluable,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CommonGroundTrialReport {
    pub qualification: QualificationReport,
    pub outcome: CommonGroundOutcomeStatus,
    pub joint_success: Option<bool>,
    pub exact_placements: usize,
    pub required_placements: usize,
    pub incorrect_or_rejected_actions: usize,
    pub duplicate_slot_actions: usize,
    pub duplicate_object_actions: usize,
    pub clarification_requests: usize,
    pub resolved_clarifications: usize,
    pub confirmed_referents: usize,
    pub verbal_acknowledgements: usize,
    pub repair_count: usize,
    pub total_repair_duration_ms: u64,
    pub repair_durations_ms: Vec<u64>,
    pub message_count: usize,
    pub task_state_view_count: usize,
    pub shared_worker_state_exposed_to_helper: bool,
    pub elapsed_ms: Option<u64>,
}

fn empty_report(qualification: QualificationReport) -> CommonGroundTrialReport {
    CommonGroundTrialReport {
        qualification,
        outcome: CommonGroundOutcomeStatus::NotEvaluable,
        joint_success: None,
        exact_placements: 0,
        required_placements: 0,
        incorrect_or_rejected_actions: 0,
        duplicate_slot_actions: 0,
        duplicate_object_actions: 0,
        clarification_requests: 0,
        resolved_clarifications: 0,
        confirmed_referents: 0,
        verbal_acknowledgements: 0,
        repair_count: 0,
        total_repair_duration_ms: 0,
        repair_durations_ms: Vec::new(),
        message_count: 0,
        task_state_view_count: 0,
        shared_worker_state_exposed_to_helper: false,
        elapsed_ms: None,
    }
}
fn valid_role(contract: &CommonGroundTrialContract, actor: &str) -> bool {
    actor == contract.helper_id || actor == contract.worker_id
}

/// Qualifies one observable Helper/Worker collaboration trace.
///
/// A qualification PASS means the event trace is structurally complete and
/// bound to the frozen task/oracle/view condition. Joint success is reported
/// separately from trace qualification, so unsuccessful attempts remain data.
pub fn qualify_common_ground_trial(
    contract: &CommonGroundTrialContract,
    events: &[CommonGroundEvent],
) -> CommonGroundTrialReport {
    let mut qualification = QualificationReport {
        status: QualificationStatus::Pass,
        reasons: Vec::new(),
    };
    if contract.trial_id.trim().is_empty()
        || contract.benchmark_version.trim().is_empty()
        || contract.helper_id.trim().is_empty()
        || contract.worker_id.trim().is_empty()
        || contract.helper_id == contract.worker_id
    {
        fail(&mut qualification, "trial, benchmark and distinct helper/worker identities are required");
    }
    for (label, digest) in [
        ("task", contract.task_digest.as_str()),
        ("oracle specification", contract.oracle_spec_digest.as_str()),
        ("expected final state", contract.expected_final_state_digest.as_str()),
        ("oracle attestation", contract.oracle_attestation_digest.as_str()),
        ("worker state", contract.worker_state_digest.as_str()),
    ] {
        if !valid_digest(digest) {
            fail(&mut qualification, format!("{label} digest is malformed"));
        }
    }
    if contract.time_budget_ms == 0 || contract.turn_budget == 0 {
        fail(&mut qualification, "time and turn budgets must be non-zero");
    }
    if contract.required_placements.is_empty() {
        fail(&mut qualification, "task oracle must contain at least one required placement");
    }

    let mut expected_by_slot = BTreeMap::new();
    let mut expected_objects = BTreeSet::new();
    for placement in &contract.required_placements {
        if placement.object_id.trim().is_empty()
            || placement.target_slot_id.trim().is_empty()
            || expected_by_slot
                .insert(placement.target_slot_id.as_str(), placement.object_id.as_str())
                .is_some()
            || !expected_objects.insert(placement.object_id.as_str())
        {
            fail(&mut qualification, "required placements must use unique non-empty object and slot identities");
        }
    }
    if events.is_empty() {
        indeterminate(&mut qualification, "trial event trace is empty");
    }
    if qualification.status == QualificationStatus::Fail {
        return empty_report(qualification);
    }

    let mut seen_event_ids = BTreeSet::new();
    let mut message_deliveries: BTreeMap<&str, (&str, &str)> = BTreeMap::new();
    let mut clarification_requests: BTreeMap<&str, (&str, &str)> = BTreeMap::new();
    let mut clarification_resolutions: BTreeMap<&str, (&str, &str)> = BTreeMap::new();
    let mut confirmed_requests: BTreeMap<&str, &str> = BTreeMap::new();
    let mut repair_starts: BTreeMap<&str, u64> = BTreeMap::new();
    let mut repair_durations = Vec::new();
    let mut state_views: BTreeMap<&str, (u64, u64)> = BTreeMap::new();
    let mut first_message_timestamp: Option<u64> = None;
    let mut action_ids = BTreeSet::new();
    let mut actions = Vec::new();
    let mut completion: Option<(&str, u64)> = None;
    let mut previous_timestamp = 0u64;
    let mut request_count = 0usize;
    let mut resolution_count = 0usize;
    let mut confirmation_count = 0usize;
    let mut acknowledgements = 0usize;
    let mut message_count = 0usize;
    let mut repair_total = 0u64;
    let mut repair_count = 0usize;

    for (index, event) in events.iter().enumerate() {
        if event.sequence != index as u64 + 1 {
            fail(&mut qualification, "event sequence is not contiguous from one");
        }
        if event.event_id.trim().is_empty() || !seen_event_ids.insert(event.event_id.as_str()) {
            fail(&mut qualification, "event IDs must be non-empty and unique");
        }
        if index > 0 && event.timestamp_ms < previous_timestamp {
            fail(&mut qualification, "event timestamps are not monotonic");
        }
        if event.timestamp_ms > contract.time_budget_ms {
            fail(&mut qualification, "event exceeds the frozen trial time budget");
        }
        previous_timestamp = event.timestamp_ms;

        match &event.kind {
            CommonGroundEventKind::TaskStateVisible {
                viewer_id,
                state_owner_id,
                state_digest,
                state_version,
                exposure_receipt_digest,
            } => {
                if !valid_role(contract, viewer_id)
                    || state_owner_id != &contract.worker_id
                    || state_digest != &contract.worker_state_digest
                    || *state_version != contract.worker_state_version
                    || !valid_digest(exposure_receipt_digest)
                    || state_views.insert(viewer_id, (*state_version, event.timestamp_ms)).is_some()
                {
                    fail(&mut qualification, "task-state view is invalid, duplicated or lacks exposure evidence");
                }
            }
            CommonGroundEventKind::MessageDelivered {
                message_id,
                sender_id,
                recipient_id,
                payload_digest,
                delivery_receipt_digest,
            } => {
                if message_id.trim().is_empty()
                    || !valid_role(contract, sender_id)
                    || !valid_role(contract, recipient_id)
                    || sender_id == recipient_id
                    || !valid_digest(payload_digest)
                    || !valid_digest(delivery_receipt_digest)
                    || message_deliveries.insert(message_id, (sender_id, recipient_id)).is_some()
                {
                    fail(&mut qualification, "message delivery is invalid, duplicated or lacks evidence");
                } else {
                    first_message_timestamp.get_or_insert(event.timestamp_ms);
                    message_count += 1;
                }
            }
            CommonGroundEventKind::ClarificationRequested {
                request_id,
                requester_id,
                message_id,
                ambiguous_referent_digest,
            } => {
                if request_id.trim().is_empty()
                    || !valid_role(contract, requester_id)
                    || !valid_digest(ambiguous_referent_digest)
                    || clarification_requests.contains_key(request_id.as_str())
                {
                    fail(&mut qualification, "clarification request identity or referent evidence is invalid");
                    continue;
                }
                match message_deliveries.get(message_id.as_str()).copied() {
                    Some((sender, recipient)) if sender == requester_id.as_str() && recipient != requester_id.as_str() => {
                        clarification_requests.insert(request_id, (requester_id, message_id));
                        request_count += 1;
                    }
                    _ => fail(&mut qualification, "clarification must bind to a message actually delivered by the requester"),
                }
            }
            CommonGroundEventKind::ClarificationResolved {
                request_id,
                resolver_id,
                resolved_referent_id,
                object_id,
                rationale_digest,
            } => {
                let Some((requester, message_id)) = clarification_requests.get(request_id.as_str()).copied() else {
                    fail(&mut qualification, "clarification resolution has no prior request");
                    continue;
                };
                let recipient = message_deliveries.get(message_id).map(|x| x.1);
                if !valid_role(contract, resolver_id)
                    || resolver_id.as_str() == requester
                    || recipient != Some(resolver_id.as_str())
                    || resolved_referent_id.trim().is_empty()
                    || object_id.trim().is_empty()
                    || !valid_digest(rationale_digest)
                    || clarification_resolutions.contains_key(request_id.as_str())
                {
                    fail(&mut qualification, "resolver does not match delivered message or resolution evidence");
                } else {
                    clarification_resolutions.insert(request_id, (resolver_id, object_id));
                    resolution_count += 1;
                }
            }
            CommonGroundEventKind::ReferentConfirmed { request_id, confirmer_id } => {
                let Some((resolver, _)) = clarification_resolutions.get(request_id.as_str()).copied() else {
                    fail(&mut qualification, "referent confirmation has no prior resolution");
                    continue;
                };
                if !valid_role(contract, confirmer_id)
                    || confirmer_id.as_str() == resolver
                    || confirmed_requests.insert(request_id, confirmer_id).is_some()
                {
                    fail(&mut qualification, "referent confirmation must be unique and come from the other role");
                } else {
                    confirmation_count += 1;
                }
            }
            CommonGroundEventKind::JointActionExecuted {
                action_id,
                initiator_id,
                confirmer_id,
                object_id,
                target_slot_id,
                clarification_request_id,
                environment_accepted,
                effect_receipt_digest,
            } => {
                if action_id.trim().is_empty()
                    || !action_ids.insert(action_id.as_str())
                    || !valid_role(contract, initiator_id)
                    || !valid_role(contract, confirmer_id)
                    || initiator_id == confirmer_id
                    || object_id.trim().is_empty()
                    || target_slot_id.trim().is_empty()
                    || !valid_digest(effect_receipt_digest)
                {
                    fail(&mut qualification, "joint action identity, roles or effect receipt is invalid");
                    continue;
                }
                if let Some(request_id) = clarification_request_id {
                    match (
                        clarification_resolutions.get(request_id.as_str()).copied(),
                        confirmed_requests.get(request_id.as_str()).copied(),
                    ) {
                        (Some((resolver, resolved_object)), Some(confirmed_by))
                            if confirmed_by != resolver
                                && (initiator_id.as_str() == resolver || confirmer_id.as_str() == resolver)
                                && (initiator_id.as_str() == confirmed_by || confirmer_id.as_str() == confirmed_by)
                                && resolved_object == object_id.as_str() => {}
                        _ => fail(&mut qualification, "action uses an ambiguous referent without resolved, cross-role-confirmed grounding"),
                    }
                }
                actions.push((object_id.as_str(), target_slot_id.as_str(), *environment_accepted));
            }
            CommonGroundEventKind::RepairStarted { repair_id, actor_id, trigger_event_id } => {
                if repair_id.trim().is_empty()
                    || !valid_role(contract, actor_id)
                    || !seen_event_ids.contains(trigger_event_id.as_str())
                    || repair_starts.insert(repair_id, event.timestamp_ms).is_some()
                {
                    fail(&mut qualification, "repair start is invalid or references an unknown trigger");
                } else {
                    repair_count += 1;
                }
            }
            CommonGroundEventKind::RepairCompleted { repair_id, outcome_digest, repair_duration_ms } => {
                if !valid_digest(outcome_digest) {
                    fail(&mut qualification, "repair completion digest is malformed");
                    continue;
                }
                let Some(started) = repair_starts.get(repair_id.as_str()).copied() else {
                    fail(&mut qualification, "repair completion has no prior start");
                    continue;
                };
                if event.timestamp_ms < started || event.timestamp_ms - started != *repair_duration_ms {
                    fail(&mut qualification, "repair duration differs from observed timestamps");
                } else {
                    repair_total = repair_total.saturating_add(*repair_duration_ms);
                    repair_durations.push(*repair_duration_ms);
                }
            }
            CommonGroundEventKind::VerbalAcknowledgement { actor_id, message_id } => {
                if !valid_role(contract, actor_id) || !message_deliveries.contains_key(message_id.as_str()) {
                    fail(&mut qualification, "acknowledgement does not bind to a delivered message");
                } else {
                    acknowledgements += 1;
                }
            }
            CommonGroundEventKind::TrialCompleted { final_state_digest, environment_completion_receipt_digest } => {
                if completion.is_some()
                    || !valid_digest(final_state_digest)
                    || !valid_digest(environment_completion_receipt_digest)
                    || event.sequence as usize != events.len()
                {
                    fail(&mut qualification, "completion must be unique, valid and the final event");
                } else {
                    completion = Some((final_state_digest, event.timestamp_ms));
                }
            }
        }
    }

    if repair_starts.len() != repair_durations.len() {
        indeterminate(&mut qualification, "one or more repair events have no completion receipt");
    }

    let expected_viewers: BTreeSet<&str> = match contract.view_mode {
        CommonGroundViewMode::SharedWorkerState => [contract.helper_id.as_str(), contract.worker_id.as_str()].into_iter().collect(),
        CommonGroundViewMode::PrivateWorkerState => [contract.worker_id.as_str()].into_iter().collect(),
    };
    let actual_viewers: BTreeSet<&str> = state_views
        .iter()
        .filter(|entry| entry.1.0 == contract.worker_state_version)
        .map(|entry| *entry.0)
        .collect();
    if let Some(first_message) = first_message_timestamp {
        if state_views.values().any(|(_, timestamp)| *timestamp >= first_message) {
            indeterminate(
                &mut qualification,
                "task-state view receipt arrived after the first delivered dialogue message",
            );
        }
    }
    if actual_viewers != expected_viewers {
        indeterminate(&mut qualification, "task-state exposure does not match the preregistered view condition");
    }
    let shared_worker_state_exposed_to_helper = actual_viewers.contains(contract.helper_id.as_str());

    let Some((observed_final_state, elapsed_ms)) = completion else {
        indeterminate(&mut qualification, "independent environment completion receipt is missing");
        return empty_report(qualification);
    };
    if qualification.status != QualificationStatus::Pass {
        return empty_report(qualification);
    }

    let mut placed_slots = BTreeMap::new();
    let mut placed_objects = BTreeSet::new();
    let mut exact_placements = 0usize;
    let mut incorrect_or_rejected = 0usize;
    let mut duplicate_slots = 0usize;
    let mut duplicate_objects = 0usize;
    for (object, slot, accepted) in &actions {
        if !accepted {
            incorrect_or_rejected += 1;
            continue;
        }
        if placed_slots.insert(*slot, *object).is_some() {
            duplicate_slots += 1;
        }
        if !placed_objects.insert(*object) {
            duplicate_objects += 1;
        }
        if expected_by_slot.get(*slot).copied().is_some_and(|expected| expected == *object) {
            exact_placements += 1;
        } else {
            incorrect_or_rejected += 1;
        }
    }
    let all_placements_correct = exact_placements == contract.required_placements.len()
        && incorrect_or_rejected == 0
        && duplicate_slots == 0
        && duplicate_objects == 0
        && placed_slots.len() == contract.required_placements.len();
    let joint_success = observed_final_state == contract.expected_final_state_digest && all_placements_correct;
    let outcome = if joint_success {
        CommonGroundOutcomeStatus::Success
    } else if actions.is_empty() {
        CommonGroundOutcomeStatus::Incomplete
    } else {
        CommonGroundOutcomeStatus::Incorrect
    };

    CommonGroundTrialReport {
        qualification,
        outcome,
        joint_success: Some(joint_success),
        exact_placements,
        required_placements: contract.required_placements.len(),
        incorrect_or_rejected_actions: incorrect_or_rejected,
        duplicate_slot_actions: duplicate_slots,
        duplicate_object_actions: duplicate_objects,
        clarification_requests: request_count,
        resolved_clarifications: resolution_count,
        confirmed_referents: confirmation_count,
        verbal_acknowledgements: acknowledgements,
        repair_count,
        total_repair_duration_ms: repair_total,
        repair_durations_ms: repair_durations,
        message_count,
        task_state_view_count: actual_viewers.len(),
        shared_worker_state_exposed_to_helper,
        elapsed_ms: Some(elapsed_ms),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String { std::iter::repeat_n(ch, 64).collect() }
    fn contract(view_mode: CommonGroundViewMode) -> CommonGroundTrialContract {
        CommonGroundTrialContract {
            trial_id: "trial-1".into(),
            benchmark_version: "common-ground-adapter-v0.1".into(),
            task_digest: digest('a'),
            oracle_spec_digest: digest('b'),
            expected_final_state_digest: digest('c'),
            oracle_attestation_digest: digest('d'),
            helper_id: "helper".into(),
            worker_id: "worker".into(),
            worker_state_digest: digest('e'),
            worker_state_version: 3,
            view_mode,
            required_placements: vec![
                RequiredPlacement { object_id: "piece-1".into(), target_slot_id: "slot-1".into() },
                RequiredPlacement { object_id: "piece-2".into(), target_slot_id: "slot-2".into() },
            ],
            time_budget_ms: 10_000,
            turn_budget: 20,
        }
    }
    fn ev(sequence: u64, id: &str, timestamp_ms: u64, kind: CommonGroundEventKind) -> CommonGroundEvent {
        CommonGroundEvent { sequence, event_id: id.into(), timestamp_ms, kind }
    }
    fn success_trace(mode: CommonGroundViewMode) -> Vec<CommonGroundEvent> {
        let c = contract(mode);
        let mut v = vec![ev(1,"vw",10,CommonGroundEventKind::TaskStateVisible {
            viewer_id:"worker".into(),state_owner_id:"worker".into(),state_digest:c.worker_state_digest.clone(),
            state_version:c.worker_state_version,exposure_receipt_digest:digest('f')
        })];
        if mode == CommonGroundViewMode::SharedWorkerState {
            v.push(ev(2,"vh",12,CommonGroundEventKind::TaskStateVisible {
                viewer_id:"helper".into(),state_owner_id:"worker".into(),state_digest:c.worker_state_digest.clone(),
                state_version:c.worker_state_version,exposure_receipt_digest:digest('1')
            }));
        }
        let n=v.len() as u64+1;
        v.push(ev(n,"msg",100,CommonGroundEventKind::MessageDelivered {
            message_id:"msg-1".into(),sender_id:"worker".into(),recipient_id:"helper".into(),
            payload_digest:digest('2'),delivery_receipt_digest:digest('3')
        }));
        let n=v.len() as u64+1;
        v.push(ev(n,"req",120,CommonGroundEventKind::ClarificationRequested {
            request_id:"clar-1".into(),requester_id:"worker".into(),message_id:"msg-1".into(),
            ambiguous_referent_digest:digest('4')
        }));
        let n=v.len() as u64+1;
        v.push(ev(n,"res",180,CommonGroundEventKind::ClarificationResolved {
            request_id:"clar-1".into(),resolver_id:"helper".into(),resolved_referent_id:"blue-piece".into(),
            object_id:"piece-1".into(),rationale_digest:digest('5')
        }));
        let n=v.len() as u64+1;
        v.push(ev(n,"confirm",200,CommonGroundEventKind::ReferentConfirmed {
            request_id:"clar-1".into(),confirmer_id:"worker".into()
        }));
        let n=v.len() as u64+1;
        v.push(ev(n,"act1",300,CommonGroundEventKind::JointActionExecuted {
            action_id:"place1".into(),initiator_id:"worker".into(),confirmer_id:"helper".into(),
            object_id:"piece-1".into(),target_slot_id:"slot-1".into(),clarification_request_id:Some("clar-1".into()),
            environment_accepted:true,effect_receipt_digest:digest('6')
        }));
        let n=v.len() as u64+1;
        v.push(ev(n,"act2",400,CommonGroundEventKind::JointActionExecuted {
            action_id:"place2".into(),initiator_id:"helper".into(),confirmer_id:"worker".into(),
            object_id:"piece-2".into(),target_slot_id:"slot-2".into(),clarification_request_id:None,
            environment_accepted:true,effect_receipt_digest:digest('7')
        }));
        let n=v.len() as u64+1;
        v.push(ev(n,"ack",450,CommonGroundEventKind::VerbalAcknowledgement {
            actor_id:"worker".into(),message_id:"msg-1".into()
        }));
        let n=v.len() as u64+1;
        v.push(ev(n,"done",600,CommonGroundEventKind::TrialCompleted {
            final_state_digest:c.expected_final_state_digest,environment_completion_receipt_digest:digest('8')
        }));
        v
    }

    #[test]
    fn shared_view_joint_action_success_passes() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let r=qualify_common_ground_trial(&c,&success_trace(c.view_mode));
        assert_eq!(r.qualification.status,QualificationStatus::Pass);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::Success);
        assert_eq!(r.exact_placements,2);
        assert!(r.shared_worker_state_exposed_to_helper);
    }
    #[test]
    fn private_view_condition_is_measured_from_receipts() {
        let c=contract(CommonGroundViewMode::PrivateWorkerState);
        let r=qualify_common_ground_trial(&c,&success_trace(c.view_mode));
        assert_eq!(r.qualification.status,QualificationStatus::Pass);
        assert!(!r.shared_worker_state_exposed_to_helper);
    }
    #[test]
    fn acknowledgements_alone_never_establish_common_ground() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        v.retain(|e| !matches!(&e.kind,CommonGroundEventKind::JointActionExecuted{..}));
        v.retain(|e| !matches!(&e.kind,CommonGroundEventKind::ClarificationResolved{..}));
        v.retain(|e| !matches!(&e.kind,CommonGroundEventKind::ReferentConfirmed{..}));
        for (i,e) in v.iter_mut().enumerate(){e.sequence=i as u64+1;}
        if let Some(e)=v.last_mut(){e.kind=CommonGroundEventKind::TrialCompleted{final_state_digest:digest('9'),environment_completion_receipt_digest:digest('8')};}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::Incomplete);
        assert_eq!(r.verbal_acknowledgements,1);
        assert_eq!(r.joint_success,Some(false));
    }
    #[test]
    fn ambiguous_referent_requires_resolution_and_confirmation() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        v.retain(|e| !matches!(&e.kind,CommonGroundEventKind::ReferentConfirmed{..}));
        for (i,e) in v.iter_mut().enumerate(){e.sequence=i as u64+1;}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }
    #[test]
    fn clarification_must_reference_delivered_message() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::ClarificationRequested{message_id,..}=&mut v.iter_mut().find(|e|e.event_id=="req").unwrap().kind{*message_id="missing".into();}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }
    #[test]
    fn resolver_must_be_message_recipient() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::ClarificationResolved{resolver_id,..}=&mut v.iter_mut().find(|e|e.event_id=="res").unwrap().kind{*resolver_id="worker".into();}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }
    #[test]
    fn wrong_action_is_observed_but_not_successful() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::JointActionExecuted{object_id,..}=&mut v.iter_mut().find(|e|e.event_id=="act1").unwrap().kind{*object_id="wrong".into();}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Pass);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::Incorrect);
    }
    #[test]
    fn duplicate_slots_prevent_success() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::JointActionExecuted{target_slot_id,..}=&mut v.iter_mut().find(|e|e.event_id=="act2").unwrap().kind{*target_slot_id="slot-1".into();}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.duplicate_slot_actions,1);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::Incorrect);
    }
    #[test]
    fn final_environment_state_must_match_oracle() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::TrialCompleted{final_state_digest,..}=&mut v.last_mut().unwrap().kind{*final_state_digest=digest('9');}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Pass);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::Incorrect);
    }
    #[test]
    fn missing_shared_state_exposure_is_indeterminate() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        v.retain(|e|e.event_id!="vh");
        for (i,e) in v.iter_mut().enumerate(){e.sequence=i as u64+1;}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Indeterminate);
    }

    #[test]
    fn task_state_view_must_precede_first_dialogue_message() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        let helper_view=v.iter_mut().find(|e|e.event_id=="view-helper").unwrap();
        helper_view.timestamp_ms=150;
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Indeterminate);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::NotEvaluable);
    }

    #[test]
    fn private_view_rejects_helper_exposure() {
        let c=contract(CommonGroundViewMode::PrivateWorkerState);
        let mut v=success_trace(c.view_mode);
        v.insert(1,ev(2,"vh",11,CommonGroundEventKind::TaskStateVisible {
            viewer_id:"helper".into(),state_owner_id:"worker".into(),state_digest:c.worker_state_digest.clone(),
            state_version:c.worker_state_version,exposure_receipt_digest:digest('1')
        }));
        for (i,e) in v.iter_mut().enumerate(){e.sequence=i as u64+1;}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Indeterminate);
    }
    #[test]
    fn missing_completion_is_not_evaluable() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode); v.pop();
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Indeterminate);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::NotEvaluable);
    }
    #[test]
    fn sequence_gap_fails_trace() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode); v[2].sequence=99;
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }
    #[test]
    fn repair_duration_uses_observed_timestamps() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        let i=v.len()-1;
        v.insert(i,ev(i as u64+1,"repair-start",250,CommonGroundEventKind::RepairStarted {
            repair_id:"repair-1".into(),actor_id:"helper".into(),trigger_event_id:"res".into()
        }));
        v.insert(i+1,ev(i as u64+2,"repair-done",280,CommonGroundEventKind::RepairCompleted {
            repair_id:"repair-1".into(),outcome_digest:digest('9'),repair_duration_ms:30
        }));
        for (n,e) in v.iter_mut().enumerate(){e.sequence=n as u64+1;}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Pass);
        assert_eq!(r.total_repair_duration_ms,30);
    }
    #[test]
    fn missing_repair_completion_is_indeterminate() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        let i=v.len()-1;
        v.insert(i,ev(i as u64+1,"repair-start",250,CommonGroundEventKind::RepairStarted {
            repair_id:"repair-1".into(),actor_id:"helper".into(),trigger_event_id:"res".into()
        }));
        for (n,e) in v.iter_mut().enumerate(){e.sequence=n as u64+1;}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Indeterminate);
    }
    #[test]
    fn time_budget_is_enforced() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        v.last_mut().unwrap().timestamp_ms=c.time_budget_ms+1;
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }
    #[test]
    fn joint_action_requires_two_roles() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::JointActionExecuted{initiator_id,confirmer_id,..}=&mut v.iter_mut().find(|e|e.event_id=="act1").unwrap().kind{*confirmer_id=initiator_id.clone();}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }

    #[test]
    fn delivered_turn_budget_is_enforced() {
        let mut c=contract(CommonGroundViewMode::SharedWorkerState);
        c.turn_budget=1;
        let r=qualify_common_ground_trial(&c,&success_trace(c.view_mode));
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::NotEvaluable);
    }

    #[test]
    fn joint_action_must_include_both_resolver_and_confirming_role() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::JointActionExecuted{initiator_id,confirmer_id,..}=
            &mut v.iter_mut().find(|e|e.event_id=="act1").unwrap().kind {
            *initiator_id="helper".into();
            *confirmer_id="worker".into();
        }
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Pass);
        assert_eq!(r.outcome,CommonGroundOutcomeStatus::Success);
    }

    #[test]
    fn repair_cannot_reference_itself_as_trigger() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        let i=v.len()-1;
        v.insert(i,ev(i as u64+1,"repair-start",250,CommonGroundEventKind::RepairStarted {
            repair_id:"repair-1".into(),actor_id:"helper".into(),trigger_event_id:"repair-start".into()
        }));
        v.insert(i+1,ev(i as u64+2,"repair-done",280,CommonGroundEventKind::RepairCompleted {
            repair_id:"repair-1".into(),outcome_digest:digest('9'),repair_duration_ms:30
        }));
        for (n,e) in v.iter_mut().enumerate(){e.sequence=n as u64+1;}
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }


    #[test]
    fn duplicate_action_ids_fail_even_when_event_ids_are_distinct() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode);
        if let CommonGroundEventKind::JointActionExecuted{action_id,..}=
            &mut v.iter_mut().find(|e|e.event_id=="act2").unwrap().kind {
            *action_id="place1".into();
        }
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }

    #[test]
    fn duplicate_event_ids_fail() {
        let c=contract(CommonGroundViewMode::SharedWorkerState);
        let mut v=success_trace(c.view_mode); v[1].event_id=v[0].event_id.clone();
        let r=qualify_common_ground_trial(&c,&v);
        assert_eq!(r.qualification.status,QualificationStatus::Fail);
    }
}
