//! Bitemporal execution evidence contract for manufacturing/physical-state handoffs.
//!
//! This module is deliberately claim-bounded. A planned operation or generic Completed lifecycle
//! event cannot prove that a physical placement is clear. A placement-state claim requires an
//! explicit, source-referenced event whose semantic kind, target binding, timebase, recorded-time
//! cutoff and mapping-loss declaration all support the claim.
//!
//! Tick values must already be normalized to the declared common monotonic timebase. This module
//! does not perform clock synchronization, authenticate source systems, or prove that a sensor is
//! truthful; it verifies the semantics of the evidence record presented to it.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PhysicalObservationKind {
    OperationStarted,
    OperationCompleted,
    PlacementClearanceConfirmed,
    PlacementOccupancyObserved,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BitemporalExecutionObservation {
    pub observation_id: String,
    pub operation_id: String,
    /// Required for clearance/presence observations. Generic operation events may leave this None.
    pub placement_reservation_id: Option<String>,
    pub kind: PhysicalObservationKind,
    /// Time the source asserts the event occurred, normalized to timebase_id.
    pub occurred_tick: u64,
    /// Time the event became available to this consumer, normalized to timebase_id.
    pub recorded_tick: u64,
    pub timebase_id: String,
    pub source_id: String,
    pub source_schema_version: String,
    pub mapping_profile_id: String,
    pub mapping_profile_version: u32,
    /// Must be true even when mapping_losses is empty. Unknown loss state is not "no loss".
    pub mapping_loss_assessed: bool,
    /// Explicit loss report from the adapter. Non-empty loss prevents physical-state promotion in V0.
    pub mapping_losses: Vec<String>,
    /// Immutable reference to source evidence; this is not an authentication proof by itself.
    pub evidence_ref: String,
    /// Algorithm-tagged digest/commitment supplied by the source adapter.
    pub payload_commitment: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionDecisionCutoff {
    pub as_of_occurred_tick: u64,
    pub as_of_recorded_tick: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlacementStateDecision {
    /// An explicitly bound physical-clearance event was available by the decision cutoff.
    ClearanceConfirmed,
    /// A positive observation says the same placement remains occupied at the decision tick.
    OccupiedAtDecision,
    /// No admissible evidence proves either current clearance or current occupancy.
    Indeterminate,
    /// Available evidence makes incompatible claims about the same placement state.
    Conflicting,
    /// The input evidence/binding is malformed or cannot be placed on the requested timebase.
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd, Serialize, Deserialize)]
pub enum ExecutionEvidenceViolationCode {
    EmptyObservationId,
    DuplicateObservationId,
    EmptyOperationId,
    EmptyPlacementReservationId,
    EmptyTimebaseId,
    TimebaseMismatch,
    RecordedBeforeOccurrence,
    EmptySourceId,
    EmptySourceSchemaVersion,
    EmptyMappingProfileId,
    InvalidMappingProfileVersion,
    MappingLossNotAssessed,
    EmptyEvidenceReference,
    EmptyPayloadCommitment,
    BindingOperationMismatch,
    DuplicateCurrentOccupancyWitness,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionEvidenceViolation {
    pub code: ExecutionEvidenceViolationCode,
    pub observation_id: String,
    pub detail: String,
}

fn push(
    violations: &mut Vec<ExecutionEvidenceViolation>,
    code: ExecutionEvidenceViolationCode,
    observation_id: &str,
    detail: &'static str,
) {
    violations.push(ExecutionEvidenceViolation {
        code,
        observation_id: observation_id.to_owned(),
        detail: detail.to_owned(),
    });
}

fn sort_violations(violations: &mut Vec<ExecutionEvidenceViolation>) {
    violations.sort_by(|left, right| {
        left.code
            .cmp(&right.code)
            .then_with(|| left.observation_id.cmp(&right.observation_id))
            .then_with(|| left.detail.cmp(&right.detail))
    });
    violations.dedup();
}

fn validate_visible_observation(
    event: &BitemporalExecutionObservation,
    expected_timebase: &str,
    violations: &mut Vec<ExecutionEvidenceViolation>,
) {
    if event.observation_id.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptyObservationId,
            &event.observation_id,
            "observation identity is required",
        );
    }
    if event.operation_id.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptyOperationId,
            &event.observation_id,
            "operation identity is required",
        );
    }
    if matches!(
        event.kind,
        PhysicalObservationKind::PlacementClearanceConfirmed
            | PhysicalObservationKind::PlacementOccupancyObserved
    ) && event
        .placement_reservation_id
        .as_deref()
        .is_none_or(str::is_empty)
    {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptyPlacementReservationId,
            &event.observation_id,
            "physical placement observations require a placement reservation binding",
        );
    }
    if event.timebase_id.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptyTimebaseId,
            &event.observation_id,
            "timebase identity is required",
        );
    } else if event.timebase_id != expected_timebase {
        push(
            violations,
            ExecutionEvidenceViolationCode::TimebaseMismatch,
            &event.observation_id,
            "event and decision cutoff must use the same normalized timebase",
        );
    }
    if event.recorded_tick < event.occurred_tick {
        push(
            violations,
            ExecutionEvidenceViolationCode::RecordedBeforeOccurrence,
            &event.observation_id,
            "normalized recording time cannot precede normalized occurrence time",
        );
    }
    if event.source_id.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptySourceId,
            &event.observation_id,
            "source identity is required",
        );
    }
    if event.source_schema_version.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptySourceSchemaVersion,
            &event.observation_id,
            "source schema version is required",
        );
    }
    if event.mapping_profile_id.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptyMappingProfileId,
            &event.observation_id,
            "mapping profile identity is required",
        );
    }
    if event.mapping_profile_version == 0 {
        push(
            violations,
            ExecutionEvidenceViolationCode::InvalidMappingProfileVersion,
            &event.observation_id,
            "mapping profile version must be positive",
        );
    }
    if !event.mapping_loss_assessed {
        push(
            violations,
            ExecutionEvidenceViolationCode::MappingLossNotAssessed,
            &event.observation_id,
            "adapter must explicitly assess whether the transformation lost meaning",
        );
    }
    if event.evidence_ref.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptyEvidenceReference,
            &event.observation_id,
            "source evidence reference is required",
        );
    }
    if event.payload_commitment.trim().is_empty() {
        push(
            violations,
            ExecutionEvidenceViolationCode::EmptyPayloadCommitment,
            &event.observation_id,
            "payload digest/commitment is required",
        );
    }
}

/// Resolve current physical-placement state using two independent time cutoffs.
///
/// The as_of_occurred_tick cutoff defines which events had occurred in the physical timeline.
/// The as_of_recorded_tick cutoff defines which evidence the consumer had actually received.
/// A generic OperationCompleted event is never promoted to PlacementClearanceConfirmed. A
/// clearance/presence event with any declared mapping loss is preserved in the report but cannot
/// promote a physical claim in V0. Missing evidence remains Indeterminate, not Occupied and not Clear.
pub fn resolve_placement_state(
    placement_reservation_id: &str,
    bound_clearance_operation_id: &str,
    timebase_id: &str,
    cutoff: ExecutionDecisionCutoff,
    observations: &[BitemporalExecutionObservation],
) -> PlacementStateResolution {
    let mut violations = Vec::new();
    if placement_reservation_id.trim().is_empty() {
        push(
            &mut violations,
            ExecutionEvidenceViolationCode::EmptyPlacementReservationId,
            "",
            "requested placement reservation identity is required",
        );
    }
    if bound_clearance_operation_id.trim().is_empty() {
        push(
            &mut violations,
            ExecutionEvidenceViolationCode::EmptyOperationId,
            "",
            "bound clearance operation identity is required",
        );
    }
    if timebase_id.trim().is_empty() {
        push(
            &mut violations,
            ExecutionEvidenceViolationCode::EmptyTimebaseId,
            "",
            "decision timebase identity is required",
        );
    }

    let known_events: Vec<&BitemporalExecutionObservation> = observations
        .iter()
        .filter(|event| event.recorded_tick <= cutoff.as_of_recorded_tick)
        .collect();
    let held_by_recorded_time = observations
        .len()
        .saturating_sub(known_events.len());
    let held_by_occurred_time = known_events
        .iter()
        .filter(|event| event.occurred_tick > cutoff.as_of_occurred_tick)
        .count();

    let mut seen_ids = BTreeSet::new();
    let mut visible_observation_ids = Vec::new();
    let mut mapping_loss_observation_ids = Vec::new();
    let mut clear_seen = false;
    let mut occupied_seen = false;
    let mut occupancy_witness_count = 0usize;

    for event in known_events {
        // A known event on another clock cannot be compared with this cutoff, even to decide
        // whether its occurrence is in the future. Ignore unrelated events; reject related ones.
        let targets_placement =
            event.placement_reservation_id.as_deref() == Some(placement_reservation_id);
        let targets_operation = event.operation_id == bound_clearance_operation_id;
        if !targets_placement && !targets_operation {
            continue;
        }
        if event.timebase_id.trim().is_empty() {
            push(
                &mut violations,
                ExecutionEvidenceViolationCode::EmptyTimebaseId,
                &event.observation_id,
                "timebase identity is required",
            );
            continue;
        }
        if event.timebase_id != timebase_id {
            push(
                &mut violations,
                ExecutionEvidenceViolationCode::TimebaseMismatch,
                &event.observation_id,
                "event and decision cutoff must use the same normalized timebase",
            );
            continue;
        }
        if event.occurred_tick > cutoff.as_of_occurred_tick {
            continue;
        }

        validate_visible_observation(event, timebase_id, &mut violations);
        if !event.observation_id.trim().is_empty()
            && !seen_ids.insert(event.observation_id.as_str())
        {
            push(
                &mut violations,
                ExecutionEvidenceViolationCode::DuplicateObservationId,
                &event.observation_id,
                "visible observation identities must be unique",
            );
        }
        visible_observation_ids.push(event.observation_id.clone());

        let target_placement = event.placement_reservation_id.as_deref();
        if target_placement == Some(placement_reservation_id)
            && matches!(
                event.kind,
                PhysicalObservationKind::PlacementClearanceConfirmed
                    | PhysicalObservationKind::PlacementOccupancyObserved
            )
            && event.operation_id != bound_clearance_operation_id
        {
            push(
                &mut violations,
                ExecutionEvidenceViolationCode::BindingOperationMismatch,
                &event.observation_id,
                "physical placement evidence does not match the explicitly bound operation",
            );
            continue;
        }
        if target_placement != Some(placement_reservation_id) {
            // A generic event for the bound operation is preserved, but cannot prove placement state.
            continue;
        }
        if !event.mapping_losses.is_empty() {
            mapping_loss_observation_ids.push(event.observation_id.clone());
            continue;
        }
        match event.kind {
            PhysicalObservationKind::PlacementClearanceConfirmed => clear_seen = true,
            PhysicalObservationKind::PlacementOccupancyObserved
                if event.operation_id == bound_clearance_operation_id
                    && event.occurred_tick == cutoff.as_of_occurred_tick =>
            {
                occupied_seen = true;
                occupancy_witness_count += 1;
            }
            _ => {}
        }
    }

    if occupancy_witness_count > 1 {
        push(
            &mut violations,
            ExecutionEvidenceViolationCode::DuplicateCurrentOccupancyWitness,
            placement_reservation_id,
            "multiple current occupancy witnesses target one placement in V0",
        );
    }
    visible_observation_ids.sort();
    mapping_loss_observation_ids.sort();
    sort_violations(&mut violations);

    let decision = if !violations.is_empty() {
        PlacementStateDecision::Rejected
    } else if clear_seen && occupied_seen {
        PlacementStateDecision::Conflicting
    } else if clear_seen {
        PlacementStateDecision::ClearanceConfirmed
    } else if occupied_seen && occupancy_witness_count == 1 {
        PlacementStateDecision::OccupiedAtDecision
    } else {
        PlacementStateDecision::Indeterminate
    };

    PlacementStateResolution {
        decision,
        placement_reservation_id: placement_reservation_id.to_owned(),
        bound_clearance_operation_id: bound_clearance_operation_id.to_owned(),
        cutoff,
        visible_observation_ids,
        held_by_recorded_time,
        held_by_occurred_time,
        mapping_loss_observation_ids,
        violations,
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlacementStateResolution {
    pub decision: PlacementStateDecision,
    pub placement_reservation_id: String,
    pub bound_clearance_operation_id: String,
    pub cutoff: ExecutionDecisionCutoff,
    pub visible_observation_ids: Vec<String>,
    pub held_by_recorded_time: usize,
    pub held_by_occurred_time: usize,
    pub mapping_loss_observation_ids: Vec<String>,
    pub violations: Vec<ExecutionEvidenceViolation>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(
        id: &str,
        kind: PhysicalObservationKind,
        occurred: u64,
        recorded: u64,
    ) -> BitemporalExecutionObservation {
        BitemporalExecutionObservation {
            observation_id: id.to_owned(),
            operation_id: "clear-block-a".to_owned(),
            placement_reservation_id: Some("placement-a".to_owned()),
            kind,
            occurred_tick: occurred,
            recorded_tick: recorded,
            timebase_id: "yard-clock-v1".to_owned(),
            source_id: "yard-mes-01".to_owned(),
            source_schema_version: "mes-v3".to_owned(),
            mapping_profile_id: "symtropy-yard-v1".to_owned(),
            mapping_profile_version: 1,
            mapping_loss_assessed: true,
            mapping_losses: Vec::new(),
            evidence_ref: format!("mes://event/{id}"),
            payload_commitment: format!("sha256:{id}"),
        }
    }

    fn cutoff(occurred: u64, recorded: u64) -> ExecutionDecisionCutoff {
        ExecutionDecisionCutoff {
            as_of_occurred_tick: occurred,
            as_of_recorded_tick: recorded,
        }
    }

    #[test]
    fn generic_operation_completion_never_clears_a_placement() {
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[observation("complete", PhysicalObservationKind::OperationCompleted, 5, 5)],
        );
        assert_eq!(result.decision, PlacementStateDecision::Indeterminate);
    }

    #[test]
    fn current_positive_occupancy_witness_can_support_occupied_claim() {
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[observation("scan-8", PhysicalObservationKind::PlacementOccupancyObserved, 8, 8)],
        );
        assert_eq!(result.decision, PlacementStateDecision::OccupiedAtDecision);
        assert_eq!(result.visible_observation_ids, vec!["scan-8"]);
    }

    #[test]
    fn placement_observation_bound_to_another_operation_is_rejected() {
        let mut event = observation(
            "scan-8",
            PhysicalObservationKind::PlacementOccupancyObserved,
            8,
            8,
        );
        event.operation_id = "unrelated-operation".to_owned();
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[event],
        );
        assert_eq!(result.decision, PlacementStateDecision::Rejected);
        assert!(result.violations.iter().any(|v| {
            v.code == ExecutionEvidenceViolationCode::BindingOperationMismatch
        }));
    }

    #[test]
    fn stale_occupancy_witness_does_not_prove_current_occupancy() {
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[observation("scan-7", PhysicalObservationKind::PlacementOccupancyObserved, 7, 7)],
        );
        assert_eq!(result.decision, PlacementStateDecision::Indeterminate);
    }

    #[test]
    fn late_recorded_clearance_is_not_visible_to_an_earlier_decision() {
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[observation("clear-7", PhysicalObservationKind::PlacementClearanceConfirmed, 7, 9)],
        );
        assert_eq!(result.decision, PlacementStateDecision::Indeterminate);
        assert_eq!(result.held_by_recorded_time, 1);
        assert!(result.visible_observation_ids.is_empty());
    }

    #[test]
    fn current_occupancy_and_clearance_claims_conflict() {
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[
                observation("clear-5", PhysicalObservationKind::PlacementClearanceConfirmed, 5, 5),
                observation("scan-8", PhysicalObservationKind::PlacementOccupancyObserved, 8, 8),
            ],
        );
        assert_eq!(result.decision, PlacementStateDecision::Conflicting);
    }

    #[test]
    fn known_mapping_loss_preserves_record_but_blocks_physical_state_promotion() {
        let mut event = observation("clear-5", PhysicalObservationKind::PlacementClearanceConfirmed, 5, 5);
        event.mapping_losses.push("source event lacks placement identity".to_owned());
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[event],
        );
        assert_eq!(result.decision, PlacementStateDecision::Indeterminate);
        assert_eq!(result.mapping_loss_observation_ids, vec!["clear-5"]);
    }

    #[test]
    fn invalid_timebase_or_unassessed_mapping_loss_fails_closed() {
        let mut event = observation("clear-5", PhysicalObservationKind::PlacementClearanceConfirmed, 5, 5);
        event.timebase_id = "foreign-clock".to_owned();
        event.mapping_loss_assessed = false;
        let result = resolve_placement_state(
            "placement-a",
            "clear-block-a",
            "yard-clock-v1",
            cutoff(8, 8),
            &[event],
        );
        assert_eq!(result.decision, PlacementStateDecision::Rejected);
        assert!(result.violations.iter().any(|v| v.code == ExecutionEvidenceViolationCode::TimebaseMismatch));
        assert!(result.violations.iter().any(|v| v.code == ExecutionEvidenceViolationCode::MappingLossNotAssessed));
    }

    #[test]
    fn evidence_order_does_not_change_resolution() {
        let events = [
            observation("clear-5", PhysicalObservationKind::PlacementClearanceConfirmed, 5, 5),
            observation("scan-8", PhysicalObservationKind::PlacementOccupancyObserved, 8, 8),
        ];
        let mut reverse = events.clone();
        reverse.reverse();
        assert_eq!(
            resolve_placement_state("placement-a", "clear-block-a", "yard-clock-v1", cutoff(8, 8), &events),
            resolve_placement_state("placement-a", "clear-block-a", "yard-clock-v1", cutoff(8, 8), &reverse),
        );
    }
}
