//! Deterministic, bounded research simulator for airspace-coordination semantics.
//!
//! This crate is not operational ATM/UTM software. It contains no real-world
//! separation minima, authorisation path, sensor ingestion, or aircraft control.
//! Every result is scoped to a synthetic fixture and the simplified trajectory
//! model documented in README.md.

use serde::{Deserialize, Serialize};

const MAX_HORIZON_S: i64 = 7 * 24 * 60 * 60;
const MAX_SAMPLE_STEP_S: i64 = 3_600;
const MAX_ABS_POSITION_MM: i64 = 1_000_000_000_000_000;
const MAX_ABS_VELOCITY_MM_S: i64 = 1_000_000_000;
const MAX_THRESHOLD_MM: u64 = 1_000_000_000_000_000;
const MAX_PAIR_SAMPLE_CHECKS: u128 = 5_000_000;

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ScenarioSet {
    pub schema: String,
    pub status: String,
    pub authority_ceiling: String,
    pub scenarios: Vec<Scenario>,
    pub lifecycle_transition_cases: Vec<LifecycleTransitionCase>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Scenario {
    pub scenario_id: String,
    pub description: String,
    pub window_start_s: i64,
    pub window_end_s: i64,
    pub policy: SeparationPolicy,
    pub flights: Vec<FlightIntent>,
    pub expected_classification: Classification,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct SeparationPolicy {
    /// Synthetic fixture threshold only; never a real-world separation minimum.
    pub horizontal_separation_mm: u64,
    /// Synthetic fixture threshold only; never a real-world separation minimum.
    pub vertical_separation_mm: u64,
    /// Samples are separated by at most this many whole seconds.
    pub sample_step_s: i64,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct FlightIntent {
    pub flight_id: String,
    pub state: IntentState,
    pub reference_frame: String,
    pub vertical_datum: String,
    /// Identifier for the common synthetic time epoch used by this fixture.
    pub time_basis: String,
    pub time_qualified: bool,
    pub valid_from_s: i64,
    pub valid_until_s: i64,
    /// Position at valid_from_s, in millimetres within a synthetic local frame.
    pub position_at_valid_from_mm: PositionMm,
    /// Constant velocity in millimetres per second. This is a deliberately
    /// simplified kinematic model, not a flight-dynamics model.
    pub velocity_mm_s: VelocityMmS,
    pub horizontal_uncertainty_mm: u64,
    pub vertical_uncertainty_mm: u64,
    pub priority: FlightPriority,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct PositionMm {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct VelocityMmS {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum IntentState {
    Proposed,
    Accepted,
    Activated,
    Withdrawn,
    Ended,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum FlightPriority {
    Routine,
    Emergency,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct IntentLifecycleEvent {
    pub intent_id: String,
    pub issuer_id: String,
    pub trajectory_digest: String,
    pub time_basis: String,
    pub sequence: u64,
    pub event_time_s: i64,
    pub state: IntentState,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct LifecycleTransitionCase {
    pub case_id: String,
    pub previous: IntentLifecycleEvent,
    pub next: IntentLifecycleEvent,
    pub expected_outcome: TransitionOutcome,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TransitionOutcome {
    Accepted,
    Rejected,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TransitionError {
    EmptyIdentity,
    IdentityChanged,
    TrajectoryDigestChanged,
    TimeBasisChanged,
    StaleOrDuplicateSequence,
    SequenceGap,
    TimeReversal,
    IllegalStateTransition,
}

#[derive(Debug, Clone, Serialize)]
pub struct LifecycleTransitionReport {
    pub case_id: String,
    pub expected_outcome: TransitionOutcome,
    pub actual_outcome: TransitionOutcome,
    pub expectation_matches: bool,
    pub error: Option<TransitionError>,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Classification {
    AcceptForPlanningOnly,
    Conflict,
    Degraded,
    Reject,
    Unresolved,
    RequiresAuthorityReview,
}

#[derive(Debug, Clone, Serialize)]
pub struct SimulationReport {
    pub scenario_id: String,
    pub classification: Classification,
    pub expected_classification: Classification,
    pub expectation_matches: bool,
    pub flights_considered: usize,
    pub pair_count: usize,
    /// Number of pair-at-time evaluations, not number of unique time instants.
    pub pair_sample_checks: u64,
    pub conflicts: Vec<ConflictObservation>,
    pub reasons: Vec<String>,
    pub authority_ceiling: &'static str,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ConflictObservation {
    pub flight_a: String,
    pub flight_b: String,
    pub first_conservative_sample_time_s: i64,
    pub nominal_horizontal_distance_mm_floor: u64,
    pub nominal_vertical_distance_mm: u64,
    pub effective_horizontal_limit_mm: u64,
    pub effective_vertical_limit_mm: u64,
}

#[derive(Debug, Clone, Serialize)]
pub struct ReplayOutput {
    pub schema: &'static str,
    pub status: &'static str,
    pub input_scenario_count: usize,
    pub input_lifecycle_transition_case_count: usize,
    pub all_expectations_match: bool,
    pub reports: Vec<SimulationReport>,
    pub lifecycle_transition_reports: Vec<LifecycleTransitionReport>,
}

const AUTHORITY_CEILING: &str =
    "synthetic planning/simulation semantics only; never flight authorization or safety proof";

/// Evaluate one synthetic scenario without network access or wall-clock inputs.
///
/// The test policy uses caller-supplied thresholds and conservative expansion
/// around each sample. A conflict is a planning-level flag, not a separation
/// determination. No conflict found is not evidence that a real flight is safe.
pub fn evaluate_scenario(scenario: &Scenario) -> SimulationReport {
    let mut reasons = Vec::new();

    if scenario.scenario_id.trim().is_empty() {
        return report(scenario, Classification::Reject, 0, 0, 0, Vec::new(), vec![
            "scenario_id must not be empty".to_owned(),
        ]);
    }

    let duration = scenario.window_end_s.checked_sub(scenario.window_start_s);
    if duration.is_none()
        || duration.unwrap_or(0) <= 0
        || duration.unwrap_or(i64::MAX) > MAX_HORIZON_S
    {
        return report(scenario, Classification::Reject, scenario.flights.len(), 0, 0,
            Vec::new(), vec!["scenario window is invalid or exceeds the seven-day fixture bound".to_owned()]);
    }

    if scenario.policy.sample_step_s <= 0
        || scenario.policy.sample_step_s > MAX_SAMPLE_STEP_S
        || scenario.policy.horizontal_separation_mm > MAX_THRESHOLD_MM
        || scenario.policy.vertical_separation_mm > MAX_THRESHOLD_MM
    {
        return report(scenario, Classification::Reject, scenario.flights.len(), 0, 0,
            Vec::new(), vec!["policy step or threshold is outside the bounded fixture domain".to_owned()]);
    }

    if scenario.flights.is_empty() {
        return report(scenario, Classification::Reject, 0, 0, 0, Vec::new(),
            vec!["scenario must contain at least one flight intent".to_owned()]);
    }

    if scenario.flights.iter().any(|flight| !flight.time_qualified) {
        return report(scenario, Classification::Unresolved, scenario.flights.len(), pair_count(scenario.flights.len()), 0,
            Vec::new(), vec!["at least one flight has an unqualified time basis".to_owned()]);
    }

    let first = &scenario.flights[0];
    if first.reference_frame.trim().is_empty()
        || first.vertical_datum.trim().is_empty()
        || first.time_basis.trim().is_empty()
    {
        return report(scenario, Classification::Unresolved, scenario.flights.len(), pair_count(scenario.flights.len()), 0,
            Vec::new(), vec!["reference frame, vertical datum, and time basis must be explicit".to_owned()]);
    }

    if scenario.flights.iter().any(|flight| {
        flight.reference_frame != first.reference_frame
            || flight.vertical_datum != first.vertical_datum
            || flight.time_basis != first.time_basis
    }) {
        return report(scenario, Classification::Unresolved, scenario.flights.len(), pair_count(scenario.flights.len()), 0,
            Vec::new(), vec!["flight intents use incompatible reference frames, vertical datums, or time bases".to_owned()]);
    }

    for flight in &scenario.flights {
        if flight.flight_id.trim().is_empty() {
            reasons.push("flight_id must not be empty".to_owned());
        }
        if matches!(flight.state, IntentState::Withdrawn | IntentState::Ended) {
            reasons.push(format!(
                "flight intent '{}' is {:?} and cannot be treated as active future intent",
                flight.flight_id, flight.state
            ));
        }
        if flight.valid_from_s > scenario.window_start_s
            || flight.valid_until_s < scenario.window_end_s
            || flight.valid_until_s <= flight.valid_from_s
        {
            reasons.push(format!("flight intent '{}' does not cover the full simulation window", flight.flight_id));
        }
        if flight.horizontal_uncertainty_mm > MAX_THRESHOLD_MM
            || flight.vertical_uncertainty_mm > MAX_THRESHOLD_MM
            || !position_in_bounds(flight.position_at_valid_from_mm)
            || !velocity_in_bounds(flight.velocity_mm_s)
        {
            reasons.push(format!("flight intent '{}' exceeds bounded fixture inputs", flight.flight_id));
        }
        if position_at(flight, scenario.window_start_s).is_none()
            || position_at(flight, scenario.window_end_s).is_none()
        {
            reasons.push(format!("flight intent '{}' produces an out-of-bounds position", flight.flight_id));
        }
    }

    if !reasons.is_empty() {
        return report(scenario, Classification::Reject, scenario.flights.len(), pair_count(scenario.flights.len()), 0,
            Vec::new(), reasons);
    }

    let pair_total = pair_count(scenario.flights.len());
    let window_duration = u128::try_from(duration.unwrap_or(0)).unwrap_or(0);
    let step = u128::try_from(scenario.policy.sample_step_s).unwrap_or(0);
    let maximum_samples_per_pair = window_duration.div_ceil(step) + 1;
    let estimated_pair_sample_checks = (pair_total as u128).saturating_mul(maximum_samples_per_pair);
    if estimated_pair_sample_checks > MAX_PAIR_SAMPLE_CHECKS {
        return report(
            scenario,
            Classification::Reject,
            scenario.flights.len(),
            pair_total,
            0,
            Vec::new(),
            vec![format!(
                "estimated work ({estimated_pair_sample_checks} pair-sample checks) exceeds the fixture budget of {MAX_PAIR_SAMPLE_CHECKS}"
            )],
        );
    }

    let mut pair_sample_checks = 0_u64;
    let mut conflicts = Vec::new();

    for left_index in 0..scenario.flights.len() {
        for right_index in (left_index + 1)..scenario.flights.len() {
            let left = &scenario.flights[left_index];
            let right = &scenario.flights[right_index];

            let (horizontal_margin, vertical_margin) =
                conservative_margins(left, right, scenario.policy.sample_step_s);
            let horizontal_limit = u128::from(scenario.policy.horizontal_separation_mm)
                + u128::from(left.horizontal_uncertainty_mm)
                + u128::from(right.horizontal_uncertainty_mm)
                + horizontal_margin;
            let vertical_limit = u128::from(scenario.policy.vertical_separation_mm)
                + u128::from(left.vertical_uncertainty_mm)
                + u128::from(right.vertical_uncertainty_mm)
                + vertical_margin;

            let mut time_s = scenario.window_start_s;
            loop {
                pair_sample_checks = pair_sample_checks.saturating_add(1);
                let left_position = position_at(left, time_s);
                let right_position = position_at(right, time_s);

                let (Some(left_position), Some(right_position)) = (left_position, right_position) else {
                    return report(scenario, Classification::Reject, scenario.flights.len(), pair_total,
                        pair_sample_checks, conflicts, vec![
                            "a trajectory position could not be represented within the bounded fixture domain".to_owned(),
                        ]);
                };

                let dx = i128::from(left_position.x) - i128::from(right_position.x);
                let dy = i128::from(left_position.y) - i128::from(right_position.y);
                let dz = i128::from(left_position.z) - i128::from(right_position.z);
                let horizontal_distance_squared = square(dx) + square(dy);
                let horizontal_limit_squared = horizontal_limit * horizontal_limit;
                let vertical_distance = dz.unsigned_abs();

                if horizontal_distance_squared <= horizontal_limit_squared
                    && vertical_distance <= vertical_limit
                {
                    conflicts.push(ConflictObservation {
                        flight_a: left.flight_id.clone(),
                        flight_b: right.flight_id.clone(),
                        first_conservative_sample_time_s: time_s,
                        nominal_horizontal_distance_mm_floor: floor_sqrt(horizontal_distance_squared) as u64,
                        nominal_vertical_distance_mm: vertical_distance as u64,
                        effective_horizontal_limit_mm: horizontal_limit as u64,
                        effective_vertical_limit_mm: vertical_limit as u64,
                    });
                    break;
                }

                if time_s == scenario.window_end_s {
                    break;
                }
                let next = time_s.saturating_add(scenario.policy.sample_step_s);
                time_s = if next >= scenario.window_end_s { scenario.window_end_s } else { next };
            }
        }
    }

    let classification = if !conflicts.is_empty() {
        reasons.push("one or more trajectory envelopes meet the synthetic conflict predicate".to_owned());
        Classification::Conflict
    } else if scenario.flights.iter().any(|flight| flight.priority == FlightPriority::Emergency) {
        reasons.push("an emergency-priority intent requires a competent-authority/operational review".to_owned());
        Classification::RequiresAuthorityReview
    } else {
        reasons.push("no conflict was found by this bounded synthetic model; this is not operational clearance".to_owned());
        Classification::AcceptForPlanningOnly
    };

    report(scenario, classification, scenario.flights.len(), pair_total, pair_sample_checks, conflicts, reasons)
}

/// Evaluate a complete versioned fixture and preserve expected classifications.
pub fn replay_set(set: &ScenarioSet) -> ReplayOutput {
    let reports: Vec<SimulationReport> = set.scenarios.iter().map(evaluate_scenario).collect();
    let lifecycle_transition_reports: Vec<LifecycleTransitionReport> = set
        .lifecycle_transition_cases
        .iter()
        .map(evaluate_lifecycle_transition_case)
        .collect();
    let all_expectations_match = reports.iter().all(|item| item.expectation_matches)
        && lifecycle_transition_reports.iter().all(|item| item.expectation_matches);
    ReplayOutput {
        schema: "airspace-replay-output-v1",
        status: "research_fixture_not_operational",
        input_scenario_count: reports.len(),
        input_lifecycle_transition_case_count: lifecycle_transition_reports.len(),
        all_expectations_match,
        reports,
        lifecycle_transition_reports,
    }
}

pub fn validate_intent_transition(
    previous: &IntentLifecycleEvent,
    next: &IntentLifecycleEvent,
) -> Result<(), TransitionError> {
    if previous.intent_id.trim().is_empty()
        || previous.issuer_id.trim().is_empty()
        || previous.trajectory_digest.trim().is_empty()
        || previous.time_basis.trim().is_empty()
        || next.intent_id.trim().is_empty()
        || next.issuer_id.trim().is_empty()
        || next.trajectory_digest.trim().is_empty()
        || next.time_basis.trim().is_empty()
    {
        return Err(TransitionError::EmptyIdentity);
    }

    if previous.intent_id != next.intent_id || previous.issuer_id != next.issuer_id {
        return Err(TransitionError::IdentityChanged);
    }
    if previous.trajectory_digest != next.trajectory_digest {
        return Err(TransitionError::TrajectoryDigestChanged);
    }
    if previous.time_basis != next.time_basis {
        return Err(TransitionError::TimeBasisChanged);
    }
    if next.sequence <= previous.sequence {
        return Err(TransitionError::StaleOrDuplicateSequence);
    }
    if previous.sequence.checked_add(1) != Some(next.sequence) {
        return Err(TransitionError::SequenceGap);
    }
    if next.event_time_s < previous.event_time_s {
        return Err(TransitionError::TimeReversal);
    }
    if !state_transition_allowed(previous.state, next.state) {
        return Err(TransitionError::IllegalStateTransition);
    }
    Ok(())
}

fn state_transition_allowed(previous: IntentState, next: IntentState) -> bool {
    matches!(
        (previous, next),
        (IntentState::Proposed, IntentState::Accepted | IntentState::Withdrawn)
            | (IntentState::Accepted, IntentState::Activated | IntentState::Withdrawn)
            | (IntentState::Activated, IntentState::Ended)
    )
}

fn evaluate_lifecycle_transition_case(
    case: &LifecycleTransitionCase,
) -> LifecycleTransitionReport {
    let validation = validate_intent_transition(&case.previous, &case.next);
    let actual_outcome = if validation.is_ok() {
        TransitionOutcome::Accepted
    } else {
        TransitionOutcome::Rejected
    };
    LifecycleTransitionReport {
        case_id: case.case_id.clone(),
        expected_outcome: case.expected_outcome,
        actual_outcome,
        expectation_matches: actual_outcome == case.expected_outcome,
        error: validation.err(),
    }
}

fn report(
    scenario: &Scenario,
    classification: Classification,
    flights_considered: usize,
    pair_count: usize,
    pair_sample_checks: u64,
    conflicts: Vec<ConflictObservation>,
    reasons: Vec<String>,
) -> SimulationReport {
    SimulationReport {
        scenario_id: scenario.scenario_id.clone(),
        classification,
        expected_classification: scenario.expected_classification,
        expectation_matches: classification == scenario.expected_classification,
        flights_considered,
        pair_count,
        pair_sample_checks,
        conflicts,
        reasons,
        authority_ceiling: AUTHORITY_CEILING,
    }
}

fn pair_count(flights: usize) -> usize {
    flights.saturating_mul(flights.saturating_sub(1)) / 2
}

fn position_in_bounds(position: PositionMm) -> bool {
    [position.x, position.y, position.z]
        .iter()
        .all(|value| value.unsigned_abs() <= MAX_ABS_POSITION_MM as u64)
}

fn velocity_in_bounds(velocity: VelocityMmS) -> bool {
    [velocity.x, velocity.y, velocity.z]
        .iter()
        .all(|value| value.unsigned_abs() <= MAX_ABS_VELOCITY_MM_S as u64)
}

fn position_at(flight: &FlightIntent, time_s: i64) -> Option<PositionMm> {
    let elapsed = i128::from(time_s) - i128::from(flight.valid_from_s);
    let position = flight.position_at_valid_from_mm;
    let velocity = flight.velocity_mm_s;
    let result = PositionMm {
        x: i64::try_from(i128::from(position.x) + i128::from(velocity.x) * elapsed).ok()?,
        y: i64::try_from(i128::from(position.y) + i128::from(velocity.y) * elapsed).ok()?,
        z: i64::try_from(i128::from(position.z) + i128::from(velocity.z) * elapsed).ok()?,
    };
    position_in_bounds(result).then_some(result)
}

fn conservative_margins(
    left: &FlightIntent,
    right: &FlightIntent,
    step_s: i64,
) -> (u128, u128) {
    let dx = i128::from(left.velocity_mm_s.x) - i128::from(right.velocity_mm_s.x);
    let dy = i128::from(left.velocity_mm_s.y) - i128::from(right.velocity_mm_s.y);
    let dz = i128::from(left.velocity_mm_s.z) - i128::from(right.velocity_mm_s.z);
    let horizontal_speed_ceiling = ceil_sqrt(square(dx) + square(dy));
    let half_step_ceiling = (u128::try_from(step_s).unwrap_or(0) + 1) / 2;

    // Since the nearest sampled instant is no farther than half a sample
    // interval away, add relative-speed * ceil(step / 2). This intentionally
    // over-approximates near sample boundaries; false-positive conflicts are
    // preferable to interpreting this prototype as a safety clearance.
    (
        horizontal_speed_ceiling * half_step_ceiling,
        dz.unsigned_abs() * half_step_ceiling,
    )
}

fn square(value: i128) -> u128 {
    let magnitude = value.unsigned_abs();
    magnitude * magnitude
}

fn floor_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut low = 1_u128;
    let mut high = value.min(u64::MAX as u128) + 1;
    while low + 1 < high {
        let mid = low + (high - low) / 2;
        if mid <= value / mid {
            low = mid;
        } else {
            high = mid;
        }
    }
    low
}

fn ceil_sqrt(value: u128) -> u128 {
    let root = floor_sqrt(value);
    if root * root == value { root } else { root + 1 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_set() -> ScenarioSet {
        serde_json::from_str(include_str!("../fixtures/scenarios.json"))
            .expect("the committed scenario fixture must parse")
    }

    #[test]
    fn all_committed_scenario_expectations_match() {
        let set = fixture_set();
        let output = replay_set(&set);
        assert_eq!(output.status, "research_fixture_not_operational");
        assert_eq!(output.input_scenario_count, set.scenarios.len());
        assert_eq!(output.input_lifecycle_transition_case_count, set.lifecycle_transition_cases.len());
        assert!(
            output.all_expectations_match,
            "mismatched traffic reports: {:#?}; mismatched lifecycle reports: {:#?}",
            output.reports.iter().filter(|report| !report.expectation_matches).collect::<Vec<_>>(),
            output.lifecycle_transition_reports
                .iter()
                .filter(|report| !report.expectation_matches)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn detects_a_crossing_that_occurs_between_sample_instants() {
        let mut scenario = fixture_set().scenarios
            .into_iter()
            .find(|item| item.scenario_id == "between_sample_crossing")
            .expect("between-sample case exists");
        scenario.policy.sample_step_s = 2;
        scenario.window_end_s = 2;
        for flight in &mut scenario.flights {
            flight.valid_until_s = 2;
        }
        let report = evaluate_scenario(&scenario);
        assert_eq!(report.classification, Classification::Conflict);
        assert_eq!(report.conflicts[0].first_conservative_sample_time_s, 0);
        assert!(report.conflicts[0].effective_horizontal_limit_mm
            > scenario.policy.horizontal_separation_mm);
    }

    #[test]
    fn emergency_priority_requires_review_but_never_authorizes() {
        let mut scenario = fixture_set().scenarios
            .into_iter()
            .find(|item| item.scenario_id == "clear_separation")
            .expect("clear case exists");
        scenario.flights[0].priority = FlightPriority::Emergency;
        let report = evaluate_scenario(&scenario);
        assert_eq!(report.classification, Classification::RequiresAuthorityReview);
        assert!(report.authority_ceiling.contains("never flight authorization"));
    }

    #[test]
    fn proposed_intent_remains_planning_only() {
        let mut scenario = fixture_set().scenarios
            .into_iter()
            .find(|item| item.scenario_id == "clear_separation")
            .expect("clear case exists");
        scenario.flights[0].state = IntentState::Proposed;
        assert_eq!(evaluate_scenario(&scenario).classification, Classification::AcceptForPlanningOnly);
        assert!(evaluate_scenario(&scenario).authority_ceiling.contains("never flight authorization"));
    }

    #[test]
    fn incompatible_time_basis_is_unresolved() {
        let mut scenario = fixture_set().scenarios
            .into_iter()
            .find(|item| item.scenario_id == "clear_separation")
            .expect("clear case exists");
        scenario.flights[1].time_basis = "unrelated-clock-epoch".to_owned();
        assert_eq!(evaluate_scenario(&scenario).classification, Classification::Unresolved);
    }

    #[test]
    fn invalid_time_step_is_rejected() {
        let mut scenario = fixture_set().scenarios.remove(0);
        scenario.policy.sample_step_s = 0;
        assert_eq!(evaluate_scenario(&scenario).classification, Classification::Reject);
    }

    #[test]
    fn excessive_pair_sample_work_is_rejected_before_the_loop() {
        let mut scenario = fixture_set().scenarios.remove(0);
        scenario.window_end_s = MAX_HORIZON_S;
        for flight in &mut scenario.flights {
            flight.valid_until_s = MAX_HORIZON_S;
        }
        let template = scenario.flights[0].clone();
        for index in 2..5 {
            let mut flight = template.clone();
            flight.flight_id = format!("LOAD-{index}");
            scenario.flights.push(flight);
        }
        assert_eq!(evaluate_scenario(&scenario).classification, Classification::Reject);
    }

    #[test]
    fn withdrawn_and_ended_intents_are_not_active_future_intents() {
        let template = fixture_set().scenarios
            .into_iter()
            .find(|item| item.scenario_id == "clear_separation")
            .expect("clear case exists");
        for state in [IntentState::Withdrawn, IntentState::Ended] {
            let mut scenario = template.clone();
            scenario.flights[0].state = state;
            assert_eq!(evaluate_scenario(&scenario).classification, Classification::Reject);
        }
    }

    #[test]
    fn integer_square_root_is_floor_and_ceiling_are_consistent() {
        for value in 0_u128..10_000 {
            let floor = floor_sqrt(value);
            assert!(floor * floor <= value);
            assert!((floor + 1) * (floor + 1) > value);
            let ceiling = ceil_sqrt(value);
            assert!(ceiling * ceiling >= value);
            assert!(ceiling == floor || ceiling == floor + 1);
        }
    }
}
