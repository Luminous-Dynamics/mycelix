//! Deterministic, bounded research simulator for airspace-coordination semantics.
//!
//! This crate is not operational ATM/UTM software. It contains no real-world
//! separation minima, authorisation path, sensor ingestion, or aircraft control.
//! Every result is scoped to a synthetic fixture and the simplified trajectory
//! model documented in README.md.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

mod polygon;
mod model_applicability;
#[cfg(test)]
mod uncertainty;
#[cfg(test)]
mod typed_uncertainty;
pub use polygon::{
    evaluate_polygon_intersection, PolygonEvaluationReport, PolygonExclusionVolume,
    PolygonGeometryError, PolygonIntersectionOutcome, PolygonPointMm,
};

const MAX_HORIZON_S: i64 = 7 * 24 * 60 * 60;
const MAX_SAMPLE_STEP_S: i64 = 3_600;
const MAX_ABS_POSITION_MM: i64 = 1_000_000_000_000_000;
const MAX_ABS_VELOCITY_MM_S: i64 = 1_000_000_000;
const MAX_THRESHOLD_MM: u64 = 1_000_000_000_000_000;
const MAX_PAIR_SAMPLE_CHECKS: u128 = 5_000_000;
const MAX_RESERVATION_CLAIMS: usize = 300;
const MAX_RESERVATION_PAIR_CHECKS: usize = 50_000;
const MAX_RESERVATION_EVENTS: usize = 1_000;
const MAX_UNCERTAINTY_HORIZON_S: i64 = 3_600;
const MAX_CONSTRAINT_INTERSECTION_CHECKS: u128 = 500_000;
const MAX_CONSTRAINT_UPDATES: usize = 1_000;
const MAX_CONSTRAINT_FLIGHTS: usize = 300;
const MAX_CONSTRAINT_SAMPLE_CHECKS: u128 = 5_000_000;

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ScenarioSet {
    pub schema: String,
    pub status: String,
    pub authority_ceiling: String,
    pub scenarios: Vec<Scenario>,
    pub lifecycle_transition_cases: Vec<LifecycleTransitionCase>,
    pub reservation_cases: Vec<ReservationCoordinationCase>,
    #[serde(default)]
    pub reservation_event_cases: Vec<ReservationEventCoordinationCase>,
    #[serde(default)]
    pub dynamic_constraint_cases: Vec<DynamicConstraintCase>,
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
    /// Catalogue profile; missing values intentionally deserialize blank and fail closed.
    #[serde(default)]
    pub traffic_profile_id: String,
    /// Exact model contract admitted for this traffic profile.
    #[serde(default)]
    pub model_profile_id: String,
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

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
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
pub enum ProviderSnapshotState {
    CompleteFresh,
    Partitioned,
    StaleOrIncomplete,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ReservationClaim {
    /// Globally unique within this fixture; duplicate IDs are never last-write-wins.
    pub reservation_id: String,
    pub provider_id: String,
    pub resource_id: String,
    pub trajectory_digest: String,
    pub time_basis: String,
    pub sequence: u64,
    pub issued_at_s: i64,
    pub expires_at_s: i64,
    /// Half-open interval [start_s, end_s), in the declared synthetic time basis.
    pub start_s: i64,
    pub end_s: i64,
    pub state: IntentState,
    pub priority: FlightPriority,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ReservationCoordinationCase {
    pub case_id: String,
    pub now_s: i64,
    pub snapshot_state: ProviderSnapshotState,
    pub claims: Vec<ReservationClaim>,
    pub expected_outcome: ReservationOutcome,
}

/// Provider-local reservation event. Digest fields are opaque fixture identifiers,
/// not computed hashes or authenticated integrity evidence.
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq, Eq)]
pub struct ReservationEvent {
    pub reservation_id: String,
    pub provider_id: String,
    pub sequence: u64,
    pub event_digest: String,
    pub previous_event_digest: Option<String>,
    pub issued_at_s: i64,
    pub expires_at_s: i64,
    pub resource_id: String,
    pub trajectory_digest: String,
    pub time_basis: String,
    pub start_s: i64,
    pub end_s: i64,
    pub state: IntentState,
    pub priority: FlightPriority,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ReservationEventCoordinationCase {
    pub case_id: String,
    pub now_s: i64,
    pub snapshot_state: ProviderSnapshotState,
    pub events: Vec<ReservationEvent>,
    pub expected_outcome: ReservationOutcome,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ConstraintState {
    Active,
    Withdrawn,
}

/// One version of a synthetic, source-qualified 4D airspace constraint.
/// Bounds are axis-aligned within the named local frame; time is half-open
/// [valid_from_s, valid_until_s). Digest fields are opaque fixture labels only.
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq, Eq)]
pub struct AirspaceConstraintUpdate {
    pub constraint_id: String,
    pub authority_id: String,
    pub sequence: u64,
    pub update_digest: String,
    pub previous_update_digest: Option<String>,
    pub issued_at_s: i64,
    pub reference_frame: String,
    pub vertical_datum: String,
    pub time_basis: String,
    pub valid_from_s: i64,
    pub valid_until_s: i64,
    pub min_position_mm: PositionMm,
    pub max_position_mm: PositionMm,
    pub horizontal_margin_mm: u64,
    pub vertical_margin_mm: u64,
    pub state: ConstraintState,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct DynamicConstraintCase {
    pub case_id: String,
    pub now_s: i64,
    pub snapshot_state: ProviderSnapshotState,
    /// Synthetic horizon for a conservative motion guard band.
    pub uncertainty_horizon_s: i64,
    pub flights: Vec<FlightIntent>,
    pub updates: Vec<AirspaceConstraintUpdate>,
    pub expected_outcome: DynamicConstraintOutcome,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DynamicConstraintOutcome {
    NoImpactInProvidedSnapshot,
    ReassessmentRequired,
    Degraded,
    Reject,
    Unresolved,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DynamicConstraintError {
    EmptyIdentity,
    TooManyUpdates,
    TooManyFlights,
    InvalidUncertaintyHorizon,
    InvalidSequence,
    FutureIssuedAt,
    InvalidValidityInterval,
    InvalidBounds,
    InvalidMargin,
    ConflictingUpdateDigest,
    SequenceGap,
    ForkedSequence,
    PredecessorMismatch,
    TimeReversal,
    AuthorityChanged,
    ReferenceFrameChanged,
    VerticalDatumChanged,
    TimeBasisChanged,
    IllegalStateTransition,
    EmptyFlightIdentity,
    DuplicateFlightIdentity,
    InvalidFlightWindow,
    InvalidFlightGeometry,
    UnsupportedModelProfile,
    UnqualifiedFlightTime,
    UnqualifiedFrame,
    IncompatibleReferenceFrame,
    IncompatibleVerticalDatum,
    IncompatibleTimeBasis,
    WorkBudgetExceeded,
}

#[derive(Debug, Clone, Serialize)]
pub struct DynamicConstraintImpact {
    pub flight_id: String,
    pub constraint_id: String,
    pub constraint_sequence: u64,
    pub first_conservative_intersection_time_s: i64,
}

#[derive(Debug, Clone, Serialize)]
pub struct DynamicConstraintReport {
    pub case_id: String,
    pub actual_outcome: DynamicConstraintOutcome,
    pub expected_outcome: DynamicConstraintOutcome,
    pub expectation_matches: bool,
    pub received_update_count: usize,
    pub constraint_stream_count: usize,
    pub unique_update_count: usize,
    pub replayed_delivery_count: usize,
    pub active_constraint_count: usize,
    pub flights_considered: usize,
    pub intersection_checks: u64,
    pub impacts: Vec<DynamicConstraintImpact>,
    pub errors: Vec<DynamicConstraintError>,
    pub reasons: Vec<String>,
    pub authority_ceiling: &'static str,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ReservationOutcome {
    NoConflictInProvidedSnapshot,
    Conflict,
    Degraded,
    Reject,
    Unresolved,
    RequiresAuthorityReview,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ReservationError {
    EmptyIdentity,
    InvalidSequence,
    InvalidInterval,
    InvalidExpiry,
    FutureIssuedAt,
    DuplicateReservationId,
    TimeBasisMismatch,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ReservationEventError {
    EmptyIdentity,
    EmptyEventSet,
    TooManyEvents,
    InvalidSequence,
    InvalidInterval,
    InvalidExpiry,
    FutureIssuedAt,
    ConflictingEventDigest,
    SequenceGap,
    ForkedSequence,
    PredecessorMismatch,
    ReservationSemanticsChanged,
    TimeReversal,
    IllegalStateTransition,
}

#[derive(Debug, Clone, Serialize)]
pub struct ReservationEventCoordinationReport {
    pub case_id: String,
    pub actual_outcome: ReservationOutcome,
    pub expected_outcome: ReservationOutcome,
    pub expectation_matches: bool,
    pub received_event_count: usize,
    pub unique_event_count: usize,
    pub replayed_delivery_count: usize,
    pub reservation_stream_count: usize,
    pub current_claim_count: usize,
    pub conflicts: Vec<ReservationConflict>,
    pub errors: Vec<ReservationEventError>,
    pub reasons: Vec<String>,
    pub authority_ceiling: &'static str,
}

#[derive(Debug, Clone, Serialize)]
pub struct ReservationConflict {
    pub reservation_a: String,
    pub provider_a: String,
    pub reservation_b: String,
    pub provider_b: String,
    pub resource_id: String,
    pub overlap_start_s: i64,
    pub overlap_end_s: i64,
}

#[derive(Debug, Clone, Serialize)]
pub struct ReservationCoordinationReport {
    pub case_id: String,
    pub actual_outcome: ReservationOutcome,
    pub expected_outcome: ReservationOutcome,
    pub expectation_matches: bool,
    pub active_claim_count: usize,
    pub conflicts: Vec<ReservationConflict>,
    pub errors: Vec<ReservationError>,
    pub reasons: Vec<String>,
    pub authority_ceiling: &'static str,
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
    pub first_conservative_intersection_time_s: i64,
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
    pub input_reservation_case_count: usize,
    pub input_reservation_event_case_count: usize,
    pub input_dynamic_constraint_case_count: usize,
    pub all_expectations_match: bool,
    pub reports: Vec<SimulationReport>,
    pub lifecycle_transition_reports: Vec<LifecycleTransitionReport>,
    pub reservation_reports: Vec<ReservationCoordinationReport>,
    pub reservation_event_reports: Vec<ReservationEventCoordinationReport>,
    pub dynamic_constraint_reports: Vec<DynamicConstraintReport>,
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

    // Flight IDs are the attribution keys in conflict reports. Never perform
    // pairwise evaluation when two input records would be indistinguishable.
    let mut seen_flight_ids = BTreeMap::<&str, ()>::new();
    for flight in &scenario.flights {
        if flight.flight_id.trim().is_empty() {
            continue; // The existing qualification loop below rejects empty IDs.
        }
        if seen_flight_ids.insert(flight.flight_id.as_str(), ()).is_some() {
            return report(
                scenario,
                Classification::Reject,
                scenario.flights.len(),
                pair_count(scenario.flights.len()),
                0,
                Vec::new(),
                vec![format!(
                    "duplicate flight_id '{}' makes pairwise conflict attribution ambiguous",
                    flight.flight_id
                )],
            );
        }
    }

    // Model applicability is an admission gate: do not run even the toy trajectory
    // evaluator for a missing, unknown, or real-world profile/model combination.
    if let Some(reason) = scenario.flights.iter().find_map(|flight| {
        model_applicability::validate_model_profile(
            &flight.traffic_profile_id,
            &flight.model_profile_id,
        )
        .err()
        .map(|error| format!(
            "flight intent '{}' model applicability unresolved: {error}",
            flight.flight_id
        ))
    }) {
        return report(
            scenario,
            Classification::Unresolved,
            scenario.flights.len(),
            pair_count(scenario.flights.len()),
            0,
            Vec::new(),
            vec![reason],
        );
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
                        first_conservative_intersection_time_s: time_s,
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

const RESERVATION_AUTHORITY_CEILING: &str =
    "synthetic reservation snapshot assessment only; no reservation grant or flight authorization";

/// Assess a single provider snapshot without choosing a winner or granting a slot.
///
/// Time intervals are half-open [start_s, end_s). This checks only the supplied
/// snapshot; an incomplete snapshot can never yield a no-conflict conclusion.
pub fn evaluate_reservation_case(case: &ReservationCoordinationCase) -> ReservationCoordinationReport {
    let mut errors = Vec::new();
    let mut reasons = Vec::new();

    if case.case_id.trim().is_empty() {
        return reservation_report(
            case,
            ReservationOutcome::Reject,
            0,
            Vec::new(),
            Vec::new(),
            vec!["case_id must not be empty".to_owned()],
        );
    }

    let claim_count = case.claims.len();
    let estimated_pair_checks = claim_count.saturating_mul(claim_count.saturating_sub(1)) / 2;
    if claim_count > MAX_RESERVATION_CLAIMS
        || estimated_pair_checks > MAX_RESERVATION_PAIR_CHECKS
    {
        return reservation_report(
            case,
            ReservationOutcome::Reject,
            0,
            Vec::new(),
            Vec::new(),
            vec![format!(
                "reservation snapshot exceeds the bounded fixture budget: claims={claim_count}, pair_checks={estimated_pair_checks}"
            )],
        );
    }

    for claim in &case.claims {
        if claim.reservation_id.trim().is_empty()
            || claim.provider_id.trim().is_empty()
            || claim.resource_id.trim().is_empty()
            || claim.trajectory_digest.trim().is_empty()
            || claim.time_basis.trim().is_empty()
        {
            errors.push(ReservationError::EmptyIdentity);
        }
        if claim.sequence == 0 {
            errors.push(ReservationError::InvalidSequence);
        }
        if claim.start_s >= claim.end_s {
            errors.push(ReservationError::InvalidInterval);
        }
        if claim.expires_at_s <= claim.issued_at_s {
            errors.push(ReservationError::InvalidExpiry);
        }
        if claim.issued_at_s > case.now_s {
            errors.push(ReservationError::FutureIssuedAt);
        }
    }

    if !errors.is_empty() {
        let has_malformed_claim = errors.iter().any(|error| {
            matches!(
                error,
                ReservationError::EmptyIdentity
                    | ReservationError::InvalidSequence
                    | ReservationError::InvalidInterval
                    | ReservationError::InvalidExpiry
            )
        });
        let outcome = if has_malformed_claim {
            ReservationOutcome::Reject
        } else {
            ReservationOutcome::Unresolved
        };
        reasons.push("one or more claims failed identity, sequence, time, or interval qualification".to_owned());
        return reservation_report(case, outcome, 0, Vec::new(), errors, reasons);
    }

    // A snapshot must contain one current projection per global reservation ID.
    // Never apply an implicit last-writer-wins policy to duplicate/competing versions.
    for left in 0..case.claims.len() {
        for right in (left + 1)..case.claims.len() {
            if case.claims[left].reservation_id == case.claims[right].reservation_id {
                errors.push(ReservationError::DuplicateReservationId);
            }
        }
    }
    if !errors.is_empty() {
        reasons.push("duplicate reservation IDs require explicit upstream reconciliation".to_owned());
        return reservation_report(
            case,
            ReservationOutcome::Unresolved,
            0,
            Vec::new(),
            errors,
            reasons,
        );
    }

    let active_claims: Vec<&ReservationClaim> = case
        .claims
        .iter()
        .filter(|claim| {
            !matches!(claim.state, IntentState::Withdrawn | IntentState::Ended)
                && case.now_s < claim.expires_at_s
                && case.now_s < claim.end_s
        })
        .collect();

    if let Some(first) = active_claims.first() {
        if active_claims.iter().any(|claim| claim.time_basis != first.time_basis) {
            reasons.push("active reservation claims use incompatible time bases".to_owned());
            return reservation_report(
                case,
                ReservationOutcome::Unresolved,
                active_claims.len(),
                Vec::new(),
                vec![ReservationError::TimeBasisMismatch],
                reasons,
            );
        }
    }

    let mut conflicts = Vec::new();
    for left_index in 0..active_claims.len() {
        for right_index in (left_index + 1)..active_claims.len() {
            let left = active_claims[left_index];
            let right = active_claims[right_index];
            if left.resource_id != right.resource_id {
                continue;
            }

            // Half-open intervals [start, end): adjacent reservations do not overlap.
            let overlap_start_s = left.start_s.max(right.start_s);
            let overlap_end_s = left.end_s.min(right.end_s);
            if overlap_start_s < overlap_end_s {
                conflicts.push(ReservationConflict {
                    reservation_a: left.reservation_id.clone(),
                    provider_a: left.provider_id.clone(),
                    reservation_b: right.reservation_id.clone(),
                    provider_b: right.provider_id.clone(),
                    resource_id: left.resource_id.clone(),
                    overlap_start_s,
                    overlap_end_s,
                });
            }
        }
    }

    let emergency_present = active_claims
        .iter()
        .any(|claim| claim.priority == FlightPriority::Emergency);
    let outcome = if !conflicts.is_empty() {
        reasons.push("overlapping current claims were found; this evaluator does not pick a winner".to_owned());
        if emergency_present {
            reasons.push("an emergency claim is present but does not automatically preempt another claim".to_owned());
        }
        ReservationOutcome::Conflict
    } else if case.snapshot_state != ProviderSnapshotState::CompleteFresh {
        reasons.push("provider snapshot is partitioned, stale, or incomplete; absence of a conflict cannot be established".to_owned());
        if emergency_present {
            reasons.push("emergency claims in an incomplete view still require the competent operational process".to_owned());
        }
        ReservationOutcome::Degraded
    } else if emergency_present {
        reasons.push("emergency claim requires competent-authority/operational review; no priority-based authorization is inferred".to_owned());
        ReservationOutcome::RequiresAuthorityReview
    } else {
        reasons.push("no overlap was found in this complete supplied snapshot; this is not a slot grant or flight authorization".to_owned());
        ReservationOutcome::NoConflictInProvidedSnapshot
    };

    reservation_report(
        case,
        outcome,
        active_claims.len(),
        conflicts,
        errors,
        reasons,
    )
}

fn reservation_report(
    case: &ReservationCoordinationCase,
    actual_outcome: ReservationOutcome,
    active_claim_count: usize,
    conflicts: Vec<ReservationConflict>,
    errors: Vec<ReservationError>,
    reasons: Vec<String>,
) -> ReservationCoordinationReport {
    ReservationCoordinationReport {
        case_id: case.case_id.clone(),
        actual_outcome,
        expected_outcome: case.expected_outcome,
        expectation_matches: actual_outcome == case.expected_outcome,
        active_claim_count,
        conflicts,
        errors,
        reasons,
        authority_ceiling: RESERVATION_AUTHORITY_CEILING,
    }
}

const RESERVATION_EVENT_AUTHORITY_CEILING: &str =
    "synthetic reservation event-stream reconciliation only; no authenticated exchange, reservation grant, or flight authorization";

/// Reconcile provider-local streams, then run the existing bounded reservation
/// overlap screen. Input arrival order is not semantic; per-reservation sequence
/// and predecessor linkage define the local history.
pub fn evaluate_reservation_event_case(
    case: &ReservationEventCoordinationCase,
) -> ReservationEventCoordinationReport {
    let mut errors = Vec::new();
    let mut reasons = Vec::new();
    let received_count = case.events.len();

    if case.case_id.trim().is_empty() {
        return reservation_event_report(
            case, ReservationOutcome::Reject, 0, 0, 0, 0, Vec::new(),
            vec![ReservationEventError::EmptyIdentity],
            vec!["case_id must not be empty".to_owned()],
        );
    }
    if received_count == 0 {
        return reservation_event_report(
            case, ReservationOutcome::Reject, 0, 0, 0, 0, Vec::new(),
            vec![ReservationEventError::EmptyEventSet],
            vec!["an empty event set cannot establish a current reservation view".to_owned()],
        );
    }
    if received_count > MAX_RESERVATION_EVENTS {
        return reservation_event_report(
            case, ReservationOutcome::Reject, 0, 0, 0, 0, Vec::new(),
            vec![ReservationEventError::TooManyEvents],
            vec![format!("received event count {received_count} exceeds the deterministic replay bound {MAX_RESERVATION_EVENTS}")],
        );
    }

    for event in &case.events {
        if event.reservation_id.trim().is_empty()
            || event.provider_id.trim().is_empty()
            || event.event_digest.trim().is_empty()
            || event.resource_id.trim().is_empty()
            || event.trajectory_digest.trim().is_empty()
            || event.time_basis.trim().is_empty()
        {
            errors.push(ReservationEventError::EmptyIdentity);
        }
        if event.sequence == 0 {
            errors.push(ReservationEventError::InvalidSequence);
        }
        if event.start_s >= event.end_s {
            errors.push(ReservationEventError::InvalidInterval);
        }
        if event.expires_at_s <= event.issued_at_s {
            errors.push(ReservationEventError::InvalidExpiry);
        }
        if event.issued_at_s > case.now_s {
            errors.push(ReservationEventError::FutureIssuedAt);
        }
    }
    if !errors.is_empty() {
        reasons.push("one or more events failed identity, sequence, interval, expiry, or issue-time qualification".to_owned());
        let outcome = reservation_event_error_outcome(&errors);
        return reservation_event_report(
            case, outcome, 0, 0, 0, 0, Vec::new(), errors, reasons,
        );
    }

    // An opaque digest may only identify one immutable event payload. Exact
    // full-event retransmissions are idempotent and counted; changed payloads
    // are unresolved. No cryptographic verification is claimed.
    let mut unique_events: BTreeMap<String, ReservationEvent> = BTreeMap::new();
    let mut replayed_delivery_count = 0_usize;
    for event in &case.events {
        if let Some(existing) = unique_events.get(&event.event_digest) {
            if existing == event {
                replayed_delivery_count = replayed_delivery_count.saturating_add(1);
            } else {
                errors.push(ReservationEventError::ConflictingEventDigest);
            }
        } else {
            unique_events.insert(event.event_digest.clone(), event.clone());
        }
    }
    if !errors.is_empty() {
        reasons.push("one event digest identifies different payloads; the provider history cannot be reconciled".to_owned());
        return reservation_event_report(
            case, ReservationOutcome::Unresolved, 0, unique_events.len(),
            replayed_delivery_count, 0, Vec::new(), errors, reasons,
        );
    }

    let unique_event_count = unique_events.len();
    let mut streams: BTreeMap<String, Vec<ReservationEvent>> = BTreeMap::new();
    for event in unique_events.values() {
        streams.entry(event.reservation_id.clone()).or_default().push(event.clone());
    }

    for events in streams.values_mut() {
        events.sort_by_key(|event| event.sequence);
        let Some(first) = events.first() else {
            continue;
        };
        if first.sequence != 1 {
            errors.push(ReservationEventError::SequenceGap);
        }
        if first.previous_event_digest.is_some() {
            errors.push(ReservationEventError::PredecessorMismatch);
        }
        if first.state != IntentState::Proposed {
            errors.push(ReservationEventError::IllegalStateTransition);
        }
        for pair in events.windows(2) {
            let previous = &pair[0];
            let next = &pair[1];
            if previous.sequence == next.sequence {
                errors.push(ReservationEventError::ForkedSequence);
                continue;
            }
            if previous.sequence.checked_add(1) != Some(next.sequence) {
                errors.push(ReservationEventError::SequenceGap);
            }
            if next.previous_event_digest.as_deref() != Some(previous.event_digest.as_str()) {
                errors.push(ReservationEventError::PredecessorMismatch);
            }
            if next.issued_at_s < previous.issued_at_s {
                errors.push(ReservationEventError::TimeReversal);
            }
            if !same_reservation_event_semantics(first, next) {
                errors.push(ReservationEventError::ReservationSemanticsChanged);
            }
            if !state_transition_allowed(previous.state, next.state) {
                errors.push(ReservationEventError::IllegalStateTransition);
            }
        }
    }

    if !errors.is_empty() {
        reasons.push("one or more streams have a gap, fork, broken predecessor link, time reversal, changed reservation semantics, or illegal lifecycle".to_owned());
        let outcome = reservation_event_error_outcome(&errors);
        return reservation_event_report(
            case, outcome, streams.len(), unique_event_count,
            replayed_delivery_count, 0, Vec::new(), errors, reasons,
        );
    }

    let mut claims = Vec::new();
    for events in streams.values() {
        let Some(latest) = events.last() else {
            continue;
        };
        // Proposed intents do not occupy an accepted reservation. Terminal,
        // expired, and ended windows are not current claims.
        if !matches!(latest.state, IntentState::Accepted | IntentState::Activated)
            || case.now_s >= latest.expires_at_s
            || case.now_s >= latest.end_s
        {
            continue;
        }
        claims.push(ReservationClaim {
            reservation_id: latest.reservation_id.clone(),
            provider_id: latest.provider_id.clone(),
            resource_id: latest.resource_id.clone(),
            trajectory_digest: latest.trajectory_digest.clone(),
            time_basis: latest.time_basis.clone(),
            sequence: latest.sequence,
            issued_at_s: latest.issued_at_s,
            expires_at_s: latest.expires_at_s,
            start_s: latest.start_s,
            end_s: latest.end_s,
            state: latest.state,
            priority: latest.priority,
        });
    }

    let projected_snapshot = ReservationCoordinationCase {
        case_id: case.case_id.clone(),
        now_s: case.now_s,
        snapshot_state: case.snapshot_state,
        claims,
        expected_outcome: case.expected_outcome,
    };
    let snapshot_report = evaluate_reservation_case(&projected_snapshot);
    reasons.extend(snapshot_report.reasons);
    reasons.push(format!(
        "reconciled {} provider-local reservation stream(s) from {} unique event(s); {} exact retransmission(s) were deduplicated",
        streams.len(), unique_event_count, replayed_delivery_count
    ));
    reasons.push("opaque event-digest linkage is a consistency fixture, not cryptographic verification or authenticated provider evidence".to_owned());

    reservation_event_report(
        case, snapshot_report.actual_outcome, streams.len(), unique_event_count,
        replayed_delivery_count, snapshot_report.active_claim_count,
        snapshot_report.conflicts, Vec::new(), reasons,
    )
}

fn same_reservation_event_semantics(first: &ReservationEvent, next: &ReservationEvent) -> bool {
    first.reservation_id == next.reservation_id
        && first.provider_id == next.provider_id
        && first.resource_id == next.resource_id
        && first.trajectory_digest == next.trajectory_digest
        && first.time_basis == next.time_basis
        && first.expires_at_s == next.expires_at_s
        && first.start_s == next.start_s
        && first.end_s == next.end_s
        && first.priority == next.priority
}

fn reservation_event_error_outcome(errors: &[ReservationEventError]) -> ReservationOutcome {
    if errors.iter().any(|error| {
        matches!(
            error,
            ReservationEventError::EmptyIdentity
                | ReservationEventError::EmptyEventSet
                | ReservationEventError::TooManyEvents
                | ReservationEventError::InvalidSequence
                | ReservationEventError::InvalidInterval
                | ReservationEventError::InvalidExpiry
        )
    }) {
        ReservationOutcome::Reject
    } else {
        ReservationOutcome::Unresolved
    }
}

fn reservation_event_report(
    case: &ReservationEventCoordinationCase,
    actual_outcome: ReservationOutcome,
    reservation_stream_count: usize,
    unique_event_count: usize,
    replayed_delivery_count: usize,
    current_claim_count: usize,
    conflicts: Vec<ReservationConflict>,
    errors: Vec<ReservationEventError>,
    reasons: Vec<String>,
) -> ReservationEventCoordinationReport {
    ReservationEventCoordinationReport {
        case_id: case.case_id.clone(),
        actual_outcome,
        expected_outcome: case.expected_outcome,
        expectation_matches: actual_outcome == case.expected_outcome,
        received_event_count: case.events.len(),
        unique_event_count,
        replayed_delivery_count,
        reservation_stream_count,
        current_claim_count,
        conflicts,
        errors,
        reasons,
        authority_ceiling: RESERVATION_EVENT_AUTHORITY_CEILING,
    }
}

/// Evaluate a complete versioned fixture and preserve expected classifications.
const DYNAMIC_CONSTRAINT_AUTHORITY_CEILING: &str =
    "synthetic dynamic-constraint impact screening only; does not update/withdraw authorizations or authorize flight";

/// Reconcile constraint-update streams, then conservatively screen current flight
/// intent against active 4D exclusion volumes. Known impacts win over snapshot
/// freshness: an observed overlap always asks for reassessment. A missing or stale
/// view can never establish absence of impact.
pub fn evaluate_dynamic_constraint_case(case: &DynamicConstraintCase) -> DynamicConstraintReport {
    let mut errors = Vec::new();
    let mut reasons = Vec::new();
    let received_count = case.updates.len();

    if case.case_id.trim().is_empty() {
        return dynamic_constraint_report(
            case, DynamicConstraintOutcome::Reject, 0, 0, 0, 0, 0,
            Vec::new(), vec![DynamicConstraintError::EmptyIdentity],
            vec!["case_id must not be empty".to_owned()],
        );
    }
    if case.uncertainty_horizon_s < 0 || case.uncertainty_horizon_s > MAX_UNCERTAINTY_HORIZON_S {
        return dynamic_constraint_report(
            case, DynamicConstraintOutcome::Reject, 0, 0, 0, 0, 0,
            Vec::new(), vec![DynamicConstraintError::InvalidUncertaintyHorizon],
            vec!["uncertainty horizon must be non-negative and within the synthetic work bound".to_owned()],
        );
    }
    if received_count > MAX_CONSTRAINT_UPDATES {
        return dynamic_constraint_report(
            case, DynamicConstraintOutcome::Reject, 0, 0, 0, 0, 0,
            Vec::new(), vec![DynamicConstraintError::TooManyUpdates],
            vec![format!("received update count {received_count} exceeds the replay bound {MAX_CONSTRAINT_UPDATES}")],
        );
    }
    if case.flights.len() > MAX_CONSTRAINT_FLIGHTS {
        return dynamic_constraint_report(
            case, DynamicConstraintOutcome::Reject, 0, 0, 0, 0, 0,
            Vec::new(), vec![DynamicConstraintError::TooManyFlights],
            vec![format!("flight count {} exceeds the replay bound {MAX_CONSTRAINT_FLIGHTS}", case.flights.len())],
        );
    }

    for update in &case.updates {
        if update.constraint_id.trim().is_empty()
            || update.authority_id.trim().is_empty()
            || update.update_digest.trim().is_empty()
            || update.reference_frame.trim().is_empty()
            || update.vertical_datum.trim().is_empty()
            || update.time_basis.trim().is_empty()
        {
            errors.push(DynamicConstraintError::EmptyIdentity);
        }
        if update.sequence == 0 {
            errors.push(DynamicConstraintError::InvalidSequence);
        }
        if update.issued_at_s > case.now_s {
            errors.push(DynamicConstraintError::FutureIssuedAt);
        }
        let duration = update.valid_until_s.checked_sub(update.valid_from_s);
        if duration.is_none()
            || duration.unwrap_or(0) <= 0
            || duration.unwrap_or(i64::MAX) > MAX_HORIZON_S
        {
            errors.push(DynamicConstraintError::InvalidValidityInterval);
        }
        if !position_in_bounds(update.min_position_mm)
            || !position_in_bounds(update.max_position_mm)
            || update.min_position_mm.x > update.max_position_mm.x
            || update.min_position_mm.y > update.max_position_mm.y
            || update.min_position_mm.z > update.max_position_mm.z
        {
            errors.push(DynamicConstraintError::InvalidBounds);
        }
        if update.horizontal_margin_mm > MAX_THRESHOLD_MM
            || update.vertical_margin_mm > MAX_THRESHOLD_MM
        {
            errors.push(DynamicConstraintError::InvalidMargin);
        }
    }
    if !errors.is_empty() {
        reasons.push("a constraint update failed source identity, version, time, volume, or uncertainty qualification".to_owned());
        let outcome = dynamic_constraint_error_outcome(&errors);
        return dynamic_constraint_report(case, outcome, 0, 0, 0, 0, 0, Vec::new(), errors, reasons);
    }

    // An update identifier may denote only one complete, typed payload.
    // Exact retransmissions are idempotent and counted; conflicting reuse is unresolved.
    let mut unique_updates: BTreeMap<String, AirspaceConstraintUpdate> = BTreeMap::new();
    let mut replayed_delivery_count = 0_usize;
    for update in &case.updates {
        if let Some(existing) = unique_updates.get(&update.update_digest) {
            if existing == update {
                replayed_delivery_count = replayed_delivery_count.saturating_add(1);
            } else {
                errors.push(DynamicConstraintError::ConflictingUpdateDigest);
            }
        } else {
            unique_updates.insert(update.update_digest.clone(), update.clone());
        }
    }
    if !errors.is_empty() {
        reasons.push("one update digest identifies different payloads; constraint history cannot be reconciled".to_owned());
        return dynamic_constraint_report(
            case, DynamicConstraintOutcome::Unresolved, 0, unique_updates.len(),
            replayed_delivery_count, 0, 0, Vec::new(), errors, reasons,
        );
    }

    let unique_update_count = unique_updates.len();
    let mut streams: BTreeMap<String, Vec<AirspaceConstraintUpdate>> = BTreeMap::new();
    for update in unique_updates.values() {
        streams.entry(update.constraint_id.clone()).or_default().push(update.clone());
    }

    for updates in streams.values_mut() {
        updates.sort_by_key(|update| update.sequence);
        let Some(first) = updates.first() else {
            continue;
        };
        if first.sequence != 1 {
            errors.push(DynamicConstraintError::SequenceGap);
        }
        if first.previous_update_digest.is_some() {
            errors.push(DynamicConstraintError::PredecessorMismatch);
        }
        if first.state != ConstraintState::Active {
            errors.push(DynamicConstraintError::IllegalStateTransition);
        }
        for pair in updates.windows(2) {
            let previous = &pair[0];
            let next = &pair[1];
            if previous.sequence == next.sequence {
                errors.push(DynamicConstraintError::ForkedSequence);
                continue;
            }
            if previous.sequence.checked_add(1) != Some(next.sequence) {
                errors.push(DynamicConstraintError::SequenceGap);
            }
            if next.previous_update_digest.as_deref() != Some(previous.update_digest.as_str()) {
                errors.push(DynamicConstraintError::PredecessorMismatch);
            }
            if next.issued_at_s < previous.issued_at_s {
                errors.push(DynamicConstraintError::TimeReversal);
            }
            if previous.authority_id != next.authority_id {
                errors.push(DynamicConstraintError::AuthorityChanged);
            }
            if previous.reference_frame != next.reference_frame {
                errors.push(DynamicConstraintError::ReferenceFrameChanged);
            }
            if previous.vertical_datum != next.vertical_datum {
                errors.push(DynamicConstraintError::VerticalDatumChanged);
            }
            if previous.time_basis != next.time_basis {
                errors.push(DynamicConstraintError::TimeBasisChanged);
            }
            if !constraint_state_transition_allowed(previous.state, next.state) {
                errors.push(DynamicConstraintError::IllegalStateTransition);
            }
        }
    }

    if !errors.is_empty() {
        reasons.push("one or more constraint histories contain a sequence gap/fork, broken predecessor, source/time qualification change, or illegal transition".to_owned());
        let outcome = dynamic_constraint_error_outcome(&errors);
        return dynamic_constraint_report(
            case, outcome, streams.len(), unique_update_count, replayed_delivery_count,
            0, 0, Vec::new(), errors, reasons,
        );
    }

    let active_constraints: Vec<&AirspaceConstraintUpdate> = streams.values()
        .filter_map(|updates| updates.last())
        .filter(|update| update.state == ConstraintState::Active && update.valid_until_s > case.now_s)
        .collect();

    let mut current_flights: Vec<&FlightIntent> = Vec::new();
    let mut seen_flight_ids = BTreeMap::<&str, ()>::new();
    for flight in &case.flights {
        if let Err(reason) = model_applicability::validate_model_profile(
            &flight.traffic_profile_id,
            &flight.model_profile_id,
        ) {
            errors.push(DynamicConstraintError::UnsupportedModelProfile);
            reasons.push(format!(
                "flight intent '{}' model applicability unresolved: {reason}",
                flight.flight_id
            ));
        }
        if flight.flight_id.trim().is_empty() {
            errors.push(DynamicConstraintError::EmptyFlightIdentity);
        } else if seen_flight_ids.insert(flight.flight_id.as_str(), ()).is_some() {
            // Duplicate identifiers make it impossible to attribute impacts
            // unambiguously; reject the entire case before intersection work.
            errors.push(DynamicConstraintError::DuplicateFlightIdentity);
        }
        if flight.valid_until_s <= flight.valid_from_s
            || flight.valid_until_s.checked_sub(flight.valid_from_s)
                .map(|duration| duration > MAX_HORIZON_S)
                .unwrap_or(true)
        {
            errors.push(DynamicConstraintError::InvalidFlightWindow);
        }
        if flight.horizontal_uncertainty_mm > MAX_THRESHOLD_MM
            || flight.vertical_uncertainty_mm > MAX_THRESHOLD_MM
            || !position_in_bounds(flight.position_at_valid_from_mm)
            || !velocity_in_bounds(flight.velocity_mm_s)
            || position_at(flight, flight.valid_from_s).is_none()
            || flight.valid_until_s.checked_sub(1)
                .and_then(|last| position_at(flight, last))
                .is_none()
        {
            errors.push(DynamicConstraintError::InvalidFlightGeometry);
        }
        if !flight.time_qualified {
            errors.push(DynamicConstraintError::UnqualifiedFlightTime);
        }
        if flight.reference_frame.trim().is_empty()
            || flight.vertical_datum.trim().is_empty()
            || flight.time_basis.trim().is_empty()
        {
            errors.push(DynamicConstraintError::UnqualifiedFrame);
        }
        if !matches!(flight.state, IntentState::Withdrawn | IntentState::Ended)
            && case.now_s < flight.valid_until_s
        {
            current_flights.push(flight);
        }
    }
    if !errors.is_empty() {
        reasons.push("one or more flight intents are malformed or lack qualified space-time inputs".to_owned());
        let outcome = dynamic_constraint_error_outcome(&errors);
        return dynamic_constraint_report(
            case, outcome, streams.len(), unique_update_count, replayed_delivery_count,
            active_constraints.len(), current_flights.len(), Vec::new(), errors, reasons,
        );
    }

    let estimated_checks = active_constraints.len().saturating_mul(current_flights.len()) as u128;
    if estimated_checks > MAX_CONSTRAINT_INTERSECTION_CHECKS {
        reasons.push(format!("estimated constraint/trajectory intersection checks ({estimated_checks}) exceeds the bound {MAX_CONSTRAINT_INTERSECTION_CHECKS}"));
        return dynamic_constraint_report(case, DynamicConstraintOutcome::Reject, streams.len(), unique_update_count,
            replayed_delivery_count, active_constraints.len(), current_flights.len(), Vec::new(),
            vec![DynamicConstraintError::WorkBudgetExceeded], reasons);
    }
    let mut impacts = Vec::new();
    let mut intersection_checks = 0_u64;
    for constraint in &active_constraints {
        for flight in &current_flights {
            let start_s=case.now_s.max(flight.valid_from_s).max(constraint.valid_from_s);
            let end_s=flight.valid_until_s.min(constraint.valid_until_s);
            if start_s>=end_s { continue; }
            if flight.reference_frame!=constraint.reference_frame { errors.push(DynamicConstraintError::IncompatibleReferenceFrame); continue; }
            if flight.vertical_datum!=constraint.vertical_datum { errors.push(DynamicConstraintError::IncompatibleVerticalDatum); continue; }
            if flight.time_basis!=constraint.time_basis { errors.push(DynamicConstraintError::IncompatibleTimeBasis); continue; }
            intersection_checks=intersection_checks.saturating_add(1);
            if let Some(first_time)=first_conservative_intersection_time_s(flight,constraint,start_s,end_s,case.uncertainty_horizon_s) {
                impacts.push(DynamicConstraintImpact {
                    flight_id:flight.flight_id.clone(), constraint_id:constraint.constraint_id.clone(),
                    constraint_sequence:constraint.sequence, first_conservative_intersection_time_s:first_time,
                });
            }
        }
    }
    if !errors.is_empty() {
        reasons.push("space-time inputs could not be compared within a common qualified reference/time basis".to_owned());
        return dynamic_constraint_report_with_intersection_checks(
            dynamic_constraint_report(
                case, DynamicConstraintOutcome::Unresolved, streams.len(), unique_update_count,
                replayed_delivery_count, active_constraints.len(), current_flights.len(),
                impacts, errors, reasons,
            ),
            intersection_checks,
        );
    }

    let outcome = if !impacts.is_empty() {
        reasons.push("one or more current flight-intent envelopes intersect an active dynamic constraint; the competent service must reassess and update or withdraw affected authorizations".to_owned());
        DynamicConstraintOutcome::ReassessmentRequired
    } else if case.snapshot_state != ProviderSnapshotState::CompleteFresh {
        reasons.push("constraint view is stale, partitioned, or incomplete; absence of an impact cannot be established".to_owned());
        DynamicConstraintOutcome::Degraded
    } else {
        reasons.push("no impact was found in this complete supplied snapshot under the bounded conservative model; this is not operational clearance".to_owned());
        DynamicConstraintOutcome::NoImpactInProvidedSnapshot
    };

    if replayed_delivery_count > 0 {
        reasons.push(format!("deduplicated {replayed_delivery_count} exact constraint-update retransmission(s)"));
    }
    reasons.push("constraint update identifiers and source labels are synthetic fixture data; no signature or authoritative source is verified".to_owned());
    dynamic_constraint_report_with_intersection_checks(
        dynamic_constraint_report(
            case, outcome, streams.len(), unique_update_count, replayed_delivery_count,
            active_constraints.len(), current_flights.len(), impacts, Vec::new(), reasons,
        ),
        intersection_checks,
    )
}

fn constraint_state_transition_allowed(previous: ConstraintState, next: ConstraintState) -> bool {
    matches!(
        (previous, next),
        (ConstraintState::Active, ConstraintState::Active | ConstraintState::Withdrawn)
            | (ConstraintState::Withdrawn, ConstraintState::Active | ConstraintState::Withdrawn)
    )
}

fn dynamic_constraint_error_outcome(errors: &[DynamicConstraintError]) -> DynamicConstraintOutcome {
    if errors.iter().any(|error| matches!(
        error,
        DynamicConstraintError::EmptyIdentity
            | DynamicConstraintError::TooManyUpdates
            | DynamicConstraintError::TooManyFlights
            | DynamicConstraintError::InvalidUncertaintyHorizon
            | DynamicConstraintError::InvalidSequence
            | DynamicConstraintError::InvalidValidityInterval
            | DynamicConstraintError::InvalidBounds
            | DynamicConstraintError::InvalidMargin
            | DynamicConstraintError::EmptyFlightIdentity
            | DynamicConstraintError::DuplicateFlightIdentity
            | DynamicConstraintError::InvalidFlightWindow
            | DynamicConstraintError::InvalidFlightGeometry
            | DynamicConstraintError::WorkBudgetExceeded
    )) {
        DynamicConstraintOutcome::Reject
    } else {
        DynamicConstraintOutcome::Unresolved
    }
}

fn dynamic_constraint_report(
    case: &DynamicConstraintCase,
    actual_outcome: DynamicConstraintOutcome,
    constraint_stream_count: usize,
    unique_update_count: usize,
    replayed_delivery_count: usize,
    active_constraint_count: usize,
    flights_considered: usize,
    impacts: Vec<DynamicConstraintImpact>,
    errors: Vec<DynamicConstraintError>,
    reasons: Vec<String>,
) -> DynamicConstraintReport {
    DynamicConstraintReport {
        case_id: case.case_id.clone(),
        actual_outcome,
        expected_outcome: case.expected_outcome,
        expectation_matches: actual_outcome == case.expected_outcome,
        received_update_count: case.updates.len(),
        constraint_stream_count,
        unique_update_count,
        replayed_delivery_count,
        active_constraint_count,
        flights_considered,
        intersection_checks: 0,
        impacts,
        errors,
        reasons,
        authority_ceiling: DYNAMIC_CONSTRAINT_AUTHORITY_CEILING,
    }
}

/// Rational elapsed time with positive denominator; bounded inputs keep products in i128.
#[derive(Debug, Clone, Copy)]
struct TimeFraction { numerator: i128, denominator: i128 }
impl TimeFraction {
    fn new(numerator:i128,denominator:i128)->Self { debug_assert!(denominator>0); Self{numerator,denominator} }
    fn compare(self,other:Self)->std::cmp::Ordering { (self.numerator*other.denominator).cmp(&(other.numerator*self.denominator)) }
    fn floor(self)->i128 { self.numerator.div_euclid(self.denominator) }
}
fn axis_intersection_interval(position_at_valid_from:i64,velocity_per_s:i64,minimum:i128,maximum:i128)
    -> Option<(Option<TimeFraction>,Option<TimeFraction>)> {
    let p=i128::from(position_at_valid_from); let v=i128::from(velocity_per_s);
    if v==0 { return (p>=minimum && p<=maximum).then_some((None,None)); }
    if v>0 { Some((Some(TimeFraction::new(minimum-p,v)),Some(TimeFraction::new(maximum-p,v)))) }
    else { let speed=-v; Some((Some(TimeFraction::new(p-maximum,speed)),Some(TimeFraction::new(p-minimum,speed)))) }
}
/// Analytically intersect a constant-velocity line with a conservative
/// axis-aligned box over [start_s,end_s). This removes coarse temporal sample
/// misses for this linear model. The guard band is synthetic, not certified.
fn first_conservative_intersection_time_s(flight:&FlightIntent,constraint:&AirspaceConstraintUpdate,start_s:i64,end_s:i64,uncertainty_horizon_s:i64)->Option<i64> {
    if start_s>=end_s { return None; }
    let p=flight.position_at_valid_from_mm; let v=flight.velocity_mm_s;
    let hm=i128::from(flight.horizontal_uncertainty_mm)+i128::from(constraint.horizontal_margin_mm);
    let vm=i128::from(flight.vertical_uncertainty_mm)+i128::from(constraint.vertical_margin_mm);
    let h=i128::from(uncertainty_horizon_s);
    let mx=hm+i128::from(v.x.unsigned_abs())*h; let my=hm+i128::from(v.y.unsigned_abs())*h; let mz=vm+i128::from(v.z.unsigned_abs())*h;
    let axes=[
        axis_intersection_interval(p.x,v.x,i128::from(constraint.min_position_mm.x)-mx,i128::from(constraint.max_position_mm.x)+mx)?,
        axis_intersection_interval(p.y,v.y,i128::from(constraint.min_position_mm.y)-my,i128::from(constraint.max_position_mm.y)+my)?,
        axis_intersection_interval(p.z,v.z,i128::from(constraint.min_position_mm.z)-mz,i128::from(constraint.max_position_mm.z)+mz)?,
    ];
    let lower=TimeFraction::new(i128::from(start_s)-i128::from(flight.valid_from_s),1);
    let upper=TimeFraction::new(i128::from(end_s)-i128::from(flight.valid_from_s),1);
    let mut enter=lower; let mut leave=upper;
    for (axis_enter,axis_leave) in axes {
        if let Some(t)=axis_enter { if t.compare(enter)==std::cmp::Ordering::Greater { enter=t; } }
        if let Some(t)=axis_leave { if t.compare(leave)==std::cmp::Ordering::Less { leave=t; } }
        if enter.compare(leave)==std::cmp::Ordering::Greater { return None; }
    }
    if enter.compare(upper)!=std::cmp::Ordering::Less || leave.compare(lower)==std::cmp::Ordering::Less { return None; }
    i64::try_from(i128::from(flight.valid_from_s)+enter.floor()).ok()
}

fn dynamic_constraint_report_with_intersection_checks(
    mut report: DynamicConstraintReport,
    intersection_checks: u64,
) -> DynamicConstraintReport {
    report.intersection_checks = intersection_checks;
    report
}

pub fn replay_set(set: &ScenarioSet) -> ReplayOutput {
    let reports: Vec<SimulationReport> = set.scenarios.iter().map(evaluate_scenario).collect();
    let lifecycle_transition_reports: Vec<LifecycleTransitionReport> = set
        .lifecycle_transition_cases
        .iter()
        .map(evaluate_lifecycle_transition_case)
        .collect();
    let reservation_reports: Vec<ReservationCoordinationReport> = set
        .reservation_cases
        .iter()
        .map(evaluate_reservation_case)
        .collect();
    let reservation_event_reports: Vec<ReservationEventCoordinationReport> = set
        .reservation_event_cases
        .iter()
        .map(evaluate_reservation_event_case)
        .collect();
    let dynamic_constraint_reports: Vec<DynamicConstraintReport> = set
        .dynamic_constraint_cases
        .iter()
        .map(evaluate_dynamic_constraint_case)
        .collect();
    let all_expectations_match = reports.iter().all(|item| item.expectation_matches)
        && lifecycle_transition_reports.iter().all(|item| item.expectation_matches)
        && reservation_reports.iter().all(|item| item.expectation_matches)
        && reservation_event_reports.iter().all(|item| item.expectation_matches)
        && dynamic_constraint_reports.iter().all(|item| item.expectation_matches);
    ReplayOutput {
        schema: "airspace-replay-output-v1",
        status: "research_fixture_not_operational",
        input_scenario_count: reports.len(),
        input_lifecycle_transition_case_count: lifecycle_transition_reports.len(),
        input_reservation_case_count: reservation_reports.len(),
        input_reservation_event_case_count: reservation_event_reports.len(),
        input_dynamic_constraint_case_count: dynamic_constraint_reports.len(),
        all_expectations_match,
        reports,
        lifecycle_transition_reports,
        reservation_reports,
        reservation_event_reports,
        dynamic_constraint_reports,
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
    fn replay_fixtures_explicitly_bind_every_flight_to_the_synthetic_model() {
        let set = fixture_set();
        let mut count = 0usize;
        for scenario in &set.scenarios {
            for flight in &scenario.flights {
                assert!(
                    model_applicability::validate_model_profile(
                        &flight.traffic_profile_id,
                        &flight.model_profile_id,
                    ).is_ok(),
                    "scenario {} flight {} must carry the allowed explicit fixture pair",
                    scenario.scenario_id,
                    flight.flight_id
                );
                count += 1;
            }
        }
        for case in &set.dynamic_constraint_cases {
            for flight in &case.flights {
                assert!(
                    model_applicability::validate_model_profile(
                        &flight.traffic_profile_id,
                        &flight.model_profile_id,
                    ).is_ok(),
                    "dynamic case {} flight {} must carry the allowed explicit fixture pair",
                    case.case_id,
                    flight.flight_id
                );
                count += 1;
            }
        }
        assert_eq!(count, 58, "the pinned fixture census changed; review model bindings");
    }

    #[test]
    fn trajectory_evaluator_fails_closed_for_missing_or_unsupported_model_pairs() {
        let mut scenario = fixture_set().scenarios[0].clone();
        let baseline = evaluate_scenario(&scenario);
        assert_eq!(baseline.classification, scenario.expected_classification);

        scenario.flights[0].traffic_profile_id = "conventional_fixed_wing".to_owned();
        scenario.flights[0].model_profile_id =
            model_applicability::CONSTANT_VELOCITY_TOY_MODEL_ID.to_owned();
        let unsupported = evaluate_scenario(&scenario);
        assert_eq!(unsupported.classification, Classification::Unresolved);
        assert_eq!(unsupported.pair_sample_checks, 0);
        assert!(unsupported.conflicts.is_empty());
        assert!(unsupported.reasons.iter().any(|reason| reason.contains("no admissible registry entry")));

        scenario.flights[0].traffic_profile_id.clear();
        let missing = evaluate_scenario(&scenario);
        assert_eq!(missing.classification, Classification::Unresolved);
        assert_eq!(missing.pair_sample_checks, 0);
        assert!(missing.conflicts.is_empty());
        assert!(missing.reasons.iter().any(|reason| reason.contains("traffic_profile_id is missing")));
    }

    #[test]
    fn omitted_model_fields_deserialize_to_unresolved_not_an_implicit_default() {
        let mut value: serde_json::Value = serde_json::from_str(include_str!(
            "../fixtures/scenarios.json"
        )).expect("the scenario fixture must parse as JSON");
        let first_flight = value["scenarios"][0]["flights"][0]
            .as_object_mut()
            .expect("first scenario flight must be an object");
        first_flight.remove("traffic_profile_id");
        first_flight.remove("model_profile_id");

        let set: ScenarioSet = serde_json::from_value(value)
            .expect("missing model fields are retained as blank for fail-closed evaluation");
        let report = evaluate_scenario(&set.scenarios[0]);
        assert_eq!(report.classification, Classification::Unresolved);
        assert_eq!(report.pair_sample_checks, 0);
        assert!(report.conflicts.is_empty());
        assert!(report.reasons.iter().any(|reason| reason.contains("traffic_profile_id is missing")));
    }

    #[test]
    fn every_real_world_archetype_is_unresolved_before_trajectory_work() {
        let catalogue: serde_json::Value = serde_json::from_str(include_str!(
            "../fixtures/traffic_diversity.json"
        )).expect("traffic-diversity catalogue must parse");
        let archetypes = catalogue["archetypes"]
            .as_array()
            .expect("catalogue must contain archetypes");
        let baseline = fixture_set().scenarios[0].clone();

        for archetype in archetypes {
            let profile_id = archetype["profile_id"]
                .as_str()
                .expect("each archetype must have an ID");
            let mut scenario = baseline.clone();
            scenario.flights[0].traffic_profile_id = profile_id.to_owned();
            scenario.flights[0].model_profile_id =
                model_applicability::CONSTANT_VELOCITY_TOY_MODEL_ID.to_owned();

            let report = evaluate_scenario(&scenario);
            assert_eq!(
                report.classification,
                Classification::Unresolved,
                "real-world archetype reached the toy kernel: {profile_id}"
            );
            assert_eq!(report.pair_sample_checks, 0, "geometry ran for {profile_id}");
            assert!(report.conflicts.is_empty(), "geometry emitted conflicts for {profile_id}");
        }
    }

    #[test]
    fn machine_readable_requirements_cover_model_applicability_negative_controls() {
        let requirements: serde_json::Value = serde_json::from_str(include_str!(
            "../../../docs/aerocommons/AIRSPACE_COORDINATION_REQUIREMENTS_V1.json"
        )).expect("airspace requirements contract must parse");
        assert!(requirements["invariants"].as_array().is_some_and(|values| {
            values.iter().any(|item| {
                item["id"] == "AIR-INV-007" && item["hard_gate"] == true
            })
        }));

        let cases = requirements["cases"]
            .as_array()
            .expect("requirements contract must contain cases");
        for (id, expected) in [
            ("AIR-CASE-016", "Unresolved"),
            ("AIR-CASE-017", "Unresolved"),
            ("AIR-CASE-018", "Unresolved"),
        ] {
            let case = cases.iter().find(|item| item["id"] == id)
                .unwrap_or_else(|| panic!("required applicability case is missing: {id}"));
            assert_eq!(case["expected_classification"].as_str(), Some(expected));
            assert!(!case["required_behavior"].as_str().unwrap_or("").trim().is_empty());
            assert!(!case["forbidden_inference"].as_str().unwrap_or("").trim().is_empty());
        }
    }

    #[test]
    fn dynamic_constraint_evaluator_fails_closed_for_unsupported_model_pairs() {
        let mut case = fixture_set()
            .dynamic_constraint_cases
            .into_iter()
            .find(|case| !case.flights.is_empty() && !case.updates.is_empty())
            .expect("at least one executable dynamic-constraint fixture is required");
        case.flights[0].traffic_profile_id = "evtol_aam".to_owned();
        case.flights[0].model_profile_id =
            model_applicability::CONSTANT_VELOCITY_TOY_MODEL_ID.to_owned();

        let report = evaluate_dynamic_constraint_case(&case);
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::Unresolved);
        assert!(report.errors.contains(&DynamicConstraintError::UnsupportedModelProfile));
        assert_eq!(report.intersection_checks, 0);
        assert!(report.impacts.is_empty());
        assert!(report.reasons.iter().any(|reason| reason.contains("no admissible registry entry")));
    }

    #[test]
    fn paradigm_shift_benchmark_contract_is_falsifiable_and_honest_about_status() {
        let manifest: serde_json::Value = serde_json::from_str(include_str!(
            "../fixtures/paradigm_shift_benchmarks.json"
        )).expect("paradigm-shift benchmark manifest must parse");

        assert_eq!(
            manifest["schema"].as_str(),
            Some("airspace-paradigm-shift-benchmarks-v1")
        );
        assert_eq!(
            manifest["status"].as_str(),
            Some("research_design_not_executable_benchmarks")
        );
        assert_eq!(
            manifest["authority_ceiling"].as_str(),
            Some("offline_research_evaluation_only_no_operational_aviation_authority")
        );

        let shared_invariants = manifest["shared_invariants"]
            .as_array()
            .expect("shared invariant vocabulary must be an array");
        let shared_ids: std::collections::BTreeSet<&str> = shared_invariants
            .iter()
            .map(|value| value.as_str().expect("invariant IDs must be strings"))
            .collect();
        assert_eq!(shared_ids.len(), shared_invariants.len());
        assert!(shared_ids.len() >= 7);
        for required in [
            "unsupported_model_never_yields_geometry_clearance",
            "hard_constraints_precede_optimization",
            "stale_or_partitioned_inputs_never_prove_absence_of_impact",
            "producer_output_is_checked_independently",
            "unknown_behavior_is_unresolved_not_defaulted",
            "contingency_optionality_is_separate_from_nominal_throughput",
            "recovery_capacity_never_relaxes_hard_constraints",
            "resilience_and_access_metrics_are_not_collapsed_into_one_safety_score",
            "allocation_priority_comes_only_from_authorized_policy",
            "access_outcomes_are_audited_by_declared_group",
            "operator_identity_or_reputation_is_not_an_implicit_priority_rule",
            "uncertainty_composition_preserves_dependencies_and_bounds",
            "heterogeneous_uncertainty_requires_explicit_dimensional_projection",
            "unrecognized_contract_fields_fail_closed",
            "assurance_independence_is_evidenced_not_declared",
        ] {
            assert!(shared_ids.contains(required), "shared invariant is missing: {required}");
        }

        let benchmarks = manifest["benchmarks"]
            .as_array()
            .expect("benchmark cases must be an array");
        assert_eq!(benchmarks.len(), 26);
        let mut ids = std::collections::BTreeSet::new();
        let allowed_statuses = [
            "implemented_pending_exact_head_verification",
            "partial_foundations_exist_not_fully_qualified",
            "research_backlog",
        ];
        for benchmark in benchmarks {
            let id = benchmark["id"].as_str().expect("benchmark needs ID");
            assert!(ids.insert(id), "duplicate paradigm benchmark ID: {id}");
            for field in ["name", "paradigm", "hypothesis", "baseline", "stimulus"] {
                assert!(
                    !benchmark[field].as_str().unwrap_or("").trim().is_empty(),
                    "benchmark {id} needs a non-empty {field}"
                );
            }
            let status = benchmark["status"].as_str().expect("benchmark needs status");
            assert!(
                allowed_statuses.contains(&status),
                "benchmark {id} has unrecognized status {status}"
            );
            for field in ["success_conditions", "falsifiers", "required_evidence", "hard_invariants"] {
                let values = benchmark[field]
                    .as_array()
                    .unwrap_or_else(|| panic!("benchmark {id} needs array {field}"));
                assert!(!values.is_empty(), "benchmark {id} must define {field}");
                for value in values {
                    let value = value.as_str()
                        .unwrap_or_else(|| panic!("benchmark {id} {field} entries must be strings"));
                    assert!(!value.trim().is_empty(), "benchmark {id} {field} cannot contain blank entries");
                }
            }
            for invariant in benchmark["hard_invariants"].as_array().unwrap() {
                assert!(
                    shared_ids.contains(invariant.as_str().unwrap()),
                    "benchmark {id} references unknown shared invariant {invariant}"
                );
            }
        }
        for required in [
            "AIR-PB-001", "AIR-PB-002", "AIR-PB-003", "AIR-PB-004",
            "AIR-PB-005", "AIR-PB-006", "AIR-PB-007", "AIR-PB-008",
            "AIR-PB-009", "AIR-PB-010", "AIR-PB-011", "AIR-PB-012",
            "AIR-PB-013", "AIR-PB-014", "AIR-PB-015", "AIR-PB-016", "AIR-PB-017", "AIR-PB-018", "AIR-PB-019", "AIR-PB-020", "AIR-PB-021", "AIR-PB-022", "AIR-PB-023", "AIR-PB-024", "AIR-PB-025", "AIR-PB-026",
        ] {
            assert!(ids.contains(required), "required research case missing: {required}");
        }
        let admission = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-001")
            .expect("model-admission benchmark must exist");
        assert_eq!(
            admission["status"].as_str(),
            Some("implemented_pending_exact_head_verification")
        );

        let uncertainty_prototype = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-017")
            .expect("bounded affine uncertainty prototype benchmark must exist");
        assert_eq!(
            uncertainty_prototype["status"].as_str(),
            Some("implemented_pending_exact_head_verification")
        );
        assert_eq!(
            uncertainty_prototype["scope"].as_str(),
            Some("test_only_scalar_additive_integer_affine_error_form; not flight dynamics, multi-axis geometry, a probability model, a validated reachable set, or operational qualification")
        );

        let dependency_identity = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-018")
            .expect("origin/dependency identity benchmark must exist");
        assert_eq!(
            dependency_identity["status"].as_str(),
            Some("implemented_pending_exact_head_verification")
        );
        assert!(
            dependency_identity["scope"].as_str().unwrap_or("").contains(
                "dependency IDs are declared claims, not authenticated correlation evidence"
            )
        );

        let typed_contract = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-019")
            .expect("typed, time-qualified dependency contract benchmark must exist");
        assert_eq!(
            typed_contract["status"].as_str(),
            Some("implemented_pending_exact_head_verification")
        );
        assert!(
            typed_contract["scope"].as_str().unwrap_or("").contains(
                "provenance remains declared_unverified and unauthenticated"
            )
        );

        let probability_contract = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-020")
            .expect("set-containment versus probability benchmark must exist");
        assert_eq!(
            probability_contract["status"].as_str(),
            Some("research_backlog")
        );
        assert!(
            probability_contract["scope"].as_str().unwrap_or("").contains(
                "no operational risk threshold"
            )
        );


        let metamorphic_contract = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-025")
            .expect("typed-lineage metamorphic benchmark must exist");
        assert_eq!(
            metamorphic_contract["status"].as_str(),
            Some("implemented_pending_exact_head_verification")
        );
        assert!(
            metamorphic_contract["stimulus"].as_str().unwrap_or("").contains("500 seeded typed domains"),
            "AIR-PB-025 must pin its generated-domain scale"
        );
        assert!(
            metamorphic_contract["success_conditions"].as_array().unwrap().iter()
                .any(|condition| condition.as_str().unwrap_or("").contains("widenings produce an output interval")),
            "AIR-PB-025 must preserve the interval-widening metamorphic relation"
        );

        let adapter_contract = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-026")
            .expect("versioned typed-lineage adapter benchmark must exist");
        assert_eq!(
            adapter_contract["status"].as_str(),
            Some("implemented_pending_exact_head_verification")
        );
        assert!(
            adapter_contract["stimulus"].as_str().unwrap_or("").contains("500 seeded generated domains"),
            "AIR-PB-026 must pin its generated adapter corpus scale"
        );
        assert!(
            adapter_contract["success_conditions"].as_array().unwrap().iter()
                .any(|condition| condition.as_str().unwrap_or("").contains("explicit null remains distinct")),
            "AIR-PB-026 must distinguish missing metadata from explicit null"
        );
        assert!(
            adapter_contract["success_conditions"].as_array().unwrap().iter()
                .any(|condition| condition.as_str().unwrap_or("").contains("numeric interval is unchanged")),
            "AIR-PB-026 must detect metadata drift even when numeric bounds are unchanged"
        );

        let receipt_contract = benchmarks.iter()
            .find(|benchmark| benchmark["id"] == "AIR-PB-021")
            .expect("content-addressed replay receipt benchmark must exist");
        assert_eq!(
            receipt_contract["status"].as_str(),
            Some("implemented_pending_exact_head_verification")
        );
        assert!(
            receipt_contract["scope"].as_str().unwrap_or("").contains(
                "not a signed attestation"
            )
        );
        assert!(
            benchmarks.iter().all(|benchmark| {
                benchmark["status"] != "passed"
                    && benchmark["status"] != "operationally_validated"
            }),
            "the research manifest must not manufacture qualification claims"
        );
    }

    #[test]
    fn traffic_diversity_catalogue_is_unique_explicit_and_fail_closed() {
        let catalogue: serde_json::Value =
            serde_json::from_str(include_str!("../fixtures/traffic_diversity.json"))
                .expect("traffic diversity catalogue must be valid JSON");

        assert_eq!(
            catalogue["schema"].as_str(),
            Some("airspace-traffic-diversity-v1")
        );
        assert_eq!(
            catalogue["status"].as_str(),
            Some("synthetic_research_metadata_only")
        );
        assert_eq!(
            catalogue["current_kernel"]["supports_real_vehicle_dynamics"].as_bool(),
            Some(false)
        );
        assert_eq!(
            catalogue["current_kernel"]["separation_minima"].as_str(),
            Some("not_provided")
        );

        let archetypes = catalogue["archetypes"]
            .as_array()
            .expect("archetypes array exists");
        assert_eq!(archetypes.len(), 15);
        let mut profile_ids = std::collections::BTreeSet::new();
        for profile in archetypes {
            let profile_id = profile["profile_id"]
                .as_str()
                .expect("every archetype needs an ID");
            assert!(
                profile_ids.insert(profile_id),
                "duplicate traffic profile ID: {profile_id}"
            );
            assert_eq!(
                profile["current_kernel_support"].as_str(),
                Some("not_implemented"),
                "real vehicle archetype must not imply the toy kernel models it: {profile_id}"
            );
            assert_eq!(
                profile["operational_claims"].as_bool(),
                Some(false),
                "traffic catalogue cannot make operational claims: {profile_id}"
            );
            assert!(
                profile["examples"].as_array().is_some_and(|values| !values.is_empty()),
                "examples must be explicit: {profile_id}"
            );
            assert!(
                profile["distinguishing_factors"]
                    .as_array()
                    .is_some_and(|values| !values.is_empty()),
                "model requirements must be explicit: {profile_id}"
            );
            assert_eq!(
                profile["required_model_or_adapter"].as_str(),
                Some("not_yet_defined"),
                "do not advertise an unevidenced model adapter: {profile_id}"
            );
        }

        for required in [
            "conventional_fixed_wing",
            "rotorcraft",
            "glider_unpowered",
            "low_altitude_multirotor_uas",
            "fixed_wing_uas",
            "large_uncrewed_cargo",
            "evtol_aam",
            "public_service_specialized",
            "balloon_airship",
            "haps_stratospheric",
            "hale_remotely_piloted",
            "supersonic_hypersonic",
            "launch_reentry_suborbital",
            "unknown_noncooperative",
            "cooperative_swarm_aggregate",
        ] {
            assert!(
                profile_ids.contains(required),
                "required traffic archetype missing: {required}"
            );
        }

        for collection_name in ["mission_contexts", "operating_regimes", "cooperation_states"] {
            let values = catalogue[collection_name]
                .as_array()
                .unwrap_or_else(|| panic!("{collection_name} must be an array"));
            let mut unique = std::collections::BTreeSet::new();
            for value in values {
                let value = value.as_str()
                    .unwrap_or_else(|| panic!("{collection_name} entries must be strings"));
                assert!(!value.trim().is_empty(), "{collection_name} cannot contain blank values");
                assert!(unique.insert(value), "duplicate {collection_name} value: {value}");
            }
        }

        let interactions = catalogue["interaction_backlog"]
            .as_array()
            .expect("interaction backlog array exists");
        assert_eq!(interactions.len(), 12);
        let mut case_ids = std::collections::BTreeSet::new();
        for case in interactions {
            let case_id = case["case_id"].as_str().expect("interaction case needs ID");
            assert!(case_ids.insert(case_id), "duplicate interaction case ID: {case_id}");
            assert!(
                !case["research_question"].as_str().unwrap_or("").trim().is_empty(),
                "interaction case needs a research question: {case_id}"
            );
            assert_eq!(
                case["status"].as_str(),
                Some("design_backlog_not_executable"),
                "do not count a design descriptor as an executable scenario: {case_id}"
            );
            let participants = case["participants"]
                .as_array()
                .expect("interaction participants must be an array");
            assert!(!participants.is_empty(), "interaction case needs participants: {case_id}");
            for participant in participants {
                let participant = participant.as_str()
                    .expect("participant profile ID must be a string");
                assert!(
                    profile_ids.contains(participant),
                    "unknown traffic profile {participant} in {case_id}"
                );
            }
        }
    }

    #[test]
    fn all_committed_scenario_expectations_match() {
        let set = fixture_set();
        let output = replay_set(&set);
        assert_eq!(output.status, "research_fixture_not_operational");
        assert_eq!(output.input_scenario_count, set.scenarios.len());
        assert_eq!(output.input_lifecycle_transition_case_count, set.lifecycle_transition_cases.len());
        assert_eq!(output.input_reservation_case_count, set.reservation_cases.len());
        assert_eq!(output.input_reservation_event_case_count, set.reservation_event_cases.len());
        assert_eq!(output.input_dynamic_constraint_case_count, set.dynamic_constraint_cases.len());
        assert!(
            output.all_expectations_match,
            "mismatched traffic reports: {:#?}; mismatched lifecycle reports: {:#?}; mismatched reservation reports: {:#?}; mismatched reservation-event reports: {:#?}; mismatched dynamic-constraint reports: {:#?}",
            output.reports.iter().filter(|report| !report.expectation_matches).collect::<Vec<_>>(),
            output.lifecycle_transition_reports
                .iter()
                .filter(|report| !report.expectation_matches)
                .collect::<Vec<_>>(),
            output.reservation_reports
                .iter()
                .filter(|report| !report.expectation_matches)
                .collect::<Vec<_>>(),
            output.reservation_event_reports
                .iter()
                .filter(|report| !report.expectation_matches)
                .collect::<Vec<_>>(),
            output.dynamic_constraint_reports
                .iter()
                .filter(|report| !report.expectation_matches)
                .collect::<Vec<_>>()
        );
    }



    #[test]
    fn multi_aircraft_reports_preserve_unique_pair_attribution() {
        let set = fixture_set();
        for scenario in &set.scenarios {
            let report = evaluate_scenario(scenario);
            let expected_pair_count = scenario.flights.len()
                .saturating_mul(scenario.flights.len().saturating_sub(1))
                / 2;
            assert_eq!(
                report.pair_count, expected_pair_count,
                "pair count mismatch for {}",
                scenario.scenario_id
            );

            let flight_ids: std::collections::BTreeSet<&str> = scenario
                .flights
                .iter()
                .map(|flight| flight.flight_id.as_str())
                .collect();
            let mut attributed_pairs = std::collections::BTreeSet::<(&str, &str)>::new();
            for conflict in &report.conflicts {
                assert_ne!(
                    conflict.flight_a, conflict.flight_b,
                    "self-conflict attribution in {}",
                    scenario.scenario_id
                );
                assert!(
                    flight_ids.contains(conflict.flight_a.as_str())
                        && flight_ids.contains(conflict.flight_b.as_str()),
                    "conflict references an unknown flight in {}: {conflict:#?}",
                    scenario.scenario_id
                );
                let pair = if conflict.flight_a.as_str() < conflict.flight_b.as_str() {
                    (conflict.flight_a.as_str(), conflict.flight_b.as_str())
                } else {
                    (conflict.flight_b.as_str(), conflict.flight_a.as_str())
                };
                assert!(
                    attributed_pairs.insert(pair),
                    "duplicate conflict pair in {}: {pair:?}",
                    scenario.scenario_id
                );
            }
        }
    }

    #[test]
    fn three_aircraft_fixture_reports_every_conflicting_pair_once() {
        let set = fixture_set();
        let scenario = set.scenarios
            .iter()
            .find(|scenario| scenario.scenario_id == "three_aircraft_multiple_pair_conflicts")
            .expect("multi-pair fixture exists");
        let report = evaluate_scenario(scenario);

        assert_eq!(report.classification, Classification::Conflict);
        assert_eq!(report.pair_count, 3);
        assert_eq!(report.conflicts.len(), 3);
        let pairs: std::collections::BTreeSet<(&str, &str)> = report.conflicts
            .iter()
            .map(|conflict| {
                if conflict.flight_a.as_str() < conflict.flight_b.as_str() {
                    (conflict.flight_a.as_str(), conflict.flight_b.as_str())
                } else {
                    (conflict.flight_b.as_str(), conflict.flight_a.as_str())
                }
            })
            .collect();
        assert_eq!(pairs.len(), 3);
        assert!(pairs.contains(&("A", "B")));
        assert!(pairs.contains(&("A", "C")));
        assert!(pairs.contains(&("B", "C")));
    }

    #[test]
    fn four_aircraft_clear_fixture_checks_all_six_pairs_without_conflicts() {
        let set = fixture_set();
        let scenario = set.scenarios
            .iter()
            .find(|scenario| scenario.scenario_id == "four_aircraft_parallel_lanes_clear")
            .expect("four-aircraft clear fixture exists");
        let report = evaluate_scenario(scenario);

        assert_eq!(report.classification, Classification::AcceptForPlanningOnly);
        assert_eq!(report.pair_count, 6);
        assert!(report.conflicts.is_empty());
    }

    #[test]
    fn seeded_random_coarse_sampling_never_misses_integer_time_conflicts() {
        fn draw(seed: &mut u64) -> u64 {
            *seed = seed.wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *seed
        }

        fn between(seed: &mut u64, low: i64, high: i64) -> i64 {
            let width = u64::try_from(high - low + 1).expect("positive random range");
            low + i64::try_from(draw(seed) % width).expect("bounded draw")
        }

        let set = fixture_set();
        let template = set.scenarios
            .iter()
            .find(|scenario| scenario.scenario_id == "clear_separation")
            .expect("clear template exists");
        let mut seed = 0xA17C_0A57_2026_1015_u64;
        let mut oracle_conflicts = 0_usize;
        let mut oracle_clear = 0_usize;

        // A deterministic, dependency-free randomized regression corpus. The
        // oracle inspects every integer second with the unexpanded thresholds;
        // the evaluator may conservatively report additional conflicts.
        for case_index in 0..512 {
            let mut scenario = template.clone();
            scenario.scenario_id = format!("seeded_integer_oracle_{case_index:04}");
            scenario.window_start_s = 0;
            scenario.window_end_s = 30;
            scenario.policy.sample_step_s = between(&mut seed, 2, 8);
            scenario.policy.horizontal_separation_mm =
                u64::try_from(between(&mut seed, 500, 3_500)).expect("positive threshold");
            scenario.policy.vertical_separation_mm =
                u64::try_from(between(&mut seed, 500, 3_500)).expect("positive threshold");

            let mut flights: Vec<FlightIntent> =
                template.flights.iter().take(2).cloned().collect();
            for (index, flight) in flights.iter_mut().enumerate() {
                flight.flight_id = format!("P{index}");
                flight.valid_from_s = 0;
                flight.valid_until_s = 30;
                flight.position_at_valid_from_mm = PositionMm {
                    x: between(&mut seed, -10_000, 10_000),
                    y: between(&mut seed, -10_000, 10_000),
                    z: between(&mut seed, 80_000, 120_000),
                };
                flight.velocity_mm_s = VelocityMmS {
                    x: between(&mut seed, -500, 500),
                    y: between(&mut seed, -500, 500),
                    z: between(&mut seed, -100, 100),
                };
                flight.horizontal_uncertainty_mm = 0;
                flight.vertical_uncertainty_mm = 0;
                flight.time_qualified = true;
                flight.state = IntentState::Activated;
                flight.priority = FlightPriority::Routine;
            }
            scenario.flights = flights;

            let left = &scenario.flights[0];
            let right = &scenario.flights[1];
            let horizontal_limit = i128::from(scenario.policy.horizontal_separation_mm);
            let vertical_limit = i128::from(scenario.policy.vertical_separation_mm);
            let mut oracle_conflict = false;

            for time_s in scenario.window_start_s..=scenario.window_end_s {
                let left_elapsed = i128::from(time_s - left.valid_from_s);
                let right_elapsed = i128::from(time_s - right.valid_from_s);
                let left_x = i128::from(left.position_at_valid_from_mm.x)
                    + i128::from(left.velocity_mm_s.x) * left_elapsed;
                let left_y = i128::from(left.position_at_valid_from_mm.y)
                    + i128::from(left.velocity_mm_s.y) * left_elapsed;
                let left_z = i128::from(left.position_at_valid_from_mm.z)
                    + i128::from(left.velocity_mm_s.z) * left_elapsed;
                let right_x = i128::from(right.position_at_valid_from_mm.x)
                    + i128::from(right.velocity_mm_s.x) * right_elapsed;
                let right_y = i128::from(right.position_at_valid_from_mm.y)
                    + i128::from(right.velocity_mm_s.y) * right_elapsed;
                let right_z = i128::from(right.position_at_valid_from_mm.z)
                    + i128::from(right.velocity_mm_s.z) * right_elapsed;
                let dx = left_x - right_x;
                let dy = left_y - right_y;
                let dz = (left_z - right_z).abs();

                if dx * dx + dy * dy <= horizontal_limit * horizontal_limit
                    && dz <= vertical_limit
                {
                    oracle_conflict = true;
                    break;
                }
            }

            let report = evaluate_scenario(&scenario);
            if oracle_conflict {
                oracle_conflicts += 1;
                assert_eq!(
                    report.classification,
                    Classification::Conflict,
                    "coarse sampling missed an integer-time conflict in {}: {report:#?}",
                    scenario.scenario_id
                );
            } else {
                oracle_clear += 1;
            }
        }

        assert!(oracle_conflicts > 0, "seed must exercise conflict cases");
        assert!(oracle_clear > 0, "seed must exercise no-conflict integer oracles");
    }

    #[test]
    fn duplicate_trajectory_flight_identity_is_rejected_before_pairwise_checks() {
        let scenario = fixture_set()
            .scenarios
            .into_iter()
            .find(|scenario| scenario.scenario_id == "duplicate_flight_identity_rejected")
            .expect("duplicate flight identity regression fixture exists");

        let report = evaluate_scenario(&scenario);
        assert_eq!(report.classification, Classification::Reject);
        assert_eq!(report.pair_sample_checks, 0);
        assert!(
            report.reasons.iter().any(|reason| reason.contains("duplicate flight_id")),
            "expected explicit duplicate-identity reason: {report:#?}"
        );
    }

    fn dynamic_constraint_fixture(case_id: &str) -> DynamicConstraintCase {
        fixture_set()
            .dynamic_constraint_cases
            .into_iter()
            .find(|case| case.case_id == case_id)
            .expect("dynamic constraint case exists")
    }

    #[test]
    fn dynamic_constraint_fixtures_match_expected_outcomes() {
        let set = fixture_set();
        let reports: Vec<_> = set.dynamic_constraint_cases
            .iter()
            .map(evaluate_dynamic_constraint_case)
            .collect();
        assert!(
            reports.iter().all(|report| report.expectation_matches),
            "mismatched dynamic-constraint reports: {:#?}",
            reports.iter().filter(|report| !report.expectation_matches).collect::<Vec<_>>()
        );
    }


    #[test]
    fn constraint_update_retransmission_is_idempotent_and_counted() {
        let report = evaluate_dynamic_constraint_case(
            &dynamic_constraint_fixture("exact_constraint_update_retransmission_idempotent"),
        );
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::NoImpactInProvidedSnapshot);
        assert_eq!(report.constraint_stream_count, 1);
        assert_eq!(report.unique_update_count, 1);
        assert_eq!(report.replayed_delivery_count, 1);
        assert!(report.intersection_checks > 0);
    }

    #[test]
    fn conflicting_constraint_update_digest_and_source_changes_fail_closed() {
        for (case_id, expected_error) in [
            ("same_constraint_digest_different_payload_unresolved", DynamicConstraintError::ConflictingUpdateDigest),
            ("constraint_authority_change_unresolved", DynamicConstraintError::AuthorityChanged),
            ("constraint_vertical_datum_mismatch_unresolved", DynamicConstraintError::IncompatibleVerticalDatum),
        ] {
            let report = evaluate_dynamic_constraint_case(&dynamic_constraint_fixture(case_id));
            assert_eq!(report.actual_outcome, DynamicConstraintOutcome::Unresolved, "{case_id}: {report:#?}");
            assert!(report.expectation_matches, "{case_id}: fixture expectation mismatch");
            assert!(report.errors.contains(&expected_error), "{case_id}: missing {expected_error:?}");
        }
    }

    #[test]
    fn dynamic_constraint_invalid_sample_step_is_rejected() {
        let report = evaluate_dynamic_constraint_case(&dynamic_constraint_fixture("invalid_constraint_sample_step_rejected"));
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::Reject);
        assert!(report.errors.contains(&DynamicConstraintError::InvalidUncertaintyHorizon));
    }

    #[test]
    fn duplicate_dynamic_flight_identity_is_rejected_before_intersection() {
        let mut case = dynamic_constraint_fixture("new_restriction_intersects_accepted_intent");
        let duplicate = case.flights[0].clone();
        case.flights.push(duplicate);

        let report = evaluate_dynamic_constraint_case(&case);
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::Reject);
        assert!(report.errors.contains(&DynamicConstraintError::DuplicateFlightIdentity));
        assert_eq!(report.intersection_checks, 0);
    }

    #[test]
    fn continuous_intersection_detects_crossing_between_integer_seconds() {
        let report = evaluate_dynamic_constraint_case(
            &dynamic_constraint_fixture("continuous_intersection_catches_between_integer_seconds"),
        );
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::ReassessmentRequired);
        assert_eq!(report.impacts[0].first_conservative_intersection_time_s, 4);
        assert!(report.intersection_checks > 0);
    }

    #[test]
    fn continuous_intersection_honors_tangency_and_half_open_end() {
        let tangent = evaluate_dynamic_constraint_case(
            &dynamic_constraint_fixture("continuous_spatial_tangent_at_start_is_impact"),
        );
        assert_eq!(tangent.actual_outcome, DynamicConstraintOutcome::ReassessmentRequired);
        let end_boundary = evaluate_dynamic_constraint_case(
            &dynamic_constraint_fixture("constraint_valid_until_boundary_is_exclusive"),
        );
        assert_eq!(end_boundary.actual_outcome, DynamicConstraintOutcome::NoImpactInProvidedSnapshot);
        assert!(end_boundary.impacts.is_empty());
    }

    #[test]
    fn newly_published_restriction_requires_reassessment() {
        let report = evaluate_dynamic_constraint_case(
            &dynamic_constraint_fixture("new_restriction_intersects_accepted_intent"),
        );
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::ReassessmentRequired);
        assert_eq!(report.impacts.len(), 1);
        assert_eq!(report.impacts[0].first_conservative_intersection_time_s, 5);
        assert!(report.authority_ceiling.contains("does not update/withdraw authorizations"));
    }

    #[test]
    fn withdrawal_removes_constraint_from_new_snapshot_but_does_not_grant_authority() {
        let report = evaluate_dynamic_constraint_case(
            &dynamic_constraint_fixture("withdrawn_constraint_no_longer_intersects"),
        );
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::NoImpactInProvidedSnapshot);
        assert!(report.impacts.is_empty());
        assert!(report.authority_ceiling.contains("does not update/withdraw authorizations"));
    }

    #[test]
    fn stale_view_cannot_hide_a_known_constraint_impact() {
        let report = evaluate_dynamic_constraint_case(
            &dynamic_constraint_fixture("stale_snapshot_known_impact_still_requires_reassessment"),
        );
        assert_eq!(report.actual_outcome, DynamicConstraintOutcome::ReassessmentRequired);
        assert!(!report.impacts.is_empty());
    }

    #[test]
    fn constraint_history_negative_controls_fail_closed() {
        let cases = [
            ("constraint_sequence_gap_unresolved", DynamicConstraintOutcome::Unresolved, DynamicConstraintError::SequenceGap),
            ("constraint_fork_unresolved", DynamicConstraintOutcome::Unresolved, DynamicConstraintError::ForkedSequence),
            ("malformed_constraint_bounds_rejected", DynamicConstraintOutcome::Reject, DynamicConstraintError::InvalidBounds),
            ("constraint_frame_mismatch_unresolved", DynamicConstraintOutcome::Unresolved, DynamicConstraintError::IncompatibleReferenceFrame),
            ("future_constraint_update_unresolved", DynamicConstraintOutcome::Unresolved, DynamicConstraintError::FutureIssuedAt),
        ];
        for (case_id, expected, error) in cases {
            let report = evaluate_dynamic_constraint_case(&dynamic_constraint_fixture(case_id));
            assert_eq!(report.actual_outcome, expected, "{case_id}: {report:#?}");
            assert!(report.expectation_matches, "{case_id}: fixture expectation mismatch");
            assert!(report.errors.contains(&error), "{case_id}: missing {error:?}");
        }
    }

    fn reservation_event_fixture(case_id: &str) -> ReservationEventCoordinationCase {
        fixture_set()
            .reservation_event_cases
            .into_iter()
            .find(|case| case.case_id == case_id)
            .expect("reservation event case exists")
    }

    #[test]
    fn reservation_event_stream_fixtures_match_expected_outcomes() {
        let set = fixture_set();
        let reports: Vec<_> = set.reservation_event_cases
            .iter()
            .map(evaluate_reservation_event_case)
            .collect();
        assert!(
            reports.iter().all(|report| report.expectation_matches),
            "mismatched reservation-event reports: {:#?}",
            reports.iter().filter(|report| !report.expectation_matches).collect::<Vec<_>>()
        );
    }

    #[test]
    fn additional_event_stream_negative_controls_fail_closed() {
        let cases = [
            ("same_digest_conflicting_payload_unresolved", ReservationOutcome::Unresolved, ReservationEventError::ConflictingEventDigest),
            ("reservation_semantics_mutation_unresolved", ReservationOutcome::Unresolved, ReservationEventError::ReservationSemanticsChanged),
            ("event_issue_time_reversal_unresolved", ReservationOutcome::Unresolved, ReservationEventError::TimeReversal),
            ("missing_initial_prefix_unresolved", ReservationOutcome::Unresolved, ReservationEventError::SequenceGap),
            ("malformed_event_interval_rejected", ReservationOutcome::Reject, ReservationEventError::InvalidInterval),
        ];
        for (case_id, expected_outcome, expected_error) in cases {
            let report = evaluate_reservation_event_case(&reservation_event_fixture(case_id));
            assert_eq!(report.actual_outcome, expected_outcome, "{case_id}: {report:#?}");
            assert!(report.expectation_matches, "{case_id}: fixture expectation mismatch");
            assert!(report.errors.contains(&expected_error), "{case_id}: missing {expected_error:?}");
        }
    }

    #[test]
    fn event_arrival_order_does_not_change_reconciled_projection() {
        let mut case = reservation_event_fixture("reordered_updates_converge_no_conflict");
        let original = evaluate_reservation_event_case(&case);
        case.events.reverse();
        let reversed = evaluate_reservation_event_case(&case);
        assert_eq!(original.actual_outcome, reversed.actual_outcome);
        assert_eq!(original.current_claim_count, reversed.current_claim_count);
        assert_eq!(original.unique_event_count, reversed.unique_event_count);
    }

    #[test]
    fn reservation_event_forks_and_gaps_fail_closed() {
        for case_id in ["sequence_gap_unresolved", "same_sequence_fork_unresolved", "predecessor_mismatch_unresolved"] {
            let report = evaluate_reservation_event_case(&reservation_event_fixture(case_id));
            assert_eq!(report.actual_outcome, ReservationOutcome::Unresolved, "{case_id}");
            assert!(report.expectation_matches, "{case_id}: {report:#?}");
            assert!(!report.errors.is_empty(), "{case_id}: missing expected error");
        }
    }

    #[test]
    fn exact_event_retransmission_is_idempotent_but_reported() {
        let report = evaluate_reservation_event_case(&reservation_event_fixture("exact_retransmission_is_idempotent"));
        assert_eq!(report.actual_outcome, ReservationOutcome::NoConflictInProvidedSnapshot);
        assert_eq!(report.replayed_delivery_count, 1);
        assert!(report.reasons.iter().any(|reason| reason.contains("deduplicated")));
    }


    fn reservation_fixture(case_id: &str) -> ReservationCoordinationCase {
        fixture_set()
            .reservation_cases
            .into_iter()
            .find(|case| case.case_id == case_id)
            .expect("reservation case exists")
    }

    #[test]
    fn oversized_reservation_snapshot_is_rejected_before_pairwise_work() {
        let mut case = reservation_fixture("concurrent_overlap_conflict");
        let template = case.claims[0].clone();
        case.claims.resize(MAX_RESERVATION_CLAIMS + 1, template);
        assert_eq!(evaluate_reservation_case(&case).actual_outcome, ReservationOutcome::Reject);
    }

    #[test]
    fn half_open_reservations_can_be_adjacent_without_conflict() {
        let case = reservation_fixture("half_open_adjacent_no_conflict");
        let report = evaluate_reservation_case(&case);
        assert_eq!(report.actual_outcome, ReservationOutcome::NoConflictInProvidedSnapshot);
        assert!(report.conflicts.is_empty());
    }

    #[test]
    fn partitioned_snapshot_cannot_claim_no_conflict() {
        let case = reservation_fixture("partitioned_snapshot_degraded");
        let report = evaluate_reservation_case(&case);
        assert_eq!(report.actual_outcome, ReservationOutcome::Degraded);
        assert!(report.authority_ceiling.contains("no reservation grant"));
    }

    #[test]
    fn duplicate_reservation_versions_require_explicit_reconciliation() {
        let case = reservation_fixture("stale_update_duplicate_version_unresolved");
        let report = evaluate_reservation_case(&case);
        assert_eq!(report.actual_outcome, ReservationOutcome::Unresolved);
        assert!(report.errors.contains(&ReservationError::DuplicateReservationId));
    }

    #[test]
    fn emergency_claim_does_not_automatically_preempt_a_conflicting_claim() {
        let case = reservation_fixture("emergency_does_not_preempt_conflict");
        let report = evaluate_reservation_case(&case);
        assert_eq!(report.actual_outcome, ReservationOutcome::Conflict);
        assert_eq!(report.conflicts.len(), 1);
        assert!(report.reasons.iter().any(|reason| reason.contains("does not automatically preempt")));
    }

    #[test]
    fn expired_and_withdrawn_claims_are_not_current_reservations() {
        for case_id in ["expired_claim_does_not_block", "withdrawn_claim_does_not_block"] {
            let report = evaluate_reservation_case(&reservation_fixture(case_id));
            assert_eq!(report.actual_outcome, ReservationOutcome::NoConflictInProvidedSnapshot);
        }
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
        assert_eq!(report.conflicts[0].first_conservative_intersection_time_s, 0);
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
