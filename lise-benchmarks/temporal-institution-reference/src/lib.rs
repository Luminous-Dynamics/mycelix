//! Finite-state temporal reference model for institutional capture and recovery.
//!
//! This is a bounded research model, not a production governance engine,
//! credential service, policy engine, formal proof of a real institution,
//! or legal certification. It makes a small transition system explicit and
//! preserves shortest counterexample traces.
//!
//! The model is deliberately conservative: incomplete control inventory is
//! Indeterminate; captured oversight is ChallengeRequired; hard recovery
//! invariant failures are Deny.

use serde::{Deserialize, Serialize};
use std::collections::{HashSet, VecDeque};

const CAP_APPOINTMENTS: u8 = 1 << 0;
const CAP_AGENDA: u8 = 1 << 1;
const CAP_TREASURY: u8 = 1 << 2;
const CAP_CREDENTIALS: u8 = 1 << 3;
const CAP_EXECUTION: u8 = 1 << 4;
const CAP_OVERSIGHT: u8 = 1 << 5;
const CAPTURE_RELEVANT_TO_OVERSIGHT: u8 =
    CAP_APPOINTMENTS | CAP_TREASURY | CAP_CREDENTIALS | CAP_EXECUTION;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Disposition {
    Permit,
    Deny,
    Hold,
    Indeterminate,
    ChallengeRequired,
}

impl Disposition {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Permit => "permit",
            Self::Deny => "deny",
            Self::Hold => "hold",
            Self::Indeterminate => "indeterminate",
            Self::ChallengeRequired => "challenge_required",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ServiceMode {
    Normal,
    DegradedReadOnly,
    Offline,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Action {
    CaptureAppointments,
    CaptureAgenda,
    CaptureTreasury,
    CaptureCredentials,
    CaptureExecution,
    CaptureOversight,
    MarkInventoryIncomplete,
    RestoreIndependentOversight,
    FounderUnavailable,
    VerifyRecoveryTrigger,
    ReachRecoveryQuorum,
    FenceOldSessions,
    RestoreDegradedReadOnly,
    RestoreOldAdministrator,
    ChangeConstitution,
    RevalidateCredentials,
    EnableHighImpact,
    DisableHighImpact,
    AdvanceTime,
    GrantEmergencyAuthority,
    RevokeEmergencyAuthority,
}

impl Action {
    pub const ALL: [Self; 21] = [
        Self::CaptureAppointments,
        Self::CaptureAgenda,
        Self::CaptureTreasury,
        Self::CaptureCredentials,
        Self::CaptureExecution,
        Self::CaptureOversight,
        Self::MarkInventoryIncomplete,
        Self::RestoreIndependentOversight,
        Self::FounderUnavailable,
        Self::VerifyRecoveryTrigger,
        Self::ReachRecoveryQuorum,
        Self::FenceOldSessions,
        Self::RestoreDegradedReadOnly,
        Self::RestoreOldAdministrator,
        Self::ChangeConstitution,
        Self::RevalidateCredentials,
        Self::EnableHighImpact,
        Self::DisableHighImpact,
        Self::AdvanceTime,
        Self::GrantEmergencyAuthority,
        Self::RevokeEmergencyAuthority,
    ];
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct State {
    /// Bitset of domains controlled by the same modeled coalition.
    pub coalition_control_mask: u8,
    pub control_inventory_complete: bool,
    pub founder_available: bool,
    pub recovery_trigger_verified: bool,
    pub recovery_quorum_met: bool,
    pub old_sessions_fenced: bool,
    pub service_mode: ServiceMode,
    pub high_impact_enabled: bool,
    pub current_constitution_epoch: u32,
    pub credential_epoch: u32,
    pub old_administrator_restored: bool,
    pub emergency_authority_active: bool,
    pub emergency_authority_expires_at: Option<u32>,
    pub now: u32,
}

impl Default for State {
    fn default() -> Self {
        Self {
            coalition_control_mask: 0,
            control_inventory_complete: true,
            founder_available: true,
            recovery_trigger_verified: false,
            recovery_quorum_met: false,
            old_sessions_fenced: false,
            service_mode: ServiceMode::Normal,
            high_impact_enabled: true,
            current_constitution_epoch: 1,
            credential_epoch: 1,
            old_administrator_restored: false,
            emergency_authority_active: false,
            emergency_authority_expires_at: None,
            now: 0,
        }
    }
}

impl State {
    fn capture(&mut self, bit: u8) {
        self.coalition_control_mask |= bit;
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Finding {
    pub disposition: Disposition,
    pub reason_code: String,
}

fn finding(disposition: Disposition, reason: &str) -> Finding {
    Finding {
        disposition,
        reason_code: reason.to_owned(),
    }
}

/// Evaluate the declared properties for one state snapshot.
///
/// Hard invariant violations take precedence over uncertainty. A complete
/// inventory with concentrated oversight returns ChallengeRequired, not a
/// model-generated finding of misconduct. An incomplete inventory never
/// returns Permit merely because no known violation is present.
pub fn assess(state: &State) -> Finding {
    if state.service_mode == ServiceMode::DegradedReadOnly && state.high_impact_enabled {
        return finding(
            Disposition::Deny,
            "DEGRADED_MODE_MUST_KEEP_HIGH_IMPACT_DISABLED",
        );
    }
    if state.service_mode == ServiceMode::DegradedReadOnly && !state.old_sessions_fenced {
        return finding(
            Disposition::Deny,
            "DEGRADED_MODE_REQUIRES_OLD_SESSION_FENCING",
        );
    }
    if state.founder_available == false
        && state.high_impact_enabled
        && !(state.recovery_trigger_verified
            && state.recovery_quorum_met
            && state.old_sessions_fenced
            && state.credential_epoch == state.current_constitution_epoch)
    {
        return finding(
            Disposition::Deny,
            "HIGH_IMPACT_RECOVERY_REQUIRES_TRIGGER_QUORUM_FENCING_AND_REVALIDATION",
        );
    }
    if state.old_administrator_restored
        && state.credential_epoch < state.current_constitution_epoch
    {
        return finding(
            Disposition::Deny,
            "STALE_ADMINISTRATOR_PRIVILEGE_RESTORED_AFTER_EPOCH_CHANGE",
        );
    }
    if state.emergency_authority_active
        && state
            .emergency_authority_expires_at
            .is_some_and(|expiry| state.now >= expiry)
    {
        return finding(
            Disposition::Deny,
            "EXPIRED_EMERGENCY_AUTHORITY_REMAINS_ACTIVE",
        );
    }

    if !state.control_inventory_complete {
        return finding(
            Disposition::Indeterminate,
            "CONTROL_INVENTORY_INCOMPLETE",
        );
    }

    let oversight_captured =
        state.coalition_control_mask & CAP_OVERSIGHT != 0;
    let critical_control_captured =
        state.coalition_control_mask & CAPTURE_RELEVANT_TO_OVERSIGHT != 0;
    if oversight_captured && critical_control_captured {
        return finding(
            Disposition::ChallengeRequired,
            "OVERSIGHT_SHARES_CONTROL_WITH_APPOINTMENT_RESOURCE_OR_CREDENTIAL_COALITION",
        );
    }

    finding(Disposition::Permit, "DECLARED_TEMPORAL_PROPERTIES_HOLD")
}

/// Apply one modeled transition. Transitions are intentionally permissive:
/// the model checker must be able to explore invalid sequences and expose
/// their counterexamples rather than prevent every unsafe action by fiat.
pub fn apply(state: &State, action: &Action) -> State {
    let mut next = state.clone();
    match action {
        Action::CaptureAppointments => next.capture(CAP_APPOINTMENTS),
        Action::CaptureAgenda => next.capture(CAP_AGENDA),
        Action::CaptureTreasury => next.capture(CAP_TREASURY),
        Action::CaptureCredentials => next.capture(CAP_CREDENTIALS),
        Action::CaptureExecution => next.capture(CAP_EXECUTION),
        Action::CaptureOversight => next.capture(CAP_OVERSIGHT),
        Action::MarkInventoryIncomplete => next.control_inventory_complete = false,
        Action::RestoreIndependentOversight => {
            next.coalition_control_mask &= !CAP_OVERSIGHT;
        }
        Action::FounderUnavailable => next.founder_available = false,
        Action::VerifyRecoveryTrigger => {
            if !next.founder_available {
                next.recovery_trigger_verified = true;
            }
        }
        Action::ReachRecoveryQuorum => {
            if next.recovery_trigger_verified && next.control_inventory_complete {
                next.recovery_quorum_met = true;
            }
        }
        Action::FenceOldSessions => next.old_sessions_fenced = true,
        Action::RestoreDegradedReadOnly => {
            next.service_mode = ServiceMode::DegradedReadOnly;
            next.high_impact_enabled = false;
        }
        Action::RestoreOldAdministrator => next.old_administrator_restored = true,
        Action::ChangeConstitution => {
            next.current_constitution_epoch =
                next.current_constitution_epoch.saturating_add(1);
        }
        Action::RevalidateCredentials => {
            next.credential_epoch = next.current_constitution_epoch;
            next.old_administrator_restored = false;
        }
        Action::EnableHighImpact => {
            next.service_mode = ServiceMode::Normal;
            next.high_impact_enabled = true;
        }
        Action::DisableHighImpact => next.high_impact_enabled = false,
        Action::AdvanceTime => next.now = next.now.saturating_add(1),
        Action::GrantEmergencyAuthority => {
            next.emergency_authority_active = true;
            next.emergency_authority_expires_at = Some(next.now.saturating_add(2));
        }
        Action::RevokeEmergencyAuthority => {
            next.emergency_authority_active = false;
            next.emergency_authority_expires_at = None;
        }
    }
    next
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Counterexample {
    pub reason_code: String,
    pub trace: Vec<Action>,
    pub final_state: State,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelCheckReport {
    pub max_depth: usize,
    pub visited_states: usize,
    pub hard_counterexamples: Vec<Counterexample>,
    pub challenge_witnesses: Vec<Counterexample>,
    pub indeterminate_witnesses: Vec<Counterexample>,
    pub truncated_by_state_budget: bool,
}

/// Explore every unique state reachable up to `max_depth` from the initial
/// state, using breadth-first search. The first witness per reason code is
/// therefore a shortest trace under the declared transition alphabet.
///
/// This is bounded exploration, not an unbounded proof. The report records
/// the state budget and truncation status so callers cannot misread absence of
/// counterexamples as universal safety.
pub fn bounded_explore(max_depth: usize, max_states: usize) -> ModelCheckReport {
    let initial = State::default();
    let mut queue: VecDeque<(State, Vec<Action>)> = VecDeque::new();
    let mut seen: HashSet<State> = HashSet::new();
    let mut hard = Vec::new();
    let mut challenges = Vec::new();
    let mut unknowns = Vec::new();
    let mut hard_codes = HashSet::new();
    let mut challenge_codes = HashSet::new();
    let mut unknown_codes = HashSet::new();
    let mut truncated = false;

    seen.insert(initial.clone());
    queue.push_back((initial, Vec::new()));

    while let Some((state, trace)) = queue.pop_front() {
        let current = assess(&state);
        let witness = Counterexample {
            reason_code: current.reason_code.clone(),
            trace: trace.clone(),
            final_state: state.clone(),
        };
        match current.disposition {
            Disposition::Deny => {
                if hard_codes.insert(current.reason_code.clone()) {
                    hard.push(witness);
                }
            }
            Disposition::ChallengeRequired => {
                if challenge_codes.insert(current.reason_code.clone()) {
                    challenges.push(witness);
                }
            }
            Disposition::Indeterminate => {
                if unknown_codes.insert(current.reason_code.clone()) {
                    unknowns.push(witness);
                }
            }
            Disposition::Permit | Disposition::Hold => {}
        }

        if trace.len() >= max_depth {
            continue;
        }

        for action in Action::ALL.iter() {
            let next = apply(&state, action);
            if next == state || seen.contains(&next) {
                continue;
            }
            if seen.len() >= max_states {
                truncated = true;
                break;
            }
            let mut next_trace = trace.clone();
            next_trace.push(action.clone());
            seen.insert(next.clone());
            queue.push_back((next, next_trace));
        }
        if truncated {
            break;
        }
    }

    ModelCheckReport {
        max_depth,
        visited_states: seen.len(),
        hard_counterexamples: hard,
        challenge_witnesses: challenges,
        indeterminate_witnesses: unknowns,
        truncated_by_state_budget: truncated,
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceScenario {
    pub id: String,
    pub title: String,
    pub actions: Vec<Action>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceSuite {
    pub schema_version: String,
    pub suite_id: String,
    pub status: String,
    pub case_count: usize,
    pub cases: Vec<TraceScenario>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceOracleRow {
    pub case_id: String,
    pub expected_disposition: String,
    pub reason_code: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceOracle {
    pub schema_version: String,
    pub suite_id: String,
    pub status: String,
    pub case_count: usize,
    pub rows: Vec<TraceOracleRow>,
}

pub fn replay_trace(actions: &[Action]) -> (State, Finding) {
    let mut state = State::default();
    for action in actions {
        state = apply(&state, action);
    }
    let result = assess(&state);
    (state, result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn safe_degraded_recovery_preserves_availability_without_authority() {
        let actions = [
            Action::FounderUnavailable,
            Action::VerifyRecoveryTrigger,
            Action::FenceOldSessions,
            Action::DisableHighImpact,
            Action::RestoreDegradedReadOnly,
        ];
        let (_, result) = replay_trace(&actions);
        assert_eq!(result.disposition, Disposition::Permit);
        assert_eq!(result.reason_code, "DECLARED_TEMPORAL_PROPERTIES_HOLD");
    }

    #[test]
    fn degraded_mode_without_session_fencing_is_denied() {
        let actions = [
            Action::FounderUnavailable,
            Action::VerifyRecoveryTrigger,
            Action::DisableHighImpact,
            Action::RestoreDegradedReadOnly,
        ];
        let (_, result) = replay_trace(&actions);
        assert_eq!(result.disposition, Disposition::Deny);
        assert_eq!(result.reason_code, "DEGRADED_MODE_REQUIRES_OLD_SESSION_FENCING");
    }

    #[test]
    fn restoring_old_administrator_after_epoch_change_is_denied() {
        let actions = [
            Action::ChangeConstitution,
            Action::RestoreOldAdministrator,
        ];
        let (_, result) = replay_trace(&actions);
        assert_eq!(result.disposition, Disposition::Deny);
        assert_eq!(
            result.reason_code,
            "STALE_ADMINISTRATOR_PRIVILEGE_RESTORED_AFTER_EPOCH_CHANGE"
        );
    }

    #[test]
    fn emergency_authority_expires_and_cannot_remain_active() {
        let actions = [
            Action::GrantEmergencyAuthority,
            Action::AdvanceTime,
            Action::AdvanceTime,
        ];
        let (_, result) = replay_trace(&actions);
        assert_eq!(result.disposition, Disposition::Deny);
        assert_eq!(
            result.reason_code,
            "EXPIRED_EMERGENCY_AUTHORITY_REMAINS_ACTIVE"
        );
    }

    #[test]
    fn coalition_oversight_overlap_requires_challenge_not_automatic_verdict() {
        let actions = [
            Action::CaptureAppointments,
            Action::CaptureTreasury,
            Action::CaptureOversight,
        ];
        let (_, result) = replay_trace(&actions);
        assert_eq!(result.disposition, Disposition::ChallengeRequired);
        assert_eq!(
            result.reason_code,
            "OVERSIGHT_SHARES_CONTROL_WITH_APPOINTMENT_RESOURCE_OR_CREDENTIAL_COALITION"
        );
    }

    #[test]
    fn incomplete_control_inventory_is_indeterminate() {
        let (_, result) = replay_trace(&[Action::MarkInventoryIncomplete]);
        assert_eq!(result.disposition, Disposition::Indeterminate);
        assert_eq!(result.reason_code, "CONTROL_INVENTORY_INCOMPLETE");
    }

    #[test]
    fn independent_oversight_restores_a_challenge_witness() {
        let actions = [
            Action::CaptureTreasury,
            Action::CaptureOversight,
            Action::RestoreIndependentOversight,
        ];
        let (_, result) = replay_trace(&actions);
        assert_eq!(result.disposition, Disposition::Permit);
    }

    #[test]
    fn high_impact_recovery_requires_trigger_quorum_fencing_and_epoch_revalidation() {
        let unsafe_actions = [
            Action::FounderUnavailable,
            Action::VerifyRecoveryTrigger,
            Action::FenceOldSessions,
            Action::EnableHighImpact,
        ];
        let (_, denied) = replay_trace(&unsafe_actions);
        assert_eq!(denied.disposition, Disposition::Deny);

        let safe_actions = [
            Action::FounderUnavailable,
            Action::VerifyRecoveryTrigger,
            Action::ReachRecoveryQuorum,
            Action::FenceOldSessions,
            Action::ChangeConstitution,
            Action::RevalidateCredentials,
            Action::EnableHighImpact,
        ];
        let (_, permitted) = replay_trace(&safe_actions);
        assert_eq!(permitted.disposition, Disposition::Permit);
    }

    #[test]
    fn bounded_explorer_reports_shortest_counterexample_traces() {
        let report = bounded_explore(5, 50_000);
        assert!(report.visited_states > 1);
        assert!(!report.hard_counterexamples.is_empty());
        assert!(report.hard_counterexamples.iter().any(|x| {
            x.reason_code == "STALE_ADMINISTRATOR_PRIVILEGE_RESTORED_AFTER_EPOCH_CHANGE"
        }));
        assert!(report.challenge_witnesses.iter().any(|x| {
            x.reason_code
                == "OVERSIGHT_SHARES_CONTROL_WITH_APPOINTMENT_RESOURCE_OR_CREDENTIAL_COALITION"
        }));
    }
}
