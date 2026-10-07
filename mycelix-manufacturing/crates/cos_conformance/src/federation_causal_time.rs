//! Causal-time and long-lived federation reference model.
//!
//! Claim ceiling: ReferenceModelOnly.
//! Wall-clock observations are bounded evidence. They never replace causal order,
//! currentness, authority validity, or semantic protocol identity.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const CAUSAL_TIME_PROFILE_ID: &str = "INTEGRAL-TIME-REF-001";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum CausalRelation {
    Same,
    Before,
    After,
    Concurrent,
    Incomparable,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TemporalInterval {
    pub earliest: u64,
    pub latest: u64,
    pub clock_domain_root: String,
}

impl TemporalInterval {
    pub fn new(earliest: u64, latest: u64, clock_domain_root: impl Into<String>) -> Option<Self> {
        if earliest > latest {
            return None;
        }
        Some(Self {
            earliest,
            latest,
            clock_domain_root: clock_domain_root.into(),
        })
    }

    pub fn contains(&self, instant: u64) -> bool {
        self.earliest <= instant && instant <= self.latest
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FreshnessStatus {
    Current,
    Stale,
    Indeterminate,
    IncomparableClockDomain,
    ProfileMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FreshnessProfile {
    pub profile_id: String,
    pub clock_domain_root: String,
    pub max_age: u64,
    pub semantic_environment_root: String,
    pub protocol_profile_root: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TemporalEvaluationReference {
    pub interval: TemporalInterval,
    pub semantic_environment_root: String,
    pub protocol_profile_root: String,
}

pub fn evaluate_freshness(
    observed: &TemporalInterval,
    reference: &TemporalEvaluationReference,
    profile: &FreshnessProfile,
) -> FreshnessStatus {
    if observed.clock_domain_root != profile.clock_domain_root
        || reference.interval.clock_domain_root != profile.clock_domain_root
    {
        return FreshnessStatus::IncomparableClockDomain;
    }
    if reference.semantic_environment_root != profile.semantic_environment_root
        || reference.protocol_profile_root != profile.protocol_profile_root
    {
        return FreshnessStatus::ProfileMismatch;
    }
    if reference.interval.latest < observed.earliest {
        return FreshnessStatus::Indeterminate;
    }

    let minimum_age = reference.interval.earliest.saturating_sub(observed.latest);
    let maximum_age = reference.interval.latest.saturating_sub(observed.earliest);

    if minimum_age > profile.max_age {
        FreshnessStatus::Stale
    } else if maximum_age <= profile.max_age {
        FreshnessStatus::Current
    } else {
        FreshnessStatus::Indeterminate
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CausalEvent {
    pub event_id: String,
    pub predecessors: BTreeSet<String>,
    pub base_frontier_root: String,
    pub temporal_interval: TemporalInterval,
    pub semantic_environment_root: String,
    pub protocol_profile_root: String,
    pub branch_id: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClosureStatus {
    Closed,
    MissingPredecessors,
    CycleDetected,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClosureReport {
    pub status: ClosureStatus,
    pub missing_predecessors: BTreeSet<String>,
    pub cycle_nodes: BTreeSet<String>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CausalGraph {
    pub events: BTreeMap<String, CausalEvent>,
}

impl CausalGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert_event(&mut self, event: CausalEvent) -> bool {
        if self.events.contains_key(&event.event_id) {
            return false;
        }
        self.events.insert(event.event_id.clone(), event);
        true
    }

    pub fn closure_report(&self, event_id: &str) -> ClosureReport {
        if !self.events.contains_key(event_id) {
            return ClosureReport {
                status: ClosureStatus::MissingPredecessors,
                missing_predecessors: BTreeSet::from([event_id.to_owned()]),
                cycle_nodes: BTreeSet::new(),
            };
        }

        let mut visiting = BTreeSet::new();
        let mut visited = BTreeSet::new();
        let mut missing = BTreeSet::new();
        let mut cycles = BTreeSet::new();
        self.walk_closure(event_id, &mut visiting, &mut visited, &mut missing, &mut cycles);

        let status = if !cycles.is_empty() {
            ClosureStatus::CycleDetected
        } else if !missing.is_empty() {
            ClosureStatus::MissingPredecessors
        } else {
            ClosureStatus::Closed
        };

        ClosureReport {
            status,
            missing_predecessors: missing,
            cycle_nodes: cycles,
        }
    }

    fn walk_closure(
        &self,
        event_id: &str,
        visiting: &mut BTreeSet<String>,
        visited: &mut BTreeSet<String>,
        missing: &mut BTreeSet<String>,
        cycles: &mut BTreeSet<String>,
    ) {
        if visited.contains(event_id) {
            return;
        }
        if !visiting.insert(event_id.to_owned()) {
            cycles.insert(event_id.to_owned());
            return;
        }
        let Some(event) = self.events.get(event_id) else {
            visiting.remove(event_id);
            missing.insert(event_id.to_owned());
            return;
        };
        for predecessor in &event.predecessors {
            if !self.events.contains_key(predecessor) {
                missing.insert(predecessor.clone());
            } else {
                self.walk_closure(predecessor, visiting, visited, missing, cycles);
            }
        }
        visiting.remove(event_id);
        visited.insert(event_id.to_owned());
    }

    fn ancestors(
        &self,
        event_id: &str,
        ancestors: &mut BTreeSet<String>,
    ) -> bool {
        let Some(event) = self.events.get(event_id) else {
            return false;
        };
        for predecessor in &event.predecessors {
            if ancestors.insert(predecessor.clone()) && !self.ancestors(predecessor, ancestors) {
                return false;
            }
        }
        true
    }

    pub fn relation(&self, left: &str, right: &str) -> Option<CausalRelation> {
        if left == right {
            return self.events.contains_key(left).then_some(CausalRelation::Same);
        }
        let left_report = self.closure_report(left);
        let right_report = self.closure_report(right);
        if left_report.status != ClosureStatus::Closed || right_report.status != ClosureStatus::Closed {
            return None;
        }

        let mut left_ancestors = BTreeSet::new();
        let mut right_ancestors = BTreeSet::new();
        if !self.ancestors(left, &mut left_ancestors) || !self.ancestors(right, &mut right_ancestors) {
            return None;
        }

        if left_ancestors.contains(right) {
            Some(CausalRelation::After)
        } else if right_ancestors.contains(left) {
            Some(CausalRelation::Before)
        } else {
            let left_base = self.events.get(left)?.base_frontier_root.clone();
            let right_base = self.events.get(right)?.base_frontier_root.clone();
            if left_base == right_base {
                Some(CausalRelation::Concurrent)
            } else {
                Some(CausalRelation::Incomparable)
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RevalidationOutcome {
    Current,
    RevalidationRequired,
    AuthorityNotCurrent,
    TemporalEvidenceInsufficient,
    SemanticEnvironmentChanged,
    ProtocolProfileChanged,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BranchTemporalState {
    pub branch_id: String,
    pub tip_event_id: String,
    pub last_observed: TemporalInterval,
    pub semantic_environment_root: String,
    pub protocol_profile_root: String,
    pub authority_current: bool,
}

pub fn revalidate_branch(
    branch: &BranchTemporalState,
    reference: &TemporalEvaluationReference,
    freshness: &FreshnessProfile,
) -> RevalidationOutcome {
    if branch.semantic_environment_root != reference.semantic_environment_root {
        return RevalidationOutcome::SemanticEnvironmentChanged;
    }
    if branch.protocol_profile_root != reference.protocol_profile_root {
        return RevalidationOutcome::ProtocolProfileChanged;
    }
    if !branch.authority_current {
        return RevalidationOutcome::AuthorityNotCurrent;
    }

    match evaluate_freshness(&branch.last_observed, reference, freshness) {
        FreshnessStatus::Current => RevalidationOutcome::Current,
        FreshnessStatus::Stale => RevalidationOutcome::RevalidationRequired,
        FreshnessStatus::Indeterminate | FreshnessStatus::IncomparableClockDomain => {
            RevalidationOutcome::TemporalEvidenceInsufficient
        }
        FreshnessStatus::ProfileMismatch => RevalidationOutcome::ProtocolProfileChanged,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReplayDisposition {
    HistoricalOnly,
    CurrentEligible,
    RevalidationRequired,
    ProfileMismatch,
}

pub fn replay_disposition(
    event: &CausalEvent,
    reference: &TemporalEvaluationReference,
    freshness: &FreshnessProfile,
) -> ReplayDisposition {
    if event.semantic_environment_root != reference.semantic_environment_root {
        return ReplayDisposition::ProfileMismatch;
    }
    if event.protocol_profile_root != reference.protocol_profile_root {
        return ReplayDisposition::ProfileMismatch;
    }
    match evaluate_freshness(&event.temporal_interval, reference, freshness) {
        FreshnessStatus::Current => ReplayDisposition::CurrentEligible,
        FreshnessStatus::Stale | FreshnessStatus::Indeterminate => ReplayDisposition::RevalidationRequired,
        FreshnessStatus::IncomparableClockDomain | FreshnessStatus::ProfileMismatch => {
            ReplayDisposition::ProfileMismatch
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TemporalWitness {
    pub event_id: String,
    pub causal_relation: Option<CausalRelation>,
    pub closure: ClosureReport,
    pub freshness: Option<FreshnessStatus>,
    pub replay: ReplayDisposition,
    pub claim_ceiling: String,
}

pub fn temporal_witness(
    graph: &CausalGraph,
    event_id: &str,
    comparison_event_id: Option<&str>,
    reference: &TemporalEvaluationReference,
    freshness: &FreshnessProfile,
) -> Option<TemporalWitness> {
    let event = graph.events.get(event_id)?;
    let closure = graph.closure_report(event_id);
    let causal_relation = comparison_event_id.and_then(|other| graph.relation(event_id, other));
    let freshness_status = if closure.status == ClosureStatus::Closed {
        Some(evaluate_freshness(&event.temporal_interval, reference, freshness))
    } else {
        None
    };
    Some(TemporalWitness {
        event_id: event_id.to_owned(),
        causal_relation,
        closure,
        freshness: freshness_status,
        replay: replay_disposition(event, reference, freshness),
        claim_ceiling: "Causal/temporal reference semantics only; no synchronized-clock or real-world chronology claim.".into(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn interval(earliest: u64, latest: u64) -> TemporalInterval {
        TemporalInterval::new(earliest, latest, "clock-1").expect("valid interval")
    }

    fn event(
        id: &str,
        predecessors: &[&str],
        earliest: u64,
        latest: u64,
        branch: &str,
        protocol: &str,
        base: &str,
    ) -> CausalEvent {
        CausalEvent {
            event_id: id.into(),
            predecessors: predecessors.iter().map(|id| (*id).into()).collect(),
            base_frontier_root: base.into(),
            temporal_interval: interval(earliest, latest),
            semantic_environment_root: "env-1".into(),
            protocol_profile_root: protocol.into(),
            branch_id: branch.into(),
        }
    }

    fn reference(earliest: u64, latest: u64, protocol: &str) -> TemporalEvaluationReference {
        TemporalEvaluationReference {
            interval: interval(earliest, latest),
            semantic_environment_root: "env-1".into(),
            protocol_profile_root: protocol.into(),
        }
    }

    fn profile(max_age: u64, protocol: &str) -> FreshnessProfile {
        FreshnessProfile {
            profile_id: "freshness-1".into(),
            clock_domain_root: "clock-1".into(),
            max_age,
            semantic_environment_root: "env-1".into(),
            protocol_profile_root: protocol.into(),
        }
    }

    #[test]
    fn causal_order_does_not_follow_wall_clock_arrival() {
        let mut graph = CausalGraph::new();
        graph.insert_event(event("a", &[], 1_000, 1_000, "branch-a", "proto-1", "base-1"));
        graph.insert_event(event("b", &[], 2_000, 2_000, "branch-b", "proto-1", "base-1"));
        assert_eq!(graph.relation("a", "b"), Some(CausalRelation::Concurrent));
    }

    #[test]
    fn explicit_predecessor_defines_causal_order_even_with_older_wall_time() {
        let mut graph = CausalGraph::new();
        graph.insert_event(event("a", &[], 2_000, 2_000, "branch-a", "proto-1", "base-1"));
        graph.insert_event(event("b", &["a"], 1_000, 1_000, "branch-a", "proto-1", "base-1"));
        assert_eq!(graph.relation("a", "b"), Some(CausalRelation::Before));
        assert_eq!(graph.relation("b", "a"), Some(CausalRelation::After));
    }

    #[test]
    fn different_bases_are_incomparable_even_in_one_environment() {
        let mut graph = CausalGraph::new();
        graph.insert_event(event("a", &[], 1_000, 1_000, "branch-a", "proto-1", "base-a"));
        graph.insert_event(event("b", &[], 2_000, 2_000, "branch-b", "proto-1", "base-b"));
        assert_eq!(graph.relation("a", "b"), Some(CausalRelation::Incomparable));
    }

    #[test]
    fn missing_predecessor_is_not_treated_as_concurrency() {
        let mut graph = CausalGraph::new();
        graph.insert_event(event("b", &["missing"], 2_000, 2_000, "branch-a", "proto-1", "base-1"));
        assert_eq!(graph.closure_report("b").status, ClosureStatus::MissingPredecessors);
        assert_eq!(graph.relation("b", "b"), Some(CausalRelation::Same));
        assert_eq!(graph.relation("b", "missing"), None);
    }

    #[test]
    fn cycles_are_rejected_from_causal_closure() {
        let mut graph = CausalGraph::new();
        graph.insert_event(event("a", &["b"], 1, 1, "branch-a", "proto-1", "base-1"));
        graph.insert_event(event("b", &["a"], 2, 2, "branch-a", "proto-1", "base-1"));
        assert_eq!(graph.closure_report("a").status, ClosureStatus::CycleDetected);
        assert_eq!(graph.relation("a", "b"), None);
    }

    #[test]
    fn uncertain_clock_bounds_produce_indeterminate_freshness() {
        let observed = interval(900, 1_100);
        let evaluation = reference(1_000, 1_200, "proto-1");
        assert_eq!(evaluate_freshness(&observed, &evaluation, &profile(50, "proto-1")), FreshnessStatus::Indeterminate);
    }

    #[test]
    fn long_offline_branch_requires_revalidation() {
        let branch = BranchTemporalState {
            branch_id: "offline-a".into(),
            tip_event_id: "tip-a".into(),
            last_observed: interval(100, 100),
            semantic_environment_root: "env-1".into(),
            protocol_profile_root: "proto-1".into(),
            authority_current: true,
        };
        let evaluation = reference(1_000, 1_000, "proto-1");
        assert_eq!(revalidate_branch(&branch, &evaluation, &profile(100, "proto-1")), RevalidationOutcome::RevalidationRequired);
    }

    #[test]
    fn elapsed_time_never_revives_noncurrent_authority() {
        let branch = BranchTemporalState {
            branch_id: "offline-revoked".into(),
            tip_event_id: "tip-a".into(),
            last_observed: interval(100, 100),
            semantic_environment_root: "env-1".into(),
            protocol_profile_root: "proto-1".into(),
            authority_current: false,
        };
        let evaluation = reference(10, 10_000, "proto-1");
        assert_eq!(revalidate_branch(&branch, &evaluation, &profile(20_000, "proto-1")), RevalidationOutcome::AuthorityNotCurrent);
    }

    #[test]
    fn protocol_generation_change_blocks_silent_replay() {
        let e = event("old", &[], 100, 100, "branch-a", "proto-v1", "base-1");
        let evaluation = reference(200, 200, "proto-v2");
        assert_eq!(replay_disposition(&e, &evaluation, &profile(10_000, "proto-v2")), ReplayDisposition::ProfileMismatch);
    }

    #[test]
    fn different_clock_domains_are_not_compared() {
        let observed = TemporalInterval::new(10, 10, "clock-a").unwrap();
        let evaluation = TemporalEvaluationReference {
            interval: TemporalInterval::new(20, 20, "clock-b").unwrap(),
            semantic_environment_root: "env-1".into(),
            protocol_profile_root: "proto-1".into(),
        };
        assert_eq!(evaluate_freshness(&observed, &evaluation, &profile(100, "proto-1")), FreshnessStatus::IncomparableClockDomain);
    }

    #[test]
    fn replay_keeps_old_profile_historical() {
        let e = event("old", &[], 100, 100, "branch-a", "proto-v1", "base-1");
        let evaluation = reference(101, 101, "proto-v2");
        assert_eq!(replay_disposition(&e, &evaluation, &profile(10_000, "proto-v2")), ReplayDisposition::ProfileMismatch);
    }

    #[test]
    fn temporal_witness_preserves_claim_ceiling() {
        let mut graph = CausalGraph::new();
        graph.insert_event(event("a", &[], 100, 100, "branch-a", "proto-1", "base-1"));
        let witness = temporal_witness(
            &graph,
            "a",
            None,
            &reference(150, 150, "proto-1"),
            &profile(100, "proto-1"),
        ).expect("event exists");
        assert!(witness.claim_ceiling.contains("no synchronized-clock"));
    }
}