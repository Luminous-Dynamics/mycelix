use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const INITIAL_POLICY_EPOCH: u64 = 1;
const INITIAL_MEMBERSHIP_EPOCH: u64 = 1;
const MANIFEST: &str = include_str!("../manifest.json");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct BudgetPolicy {
    person_per_domain: u64,
    group_per_domain: u64,
    purpose_per_domain: u64,
    observer_domain: u64,
}

impl BudgetPolicy {
    #[cfg(test)]
    const fn standard() -> Self {
        Self {
            person_per_domain: 8,
            group_per_domain: 8,
            purpose_per_domain: 12,
            observer_domain: 24,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EnforcementMode {
    PersonOnlyNegativeControl,
    CompositeScopes,
}

impl EnforcementMode {
    const fn name(self) -> &'static str {
        match self {
            Self::PersonOnlyNegativeControl => "person_only_negative_control",
            Self::CompositeScopes => "composite_scopes",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Request {
    id: u64,
    subject: u64,
    observer_domain: u64,
    purpose: u64,
    cost: u32,
    policy_epoch: u64,
    membership_epoch: u64,
    payload_digest: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AdmissionOutcome {
    Accepted,
    DuplicateReplay,
    Conflict,
    InvalidCost,
    StalePolicyEpoch,
    StaleMembershipEpoch,
    MissingMembership,
    DeniedPersonBudget,
    DeniedGroupBudget,
    DeniedPurposeBudget,
    DeniedObserverDomainBudget,
}

impl AdmissionOutcome {
    const fn is_denial(self) -> bool {
        matches!(
            self,
            Self::InvalidCost
                | Self::StalePolicyEpoch
                | Self::StaleMembershipEpoch
                | Self::MissingMembership
                | Self::DeniedPersonBudget
                | Self::DeniedGroupBudget
                | Self::DeniedPurposeBudget
                | Self::DeniedObserverDomainBudget
                | Self::Conflict
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MembershipUpdateOutcome {
    Updated,
    NoChange,
    UnknownSubject,
    DeniedRetroactiveComposition {
        group: u64,
        observer_domain: u64,
        proposed_spend: u64,
    },
    EpochExhausted,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Metrics {
    offered: u64,
    accepted_unique: u64,
    denied: u64,
    duplicate_replays: u64,
    conflicts: u64,
    invalid_cost_denials: u64,
    stale_policy_denials: u64,
    stale_membership_denials: u64,
    missing_membership_denials: u64,
    person_budget_denials: u64,
    group_budget_denials: u64,
    purpose_budget_denials: u64,
    observer_domain_budget_denials: u64,
    membership_updates_accepted: u64,
    membership_updates_denied: u64,
    latest_rejected_edge_proposed_spend: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct AuditResult {
    accepted_records: u64,
    total_spend: u64,
    group_domain_spend: BTreeMap<(u64, u64), u64>,
    group_overrun_count: u64,
    max_group_spend: u64,
}

struct Ledger {
    active_policy_epoch: u64,
    membership_epoch: u64,
    memberships: BTreeMap<u64, BTreeSet<u64>>,
    budgets: BudgetPolicy,
    mode: EnforcementMode,
    seen: BTreeMap<u64, (Request, AdmissionOutcome)>,
    accepted: Vec<Request>,
    metrics: Metrics,
}

impl Ledger {
    fn new(
        memberships: BTreeMap<u64, BTreeSet<u64>>,
        budgets: BudgetPolicy,
        mode: EnforcementMode,
    ) -> Self {
        Self {
            active_policy_epoch: INITIAL_POLICY_EPOCH,
            membership_epoch: INITIAL_MEMBERSHIP_EPOCH,
            memberships,
            budgets,
            mode,
            seen: BTreeMap::new(),
            accepted: Vec::new(),
            metrics: Metrics::default(),
        }
    }

    fn admit(&mut self, request: Request) -> AdmissionOutcome {
        self.metrics.offered = self.metrics.offered.saturating_add(1);

        if let Some((previous, previous_outcome)) = self.seen.get(&request.id) {
            if previous == &request {
                self.metrics.duplicate_replays = self.metrics.duplicate_replays.saturating_add(1);
                return if *previous_outcome == AdmissionOutcome::Accepted {
                    AdmissionOutcome::DuplicateReplay
                } else {
                    *previous_outcome
                };
            }
            self.metrics.conflicts = self.metrics.conflicts.saturating_add(1);
            self.metrics.denied = self.metrics.denied.saturating_add(1);
            return AdmissionOutcome::Conflict;
        }

        let outcome = self.validate(&request);
        if outcome == AdmissionOutcome::Accepted {
            self.accepted.push(request.clone());
            self.metrics.accepted_unique = self.metrics.accepted_unique.saturating_add(1);
        } else if outcome.is_denial() {
            self.record_denial(outcome);
        }
        self.seen.insert(request.id, (request, outcome));
        outcome
    }

    fn validate(&self, request: &Request) -> AdmissionOutcome {
        if request.cost == 0 {
            return AdmissionOutcome::InvalidCost;
        }
        if request.policy_epoch != self.active_policy_epoch {
            return AdmissionOutcome::StalePolicyEpoch;
        }
        if request.membership_epoch != self.membership_epoch {
            return AdmissionOutcome::StaleMembershipEpoch;
        }
        let Some(groups) = self.memberships.get(&request.subject) else {
            return AdmissionOutcome::MissingMembership;
        };

        let cost = u64::from(request.cost);
        if self.subject_spend(request.subject, request.observer_domain) + cost
            > self.budgets.person_per_domain
        {
            return AdmissionOutcome::DeniedPersonBudget;
        }

        if self.mode == EnforcementMode::CompositeScopes {
            for group in groups {
                if self.group_spend(*group, request.observer_domain) + cost
                    > self.budgets.group_per_domain
                {
                    return AdmissionOutcome::DeniedGroupBudget;
                }
            }
            if self.purpose_spend(request.observer_domain, request.purpose) + cost
                > self.budgets.purpose_per_domain
            {
                return AdmissionOutcome::DeniedPurposeBudget;
            }
            if self.observer_domain_spend(request.observer_domain) + cost
                > self.budgets.observer_domain
            {
                return AdmissionOutcome::DeniedObserverDomainBudget;
            }
        }

        AdmissionOutcome::Accepted
    }

    fn record_denial(&mut self, outcome: AdmissionOutcome) {
        self.metrics.denied = self.metrics.denied.saturating_add(1);
        match outcome {
            AdmissionOutcome::InvalidCost => {
                self.metrics.invalid_cost_denials =
                    self.metrics.invalid_cost_denials.saturating_add(1);
            }
            AdmissionOutcome::StalePolicyEpoch => {
                self.metrics.stale_policy_denials =
                    self.metrics.stale_policy_denials.saturating_add(1);
            }
            AdmissionOutcome::StaleMembershipEpoch => {
                self.metrics.stale_membership_denials =
                    self.metrics.stale_membership_denials.saturating_add(1);
            }
            AdmissionOutcome::MissingMembership => {
                self.metrics.missing_membership_denials =
                    self.metrics.missing_membership_denials.saturating_add(1);
            }
            AdmissionOutcome::DeniedPersonBudget => {
                self.metrics.person_budget_denials =
                    self.metrics.person_budget_denials.saturating_add(1);
            }
            AdmissionOutcome::DeniedGroupBudget => {
                self.metrics.group_budget_denials =
                    self.metrics.group_budget_denials.saturating_add(1);
            }
            AdmissionOutcome::DeniedPurposeBudget => {
                self.metrics.purpose_budget_denials =
                    self.metrics.purpose_budget_denials.saturating_add(1);
            }
            AdmissionOutcome::DeniedObserverDomainBudget => {
                self.metrics.observer_domain_budget_denials = self
                    .metrics
                    .observer_domain_budget_denials
                    .saturating_add(1);
            }
            AdmissionOutcome::Accepted
            | AdmissionOutcome::DuplicateReplay
            | AdmissionOutcome::Conflict => {}
        }
    }

    fn subject_spend(&self, subject: u64, observer_domain: u64) -> u64 {
        spend(self.accepted.iter().filter(|request| {
            request.subject == subject && request.observer_domain == observer_domain
        }))
    }

    fn group_spend(&self, group: u64, observer_domain: u64) -> u64 {
        group_spend_in(&self.accepted, &self.memberships, group, observer_domain)
    }

    fn purpose_spend(&self, observer_domain: u64, purpose: u64) -> u64 {
        spend(self.accepted.iter().filter(|request| {
            request.observer_domain == observer_domain && request.purpose == purpose
        }))
    }

    fn observer_domain_spend(&self, observer_domain: u64) -> u64 {
        spend(
            self.accepted
                .iter()
                .filter(|request| request.observer_domain == observer_domain),
        )
    }

    // Public relationship edges are monotone: deleting an edge does not erase
    // an observer's memory of historical linkability. Candidate edges are
    // evaluated against prior accepted disclosures before the new epoch commits.
    fn add_public_memberships(
        &mut self,
        subject: u64,
        groups_to_add: &[u64],
    ) -> MembershipUpdateOutcome {
        let Some(existing) = self.memberships.get(&subject).cloned() else {
            return MembershipUpdateOutcome::UnknownSubject;
        };

        let additions = groups_to_add
            .iter()
            .copied()
            .filter(|group| !existing.contains(group))
            .collect::<BTreeSet<_>>();
        if additions.is_empty() {
            return MembershipUpdateOutcome::NoChange;
        }

        let mut candidate = self.memberships.clone();
        candidate
            .entry(subject)
            .or_default()
            .extend(additions.iter().copied());

        let domains = self
            .accepted
            .iter()
            .map(|request| request.observer_domain)
            .collect::<BTreeSet<_>>();
        for group in additions {
            for observer_domain in &domains {
                let proposed_spend =
                    group_spend_in(&self.accepted, &candidate, group, *observer_domain);
                if proposed_spend > self.budgets.group_per_domain {
                    self.metrics.membership_updates_denied =
                        self.metrics.membership_updates_denied.saturating_add(1);
                    self.metrics.latest_rejected_edge_proposed_spend =
                        proposed_spend.max(self.metrics.latest_rejected_edge_proposed_spend);
                    return MembershipUpdateOutcome::DeniedRetroactiveComposition {
                        group,
                        observer_domain: *observer_domain,
                        proposed_spend,
                    };
                }
            }
        }

        let Some(next_epoch) = self.membership_epoch.checked_add(1) else {
            return MembershipUpdateOutcome::EpochExhausted;
        };
        self.memberships = candidate;
        self.membership_epoch = next_epoch;
        self.metrics.membership_updates_accepted =
            self.metrics.membership_updates_accepted.saturating_add(1);
        MembershipUpdateOutcome::Updated
    }

    // Policy changes affect future admission. Existing accepted releases remain
    // in the history and are never refunded by a policy-epoch increment.
    fn advance_policy_epoch(&mut self, next_epoch: u64) -> bool {
        if next_epoch <= self.active_policy_epoch {
            return false;
        }
        self.active_policy_epoch = next_epoch;
        true
    }

    fn audit(&self) -> AuditResult {
        audit_accepted_history(
            &self.accepted,
            &self.memberships,
            self.budgets.group_per_domain,
        )
    }
}

fn spend<'a>(requests: impl Iterator<Item = &'a Request>) -> u64 {
    requests.fold(0_u64, |total, request| {
        total.saturating_add(u64::from(request.cost))
    })
}

// Independent post-hoc auditor. It reconstructs all scope totals from accepted
// request records plus the current canonical membership registry; it never
// reads Metrics, reducer spend fields, or accepted counters.
fn audit_accepted_history(
    accepted: &[Request],
    memberships: &BTreeMap<u64, BTreeSet<u64>>,
    group_budget: u64,
) -> AuditResult {
    let mut result = AuditResult {
        accepted_records: accepted.len() as u64,
        total_spend: spend(accepted.iter()),
        ..AuditResult::default()
    };

    for request in accepted {
        if let Some(groups) = memberships.get(&request.subject) {
            for group in groups {
                let total = result
                    .group_domain_spend
                    .entry((*group, request.observer_domain))
                    .or_default();
                *total = total.saturating_add(u64::from(request.cost));
            }
        }
    }

    result.max_group_spend = result
        .group_domain_spend
        .values()
        .copied()
        .max()
        .unwrap_or(0);
    result.group_overrun_count = result
        .group_domain_spend
        .values()
        .filter(|total| **total > group_budget)
        .count() as u64;
    result
}

fn group_spend_in(
    accepted: &[Request],
    memberships: &BTreeMap<u64, BTreeSet<u64>>,
    group: u64,
    observer_domain: u64,
) -> u64 {
    spend(accepted.iter().filter(|request| {
        request.observer_domain == observer_domain
            && memberships
                .get(&request.subject)
                .is_some_and(|groups| groups.contains(&group))
    }))
}

fn empty_memberships(subjects: &[u64]) -> BTreeMap<u64, BTreeSet<u64>> {
    subjects
        .iter()
        .copied()
        .map(|subject| (subject, BTreeSet::new()))
        .collect()
}

fn memberships(entries: &[(u64, &[u64])]) -> BTreeMap<u64, BTreeSet<u64>> {
    entries
        .iter()
        .map(|(subject, groups)| (*subject, groups.iter().copied().collect::<BTreeSet<_>>()))
        .collect()
}

fn request(
    id: u64,
    subject: u64,
    observer_domain: u64,
    purpose: u64,
    cost: u32,
    policy_epoch: u64,
    membership_epoch: u64,
) -> Request {
    Request {
        id,
        subject,
        observer_domain,
        purpose,
        cost,
        policy_epoch,
        membership_epoch,
        payload_digest: id.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ subject ^ purpose,
    }
}

struct ScenarioRow {
    name: &'static str,
    ledger: Ledger,
}

fn unsafe_person_only_scenario() -> ScenarioRow {
    let membership = memberships(&[(1, &[100]), (2, &[100]), (3, &[100]), (4, &[100])]);
    let budgets = BudgetPolicy {
        person_per_domain: 5,
        group_per_domain: 8,
        purpose_per_domain: 100,
        observer_domain: 100,
    };
    let mut ledger = Ledger::new(
        membership,
        budgets,
        EnforcementMode::PersonOnlyNegativeControl,
    );
    for subject in 1..=4 {
        let req = request(
            subject,
            subject,
            1,
            10 + subject,
            3,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        );
        let _ = ledger.admit(req);
    }
    ScenarioRow {
        name: "unsafe_person_only_control",
        ledger,
    }
}

fn composite_scope_scenario() -> ScenarioRow {
    let membership = memberships(&[(1, &[100]), (2, &[100]), (3, &[100]), (4, &[100])]);
    let budgets = BudgetPolicy {
        person_per_domain: 8,
        group_per_domain: 8,
        purpose_per_domain: 100,
        observer_domain: 100,
    };
    let mut ledger = Ledger::new(membership, budgets, EnforcementMode::CompositeScopes);
    for subject in 1..=4 {
        let req = request(
            subject,
            subject,
            1,
            10 + subject,
            3,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        );
        let _ = ledger.admit(req);
    }
    ScenarioRow {
        name: "composite_atomic_gate",
        ledger,
    }
}

fn overlapping_group_scenario() -> ScenarioRow {
    let membership = memberships(&[(1, &[10, 20]), (2, &[20]), (3, &[10])]);
    let budgets = BudgetPolicy {
        person_per_domain: 10,
        group_per_domain: 5,
        purpose_per_domain: 20,
        observer_domain: 30,
    };
    let mut ledger = Ledger::new(membership, budgets, EnforcementMode::CompositeScopes);
    let requests = [
        request(
            1,
            1,
            1,
            1,
            2,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        ),
        request(
            2,
            3,
            1,
            1,
            2,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        ),
        request(
            3,
            2,
            1,
            1,
            2,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        ),
        request(
            4,
            1,
            1,
            2,
            2,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        ),
    ];
    for req in requests {
        let _ = ledger.admit(req);
    }
    ScenarioRow {
        name: "overlapping_groups_atomic_gate",
        ledger,
    }
}

fn retroactive_relink_scenario() -> ScenarioRow {
    let membership = empty_memberships(&[1, 2, 3]);
    let budgets = BudgetPolicy {
        person_per_domain: 8,
        group_per_domain: 8,
        purpose_per_domain: 12,
        observer_domain: 24,
    };
    let mut ledger = Ledger::new(membership, budgets, EnforcementMode::CompositeScopes);
    for subject in 1..=3 {
        let req = request(
            subject,
            subject,
            1,
            subject,
            3,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        );
        let _ = ledger.admit(req);
    }
    let _ = ledger.add_public_memberships(1, &[200]);
    let _ = ledger.add_public_memberships(2, &[200]);
    let _ = ledger.add_public_memberships(3, &[200]);
    ScenarioRow {
        name: "retroactive_relinking_guard",
        ledger,
    }
}

fn policy_rollover_scenario() -> ScenarioRow {
    let membership = memberships(&[(1, &[100])]);
    let budgets = BudgetPolicy {
        person_per_domain: 20,
        group_per_domain: 8,
        purpose_per_domain: 20,
        observer_domain: 20,
    };
    let mut ledger = Ledger::new(membership, budgets, EnforcementMode::CompositeScopes);
    let _ = ledger.admit(request(
        1,
        1,
        1,
        1,
        7,
        INITIAL_POLICY_EPOCH,
        INITIAL_MEMBERSHIP_EPOCH,
    ));
    let _ = ledger.advance_policy_epoch(INITIAL_POLICY_EPOCH + 1);
    let _ = ledger.admit(request(
        2,
        1,
        1,
        2,
        2,
        INITIAL_POLICY_EPOCH,
        INITIAL_MEMBERSHIP_EPOCH,
    ));
    let _ = ledger.admit(request(
        3,
        1,
        1,
        2,
        2,
        INITIAL_POLICY_EPOCH + 1,
        INITIAL_MEMBERSHIP_EPOCH,
    ));
    ScenarioRow {
        name: "policy_rollover_no_refund",
        ledger,
    }
}

fn render_report() -> String {
    debug_assert!(!MANIFEST.is_empty());
    let scenarios = [
        unsafe_person_only_scenario(),
        composite_scope_scenario(),
        overlapping_group_scenario(),
        retroactive_relink_scenario(),
        policy_rollover_scenario(),
    ];
    let mut output = String::new();
    writeln!(
        &mut output,
        "scenario,enforcement_mode,offered,accepted_unique,denied,duplicate_replays,conflicts,invalid_cost_denials,stale_policy_denials,stale_membership_denials,missing_membership_denials,person_budget_denials,group_budget_denials,purpose_budget_denials,observer_domain_budget_denials,membership_updates_accepted,membership_updates_denied,audit_accepted_records,audit_total_spend,audit_group_overrun_count,audit_max_group_spend,latest_rejected_edge_proposed_spend,membership_epoch"
    )
    .expect("writing to String cannot fail");

    for scenario in scenarios {
        let audit = scenario.ledger.audit();
        let metrics = &scenario.ledger.metrics;
        writeln!(
            &mut output,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            scenario.name,
            scenario.ledger.mode.name(),
            metrics.offered,
            metrics.accepted_unique,
            metrics.denied,
            metrics.duplicate_replays,
            metrics.conflicts,
            metrics.invalid_cost_denials,
            metrics.stale_policy_denials,
            metrics.stale_membership_denials,
            metrics.missing_membership_denials,
            metrics.person_budget_denials,
            metrics.group_budget_denials,
            metrics.purpose_budget_denials,
            metrics.observer_domain_budget_denials,
            metrics.membership_updates_accepted,
            metrics.membership_updates_denied,
            audit.accepted_records,
            audit.total_spend,
            audit.group_overrun_count,
            audit.max_group_spend,
            metrics.latest_rejected_edge_proposed_spend,
            scenario.ledger.membership_epoch,
        )
        .expect("writing to String cannot fail");
    }

    output
}

fn main() {
    print!("{}", render_report());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manifest_freezes_scope_names_budgets_and_fixtures() {
        for fixture in [
            "unsafe_person_only_group_overspend",
            "composite_gate_atomic_rejection",
            "overlapping_groups_atomic_rejection",
            "retroactive_membership_recomposition",
            "policy_epoch_no_refund",
        ] {
            assert!(MANIFEST.contains(fixture), "missing fixture {fixture}");
        }
        assert!(MANIFEST.contains("\"group_per_domain\": 8,"));
        assert!(MANIFEST.contains("\"purpose_per_domain\": 12,"));
    }

    #[test]
    fn person_only_negative_control_allows_group_overspend() {
        let scenario = unsafe_person_only_scenario();
        assert_eq!(scenario.ledger.metrics.accepted_unique, 4);
        assert_eq!(scenario.ledger.metrics.person_budget_denials, 0);
        let audit = scenario.ledger.audit();
        assert_eq!(audit.total_spend, 12);
        assert_eq!(audit.max_group_spend, 12);
        assert_eq!(audit.group_overrun_count, 1);
        assert_eq!(audit.accepted_records, 4);
    }

    #[test]
    fn composite_gate_rejects_group_overspend_without_partial_spend() {
        let mut scenario = composite_scope_scenario();
        assert_eq!(scenario.ledger.metrics.accepted_unique, 2);
        assert_eq!(scenario.ledger.metrics.group_budget_denials, 2);
        let before = scenario.ledger.audit();
        assert_eq!(before.total_spend, 6);
        assert_eq!(before.group_overrun_count, 0);

        let rejected = request(
            10,
            4,
            1,
            99,
            3,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        );
        assert_eq!(
            scenario.ledger.admit(rejected),
            AdmissionOutcome::DeniedGroupBudget
        );
        let after = scenario.ledger.audit();
        assert_eq!(after.total_spend, before.total_spend);
        assert_eq!(after.group_domain_spend, before.group_domain_spend);
    }

    #[test]
    fn overlapping_memberships_charge_each_group_and_reject_atomically() {
        let scenario = overlapping_group_scenario();
        assert_eq!(scenario.ledger.metrics.accepted_unique, 3);
        assert_eq!(scenario.ledger.metrics.group_budget_denials, 1);
        let audit = scenario.ledger.audit();
        assert_eq!(audit.group_domain_spend.get(&(10, 1)), Some(&4));
        assert_eq!(audit.group_domain_spend.get(&(20, 1)), Some(&4));
        assert_eq!(audit.group_overrun_count, 0);
        assert_eq!(audit.total_spend, 6);
    }

    #[test]
    fn newly_visible_group_edge_recomposes_historical_releases() {
        let mut scenario = retroactive_relink_scenario();
        let ledger = &mut scenario.ledger;
        assert_eq!(ledger.metrics.accepted_unique, 3);
        assert_eq!(ledger.metrics.membership_updates_accepted, 2);
        assert_eq!(ledger.metrics.membership_updates_denied, 1);
        assert_eq!(ledger.metrics.latest_rejected_edge_proposed_spend, 9);
        assert_eq!(ledger.membership_epoch, INITIAL_MEMBERSHIP_EPOCH + 2);
        assert!(!ledger.memberships.get(&3).unwrap().contains(&200));
        let audit = ledger.audit();
        assert_eq!(audit.max_group_spend, 6);
        assert_eq!(audit.group_overrun_count, 0);
    }

    #[test]
    fn stale_policy_and_membership_epochs_fail_closed() {
        let mut ledger = Ledger::new(
            memberships(&[(1, &[100])]),
            BudgetPolicy::standard(),
            EnforcementMode::CompositeScopes,
        );
        assert_eq!(
            ledger.admit(request(1, 1, 1, 1, 1, INITIAL_POLICY_EPOCH, 0)),
            AdmissionOutcome::StaleMembershipEpoch
        );
        assert_eq!(
            ledger.admit(request(
                2,
                1,
                1,
                1,
                1,
                INITIAL_POLICY_EPOCH + 1,
                INITIAL_MEMBERSHIP_EPOCH
            )),
            AdmissionOutcome::StalePolicyEpoch
        );
        assert_eq!(ledger.accepted.len(), 0);
        assert_eq!(ledger.metrics.stale_membership_denials, 1);
        assert_eq!(ledger.metrics.stale_policy_denials, 1);
    }

    #[test]
    fn policy_epoch_increment_never_refunds_group_budget() {
        let scenario = policy_rollover_scenario();
        assert_eq!(scenario.ledger.metrics.accepted_unique, 1);
        assert_eq!(scenario.ledger.metrics.stale_policy_denials, 1);
        assert_eq!(scenario.ledger.metrics.group_budget_denials, 1);
        let audit = scenario.ledger.audit();
        assert_eq!(audit.total_spend, 7);
        assert_eq!(audit.max_group_spend, 7);
    }

    #[test]
    fn retry_of_denied_request_preserves_original_denial_without_respending() {
        let mut ledger = Ledger::new(
            memberships(&[(1, &[100])]),
            BudgetPolicy::standard(),
            EnforcementMode::CompositeScopes,
        );
        let stale = request(91, 1, 1, 1, 1, INITIAL_POLICY_EPOCH, 0);
        assert_eq!(
            ledger.admit(stale.clone()),
            AdmissionOutcome::StaleMembershipEpoch
        );
        assert_eq!(ledger.admit(stale), AdmissionOutcome::StaleMembershipEpoch);
        assert_eq!(ledger.metrics.stale_membership_denials, 1);
        assert_eq!(ledger.metrics.duplicate_replays, 1);
        assert_eq!(ledger.metrics.accepted_unique, 0);
        assert_eq!(ledger.audit().total_spend, 0);
    }

    #[test]
    fn exact_replay_is_idempotent_and_request_id_reuse_conflicts() {
        let mut ledger = Ledger::new(
            memberships(&[(1, &[100])]),
            BudgetPolicy::standard(),
            EnforcementMode::CompositeScopes,
        );
        let original = request(
            44,
            1,
            1,
            2,
            2,
            INITIAL_POLICY_EPOCH,
            INITIAL_MEMBERSHIP_EPOCH,
        );
        assert_eq!(ledger.admit(original.clone()), AdmissionOutcome::Accepted);
        assert_eq!(
            ledger.admit(original.clone()),
            AdmissionOutcome::DuplicateReplay
        );
        let mut changed_cost = original.clone();
        changed_cost.cost = 3;
        let mut changed_digest = original.clone();
        changed_digest.payload_digest ^= 1;
        let mut changed_purpose = original.clone();
        changed_purpose.purpose = 3;
        assert_eq!(ledger.admit(changed_cost), AdmissionOutcome::Conflict);
        assert_eq!(ledger.admit(changed_digest), AdmissionOutcome::Conflict);
        assert_eq!(ledger.admit(changed_purpose), AdmissionOutcome::Conflict);
        assert_eq!(ledger.metrics.accepted_unique, 1);
        assert_eq!(ledger.metrics.duplicate_replays, 1);
        assert_eq!(ledger.metrics.conflicts, 3);
        assert_eq!(ledger.audit().total_spend, 2);
    }

    #[test]
    fn public_membership_edges_are_monotone_and_noop_updates_do_not_bump_epoch() {
        let mut ledger = Ledger::new(
            memberships(&[(1, &[100])]),
            BudgetPolicy::standard(),
            EnforcementMode::CompositeScopes,
        );
        assert_eq!(
            ledger.add_public_memberships(1, &[100]),
            MembershipUpdateOutcome::NoChange
        );
        assert_eq!(ledger.membership_epoch, INITIAL_MEMBERSHIP_EPOCH);
        assert_eq!(
            ledger.add_public_memberships(1, &[200]),
            MembershipUpdateOutcome::Updated
        );
        assert_eq!(ledger.membership_epoch, INITIAL_MEMBERSHIP_EPOCH + 1);
        assert_eq!(
            ledger.memberships.get(&1).unwrap(),
            &BTreeSet::from([100, 200])
        );
        assert_eq!(
            ledger.add_public_memberships(999, &[100]),
            MembershipUpdateOutcome::UnknownSubject
        );
    }

    #[test]
    fn independent_audit_does_not_read_reducer_counters() {
        let mut ledger = composite_scope_scenario().ledger;
        let before = ledger.audit();
        ledger.metrics.accepted_unique = 9000;
        ledger.metrics.denied = 8000;
        let after = ledger.audit();
        assert_eq!(before, after);
        assert_ne!(after.accepted_records, ledger.metrics.accepted_unique);
    }

    #[test]
    fn report_is_deterministic_rectangular_and_includes_audit_fields() {
        let first = render_report();
        let second = render_report();
        assert_eq!(first, second);
        let mut lines = first.lines();
        let header = lines.next().expect("header must exist");
        let width = header.split(',').count();
        assert_eq!(width, 23);
        for column in [
            "audit_group_overrun_count",
            "audit_max_group_spend",
            "latest_rejected_edge_proposed_spend",
        ] {
            assert!(header.contains(column));
        }
        let rows = lines.collect::<Vec<_>>();
        assert_eq!(rows.len(), 5);
        for row in rows {
            assert_eq!(row.split(',').count(), width, "invalid row: {row}");
        }
    }
}
