#![forbid(unsafe_code)]

//! Immutable, revisioned authorization denominator for synthetic service episodes.
//!
//! This crate proves only deterministic set accounting against a caller-supplied
//! sealed plan and typed observations. It does not authenticate the issuer,
//! source truth, field work, professional approval, permits, or external effects.

use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

pub type DigestV1 = [u8; 32];

const PLAN_DOMAIN: &[u8] = b"MYCELIX-SERVICE-ACTION-PLAN-V1\0";
const OBSERVATION_DOMAIN: &[u8] = b"MYCELIX-SERVICE-ACTION-OBSERVATION-V1\0";

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorityRefV1 {
    pub authority_ref: String,
    pub decision_ref: String,
    pub decision_digest: DigestV1,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ServiceScopeV1 {
    pub episode_ref: String,
    pub zone_ref: String,
    pub jurisdiction_ref: String,
    pub context_digest: DigestV1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum ActionRiskV1 {
    Routine,
    Consequential,
    SafetyCritical,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorizedActionV1 {
    pub action_id: String,
    pub action_type: String,
    pub risk: ActionRiskV1,
    pub responsible_role: String,
    pub required_qualification_refs: Vec<String>,
    pub required_permit_refs: Vec<String>,
    pub prerequisite_action_ids: Vec<String>,
    pub evidence_profile_ref: String,
    pub evidence_profile_digest: DigestV1,
    pub minimum_independent_groups: u16,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionPlanDraftV1 {
    pub plan_ref: String,
    pub episode_ref: String,
    pub plan_revision: u64,
    pub supersedes_plan_digest: Option<DigestV1>,
    pub authority: AuthorityRefV1,
    pub scope: ServiceScopeV1,
    pub policy_profile_ref: String,
    pub policy_profile_digest: DigestV1,
    pub issued_at: u64,
    pub valid_until: u64,
    pub actions: Vec<AuthorizedActionV1>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorizedActionPlanV1 {
    draft: ActionPlanDraftV1,
    digest: DigestV1,
}

impl AuthorizedActionPlanV1 {
    /// Canonicalizes the action set and seals the immutable denominator.
    pub fn seal(mut draft: ActionPlanDraftV1) -> Result<Self, PlanErrorV1> {
        validate_plan_header(&draft)?;
        draft.actions.sort_by(|a, b| a.action_id.cmp(&b.action_id));
        for action in &mut draft.actions {
            action.required_qualification_refs.sort();
            action.required_qualification_refs.dedup();
            action.required_permit_refs.sort();
            action.required_permit_refs.dedup();
            action.prerequisite_action_ids.sort();
            action.prerequisite_action_ids.dedup();
        }
        validate_actions(&draft.actions)?;
        let digest = hash_plan(&draft);
        Ok(Self { draft, digest })
    }

    pub fn draft(&self) -> &ActionPlanDraftV1 {
        &self.draft
    }

    pub fn digest(&self) -> DigestV1 {
        self.digest
    }

    pub fn actions(&self) -> &[AuthorizedActionV1] {
        &self.draft.actions
    }

    pub fn plan_ref(&self) -> &str {
        &self.draft.plan_ref
    }

    pub fn episode_ref(&self) -> &str {
        &self.draft.episode_ref
    }

    pub fn revision(&self) -> u64 {
        self.draft.plan_revision
    }

    pub fn scope(&self) -> &ServiceScopeV1 {
        &self.draft.scope
    }

    pub fn validate_integrity(&self) -> Result<(), PlanErrorV1> {
        validate_plan_header(&self.draft)?;
        validate_actions(&self.draft.actions)?;
        if hash_plan(&self.draft) != self.digest {
            return Err(PlanErrorV1::DigestMismatch);
        }
        Ok(())
    }
}

/// Validates an amendment without mutating or invalidating the previous plan.
/// Historical evaluation must continue to use the prior plan digest.
pub fn validate_plan_amendment_v1(
    previous: &AuthorizedActionPlanV1,
    next: &AuthorizedActionPlanV1,
) -> Result<(), PlanErrorV1> {
    previous.validate_integrity()?;
    next.validate_integrity()?;
    if previous.draft.plan_ref != next.draft.plan_ref
        || previous.draft.episode_ref != next.draft.episode_ref
    {
        return Err(PlanErrorV1::AmendmentIdentityMismatch);
    }
    if previous.draft.plan_revision.checked_add(1) != Some(next.draft.plan_revision) {
        return Err(PlanErrorV1::NonSequentialRevision);
    }
    if next.draft.supersedes_plan_digest != Some(previous.digest) {
        return Err(PlanErrorV1::MissingSupersessionBinding);
    }
    if next.draft.authority.decision_digest == previous.draft.authority.decision_digest
        || next.draft.authority.decision_ref == previous.draft.authority.decision_ref
    {
        return Err(PlanErrorV1::AmendmentNeedsNewAuthorityDecision);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum ObservationStatusV1 {
    VerifiedComplete,
    Attempted,
    NotComplete,
    Expired,
    Revoked,
    Disputed,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionObservationV1 {
    pub observation_id: String,
    pub plan_digest: DigestV1,
    pub episode_ref: String,
    pub action_id: String,
    pub zone_ref: String,
    pub jurisdiction_ref: String,
    pub context_digest: DigestV1,
    pub source_ref: String,
    pub source_revision: u64,
    pub evidence_digest: DigestV1,
    pub independence_group: String,
    pub verifier_ref: String,
    pub observed_at: u64,
    pub valid_until: u64,
    pub status: ObservationStatusV1,
}

impl ActionObservationV1 {
    pub fn digest(&self) -> DigestV1 {
        let mut e = Encoder::new(OBSERVATION_DOMAIN);
        e.string(&self.observation_id);
        e.digest(&self.plan_digest);
        e.string(&self.episode_ref);
        e.string(&self.action_id);
        e.string(&self.zone_ref);
        e.string(&self.jurisdiction_ref);
        e.digest(&self.context_digest);
        e.string(&self.source_ref);
        e.u64(self.source_revision);
        e.digest(&self.evidence_digest);
        e.string(&self.independence_group);
        e.string(&self.verifier_ref);
        e.u64(self.observed_at);
        e.u64(self.valid_until);
        e.u8(self.status as u8);
        e.finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RejectionReasonV1 {
    WrongPlan,
    WrongEpisode,
    WrongZone,
    WrongJurisdiction,
    WrongContext,
    UnknownAction,
    MalformedObservation,
    NotYetObserved,
    ExpiredObservation,
    ExplicitRevocation,
    ExplicitExpiry,
    ExplicitDispute,
    NonCompletionStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionCoverageStatusV1 {
    Covered,
    MissingEvidence,
    InsufficientIndependentGroups,
    BlockedByPrerequisite,
    Conflict,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionCoverageV1 {
    pub action_id: String,
    pub required_independent_groups: u16,
    pub observed_independent_groups: u16,
    pub status: ActionCoverageStatusV1,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RejectedObservationV1 {
    pub observation_id: String,
    pub action_id: String,
    pub reason: RejectionReasonV1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoverageDispositionV1 {
    CompleteForAuthorizedPlan,
    Incomplete,
    Conflict,
    Indeterminate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionCoverageReportV1 {
    pub plan_ref: String,
    pub plan_revision: u64,
    pub plan_digest: DigestV1,
    pub disposition: CoverageDispositionV1,
    pub denominator_count: u64,
    pub covered_count: u64,
    /// Integer basis points; never use floating-point percentages for decisions.
    pub coverage_basis_points: u16,
    pub covered_action_ids: Vec<String>,
    pub uncovered_action_ids: Vec<String>,
    pub action_results: Vec<ActionCoverageV1>,
    pub rejected_observations: Vec<RejectedObservationV1>,
    pub identical_duplicate_observations_ignored: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlanErrorV1 {
    EmptyIdentifier,
    ZeroDigest,
    InvalidPlanLifetime,
    InvalidRevisionBinding,
    EmptyActionSet,
    DuplicateActionId,
    UnknownPrerequisite,
    SelfDependency,
    DependencyCycle,
    InvalidMinimumIndependentGroups,
    InvalidAction,
    DigestMismatch,
    AmendmentIdentityMismatch,
    NonSequentialRevision,
    MissingSupersessionBinding,
    AmendmentNeedsNewAuthorityDecision,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EvaluationErrorV1 {
    InvalidPlan(PlanErrorV1),
    ConflictingObservationIdentity,
    ArithmeticOverflow,
}

pub fn evaluate_action_coverage_v1(
    plan: &AuthorizedActionPlanV1,
    observations: &[ActionObservationV1],
    now: u64,
) -> Result<ActionCoverageReportV1, EvaluationErrorV1> {
    plan.validate_integrity()
        .map_err(EvaluationErrorV1::InvalidPlan)?;

    if now < plan.draft.issued_at || now >= plan.draft.valid_until {
        return Ok(empty_report(plan, CoverageDispositionV1::Indeterminate));
    }

    let action_ids: BTreeSet<&str> = plan
        .actions()
        .iter()
        .map(|a| a.action_id.as_str())
        .collect();
    let mut seen_observation_ids = BTreeMap::<&str, DigestV1>::new();
    let mut duplicates_ignored = 0_u64;
    let mut rejected = Vec::new();
    let mut by_action = BTreeMap::<&str, BTreeMap<&str, Vec<&ActionObservationV1>>>::new();
    let mut unavailable_groups = BTreeMap::<&str, BTreeSet<&str>>::new();
    let mut disputed_groups = BTreeMap::<&str, BTreeSet<&str>>::new();

    for observation in observations {
        let digest = observation.digest();
        if let Some(previous) = seen_observation_ids.get(observation.observation_id.as_str()) {
            if *previous == digest {
                duplicates_ignored = duplicates_ignored
                    .checked_add(1)
                    .ok_or(EvaluationErrorV1::ArithmeticOverflow)?;
                continue;
            }
            return Err(EvaluationErrorV1::ConflictingObservationIdentity);
        }
        seen_observation_ids.insert(&observation.observation_id, digest);

        let reason = if observation.observation_id.trim().is_empty()
            || observation.action_id.trim().is_empty()
            || observation.source_ref.trim().is_empty()
            || observation.independence_group.trim().is_empty()
            || observation.verifier_ref.trim().is_empty()
            || observation.source_revision == 0
            || observation.evidence_digest == [0; 32]
            || observation.plan_digest == [0; 32]
            || observation.context_digest == [0; 32]
            || observation.valid_until <= observation.observed_at
        {
            Some(RejectionReasonV1::MalformedObservation)
        } else if observation.plan_digest != plan.digest {
            Some(RejectionReasonV1::WrongPlan)
        } else if observation.episode_ref != plan.episode_ref() {
            Some(RejectionReasonV1::WrongEpisode)
        } else if observation.zone_ref != plan.scope().zone_ref {
            Some(RejectionReasonV1::WrongZone)
        } else if observation.jurisdiction_ref != plan.scope().jurisdiction_ref {
            Some(RejectionReasonV1::WrongJurisdiction)
        } else if observation.context_digest != plan.scope().context_digest {
            Some(RejectionReasonV1::WrongContext)
        } else if !action_ids.contains(observation.action_id.as_str()) {
            Some(RejectionReasonV1::UnknownAction)
        } else if observation.observed_at > now {
            Some(RejectionReasonV1::NotYetObserved)
        } else if observation.valid_until <= now {
            Some(RejectionReasonV1::ExpiredObservation)
        } else if observation.status == ObservationStatusV1::Revoked {
            Some(RejectionReasonV1::ExplicitRevocation)
        } else if observation.status == ObservationStatusV1::Expired {
            Some(RejectionReasonV1::ExplicitExpiry)
        } else if observation.status == ObservationStatusV1::Disputed {
            Some(RejectionReasonV1::ExplicitDispute)
        } else if observation.status != ObservationStatusV1::VerifiedComplete {
            Some(RejectionReasonV1::NonCompletionStatus)
        } else {
            None
        };

        if let Some(reason) = reason {
            match reason {
                RejectionReasonV1::ExplicitRevocation | RejectionReasonV1::ExplicitExpiry => {
                    unavailable_groups
                        .entry(observation.action_id.as_str())
                        .or_default()
                        .insert(observation.independence_group.as_str());
                }
                RejectionReasonV1::ExplicitDispute => {
                    disputed_groups
                        .entry(observation.action_id.as_str())
                        .or_default()
                        .insert(observation.independence_group.as_str());
                }
                _ => {}
            }
            rejected.push(RejectedObservationV1 {
                observation_id: observation.observation_id.clone(),
                action_id: observation.action_id.clone(),
                reason,
            });
            continue;
        }

        by_action
            .entry(observation.action_id.as_str())
            .or_default()
            .entry(observation.independence_group.as_str())
            .or_default()
            .push(observation);
    }

    let mut group_counts = BTreeMap::<String, u16>::new();
    let mut group_conflicts = BTreeSet::<String>::new();
    for action in plan.actions() {
        let groups = by_action.get(action.action_id.as_str());
        let mut valid_group_count = 0_u16;
        if disputed_groups
            .get(action.action_id.as_str())
            .is_some_and(|groups| !groups.is_empty())
        {
            group_conflicts.insert(action.action_id.clone());
        }
        if let Some(groups) = groups {
            for (group_id, group_observations) in groups {
                let unique: BTreeSet<DigestV1> = group_observations
                    .iter()
                    .map(|o| o.evidence_digest)
                    .collect();
                let unavailable = unavailable_groups
                    .get(action.action_id.as_str())
                    .is_some_and(|groups| groups.contains(group_id));
                let disputed = disputed_groups
                    .get(action.action_id.as_str())
                    .is_some_and(|groups| groups.contains(group_id));
                if unique.len() > 1 || disputed {
                    group_conflicts.insert(action.action_id.clone());
                } else if !unavailable && !group_observations.is_empty() {
                    valid_group_count = valid_group_count
                        .checked_add(1)
                        .ok_or(EvaluationErrorV1::ArithmeticOverflow)?;
                }
            }
        }
        group_counts.insert(action.action_id.clone(), valid_group_count);
    }

    let mut covered = BTreeSet::<String>::new();
    loop {
        let mut progress = false;
        for action in plan.actions() {
            if covered.contains(&action.action_id) {
                continue;
            }
            if group_conflicts.contains(&action.action_id) {
                continue;
            }
            if !action
                .prerequisite_action_ids
                .iter()
                .all(|id| covered.contains(id))
            {
                continue;
            }
            let groups = *group_counts.get(&action.action_id).unwrap_or(&0);
            if groups >= action.minimum_independent_groups {
                covered.insert(action.action_id.clone());
                progress = true;
            }
        }
        if !progress {
            break;
        }
    }

    let mut action_results = Vec::with_capacity(plan.actions().len());
    let mut uncovered = Vec::new();
    for action in plan.actions() {
        let status = if covered.contains(&action.action_id) {
            ActionCoverageStatusV1::Covered
        } else if group_conflicts.contains(&action.action_id) {
            ActionCoverageStatusV1::Conflict
        } else if !action
            .prerequisite_action_ids
            .iter()
            .all(|id| covered.contains(id))
        {
            ActionCoverageStatusV1::BlockedByPrerequisite
        } else {
            let groups = *group_counts.get(&action.action_id).unwrap_or(&0);
            if groups == 0 {
                ActionCoverageStatusV1::MissingEvidence
            } else {
                ActionCoverageStatusV1::InsufficientIndependentGroups
            }
        };
        if status != ActionCoverageStatusV1::Covered {
            uncovered.push(action.action_id.clone());
        }
        action_results.push(ActionCoverageV1 {
            action_id: action.action_id.clone(),
            required_independent_groups: action.minimum_independent_groups,
            observed_independent_groups: *group_counts.get(&action.action_id).unwrap_or(&0),
            status,
        });
    }

    let denominator =
        u64::try_from(plan.actions().len()).map_err(|_| EvaluationErrorV1::ArithmeticOverflow)?;
    let covered_count =
        u64::try_from(covered.len()).map_err(|_| EvaluationErrorV1::ArithmeticOverflow)?;
    let basis_points = covered_count
        .checked_mul(10_000)
        .ok_or(EvaluationErrorV1::ArithmeticOverflow)?
        .checked_div(denominator)
        .ok_or(EvaluationErrorV1::ArithmeticOverflow)?;
    let disposition = if !group_conflicts.is_empty() {
        CoverageDispositionV1::Conflict
    } else if covered_count == denominator {
        CoverageDispositionV1::CompleteForAuthorizedPlan
    } else {
        CoverageDispositionV1::Incomplete
    };

    rejected.sort_by(|a, b| {
        a.action_id
            .cmp(&b.action_id)
            .then(a.observation_id.cmp(&b.observation_id))
            .then((a.reason as u8).cmp(&(b.reason as u8)))
    });

    Ok(ActionCoverageReportV1 {
        plan_ref: plan.plan_ref().to_owned(),
        plan_revision: plan.revision(),
        plan_digest: plan.digest(),
        disposition,
        denominator_count: denominator,
        covered_count,
        coverage_basis_points: basis_points as u16,
        covered_action_ids: covered.into_iter().collect(),
        uncovered_action_ids: uncovered,
        action_results,
        rejected_observations: rejected,
        identical_duplicate_observations_ignored: duplicates_ignored,
    })
}

fn empty_report(
    plan: &AuthorizedActionPlanV1,
    disposition: CoverageDispositionV1,
) -> ActionCoverageReportV1 {
    let action_results = plan
        .actions()
        .iter()
        .map(|action| ActionCoverageV1 {
            action_id: action.action_id.clone(),
            required_independent_groups: action.minimum_independent_groups,
            observed_independent_groups: 0,
            status: ActionCoverageStatusV1::MissingEvidence,
        })
        .collect();
    ActionCoverageReportV1 {
        plan_ref: plan.plan_ref().to_owned(),
        plan_revision: plan.revision(),
        plan_digest: plan.digest(),
        disposition,
        denominator_count: plan.actions().len() as u64,
        covered_count: 0,
        coverage_basis_points: 0,
        covered_action_ids: Vec::new(),
        uncovered_action_ids: plan.actions().iter().map(|a| a.action_id.clone()).collect(),
        action_results,
        rejected_observations: Vec::new(),
        identical_duplicate_observations_ignored: 0,
    }
}

fn validate_plan_header(draft: &ActionPlanDraftV1) -> Result<(), PlanErrorV1> {
    for value in [
        draft.plan_ref.as_str(),
        draft.episode_ref.as_str(),
        draft.authority.authority_ref.as_str(),
        draft.authority.decision_ref.as_str(),
        draft.scope.episode_ref.as_str(),
        draft.scope.zone_ref.as_str(),
        draft.scope.jurisdiction_ref.as_str(),
        draft.policy_profile_ref.as_str(),
    ] {
        require_identifier(value)?;
    }
    if draft.scope.episode_ref != draft.episode_ref
        || draft.authority.decision_digest == [0; 32]
        || draft.scope.context_digest == [0; 32]
        || draft.policy_profile_digest == [0; 32]
    {
        return Err(PlanErrorV1::ZeroDigest);
    }
    if draft.valid_until <= draft.issued_at {
        return Err(PlanErrorV1::InvalidPlanLifetime);
    }
    if (draft.plan_revision == 1 && draft.supersedes_plan_digest.is_some())
        || (draft.plan_revision > 1 && draft.supersedes_plan_digest.is_none())
        || draft.plan_revision == 0
    {
        return Err(PlanErrorV1::InvalidRevisionBinding);
    }
    Ok(())
}

fn validate_actions(actions: &[AuthorizedActionV1]) -> Result<(), PlanErrorV1> {
    if actions.is_empty() {
        return Err(PlanErrorV1::EmptyActionSet);
    }
    let mut ids = BTreeSet::new();
    for action in actions {
        require_identifier(&action.action_id)?;
        require_identifier(&action.action_type)?;
        require_identifier(&action.responsible_role)?;
        require_identifier(&action.evidence_profile_ref)?;
        if action.evidence_profile_digest == [0; 32] || action.minimum_independent_groups == 0 {
            return Err(PlanErrorV1::InvalidMinimumIndependentGroups);
        }
        for reference in action
            .required_qualification_refs
            .iter()
            .chain(action.required_permit_refs.iter())
            .chain(action.prerequisite_action_ids.iter())
        {
            require_identifier(reference)?;
        }
        if !ids.insert(action.action_id.as_str()) {
            return Err(PlanErrorV1::DuplicateActionId);
        }
    }
    let known: BTreeSet<&str> = actions.iter().map(|a| a.action_id.as_str()).collect();
    for action in actions {
        for prerequisite in &action.prerequisite_action_ids {
            if prerequisite == &action.action_id {
                return Err(PlanErrorV1::SelfDependency);
            }
            if !known.contains(prerequisite.as_str()) {
                return Err(PlanErrorV1::UnknownPrerequisite);
            }
        }
    }
    validate_acyclic(actions)?;
    Ok(())
}

fn validate_acyclic(actions: &[AuthorizedActionV1]) -> Result<(), PlanErrorV1> {
    let mut indegree = BTreeMap::<&str, usize>::new();
    let mut dependents = BTreeMap::<&str, Vec<&str>>::new();
    for action in actions {
        indegree.insert(
            action.action_id.as_str(),
            action.prerequisite_action_ids.len(),
        );
        for prerequisite in &action.prerequisite_action_ids {
            dependents
                .entry(prerequisite.as_str())
                .or_default()
                .push(action.action_id.as_str());
        }
    }
    let mut ready = VecDeque::new();
    for (action_id, degree) in &indegree {
        if *degree == 0 {
            ready.push_back(*action_id);
        }
    }
    let mut visited = 0_usize;
    while let Some(action_id) = ready.pop_front() {
        visited += 1;
        for dependent in dependents.get(action_id).into_iter().flatten() {
            let degree = indegree.get_mut(dependent).expect("known dependent");
            *degree -= 1;
            if *degree == 0 {
                ready.push_back(dependent);
            }
        }
    }
    if visited == actions.len() {
        Ok(())
    } else {
        Err(PlanErrorV1::DependencyCycle)
    }
}

fn require_identifier(value: &str) -> Result<(), PlanErrorV1> {
    if value.trim().is_empty() || value.len() > 1024 || value.trim() != value {
        Err(PlanErrorV1::EmptyIdentifier)
    } else {
        Ok(())
    }
}

struct Encoder {
    bytes: Vec<u8>,
}

impl Encoder {
    fn new(domain: &[u8]) -> Self {
        Self {
            bytes: domain.to_vec(),
        }
    }
    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }
    fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }
    fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }
    fn string(&mut self, value: &str) {
        self.u64(value.len() as u64);
        self.bytes.extend_from_slice(value.as_bytes());
    }
    fn digest(&mut self, value: &DigestV1) {
        self.bytes.extend_from_slice(value);
    }
    fn option_digest(&mut self, value: Option<&DigestV1>) {
        match value {
            Some(digest) => {
                self.u8(1);
                self.digest(digest);
            }
            None => self.u8(0),
        }
    }
    fn finish(self) -> DigestV1 {
        Sha256::digest(self.bytes).into()
    }
}

fn hash_plan(draft: &ActionPlanDraftV1) -> DigestV1 {
    let mut e = Encoder::new(PLAN_DOMAIN);
    e.string(&draft.plan_ref);
    e.string(&draft.episode_ref);
    e.u64(draft.plan_revision);
    e.option_digest(draft.supersedes_plan_digest.as_ref());
    e.string(&draft.authority.authority_ref);
    e.string(&draft.authority.decision_ref);
    e.digest(&draft.authority.decision_digest);
    e.string(&draft.scope.episode_ref);
    e.string(&draft.scope.zone_ref);
    e.string(&draft.scope.jurisdiction_ref);
    e.digest(&draft.scope.context_digest);
    e.string(&draft.policy_profile_ref);
    e.digest(&draft.policy_profile_digest);
    e.u64(draft.issued_at);
    e.u64(draft.valid_until);
    e.u64(draft.actions.len() as u64);
    for action in &draft.actions {
        e.string(&action.action_id);
        e.string(&action.action_type);
        e.u8(action.risk as u8);
        e.string(&action.responsible_role);
        e.u64(action.required_qualification_refs.len() as u64);
        for reference in &action.required_qualification_refs {
            e.string(reference);
        }
        e.u64(action.required_permit_refs.len() as u64);
        for reference in &action.required_permit_refs {
            e.string(reference);
        }
        e.u64(action.prerequisite_action_ids.len() as u64);
        for prerequisite in &action.prerequisite_action_ids {
            e.string(prerequisite);
        }
        e.string(&action.evidence_profile_ref);
        e.digest(&action.evidence_profile_digest);
        e.u16(action.minimum_independent_groups);
    }
    e.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> DigestV1 {
        [byte; 32]
    }

    fn action(id: &str, prerequisites: Vec<&str>, minimum_groups: u16) -> AuthorizedActionV1 {
        AuthorizedActionV1 {
            action_id: id.to_owned(),
            action_type: format!("service-action:{id}"),
            risk: ActionRiskV1::Consequential,
            responsible_role: "municipal-maintainer".to_owned(),
            required_qualification_refs: vec!["qualification:current".to_owned()],
            required_permit_refs: vec!["permit:current".to_owned()],
            prerequisite_action_ids: prerequisites.into_iter().map(str::to_owned).collect(),
            evidence_profile_ref: "water-repair-evidence-v1".to_owned(),
            evidence_profile_digest: digest(90),
            minimum_independent_groups: minimum_groups,
        }
    }

    fn draft(actions: Vec<AuthorizedActionV1>) -> ActionPlanDraftV1 {
        ActionPlanDraftV1 {
            plan_ref: "wsr-plan:episode-1".to_owned(),
            episode_ref: "wsr-episode:1".to_owned(),
            plan_revision: 1,
            supersedes_plan_digest: None,
            authority: AuthorityRefV1 {
                authority_ref: "authority:local-water-office".to_owned(),
                decision_ref: "authority-decision:1".to_owned(),
                decision_digest: digest(1),
            },
            scope: ServiceScopeV1 {
                episode_ref: "wsr-episode:1".to_owned(),
                zone_ref: "zone:7".to_owned(),
                jurisdiction_ref: "jurisdiction:za-gp".to_owned(),
                context_digest: digest(2),
            },
            policy_profile_ref: "water-service-policy-v1".to_owned(),
            policy_profile_digest: digest(3),
            issued_at: 10,
            valid_until: 1000,
            actions,
        }
    }

    fn observation(
        plan: &AuthorizedActionPlanV1,
        id: &str,
        action_id: &str,
        group: &str,
        observed_at: u64,
        valid_until: u64,
    ) -> ActionObservationV1 {
        ActionObservationV1 {
            observation_id: id.to_owned(),
            plan_digest: plan.digest(),
            episode_ref: plan.episode_ref().to_owned(),
            action_id: action_id.to_owned(),
            zone_ref: plan.scope().zone_ref.clone(),
            jurisdiction_ref: plan.scope().jurisdiction_ref.clone(),
            context_digest: plan.scope().context_digest,
            source_ref: format!("source:{group}"),
            source_revision: 1,
            evidence_digest: digest(group.as_bytes()[0]),
            independence_group: group.to_owned(),
            verifier_ref: "verifier:water-review".to_owned(),
            observed_at,
            valid_until,
            status: ObservationStatusV1::VerifiedComplete,
        }
    }

    #[test]
    fn sha256_known_answer() {
        let actual: DigestV1 = Sha256::digest(b"abc").into();
        assert_eq!(
            actual,
            [
                0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
                0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
                0xf2, 0x00, 0x15, 0xad,
            ]
        );
    }

    #[test]
    fn canonical_plan_digest_is_independent_of_input_order() {
        let a = AuthorizedActionPlanV1::seal(draft(vec![
            action("action:b", vec!["action:a"], 1),
            action("action:a", vec![], 1),
        ]))
        .unwrap();
        let b = AuthorizedActionPlanV1::seal(draft(vec![
            action("action:a", vec![], 1),
            action("action:b", vec!["action:a"], 1),
        ]))
        .unwrap();
        assert_eq!(a.digest(), b.digest());
    }

    #[test]
    fn six_authorized_actions_with_five_observed_remain_incomplete() {
        let plan = AuthorizedActionPlanV1::seal(draft(
            (1..=6)
                .map(|i| action(&format!("action:{i}"), vec![], 1))
                .collect(),
        ))
        .unwrap();
        let observations: Vec<_> = (1..=5)
            .map(|i| {
                observation(
                    &plan,
                    &format!("obs:{i}"),
                    &format!("action:{i}"),
                    &format!("independent:{i}"),
                    100,
                    900,
                )
            })
            .collect();
        let report = evaluate_action_coverage_v1(&plan, &observations, 200).unwrap();
        assert_eq!(report.denominator_count, 6);
        assert_eq!(report.covered_count, 5);
        assert_eq!(report.coverage_basis_points, 8333);
        assert_eq!(report.disposition, CoverageDispositionV1::Incomplete);
        assert_eq!(report.uncovered_action_ids, vec!["action:6"]);
    }

    #[test]
    fn identical_duplicate_observation_is_idempotent() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let item = observation(&plan, "obs:1", "action:1", "independent:1", 100, 900);
        let report = evaluate_action_coverage_v1(&plan, &[item.clone(), item], 200).unwrap();
        assert_eq!(report.covered_count, 1);
        assert_eq!(report.identical_duplicate_observations_ignored, 1);
    }

    #[test]
    fn different_observation_payload_under_same_id_is_conflict() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let first = observation(&plan, "obs:1", "action:1", "independent:1", 100, 900);
        let mut second = first.clone();
        second.evidence_digest = digest(44);
        assert_eq!(
            evaluate_action_coverage_v1(&plan, &[first, second], 200),
            Err(EvaluationErrorV1::ConflictingObservationIdentity)
        );
    }

    #[test]
    fn duplicate_independence_group_does_not_inflate_numerator() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 2)])).unwrap();
        let first = observation(&plan, "obs:1", "action:1", "same-meter-cluster", 100, 900);
        let mut second = observation(&plan, "obs:2", "action:1", "same-meter-cluster", 110, 900);
        second.evidence_digest = first.evidence_digest;
        second.source_ref = first.source_ref.clone();
        let report = evaluate_action_coverage_v1(&plan, &[first, second], 200).unwrap();
        assert_eq!(report.covered_count, 0);
        assert_eq!(
            report.action_results[0].status,
            ActionCoverageStatusV1::InsufficientIndependentGroups
        );
    }

    #[test]
    fn wrong_zone_observation_is_rejected_and_not_counted() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let mut item = observation(
            &plan,
            "obs:wrong-zone",
            "action:1",
            "independent:1",
            100,
            900,
        );
        item.zone_ref = "zone:other".to_owned();
        let report = evaluate_action_coverage_v1(&plan, &[item], 200).unwrap();
        assert_eq!(report.covered_count, 0);
        assert_eq!(
            report.rejected_observations[0].reason,
            RejectionReasonV1::WrongZone
        );
    }

    #[test]
    fn expired_or_revoked_observations_never_count_as_coverage() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let mut expired = observation(&plan, "obs:expired", "action:1", "source:1", 100, 150);
        expired.status = ObservationStatusV1::Expired;
        let report = evaluate_action_coverage_v1(&plan, &[expired], 200).unwrap();
        assert_eq!(report.covered_count, 0);
    }

    #[test]
    fn explicit_revocation_invalidates_prior_positive_in_same_group() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let positive = observation(&plan, "obs:positive", "action:1", "group:meter-1", 100, 900);
        let mut revoked = observation(&plan, "obs:revoked", "action:1", "group:meter-1", 150, 900);
        revoked.status = ObservationStatusV1::Revoked;
        let report = evaluate_action_coverage_v1(&plan, &[positive, revoked], 200).unwrap();
        assert_eq!(report.covered_count, 0);
        assert_eq!(
            report.action_results[0].status,
            ActionCoverageStatusV1::MissingEvidence
        );
        assert!(report
            .rejected_observations
            .iter()
            .any(|r| r.reason == RejectionReasonV1::ExplicitRevocation));
    }

    #[test]
    fn explicit_expiry_invalidates_prior_positive_in_same_group() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let positive = observation(&plan, "obs:positive", "action:1", "group:meter-1", 100, 900);
        let mut expired = observation(
            &plan,
            "obs:expired-marker",
            "action:1",
            "group:meter-1",
            150,
            900,
        );
        expired.status = ObservationStatusV1::Expired;
        let report = evaluate_action_coverage_v1(&plan, &[positive, expired], 200).unwrap();
        assert_eq!(report.covered_count, 0);
        assert!(report
            .rejected_observations
            .iter()
            .any(|r| r.reason == RejectionReasonV1::ExplicitExpiry));
    }

    #[test]
    fn explicit_dispute_surfaces_conflict_for_the_action() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let mut disputed =
            observation(&plan, "obs:disputed", "action:1", "group:meter-1", 150, 900);
        disputed.status = ObservationStatusV1::Disputed;
        let report = evaluate_action_coverage_v1(&plan, &[disputed], 200).unwrap();
        assert_eq!(report.disposition, CoverageDispositionV1::Conflict);
        assert_eq!(
            report.action_results[0].status,
            ActionCoverageStatusV1::Conflict
        );
    }

    #[test]
    fn unsatisfied_prerequisite_blocks_dependent_action() {
        let plan = AuthorizedActionPlanV1::seal(draft(vec![
            action("action:1", vec![], 1),
            action("action:2", vec!["action:1"], 1),
        ]))
        .unwrap();
        let second = observation(&plan, "obs:2", "action:2", "independent:2", 100, 900);
        let report = evaluate_action_coverage_v1(&plan, &[second], 200).unwrap();
        assert_eq!(report.covered_count, 0);
        assert_eq!(
            report
                .action_results
                .iter()
                .find(|a| a.action_id == "action:2")
                .unwrap()
                .status,
            ActionCoverageStatusV1::BlockedByPrerequisite
        );
    }

    #[test]
    fn denominator_amendment_requires_new_authority_revision_and_preserves_history() {
        let previous = AuthorizedActionPlanV1::seal(draft(vec![
            action("action:1", vec![], 1),
            action("action:2", vec![], 1),
        ]))
        .unwrap();
        let mut amended_draft = draft(vec![action("action:1", vec![], 1)]);
        amended_draft.plan_revision = 2;
        amended_draft.supersedes_plan_digest = Some(previous.digest());
        amended_draft.authority.decision_ref = "authority-decision:2".to_owned();
        amended_draft.authority.decision_digest = digest(4);
        let amended = AuthorizedActionPlanV1::seal(amended_draft).unwrap();
        validate_plan_amendment_v1(&previous, &amended).unwrap();

        let prior_observation =
            observation(&previous, "obs:1", "action:1", "independent:1", 100, 900);
        let prior_report =
            evaluate_action_coverage_v1(&previous, &[prior_observation.clone()], 200).unwrap();
        assert_eq!(prior_report.denominator_count, 2);
        assert_eq!(prior_report.covered_count, 1);

        let attempted_new_plan = AuthorizedActionPlanV1::seal({
            let mut draft = draft(vec![action("action:1", vec![], 1)]);
            draft.plan_revision = 2;
            draft.supersedes_plan_digest = Some(previous.digest());
            draft
        })
        .unwrap();
        assert_eq!(
            validate_plan_amendment_v1(&previous, &attempted_new_plan),
            Err(PlanErrorV1::AmendmentNeedsNewAuthorityDecision)
        );
        let replay =
            evaluate_action_coverage_v1(&previous, &[prior_observation.clone()], 200).unwrap();
        assert_eq!(replay, prior_report);

        // An observation admitted under P1 cannot be carried forward to P2,
        // even though P2 has a smaller authorized set.
        let p2_with_p1_observation =
            evaluate_action_coverage_v1(&amended, &[prior_observation], 200).unwrap();
        assert_eq!(p2_with_p1_observation.covered_count, 0);
        assert_eq!(
            p2_with_p1_observation.rejected_observations[0].reason,
            RejectionReasonV1::WrongPlan
        );
    }

    #[test]
    fn cyclic_action_prerequisites_are_rejected_when_plan_is_sealed() {
        let result = AuthorizedActionPlanV1::seal(draft(vec![
            action("action:a", vec!["action:b"], 1),
            action("action:b", vec!["action:a"], 1),
        ]));
        assert_eq!(result, Err(PlanErrorV1::DependencyCycle));
    }

    #[test]
    fn duplicate_action_ids_are_rejected_when_plan_is_sealed() {
        let result = AuthorizedActionPlanV1::seal(draft(vec![
            action("action:a", vec![], 1),
            action("action:a", vec![], 1),
        ]));
        assert_eq!(result, Err(PlanErrorV1::DuplicateActionId));
    }

    #[test]
    fn plan_is_indeterminate_outside_its_authorized_lifetime() {
        let plan =
            AuthorizedActionPlanV1::seal(draft(vec![action("action:1", vec![], 1)])).unwrap();
        let before_issue = evaluate_action_coverage_v1(&plan, &[], 9).unwrap();
        let after_expiry = evaluate_action_coverage_v1(&plan, &[], 1000).unwrap();
        assert_eq!(
            before_issue.disposition,
            CoverageDispositionV1::Indeterminate
        );
        assert_eq!(
            after_expiry.disposition,
            CoverageDispositionV1::Indeterminate
        );
    }

    #[test]
    fn same_count_with_different_actions_has_different_denominator_digest() {
        let first = AuthorizedActionPlanV1::seal(draft(vec![
            action("repair:valve", vec![], 1),
            action("test:pressure", vec![], 1),
        ]))
        .unwrap();
        let second = AuthorizedActionPlanV1::seal(draft(vec![
            action("repair:pipe", vec![], 1),
            action("test:pressure", vec![], 1),
        ]))
        .unwrap();
        assert_eq!(first.actions().len(), second.actions().len());
        assert_ne!(first.digest(), second.digest());
    }
}
