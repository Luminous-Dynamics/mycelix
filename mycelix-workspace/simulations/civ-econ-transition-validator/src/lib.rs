//! Dependency-free, non-authoritative reference checks for CIV-ECON-001A.
//!
//! This validates typed invariants and lifecycle gates. It does not parse the
//! JSON Schema, hash manifests, verify signatures, query live ledgers, prove
//! legal authority/capacity, or establish external settlement finality.

use std::collections::{HashMap, HashSet};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Violation {
    EmptyField(&'static str),
    InvalidDigest(&'static str),
    EmptyScope,
    DuplicateScopeItem(String),
    DuplicateMapping(String),
    MissingMapping(String),
    OutOfScopeMapping(String),
    MappingKindMismatch(String),
    MissingMappingField { item: String, field: &'static str },
    InvalidDispositionForKind(String),
    InvalidAtomicQuantity(&'static str),
    OriginChanged(String),
    PreservedSemanticsChanged(String),
    UnexpectedTargetMapping(String),
    DuplicatePrincipal(String),
    ForbiddenRoleOverlap(String),
    InvalidTransition { from: Stage, to: Stage },
    FrozenPlanMismatch,
    StaleSourceSnapshot,
    SimulationInputMismatch,
    MissingSimulationOutput,
    MissingSimulationReceipt,
    SimulationCorpusMismatch,
    EvaluationInputMismatch,
    EvaluationOutputMismatch,
    EvaluationNotIndependent,
    QualificationFailed,
    MissingEvaluationReceipt,
    MissingEvaluationReportDigest,
    MissingEvaluationEnvironment,
    MissingApprovalReceipt,
    EvaluatorIdentityMismatch,
    ApprovalInputMismatch,
    ApprovalAuthorityMismatch,
    ApprovalNotCurrent,
    PilotControlsMissing,
    RightsFloorNotSatisfied,
    ResourceReservationsUnreconciled,
    ExternalObligationsUnaccounted,
    CutoverEffectsNotStarted,
    EffectReceiptsNotDurable,
    EffectFrontierUnresolved,
    ReconciliationSubjectMismatch,
    ReconciliationEvidenceMissing,
    ReconciliationDigestChanged,
    ReconciliationFailed,
    MandatoryUnresolvedItems(usize),
    AbortAfterPossibleEffect,
    PossibleEffectMustBeIndeterminate,
    IndeterminateWithoutPossibleEffect,
    QuarantineReasonMissing,
    EffectIdEmpty,
    EffectRequestDigestInvalid,
    TerminalState,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProfileRef {
    pub id: String,
    pub version: String,
    pub content_digest: String,
    pub authority_ref: String,
}

impl ProfileRef {
    pub fn validate(&self, field: &'static str) -> Result<(), Violation> {
        if self.id.trim().is_empty() || self.version.trim().is_empty() {
            return Err(Violation::EmptyField(field));
        }
        if self.authority_ref.trim().is_empty() {
            return Err(Violation::EmptyField("profile authority_ref"));
        }
        validate_digest(&self.content_digest, field)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InstrumentRef {
    pub instrument_id: String,
    pub version: String,
    pub content_digest: String,
    pub issuer_ref: String,
    pub economic_form_ref: String,
    pub source_profile_ref: Option<ProfileRef>,
}

impl InstrumentRef {
    fn validate(&self) -> Result<(), Violation> {
        if self.instrument_id.trim().is_empty() || self.version.trim().is_empty() {
            return Err(Violation::EmptyField("instrument identity"));
        }
        if self.issuer_ref.trim().is_empty() {
            return Err(Violation::EmptyField("instrument issuer_ref"));
        }
        if self.economic_form_ref.trim().is_empty() {
            return Err(Violation::EmptyField("instrument economic_form_ref"));
        }
        validate_digest(&self.content_digest, "instrument content_digest")?;
        if let Some(profile) = &self.source_profile_ref {
            profile.validate("instrument source_profile_ref")?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourceSnapshot {
    pub frontier_ref: String,
    pub state_digest: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ItemKind {
    Claim,
    Obligation,
    Entitlement,
    Resource,
    Reservation,
    Dispute,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnitRef {
    pub unit_id: String,
    pub version: String,
    pub content_digest: String,
    pub dimension: String,
    /// Positive count of atomic units per display unit, encoded as canonical decimal digits.
    pub atomic_scale: String,
}

impl UnitRef {
    fn validate(&self) -> Result<(), Violation> {
        if self.unit_id.trim().is_empty() || self.version.trim().is_empty() {
            return Err(Violation::EmptyField("unit identity"));
        }
        validate_digest(&self.content_digest, "unit content_digest")?;
        if self.dimension.trim().is_empty() {
            return Err(Violation::EmptyField("unit dimension"));
        }
        let parsed = self.atomic_scale.parse::<u64>()
            .map_err(|_| Violation::InvalidAtomicQuantity("unit.atomic_scale"))?;
        if parsed == 0 || parsed.to_string() != self.atomic_scale {
            return Err(Violation::InvalidAtomicQuantity("unit.atomic_scale"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EconomicIdentity {
    /// Immutable provenance identity; conversion does not silently replace it.
    pub origin_ref: String,
    /// Absent for purely physical resources; present for monetary claim instruments.
    pub instrument_ref: Option<InstrumentRef>,
    pub unit_ref: UnitRef,
}

impl EconomicIdentity {
    fn validate(&self, kind: ItemKind) -> Result<(), Violation> {
        if self.origin_ref.trim().is_empty() {
            return Err(Violation::EmptyField("origin_ref"));
        }
        self.unit_ref.validate()?;
        if let Some(instrument) = &self.instrument_ref {
            instrument.validate()?;
        } else if matches!(kind, ItemKind::Claim | ItemKind::Obligation) {
            return Err(Violation::EmptyField("instrument_profile"));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Disposition {
    Preserve,
    Convert,
    Novate,
    Discharge,
    Freeze,
    Dispute,
    Quarantine,
    RetainSourceOnly,
    ReReserve,
    Release,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScopedItem {
    pub id: String,
    pub kind: ItemKind,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ItemMapping {
    pub source_item_id: String,
    pub kind: ItemKind,
    pub disposition: Disposition,
    pub source_identity: EconomicIdentity,
    pub source_quantity_atomic: String,
    pub target_identity: Option<EconomicIdentity>,
    pub target_quantity_atomic: Option<String>,
    pub target_ref: Option<String>,
    pub conversion_profile_ref: Option<ProfileRef>,
    pub authority_decision_ref: Option<String>,
    pub counterparty_acceptance_ref: Option<String>,
    pub discharge_receipt_ref: Option<String>,
}

impl ItemMapping {
    fn validate(&self) -> Result<(), Violation> {
        if self.source_item_id.trim().is_empty() {
            return Err(Violation::EmptyField("source_item_id"));
        }
        self.source_identity.validate(self.kind)?;
        validate_atomic_quantity(&self.source_quantity_atomic, "source_quantity_atomic")?;

        match (&self.target_identity, &self.target_quantity_atomic) {
            (Some(identity), Some(quantity)) => {
                identity.validate(self.kind)?;
                validate_atomic_quantity(quantity, "target_quantity_atomic")?;
            }
            (None, None) => {}
            (Some(_), None) => {
                return Err(Violation::MissingMappingField {
                    item: self.source_item_id.clone(),
                    field: "target_quantity_atomic",
                });
            }
            (None, Some(_)) => {
                return Err(Violation::MissingMappingField {
                    item: self.source_item_id.clone(),
                    field: "target_identity",
                });
            }
        }

        match self.disposition {
            Disposition::Preserve => {
                self.require(self.target_ref.as_deref(), "target_ref")?;
                self.require_target()?;
                self.require_same_identity_and_quantity()?;
            }
            Disposition::Convert => {
                self.require(self.target_ref.as_deref(), "target_ref")?;
                self.conversion_profile_ref
                    .as_ref()
                    .ok_or(Violation::MissingMappingField {
                        item: self.source_item_id.clone(),
                        field: "conversion_profile_ref",
                    })?
                    .validate("conversion_profile_ref")?;
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
                self.require_target()?;
                let target = self.target_identity.as_ref().expect("required above");
                if target.origin_ref != self.source_identity.origin_ref {
                    return Err(Violation::OriginChanged(self.source_item_id.clone()));
                }
            }
            Disposition::Novate => {
                self.require(self.target_ref.as_deref(), "target_ref")?;
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
                self.require(self.counterparty_acceptance_ref.as_deref(), "counterparty_acceptance_ref")?;
                self.require_target()?;
                let target = self.target_identity.as_ref().expect("required above");
                if target.origin_ref != self.source_identity.origin_ref {
                    return Err(Violation::OriginChanged(self.source_item_id.clone()));
                }
                if target.instrument_ref != self.source_identity.instrument_ref
                    || target.unit_ref != self.source_identity.unit_ref
                    || self.target_quantity_atomic.as_deref() != Some(self.source_quantity_atomic.as_str())
                {
                    return Err(Violation::PreservedSemanticsChanged(self.source_item_id.clone()));
                }
            }
            Disposition::Discharge => {
                self.require(self.discharge_receipt_ref.as_deref(), "discharge_receipt_ref")?;
                self.require_no_target()?;
            }
            Disposition::ReReserve => {
                if !matches!(self.kind, ItemKind::Resource | ItemKind::Reservation) {
                    return Err(Violation::InvalidDispositionForKind(self.source_item_id.clone()));
                }
                self.require(self.target_ref.as_deref(), "target_ref")?;
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
                self.require_target()?;
                self.require_same_identity_and_quantity()?;
            }
            Disposition::Release => {
                if !matches!(self.kind, ItemKind::Resource | ItemKind::Reservation) {
                    return Err(Violation::InvalidDispositionForKind(self.source_item_id.clone()));
                }
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
                self.require_no_target()?;
            }
            Disposition::Freeze | Disposition::Dispute => {
                if self.target_identity.is_some() {
                    self.require_same_identity_and_quantity()?;
                }
            }
            Disposition::Quarantine | Disposition::RetainSourceOnly => {
                self.require_no_target()?;
            }
        }
        Ok(())
    }

    fn require(&self, value: Option<&str>, field: &'static str) -> Result<(), Violation> {
        if value.map(str::trim).filter(|s| !s.is_empty()).is_none() {
            return Err(Violation::MissingMappingField {
                item: self.source_item_id.clone(),
                field,
            });
        }
        Ok(())
    }

    fn require_target(&self) -> Result<(), Violation> {
        if self.target_identity.is_none() {
            return Err(Violation::MissingMappingField {
                item: self.source_item_id.clone(),
                field: "target_identity",
            });
        }
        if self.target_quantity_atomic.is_none() {
            return Err(Violation::MissingMappingField {
                item: self.source_item_id.clone(),
                field: "target_quantity_atomic",
            });
        }
        Ok(())
    }

    fn require_no_target(&self) -> Result<(), Violation> {
        if self.target_ref.is_some()
            || self.target_identity.is_some()
            || self.target_quantity_atomic.is_some()
        {
            return Err(Violation::UnexpectedTargetMapping(self.source_item_id.clone()));
        }
        Ok(())
    }

    fn require_same_identity_and_quantity(&self) -> Result<(), Violation> {
        if self.target_identity.as_ref() != Some(&self.source_identity)
            || self.target_quantity_atomic.as_ref() != Some(&self.source_quantity_atomic)
        {
            return Err(Violation::PreservedSemanticsChanged(self.source_item_id.clone()));
        }
        Ok(())
    }
}

fn validate_atomic_quantity(value: &str, field: &'static str) -> Result<(), Violation> {
    if value.is_empty() || value == "-0" {
        return Err(Violation::InvalidAtomicQuantity(field));
    }
    let digits = value.strip_prefix('-').unwrap_or(value);
    if digits.is_empty()
        || !digits.bytes().all(|byte| byte.is_ascii_digit())
        || (digits.len() > 1 && digits.starts_with('0'))
    {
        return Err(Violation::InvalidAtomicQuantity(field));
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthorityRoles {
    pub proposer: String,
    pub evaluator: String,
    pub approver: String,
    pub executor: String,
    pub reconciler: String,
    /// A separately verified, risk-qualified exception may permit role overlap.
    pub separation_exception_ref: Option<String>,
}

impl AuthorityRoles {
    fn validate(&self) -> Result<(), Violation> {
        let roles = [
            ("proposer", self.proposer.as_str()),
            ("evaluator", self.evaluator.as_str()),
            ("approver", self.approver.as_str()),
            ("executor", self.executor.as_str()),
            ("reconciler", self.reconciler.as_str()),
        ];
        let never_combined = [
            ("proposer", self.proposer.as_str(), "evaluator", self.evaluator.as_str()),
            ("proposer", self.proposer.as_str(), "approver", self.approver.as_str()),
            ("proposer", self.proposer.as_str(), "reconciler", self.reconciler.as_str()),
            ("evaluator", self.evaluator.as_str(), "approver", self.approver.as_str()),
            ("evaluator", self.evaluator.as_str(), "executor", self.executor.as_str()),
            ("evaluator", self.evaluator.as_str(), "reconciler", self.reconciler.as_str()),
            ("approver", self.approver.as_str(), "executor", self.executor.as_str()),
            ("approver", self.approver.as_str(), "reconciler", self.reconciler.as_str()),
            ("executor", self.executor.as_str(), "reconciler", self.reconciler.as_str()),
        ];
        for (left_role, left, right_role, right) in never_combined {
            if left == right {
                return Err(Violation::ForbiddenRoleOverlap(format!("{left_role}+{right_role}:{left}")));
            }
        }

        let mut seen: HashMap<&str, &str> = HashMap::new();
        let mut duplicates = Vec::new();

        for (role, principal) in roles {
            if principal.trim().is_empty() {
                return Err(Violation::EmptyField("authority principal"));
            }
            if let Some(first_role) = seen.insert(principal, role) {
                duplicates.push(format!("{first_role}+{role}:{principal}"));
            }
        }

        if let Some(first) = duplicates.first() {
            let has_exception = self.separation_exception_ref.as_deref()
                .map(str::trim).filter(|s| !s.is_empty()).is_some();
            if !has_exception {
                return Err(Violation::DuplicatePrincipal(first.clone()));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransitionManifest {
    pub transition_id: String,
    pub source_profile: ProfileRef,
    pub target_profile: ProfileRef,
    pub source_snapshot: SourceSnapshot,
    pub plan_digest: String,
    pub scope: Vec<ScopedItem>,
    pub mappings: Vec<ItemMapping>,
    pub authorities: AuthorityRoles,
}

impl TransitionManifest {
    pub fn validate(&self) -> Result<(), Violation> {
        if self.transition_id.trim().is_empty() {
            return Err(Violation::EmptyField("transition_id"));
        }
        self.source_profile.validate("source_profile")?;
        self.target_profile.validate("target_profile")?;
        if self.source_snapshot.frontier_ref.trim().is_empty() {
            return Err(Violation::EmptyField("source_snapshot.frontier_ref"));
        }
        validate_digest(&self.source_snapshot.state_digest, "source_snapshot.state_digest")?;
        validate_digest(&self.plan_digest, "plan_digest")?;
        self.authorities.validate()?;

        if self.scope.is_empty() {
            return Err(Violation::EmptyScope);
        }
        let mut scoped: HashMap<&str, ItemKind> = HashMap::new();
        for item in &self.scope {
            if item.id.trim().is_empty() {
                return Err(Violation::EmptyField("scope item id"));
            }
            if scoped.insert(item.id.as_str(), item.kind).is_some() {
                return Err(Violation::DuplicateScopeItem(item.id.clone()));
            }
        }

        let mut mapped = HashSet::new();
        for mapping in &self.mappings {
            if !mapped.insert(mapping.source_item_id.as_str()) {
                return Err(Violation::DuplicateMapping(mapping.source_item_id.clone()));
            }
            let Some(kind) = scoped.get(mapping.source_item_id.as_str()) else {
                return Err(Violation::OutOfScopeMapping(mapping.source_item_id.clone()));
            };
            if *kind != mapping.kind {
                return Err(Violation::MappingKindMismatch(mapping.source_item_id.clone()));
            }
            mapping.validate()?;
        }

        for item in &self.scope {
            if !mapped.contains(item.id.as_str()) {
                return Err(Violation::MissingMapping(item.id.clone()));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Draft,
    Frozen,
    Simulated,
    Qualified,
    Authorized,
    PilotActive,
    CutoverReady,
    CutoverCommitted,
    Reconciled,
    Completed,
    Rejected,
    Aborted,
    Quarantined,
    Indeterminate,
}

impl Stage {
    fn is_terminal(self) -> bool {
        matches!(self, Stage::Completed | Stage::Rejected | Stage::Aborted | Stage::Quarantined)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StageEvidence {
    pub frozen_plan_digest: Option<String>,
    pub source_snapshot_current: bool,
    pub simulation_input_digest: Option<String>,
    pub simulation_output_digest: Option<String>,
    pub simulation_receipt_ref: Option<String>,
    pub simulation_scenario_corpus_digest: Option<String>,
    pub simulation_environment_ref: Option<String>,
    pub evaluation_plan_digest: Option<String>,
    /// Digest of the candidate output being evaluated, not the evaluation report.
    pub evaluation_subject_digest: Option<String>,
    pub evaluation_scenario_corpus_digest: Option<String>,
    pub evaluation_environment_ref: Option<String>,
    pub evaluation_report_digest: Option<String>,
    pub evaluation_passed: bool,
    pub evaluation_receipt_ref: Option<String>,
    pub independent_oracle: bool,
    pub evaluator_id: Option<String>,
    pub approval_plan_digest: Option<String>,
    pub approver_id: Option<String>,
    pub approval_current: bool,
    pub approval_receipt_ref: Option<String>,
    pub pilot_controls_present: bool,
    pub rights_floors_satisfied: bool,
    pub resource_reservations_reconciled: bool,
    /// Every external obligation is resolved or explicitly dispositioned.
    pub external_obligations_accounted: bool,
    pub cutover_effects_started: bool,
    pub effect_receipts_durable: bool,
    pub effect_frontier_resolved: bool,
    pub reconciliation_plan_digest: Option<String>,
    pub reconciliation_digest: Option<String>,
    pub reconciliation_passed: bool,
    pub unresolved_mandatory_items: usize,
    pub quarantine_reason: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StageEvent {
    pub from: Stage,
    pub to: Stage,
    pub plan_digest: String,
    /// Monotonic safety facts at this transition boundary.
    pub effects_may_have_started: bool,
    pub effect_frontier_resolved: bool,
}

pub struct TransitionMachine {
    manifest: TransitionManifest,
    stage: Stage,
    simulation_output_digest: Option<String>,
    simulation_scenario_corpus_digest: Option<String>,
    reconciliation_digest: Option<String>,
    reconciliation_passed: bool,
    unresolved_mandatory_items: usize,
    /// Once true, later callers cannot erase possible execution by passing weaker evidence.
    effects_may_have_started: bool,
    /// Set only after the Reconciled stage validates a resolved effect frontier.
    effect_frontier_resolved: bool,
    history: Vec<StageEvent>,
}

impl TransitionMachine {
    pub fn new(manifest: TransitionManifest) -> Self {
        Self {
            manifest,
            stage: Stage::Draft,
            simulation_output_digest: None,
            simulation_scenario_corpus_digest: None,
            reconciliation_digest: None,
            reconciliation_passed: false,
            unresolved_mandatory_items: 0,
            effects_may_have_started: false,
            effect_frontier_resolved: false,
            history: Vec::new(),
        }
    }

    pub fn manifest(&self) -> &TransitionManifest { &self.manifest }
    pub fn stage(&self) -> Stage { self.stage }
    pub fn history(&self) -> &[StageEvent] { &self.history }

    pub fn advance(&mut self, next: Stage, ev: &StageEvidence) -> Result<(), Violation> {
        if self.stage.is_terminal() {
            return Err(Violation::TerminalState);
        }

        // Treat any credible indication that an effect may have started as
        // monotonic. A later request cannot erase it by supplying weaker evidence.
        if ev.cutover_effects_started {
            self.effects_may_have_started = true;
            // A newly reported possible effect invalidates any prior resolved
            // frontier until the Reconciled gate validates it again.
            self.effect_frontier_resolved = false;
        }

        if next == Stage::Quarantined {
            if self.effects_may_have_started && !self.effect_frontier_resolved {
                return Err(Violation::PossibleEffectMustBeIndeterminate);
            }
            if ev.quarantine_reason.as_deref().map(str::trim).filter(|s| !s.is_empty()).is_none() {
                return Err(Violation::QuarantineReasonMissing);
            }
            return self.record_stage(next);
        }
        if next == Stage::Rejected || next == Stage::Aborted {
            if !matches!(self.stage, Stage::Draft | Stage::Frozen | Stage::Simulated | Stage::Qualified | Stage::Authorized | Stage::PilotActive | Stage::CutoverReady) {
                return Err(Violation::InvalidTransition { from: self.stage, to: next });
            }
            if self.effects_may_have_started {
                return Err(Violation::AbortAfterPossibleEffect);
            }
            return self.record_stage(next);
        }
        if next == Stage::Indeterminate {
            if !self.effects_may_have_started || self.effect_frontier_resolved {
                return Err(Violation::IndeterminateWithoutPossibleEffect);
            }
            // Indeterminate is an exceptional recovery state, not a normal
            // forward transition. It must remain reachable if an effect may
            // have escaped early or its acknowledgement was lost.
            return self.record_stage(next);
        }

        // Once an effect may have escaped, the machine cannot continue normal
        // preparation or declare a terminal outcome. It may retry the precise
        // cutover effect, reconcile from the committed boundary, or enter
        // Indeterminate. Caller-supplied weaker booleans cannot clear this.
        let effect_recovery_transition = matches!(
            (self.stage, next),
            (Stage::CutoverReady, Stage::CutoverCommitted)
                | (Stage::CutoverCommitted, Stage::Reconciled)
                | (Stage::Indeterminate, Stage::Reconciled)
        );
        if self.effects_may_have_started && !self.effect_frontier_resolved
            && !effect_recovery_transition
        {
            return Err(Violation::PossibleEffectMustBeIndeterminate);
        }

        let allowed = matches!(
            (self.stage, next),
            (Stage::Draft, Stage::Frozen)
                | (Stage::Frozen, Stage::Simulated)
                | (Stage::Simulated, Stage::Qualified)
                | (Stage::Qualified, Stage::Authorized)
                | (Stage::Authorized, Stage::PilotActive)
                | (Stage::Authorized, Stage::CutoverReady)
                | (Stage::PilotActive, Stage::CutoverReady)
                | (Stage::CutoverReady, Stage::CutoverCommitted)
                | (Stage::CutoverCommitted, Stage::Reconciled)
                | (Stage::Indeterminate, Stage::Reconciled)
                | (Stage::Reconciled, Stage::Completed)
        );
        if !allowed {
            return Err(Violation::InvalidTransition { from: self.stage, to: next });
        }

        match (self.stage, next) {
            (Stage::Draft, Stage::Frozen) => {
                self.manifest.validate()?;
                if ev.frozen_plan_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::FrozenPlanMismatch);
                }
            }
            (Stage::Frozen, Stage::Simulated) => {
                require_current_snapshot(ev)?;
                if ev.simulation_input_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::SimulationInputMismatch);
                }
                let output = ev.simulation_output_digest.as_deref().ok_or(Violation::MissingSimulationOutput)?;
                validate_digest(output, "simulation_output_digest")?;
                if ev.simulation_receipt_ref.as_deref().map(str::trim).filter(|s| !s.is_empty()).is_none() {
                    return Err(Violation::MissingSimulationReceipt);
                }
                let corpus = ev.simulation_scenario_corpus_digest.as_deref()
                    .ok_or(Violation::MissingSimulationOutput)?;
                validate_digest(corpus, "simulation_scenario_corpus_digest")?;
                ev.simulation_environment_ref.as_deref()
                    .map(str::trim).filter(|s| !s.is_empty())
                    .ok_or(Violation::MissingSimulationOutput)?;
                self.simulation_output_digest = Some(output.to_owned());
                self.simulation_scenario_corpus_digest = Some(corpus.to_owned());
            }
            (Stage::Simulated, Stage::Qualified) => {
                require_current_snapshot(ev)?;
                if ev.evaluation_plan_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::EvaluationInputMismatch);
                }
                if ev.evaluation_subject_digest != self.simulation_output_digest {
                    return Err(Violation::EvaluationOutputMismatch);
                }
                if ev.evaluation_scenario_corpus_digest != self.simulation_scenario_corpus_digest {
                    return Err(Violation::SimulationCorpusMismatch);
                }
                if ev.evaluation_environment_ref.as_deref().map(str::trim).filter(|s| !s.is_empty()).is_none() {
                    return Err(Violation::MissingEvaluationEnvironment);
                }
                if !ev.independent_oracle {
                    return Err(Violation::EvaluationNotIndependent);
                }
                if ev.evaluator_id.as_deref() != Some(self.manifest.authorities.evaluator.as_str()) {
                    return Err(Violation::EvaluatorIdentityMismatch);
                }
                if !ev.evaluation_passed {
                    return Err(Violation::QualificationFailed);
                }
                if ev.evaluation_receipt_ref.as_deref().map(str::trim).filter(|s| !s.is_empty()).is_none() {
                    return Err(Violation::MissingEvaluationReceipt);
                }
                let report = ev.evaluation_report_digest.as_deref()
                    .ok_or(Violation::MissingEvaluationReportDigest)?;
                validate_digest(report, "evaluation_report_digest")?;
            }
            (Stage::Qualified, Stage::Authorized) => {
                require_current_snapshot(ev)?;
                if ev.approval_plan_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::ApprovalInputMismatch);
                }
                if ev.approver_id.as_deref() != Some(self.manifest.authorities.approver.as_str()) {
                    return Err(Violation::ApprovalAuthorityMismatch);
                }
                if !ev.approval_current {
                    return Err(Violation::ApprovalNotCurrent);
                }
                if ev.approval_receipt_ref.as_deref().map(str::trim).filter(|s| !s.is_empty()).is_none() {
                    return Err(Violation::MissingApprovalReceipt);
                }
            }
            (Stage::Authorized, Stage::PilotActive) => {
                require_current_snapshot(ev)?;
                if !ev.pilot_controls_present {
                    return Err(Violation::PilotControlsMissing);
                }
            }
            (Stage::Authorized, Stage::CutoverReady) | (Stage::PilotActive, Stage::CutoverReady) => {
                require_current_snapshot(ev)?;
                if !ev.rights_floors_satisfied { return Err(Violation::RightsFloorNotSatisfied); }
                if !ev.resource_reservations_reconciled { return Err(Violation::ResourceReservationsUnreconciled); }
                if !ev.external_obligations_accounted { return Err(Violation::ExternalObligationsUnaccounted); }
                self.manifest.validate()?;
            }
            (Stage::CutoverReady, Stage::CutoverCommitted) => {
                if !ev.cutover_effects_started { return Err(Violation::CutoverEffectsNotStarted); }
                if !ev.effect_receipts_durable { return Err(Violation::EffectReceiptsNotDurable); }
            }
            (Stage::CutoverCommitted, Stage::Reconciled) | (Stage::Indeterminate, Stage::Reconciled) => {
                if !ev.effect_frontier_resolved { return Err(Violation::EffectFrontierUnresolved); }
                if ev.reconciliation_plan_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::ReconciliationSubjectMismatch);
                }
                let digest = ev.reconciliation_digest.as_deref().ok_or(Violation::ReconciliationEvidenceMissing)?;
                validate_digest(digest, "reconciliation_digest")?;
                self.reconciliation_digest = Some(digest.to_owned());
                self.reconciliation_passed = ev.reconciliation_passed;
                self.unresolved_mandatory_items = ev.unresolved_mandatory_items;
                // Crossing this gate means the uncertain effect frontier has
                // been explicitly reconciled, even if the reconciliation failed.
                self.effect_frontier_resolved = true;
            }
            (Stage::Reconciled, Stage::Completed) => {
                if !ev.effect_frontier_resolved { return Err(Violation::EffectFrontierUnresolved); }
                if ev.reconciliation_plan_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::ReconciliationEvidenceMissing);
                }
                let digest = ev.reconciliation_digest.as_deref().ok_or(Violation::ReconciliationEvidenceMissing)?;
                validate_digest(digest, "reconciliation_digest")
                    .map_err(|_| Violation::ReconciliationEvidenceMissing)?;
                if Some(digest) != self.reconciliation_digest.as_deref() {
                    return Err(Violation::ReconciliationDigestChanged);
                }
                if !self.reconciliation_passed || !ev.reconciliation_passed {
                    return Err(Violation::ReconciliationFailed);
                }
                let unresolved = ev.unresolved_mandatory_items.max(self.unresolved_mandatory_items);
                if unresolved != 0 { return Err(Violation::MandatoryUnresolvedItems(unresolved)); }
            }
            _ => unreachable!("transition checked above"),
        }

        self.record_stage(next)
    }

    fn record_stage(&mut self, next: Stage) -> Result<(), Violation> {
        self.history.push(StageEvent {
            from: self.stage,
            to: next,
            plan_digest: self.manifest.plan_digest.clone(),
            effects_may_have_started: self.effects_may_have_started,
            effect_frontier_resolved: self.effect_frontier_resolved,
        });
        self.stage = next;
        Ok(())
    }
}

fn require_current_snapshot(ev: &StageEvidence) -> Result<(), Violation> {
    if ev.source_snapshot_current { Ok(()) } else { Err(Violation::StaleSourceSnapshot) }
}

pub fn validate_digest(digest: &str, field: &'static str) -> Result<(), Violation> {
    let Some(hex) = digest.strip_prefix("sha256:") else {
        return Err(Violation::InvalidDigest(field));
    };
    if hex.len() != 64 || !hex.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)) {
        return Err(Violation::InvalidDigest(field));
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecordEffectResult { New, ReplaySameRequest, Conflict }

#[derive(Default)]
pub struct EffectLedger {
    requests: HashMap<String, String>,
}

impl EffectLedger {
    /// Records a stable effect identity and canonical request digest.
    /// The caller must derive effect_id from the attempt and semantic action.
    pub fn record(&mut self, id: &str, request_digest: &str) -> Result<RecordEffectResult, Violation> {
        if id.trim().is_empty() { return Err(Violation::EffectIdEmpty); }
        validate_digest(request_digest, "effect_request_digest")
            .map_err(|_| Violation::EffectRequestDigestInvalid)?;
        match self.requests.get(id) {
            None => {
                self.requests.insert(id.to_owned(), request_digest.to_owned());
                Ok(RecordEffectResult::New)
            }
            Some(old) if old == request_digest => Ok(RecordEffectResult::ReplaySameRequest),
            Some(_) => Ok(RecordEffectResult::Conflict),
        }
    }
}

/// Length-prefixed identity avoids ambiguity from separators inside input fields.
/// It names an effect; it does not authorize or prove the effect.
pub fn effect_id(attempt: &str, action: &str, kind: &str) -> Result<String, Violation> {
    if attempt.trim().is_empty() || action.trim().is_empty() || kind.trim().is_empty() {
        return Err(Violation::EffectIdEmpty);
    }
    Ok(format!("{}:{}{}:{}{}:{}", attempt.len(), attempt, action.len(), action, kind.len(), kind))
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: &str = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const B: &str = "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const C: &str = "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

    fn profile(id: &str, digest: &str) -> ProfileRef {
        ProfileRef {
            id: id.into(),
            version: "1.0.0".into(),
            content_digest: digest.into(),
            authority_ref: format!("authority:{id}"),
        }
    }

    fn identity(origin: &str, kind: ItemKind) -> EconomicIdentity {
        let instrument_ref = if matches!(kind, ItemKind::Claim | ItemKind::Obligation | ItemKind::Entitlement | ItemKind::Dispute) {
            Some(InstrumentRef {
                instrument_id: "instrument:synthetic".into(),
                version: "1.0.0".into(),
                content_digest: C.into(),
                issuer_ref: "issuer:synthetic".into(),
                economic_form_ref: "synthetic-claim".into(),
                source_profile_ref: Some(profile("profile:synthetic-instrument", A)),
            })
        } else {
            None
        };
        let dimension = if instrument_ref.is_some() { "currency:synthetic" } else { "resource:capacity" };
        EconomicIdentity {
            origin_ref: origin.into(),
            instrument_ref,
            unit_ref: UnitRef {
                unit_id: "unit:synthetic".into(),
                version: "1.0.0".into(),
                content_digest: B.into(),
                dimension: dimension.into(),
                atomic_scale: "1000000".into(),
            },
        }
    }

    fn mapping(id: &str, kind: ItemKind, disposition: Disposition) -> ItemMapping {
        let source_identity = identity(&format!("origin:{id}"), kind);
        let with_target = matches!(
            disposition,
            Disposition::Preserve | Disposition::Convert | Disposition::Novate
                | Disposition::Freeze | Disposition::Dispute | Disposition::ReReserve
        );
        ItemMapping {
            source_item_id: id.into(),
            kind,
            disposition,
            source_identity: source_identity.clone(),
            source_quantity_atomic: "100".into(),
            target_identity: with_target.then_some(source_identity),
            target_quantity_atomic: with_target.then(|| "100".into()),
            target_ref: with_target.then(|| format!("target:{id}")),
            conversion_profile_ref: None,
            authority_decision_ref: None,
            counterparty_acceptance_ref: None,
            discharge_receipt_ref: None,
        }
    }

    fn manifest() -> TransitionManifest {
        TransitionManifest {
            transition_id: "t-1".into(),
            source_profile: profile("market", A),
            target_profile: profile("mixed-commons", B),
            source_snapshot: SourceSnapshot { frontier_ref: "frontier-1".into(), state_digest: C.into() },
            plan_digest: A.into(),
            scope: vec![
                ScopedItem { id: "claim-1".into(), kind: ItemKind::Claim },
                ScopedItem { id: "reservation-1".into(), kind: ItemKind::Reservation },
            ],
            mappings: vec![
                mapping("claim-1", ItemKind::Claim, Disposition::Preserve),
                ItemMapping {
                    target_ref: Some("target-reservation-1".into()),
                    authority_decision_ref: Some("approval-reserve-1".into()),
                    ..mapping("reservation-1", ItemKind::Reservation, Disposition::ReReserve)
                },
            ],
            authorities: AuthorityRoles {
                proposer: "p".into(), evaluator: "e".into(), approver: "a".into(),
                executor: "x".into(), reconciler: "r".into(), separation_exception_ref: None,
            },
        }
    }

    fn ev() -> StageEvidence {
        StageEvidence { source_snapshot_current: true, ..StageEvidence::default() }
    }

    fn advance_to_authorized(machine: &mut TransitionMachine) {
        let mut e = ev(); e.frozen_plan_digest = Some(A.into());
        machine.advance(Stage::Frozen, &e).unwrap();
        let mut e = ev(); e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into()); e.simulation_receipt_ref = Some("simulation-receipt-1".into()); e.simulation_scenario_corpus_digest = Some(B.into()); e.simulation_environment_ref = Some("simulation-env-v1".into());
        machine.advance(Stage::Simulated, &e).unwrap();
        let mut e = ev(); e.evaluation_plan_digest = Some(A.into()); e.evaluation_subject_digest = Some(C.into()); e.evaluation_scenario_corpus_digest = Some(B.into()); e.evaluation_environment_ref = Some("verifier-env-v1".into()); e.evaluation_report_digest = Some(A.into());
        e.evaluation_passed = true; e.evaluation_receipt_ref = Some("eval-receipt-1".into()); e.independent_oracle = true; e.evaluator_id = Some("e".into());
        machine.advance(Stage::Qualified, &e).unwrap();
        let mut e = ev(); e.approval_plan_digest = Some(A.into()); e.approver_id = Some("a".into());
        e.approval_current = true; e.approval_receipt_ref = Some("approval-receipt-1".into()); machine.advance(Stage::Authorized, &e).unwrap();
    }

    #[test] fn accepts_valid_manifest() { assert_eq!(manifest().validate(), Ok(())); }

    #[test] fn rejects_uppercase_or_malformed_digest() {
        assert_eq!(validate_digest(&format!("sha256:{}", "A".repeat(64)), "d"), Err(Violation::InvalidDigest("d")));
        assert_eq!(validate_digest("sha256:xyz", "d"), Err(Violation::InvalidDigest("d")));
    }

    #[test] fn rejects_missing_mapping() {
        let mut m = manifest(); m.mappings.pop();
        assert_eq!(m.validate(), Err(Violation::MissingMapping("reservation-1".into())));
    }

    #[test] fn rejects_duplicate_mapping() {
        let mut m = manifest(); m.mappings.push(m.mappings[0].clone());
        assert_eq!(m.validate(), Err(Violation::DuplicateMapping("claim-1".into())));
    }

    #[test] fn rejects_out_of_scope_mapping() {
        let mut m = manifest(); m.mappings.push(mapping("extra", ItemKind::Claim, Disposition::Preserve));
        assert_eq!(m.validate(), Err(Violation::OutOfScopeMapping("extra".into())));
    }

    #[test] fn conversion_requires_versioned_conversion_profile_and_authority() {
        let mut m = manifest(); m.mappings[0].disposition = Disposition::Convert;
        assert_eq!(m.validate(), Err(Violation::MissingMappingField { item: "claim-1".into(), field: "conversion_profile_ref" }));
    }

    #[test] fn preserve_rejects_unit_semantic_drift() {
        let mut m = manifest();
        m.mappings[0].target_identity.as_mut().unwrap().unit_ref.dimension = "energy".into();
        assert_eq!(m.validate(), Err(Violation::PreservedSemanticsChanged("claim-1".into())));
    }

    #[test] fn conversion_preserves_origin_even_when_instrument_changes() {
        let mut m = manifest();
        let mapping = &mut m.mappings[0];
        mapping.disposition = Disposition::Convert;
        mapping.conversion_profile_ref = Some(profile("conversion-policy-v1", C));
        mapping.authority_decision_ref = Some("authority-approval-1".into());
        mapping.target_identity.as_mut().unwrap().origin_ref = "foreign-origin".into();
        mapping.target_identity.as_mut().unwrap().instrument_ref.as_mut().unwrap().instrument_id = "target-instrument".into();
        assert_eq!(m.validate(), Err(Violation::OriginChanged("claim-1".into())));
    }

    #[test] fn novation_cannot_hide_a_quantity_change() {
        let mut m = manifest();
        let mapping = &mut m.mappings[0];
        mapping.disposition = Disposition::Novate;
        mapping.authority_decision_ref = Some("authority-approval-1".into());
        mapping.counterparty_acceptance_ref = Some("counterparty-accepted-1".into());
        mapping.target_quantity_atomic = Some("99".into());
        assert_eq!(m.validate(), Err(Violation::PreservedSemanticsChanged("claim-1".into())));
    }

    #[test] fn atomic_quantities_reject_floats_and_noncanonical_integers() {
        assert_eq!(validate_atomic_quantity("1.5", "q"), Err(Violation::InvalidAtomicQuantity("q")));
        assert_eq!(validate_atomic_quantity("01", "q"), Err(Violation::InvalidAtomicQuantity("q")));
        assert_eq!(validate_atomic_quantity("-0", "q"), Err(Violation::InvalidAtomicQuantity("q")));
        assert_eq!(validate_atomic_quantity("-12", "q"), Ok(()));
    }

    #[test] fn unit_scale_must_be_canonical_positive_integer() {
        let mut identity = identity("origin:unit-test", ItemKind::Claim);
        identity.unit_ref.atomic_scale = "01".into();
        assert_eq!(identity.validate(ItemKind::Claim), Err(Violation::InvalidAtomicQuantity("unit.atomic_scale")));
    }

    #[test] fn novation_requires_counterparty_acceptance() {
        let mut m = manifest(); let x = &mut m.mappings[0];
        x.disposition = Disposition::Novate; x.target_ref = Some("target".into()); x.authority_decision_ref = Some("approval".into());
        assert_eq!(m.validate(), Err(Violation::MissingMappingField { item: "claim-1".into(), field: "counterparty_acceptance_ref" }));
    }

    #[test] fn discharge_requires_receipt() {
        let mut m = manifest();
        m.mappings[0].disposition = Disposition::Discharge;
        m.mappings[0].target_identity = None;
        m.mappings[0].target_quantity_atomic = None;
        m.mappings[0].target_ref = None;
        assert_eq!(m.validate(), Err(Violation::MissingMappingField { item: "claim-1".into(), field: "discharge_receipt_ref" }));
    }

    #[test] fn limited_role_overlap_needs_exception_reference() {
        let mut m = manifest(); m.authorities.executor = m.authorities.proposer.clone();
        assert!(matches!(m.validate(), Err(Violation::DuplicatePrincipal(_))));
        m.authorities.separation_exception_ref = Some("risk-exception".into());
        assert_eq!(m.validate(), Ok(()));
    }

    #[test] fn separation_exception_cannot_authorize_self_evaluation() {
        let mut m = manifest();
        m.authorities.evaluator = m.authorities.proposer.clone();
        m.authorities.separation_exception_ref = Some("risk-exception".into());
        assert!(matches!(m.validate(), Err(Violation::ForbiddenRoleOverlap(_))));
    }

    #[test] fn rejects_stage_skip() {
        let mut m = TransitionMachine::new(manifest());
        assert_eq!(m.advance(Stage::Authorized, &ev()), Err(Violation::InvalidTransition { from: Stage::Draft, to: Stage::Authorized }));
    }

    #[test] fn frozen_state_binds_manifest_digest() {
        let mut m = TransitionMachine::new(manifest()); let mut e = ev(); e.frozen_plan_digest = Some(B.into());
        assert_eq!(m.advance(Stage::Frozen, &e), Err(Violation::FrozenPlanMismatch));
        assert_eq!(m.stage(), Stage::Draft);
    }

    #[test] fn stale_source_blocks_simulation() {
        let mut m = TransitionMachine::new(manifest()); let mut e = ev(); e.frozen_plan_digest = Some(A.into());
        m.advance(Stage::Frozen, &e).unwrap();
        let mut e = ev(); e.source_snapshot_current = false; e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into()); e.simulation_receipt_ref = Some("simulation-receipt-1".into()); e.simulation_scenario_corpus_digest = Some(B.into()); e.simulation_environment_ref = Some("simulation-env-v1".into());
        assert_eq!(m.advance(Stage::Simulated, &e), Err(Violation::StaleSourceSnapshot));
    }

    #[test] fn simulation_requires_durable_run_receipt() {
        let mut m = TransitionMachine::new(manifest());
        let mut freeze = ev(); freeze.frozen_plan_digest = Some(A.into());
        m.advance(Stage::Frozen, &freeze).unwrap();
        let mut sim = ev();
        sim.simulation_input_digest = Some(A.into());
        sim.simulation_output_digest = Some(C.into());
        sim.simulation_scenario_corpus_digest = Some(B.into());
        sim.simulation_environment_ref = Some("simulation-env-v1".into());
        assert_eq!(m.advance(Stage::Simulated, &sim), Err(Violation::MissingSimulationReceipt));
    }

    #[test] fn evaluator_must_use_the_same_scenario_corpus() {
        let mut m = TransitionMachine::new(manifest());
        let mut freeze = ev(); freeze.frozen_plan_digest = Some(A.into());
        m.advance(Stage::Frozen, &freeze).unwrap();
        let mut sim = ev();
        sim.simulation_input_digest = Some(A.into());
        sim.simulation_output_digest = Some(C.into());
        sim.simulation_receipt_ref = Some("simulation-receipt".into());
        sim.simulation_scenario_corpus_digest = Some(B.into());
        sim.simulation_environment_ref = Some("simulation-env-v1".into());
        m.advance(Stage::Simulated, &sim).unwrap();
        let mut eval = ev();
        eval.evaluation_plan_digest = Some(A.into());
        eval.evaluation_subject_digest = Some(C.into());
        eval.evaluation_scenario_corpus_digest = Some(C.into());
        eval.evaluation_environment_ref = Some("verifier-env-v1".into());
        eval.evaluation_report_digest = Some(A.into());
        eval.evaluation_receipt_ref = Some("evidence-receipt".into());
        eval.evaluation_passed = true;
        eval.independent_oracle = true;
        eval.evaluator_id = Some("e".into());
        assert_eq!(m.advance(Stage::Qualified, &eval), Err(Violation::SimulationCorpusMismatch));
    }

    #[test] fn qualification_requires_report_digest_separate_from_subject_digest() {
        let mut m = TransitionMachine::new(manifest());
        let mut freeze = ev(); freeze.frozen_plan_digest = Some(A.into());
        m.advance(Stage::Frozen, &freeze).unwrap();
        let mut sim = ev();
        sim.simulation_input_digest = Some(A.into());
        sim.simulation_output_digest = Some(C.into());
        sim.simulation_receipt_ref = Some("simulation-receipt".into());
        sim.simulation_scenario_corpus_digest = Some(B.into());
        sim.simulation_environment_ref = Some("simulation-env-v1".into());
        m.advance(Stage::Simulated, &sim).unwrap();
        let mut eval = ev();
        eval.evaluation_plan_digest = Some(A.into());
        eval.evaluation_subject_digest = Some(C.into());
        eval.evaluation_scenario_corpus_digest = Some(B.into());
        eval.evaluation_environment_ref = Some("verifier-env-v1".into());
        eval.evaluation_receipt_ref = Some("evidence-receipt".into());
        eval.evaluation_passed = true;
        eval.independent_oracle = true;
        eval.evaluator_id = Some("e".into());
        assert_eq!(m.advance(Stage::Qualified, &eval), Err(Violation::MissingEvaluationReportDigest));
    }

    #[test] fn source_only_disposition_rejects_target_reference() {
        let mut m = manifest();
        let mapping = &mut m.mappings[0];
        mapping.disposition = Disposition::Quarantine;
        mapping.target_ref = Some("ghost-target".into());
        mapping.target_identity = None;
        mapping.target_quantity_atomic = None;
        assert_eq!(m.validate(), Err(Violation::UnexpectedTargetMapping("claim-1".into())));
    }

    #[test] fn evaluation_must_bind_exact_simulation_output() {
        let mut m = TransitionMachine::new(manifest()); let mut e = ev(); e.frozen_plan_digest = Some(A.into()); m.advance(Stage::Frozen, &e).unwrap();
        let mut e = ev(); e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into()); e.simulation_receipt_ref = Some("simulation-receipt-1".into()); e.simulation_scenario_corpus_digest = Some(B.into()); e.simulation_environment_ref = Some("simulation-env-v1".into()); m.advance(Stage::Simulated, &e).unwrap();
        let mut e = ev(); e.evaluation_plan_digest = Some(A.into()); e.evaluation_subject_digest = Some(B.into()); e.evaluation_scenario_corpus_digest = Some(B.into()); e.evaluation_environment_ref = Some("verifier-env-v1".into()); e.evaluation_report_digest = Some(A.into());
        e.evaluation_passed = true; e.independent_oracle = true; e.evaluator_id = Some("e".into());
        assert_eq!(m.advance(Stage::Qualified, &e), Err(Violation::EvaluationOutputMismatch));
    }

    #[test] fn qualification_requires_verifier_receipt() {
        let mut m = TransitionMachine::new(manifest());
        let mut e = ev(); e.frozen_plan_digest = Some(A.into()); m.advance(Stage::Frozen, &e).unwrap();
        let mut e = ev(); e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into()); e.simulation_receipt_ref = Some("simulation-receipt-1".into()); e.simulation_scenario_corpus_digest = Some(B.into()); e.simulation_environment_ref = Some("simulation-env-v1".into()); m.advance(Stage::Simulated, &e).unwrap();
        let mut e = ev(); e.evaluation_plan_digest = Some(A.into()); e.evaluation_subject_digest = Some(C.into()); e.evaluation_scenario_corpus_digest = Some(B.into()); e.evaluation_environment_ref = Some("verifier-env-v1".into()); e.evaluation_report_digest = Some(A.into());
        e.evaluation_scenario_corpus_digest = Some(B.into()); e.evaluation_environment_ref = Some("verifier-env-v1".into()); e.evaluation_report_digest = Some(A.into());
        e.evaluation_passed = true; e.independent_oracle = true; e.evaluator_id = Some("e".into());
        assert_eq!(m.advance(Stage::Qualified, &e), Err(Violation::MissingEvaluationReceipt));
    }

    #[test] fn authorization_requires_approval_receipt() {
        let mut m = TransitionMachine::new(manifest());
        let mut e = ev(); e.frozen_plan_digest = Some(A.into()); m.advance(Stage::Frozen, &e).unwrap();
        let mut e = ev(); e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into()); e.simulation_receipt_ref = Some("simulation-receipt-1".into()); e.simulation_scenario_corpus_digest = Some(B.into()); e.simulation_environment_ref = Some("simulation-env-v1".into()); m.advance(Stage::Simulated, &e).unwrap();
        let mut e = ev(); e.evaluation_plan_digest = Some(A.into()); e.evaluation_subject_digest = Some(C.into()); e.evaluation_scenario_corpus_digest = Some(B.into()); e.evaluation_environment_ref = Some("verifier-env-v1".into()); e.evaluation_report_digest = Some(A.into());
        e.evaluation_passed = true; e.evaluation_receipt_ref = Some("eval-receipt".into()); e.independent_oracle = true; e.evaluator_id = Some("e".into());
        m.advance(Stage::Qualified, &e).unwrap();
        let mut e = ev(); e.approval_plan_digest = Some(A.into()); e.approver_id = Some("a".into()); e.approval_current = true;
        assert_eq!(m.advance(Stage::Authorized, &e), Err(Violation::MissingApprovalReceipt));
    }

    #[test] fn full_happy_path_requires_complete_reconciliation() {
        let mut m = TransitionMachine::new(manifest());
        advance_to_authorized(&mut m);
        let mut ready = ev(); ready.rights_floors_satisfied = true; ready.resource_reservations_reconciled = true;
        ready.external_obligations_accounted = true;
        m.advance(Stage::CutoverReady, &ready).unwrap();
        let mut commit = ev(); commit.cutover_effects_started = true; commit.effect_receipts_durable = true;
        m.advance(Stage::CutoverCommitted, &commit).unwrap();
        let mut recon = ev(); recon.effect_frontier_resolved = true; recon.reconciliation_plan_digest = Some(A.into());
        recon.reconciliation_digest = Some(B.into()); recon.reconciliation_passed = true;
        recon.unresolved_mandatory_items = 0;
        m.advance(Stage::Reconciled, &recon).unwrap();
        m.advance(Stage::Completed, &recon).unwrap();
        assert_eq!(m.stage(), Stage::Completed);
        assert_eq!(m.history().len(), 8);
    }

    #[test] fn cutover_requires_rights_resource_and_external_obligation_gates() {
        let mut m = TransitionMachine::new(manifest()); advance_to_authorized(&mut m);
        let mut e = ev(); e.resource_reservations_reconciled = true; e.external_obligations_accounted = true;
        assert_eq!(m.advance(Stage::CutoverReady, &e), Err(Violation::RightsFloorNotSatisfied));
    }

    #[test] fn abort_is_forbidden_after_effect_may_have_started() {
        let mut m = TransitionMachine::new(manifest()); advance_to_authorized(&mut m);
        let mut ready = ev(); ready.rights_floors_satisfied = true; ready.resource_reservations_reconciled = true; ready.external_obligations_accounted = true;
        m.advance(Stage::CutoverReady, &ready).unwrap();
        let mut e = ev(); e.cutover_effects_started = true;
        assert_eq!(m.advance(Stage::Aborted, &e), Err(Violation::AbortAfterPossibleEffect));
    }

    #[test] fn rejected_cannot_hide_an_already_committed_effect() {
        let mut m = TransitionMachine::new(manifest());
        advance_to_authorized(&mut m);
        let mut ready = ev(); ready.rights_floors_satisfied = true; ready.resource_reservations_reconciled = true;
        ready.external_obligations_accounted = true;
        m.advance(Stage::CutoverReady, &ready).unwrap();
        let mut commit = ev(); commit.cutover_effects_started = true; commit.effect_receipts_durable = true;
        m.advance(Stage::CutoverCommitted, &commit).unwrap();
        assert_eq!(
            m.advance(Stage::Rejected, &ev()),
            Err(Violation::InvalidTransition { from: Stage::CutoverCommitted, to: Stage::Rejected })
        );
    }

    #[test] fn indeterminate_requires_unresolved_effect_frontier_and_is_exceptionally_reachable() {
        let mut m = TransitionMachine::new(manifest());
        assert_eq!(m.advance(Stage::Indeterminate, &ev()), Err(Violation::IndeterminateWithoutPossibleEffect));
        let mut possible = ev(); possible.cutover_effects_started = true;
        assert_eq!(m.advance(Stage::Indeterminate, &possible), Ok(()));
        assert_eq!(m.stage(), Stage::Indeterminate);
        assert!(m.history()[0].effects_may_have_started);
        assert!(!m.history()[0].effect_frontier_resolved);

        let mut ordinary = TransitionMachine::new(manifest());
        advance_to_authorized(&mut ordinary);
        let mut ready = ev();
        ready.rights_floors_satisfied = true;
        ready.resource_reservations_reconciled = true;
        ready.external_obligations_accounted = true;
        ordinary.advance(Stage::CutoverReady, &ready).unwrap();
        assert_eq!(ordinary.advance(Stage::Indeterminate, &ev()), Err(Violation::IndeterminateWithoutPossibleEffect));
        let mut boundary = ev(); boundary.cutover_effects_started = true;
        assert_eq!(ordinary.advance(Stage::Indeterminate, &boundary), Ok(()));
        assert_eq!(ordinary.stage(), Stage::Indeterminate);
    }

    #[test] fn possible_effect_evidence_blocks_normal_progress_outside_recovery_path() {
        let mut machine = TransitionMachine::new(manifest());
        let mut evidence = ev();
        evidence.cutover_effects_started = true;
        assert_eq!(
            machine.advance(Stage::Frozen, &evidence),
            Err(Violation::PossibleEffectMustBeIndeterminate)
        );
        assert_eq!(machine.stage(), Stage::Draft);
        assert_eq!(machine.advance(Stage::Indeterminate, &ev()), Ok(()));
        assert_eq!(machine.stage(), Stage::Indeterminate);
    }

    #[test] fn quarantine_cannot_hide_committed_effect_when_later_evidence_understates_it() {
        let mut machine = TransitionMachine::new(manifest());
        advance_to_authorized(&mut machine);
        let mut ready = ev();
        ready.rights_floors_satisfied = true;
        ready.resource_reservations_reconciled = true;
        ready.external_obligations_accounted = true;
        machine.advance(Stage::CutoverReady, &ready).unwrap();

        let mut commit = ev();
        commit.cutover_effects_started = true;
        commit.effect_receipts_durable = true;
        machine.advance(Stage::CutoverCommitted, &commit).unwrap();

        let mut later = ev();
        later.quarantine_reason = Some("attempt to mask a possibly executed effect".into());
        // Caller-provided per-call fields understate the machine's prior history.
        later.cutover_effects_started = false;
        later.effect_frontier_resolved = true;
        assert_eq!(
            machine.advance(Stage::Quarantined, &later),
            Err(Violation::PossibleEffectMustBeIndeterminate)
        );
        assert_eq!(machine.stage(), Stage::CutoverCommitted);
    }

    #[test] fn failed_commit_attempt_remembers_possible_effect_and_forbids_abort() {
        let mut machine = TransitionMachine::new(manifest());
        advance_to_authorized(&mut machine);
        let mut ready = ev();
        ready.rights_floors_satisfied = true;
        ready.resource_reservations_reconciled = true;
        ready.external_obligations_accounted = true;
        machine.advance(Stage::CutoverReady, &ready).unwrap();

        let mut uncertain = ev();
        uncertain.cutover_effects_started = true;
        uncertain.effect_receipts_durable = false;
        assert_eq!(
            machine.advance(Stage::CutoverCommitted, &uncertain),
            Err(Violation::EffectReceiptsNotDurable)
        );
        assert_eq!(machine.stage(), Stage::CutoverReady);
        assert_eq!(
            machine.advance(Stage::Aborted, &ev()),
            Err(Violation::AbortAfterPossibleEffect)
        );
        assert_eq!(machine.advance(Stage::Indeterminate, &ev()), Ok(()));
    }

    #[test] fn unresolved_mandatory_residual_blocks_completion() {
        let mut m = TransitionMachine::new(manifest()); advance_to_authorized(&mut m);
        let mut ready = ev(); ready.rights_floors_satisfied = true; ready.resource_reservations_reconciled = true; ready.external_obligations_accounted = true;
        m.advance(Stage::CutoverReady, &ready).unwrap();
        let mut commit = ev(); commit.cutover_effects_started = true; commit.effect_receipts_durable = true;
        m.advance(Stage::CutoverCommitted, &commit).unwrap();
        let mut recon = ev(); recon.effect_frontier_resolved = true; recon.reconciliation_plan_digest = Some(A.into());
        recon.reconciliation_digest = Some(B.into()); recon.reconciliation_passed = true; recon.unresolved_mandatory_items = 1;
        m.advance(Stage::Reconciled, &recon).unwrap();
        assert_eq!(m.advance(Stage::Completed, &recon), Err(Violation::MandatoryUnresolvedItems(1)));
        assert_eq!(m.stage(), Stage::Reconciled);
    }

    #[test] fn completed_cannot_swap_the_reconciliation_report() {
        let mut machine = TransitionMachine::new(manifest());
        advance_to_authorized(&mut machine);
        let mut ready = ev();
        ready.rights_floors_satisfied = true;
        ready.resource_reservations_reconciled = true;
        ready.external_obligations_accounted = true;
        machine.advance(Stage::CutoverReady, &ready).unwrap();

        let mut commit = ev();
        commit.cutover_effects_started = true;
        commit.effect_receipts_durable = true;
        machine.advance(Stage::CutoverCommitted, &commit).unwrap();

        let mut recon = ev();
        recon.effect_frontier_resolved = true;
        recon.reconciliation_plan_digest = Some(A.into());
        recon.reconciliation_digest = Some(B.into());
        recon.reconciliation_passed = true;
        recon.unresolved_mandatory_items = 0;
        machine.advance(Stage::Reconciled, &recon).unwrap();

        recon.reconciliation_digest = Some(C.into());
        assert_eq!(
            machine.advance(Stage::Completed, &recon),
            Err(Violation::ReconciliationDigestChanged)
        );
        assert_eq!(machine.stage(), Stage::Reconciled);
    }

    #[test] fn effect_recording_is_idempotent_and_detects_payload_conflict() {
        let mut ledger = EffectLedger::default();
        assert_eq!(ledger.record("effect-1", A), Ok(RecordEffectResult::New));
        assert_eq!(ledger.record("effect-1", A), Ok(RecordEffectResult::ReplaySameRequest));
        assert_eq!(ledger.record("effect-1", B), Ok(RecordEffectResult::Conflict));
    }

    #[test] fn length_prefixed_effect_identity_avoids_separator_collision() {
        assert_ne!(effect_id("a:b", "c", "d"), effect_id("a", "b:c", "d"));
    }

    #[test] fn quarantine_cannot_hide_unresolved_post_effect_uncertainty() {
        let mut machine = TransitionMachine::new(manifest());
        let mut evidence = ev();
        evidence.quarantine_reason = Some("external effect may have occurred".into());
        evidence.cutover_effects_started = true;
        evidence.effect_frontier_resolved = false;
        assert_eq!(
            machine.advance(Stage::Quarantined, &evidence),
            Err(Violation::PossibleEffectMustBeIndeterminate)
        );
    }

    #[test] fn quarantined_transition_is_not_reopened() {
        let mut m = TransitionMachine::new(manifest()); let mut e = ev(); e.quarantine_reason = Some("untrusted identity".into());
        m.advance(Stage::Quarantined, &e).unwrap();
        assert_eq!(m.advance(Stage::Frozen, &ev()), Err(Violation::TerminalState));
    }
}
