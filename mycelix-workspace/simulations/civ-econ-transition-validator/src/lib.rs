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
    DuplicatePrincipal(String),
    MissingSeparationException,
    InvalidTransition { from: Stage, to: Stage },
    FrozenPlanMismatch,
    StaleSourceSnapshot,
    SimulationInputMismatch,
    MissingSimulationOutput,
    EvaluationInputMismatch,
    EvaluationOutputMismatch,
    EvaluationNotIndependent,
    QualificationFailed,
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
    ReconciliationFailed,
    MandatoryUnresolvedItems(usize),
    AbortAfterPossibleEffect,
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
}

impl ProfileRef {
    pub fn validate(&self, field: &'static str) -> Result<(), Violation> {
        if self.id.trim().is_empty() || self.version.trim().is_empty() {
            return Err(Violation::EmptyField(field));
        }
        validate_digest(&self.content_digest, field)
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
    pub target_ref: Option<String>,
    pub conversion_profile_ref: Option<String>,
    pub authority_decision_ref: Option<String>,
    pub counterparty_acceptance_ref: Option<String>,
    pub discharge_receipt_ref: Option<String>,
}

impl ItemMapping {
    fn validate(&self) -> Result<(), Violation> {
        if self.source_item_id.trim().is_empty() {
            return Err(Violation::EmptyField("source_item_id"));
        }

        match self.disposition {
            Disposition::Convert => {
                self.require(self.target_ref.as_deref(), "target_ref")?;
                self.require(self.conversion_profile_ref.as_deref(), "conversion_profile_ref")?;
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
            }
            Disposition::Novate => {
                self.require(self.target_ref.as_deref(), "target_ref")?;
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
                self.require(self.counterparty_acceptance_ref.as_deref(), "counterparty_acceptance_ref")?;
            }
            Disposition::Discharge => {
                self.require(self.discharge_receipt_ref.as_deref(), "discharge_receipt_ref")?;
            }
            Disposition::ReReserve => {
                if !matches!(self.kind, ItemKind::Resource | ItemKind::Reservation) {
                    return Err(Violation::InvalidDispositionForKind(self.source_item_id.clone()));
                }
                self.require(self.target_ref.as_deref(), "target_ref")?;
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
            }
            Disposition::Release => {
                if !matches!(self.kind, ItemKind::Resource | ItemKind::Reservation) {
                    return Err(Violation::InvalidDispositionForKind(self.source_item_id.clone()));
                }
                self.require(self.authority_decision_ref.as_deref(), "authority_decision_ref")?;
            }
            Disposition::Preserve
            | Disposition::Freeze
            | Disposition::Dispute
            | Disposition::Quarantine
            | Disposition::RetainSourceOnly => {}
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
    pub evaluation_plan_digest: Option<String>,
    pub evaluation_output_digest: Option<String>,
    pub evaluation_passed: bool,
    pub independent_oracle: bool,
    pub evaluator_id: Option<String>,
    pub approval_plan_digest: Option<String>,
    pub approver_id: Option<String>,
    pub approval_current: bool,
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
}

pub struct TransitionMachine {
    manifest: TransitionManifest,
    stage: Stage,
    simulation_output_digest: Option<String>,
    reconciliation_passed: bool,
    unresolved_mandatory_items: usize,
    history: Vec<StageEvent>,
}

impl TransitionMachine {
    pub fn new(manifest: TransitionManifest) -> Self {
        Self {
            manifest,
            stage: Stage::Draft,
            simulation_output_digest: None,
            reconciliation_passed: false,
            unresolved_mandatory_items: 0,
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

        if next == Stage::Quarantined {
            if ev.quarantine_reason.as_deref().map(str::trim).filter(|s| !s.is_empty()).is_none() {
                return Err(Violation::QuarantineReasonMissing);
            }
            return self.record_stage(next);
        }
        if next == Stage::Rejected {
            if ev.cutover_effects_started {
                return Err(Violation::AbortAfterPossibleEffect);
            }
            return self.record_stage(next);
        }
        if next == Stage::Aborted {
            if ev.cutover_effects_started || self.stage == Stage::CutoverCommitted {
                return Err(Violation::AbortAfterPossibleEffect);
            }
            return self.record_stage(next);
        }
        if next == Stage::Indeterminate {
            if !ev.cutover_effects_started || ev.effect_frontier_resolved {
                return Err(Violation::IndeterminateWithoutPossibleEffect);
            }
            return self.record_stage(next);
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
                self.simulation_output_digest = Some(output.to_owned());
            }
            (Stage::Simulated, Stage::Qualified) => {
                require_current_snapshot(ev)?;
                if ev.evaluation_plan_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::EvaluationInputMismatch);
                }
                if ev.evaluation_output_digest != self.simulation_output_digest {
                    return Err(Violation::EvaluationOutputMismatch);
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
                self.reconciliation_passed = ev.reconciliation_passed;
                self.unresolved_mandatory_items = ev.unresolved_mandatory_items;
            }
            (Stage::Reconciled, Stage::Completed) => {
                if !ev.effect_frontier_resolved { return Err(Violation::EffectFrontierUnresolved); }
                if ev.reconciliation_plan_digest.as_deref() != Some(self.manifest.plan_digest.as_str()) {
                    return Err(Violation::ReconciliationEvidenceMissing);
                }
                validate_digest(ev.reconciliation_digest.as_deref().unwrap_or(""), "reconciliation_digest")
                    .map_err(|_| Violation::ReconciliationEvidenceMissing)?;
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
        ProfileRef { id: id.into(), version: "1.0.0".into(), content_digest: digest.into() }
    }

    fn mapping(id: &str, kind: ItemKind, disposition: Disposition) -> ItemMapping {
        ItemMapping {
            source_item_id: id.into(), kind, disposition, target_ref: None,
            conversion_profile_ref: None, authority_decision_ref: None,
            counterparty_acceptance_ref: None, discharge_receipt_ref: None,
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
        let mut e = ev(); e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into());
        machine.advance(Stage::Simulated, &e).unwrap();
        let mut e = ev(); e.evaluation_plan_digest = Some(A.into()); e.evaluation_output_digest = Some(C.into());
        e.evaluation_passed = true; e.independent_oracle = true; e.evaluator_id = Some("e".into());
        machine.advance(Stage::Qualified, &e).unwrap();
        let mut e = ev(); e.approval_plan_digest = Some(A.into()); e.approver_id = Some("a".into());
        e.approval_current = true; machine.advance(Stage::Authorized, &e).unwrap();
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

    #[test] fn conversion_requires_target_profile_and_authority() {
        let mut m = manifest(); m.mappings[0].disposition = Disposition::Convert;
        assert_eq!(m.validate(), Err(Violation::MissingMappingField { item: "claim-1".into(), field: "target_ref" }));
    }

    #[test] fn novation_requires_counterparty_acceptance() {
        let mut m = manifest(); let x = &mut m.mappings[0];
        x.disposition = Disposition::Novate; x.target_ref = Some("target".into()); x.authority_decision_ref = Some("approval".into());
        assert_eq!(m.validate(), Err(Violation::MissingMappingField { item: "claim-1".into(), field: "counterparty_acceptance_ref" }));
    }

    #[test] fn discharge_requires_receipt() {
        let mut m = manifest(); m.mappings[0].disposition = Disposition::Discharge;
        assert_eq!(m.validate(), Err(Violation::MissingMappingField { item: "claim-1".into(), field: "discharge_receipt_ref" }));
    }

    #[test] fn overlapping_roles_need_exception_reference() {
        let mut m = manifest(); m.authorities.evaluator = m.authorities.proposer.clone();
        assert!(matches!(m.validate(), Err(Violation::DuplicatePrincipal(_))));
        m.authorities.separation_exception_ref = Some("risk-exception".into());
        assert_eq!(m.validate(), Ok(()));
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
        let mut e = ev(); e.source_snapshot_current = false; e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into());
        assert_eq!(m.advance(Stage::Simulated, &e), Err(Violation::StaleSourceSnapshot));
    }

    #[test] fn evaluation_must_bind_exact_simulation_output() {
        let mut m = TransitionMachine::new(manifest()); let mut e = ev(); e.frozen_plan_digest = Some(A.into()); m.advance(Stage::Frozen, &e).unwrap();
        let mut e = ev(); e.simulation_input_digest = Some(A.into()); e.simulation_output_digest = Some(C.into()); m.advance(Stage::Simulated, &e).unwrap();
        let mut e = ev(); e.evaluation_plan_digest = Some(A.into()); e.evaluation_output_digest = Some(B.into());
        e.evaluation_passed = true; e.independent_oracle = true; e.evaluator_id = Some("e".into());
        assert_eq!(m.advance(Stage::Qualified, &e), Err(Violation::EvaluationOutputMismatch));
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

    #[test] fn indeterminate_requires_unresolved_effect_frontier() {
        let mut m = TransitionMachine::new(manifest());
        assert_eq!(m.advance(Stage::Indeterminate, &ev()), Err(Violation::IndeterminateWithoutPossibleEffect));
        let mut e = ev(); e.cutover_effects_started = true;
        assert_eq!(m.advance(Stage::Indeterminate, &e), Ok(()));
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

    #[test] fn effect_recording_is_idempotent_and_detects_payload_conflict() {
        let mut ledger = EffectLedger::default();
        assert_eq!(ledger.record("effect-1", A), Ok(RecordEffectResult::New));
        assert_eq!(ledger.record("effect-1", A), Ok(RecordEffectResult::ReplaySameRequest));
        assert_eq!(ledger.record("effect-1", B), Ok(RecordEffectResult::Conflict));
    }

    #[test] fn length_prefixed_effect_identity_avoids_separator_collision() {
        assert_ne!(effect_id("a:b", "c", "d"), effect_id("a", "b:c", "d"));
    }

    #[test] fn quarantined_transition_is_not_reopened() {
        let mut m = TransitionMachine::new(manifest()); let mut e = ev(); e.quarantine_reason = Some("untrusted identity".into());
        m.advance(Stage::Quarantined, &e).unwrap();
        assert_eq!(m.advance(Stage::Frozen, &ev()), Err(Violation::TerminalState));
    }
}
