use serde::{Deserialize, Serialize};

use crate::identity_lineage::{ApplicabilityInterval, IdentityKind, IdentityRef, LineageEdge, LineageRelation, TemporalConfigurationApplicability};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceEventInterval {
    pub start: u64,
    pub end: Option<u64>,
}

impl EvidenceEventInterval {
    pub fn validate(&self) -> Result<(), String> {
        ApplicabilityInterval { start: self.start, end: self.end }
            .validate()
            .map_err(|e| format!("invalid evidence event interval: {e}"))
    }
}

/// Current documented epistemic disposition of an evidence record.
///
/// This is deliberately separate from event/effectivity time. A disposition
/// never rewrites or erases the historical event or applicability interval.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceDisposition {
    Active,
    Superseded { by: IdentityRef },
    Disputed { by: IdentityRef },
    Retracted { by: IdentityRef },
    Unresolved,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDispositionTransition {
    /// Stable address of this immutable transition record.
    pub transition_id: IdentityRef,
    /// Evidence record whose epistemic state is changing.
    pub evidence: IdentityRef,
    /// Explicit predecessor transition. None is permitted only for the initial
    /// disposition assertion; subsequent transitions must point to exactly one
    /// prior transition record.
    pub predecessor: Option<IdentityRef>,
    pub from: EvidenceDisposition,
    pub to: EvidenceDisposition,
    /// Addressable witness for why the transition is being asserted.
    pub basis: IdentityRef,
}

impl EvidenceDispositionTransition {
    pub fn validate(&self) -> Result<(), String> {
        self.transition_id.validate()?;
        if !matches!(self.transition_id.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
            return Err("disposition transition identity must be an EvidenceRecord or ReconciliationWitness".into());
        }
        if self.transition_id == self.evidence {
            return Err("disposition transition identity cannot equal its evidence record".into());
        }
        self.evidence.validate()?;
        if !matches!(
            self.evidence.kind,
            IdentityKind::InspectionRecord
                | IdentityKind::TestRecord
                | IdentityKind::OperationalObservation
                | IdentityKind::MaintenanceEvent
        ) {
            return Err("disposition transition requires a typed evidence event".into());
        }

        if let Some(predecessor) = &self.predecessor {
            predecessor.validate()?;
            if predecessor == &self.transition_id {
                return Err("disposition transition cannot be its own predecessor".into());
            }
            if predecessor == &self.evidence {
                return Err("disposition predecessor cannot be the evidence record itself".into());
            }
            if !matches!(
                predecessor.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
                return Err("disposition predecessor must be an EvidenceRecord or ReconciliationWitness".into());
            }
        }

        self.basis.validate()?;
        if self.basis == self.evidence {
            return Err("disposition basis cannot be the evidence record itself".into());
        }
        if !matches!(
            self.basis.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err("disposition basis must be an EvidenceRecord or ReconciliationWitness".into());
        }

        self.from.validate(&self.evidence)?;
        self.to.validate(&self.evidence)?;

        if self.from == self.to {
            return Err("disposition transition must change epistemic state".into());
        }

        if self.predecessor.is_none() && self.from != EvidenceDisposition::Active {
            return Err("genesis disposition must start from Active".into());
        }

        match (&self.from, &self.to) {
            (EvidenceDisposition::Active, EvidenceDisposition::Disputed { .. })
            | (EvidenceDisposition::Active, EvidenceDisposition::Superseded { .. })
            | (EvidenceDisposition::Active, EvidenceDisposition::Retracted { .. })
            | (EvidenceDisposition::Active, EvidenceDisposition::Unresolved)
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Active)
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Superseded { .. })
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Retracted { .. })
            | (EvidenceDisposition::Disputed { .. }, EvidenceDisposition::Unresolved)
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Active)
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Disputed { .. })
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Superseded { .. })
            | (EvidenceDisposition::Unresolved, EvidenceDisposition::Retracted { .. }) => Ok(()),
            (EvidenceDisposition::Superseded { .. }, _)
            | (EvidenceDisposition::Retracted { .. }, _) => {
                Err("superseded and retracted dispositions are terminal".into())
            }
            _ => Err("unsupported evidence disposition transition".into()),
        }
    }
}

/// Result of validating the available transition graph. Missing records are
/// surfaced separately from invalid records; branch points are retained.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DispositionChainAssessment {
    Complete { branch_points: Vec<IdentityRef> },
    Unresolved { missing: Vec<IdentityRef>, branch_points: Vec<IdentityRef> },
}

impl EvidenceDispositionTransition {
    /// Validate supplied transitions by exact identity, without mutable
    /// "latest" indexes or wall-clock ordering.
    pub fn validate_graph(
        transitions: &[EvidenceDispositionTransition],
    ) -> Result<DispositionChainAssessment, String> {
        use std::collections::BTreeMap;
        let mut by_id = BTreeMap::new();
        let mut children: BTreeMap<IdentityRef, usize> = BTreeMap::new();
        let mut genesis: BTreeMap<IdentityRef, usize> = BTreeMap::new();
        for transition in transitions {
            transition.validate()?;
            if by_id.insert(transition.transition_id.clone(), transition).is_some() {
                return Err("duplicate disposition transition identity".into());
            }
            if let Some(parent) = &transition.predecessor {
                *children.entry(parent.clone()).or_default() += 1;
            } else {
                *genesis.entry(transition.evidence.clone()).or_default() += 1;
            }
        }
        if genesis.values().any(|count| *count > 1) {
            return Err("evidence record has multiple genesis disposition assertions".into());
        }
        let mut missing = Vec::new();
        for transition in transitions {
            if let Some(parent_id) = &transition.predecessor {
                match by_id.get(parent_id) {
                    None => missing.push(parent_id.clone()),
                    Some(parent) => {
                        if parent.evidence != transition.evidence {
                            return Err("disposition predecessor belongs to different evidence".into());
                        }
                        if parent.to != transition.from {
                            return Err("transition source state does not match predecessor target state".into());
                        }
                    }
                }
            }
        }
        missing.sort_by(|a, b| (&a.namespace, &a.id).cmp(&(&b.namespace, &b.id)));
        missing.dedup();

        // A finite predecessor graph must bottom out at a genesis assertion.
        // Otherwise the graph contains a closed cycle with no historical root.
        // Such a cycle is not an unresolved DHT dependency: all referenced
        // transitions are present, but the claimed append-only history is
        // structurally impossible.
        for transition in transitions {
            let mut seen = std::collections::BTreeSet::new();
            let mut cursor = transition;
            while let Some(parent_id) = &cursor.predecessor {
                if !seen.insert(cursor.transition_id.clone()) {
                    return Err("disposition transition graph contains a predecessor cycle".into());
                }
                match by_id.get(parent_id) {
                    Some(parent) => cursor = parent,
                    None => break,
                }
            }
        }

        let branch_points = children.into_iter()
            .filter_map(|(parent, count)| if count > 1 { Some(parent) } else { None })
            .collect();
        if missing.is_empty() {
            Ok(DispositionChainAssessment::Complete { branch_points })
        } else {
            Ok(DispositionChainAssessment::Unresolved { missing, branch_points })
        }
    }
}

/// Explicitly records reconciliation of competing disposition branches.
///
/// This record never selects a branch implicitly. It identifies the exact
/// branch heads considered, the branch point they descend from, the evidence
/// record they concern, and the addressable authority/basis for the
/// reconciliation. The reconciliation itself is a provenance fact; it does
/// not establish truth, safety, certification, or causal correctness.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDispositionReconciliation {
    pub reconciliation_id: IdentityRef,
    pub evidence: IdentityRef,
    pub branch_point: IdentityRef,
    pub branch_heads: Vec<IdentityRef>,
    pub authority: IdentityRef,
    pub authority_scope: IdentityRef,
    pub authority_delegation: IdentityRef,
    pub basis: Vec<IdentityRef>,
}

/// Explicitly binds a reconciliation authority to the exact reconciliation
/// witness it is authorized to speak about in this provenance model.
///
/// This is an addressable scope assertion, not proof of real-world institutional
/// authority. Its purpose is to prevent an authority witness from being silently
/// reused for an unrelated reconciliation subject.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDispositionAuthorityScope {
    pub scope_id: IdentityRef,
    pub authority: IdentityRef,
    pub subject: IdentityRef,
    pub delegation: IdentityRef,
    pub basis: Vec<IdentityRef>,
}

/// Addressable assertion describing how an authority scope was delegated.
///
/// This records provenance of the authority claim; it does not independently
/// establish that the grantor has real-world legal or institutional power.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDispositionAuthorityDelegation {
    pub delegation_id: IdentityRef,
    pub grantor: IdentityRef,
    pub grantee: IdentityRef,
    pub subject: IdentityRef,
    pub predecessor: Option<IdentityRef>,
    pub basis: Vec<IdentityRef>,
}

impl EvidenceDispositionAuthorityDelegation {
    pub fn validate(&self) -> Result<(), String> {
        self.delegation_id.validate()?;
        self.grantor.validate()?;
        self.grantee.validate()?;
        self.subject.validate()?;
        if self.delegation_id.kind != IdentityKind::ReconciliationWitness {
            return Err("authority delegation identity must be a ReconciliationWitness".into());
        }
        for (name, identity) in [("grantor", &self.grantor), ("grantee", &self.grantee)] {
            if !matches!(identity.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err(format!("authority delegation {name} must be an addressable witness"));
            }
        }
        if self.subject.kind != IdentityKind::ReconciliationWitness {
            return Err("authority delegation subject must be a ReconciliationWitness".into());
        }
        if self.delegation_id == self.grantor
            || self.delegation_id == self.grantee
            || self.delegation_id == self.subject
            || self.grantor == self.grantee
            || self.grantor == self.subject
            || self.grantee == self.subject
        {
            return Err("authority delegation identities must be distinct".into());
        }
        if let Some(predecessor) = &self.predecessor {
            predecessor.validate()?;
            if predecessor == &self.delegation_id {
                return Err("authority delegation cannot be its own predecessor".into());
            }
            if predecessor == &self.subject {
                return Err("authority delegation predecessor cannot be the subject".into());
            }
            if !matches!(predecessor.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("authority delegation predecessor must be an addressable witness".into());
            }
        }
        let mut basis = std::collections::BTreeSet::new();
        for witness in &self.basis {
            witness.validate()?;
            if !matches!(witness.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("authority delegation basis must be an addressable witness".into());
            }
            if !basis.insert(witness.clone()) {
                return Err("authority delegation basis must be unique".into());
            }
        }
        Ok(())
    }

    pub fn validate_against_scope(
        &self,
        scope: &EvidenceDispositionAuthorityScope,
    ) -> Result<(), String> {
        self.validate()?;
        scope.validate()?;
        if self.grantee != scope.authority {
            return Err("authority delegation grantee must match scope authority".into());
        }
        if self.subject != scope.subject {
            return Err("authority delegation subject must match scope subject".into());
        }
        for basis in &scope.basis {
            if !self.basis.contains(basis) {
                return Err("authority delegation basis must include every scope basis witness".into());
            }
        }
        Ok(())
    }
}

impl EvidenceDispositionAuthorityScope {
    pub fn validate(&self) -> Result<(), String> {
        self.scope_id.validate()?;
        self.authority.validate()?;
        self.subject.validate()?;
        self.delegation.validate()?;
        if self.scope_id.kind != IdentityKind::ReconciliationWitness {
            return Err("authority scope identity must be a ReconciliationWitness".into());
        }
        if !matches!(self.authority.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
            return Err("authority scope authority must be an addressable witness".into());
        }
        if self.subject.kind != IdentityKind::ReconciliationWitness {
            return Err("authority scope subject must be a ReconciliationWitness".into());
        }
        if self.delegation.kind != IdentityKind::ReconciliationWitness {
            return Err("authority scope delegation must be a ReconciliationWitness".into());
        }
        if self.scope_id == self.authority
            || self.scope_id == self.subject
            || self.authority == self.subject
            || self.delegation == self.scope_id
            || self.delegation == self.authority
            || self.delegation == self.subject
        {
            return Err("authority scope identities must be distinct".into());
        }
        let mut basis = std::collections::BTreeSet::new();
        for witness in &self.basis {
            witness.validate()?;
            if !matches!(witness.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("authority scope basis must be an addressable witness".into());
            }
            if !basis.insert(witness.clone()) {
                return Err("authority scope basis must be unique".into());
            }
        }
        Ok(())
    }

    pub fn validate_against_reconciliation(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        delegation: &EvidenceDispositionAuthorityDelegation,
    ) -> Result<(), String> {
        self.validate()?;
        reconciliation.validate()?;
        delegation.validate_against_scope(self)?;
        if self.subject != reconciliation.reconciliation_id {
            return Err("authority scope subject must match reconciliation identity".into());
        }
        if self.authority != reconciliation.authority {
            return Err("authority scope authority must match reconciliation authority".into());
        }
        for basis in &reconciliation.basis {
            if !self.basis.contains(basis) {
                return Err("authority scope basis must include every reconciliation basis witness".into());
            }
        }
        Ok(())
    }
}

impl EvidenceDispositionReconciliation {
    pub fn validate(&self) -> Result<(), String> {
        self.reconciliation_id.validate()?;
        self.evidence.validate()?;
        self.branch_point.validate()?;
        self.authority.validate()?;
        self.authority_scope.validate()?;
        self.authority_delegation.validate()?;
        if self.authority_scope.kind != IdentityKind::ReconciliationWitness {
            return Err("reconciliation authority scope must be a ReconciliationWitness".into());
        }
        if self.authority_scope == self.reconciliation_id || self.authority_scope == self.authority || self.authority_delegation == self.reconciliation_id || self.authority_delegation == self.authority || self.authority_delegation == self.authority_scope {
            return Err("reconciliation authority scope cannot equal reconciliation or authority identity".into());
        }
        if !matches!(self.reconciliation_id.kind, IdentityKind::ReconciliationWitness) {
            return Err("reconciliation identity must be a ReconciliationWitness".into());
        }
        if self.reconciliation_id == self.evidence || self.reconciliation_id == self.branch_point {
            return Err("reconciliation identity cannot equal evidence or branch point".into());
        }
        if !matches!(
            self.evidence.kind,
            IdentityKind::InspectionRecord
                | IdentityKind::TestRecord
                | IdentityKind::OperationalObservation
                | IdentityKind::MaintenanceEvent
        ) {
            return Err("reconciliation requires a typed evidence event".into());
        }
        if !matches!(
            self.branch_point.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err("reconciliation branch point must be a transition identity".into());
        }
        if !matches!(
            self.authority.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err("reconciliation authority must be an addressable witness".into());
        }
        if self.branch_heads.len() < 2 {
            return Err("reconciliation requires at least two competing branch heads".into());
        }
        let mut heads = std::collections::BTreeSet::new();
        for head in &self.branch_heads {
            head.validate()?;
            if !matches!(head.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("reconciliation branch head must be a transition identity".into());
            }
            if !heads.insert(head.clone()) {
                return Err("reconciliation branch heads must be unique".into());
            }
            if head == &self.reconciliation_id || head == &self.evidence {
                return Err("reconciliation branch head cannot equal reconciliation or evidence identity".into());
            }
        }
        for basis in &self.basis {
            basis.validate()?;
            if !matches!(basis.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("reconciliation basis must be an addressable witness".into());
            }
        }
        Ok(())
    }

    /// Verify that every named branch head exists in the supplied transition
    /// graph, concerns the same evidence, and has a predecessor path to the
    /// declared branch point. This does not choose a winning branch.
    pub fn validate_against_graph(
        &self,
        authority_scope: &EvidenceDispositionAuthorityScope,
        authority_delegation: &EvidenceDispositionAuthorityDelegation,
        transitions: &[EvidenceDispositionTransition],
    ) -> Result<(), String> {
        self.validate()?;
        if self.authority_scope != authority_scope.scope_id {
            return Err("reconciliation references a different authority scope".into());
        }
        authority_scope.validate_against_reconciliation(self, authority_delegation)?;
        let mut by_id = std::collections::BTreeMap::new();
        for transition in transitions {
            transition.validate()?;
            if by_id.insert(transition.transition_id.clone(), transition).is_some() {
                return Err("duplicate disposition transition identity".into());
            }
        }

        let branch_point = by_id.get(&self.branch_point)
            .ok_or_else(|| "reconciliation branch point is missing".to_string())?;
        if branch_point.evidence != self.evidence {
            return Err("reconciliation branch point belongs to different evidence".into());
        }

        for head_id in &self.branch_heads {
            if head_id == &self.branch_point {
                return Err("reconciliation branch head cannot equal the branch point".into());
            }
            let mut cursor = by_id.get(head_id)
                .ok_or_else(|| format!("reconciliation branch head is missing: {}", head_id.id))?;
            if cursor.evidence != self.evidence {
                return Err("reconciliation branch head belongs to different evidence".into());
            }
            let mut seen = std::collections::BTreeSet::new();
            loop {
                if !seen.insert(cursor.transition_id.clone()) {
                    return Err("reconciliation graph contains a predecessor cycle".into());
                }
                if cursor.transition_id == self.branch_point {
                    break;
                }
                let predecessor = cursor.predecessor.as_ref()
                    .ok_or_else(|| "reconciliation branch head does not descend from branch point".to_string())?;
                cursor = by_id.get(predecessor)
                    .ok_or_else(|| "reconciliation branch ancestry is unresolved".to_string())?;
                if cursor.evidence != self.evidence {
                    return Err("reconciliation branch ancestry belongs to different evidence".into());
                }
            }
        }

        // Two heads only constitute competing branches when neither head is an
        // ancestor of the other. Otherwise the later head is simply a
        // continuation of the earlier branch, not an independent branch to
        // reconcile.
        for (index, left_id) in self.branch_heads.iter().enumerate() {
            for right_id in self.branch_heads.iter().skip(index + 1) {
                let mut cursor = by_id.get(left_id)
                    .ok_or_else(|| "reconciliation branch head is missing".to_string())?;
                let mut seen = std::collections::BTreeSet::new();
                while let Some(predecessor) = cursor.predecessor.as_ref() {
                    if !seen.insert(cursor.transition_id.clone()) {
                        return Err("reconciliation graph contains a predecessor cycle".into());
                    }
                    if predecessor == right_id {
                        return Err("reconciliation branch heads must be incomparable descendants".into());
                    }
                    if predecessor == &self.branch_point {
                        break;
                    }
                    cursor = by_id.get(predecessor)
                        .ok_or_else(|| "reconciliation branch ancestry is unresolved".to_string())?;
                }

                let mut cursor = by_id.get(right_id)
                    .ok_or_else(|| "reconciliation branch head is missing".to_string())?;
                let mut seen = std::collections::BTreeSet::new();
                while let Some(predecessor) = cursor.predecessor.as_ref() {
                    if !seen.insert(cursor.transition_id.clone()) {
                        return Err("reconciliation graph contains a predecessor cycle".into());
                    }
                    if predecessor == left_id {
                        return Err("reconciliation branch heads must be incomparable descendants".into());
                    }
                    if predecessor == &self.branch_point {
                        break;
                    }
                    cursor = by_id.get(predecessor)
                        .ok_or_else(|| "reconciliation branch ancestry is unresolved".to_string())?;
                }
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDispositionCoverageBoundary {
    pub boundary_id: IdentityRef,
    /// Exact reconciliation witness whose bounded coverage this boundary scopes.
    pub reconciliation: IdentityRef,
    pub evidence: IdentityRef,
    pub branch_point: IdentityRef,
    pub branch_heads: Vec<IdentityRef>,
    pub authority: IdentityRef,
    pub authority_scope: IdentityRef,
    pub authority_delegation: IdentityRef,
    pub basis: Vec<IdentityRef>,
}

impl EvidenceDispositionCoverageBoundary {
    pub fn validate(&self) -> Result<(), String> {
        self.boundary_id.validate()?;
        self.reconciliation.validate()?;
        self.evidence.validate()?;
        self.branch_point.validate()?;
        self.authority.validate()?;
        self.authority_scope.validate()?;
        self.authority_delegation.validate()?;
        if self.authority_scope.kind != IdentityKind::ReconciliationWitness {
            return Err("coverage boundary authority scope must be a ReconciliationWitness".into());
        }
        if self.authority_scope == self.boundary_id || self.authority_scope == self.authority || self.authority_delegation == self.boundary_id || self.authority_delegation == self.authority || self.authority_delegation == self.authority_scope {
            return Err("coverage boundary authority scope cannot equal boundary or authority identity".into());
        }
        if self.boundary_id.kind != IdentityKind::ReconciliationWitness {
            return Err("coverage boundary identity must be a ReconciliationWitness".into());
        }
        if self.reconciliation.kind != IdentityKind::ReconciliationWitness {
            return Err("coverage boundary reconciliation must be a ReconciliationWitness".into());
        }
        if self.reconciliation == self.boundary_id || self.reconciliation == self.evidence {
            return Err("coverage boundary reconciliation cannot equal boundary or evidence identity".into());
        }
        if !matches!(
            self.evidence.kind,
            IdentityKind::InspectionRecord
                | IdentityKind::TestRecord
                | IdentityKind::OperationalObservation
                | IdentityKind::MaintenanceEvent
        ) {
            return Err("coverage boundary requires a typed evidence event".into());
        }
        if !matches!(self.branch_point.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
            return Err("coverage boundary branch point must be a transition identity".into());
        }
        if !matches!(self.authority.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
            return Err("coverage boundary authority must be an addressable witness".into());
        }
        if self.branch_heads.is_empty() {
            return Err("coverage boundary requires at least one branch head".into());
        }
        let mut heads = std::collections::BTreeSet::new();
        for head in &self.branch_heads {
            head.validate()?;
            if !matches!(head.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("coverage boundary branch head must be a transition identity".into());
            }
            if !heads.insert(head.clone()) {
                return Err("coverage boundary branch heads must be unique".into());
            }
            if head == &self.boundary_id || head == &self.evidence || head == &self.branch_point {
                return Err("coverage boundary branch head cannot equal boundary, evidence, or branch point".into());
            }
        }
        let mut basis_set = std::collections::BTreeSet::new();
        for basis in &self.basis {
            basis.validate()?;
            if !matches!(basis.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("coverage boundary basis must be an addressable witness".into());
            }
            if !basis_set.insert(basis.clone()) {
                return Err("coverage boundary basis must be unique".into());
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceDispositionReconciliationCoverage {
    pub coverage_id: IdentityRef,
    pub reconciliation: IdentityRef,
    pub branch_point: IdentityRef,
    pub covered_branch_heads: Vec<IdentityRef>,
    pub boundary: IdentityRef,
    pub basis: Vec<IdentityRef>,
}

impl EvidenceDispositionReconciliationCoverage {
    pub fn validate(&self) -> Result<(), String> {
        self.coverage_id.validate()?;
        self.reconciliation.validate()?;
        self.branch_point.validate()?;
        self.boundary.validate()?;
        if self.coverage_id.kind != IdentityKind::ReconciliationWitness {
            return Err("reconciliation coverage identity must be a ReconciliationWitness".into());
        }
        if self.reconciliation.kind != IdentityKind::ReconciliationWitness {
            return Err("reconciliation coverage must reference a ReconciliationWitness".into());
        }
        if !matches!(self.branch_point.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
            return Err("reconciliation coverage branch point must be a transition identity".into());
        }
        if self.boundary.kind != IdentityKind::ReconciliationWitness {
            return Err("reconciliation coverage boundary must be a ReconciliationWitness".into());
        }
        if self.covered_branch_heads.is_empty() {
            return Err("reconciliation coverage requires at least one covered branch head".into());
        }
        let mut heads = std::collections::BTreeSet::new();
        for head in &self.covered_branch_heads {
            head.validate()?;
            if !matches!(head.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("reconciliation coverage branch head must be a transition identity".into());
            }
            if !heads.insert(head.clone()) {
                return Err("reconciliation coverage branch heads must be unique".into());
            }
            if head == &self.coverage_id || head == &self.reconciliation {
                return Err("reconciliation coverage branch head cannot equal a coverage or reconciliation identity".into());
            }
        }
        for basis in &self.basis {
            basis.validate()?;
            if !matches!(basis.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("reconciliation coverage basis must be an addressable witness".into());
            }
        }
        Ok(())
    }

    /// Validates a bounded, explicitly named coverage set. It does not
    /// enumerate the DHT or establish global completeness.
    pub fn validate_against_graph(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        authority_scope: &EvidenceDispositionAuthorityScope,
        authority_delegation: &EvidenceDispositionAuthorityDelegation,
        boundary: &EvidenceDispositionCoverageBoundary,
        transitions: &[EvidenceDispositionTransition],
    ) -> Result<(), String> {
        self.validate()?;
        reconciliation.validate_against_graph(authority_scope, authority_delegation, transitions)?;
        authority_scope.validate_against_reconciliation(reconciliation, authority_delegation)?;
        boundary.validate()?;
        if self.boundary != boundary.boundary_id {
            return Err("reconciliation coverage references a different coverage boundary".into());
        }
        if boundary.evidence != reconciliation.evidence {
            return Err("coverage boundary references different evidence".into());
        }
        if self.branch_point != boundary.branch_point {
            return Err("coverage boundary references a different branch point".into());
        }
        if self.covered_branch_heads != boundary.branch_heads {
            return Err("reconciliation coverage heads must exactly match the coverage boundary".into());
        }
        if self.basis != boundary.basis {
            return Err("reconciliation coverage basis must exactly match the coverage boundary".into());
        }
        if boundary.reconciliation != reconciliation.reconciliation_id {
            return Err("coverage boundary references a different reconciliation".into());
        }
        if boundary.authority != reconciliation.authority {
            return Err("coverage boundary authority must match reconciliation authority".into());
        }
        if boundary.authority_scope != reconciliation.authority_scope {
            return Err("coverage boundary authority scope must match reconciliation authority scope".into());
        }
        if boundary.authority_delegation != reconciliation.authority_delegation {
            return Err("coverage boundary authority delegation must match reconciliation authority delegation".into());
        }
        if boundary.authority_delegation != authority_delegation.delegation_id {
            return Err("coverage boundary authority delegation does not match supplied delegation witness".into());
        }
        if boundary.authority_scope != authority_scope.scope_id {
            return Err("coverage boundary authority scope does not match supplied scope witness".into());
        }
        for basis in &reconciliation.basis {
            if !boundary.basis.contains(basis) {
                return Err("coverage boundary basis must include every reconciliation basis witness".into());
            }
        }
        if self.reconciliation != reconciliation.reconciliation_id {
            return Err("reconciliation coverage references a different reconciliation".into());
        }
        if self.branch_point != reconciliation.branch_point {
            return Err("reconciliation coverage references a different branch point".into());
        }
        for head in &reconciliation.branch_heads {
            if !self.covered_branch_heads.contains(head) {
                return Err("reconciliation branch head is outside the declared coverage set".into());
            }
        }
        let mut by_id = std::collections::BTreeMap::new();
        for transition in transitions {
            if by_id.insert(transition.transition_id.clone(), transition).is_some() {
                return Err("duplicate disposition transition identity".into());
            }
        }
        let branch_point = by_id.get(&self.branch_point)
            .ok_or_else(|| "reconciliation coverage branch point is missing".to_string())?;
        for head_id in &self.covered_branch_heads {
            if head_id == &self.branch_point {
                return Err("reconciliation coverage branch head cannot equal the branch point".into());
            }
            let mut cursor = by_id.get(head_id)
                .ok_or_else(|| format!("reconciliation coverage branch head is missing: {}", head_id.id))?;
            if cursor.evidence != branch_point.evidence {
                return Err("reconciliation coverage branch head belongs to different evidence".into());
            }
            let mut seen = std::collections::BTreeSet::new();
            loop {
                if !seen.insert(cursor.transition_id.clone()) {
                    return Err("reconciliation coverage graph contains a predecessor cycle".into());
                }
                if cursor.transition_id == self.branch_point {
                    break;
                }
                let predecessor = cursor.predecessor.as_ref()
                    .ok_or_else(|| "reconciliation coverage branch head does not descend from branch point".to_string())?;
                cursor = by_id.get(predecessor)
                    .ok_or_else(|| "reconciliation coverage ancestry is unresolved".to_string())?;
            }
        }

        // A coverage set names branch heads, so its members must be maximal
        // descendants under the declared branch point. An ancestor/descendant
        // pair is one continuing branch, not two independently covered heads.
        for (index, left_id) in self.covered_branch_heads.iter().enumerate() {
            for right_id in self.covered_branch_heads.iter().skip(index + 1) {
                let mut cursor = by_id.get(left_id)
                    .ok_or_else(|| "reconciliation coverage branch head is missing".to_string())?;
                let mut seen = std::collections::BTreeSet::new();
                while let Some(predecessor) = cursor.predecessor.as_ref() {
                    if !seen.insert(cursor.transition_id.clone()) {
                        return Err("reconciliation coverage graph contains a predecessor cycle".into());
                    }
                    if predecessor == right_id {
                        return Err("reconciliation coverage branch heads must be incomparable descendants".into());
                    }
                    if predecessor == &self.branch_point {
                        break;
                    }
                    cursor = by_id.get(predecessor)
                        .ok_or_else(|| "reconciliation coverage ancestry is unresolved".to_string())?;
                }

                let mut cursor = by_id.get(right_id)
                    .ok_or_else(|| "reconciliation coverage branch head is missing".to_string())?;
                let mut seen = std::collections::BTreeSet::new();
                while let Some(predecessor) = cursor.predecessor.as_ref() {
                    if !seen.insert(cursor.transition_id.clone()) {
                        return Err("reconciliation coverage graph contains a predecessor cycle".into());
                    }
                    if predecessor == left_id {
                        return Err("reconciliation coverage branch heads must be incomparable descendants".into());
                    }
                    if predecessor == &self.branch_point {
                        break;
                    }
                    cursor = by_id.get(predecessor)
                        .ok_or_else(|| "reconciliation coverage ancestry is unresolved".to_string())?;
                }
            }
        }
        Ok(())
    }
}

impl EvidenceDisposition {
    pub fn validate(&self, evidence: &IdentityRef) -> Result<(), String> {
        let witness = match self {
            Self::Active | Self::Unresolved => return Ok(()),
            Self::Superseded { by } | Self::Disputed { by } | Self::Retracted { by } => by,
        };
        witness.validate()?;
        if witness == evidence {
            return Err("evidence disposition witness cannot be the evidence record itself".into());
        }
        if !matches!(witness.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
            return Err("evidence disposition witness must be an EvidenceRecord or ReconciliationWitness".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalEvidenceApplicability {
    pub evidence: IdentityRef,
    pub configuration: IdentityRef,
    pub artifact: IdentityRef,
    pub evidence_target: LineageEdge,
    /// Explicit temporal witness for the exact configuration-to-artifact applicability.
    /// The evidence effectivity interval must be contained by this witness; the
    /// underlying lineage edge is never treated as timeless applicability.
    pub configuration_applicability: TemporalConfigurationApplicability,
    pub event_interval: EvidenceEventInterval,
    pub effectivity_interval: ApplicabilityInterval,
    /// Epistemic status is orthogonal to temporal scope and does not rewrite it.
    pub disposition: EvidenceDisposition,
}

impl TemporalEvidenceApplicability {
    pub fn validate(&self) -> Result<(), String> {
        self.evidence.validate()?;
        self.configuration.validate()?;
        self.artifact.validate()?;
        self.evidence_target.validate()?;
        self.configuration_applicability.validate()?;
        self.event_interval.validate()?
            .and_then(|_| self.validate_effectivity_containment())?;
        self.effectivity_interval.validate()?;
        self.disposition.validate(&self.evidence)?;

        if self.configuration.kind != IdentityKind::ConfigurationRevision {
            return Err("temporal evidence applicability requires a configuration revision".into());
        }
        if self.artifact.kind != IdentityKind::PhysicalArtifact {
            return Err("temporal evidence applicability requires a physical artifact".into());
        }

        let expected_relation = match self.evidence.kind {
            IdentityKind::InspectionRecord => LineageRelation::InspectedAs,
            IdentityKind::TestRecord => LineageRelation::TestedAs,
            IdentityKind::OperationalObservation => LineageRelation::ObservedAs,
            IdentityKind::MaintenanceEvent => LineageRelation::MaintainedAs,
            _ => return Err("temporal evidence applicability requires an inspection, test, observation, or maintenance event".into()),
        };

        if self.evidence_target.relation != expected_relation
            || self.evidence_target.source != self.evidence
            || self.evidence_target.target != self.artifact
        {
            return Err("evidence target must exactly bind the typed evidence event to the physical artifact".into());
        }

        if self.configuration_applicability.configuration != self.configuration
            || self.configuration_applicability.artifact != self.artifact
        {
            return Err("temporal configuration applicability must exactly bind the declared configuration and physical artifact".into());
        }

        Ok(())
    }

    /// Enforce scope containment without using event/effectivity overlap as a
    /// validity rule. Half-open intervals mean equal bounded ends are allowed;
    /// a finite configuration interval cannot contain an open-ended effectivity.
    fn validate_effectivity_containment(&self) -> Result<(), String> {
        let config = &self.configuration_applicability.interval;
        let effectivity = &self.effectivity_interval;

        if effectivity.start < config.start {
            return Err("evidence effectivity starts before configuration applicability".into());
        }

        match (config.end, effectivity.end) {
            (Some(config_end), Some(effectivity_end)) if effectivity_end > config_end => {
                Err("evidence effectivity extends beyond configuration applicability".into())
            }
            (Some(_), None) => {
                Err("open-ended evidence effectivity requires open-ended configuration applicability".into())
            }
            _ => Ok(()),
        }
    }

    pub fn temporal_overlap(&self) -> bool {
        let event = ApplicabilityInterval {
            start: self.event_interval.start,
            end: self.event_interval.end,
        };
        event.overlaps(&self.effectivity_interval)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(kind: IdentityKind, value: &str) -> IdentityRef {
        IdentityRef { kind, namespace: "mobility".into(), id: value.into() }
    }

    fn applies_to(configuration: &IdentityRef, artifact: &IdentityRef) -> LineageEdge {
        LineageEdge {
            relation: LineageRelation::AppliesTo,
            source: configuration.clone(),
            target: artifact.clone(),
        }
    }

    fn evidence_case() -> TemporalEvidenceApplicability {
        let evidence = id(IdentityKind::InspectionRecord, "inspection-1");
        let configuration = id(IdentityKind::ConfigurationRevision, "cfg-1");
        let artifact = id(IdentityKind::PhysicalArtifact, "artifact-1");

        TemporalEvidenceApplicability {
            evidence: evidence.clone(),
            configuration: configuration.clone(),
            artifact: artifact.clone(),
            evidence_target: LineageEdge {
                relation: LineageRelation::InspectedAs,
                source: evidence,
                target: artifact.clone(),
            },
            configuration_applicability: TemporalConfigurationApplicability {
                configuration: configuration.clone(),
                artifact: artifact.clone(),
                applicability: applies_to(&configuration, &artifact),
                interval: ApplicabilityInterval { start: 90, end: Some(300) },
            },
            event_interval: EvidenceEventInterval { start: 100, end: Some(110) },
            effectivity_interval: ApplicabilityInterval { start: 120, end: Some(200) },
            disposition: EvidenceDisposition::Active,
        }
    }

    #[test]
    fn event_time_and_effectivity_can_be_disjoint() {
        let value = evidence_case();
        assert!(value.validate().is_ok());
        assert!(!value.temporal_overlap());
    }

    #[test]
    fn effectivity_uses_half_open_bounds() {
        let value = evidence_case();
        assert!(value.effectivity_interval.contains(120));
        assert!(value.effectivity_interval.contains(199));
        assert!(!value.effectivity_interval.contains(200));
    }

    #[test]
    fn open_ended_effectivity_is_explicit() {
        let mut value = evidence_case();
        value.effectivity_interval.end = None;
        assert!(value.validate().is_ok());
    }

    #[test]
    fn zero_length_effectivity_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval.end = Some(value.effectivity_interval.start);
        assert!(value.validate().is_err());
    }

    #[test]
    fn wrong_artifact_binding_is_rejected() {
        let mut value = evidence_case();
        value.artifact = id(IdentityKind::PhysicalArtifact, "artifact-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn wrong_configuration_binding_is_rejected() {
        let mut value = evidence_case();
        value.configuration = id(IdentityKind::ConfigurationRevision, "cfg-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn evidence_does_not_inherit_to_a_replacement_artifact() {
        let mut value = evidence_case();
        value.evidence_target.target = id(IdentityKind::PhysicalArtifact, "artifact-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn evidence_does_not_inherit_to_a_successor_configuration() {
        let mut value = evidence_case();
        value.configuration_applicability.applicability.source = id(IdentityKind::ConfigurationRevision, "cfg-2");
        assert!(value.validate().is_err());
    }

    #[test]
    fn holochain_shaped_artifact_identity_is_rejected() {
        let mut value = evidence_case();
        value.artifact.namespace = "holochain".into();
        assert!(value.validate().is_err());
    }

    #[test]
    fn later_effectivity_does_not_erase_historical_event_time() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 1_000, end: Some(2_000) };
        assert!(value.validate().is_ok());
        assert_eq!(value.event_interval.start, 100);
    }

    #[test]
    fn effectivity_exactly_matches_configuration_interval() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 90, end: Some(300) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_strictly_inside_configuration_interval_is_accepted() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 100, end: Some(299) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_starting_at_configuration_end_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 300, end: Some(301) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn effectivity_starting_before_configuration_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 89, end: Some(200) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn effectivity_ending_at_configuration_end_is_accepted() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(300) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_extending_beyond_configuration_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(301) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn finite_configuration_cannot_contain_open_ended_effectivity() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: None };
        assert!(value.validate().is_err());
    }

    #[test]
    fn open_ended_configuration_can_contain_open_ended_effectivity() {
        let mut value = evidence_case();
        value.configuration_applicability.interval.end = None;
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: None };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn event_time_can_be_outside_effectivity_without_bypassing_containment() {
        let mut value = evidence_case();
        value.event_interval = EvidenceEventInterval { start: 10, end: Some(20) };
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(200) };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn temporal_applicability_witness_mismatch_is_rejected() {
        let mut value = evidence_case();
        value.configuration_applicability.interval.start = 100;
        value.configuration_applicability.interval.end = Some(200);
        value.effectivity_interval = ApplicabilityInterval { start: 150, end: Some(250) };
        assert!(value.validate().is_err());
    }

    #[test]
    fn disputed_disposition_does_not_change_temporal_scope() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "witness-1"),
        };
        assert!(value.validate().is_ok());
        assert_eq!(value.effectivity_interval, ApplicabilityInterval { start: 120, end: Some(200) });
        assert_eq!(value.event_interval.start, 100);
    }

    #[test]
    fn superseded_disposition_is_explicitly_witnessed() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Superseded {
            by: id(IdentityKind::EvidenceRecord, "evidence-2"),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn retracted_disposition_is_explicitly_witnessed() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Retracted {
            by: id(IdentityKind::ReconciliationWitness, "correction-1"),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn disposition_cannot_self_reference() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Disputed {
            by: value.evidence.clone(),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn disposition_witness_must_be_typed() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Superseded {
            by: id(IdentityKind::ConfigurationRevision, "cfg-2"),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn unresolved_disposition_preserves_temporal_evidence() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Unresolved;
        assert!(value.validate().is_ok());
        assert_eq!(value.effectivity_interval.start, 120);
    }

    fn graph_transition(
        transition_id: &str,
        predecessor: Option<&str>,
        from: EvidenceDisposition,
        to: EvidenceDisposition,
    ) -> EvidenceDispositionTransition {
        EvidenceDispositionTransition {
            transition_id: id(IdentityKind::ReconciliationWitness, transition_id),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            predecessor: predecessor.map(|p| id(IdentityKind::ReconciliationWitness, p)),
            from,
            to,
            basis: id(IdentityKind::ReconciliationWitness, "basis-graph"),
        }
    }

    #[test]
    fn transition_requires_genesis_active_state() {
        let t = graph_transition("genesis", None,
            EvidenceDisposition::Unresolved, EvidenceDisposition::Active);
        assert!(t.validate().is_err());
    }

    #[test]
    fn transition_rejects_terminal_retraction_reopening() {
        let t = graph_transition("later", Some("earlier"),
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "r1") },
            EvidenceDisposition::Active);
        assert!(t.validate().is_err());
    }

    #[test]
    fn transition_rejects_terminal_supersession_reopening() {
        let t = graph_transition("later", Some("earlier"),
            EvidenceDisposition::Superseded { by: id(IdentityKind::EvidenceRecord, "replacement") },
            EvidenceDisposition::Active);
        assert!(t.validate().is_err());
    }

    #[test]
    fn transition_requires_typed_basis() {
        let mut t = graph_transition("t1", None, EvidenceDisposition::Active,
            EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") });
        t.basis = id(IdentityKind::ConfigurationRevision, "cfg");
        assert!(t.validate().is_err());
    }

    #[test]
    fn graph_validates_predecessor_state_continuity() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let child = graph_transition("t2", Some("t1"), disputed, EvidenceDisposition::Active);
        assert_eq!(EvidenceDispositionTransition::validate_graph(&[root, child]),
            Ok(DispositionChainAssessment::Complete { branch_points: vec![] }));
    }

    #[test]
    fn graph_reports_missing_predecessor_as_unresolved() {
        let child = graph_transition("t2", Some("missing"), EvidenceDisposition::Active,
            EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") });
        assert_eq!(EvidenceDispositionTransition::validate_graph(&[child]),
            Ok(DispositionChainAssessment::Unresolved {
                missing: vec![id(IdentityKind::ReconciliationWitness, "missing")],
                branch_points: vec![],
            }));
    }

    #[test]
    fn graph_rejects_predecessor_state_mismatch() {
        let root = graph_transition("t1", None, EvidenceDisposition::Active,
            EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") });
        let child = graph_transition("t2", Some("t1"), EvidenceDisposition::Active,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") });
        assert!(EvidenceDispositionTransition::validate_graph(&[root, child]).is_err());
    }

    #[test]
    fn graph_preserves_competing_branches_explicitly() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition("t3", Some("t1"), disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") });
        assert_eq!(EvidenceDispositionTransition::validate_graph(&[root, left, right]),
            Ok(DispositionChainAssessment::Complete {
                branch_points: vec![id(IdentityKind::ReconciliationWitness, "t1")],
            }));
    }

    #[test]
    fn graph_rejects_predecessor_cycle() {
        let first = graph_transition(
            "cycle-a",
            Some("cycle-b"),
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "w1"),
            },
        );
        let second = graph_transition(
            "cycle-b",
            Some("cycle-a"),
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "w1"),
            },
            EvidenceDisposition::Active,
        );
        assert!(EvidenceDispositionTransition::validate_graph(&[first, second]).is_err());
    }

    #[test]
    fn graph_rejects_duplicate_transition_identity() {
        let one = graph_transition("t1", None, EvidenceDisposition::Active,
            EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") });
        assert!(EvidenceDispositionTransition::validate_graph(&[one.clone(), one]).is_err());
    }

    #[test]
    fn authority_scope_carries_reconciliation_basis() {
        let required = id(IdentityKind::EvidenceRecord, "authority-basis-1");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-authority-basis"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![required.clone()],
        };
        let missing = authority_scope("reconcile-authority-basis");
        assert!(missing.validate_against_reconciliation(&reconciliation, &authority_delegation(&reconciliation.reconciliation_id.id)).is_err());

        let complete = EvidenceDispositionAuthorityScope {
            scope_id: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![required],
        };
        assert!(complete.validate_against_reconciliation(&reconciliation, &authority_delegation(&reconciliation.reconciliation_id.id)).is_ok());
    }

    #[test]
    fn reconciliation_requires_competing_branch_heads() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-1"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2")],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        assert!(reconciliation.validate().is_err());
    }

    #[test]
    fn reconciliation_validates_competing_branch_ancestry_without_selecting_winner() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-1"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        assert!(reconciliation.validate_against_graph(&authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &[root, left, right]).is_ok());
    }



    #[test]
    fn reconciliation_rejects_branch_point_as_head() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let right = graph_transition(
            "t2",
            Some("t1"),
            disputed,
            EvidenceDisposition::Active,
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-branch-point"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t1"),
                id(IdentityKind::ReconciliationWitness, "t2"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        assert!(reconciliation.validate_against_graph(&authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &[root, right]).is_err());
    }

    #[test]
    fn reconciliation_rejects_nested_heads_as_competing_branches() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let first = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let second = graph_transition(
            "t3",
            Some("t2"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-nested"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        assert!(reconciliation.validate_against_graph(&authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &[root, first, second]).is_err());
    }


    fn authority_delegation(subject: &str) -> EvidenceDispositionAuthorityDelegation {
        EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            grantor: id(IdentityKind::ReconciliationWitness, "authority-grantor-1"),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-1"),
            subject: id(IdentityKind::ReconciliationWitness, subject),
            predecessor: None,
            basis: vec![],
        }
    }

    fn authority_scope(subject: &str) -> EvidenceDispositionAuthorityScope {
        EvidenceDispositionAuthorityScope {
            scope_id: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            subject: id(IdentityKind::ReconciliationWitness, subject),
            delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        }
    }

    fn coverage_boundary() -> EvidenceDispositionCoverageBoundary {
        EvidenceDispositionCoverageBoundary {
            boundary_id: id(IdentityKind::ReconciliationWitness, "enumeration-boundary-1"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        }
    }

    #[test]
    fn reconciliation_coverage_accepts_bounded_declared_head_set() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition("t3", Some("t1"), disputed, EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") });
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2"), id(IdentityKind::ReconciliationWitness, "t3")],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-1"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2"), id(IdentityKind::ReconciliationWitness, "t3")],
            boundary: id(IdentityKind::ReconciliationWitness, "enumeration-boundary-1"),
            basis: vec![],
        };
        assert!(coverage.validate_against_graph(&reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &coverage_boundary(), &[root, left, right]).is_ok());
    }

    #[test]
    fn reconciliation_coverage_rejects_nested_heads() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let first = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let second = graph_transition(
            "t3",
            Some("t2"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-coverage-nested"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-nested"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-coverage-nested"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: id(IdentityKind::ReconciliationWitness, "enumeration-boundary-1"),
            basis: vec![],
        };
        assert!(coverage.validate_against_graph(&reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &coverage_boundary(), &[root, first, second]).is_err());
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_mismatch() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition("t3", Some("t1"), disputed, EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") });
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-boundary-mismatch"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2"), id(IdentityKind::ReconciliationWitness, "t3")],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-boundary-mismatch"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-boundary-mismatch"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2"), id(IdentityKind::ReconciliationWitness, "t3")],
            boundary: id(IdentityKind::ReconciliationWitness, "different-boundary"),
            basis: vec![],
        };
        assert!(coverage.validate_against_graph(
            &reconciliation,
            &coverage_boundary(),
            &[root, left, right],
        ).is_err());
    }

    #[test]
    fn reconciliation_coverage_rejects_uncovered_reconciliation_head() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition("t3", Some("t1"), disputed, EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") });
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2"), id(IdentityKind::ReconciliationWitness, "t3")],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-2"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2")],
            boundary: id(IdentityKind::ReconciliationWitness, "enumeration-boundary-1"),
            basis: vec![],
        };
        assert!(coverage.validate_against_graph(&reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &coverage_boundary(), &[root, left, right]).is_err());
    }


    #[test]
    fn reconciliation_coverage_rejects_boundary_reconciliation_mismatch() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-boundary-reconciliation-mismatch"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        let boundary = EvidenceDispositionCoverageBoundary {
            reconciliation: id(IdentityKind::ReconciliationWitness, "different-reconciliation"),
            ..coverage_boundary()
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-reconciliation-mismatch"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: boundary.boundary_id.clone(),
            basis: vec![],
        };
        assert!(coverage.validate_against_graph(&reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &boundary, &[root, left, right]).is_err());
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_authority_mismatch() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-boundary-authority-mismatch"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        let boundary = EvidenceDispositionCoverageBoundary {
            authority: id(IdentityKind::ReconciliationWitness, "different-authority"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            ..coverage_boundary()
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-authority-mismatch"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: boundary.boundary_id.clone(),
            basis: vec![],
        };
        assert!(coverage.validate_against_graph(&reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &boundary, &[root, left, right]).is_err());
    }


    #[test]
    fn reconciliation_coverage_rejects_boundary_authority_scope_mismatch() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-boundary-scope-mismatch"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.authority_scope = id(IdentityKind::ReconciliationWitness, "different-authority-scope");
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-boundary-scope-mismatch"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(coverage.validate_against_graph(
            &reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &boundary,
            &[root, left, right],
        ).is_err());
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_basis_omission() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let required_basis = id(IdentityKind::EvidenceRecord, "reconciliation-basis-1");
        let mut reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-basis-omission"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![required_basis],
        };
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-basis-omission"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(coverage.validate_against_graph(&reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &boundary, &[root, left, right]).is_err());
        reconciliation.basis.clear();
    }

    #[test]
    fn reconciliation_coverage_accepts_boundary_basis_extension() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let required_basis = id(IdentityKind::EvidenceRecord, "reconciliation-basis-1");
        let additional_basis = id(IdentityKind::ReconciliationWitness, "coverage-basis-1");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-basis-extension"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![required_basis.clone()],
        };
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![required_basis, additional_basis];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-basis-extension"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(coverage.validate_against_graph(&reconciliation, &authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &boundary, &[root, left, right]).is_ok());
    }

    #[test]
    fn reconciliation_rejects_branch_head_from_unrelated_evidence() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let other = EvidenceDispositionTransition {
            transition_id: id(IdentityKind::ReconciliationWitness, "other"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-other"),
            predecessor: None,
            from: EvidenceDisposition::Active,
            to: disputed,
            basis: id(IdentityKind::ReconciliationWitness, "basis-other"),
        };
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-2"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t1"),
                id(IdentityKind::ReconciliationWitness, "other"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![],
        };
        assert!(reconciliation.validate_against_graph(&authority_scope(&reconciliation.reconciliation_id.id), &authority_delegation(&reconciliation.reconciliation_id.id), &[root, other]).is_err());
    }

    #[test]
    fn wrong_evidence_kind_is_rejected() {
        let mut value = evidence_case();
        value.evidence = id(IdentityKind::DesignRevision, "design-1");
        assert!(value.validate().is_err());
    }
}
