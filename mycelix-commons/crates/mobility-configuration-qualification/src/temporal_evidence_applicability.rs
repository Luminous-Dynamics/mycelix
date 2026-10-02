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
    /// Exact prior delegation whose grantee granted this delegation.
    pub predecessor: Option<IdentityRef>,
    pub basis: Vec<IdentityRef>,
}

/// Result of validating a supplied authority-delegation graph.
///
/// Multiple roots are allowed because independently issued root delegations
/// may coexist. A delegated edge must, however, preserve the subject and make
/// the predecessor's grantee the current grantor. This is structural
/// provenance only; it does not prove real-world authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthorityDelegationChainAssessment {
    Complete { roots: Vec<IdentityRef> },
    Unresolved { missing: Vec<IdentityRef>, roots: Vec<IdentityRef> },
}

impl EvidenceDispositionAuthorityDelegation {
    /// Validate the supplied delegation records as a finite, addressable
    /// predecessor graph. Missing predecessors remain unresolved; a closed
    /// cycle is invalid because it has no historical root.
    /// Validate only the predecessor chain rooted at a named delegation.
    ///
    /// This is intentionally narrower than validate_graph: unrelated
    /// delegations in the supplied set do not become dependencies of the
    /// named chain. Only the target and its explicit predecessors are
    /// structurally validated.
    pub fn validate_chain(
        delegation_id: &IdentityRef,
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> Result<AuthorityDelegationChainAssessment, String> {
        use std::collections::BTreeMap;

        let mut by_id: BTreeMap<IdentityRef, Vec<&EvidenceDispositionAuthorityDelegation>> = BTreeMap::new();
        for delegation in delegations {
            by_id.entry(delegation.delegation_id.clone()).or_default().push(delegation);
        }

        let target = match by_id.get(delegation_id) {
            None => {
                return Ok(AuthorityDelegationChainAssessment::Unresolved {
                    missing: vec![delegation_id.clone()],
                    roots: vec![],
                });
            }
            Some(candidates) if candidates.len() != 1 => {
                return Err("duplicate authority delegation identity".into());
            }
            Some(candidates) => candidates[0],
        };
        target.validate()?;

        let mut current = target;
        let mut seen = std::collections::BTreeSet::new();
        loop {
            if !seen.insert(current.delegation_id.clone()) {
                return Err("authority delegation graph contains a predecessor cycle".into());
            }
            match &current.predecessor {
                None => {
                    return Ok(AuthorityDelegationChainAssessment::Complete {
                        roots: vec![current.delegation_id.clone()],
                    });
                }
                Some(predecessor_id) => match by_id.get(predecessor_id) {
                    None => {
                        return Ok(AuthorityDelegationChainAssessment::Unresolved {
                            missing: vec![predecessor_id.clone()],
                            roots: vec![],
                        });
                    }
                    Some(candidates) if candidates.len() != 1 => {
                        return Err("duplicate authority delegation identity".into());
                    }
                    Some(candidates) => {
                        let predecessor = candidates[0];
                        predecessor.validate()?;
                        if predecessor.subject != current.subject {
                            return Err("authority delegation predecessor subject must match".into());
                        }
                        if predecessor.grantee != current.grantor {
                            return Err("authority delegation grantor must match predecessor grantee".into());
                        }
                        for basis in &predecessor.basis {
                            if !current.basis.contains(basis) {
                                return Err("authority delegation basis must preserve predecessor provenance".into());
                            }
                        }
                        current = predecessor;
                    }
                },
            }
        }
    }

    pub fn validate_graph(
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> Result<AuthorityDelegationChainAssessment, String> {
        use std::collections::{BTreeMap, BTreeSet};

        let mut by_id = BTreeMap::new();
        for delegation in delegations {
            delegation.validate()?;
            if by_id.insert(delegation.delegation_id.clone(), delegation).is_some() {
                return Err("duplicate authority delegation identity".into());
            }
        }

        let mut missing = Vec::new();
        let mut roots = Vec::new();
        for delegation in delegations {
            match &delegation.predecessor {
                None => roots.push(delegation.delegation_id.clone()),
                Some(predecessor_id) => match by_id.get(predecessor_id) {
                    None => missing.push(predecessor_id.clone()),
                    Some(predecessor) => {
                        if predecessor.subject != delegation.subject {
                            return Err("authority delegation predecessor subject must match".into());
                        }
                        if predecessor.grantee != delegation.grantor {
                            return Err("authority delegation grantor must match predecessor grantee".into());
                        }
                        for basis in &predecessor.basis {
                            if !delegation.basis.contains(basis) {
                                return Err("authority delegation basis must preserve predecessor provenance".into());
                            }
                        }
                    }
                },
            }
        }

        missing.sort_by(|a, b| (&a.namespace, &a.id).cmp(&(&b.namespace, &b.id)));
        missing.dedup();
        roots.sort_by(|a, b| (&a.namespace, &a.id).cmp(&(&b.namespace, &b.id)));

        for delegation in delegations {
            let mut seen = BTreeSet::new();
            let mut cursor = delegation;
            while let Some(predecessor_id) = &cursor.predecessor {
                if !seen.insert(cursor.delegation_id.clone()) {
                    return Err("authority delegation graph contains a predecessor cycle".into());
                }
                match by_id.get(predecessor_id) {
                    Some(predecessor) => cursor = predecessor,
                    None => break,
                }
            }
        }

        if missing.is_empty() {
            Ok(AuthorityDelegationChainAssessment::Complete { roots })
        } else {
            Ok(AuthorityDelegationChainAssessment::Unresolved { missing, roots })
        }
    }

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
        if self.delegation_id != scope.delegation {
            return Err("authority scope references a different delegation witness".into());
        }
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

    /// Validate this scope against its reconciliation and the complete named
    /// authority delegation chain. The delegation graph is scoped to the
    /// exact delegation identity referenced by this scope.
    pub fn validate_against_reconciliation_and_authority_chain(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> Result<AuthorityDelegationChainAssessment, String> {
        self.validate()?;
        reconciliation.validate()?;
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
        EvidenceDispositionAuthorityDelegation::validate_chain(
            &self.delegation,
            delegations,
        ).and_then(|assessment| {
            let delegation = delegations.iter().find(|d| d.delegation_id == self.delegation)
                .ok_or_else(|| "authority scope delegation is missing".to_string())?;
            delegation.validate_against_scope(self)?;
            Ok(assessment)
        })
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
        if self.authority_scope == self.reconciliation_id
            || self.authority_scope == self.authority
            || self.authority_delegation == self.reconciliation_id
            || self.authority_delegation == self.authority
            || self.authority_delegation == self.authority_scope
        {
            return Err("reconciliation authority scope cannot equal reconciliation or authority identity".into());
        }
        if self.authority_delegation != self.authority_scope.delegation {
            return Err("reconciliation authority delegation must match authority scope delegation".into());
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
        let mut basis = std::collections::BTreeSet::new();
        for witness in &self.basis {
            witness.validate()?;
            if !matches!(witness.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("reconciliation basis must be an addressable witness".into());
            }
            if !basis.insert(witness.clone()) {
                return Err("reconciliation basis must be unique".into());
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
        if self.authority_scope == self.boundary_id
            || self.authority_scope == self.authority
            || self.authority_delegation == self.boundary_id
            || self.authority_delegation == self.authority
            || self.authority_delegation == self.authority_scope
        {
            return Err("coverage boundary authority scope cannot equal boundary or authority identity".into());
        }
        if self.authority_delegation != self.authority_scope.delegation {
            return Err("coverage boundary authority delegation must match authority scope delegation".into());
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EvidenceDispositionCoverageAssessment {
    Complete { authority_roots: Vec<IdentityRef> },
    Unresolved {
        missing: Vec<IdentityRef>,
        authority_roots: Vec<IdentityRef>,
    },
}

impl EvidenceDispositionReconciliationCoverage {
    fn collect_missing_transition_dependencies(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        transitions: &[EvidenceDispositionTransition],
    ) -> Result<Vec<IdentityRef>, String> {
        use std::collections::{BTreeMap, BTreeSet};

        let mut by_id = BTreeMap::new();
        for transition in transitions {
            transition.validate()?;
            if by_id
                .insert(transition.transition_id.clone(), transition)
                .is_some()
            {
                return Err("duplicate disposition transition identity".into());
            }
        }

        let mut missing = BTreeSet::new();
        let mut roots_to_check = vec![reconciliation.branch_point.clone()];
        roots_to_check.extend(reconciliation.branch_heads.iter().cloned());
        roots_to_check.extend(self.covered_branch_heads.iter().cloned());

        for start_id in roots_to_check {
            let Some(mut cursor) = by_id.get(&start_id).copied() else {
                missing.insert(start_id);
                continue;
            };
            let mut seen = BTreeSet::new();

            loop {
                if !seen.insert(cursor.transition_id.clone()) {
                    return Err("reconciliation coverage graph contains a predecessor cycle".into());
                }

                if cursor.evidence != reconciliation.evidence {
                    return Err(
                        "reconciliation coverage transition belongs to different evidence".into()
                    );
                }

                match &cursor.predecessor {
                    None => break,
                    Some(predecessor_id) => match by_id.get(predecessor_id) {
                        None => {
                            missing.insert(predecessor_id.clone());
                            break;
                        }
                        Some(predecessor) => {
                            if predecessor.evidence != cursor.evidence {
                                return Err(
                                    "reconciliation coverage predecessor belongs to different evidence"
                                        .into(),
                                );
                            }
                            if predecessor.to != cursor.from {
                                return Err(
                                    "reconciliation coverage transition source state does not match predecessor target state"
                                        .into(),
                                );
                            }
                            cursor = predecessor;
                        }
                    },
                }
            }
        }

        Ok(missing.into_iter().collect())
    }

    /// Canonical bounded qualification assessment.
    ///
    /// Unlike the legacy unit-returning validators, this preserves the
    /// distinction between structural invalidity and unavailable transition or
    /// authority dependencies.
    pub fn validate_against_graph_and_authority_chain_assessment(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        authority_scope: &EvidenceDispositionAuthorityScope,
        authority_delegation: &EvidenceDispositionAuthorityDelegation,
        delegations: &[EvidenceDispositionAuthorityDelegation],
        boundary: &EvidenceDispositionCoverageBoundary,
        transitions: &[EvidenceDispositionTransition],
    ) -> Result<EvidenceDispositionCoverageAssessment, String> {
        self.validate()?;
        reconciliation.validate()?;
        authority_scope.validate_against_reconciliation(
            reconciliation,
            authority_delegation,
        )?;
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
        let covered_heads: std::collections::BTreeSet<_> =
            self.covered_branch_heads.iter().cloned().collect();
        let boundary_heads: std::collections::BTreeSet<_> =
            boundary.branch_heads.iter().cloned().collect();
        if covered_heads != boundary_heads {
            return Err("reconciliation coverage heads must match the coverage boundary set".into());
        }
        let coverage_basis: std::collections::BTreeSet<_> =
            self.basis.iter().cloned().collect();
        let boundary_basis: std::collections::BTreeSet<_> =
            boundary.basis.iter().cloned().collect();
        if coverage_basis != boundary_basis {
            return Err("reconciliation coverage basis must match the coverage boundary set".into());
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
        for basis in &authority_scope.basis {
            if !boundary.basis.contains(basis) {
                return Err("coverage boundary basis must include every authority scope basis witness".into());
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

        let missing_transitions =
            self.collect_missing_transition_dependencies(reconciliation, transitions)?;
        if !missing_transitions.is_empty() {
            return Ok(EvidenceDispositionCoverageAssessment::Unresolved {
                missing: missing_transitions,
                authority_roots: vec![],
            });
        }

        self.validate_against_graph(
            reconciliation,
            authority_scope,
            authority_delegation,
            boundary,
            transitions,
        )?;

        match EvidenceDispositionAuthorityDelegation::validate_chain(
            &authority_delegation.delegation_id,
            delegations,
        )? {
            AuthorityDelegationChainAssessment::Complete { roots } => {
                Ok(EvidenceDispositionCoverageAssessment::Complete {
                    authority_roots: roots,
                })
            }
            AuthorityDelegationChainAssessment::Unresolved { missing, roots } => {
                Ok(EvidenceDispositionCoverageAssessment::Unresolved {
                    missing,
                    authority_roots: roots,
                })
            }
        }
    }

    /// Validate bounded coverage and the exact authority delegation chain it relies on.
    ///
    /// The delegation graph is scoped to the named delegation identity;
    /// unrelated delegation records do not become dependencies of this check.
    pub fn validate_against_graph_and_authority_chain(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        authority_scope: &EvidenceDispositionAuthorityScope,
        authority_delegation: &EvidenceDispositionAuthorityDelegation,
        delegations: &[EvidenceDispositionAuthorityDelegation],
        boundary: &EvidenceDispositionCoverageBoundary,
        transitions: &[EvidenceDispositionTransition],
    ) -> Result<AuthorityDelegationChainAssessment, String> {
        self.validate_against_graph(
            reconciliation,
            authority_scope,
            authority_delegation,
            boundary,
            transitions,
        )?;
        EvidenceDispositionAuthorityDelegation::validate_chain(
            &authority_delegation.delegation_id,
            delegations,
        )
    }

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
        let mut basis = std::collections::BTreeSet::new();
        for witness in &self.basis {
            witness.validate()?;
            if !matches!(witness.kind, IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness) {
                return Err("reconciliation coverage basis must be an addressable witness".into());
            }
            if !basis.insert(witness.clone()) {
                return Err("reconciliation coverage basis must be unique".into());
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
        let covered_heads: std::collections::BTreeSet<_> =
            self.covered_branch_heads.iter().cloned().collect();
        let boundary_heads: std::collections::BTreeSet<_> =
            boundary.branch_heads.iter().cloned().collect();
        if covered_heads != boundary_heads {
            return Err("reconciliation coverage heads must match the coverage boundary set".into());
        }
        let coverage_basis: std::collections::BTreeSet<_> =
            self.basis.iter().cloned().collect();
        let boundary_basis: std::collections::BTreeSet<_> =
            boundary.basis.iter().cloned().collect();
        if coverage_basis != boundary_basis {
            return Err("reconciliation coverage basis must match the coverage boundary set".into());
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
        for basis in &authority_scope.basis {
            if !boundary.basis.contains(basis) {
                return Err("coverage boundary basis must include every authority scope basis witness".into());
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
    fn authority_delegation_graph_requires_chain_continuity() {
        let root = authority_delegation("reconcile-delegation-chain");
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_graph(&[root.clone(), child]),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![root.delegation_id],
            })
        );
    }

    #[test]
    fn authority_delegation_target_chain_ignores_unrelated_invalid_record() {
        let root = authority_delegation("reconcile-delegation-target-valid");
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-target-valid-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-target-valid-child"),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        let unrelated = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-unrelated-invalid"),
            grantor: id(IdentityKind::ConfigurationRevision, "invalid-grantor-kind"),
            grantee: id(IdentityKind::ReconciliationWitness, "unrelated-grantee"),
            subject: id(IdentityKind::ReconciliationWitness, "unrelated-subject"),
            predecessor: None,
            basis: vec![],
        };

        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_chain(
                &child.delegation_id,
                &[root.clone(), child, unrelated],
            ),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![root.delegation_id],
            })
        );
    }

    #[test]
    fn authority_delegation_target_chain_reports_missing_predecessor() {
        let delegation = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-target-missing"),
            grantor: id(IdentityKind::ReconciliationWitness, "authority-grantor-target"),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-target"),
            subject: id(IdentityKind::ReconciliationWitness, "reconcile-target-missing"),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-target-missing")),
            basis: vec![],
        };
        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_chain(
                &delegation.delegation_id,
                &[delegation],
            ),
            Ok(AuthorityDelegationChainAssessment::Unresolved {
                missing: vec![id(
                    IdentityKind::ReconciliationWitness,
                    "authority-target-missing"
                )],
                roots: vec![],
            })
        );
    }
    #[test]
    fn authority_delegation_target_chain_reports_missing_target_as_unresolved() {
        let missing = id(
            IdentityKind::ReconciliationWitness,
            "authority-delegation-target-absent",
        );

        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_chain(&missing, &[]),
            Ok(AuthorityDelegationChainAssessment::Unresolved {
                missing: vec![missing],
                roots: vec![],
            })
        );
    }

    #[test]
    fn reconciliation_coverage_assessment_reports_missing_transition_dependency() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let left = graph_transition(
            "t2",
            Some("missing-transition-parent"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("missing-transition-parent"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-transition-missing",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(
                IdentityKind::ReconciliationWitness,
                "missing-transition-parent",
            ),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
            basis: vec![],
        };
        let scope = authority_scope(&reconciliation.reconciliation_id.id);
        let delegation = authority_delegation(&reconciliation.reconciliation_id.id);
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-transition-missing",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain_assessment(
                &reconciliation,
                &scope,
                &delegation,
                &[delegation.clone()],
                &boundary,
                &[left, right],
            ),
            Ok(EvidenceDispositionCoverageAssessment::Unresolved {
                missing: vec![id(
                    IdentityKind::ReconciliationWitness,
                    "missing-transition-parent",
                )],
                authority_roots: vec![],
            })
        );
    }

    #[test]
    fn authority_delegation_graph_requires_basis_continuity() {
        let mut root = authority_delegation("reconcile-delegation-basis");
        let inherited = id(IdentityKind::EvidenceRecord, "delegation-root-basis");
        root.basis = vec![inherited.clone()];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![inherited],
        };
        assert!(EvidenceDispositionAuthorityDelegation::validate_graph(&[root, child]).is_ok());
    }

    #[test]
    fn authority_provenance_chain_allows_monotonic_basis_accumulation() {
        let mut root = authority_delegation("reconcile-delegation-basis-monotonic");
        let delegation_basis = id(IdentityKind::EvidenceRecord, "delegation-monotonic-basis");
        let scope_basis = id(IdentityKind::ReconciliationWitness, "scope-monotonic-basis");
        let boundary_basis = id(IdentityKind::EvidenceRecord, "boundary-monotonic-basis");
        root.basis = vec![delegation_basis.clone()];

        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-monotonic-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-monotonic-child"),
            subject: id(IdentityKind::ReconciliationWitness, "reconcile-monotonic"),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![delegation_basis.clone(), scope_basis.clone()],
        };
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-monotonic"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: child.grantee.clone(),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-monotonic"),
            authority_delegation: child.delegation_id.clone(),
            basis: vec![delegation_basis.clone()],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: child.grantee.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: child.delegation_id.clone(),
            basis: vec![delegation_basis.clone(), scope_basis.clone()],
        };
        let boundary = EvidenceDispositionCoverageBoundary {
            boundary_id: id(IdentityKind::ReconciliationWitness, "boundary-monotonic"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            evidence: reconciliation.evidence.clone(),
            branch_point: reconciliation.branch_point.clone(),
            branch_heads: reconciliation.branch_heads.clone(),
            authority: reconciliation.authority.clone(),
            authority_scope: reconciliation.authority_scope.clone(),
            authority_delegation: child.delegation_id.clone(),
            basis: vec![delegation_basis, scope_basis, boundary_basis],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-monotonic"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root_transition = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain(
                &reconciliation,
                &scope,
                &child,
                &[root, child.clone()],
                &boundary,
                &[root_transition, left, right],
            ),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![child.predecessor.unwrap()],
            })
        );
    }

    #[test]
    fn reconciliation_coverage_reports_unresolved_named_authority_chain() {
        let mut root = authority_delegation("reconcile-coverage-authority-missing");
        root.delegation_id = id(IdentityKind::ReconciliationWitness, "authority-coverage-root");
        root.grantee = id(IdentityKind::ReconciliationWitness, "authority-coverage-grantor");
        root.subject = id(IdentityKind::ReconciliationWitness, "reconcile-coverage-authority-missing");
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-coverage-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-coverage"),
            subject: root.subject.clone(),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-coverage-missing")),
            basis: vec![],
        };
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: root.subject.clone(),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: child.grantee.clone(),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-coverage-missing"),
            authority_delegation: child.delegation_id.clone(),
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: child.delegation_id.clone(),
            basis: vec![],
        };
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.authority = reconciliation.authority.clone();
        boundary.authority_scope = reconciliation.authority_scope.clone();
        boundary.authority_delegation = reconciliation.authority_delegation.clone();
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-authority-missing"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root_transition = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain(
                &reconciliation,
                &scope,
                &child,
                &[root, child],
                &boundary,
                &[root_transition, left, right],
            ),
            Ok(AuthorityDelegationChainAssessment::Unresolved {
                missing: vec![id(
                    IdentityKind::ReconciliationWitness,
                    "authority-coverage-missing"
                )],
                roots: vec![],
            })
        );
    }

    #[test]
    fn reconciliation_coverage_validates_named_authority_chain_and_ignores_unrelated_invalid_delegation() {
        let mut root = authority_delegation("reconcile-delegation-e2e");
        root.basis = vec![id(IdentityKind::EvidenceRecord, "delegation-e2e-basis")];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-chain-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-e2e-child"),
            subject: id(IdentityKind::ReconciliationWitness, "reconcile-e2e"),
            predecessor: Some(root.delegation_id.clone()),
            basis: root.basis.clone(),
        };
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-e2e"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: child.grantee.clone(),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-e2e"),
            authority_delegation: child.delegation_id.clone(),
            basis: root.basis.clone(),
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: child.grantee.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: child.delegation_id.clone(),
            basis: root.basis.clone(),
        };
        let boundary = EvidenceDispositionCoverageBoundary {
            boundary_id: id(IdentityKind::ReconciliationWitness, "boundary-e2e"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            evidence: reconciliation.evidence.clone(),
            branch_point: reconciliation.branch_point.clone(),
            branch_heads: reconciliation.branch_heads.clone(),
            authority: reconciliation.authority.clone(),
            authority_scope: reconciliation.authority_scope.clone(),
            authority_delegation: child.delegation_id.clone(),
            basis: root.basis.clone(),
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-e2e"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root_transition = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let unrelated = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-e2e-unrelated-invalid"),
            grantor: id(IdentityKind::ConfigurationRevision, "invalid-grantor-e2e"),
            grantee: id(IdentityKind::ReconciliationWitness, "unrelated-grantee-e2e"),
            subject: id(IdentityKind::ReconciliationWitness, "unrelated-subject-e2e"),
            predecessor: None,
            basis: vec![],
        };
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain(
                &reconciliation,
                &scope,
                &child,
                &[root, child.clone(), unrelated],
                &boundary,
                &[root_transition, left, right],
            ),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![child.predecessor.unwrap()],
            })
        );
    }

    #[test]
    fn authority_delegation_graph_allows_basis_extension() {
        let mut root = authority_delegation("reconcile-delegation-basis-extension");
        let inherited = id(IdentityKind::EvidenceRecord, "delegation-root-basis");
        let extension = id(IdentityKind::ReconciliationWitness, "delegation-child-basis");
        root.basis = vec![inherited.clone()];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![inherited, extension],
        };
        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_graph(&[root, child]),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![id(
                    IdentityKind::ReconciliationWitness,
                    "reconcile-delegation-basis-extension"
                )],
            })
        );
    }

    #[test]
    fn authority_delegation_target_chain_ignores_unrelated_missing_predecessor() {
        let mut root = authority_delegation("reconcile-delegation-target-root");
        root.basis = vec![id(IdentityKind::EvidenceRecord, "target-root-basis")];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-target-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-target-child"),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: root.basis.clone(),
        };
        let unrelated = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-unrelated"),
            grantor: id(IdentityKind::ReconciliationWitness, "unrelated-grantor"),
            grantee: id(IdentityKind::ReconciliationWitness, "unrelated-grantee"),
            subject: id(IdentityKind::ReconciliationWitness, "unrelated-subject"),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "unrelated-missing")),
            basis: vec![],
        };

        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_chain(
                &child.delegation_id,
                &[root.clone(), child, unrelated],
            ),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![root.delegation_id],
            })
        );
    }
    #[test]
    fn authority_delegation_graph_rejects_provenance_drop() {
        let mut root = authority_delegation("reconcile-delegation-basis-drop");
        root.basis = vec![id(IdentityKind::EvidenceRecord, "delegation-root-basis")];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        assert!(EvidenceDispositionAuthorityDelegation::validate_graph(&[root, child]).is_err());
    }

    #[test]
    fn authority_delegation_graph_rejects_grantor_discontinuity() {
        let root = authority_delegation("reconcile-delegation-discontinuity");
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-child"),
            grantor: id(IdentityKind::ReconciliationWitness, "unrelated-grantor"),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        assert!(EvidenceDispositionAuthorityDelegation::validate_graph(&[root, child]).is_err());
    }

    #[test]
    fn authority_delegation_graph_rejects_subject_change() {
        let root = authority_delegation("reconcile-delegation-subject");
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-child"),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: id(IdentityKind::ReconciliationWitness, "different-subject"),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        assert!(EvidenceDispositionAuthorityDelegation::validate_graph(&[root, child]).is_err());
    }

    #[test]
    fn authority_delegation_graph_reports_missing_predecessor() {
        let delegation = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-missing"),
            grantor: id(IdentityKind::ReconciliationWitness, "authority-grantor-1"),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: id(IdentityKind::ReconciliationWitness, "reconcile-delegation-missing"),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-delegation-absent")),
            basis: vec![],
        };
        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_graph(&[delegation]),
            Ok(AuthorityDelegationChainAssessment::Unresolved {
                missing: vec![id(IdentityKind::ReconciliationWitness, "authority-delegation-absent")],
                roots: vec![],
            })
        );
    }

    #[test]
    fn authority_delegation_graph_rejects_cycle() {
        let a = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-cycle-a"),
            grantor: id(IdentityKind::ReconciliationWitness, "grantor-a"),
            grantee: id(IdentityKind::ReconciliationWitness, "grantor-b"),
            subject: id(IdentityKind::ReconciliationWitness, "reconcile-delegation-cycle"),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-cycle-b")),
            basis: vec![],
        };
        let b = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-cycle-b"),
            grantor: id(IdentityKind::ReconciliationWitness, "grantor-b"),
            grantee: id(IdentityKind::ReconciliationWitness, "grantor-a"),
            subject: id(IdentityKind::ReconciliationWitness, "reconcile-delegation-cycle"),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-cycle-a")),
            basis: vec![],
        };
        assert!(EvidenceDispositionAuthorityDelegation::validate_graph(&[a, b]).is_err());
    }

    #[test]
    fn authority_delegation_scope_requires_exact_delegation_identity() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-delegation-binding"),
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
        let scope = authority_scope(&reconciliation.reconciliation_id.id);
        let mut delegation = authority_delegation(&reconciliation.reconciliation_id.id);
        delegation.delegation_id = id(IdentityKind::ReconciliationWitness, "different-delegation");
        assert!(scope.validate_against_reconciliation(&reconciliation, &delegation).is_err());
    }

    #[test]
    fn authority_scope_ignores_unrelated_invalid_delegation() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-scope-isolation"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-scope-isolation"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-isolation-witness"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-scope-isolation"),
            basis: vec![],
        };
        let mut root = authority_delegation(&reconciliation.reconciliation_id.id);
        root.delegation_id = id(IdentityKind::ReconciliationWitness, "authority-delegation-isolation-root");
        root.grantee = id(IdentityKind::ReconciliationWitness, "authority-scope-isolation-grantor");
        root.subject = reconciliation.reconciliation_id.clone();
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: reconciliation.authority_delegation.clone(),
            grantor: root.grantee.clone(),
            grantee: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        let unrelated = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-delegation-isolation-invalid"),
            grantor: id(IdentityKind::ConfigurationRevision, "invalid-grantor"),
            grantee: id(IdentityKind::ReconciliationWitness, "unrelated-grantee"),
            subject: id(IdentityKind::ReconciliationWitness, "unrelated-subject"),
            predecessor: None,
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: child.delegation_id.clone(),
            basis: vec![],
        };

        assert_eq!(
            scope.validate_against_reconciliation_and_authority_chain(
                &reconciliation,
                &[root.clone(), child, unrelated],
            ),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![root.delegation_id],
            })
        );
    }

    #[test]
    fn authority_scope_reports_missing_named_delegation_predecessor() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-scope-chain-missing"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-scope-chain-missing"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-chain-missing-witness"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-scope-chain-missing"),
            basis: vec![],
        };
        let delegation = EvidenceDispositionAuthorityDelegation {
            delegation_id: reconciliation.authority_delegation.clone(),
            grantor: id(IdentityKind::ReconciliationWitness, "authority-grantor-scope-chain-missing"),
            grantee: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-delegation-missing-predecessor")),
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: delegation.delegation_id.clone(),
            basis: vec![],
        };
        assert_eq!(
            scope.validate_against_reconciliation_and_authority_chain(
                &reconciliation,
                &[delegation],
            ),
            Ok(AuthorityDelegationChainAssessment::Unresolved {
                missing: vec![id(
                    IdentityKind::ReconciliationWitness,
                    "authority-delegation-missing-predecessor"
                )],
                roots: vec![],
            })
        );
    }

    #[test]
    fn authority_scope_validates_named_delegation_chain() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-scope-chain"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-scope-chain"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-chain-witness"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-scope-chain"),
            basis: vec![],
        };
        let mut root = authority_delegation(&reconciliation.reconciliation_id.id);
        root.delegation_id = id(IdentityKind::ReconciliationWitness, "authority-delegation-scope-root");
        root.grantee = id(IdentityKind::ReconciliationWitness, "authority-scope-chain-grantor");
        root.subject = reconciliation.reconciliation_id.clone();
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: reconciliation.authority_delegation.clone(),
            grantor: root.grantee.clone(),
            grantee: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: child.delegation_id.clone(),
            basis: vec![],
        };
        assert_eq!(
            scope.validate_against_reconciliation_and_authority_chain(
                &reconciliation,
                &[root.clone(), child],
            ),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![root.delegation_id],
            })
        );
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
    fn reconciliation_basis_must_be_unique() {
        let witness = id(IdentityKind::EvidenceRecord, "reconciliation-basis-duplicate");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-basis-duplicate"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![witness.clone(), witness],
        };
        assert!(reconciliation.validate().is_err());
    }

    #[test]
    fn reconciliation_coverage_basis_must_be_unique() {
        let witness = id(IdentityKind::EvidenceRecord, "coverage-basis-duplicate");
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-basis-duplicate"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-basis-duplicate"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: id(IdentityKind::ReconciliationWitness, "boundary-1"),
            basis: vec![witness.clone(), witness],
        };
        assert!(coverage.validate().is_err());
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
    fn reconciliation_coverage_accepts_reordered_boundary_sets() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition("t3", Some("t1"), disputed, EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") });
        let basis_a = id(IdentityKind::EvidenceRecord, "coverage-set-basis-a");
        let basis_b = id(IdentityKind::ReconciliationWitness, "coverage-set-basis-b");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-coverage-reordered"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2"), id(IdentityKind::ReconciliationWitness, "t3")],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(IdentityKind::ReconciliationWitness, "authority-delegation-1"),
            basis: vec![basis_a.clone(), basis_b.clone()],
        };
        let mut delegation = authority_delegation(&reconciliation.reconciliation_id.id);
        delegation.basis = vec![basis_a.clone(), basis_b.clone()];
        let mut scope = authority_scope(&reconciliation.reconciliation_id.id);
        scope.basis = vec![basis_a.clone(), basis_b.clone()];
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.branch_heads.reverse();
        boundary.authority_delegation = reconciliation.authority_delegation.clone();
        boundary.basis = vec![basis_b.clone(), basis_a.clone()];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-reordered"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2"), id(IdentityKind::ReconciliationWitness, "t3")],
            boundary: boundary.boundary_id.clone(),
            basis: vec![basis_a, basis_b],
        };

        assert!(coverage.validate_against_graph(
            &reconciliation,
            &scope,
            &delegation,
            &boundary,
            &[root, left, right],
        ).is_ok());
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
            &authority_scope(&reconciliation.reconciliation_id.id),
            &authority_delegation(&reconciliation.reconciliation_id.id),
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
    fn reconciliation_coverage_rejects_authority_scope_basis_omission() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let scope_basis = id(IdentityKind::ReconciliationWitness, "authority-scope-basis-only");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-scope-basis-omission"),
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
        let mut scope = authority_scope(&reconciliation.reconciliation_id.id);
        scope.basis = vec![scope_basis];
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-scope-basis-omission"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(coverage.validate_against_graph(
            &reconciliation,
            &scope,
            &authority_delegation(&reconciliation.reconciliation_id.id),
            &boundary,
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
    fn reconciliation_coverage_accepts_authority_scope_basis_at_boundary() {
        let disputed = EvidenceDisposition::Disputed { by: id(IdentityKind::ReconciliationWitness, "w1") };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition("t2", Some("t1"), disputed.clone(), EvidenceDisposition::Active);
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted { by: id(IdentityKind::ReconciliationWitness, "w2") },
        );
        let scope_basis = id(IdentityKind::ReconciliationWitness, "authority-scope-basis-carried");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-scope-basis-carried"),
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
        let mut delegation = authority_delegation(&reconciliation.reconciliation_id.id);
        delegation.basis = vec![scope_basis.clone()];
        let mut scope = authority_scope(&reconciliation.reconciliation_id.id);
        scope.basis = vec![scope_basis.clone()];
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![scope_basis];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-scope-basis-carried"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(coverage.validate_against_graph(
            &reconciliation,
            &scope,
            &delegation,
            &boundary,
            &[root, left, right],
        ).is_ok());
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
