use serde::{Deserialize, Serialize};

use crate::qualification::{
    QualificationOutcome, QualificationStatus, QualificationValidationError,
};

use crate::identity_lineage::{
    ApplicabilityInterval, IdentityKind, IdentityRef, LineageEdge, LineageRelation,
    TemporalConfigurationApplicability,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceEventInterval {
    pub start: u64,
    pub end: Option<u64>,
}

impl EvidenceEventInterval {
    pub fn validate(&self) -> Result<(), String> {
        ApplicabilityInterval {
            start: self.start,
            end: self.end,
        }
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
        if !matches!(
            self.transition_id.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
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
                return Err(
                    "disposition predecessor must be an EvidenceRecord or ReconciliationWitness"
                        .into(),
                );
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
            return Err(
                "disposition basis must be an EvidenceRecord or ReconciliationWitness".into(),
            );
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
    Complete {
        branch_points: Vec<IdentityRef>,
    },
    Unresolved {
        missing: Vec<IdentityRef>,
        branch_points: Vec<IdentityRef>,
    },
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
            if by_id
                .insert(transition.transition_id.clone(), transition)
                .is_some()
            {
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
                            return Err(
                                "disposition predecessor belongs to different evidence".into()
                            );
                        }
                        if parent.to != transition.from {
                            return Err(
                                "transition source state does not match predecessor target state"
                                    .into(),
                            );
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

        let branch_points = children
            .into_iter()
            .filter_map(|(parent, count)| if count > 1 { Some(parent) } else { None })
            .collect();
        if missing.is_empty() {
            Ok(DispositionChainAssessment::Complete { branch_points })
        } else {
            Ok(DispositionChainAssessment::Unresolved {
                missing,
                branch_points,
            })
        }
    }

    pub fn validate_graph_qualified(
        transitions: &[EvidenceDispositionTransition],
    ) -> QualificationOutcome<DispositionChainQualification> {
        Self::validate_graph(transitions)
            .map_err(QualificationValidationError::from)
            .map(DispositionChainAssessment::into_qualification)
    }

    /// Validate adding exactly one new transition to an existing append-only graph.
    ///
    /// The existing graph is validated first, then the candidate is checked for
    /// structural validity and exact-identity uniqueness. The candidate may extend
    /// any existing branch or create a new branch, but it may not replace an
    /// already-addressed transition record. Missing candidate predecessors remain
    /// unresolved through the normal graph assessment.
    pub fn validate_append(
        existing: &[EvidenceDispositionTransition],
        candidate: &EvidenceDispositionTransition,
    ) -> Result<DispositionChainAssessment, String> {
        Self::validate_graph(existing)?;
        candidate.validate()?;

        if existing
            .iter()
            .any(|transition| transition.transition_id == candidate.transition_id)
        {
            return Err("cannot append disposition transition with an existing identity".into());
        }

        let mut combined = existing.to_vec();
        combined.push(candidate.clone());
        Self::validate_graph(&combined)
    }

    pub fn validate_append_qualified(
        existing: &[EvidenceDispositionTransition],
        candidate: &EvidenceDispositionTransition,
    ) -> QualificationOutcome<DispositionChainQualification> {
        Self::validate_append(existing, candidate)
            .map_err(QualificationValidationError::from)
            .map(DispositionChainAssessment::into_qualification)
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
    Complete {
        roots: Vec<IdentityRef>,
    },
    Unresolved {
        missing: Vec<IdentityRef>,
        roots: Vec<IdentityRef>,
    },
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

        let mut by_id: BTreeMap<IdentityRef, Vec<&EvidenceDispositionAuthorityDelegation>> =
            BTreeMap::new();
        for delegation in delegations {
            by_id
                .entry(delegation.delegation_id.clone())
                .or_default()
                .push(delegation);
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

        Self::validate_chain_with_target(target, delegations)
    }

    pub fn validate_chain_qualified(
        delegation_id: &IdentityRef,
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> QualificationOutcome<AuthorityDelegationChainQualification> {
        Self::validate_chain(delegation_id, delegations)
            .map_err(QualificationValidationError::from)
            .map(AuthorityDelegationChainAssessment::into_qualification)
    }

    /// Validate a named delegation chain when the target delegation record is
    /// already available as part of the object currently being validated.
    ///
    /// The target is therefore not itself a DHT dependency of this operation.
    /// The supplied slice contains additional predecessor dependencies. If the
    /// same target identity is also present, it must be structurally identical
    /// to the direct target witness; unrelated records remain outside the
    /// dependency cone.
    pub fn validate_chain_with_target(
        target: &EvidenceDispositionAuthorityDelegation,
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> Result<AuthorityDelegationChainAssessment, String> {
        use std::collections::BTreeMap;

        target.validate()?;

        let mut by_id: BTreeMap<IdentityRef, Vec<&EvidenceDispositionAuthorityDelegation>> =
            BTreeMap::new();
        for delegation in delegations {
            by_id
                .entry(delegation.delegation_id.clone())
                .or_default()
                .push(delegation);
        }

        if let Some(candidates) = by_id.get(&target.delegation_id) {
            if candidates.len() != 1 {
                return Err("duplicate authority delegation identity".into());
            }
            if *candidates[0] != *target {
                return Err(
                    "provided authority delegation target conflicts with supplied dependency"
                        .into(),
                );
            }
        }

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
                            return Err(
                                "authority delegation predecessor subject must match".into()
                            );
                        }
                        if predecessor.grantee != current.grantor {
                            return Err(
                                "authority delegation grantor must match predecessor grantee"
                                    .into(),
                            );
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

    pub fn validate_chain_with_target_qualified(
        target: &EvidenceDispositionAuthorityDelegation,
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> QualificationOutcome<AuthorityDelegationChainQualification> {
        Self::validate_chain_with_target(target, delegations)
            .map_err(QualificationValidationError::from)
            .map(AuthorityDelegationChainAssessment::into_qualification)
    }

    pub fn validate_graph_qualified(
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> QualificationOutcome<AuthorityDelegationChainQualification> {
        Self::validate_graph(delegations)
            .map_err(QualificationValidationError::from)
            .map(AuthorityDelegationChainAssessment::into_qualification)
    }

    pub fn validate_graph(
        delegations: &[EvidenceDispositionAuthorityDelegation],
    ) -> Result<AuthorityDelegationChainAssessment, String> {
        use std::collections::{BTreeMap, BTreeSet};

        let mut by_id = BTreeMap::new();
        for delegation in delegations {
            delegation.validate()?;
            if by_id
                .insert(delegation.delegation_id.clone(), delegation)
                .is_some()
            {
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
                            return Err(
                                "authority delegation predecessor subject must match".into()
                            );
                        }
                        if predecessor.grantee != delegation.grantor {
                            return Err(
                                "authority delegation grantor must match predecessor grantee"
                                    .into(),
                            );
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
            if !matches!(
                identity.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
                return Err(format!(
                    "authority delegation {name} must be an addressable witness"
                ));
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
            if !matches!(
                predecessor.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
                return Err(
                    "authority delegation predecessor must be an addressable witness".into(),
                );
            }
        }
        let mut basis = std::collections::BTreeSet::new();
        for witness in &self.basis {
            witness.validate()?;
            if !matches!(
                witness.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
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
                return Err(
                    "authority delegation basis must include every scope basis witness".into(),
                );
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
        if !matches!(
            self.authority.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
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
            if !matches!(
                witness.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
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
                return Err(
                    "authority scope basis must include every reconciliation basis witness".into(),
                );
            }
        }
        let target = delegations
            .iter()
            .find(|delegation| delegation.delegation_id == self.delegation);

        let assessment =
            EvidenceDispositionAuthorityDelegation::validate_chain(&self.delegation, delegations)?;

        // Preserve structural binding precedence whenever the named target is
        // present. Only an actually absent target is allowed to remain solely
        // an unresolved dependency at this scope boundary.
        if let Some(delegation) = target {
            delegation.validate_against_scope(self)?;
        }

        Ok(assessment)
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
                return Err(
                    "authority scope basis must include every reconciliation basis witness".into(),
                );
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
            return Err(
                "reconciliation authority scope cannot equal reconciliation or authority identity"
                    .into(),
            );
        }
        if self.authority_delegation != self.authority_scope.delegation {
            return Err(
                "reconciliation authority delegation must match authority scope delegation".into(),
            );
        }
        if !matches!(
            self.reconciliation_id.kind,
            IdentityKind::ReconciliationWitness
        ) {
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
            if !matches!(
                head.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
                return Err("reconciliation branch head must be a transition identity".into());
            }
            if !heads.insert(head.clone()) {
                return Err("reconciliation branch heads must be unique".into());
            }
            if head == &self.reconciliation_id || head == &self.evidence {
                return Err(
                    "reconciliation branch head cannot equal reconciliation or evidence identity"
                        .into(),
                );
            }
        }
        let mut basis = std::collections::BTreeSet::new();
        for witness in &self.basis {
            witness.validate()?;
            if !matches!(
                witness.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
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

        let mut by_id =
            std::collections::BTreeMap::<IdentityRef, Vec<&EvidenceDispositionTransition>>::new();
        for transition in transitions {
            transition.validate()?;
            by_id
                .entry(transition.transition_id.clone())
                .or_default()
                .push(transition);
        }
        if by_id.values().any(|candidates| candidates.len() != 1) {
            return Err("duplicate disposition transition identity".into());
        }

        let branch_point = by_id
            .get(&self.branch_point)
            .ok_or_else(|| "reconciliation branch point is missing".to_string())?;
        if branch_point.len() != 1 {
            return Err("duplicate disposition transition identity".into());
        }
        if branch_point[0].evidence != self.evidence {
            return Err("reconciliation branch point belongs to different evidence".into());
        }

        let mut walks = Vec::new();
        let mut missing_named_head = None;

        for head_id in &self.branch_heads {
            if head_id == &self.branch_point {
                return Err("reconciliation branch head cannot equal the branch point".into());
            }

            let candidates = match by_id.get(head_id) {
                None => {
                    missing_named_head.get_or_insert_with(|| head_id.clone());
                    let mut ancestor_ids = std::collections::BTreeSet::new();
                    ancestor_ids.insert(head_id.clone());
                    walks.push((head_id.clone(), ancestor_ids));
                    continue;
                }
                Some(candidates) => candidates,
            };
            if candidates.len() != 1 {
                return Err("duplicate disposition transition identity".into());
            }

            let walk =
                TransitionBoundaryWalk::walk(head_id, &self.branch_point, &self.evidence, &by_id)?;
            if walk.missing.is_some() {
                return Err("reconciliation branch ancestry is unresolved".into());
            }
            walks.push((head_id.clone(), walk.ancestor_ids));
        }

        // Use the same bounded ancestry produced by TransitionBoundaryWalk for
        // the competing-head contradiction check. This keeps legacy
        // reconciliation validation from maintaining a second traversal
        // algorithm with different boundary or missing-dependency semantics.
        for (index, (left_id, left_ancestors)) in walks.iter().enumerate() {
            for (right_id, right_ancestors) in walks.iter().skip(index + 1) {
                if left_ancestors.contains(right_id) || right_ancestors.contains(left_id) {
                    return Err(
                        "reconciliation branch heads must be incomparable descendants".into(),
                    );
                }
            }
        }

        if let Some(head_id) = missing_named_head {
            return Err(format!(
                "reconciliation branch head is missing: {}",
                head_id.id
            ));
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
            return Err(
                "coverage boundary authority scope cannot equal boundary or authority identity"
                    .into(),
            );
        }
        if self.authority_delegation != self.authority_scope.delegation {
            return Err(
                "coverage boundary authority delegation must match authority scope delegation"
                    .into(),
            );
        }
        if self.boundary_id.kind != IdentityKind::ReconciliationWitness {
            return Err("coverage boundary identity must be a ReconciliationWitness".into());
        }
        if self.reconciliation.kind != IdentityKind::ReconciliationWitness {
            return Err("coverage boundary reconciliation must be a ReconciliationWitness".into());
        }
        if self.reconciliation == self.boundary_id || self.reconciliation == self.evidence {
            return Err(
                "coverage boundary reconciliation cannot equal boundary or evidence identity"
                    .into(),
            );
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
        if !matches!(
            self.branch_point.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err("coverage boundary branch point must be a transition identity".into());
        }
        if !matches!(
            self.authority.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err("coverage boundary authority must be an addressable witness".into());
        }
        if self.branch_heads.is_empty() {
            return Err("coverage boundary requires at least one branch head".into());
        }
        let mut heads = std::collections::BTreeSet::new();
        for head in &self.branch_heads {
            head.validate()?;
            if !matches!(
                head.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
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
            if !matches!(
                basis.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
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
    Complete {
        authority_roots: Vec<IdentityRef>,
    },
    Unresolved {
        missing: Vec<IdentityRef>,
        authority_roots: Vec<IdentityRef>,
    },
}

pub type EvidenceDispositionCoverageValidationError = QualificationValidationError;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DispositionChainQualification {
    pub branch_points: Vec<IdentityRef>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityDelegationChainQualification {
    pub roots: Vec<IdentityRef>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceDispositionCoverageQualification {
    pub authority_roots: Vec<IdentityRef>,
}

impl DispositionChainAssessment {
    pub fn into_qualification(self) -> QualificationStatus<DispositionChainQualification> {
        match self {
            Self::Complete { branch_points } => {
                QualificationStatus::Complete(DispositionChainQualification { branch_points })
            }
            Self::Unresolved {
                missing,
                branch_points,
            } => QualificationStatus::unresolved(
                missing,
                DispositionChainQualification { branch_points },
            ),
        }
    }
}

impl AuthorityDelegationChainAssessment {
    pub fn into_qualification(
        self,
    ) -> QualificationStatus<AuthorityDelegationChainQualification> {
        match self {
            Self::Complete { roots } => {
                QualificationStatus::Complete(AuthorityDelegationChainQualification { roots })
            }
            Self::Unresolved { missing, roots } => QualificationStatus::unresolved(
                missing,
                AuthorityDelegationChainQualification { roots },
            ),
        }
    }
}

impl EvidenceDispositionCoverageAssessment {
    pub fn into_qualification(
        self,
    ) -> QualificationStatus<EvidenceDispositionCoverageQualification> {
        match self {
            Self::Complete { authority_roots } => QualificationStatus::Complete(
                EvidenceDispositionCoverageQualification { authority_roots },
            ),
            Self::Unresolved {
                missing,
                authority_roots,
            } => QualificationStatus::unresolved(
                missing,
                EvidenceDispositionCoverageQualification { authority_roots },
            ),
        }
    }
}

struct TransitionBoundaryWalk {
    reachable: Vec<EvidenceDispositionTransition>,
    ancestor_ids: std::collections::BTreeSet<IdentityRef>,
    missing: Option<IdentityRef>,
}

impl TransitionBoundaryWalk {
    fn walk(
        head_id: &IdentityRef,
        branch_point: &IdentityRef,
        evidence: &IdentityRef,
        by_id: &std::collections::BTreeMap<IdentityRef, Vec<&EvidenceDispositionTransition>>,
    ) -> Result<Self, String> {
        let candidates = by_id
            .get(head_id)
            .ok_or_else(|| "reconciliation coverage branch head is missing".to_string())?;
        if candidates.len() != 1 {
            return Err("duplicate disposition transition identity".into());
        }

        let mut cursor = candidates[0];
        let mut seen = std::collections::BTreeSet::new();
        let mut reachable = Vec::new();
        let mut ancestor_ids = std::collections::BTreeSet::new();

        loop {
            if !seen.insert(cursor.transition_id.clone()) {
                return Err("reconciliation coverage graph contains a predecessor cycle".into());
            }
            cursor.validate()?;
            if cursor.evidence != *evidence {
                return Err(
                    "reconciliation coverage transition belongs to different evidence".into(),
                );
            }

            ancestor_ids.insert(cursor.transition_id.clone());
            reachable.push(cursor.clone());

            // The bounded assessment intentionally treats the declared
            // reconciliation branch point as its dependency boundary. Its
            // own predecessor history belongs to an upstream qualification
            // context and is not traversed here.
            if cursor.transition_id == *branch_point {
                return Ok(Self {
                    reachable,
                    ancestor_ids,
                    missing: None,
                });
            }

            let predecessor_id = match &cursor.predecessor {
                None => {
                    return Err(
                        "reconciliation coverage branch head does not descend from branch point"
                            .into(),
                    );
                }
                Some(predecessor_id) => predecessor_id,
            };

            // Retain the referenced predecessor identity even when its
            // record is unavailable. This lets bounded contradiction checks
            // recognize that a present branch head directly or indirectly
            // names another declared branch head as an ancestor.
            ancestor_ids.insert(predecessor_id.clone());

            let predecessor = match by_id.get(predecessor_id) {
                None => {
                    return Ok(Self {
                        reachable,
                        ancestor_ids,
                        missing: Some(predecessor_id.clone()),
                    });
                }
                Some(candidates) if candidates.len() != 1 => {
                    return Err("duplicate disposition transition identity".into());
                }
                Some(candidates) => candidates[0],
            };

            predecessor.validate()?;
            if predecessor.evidence != cursor.evidence {
                return Err(
                    "reconciliation coverage predecessor belongs to different evidence".into(),
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
    }
}

impl EvidenceDispositionReconciliationCoverage {
    fn collect_named_transition_dependencies(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        transitions: &[EvidenceDispositionTransition],
    ) -> Result<(Vec<EvidenceDispositionTransition>, Vec<IdentityRef>), String> {
        use std::collections::{BTreeMap, BTreeSet};

        let mut by_id: BTreeMap<IdentityRef, Vec<&EvidenceDispositionTransition>> = BTreeMap::new();
        for transition in transitions {
            by_id
                .entry(transition.transition_id.clone())
                .or_default()
                .push(transition);
        }

        let mut missing = BTreeSet::new();
        let mut reachable = BTreeMap::<IdentityRef, EvidenceDispositionTransition>::new();
        let mut walks = Vec::new();

        match by_id.get(&reconciliation.branch_point) {
            None => {
                missing.insert(reconciliation.branch_point.clone());
            }
            Some(candidates) if candidates.len() != 1 => {
                return Err("duplicate disposition transition identity".into());
            }
            Some(candidates) => {
                let branch_point = candidates[0];
                branch_point.validate()?;
                if branch_point.evidence != reconciliation.evidence {
                    return Err(
                        "reconciliation coverage branch point belongs to different evidence".into(),
                    );
                }
                reachable.insert(branch_point.transition_id.clone(), branch_point.clone());
            }
        }

        // The coverage boundary, not merely the reconciliation minimum set,
        // defines the complete bounded transition dependency cone. Additional
        // explicitly covered heads are valid and must be resolved as part of
        // the same deterministic walk.
        for head_id in &self.covered_branch_heads {
            if !by_id.contains_key(head_id) {
                missing.insert(head_id.clone());
                let mut ancestor_ids = std::collections::BTreeSet::new();
                ancestor_ids.insert(head_id.clone());
                walks.push((head_id.clone(), ancestor_ids));
                continue;
            }

            let walk = TransitionBoundaryWalk::walk(
                head_id,
                &reconciliation.branch_point,
                &reconciliation.evidence,
                &by_id,
            )?;

            for transition in &walk.reachable {
                reachable
                    .entry(transition.transition_id.clone())
                    .or_insert_with(|| transition.clone());
            }
            if let Some(dependency) = &walk.missing {
                missing.insert(dependency.clone());
            }
            walks.push((head_id.clone(), walk.ancestor_ids));
        }

        // Because every named head now comes from the same bounded walk,
        // nested-head detection cannot accidentally traverse past the
        // declared branch point or use different missing-dependency rules.
        for (index, (left_id, left_ancestors)) in walks.iter().enumerate() {
            for (right_id, right_ancestors) in walks.iter().skip(index + 1) {
                if left_ancestors.contains(right_id) || right_ancestors.contains(left_id) {
                    return Err(
                        "reconciliation branch heads must be incomparable descendants".into(),
                    );
                }
            }
        }

        Ok((
            reachable.into_values().collect(),
            missing.into_iter().collect(),
        ))
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
    ) -> Result<EvidenceDispositionCoverageAssessment, EvidenceDispositionCoverageValidationError>
    {
        self.validate()?;
        reconciliation.validate()?;
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
            return Err(
                "reconciliation coverage heads must match the coverage boundary set".into(),
            );
        }
        let coverage_basis: std::collections::BTreeSet<_> = self.basis.iter().cloned().collect();
        let boundary_basis: std::collections::BTreeSet<_> =
            boundary.basis.iter().cloned().collect();
        if coverage_basis != boundary_basis {
            return Err(
                "reconciliation coverage basis must match the coverage boundary set".into(),
            );
        }
        if boundary.reconciliation != reconciliation.reconciliation_id {
            return Err("coverage boundary references a different reconciliation".into());
        }
        if boundary.authority != reconciliation.authority {
            return Err("coverage boundary authority must match reconciliation authority".into());
        }
        if boundary.authority_scope != reconciliation.authority_scope {
            return Err(
                "coverage boundary authority scope must match reconciliation authority scope"
                    .into(),
            );
        }
        if boundary.authority_delegation != reconciliation.authority_delegation {
            return Err("coverage boundary authority delegation must match reconciliation authority delegation".into());
        }
        if boundary.authority_delegation != authority_delegation.delegation_id {
            return Err(
                "coverage boundary authority delegation does not match supplied delegation witness"
                    .into(),
            );
        }
        if boundary.authority_scope != authority_scope.scope_id {
            return Err(
                "coverage boundary authority scope does not match supplied scope witness".into(),
            );
        }
        for basis in &reconciliation.basis {
            if !boundary.basis.contains(basis) {
                return Err(
                    "coverage boundary basis must include every reconciliation basis witness"
                        .into(),
                );
            }
        }
        for basis in &authority_scope.basis {
            if !boundary.basis.contains(basis) {
                return Err(
                    "coverage boundary basis must include every authority scope basis witness"
                        .into(),
                );
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
                return Err(
                    "reconciliation branch head is outside the declared coverage set".into(),
                );
            }
        }

        let (named_transitions, missing_transitions) =
            self.collect_named_transition_dependencies(reconciliation, transitions)?;

        let authority_assessment =
            EvidenceDispositionAuthorityDelegation::validate_chain_with_target(
                authority_delegation,
                delegations,
            )?;

        let has_missing_transitions = !missing_transitions.is_empty();
        let combined_qualification = authority_assessment
            .into_qualification()
            .require_missing(missing_transitions);

        // Preserve contradiction precedence: once the transition dependency
        // cone is fully present, validate its bounded structural relationships
        // before returning an unresolved authority dependency.
        let partial = match combined_qualification {
            QualificationStatus::Unresolved {
                missing,
                partial,
            } if !missing.is_empty() => {
                if has_missing_transitions {
                    return Ok(EvidenceDispositionCoverageAssessment::Unresolved {
                        missing,
                        authority_roots: partial.authority_roots,
                    });
                }

                self.validate_against_graph(
                    reconciliation,
                    authority_scope,
                    authority_delegation,
                    boundary,
                    &named_transitions,
                )?;

                return Ok(EvidenceDispositionCoverageAssessment::Unresolved {
                    missing,
                    authority_roots: partial.authority_roots,
                });
            }
            QualificationStatus::Complete(partial) => partial,
            QualificationStatus::Unresolved { .. } => {
                unreachable!("unresolved qualification must carry at least one missing dependency")
            }
        };

        self.validate_against_graph(
            reconciliation,
            authority_scope,
            authority_delegation,
            boundary,
            &named_transitions,
        )?;

        Ok(EvidenceDispositionCoverageAssessment::Complete {
            authority_roots: partial.authority_roots,
        })
    }

    /// Validate bounded coverage and the exact authority delegation chain it relies on.
    ///
    /// The delegation graph is scoped to the named delegation identity;
    /// unrelated delegation records do not become dependencies of this check.
    pub fn validate_against_graph_and_authority_chain_qualified(
        &self,
        reconciliation: &EvidenceDispositionReconciliation,
        authority_scope: &EvidenceDispositionAuthorityScope,
        authority_delegation: &EvidenceDispositionAuthorityDelegation,
        delegations: &[EvidenceDispositionAuthorityDelegation],
        boundary: &EvidenceDispositionCoverageBoundary,
        transitions: &[EvidenceDispositionTransition],
    ) -> QualificationOutcome<EvidenceDispositionCoverageQualification> {
        self.validate_against_graph_and_authority_chain_assessment(
            reconciliation,
            authority_scope,
            authority_delegation,
            delegations,
            boundary,
            transitions,
        )
        .map_err(QualificationValidationError::from)
        .map(EvidenceDispositionCoverageAssessment::into_qualification)
    }

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
        EvidenceDispositionAuthorityDelegation::validate_chain_with_target(
            authority_delegation,
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
        if !matches!(
            self.branch_point.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err(
                "reconciliation coverage branch point must be a transition identity".into(),
            );
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
            if !matches!(
                head.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
                return Err(
                    "reconciliation coverage branch head must be a transition identity".into(),
                );
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
            if !matches!(
                witness.kind,
                IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
            ) {
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
        reconciliation.validate_against_graph(
            authority_scope,
            authority_delegation,
            transitions,
        )?;
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
            return Err(
                "reconciliation coverage heads must match the coverage boundary set".into(),
            );
        }
        let coverage_basis: std::collections::BTreeSet<_> = self.basis.iter().cloned().collect();
        let boundary_basis: std::collections::BTreeSet<_> =
            boundary.basis.iter().cloned().collect();
        if coverage_basis != boundary_basis {
            return Err(
                "reconciliation coverage basis must match the coverage boundary set".into(),
            );
        }
        if boundary.reconciliation != reconciliation.reconciliation_id {
            return Err("coverage boundary references a different reconciliation".into());
        }
        if boundary.authority != reconciliation.authority {
            return Err("coverage boundary authority must match reconciliation authority".into());
        }
        if boundary.authority_scope != reconciliation.authority_scope {
            return Err(
                "coverage boundary authority scope must match reconciliation authority scope"
                    .into(),
            );
        }
        if boundary.authority_delegation != reconciliation.authority_delegation {
            return Err("coverage boundary authority delegation must match reconciliation authority delegation".into());
        }
        if boundary.authority_delegation != authority_delegation.delegation_id {
            return Err(
                "coverage boundary authority delegation does not match supplied delegation witness"
                    .into(),
            );
        }
        if boundary.authority_scope != authority_scope.scope_id {
            return Err(
                "coverage boundary authority scope does not match supplied scope witness".into(),
            );
        }
        for basis in &reconciliation.basis {
            if !boundary.basis.contains(basis) {
                return Err(
                    "coverage boundary basis must include every reconciliation basis witness"
                        .into(),
                );
            }
        }
        for basis in &authority_scope.basis {
            if !boundary.basis.contains(basis) {
                return Err(
                    "coverage boundary basis must include every authority scope basis witness"
                        .into(),
                );
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
                return Err(
                    "reconciliation branch head is outside the declared coverage set".into(),
                );
            }
        }
        let mut by_id =
            std::collections::BTreeMap::<IdentityRef, Vec<&EvidenceDispositionTransition>>::new();
        for transition in transitions {
            by_id
                .entry(transition.transition_id.clone())
                .or_default()
                .push(transition);
        }

        let mut walks = Vec::new();
        let mut missing_named_head = None;
        for head_id in &self.covered_branch_heads {
            if head_id == &self.branch_point {
                return Err(
                    "reconciliation coverage branch head cannot equal the branch point".into(),
                );
            }

            let candidates = match by_id.get(head_id) {
                None => {
                    missing_named_head.get_or_insert_with(|| head_id.clone());
                    let mut ancestor_ids = std::collections::BTreeSet::new();
                    ancestor_ids.insert(head_id.clone());
                    walks.push((head_id.clone(), ancestor_ids));
                    continue;
                }
                Some(candidates) => candidates,
            };
            if candidates.len() != 1 {
                return Err("duplicate disposition transition identity".into());
            }
            if candidates[0].evidence != reconciliation.evidence {
                return Err(
                    "reconciliation coverage branch head belongs to different evidence".into(),
                );
            }

            let walk = TransitionBoundaryWalk::walk(
                head_id,
                &reconciliation.branch_point,
                &reconciliation.evidence,
                &by_id,
            )?;
            if walk.missing.is_some() {
                return Err("reconciliation coverage ancestry is unresolved".into());
            }
            walks.push((head_id.clone(), walk.ancestor_ids));
        }

        // Covered-head incomparability uses the same bounded ancestry that
        // admitted each head into the dependency cone. This prevents the
        // legacy coverage validator from maintaining a second traversal path
        // that could disagree with the canonical composed assessment.
        for (index, (left_id, left_ancestors)) in walks.iter().enumerate() {
            for (right_id, right_ancestors) in walks.iter().skip(index + 1) {
                if left_ancestors.contains(right_id) || right_ancestors.contains(left_id) {
                    return Err(
                        "reconciliation coverage branch heads must be incomparable descendants"
                            .into(),
                    );
                }
            }
        }

        if let Some(head_id) = missing_named_head {
            return Err(format!(
                "reconciliation coverage branch head is missing: {}",
                head_id.id
            ));
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
        if !matches!(
            witness.kind,
            IdentityKind::EvidenceRecord | IdentityKind::ReconciliationWitness
        ) {
            return Err(
                "evidence disposition witness must be an EvidenceRecord or ReconciliationWitness"
                    .into(),
            );
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
        self.event_interval
            .validate()?
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
            (Some(_), None) => Err(
                "open-ended evidence effectivity requires open-ended configuration applicability"
                    .into(),
            ),
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
        IdentityRef {
            kind,
            namespace: "mobility".into(),
            id: value.into(),
        }
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
                interval: ApplicabilityInterval {
                    start: 90,
                    end: Some(300),
                },
            },
            event_interval: EvidenceEventInterval {
                start: 100,
                end: Some(110),
            },
            effectivity_interval: ApplicabilityInterval {
                start: 120,
                end: Some(200),
            },
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
        value.configuration_applicability.applicability.source =
            id(IdentityKind::ConfigurationRevision, "cfg-2");
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
        value.effectivity_interval = ApplicabilityInterval {
            start: 1_000,
            end: Some(2_000),
        };
        assert!(value.validate().is_ok());
        assert_eq!(value.event_interval.start, 100);
    }

    #[test]
    fn effectivity_exactly_matches_configuration_interval() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval {
            start: 90,
            end: Some(300),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_strictly_inside_configuration_interval_is_accepted() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval {
            start: 100,
            end: Some(299),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_starting_at_configuration_end_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval {
            start: 300,
            end: Some(301),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn effectivity_starting_before_configuration_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval {
            start: 89,
            end: Some(200),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn effectivity_ending_at_configuration_end_is_accepted() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval {
            start: 150,
            end: Some(300),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn effectivity_extending_beyond_configuration_is_rejected() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval {
            start: 150,
            end: Some(301),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn finite_configuration_cannot_contain_open_ended_effectivity() {
        let mut value = evidence_case();
        value.effectivity_interval = ApplicabilityInterval {
            start: 150,
            end: None,
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn open_ended_configuration_can_contain_open_ended_effectivity() {
        let mut value = evidence_case();
        value.configuration_applicability.interval.end = None;
        value.effectivity_interval = ApplicabilityInterval {
            start: 150,
            end: None,
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn event_time_can_be_outside_effectivity_without_bypassing_containment() {
        let mut value = evidence_case();
        value.event_interval = EvidenceEventInterval {
            start: 10,
            end: Some(20),
        };
        value.effectivity_interval = ApplicabilityInterval {
            start: 150,
            end: Some(200),
        };
        assert!(value.validate().is_ok());
    }

    #[test]
    fn temporal_applicability_witness_mismatch_is_rejected() {
        let mut value = evidence_case();
        value.configuration_applicability.interval.start = 100;
        value.configuration_applicability.interval.end = Some(200);
        value.effectivity_interval = ApplicabilityInterval {
            start: 150,
            end: Some(250),
        };
        assert!(value.validate().is_err());
    }

    #[test]
    fn disputed_disposition_does_not_change_temporal_scope() {
        let mut value = evidence_case();
        value.disposition = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "witness-1"),
        };
        assert!(value.validate().is_ok());
        assert_eq!(
            value.effectivity_interval,
            ApplicabilityInterval {
                start: 120,
                end: Some(200)
            }
        );
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

    #[test]
    fn append_qualified_preserves_unresolved_status() {
        let candidate = graph_transition(
            "qualified-successor",
            Some("qualified-missing"),
            EvidenceDisposition::Active,
            EvidenceDisposition::Unresolved,
        );

        assert_eq!(
            EvidenceDispositionTransition::validate_append_qualified(&[], &candidate).unwrap(),
            QualificationStatus::Unresolved {
                missing: vec![id(
                    IdentityKind::ReconciliationWitness,
                    "qualified-missing",
                )],
                partial: DispositionChainQualification {
                    branch_points: vec![],
                },
            }
        );
    }

    #[test]
    fn append_qualified_surfaces_identity_reuse_as_structural() {
        let genesis = graph_transition(
            "qualified-reuse",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "dispute"),
            },
        );

        let error =
            EvidenceDispositionTransition::validate_append_qualified(&[genesis.clone()], &genesis)
                .unwrap_err();

        assert_eq!(
            error,
            QualificationValidationError::Structural {
                reason: "cannot append disposition transition with an existing identity".into(),
            }
        );
    }

    #[test]
    fn authority_target_qualified_preserves_missing_target_as_unresolved() {
        let target = id(
            IdentityKind::ReconciliationWitness,
            "qualified-missing-authority",
        );

        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_chain_qualified(&target, &[]).unwrap(),
            QualificationStatus::Unresolved {
                missing: vec![target],
                partial: AuthorityDelegationChainQualification { roots: vec![] },
            }
        );
    }

    #[test]
    fn append_accepts_a_new_successor_of_an_existing_transition() {
        let genesis = graph_transition(
            "genesis",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "dispute"),
            },
        );
        let successor = graph_transition(
            "successor",
            Some("genesis"),
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "dispute"),
            },
            EvidenceDisposition::Active,
        );

        let assessment =
            EvidenceDispositionTransition::validate_append(&[genesis], &successor).unwrap();
        assert_eq!(
            assessment,
            DispositionChainAssessment::Complete {
                branch_points: vec![],
            }
        );
    }

    #[test]
    fn append_rejects_rewriting_an_existing_transition_identity() {
        let genesis = graph_transition(
            "genesis",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "dispute"),
            },
        );
        let replacement = graph_transition(
            "genesis",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Unresolved,
        );

        let error =
            EvidenceDispositionTransition::validate_append(&[genesis], &replacement).unwrap_err();
        assert_eq!(
            error,
            "cannot append disposition transition with an existing identity"
        );
    }

    #[test]
    fn append_can_fill_a_previously_missing_predecessor() {
        let child = graph_transition(
            "successor",
            Some("missing"),
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "dispute"),
            },
            EvidenceDisposition::Active,
        );
        let predecessor = graph_transition(
            "missing",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "dispute"),
            },
        );

        let assessment =
            EvidenceDispositionTransition::validate_append(&[child], &predecessor).unwrap();
        assert_eq!(
            assessment,
            DispositionChainAssessment::Complete {
                branch_points: vec![],
            }
        );
    }

    #[test]
    fn append_preserves_unresolved_missing_predecessor_status() {
        let candidate = graph_transition(
            "successor",
            Some("missing"),
            EvidenceDisposition::Active,
            EvidenceDisposition::Unresolved,
        );

        let assessment = EvidenceDispositionTransition::validate_append(&[], &candidate).unwrap();
        assert_eq!(
            assessment,
            DispositionChainAssessment::Unresolved {
                missing: vec![id(IdentityKind::ReconciliationWitness, "missing")],
                branch_points: vec![],
            }
        );
    }

    #[test]
    fn transition_graph_exposes_shared_qualification_status() {
        let genesis = graph_transition(
            "shared-transition",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "dispute"),
            },
        );

        assert_eq!(
            EvidenceDispositionTransition::validate_graph_qualified(&[genesis]).unwrap(),
            QualificationStatus::Complete(DispositionChainQualification {
                branch_points: vec![],
            })
        );
    }

    #[test]
    fn authority_chain_exposes_shared_qualification_status() {
        let root = authority_delegation("shared-authority");

        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_chain_with_target_qualified(
                &root,
                &[root.clone()],
            )
            .unwrap(),
            QualificationStatus::Complete(AuthorityDelegationChainQualification {
                roots: vec![root.delegation_id],
            })
        );
    }

    #[test]
    fn authority_graph_exposes_shared_qualification_status() {
        let root = authority_delegation("shared-graph-authority");

        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_graph_qualified(&[root.clone()])
                .unwrap(),
            QualificationStatus::Complete(AuthorityDelegationChainQualification {
                roots: vec![root.delegation_id],
            })
        );
    }

    #[test]
    fn coverage_assessment_exposes_shared_qualification_status() {
        let missing = id(
            IdentityKind::ReconciliationWitness,
            "shared-coverage-missing",
        );

        let status = EvidenceDispositionCoverageAssessment::Unresolved {
            missing: vec![missing.clone()],
            authority_roots: vec![],
        }
        .into_qualification();

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![missing],
                partial: EvidenceDispositionCoverageQualification {
                    authority_roots: vec![],
                },
            }
        );
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
        let t = graph_transition(
            "genesis",
            None,
            EvidenceDisposition::Unresolved,
            EvidenceDisposition::Active,
        );
        assert!(t.validate().is_err());
    }

    #[test]
    fn transition_rejects_terminal_retraction_reopening() {
        let t = graph_transition(
            "later",
            Some("earlier"),
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "r1"),
            },
            EvidenceDisposition::Active,
        );
        assert!(t.validate().is_err());
    }

    #[test]
    fn transition_rejects_terminal_supersession_reopening() {
        let t = graph_transition(
            "later",
            Some("earlier"),
            EvidenceDisposition::Superseded {
                by: id(IdentityKind::EvidenceRecord, "replacement"),
            },
            EvidenceDisposition::Active,
        );
        assert!(t.validate().is_err());
    }

    #[test]
    fn transition_requires_typed_basis() {
        let mut t = graph_transition(
            "t1",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "w1"),
            },
        );
        t.basis = id(IdentityKind::ConfigurationRevision, "cfg");
        assert!(t.validate().is_err());
    }

    #[test]
    fn graph_validates_predecessor_state_continuity() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let child = graph_transition("t2", Some("t1"), disputed, EvidenceDisposition::Active);
        assert_eq!(
            EvidenceDispositionTransition::validate_graph(&[root, child]),
            Ok(DispositionChainAssessment::Complete {
                branch_points: vec![]
            })
        );
    }

    #[test]
    fn graph_reports_missing_predecessor_as_unresolved() {
        let child = graph_transition(
            "t2",
            Some("missing"),
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "w1"),
            },
        );
        assert_eq!(
            EvidenceDispositionTransition::validate_graph(&[child]),
            Ok(DispositionChainAssessment::Unresolved {
                missing: vec![id(IdentityKind::ReconciliationWitness, "missing")],
                branch_points: vec![],
            })
        );
    }

    #[test]
    fn graph_rejects_predecessor_state_mismatch() {
        let root = graph_transition(
            "t1",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "w1"),
            },
        );
        let child = graph_transition(
            "t2",
            Some("t1"),
            EvidenceDisposition::Active,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        assert!(EvidenceDispositionTransition::validate_graph(&[root, child]).is_err());
    }

    #[test]
    fn graph_preserves_competing_branches_explicitly() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        assert_eq!(
            EvidenceDispositionTransition::validate_graph(&[root, left, right]),
            Ok(DispositionChainAssessment::Complete {
                branch_points: vec![id(IdentityKind::ReconciliationWitness, "t1")],
            })
        );
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
        let one = graph_transition(
            "t1",
            None,
            EvidenceDisposition::Active,
            EvidenceDisposition::Disputed {
                by: id(IdentityKind::ReconciliationWitness, "w1"),
            },
        );
        assert!(EvidenceDispositionTransition::validate_graph(&[one.clone(), one]).is_err());
    }

    #[test]
    fn authority_delegation_graph_requires_chain_continuity() {
        let root = authority_delegation("reconcile-delegation-chain");
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-child",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-target-valid-child",
            ),
            grantor: root.grantee.clone(),
            grantee: id(
                IdentityKind::ReconciliationWitness,
                "authority-target-valid-child",
            ),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: vec![],
        };
        let unrelated = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-unrelated-invalid",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-target-missing",
            ),
            grantor: id(
                IdentityKind::ReconciliationWitness,
                "authority-grantor-target",
            ),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-target"),
            subject: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-target-missing",
            ),
            predecessor: Some(id(
                IdentityKind::ReconciliationWitness,
                "authority-target-missing",
            )),
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
    fn reconciliation_coverage_assessment_rejects_invalid_authority_chain_before_unresolved() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let left = graph_transition(
            "t2",
            Some("missing-transition-parent-invalid-authority"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("missing-transition-parent-invalid-authority"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-invalid-authority-precedence",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(
                IdentityKind::ReconciliationWitness,
                "missing-transition-parent-invalid-authority",
            ),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(
                IdentityKind::ReconciliationWitness,
                "authority-invalid-precedence",
            ),
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-invalid-precedence",
            ),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-invalid-precedence-child",
            ),
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: reconciliation.authority_delegation.clone(),
            basis: vec![],
        };
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: reconciliation.authority_delegation.clone(),
            grantor: id(
                IdentityKind::ReconciliationWitness,
                "authority-grantor-invalid-precedence",
            ),
            grantee: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            predecessor: Some(id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-invalid-precedence-root",
            )),
            basis: vec![],
        };
        let invalid_root = EvidenceDispositionAuthorityDelegation {
            delegation_id: child.predecessor.clone().unwrap(),
            grantor: id(
                IdentityKind::ReconciliationWitness,
                "authority-root-grantor-invalid-precedence",
            ),
            grantee: child.grantor.clone(),
            subject: id(
                IdentityKind::ReconciliationWitness,
                "different-authority-subject",
            ),
            predecessor: None,
            basis: vec![],
        };
        let boundary = EvidenceDispositionCoverageBoundary {
            boundary_id: id(
                IdentityKind::ReconciliationWitness,
                "boundary-invalid-authority-precedence",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            evidence: reconciliation.evidence.clone(),
            branch_point: reconciliation.branch_point.clone(),
            branch_heads: reconciliation.branch_heads.clone(),
            authority: reconciliation.authority.clone(),
            authority_scope: reconciliation.authority_scope.clone(),
            authority_delegation: reconciliation.authority_delegation.clone(),
            basis: vec![],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-invalid-authority-precedence",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };

        assert!(
            coverage
                .validate_against_graph_and_authority_chain_assessment(
                    &reconciliation,
                    &scope,
                    &child,
                    &[invalid_root, child],
                    &boundary,
                    &[left, right],
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_assessment_aggregates_missing_transition_and_authority_dependencies()
    {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let missing_transition = id(
            IdentityKind::ReconciliationWitness,
            "missing-transition-and-authority",
        );
        let left = graph_transition(
            "t2",
            Some("missing-transition-and-authority"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("missing-transition-and-authority"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let missing_delegation = id(
            IdentityKind::ReconciliationWitness,
            "missing-authority-dependency",
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-multiple-missing",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: missing_transition.clone(),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(
                IdentityKind::ReconciliationWitness,
                "authority-multiple-missing",
            ),
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-multiple-missing",
            ),
            authority_delegation: missing_delegation.clone(),
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: reconciliation.authority_delegation.clone(),
            basis: vec![],
        };
        let delegation = EvidenceDispositionAuthorityDelegation {
            delegation_id: missing_delegation.clone(),
            grantor: id(
                IdentityKind::ReconciliationWitness,
                "authority-grantor-multiple-missing",
            ),
            grantee: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            predecessor: None,
            basis: vec![],
        };
        let boundary = EvidenceDispositionCoverageBoundary {
            boundary_id: id(
                IdentityKind::ReconciliationWitness,
                "boundary-multiple-missing",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            evidence: reconciliation.evidence.clone(),
            branch_point: reconciliation.branch_point.clone(),
            branch_heads: reconciliation.branch_heads.clone(),
            authority: reconciliation.authority.clone(),
            authority_scope: reconciliation.authority_scope.clone(),
            authority_delegation: reconciliation.authority_delegation.clone(),
            basis: vec![],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-multiple-missing",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };

        let mut expected_missing = vec![missing_delegation, missing_transition];
        expected_missing.sort_by(|a, b| (&a.namespace, &a.id).cmp(&(&b.namespace, &b.id)));

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain_assessment(
                &reconciliation,
                &scope,
                &delegation,
                &[],
                &boundary,
                &[left, right],
            ),
            Ok(EvidenceDispositionCoverageAssessment::Unresolved {
                missing: expected_missing.clone(),
                authority_roots: vec![],
            })
        );

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain_qualified(
                &reconciliation,
                &scope,
                &delegation,
                &[],
                &boundary,
                &[left, right],
            ),
            Ok(QualificationStatus::Unresolved {
                missing: expected_missing,
                partial: EvidenceDispositionCoverageQualification {
                    authority_roots: vec![],
                },
            })
        );
    }

    #[test]
    fn authority_delegation_direct_target_conflict_is_rejected() {
        let target = authority_delegation("reconcile-direct-target-conflict");
        let mut conflicting = target.clone();
        conflicting.grantee = id(
            IdentityKind::ReconciliationWitness,
            "different-direct-target-grantee",
        );

        assert!(
            EvidenceDispositionAuthorityDelegation::validate_chain_with_target(
                &target,
                &[conflicting],
            )
            .is_err()
        );
    }

    #[test]
    fn authority_delegation_graph_requires_basis_continuity() {
        let mut root = authority_delegation("reconcile-delegation-basis");
        let inherited = id(IdentityKind::EvidenceRecord, "delegation-root-basis");
        root.basis = vec![inherited.clone()];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-child",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-monotonic-child",
            ),
            grantor: root.grantee.clone(),
            grantee: id(
                IdentityKind::ReconciliationWitness,
                "authority-monotonic-child",
            ),
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
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-monotonic",
            ),
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
        let root_transition =
            graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
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
    fn reconciliation_coverage_assessment_rejects_structural_mismatch_before_unresolved() {
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
                "reconcile-transition-missing-contradiction",
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
        boundary.authority = id(
            IdentityKind::ReconciliationWitness,
            "contradictory-authority",
        );
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-transition-missing-contradiction",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };

        assert!(
            coverage
                .validate_against_graph_and_authority_chain_assessment(
                    &reconciliation,
                    &scope,
                    &delegation,
                    &[delegation],
                    &boundary,
                    &[left, right],
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_assessment_surfaces_structural_error_as_typed_error() {
        let witness = id(IdentityKind::EvidenceRecord, "typed-structural-error-basis");
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "typed-structural-error-coverage",
            ),
            reconciliation: id(
                IdentityKind::ReconciliationWitness,
                "typed-structural-error-reconciliation",
            ),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: id(
                IdentityKind::ReconciliationWitness,
                "enumeration-boundary-1",
            ),
            basis: vec![witness.clone(), witness],
        };
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: coverage.reconciliation.clone(),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: coverage.branch_point.clone(),
            branch_heads: coverage.covered_branch_heads.clone(),
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
        let boundary = coverage_boundary();

        let result = coverage.validate_against_graph_and_authority_chain_assessment(
            &reconciliation,
            &scope,
            &delegation,
            &[delegation.clone()],
            &boundary,
            &[],
        );

        match result {
            Err(EvidenceDispositionCoverageValidationError::Structural { reason }) => {
                assert!(reason.contains("reconciliation coverage basis must be unique"));
            }
            other => panic!("expected typed structural error, got {other:?}"),
        }
    }

    #[test]
    fn reconciliation_coverage_assessment_ignores_unrelated_invalid_transition() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root_transition = graph_transition(
            "isolation-t1",
            None,
            EvidenceDisposition::Active,
            disputed.clone(),
        );
        let left = graph_transition(
            "isolation-t2",
            Some("isolation-t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "isolation-t3",
            Some("isolation-t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-transition-isolation",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: root_transition.transition_id.clone(),
            branch_heads: vec![left.transition_id.clone(), right.transition_id.clone()],
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
                "coverage-transition-isolation",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        let unrelated = EvidenceDispositionTransition {
            transition_id: id(
                IdentityKind::ReconciliationWitness,
                "unrelated-invalid-transition",
            ),
            evidence: id(IdentityKind::DesignRevision, "unrelated-design"),
            predecessor: None,
            from: EvidenceDisposition::Active,
            to: EvidenceDisposition::Unresolved,
            basis: id(
                IdentityKind::ReconciliationWitness,
                "unrelated-invalid-transition-basis",
            ),
        };

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain_assessment(
                &reconciliation,
                &scope,
                &delegation,
                &[delegation],
                &boundary,
                &[root_transition, left, right, unrelated],
            ),
            Ok(EvidenceDispositionCoverageAssessment::Complete {
                authority_roots: vec![delegation.delegation_id],
            })
        );
    }

    #[test]
    fn reconciliation_coverage_assessment_stops_at_declared_branch_point() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "bounded-w1"),
        };
        let branch_point = graph_transition(
            "bounded-t1",
            Some("missing-ancestor-before-boundary"),
            EvidenceDisposition::Active,
            disputed.clone(),
        );
        let left = graph_transition(
            "bounded-t2",
            Some("bounded-t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "bounded-t3",
            Some("bounded-t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "bounded-w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-bounded-branch-point",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: branch_point.transition_id.clone(),
            branch_heads: vec![left.transition_id.clone(), right.transition_id.clone()],
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
                "coverage-bounded-branch-point",
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
                &[branch_point, left, right],
            ),
            Ok(EvidenceDispositionCoverageAssessment::Complete {
                authority_roots: vec![delegation.delegation_id],
            })
        );
    }

    #[test]
    fn reconciliation_coverage_assessment_rejects_present_descendant_of_missing_named_head() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "missing-head-w1"),
        };
        let missing_head = id(IdentityKind::ReconciliationWitness, "missing-named-head");
        let descendant_head = graph_transition(
            "present-descendant-of-missing-head",
            Some("missing-named-head"),
            disputed,
            EvidenceDisposition::Active,
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-present-descendant-of-missing-head",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(
                IdentityKind::ReconciliationWitness,
                "missing-branch-point-for-named-head",
            ),
            branch_heads: vec![missing_head.clone(), descendant_head.transition_id.clone()],
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
        boundary.branch_point = reconciliation.branch_point.clone();
        boundary.branch_heads = reconciliation.branch_heads.clone();
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-present-descendant-of-missing-head",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };

        let result = coverage.validate_against_graph_and_authority_chain_assessment(
            &reconciliation,
            &scope,
            &delegation,
            &[delegation],
            &boundary,
            &[descendant_head],
        );

        assert!(matches!(
            result,
            Err(
                EvidenceDispositionCoverageValidationError::Structural { reason }
            ) if reason == "reconciliation branch heads must be incomparable descendants"
        ));
    }

    #[test]
    fn reconciliation_coverage_assessment_rejects_known_nested_heads_even_with_missing_boundary() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "nested-w1"),
        };
        let ancestor_head = graph_transition(
            "nested-t2",
            Some("missing-boundary"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let descendant_head = graph_transition(
            "nested-t3",
            Some("nested-t2"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "nested-w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-known-nested-heads",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "missing-boundary"),
            branch_heads: vec![
                ancestor_head.transition_id.clone(),
                descendant_head.transition_id.clone(),
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
        boundary.branch_point = reconciliation.branch_point.clone();
        boundary.branch_heads = reconciliation.branch_heads.clone();
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-known-nested-heads",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };

        assert!(
            coverage
                .validate_against_graph_and_authority_chain_assessment(
                    &reconciliation,
                    &scope,
                    &delegation,
                    &[delegation],
                    &boundary,
                    &[ancestor_head, descendant_head],
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_assessment_is_invariant_to_transition_order() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root_transition = graph_transition(
            "order-t1",
            None,
            EvidenceDisposition::Active,
            disputed.clone(),
        );
        let left = graph_transition(
            "order-t2",
            Some("order-t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "order-t3",
            Some("order-t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-transition-order",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: root_transition.transition_id.clone(),
            branch_heads: vec![left.transition_id.clone(), right.transition_id.clone()],
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
                "coverage-transition-order",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        let forward = vec![root_transition.clone(), left.clone(), right.clone()];
        let reverse = vec![right, left, root_transition];

        let forward_result = coverage
            .validate_against_graph_and_authority_chain_assessment(
                &reconciliation,
                &scope,
                &delegation,
                &[delegation.clone()],
                &boundary,
                &forward,
            )
            .unwrap();
        let reverse_result = coverage
            .validate_against_graph_and_authority_chain_assessment(
                &reconciliation,
                &scope,
                &delegation,
                &[delegation],
                &boundary,
                &reverse,
            )
            .unwrap();

        assert_eq!(forward_result, reverse_result);
    }

    #[test]
    fn reconciliation_coverage_reports_unresolved_named_authority_chain() {
        let mut root = authority_delegation("reconcile-coverage-authority-missing");
        root.delegation_id = id(
            IdentityKind::ReconciliationWitness,
            "authority-coverage-root",
        );
        root.grantee = id(
            IdentityKind::ReconciliationWitness,
            "authority-coverage-grantor",
        );
        root.subject = id(
            IdentityKind::ReconciliationWitness,
            "reconcile-coverage-authority-missing",
        );
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-coverage-child",
            ),
            grantor: root.grantee.clone(),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-coverage"),
            subject: root.subject.clone(),
            predecessor: Some(id(
                IdentityKind::ReconciliationWitness,
                "authority-coverage-missing",
            )),
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
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-coverage-missing",
            ),
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
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-authority-missing",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root_transition =
            graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
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
    fn reconciliation_coverage_uses_direct_authority_target_without_requiring_duplicate_dependency()
    {
        let mut root = authority_delegation("reconcile-direct-authority-target");
        root.delegation_id = id(
            IdentityKind::ReconciliationWitness,
            "authority-direct-target-root",
        );
        root.grantee = id(
            IdentityKind::ReconciliationWitness,
            "authority-direct-target-grantor",
        );
        root.subject = id(
            IdentityKind::ReconciliationWitness,
            "reconcile-direct-authority-target",
        );

        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-direct-target-child",
            ),
            grantor: root.grantee.clone(),
            grantee: id(
                IdentityKind::ReconciliationWitness,
                "authority-direct-target",
            ),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
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
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-direct-target",
            ),
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
        let boundary = EvidenceDispositionCoverageBoundary {
            boundary_id: id(
                IdentityKind::ReconciliationWitness,
                "boundary-direct-target",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            evidence: reconciliation.evidence.clone(),
            branch_point: reconciliation.branch_point.clone(),
            branch_heads: reconciliation.branch_heads.clone(),
            authority: reconciliation.authority.clone(),
            authority_scope: reconciliation.authority_scope.clone(),
            authority_delegation: child.delegation_id.clone(),
            basis: vec![],
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-direct-target",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: reconciliation.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: vec![],
        };

        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root_transition =
            graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain_assessment(
                &reconciliation,
                &scope,
                &child,
                &[root],
                &boundary,
                &[root_transition, left, right],
            ),
            Ok(EvidenceDispositionCoverageAssessment::Complete {
                authority_roots: vec![root.delegation_id],
            })
        );

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain(
                &reconciliation,
                &scope,
                &child,
                &[root],
                &boundary,
                &[root_transition.clone(), left.clone(), right.clone()],
            ),
            Ok(AuthorityDelegationChainAssessment::Complete {
                roots: vec![root.delegation_id],
            })
        );
    }

    #[test]
    fn reconciliation_coverage_validates_named_authority_chain_and_ignores_unrelated_invalid_delegation()
     {
        let mut root = authority_delegation("reconcile-delegation-e2e");
        root.basis = vec![id(IdentityKind::EvidenceRecord, "delegation-e2e-basis")];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-chain-child",
            ),
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
        let root_transition =
            graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let unrelated = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-e2e-unrelated-invalid",
            ),
            grantor: id(IdentityKind::ConfigurationRevision, "invalid-grantor-e2e"),
            grantee: id(IdentityKind::ReconciliationWitness, "unrelated-grantee-e2e"),
            subject: id(IdentityKind::ReconciliationWitness, "unrelated-subject-e2e"),
            predecessor: None,
            basis: vec![],
        };
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
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
        let extension = id(
            IdentityKind::ReconciliationWitness,
            "delegation-child-basis",
        );
        root.basis = vec![inherited.clone()];
        let child = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-child",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-target-child",
            ),
            grantor: root.grantee.clone(),
            grantee: id(
                IdentityKind::ReconciliationWitness,
                "authority-target-child",
            ),
            subject: root.subject.clone(),
            predecessor: Some(root.delegation_id.clone()),
            basis: root.basis.clone(),
        };
        let unrelated = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-unrelated",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-child",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-child",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-child",
            ),
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-missing",
            ),
            grantor: id(IdentityKind::ReconciliationWitness, "authority-grantor-1"),
            grantee: id(IdentityKind::ReconciliationWitness, "authority-child"),
            subject: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-delegation-missing",
            ),
            predecessor: Some(id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-absent",
            )),
            basis: vec![],
        };
        assert_eq!(
            EvidenceDispositionAuthorityDelegation::validate_graph(&[delegation]),
            Ok(AuthorityDelegationChainAssessment::Unresolved {
                missing: vec![id(
                    IdentityKind::ReconciliationWitness,
                    "authority-delegation-absent"
                )],
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
            subject: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-delegation-cycle",
            ),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-cycle-b")),
            basis: vec![],
        };
        let b = EvidenceDispositionAuthorityDelegation {
            delegation_id: id(IdentityKind::ReconciliationWitness, "authority-cycle-b"),
            grantor: id(IdentityKind::ReconciliationWitness, "grantor-b"),
            grantee: id(IdentityKind::ReconciliationWitness, "grantor-a"),
            subject: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-delegation-cycle",
            ),
            predecessor: Some(id(IdentityKind::ReconciliationWitness, "authority-cycle-a")),
            basis: vec![],
        };
        assert!(EvidenceDispositionAuthorityDelegation::validate_graph(&[a, b]).is_err());
    }

    #[test]
    fn authority_delegation_scope_requires_exact_delegation_identity() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-delegation-binding",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let mut delegation = authority_delegation(&reconciliation.reconciliation_id.id);
        delegation.delegation_id = id(IdentityKind::ReconciliationWitness, "different-delegation");
        assert!(
            scope
                .validate_against_reconciliation(&reconciliation, &delegation)
                .is_err()
        );
    }

    #[test]
    fn authority_scope_ignores_unrelated_invalid_delegation() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-scope-isolation",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-isolation",
            ),
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-isolation-witness",
            ),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-scope-isolation",
            ),
            basis: vec![],
        };
        let mut root = authority_delegation(&reconciliation.reconciliation_id.id);
        root.delegation_id = id(
            IdentityKind::ReconciliationWitness,
            "authority-delegation-isolation-root",
        );
        root.grantee = id(
            IdentityKind::ReconciliationWitness,
            "authority-scope-isolation-grantor",
        );
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
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-isolation-invalid",
            ),
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
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-scope-chain-missing",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-chain-missing",
            ),
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-chain-missing-witness",
            ),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-scope-chain-missing",
            ),
            basis: vec![],
        };
        let delegation = EvidenceDispositionAuthorityDelegation {
            delegation_id: reconciliation.authority_delegation.clone(),
            grantor: id(
                IdentityKind::ReconciliationWitness,
                "authority-grantor-scope-chain-missing",
            ),
            grantee: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            predecessor: Some(id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-missing-predecessor",
            )),
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
    fn authority_scope_rejects_binding_mismatch_before_unresolved_predecessor() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-scope-binding-precedence",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-binding-precedence",
            ),
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-binding-precedence-witness",
            ),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-scope-binding-precedence",
            ),
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: reconciliation.authority_delegation.clone(),
            basis: vec![],
        };
        let target = EvidenceDispositionAuthorityDelegation {
            delegation_id: reconciliation.authority_delegation.clone(),
            grantor: id(
                IdentityKind::ReconciliationWitness,
                "authority-grantor-scope-binding-precedence",
            ),
            grantee: id(IdentityKind::ReconciliationWitness, "different-authority"),
            subject: reconciliation.reconciliation_id.clone(),
            predecessor: Some(id(
                IdentityKind::ReconciliationWitness,
                "authority-missing-before-binding",
            )),
            basis: vec![],
        };

        assert!(
            scope
                .validate_against_reconciliation_and_authority_chain(&reconciliation, &[target],)
                .is_err()
        );
    }

    #[test]
    fn authority_scope_reports_missing_named_delegation_target() {
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-scope-target-missing",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            authority: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-target-missing",
            ),
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-target-missing-witness",
            ),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-scope-target-missing",
            ),
            basis: vec![],
        };
        let scope = EvidenceDispositionAuthorityScope {
            scope_id: reconciliation.authority_scope.clone(),
            authority: reconciliation.authority.clone(),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: reconciliation.authority_delegation.clone(),
            basis: vec![],
        };

        assert_eq!(
            scope.validate_against_reconciliation_and_authority_chain(&reconciliation, &[],),
            Ok(AuthorityDelegationChainAssessment::Unresolved {
                missing: vec![reconciliation.authority_delegation],
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
            authority_scope: id(
                IdentityKind::ReconciliationWitness,
                "authority-scope-chain-witness",
            ),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-scope-chain",
            ),
            basis: vec![],
        };
        let mut root = authority_delegation(&reconciliation.reconciliation_id.id);
        root.delegation_id = id(
            IdentityKind::ReconciliationWitness,
            "authority-delegation-scope-root",
        );
        root.grantee = id(
            IdentityKind::ReconciliationWitness,
            "authority-scope-chain-grantor",
        );
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
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-authority-basis",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
            basis: vec![required.clone()],
        };
        let missing = authority_scope("reconcile-authority-basis");
        assert!(
            missing
                .validate_against_reconciliation(
                    &reconciliation,
                    &authority_delegation(&reconciliation.reconciliation_id.id)
                )
                .is_err()
        );

        let complete = EvidenceDispositionAuthorityScope {
            scope_id: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            subject: reconciliation.reconciliation_id.clone(),
            delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
            basis: vec![required],
        };
        assert!(
            complete
                .validate_against_reconciliation(
                    &reconciliation,
                    &authority_delegation(&reconciliation.reconciliation_id.id)
                )
                .is_ok()
        );
    }

    #[test]
    fn reconciliation_basis_must_be_unique() {
        let witness = id(
            IdentityKind::EvidenceRecord,
            "reconciliation-basis-duplicate",
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-basis-duplicate",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
            basis: vec![witness.clone(), witness],
        };
        assert!(reconciliation.validate().is_err());
    }

    #[test]
    fn reconciliation_coverage_basis_must_be_unique() {
        let witness = id(IdentityKind::EvidenceRecord, "coverage-basis-duplicate");
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-basis-duplicate",
            ),
            reconciliation: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-basis-duplicate",
            ),
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
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
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
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
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
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
            basis: vec![],
        };
        assert!(
            reconciliation
                .validate_against_graph(
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &[root, left, right]
                )
                .is_ok()
        );
    }

    #[test]
    fn reconciliation_rejects_branch_point_as_head() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let right = graph_transition("t2", Some("t1"), disputed, EvidenceDisposition::Active);
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-branch-point",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t1"),
                id(IdentityKind::ReconciliationWitness, "t2"),
            ],
            authority: id(IdentityKind::ReconciliationWitness, "authority-1"),
            authority_scope: id(IdentityKind::ReconciliationWitness, "authority-scope-1"),
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
            basis: vec![],
        };
        assert!(
            reconciliation
                .validate_against_graph(
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &[root, right]
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_rejects_nested_heads_as_competing_branches() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let first = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
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
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
            basis: vec![],
        };
        assert!(
            reconciliation
                .validate_against_graph(
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &[root, first, second]
                )
                .is_err()
        );
    }

    fn authority_delegation(subject: &str) -> EvidenceDispositionAuthorityDelegation {
        EvidenceDispositionAuthorityDelegation {
            delegation_id: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
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
            delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
            basis: vec![],
        }
    }

    fn coverage_boundary() -> EvidenceDispositionCoverageBoundary {
        EvidenceDispositionCoverageBoundary {
            boundary_id: id(
                IdentityKind::ReconciliationWitness,
                "enumeration-boundary-1",
            ),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        }
    }

    #[test]
    fn reconciliation_coverage_accepts_reordered_boundary_sets() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let basis_a = id(IdentityKind::EvidenceRecord, "coverage-set-basis-a");
        let basis_b = id(IdentityKind::ReconciliationWitness, "coverage-set-basis-b");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-coverage-reordered",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: boundary.boundary_id.clone(),
            basis: vec![basis_a, basis_b],
        };

        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &scope,
                    &delegation,
                    &boundary,
                    &[root, left, right],
                )
                .is_ok()
        );
    }

    #[test]
    fn reconciliation_coverage_accepts_bounded_declared_head_set() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-1"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: id(
                IdentityKind::ReconciliationWitness,
                "enumeration-boundary-1",
            ),
            basis: vec![],
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &coverage_boundary(),
                    &[root, left, right]
                )
                .is_ok()
        );
    }

    #[test]
    fn reconciliation_coverage_assessment_accepts_additional_covered_branch_head() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "extra-head-w1"),
        };
        let root = graph_transition(
            "extra-head-t1",
            None,
            EvidenceDisposition::Active,
            disputed.clone(),
        );
        let left = graph_transition(
            "extra-head-t2",
            Some("extra-head-t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "extra-head-t3",
            Some("extra-head-t1"),
            disputed.clone(),
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "extra-head-w2"),
            },
        );
        let additional = graph_transition(
            "extra-head-t4",
            Some("extra-head-t1"),
            disputed,
            EvidenceDisposition::Unresolved,
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-extra-covered-head",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: root.transition_id.clone(),
            branch_heads: vec![left.transition_id.clone(), right.transition_id.clone()],
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
        boundary.branch_point = reconciliation.branch_point.clone();
        boundary.branch_heads = vec![
            left.transition_id.clone(),
            right.transition_id.clone(),
            additional.transition_id.clone(),
        ];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-extra-covered-head",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: boundary.branch_heads.clone(),
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
                &[root, left, right, additional],
            ),
            Ok(EvidenceDispositionCoverageAssessment::Complete {
                authority_roots: vec![delegation.delegation_id],
            })
        );

        assert_eq!(
            coverage.validate_against_graph_and_authority_chain_qualified(
                &reconciliation,
                &scope,
                &delegation,
                &[delegation.clone()],
                &boundary,
                &[root, left, right, additional],
            ),
            Ok(QualificationStatus::Complete(
                EvidenceDispositionCoverageQualification {
                    authority_roots: vec![delegation.delegation_id],
                },
            ))
        );
    }

    #[test]
    fn disposition_corpus_is_contiguous_and_machine_readable() {
        let corpus_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../docs/mobility/MOBILITY_EVIDENCE_DISPOSITION_TRANSITION_V1.json");
        let corpus = std::fs::read_to_string(&corpus_path)
            .unwrap_or_else(|error| panic!("cannot read disposition corpus: {error}"));
        let document: serde_json::Value =
            serde_json::from_str(&corpus).expect("disposition corpus must be valid JSON");

        let vectors = document
            .get("vectors")
            .and_then(serde_json::Value::as_array)
            .expect("disposition corpus must contain a vectors array");

        assert_eq!(vectors.len(), 90);

        for (index, vector) in vectors.iter().enumerate() {
            let expected_id = format!("EDT-{:03}", index + 1);
            assert_eq!(
                vector.get("id").and_then(serde_json::Value::as_str),
                Some(expected_id.as_str())
            );
            assert!(
                matches!(
                    vector.get("expected").and_then(serde_json::Value::as_str),
                    Some("accepted")
                        | Some("rejected")
                        | Some("typed_structural_error")
                        | Some("unresolved_at_protocol_layer")
                ),
                "EDT vector has unsupported expected outcome: {vector:?}"
            );
        }
    }

    #[test]
    fn reconciliation_coverage_rejects_nested_heads() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let first = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let second = graph_transition(
            "t3",
            Some("t2"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-coverage-nested",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-nested"),
            reconciliation: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-coverage-nested",
            ),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: id(
                IdentityKind::ReconciliationWitness,
                "enumeration-boundary-1",
            ),
            basis: vec![],
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &coverage_boundary(),
                    &[root.clone(), first.clone(), second.clone()]
                )
                .is_err()
        );

        let mut unresolved_delegation = authority_delegation(&reconciliation.reconciliation_id.id);
        unresolved_delegation.predecessor = Some(id(
            IdentityKind::ReconciliationWitness,
            "missing-authority-predecessor-nested",
        ));
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();

        assert!(
            coverage
                .validate_against_graph_and_authority_chain_qualified(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &unresolved_delegation,
                    &[unresolved_delegation.clone()],
                    &boundary,
                    &[root, first, second]
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_mismatch() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-boundary-mismatch",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-boundary-mismatch",
            ),
            reconciliation: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-boundary-mismatch",
            ),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: id(IdentityKind::ReconciliationWitness, "different-boundary"),
            basis: vec![],
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &coverage_boundary(),
                    &[root, left, right],
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_rejects_uncovered_reconciliation_head() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(IdentityKind::ReconciliationWitness, "coverage-2"),
            reconciliation: id(IdentityKind::ReconciliationWitness, "reconcile-coverage"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![id(IdentityKind::ReconciliationWitness, "t2")],
            boundary: id(
                IdentityKind::ReconciliationWitness,
                "enumeration-boundary-1",
            ),
            basis: vec![],
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &coverage_boundary(),
                    &[root, left, right]
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_reconciliation_mismatch() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-boundary-reconciliation-mismatch",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let boundary = EvidenceDispositionCoverageBoundary {
            reconciliation: id(
                IdentityKind::ReconciliationWitness,
                "different-reconciliation",
            ),
            ..coverage_boundary()
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-reconciliation-mismatch",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: boundary.boundary_id.clone(),
            basis: vec![],
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &boundary,
                    &[root, left, right]
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_authority_mismatch() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-boundary-authority-mismatch",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let boundary = EvidenceDispositionCoverageBoundary {
            authority: id(IdentityKind::ReconciliationWitness, "different-authority"),
            reconciliation: reconciliation.reconciliation_id.clone(),
            ..coverage_boundary()
        };
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-authority-mismatch",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: vec![
                id(IdentityKind::ReconciliationWitness, "t2"),
                id(IdentityKind::ReconciliationWitness, "t3"),
            ],
            boundary: boundary.boundary_id.clone(),
            basis: vec![],
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &boundary,
                    &[root, left, right]
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_authority_scope_mismatch() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-boundary-scope-mismatch",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.authority_scope = id(
            IdentityKind::ReconciliationWitness,
            "different-authority-scope",
        );
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-boundary-scope-mismatch",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &boundary,
                    &[root, left, right],
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_rejects_authority_scope_basis_omission() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let scope_basis = id(
            IdentityKind::ReconciliationWitness,
            "authority-scope-basis-only",
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-scope-basis-omission",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let mut scope = authority_scope(&reconciliation.reconciliation_id.id);
        scope.basis = vec![scope_basis];
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-scope-basis-omission",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &scope,
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &boundary,
                    &[root, left, right],
                )
                .is_err()
        );
    }

    #[test]
    fn reconciliation_coverage_rejects_boundary_basis_omission() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let required_basis = id(IdentityKind::EvidenceRecord, "reconciliation-basis-1");
        let mut reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-basis-omission",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
            basis: vec![required_basis],
        };
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-basis-omission",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &boundary,
                    &[root, left, right]
                )
                .is_err()
        );
        reconciliation.basis.clear();
    }

    #[test]
    fn reconciliation_coverage_accepts_authority_scope_basis_at_boundary() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let scope_basis = id(
            IdentityKind::ReconciliationWitness,
            "authority-scope-basis-carried",
        );
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-scope-basis-carried",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
        let mut delegation = authority_delegation(&reconciliation.reconciliation_id.id);
        delegation.basis = vec![scope_basis.clone()];
        let mut scope = authority_scope(&reconciliation.reconciliation_id.id);
        scope.basis = vec![scope_basis.clone()];
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![scope_basis];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-scope-basis-carried",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: reconciliation.branch_point.clone(),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &scope,
                    &delegation,
                    &boundary,
                    &[root, left, right],
                )
                .is_ok()
        );
    }

    #[test]
    fn reconciliation_coverage_accepts_boundary_basis_extension() {
        let disputed = EvidenceDisposition::Disputed {
            by: id(IdentityKind::ReconciliationWitness, "w1"),
        };
        let root = graph_transition("t1", None, EvidenceDisposition::Active, disputed.clone());
        let left = graph_transition(
            "t2",
            Some("t1"),
            disputed.clone(),
            EvidenceDisposition::Active,
        );
        let right = graph_transition(
            "t3",
            Some("t1"),
            disputed,
            EvidenceDisposition::Retracted {
                by: id(IdentityKind::ReconciliationWitness, "w2"),
            },
        );
        let required_basis = id(IdentityKind::EvidenceRecord, "reconciliation-basis-1");
        let additional_basis = id(IdentityKind::ReconciliationWitness, "coverage-basis-1");
        let reconciliation = EvidenceDispositionReconciliation {
            reconciliation_id: id(
                IdentityKind::ReconciliationWitness,
                "reconcile-basis-extension",
            ),
            evidence: id(IdentityKind::InspectionRecord, "inspection-graph"),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
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
            basis: vec![required_basis.clone()],
        };
        let mut boundary = coverage_boundary();
        boundary.reconciliation = reconciliation.reconciliation_id.clone();
        boundary.basis = vec![required_basis, additional_basis];
        let coverage = EvidenceDispositionReconciliationCoverage {
            coverage_id: id(
                IdentityKind::ReconciliationWitness,
                "coverage-basis-extension",
            ),
            reconciliation: reconciliation.reconciliation_id.clone(),
            branch_point: id(IdentityKind::ReconciliationWitness, "t1"),
            covered_branch_heads: boundary.branch_heads.clone(),
            boundary: boundary.boundary_id.clone(),
            basis: boundary.basis.clone(),
        };
        assert!(
            coverage
                .validate_against_graph(
                    &reconciliation,
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &boundary,
                    &[root, left, right]
                )
                .is_ok()
        );
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
            authority_delegation: id(
                IdentityKind::ReconciliationWitness,
                "authority-delegation-1",
            ),
            basis: vec![],
        };
        assert!(
            reconciliation
                .validate_against_graph(
                    &authority_scope(&reconciliation.reconciliation_id.id),
                    &authority_delegation(&reconciliation.reconciliation_id.id),
                    &[root, other]
                )
                .is_err()
        );
    }

    #[test]
    fn wrong_evidence_kind_is_rejected() {
        let mut value = evidence_case();
        value.evidence = id(IdentityKind::DesignRevision, "design-1");
        assert!(value.validate().is_err());
    }
}
