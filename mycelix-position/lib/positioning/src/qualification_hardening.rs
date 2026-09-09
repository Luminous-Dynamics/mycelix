// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Bounded, non-recursive qualification registry hardening.
//!
//! This module is an additive hardening layer over the v0.1 qualification
//! types. It is intended for evidence and policy boundaries that may receive
//! large or adversarially shaped semantic objects.
//!
//! The limits here apply **after deserialization**. Callers accepting untrusted
//! bytes must still impose an upstream encoded-size limit before parsing; an
//! object-level theorem/facet limit cannot prevent a parser from first
//! allocating an arbitrarily large wire object.
//!
//! The default limits are implementation admission limits for this v0.2
//! profile, not immutable protocol constants. Durable evidence should commit
//! to the admission profile used when that distinction becomes authoritative.

use crate::qualification::{
    FacetStatus, QualificationError, QualificationFacet, QualificationManifest,
    QualificationRequirementProfile, RequirementEvaluation, TheoremDefinition, TheoremId,
    UnacceptableFacet,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Default v0.2 admission limits for qualification semantic objects.
///
/// String limits are measured in UTF-8 bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationLimits {
    pub max_theorems: usize,
    pub max_prerequisites_per_theorem: usize,
    pub max_total_prerequisite_edges: usize,
    pub max_facets: usize,
    pub max_required_theorems: usize,
    pub max_accepted_statuses: usize,
    pub max_evidence_refs_per_facet: usize,
    pub max_dependency_commitments_per_facet: usize,
    pub max_explanation_blockers: usize,
    pub max_explanation_path_theorems: usize,
    pub max_theorem_id_bytes: usize,
    pub max_profile_id_bytes: usize,
    pub max_subject_commitment_bytes: usize,
    pub max_verifier_profile_bytes: usize,
    pub max_evidence_ref_bytes: usize,
    pub max_dependency_commitment_bytes: usize,
    pub max_diagnostics_commitment_bytes: usize,
}

impl Default for QualificationLimits {
    fn default() -> Self {
        Self {
            max_theorems: 2_048,
            max_prerequisites_per_theorem: 32,
            max_total_prerequisite_edges: 65_536,
            max_facets: 2_048,
            max_required_theorems: 256,
            max_accepted_statuses: 7,
            max_evidence_refs_per_facet: 64,
            max_dependency_commitments_per_facet: 64,
            max_explanation_blockers: 256,
            max_explanation_path_theorems: 2_048,
            max_theorem_id_bytes: 192,
            max_profile_id_bytes: 192,
            max_subject_commitment_bytes: 512,
            max_verifier_profile_bytes: 512,
            max_evidence_ref_bytes: 2_048,
            max_dependency_commitment_bytes: 1_024,
            max_diagnostics_commitment_bytes: 1_024,
        }
    }
}

/// Resource dimension rejected by the hardened semantic admission boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum QualificationResource {
    RegistryTheorems,
    TheoremPrerequisites,
    TotalPrerequisiteEdges,
    ManifestFacets,
    ProfileRequiredTheorems,
    ProfileAcceptedStatuses,
    FacetEvidenceRefs,
    FacetDependencyCommitments,
    ExplanationBlockers,
    ExplanationPathTheorems,
    TheoremIdBytes,
    ProfileIdBytes,
    SubjectCommitmentBytes,
    VerifierProfileBytes,
    EvidenceRefBytes,
    DependencyCommitmentBytes,
    DiagnosticsCommitmentBytes,
}

/// Hardened qualification failures keep v0.1 semantic errors distinct from
/// explicit resource-admission failures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationHardeningError {
    Semantic(QualificationError),
    LimitExceeded {
        resource: QualificationResource,
        limit: usize,
        actual: usize,
        theorem_id: Option<TheoremId>,
    },
}

impl From<QualificationError> for QualificationHardeningError {
    fn from(value: QualificationError) -> Self {
        Self::Semantic(value)
    }
}

impl std::fmt::Display for QualificationHardeningError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Semantic(error) => error.fmt(f),
            Self::LimitExceeded {
                resource,
                limit,
                actual,
                theorem_id,
            } => {
                if let Some(theorem_id) = theorem_id {
                    write!(
                        f,
                        "qualification resource {:?} exceeds limit for theorem {}: {} > {}",
                        resource,
                        theorem_id.as_str(),
                        actual,
                        limit
                    )
                } else {
                    write!(
                        f,
                        "qualification resource {:?} exceeds limit: {} > {}",
                        resource, actual, limit
                    )
                }
            }
        }
    }
}

impl std::error::Error for QualificationHardeningError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Semantic(error) => Some(error),
            Self::LimitExceeded { .. } => None,
        }
    }
}

/// Terminal reason that an establishment proof path is blocked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EstablishmentBlockerKind {
    MissingFacet,
    UnacceptableStatus { status: FacetStatus },
}

/// One canonical dependency path from a requested theorem to a blocked facet.
///
/// `path[0]` is always the requested theorem. The final element is the missing
/// or non-established facet. The hardened registry selects a shortest path;
/// equal-length alternatives use a deterministic lexical parent tie-break.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EstablishmentBlocker {
    pub requested_theorem: TheoremId,
    pub path: Vec<TheoremId>,
    pub kind: EstablishmentBlockerKind,
}

/// Deterministic explanation of whether a theorem and its prerequisite closure
/// are established in one manifest.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EstablishmentExplanation {
    pub theorem_id: TheoremId,
    pub established: bool,
    pub blockers: Vec<EstablishmentBlocker>,
}

/// Resource-bounded theorem registry with iterative deterministic DAG
/// validation.
#[derive(Debug, Clone)]
pub struct HardenedTheoremRegistry {
    definitions: BTreeMap<TheoremId, TheoremDefinition>,
    topological_order: Vec<TheoremId>,
    limits: QualificationLimits,
}

impl HardenedTheoremRegistry {
    /// Construct with the default v0.2 admission profile.
    pub fn new(
        definitions: Vec<TheoremDefinition>,
    ) -> Result<Self, QualificationHardeningError> {
        Self::with_limits(definitions, QualificationLimits::default())
    }

    /// Construct with an explicit admission profile.
    pub fn with_limits(
        definitions: Vec<TheoremDefinition>,
        limits: QualificationLimits,
    ) -> Result<Self, QualificationHardeningError> {
        Self::check_limit(
            QualificationResource::RegistryTheorems,
            definitions.len(),
            limits.max_theorems,
            None,
        )?;

        let total_edges = definitions
            .iter()
            .try_fold(0usize, |acc, definition| {
                acc.checked_add(definition.prerequisites.len())
            })
            .unwrap_or(usize::MAX);
        Self::check_limit(
            QualificationResource::TotalPrerequisiteEdges,
            total_edges,
            limits.max_total_prerequisite_edges,
            None,
        )?;

        let mut by_id = BTreeMap::new();
        for definition in definitions {
            Self::validate_theorem_id(&definition.id, &limits)?;
            Self::check_limit(
                QualificationResource::TheoremPrerequisites,
                definition.prerequisites.len(),
                limits.max_prerequisites_per_theorem,
                Some(&definition.id),
            )?;

            if by_id.contains_key(&definition.id) {
                return Err(QualificationError::DuplicateTheorem {
                    theorem_id: definition.id,
                }
                .into());
            }
            by_id.insert(definition.id.clone(), definition);
        }

        for definition in by_id.values() {
            let mut seen = BTreeSet::new();
            for prerequisite in &definition.prerequisites {
                Self::validate_theorem_id(prerequisite, &limits)?;
                if !seen.insert(prerequisite.clone()) {
                    return Err(QualificationError::DuplicatePrerequisite {
                        theorem_id: definition.id.clone(),
                        prerequisite: prerequisite.clone(),
                    }
                    .into());
                }
                if !by_id.contains_key(prerequisite) {
                    return Err(QualificationError::UnknownPrerequisite {
                        theorem_id: definition.id.clone(),
                        prerequisite: prerequisite.clone(),
                    }
                    .into());
                }
            }
        }

        let topological_order = Self::topological_sort(&by_id)?;
        Ok(Self {
            definitions: by_id,
            topological_order,
            limits,
        })
    }

    pub fn limits(&self) -> QualificationLimits {
        self.limits
    }

    pub fn len(&self) -> usize {
        self.definitions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.definitions.is_empty()
    }

    pub fn contains(&self, theorem_id: &TheoremId) -> bool {
        self.definitions.contains_key(theorem_id)
    }

    pub fn definition(&self, theorem_id: &TheoremId) -> Option<&TheoremDefinition> {
        self.definitions.get(theorem_id)
    }

    /// Definitions in canonical theorem-ID order.
    pub fn definitions(&self) -> impl Iterator<Item = &TheoremDefinition> {
        self.definitions.values()
    }

    /// Canonical dependency-first theorem order produced by lexical Kahn
    /// traversal.
    pub fn topological_order(&self) -> &[TheoremId] {
        &self.topological_order
    }

    /// Return the full transitive prerequisite set in lexical theorem-ID order.
    /// The requested theorem itself is not included.
    pub fn prerequisite_closure(
        &self,
        theorem_id: &TheoremId,
    ) -> Result<Vec<TheoremId>, QualificationHardeningError> {
        if !self.contains(theorem_id) {
            return Err(QualificationError::UnknownRequiredTheorem {
                theorem_id: theorem_id.clone(),
            }
            .into());
        }

        let mut pending = BTreeSet::new();
        let mut closure = BTreeSet::new();
        if let Some(definition) = self.definitions.get(theorem_id) {
            pending.extend(definition.prerequisites.iter().cloned());
        }

        while let Some(current) = pending.pop_first() {
            if !closure.insert(current.clone()) {
                continue;
            }
            let definition = self
                .definitions
                .get(&current)
                .expect("registry construction validated all prerequisite IDs");
            pending.extend(definition.prerequisites.iter().cloned());
        }

        Ok(closure.into_iter().collect())
    }

    /// Validate manifest shape/resource limits without requiring established
    /// facets to have already-established prerequisites.
    ///
    /// This is intentionally public so explanation tooling can diagnose a
    /// semantically blocked manifest rather than rejecting it before producing
    /// causal paths.
    pub fn validate_manifest_shape(
        &self,
        manifest: &QualificationManifest,
    ) -> Result<(), QualificationHardeningError> {
        self.manifest_facets(manifest).map(|_| ())
    }

    /// Validate the complete v0.1 establishment invariant after applying v0.2
    /// resource bounds.
    pub fn validate_manifest(
        &self,
        manifest: &QualificationManifest,
    ) -> Result<(), QualificationHardeningError> {
        let facets = self.manifest_facets(manifest)?;

        for facet in facets.values() {
            if facet.status != FacetStatus::Established {
                continue;
            }
            let definition = self
                .definitions
                .get(&facet.theorem_id)
                .expect("manifest theorem existence checked above");
            for prerequisite in &definition.prerequisites {
                let prerequisite_established = facets
                    .get(prerequisite)
                    .is_some_and(|candidate| candidate.status == FacetStatus::Established);
                if !prerequisite_established {
                    return Err(QualificationError::EstablishedFacetMissingPrerequisite {
                        theorem_id: facet.theorem_id.clone(),
                        prerequisite: prerequisite.clone(),
                    }
                    .into());
                }
            }
        }

        Ok(())
    }

    pub fn validate_requirement_profile(
        &self,
        profile: &QualificationRequirementProfile,
    ) -> Result<(), QualificationHardeningError> {
        if profile.profile_id.trim().is_empty() {
            return Err(QualificationError::EmptyProfileId.into());
        }
        Self::check_limit(
            QualificationResource::ProfileIdBytes,
            profile.profile_id.len(),
            self.limits.max_profile_id_bytes,
            None,
        )?;
        Self::check_limit(
            QualificationResource::ProfileRequiredTheorems,
            profile.required_theorems.len(),
            self.limits.max_required_theorems,
            None,
        )?;
        Self::check_limit(
            QualificationResource::ProfileAcceptedStatuses,
            profile.accepted_statuses.len(),
            self.limits.max_accepted_statuses,
            None,
        )?;
        if profile.accepted_statuses.is_empty() {
            return Err(QualificationError::EmptyAcceptedStatuses.into());
        }

        let mut seen = BTreeSet::new();
        for theorem_id in &profile.required_theorems {
            Self::validate_theorem_id(theorem_id, &self.limits)?;
            if !self.contains(theorem_id) {
                return Err(QualificationError::UnknownRequiredTheorem {
                    theorem_id: theorem_id.clone(),
                }
                .into());
            }
            if !seen.insert(theorem_id.clone()) {
                return Err(QualificationError::DuplicateRequiredTheorem {
                    theorem_id: theorem_id.clone(),
                }
                .into());
            }
        }
        Ok(())
    }

    /// Evaluate using the same consumer-policy semantics as v0.1 after hardened
    /// resource and structural validation succeeds.
    pub fn evaluate(
        &self,
        manifest: &QualificationManifest,
        profile: &QualificationRequirementProfile,
    ) -> Result<RequirementEvaluation, QualificationHardeningError> {
        self.validate_manifest(manifest)?;
        self.validate_requirement_profile(profile)?;

        let facet_statuses: BTreeMap<_, _> = manifest
            .facets
            .iter()
            .map(|facet| (facet.theorem_id.clone(), facet.status))
            .collect();
        let accepted: BTreeSet<_> = profile.accepted_statuses.iter().copied().collect();
        let mut required = profile.required_theorems.clone();
        required.sort();

        let mut missing_theorems = Vec::new();
        let mut unacceptable_facets = Vec::new();
        for theorem_id in required {
            match facet_statuses.get(&theorem_id).copied() {
                None => missing_theorems.push(theorem_id),
                Some(status) if !accepted.contains(&status) => {
                    unacceptable_facets.push(UnacceptableFacet { theorem_id, status });
                }
                Some(_) => {}
            }
        }

        Ok(RequirementEvaluation {
            profile_id: profile.profile_id.clone(),
            subject_commitment: manifest.subject_commitment.clone(),
            satisfied: missing_theorems.is_empty() && unacceptable_facets.is_empty(),
            missing_theorems,
            unacceptable_facets,
        })
    }

    /// Explain why a theorem is or is not established through its prerequisite
    /// closure.
    ///
    /// Traversal is iterative. It stores one distance and one predecessor per
    /// reached theorem rather than cloning whole paths into the frontier. That
    /// keeps traversal state O(V + E). Output paths are reconstructed only for
    /// terminal blockers and are themselves explicitly bounded.
    ///
    /// Canonical path rule: shortest dependency path first; equal-length paths
    /// choose the lexically smaller immediate predecessor for the reached node.
    pub fn explain_establishment(
        &self,
        manifest: &QualificationManifest,
        theorem_id: &TheoremId,
    ) -> Result<EstablishmentExplanation, QualificationHardeningError> {
        if !self.contains(theorem_id) {
            return Err(QualificationError::UnknownRequiredTheorem {
                theorem_id: theorem_id.clone(),
            }
            .into());
        }

        let facets = self.manifest_facets(manifest)?;
        let mut frontier: BTreeSet<(usize, TheoremId)> = BTreeSet::new();
        let mut distance: BTreeMap<TheoremId, usize> = BTreeMap::new();
        let mut predecessor: BTreeMap<TheoremId, TheoremId> = BTreeMap::new();
        let mut processed = BTreeSet::new();
        let mut terminal: Vec<(TheoremId, EstablishmentBlockerKind)> = Vec::new();

        distance.insert(theorem_id.clone(), 0);
        frontier.insert((0, theorem_id.clone()));

        while let Some((current_distance, current)) = frontier.pop_first() {
            if !processed.insert(current.clone()) {
                continue;
            }

            match facets.get(&current) {
                None => {
                    Self::push_terminal(
                        &mut terminal,
                        current,
                        EstablishmentBlockerKind::MissingFacet,
                        &self.limits,
                    )?;
                }
                Some(facet) if facet.status != FacetStatus::Established => {
                    Self::push_terminal(
                        &mut terminal,
                        current,
                        EstablishmentBlockerKind::UnacceptableStatus {
                            status: facet.status,
                        },
                        &self.limits,
                    )?;
                }
                Some(_) => {
                    let definition = self
                        .definitions
                        .get(&current)
                        .expect("known theorem checked by registry construction");
                    let mut prerequisites = definition.prerequisites.clone();
                    prerequisites.sort();

                    for prerequisite in prerequisites {
                        let candidate_distance = current_distance.saturating_add(1);
                        match distance.get(&prerequisite).copied() {
                            None => {
                                distance.insert(prerequisite.clone(), candidate_distance);
                                predecessor.insert(prerequisite.clone(), current.clone());
                                frontier.insert((candidate_distance, prerequisite));
                            }
                            Some(existing) if candidate_distance < existing => {
                                frontier.remove(&(existing, prerequisite.clone()));
                                distance.insert(prerequisite.clone(), candidate_distance);
                                predecessor.insert(prerequisite.clone(), current.clone());
                                frontier.insert((candidate_distance, prerequisite));
                            }
                            Some(existing) if candidate_distance == existing => {
                                let replace_parent = predecessor
                                    .get(&prerequisite)
                                    .is_none_or(|parent| current < *parent);
                                if replace_parent {
                                    predecessor.insert(prerequisite, current.clone());
                                }
                            }
                            Some(_) => {}
                        }
                    }
                }
            }
        }

        let mut blockers = Vec::with_capacity(terminal.len());
        for (blocked, kind) in terminal {
            let path = self.reconstruct_path(theorem_id, &blocked, &predecessor)?;
            blockers.push(EstablishmentBlocker {
                requested_theorem: theorem_id.clone(),
                path,
                kind,
            });
        }
        blockers.sort_by(|left, right| {
            left.path
                .cmp(&right.path)
                .then_with(|| left.kind.cmp(&right.kind))
        });

        Ok(EstablishmentExplanation {
            theorem_id: theorem_id.clone(),
            established: blockers.is_empty(),
            blockers,
        })
    }

    fn push_terminal(
        terminal: &mut Vec<(TheoremId, EstablishmentBlockerKind)>,
        theorem_id: TheoremId,
        kind: EstablishmentBlockerKind,
        limits: &QualificationLimits,
    ) -> Result<(), QualificationHardeningError> {
        let actual = terminal.len().saturating_add(1);
        Self::check_limit(
            QualificationResource::ExplanationBlockers,
            actual,
            limits.max_explanation_blockers,
            Some(&theorem_id),
        )?;
        terminal.push((theorem_id, kind));
        Ok(())
    }

    fn reconstruct_path(
        &self,
        requested: &TheoremId,
        blocked: &TheoremId,
        predecessor: &BTreeMap<TheoremId, TheoremId>,
    ) -> Result<Vec<TheoremId>, QualificationHardeningError> {
        let mut reverse = Vec::new();
        let mut current = blocked.clone();

        loop {
            reverse.push(current.clone());
            Self::check_limit(
                QualificationResource::ExplanationPathTheorems,
                reverse.len(),
                self.limits.max_explanation_path_theorems,
                Some(blocked),
            )?;

            if &current == requested {
                break;
            }
            current = predecessor
                .get(&current)
                .expect("reached blocker always has a predecessor path")
                .clone();
        }

        reverse.reverse();
        Ok(reverse)
    }

    fn manifest_facets<'a>(
        &self,
        manifest: &'a QualificationManifest,
    ) -> Result<BTreeMap<TheoremId, &'a QualificationFacet>, QualificationHardeningError> {
        if manifest.subject_commitment.trim().is_empty() {
            return Err(QualificationError::EmptySubjectCommitment.into());
        }
        Self::check_limit(
            QualificationResource::SubjectCommitmentBytes,
            manifest.subject_commitment.len(),
            self.limits.max_subject_commitment_bytes,
            None,
        )?;
        Self::check_limit(
            QualificationResource::ManifestFacets,
            manifest.facets.len(),
            self.limits.max_facets,
            None,
        )?;

        let mut facets = BTreeMap::new();
        for facet in &manifest.facets {
            Self::validate_theorem_id(&facet.theorem_id, &self.limits)?;
            Self::check_limit(
                QualificationResource::SubjectCommitmentBytes,
                facet.subject_commitment.len(),
                self.limits.max_subject_commitment_bytes,
                Some(&facet.theorem_id),
            )?;
            if !self.contains(&facet.theorem_id) {
                return Err(QualificationError::UnknownFacetTheorem {
                    theorem_id: facet.theorem_id.clone(),
                }
                .into());
            }
            if facet.subject_commitment != manifest.subject_commitment {
                return Err(QualificationError::FacetSubjectMismatch {
                    theorem_id: facet.theorem_id.clone(),
                }
                .into());
            }

            if let Some(verifier) = &facet.verifier_or_profile {
                Self::check_limit(
                    QualificationResource::VerifierProfileBytes,
                    verifier.len(),
                    self.limits.max_verifier_profile_bytes,
                    Some(&facet.theorem_id),
                )?;
            }

            Self::check_limit(
                QualificationResource::FacetEvidenceRefs,
                facet.evidence_refs.len(),
                self.limits.max_evidence_refs_per_facet,
                Some(&facet.theorem_id),
            )?;
            for evidence_ref in &facet.evidence_refs {
                Self::check_limit(
                    QualificationResource::EvidenceRefBytes,
                    evidence_ref.len(),
                    self.limits.max_evidence_ref_bytes,
                    Some(&facet.theorem_id),
                )?;
            }

            Self::check_limit(
                QualificationResource::FacetDependencyCommitments,
                facet.dependency_commitments.len(),
                self.limits.max_dependency_commitments_per_facet,
                Some(&facet.theorem_id),
            )?;
            for commitment in &facet.dependency_commitments {
                Self::check_limit(
                    QualificationResource::DependencyCommitmentBytes,
                    commitment.len(),
                    self.limits.max_dependency_commitment_bytes,
                    Some(&facet.theorem_id),
                )?;
            }

            if let Some(commitment) = &facet.diagnostics_commitment {
                Self::check_limit(
                    QualificationResource::DiagnosticsCommitmentBytes,
                    commitment.len(),
                    self.limits.max_diagnostics_commitment_bytes,
                    Some(&facet.theorem_id),
                )?;
            }

            if facets.insert(facet.theorem_id.clone(), facet).is_some() {
                return Err(QualificationError::DuplicateFacet {
                    theorem_id: facet.theorem_id.clone(),
                }
                .into());
            }
        }

        Ok(facets)
    }

    fn validate_theorem_id(
        theorem_id: &TheoremId,
        limits: &QualificationLimits,
    ) -> Result<(), QualificationHardeningError> {
        if theorem_id.as_str().trim().is_empty() {
            return Err(QualificationError::EmptyTheoremId.into());
        }
        Self::check_limit(
            QualificationResource::TheoremIdBytes,
            theorem_id.as_str().len(),
            limits.max_theorem_id_bytes,
            Some(theorem_id),
        )
    }

    fn check_limit(
        resource: QualificationResource,
        actual: usize,
        limit: usize,
        theorem_id: Option<&TheoremId>,
    ) -> Result<(), QualificationHardeningError> {
        if actual > limit {
            return Err(QualificationHardeningError::LimitExceeded {
                resource,
                limit,
                actual,
                theorem_id: theorem_id.cloned(),
            });
        }
        Ok(())
    }

    fn topological_sort(
        definitions: &BTreeMap<TheoremId, TheoremDefinition>,
    ) -> Result<Vec<TheoremId>, QualificationHardeningError> {
        let mut remaining_prerequisites: BTreeMap<TheoremId, usize> = definitions
            .iter()
            .map(|(id, definition)| (id.clone(), definition.prerequisites.len()))
            .collect();
        let mut dependents: BTreeMap<TheoremId, BTreeSet<TheoremId>> = definitions
            .keys()
            .cloned()
            .map(|id| (id, BTreeSet::new()))
            .collect();

        for definition in definitions.values() {
            for prerequisite in &definition.prerequisites {
                dependents
                    .get_mut(prerequisite)
                    .expect("prerequisite existence checked before sort")
                    .insert(definition.id.clone());
            }
        }

        let mut ready: BTreeSet<TheoremId> = remaining_prerequisites
            .iter()
            .filter(|(_, count)| **count == 0)
            .map(|(id, _)| id.clone())
            .collect();
        let mut order = Vec::with_capacity(definitions.len());

        while let Some(current) = ready.pop_first() {
            order.push(current.clone());
            if let Some(children) = dependents.get(&current) {
                for child in children {
                    let remaining = remaining_prerequisites
                        .get_mut(child)
                        .expect("dependent theorem exists");
                    *remaining = remaining
                        .checked_sub(1)
                        .expect("dependent prerequisite count does not underflow");
                    if *remaining == 0 {
                        ready.insert(child.clone());
                    }
                }
            }
        }

        if order.len() != definitions.len() {
            let theorem_id = remaining_prerequisites
                .iter()
                .find(|(_, count)| **count > 0)
                .map(|(id, _)| id.clone())
                .expect("cycle leaves at least one theorem with unmet prerequisites");
            return Err(QualificationError::DependencyCycle { theorem_id }.into());
        }

        Ok(order)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> TheoremId {
        TheoremId::from(value)
    }

    fn definition(value: &str, prerequisites: &[&str]) -> TheoremDefinition {
        TheoremDefinition::new(
            value,
            prerequisites.iter().map(|value| id(value)).collect(),
        )
    }

    fn facet(theorem: &str, subject: &str, status: FacetStatus) -> QualificationFacet {
        QualificationFacet {
            theorem_id: id(theorem),
            subject_commitment: subject.to_string(),
            status,
            verifier_or_profile: None,
            evidence_refs: Vec::new(),
            dependency_commitments: Vec::new(),
            diagnostics_commitment: None,
        }
    }

    fn registry() -> HardenedTheoremRegistry {
        HardenedTheoremRegistry::new(vec![
            definition("Q0.SerializationSafety.v1", &[]),
            definition("Q2.FrameIdentity.v1", &["Q0.SerializationSafety.v1"]),
            definition(
                "Q2.TransformPath.v1",
                &["Q0.SerializationSafety.v1", "Q2.FrameIdentity.v1"],
            ),
            definition(
                "Q4.GeometryObservability.v1",
                &["Q0.SerializationSafety.v1"],
            ),
        ])
        .unwrap()
    }

    #[test]
    fn rejects_registry_theorem_limit_before_graph_traversal() {
        let limits = QualificationLimits {
            max_theorems: 1,
            ..QualificationLimits::default()
        };
        let result = HardenedTheoremRegistry::with_limits(
            vec![definition("Q0", &[]), definition("Q1", &[])],
            limits,
        );
        assert!(matches!(
            result,
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::RegistryTheorems,
                limit: 1,
                actual: 2,
                ..
            })
        ));
    }

    #[test]
    fn rejects_total_edge_budget() {
        let limits = QualificationLimits {
            max_total_prerequisite_edges: 1,
            ..QualificationLimits::default()
        };
        let result = HardenedTheoremRegistry::with_limits(
            vec![
                definition("Q0", &[]),
                definition("Q1", &["Q0"]),
                definition("Q2", &["Q0"]),
            ],
            limits,
        );
        assert!(matches!(
            result,
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::TotalPrerequisiteEdges,
                limit: 1,
                actual: 2,
                ..
            })
        ));
    }

    #[test]
    fn rejects_overlong_theorem_id() {
        let limits = QualificationLimits {
            max_theorem_id_bytes: 4,
            ..QualificationLimits::default()
        };
        let result =
            HardenedTheoremRegistry::with_limits(vec![definition("ABCDE", &[])], limits);
        assert!(matches!(
            result,
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::TheoremIdBytes,
                ..
            })
        ));
    }

    #[test]
    fn iterative_topological_validation_handles_deep_chain() {
        let depth = 1_500usize;
        let mut definitions = Vec::with_capacity(depth);
        for index in 0..depth {
            let theorem = format!("Q{index:04}");
            let prerequisites = if index == 0 {
                Vec::new()
            } else {
                vec![id(&format!("Q{:04}", index - 1))]
            };
            definitions.push(TheoremDefinition::new(theorem, prerequisites));
        }

        let registry = HardenedTheoremRegistry::new(definitions).unwrap();
        assert_eq!(registry.len(), depth);
        assert_eq!(registry.topological_order().len(), depth);
        assert_eq!(registry.topological_order().first(), Some(&id("Q0000")));
        assert_eq!(
            registry.topological_order().last(),
            Some(&id("Q1499"))
        );
    }

    #[test]
    fn cycle_error_is_deterministic() {
        let first = HardenedTheoremRegistry::new(vec![
            definition("Q2", &["Q1"]),
            definition("Q1", &["Q2"]),
        ])
        .unwrap_err();
        let second = HardenedTheoremRegistry::new(vec![
            definition("Q1", &["Q2"]),
            definition("Q2", &["Q1"]),
        ])
        .unwrap_err();

        assert_eq!(first, second);
        assert!(matches!(
            first,
            QualificationHardeningError::Semantic(QualificationError::DependencyCycle {
                theorem_id
            }) if theorem_id == id("Q1")
        ));
    }

    #[test]
    fn dependency_first_order_uses_lexical_tie_break() {
        let registry = HardenedTheoremRegistry::new(vec![
            definition("Q3", &["Q1", "Q2"]),
            definition("Q2", &[]),
            definition("Q1", &[]),
        ])
        .unwrap();
        assert_eq!(
            registry.topological_order(),
            &[id("Q1"), id("Q2"), id("Q3")]
        );
    }

    #[test]
    fn prerequisite_closure_is_transitive_and_lexical() {
        let registry = registry();
        assert_eq!(
            registry.prerequisite_closure(&id("Q2.TransformPath.v1")).unwrap(),
            vec![id("Q0.SerializationSafety.v1"), id("Q2.FrameIdentity.v1")]
        );
    }

    #[test]
    fn established_child_still_requires_established_prerequisite() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet(
                "Q2.FrameIdentity.v1",
                "state:abc",
                FacetStatus::Established,
            )],
        };
        assert!(matches!(
            registry.validate_manifest(&manifest),
            Err(QualificationHardeningError::Semantic(
                QualificationError::EstablishedFacetMissingPrerequisite { .. }
            ))
        ));
    }

    #[test]
    fn explanation_catches_established_claim_with_missing_prerequisite() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet(
                "Q2.FrameIdentity.v1",
                "state:abc",
                FacetStatus::Established,
            )],
        };

        let explanation = registry
            .explain_establishment(&manifest, &id("Q2.FrameIdentity.v1"))
            .unwrap();
        assert!(!explanation.established);
        assert_eq!(explanation.blockers.len(), 1);
        assert_eq!(
            explanation.blockers[0].path,
            vec![id("Q2.FrameIdentity.v1"), id("Q0.SerializationSafety.v1")]
        );
        assert_eq!(
            explanation.blockers[0].kind,
            EstablishmentBlockerKind::MissingFacet
        );
    }

    #[test]
    fn explanation_preserves_indeterminate_blocker_status() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet(
                    "Q0.SerializationSafety.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
                facet(
                    "Q2.FrameIdentity.v1",
                    "state:abc",
                    FacetStatus::Indeterminate,
                ),
                facet(
                    "Q2.TransformPath.v1",
                    "state:abc",
                    FacetStatus::Established,
                ),
            ],
        };

        let explanation = registry
            .explain_establishment(&manifest, &id("Q2.TransformPath.v1"))
            .unwrap();
        assert!(!explanation.established);
        assert_eq!(explanation.blockers.len(), 1);
        assert_eq!(
            explanation.blockers[0].path,
            vec![id("Q2.TransformPath.v1"), id("Q2.FrameIdentity.v1")]
        );
        assert_eq!(
            explanation.blockers[0].kind,
            EstablishmentBlockerKind::UnacceptableStatus {
                status: FacetStatus::Indeterminate
            }
        );
    }

    #[test]
    fn shared_dependency_emits_one_canonical_blocker() {
        let registry = HardenedTheoremRegistry::new(vec![
            definition("A", &[]),
            definition("B", &["A"]),
            definition("C", &["A"]),
            definition("D", &["C", "B"]),
        ])
        .unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet("D", "state:abc", FacetStatus::Established),
                facet("B", "state:abc", FacetStatus::Established),
                facet("C", "state:abc", FacetStatus::Established),
            ],
        };

        let explanation = registry.explain_establishment(&manifest, &id("D")).unwrap();
        assert_eq!(explanation.blockers.len(), 1);
        assert_eq!(
            explanation.blockers[0].path,
            vec![id("D"), id("B"), id("A")]
        );
    }

    #[test]
    fn canonical_blocker_path_is_input_order_independent() {
        let first = HardenedTheoremRegistry::new(vec![
            definition("A", &[]),
            definition("B", &["A"]),
            definition("C", &["A"]),
            definition("D", &["C", "B"]),
        ])
        .unwrap();
        let second = HardenedTheoremRegistry::new(vec![
            definition("D", &["B", "C"]),
            definition("C", &["A"]),
            definition("B", &["A"]),
            definition("A", &[]),
        ])
        .unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet("D", "state:abc", FacetStatus::Established),
                facet("B", "state:abc", FacetStatus::Established),
                facet("C", "state:abc", FacetStatus::Established),
            ],
        };

        assert_eq!(
            first.explain_establishment(&manifest, &id("D")).unwrap(),
            second.explain_establishment(&manifest, &id("D")).unwrap()
        );
    }

    #[test]
    fn explanation_blocker_count_is_bounded() {
        let limits = QualificationLimits {
            max_explanation_blockers: 1,
            ..QualificationLimits::default()
        };
        let registry = HardenedTheoremRegistry::with_limits(
            vec![
                definition("A", &[]),
                definition("B", &[]),
                definition("C", &["A", "B"]),
            ],
            limits,
        )
        .unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet("C", "state:abc", FacetStatus::Established)],
        };

        assert!(matches!(
            registry.explain_establishment(&manifest, &id("C")),
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::ExplanationBlockers,
                ..
            })
        ));
    }

    #[test]
    fn explanation_path_length_is_bounded() {
        let limits = QualificationLimits {
            max_explanation_path_theorems: 2,
            ..QualificationLimits::default()
        };
        let registry = HardenedTheoremRegistry::with_limits(
            vec![
                definition("A", &[]),
                definition("B", &["A"]),
                definition("C", &["B"]),
            ],
            limits,
        )
        .unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![
                facet("C", "state:abc", FacetStatus::Established),
                facet("B", "state:abc", FacetStatus::Established),
            ],
        };

        assert!(matches!(
            registry.explain_establishment(&manifest, &id("C")),
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::ExplanationPathTheorems,
                ..
            })
        ));
    }

    #[test]
    fn facet_reference_limits_are_enforced() {
        let limits = QualificationLimits {
            max_evidence_refs_per_facet: 1,
            ..QualificationLimits::default()
        };
        let registry =
            HardenedTheoremRegistry::with_limits(vec![definition("Q0", &[])], limits).unwrap();
        let mut q0 = facet("Q0", "state:abc", FacetStatus::Established);
        q0.evidence_refs = vec!["evidence:1".into(), "evidence:2".into()];
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![q0],
        };

        assert!(matches!(
            registry.validate_manifest_shape(&manifest),
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::FacetEvidenceRefs,
                ..
            })
        ));
    }

    #[test]
    fn profile_and_subject_string_limits_are_enforced() {
        let limits = QualificationLimits {
            max_profile_id_bytes: 3,
            max_subject_commitment_bytes: 4,
            ..QualificationLimits::default()
        };
        let registry =
            HardenedTheoremRegistry::with_limits(vec![definition("Q0", &[])], limits).unwrap();

        let profile =
            QualificationRequirementProfile::established_only("LONG", vec![id("Q0")]);
        assert!(matches!(
            registry.validate_requirement_profile(&profile),
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::ProfileIdBytes,
                ..
            })
        ));

        let manifest = QualificationManifest {
            subject_commitment: "state".into(),
            facets: Vec::new(),
        };
        assert!(matches!(
            registry.validate_manifest_shape(&manifest),
            Err(QualificationHardeningError::LimitExceeded {
                resource: QualificationResource::SubjectCommitmentBytes,
                ..
            })
        ));
    }

    #[test]
    fn hardened_evaluation_matches_v01_semantics_for_valid_input() {
        let registry = registry();
        let manifest = QualificationManifest {
            subject_commitment: "state:abc".into(),
            facets: vec![facet(
                "Q0.SerializationSafety.v1",
                "state:abc",
                FacetStatus::Indeterminate,
            )],
        };
        let profile = QualificationRequirementProfile::established_only(
            "StrictDisplay.v1",
            vec![id("Q0.SerializationSafety.v1")],
        );

        let evaluation = registry.evaluate(&manifest, &profile).unwrap();
        assert!(!evaluation.satisfied);
        assert!(evaluation.missing_theorems.is_empty());
        assert_eq!(evaluation.unacceptable_facets.len(), 1);
        assert_eq!(
            evaluation.unacceptable_facets[0].status,
            FacetStatus::Indeterminate
        );
    }
}
