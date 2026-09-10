// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Bounded, non-authoritative analysis of DID-side and external-key-side
//! lifecycle evidence for one key-binding lineage.
//!
//! The analyzer intentionally does not produce a "current key", trust score, or
//! winner. It surfaces graph structure and preserves conflicts for policy/human
//! interpretation after the underlying evidence has been independently verified.

use std::collections::{HashMap, HashSet, VecDeque};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::KeyDidBindingArtifact;

/// Maximum lifecycle observations accepted in one analysis call.
pub const MAX_BINDING_LIFECYCLE_OBSERVATIONS: usize = 4_096;
/// Maximum UTF-8 byte length for a source evidence reference.
pub const MAX_LIFECYCLE_EVIDENCE_REF_LEN: usize = 1_024;

/// Authority plane from which a normalized lifecycle observation originated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LifecyclePlane {
    /// Mycelix DID-controller/Holochain evidence.
    DidController,
    /// Xenia external-key cryptographic evidence.
    ExternalKey,
}

/// Normalized lifecycle semantics shared across the two evidence planes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LifecycleDisposition {
    /// The plane withdrew/disavowed this exact binding.
    Withdraw,
    /// The plane proposed continuity to a successor binding.
    Supersede,
}

/// One normalized observation produced after a caller has verified its source plane.
///
/// This type is deliberately called an `Observation`, not `Verified*`: this crate
/// cannot prove the caller actually performed Holochain/Xenia verification.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BindingLifecycleObservation {
    /// Source authority plane.
    pub plane: LifecyclePlane,
    /// Exact binding affected by the observation.
    pub target: KeyDidBindingArtifact,
    /// Withdrawal or supersession.
    pub disposition: LifecycleDisposition,
    /// Successor for `Supersede`; absent for `Withdraw`.
    pub replacement: Option<KeyDidBindingArtifact>,
    /// Opaque reference to the independently verified source evidence.
    pub evidence_ref: String,
}

impl BindingLifecycleObservation {
    /// Construct and validate a normalized lifecycle observation.
    pub fn new(
        plane: LifecyclePlane,
        target: KeyDidBindingArtifact,
        disposition: LifecycleDisposition,
        replacement: Option<KeyDidBindingArtifact>,
        evidence_ref: impl Into<String>,
    ) -> Result<Self, BindingLifecycleAnalysisError> {
        let observation = Self {
            plane,
            target,
            disposition,
            replacement,
            evidence_ref: evidence_ref.into(),
        };
        observation.validate()?;
        Ok(observation)
    }

    /// Validate normalized observation structure.
    pub fn validate(&self) -> Result<(), BindingLifecycleAnalysisError> {
        self.target
            .validate()
            .map_err(|error| BindingLifecycleAnalysisError::InvalidBinding(error.to_string()))?;
        validate_evidence_ref(&self.evidence_ref)?;

        match (self.disposition, &self.replacement) {
            (LifecycleDisposition::Withdraw, None) => Ok(()),
            (LifecycleDisposition::Withdraw, Some(_)) => {
                Err(BindingLifecycleAnalysisError::WithdrawalHasReplacement)
            }
            (LifecycleDisposition::Supersede, None) => {
                Err(BindingLifecycleAnalysisError::SupersessionMissingReplacement)
            }
            (LifecycleDisposition::Supersede, Some(replacement)) => {
                replacement.validate().map_err(|error| {
                    BindingLifecycleAnalysisError::InvalidBinding(error.to_string())
                })?;
                if replacement.did != self.target.did {
                    return Err(BindingLifecycleAnalysisError::ReplacementDidMismatch);
                }
                if replacement.purpose != self.target.purpose {
                    return Err(BindingLifecycleAnalysisError::ReplacementPurposeMismatch);
                }
                if replacement.scope != self.target.scope {
                    return Err(BindingLifecycleAnalysisError::ReplacementScopeMismatch);
                }
                if replacement == &self.target {
                    return Err(BindingLifecycleAnalysisError::NoOpSupersession);
                }
                Ok(())
            }
        }
    }
}

/// One source binding with more than one distinct observed successor.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BindingFork {
    /// Forking source binding.
    pub source: KeyDidBindingArtifact,
    /// Distinct proposed successors, deterministically ordered by canonical bytes.
    pub successors: Vec<KeyDidBindingArtifact>,
}

/// A source binding for which DID-controller and external-key planes expressed
/// incompatible lifecycle semantics.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CrossPlaneDisagreement {
    /// Binding on which the planes disagree.
    pub source: KeyDidBindingArtifact,
    /// DID-controller observations affecting this source.
    pub did_controller: Vec<BindingLifecycleObservation>,
    /// External-key observations affecting this source.
    pub external_key: Vec<BindingLifecycleObservation>,
}

/// A successor relation independently observed from both authority planes.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CrossPlaneSupersessionAgreement {
    /// Source binding.
    pub source: KeyDidBindingArtifact,
    /// Successor named by both planes.
    pub successor: KeyDidBindingArtifact,
}

/// Structural report over the lifecycle evidence reachable from one origin binding.
///
/// `unwithdrawn_leaf_candidates` means exactly what it says: no reachable outgoing
/// supersession and no reachable withdrawal was observed for those leaves. It does
/// **not** mean active, valid, trusted, authorized, or current.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BindingLifecycleReport {
    /// Binding from which reachability was evaluated.
    pub origin: KeyDidBindingArtifact,
    /// Number of distinct reachable binding nodes.
    pub reachable_binding_count: usize,
    /// Reachable bindings with at least one DID-side withdrawal observation.
    pub did_withdrawn: Vec<KeyDidBindingArtifact>,
    /// Reachable bindings with at least one external-key withdrawal observation.
    pub key_withdrawn: Vec<KeyDidBindingArtifact>,
    /// Sources with multiple distinct successors.
    pub forks: Vec<BindingFork>,
    /// Sources where both planes proposed the exact same successor.
    pub cross_plane_supersession_agreements: Vec<CrossPlaneSupersessionAgreement>,
    /// Sources where both planes spoke but their normalized lifecycle sets differed.
    pub cross_plane_disagreements: Vec<CrossPlaneDisagreement>,
    /// Whether a reachable supersession cycle exists.
    pub cycle_detected: bool,
    /// Unwithdrawn leaves, only populated when the reachable supersession graph is acyclic.
    pub unwithdrawn_leaf_candidates: Vec<KeyDidBindingArtifact>,
}

/// Analyze only lifecycle evidence transitively reachable from `origin`.
///
/// Unrelated graph material is validated for basic bounded shape but is otherwise ignored
/// so graph stuffing cannot change the report for an unrelated binding lineage.
pub fn analyze_binding_lifecycle(
    origin: &KeyDidBindingArtifact,
    observations: &[BindingLifecycleObservation],
) -> Result<BindingLifecycleReport, BindingLifecycleAnalysisError> {
    origin
        .validate()
        .map_err(|error| BindingLifecycleAnalysisError::InvalidBinding(error.to_string()))?;
    if observations.len() > MAX_BINDING_LIFECYCLE_OBSERVATIONS {
        return Err(BindingLifecycleAnalysisError::TooManyObservations {
            count: observations.len(),
            max: MAX_BINDING_LIFECYCLE_OBSERVATIONS,
        });
    }

    let origin_key = binding_key(origin)?;
    let mut by_target: HashMap<Vec<u8>, Vec<&BindingLifecycleObservation>> = HashMap::new();
    let mut seen_refs = HashSet::new();
    for observation in observations {
        observation.validate()?;
        if !seen_refs.insert(observation.evidence_ref.as_str()) {
            return Err(BindingLifecycleAnalysisError::DuplicateEvidenceRef(
                observation.evidence_ref.clone(),
            ));
        }
        by_target
            .entry(binding_key(&observation.target)?)
            .or_default()
            .push(observation);
    }

    let mut reachable_keys = HashSet::new();
    let mut nodes: HashMap<Vec<u8>, KeyDidBindingArtifact> = HashMap::new();
    let mut queue = VecDeque::from([origin_key.clone()]);
    nodes.insert(origin_key.clone(), origin.clone());

    while let Some(source_key) = queue.pop_front() {
        if !reachable_keys.insert(source_key.clone()) {
            continue;
        }
        for observation in by_target.get(&source_key).into_iter().flatten() {
            if observation.disposition != LifecycleDisposition::Supersede {
                continue;
            }
            let replacement = observation
                .replacement
                .as_ref()
                .expect("validated supersession has replacement");
            let replacement_key = binding_key(replacement)?;
            nodes
                .entry(replacement_key.clone())
                .or_insert_with(|| replacement.clone());
            if !reachable_keys.contains(&replacement_key) {
                queue.push_back(replacement_key);
            }
        }
    }

    let mut successors: HashMap<Vec<u8>, HashSet<Vec<u8>>> = HashMap::new();
    let mut did_withdrawn_keys = HashSet::new();
    let mut key_withdrawn_keys = HashSet::new();
    for source_key in &reachable_keys {
        for observation in by_target.get(source_key).into_iter().flatten() {
            match observation.disposition {
                LifecycleDisposition::Withdraw => match observation.plane {
                    LifecyclePlane::DidController => {
                        did_withdrawn_keys.insert(source_key.clone());
                    }
                    LifecyclePlane::ExternalKey => {
                        key_withdrawn_keys.insert(source_key.clone());
                    }
                },
                LifecycleDisposition::Supersede => {
                    let replacement = observation
                        .replacement
                        .as_ref()
                        .expect("validated supersession has replacement");
                    successors
                        .entry(source_key.clone())
                        .or_default()
                        .insert(binding_key(replacement)?);
                }
            }
        }
    }

    let cycle_detected = has_reachable_cycle(&reachable_keys, &successors);

    let mut forks = Vec::new();
    for (source_key, successor_keys) in &successors {
        if successor_keys.len() <= 1 {
            continue;
        }
        let mut successor_artifacts = successor_keys
            .iter()
            .filter_map(|key| nodes.get(key).cloned())
            .collect::<Vec<_>>();
        sort_bindings(&mut successor_artifacts)?;
        if let Some(source) = nodes.get(source_key) {
            forks.push(BindingFork {
                source: source.clone(),
                successors: successor_artifacts,
            });
        }
    }
    sort_forks(&mut forks)?;

    let mut agreements = Vec::new();
    let mut disagreements = Vec::new();
    for source_key in &reachable_keys {
        let source_observations = by_target.get(source_key).cloned().unwrap_or_default();
        let did = source_observations
            .iter()
            .filter(|observation| observation.plane == LifecyclePlane::DidController)
            .copied()
            .collect::<Vec<_>>();
        let key = source_observations
            .iter()
            .filter(|observation| observation.plane == LifecyclePlane::ExternalKey)
            .copied()
            .collect::<Vec<_>>();
        if did.is_empty() || key.is_empty() {
            continue;
        }

        let did_semantics = normalized_semantics(&did)?;
        let key_semantics = normalized_semantics(&key)?;
        if did_semantics == key_semantics {
            for semantic in &did_semantics {
                if let Semantic::Supersede(successor_key) = semantic {
                    if let (Some(source), Some(successor)) =
                        (nodes.get(source_key), nodes.get(successor_key))
                    {
                        agreements.push(CrossPlaneSupersessionAgreement {
                            source: source.clone(),
                            successor: successor.clone(),
                        });
                    }
                }
            }
        } else if let Some(source) = nodes.get(source_key) {
            let mut did_controller = did.into_iter().cloned().collect::<Vec<_>>();
            let mut external_key = key.into_iter().cloned().collect::<Vec<_>>();
            sort_observations(&mut did_controller)?;
            sort_observations(&mut external_key)?;
            disagreements.push(CrossPlaneDisagreement {
                source: source.clone(),
                did_controller,
                external_key,
            });
        }
    }
    sort_agreements(&mut agreements)?;
    sort_disagreements(&mut disagreements)?;

    let mut did_withdrawn = did_withdrawn_keys
        .iter()
        .filter_map(|key| nodes.get(key).cloned())
        .collect::<Vec<_>>();
    let mut key_withdrawn = key_withdrawn_keys
        .iter()
        .filter_map(|key| nodes.get(key).cloned())
        .collect::<Vec<_>>();
    sort_bindings(&mut did_withdrawn)?;
    sort_bindings(&mut key_withdrawn)?;

    let mut unwithdrawn_leaf_candidates = Vec::new();
    if !cycle_detected {
        for node_key in &reachable_keys {
            let has_successor = successors
                .get(node_key)
                .is_some_and(|set| !set.is_empty());
            let withdrawn = did_withdrawn_keys.contains(node_key) || key_withdrawn_keys.contains(node_key);
            if !has_successor && !withdrawn {
                if let Some(node) = nodes.get(node_key) {
                    unwithdrawn_leaf_candidates.push(node.clone());
                }
            }
        }
        sort_bindings(&mut unwithdrawn_leaf_candidates)?;
    }

    Ok(BindingLifecycleReport {
        origin: origin.clone(),
        reachable_binding_count: reachable_keys.len(),
        did_withdrawn,
        key_withdrawn,
        forks,
        cross_plane_supersession_agreements: agreements,
        cross_plane_disagreements: disagreements,
        cycle_detected,
        unwithdrawn_leaf_candidates,
    })
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
enum Semantic {
    Withdraw,
    Supersede(Vec<u8>),
}

fn normalized_semantics(
    observations: &[&BindingLifecycleObservation],
) -> Result<HashSet<Semantic>, BindingLifecycleAnalysisError> {
    let mut semantics = HashSet::new();
    for observation in observations {
        match observation.disposition {
            LifecycleDisposition::Withdraw => {
                semantics.insert(Semantic::Withdraw);
            }
            LifecycleDisposition::Supersede => {
                let replacement = observation
                    .replacement
                    .as_ref()
                    .expect("validated supersession has replacement");
                semantics.insert(Semantic::Supersede(binding_key(replacement)?));
            }
        }
    }
    Ok(semantics)
}

fn has_reachable_cycle(
    reachable: &HashSet<Vec<u8>>,
    successors: &HashMap<Vec<u8>, HashSet<Vec<u8>>>,
) -> bool {
    let mut indegree = reachable
        .iter()
        .map(|key| (key.clone(), 0usize))
        .collect::<HashMap<_, _>>();
    for (source, targets) in successors {
        if !reachable.contains(source) {
            continue;
        }
        for target in targets {
            if let Some(value) = indegree.get_mut(target) {
                *value += 1;
            }
        }
    }
    let mut queue = indegree
        .iter()
        .filter_map(|(key, degree)| (*degree == 0).then_some(key.clone()))
        .collect::<VecDeque<_>>();
    let mut removed = 0usize;
    while let Some(source) = queue.pop_front() {
        removed += 1;
        for target in successors.get(&source).into_iter().flatten() {
            if let Some(degree) = indegree.get_mut(target) {
                *degree -= 1;
                if *degree == 0 {
                    queue.push_back(target.clone());
                }
            }
        }
    }
    removed != reachable.len()
}

fn binding_key(binding: &KeyDidBindingArtifact) -> Result<Vec<u8>, BindingLifecycleAnalysisError> {
    binding
        .canonical_bytes()
        .map_err(|error| BindingLifecycleAnalysisError::InvalidBinding(error.to_string()))
}

fn validate_evidence_ref(reference: &str) -> Result<(), BindingLifecycleAnalysisError> {
    if reference.trim().is_empty() {
        return Err(BindingLifecycleAnalysisError::EmptyEvidenceRef);
    }
    if reference.len() > MAX_LIFECYCLE_EVIDENCE_REF_LEN {
        return Err(BindingLifecycleAnalysisError::EvidenceRefTooLong {
            max: MAX_LIFECYCLE_EVIDENCE_REF_LEN,
            found: reference.len(),
        });
    }
    if reference.trim() != reference {
        return Err(BindingLifecycleAnalysisError::NonCanonicalEvidenceRef);
    }
    Ok(())
}

fn sort_bindings(bindings: &mut [KeyDidBindingArtifact]) -> Result<(), BindingLifecycleAnalysisError> {
    let mut keyed = bindings
        .iter()
        .cloned()
        .map(|binding| Ok((binding_key(&binding)?, binding)))
        .collect::<Result<Vec<_>, BindingLifecycleAnalysisError>>()?;
    keyed.sort_by(|a, b| a.0.cmp(&b.0));
    for (slot, (_, binding)) in bindings.iter_mut().zip(keyed) {
        *slot = binding;
    }
    Ok(())
}

fn sort_observations(
    observations: &mut [BindingLifecycleObservation],
) -> Result<(), BindingLifecycleAnalysisError> {
    let mut keyed = observations
        .iter()
        .cloned()
        .map(|observation| {
            let target = binding_key(&observation.target)?;
            let replacement = observation
                .replacement
                .as_ref()
                .map(binding_key)
                .transpose()?
                .unwrap_or_default();
            Ok((
                (
                    plane_tag(observation.plane),
                    disposition_tag(observation.disposition),
                    target,
                    replacement,
                    observation.evidence_ref.clone(),
                ),
                observation,
            ))
        })
        .collect::<Result<Vec<_>, BindingLifecycleAnalysisError>>()?;
    keyed.sort_by(|a, b| a.0.cmp(&b.0));
    for (slot, (_, observation)) in observations.iter_mut().zip(keyed) {
        *slot = observation;
    }
    Ok(())
}

fn sort_forks(forks: &mut [BindingFork]) -> Result<(), BindingLifecycleAnalysisError> {
    let mut keyed = forks
        .iter()
        .cloned()
        .map(|fork| Ok((binding_key(&fork.source)?, fork)))
        .collect::<Result<Vec<_>, BindingLifecycleAnalysisError>>()?;
    keyed.sort_by(|a, b| a.0.cmp(&b.0));
    for (slot, (_, fork)) in forks.iter_mut().zip(keyed) {
        *slot = fork;
    }
    Ok(())
}

fn sort_agreements(
    agreements: &mut [CrossPlaneSupersessionAgreement],
) -> Result<(), BindingLifecycleAnalysisError> {
    let mut keyed = agreements
        .iter()
        .cloned()
        .map(|agreement| {
            Ok(((binding_key(&agreement.source)?, binding_key(&agreement.successor)?), agreement))
        })
        .collect::<Result<Vec<_>, BindingLifecycleAnalysisError>>()?;
    keyed.sort_by(|a, b| a.0.cmp(&b.0));
    for (slot, (_, agreement)) in agreements.iter_mut().zip(keyed) {
        *slot = agreement;
    }
    Ok(())
}

fn sort_disagreements(
    disagreements: &mut [CrossPlaneDisagreement],
) -> Result<(), BindingLifecycleAnalysisError> {
    let mut keyed = disagreements
        .iter()
        .cloned()
        .map(|disagreement| Ok((binding_key(&disagreement.source)?, disagreement)))
        .collect::<Result<Vec<_>, BindingLifecycleAnalysisError>>()?;
    keyed.sort_by(|a, b| a.0.cmp(&b.0));
    for (slot, (_, disagreement)) in disagreements.iter_mut().zip(keyed) {
        *slot = disagreement;
    }
    Ok(())
}

const fn plane_tag(plane: LifecyclePlane) -> u8 {
    match plane {
        LifecyclePlane::DidController => 0,
        LifecyclePlane::ExternalKey => 1,
    }
}

const fn disposition_tag(disposition: LifecycleDisposition) -> u8 {
    match disposition {
        LifecycleDisposition::Withdraw => 0,
        LifecycleDisposition::Supersede => 1,
    }
}

/// Structural-analysis errors. None imply fraud or invalid identity.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum BindingLifecycleAnalysisError {
    /// A binding failed canonical validation.
    #[error("invalid key-DID binding: {0}")]
    InvalidBinding(String),
    /// Too many observations were supplied for one bounded analysis call.
    #[error("too many lifecycle observations: found {count}, maximum {max}")]
    TooManyObservations {
        /// Observed count.
        count: usize,
        /// Maximum allowed count.
        max: usize,
    },
    /// Evidence references are required for provenance.
    #[error("lifecycle evidence reference must not be empty")]
    EmptyEvidenceRef,
    /// Evidence reference exceeded its bound.
    #[error("lifecycle evidence reference exceeds maximum {max}: found {found}")]
    EvidenceRefTooLong {
        /// Maximum UTF-8 byte length.
        max: usize,
        /// Observed UTF-8 byte length.
        found: usize,
    },
    /// Evidence references must be whitespace-canonical.
    #[error("lifecycle evidence reference must not have leading/trailing whitespace")]
    NonCanonicalEvidenceRef,
    /// Reusing one evidence reference for multiple normalized observations is ambiguous.
    #[error("duplicate lifecycle evidence reference: {0}")]
    DuplicateEvidenceRef(String),
    /// Withdrawal observations cannot name a successor.
    #[error("withdrawal observation must not include a replacement")]
    WithdrawalHasReplacement,
    /// Supersession observations require a successor.
    #[error("supersession observation requires a replacement")]
    SupersessionMissingReplacement,
    /// Successor DID must match source DID.
    #[error("replacement DID mismatch")]
    ReplacementDidMismatch,
    /// Successor purpose must match source purpose.
    #[error("replacement purpose mismatch")]
    ReplacementPurposeMismatch,
    /// Successor scope must match source scope.
    #[error("replacement scope mismatch")]
    ReplacementScopeMismatch,
    /// A binding cannot supersede itself.
    #[error("no-op supersession is invalid")]
    NoOpSupersession,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::KeyAssociationPurpose;

    fn binding(byte: u8) -> KeyDidBindingArtifact {
        KeyDidBindingArtifact::new(
            "did:mycelix:uhCAk-test-agent",
            [byte; 32],
            "ed25519-rfc8032",
            KeyAssociationPurpose::EvidenceAttestor,
            "symthaea-generativity-provenance",
        )
        .unwrap()
    }

    fn supersede(
        plane: LifecyclePlane,
        source: KeyDidBindingArtifact,
        target: KeyDidBindingArtifact,
        reference: &str,
    ) -> BindingLifecycleObservation {
        BindingLifecycleObservation::new(
            plane,
            source,
            LifecycleDisposition::Supersede,
            Some(target),
            reference,
        )
        .unwrap()
    }

    fn withdraw(
        plane: LifecyclePlane,
        target: KeyDidBindingArtifact,
        reference: &str,
    ) -> BindingLifecycleObservation {
        BindingLifecycleObservation::new(
            plane,
            target,
            LifecycleDisposition::Withdraw,
            None,
            reference,
        )
        .unwrap()
    }

    #[test]
    fn no_evidence_means_unwithdrawn_leaf_not_active() {
        let origin = binding(1);
        let report = analyze_binding_lifecycle(&origin, &[]).unwrap();
        assert!(!report.cycle_detected);
        assert_eq!(report.unwithdrawn_leaf_candidates, vec![origin]);
        assert!(report.did_withdrawn.is_empty());
        assert!(report.key_withdrawn.is_empty());
    }

    #[test]
    fn either_plane_withdrawal_removes_leaf_candidate() {
        let origin = binding(1);
        let report = analyze_binding_lifecycle(
            &origin,
            &[withdraw(LifecyclePlane::ExternalKey, origin.clone(), "xenia:1")],
        )
        .unwrap();
        assert!(report.unwithdrawn_leaf_candidates.is_empty());
        assert_eq!(report.key_withdrawn, vec![origin]);
    }

    #[test]
    fn matching_cross_plane_supersession_is_agreement() {
        let a = binding(1);
        let b = binding(2);
        let observations = vec![
            supersede(LifecyclePlane::DidController, a.clone(), b.clone(), "hc:1"),
            supersede(LifecyclePlane::ExternalKey, a.clone(), b.clone(), "xenia:1"),
        ];
        let report = analyze_binding_lifecycle(&a, &observations).unwrap();
        assert_eq!(report.cross_plane_supersession_agreements.len(), 1);
        assert!(report.cross_plane_disagreements.is_empty());
        assert_eq!(report.unwithdrawn_leaf_candidates, vec![b]);
    }

    #[test]
    fn divergent_cross_plane_supersession_is_preserved() {
        let a = binding(1);
        let b = binding(2);
        let c = binding(3);
        let observations = vec![
            supersede(LifecyclePlane::DidController, a.clone(), b, "hc:1"),
            supersede(LifecyclePlane::ExternalKey, a.clone(), c, "xenia:1"),
        ];
        let report = analyze_binding_lifecycle(&a, &observations).unwrap();
        assert_eq!(report.forks.len(), 1);
        assert_eq!(report.cross_plane_disagreements.len(), 1);
        assert_eq!(report.unwithdrawn_leaf_candidates.len(), 2);
    }

    #[test]
    fn withdraw_vs_supersede_is_cross_plane_disagreement() {
        let a = binding(1);
        let b = binding(2);
        let observations = vec![
            withdraw(LifecyclePlane::DidController, a.clone(), "hc:withdraw"),
            supersede(LifecyclePlane::ExternalKey, a.clone(), b, "xenia:rotate"),
        ];
        let report = analyze_binding_lifecycle(&a, &observations).unwrap();
        assert_eq!(report.cross_plane_disagreements.len(), 1);
    }

    #[test]
    fn reachable_cycle_suppresses_leaf_resolution() {
        let a = binding(1);
        let b = binding(2);
        let observations = vec![
            supersede(LifecyclePlane::DidController, a.clone(), b.clone(), "hc:1"),
            supersede(LifecyclePlane::DidController, b, a.clone(), "hc:2"),
        ];
        let report = analyze_binding_lifecycle(&a, &observations).unwrap();
        assert!(report.cycle_detected);
        assert!(report.unwithdrawn_leaf_candidates.is_empty());
    }

    #[test]
    fn unrelated_cycle_does_not_poison_origin() {
        let a = binding(1);
        let b = binding(2);
        let unrelated_a = binding(8);
        let unrelated_b = binding(9);
        let observations = vec![
            supersede(LifecyclePlane::DidController, a.clone(), b.clone(), "hc:main"),
            supersede(
                LifecyclePlane::DidController,
                unrelated_a.clone(),
                unrelated_b.clone(),
                "hc:u1",
            ),
            supersede(
                LifecyclePlane::DidController,
                unrelated_b,
                unrelated_a,
                "hc:u2",
            ),
        ];
        let report = analyze_binding_lifecycle(&a, &observations).unwrap();
        assert!(!report.cycle_detected);
        assert_eq!(report.unwithdrawn_leaf_candidates, vec![b]);
    }

    #[test]
    fn duplicate_evidence_reference_fails_closed() {
        let a = binding(1);
        let b = binding(2);
        let observations = vec![
            supersede(LifecyclePlane::DidController, a.clone(), b.clone(), "same"),
            supersede(LifecyclePlane::ExternalKey, a.clone(), b, "same"),
        ];
        assert!(matches!(
            analyze_binding_lifecycle(&a, &observations),
            Err(BindingLifecycleAnalysisError::DuplicateEvidenceRef(_))
        ));
    }
}
