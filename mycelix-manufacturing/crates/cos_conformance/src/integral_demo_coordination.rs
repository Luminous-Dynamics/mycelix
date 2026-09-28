//! A1 generalized cybernetic coordination-loop reference model.
//!
//! Extracts a domain-neutral coordination primitive from Integral's five-system
//! loop. It models semantic artifacts and explicit transitions, not execution,
//! governance legitimacy, economic validity, or production behavior.
//!
//! Claim ceiling: ReferenceModelOnly.

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CoordinationKind {
    Intent,
    Decision,
    Design,
    Authorization,
    ExecutionIntent,
    Observation,
    Assessment,
    Recommendation,
    HumanDisposition,
    Revision,
    Appeal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordinationOrigin {
    Local,
    Foreign,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Disposition {
    Accepted,
    Rejected,
    Deferred,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoordinationArtifact {
    pub id: &'static str,
    pub kind: CoordinationKind,
    pub origin: CoordinationOrigin,
    pub generation: u32,
    pub source_ref: &'static str,
    pub parent_ref: Option<&'static str>,
    /// Evidence and authority are separate references with separate meanings.
    pub evidence_ref: Option<&'static str>,
    pub authority_ref: Option<&'static str>,
    pub disposition: Option<Disposition>,
    pub uncertainty_present: bool,
    pub challengeable: bool,
    pub reversible: bool,
    pub recovery_ref: Option<&'static str>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoordinationError {
    EmptyIdentity,
    DuplicateIdentity,
    MissingParent,
    InvalidGeneration,
    InvalidTransition,
    MissingAuthority,
    AuthorityOnRecommendation,
    MissingDisposition,
    MissingEvidence,
    UncertaintyLoss,
    OriginMutation,
    UnchallengeableConsequence,
    MissingRecovery,
    InvalidRevision,
    AppealNotIndependent,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CoordinationLoop {
    pub artifacts: Vec<CoordinationArtifact>,
}

fn allowed(from: CoordinationKind, to: CoordinationKind) -> bool {
    matches!(
        (from, to),
        (CoordinationKind::Intent, CoordinationKind::Decision)
            | (CoordinationKind::Decision, CoordinationKind::Design)
            | (CoordinationKind::Design, CoordinationKind::Authorization)
            | (CoordinationKind::Authorization, CoordinationKind::ExecutionIntent)
            | (CoordinationKind::ExecutionIntent, CoordinationKind::Observation)
            | (CoordinationKind::Observation, CoordinationKind::Assessment)
            | (CoordinationKind::Assessment, CoordinationKind::Recommendation)
            | (CoordinationKind::Recommendation, CoordinationKind::HumanDisposition)
            | (CoordinationKind::HumanDisposition, CoordinationKind::Revision)
            | (CoordinationKind::Revision, CoordinationKind::Decision)
            | (_, CoordinationKind::Appeal)
    )
}

/// Validate explicit parent-linked artifacts in canonical append order.
/// This validates the model's declared semantics, not the truth of its evidence.
pub fn validate_loop(loop_: &CoordinationLoop) -> Result<(), CoordinationError> {
    let artifacts = &loop_.artifacts;
    if artifacts.is_empty() {
        return Err(CoordinationError::EmptyIdentity);
    }

    for (index, artifact) in artifacts.iter().enumerate() {
        if artifact.id.trim().is_empty() || artifact.source_ref.trim().is_empty() {
            return Err(CoordinationError::EmptyIdentity);
        }
        if artifacts[..index].iter().any(|prior| prior.id == artifact.id) {
            return Err(CoordinationError::DuplicateIdentity);
        }
        if artifact.kind == CoordinationKind::Recommendation && artifact.authority_ref.is_some() {
            return Err(CoordinationError::AuthorityOnRecommendation);
        }
        if matches!(artifact.kind, CoordinationKind::Decision | CoordinationKind::Authorization | CoordinationKind::HumanDisposition)
            && artifact.authority_ref.map_or(true, str::is_empty)
        {
            return Err(CoordinationError::MissingAuthority);
        }
        if artifact.kind == CoordinationKind::HumanDisposition && artifact.disposition.is_none() {
            return Err(CoordinationError::MissingDisposition);
        }
        if matches!(artifact.kind, CoordinationKind::Observation | CoordinationKind::Assessment)
            && artifact.evidence_ref.map_or(true, str::is_empty)
        {
            return Err(CoordinationError::MissingEvidence);
        }
        if artifact.kind == CoordinationKind::HumanDisposition && artifact.parent_ref.is_none() {
            return Err(CoordinationError::MissingParent);
        }
        if artifact.kind == CoordinationKind::Revision
            && (artifact.generation == 0 || artifact.parent_ref.is_none())
        {
            return Err(CoordinationError::InvalidRevision);
        }

        if let Some(parent_id) = artifact.parent_ref {
            let parent = artifacts[..index]
                .iter()
                .find(|candidate| candidate.id == parent_id)
                .ok_or(CoordinationError::MissingParent)?;
            if artifact.generation < parent.generation {
                return Err(CoordinationError::InvalidGeneration);
            }
            if !allowed(parent.kind, artifact.kind) {
                return Err(CoordinationError::InvalidTransition);
            }
            if parent.uncertainty_present && !artifact.uncertainty_present {
                return Err(CoordinationError::UncertaintyLoss);
            }
            // Only evidence-bearing descendants are required to retain the
            // source origin. Human governance may act locally on foreign facts.
            if matches!(artifact.kind, CoordinationKind::Observation | CoordinationKind::Assessment)
                && parent.origin == CoordinationOrigin::Foreign
                && artifact.origin != CoordinationOrigin::Foreign
            {
                return Err(CoordinationError::OriginMutation);
            }
            if artifact.kind == CoordinationKind::ExecutionIntent
                && (artifact.authority_ref.is_none() || parent.kind != CoordinationKind::Authorization)
            {
                return Err(CoordinationError::MissingAuthority);
            }
            if artifact.kind == CoordinationKind::Revision
                && artifact.generation <= parent.generation
            {
                return Err(CoordinationError::InvalidRevision);
            }
            if artifact.kind == CoordinationKind::HumanDisposition
                && parent.kind != CoordinationKind::Recommendation
            {
                return Err(CoordinationError::InvalidTransition);
            }
        }
        if artifact.kind == CoordinationKind::Recommendation && artifact.authority_ref.is_some() {
            return Err(CoordinationError::AuthorityOnRecommendation);
        }
        if artifact.kind == CoordinationKind::HumanDisposition
            && artifact.disposition == Some(Disposition::Accepted)
            && !artifact.challengeable
        {
            return Err(CoordinationError::UnchallengeableConsequence);
        }
        if artifact.kind == CoordinationKind::HumanDisposition
            && !artifact.reversible
            && artifact.recovery_ref.map_or(true, str::is_empty)
        {
            return Err(CoordinationError::MissingRecovery);
        }
        if artifact.kind == CoordinationKind::Appeal
            && artifact.parent_ref.is_none()
        {
            return Err(CoordinationError::AppealNotIndependent);
        }
    }
    Ok(())
}

/// An explicit recommendation remains advisory until a human disposition exists.
pub fn recommendation_has_governance_disposition(
    recommendation_id: &str,
    artifacts: &[CoordinationArtifact],
) -> bool {
    artifacts.iter().any(|artifact| {
        artifact.kind == CoordinationKind::HumanDisposition
            && artifact.parent_ref == Some(recommendation_id)
            && artifact.disposition.is_some()
            && artifact.authority_ref.is_some()
    })
}

/// A successful observation is evidence of an observation only, never a credential.
pub fn observation_is_not_qualification(artifact: &CoordinationArtifact) -> bool {
    artifact.kind == CoordinationKind::Observation && artifact.authority_ref.is_none()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(id: &'static str, kind: CoordinationKind, parent_ref: Option<&'static str>) -> CoordinationArtifact {
        CoordinationArtifact {
            id, kind, origin: CoordinationOrigin::Local, generation: 1,
            source_ref: "source://fixture", parent_ref,
            evidence_ref: if matches!(kind, CoordinationKind::Observation | CoordinationKind::Assessment) { Some("evidence://1") } else { None },
            authority_ref: if matches!(kind, CoordinationKind::Decision | CoordinationKind::Authorization | CoordinationKind::HumanDisposition | CoordinationKind::ExecutionIntent) { Some("authority://human") } else { None },
            disposition: if kind == CoordinationKind::HumanDisposition { Some(Disposition::Accepted) } else { None },
            uncertainty_present: true, challengeable: true, reversible: true, recovery_ref: None,
        }
    }

    fn valid_loop() -> CoordinationLoop {
        CoordinationLoop { artifacts: vec![
            item("i1", CoordinationKind::Intent, None),
            item("d1", CoordinationKind::Decision, Some("i1")),
            item("g1", CoordinationKind::Design, Some("d1")),
            item("a1", CoordinationKind::Authorization, Some("g1")),
            item("x1", CoordinationKind::ExecutionIntent, Some("a1")),
            item("o1", CoordinationKind::Observation, Some("x1")),
            item("s1", CoordinationKind::Assessment, Some("o1")),
            item("r1", CoordinationKind::Recommendation, Some("s1")),
            item("h1", CoordinationKind::HumanDisposition, Some("r1")),
            item("v2", CoordinationKind::Revision, Some("h1")),
        ]}
    }

    #[test]
    fn complete_loop_requires_explicit_semantic_stages() {
        let mut loop_ = valid_loop();
        loop_.artifacts[9].generation = 2;
        assert_eq!(validate_loop(&loop_), Ok(()));
    }

    #[test]
    fn recommendation_cannot_carry_authority() {
        let mut loop_ = valid_loop();
        loop_.artifacts[7].authority_ref = Some("authority://hidden");
        assert_eq!(validate_loop(&loop_), Err(CoordinationError::AuthorityOnRecommendation));
    }

    #[test]
    fn missing_decision_authority_fails_closed() {
        let mut loop_ = valid_loop();
        loop_.artifacts[1].authority_ref = None;
        assert_eq!(validate_loop(&loop_), Err(CoordinationError::MissingAuthority));
    }

    #[test]
    fn missing_human_disposition_fails_closed() {
        let mut loop_ = valid_loop();
        loop_.artifacts[8].disposition = None;
        assert_eq!(validate_loop(&loop_), Err(CoordinationError::MissingDisposition));
    }

    #[test]
    fn stale_generation_cannot_move_backwards() {
        let mut loop_ = valid_loop();
        loop_.artifacts[9].generation = 0;
        assert_eq!(validate_loop(&loop_), Err(CoordinationError::InvalidGeneration));
    }

    #[test]
    fn uncertainty_cannot_be_silently_dropped() {
        let mut loop_ = valid_loop();
        loop_.artifacts[6].uncertainty_present = false;
        assert_eq!(validate_loop(&loop_), Err(CoordinationError::UncertaintyLoss));
    }

    #[test]
    fn foreign_evidence_origin_is_not_laundered() {
        let mut loop_ = valid_loop();
        loop_.artifacts[5].origin = CoordinationOrigin::Foreign;
        loop_.artifacts[6].origin = CoordinationOrigin::Local;
        assert_eq!(validate_loop(&loop_), Err(CoordinationError::OriginMutation));
    }

    #[test]
    fn observation_does_not_establish_qualification() {
        assert!(observation_is_not_qualification(&item("o", CoordinationKind::Observation, None)));
    }

    #[test]
    fn recommendation_requires_explicit_disposition_for_governance() {
        let loop_ = valid_loop();
        assert!(recommendation_has_governance_disposition("r1", &loop_.artifacts));
        assert!(!recommendation_has_governance_disposition("r-missing", &loop_.artifacts));
    }

    #[test]
    fn duplicate_artifact_identity_fails_closed() {
        let mut loop_ = valid_loop();
        loop_.artifacts[9].id = "h1";
        assert_eq!(validate_loop(&loop_), Err(CoordinationError::DuplicateIdentity));
    }

    #[test]
    fn rejected_or_deferred_disposition_is_not_accepted_as_execution_authority() {
        let mut loop_ = valid_loop();
        loop_.artifacts[8].disposition = Some(Disposition::Deferred);
        assert_eq!(validate_loop(&loop_), Ok(()));
        assert!(!recommendation_has_governance_disposition("r1", &loop_.artifacts));
    }
}
