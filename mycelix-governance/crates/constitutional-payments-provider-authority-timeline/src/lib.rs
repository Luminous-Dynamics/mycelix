#![deny(unsafe_code)]

use constitutional_payments_provider_authority::{
    authorize_record, validate_rotation, HistoricalAuthorEvidence, ProviderAuthorGrant,
    ProviderAuthorityError, ProviderGrantStatus,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use thiserror::Error;

const MAX_ID_LEN: usize = 512;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum TimelineError {
    #[error("{0}")]
    Violation(String),
}

pub type TimelineResult<T> = Result<T, TimelineError>;

fn violation(message: impl Into<String>) -> TimelineError {
    TimelineError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str) -> TimelineResult<()> {
    if value.trim().is_empty() || value.len() > MAX_ID_LEN {
        return Err(violation(format!(
            "{label} must be non-empty and <= {MAX_ID_LEN} bytes"
        )));
    }
    Ok(())
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct GrantIssuanceEvidence {
    pub issuance_id: String,
    pub grant_commitment: String,
    pub issuer_capability_id: String,
    pub issuer_holder_id: String,
    pub issuer_jurisdiction: String,
    pub issuer_authority_commitment: String,
    pub issued_at_us: i64,
    /// Commitment to an external issuance proof. This crate validates binding
    /// and chronology only; it does not cryptographically verify the proof.
    pub proof_commitment: String,
}

impl GrantIssuanceEvidence {
    pub fn validate_for(&self, grant: &ProviderAuthorGrant) -> TimelineResult<()> {
        grant
            .validate()
            .map_err(|error| violation(format!("invalid provider author grant: {error}")))?;
        require_opaque("issuance id", &self.issuance_id)?;
        require_opaque("issuance grant commitment", &self.grant_commitment)?;
        require_opaque("issuer capability id", &self.issuer_capability_id)?;
        require_opaque("issuer holder id", &self.issuer_holder_id)?;
        require_opaque("issuer jurisdiction", &self.issuer_jurisdiction)?;
        require_opaque(
            "issuer authority commitment",
            &self.issuer_authority_commitment,
        )?;
        require_opaque("issuance proof commitment", &self.proof_commitment)?;

        if self.grant_commitment != grant.grant_commitment {
            return Err(violation(
                "issuance evidence does not bind exact grant commitment",
            ));
        }
        let anchor = &grant.upstream_authority;
        if self.issuer_capability_id != anchor.capability_id
            || self.issuer_holder_id != anchor.holder_id
            || self.issuer_jurisdiction != anchor.jurisdiction
            || self.issuer_authority_commitment != anchor.capability_commitment
        {
            return Err(violation(
                "issuance evidence does not bind exact upstream constitutional authority",
            ));
        }
        if self.issued_at_us > grant.valid_from_us {
            return Err(violation(
                "grant issuance cannot occur after grant validity begins",
            ));
        }
        if !anchor.valid_at(self.issued_at_us) {
            return Err(violation(
                "grant issuance occurred outside upstream constitutional validity",
            ));
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct GrantRevocationEvidence {
    pub revocation_id: String,
    pub grant_commitment: String,
    pub issuer_capability_id: String,
    pub issuer_holder_id: String,
    pub issuer_jurisdiction: String,
    pub issuer_authority_commitment: String,
    pub authorized_at_us: i64,
    pub effective_at_us: i64,
    /// Commitment to an external revocation proof. This crate validates binding
    /// and chronology only; it does not cryptographically verify the proof.
    pub proof_commitment: String,
}

impl GrantRevocationEvidence {
    pub fn validate_for(&self, grant: &ProviderAuthorGrant) -> TimelineResult<()> {
        grant
            .validate()
            .map_err(|error| violation(format!("invalid provider author grant: {error}")))?;
        require_opaque("revocation id", &self.revocation_id)?;
        require_opaque("revocation grant commitment", &self.grant_commitment)?;
        require_opaque("revocation issuer capability id", &self.issuer_capability_id)?;
        require_opaque("revocation issuer holder id", &self.issuer_holder_id)?;
        require_opaque("revocation issuer jurisdiction", &self.issuer_jurisdiction)?;
        require_opaque(
            "revocation issuer authority commitment",
            &self.issuer_authority_commitment,
        )?;
        require_opaque("revocation proof commitment", &self.proof_commitment)?;

        if self.grant_commitment != grant.grant_commitment {
            return Err(violation(
                "revocation evidence does not bind exact grant commitment",
            ));
        }
        let anchor = &grant.upstream_authority;
        if self.issuer_capability_id != anchor.capability_id
            || self.issuer_holder_id != anchor.holder_id
            || self.issuer_jurisdiction != anchor.jurisdiction
            || self.issuer_authority_commitment != anchor.capability_commitment
        {
            return Err(violation(
                "revocation evidence does not bind exact upstream constitutional authority",
            ));
        }
        if self.authorized_at_us > self.effective_at_us {
            return Err(violation(
                "revocation authorization cannot occur after its effective time",
            ));
        }
        if self.effective_at_us < grant.valid_from_us {
            return Err(violation(
                "revocation cannot become effective before grant validity begins",
            ));
        }
        if !anchor.valid_at(self.authorized_at_us) || !anchor.valid_at(self.effective_at_us) {
            return Err(violation(
                "revocation authority must be valid at authorization and effective time",
            ));
        }
        if let Some(expiry) = grant.expires_at_us {
            if self.effective_at_us > expiry {
                return Err(violation(
                    "revocation cannot become effective after grant expiry",
                ));
            }
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum AuthorityTimelineEventBody {
    GrantIssued {
        grant: ProviderAuthorGrant,
        issuance: GrantIssuanceEvidence,
    },
    GrantRevoked {
        grant_commitment: String,
        evidence: GrantRevocationEvidence,
    },
    GrantSuperseded {
        predecessor_grant_commitment: String,
        successor: ProviderAuthorGrant,
        issuance: GrantIssuanceEvidence,
        cutover_us: i64,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct AuthorityTimelineEvent {
    pub sequence: u64,
    pub event_id: String,
    pub body: AuthorityTimelineEventBody,
}

impl AuthorityTimelineEvent {
    fn validate_identity(&self) -> TimelineResult<()> {
        require_opaque("authority timeline event id", &self.event_id)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum TimelineFault {
    SameSequenceConflict { sequence: u64 },
    EventIdConflict { event_id: String },
    ConflictingIssuance { grant_commitment: String },
    CompetingSuccessor { predecessor_grant_commitment: String },
    ConflictingTerminalHistory { grant_commitment: String },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum TimelineRejection {
    InvalidCandidate(String),
    UnknownGrant(String),
    NonGenesisGrant,
    OutOfOrderSequence { expected: u64, actual: u64 },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum TimelineDecision {
    Applied,
    ExistingSame,
    Rejected(TimelineRejection),
    IntegrityHalted(TimelineFault),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum HistoricalGrantStatus {
    NotYetValid { valid_from_us: i64 },
    Active,
    Revoked { revocation_id: String, effective_at_us: i64 },
    Superseded {
        successor_grant_commitment: String,
        effective_at_us: i64,
    },
    Expired { expires_at_us: i64 },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
enum TerminalAuthorityEvent {
    Revoked(GrantRevocationEvidence),
    Superseded {
        successor_grant_commitment: String,
        effective_at_us: i64,
    },
}

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum TimelineAuthorizationError {
    #[error("authority timeline integrity halted: {0:?}")]
    IntegrityHalted(TimelineFault),
    #[error("unknown provider author grant: {0}")]
    UnknownGrant(String),
    #[error("provider author grant is not active at action time: {0:?}")]
    NotActive(HistoricalGrantStatus),
    #[error("provider authority rejected: {0}")]
    Provider(ProviderAuthorityError),
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ProviderAuthorityTimeline {
    events: BTreeMap<u64, AuthorityTimelineEvent>,
    event_ids: BTreeMap<String, u64>,
    grants: BTreeMap<String, ProviderAuthorGrant>,
    issuances: BTreeMap<String, GrantIssuanceEvidence>,
    terminal: BTreeMap<String, TerminalAuthorityEvent>,
    successor_by_predecessor: BTreeMap<String, String>,
    integrity_fault: Option<TimelineFault>,
}

impl ProviderAuthorityTimeline {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn events(&self) -> impl Iterator<Item = &AuthorityTimelineEvent> {
        self.events.values()
    }

    pub fn event_count(&self) -> usize {
        self.events.len()
    }

    pub fn integrity_fault(&self) -> Option<&TimelineFault> {
        self.integrity_fault.as_ref()
    }

    pub fn grant(&self, grant_commitment: &str) -> Option<&ProviderAuthorGrant> {
        self.grants.get(grant_commitment)
    }

    pub fn issuance(&self, grant_commitment: &str) -> Option<&GrantIssuanceEvidence> {
        self.issuances.get(grant_commitment)
    }

    pub fn successor_of(&self, predecessor_grant_commitment: &str) -> Option<&str> {
        self.successor_by_predecessor
            .get(predecessor_grant_commitment)
            .map(String::as_str)
    }

    fn halted(&self) -> Option<TimelineDecision> {
        self.integrity_fault
            .clone()
            .map(TimelineDecision::IntegrityHalted)
    }

    fn halt(&mut self, fault: TimelineFault) -> TimelineDecision {
        if self.integrity_fault.is_none() {
            self.integrity_fault = Some(fault.clone());
        }
        TimelineDecision::IntegrityHalted(
            self.integrity_fault
                .clone()
                .expect("integrity fault was just installed"),
        )
    }

    fn preflight_event(&mut self, candidate: &AuthorityTimelineEvent) -> Option<TimelineDecision> {
        if let Some(halted) = self.halted() {
            return Some(halted);
        }
        if let Err(error) = candidate.validate_identity() {
            return Some(TimelineDecision::Rejected(
                TimelineRejection::InvalidCandidate(error.to_string()),
            ));
        }
        if let Some(existing) = self.events.get(&candidate.sequence) {
            if existing == candidate {
                return Some(TimelineDecision::ExistingSame);
            }
            return Some(self.halt(TimelineFault::SameSequenceConflict {
                sequence: candidate.sequence,
            }));
        }
        if self.event_ids.contains_key(&candidate.event_id) {
            return Some(self.halt(TimelineFault::EventIdConflict {
                event_id: candidate.event_id.clone(),
            }));
        }
        let expected = self.events.len() as u64;
        if candidate.sequence != expected {
            return Some(TimelineDecision::Rejected(
                TimelineRejection::OutOfOrderSequence {
                    expected,
                    actual: candidate.sequence,
                },
            ));
        }
        None
    }

    fn commit_event(&mut self, event: AuthorityTimelineEvent) {
        self.event_ids.insert(event.event_id.clone(), event.sequence);
        self.events.insert(event.sequence, event);
    }

    pub fn issue_genesis(
        &mut self,
        sequence: u64,
        event_id: impl Into<String>,
        grant: ProviderAuthorGrant,
        issuance: GrantIssuanceEvidence,
    ) -> TimelineDecision {
        let event = AuthorityTimelineEvent {
            sequence,
            event_id: event_id.into(),
            body: AuthorityTimelineEventBody::GrantIssued {
                grant: grant.clone(),
                issuance: issuance.clone(),
            },
        };
        if let Some(decision) = self.preflight_event(&event) {
            return decision;
        }

        if grant.epoch != 1 || grant.predecessor_grant_commitment.is_some() {
            return TimelineDecision::Rejected(TimelineRejection::NonGenesisGrant);
        }
        if let Some(existing) = self.grants.get(&grant.grant_commitment) {
            if existing == &grant
                && self.issuances.get(&grant.grant_commitment) == Some(&issuance)
            {
                return TimelineDecision::ExistingSame;
            }
            return self.halt(TimelineFault::ConflictingIssuance {
                grant_commitment: grant.grant_commitment.clone(),
            });
        }
        if let Err(error) = issuance.validate_for(&grant) {
            return TimelineDecision::Rejected(TimelineRejection::InvalidCandidate(
                error.to_string(),
            ));
        }

        self.issuances
            .insert(grant.grant_commitment.clone(), issuance);
        self.grants.insert(grant.grant_commitment.clone(), grant);
        self.commit_event(event);
        TimelineDecision::Applied
    }

    pub fn rotate(
        &mut self,
        sequence: u64,
        event_id: impl Into<String>,
        predecessor_grant_commitment: impl Into<String>,
        successor: ProviderAuthorGrant,
        issuance: GrantIssuanceEvidence,
        cutover_us: i64,
    ) -> TimelineDecision {
        let predecessor_grant_commitment = predecessor_grant_commitment.into();
        let event = AuthorityTimelineEvent {
            sequence,
            event_id: event_id.into(),
            body: AuthorityTimelineEventBody::GrantSuperseded {
                predecessor_grant_commitment: predecessor_grant_commitment.clone(),
                successor: successor.clone(),
                issuance: issuance.clone(),
                cutover_us,
            },
        };
        if let Some(decision) = self.preflight_event(&event) {
            return decision;
        }

        let Some(predecessor) = self.grants.get(&predecessor_grant_commitment).cloned() else {
            return TimelineDecision::Rejected(TimelineRejection::UnknownGrant(
                predecessor_grant_commitment,
            ));
        };

        if let Some(existing_successor) = self
            .successor_by_predecessor
            .get(&predecessor_grant_commitment)
            .cloned()
        {
            if existing_successor != successor.grant_commitment {
                return self.halt(TimelineFault::CompetingSuccessor {
                    predecessor_grant_commitment,
                });
            }
            if self.grants.get(&existing_successor) != Some(&successor)
                || self.issuances.get(&existing_successor) != Some(&issuance)
            {
                return self.halt(TimelineFault::ConflictingIssuance {
                    grant_commitment: existing_successor,
                });
            }
            let exact_terminal = matches!(
                self.terminal.get(&predecessor_grant_commitment),
                Some(TerminalAuthorityEvent::Superseded {
                    successor_grant_commitment,
                    effective_at_us,
                }) if successor_grant_commitment == &successor.grant_commitment
                    && *effective_at_us == cutover_us
            );
            if exact_terminal {
                return TimelineDecision::ExistingSame;
            }
            return self.halt(TimelineFault::ConflictingTerminalHistory {
                grant_commitment: predecessor_grant_commitment,
            });
        }

        if self.terminal.contains_key(&predecessor_grant_commitment) {
            return self.halt(TimelineFault::ConflictingTerminalHistory {
                grant_commitment: predecessor_grant_commitment,
            });
        }
        if self.grants.contains_key(&successor.grant_commitment) {
            return self.halt(TimelineFault::ConflictingIssuance {
                grant_commitment: successor.grant_commitment.clone(),
            });
        }
        if let Err(error) = validate_rotation(&predecessor, &successor, cutover_us) {
            return TimelineDecision::Rejected(TimelineRejection::InvalidCandidate(format!(
                "invalid provider author rotation: {error}"
            )));
        }
        if let Some(expiry) = predecessor.expires_at_us {
            if cutover_us > expiry {
                return TimelineDecision::Rejected(TimelineRejection::InvalidCandidate(
                    "rotation cutover cannot occur after predecessor expiry".into(),
                ));
            }
        }
        if let Err(error) = issuance.validate_for(&successor) {
            return TimelineDecision::Rejected(TimelineRejection::InvalidCandidate(
                error.to_string(),
            ));
        }

        self.issuances
            .insert(successor.grant_commitment.clone(), issuance);
        self.grants
            .insert(successor.grant_commitment.clone(), successor.clone());
        self.successor_by_predecessor.insert(
            predecessor_grant_commitment.clone(),
            successor.grant_commitment.clone(),
        );
        self.terminal.insert(
            predecessor_grant_commitment,
            TerminalAuthorityEvent::Superseded {
                successor_grant_commitment: successor.grant_commitment,
                effective_at_us: cutover_us,
            },
        );
        self.commit_event(event);
        TimelineDecision::Applied
    }

    pub fn revoke(
        &mut self,
        sequence: u64,
        event_id: impl Into<String>,
        grant_commitment: impl Into<String>,
        evidence: GrantRevocationEvidence,
    ) -> TimelineDecision {
        let grant_commitment = grant_commitment.into();
        let event = AuthorityTimelineEvent {
            sequence,
            event_id: event_id.into(),
            body: AuthorityTimelineEventBody::GrantRevoked {
                grant_commitment: grant_commitment.clone(),
                evidence: evidence.clone(),
            },
        };
        if let Some(decision) = self.preflight_event(&event) {
            return decision;
        }

        let Some(grant) = self.grants.get(&grant_commitment).cloned() else {
            return TimelineDecision::Rejected(TimelineRejection::UnknownGrant(
                grant_commitment,
            ));
        };
        if let Some(existing) = self.terminal.get(&grant_commitment) {
            if existing == &TerminalAuthorityEvent::Revoked(evidence.clone()) {
                return TimelineDecision::ExistingSame;
            }
            return self.halt(TimelineFault::ConflictingTerminalHistory {
                grant_commitment,
            });
        }
        if let Err(error) = evidence.validate_for(&grant) {
            return TimelineDecision::Rejected(TimelineRejection::InvalidCandidate(
                error.to_string(),
            ));
        }

        self.terminal
            .insert(grant_commitment, TerminalAuthorityEvent::Revoked(evidence));
        self.commit_event(event);
        TimelineDecision::Applied
    }

    pub fn status_at(
        &self,
        grant_commitment: &str,
        action_time_us: i64,
    ) -> Result<HistoricalGrantStatus, TimelineRejection> {
        let Some(grant) = self.grants.get(grant_commitment) else {
            return Err(TimelineRejection::UnknownGrant(grant_commitment.into()));
        };
        if action_time_us < grant.valid_from_us {
            return Ok(HistoricalGrantStatus::NotYetValid {
                valid_from_us: grant.valid_from_us,
            });
        }
        if let Some(terminal) = self.terminal.get(grant_commitment) {
            match terminal {
                TerminalAuthorityEvent::Revoked(evidence)
                    if action_time_us >= evidence.effective_at_us =>
                {
                    return Ok(HistoricalGrantStatus::Revoked {
                        revocation_id: evidence.revocation_id.clone(),
                        effective_at_us: evidence.effective_at_us,
                    });
                }
                TerminalAuthorityEvent::Superseded {
                    successor_grant_commitment,
                    effective_at_us,
                } if action_time_us >= *effective_at_us => {
                    return Ok(HistoricalGrantStatus::Superseded {
                        successor_grant_commitment: successor_grant_commitment.clone(),
                        effective_at_us: *effective_at_us,
                    });
                }
                _ => {}
            }
        }
        if let Some(expiry) = grant.expires_at_us {
            if action_time_us >= expiry {
                return Ok(HistoricalGrantStatus::Expired {
                    expires_at_us: expiry,
                });
            }
        }
        Ok(HistoricalGrantStatus::Active)
    }

    pub fn authorize_at(
        &self,
        grant_commitment: &str,
        expected_provider_profile_id: &str,
        expected_provider_profile_commitment: &str,
        expected_jurisdiction: &str,
        evidence: &HistoricalAuthorEvidence,
    ) -> Result<(), TimelineAuthorizationError> {
        if let Some(fault) = &self.integrity_fault {
            return Err(TimelineAuthorizationError::IntegrityHalted(fault.clone()));
        }
        let Some(grant) = self.grants.get(grant_commitment) else {
            return Err(TimelineAuthorizationError::UnknownGrant(
                grant_commitment.into(),
            ));
        };
        let historical = self
            .status_at(grant_commitment, evidence.action_time_us)
            .map_err(|_| TimelineAuthorizationError::UnknownGrant(grant_commitment.into()))?;
        let provider_status = match historical.clone() {
            HistoricalGrantStatus::Active => ProviderGrantStatus::Active,
            HistoricalGrantStatus::Revoked {
                revocation_id,
                effective_at_us,
            } => ProviderGrantStatus::Revoked {
                revocation_id,
                effective_at_us,
            },
            HistoricalGrantStatus::Superseded {
                successor_grant_commitment,
                effective_at_us,
            } => ProviderGrantStatus::Superseded {
                successor_grant_commitment,
                effective_at_us,
            },
            HistoricalGrantStatus::NotYetValid { .. } | HistoricalGrantStatus::Expired { .. } => {
                return Err(TimelineAuthorizationError::NotActive(historical));
            }
        };
        authorize_record(
            grant,
            &provider_status,
            expected_provider_profile_id,
            expected_provider_profile_commitment,
            expected_jurisdiction,
            evidence,
        )
        .map_err(TimelineAuthorizationError::Provider)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use constitutional_payments_provider_authority::{
        ProviderAuthorMode, ProviderAuthorRole, PublicFundsAuthorityAnchor, RecordAuthorProof,
    };

    const PROFILE_ID: &str = "payments-provider-v1";
    const PROFILE_COMMITMENT: &str = "provider-profile-commitment-1";
    const JURISDICTION: &str = "municipality:example";

    fn anchor() -> PublicFundsAuthorityAnchor {
        PublicFundsAuthorityAnchor {
            capability_id: "cap-execute-appropriation".into(),
            holder_id: "stewardship-office".into(),
            jurisdiction: JURISDICTION.into(),
            capability_commitment: "capability-commitment-1".into(),
            valid_from_us: 100,
            expires_at_us: Some(10_000),
        }
    }

    fn grant(
        grant_id: &str,
        epoch: u64,
        valid_from_us: i64,
        predecessor: Option<String>,
        author_id: &str,
        roles: impl IntoIterator<Item = ProviderAuthorRole>,
    ) -> ProviderAuthorGrant {
        ProviderAuthorGrant::new(
            grant_id,
            epoch,
            PROFILE_ID,
            PROFILE_COMMITMENT,
            JURISDICTION,
            anchor(),
            roles,
            ProviderAuthorMode::DirectAuthor {
                author_id: author_id.into(),
            },
            valid_from_us,
            Some(9_000),
            predecessor,
        )
        .unwrap()
    }

    fn issuance(
        grant: &ProviderAuthorGrant,
        issuance_id: &str,
        proof: &str,
    ) -> GrantIssuanceEvidence {
        GrantIssuanceEvidence {
            issuance_id: issuance_id.into(),
            grant_commitment: grant.grant_commitment.clone(),
            issuer_capability_id: grant.upstream_authority.capability_id.clone(),
            issuer_holder_id: grant.upstream_authority.holder_id.clone(),
            issuer_jurisdiction: grant.upstream_authority.jurisdiction.clone(),
            issuer_authority_commitment: grant.upstream_authority.capability_commitment.clone(),
            issued_at_us: grant.valid_from_us - 1,
            proof_commitment: proof.into(),
        }
    }

    fn revocation(
        grant: &ProviderAuthorGrant,
        revocation_id: &str,
        effective_at_us: i64,
    ) -> GrantRevocationEvidence {
        GrantRevocationEvidence {
            revocation_id: revocation_id.into(),
            grant_commitment: grant.grant_commitment.clone(),
            issuer_capability_id: grant.upstream_authority.capability_id.clone(),
            issuer_holder_id: grant.upstream_authority.holder_id.clone(),
            issuer_jurisdiction: grant.upstream_authority.jurisdiction.clone(),
            issuer_authority_commitment: grant.upstream_authority.capability_commitment.clone(),
            authorized_at_us: effective_at_us - 1,
            effective_at_us,
            proof_commitment: format!("{revocation_id}-proof"),
        }
    }

    fn author_evidence(
        grant: &ProviderAuthorGrant,
        role: ProviderAuthorRole,
        author_id: &str,
        action_time_us: i64,
    ) -> HistoricalAuthorEvidence {
        HistoricalAuthorEvidence {
            grant_commitment: grant.grant_commitment.clone(),
            grant_epoch: grant.epoch,
            action_time_us,
            role,
            record_commitment: "journal-record-commitment-1".into(),
            proof: RecordAuthorProof::Direct {
                action_author_id: author_id.into(),
            },
        }
    }

    fn provider_grant() -> ProviderAuthorGrant {
        grant(
            "provider-grant-1",
            1,
            200,
            None,
            "did:mycelix:payments-provider",
            [ProviderAuthorRole::ProviderObservation],
        )
    }

    fn orchestrator_grant() -> ProviderAuthorGrant {
        grant(
            "orchestrator-grant-1",
            1,
            200,
            None,
            "did:mycelix:orchestrator-1",
            [
                ProviderAuthorRole::OrchestratorIntent,
                ProviderAuthorRole::OrchestratorDispatch,
                ProviderAuthorRole::OperationIndex,
            ],
        )
    }

    fn rotated_grant(
        predecessor: &ProviderAuthorGrant,
        grant_id: &str,
        author_id: &str,
        cutover_us: i64,
    ) -> ProviderAuthorGrant {
        grant(
            grant_id,
            predecessor.epoch + 1,
            cutover_us,
            Some(predecessor.grant_commitment.clone()),
            author_id,
            predecessor.roles.clone(),
        )
    }

    fn issue(
        timeline: &mut ProviderAuthorityTimeline,
        grant: &ProviderAuthorGrant,
    ) -> TimelineDecision {
        timeline.issue_genesis(
            0,
            "event-0",
            grant.clone(),
            issuance(grant, "issuance-1", "issuance-proof-1"),
        )
    }

    #[test]
    fn genesis_requires_bound_issuance_and_is_append_only() {
        let grant = provider_grant();
        let evidence = issuance(&grant, "issuance-1", "issuance-proof-1");
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(
            timeline.issue_genesis(0, "event-0", grant.clone(), evidence.clone()),
            TimelineDecision::Applied
        );
        assert_eq!(timeline.event_count(), 1);
        assert_eq!(timeline.grant(&grant.grant_commitment), Some(&grant));
        assert_eq!(timeline.issuance(&grant.grant_commitment), Some(&evidence));
    }

    #[test]
    fn exact_event_replay_is_idempotent() {
        let grant = provider_grant();
        let evidence = issuance(&grant, "issuance-1", "issuance-proof-1");
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(
            timeline.issue_genesis(0, "event-0", grant.clone(), evidence.clone()),
            TimelineDecision::Applied
        );
        assert_eq!(
            timeline.issue_genesis(0, "event-0", grant, evidence),
            TimelineDecision::ExistingSame
        );
        assert_eq!(timeline.event_count(), 1);
    }

    #[test]
    fn same_sequence_different_event_halts() {
        let first = provider_grant();
        let second = orchestrator_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);
        assert!(matches!(
            timeline.issue_genesis(
                0,
                "different-event",
                second.clone(),
                issuance(&second, "issuance-2", "issuance-proof-2"),
            ),
            TimelineDecision::IntegrityHalted(TimelineFault::SameSequenceConflict { sequence: 0 })
        ));
    }

    #[test]
    fn event_id_reuse_at_new_sequence_halts() {
        let first = provider_grant();
        let second = orchestrator_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);
        assert!(matches!(
            timeline.issue_genesis(
                1,
                "event-0",
                second.clone(),
                issuance(&second, "issuance-2", "issuance-proof-2"),
            ),
            TimelineDecision::IntegrityHalted(TimelineFault::EventIdConflict { .. })
        ));
    }

    #[test]
    fn conflicting_issuance_for_same_grant_halts_integrity() {
        let grant = provider_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &grant), TimelineDecision::Applied);
        let conflicting = issuance(&grant, "issuance-2", "different-proof");
        assert!(matches!(
            timeline.issue_genesis(1, "event-1", grant, conflicting),
            TimelineDecision::IntegrityHalted(TimelineFault::ConflictingIssuance { .. })
        ));
    }

    #[test]
    fn issuance_after_validity_start_is_rejected() {
        let grant = provider_grant();
        let mut evidence = issuance(&grant, "issuance-1", "issuance-proof-1");
        evidence.issued_at_us = grant.valid_from_us + 1;
        let mut timeline = ProviderAuthorityTimeline::new();
        assert!(matches!(
            timeline.issue_genesis(0, "event-0", grant, evidence),
            TimelineDecision::Rejected(TimelineRejection::InvalidCandidate(_))
        ));
        assert_eq!(timeline.event_count(), 0);
        assert!(timeline.integrity_fault().is_none());
    }

    #[test]
    fn rotation_has_one_canonical_successor() {
        let first = orchestrator_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);

        let second = rotated_grant(&first, "orchestrator-grant-2", "did:mycelix:orchestrator-2", 1_000);
        let second_issuance = issuance(&second, "issuance-2", "issuance-proof-2");
        assert_eq!(
            timeline.rotate(
                1,
                "event-1",
                first.grant_commitment.clone(),
                second.clone(),
                second_issuance,
                1_000,
            ),
            TimelineDecision::Applied
        );
        assert_eq!(
            timeline.successor_of(&first.grant_commitment),
            Some(second.grant_commitment.as_str())
        );

        let competing = rotated_grant(
            &first,
            "orchestrator-grant-2b",
            "did:mycelix:orchestrator-2b",
            1_000,
        );
        assert!(matches!(
            timeline.rotate(
                2,
                "event-2",
                first.grant_commitment.clone(),
                competing.clone(),
                issuance(&competing, "issuance-2b", "issuance-proof-2b"),
                1_000,
            ),
            TimelineDecision::IntegrityHalted(TimelineFault::CompetingSuccessor { .. })
        ));
    }

    #[test]
    fn same_successor_with_different_cutover_halts() {
        let first = orchestrator_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);
        let second = rotated_grant(&first, "orchestrator-grant-2", "did:mycelix:orchestrator-2", 1_000);
        let second_issuance = issuance(&second, "issuance-2", "issuance-proof-2");
        assert_eq!(
            timeline.rotate(
                1,
                "event-1",
                first.grant_commitment.clone(),
                second.clone(),
                second_issuance.clone(),
                1_000,
            ),
            TimelineDecision::Applied
        );
        assert!(matches!(
            timeline.rotate(
                2,
                "event-2",
                first.grant_commitment,
                second,
                second_issuance,
                1_001,
            ),
            TimelineDecision::IntegrityHalted(TimelineFault::ConflictingTerminalHistory { .. })
        ));
    }

    #[test]
    fn exact_semantic_rotation_replay_with_new_event_is_idempotent() {
        let first = orchestrator_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);
        let second = rotated_grant(&first, "orchestrator-grant-2", "did:mycelix:orchestrator-2", 1_000);
        let second_issuance = issuance(&second, "issuance-2", "issuance-proof-2");
        assert_eq!(
            timeline.rotate(
                1,
                "event-1",
                first.grant_commitment.clone(),
                second.clone(),
                second_issuance.clone(),
                1_000,
            ),
            TimelineDecision::Applied
        );
        assert_eq!(
            timeline.rotate(
                2,
                "event-2",
                first.grant_commitment,
                second,
                second_issuance,
                1_000,
            ),
            TimelineDecision::ExistingSame
        );
        assert_eq!(timeline.event_count(), 2);
    }

    #[test]
    fn rotation_cannot_amplify_roles() {
        let first = orchestrator_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);
        let amplified = grant(
            "orchestrator-grant-2",
            2,
            1_000,
            Some(first.grant_commitment.clone()),
            "did:mycelix:orchestrator-2",
            [
                ProviderAuthorRole::OrchestratorIntent,
                ProviderAuthorRole::OrchestratorDispatch,
                ProviderAuthorRole::OperationIndex,
                ProviderAuthorRole::ProviderObservation,
            ],
        );
        let amplified_issuance = issuance(&amplified, "issuance-2", "issuance-proof-2");
        assert!(matches!(
            timeline.rotate(
                1,
                "event-1",
                first.grant_commitment,
                amplified,
                amplified_issuance,
                1_000,
            ),
            TimelineDecision::Rejected(TimelineRejection::InvalidCandidate(_))
        ));
        assert!(timeline.integrity_fault().is_none());
    }

    #[test]
    fn revocation_is_historical_not_retroactive() {
        let grant = provider_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &grant), TimelineDecision::Applied);
        assert_eq!(
            timeline.revoke(
                1,
                "event-1",
                grant.grant_commitment.clone(),
                revocation(&grant, "revoke-1", 1_000),
            ),
            TimelineDecision::Applied
        );
        assert_eq!(
            timeline.status_at(&grant.grant_commitment, 999).unwrap(),
            HistoricalGrantStatus::Active
        );
        assert!(matches!(
            timeline.status_at(&grant.grant_commitment, 1_000).unwrap(),
            HistoricalGrantStatus::Revoked { .. }
        ));
    }

    #[test]
    fn supersession_is_historical_not_retroactive() {
        let first = orchestrator_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);
        let second = rotated_grant(&first, "orchestrator-grant-2", "did:mycelix:orchestrator-2", 1_000);
        assert_eq!(
            timeline.rotate(
                1,
                "event-1",
                first.grant_commitment.clone(),
                second.clone(),
                issuance(&second, "issuance-2", "issuance-proof-2"),
                1_000,
            ),
            TimelineDecision::Applied
        );
        assert_eq!(
            timeline.status_at(&first.grant_commitment, 999).unwrap(),
            HistoricalGrantStatus::Active
        );
        assert!(matches!(
            timeline.status_at(&first.grant_commitment, 1_000).unwrap(),
            HistoricalGrantStatus::Superseded { .. }
        ));
        assert_eq!(
            timeline.status_at(&second.grant_commitment, 1_000).unwrap(),
            HistoricalGrantStatus::Active
        );
    }

    #[test]
    fn conflicting_terminal_history_halts() {
        let first = provider_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &first), TimelineDecision::Applied);
        assert_eq!(
            timeline.revoke(
                1,
                "event-1",
                first.grant_commitment.clone(),
                revocation(&first, "revoke-1", 1_000),
            ),
            TimelineDecision::Applied
        );
        let successor = rotated_grant(
            &first,
            "provider-grant-2",
            "did:mycelix:payments-provider-2",
            1_500,
        );
        assert!(matches!(
            timeline.rotate(
                2,
                "event-2",
                first.grant_commitment.clone(),
                successor.clone(),
                issuance(&successor, "issuance-2", "issuance-proof-2"),
                1_500,
            ),
            TimelineDecision::IntegrityHalted(TimelineFault::ConflictingTerminalHistory { .. })
        ));
    }

    #[test]
    fn authorize_at_composes_timeline_with_role_aware_authority() {
        let grant = provider_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &grant), TimelineDecision::Applied);
        assert_eq!(
            timeline.revoke(
                1,
                "event-1",
                grant.grant_commitment.clone(),
                revocation(&grant, "revoke-1", 1_000),
            ),
            TimelineDecision::Applied
        );

        let before = author_evidence(
            &grant,
            ProviderAuthorRole::ProviderObservation,
            "did:mycelix:payments-provider",
            999,
        );
        timeline
            .authorize_at(
                &grant.grant_commitment,
                PROFILE_ID,
                PROFILE_COMMITMENT,
                JURISDICTION,
                &before,
            )
            .unwrap();

        let at = author_evidence(
            &grant,
            ProviderAuthorRole::ProviderObservation,
            "did:mycelix:payments-provider",
            1_000,
        );
        assert!(timeline
            .authorize_at(
                &grant.grant_commitment,
                PROFILE_ID,
                PROFILE_COMMITMENT,
                JURISDICTION,
                &at,
            )
            .is_err());
    }

    #[test]
    fn out_of_order_event_is_rejected_without_halting() {
        let grant = provider_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert!(matches!(
            timeline.issue_genesis(
                1,
                "event-1",
                grant.clone(),
                issuance(&grant, "issuance-1", "issuance-proof-1"),
            ),
            TimelineDecision::Rejected(TimelineRejection::OutOfOrderSequence {
                expected: 0,
                actual: 1,
            })
        ));
        assert!(timeline.integrity_fault().is_none());
        assert_eq!(timeline.event_count(), 0);
    }

    #[test]
    fn integrity_halt_is_sticky_and_blocks_authorization() {
        let grant = provider_grant();
        let mut timeline = ProviderAuthorityTimeline::new();
        assert_eq!(issue(&mut timeline, &grant), TimelineDecision::Applied);
        let conflicting = issuance(&grant, "issuance-2", "different-proof");
        assert!(matches!(
            timeline.issue_genesis(1, "event-1", grant.clone(), conflicting),
            TimelineDecision::IntegrityHalted(_)
        ));
        assert!(matches!(
            timeline.revoke(
                1,
                "event-after-halt",
                grant.grant_commitment.clone(),
                revocation(&grant, "revoke-1", 1_000),
            ),
            TimelineDecision::IntegrityHalted(_)
        ));
        let historical = author_evidence(
            &grant,
            ProviderAuthorRole::ProviderObservation,
            "did:mycelix:payments-provider",
            500,
        );
        assert!(matches!(
            timeline.authorize_at(
                &grant.grant_commitment,
                PROFILE_ID,
                PROFILE_COMMITMENT,
                JURISDICTION,
                &historical,
            ),
            Err(TimelineAuthorizationError::IntegrityHalted(_))
        ));
        assert_eq!(timeline.event_count(), 1);
    }
}
