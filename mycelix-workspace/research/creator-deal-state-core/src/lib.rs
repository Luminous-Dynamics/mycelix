//! Evidence-backed creator deal lifecycle state machine.
//!
//! This crate deliberately does not model payment settlement. Accepted
//! deliverables are workflow facts, not proof that money reached the creator.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

pub const DEAL_SCHEMA_VERSION: u16 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DealState {
    Draft,
    TermsProposed,
    Agreed,
    InProgress,
    AwaitingAcceptance,
    Disputed,
    Accepted,
    Cancelled,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PartyRole {
    Creator,
    Sponsor,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DealTerms {
    pub version: u32,
    /// Positive monetary compensation in minor currency units for this deal.
    pub fee_minor_units: u64,
    /// Three uppercase ASCII letters; this is syntax validation, not ISO registry validation.
    pub currency: String,
    /// Opaque reference to the exact terms document/version being approved.
    pub terms_document_ref: String,
    /// Opaque reference to the approved rights/usage schedule.
    pub rights_schedule_ref: String,
    /// Expected sequential deliverables in this version of the deal.
    pub deliverable_count: u16,
    pub disclosure_required: bool,
    /// Explicit creator/sponsor agreement to synthetic voice, face, or likeness use.
    pub synthetic_likeness_allowed: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DealAction {
    ProposeTerms { terms: DealTerms },
    ApproveTerms { terms_version: u32 },
    StartWork,
    SubmitDeliverable {
        deliverable_id: String,
        final_for_deal: bool,
    },
    AcceptDeliverable { deliverable_id: String },
    RejectDeliverable {
        deliverable_id: String,
        reason_ref: String,
    },
    RaiseDispute { reason_ref: String },
    Cancel { reason_ref: String },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DealEvent {
    pub event_id: String,
    /// Unix timestamp in seconds (UTC). Imported histories must be applied in event order.
    pub occurred_at_utc_seconds: i64,
    /// Application-supplied stable identity of the acting party; this crate does not authenticate it.
    pub actor_party_id: String,
    /// Evidence source for this transition. Opaque, non-empty reference; not independently verified.
    pub evidence_ref: String,
    pub action: DealAction,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct PendingDeliverable {
    deliverable_id: String,
    final_for_deal: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DealWorkflow {
    deal_id: String,
    creator_party_id: String,
    sponsor_party_id: String,
    state: DealState,
    current_terms: Option<DealTerms>,
    creator_approved_version: Option<u32>,
    sponsor_approved_version: Option<u32>,
    pending_deliverable: Option<PendingDeliverable>,
    accepted_deliverable_ids: Vec<String>,
    event_by_id: BTreeMap<String, DealEvent>,
    history: Vec<DealEvent>,
    last_event_time: Option<i64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AppendOutcome {
    Appended,
    IdenticalReplay,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DealError {
    EmptyField(&'static str),
    SamePartyIds,
    UnknownActor(String),
    InvalidCurrency(String),
    InvalidTermsVersion { expected: u32, actual: u32 },
    InvalidFee,
    InvalidDeliverableCount,
    InvalidTransition { state: DealState, action: &'static str },
    WrongActor { required: PartyRole },
    WrongTermsVersion { current: u32, requested: u32 },
    PartyAlreadyApproved { role: PartyRole, version: u32 },
    TermsNotFullyApproved,
    UnknownDeliverable(String),
    WrongDeliverable {
        pending: String,
        supplied: String,
    },
    ConflictingReplay(String),
    NonMonotonicEventTime { previous: i64, incoming: i64 },
}

impl fmt::Display for DealError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyField(field) => write!(f, "{field} must not be empty"),
            Self::SamePartyIds => write!(f, "creator and sponsor must have distinct party IDs"),
            Self::UnknownActor(actor) => write!(f, "actor is not a party to this deal: {actor:?}"),
            Self::InvalidCurrency(currency) => write!(
                f,
                "currency must be three uppercase ASCII letters: {currency:?}"
            ),
            Self::InvalidTermsVersion { expected, actual } => write!(
                f,
                "terms version must be {expected}, got {actual}"
            ),
            Self::InvalidFee => write!(f, "fee must be positive for this paid-deal model"),
            Self::InvalidDeliverableCount => write!(f, "deliverable_count must be positive"),
            Self::InvalidTransition { state, action } => {
                write!(f, "action {action} is invalid from state {state:?}")
            }
            Self::WrongActor { required } => {
                write!(f, "action requires actor role {required:?}")
            }
            Self::WrongTermsVersion { current, requested } => write!(
                f,
                "approval is for terms version {requested}, current version is {current}"
            ),
            Self::PartyAlreadyApproved { role, version } => write!(
                f,
                "party role {role:?} already approved terms version {version}"
            ),
            Self::TermsNotFullyApproved => {
                write!(f, "work cannot start until both parties approve the same terms version")
            }
            Self::UnknownDeliverable(id) => write!(f, "unknown or non-pending deliverable: {id:?}"),
            Self::WrongDeliverable { pending, supplied } => write!(
                f,
                "pending deliverable is {pending:?}, not {supplied:?}"
            ),
            Self::ConflictingReplay(id) => {
                write!(f, "event ID {id:?} was replayed with different content")
            }
            Self::NonMonotonicEventTime { previous, incoming } => write!(
                f,
                "event timestamp {incoming} is earlier than the previous event timestamp {previous}"
            ),
        }
    }
}

impl Error for DealError {}

impl DealWorkflow {
    pub fn new(
        deal_id: impl Into<String>,
        creator_party_id: impl Into<String>,
        sponsor_party_id: impl Into<String>,
    ) -> Result<Self, DealError> {
        let deal_id = deal_id.into();
        let creator_party_id = creator_party_id.into();
        let sponsor_party_id = sponsor_party_id.into();
        require_non_empty(&deal_id, "deal_id")?;
        require_non_empty(&creator_party_id, "creator_party_id")?;
        require_non_empty(&sponsor_party_id, "sponsor_party_id")?;
        if creator_party_id == sponsor_party_id {
            return Err(DealError::SamePartyIds);
        }

        Ok(Self {
            deal_id,
            creator_party_id,
            sponsor_party_id,
            state: DealState::Draft,
            current_terms: None,
            creator_approved_version: None,
            sponsor_approved_version: None,
            pending_deliverable: None,
            accepted_deliverable_ids: Vec::new(),
            event_by_id: BTreeMap::new(),
            history: Vec::new(),
            last_event_time: None,
        })
    }

    pub fn deal_id(&self) -> &str {
        &self.deal_id
    }

    pub fn state(&self) -> DealState {
        self.state
    }

    pub fn current_terms(&self) -> Option<&DealTerms> {
        self.current_terms.as_ref()
    }

    pub fn pending_deliverable_id(&self) -> Option<&str> {
        self.pending_deliverable
            .as_ref()
            .map(|item| item.deliverable_id.as_str())
    }

    pub fn accepted_deliverable_ids(&self) -> &[String] {
        &self.accepted_deliverable_ids
    }

    pub fn history(&self) -> &[DealEvent] {
        &self.history
    }

    pub fn approvals(&self) -> (Option<u32>, Option<u32>) {
        (self.creator_approved_version, self.sponsor_approved_version)
    }

    pub fn append(&mut self, event: DealEvent) -> Result<AppendOutcome, DealError> {
        require_non_empty(&event.event_id, "event_id")?;
        require_non_empty(&event.actor_party_id, "actor_party_id")?;
        require_non_empty(&event.evidence_ref, "evidence_ref")?;

        if let Some(existing) = self.event_by_id.get(&event.event_id) {
            return if existing == &event {
                Ok(AppendOutcome::IdenticalReplay)
            } else {
                Err(DealError::ConflictingReplay(event.event_id))
            };
        }

        if let Some(previous) = self.last_event_time {
            if event.occurred_at_utc_seconds < previous {
                return Err(DealError::NonMonotonicEventTime {
                    previous,
                    incoming: event.occurred_at_utc_seconds,
                });
            }
        }

        // Apply to a clone so any failed transition leaves the original state and
        // event journal unchanged. No partial approval/state mutation on errors.
        let mut next = self.clone();
        next.apply_action(&event)?;
        next.last_event_time = Some(event.occurred_at_utc_seconds);
        next.event_by_id.insert(event.event_id.clone(), event.clone());
        next.history.push(event);
        *self = next;
        Ok(AppendOutcome::Appended)
    }

    fn apply_action(&mut self, event: &DealEvent) -> Result<(), DealError> {
        let role = self.role_for_actor(&event.actor_party_id)?;
        match &event.action {
            DealAction::ProposeTerms { terms } => {
                self.require_state(&[DealState::Draft, DealState::TermsProposed, DealState::Agreed], "ProposeTerms")?;
                validate_terms(terms)?;
                let expected_version = match &self.current_terms {
                    None => 1,
                    Some(current) => current
                        .version
                        .checked_add(1)
                        .ok_or(DealError::InvalidTermsVersion {
                            expected: u32::MAX,
                            actual: terms.version,
                        })?,
                };
                if terms.version != expected_version {
                    return Err(DealError::InvalidTermsVersion {
                        expected: expected_version,
                        actual: terms.version,
                    });
                }
                self.current_terms = Some(terms.clone());
                self.creator_approved_version = None;
                self.sponsor_approved_version = None;
                self.state = DealState::TermsProposed;
                Ok(())
            }
            DealAction::ApproveTerms { terms_version } => {
                self.require_state(&[DealState::TermsProposed], "ApproveTerms")?;
                let current_version = self
                    .current_terms
                    .as_ref()
                    .ok_or(DealError::TermsNotFullyApproved)?
                    .version;
                if *terms_version != current_version {
                    return Err(DealError::WrongTermsVersion {
                        current: current_version,
                        requested: *terms_version,
                    });
                }
                match role {
                    PartyRole::Creator => {
                        if self.creator_approved_version == Some(*terms_version) {
                            return Err(DealError::PartyAlreadyApproved {
                                role,
                                version: *terms_version,
                            });
                        }
                        self.creator_approved_version = Some(*terms_version);
                    }
                    PartyRole::Sponsor => {
                        if self.sponsor_approved_version == Some(*terms_version) {
                            return Err(DealError::PartyAlreadyApproved {
                                role,
                                version: *terms_version,
                            });
                        }
                        self.sponsor_approved_version = Some(*terms_version);
                    }
                }
                if self.creator_approved_version == Some(*terms_version)
                    && self.sponsor_approved_version == Some(*terms_version)
                {
                    self.state = DealState::Agreed;
                }
                Ok(())
            }
            DealAction::StartWork => {
                self.require_state(&[DealState::Agreed], "StartWork")?;
                require_role(role, PartyRole::Creator)?;
                let current_version = self
                    .current_terms
                    .as_ref()
                    .ok_or(DealError::TermsNotFullyApproved)?
                    .version;
                if self.creator_approved_version != Some(current_version)
                    || self.sponsor_approved_version != Some(current_version)
                {
                    return Err(DealError::TermsNotFullyApproved);
                }
                self.state = DealState::InProgress;
                Ok(())
            }
            DealAction::SubmitDeliverable {
                deliverable_id,
                final_for_deal,
            } => {
                self.require_state(&[DealState::InProgress], "SubmitDeliverable")?;
                require_role(role, PartyRole::Creator)?;
                require_non_empty(deliverable_id, "deliverable_id")?;
                if self.accepted_deliverable_ids.contains(deliverable_id)
                    || self.pending_deliverable.is_some()
                {
                    return Err(DealError::UnknownDeliverable(deliverable_id.clone()));
                }
                let terms = self.current_terms.as_ref().ok_or(DealError::TermsNotFullyApproved)?;
                if self.accepted_deliverable_ids.len() >= usize::from(terms.deliverable_count) {
                    return Err(DealError::InvalidDeliverableCount);
                }
                if *final_for_deal
                    != (self.accepted_deliverable_ids.len() + 1 == usize::from(terms.deliverable_count))
                {
                    return Err(DealError::InvalidDeliverableCount);
                }
                self.pending_deliverable = Some(PendingDeliverable {
                    deliverable_id: deliverable_id.clone(),
                    final_for_deal: *final_for_deal,
                });
                self.state = DealState::AwaitingAcceptance;
                Ok(())
            }
            DealAction::AcceptDeliverable { deliverable_id } => {
                self.require_state(&[DealState::AwaitingAcceptance], "AcceptDeliverable")?;
                require_role(role, PartyRole::Sponsor)?;
                let pending = self
                    .pending_deliverable
                    .as_ref()
                    .ok_or_else(|| DealError::UnknownDeliverable(deliverable_id.clone()))?;
                if pending.deliverable_id != *deliverable_id {
                    return Err(DealError::WrongDeliverable {
                        pending: pending.deliverable_id.clone(),
                        supplied: deliverable_id.clone(),
                    });
                }
                let is_final = pending.final_for_deal;
                self.accepted_deliverable_ids.push(deliverable_id.clone());
                self.pending_deliverable = None;
                self.state = if is_final {
                    DealState::Accepted
                } else {
                    DealState::InProgress
                };
                Ok(())
            }
            DealAction::RejectDeliverable {
                deliverable_id,
                reason_ref,
            } => {
                self.require_state(&[DealState::AwaitingAcceptance], "RejectDeliverable")?;
                require_role(role, PartyRole::Sponsor)?;
                require_non_empty(reason_ref, "reason_ref")?;
                let pending = self
                    .pending_deliverable
                    .as_ref()
                    .ok_or_else(|| DealError::UnknownDeliverable(deliverable_id.clone()))?;
                if pending.deliverable_id != *deliverable_id {
                    return Err(DealError::WrongDeliverable {
                        pending: pending.deliverable_id.clone(),
                        supplied: deliverable_id.clone(),
                    });
                }
                self.state = DealState::Disputed;
                Ok(())
            }
            DealAction::RaiseDispute { reason_ref } => {
                self.require_state(
                    &[
                        DealState::TermsProposed,
                        DealState::Agreed,
                        DealState::InProgress,
                        DealState::AwaitingAcceptance,
                    ],
                    "RaiseDispute",
                )?;
                require_non_empty(reason_ref, "reason_ref")?;
                self.state = DealState::Disputed;
                Ok(())
            }
            DealAction::Cancel { reason_ref } => {
                self.require_state(
                    &[
                        DealState::Draft,
                        DealState::TermsProposed,
                        DealState::Agreed,
                        DealState::InProgress,
                        DealState::AwaitingAcceptance,
                    ],
                    "Cancel",
                )?;
                require_non_empty(reason_ref, "reason_ref")?;
                self.state = DealState::Cancelled;
                Ok(())
            }
        }
    }

    fn role_for_actor(&self, actor: &str) -> Result<PartyRole, DealError> {
        if actor == self.creator_party_id {
            Ok(PartyRole::Creator)
        } else if actor == self.sponsor_party_id {
            Ok(PartyRole::Sponsor)
        } else {
            Err(DealError::UnknownActor(actor.to_owned()))
        }
    }

    fn require_state(&self, states: &[DealState], action: &'static str) -> Result<(), DealError> {
        if states.contains(&self.state) {
            Ok(())
        } else {
            Err(DealError::InvalidTransition {
                state: self.state,
                action,
            })
        }
    }
}

fn validate_terms(terms: &DealTerms) -> Result<(), DealError> {
    if terms.version == 0 {
        return Err(DealError::InvalidTermsVersion {
            expected: 1,
            actual: terms.version,
        });
    }
    if terms.fee_minor_units == 0 {
        return Err(DealError::InvalidFee);
    }
    if terms.currency.len() != 3
        || !terms.currency.bytes().all(|byte| byte.is_ascii_uppercase())
    {
        return Err(DealError::InvalidCurrency(terms.currency.clone()));
    }
    require_non_empty(&terms.terms_document_ref, "terms_document_ref")?;
    require_non_empty(&terms.rights_schedule_ref, "rights_schedule_ref")?;
    if terms.deliverable_count == 0 {
        return Err(DealError::InvalidDeliverableCount);
    }
    Ok(())
}

fn require_role(actual: PartyRole, required: PartyRole) -> Result<(), DealError> {
    if actual == required {
        Ok(())
    } else {
        Err(DealError::WrongActor { required })
    }
}

fn require_non_empty(value: &str, field: &'static str) -> Result<(), DealError> {
    if value.trim().is_empty() {
        Err(DealError::EmptyField(field))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn terms(version: u32, count: u16) -> DealTerms {
        DealTerms {
            version,
            fee_minor_units: 100_000,
            currency: "USD".into(),
            terms_document_ref: format!("terms-v{version}"),
            rights_schedule_ref: format!("rights-v{version}"),
            deliverable_count: count,
            disclosure_required: true,
            synthetic_likeness_allowed: false,
        }
    }

    fn event(id: &str, time: i64, actor: &str, action: DealAction) -> DealEvent {
        DealEvent {
            event_id: id.into(),
            occurred_at_utc_seconds: time,
            actor_party_id: actor.into(),
            evidence_ref: format!("evidence-{id}"),
            action,
        }
    }

    fn draft() -> DealWorkflow {
        DealWorkflow::new("deal-1", "creator-1", "sponsor-1").unwrap()
    }

    fn propose_and_agree(workflow: &mut DealWorkflow, count: u16) {
        workflow.append(event("propose", 1, "sponsor-1", DealAction::ProposeTerms { terms: terms(1, count) })).unwrap();
        workflow.append(event("creator-approve", 2, "creator-1", DealAction::ApproveTerms { terms_version: 1 })).unwrap();
        workflow.append(event("sponsor-approve", 3, "sponsor-1", DealAction::ApproveTerms { terms_version: 1 })).unwrap();
    }

    #[test]
    fn work_cannot_start_before_both_parties_approve_same_terms() {
        let mut workflow = draft();
        workflow.append(event("propose", 1, "sponsor-1", DealAction::ProposeTerms { terms: terms(1, 1) })).unwrap();
        workflow.append(event("creator-approve", 2, "creator-1", DealAction::ApproveTerms { terms_version: 1 })).unwrap();
        assert_eq!(
            workflow.append(event("start", 3, "creator-1", DealAction::StartWork)),
            Err(DealError::InvalidTransition { state: DealState::TermsProposed, action: "StartWork" })
        );
        assert_eq!(workflow.state(), DealState::TermsProposed);
        assert_eq!(workflow.history().len(), 2);
    }

    #[test]
    fn term_revision_clears_both_approvals() {
        let mut workflow = draft();
        propose_and_agree(&mut workflow, 1);
        assert_eq!(workflow.state(), DealState::Agreed);
        workflow.append(event("revise", 4, "sponsor-1", DealAction::ProposeTerms { terms: terms(2, 1) })).unwrap();
        assert_eq!(workflow.state(), DealState::TermsProposed);
        assert_eq!(workflow.approvals(), (None, None));
        assert_eq!(
            workflow.append(event("old-approve", 5, "creator-1", DealAction::ApproveTerms { terms_version: 1 })),
            Err(DealError::WrongTermsVersion { current: 2, requested: 1 })
        );
    }

    #[test]
    fn role_boundaries_prevent_sponsor_starting_work_or_creator_self_acceptance() {
        let mut workflow = draft();
        propose_and_agree(&mut workflow, 1);
        assert_eq!(
            workflow.append(event("sponsor-start", 4, "sponsor-1", DealAction::StartWork)),
            Err(DealError::WrongActor { required: PartyRole::Creator })
        );
        workflow.append(event("start", 5, "creator-1", DealAction::StartWork)).unwrap();
        workflow.append(event("deliver", 6, "creator-1", DealAction::SubmitDeliverable {
            deliverable_id: "d1".into(), final_for_deal: true
        })).unwrap();
        assert_eq!(
            workflow.append(event("creator-accept", 7, "creator-1", DealAction::AcceptDeliverable {
                deliverable_id: "d1".into()
            })),
            Err(DealError::WrongActor { required: PartyRole::Sponsor })
        );
        assert_eq!(workflow.state(), DealState::AwaitingAcceptance);
    }

    #[test]
    fn final_acceptance_is_not_payment_evidence() {
        let mut workflow = draft();
        propose_and_agree(&mut workflow, 1);
        workflow.append(event("start", 4, "creator-1", DealAction::StartWork)).unwrap();
        workflow.append(event("deliver", 5, "creator-1", DealAction::SubmitDeliverable {
            deliverable_id: "d1".into(), final_for_deal: true
        })).unwrap();
        workflow.append(event("accept", 6, "sponsor-1", DealAction::AcceptDeliverable {
            deliverable_id: "d1".into()
        })).unwrap();
        assert_eq!(workflow.state(), DealState::Accepted);
        assert!(workflow.history().iter().all(|entry| !matches!(
            entry.action,
            DealAction::AcceptDeliverable { .. }
        ) || entry.actor_party_id == "sponsor-1"));
        // This module intentionally has no field representing cash received.
    }

    #[test]
    fn multiple_deliverables_must_match_agreed_count_and_final_flag() {
        let mut workflow = draft();
        propose_and_agree(&mut workflow, 2);
        workflow.append(event("start", 4, "creator-1", DealAction::StartWork)).unwrap();
        assert_eq!(
            workflow.append(event("deliver-wrong-final", 5, "creator-1", DealAction::SubmitDeliverable {
                deliverable_id: "d1".into(), final_for_deal: true
            })),
            Err(DealError::InvalidDeliverableCount)
        );
        assert_eq!(workflow.state(), DealState::InProgress);
        workflow.append(event("deliver-1", 6, "creator-1", DealAction::SubmitDeliverable {
            deliverable_id: "d1".into(), final_for_deal: false
        })).unwrap();
        workflow.append(event("accept-1", 7, "sponsor-1", DealAction::AcceptDeliverable {
            deliverable_id: "d1".into()
        })).unwrap();
        assert_eq!(workflow.state(), DealState::InProgress);
        workflow.append(event("deliver-2", 8, "creator-1", DealAction::SubmitDeliverable {
            deliverable_id: "d2".into(), final_for_deal: true
        })).unwrap();
        workflow.append(event("accept-2", 9, "sponsor-1", DealAction::AcceptDeliverable {
            deliverable_id: "d2".into()
        })).unwrap();
        assert_eq!(workflow.state(), DealState::Accepted);
        assert_eq!(workflow.accepted_deliverable_ids(), &["d1".to_string(), "d2".to_string()]);
    }

    #[test]
    fn wrong_deliverable_cannot_be_accepted() {
        let mut workflow = draft();
        propose_and_agree(&mut workflow, 1);
        workflow.append(event("start", 4, "creator-1", DealAction::StartWork)).unwrap();
        workflow.append(event("deliver", 5, "creator-1", DealAction::SubmitDeliverable {
            deliverable_id: "d1".into(), final_for_deal: true
        })).unwrap();
        assert_eq!(
            workflow.append(event("accept-wrong", 6, "sponsor-1", DealAction::AcceptDeliverable {
                deliverable_id: "other".into()
            })),
            Err(DealError::WrongDeliverable { pending: "d1".into(), supplied: "other".into() })
        );
        assert_eq!(workflow.state(), DealState::AwaitingAcceptance);
        assert_eq!(workflow.history().len(), 3);
    }

    #[test]
    fn event_replay_is_idempotent_and_conflicting_replay_fails_closed() {
        let mut workflow = draft();
        let proposal = event("propose", 1, "sponsor-1", DealAction::ProposeTerms { terms: terms(1, 1) });
        assert_eq!(workflow.append(proposal.clone()).unwrap(), AppendOutcome::Appended);
        assert_eq!(workflow.append(proposal.clone()).unwrap(), AppendOutcome::IdenticalReplay);
        let conflicting = event("propose", 1, "sponsor-1", DealAction::ProposeTerms { terms: terms(1, 2) });
        assert_eq!(workflow.append(conflicting), Err(DealError::ConflictingReplay("propose".into())));
        assert_eq!(workflow.history().len(), 1);
    }

    #[test]
    fn out_of_order_time_and_unknown_actors_are_rejected_atomically() {
        let mut workflow = draft();
        workflow.append(event("propose", 10, "sponsor-1", DealAction::ProposeTerms { terms: terms(1, 1) })).unwrap();
        assert_eq!(
            workflow.append(event("old-time", 9, "creator-1", DealAction::ApproveTerms { terms_version: 1 })),
            Err(DealError::NonMonotonicEventTime { previous: 10, incoming: 9 })
        );
        assert_eq!(
            workflow.append(event("stranger", 11, "intruder", DealAction::ApproveTerms { terms_version: 1 })),
            Err(DealError::UnknownActor("intruder".into()))
        );
        assert_eq!(workflow.history().len(), 1);
        assert_eq!(workflow.state(), DealState::TermsProposed);
    }

    #[test]
    fn rejected_deliverable_enters_dispute_without_claiming_legal_outcome() {
        let mut workflow = draft();
        propose_and_agree(&mut workflow, 1);
        workflow.append(event("start", 4, "creator-1", DealAction::StartWork)).unwrap();
        workflow.append(event("deliver", 5, "creator-1", DealAction::SubmitDeliverable {
            deliverable_id: "d1".into(), final_for_deal: true
        })).unwrap();
        workflow.append(event("reject", 6, "sponsor-1", DealAction::RejectDeliverable {
            deliverable_id: "d1".into(), reason_ref: "reason-1".into()
        })).unwrap();
        assert_eq!(workflow.state(), DealState::Disputed);
        assert_eq!(
            workflow.append(event("accept-later", 7, "sponsor-1", DealAction::AcceptDeliverable {
                deliverable_id: "d1".into()
            })),
            Err(DealError::InvalidTransition { state: DealState::Disputed, action: "AcceptDeliverable" })
        );
    }

    #[test]
    fn cancel_and_dispute_are_terminal_in_v1() {
        let mut cancelled = draft();
        cancelled.append(event("cancel", 1, "creator-1", DealAction::Cancel {
            reason_ref: "cancel-evidence".into()
        })).unwrap();
        assert_eq!(cancelled.state(), DealState::Cancelled);
        assert!(matches!(
            cancelled.append(event("propose-later", 2, "sponsor-1", DealAction::ProposeTerms { terms: terms(1, 1) })),
            Err(DealError::InvalidTransition { .. })
        ));

        let mut disputed = draft();
        disputed.append(event("propose", 1, "sponsor-1", DealAction::ProposeTerms { terms: terms(1, 1) })).unwrap();
        disputed.append(event("dispute", 2, "creator-1", DealAction::RaiseDispute {
            reason_ref: "dispute-evidence".into()
        })).unwrap();
        assert_eq!(disputed.state(), DealState::Disputed);
        assert!(matches!(
            disputed.append(event("approve-after-dispute", 3, "creator-1", DealAction::ApproveTerms { terms_version: 1 })),
            Err(DealError::InvalidTransition { .. })
        ));
    }

    #[test]
    fn terms_must_include_rights_scope_currency_and_paid_fee() {
        let mut workflow = draft();
        let mut invalid = terms(1, 1);
        invalid.rights_schedule_ref = " ".into();
        assert_eq!(
            workflow.append(event("bad-rights", 1, "sponsor-1", DealAction::ProposeTerms { terms: invalid })),
            Err(DealError::EmptyField("rights_schedule_ref"))
        );
        let mut invalid = terms(1, 1);
        invalid.currency = "usd".into();
        assert_eq!(
            workflow.append(event("bad-currency", 1, "sponsor-1", DealAction::ProposeTerms { terms: invalid })),
            Err(DealError::InvalidCurrency("usd".into()))
        );
        let mut invalid = terms(1, 1);
        invalid.fee_minor_units = 0;
        assert_eq!(
            workflow.append(event("bad-fee", 1, "sponsor-1", DealAction::ProposeTerms { terms: invalid })),
            Err(DealError::InvalidFee)
        );
        assert_eq!(workflow.state(), DealState::Draft);
        assert!(workflow.history().is_empty());
    }
}
