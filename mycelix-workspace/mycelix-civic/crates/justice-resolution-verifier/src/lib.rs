// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Pure qualification kernel for final Justice monetary remedies.
//!
//! Callers supply transport-neutral snapshots that an owning Justice adapter
//! must obtain from exact authoritative records. Positive finality is not a
//! caller-authored appeal-state assertion: it must arrive as a sealed
//! `QualifiedJusticeFinalityV1` minted by the pure Justice finality qualifier.
//! This crate proves the relationships between the exact case/arbitration/
//! decision/remedy/policy cut and that sealed finality cut, then constructs a
//! positive typed result plus auditable verification receipts. It does not fetch
//! Holochain state or authenticate runtime records or finality evidence itself.

use core::fmt;
use std::collections::BTreeSet;

use justice_finality_qualification::{
    JusticeFinalityQualificationReceiptV1, QualifiedJusticeFinalityV1,
};
use justice_resolution_types::{
    FinalMonetaryRemedyV1, JusticeFinalityBasisV1, MonetaryRemedyKindV1,
};

/// Frozen decision-rule profile for the deliberately narrow v0.1 qualifier.
pub const TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE: &str =
    "justice.two-party-prevailing-party-full-award";
/// Semantic version of the v0.1 decision rule.
pub const TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION: u32 = 1;

/// Exact two-party case facts required by the v0.1 full-award profile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TwoPartyCaseSnapshotV1 {
    pub case_ref: String,
    pub complainant_ref: String,
    pub respondent_ref: String,
    /// Exact institutional subject, e.g. an order/agreement/obligation/effect.
    pub subject_ref: String,
}

/// One arbitration panel member as observed from the exact arbitration record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PanelMemberSnapshotV1 {
    pub party_ref: String,
    pub accepted: bool,
    pub recused: bool,
}

/// Exact arbitration facts required by the verifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ArbitrationSnapshotV1 {
    pub arbitration_ref: String,
    pub case_ref: String,
    pub panel: Vec<PanelMemberSnapshotV1>,
}

/// Full two-party decision outcomes supported by v0.1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FullDecisionOutcomeV1 {
    ForComplainant,
    ForRespondent,
}

/// Vote choices supported by the current Justice arbitration shape.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecisionVoteChoiceV1 {
    ForComplainant,
    ForRespondent,
    Abstain,
}

/// One exact recorded vote.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionVoteSnapshotV1 {
    pub voter_ref: String,
    pub choice: DecisionVoteChoiceV1,
}

/// Exact decision facts consumed by the pure verifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionSnapshotV1 {
    pub decision_ref: String,
    pub case_ref: String,
    pub arbitration_ref: String,
    pub outcome: FullDecisionOutcomeV1,
    pub votes: Vec<DecisionVoteSnapshotV1>,
    pub rendered_at_unix_ms: u64,
    pub appeal_deadline_unix_ms: u64,
    /// Runtime hint retained only so tests can prove it is not authority.
    pub declared_finalized: bool,
}

/// Runtime remedy classes that can be mapped into the v0.1 Justice vocabulary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RuntimeMonetaryRemedyKindV1 {
    Compensation,
    Restitution,
    Unsupported,
}

/// Exact target remedy facts from one Decision remedy position.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MonetaryRemedySnapshotV1 {
    pub remedy_index: u32,
    pub kind: RuntimeMonetaryRemedyKindV1,
    pub responsible_party_ref: String,
    pub amount: Option<u128>,
    pub unit: Option<String>,
}

/// Exact Justice policy inputs for the frozen two-party prevailing-party rule.
///
/// The policy record reference and semantic profile/version are both explicit.
/// A runtime adapter must bind this whole policy view to authoritative Justice
/// policy state; this pure verifier does not establish policy authority.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FullAwardPolicyV1 {
    pub policy_ref: String,
    pub semantic_profile: String,
    pub semantic_version: u32,
    pub minimum_quorum_votes: usize,
    pub minimum_support_votes: usize,
}

/// Complete explicit input cut for one monetary-remedy qualification.
///
/// `qualified_finality` cannot be constructed directly outside its owning
/// qualifier. The owning Justice runtime still must authenticate the evidence
/// supplied to that qualifier before this consumer may trust the token.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MonetaryRemedyQualificationBasisV1 {
    pub case: TwoPartyCaseSnapshotV1,
    pub arbitration: ArbitrationSnapshotV1,
    pub decision: DecisionSnapshotV1,
    pub remedy: MonetaryRemedySnapshotV1,
    pub qualified_finality: QualifiedJusticeFinalityV1,
    pub policy: FullAwardPolicyV1,
    pub qualification_time_unix_ms: u64,
}

/// Compact verifier-owned receipt preserving the exact qualification basis that
/// justified a positive result without embedding raw runtime records.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JusticeVerificationReceiptV1 {
    case_ref: String,
    arbitration_ref: String,
    decision_ref: String,
    decision_rendered_at_unix_ms: u64,
    appeal_deadline_unix_ms: u64,
    remedy_index: u32,
    policy_ref: String,
    policy_semantic_profile: String,
    policy_semantic_version: u32,
    minimum_quorum_votes: usize,
    minimum_support_votes: usize,
    eligible_panel: BTreeSet<String>,
    voters: BTreeSet<String>,
    support_votes: usize,
    qualification_time_unix_ms: u64,
    finality: JusticeFinalityBasisV1,
    finality_receipt: JusticeFinalityQualificationReceiptV1,
}

impl JusticeVerificationReceiptV1 {
    #[must_use]
    pub fn case_ref(&self) -> &str {
        &self.case_ref
    }

    #[must_use]
    pub fn arbitration_ref(&self) -> &str {
        &self.arbitration_ref
    }

    #[must_use]
    pub fn decision_ref(&self) -> &str {
        &self.decision_ref
    }

    #[must_use]
    pub const fn decision_rendered_at_unix_ms(&self) -> u64 {
        self.decision_rendered_at_unix_ms
    }

    #[must_use]
    pub const fn appeal_deadline_unix_ms(&self) -> u64 {
        self.appeal_deadline_unix_ms
    }

    #[must_use]
    pub const fn remedy_index(&self) -> u32 {
        self.remedy_index
    }

    #[must_use]
    pub fn policy_ref(&self) -> &str {
        &self.policy_ref
    }

    #[must_use]
    pub fn policy_semantic_profile(&self) -> &str {
        &self.policy_semantic_profile
    }

    #[must_use]
    pub const fn policy_semantic_version(&self) -> u32 {
        self.policy_semantic_version
    }

    #[must_use]
    pub const fn minimum_quorum_votes(&self) -> usize {
        self.minimum_quorum_votes
    }

    #[must_use]
    pub const fn minimum_support_votes(&self) -> usize {
        self.minimum_support_votes
    }

    #[must_use]
    pub const fn eligible_panel(&self) -> &BTreeSet<String> {
        &self.eligible_panel
    }

    #[must_use]
    pub const fn voters(&self) -> &BTreeSet<String> {
        &self.voters
    }

    #[must_use]
    pub const fn support_votes(&self) -> usize {
        self.support_votes
    }

    #[must_use]
    pub const fn qualification_time_unix_ms(&self) -> u64 {
        self.qualification_time_unix_ms
    }

    #[must_use]
    pub const fn finality(&self) -> &JusticeFinalityBasisV1 {
        &self.finality
    }

    #[must_use]
    pub const fn finality_receipt(&self) -> &JusticeFinalityQualificationReceiptV1 {
        &self.finality_receipt
    }
}

/// Positive output whose constructor is verifier-owned.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedJusticeMonetaryRemedyV1 {
    outcome: FinalMonetaryRemedyV1,
    receipt: JusticeVerificationReceiptV1,
}

impl VerifiedJusticeMonetaryRemedyV1 {
    #[must_use]
    pub const fn outcome(&self) -> &FinalMonetaryRemedyV1 {
        &self.outcome
    }

    #[must_use]
    pub const fn receipt(&self) -> &JusticeVerificationReceiptV1 {
        &self.receipt
    }

    #[must_use]
    pub fn into_parts(self) -> (FinalMonetaryRemedyV1, JusticeVerificationReceiptV1) {
        (self.outcome, self.receipt)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct VoteQualificationV1 {
    voters: BTreeSet<String>,
    support_votes: usize,
}

/// Deterministic denial reasons for v0.1 qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JusticeVerificationError {
    EmptyCaseRef,
    EmptyComplainantRef,
    EmptyRespondentRef,
    SameCaseParties,
    EmptySubjectRef,
    EmptyArbitrationRef,
    ArbitrationCaseMismatch,
    EmptyPanel,
    EmptyPanelMemberRef,
    DuplicatePanelMember,
    NoActivePanelMember,
    EmptyDecisionRef,
    DecisionCaseMismatch,
    DecisionArbitrationMismatch,
    AppealDeadlineBeforeDecision,
    FinalityDecisionMismatch,
    FinalityDecisionRenderedAtMismatch,
    FinalityAppealDeadlineMismatch,
    FinalityQualificationTimeMismatch,
    EmptyPolicyRef,
    WrongPolicyProfile,
    InvalidPolicyThreshold,
    EmptyVoteRef,
    DuplicateVote,
    VoteFromIneligiblePanelMember,
    InsufficientQuorum,
    InsufficientSupport,
    UnsupportedRemedyKind,
    WrongResponsibleParty,
    MissingAmount,
    ZeroAmount,
    MissingUnit,
    OutputConstructionFailed,
}

impl fmt::Display for JusticeVerificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "justice resolution qualification denied: {self:?}")
    }
}

impl std::error::Error for JusticeVerificationError {}

/// Qualify an exact two-party full monetary remedy without ambient authority.
///
/// The runtime `declared_finalized` bit is intentionally ignored. Positive
/// finality comes exclusively from `QualifiedJusticeFinalityV1`, and that token
/// must bind the exact Decision ref/rendered time/appeal deadline/qualification
/// time consumed by this same verifier invocation.
pub fn qualify_monetary_remedy_v1(
    basis: MonetaryRemedyQualificationBasisV1,
) -> Result<VerifiedJusticeMonetaryRemedyV1, JusticeVerificationError> {
    validate_case(&basis.case)?;
    let active_panel = validate_arbitration(&basis.arbitration, &basis.case)?;
    validate_decision_links(&basis.decision, &basis.case, &basis.arbitration)?;
    validate_finality_cut(
        &basis.decision,
        &basis.qualified_finality,
        basis.qualification_time_unix_ms,
    )?;
    validate_policy(&basis.policy, active_panel.len())?;
    let vote_qualification = validate_votes(
        &basis.decision,
        &active_panel,
        basis.policy.minimum_quorum_votes,
        basis.policy.minimum_support_votes,
    )?;

    let (winner, loser) = match basis.decision.outcome {
        FullDecisionOutcomeV1::ForComplainant => (
            basis.case.complainant_ref.as_str(),
            basis.case.respondent_ref.as_str(),
        ),
        FullDecisionOutcomeV1::ForRespondent => (
            basis.case.respondent_ref.as_str(),
            basis.case.complainant_ref.as_str(),
        ),
    };

    if basis.remedy.responsible_party_ref != loser {
        return Err(JusticeVerificationError::WrongResponsibleParty);
    }

    let remedy_kind = match basis.remedy.kind {
        RuntimeMonetaryRemedyKindV1::Compensation => MonetaryRemedyKindV1::Compensation,
        RuntimeMonetaryRemedyKindV1::Restitution => MonetaryRemedyKindV1::Restitution,
        RuntimeMonetaryRemedyKindV1::Unsupported => {
            return Err(JusticeVerificationError::UnsupportedRemedyKind);
        }
    };

    let amount = basis
        .remedy
        .amount
        .ok_or(JusticeVerificationError::MissingAmount)?;
    if amount == 0 {
        return Err(JusticeVerificationError::ZeroAmount);
    }

    let unit = basis
        .remedy
        .unit
        .as_deref()
        .ok_or(JusticeVerificationError::MissingUnit)?;
    if unit.trim().is_empty() {
        return Err(JusticeVerificationError::MissingUnit);
    }

    let finality = basis.qualified_finality.finality().clone();
    let finality_receipt = basis.qualified_finality.receipt().clone();

    let remedy_ref = derived_remedy_ref(&basis.decision.decision_ref, basis.remedy.remedy_index);
    let effect_id = derived_effect_id(&basis.decision.decision_ref, basis.remedy.remedy_index);

    let outcome = FinalMonetaryRemedyV1::new(
        basis.case.case_ref.clone(),
        basis.decision.decision_ref.clone(),
        remedy_ref,
        remedy_kind,
        basis.case.subject_ref.clone(),
        effect_id,
        basis.remedy.responsible_party_ref.clone(),
        winner.to_owned(),
        unit.to_owned(),
        amount,
        finality.clone(),
    )
    .map_err(|_| JusticeVerificationError::OutputConstructionFailed)?;

    let receipt = JusticeVerificationReceiptV1 {
        case_ref: basis.case.case_ref,
        arbitration_ref: basis.arbitration.arbitration_ref,
        decision_ref: basis.decision.decision_ref,
        decision_rendered_at_unix_ms: basis.decision.rendered_at_unix_ms,
        appeal_deadline_unix_ms: basis.decision.appeal_deadline_unix_ms,
        remedy_index: basis.remedy.remedy_index,
        policy_ref: basis.policy.policy_ref,
        policy_semantic_profile: basis.policy.semantic_profile,
        policy_semantic_version: basis.policy.semantic_version,
        minimum_quorum_votes: basis.policy.minimum_quorum_votes,
        minimum_support_votes: basis.policy.minimum_support_votes,
        eligible_panel: active_panel,
        voters: vote_qualification.voters,
        support_votes: vote_qualification.support_votes,
        qualification_time_unix_ms: basis.qualification_time_unix_ms,
        finality,
        finality_receipt,
    };

    Ok(VerifiedJusticeMonetaryRemedyV1 { outcome, receipt })
}

fn validate_case(case: &TwoPartyCaseSnapshotV1) -> Result<(), JusticeVerificationError> {
    if case.case_ref.trim().is_empty() {
        return Err(JusticeVerificationError::EmptyCaseRef);
    }
    if case.complainant_ref.trim().is_empty() {
        return Err(JusticeVerificationError::EmptyComplainantRef);
    }
    if case.respondent_ref.trim().is_empty() {
        return Err(JusticeVerificationError::EmptyRespondentRef);
    }
    if case.complainant_ref == case.respondent_ref {
        return Err(JusticeVerificationError::SameCaseParties);
    }
    if case.subject_ref.trim().is_empty() {
        return Err(JusticeVerificationError::EmptySubjectRef);
    }
    Ok(())
}

fn validate_arbitration(
    arbitration: &ArbitrationSnapshotV1,
    case: &TwoPartyCaseSnapshotV1,
) -> Result<BTreeSet<String>, JusticeVerificationError> {
    if arbitration.arbitration_ref.trim().is_empty() {
        return Err(JusticeVerificationError::EmptyArbitrationRef);
    }
    if arbitration.case_ref != case.case_ref {
        return Err(JusticeVerificationError::ArbitrationCaseMismatch);
    }
    if arbitration.panel.is_empty() {
        return Err(JusticeVerificationError::EmptyPanel);
    }

    let mut all = BTreeSet::new();
    let mut active = BTreeSet::new();
    for member in &arbitration.panel {
        if member.party_ref.trim().is_empty() {
            return Err(JusticeVerificationError::EmptyPanelMemberRef);
        }
        if !all.insert(member.party_ref.clone()) {
            return Err(JusticeVerificationError::DuplicatePanelMember);
        }
        if member.accepted && !member.recused {
            active.insert(member.party_ref.clone());
        }
    }
    if active.is_empty() {
        return Err(JusticeVerificationError::NoActivePanelMember);
    }
    Ok(active)
}

fn validate_decision_links(
    decision: &DecisionSnapshotV1,
    case: &TwoPartyCaseSnapshotV1,
    arbitration: &ArbitrationSnapshotV1,
) -> Result<(), JusticeVerificationError> {
    if decision.decision_ref.trim().is_empty() {
        return Err(JusticeVerificationError::EmptyDecisionRef);
    }
    if decision.case_ref != case.case_ref {
        return Err(JusticeVerificationError::DecisionCaseMismatch);
    }
    if decision.arbitration_ref != arbitration.arbitration_ref {
        return Err(JusticeVerificationError::DecisionArbitrationMismatch);
    }
    if decision.appeal_deadline_unix_ms < decision.rendered_at_unix_ms {
        return Err(JusticeVerificationError::AppealDeadlineBeforeDecision);
    }
    Ok(())
}

fn validate_finality_cut(
    decision: &DecisionSnapshotV1,
    qualified_finality: &QualifiedJusticeFinalityV1,
    qualification_time_unix_ms: u64,
) -> Result<(), JusticeVerificationError> {
    if qualified_finality.decision_ref() != decision.decision_ref {
        return Err(JusticeVerificationError::FinalityDecisionMismatch);
    }
    if qualified_finality.decision_rendered_at_unix_ms() != decision.rendered_at_unix_ms {
        return Err(JusticeVerificationError::FinalityDecisionRenderedAtMismatch);
    }
    if qualified_finality.appeal_deadline_unix_ms() != decision.appeal_deadline_unix_ms {
        return Err(JusticeVerificationError::FinalityAppealDeadlineMismatch);
    }
    if qualified_finality.qualification_time_unix_ms() != qualification_time_unix_ms {
        return Err(JusticeVerificationError::FinalityQualificationTimeMismatch);
    }
    Ok(())
}

fn validate_policy(
    policy: &FullAwardPolicyV1,
    active_panel_size: usize,
) -> Result<(), JusticeVerificationError> {
    if policy.policy_ref.trim().is_empty() {
        return Err(JusticeVerificationError::EmptyPolicyRef);
    }
    if policy.semantic_profile != TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE
        || policy.semantic_version != TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION
    {
        return Err(JusticeVerificationError::WrongPolicyProfile);
    }
    if policy.minimum_quorum_votes == 0
        || policy.minimum_support_votes == 0
        || policy.minimum_support_votes > policy.minimum_quorum_votes
        || policy.minimum_quorum_votes > active_panel_size
    {
        return Err(JusticeVerificationError::InvalidPolicyThreshold);
    }
    Ok(())
}

fn validate_votes(
    decision: &DecisionSnapshotV1,
    active_panel: &BTreeSet<String>,
    minimum_quorum_votes: usize,
    minimum_support_votes: usize,
) -> Result<VoteQualificationV1, JusticeVerificationError> {
    let mut voters = BTreeSet::new();
    let mut support_votes = 0usize;

    for vote in &decision.votes {
        if vote.voter_ref.trim().is_empty() {
            return Err(JusticeVerificationError::EmptyVoteRef);
        }
        if !voters.insert(vote.voter_ref.clone()) {
            return Err(JusticeVerificationError::DuplicateVote);
        }
        if !active_panel.contains(&vote.voter_ref) {
            return Err(JusticeVerificationError::VoteFromIneligiblePanelMember);
        }
        let supports_outcome = matches!(
            (decision.outcome, vote.choice),
            (
                FullDecisionOutcomeV1::ForComplainant,
                DecisionVoteChoiceV1::ForComplainant
            ) | (
                FullDecisionOutcomeV1::ForRespondent,
                DecisionVoteChoiceV1::ForRespondent
            )
        );
        if supports_outcome {
            support_votes += 1;
        }
    }

    if voters.len() < minimum_quorum_votes {
        return Err(JusticeVerificationError::InsufficientQuorum);
    }
    if support_votes < minimum_support_votes {
        return Err(JusticeVerificationError::InsufficientSupport);
    }
    Ok(VoteQualificationV1 {
        voters,
        support_votes,
    })
}

fn derived_remedy_ref(decision_ref: &str, remedy_index: u32) -> String {
    let mut out = String::from("justice-remedy-v1");
    push_text_field(&mut out, "decision", decision_ref);
    out.push_str("|index:");
    out.push_str(&remedy_index.to_string());
    out
}

fn derived_effect_id(decision_ref: &str, remedy_index: u32) -> String {
    let mut out = String::from("justice-remedy-effect-v1");
    push_text_field(&mut out, "decision", decision_ref);
    out.push_str("|index:");
    out.push_str(&remedy_index.to_string());
    out
}

fn push_text_field(out: &mut String, name: &str, value: &str) {
    out.push('|');
    out.push_str(name);
    out.push(':');
    out.push_str(&value.len().to_string());
    out.push(':');
    out.push_str(value);
}

#[cfg(test)]
mod tests {
    use super::*;
    use justice_finality_qualification::{
        APPEAL_FILING_PROFILE, APPEAL_FILING_VERSION, AuthenticatedAppealFilingEvidenceV1,
        AuthenticatedTerminalAppealResolutionEvidenceV1, COMPLETE_APPEAL_COVERAGE_PROFILE,
        COMPLETE_APPEAL_COVERAGE_VERSION, CompleteAppealCoverageEvidenceV1,
        FINALITY_QUALIFICATION_PROFILE, JusticeFinalityQualificationBasisV1,
        TERMINAL_APPEAL_RESOLUTION_PROFILE, TERMINAL_APPEAL_RESOLUTION_VERSION,
        TerminalAppealDispositionV1, qualify_justice_finality_v1,
    };

    fn no_appeal_finality(
        decision_ref: &str,
        rendered_at_unix_ms: u64,
        appeal_deadline_unix_ms: u64,
        qualification_time_unix_ms: u64,
    ) -> QualifiedJusticeFinalityV1 {
        qualify_justice_finality_v1(JusticeFinalityQualificationBasisV1::NoAppealCoverage {
            decision_ref: decision_ref.into(),
            decision_rendered_at_unix_ms: rendered_at_unix_ms,
            appeal_deadline_unix_ms,
            qualification_time_unix_ms,
            coverage: CompleteAppealCoverageEvidenceV1 {
                coverage_ref: format!(
                    "appeal-coverage:{decision_ref}:through-{qualification_time_unix_ms}"
                ),
                decision_ref: decision_ref.into(),
                authority_evidence_ref: "justice-finality-authority:test".into(),
                semantic_profile: COMPLETE_APPEAL_COVERAGE_PROFILE.into(),
                semantic_version: COMPLETE_APPEAL_COVERAGE_VERSION,
                covered_from_unix_ms: rendered_at_unix_ms,
                covered_through_unix_ms: qualification_time_unix_ms,
                observed_appeal_refs: vec![],
            },
        })
        .unwrap()
    }

    fn terminal_affirmance_finality() -> QualifiedJusticeFinalityV1 {
        qualify_justice_finality_v1(
            JusticeFinalityQualificationBasisV1::TerminalAppealResolution {
                decision_ref: "decision:1".into(),
                decision_rendered_at_unix_ms: 100,
                appeal_deadline_unix_ms: 200,
                qualification_time_unix_ms: 190,
                appeal: AuthenticatedAppealFilingEvidenceV1 {
                    appeal_ref: "appeal:1".into(),
                    decision_ref: "decision:1".into(),
                    appellant_ref: "party:merchant".into(),
                    appeal_number: 1,
                    semantic_profile: APPEAL_FILING_PROFILE.into(),
                    semantic_version: APPEAL_FILING_VERSION,
                    filed_at_unix_ms: 150,
                },
                resolution: AuthenticatedTerminalAppealResolutionEvidenceV1 {
                    resolution_ref: "appeal-resolution:affirmed:1".into(),
                    appeal_ref: "appeal:1".into(),
                    decision_ref: "decision:1".into(),
                    authority_evidence_ref: "appellate-authority:test".into(),
                    semantic_profile: TERMINAL_APPEAL_RESOLUTION_PROFILE.into(),
                    semantic_version: TERMINAL_APPEAL_RESOLUTION_VERSION,
                    resolved_at_unix_ms: 180,
                    disposition: TerminalAppealDispositionV1::Affirmed,
                },
            },
        )
        .unwrap()
    }

    fn basis() -> MonetaryRemedyQualificationBasisV1 {
        MonetaryRemedyQualificationBasisV1 {
            case: TwoPartyCaseSnapshotV1 {
                case_ref: "case:1".to_owned(),
                complainant_ref: "party:customer".to_owned(),
                respondent_ref: "party:merchant".to_owned(),
                subject_ref: "order:1".to_owned(),
            },
            arbitration: ArbitrationSnapshotV1 {
                arbitration_ref: "arbitration:1".to_owned(),
                case_ref: "case:1".to_owned(),
                panel: vec![
                    PanelMemberSnapshotV1 {
                        party_ref: "arb:1".to_owned(),
                        accepted: true,
                        recused: false,
                    },
                    PanelMemberSnapshotV1 {
                        party_ref: "arb:2".to_owned(),
                        accepted: true,
                        recused: false,
                    },
                    PanelMemberSnapshotV1 {
                        party_ref: "arb:3".to_owned(),
                        accepted: true,
                        recused: false,
                    },
                ],
            },
            decision: DecisionSnapshotV1 {
                decision_ref: "decision:1".to_owned(),
                case_ref: "case:1".to_owned(),
                arbitration_ref: "arbitration:1".to_owned(),
                outcome: FullDecisionOutcomeV1::ForComplainant,
                votes: vec![
                    DecisionVoteSnapshotV1 {
                        voter_ref: "arb:1".to_owned(),
                        choice: DecisionVoteChoiceV1::ForComplainant,
                    },
                    DecisionVoteSnapshotV1 {
                        voter_ref: "arb:2".to_owned(),
                        choice: DecisionVoteChoiceV1::ForComplainant,
                    },
                    DecisionVoteSnapshotV1 {
                        voter_ref: "arb:3".to_owned(),
                        choice: DecisionVoteChoiceV1::ForRespondent,
                    },
                ],
                rendered_at_unix_ms: 100,
                appeal_deadline_unix_ms: 200,
                declared_finalized: false,
            },
            remedy: MonetaryRemedySnapshotV1 {
                remedy_index: 0,
                kind: RuntimeMonetaryRemedyKindV1::Restitution,
                responsible_party_ref: "party:merchant".to_owned(),
                amount: Some(5_000),
                unit: Some("USD-cent".to_owned()),
            },
            qualified_finality: no_appeal_finality("decision:1", 100, 200, 201),
            policy: FullAwardPolicyV1 {
                policy_ref: "justice-policy:full-award:v1".to_owned(),
                semantic_profile: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE.to_owned(),
                semantic_version: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION,
                minimum_quorum_votes: 3,
                minimum_support_votes: 2,
            },
            qualification_time_unix_ms: 201,
        }
    }

    #[test]
    fn exact_two_party_restitution_qualifies_with_auditable_receipts() {
        let verified = qualify_monetary_remedy_v1(basis()).unwrap();
        let outcome = verified.outcome();
        assert_eq!(outcome.remedy_kind(), MonetaryRemedyKindV1::Restitution);
        assert_eq!(outcome.subject_ref(), "order:1");
        assert_eq!(outcome.responsible_party_ref(), "party:merchant");
        assert_eq!(outcome.beneficiary_party_ref(), "party:customer");
        assert_eq!(outcome.amount(), 5_000);
        assert_eq!(outcome.unit(), "USD-cent");

        let receipt = verified.receipt();
        assert_eq!(receipt.case_ref(), "case:1");
        assert_eq!(receipt.arbitration_ref(), "arbitration:1");
        assert_eq!(receipt.decision_ref(), "decision:1");
        assert_eq!(receipt.decision_rendered_at_unix_ms(), 100);
        assert_eq!(receipt.appeal_deadline_unix_ms(), 200);
        assert_eq!(receipt.remedy_index(), 0);
        assert_eq!(receipt.policy_ref(), "justice-policy:full-award:v1");
        assert_eq!(
            receipt.policy_semantic_profile(),
            TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE
        );
        assert_eq!(receipt.policy_semantic_version(), 1);
        assert_eq!(receipt.minimum_quorum_votes(), 3);
        assert_eq!(receipt.minimum_support_votes(), 2);
        assert_eq!(receipt.eligible_panel().len(), 3);
        assert_eq!(receipt.voters().len(), 3);
        assert_eq!(receipt.support_votes(), 2);
        assert_eq!(receipt.qualification_time_unix_ms(), 201);
        assert_eq!(
            receipt.finality_receipt().semantic_profile(),
            FINALITY_QUALIFICATION_PROFILE
        );
        assert_eq!(receipt.finality_receipt().decision_ref(), "decision:1");
        assert_eq!(
            receipt.finality_receipt().decision_rendered_at_unix_ms(),
            100
        );
        assert_eq!(receipt.finality_receipt().appeal_deadline_unix_ms(), 200);
        assert_eq!(receipt.finality_receipt().qualification_time_unix_ms(), 201);
    }

    #[test]
    fn same_exact_basis_is_deterministic() {
        let one = qualify_monetary_remedy_v1(basis()).unwrap();
        let two = qualify_monetary_remedy_v1(basis()).unwrap();
        assert_eq!(one, two);
    }

    #[test]
    fn wrong_policy_profile_is_denied() {
        let mut candidate = basis();
        candidate.policy.semantic_version = 2;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::WrongPolicyProfile)
        );
    }

    #[test]
    fn runtime_finalized_flag_is_not_authority() {
        let expected = qualify_monetary_remedy_v1(basis()).unwrap();
        let mut candidate = basis();
        candidate.decision.declared_finalized = true;
        let actual = qualify_monetary_remedy_v1(candidate).unwrap();
        assert_eq!(actual, expected);
    }

    #[test]
    fn finality_must_reference_exact_decision() {
        let mut candidate = basis();
        candidate.decision.decision_ref = "decision:other".into();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::FinalityDecisionMismatch)
        );
    }

    #[test]
    fn finality_must_bind_exact_rendered_time() {
        let mut candidate = basis();
        candidate.decision.rendered_at_unix_ms = 101;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::FinalityDecisionRenderedAtMismatch)
        );
    }

    #[test]
    fn finality_must_bind_exact_appeal_deadline() {
        let mut candidate = basis();
        candidate.decision.appeal_deadline_unix_ms = 201;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::FinalityAppealDeadlineMismatch)
        );
    }

    #[test]
    fn finality_must_bind_exact_qualification_time() {
        let mut candidate = basis();
        candidate.qualification_time_unix_ms = 202;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::FinalityQualificationTimeMismatch)
        );
    }

    #[test]
    fn arbitration_must_reference_exact_case() {
        let mut candidate = basis();
        candidate.arbitration.case_ref = "case:other".to_owned();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::ArbitrationCaseMismatch)
        );
    }

    #[test]
    fn decision_must_reference_exact_case() {
        let mut candidate = basis();
        candidate.decision.case_ref = "case:other".to_owned();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::DecisionCaseMismatch)
        );
    }

    #[test]
    fn decision_must_reference_exact_arbitration() {
        let mut candidate = basis();
        candidate.decision.arbitration_ref = "arbitration:other".to_owned();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::DecisionArbitrationMismatch)
        );
    }

    #[test]
    fn deadline_cannot_precede_decision() {
        let mut candidate = basis();
        candidate.decision.appeal_deadline_unix_ms = 99;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::AppealDeadlineBeforeDecision)
        );
    }

    #[test]
    fn duplicate_panel_member_is_denied() {
        let mut candidate = basis();
        candidate.arbitration.panel[2].party_ref = "arb:2".to_owned();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::DuplicatePanelMember)
        );
    }

    #[test]
    fn invalid_policy_threshold_is_denied() {
        let mut candidate = basis();
        candidate.policy.minimum_quorum_votes = 4;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::InvalidPolicyThreshold)
        );
    }

    #[test]
    fn vote_from_non_panel_actor_is_denied() {
        let mut candidate = basis();
        candidate.decision.votes[2].voter_ref = "arb:outsider".to_owned();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::VoteFromIneligiblePanelMember)
        );
    }

    #[test]
    fn recused_panel_member_cannot_vote() {
        let mut candidate = basis();
        candidate.arbitration.panel[2].recused = true;
        candidate.policy.minimum_quorum_votes = 2;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::VoteFromIneligiblePanelMember)
        );
    }

    #[test]
    fn duplicate_vote_is_denied() {
        let mut candidate = basis();
        candidate.decision.votes[2].voter_ref = "arb:2".to_owned();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::DuplicateVote)
        );
    }

    #[test]
    fn insufficient_quorum_is_denied() {
        let mut candidate = basis();
        candidate.decision.votes.pop();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::InsufficientQuorum)
        );
    }

    #[test]
    fn insufficient_support_is_denied() {
        let mut candidate = basis();
        candidate.decision.votes[1].choice = DecisionVoteChoiceV1::ForRespondent;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::InsufficientSupport)
        );
    }

    #[test]
    fn wrong_responsible_party_is_denied() {
        let mut candidate = basis();
        candidate.remedy.responsible_party_ref = "party:customer".to_owned();
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::WrongResponsibleParty)
        );
    }

    #[test]
    fn incomplete_money_is_denied() {
        let mut missing_amount = basis();
        missing_amount.remedy.amount = None;
        assert_eq!(
            qualify_monetary_remedy_v1(missing_amount),
            Err(JusticeVerificationError::MissingAmount)
        );

        let mut missing_unit = basis();
        missing_unit.remedy.unit = None;
        assert_eq!(
            qualify_monetary_remedy_v1(missing_unit),
            Err(JusticeVerificationError::MissingUnit)
        );
    }

    #[test]
    fn zero_amount_is_denied() {
        let mut candidate = basis();
        candidate.remedy.amount = Some(0);
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::ZeroAmount)
        );
    }

    #[test]
    fn blank_unit_is_denied() {
        let mut candidate = basis();
        candidate.remedy.unit = Some(" ".to_owned());
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::MissingUnit)
        );
    }

    #[test]
    fn unsupported_remedy_kind_is_denied() {
        let mut candidate = basis();
        candidate.remedy.kind = RuntimeMonetaryRemedyKindV1::Unsupported;
        assert_eq!(
            qualify_monetary_remedy_v1(candidate),
            Err(JusticeVerificationError::UnsupportedRemedyKind)
        );
    }

    #[test]
    fn terminal_affirmance_can_establish_finality_before_window_expiry() {
        let mut candidate = basis();
        candidate.qualification_time_unix_ms = 190;
        candidate.qualified_finality = terminal_affirmance_finality();
        let verified = qualify_monetary_remedy_v1(candidate).unwrap();
        assert!(matches!(
            verified.receipt().finality(),
            JusticeFinalityBasisV1::AppealResolved {
                appeal_ref,
                appeal_resolution_ref,
            } if appeal_ref == "appeal:1" && appeal_resolution_ref == "appeal-resolution:affirmed:1"
        ));
    }

    #[test]
    fn respondent_win_reverses_beneficiary_and_responsible_party() {
        let mut candidate = basis();
        candidate.decision.outcome = FullDecisionOutcomeV1::ForRespondent;
        candidate.decision.votes[0].choice = DecisionVoteChoiceV1::ForRespondent;
        candidate.decision.votes[1].choice = DecisionVoteChoiceV1::ForRespondent;
        candidate.decision.votes[2].choice = DecisionVoteChoiceV1::ForComplainant;
        candidate.remedy.responsible_party_ref = "party:customer".to_owned();

        let verified = qualify_monetary_remedy_v1(candidate).unwrap();
        assert_eq!(verified.outcome().beneficiary_party_ref(), "party:merchant");
        assert_eq!(verified.outcome().responsible_party_ref(), "party:customer");
    }
}
