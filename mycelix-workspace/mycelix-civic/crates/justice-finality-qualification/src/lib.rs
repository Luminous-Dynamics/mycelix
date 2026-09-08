// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Pure, positive-evidence qualification of Mycelix Justice finality.
//!
//! This crate intentionally does not fetch Holochain state, inspect a local DHT
//! query, read an ambient clock, or decide who has appellate authority. The
//! owning Justice runtime must authenticate the evidence supplied here.
//!
//! The output is deliberately non-forgeable outside this crate: downstream
//! execution code receives `QualifiedJusticeFinalityV1`, not a caller-declared
//! `no appeal` boolean or mutable `Appeal.status` / `Decision.finalized` field.

use core::fmt;
use std::collections::BTreeSet;

use justice_resolution_types::JusticeFinalityBasisV1;

/// Semantic profile of this positive finality qualifier.
pub const FINALITY_QUALIFICATION_PROFILE: &str = "justice.finality-qualification";
/// Semantic version of this positive finality qualifier.
pub const FINALITY_QUALIFICATION_VERSION: u32 = 1;

/// Profile for an authenticated, complete appeal-set coverage artifact.
pub const COMPLETE_APPEAL_COVERAGE_PROFILE: &str = "justice.complete-appeal-coverage";
pub const COMPLETE_APPEAL_COVERAGE_VERSION: u32 = 1;

/// Profile for an authenticated appeal filing consumed by v0.1 finality.
pub const APPEAL_FILING_PROFILE: &str = "justice.appeal-filing";
pub const APPEAL_FILING_VERSION: u32 = 1;

/// Profile for a terminal appellate resolution. v0.1 deliberately models one
/// terminal appeal level; multi-level appeals require a future profile/version.
pub const TERMINAL_APPEAL_RESOLUTION_PROFILE: &str = "justice.terminal-appeal-resolution";
pub const TERMINAL_APPEAL_RESOLUTION_VERSION: u32 = 1;

/// Positive completeness evidence for one exact decision's appeal set over an
/// exact interval.
///
/// The owning Justice runtime is responsible for authenticating that this is a
/// genuinely complete checkpoint/coverage artifact. A local empty query is not
/// sufficient to instantiate this semantic claim.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompleteAppealCoverageEvidenceV1 {
    pub coverage_ref: String,
    pub decision_ref: String,
    pub authority_evidence_ref: String,
    pub semantic_profile: String,
    pub semantic_version: u32,
    pub covered_from_unix_ms: u64,
    pub covered_through_unix_ms: u64,
    /// Complete appeal identities observed for this decision in the covered
    /// interval. The no-appeal finality path requires this exact set to be empty.
    pub observed_appeal_refs: Vec<String>,
}

/// One already-authenticated appeal filing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedAppealFilingEvidenceV1 {
    pub appeal_ref: String,
    pub decision_ref: String,
    pub appellant_ref: String,
    pub appeal_number: u8,
    pub semantic_profile: String,
    pub semantic_version: u32,
    pub filed_at_unix_ms: u64,
}

/// Terminal effect of an authenticated appellate resolution.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TerminalAppealDispositionV1 {
    Affirmed,
    Changed,
}

/// One already-authenticated terminal appellate resolution.
///
/// `authority_evidence_ref` binds the independently established appellate
/// authority basis. This crate preserves but does not itself authenticate that
/// authority; issue #358 owns the runtime authority contract.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthenticatedTerminalAppealResolutionEvidenceV1 {
    pub resolution_ref: String,
    pub appeal_ref: String,
    pub decision_ref: String,
    pub authority_evidence_ref: String,
    pub semantic_profile: String,
    pub semantic_version: u32,
    pub resolved_at_unix_ms: u64,
    pub disposition: TerminalAppealDispositionV1,
}

/// Exact positive evidence basis for one finality qualification.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JusticeFinalityQualificationBasisV1 {
    /// Qualify after the appeal window only from a complete, positively
    /// attributable appeal-set coverage artifact.
    NoAppealCoverage {
        decision_ref: String,
        decision_rendered_at_unix_ms: u64,
        appeal_deadline_unix_ms: u64,
        qualification_time_unix_ms: u64,
        coverage: CompleteAppealCoverageEvidenceV1,
    },
    /// Qualify an exact timely appeal whose exact terminal resolution affirmed
    /// the original decision.
    TerminalAppealResolution {
        decision_ref: String,
        decision_rendered_at_unix_ms: u64,
        appeal_deadline_unix_ms: u64,
        qualification_time_unix_ms: u64,
        appeal: AuthenticatedAppealFilingEvidenceV1,
        resolution: AuthenticatedTerminalAppealResolutionEvidenceV1,
    },
}

/// Evidence preserved in a positive finality qualification receipt.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum JusticeFinalityEvidenceReceiptV1 {
    NoAppealCoverage {
        coverage_ref: String,
        authority_evidence_ref: String,
        covered_from_unix_ms: u64,
        covered_through_unix_ms: u64,
    },
    TerminalAppealResolution {
        appeal_ref: String,
        appellant_ref: String,
        appeal_number: u8,
        filed_at_unix_ms: u64,
        resolution_ref: String,
        authority_evidence_ref: String,
        resolved_at_unix_ms: u64,
    },
}

/// Verifier-owned audit receipt explaining one positive finality result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JusticeFinalityQualificationReceiptV1 {
    decision_ref: String,
    semantic_profile: &'static str,
    semantic_version: u32,
    qualification_time_unix_ms: u64,
    evidence: JusticeFinalityEvidenceReceiptV1,
}

impl JusticeFinalityQualificationReceiptV1 {
    #[must_use]
    pub fn decision_ref(&self) -> &str {
        &self.decision_ref
    }

    #[must_use]
    pub const fn semantic_profile(&self) -> &'static str {
        self.semantic_profile
    }

    #[must_use]
    pub const fn semantic_version(&self) -> u32 {
        self.semantic_version
    }

    #[must_use]
    pub const fn qualification_time_unix_ms(&self) -> u64 {
        self.qualification_time_unix_ms
    }

    #[must_use]
    pub const fn evidence(&self) -> &JusticeFinalityEvidenceReceiptV1 {
        &self.evidence
    }
}

/// Positive finality token. Its fields are private and no public constructor is
/// exposed; only `qualify_justice_finality_v1` can mint it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedJusticeFinalityV1 {
    decision_ref: String,
    qualification_time_unix_ms: u64,
    finality: JusticeFinalityBasisV1,
    receipt: JusticeFinalityQualificationReceiptV1,
}

impl QualifiedJusticeFinalityV1 {
    #[must_use]
    pub fn decision_ref(&self) -> &str {
        &self.decision_ref
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
    pub const fn receipt(&self) -> &JusticeFinalityQualificationReceiptV1 {
        &self.receipt
    }

    #[must_use]
    pub fn into_parts(
        self,
    ) -> (
        JusticeFinalityBasisV1,
        JusticeFinalityQualificationReceiptV1,
    ) {
        (self.finality, self.receipt)
    }
}

/// Deterministic denial reasons for v0.1 positive finality qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JusticeFinalityQualificationError {
    EmptyDecisionRef,
    AppealDeadlineBeforeDecision,
    QualificationBeforeDecision,
    EmptyCoverageRef,
    EmptyCoverageAuthorityRef,
    CoverageDecisionMismatch,
    WrongCoverageProfile,
    CoverageEndsBeforeStart,
    CoverageStartsAfterDecision,
    CoverageEndsBeforeQualification,
    QualificationBeforeAppealDeadline,
    EmptyObservedAppealRef,
    DuplicateObservedAppealRef,
    AppealObserved,
    EmptyAppealRef,
    EmptyAppellantRef,
    AppealDecisionMismatch,
    WrongAppealFilingProfile,
    UnsupportedAppealNumber,
    AppealFiledBeforeDecision,
    AppealFiledAfterDeadline,
    AppealFiledAfterQualification,
    EmptyAppealResolutionRef,
    EmptyAppealResolutionAuthorityRef,
    ResolutionDecisionMismatch,
    ResolutionAppealMismatch,
    WrongAppealResolutionProfile,
    ResolutionBeforeAppeal,
    ResolutionAfterQualification,
    AppealChangedDecision,
    OutputConstructionFailed,
}

impl fmt::Display for JusticeFinalityQualificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "justice finality qualification denied: {self:?}")
    }
}

impl std::error::Error for JusticeFinalityQualificationError {}

/// Qualify one exact positive finality basis without ambient time or runtime
/// authority. The owning Justice adapter must authenticate every evidence value
/// before invoking this function.
pub fn qualify_justice_finality_v1(
    basis: JusticeFinalityQualificationBasisV1,
) -> Result<QualifiedJusticeFinalityV1, JusticeFinalityQualificationError> {
    match basis {
        JusticeFinalityQualificationBasisV1::NoAppealCoverage {
            decision_ref,
            decision_rendered_at_unix_ms,
            appeal_deadline_unix_ms,
            qualification_time_unix_ms,
            coverage,
        } => qualify_no_appeal_coverage(
            decision_ref,
            decision_rendered_at_unix_ms,
            appeal_deadline_unix_ms,
            qualification_time_unix_ms,
            coverage,
        ),
        JusticeFinalityQualificationBasisV1::TerminalAppealResolution {
            decision_ref,
            decision_rendered_at_unix_ms,
            appeal_deadline_unix_ms,
            qualification_time_unix_ms,
            appeal,
            resolution,
        } => qualify_terminal_appeal_resolution(
            decision_ref,
            decision_rendered_at_unix_ms,
            appeal_deadline_unix_ms,
            qualification_time_unix_ms,
            appeal,
            resolution,
        ),
    }
}

fn validate_common(
    decision_ref: &str,
    decision_rendered_at_unix_ms: u64,
    appeal_deadline_unix_ms: u64,
    qualification_time_unix_ms: u64,
) -> Result<(), JusticeFinalityQualificationError> {
    if decision_ref.trim().is_empty() {
        return Err(JusticeFinalityQualificationError::EmptyDecisionRef);
    }
    if appeal_deadline_unix_ms < decision_rendered_at_unix_ms {
        return Err(JusticeFinalityQualificationError::AppealDeadlineBeforeDecision);
    }
    if qualification_time_unix_ms < decision_rendered_at_unix_ms {
        return Err(JusticeFinalityQualificationError::QualificationBeforeDecision);
    }
    Ok(())
}

fn qualify_no_appeal_coverage(
    decision_ref: String,
    decision_rendered_at_unix_ms: u64,
    appeal_deadline_unix_ms: u64,
    qualification_time_unix_ms: u64,
    coverage: CompleteAppealCoverageEvidenceV1,
) -> Result<QualifiedJusticeFinalityV1, JusticeFinalityQualificationError> {
    validate_common(
        &decision_ref,
        decision_rendered_at_unix_ms,
        appeal_deadline_unix_ms,
        qualification_time_unix_ms,
    )?;

    if coverage.coverage_ref.trim().is_empty() {
        return Err(JusticeFinalityQualificationError::EmptyCoverageRef);
    }
    if coverage.authority_evidence_ref.trim().is_empty() {
        return Err(JusticeFinalityQualificationError::EmptyCoverageAuthorityRef);
    }
    if coverage.decision_ref != decision_ref {
        return Err(JusticeFinalityQualificationError::CoverageDecisionMismatch);
    }
    if coverage.semantic_profile != COMPLETE_APPEAL_COVERAGE_PROFILE
        || coverage.semantic_version != COMPLETE_APPEAL_COVERAGE_VERSION
    {
        return Err(JusticeFinalityQualificationError::WrongCoverageProfile);
    }
    if coverage.covered_through_unix_ms < coverage.covered_from_unix_ms {
        return Err(JusticeFinalityQualificationError::CoverageEndsBeforeStart);
    }
    if coverage.covered_from_unix_ms > decision_rendered_at_unix_ms {
        return Err(JusticeFinalityQualificationError::CoverageStartsAfterDecision);
    }
    if coverage.covered_through_unix_ms < qualification_time_unix_ms {
        return Err(JusticeFinalityQualificationError::CoverageEndsBeforeQualification);
    }
    if qualification_time_unix_ms < appeal_deadline_unix_ms {
        return Err(JusticeFinalityQualificationError::QualificationBeforeAppealDeadline);
    }

    let mut observed = BTreeSet::new();
    for appeal_ref in &coverage.observed_appeal_refs {
        if appeal_ref.trim().is_empty() {
            return Err(JusticeFinalityQualificationError::EmptyObservedAppealRef);
        }
        if !observed.insert(appeal_ref.clone()) {
            return Err(JusticeFinalityQualificationError::DuplicateObservedAppealRef);
        }
    }
    if !observed.is_empty() {
        return Err(JusticeFinalityQualificationError::AppealObserved);
    }

    let finality = JusticeFinalityBasisV1::AppealWindowExpired {
        appeal_deadline_unix_ms,
        qualified_at_unix_ms: qualification_time_unix_ms,
        no_live_appeal_evidence_ref: coverage.coverage_ref.clone(),
    };
    finality
        .validate()
        .map_err(|_| JusticeFinalityQualificationError::OutputConstructionFailed)?;

    let receipt = JusticeFinalityQualificationReceiptV1 {
        decision_ref: decision_ref.clone(),
        semantic_profile: FINALITY_QUALIFICATION_PROFILE,
        semantic_version: FINALITY_QUALIFICATION_VERSION,
        qualification_time_unix_ms,
        evidence: JusticeFinalityEvidenceReceiptV1::NoAppealCoverage {
            coverage_ref: coverage.coverage_ref,
            authority_evidence_ref: coverage.authority_evidence_ref,
            covered_from_unix_ms: coverage.covered_from_unix_ms,
            covered_through_unix_ms: coverage.covered_through_unix_ms,
        },
    };

    Ok(QualifiedJusticeFinalityV1 {
        decision_ref,
        qualification_time_unix_ms,
        finality,
        receipt,
    })
}

fn qualify_terminal_appeal_resolution(
    decision_ref: String,
    decision_rendered_at_unix_ms: u64,
    appeal_deadline_unix_ms: u64,
    qualification_time_unix_ms: u64,
    appeal: AuthenticatedAppealFilingEvidenceV1,
    resolution: AuthenticatedTerminalAppealResolutionEvidenceV1,
) -> Result<QualifiedJusticeFinalityV1, JusticeFinalityQualificationError> {
    validate_common(
        &decision_ref,
        decision_rendered_at_unix_ms,
        appeal_deadline_unix_ms,
        qualification_time_unix_ms,
    )?;

    if appeal.appeal_ref.trim().is_empty() {
        return Err(JusticeFinalityQualificationError::EmptyAppealRef);
    }
    if appeal.appellant_ref.trim().is_empty() {
        return Err(JusticeFinalityQualificationError::EmptyAppellantRef);
    }
    if appeal.decision_ref != decision_ref {
        return Err(JusticeFinalityQualificationError::AppealDecisionMismatch);
    }
    if appeal.semantic_profile != APPEAL_FILING_PROFILE
        || appeal.semantic_version != APPEAL_FILING_VERSION
    {
        return Err(JusticeFinalityQualificationError::WrongAppealFilingProfile);
    }
    if appeal.appeal_number != 1 {
        return Err(JusticeFinalityQualificationError::UnsupportedAppealNumber);
    }
    if appeal.filed_at_unix_ms < decision_rendered_at_unix_ms {
        return Err(JusticeFinalityQualificationError::AppealFiledBeforeDecision);
    }
    if appeal.filed_at_unix_ms > appeal_deadline_unix_ms {
        return Err(JusticeFinalityQualificationError::AppealFiledAfterDeadline);
    }
    if appeal.filed_at_unix_ms > qualification_time_unix_ms {
        return Err(JusticeFinalityQualificationError::AppealFiledAfterQualification);
    }

    if resolution.resolution_ref.trim().is_empty() {
        return Err(JusticeFinalityQualificationError::EmptyAppealResolutionRef);
    }
    if resolution.authority_evidence_ref.trim().is_empty() {
        return Err(JusticeFinalityQualificationError::EmptyAppealResolutionAuthorityRef);
    }
    if resolution.decision_ref != decision_ref {
        return Err(JusticeFinalityQualificationError::ResolutionDecisionMismatch);
    }
    if resolution.appeal_ref != appeal.appeal_ref {
        return Err(JusticeFinalityQualificationError::ResolutionAppealMismatch);
    }
    if resolution.semantic_profile != TERMINAL_APPEAL_RESOLUTION_PROFILE
        || resolution.semantic_version != TERMINAL_APPEAL_RESOLUTION_VERSION
    {
        return Err(JusticeFinalityQualificationError::WrongAppealResolutionProfile);
    }
    if resolution.resolved_at_unix_ms < appeal.filed_at_unix_ms {
        return Err(JusticeFinalityQualificationError::ResolutionBeforeAppeal);
    }
    if resolution.resolved_at_unix_ms > qualification_time_unix_ms {
        return Err(JusticeFinalityQualificationError::ResolutionAfterQualification);
    }
    if resolution.disposition == TerminalAppealDispositionV1::Changed {
        return Err(JusticeFinalityQualificationError::AppealChangedDecision);
    }

    let finality = JusticeFinalityBasisV1::AppealResolved {
        appeal_ref: appeal.appeal_ref.clone(),
        appeal_resolution_ref: resolution.resolution_ref.clone(),
    };
    finality
        .validate()
        .map_err(|_| JusticeFinalityQualificationError::OutputConstructionFailed)?;

    let receipt = JusticeFinalityQualificationReceiptV1 {
        decision_ref: decision_ref.clone(),
        semantic_profile: FINALITY_QUALIFICATION_PROFILE,
        semantic_version: FINALITY_QUALIFICATION_VERSION,
        qualification_time_unix_ms,
        evidence: JusticeFinalityEvidenceReceiptV1::TerminalAppealResolution {
            appeal_ref: appeal.appeal_ref,
            appellant_ref: appeal.appellant_ref,
            appeal_number: appeal.appeal_number,
            filed_at_unix_ms: appeal.filed_at_unix_ms,
            resolution_ref: resolution.resolution_ref,
            authority_evidence_ref: resolution.authority_evidence_ref,
            resolved_at_unix_ms: resolution.resolved_at_unix_ms,
        },
    };

    Ok(QualifiedJusticeFinalityV1 {
        decision_ref,
        qualification_time_unix_ms,
        finality,
        receipt,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn coverage_basis() -> JusticeFinalityQualificationBasisV1 {
        JusticeFinalityQualificationBasisV1::NoAppealCoverage {
            decision_ref: "decision-action:1".into(),
            decision_rendered_at_unix_ms: 100,
            appeal_deadline_unix_ms: 200,
            qualification_time_unix_ms: 250,
            coverage: CompleteAppealCoverageEvidenceV1 {
                coverage_ref: "appeal-coverage:decision-1:through-250".into(),
                decision_ref: "decision-action:1".into(),
                authority_evidence_ref: "justice-finality-authority:1".into(),
                semantic_profile: COMPLETE_APPEAL_COVERAGE_PROFILE.into(),
                semantic_version: COMPLETE_APPEAL_COVERAGE_VERSION,
                covered_from_unix_ms: 100,
                covered_through_unix_ms: 250,
                observed_appeal_refs: vec![],
            },
        }
    }

    fn resolved_basis() -> JusticeFinalityQualificationBasisV1 {
        JusticeFinalityQualificationBasisV1::TerminalAppealResolution {
            decision_ref: "decision-action:1".into(),
            decision_rendered_at_unix_ms: 100,
            appeal_deadline_unix_ms: 200,
            qualification_time_unix_ms: 190,
            appeal: AuthenticatedAppealFilingEvidenceV1 {
                appeal_ref: "appeal-action:1".into(),
                decision_ref: "decision-action:1".into(),
                appellant_ref: "party:merchant".into(),
                appeal_number: 1,
                semantic_profile: APPEAL_FILING_PROFILE.into(),
                semantic_version: APPEAL_FILING_VERSION,
                filed_at_unix_ms: 150,
            },
            resolution: AuthenticatedTerminalAppealResolutionEvidenceV1 {
                resolution_ref: "appeal-resolution:1".into(),
                appeal_ref: "appeal-action:1".into(),
                decision_ref: "decision-action:1".into(),
                authority_evidence_ref: "appellate-authority:panel-receipt:1".into(),
                semantic_profile: TERMINAL_APPEAL_RESOLUTION_PROFILE.into(),
                semantic_version: TERMINAL_APPEAL_RESOLUTION_VERSION,
                resolved_at_unix_ms: 180,
                disposition: TerminalAppealDispositionV1::Affirmed,
            },
        }
    }

    #[test]
    fn complete_positive_coverage_qualifies_no_appeal_finality() {
        let qualified = qualify_justice_finality_v1(coverage_basis()).unwrap();
        assert_eq!(qualified.decision_ref(), "decision-action:1");
        assert_eq!(qualified.qualification_time_unix_ms(), 250);
        assert_eq!(
            qualified.receipt().semantic_profile(),
            FINALITY_QUALIFICATION_PROFILE
        );
        assert_eq!(qualified.receipt().semantic_version(), 1);
        assert!(matches!(
            qualified.finality(),
            JusticeFinalityBasisV1::AppealWindowExpired {
                no_live_appeal_evidence_ref,
                ..
            } if no_live_appeal_evidence_ref == "appeal-coverage:decision-1:through-250"
        ));
    }

    #[test]
    fn local_absence_without_positive_coverage_ref_is_denied() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage { coverage, .. } = &mut basis else {
            unreachable!();
        };
        coverage.coverage_ref.clear();
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::EmptyCoverageRef)
        );
    }

    #[test]
    fn coverage_without_authority_provenance_is_denied() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage { coverage, .. } = &mut basis else {
            unreachable!();
        };
        coverage.authority_evidence_ref.clear();
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::EmptyCoverageAuthorityRef)
        );
    }

    #[test]
    fn coverage_profile_drift_is_denied() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage { coverage, .. } = &mut basis else {
            unreachable!();
        };
        coverage.semantic_version = 2;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::WrongCoverageProfile)
        );
    }

    #[test]
    fn partial_coverage_starting_after_decision_is_denied() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage { coverage, .. } = &mut basis else {
            unreachable!();
        };
        coverage.covered_from_unix_ms = 101;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::CoverageStartsAfterDecision)
        );
    }

    #[test]
    fn stale_coverage_ending_before_qualification_is_denied() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage { coverage, .. } = &mut basis else {
            unreachable!();
        };
        coverage.covered_through_unix_ms = 249;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::CoverageEndsBeforeQualification)
        );
    }

    #[test]
    fn qualification_before_appeal_deadline_is_denied() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage {
            qualification_time_unix_ms,
            coverage,
            ..
        } = &mut basis
        else {
            unreachable!();
        };
        *qualification_time_unix_ms = 199;
        coverage.covered_through_unix_ms = 250;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::QualificationBeforeAppealDeadline)
        );
    }

    #[test]
    fn observed_appeal_blocks_no_appeal_finality() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage { coverage, .. } = &mut basis else {
            unreachable!();
        };
        coverage.observed_appeal_refs.push("appeal-action:1".into());
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::AppealObserved)
        );
    }

    #[test]
    fn duplicate_observed_appeal_identity_is_denied_before_absence_claim() {
        let mut basis = coverage_basis();
        let JusticeFinalityQualificationBasisV1::NoAppealCoverage { coverage, .. } = &mut basis else {
            unreachable!();
        };
        coverage.observed_appeal_refs = vec!["appeal:1".into(), "appeal:1".into()];
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::DuplicateObservedAppealRef)
        );
    }

    #[test]
    fn exact_terminal_affirmance_qualifies() {
        let qualified = qualify_justice_finality_v1(resolved_basis()).unwrap();
        assert!(matches!(
            qualified.finality(),
            JusticeFinalityBasisV1::AppealResolved {
                appeal_ref,
                appeal_resolution_ref,
            } if appeal_ref == "appeal-action:1" && appeal_resolution_ref == "appeal-resolution:1"
        ));
        assert!(matches!(
            qualified.receipt().evidence(),
            JusticeFinalityEvidenceReceiptV1::TerminalAppealResolution {
                authority_evidence_ref,
                ..
            } if authority_evidence_ref == "appellate-authority:panel-receipt:1"
        ));
    }

    #[test]
    fn changed_appeal_denies_original_decision_finality() {
        let mut basis = resolved_basis();
        let JusticeFinalityQualificationBasisV1::TerminalAppealResolution { resolution, .. } = &mut basis else {
            unreachable!();
        };
        resolution.disposition = TerminalAppealDispositionV1::Changed;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::AppealChangedDecision)
        );
    }

    #[test]
    fn late_appeal_is_denied_by_v0_1_strict_deadline_profile() {
        let mut basis = resolved_basis();
        let JusticeFinalityQualificationBasisV1::TerminalAppealResolution {
            appeal,
            qualification_time_unix_ms,
            resolution,
            ..
        } = &mut basis
        else {
            unreachable!();
        };
        appeal.filed_at_unix_ms = 201;
        resolution.resolved_at_unix_ms = 210;
        *qualification_time_unix_ms = 220;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::AppealFiledAfterDeadline)
        );
    }

    #[test]
    fn second_level_appeal_requires_a_future_profile() {
        let mut basis = resolved_basis();
        let JusticeFinalityQualificationBasisV1::TerminalAppealResolution { appeal, .. } = &mut basis else {
            unreachable!();
        };
        appeal.appeal_number = 2;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::UnsupportedAppealNumber)
        );
    }

    #[test]
    fn resolution_before_filing_is_denied() {
        let mut basis = resolved_basis();
        let JusticeFinalityQualificationBasisV1::TerminalAppealResolution { resolution, .. } = &mut basis else {
            unreachable!();
        };
        resolution.resolved_at_unix_ms = 149;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::ResolutionBeforeAppeal)
        );
    }

    #[test]
    fn future_resolution_is_denied() {
        let mut basis = resolved_basis();
        let JusticeFinalityQualificationBasisV1::TerminalAppealResolution { resolution, .. } = &mut basis else {
            unreachable!();
        };
        resolution.resolved_at_unix_ms = 191;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::ResolutionAfterQualification)
        );
    }

    #[test]
    fn resolution_must_match_exact_appeal() {
        let mut basis = resolved_basis();
        let JusticeFinalityQualificationBasisV1::TerminalAppealResolution { resolution, .. } = &mut basis else {
            unreachable!();
        };
        resolution.appeal_ref = "appeal-action:other".into();
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::ResolutionAppealMismatch)
        );
    }

    #[test]
    fn terminal_resolution_profile_drift_is_denied() {
        let mut basis = resolved_basis();
        let JusticeFinalityQualificationBasisV1::TerminalAppealResolution { resolution, .. } = &mut basis else {
            unreachable!();
        };
        resolution.semantic_version = 2;
        assert_eq!(
            qualify_justice_finality_v1(basis),
            Err(JusticeFinalityQualificationError::WrongAppealResolutionProfile)
        );
    }

    #[test]
    fn same_exact_positive_basis_is_deterministic() {
        let one = qualify_justice_finality_v1(resolved_basis()).unwrap();
        let two = qualify_justice_finality_v1(resolved_basis()).unwrap();
        assert_eq!(one, two);
    }
}
