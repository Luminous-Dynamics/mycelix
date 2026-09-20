// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Profile-relative admission of reciprocity satisfaction evidence.
//!
//! STEW-011A evaluates the structural relationship among receipts, an obligation
//! identity, an explicit evaluation profile, currentness, beneficiary acceptance
//! assertions, and explicit conflict-resolution evidence. It never produces
//! universal satisfaction, legal discharge, or runtime authorization.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};
use mycelix_stewardship_reciprocity::ReciprocityObligationIdV1;
use mycelix_stewardship_reciprocity_receipts::{
    ReciprocityReceiptV1, ReciprocityReportedOutcomeV1,
};

pub const RECIPROCITY_SATISFACTION_ADMISSION_PROFILE_V1: &str =
    "mycelix/reciprocity-satisfaction-admission/v1";
pub const MAX_SATISFACTION_RECEIPTS_V1: usize = 32;
pub const MAX_SATISFACTION_EVIDENCE_REFS_V1: usize = 32;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SatisfactionEvidenceRefV1(CanonicalIdV1);

impl SatisfactionEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Closed v1 admission vocabulary.
///
/// `AdmittedSatisfiedUnderProfile` is intentionally profile-relative and is not
/// a universal `Satisfied` state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SatisfactionAdmissionDispositionV1 {
    AdmittedSatisfiedUnderProfile,
    RejectedUnderProfile,
    DisputedUnderProfile,
    Indeterminate,
}

impl SatisfactionAdmissionDispositionV1 {
    pub const fn is_positive(self) -> bool {
        matches!(self, Self::AdmittedSatisfiedUnderProfile)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum BeneficiaryAcceptanceRequirementV1 {
    NotRequiredByProfile,
    RequiredByProfile,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum BeneficiaryAcceptanceAssertionV1 {
    NotRequired,
    AssertedAccepted,
    AssertedRejected,
    Disputed,
    Indeterminate,
}

/// Currentness is independent of the satisfaction disposition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SatisfactionCurrentnessAssertionV1 {
    AssertedCurrent,
    AssertedRevoked,
    AssertedSuperseded,
    AssertedExpired,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SatisfactionAdmissionErrorV1 {
    NoReceipts,
    TooManyReceipts,
    DuplicateReceipt,
    ReceiptObligationMismatch,
    NoEvaluationEvidence,
    TooManyEvaluationEvidenceReferences,
    DuplicateEvaluationEvidenceReference,
    NoCurrentnessEvidence,
    TooManyCurrentnessEvidenceReferences,
    DuplicateCurrentnessEvidenceReference,
    TooManyConflictResolutionEvidenceReferences,
    DuplicateConflictResolutionEvidenceReference,
    AcceptanceRequirementMismatch,
    NoAcceptanceEvidence,
    TooManyAcceptanceEvidenceReferences,
    DuplicateAcceptanceEvidenceReference,
    PositiveAdmissionNotCurrent,
    PositiveAdmissionLacksFulfilledReport,
    PositiveAdmissionMissingRequiredAcceptance,
    PositiveAdmissionHasUnresolvedReceiptConflict,
}

impl fmt::Display for SatisfactionAdmissionErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::NoReceipts => "satisfaction admission requires at least one receipt",
            Self::TooManyReceipts => "too many receipts for v1 satisfaction admission",
            Self::DuplicateReceipt => "duplicate receipt in satisfaction admission",
            Self::ReceiptObligationMismatch => {
                "receipt references a different reciprocity obligation"
            }
            Self::NoEvaluationEvidence => "satisfaction admission requires evaluation evidence",
            Self::TooManyEvaluationEvidenceReferences => {
                "too many evaluation evidence references"
            }
            Self::DuplicateEvaluationEvidenceReference => {
                "duplicate evaluation evidence reference"
            }
            Self::NoCurrentnessEvidence => "satisfaction admission requires currentness evidence",
            Self::TooManyCurrentnessEvidenceReferences => {
                "too many currentness evidence references"
            }
            Self::DuplicateCurrentnessEvidenceReference => {
                "duplicate currentness evidence reference"
            }
            Self::TooManyConflictResolutionEvidenceReferences => {
                "too many conflict-resolution evidence references"
            }
            Self::DuplicateConflictResolutionEvidenceReference => {
                "duplicate conflict-resolution evidence reference"
            }
            Self::AcceptanceRequirementMismatch => {
                "beneficiary acceptance assertion does not match profile requirement"
            }
            Self::NoAcceptanceEvidence => "required beneficiary acceptance requires evidence",
            Self::TooManyAcceptanceEvidenceReferences => {
                "too many beneficiary acceptance evidence references"
            }
            Self::DuplicateAcceptanceEvidenceReference => {
                "duplicate beneficiary acceptance evidence reference"
            }
            Self::PositiveAdmissionNotCurrent => {
                "positive satisfaction admission must be asserted current"
            }
            Self::PositiveAdmissionLacksFulfilledReport => {
                "positive satisfaction admission lacks a ReportedFulfilled receipt"
            }
            Self::PositiveAdmissionMissingRequiredAcceptance => {
                "positive satisfaction admission lacks required beneficiary acceptance"
            }
            Self::PositiveAdmissionHasUnresolvedReceiptConflict => {
                "divergent receipt outcomes require explicit conflict-resolution evidence before positive admission"
            }
        };
        f.write_str(message)
    }
}

fn validate_refs(
    refs: &[SatisfactionEvidenceRefV1],
    empty: SatisfactionAdmissionErrorV1,
    too_many: SatisfactionAdmissionErrorV1,
    duplicate: SatisfactionAdmissionErrorV1,
) -> Result<(), SatisfactionAdmissionErrorV1> {
    if refs.is_empty() {
        return Err(empty);
    }
    validate_optional_refs(refs, too_many, duplicate)
}

fn validate_optional_refs(
    refs: &[SatisfactionEvidenceRefV1],
    too_many: SatisfactionAdmissionErrorV1,
    duplicate: SatisfactionAdmissionErrorV1,
) -> Result<(), SatisfactionAdmissionErrorV1> {
    if refs.len() > MAX_SATISFACTION_EVIDENCE_REFS_V1 {
        return Err(too_many);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(duplicate);
        }
    }
    Ok(())
}

fn receipt_outcomes_diverge(receipts: &[ReciprocityReceiptV1]) -> bool {
    let Some(first) = receipts.first() else {
        return false;
    };
    receipts
        .iter()
        .skip(1)
        .any(|receipt| receipt.outcome() != first.outcome())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReciprocitySatisfactionAdmissionV1 {
    admission_id: CanonicalIdV1,
    obligation_id: ReciprocityObligationIdV1,
    receipts: Vec<ReciprocityReceiptV1>,
    evaluation_profile_ref: CanonicalIdV1,
    evaluated_by_ref: CanonicalIdV1,
    disposition: SatisfactionAdmissionDispositionV1,
    beneficiary_acceptance_requirement: BeneficiaryAcceptanceRequirementV1,
    beneficiary_acceptance: BeneficiaryAcceptanceAssertionV1,
    currentness: SatisfactionCurrentnessAssertionV1,
    evaluation_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
    conflict_resolution_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
    acceptance_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
    currentness_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
}

impl ReciprocitySatisfactionAdmissionV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        admission_id: CanonicalIdV1,
        obligation_id: ReciprocityObligationIdV1,
        receipts: Vec<ReciprocityReceiptV1>,
        evaluation_profile_ref: CanonicalIdV1,
        evaluated_by_ref: CanonicalIdV1,
        disposition: SatisfactionAdmissionDispositionV1,
        beneficiary_acceptance_requirement: BeneficiaryAcceptanceRequirementV1,
        beneficiary_acceptance: BeneficiaryAcceptanceAssertionV1,
        currentness: SatisfactionCurrentnessAssertionV1,
        evaluation_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
        conflict_resolution_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
        acceptance_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
        currentness_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
    ) -> Result<Self, SatisfactionAdmissionErrorV1> {
        if receipts.is_empty() {
            return Err(SatisfactionAdmissionErrorV1::NoReceipts);
        }
        if receipts.len() > MAX_SATISFACTION_RECEIPTS_V1 {
            return Err(SatisfactionAdmissionErrorV1::TooManyReceipts);
        }
        for (index, receipt) in receipts.iter().enumerate() {
            if receipt.obligation_id() != &obligation_id {
                return Err(SatisfactionAdmissionErrorV1::ReceiptObligationMismatch);
            }
            if receipts[..index]
                .iter()
                .any(|previous| previous.receipt_id() == receipt.receipt_id())
            {
                return Err(SatisfactionAdmissionErrorV1::DuplicateReceipt);
            }
        }

        validate_refs(
            &evaluation_evidence_refs,
            SatisfactionAdmissionErrorV1::NoEvaluationEvidence,
            SatisfactionAdmissionErrorV1::TooManyEvaluationEvidenceReferences,
            SatisfactionAdmissionErrorV1::DuplicateEvaluationEvidenceReference,
        )?;
        validate_refs(
            &currentness_evidence_refs,
            SatisfactionAdmissionErrorV1::NoCurrentnessEvidence,
            SatisfactionAdmissionErrorV1::TooManyCurrentnessEvidenceReferences,
            SatisfactionAdmissionErrorV1::DuplicateCurrentnessEvidenceReference,
        )?;
        validate_optional_refs(
            &conflict_resolution_evidence_refs,
            SatisfactionAdmissionErrorV1::TooManyConflictResolutionEvidenceReferences,
            SatisfactionAdmissionErrorV1::DuplicateConflictResolutionEvidenceReference,
        )?;

        match beneficiary_acceptance_requirement {
            BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile => {
                if beneficiary_acceptance != BeneficiaryAcceptanceAssertionV1::NotRequired
                    || !acceptance_evidence_refs.is_empty()
                {
                    return Err(SatisfactionAdmissionErrorV1::AcceptanceRequirementMismatch);
                }
            }
            BeneficiaryAcceptanceRequirementV1::RequiredByProfile => {
                if beneficiary_acceptance == BeneficiaryAcceptanceAssertionV1::NotRequired {
                    return Err(SatisfactionAdmissionErrorV1::AcceptanceRequirementMismatch);
                }
                validate_refs(
                    &acceptance_evidence_refs,
                    SatisfactionAdmissionErrorV1::NoAcceptanceEvidence,
                    SatisfactionAdmissionErrorV1::TooManyAcceptanceEvidenceReferences,
                    SatisfactionAdmissionErrorV1::DuplicateAcceptanceEvidenceReference,
                )?;
            }
        }

        if disposition.is_positive() {
            if currentness != SatisfactionCurrentnessAssertionV1::AssertedCurrent {
                return Err(SatisfactionAdmissionErrorV1::PositiveAdmissionNotCurrent);
            }
            if !receipts.iter().any(|receipt| {
                receipt.outcome() == ReciprocityReportedOutcomeV1::ReportedFulfilled
            }) {
                return Err(SatisfactionAdmissionErrorV1::PositiveAdmissionLacksFulfilledReport);
            }
            if beneficiary_acceptance_requirement
                == BeneficiaryAcceptanceRequirementV1::RequiredByProfile
                && beneficiary_acceptance != BeneficiaryAcceptanceAssertionV1::AssertedAccepted
            {
                return Err(
                    SatisfactionAdmissionErrorV1::PositiveAdmissionMissingRequiredAcceptance,
                );
            }
            if receipt_outcomes_diverge(&receipts) && conflict_resolution_evidence_refs.is_empty() {
                return Err(
                    SatisfactionAdmissionErrorV1::PositiveAdmissionHasUnresolvedReceiptConflict,
                );
            }
        }

        Ok(Self {
            admission_id,
            obligation_id,
            receipts,
            evaluation_profile_ref,
            evaluated_by_ref,
            disposition,
            beneficiary_acceptance_requirement,
            beneficiary_acceptance,
            currentness,
            evaluation_evidence_refs,
            conflict_resolution_evidence_refs,
            acceptance_evidence_refs,
            currentness_evidence_refs,
        })
    }

    pub fn admission_id(&self) -> &CanonicalIdV1 {
        &self.admission_id
    }

    pub fn obligation_id(&self) -> &ReciprocityObligationIdV1 {
        &self.obligation_id
    }

    pub fn receipts(&self) -> &[ReciprocityReceiptV1] {
        &self.receipts
    }

    pub fn evaluation_profile_ref(&self) -> &CanonicalIdV1 {
        &self.evaluation_profile_ref
    }

    pub fn evaluated_by_ref(&self) -> &CanonicalIdV1 {
        &self.evaluated_by_ref
    }

    pub const fn disposition(&self) -> SatisfactionAdmissionDispositionV1 {
        self.disposition
    }

    pub const fn beneficiary_acceptance_requirement(&self) -> BeneficiaryAcceptanceRequirementV1 {
        self.beneficiary_acceptance_requirement
    }

    pub const fn beneficiary_acceptance(&self) -> BeneficiaryAcceptanceAssertionV1 {
        self.beneficiary_acceptance
    }

    pub const fn currentness(&self) -> SatisfactionCurrentnessAssertionV1 {
        self.currentness
    }

    pub fn evaluation_evidence_refs(&self) -> &[SatisfactionEvidenceRefV1] {
        &self.evaluation_evidence_refs
    }

    pub fn conflict_resolution_evidence_refs(&self) -> &[SatisfactionEvidenceRefV1] {
        &self.conflict_resolution_evidence_refs
    }

    pub fn acceptance_evidence_refs(&self) -> &[SatisfactionEvidenceRefV1] {
        &self.acceptance_evidence_refs
    }

    pub fn currentness_evidence_refs(&self) -> &[SatisfactionEvidenceRefV1] {
        &self.currentness_evidence_refs
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_reciprocity_receipts::{
        ReciprocityReceiptEvidenceRefV1, ReciprocityReceiptIdV1,
    };

    fn id(value: &str) -> CanonicalIdV1 {
        CanonicalIdV1::new(value).unwrap()
    }

    fn evidence(value: &str) -> SatisfactionEvidenceRefV1 {
        SatisfactionEvidenceRefV1::new(value).unwrap()
    }

    fn obligation(value: &str) -> ReciprocityObligationIdV1 {
        ReciprocityObligationIdV1::new(value).unwrap()
    }

    fn receipt(
        id_value: &str,
        obligation_id: ReciprocityObligationIdV1,
        outcome: ReciprocityReportedOutcomeV1,
    ) -> ReciprocityReceiptV1 {
        ReciprocityReceiptV1::new(
            ReciprocityReceiptIdV1::new(id_value).unwrap(),
            obligation_id,
            id("reporter:1"),
            id("performer:1"),
            id("event:1"),
            outcome,
            vec![ReciprocityReceiptEvidenceRefV1::new("receipt-evidence:1").unwrap()],
        )
        .unwrap()
    }

    #[allow(clippy::too_many_arguments)]
    fn admission_with_conflict_evidence(
        receipts: Vec<ReciprocityReceiptV1>,
        obligation_id: ReciprocityObligationIdV1,
        disposition: SatisfactionAdmissionDispositionV1,
        requirement: BeneficiaryAcceptanceRequirementV1,
        acceptance: BeneficiaryAcceptanceAssertionV1,
        currentness: SatisfactionCurrentnessAssertionV1,
        conflict_resolution_evidence_refs: Vec<SatisfactionEvidenceRefV1>,
    ) -> Result<ReciprocitySatisfactionAdmissionV1, SatisfactionAdmissionErrorV1> {
        let acceptance_evidence_refs = if requirement
            == BeneficiaryAcceptanceRequirementV1::RequiredByProfile
        {
            vec![evidence("evidence:acceptance:1")]
        } else {
            vec![]
        };
        ReciprocitySatisfactionAdmissionV1::new(
            id("satisfaction-admission:1"),
            obligation_id,
            receipts,
            id("satisfaction-profile:1"),
            id("evaluator:1"),
            disposition,
            requirement,
            acceptance,
            currentness,
            vec![evidence("evidence:evaluation:1")],
            conflict_resolution_evidence_refs,
            acceptance_evidence_refs,
            vec![evidence("evidence:currentness:1")],
        )
    }

    fn admission(
        receipts: Vec<ReciprocityReceiptV1>,
        obligation_id: ReciprocityObligationIdV1,
        disposition: SatisfactionAdmissionDispositionV1,
        requirement: BeneficiaryAcceptanceRequirementV1,
        acceptance: BeneficiaryAcceptanceAssertionV1,
        currentness: SatisfactionCurrentnessAssertionV1,
    ) -> Result<ReciprocitySatisfactionAdmissionV1, SatisfactionAdmissionErrorV1> {
        admission_with_conflict_evidence(
            receipts,
            obligation_id,
            disposition,
            requirement,
            acceptance,
            currentness,
            vec![],
        )
    }

    #[test]
    fn fulfilled_report_plus_required_acceptance_yields_only_profile_relative_admission() {
        let obligation_id = obligation("obligation:1");
        let record = admission(
            vec![receipt(
                "receipt:1",
                obligation_id.clone(),
                ReciprocityReportedOutcomeV1::ReportedFulfilled,
            )],
            obligation_id,
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::RequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::AssertedAccepted,
            SatisfactionCurrentnessAssertionV1::AssertedCurrent,
        )
        .unwrap();
        assert_eq!(
            record.disposition(),
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile
        );
    }

    #[test]
    fn partial_report_alone_cannot_be_promoted_to_satisfied() {
        let obligation_id = obligation("obligation:1");
        let result = admission(
            vec![receipt(
                "receipt:1",
                obligation_id.clone(),
                ReciprocityReportedOutcomeV1::ReportedPartiallyFulfilled,
            )],
            obligation_id,
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::NotRequired,
            SatisfactionCurrentnessAssertionV1::AssertedCurrent,
        );
        assert_eq!(
            result,
            Err(SatisfactionAdmissionErrorV1::PositiveAdmissionLacksFulfilledReport)
        );
    }

    #[test]
    fn divergent_receipts_block_positive_admission_without_resolution_evidence() {
        let obligation_id = obligation("obligation:1");
        let result = admission(
            vec![
                receipt(
                    "receipt:1",
                    obligation_id.clone(),
                    ReciprocityReportedOutcomeV1::ReportedFulfilled,
                ),
                receipt(
                    "receipt:2",
                    obligation_id.clone(),
                    ReciprocityReportedOutcomeV1::Disputed,
                ),
            ],
            obligation_id,
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::NotRequired,
            SatisfactionCurrentnessAssertionV1::AssertedCurrent,
        );
        assert_eq!(
            result,
            Err(SatisfactionAdmissionErrorV1::PositiveAdmissionHasUnresolvedReceiptConflict)
        );
    }

    #[test]
    fn divergent_receipts_can_support_only_profile_relative_positive_admission_with_explicit_resolution_evidence() {
        let obligation_id = obligation("obligation:1");
        let record = admission_with_conflict_evidence(
            vec![
                receipt(
                    "receipt:1",
                    obligation_id.clone(),
                    ReciprocityReportedOutcomeV1::ReportedFulfilled,
                ),
                receipt(
                    "receipt:2",
                    obligation_id.clone(),
                    ReciprocityReportedOutcomeV1::Disputed,
                ),
            ],
            obligation_id,
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::NotRequired,
            SatisfactionCurrentnessAssertionV1::AssertedCurrent,
            vec![evidence("evidence:conflict-resolution:1")],
        )
        .unwrap();
        assert_eq!(
            record.disposition(),
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile
        );
        assert_eq!(record.conflict_resolution_evidence_refs().len(), 1);
    }

    #[test]
    fn failed_disputed_or_indeterminate_report_cannot_support_positive_admission_alone() {
        for outcome in [
            ReciprocityReportedOutcomeV1::ReportedFailed,
            ReciprocityReportedOutcomeV1::Disputed,
            ReciprocityReportedOutcomeV1::Indeterminate,
        ] {
            let obligation_id = obligation("obligation:1");
            let result = admission(
                vec![receipt("receipt:1", obligation_id.clone(), outcome)],
                obligation_id,
                SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
                BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile,
                BeneficiaryAcceptanceAssertionV1::NotRequired,
                SatisfactionCurrentnessAssertionV1::AssertedCurrent,
            );
            assert_eq!(
                result,
                Err(SatisfactionAdmissionErrorV1::PositiveAdmissionLacksFulfilledReport)
            );
        }
    }

    #[test]
    fn required_beneficiary_rejection_blocks_positive_admission() {
        let obligation_id = obligation("obligation:1");
        let result = admission(
            vec![receipt(
                "receipt:1",
                obligation_id.clone(),
                ReciprocityReportedOutcomeV1::ReportedFulfilled,
            )],
            obligation_id,
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::RequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::AssertedRejected,
            SatisfactionCurrentnessAssertionV1::AssertedCurrent,
        );
        assert_eq!(
            result,
            Err(SatisfactionAdmissionErrorV1::PositiveAdmissionMissingRequiredAcceptance)
        );
    }

    #[test]
    fn non_current_admission_cannot_be_positive() {
        for currentness in [
            SatisfactionCurrentnessAssertionV1::AssertedRevoked,
            SatisfactionCurrentnessAssertionV1::AssertedSuperseded,
            SatisfactionCurrentnessAssertionV1::AssertedExpired,
            SatisfactionCurrentnessAssertionV1::Indeterminate,
        ] {
            let obligation_id = obligation("obligation:1");
            let result = admission(
                vec![receipt(
                    "receipt:1",
                    obligation_id.clone(),
                    ReciprocityReportedOutcomeV1::ReportedFulfilled,
                )],
                obligation_id,
                SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
                BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile,
                BeneficiaryAcceptanceAssertionV1::NotRequired,
                currentness,
            );
            assert_eq!(
                result,
                Err(SatisfactionAdmissionErrorV1::PositiveAdmissionNotCurrent)
            );
        }
    }

    #[test]
    fn conflicting_receipts_can_be_preserved_as_disputed_without_resolution_evidence() {
        let obligation_id = obligation("obligation:1");
        let record = admission(
            vec![
                receipt(
                    "receipt:1",
                    obligation_id.clone(),
                    ReciprocityReportedOutcomeV1::ReportedFulfilled,
                ),
                receipt(
                    "receipt:2",
                    obligation_id.clone(),
                    ReciprocityReportedOutcomeV1::Disputed,
                ),
            ],
            obligation_id,
            SatisfactionAdmissionDispositionV1::DisputedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::RequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::Disputed,
            SatisfactionCurrentnessAssertionV1::Indeterminate,
        )
        .unwrap();
        assert_eq!(record.receipts().len(), 2);
        assert_eq!(
            record.disposition(),
            SatisfactionAdmissionDispositionV1::DisputedUnderProfile
        );
    }

    #[test]
    fn receipt_for_different_obligation_is_rejected() {
        let expected = obligation("obligation:1");
        let result = admission(
            vec![receipt(
                "receipt:1",
                obligation("obligation:2"),
                ReciprocityReportedOutcomeV1::ReportedFulfilled,
            )],
            expected,
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::NotRequired,
            SatisfactionCurrentnessAssertionV1::AssertedCurrent,
        );
        assert_eq!(
            result,
            Err(SatisfactionAdmissionErrorV1::ReceiptObligationMismatch)
        );
    }

    #[test]
    fn duplicate_receipt_is_rejected() {
        let obligation_id = obligation("obligation:1");
        let first = receipt(
            "receipt:1",
            obligation_id.clone(),
            ReciprocityReportedOutcomeV1::ReportedFulfilled,
        );
        let second = receipt(
            "receipt:1",
            obligation_id.clone(),
            ReciprocityReportedOutcomeV1::ReportedFulfilled,
        );
        let result = admission(
            vec![first, second],
            obligation_id,
            SatisfactionAdmissionDispositionV1::AdmittedSatisfiedUnderProfile,
            BeneficiaryAcceptanceRequirementV1::NotRequiredByProfile,
            BeneficiaryAcceptanceAssertionV1::NotRequired,
            SatisfactionCurrentnessAssertionV1::AssertedCurrent,
        );
        assert_eq!(result, Err(SatisfactionAdmissionErrorV1::DuplicateReceipt));
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            RECIPROCITY_SATISFACTION_ADMISSION_PROFILE_V1,
            "mycelix/reciprocity-satisfaction-admission/v1"
        );
    }
}
