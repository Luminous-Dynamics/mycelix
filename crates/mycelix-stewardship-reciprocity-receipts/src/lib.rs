// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Evidence-bearing reports about reciprocity performance.
//!
//! STEW-011 deliberately records reports rather than deciding satisfaction.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};
use mycelix_stewardship_reciprocity::ReciprocityObligationIdV1;

/// Stable profile identifier for this theorem.
pub const RECIPROCITY_RECEIPT_PROFILE_V1: &str = "mycelix/reciprocity-receipt/v1";

/// Maximum evidence references attached to one receipt.
pub const MAX_RECEIPT_EVIDENCE_REFS_V1: usize = 32;

/// Explicitly reported outcome; none of these variants is a system verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ReciprocityReportedOutcomeV1 {
    ReportedFulfilled,
    ReportedPartiallyFulfilled,
    ReportedFailed,
    Disputed,
    Indeterminate,
}

/// Typed receipt identity.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ReciprocityReceiptIdV1(CanonicalIdV1);

impl ReciprocityReceiptIdV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Typed opaque evidence reference.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ReciprocityReceiptEvidenceRefV1(CanonicalIdV1);

impl ReciprocityReceiptEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Structural receipt construction errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReciprocityReceiptErrorV1 {
    NoEvidenceReferences,
    TooManyEvidenceReferences,
    DuplicateEvidenceReference,
}

impl fmt::Display for ReciprocityReceiptErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoEvidenceReferences => {
                f.write_str("reciprocity receipt requires evidence references")
            }
            Self::TooManyEvidenceReferences => {
                f.write_str("too many reciprocity receipt evidence references for v1")
            }
            Self::DuplicateEvidenceReference => {
                f.write_str("duplicate reciprocity receipt evidence reference")
            }
        }
    }
}

/// Evidence-bearing report concerning one STEW-010 reciprocity obligation.
///
/// Core separation:
///
/// ```text
/// reported outcome
/// != verified outcome
/// != beneficiary acceptance
/// != obligation satisfaction
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReciprocityReceiptV1 {
    receipt_id: ReciprocityReceiptIdV1,
    obligation_id: ReciprocityObligationIdV1,
    reported_by_ref: CanonicalIdV1,
    performer_ref: CanonicalIdV1,
    event_ref: CanonicalIdV1,
    outcome: ReciprocityReportedOutcomeV1,
    evidence_refs: Vec<ReciprocityReceiptEvidenceRefV1>,
}

impl ReciprocityReceiptV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        receipt_id: ReciprocityReceiptIdV1,
        obligation_id: ReciprocityObligationIdV1,
        reported_by_ref: CanonicalIdV1,
        performer_ref: CanonicalIdV1,
        event_ref: CanonicalIdV1,
        outcome: ReciprocityReportedOutcomeV1,
        evidence_refs: Vec<ReciprocityReceiptEvidenceRefV1>,
    ) -> Result<Self, ReciprocityReceiptErrorV1> {
        validate_evidence_refs(&evidence_refs)?;
        Ok(Self {
            receipt_id,
            obligation_id,
            reported_by_ref,
            performer_ref,
            event_ref,
            outcome,
            evidence_refs,
        })
    }

    pub fn receipt_id(&self) -> &ReciprocityReceiptIdV1 {
        &self.receipt_id
    }

    pub fn obligation_id(&self) -> &ReciprocityObligationIdV1 {
        &self.obligation_id
    }

    pub fn reported_by_ref(&self) -> &CanonicalIdV1 {
        &self.reported_by_ref
    }

    pub fn performer_ref(&self) -> &CanonicalIdV1 {
        &self.performer_ref
    }

    pub fn event_ref(&self) -> &CanonicalIdV1 {
        &self.event_ref
    }

    pub const fn outcome(&self) -> ReciprocityReportedOutcomeV1 {
        self.outcome
    }

    pub fn evidence_refs(&self) -> &[ReciprocityReceiptEvidenceRefV1] {
        &self.evidence_refs
    }
}

fn validate_evidence_refs(
    refs: &[ReciprocityReceiptEvidenceRefV1],
) -> Result<(), ReciprocityReceiptErrorV1> {
    if refs.is_empty() {
        return Err(ReciprocityReceiptErrorV1::NoEvidenceReferences);
    }
    if refs.len() > MAX_RECEIPT_EVIDENCE_REFS_V1 {
        return Err(ReciprocityReceiptErrorV1::TooManyEvidenceReferences);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(ReciprocityReceiptErrorV1::DuplicateEvidenceReference);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> CanonicalIdV1 {
        CanonicalIdV1::new(value).unwrap()
    }

    fn evidence(value: &str) -> ReciprocityReceiptEvidenceRefV1 {
        ReciprocityReceiptEvidenceRefV1::new(value).unwrap()
    }

    fn receipt(outcome: ReciprocityReportedOutcomeV1, suffix: &str) -> ReciprocityReceiptV1 {
        ReciprocityReceiptV1::new(
            ReciprocityReceiptIdV1::new(format!("receipt:{suffix}")).unwrap(),
            ReciprocityObligationIdV1::new("reciprocity:obligation:1").unwrap(),
            id("reporter:1"),
            id("performer:1"),
            id("event:1"),
            outcome,
            vec![evidence(&format!("evidence:{suffix}"))],
        )
        .unwrap()
    }

    #[test]
    fn reported_fulfillment_remains_a_reported_outcome() {
        let receipt = receipt(ReciprocityReportedOutcomeV1::ReportedFulfilled, "fulfilled");
        assert_eq!(
            receipt.outcome(),
            ReciprocityReportedOutcomeV1::ReportedFulfilled
        );
    }

    #[test]
    fn conflicting_receipts_for_one_obligation_can_coexist() {
        let fulfilled = receipt(ReciprocityReportedOutcomeV1::ReportedFulfilled, "a");
        let disputed = receipt(ReciprocityReportedOutcomeV1::Disputed, "b");
        assert_eq!(fulfilled.obligation_id(), disputed.obligation_id());
        assert_ne!(fulfilled.outcome(), disputed.outcome());
    }

    #[test]
    fn evidence_is_required() {
        let result = ReciprocityReceiptV1::new(
            ReciprocityReceiptIdV1::new("receipt:no-evidence").unwrap(),
            ReciprocityObligationIdV1::new("reciprocity:obligation:1").unwrap(),
            id("reporter:1"),
            id("performer:1"),
            id("event:1"),
            ReciprocityReportedOutcomeV1::Indeterminate,
            vec![],
        );
        assert_eq!(result, Err(ReciprocityReceiptErrorV1::NoEvidenceReferences));
    }

    #[test]
    fn duplicate_evidence_is_rejected() {
        let duplicate = evidence("evidence:duplicate");
        let result = ReciprocityReceiptV1::new(
            ReciprocityReceiptIdV1::new("receipt:duplicate").unwrap(),
            ReciprocityObligationIdV1::new("reciprocity:obligation:1").unwrap(),
            id("reporter:1"),
            id("performer:1"),
            id("event:1"),
            ReciprocityReportedOutcomeV1::Indeterminate,
            vec![duplicate.clone(), duplicate],
        );
        assert_eq!(
            result,
            Err(ReciprocityReceiptErrorV1::DuplicateEvidenceReference)
        );
    }

    #[test]
    fn reporter_and_performer_are_distinct_roles() {
        let receipt = ReciprocityReceiptV1::new(
            ReciprocityReceiptIdV1::new("receipt:roles").unwrap(),
            ReciprocityObligationIdV1::new("reciprocity:obligation:1").unwrap(),
            id("reporter:beneficiary:1"),
            id("performer:organization:1"),
            id("event:delivery:1"),
            ReciprocityReportedOutcomeV1::ReportedPartiallyFulfilled,
            vec![evidence("evidence:delivery:1")],
        )
        .unwrap();
        assert_ne!(receipt.reported_by_ref(), receipt.performer_ref());
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            RECIPROCITY_RECEIPT_PROFILE_V1,
            "mycelix/reciprocity-receipt/v1"
        );
    }
}
