// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Evidence-oriented living-heritage transmission records.
//!
//! STEW-041 records continuity without requiring protected transmitted content
//! or personally identifying recipient data to be embedded in the record.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdV1};

pub const LIVING_TRANSMISSION_PROFILE_V1: &str = "mycelix/living-transmission/v1";
pub const MAX_TRANSMISSION_EVIDENCE_REFS_V1: usize = 32;
pub const MAX_TRANSMISSION_CONTEXT_REFS_V1: usize = 32;

/// Descriptive mode of transmission. Recording a mode does not prove that the
/// event happened or that the transmitter was authorized to teach it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TransmissionModeV1 {
    Apprenticeship,
    OralTeaching,
    Demonstration,
    Practice,
    Performance,
    Ceremony,
    Mentorship,
    Workshop,
    CommunityTransmission,
    Other,
}

/// Recipient disclosure is independent from evidence that transmission occurred.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransmissionRecipientV1 {
    PrincipalRef(CanonicalIdV1),
    CollectiveRef(CanonicalIdV1),
    /// Recipient identity exists outside this record and is deliberately not
    /// exposed here. This is not an anonymity guarantee.
    Withheld,
}

impl TransmissionRecipientV1 {
    pub fn principal(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        Ok(Self::PrincipalRef(CanonicalIdV1::new(value)?))
    }

    pub fn collective(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        Ok(Self::CollectiveRef(CanonicalIdV1::new(value)?))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TransmissionContextKindV1 {
    CulturalProtocol,
    PlaceOrSetting,
    Language,
    Lineage,
    TeachingMethod,
    SeasonalContext,
    MaterialOrInstrument,
    ConsentOrPermissionEvidence,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TransmissionContextRefV1 {
    kind: TransmissionContextKindV1,
    reference: CanonicalIdV1,
}

impl TransmissionContextRefV1 {
    pub fn new(
        kind: TransmissionContextKindV1,
        reference: impl Into<String>,
    ) -> Result<Self, CanonicalIdErrorV1> {
        Ok(Self {
            kind,
            reference: CanonicalIdV1::new(reference)?,
        })
    }

    pub const fn kind(&self) -> TransmissionContextKindV1 {
        self.kind
    }

    pub fn reference(&self) -> &CanonicalIdV1 {
        &self.reference
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TransmissionEvidenceRefV1(CanonicalIdV1);

impl TransmissionEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LivingTransmissionErrorV1 {
    NoEvidenceReferences,
    TooManyEvidenceReferences,
    DuplicateEvidenceReference,
    TooManyContextReferences,
    DuplicateContextReference,
}

impl fmt::Display for LivingTransmissionErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoEvidenceReferences => f.write_str("transmission event requires evidence references"),
            Self::TooManyEvidenceReferences => f.write_str("too many transmission evidence references for v1"),
            Self::DuplicateEvidenceReference => f.write_str("duplicate transmission evidence reference"),
            Self::TooManyContextReferences => f.write_str("too many transmission context references for v1"),
            Self::DuplicateContextReference => f.write_str("duplicate transmission context reference"),
        }
    }
}

fn validate_evidence(refs: &[TransmissionEvidenceRefV1]) -> Result<(), LivingTransmissionErrorV1> {
    if refs.is_empty() {
        return Err(LivingTransmissionErrorV1::NoEvidenceReferences);
    }
    if refs.len() > MAX_TRANSMISSION_EVIDENCE_REFS_V1 {
        return Err(LivingTransmissionErrorV1::TooManyEvidenceReferences);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(LivingTransmissionErrorV1::DuplicateEvidenceReference);
        }
    }
    Ok(())
}

fn validate_context(refs: &[TransmissionContextRefV1]) -> Result<(), LivingTransmissionErrorV1> {
    if refs.len() > MAX_TRANSMISSION_CONTEXT_REFS_V1 {
        return Err(LivingTransmissionErrorV1::TooManyContextReferences);
    }
    for (index, reference) in refs.iter().enumerate() {
        if refs[..index].contains(reference) {
            return Err(LivingTransmissionErrorV1::DuplicateContextReference);
        }
    }
    Ok(())
}

/// Evidence-bearing assertion that a living subject/tradition was transmitted.
///
/// No content payload, location text, recipient name, authenticity judgment, or
/// authority judgment is embedded in this theorem.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LivingTransmissionEventV1 {
    event_id: CanonicalIdV1,
    subject: StewardedSubjectIdV1,
    asserted_transmitter_ref: CanonicalIdV1,
    recipient: TransmissionRecipientV1,
    mode: TransmissionModeV1,
    context_refs: Vec<TransmissionContextRefV1>,
    evidence_refs: Vec<TransmissionEvidenceRefV1>,
}

impl LivingTransmissionEventV1 {
    pub fn new(
        event_id: CanonicalIdV1,
        subject: StewardedSubjectIdV1,
        asserted_transmitter_ref: CanonicalIdV1,
        recipient: TransmissionRecipientV1,
        mode: TransmissionModeV1,
        context_refs: Vec<TransmissionContextRefV1>,
        evidence_refs: Vec<TransmissionEvidenceRefV1>,
    ) -> Result<Self, LivingTransmissionErrorV1> {
        validate_context(&context_refs)?;
        validate_evidence(&evidence_refs)?;
        Ok(Self {
            event_id,
            subject,
            asserted_transmitter_ref,
            recipient,
            mode,
            context_refs,
            evidence_refs,
        })
    }

    pub fn event_id(&self) -> &CanonicalIdV1 {
        &self.event_id
    }

    pub fn subject(&self) -> &StewardedSubjectIdV1 {
        &self.subject
    }

    pub fn asserted_transmitter_ref(&self) -> &CanonicalIdV1 {
        &self.asserted_transmitter_ref
    }

    pub fn recipient(&self) -> &TransmissionRecipientV1 {
        &self.recipient
    }

    pub const fn mode(&self) -> TransmissionModeV1 {
        self.mode
    }

    pub fn context_refs(&self) -> &[TransmissionContextRefV1] {
        &self.context_refs
    }

    pub fn evidence_refs(&self) -> &[TransmissionEvidenceRefV1] {
        &self.evidence_refs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn subject() -> StewardedSubjectIdV1 {
        StewardedSubjectIdV1::new("subject:living-tradition:1").unwrap()
    }

    fn evidence(id: &str) -> TransmissionEvidenceRefV1 {
        TransmissionEvidenceRefV1::new(id).unwrap()
    }

    #[test]
    fn transmission_can_preserve_continuity_without_recipient_identity() {
        let event = LivingTransmissionEventV1::new(
            CanonicalIdV1::new("transmission:1").unwrap(),
            subject(),
            CanonicalIdV1::new("principal:asserted-teacher:1").unwrap(),
            TransmissionRecipientV1::Withheld,
            TransmissionModeV1::Apprenticeship,
            vec![
                TransmissionContextRefV1::new(
                    TransmissionContextKindV1::CulturalProtocol,
                    "cultural-protocol:protected:1",
                )
                .unwrap(),
            ],
            vec![evidence("evidence:transmission:1")],
        )
        .unwrap();
        assert!(matches!(event.recipient(), TransmissionRecipientV1::Withheld));
    }

    #[test]
    fn logical_subject_does_not_freeze_one_canonical_representation() {
        let event = LivingTransmissionEventV1::new(
            CanonicalIdV1::new("transmission:2").unwrap(),
            subject(),
            CanonicalIdV1::new("principal:transmitter:1").unwrap(),
            TransmissionRecipientV1::collective("collective:learners:1").unwrap(),
            TransmissionModeV1::OralTeaching,
            vec![],
            vec![evidence("evidence:2")],
        )
        .unwrap();
        assert_eq!(event.subject().as_str(), "subject:living-tradition:1");
    }

    #[test]
    fn evidence_is_required_but_not_interpreted() {
        let result = LivingTransmissionEventV1::new(
            CanonicalIdV1::new("transmission:3").unwrap(),
            subject(),
            CanonicalIdV1::new("principal:transmitter:1").unwrap(),
            TransmissionRecipientV1::Withheld,
            TransmissionModeV1::Practice,
            vec![],
            vec![],
        );
        assert_eq!(result, Err(LivingTransmissionErrorV1::NoEvidenceReferences));
    }

    #[test]
    fn duplicate_context_and_evidence_are_rejected() {
        let context = TransmissionContextRefV1::new(
            TransmissionContextKindV1::Language,
            "context:language:1",
        )
        .unwrap();
        let duplicate_context = LivingTransmissionEventV1::new(
            CanonicalIdV1::new("transmission:4").unwrap(),
            subject(),
            CanonicalIdV1::new("principal:transmitter:1").unwrap(),
            TransmissionRecipientV1::Withheld,
            TransmissionModeV1::Practice,
            vec![context.clone(), context],
            vec![evidence("evidence:1")],
        );
        assert_eq!(
            duplicate_context,
            Err(LivingTransmissionErrorV1::DuplicateContextReference)
        );

        let ev = evidence("evidence:1");
        let duplicate_evidence = LivingTransmissionEventV1::new(
            CanonicalIdV1::new("transmission:5").unwrap(),
            subject(),
            CanonicalIdV1::new("principal:transmitter:1").unwrap(),
            TransmissionRecipientV1::Withheld,
            TransmissionModeV1::Practice,
            vec![],
            vec![ev.clone(), ev],
        );
        assert_eq!(
            duplicate_evidence,
            Err(LivingTransmissionErrorV1::DuplicateEvidenceReference)
        );
    }

    #[test]
    fn recipient_can_be_principal_collective_or_withheld() {
        let principal = TransmissionRecipientV1::principal("principal:learner:1").unwrap();
        let collective = TransmissionRecipientV1::collective("collective:learners:1").unwrap();
        assert!(matches!(principal, TransmissionRecipientV1::PrincipalRef(_)));
        assert!(matches!(collective, TransmissionRecipientV1::CollectiveRef(_)));
        assert_ne!(principal, collective);
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            LIVING_TRANSMISSION_PROFILE_V1,
            "mycelix/living-transmission/v1"
        );
    }
}
