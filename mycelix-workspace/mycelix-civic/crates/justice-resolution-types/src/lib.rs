// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Transport-neutral, versioned semantic vocabulary for final Mycelix Justice
//! resolution outcomes.
//!
//! This crate defines the meaning of a Justice result that downstream domains
//! may consume after an owning Justice verifier has established provenance and
//! finality. It contains no Holochain/HDK, Business, Finance, database, network,
//! clock, filesystem, process, provider, UI, or AI API.
//!
//! The types themselves do **not** prove that a Justice decision is valid,
//! authorized, final, or actually stored at any particular record. Those are
//! owning-domain verification responsibilities.

use core::fmt;
use serde::{Deserialize, Deserializer, Serialize};

/// Semantic profile for a final Justice monetary remedy.
pub const FINAL_MONETARY_REMEDY_PROFILE: &str = "justice.final-monetary-remedy";
/// Version of the final Justice monetary-remedy profile.
pub const FINAL_MONETARY_REMEDY_VERSION: u32 = 1;
/// Semantic profile for a final Justice disposition retaining an exact exception.
pub const FINAL_RETAINED_EXCEPTION_PROFILE: &str =
    "justice.final-retained-exception-disposition";
/// Version of the final retained-exception profile.
pub const FINAL_RETAINED_EXCEPTION_VERSION: u32 = 1;

/// Static semantic-profile descriptor owned by Justice.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JusticeSemanticProfile {
    pub name: &'static str,
    pub version: u32,
}

/// Structural failures for Justice resolution vocabulary values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JusticeResolutionError {
    EmptyCaseRef,
    EmptyDecisionRef,
    EmptyRemedyRef,
    EmptySubjectRef,
    EmptyEffectId,
    EmptyResponsiblePartyRef,
    EmptyBeneficiaryPartyRef,
    EmptyUnit,
    ZeroAmount,
    EmptyPolicyRef,
    EmptyNoLiveAppealEvidenceRef,
    QualificationBeforeAppealDeadline,
    EmptyAppealRef,
    EmptyAppealResolutionRef,
    EmptySettlementRef,
    EmptyExceptionDomain,
    EmptyExceptionId,
}

impl fmt::Display for JusticeResolutionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::EmptyCaseRef => "case reference must not be empty",
            Self::EmptyDecisionRef => "decision reference must not be empty",
            Self::EmptyRemedyRef => "remedy reference must not be empty",
            Self::EmptySubjectRef => "institutional subject reference must not be empty",
            Self::EmptyEffectId => "expected effect id must not be empty",
            Self::EmptyResponsiblePartyRef => "responsible-party reference must not be empty",
            Self::EmptyBeneficiaryPartyRef => "beneficiary-party reference must not be empty",
            Self::EmptyUnit => "monetary remedy unit must not be empty",
            Self::ZeroAmount => "monetary remedy amount must be greater than zero",
            Self::EmptyPolicyRef => "finality policy reference must not be empty",
            Self::EmptyNoLiveAppealEvidenceRef => {
                "appeal-window finality requires exact no-live-appeal evidence"
            }
            Self::QualificationBeforeAppealDeadline => {
                "appeal-window finality cannot qualify before the appeal deadline"
            }
            Self::EmptyAppealRef => "appeal reference must not be empty",
            Self::EmptyAppealResolutionRef => "appeal resolution reference must not be empty",
            Self::EmptySettlementRef => "final consent settlement reference must not be empty",
            Self::EmptyExceptionDomain => "retained exception domain must not be empty",
            Self::EmptyExceptionId => "retained exception id must not be empty",
        })
    }
}

impl std::error::Error for JusticeResolutionError {}

/// Explicit basis under which an owning Justice verifier may conclude that a
/// decision/remedy is final for downstream execution.
///
/// This is evidence vocabulary, not self-authenticating proof. In particular,
/// `AppealWindowExpired` still requires an exact evidence reference establishing
/// that no live appeal changes the decision's current disposition.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum JusticeFinalityBasisV1 {
    /// The applicable Justice policy makes this decision non-appealable.
    NoAppealPermitted { policy_ref: String },
    /// The appeal window has elapsed and exact evidence establishes no live appeal.
    AppealWindowExpired {
        appeal_deadline_unix_ms: u64,
        qualified_at_unix_ms: u64,
        no_live_appeal_evidence_ref: String,
    },
    /// An appeal exists and its exact resolution makes this decision/remedy final.
    AppealResolved {
        appeal_ref: String,
        appeal_resolution_ref: String,
    },
    /// Parties entered an exact settlement whose Justice profile makes it final.
    ConsentFinal { settlement_ref: String },
}

impl JusticeFinalityBasisV1 {
    /// Validate only the structural relationships represented by this vocabulary.
    pub fn validate(&self) -> Result<(), JusticeResolutionError> {
        match self {
            Self::NoAppealPermitted { policy_ref } => {
                require_text(policy_ref, JusticeResolutionError::EmptyPolicyRef)
            }
            Self::AppealWindowExpired {
                appeal_deadline_unix_ms,
                qualified_at_unix_ms,
                no_live_appeal_evidence_ref,
            } => {
                require_text(
                    no_live_appeal_evidence_ref,
                    JusticeResolutionError::EmptyNoLiveAppealEvidenceRef,
                )?;
                if qualified_at_unix_ms < appeal_deadline_unix_ms {
                    return Err(JusticeResolutionError::QualificationBeforeAppealDeadline);
                }
                Ok(())
            }
            Self::AppealResolved {
                appeal_ref,
                appeal_resolution_ref,
            } => {
                require_text(appeal_ref, JusticeResolutionError::EmptyAppealRef)?;
                require_text(
                    appeal_resolution_ref,
                    JusticeResolutionError::EmptyAppealResolutionRef,
                )
            }
            Self::ConsentFinal { settlement_ref } => {
                require_text(settlement_ref, JusticeResolutionError::EmptySettlementRef)
            }
        }
    }

    /// Deterministic v1 textual material for the finality basis.
    #[must_use]
    pub fn canonical_material_v1(&self) -> String {
        let mut out = String::from("justice-finality-v1");
        match self {
            Self::NoAppealPermitted { policy_ref } => {
                out.push_str("|no-appeal");
                push_text_field(&mut out, "policy", policy_ref);
            }
            Self::AppealWindowExpired {
                appeal_deadline_unix_ms,
                qualified_at_unix_ms,
                no_live_appeal_evidence_ref,
            } => {
                out.push_str("|appeal-window-expired");
                out.push_str("|deadline-ms:");
                out.push_str(&appeal_deadline_unix_ms.to_string());
                out.push_str("|qualified-ms:");
                out.push_str(&qualified_at_unix_ms.to_string());
                push_text_field(
                    &mut out,
                    "no-live-appeal-evidence",
                    no_live_appeal_evidence_ref,
                );
            }
            Self::AppealResolved {
                appeal_ref,
                appeal_resolution_ref,
            } => {
                out.push_str("|appeal-resolved");
                push_text_field(&mut out, "appeal", appeal_ref);
                push_text_field(&mut out, "appeal-resolution", appeal_resolution_ref);
            }
            Self::ConsentFinal { settlement_ref } => {
                out.push_str("|consent-final");
                push_text_field(&mut out, "settlement", settlement_ref);
            }
        }
        out
    }
}

/// Justice-owned semantic class of a monetary remedy.
///
/// This deliberately stops short of a Finance operation. In particular,
/// `Restitution` does not automatically mean `finance.refund`; a cross-domain
/// mapping must additionally qualify the exact subject and execution profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MonetaryRemedyKindV1 {
    Compensation,
    Restitution,
}

impl MonetaryRemedyKindV1 {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Compensation => "compensation",
            Self::Restitution => "restitution",
        }
    }
}

/// Exact monetary remedy substance issued by Justice for downstream execution.
///
/// `subject_ref` identifies the exact institutional subject affected by the
/// remedy (for example an order, agreement, obligation, or prior economic
/// effect). `amount` is an integer quantity in the exact semantic `unit`.
/// Justice does not assume decimal currency rules or dictate how Finance
/// represents settlement.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct FinalMonetaryRemedyV1 {
    case_ref: String,
    decision_ref: String,
    remedy_ref: String,
    remedy_kind: MonetaryRemedyKindV1,
    subject_ref: String,
    effect_id: String,
    responsible_party_ref: String,
    beneficiary_party_ref: String,
    unit: String,
    amount: u128,
    finality: JusticeFinalityBasisV1,
}

impl FinalMonetaryRemedyV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        case_ref: impl Into<String>,
        decision_ref: impl Into<String>,
        remedy_ref: impl Into<String>,
        remedy_kind: MonetaryRemedyKindV1,
        subject_ref: impl Into<String>,
        effect_id: impl Into<String>,
        responsible_party_ref: impl Into<String>,
        beneficiary_party_ref: impl Into<String>,
        unit: impl Into<String>,
        amount: u128,
        finality: JusticeFinalityBasisV1,
    ) -> Result<Self, JusticeResolutionError> {
        let value = Self {
            case_ref: case_ref.into(),
            decision_ref: decision_ref.into(),
            remedy_ref: remedy_ref.into(),
            remedy_kind,
            subject_ref: subject_ref.into(),
            effect_id: effect_id.into(),
            responsible_party_ref: responsible_party_ref.into(),
            beneficiary_party_ref: beneficiary_party_ref.into(),
            unit: unit.into(),
            amount,
            finality,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), JusticeResolutionError> {
        require_text(&self.case_ref, JusticeResolutionError::EmptyCaseRef)?;
        require_text(&self.decision_ref, JusticeResolutionError::EmptyDecisionRef)?;
        require_text(&self.remedy_ref, JusticeResolutionError::EmptyRemedyRef)?;
        require_text(&self.subject_ref, JusticeResolutionError::EmptySubjectRef)?;
        require_text(&self.effect_id, JusticeResolutionError::EmptyEffectId)?;
        require_text(
            &self.responsible_party_ref,
            JusticeResolutionError::EmptyResponsiblePartyRef,
        )?;
        require_text(
            &self.beneficiary_party_ref,
            JusticeResolutionError::EmptyBeneficiaryPartyRef,
        )?;
        require_text(&self.unit, JusticeResolutionError::EmptyUnit)?;
        if self.amount == 0 {
            return Err(JusticeResolutionError::ZeroAmount);
        }
        self.finality.validate()
    }

    #[must_use]
    pub fn case_ref(&self) -> &str {
        &self.case_ref
    }

    #[must_use]
    pub fn decision_ref(&self) -> &str {
        &self.decision_ref
    }

    #[must_use]
    pub fn remedy_ref(&self) -> &str {
        &self.remedy_ref
    }

    #[must_use]
    pub const fn remedy_kind(&self) -> MonetaryRemedyKindV1 {
        self.remedy_kind
    }

    #[must_use]
    pub fn subject_ref(&self) -> &str {
        &self.subject_ref
    }

    #[must_use]
    pub fn effect_id(&self) -> &str {
        &self.effect_id
    }

    #[must_use]
    pub fn responsible_party_ref(&self) -> &str {
        &self.responsible_party_ref
    }

    #[must_use]
    pub fn beneficiary_party_ref(&self) -> &str {
        &self.beneficiary_party_ref
    }

    #[must_use]
    pub fn unit(&self) -> &str {
        &self.unit
    }

    #[must_use]
    pub const fn amount(&self) -> u128 {
        self.amount
    }

    #[must_use]
    pub const fn finality(&self) -> &JusticeFinalityBasisV1 {
        &self.finality
    }

    /// Canonical material for the economic/remedy effect itself, excluding finality.
    #[must_use]
    pub fn canonical_effect_material_v1(&self) -> String {
        let mut out = String::from("justice-monetary-remedy-effect-v1");
        push_text_field(&mut out, "case", &self.case_ref);
        push_text_field(&mut out, "decision", &self.decision_ref);
        push_text_field(&mut out, "remedy", &self.remedy_ref);
        push_text_field(&mut out, "remedy-kind", self.remedy_kind.as_str());
        push_text_field(&mut out, "subject", &self.subject_ref);
        push_text_field(&mut out, "effect", &self.effect_id);
        push_text_field(&mut out, "responsible", &self.responsible_party_ref);
        push_text_field(&mut out, "beneficiary", &self.beneficiary_party_ref);
        push_text_field(&mut out, "unit", &self.unit);
        out.push_str("|amount:");
        out.push_str(&self.amount.to_string());
        out
    }

    /// Canonical material for the full downstream-qualifiable resolution result.
    #[must_use]
    pub fn canonical_resolution_material_v1(&self) -> String {
        let mut out = self.canonical_effect_material_v1();
        push_text_field(
            &mut out,
            "finality",
            &self.finality.canonical_material_v1(),
        );
        out
    }
}

impl<'de> Deserialize<'de> for FinalMonetaryRemedyV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            case_ref: String,
            decision_ref: String,
            remedy_ref: String,
            remedy_kind: MonetaryRemedyKindV1,
            subject_ref: String,
            effect_id: String,
            responsible_party_ref: String,
            beneficiary_party_ref: String,
            unit: String,
            amount: u128,
            finality: JusticeFinalityBasisV1,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.case_ref,
            wire.decision_ref,
            wire.remedy_ref,
            wire.remedy_kind,
            wire.subject_ref,
            wire.effect_id,
            wire.responsible_party_ref,
            wire.beneficiary_party_ref,
            wire.unit,
            wire.amount,
            wire.finality,
        )
        .map_err(serde::de::Error::custom)
    }
}

/// Exact exception identity Justice permits to remain unresolved at terminal
/// disposition, while preserving the fact that the exception remains distinct
/// from satisfaction.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub struct FinalRetainedExceptionDispositionV1 {
    case_ref: String,
    decision_ref: String,
    subject_ref: String,
    exception_domain: String,
    exception_id: String,
    finality: JusticeFinalityBasisV1,
}

impl FinalRetainedExceptionDispositionV1 {
    pub fn new(
        case_ref: impl Into<String>,
        decision_ref: impl Into<String>,
        subject_ref: impl Into<String>,
        exception_domain: impl Into<String>,
        exception_id: impl Into<String>,
        finality: JusticeFinalityBasisV1,
    ) -> Result<Self, JusticeResolutionError> {
        let value = Self {
            case_ref: case_ref.into(),
            decision_ref: decision_ref.into(),
            subject_ref: subject_ref.into(),
            exception_domain: exception_domain.into(),
            exception_id: exception_id.into(),
            finality,
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), JusticeResolutionError> {
        require_text(&self.case_ref, JusticeResolutionError::EmptyCaseRef)?;
        require_text(&self.decision_ref, JusticeResolutionError::EmptyDecisionRef)?;
        require_text(&self.subject_ref, JusticeResolutionError::EmptySubjectRef)?;
        require_text(
            &self.exception_domain,
            JusticeResolutionError::EmptyExceptionDomain,
        )?;
        require_text(&self.exception_id, JusticeResolutionError::EmptyExceptionId)?;
        self.finality.validate()
    }

    #[must_use]
    pub fn case_ref(&self) -> &str {
        &self.case_ref
    }

    #[must_use]
    pub fn decision_ref(&self) -> &str {
        &self.decision_ref
    }

    #[must_use]
    pub fn subject_ref(&self) -> &str {
        &self.subject_ref
    }

    #[must_use]
    pub fn exception_domain(&self) -> &str {
        &self.exception_domain
    }

    #[must_use]
    pub fn exception_id(&self) -> &str {
        &self.exception_id
    }

    #[must_use]
    pub const fn finality(&self) -> &JusticeFinalityBasisV1 {
        &self.finality
    }

    #[must_use]
    pub fn canonical_resolution_material_v1(&self) -> String {
        let mut out = String::from("justice-retained-exception-v1");
        push_text_field(&mut out, "case", &self.case_ref);
        push_text_field(&mut out, "decision", &self.decision_ref);
        push_text_field(&mut out, "subject", &self.subject_ref);
        push_text_field(&mut out, "exception-domain", &self.exception_domain);
        push_text_field(&mut out, "exception-id", &self.exception_id);
        push_text_field(
            &mut out,
            "finality",
            &self.finality.canonical_material_v1(),
        );
        out
    }
}

impl<'de> Deserialize<'de> for FinalRetainedExceptionDispositionV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            case_ref: String,
            decision_ref: String,
            subject_ref: String,
            exception_domain: String,
            exception_id: String,
            finality: JusticeFinalityBasisV1,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::new(
            wire.case_ref,
            wire.decision_ref,
            wire.subject_ref,
            wire.exception_domain,
            wire.exception_id,
            wire.finality,
        )
        .map_err(serde::de::Error::custom)
    }
}

/// Version-1 downstream-consumable semantic result of a final Justice process.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum JusticeResolutionOutcomeV1 {
    MonetaryRemedy(FinalMonetaryRemedyV1),
    RetainedException(FinalRetainedExceptionDispositionV1),
}

impl JusticeResolutionOutcomeV1 {
    #[must_use]
    pub const fn semantic_profile(&self) -> JusticeSemanticProfile {
        match self {
            Self::MonetaryRemedy(_) => JusticeSemanticProfile {
                name: FINAL_MONETARY_REMEDY_PROFILE,
                version: FINAL_MONETARY_REMEDY_VERSION,
            },
            Self::RetainedException(_) => JusticeSemanticProfile {
                name: FINAL_RETAINED_EXCEPTION_PROFILE,
                version: FINAL_RETAINED_EXCEPTION_VERSION,
            },
        }
    }

    pub fn validate(&self) -> Result<(), JusticeResolutionError> {
        match self {
            Self::MonetaryRemedy(value) => value.validate(),
            Self::RetainedException(value) => value.validate(),
        }
    }
}

fn require_text(value: &str, error: JusticeResolutionError) -> Result<(), JusticeResolutionError> {
    if value.trim().is_empty() {
        Err(error)
    } else {
        Ok(())
    }
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

    fn finality() -> JusticeFinalityBasisV1 {
        JusticeFinalityBasisV1::AppealWindowExpired {
            appeal_deadline_unix_ms: 1_000,
            qualified_at_unix_ms: 1_001,
            no_live_appeal_evidence_ref: "justice:no-live-appeal:decision-1".to_owned(),
        }
    }

    fn remedy(amount: u128) -> FinalMonetaryRemedyV1 {
        FinalMonetaryRemedyV1::new(
            "case:1",
            "decision:1",
            "remedy:0",
            MonetaryRemedyKindV1::Restitution,
            "order:1",
            "effect:restitution:1",
            "party:merchant",
            "party:customer",
            "USD-cent",
            amount,
            finality(),
        )
        .unwrap()
    }

    #[test]
    fn monetary_remedy_profile_is_frozen() {
        let outcome = JusticeResolutionOutcomeV1::MonetaryRemedy(remedy(5_000));
        assert_eq!(
            outcome.semantic_profile(),
            JusticeSemanticProfile {
                name: FINAL_MONETARY_REMEDY_PROFILE,
                version: 1,
            }
        );
    }

    #[test]
    fn material_amount_change_changes_effect_commitment() {
        let fifty = remedy(5_000);
        let sixty = remedy(6_000);
        assert_ne!(
            fifty.canonical_effect_material_v1(),
            sixty.canonical_effect_material_v1()
        );
    }

    #[test]
    fn remedy_kind_change_changes_effect_commitment() {
        let restitution = remedy(5_000);
        let compensation = FinalMonetaryRemedyV1::new(
            "case:1",
            "decision:1",
            "remedy:0",
            MonetaryRemedyKindV1::Compensation,
            "order:1",
            "effect:restitution:1",
            "party:merchant",
            "party:customer",
            "USD-cent",
            5_000,
            finality(),
        )
        .unwrap();

        assert_ne!(
            restitution.canonical_effect_material_v1(),
            compensation.canonical_effect_material_v1()
        );
    }

    #[test]
    fn subject_change_changes_effect_commitment() {
        let order_one = remedy(5_000);
        let order_two = FinalMonetaryRemedyV1::new(
            "case:1",
            "decision:1",
            "remedy:0",
            MonetaryRemedyKindV1::Restitution,
            "order:2",
            "effect:restitution:1",
            "party:merchant",
            "party:customer",
            "USD-cent",
            5_000,
            finality(),
        )
        .unwrap();

        assert_ne!(
            order_one.canonical_effect_material_v1(),
            order_two.canonical_effect_material_v1()
        );
    }

    #[test]
    fn finality_change_changes_resolution_but_not_effect_material() {
        let base = remedy(5_000);
        let changed = FinalMonetaryRemedyV1::new(
            "case:1",
            "decision:1",
            "remedy:0",
            MonetaryRemedyKindV1::Restitution,
            "order:1",
            "effect:restitution:1",
            "party:merchant",
            "party:customer",
            "USD-cent",
            5_000,
            JusticeFinalityBasisV1::AppealResolved {
                appeal_ref: "appeal:1".to_owned(),
                appeal_resolution_ref: "appeal-resolution:1".to_owned(),
            },
        )
        .unwrap();

        assert_eq!(
            base.canonical_effect_material_v1(),
            changed.canonical_effect_material_v1()
        );
        assert_ne!(
            base.canonical_resolution_material_v1(),
            changed.canonical_resolution_material_v1()
        );
    }

    #[test]
    fn appeal_window_cannot_be_declared_final_before_deadline() {
        assert_eq!(
            JusticeFinalityBasisV1::AppealWindowExpired {
                appeal_deadline_unix_ms: 1_000,
                qualified_at_unix_ms: 999,
                no_live_appeal_evidence_ref: "evidence:1".to_owned(),
            }
            .validate(),
            Err(JusticeResolutionError::QualificationBeforeAppealDeadline)
        );
    }

    #[test]
    fn appeal_window_requires_exact_no_live_appeal_evidence() {
        assert_eq!(
            JusticeFinalityBasisV1::AppealWindowExpired {
                appeal_deadline_unix_ms: 1_000,
                qualified_at_unix_ms: 1_000,
                no_live_appeal_evidence_ref: " ".to_owned(),
            }
            .validate(),
            Err(JusticeResolutionError::EmptyNoLiveAppealEvidenceRef)
        );
    }

    #[test]
    fn zero_monetary_remedy_fails_closed() {
        assert_eq!(
            FinalMonetaryRemedyV1::new(
                "case:1",
                "decision:1",
                "remedy:0",
                MonetaryRemedyKindV1::Compensation,
                "obligation:1",
                "effect:1",
                "party:a",
                "party:b",
                "USD-cent",
                0,
                finality(),
            ),
            Err(JusticeResolutionError::ZeroAmount)
        );
    }

    #[test]
    fn empty_subject_fails_closed() {
        assert_eq!(
            FinalMonetaryRemedyV1::new(
                "case:1",
                "decision:1",
                "remedy:0",
                MonetaryRemedyKindV1::Restitution,
                " ",
                "effect:1",
                "party:a",
                "party:b",
                "USD-cent",
                1,
                finality(),
            ),
            Err(JusticeResolutionError::EmptySubjectRef)
        );
    }

    #[test]
    fn length_prefixing_prevents_delimiter_ambiguity() {
        let a = FinalMonetaryRemedyV1::new(
            "case:a|b",
            "decision:c",
            "remedy:0",
            MonetaryRemedyKindV1::Restitution,
            "order:a|b",
            "effect:1",
            "party:a",
            "party:b",
            "unit|x",
            1,
            finality(),
        )
        .unwrap();
        let b = FinalMonetaryRemedyV1::new(
            "case:a",
            "decision:b|c",
            "remedy:0",
            MonetaryRemedyKindV1::Restitution,
            "order:a",
            "effect:1",
            "party:a",
            "party:b",
            "unit|x",
            1,
            finality(),
        )
        .unwrap();

        assert_ne!(
            a.canonical_effect_material_v1(),
            b.canonical_effect_material_v1()
        );
    }

    #[test]
    fn retained_exception_preserves_exact_subject_identity_and_finality() {
        let retained = FinalRetainedExceptionDispositionV1::new(
            "case:1",
            "decision:1",
            "order:1",
            "finance",
            "chargeback:pending:1",
            JusticeFinalityBasisV1::NoAppealPermitted {
                policy_ref: "justice-policy:no-appeal:v1".to_owned(),
            },
        )
        .unwrap();
        let outcome = JusticeResolutionOutcomeV1::RetainedException(retained.clone());

        assert_eq!(retained.subject_ref(), "order:1");
        assert_eq!(retained.exception_domain(), "finance");
        assert_eq!(retained.exception_id(), "chargeback:pending:1");
        assert_eq!(
            outcome.semantic_profile(),
            JusticeSemanticProfile {
                name: FINAL_RETAINED_EXCEPTION_PROFILE,
                version: 1,
            }
        );
    }
}
