// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![forbid(unsafe_code)]

//! Pure qualification of permission to begin enforcing an already-verified
//! Mycelix Justice monetary remedy.
//!
//! This crate deliberately does not re-derive Decision, remedy, vote, appeal,
//! or finality truth. Its positive input is `VerifiedJusticeMonetaryRemedyV1`,
//! minted by the owning Justice resolution verifier. The only additional
//! semantic question answered here is whether an exact Justice enforcement
//! policy permits that already-verified remedy to enter an execution-attempt
//! workflow at the verifier's exact qualification time.
//!
//! A positive result is **not** evidence that Finance paid anything, that an
//! external action was dispatched, or that an Enforcement record is complete.
//! It is historical/as-of authorization only. Irreversible dispatch must still
//! revalidate current authority/supersession and execution-domain constraints.

use core::fmt;
use std::collections::BTreeSet;

use justice_resolution_types::{FinalMonetaryRemedyV1, MonetaryRemedyKindV1};
use justice_resolution_verifier::{JusticeVerificationReceiptV1, VerifiedJusticeMonetaryRemedyV1};

/// Semantic profile for v0.1 monetary enforcement policy.
pub const MONETARY_ENFORCEMENT_POLICY_PROFILE: &str = "justice.monetary-enforcement-policy";
/// Version of the v0.1 monetary enforcement policy.
pub const MONETARY_ENFORCEMENT_POLICY_VERSION: u32 = 1;
/// Semantic profile for the positive authorization minted by this crate.
pub const MONETARY_ENFORCEMENT_AUTHORIZATION_PROFILE: &str =
    "justice.monetary-enforcement-authorization";
/// Version of the positive authorization minted by this crate.
pub const MONETARY_ENFORCEMENT_AUTHORIZATION_VERSION: u32 = 1;

/// Exact policy inputs governing whether a verified monetary remedy may enter
/// an enforcement-attempt workflow.
///
/// The owning runtime must authenticate the policy record represented by
/// `policy_ref`. This pure crate validates only the explicit semantic cut.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JusticeMonetaryEnforcementPolicyV1 {
    pub policy_ref: String,
    pub semantic_profile: String,
    pub semantic_version: u32,
    pub valid_from_unix_ms: u64,
    pub valid_through_unix_ms: Option<u64>,
    pub allowed_remedy_kinds: BTreeSet<MonetaryRemedyKindV1>,
}

/// Complete explicit input cut for one enforcement authorization.
///
/// No raw Decision, Appeal, finality flag, amount, party, or effect identity is
/// supplied here. Those facts come only from the sealed verified remedy.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MonetaryEnforcementQualificationBasisV1 {
    pub verified_remedy: VerifiedJusticeMonetaryRemedyV1,
    pub policy: JusticeMonetaryEnforcementPolicyV1,
}

/// Verifier-owned receipt for the additional enforcement-policy decision.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JusticeEnforcementQualificationReceiptV1 {
    policy_ref: String,
    policy_semantic_profile: String,
    policy_semantic_version: u32,
    policy_valid_from_unix_ms: u64,
    policy_valid_through_unix_ms: Option<u64>,
    allowed_remedy_kinds: BTreeSet<MonetaryRemedyKindV1>,
    qualification_time_unix_ms: u64,
    authorization_profile: &'static str,
    authorization_version: u32,
    current_revalidation_required_before_dispatch: bool,
}

impl JusticeEnforcementQualificationReceiptV1 {
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
    pub const fn policy_valid_from_unix_ms(&self) -> u64 {
        self.policy_valid_from_unix_ms
    }

    #[must_use]
    pub const fn policy_valid_through_unix_ms(&self) -> Option<u64> {
        self.policy_valid_through_unix_ms
    }

    #[must_use]
    pub const fn allowed_remedy_kinds(&self) -> &BTreeSet<MonetaryRemedyKindV1> {
        &self.allowed_remedy_kinds
    }

    #[must_use]
    pub const fn qualification_time_unix_ms(&self) -> u64 {
        self.qualification_time_unix_ms
    }

    #[must_use]
    pub const fn authorization_profile(&self) -> &'static str {
        self.authorization_profile
    }

    #[must_use]
    pub const fn authorization_version(&self) -> u32 {
        self.authorization_version
    }

    #[must_use]
    pub const fn current_revalidation_required_before_dispatch(&self) -> bool {
        self.current_revalidation_required_before_dispatch
    }
}

/// Positive permission to begin an execution-attempt workflow for one exact
/// already-verified Justice remedy.
///
/// Fields are private and there is no public constructor. This type does not
/// mint a new logical economic effect identity: `logical_effect_id()` delegates
/// directly to the verified remedy's existing Justice-owned `effect_id`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedJusticeEnforcementV1 {
    verified_remedy: VerifiedJusticeMonetaryRemedyV1,
    receipt: JusticeEnforcementQualificationReceiptV1,
}

impl QualifiedJusticeEnforcementV1 {
    #[must_use]
    pub const fn verified_remedy(&self) -> &VerifiedJusticeMonetaryRemedyV1 {
        &self.verified_remedy
    }

    #[must_use]
    pub const fn remedy(&self) -> &FinalMonetaryRemedyV1 {
        self.verified_remedy.outcome()
    }

    #[must_use]
    pub const fn justice_verification_receipt(&self) -> &JusticeVerificationReceiptV1 {
        self.verified_remedy.receipt()
    }

    #[must_use]
    pub const fn receipt(&self) -> &JusticeEnforcementQualificationReceiptV1 {
        &self.receipt
    }

    /// The one logical economic effect identity. Enforcement must carry this
    /// forward rather than minting another economic effect ID.
    #[must_use]
    pub fn logical_effect_id(&self) -> &str {
        self.verified_remedy.outcome().effect_id()
    }

    /// Deterministic semantic material for this authorization.
    ///
    /// This is not a substitute for the full verifier-owned receipts retained in
    /// this token. It is suitable as deterministic authorization material or as
    /// input to a runtime-owned digest/idempotency key.
    #[must_use]
    pub fn canonical_authorization_material_v1(&self) -> String {
        let mut out = String::from("justice-monetary-enforcement-authorization-v1");
        push_text_field(
            &mut out,
            "verified-remedy",
            &self
                .verified_remedy
                .outcome()
                .canonical_resolution_material_v1(),
        );
        push_text_field(&mut out, "policy", &self.receipt.policy_ref);
        push_text_field(
            &mut out,
            "policy-profile",
            &self.receipt.policy_semantic_profile,
        );
        out.push_str("|policy-version:");
        out.push_str(&self.receipt.policy_semantic_version.to_string());
        out.push_str("|policy-valid-from-ms:");
        out.push_str(&self.receipt.policy_valid_from_unix_ms.to_string());
        match self.receipt.policy_valid_through_unix_ms {
            Some(valid_through) => {
                out.push_str("|policy-valid-through-ms:");
                out.push_str(&valid_through.to_string());
            }
            None => out.push_str("|policy-valid-through-ms:none"),
        }
        for kind in &self.receipt.allowed_remedy_kinds {
            push_text_field(&mut out, "allowed-remedy-kind", kind.as_str());
        }
        out.push_str("|qualification-ms:");
        out.push_str(&self.receipt.qualification_time_unix_ms.to_string());
        out.push_str("|current-revalidation-before-dispatch:true");
        out
    }

    #[must_use]
    pub fn into_parts(
        self,
    ) -> (
        VerifiedJusticeMonetaryRemedyV1,
        JusticeEnforcementQualificationReceiptV1,
    ) {
        (self.verified_remedy, self.receipt)
    }
}

/// Deterministic denial reasons for v0.1 enforcement qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JusticeEnforcementQualificationError {
    EmptyPolicyRef,
    WrongPolicyProfile,
    InvalidPolicyWindow,
    EmptyAllowedRemedyKinds,
    PolicyNotYetEffective,
    PolicyExpired,
    RemedyKindNotAllowed,
}

impl fmt::Display for JusticeEnforcementQualificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "justice enforcement qualification denied: {self:?}")
    }
}

impl std::error::Error for JusticeEnforcementQualificationError {}

/// Qualify permission to enter an execution-attempt workflow.
///
/// The qualification time is deliberately **derived** from the exact Justice
/// verification receipt rather than supplied independently. A later-time
/// current authorization requires a newly authenticated/requalified Justice
/// cut; this function never promotes a historical verified remedy into current
/// truth merely because wall-clock time advanced.
pub fn qualify_monetary_enforcement_v1(
    basis: MonetaryEnforcementQualificationBasisV1,
) -> Result<QualifiedJusticeEnforcementV1, JusticeEnforcementQualificationError> {
    validate_policy_shape(&basis.policy)?;

    let qualification_time_unix_ms = basis
        .verified_remedy
        .receipt()
        .qualification_time_unix_ms();

    if qualification_time_unix_ms < basis.policy.valid_from_unix_ms {
        return Err(JusticeEnforcementQualificationError::PolicyNotYetEffective);
    }
    if basis
        .policy
        .valid_through_unix_ms
        .is_some_and(|valid_through| qualification_time_unix_ms > valid_through)
    {
        return Err(JusticeEnforcementQualificationError::PolicyExpired);
    }

    let remedy_kind = basis.verified_remedy.outcome().remedy_kind();
    if !basis.policy.allowed_remedy_kinds.contains(&remedy_kind) {
        return Err(JusticeEnforcementQualificationError::RemedyKindNotAllowed);
    }

    let receipt = JusticeEnforcementQualificationReceiptV1 {
        policy_ref: basis.policy.policy_ref,
        policy_semantic_profile: basis.policy.semantic_profile,
        policy_semantic_version: basis.policy.semantic_version,
        policy_valid_from_unix_ms: basis.policy.valid_from_unix_ms,
        policy_valid_through_unix_ms: basis.policy.valid_through_unix_ms,
        allowed_remedy_kinds: basis.policy.allowed_remedy_kinds,
        qualification_time_unix_ms,
        authorization_profile: MONETARY_ENFORCEMENT_AUTHORIZATION_PROFILE,
        authorization_version: MONETARY_ENFORCEMENT_AUTHORIZATION_VERSION,
        current_revalidation_required_before_dispatch: true,
    };

    Ok(QualifiedJusticeEnforcementV1 {
        verified_remedy: basis.verified_remedy,
        receipt,
    })
}

fn validate_policy_shape(
    policy: &JusticeMonetaryEnforcementPolicyV1,
) -> Result<(), JusticeEnforcementQualificationError> {
    if policy.policy_ref.trim().is_empty() {
        return Err(JusticeEnforcementQualificationError::EmptyPolicyRef);
    }
    if policy.semantic_profile != MONETARY_ENFORCEMENT_POLICY_PROFILE
        || policy.semantic_version != MONETARY_ENFORCEMENT_POLICY_VERSION
    {
        return Err(JusticeEnforcementQualificationError::WrongPolicyProfile);
    }
    if policy
        .valid_through_unix_ms
        .is_some_and(|valid_through| valid_through < policy.valid_from_unix_ms)
    {
        return Err(JusticeEnforcementQualificationError::InvalidPolicyWindow);
    }
    if policy.allowed_remedy_kinds.is_empty() {
        return Err(JusticeEnforcementQualificationError::EmptyAllowedRemedyKinds);
    }
    Ok(())
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
        COMPLETE_APPEAL_COVERAGE_PROFILE, COMPLETE_APPEAL_COVERAGE_VERSION,
        CompleteAppealCoverageEvidenceV1, JusticeFinalityQualificationBasisV1,
        qualify_justice_finality_v1,
    };
    use justice_resolution_verifier::{
        ArbitrationSnapshotV1, DecisionSnapshotV1, DecisionVoteChoiceV1,
        DecisionVoteSnapshotV1, FullAwardPolicyV1, FullDecisionOutcomeV1,
        MonetaryRemedyQualificationBasisV1, MonetaryRemedySnapshotV1, PanelMemberSnapshotV1,
        RuntimeMonetaryRemedyKindV1, TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE,
        TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION, TwoPartyCaseSnapshotV1,
        qualify_monetary_remedy_v1,
    };

    fn verified_remedy() -> VerifiedJusticeMonetaryRemedyV1 {
        let finality = qualify_justice_finality_v1(
            JusticeFinalityQualificationBasisV1::NoAppealCoverage {
                decision_ref: "decision:1".into(),
                decision_rendered_at_unix_ms: 100,
                appeal_deadline_unix_ms: 200,
                qualification_time_unix_ms: 201,
                coverage: CompleteAppealCoverageEvidenceV1 {
                    coverage_ref: "appeal-coverage:decision:1:201".into(),
                    decision_ref: "decision:1".into(),
                    authority_evidence_ref: "justice-finality-authority:test".into(),
                    semantic_profile: COMPLETE_APPEAL_COVERAGE_PROFILE.into(),
                    semantic_version: COMPLETE_APPEAL_COVERAGE_VERSION,
                    covered_from_unix_ms: 100,
                    covered_through_unix_ms: 201,
                    observed_appeal_refs: vec![],
                },
            },
        )
        .unwrap();

        qualify_monetary_remedy_v1(MonetaryRemedyQualificationBasisV1 {
            case: TwoPartyCaseSnapshotV1 {
                case_ref: "case:1".into(),
                complainant_ref: "party:customer".into(),
                respondent_ref: "party:merchant".into(),
                subject_ref: "order:1".into(),
            },
            arbitration: ArbitrationSnapshotV1 {
                arbitration_ref: "arbitration:1".into(),
                case_ref: "case:1".into(),
                panel: vec![
                    PanelMemberSnapshotV1 {
                        party_ref: "arb:1".into(),
                        accepted: true,
                        recused: false,
                    },
                    PanelMemberSnapshotV1 {
                        party_ref: "arb:2".into(),
                        accepted: true,
                        recused: false,
                    },
                    PanelMemberSnapshotV1 {
                        party_ref: "arb:3".into(),
                        accepted: true,
                        recused: false,
                    },
                ],
            },
            decision: DecisionSnapshotV1 {
                decision_ref: "decision:1".into(),
                case_ref: "case:1".into(),
                arbitration_ref: "arbitration:1".into(),
                outcome: FullDecisionOutcomeV1::ForComplainant,
                votes: vec![
                    DecisionVoteSnapshotV1 {
                        voter_ref: "arb:1".into(),
                        choice: DecisionVoteChoiceV1::ForComplainant,
                    },
                    DecisionVoteSnapshotV1 {
                        voter_ref: "arb:2".into(),
                        choice: DecisionVoteChoiceV1::ForComplainant,
                    },
                    DecisionVoteSnapshotV1 {
                        voter_ref: "arb:3".into(),
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
                responsible_party_ref: "party:merchant".into(),
                amount: Some(5_000),
                unit: Some("USD-cent".into()),
            },
            qualified_finality: finality,
            policy: FullAwardPolicyV1 {
                policy_ref: "justice-policy:full-award:v1".into(),
                semantic_profile: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_PROFILE.into(),
                semantic_version: TWO_PARTY_PREVAILING_PARTY_FULL_AWARD_VERSION,
                minimum_quorum_votes: 3,
                minimum_support_votes: 2,
            },
            qualification_time_unix_ms: 201,
        })
        .unwrap()
    }

    fn policy() -> JusticeMonetaryEnforcementPolicyV1 {
        JusticeMonetaryEnforcementPolicyV1 {
            policy_ref: "justice-enforcement-policy:1".into(),
            semantic_profile: MONETARY_ENFORCEMENT_POLICY_PROFILE.into(),
            semantic_version: MONETARY_ENFORCEMENT_POLICY_VERSION,
            valid_from_unix_ms: 150,
            valid_through_unix_ms: Some(300),
            allowed_remedy_kinds: BTreeSet::from([MonetaryRemedyKindV1::Restitution]),
        }
    }

    fn basis() -> MonetaryEnforcementQualificationBasisV1 {
        MonetaryEnforcementQualificationBasisV1 {
            verified_remedy: verified_remedy(),
            policy: policy(),
        }
    }

    #[test]
    fn exact_verified_remedy_and_policy_qualify() {
        let qualified = qualify_monetary_enforcement_v1(basis()).unwrap();
        assert_eq!(qualified.remedy().decision_ref(), "decision:1");
        assert_eq!(qualified.remedy().remedy_ref(), "justice-remedy-v1|decision:10:decision:1|index:0");
        assert_eq!(qualified.remedy().amount(), 5_000);
        assert_eq!(qualified.logical_effect_id(), verified_remedy().outcome().effect_id());
        assert_eq!(qualified.receipt().qualification_time_unix_ms(), 201);
        assert_eq!(
            qualified.receipt().authorization_profile(),
            MONETARY_ENFORCEMENT_AUTHORIZATION_PROFILE
        );
        assert_eq!(qualified.receipt().authorization_version(), 1);
        assert!(qualified.receipt().current_revalidation_required_before_dispatch());
    }

    #[test]
    fn same_exact_cut_is_deterministic() {
        let one = qualify_monetary_enforcement_v1(basis()).unwrap();
        let two = qualify_monetary_enforcement_v1(basis()).unwrap();
        assert_eq!(one, two);
        assert_eq!(
            one.canonical_authorization_material_v1(),
            two.canonical_authorization_material_v1()
        );
    }

    #[test]
    fn logical_effect_identity_is_preserved_not_reminted() {
        let verified = verified_remedy();
        let expected = verified.outcome().effect_id().to_owned();
        let qualified = qualify_monetary_enforcement_v1(MonetaryEnforcementQualificationBasisV1 {
            verified_remedy: verified,
            policy: policy(),
        })
        .unwrap();
        assert_eq!(qualified.logical_effect_id(), expected);
        assert!(qualified
            .canonical_authorization_material_v1()
            .contains(&expected));
    }

    #[test]
    fn empty_policy_ref_is_denied() {
        let mut candidate = basis();
        candidate.policy.policy_ref = " ".into();
        assert_eq!(
            qualify_monetary_enforcement_v1(candidate),
            Err(JusticeEnforcementQualificationError::EmptyPolicyRef)
        );
    }

    #[test]
    fn wrong_policy_profile_or_version_is_denied() {
        let mut wrong_profile = basis();
        wrong_profile.policy.semantic_profile = "justice.other-policy".into();
        assert_eq!(
            qualify_monetary_enforcement_v1(wrong_profile),
            Err(JusticeEnforcementQualificationError::WrongPolicyProfile)
        );

        let mut wrong_version = basis();
        wrong_version.policy.semantic_version = 2;
        assert_eq!(
            qualify_monetary_enforcement_v1(wrong_version),
            Err(JusticeEnforcementQualificationError::WrongPolicyProfile)
        );
    }

    #[test]
    fn inverted_policy_window_is_denied() {
        let mut candidate = basis();
        candidate.policy.valid_from_unix_ms = 400;
        candidate.policy.valid_through_unix_ms = Some(300);
        assert_eq!(
            qualify_monetary_enforcement_v1(candidate),
            Err(JusticeEnforcementQualificationError::InvalidPolicyWindow)
        );
    }

    #[test]
    fn empty_allowed_kind_set_is_denied() {
        let mut candidate = basis();
        candidate.policy.allowed_remedy_kinds.clear();
        assert_eq!(
            qualify_monetary_enforcement_v1(candidate),
            Err(JusticeEnforcementQualificationError::EmptyAllowedRemedyKinds)
        );
    }

    #[test]
    fn policy_not_yet_effective_at_exact_justice_cut_is_denied() {
        let mut candidate = basis();
        candidate.policy.valid_from_unix_ms = 202;
        candidate.policy.valid_through_unix_ms = Some(300);
        assert_eq!(
            qualify_monetary_enforcement_v1(candidate),
            Err(JusticeEnforcementQualificationError::PolicyNotYetEffective)
        );
    }

    #[test]
    fn expired_policy_at_exact_justice_cut_is_denied() {
        let mut candidate = basis();
        candidate.policy.valid_from_unix_ms = 100;
        candidate.policy.valid_through_unix_ms = Some(200);
        assert_eq!(
            qualify_monetary_enforcement_v1(candidate),
            Err(JusticeEnforcementQualificationError::PolicyExpired)
        );
    }

    #[test]
    fn unsupported_verified_remedy_kind_is_denied() {
        let mut candidate = basis();
        candidate.policy.allowed_remedy_kinds =
            BTreeSet::from([MonetaryRemedyKindV1::Compensation]);
        assert_eq!(
            qualify_monetary_enforcement_v1(candidate),
            Err(JusticeEnforcementQualificationError::RemedyKindNotAllowed)
        );
    }

    #[test]
    fn open_ended_policy_is_allowed_without_inventing_a_lease_end() {
        let mut candidate = basis();
        candidate.policy.valid_through_unix_ms = None;
        let qualified = qualify_monetary_enforcement_v1(candidate).unwrap();
        assert_eq!(qualified.receipt().policy_valid_through_unix_ms(), None);
        assert!(qualified.receipt().current_revalidation_required_before_dispatch());
    }

    #[test]
    fn authorization_material_changes_with_policy_not_effect_identity() {
        let one = qualify_monetary_enforcement_v1(basis()).unwrap();
        let mut changed = basis();
        changed.policy.policy_ref = "justice-enforcement-policy:2".into();
        let two = qualify_monetary_enforcement_v1(changed).unwrap();

        assert_eq!(one.logical_effect_id(), two.logical_effect_id());
        assert_ne!(
            one.canonical_authorization_material_v1(),
            two.canonical_authorization_material_v1()
        );
    }

    #[test]
    fn no_raw_runtime_finality_or_status_input_exists() {
        let candidate = basis();
        let qualified = qualify_monetary_enforcement_v1(candidate).unwrap();
        assert_eq!(qualified.remedy().finality(), verified_remedy().outcome().finality());
        assert!(qualified.receipt().current_revalidation_required_before_dispatch());
    }
}
