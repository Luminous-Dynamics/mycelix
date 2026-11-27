//! Reference validation for plural, owner-scoped Craft economic agreements.
//!
//! This crate validates immutable agreement references and local invariants. It
//! does not issue instruments, calculate FX, mutate balances or journals, reserve
//! resources, send payments, or declare settlement finality. Profile signatures,
//! participant signatures, authority eligibility, and domain-owned effects must
//! be verified or executed by their respective owners before use.

#![forbid(unsafe_code)]

use std::collections::HashSet;

/// Immutable identity of an economic profile, including issuer and revision.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct EconomicProfileRefV1 {
    /// Canonical profile identifier; never a ticker or display symbol.
    pub profile_id: String,
    /// Issuer/community identity.
    pub issuer_id: String,
    /// Monotonically versioned profile revision.
    pub revision: u64,
    /// Digest of the exact profile document.
    pub digest: [u8; 32],
}

/// Economic arrangement types recognized by the agreement envelope.
///
/// Non-transferable reputation is intentionally not an economic component type.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum EconomicSystemKindV1 {
    /// Fiat wage, invoice or equivalent monetary obligation.
    FiatCompensation,
    /// Issuer-scoped community mutual credit.
    MutualCredit,
    /// Community-scoped time or service credit.
    TimeCredit,
    /// Grant, award or public-benefit obligation.
    Grant,
    /// Access to a commons-owned resource or budget.
    CommonsAllocation,
    /// Cooperative member capital, patronage or surplus participation.
    CooperativeParticipation,
    /// In-kind goods, materials, equipment or services.
    InKind,
    /// Explicitly defined, non-monetary contribution allocation.
    ContributionAllocation,
    /// Explicitly voluntary contribution with no implied payment.
    VoluntaryContribution,
    /// A separately governed arrangement with its own profile semantics.
    Custom,
}

/// Lifecycle of a profile in its owning registry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProfileStatusV1 {
    /// May be referenced for new obligations.
    Active,
    /// Temporarily unavailable for new obligations.
    Suspended,
    /// Retired; usable for historical interpretation only.
    Retired,
}

/// Owner-authenticated profile snapshot supplied by the calling domain.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EconomicProfileV1 {
    /// Exact immutable profile identity.
    pub reference: EconomicProfileRefV1,
    /// Typed economic arrangement.
    pub system_kind: EconomicSystemKindV1,
    /// Authority that owns issuance, allocation or settlement.
    pub owner_authority_id: String,
    /// Canonical unit profile. This is not a display ticker.
    pub unit_profile_id: String,
    /// Exact decimal scale used by the owner profile.
    pub unit_scale: u8,
    /// Current lifecycle state.
    pub status: ProfileStatusV1,
    /// Inclusive start timestamp (Unix seconds).
    pub valid_from: i64,
    /// Exclusive end timestamp, if bounded.
    pub valid_until: Option<i64>,
}

/// Exact quantity in the unit profile's atomic units.
///
/// There is intentionally no arithmetic or conversion implementation.
/// Equal integers in different profiles are not fungible.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactQuantityV1 {
    /// Positive number of atomic units.
    pub atoms: u128,
    /// Decimal scale pinned to the profile.
    pub scale: u8,
}

/// One independently typed economic obligation inside a work agreement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EconomicComponentV1 {
    /// Stable ID within the agreement.
    pub component_id: String,
    /// Exact category, which must match the referenced profile.
    pub system_kind: EconomicSystemKindV1,
    /// Immutable issuer-scoped profile reference.
    pub profile: EconomicProfileRefV1,
    /// Exact owner expected to authorize the economic effect.
    pub owner_authority_id: String,
    /// Canonical unit profile.
    pub unit_profile_id: String,
    /// Exact amount; no implicit unit conversion is performed.
    pub quantity: ExactQuantityV1,
    /// Owner-scoped obligation reference.
    pub obligation_id: String,
    /// Digest of the immutable owner-side obligation intent.
    pub obligation_digest: [u8; 32],
    /// Work milestone or condition that makes the obligation claimable.
    pub milestone_id: String,
    /// Reference to the evidence/acceptance policy for this component.
    pub evidence_policy_id: String,
}

/// One party's acceptance of the exact agreement terms.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ParticipantConsentV1 {
    /// Participant identity.
    pub participant_id: String,
    /// Must equal the enclosing agreement's terms digest.
    pub terms_digest: [u8; 32],
    /// Acceptance timestamp (Unix seconds).
    pub accepted_at: i64,
    /// Withdrawal timestamp, if any; effective at that timestamp.
    pub withdrawn_at: Option<i64>,
}

/// Explicit quote metadata for a conversion between two profiles.
///
/// This is a quote reference, not an exchange execution. The calling system must
/// verify the quote issuer, signatures, source, and owner-side effect separately.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConversionQuoteV1 {
    /// Stable quote identifier.
    pub quote_id: String,
    /// Digest of the exact quote document.
    pub quote_digest: [u8; 32],
    /// Source profile.
    pub from_profile: EconomicProfileRefV1,
    /// Destination profile.
    pub to_profile: EconomicProfileRefV1,
    /// Agreement component representing the source side of this quote.
    pub from_component_id: String,
    /// Agreement component representing the destination side of this quote.
    pub to_component_id: String,
    /// Exact source quantity.
    pub from_quantity: ExactQuantityV1,
    /// Exact quoted destination quantity.
    pub to_quantity: ExactQuantityV1,
    /// Optional, separately typed quote fee.
    pub fee: Option<ConversionFeeV1>,
    /// Named conversion method or policy.
    pub method: String,
    /// Evidence/source reference used to produce the quote.
    pub source_reference: String,
    /// Quote issue time (Unix seconds).
    pub issued_at: i64,
    /// Exclusive expiry timestamp (Unix seconds).
    pub expires_at: i64,
    /// Participants who explicitly accepted this quote.
    pub accepted_by: Vec<String>,
}

/// A quote fee is itself exact and owner-scoped.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConversionFeeV1 {
    /// Profile in which the fee is denominated.
    pub profile: EconomicProfileRefV1,
    /// Exact fee amount.
    pub quantity: ExactQuantityV1,
}

/// Immutable work agreement envelope.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CraftEconomicAgreementV1 {
    /// Stable agreement identifier.
    pub agreement_id: String,
    /// Work opportunity or work-scope reference.
    pub work_reference: String,
    /// Applicable jurisdiction or declared governing context.
    pub jurisdiction: String,
    /// Digest of the exact terms document.
    pub terms_digest: [u8; 32],
    /// All parties required to accept these terms.
    pub participants: Vec<String>,
    /// Consent records bound to terms_digest.
    pub consents: Vec<ParticipantConsentV1>,
    /// Independently typed obligations; they are never added together here.
    pub components: Vec<EconomicComponentV1>,
    /// Optional explicit, mutually accepted conversion quote references.
    pub conversion_quotes: Vec<ConversionQuoteV1>,
}

/// Reasons an agreement is rejected for new obligations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AgreementErrorV1 {
    /// Required identifier or policy reference is blank.
    EmptyField,
    /// Agreement or profile digest is the all-zero sentinel.
    MissingDigest,
    /// Fewer than two parties are present.
    TooFewParticipants,
    /// A participant or consent is duplicated.
    DuplicateParticipantOrConsent,
    /// Consent belongs to a non-party.
    ConsentFromNonParticipant,
    /// A required participant did not consent.
    MissingConsent,
    /// Consent is for a different terms digest.
    ConsentDigestMismatch,
    /// Consent is future-dated.
    FutureConsent,
    /// A party withdrew consent before new obligations were authorized.
    ConsentWithdrawn,
    /// Withdrawal is earlier than acceptance.
    InvalidWithdrawalTime,
    /// No economic components are present.
    EmptyComponents,
    /// A component ID is duplicated.
    DuplicateComponentId,
    /// An owner/obligation pair is repeated.
    DuplicateOwnerObligation,
    /// A profile is not present in the trusted registry snapshot.
    UnknownProfile,
    /// The same issuer/profile exists but its revision or digest differs.
    StaleProfile,
    /// The trusted registry contains multiple exact copies of a profile.
    DuplicateProfile,
    /// The profile document violates its own invariants.
    InvalidProfile,
    /// Profile is suspended or retired.
    ProfileNotActive,
    /// Profile is outside its effective period.
    ProfileOutsideValidity,
    /// Component arrangement does not match the profile.
    SystemKindMismatch,
    /// Component owner differs from profile owner.
    OwnerMismatch,
    /// Unit profile differs from the pinned owner profile.
    UnitMismatch,
    /// Quantity is zero or has an unsupported scale.
    InvalidQuantity,
    /// Component lacks an immutable owner-obligation digest.
    MissingObligationDigest,
    /// Quote identifier is duplicated.
    DuplicateQuoteId,
    /// Quote digest is missing.
    MissingQuoteDigest,
    /// A quote uses the same source and destination profile.
    QuoteDoesNotConvert,
    /// Quote validity interval is malformed or expired.
    QuoteExpiredOrInvalid,
    /// Quote source or destination profile is not used by this agreement.
    QuoteNotBoundToAgreement,
    /// Quote lacks one or more explicit participant acceptances.
    QuoteNotAcceptedByAllParties,
    /// Quote fee is malformed.
    InvalidQuoteFee,
}

/// Validate the agreement for new obligations at now.
///
/// This function is deliberately read-only. It validates contract references and
/// local invariants; it does not create, authorize, or settle any economic effect.
pub fn validate_agreement(
    agreement: &CraftEconomicAgreementV1,
    profiles: &[EconomicProfileV1],
    now: i64,
) -> Result<(), AgreementErrorV1> {
    require_non_empty(&agreement.agreement_id)?;
    require_non_empty(&agreement.work_reference)?;
    require_non_empty(&agreement.jurisdiction)?;
    require_digest(&agreement.terms_digest)?;

    if agreement.participants.len() < 2 {
        return Err(AgreementErrorV1::TooFewParticipants);
    }
    let mut parties = HashSet::new();
    for participant in &agreement.participants {
        require_non_empty(participant)?;
        if !parties.insert(participant.as_str()) {
            return Err(AgreementErrorV1::DuplicateParticipantOrConsent);
        }
    }

    let mut consented = HashSet::new();
    for consent in &agreement.consents {
        require_non_empty(&consent.participant_id)?;
        if !parties.contains(consent.participant_id.as_str()) {
            return Err(AgreementErrorV1::ConsentFromNonParticipant);
        }
        if !consented.insert(consent.participant_id.as_str()) {
            return Err(AgreementErrorV1::DuplicateParticipantOrConsent);
        }
        if consent.terms_digest != agreement.terms_digest {
            return Err(AgreementErrorV1::ConsentDigestMismatch);
        }
        if consent.accepted_at > now {
            return Err(AgreementErrorV1::FutureConsent);
        }
        if let Some(withdrawn_at) = consent.withdrawn_at {
            if withdrawn_at < consent.accepted_at {
                return Err(AgreementErrorV1::InvalidWithdrawalTime);
            }
            if now >= withdrawn_at {
                return Err(AgreementErrorV1::ConsentWithdrawn);
            }
        }
    }
    for participant in &parties {
        if !consented.contains(*participant) {
            return Err(AgreementErrorV1::MissingConsent);
        }
    }

    if agreement.components.is_empty() {
        return Err(AgreementErrorV1::EmptyComponents);
    }

    let mut component_ids = HashSet::new();
    let mut owner_obligations = HashSet::new();
    for component in &agreement.components {
        require_non_empty(&component.component_id)?;
        require_non_empty(&component.owner_authority_id)?;
        require_non_empty(&component.unit_profile_id)?;
        require_non_empty(&component.obligation_id)?;
        require_non_empty(&component.milestone_id)?;
        require_non_empty(&component.evidence_policy_id)?;

        if !component_ids.insert(component.component_id.as_str()) {
            return Err(AgreementErrorV1::DuplicateComponentId);
        }
        if !owner_obligations.insert((
            component.owner_authority_id.as_str(),
            component.obligation_id.as_str(),
        )) {
            return Err(AgreementErrorV1::DuplicateOwnerObligation);
        }
        if component.obligation_digest.iter().all(|byte| *byte == 0) {
            return Err(AgreementErrorV1::MissingObligationDigest);
        }
        if component.quantity.atoms == 0 || component.quantity.scale > 18 {
            return Err(AgreementErrorV1::InvalidQuantity);
        }

        let profile = find_active_profile(&component.profile, profiles, now)?;
        if component.system_kind != profile.system_kind {
            return Err(AgreementErrorV1::SystemKindMismatch);
        }
        if component.owner_authority_id != profile.owner_authority_id {
            return Err(AgreementErrorV1::OwnerMismatch);
        }
        if component.unit_profile_id != profile.unit_profile_id
            || component.quantity.scale != profile.unit_scale
        {
            return Err(AgreementErrorV1::UnitMismatch);
        }
    }

    let mut quote_ids = HashSet::new();
    for quote in &agreement.conversion_quotes {
        if !quote_ids.insert(quote.quote_id.as_str()) {
            return Err(AgreementErrorV1::DuplicateQuoteId);
        }
        validate_quote(quote, agreement, profiles, now, &parties)?;
    }
    Ok(())
}

fn validate_quote(
    quote: &ConversionQuoteV1,
    agreement: &CraftEconomicAgreementV1,
    profiles: &[EconomicProfileV1],
    now: i64,
    parties: &HashSet<&str>,
) -> Result<(), AgreementErrorV1> {
    require_non_empty(&quote.quote_id)?;
    require_non_empty(&quote.method)?;
    require_non_empty(&quote.source_reference)?;
    if quote.quote_digest.iter().all(|byte| *byte == 0) {
        return Err(AgreementErrorV1::MissingQuoteDigest);
    }
    if quote.from_profile.profile_id == quote.to_profile.profile_id
        && quote.from_profile.issuer_id == quote.to_profile.issuer_id
    {
        return Err(AgreementErrorV1::QuoteDoesNotConvert);
    }
    require_non_empty(&quote.from_component_id)?;
    require_non_empty(&quote.to_component_id)?;
    if quote.from_component_id == quote.to_component_id {
        return Err(AgreementErrorV1::QuoteNotBoundToAgreement);
    }
    if quote.issued_at >= quote.expires_at || now < quote.issued_at || now >= quote.expires_at {
        return Err(AgreementErrorV1::QuoteExpiredOrInvalid);
    }

    let from_profile = find_active_profile(&quote.from_profile, profiles, now)?;
    let to_profile = find_active_profile(&quote.to_profile, profiles, now)?;
    if quote.from_quantity.atoms == 0
        || quote.to_quantity.atoms == 0
        || quote.from_quantity.scale != from_profile.unit_scale
        || quote.to_quantity.scale != to_profile.unit_scale
    {
        return Err(AgreementErrorV1::InvalidQuantity);
    }

    let from_component = agreement.components.iter()
        .find(|component| component.component_id == quote.from_component_id)
        .ok_or(AgreementErrorV1::QuoteNotBoundToAgreement)?;
    let to_component = agreement.components.iter()
        .find(|component| component.component_id == quote.to_component_id)
        .ok_or(AgreementErrorV1::QuoteNotBoundToAgreement)?;
    if from_component.profile != quote.from_profile
        || to_component.profile != quote.to_profile
        || from_component.quantity != quote.from_quantity
        || to_component.quantity != quote.to_quantity
    {
        return Err(AgreementErrorV1::QuoteNotBoundToAgreement);
    }

    let mut accepted = HashSet::new();
    for participant in &quote.accepted_by {
        require_non_empty(participant)?;
        if !accepted.insert(participant.as_str()) {
            return Err(AgreementErrorV1::DuplicateParticipantOrConsent);
        }
    }
    if accepted.len() != parties.len() || !accepted.iter().all(|p| parties.contains(*p)) {
        return Err(AgreementErrorV1::QuoteNotAcceptedByAllParties);
    }

    if let Some(fee) = &quote.fee {
        let fee_profile = find_active_profile(&fee.profile, profiles, now)?;
        if fee.quantity.atoms == 0
            || fee.quantity.scale > 18
            || fee.quantity.scale != fee_profile.unit_scale
        {
            return Err(AgreementErrorV1::InvalidQuoteFee);
        }
    }
    Ok(())
}

fn find_active_profile<'a>(
    reference: &EconomicProfileRefV1,
    profiles: &'a [EconomicProfileV1],
    now: i64,
) -> Result<&'a EconomicProfileV1, AgreementErrorV1> {
    let mut identity_seen = false;
    let mut exact: Option<&EconomicProfileV1> = None;
    for profile in profiles {
        if profile.reference.profile_id == reference.profile_id
            && profile.reference.issuer_id == reference.issuer_id
        {
            identity_seen = true;
            if &profile.reference == reference {
                if exact.is_some() {
                    return Err(AgreementErrorV1::DuplicateProfile);
                }
                exact = Some(profile);
            }
        }
    }
    if !identity_seen {
        return Err(AgreementErrorV1::UnknownProfile);
    }
    let profile = exact.ok_or(AgreementErrorV1::StaleProfile)?;
    validate_profile(profile)?;
    if profile.status != ProfileStatusV1::Active {
        return Err(AgreementErrorV1::ProfileNotActive);
    }
    if now < profile.valid_from
        || profile.valid_until.map(|end| now >= end).unwrap_or(false)
    {
        return Err(AgreementErrorV1::ProfileOutsideValidity);
    }
    Ok(profile)
}

fn validate_profile(profile: &EconomicProfileV1) -> Result<(), AgreementErrorV1> {
    require_non_empty(&profile.reference.profile_id)?;
    require_non_empty(&profile.reference.issuer_id)?;
    require_non_empty(&profile.owner_authority_id)?;
    require_non_empty(&profile.unit_profile_id)?;
    if profile.reference.revision == 0 || profile.unit_scale > 18 {
        return Err(AgreementErrorV1::InvalidProfile);
    }
    require_digest(&profile.reference.digest)?;
    if let Some(end) = profile.valid_until {
        if end <= profile.valid_from {
            return Err(AgreementErrorV1::InvalidProfile);
        }
    }
    Ok(())
}

fn require_non_empty(value: &str) -> Result<(), AgreementErrorV1> {
    if value.trim().is_empty() {
        Err(AgreementErrorV1::EmptyField)
    } else {
        Ok(())
    }
}

fn require_digest(value: &[u8; 32]) -> Result<(), AgreementErrorV1> {
    if value.iter().all(|byte| *byte == 0) {
        Err(AgreementErrorV1::MissingDigest)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: i64 = 10;

    fn profile_ref(id: &str, issuer: &str, revision: u64, marker: u8) -> EconomicProfileRefV1 {
        EconomicProfileRefV1 {
            profile_id: id.to_owned(),
            issuer_id: issuer.to_owned(),
            revision,
            digest: [marker; 32],
        }
    }

    fn profile(
        id: &str,
        issuer: &str,
        owner: &str,
        unit: &str,
        scale: u8,
        kind: EconomicSystemKindV1,
        marker: u8,
    ) -> EconomicProfileV1 {
        EconomicProfileV1 {
            reference: profile_ref(id, issuer, 1, marker),
            system_kind: kind,
            owner_authority_id: owner.to_owned(),
            unit_profile_id: unit.to_owned(),
            unit_scale: scale,
            status: ProfileStatusV1::Active,
            valid_from: 0,
            valid_until: None,
        }
    }

    fn component(id: &str, profile: &EconomicProfileV1, obligation: &str) -> EconomicComponentV1 {
        EconomicComponentV1 {
            component_id: id.to_owned(),
            system_kind: profile.system_kind,
            profile: profile.reference.clone(),
            owner_authority_id: profile.owner_authority_id.clone(),
            unit_profile_id: profile.unit_profile_id.clone(),
            quantity: ExactQuantityV1 { atoms: 100, scale: profile.unit_scale },
            obligation_id: obligation.to_owned(),
            obligation_digest: [7; 32],
            milestone_id: "milestone-1".to_owned(),
            evidence_policy_id: "evidence-policy-1".to_owned(),
        }
    }

    fn base() -> (CraftEconomicAgreementV1, Vec<EconomicProfileV1>) {
        let fiat = profile(
            "fiat-profile", "did:issuer:employer", "did:owner:payroll",
            "unit:usd-cent", 0, EconomicSystemKindV1::FiatCompensation, 1,
        );
        let mutual = profile(
            "local-credit", "did:issuer:community", "did:owner:community-ledger",
            "unit:service-credit", 0, EconomicSystemKindV1::MutualCredit, 2,
        );
        let digest = [9; 32];
        let agreement = CraftEconomicAgreementV1 {
            agreement_id: "agreement-1".to_owned(),
            work_reference: "work-1".to_owned(),
            jurisdiction: "ZA-GP".to_owned(),
            terms_digest: digest,
            participants: vec!["worker".to_owned(), "guild".to_owned()],
            consents: vec![
                ParticipantConsentV1 {
                    participant_id: "worker".to_owned(), terms_digest: digest,
                    accepted_at: 1, withdrawn_at: None,
                },
                ParticipantConsentV1 {
                    participant_id: "guild".to_owned(), terms_digest: digest,
                    accepted_at: 1, withdrawn_at: None,
                },
            ],
            components: vec![
                component("wage", &fiat, "obligation-wage-1"),
                component("local-services", &mutual, "obligation-credit-1"),
            ],
            conversion_quotes: Vec::new(),
        };
        (agreement, vec![fiat, mutual])
    }

    #[test]
    fn mixed_components_are_valid_without_becoming_fungible() {
        let (agreement, profiles) = base();
        assert!(agreement.conversion_quotes.is_empty());
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Ok(()));
    }

    #[test]
    fn unknown_profile_fails_closed() {
        let (agreement, profiles) = base();
        assert_eq!(validate_agreement(&agreement, &profiles[..1], NOW), Err(AgreementErrorV1::UnknownProfile));
    }

    #[test]
    fn stale_profile_revision_fails_closed() {
        let (mut agreement, profiles) = base();
        agreement.components[0].profile.revision = 2;
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::StaleProfile));
    }

    #[test]
    fn suspended_profile_cannot_back_new_obligations() {
        let (agreement, mut profiles) = base();
        profiles[0].status = ProfileStatusV1::Suspended;
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::ProfileNotActive));
    }

    #[test]
    fn owner_mismatch_fails_closed() {
        let (mut agreement, profiles) = base();
        agreement.components[0].owner_authority_id = "did:owner:other".to_owned();
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::OwnerMismatch));
    }

    #[test]
    fn unit_mismatch_fails_closed() {
        let (mut agreement, profiles) = base();
        agreement.components[0].unit_profile_id = "unit:another".to_owned();
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::UnitMismatch));
    }

    #[test]
    fn duplicate_owner_obligation_is_rejected() {
        let (mut agreement, profiles) = base();
        let mut duplicate = agreement.components[0].clone();
        duplicate.component_id = "second-wage-component".to_owned();
        agreement.components.push(duplicate);
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::DuplicateOwnerObligation));
    }

    #[test]
    fn every_party_must_consent_to_exact_terms() {
        let (mut agreement, profiles) = base();
        agreement.consents.pop();
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::MissingConsent));
    }

    #[test]
    fn withdrawal_blocks_new_obligations() {
        let (mut agreement, profiles) = base();
        agreement.consents[0].withdrawn_at = Some(NOW);
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::ConsentWithdrawn));
    }

    #[test]
    fn conversion_quote_requires_explicit_acceptance_by_every_party() {
        let (mut agreement, profiles) = base();
        agreement.conversion_quotes.push(ConversionQuoteV1 {
            quote_id: "quote-1".to_owned(),
            quote_digest: [8; 32],
            from_profile: profiles[0].reference.clone(),
            to_profile: profiles[1].reference.clone(),
            from_component_id: "wage".to_owned(),
            to_component_id: "local-services".to_owned(),
            from_quantity: ExactQuantityV1 { atoms: 100, scale: 0 },
            to_quantity: ExactQuantityV1 { atoms: 100, scale: 0 },
            fee: None,
            method: "issuer-quote".to_owned(),
            source_reference: "quote-evidence-1".to_owned(),
            issued_at: 1,
            expires_at: 20,
            accepted_by: vec!["worker".to_owned()],
        });
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::QuoteNotAcceptedByAllParties));
    }

    #[test]
    fn fully_accepted_quote_remains_only_a_quote_reference() {
        let (mut agreement, profiles) = base();
        agreement.conversion_quotes.push(ConversionQuoteV1 {
            quote_id: "quote-2".to_owned(),
            quote_digest: [8; 32],
            from_profile: profiles[0].reference.clone(),
            to_profile: profiles[1].reference.clone(),
            from_component_id: "wage".to_owned(),
            to_component_id: "local-services".to_owned(),
            from_quantity: ExactQuantityV1 { atoms: 100, scale: 0 },
            to_quantity: ExactQuantityV1 { atoms: 100, scale: 0 },
            fee: None,
            method: "issuer-quote".to_owned(),
            source_reference: "quote-evidence-2".to_owned(),
            issued_at: 1,
            expires_at: 20,
            accepted_by: vec!["worker".to_owned(), "guild".to_owned()],
        });
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Ok(()));
    }

    #[test]
    fn expired_conversion_quote_is_rejected() {
        let (mut agreement, profiles) = base();
        agreement.conversion_quotes.push(ConversionQuoteV1 {
            quote_id: "quote-3".to_owned(),
            quote_digest: [8; 32],
            from_profile: profiles[0].reference.clone(),
            to_profile: profiles[1].reference.clone(),
            from_component_id: "wage".to_owned(),
            to_component_id: "local-services".to_owned(),
            from_quantity: ExactQuantityV1 { atoms: 100, scale: 0 },
            to_quantity: ExactQuantityV1 { atoms: 100, scale: 0 },
            fee: None,
            method: "issuer-quote".to_owned(),
            source_reference: "quote-evidence-3".to_owned(),
            issued_at: 1,
            expires_at: NOW,
            accepted_by: vec!["worker".to_owned(), "guild".to_owned()],
        });
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::QuoteExpiredOrInvalid));
    }


    #[test]
    fn conversion_quote_must_match_the_bound_component_quantities() {
        let (mut agreement, profiles) = base();
        agreement.conversion_quotes.push(ConversionQuoteV1 {
            quote_id: "quote-amount-mismatch".to_owned(),
            quote_digest: [8; 32],
            from_profile: profiles[0].reference.clone(),
            to_profile: profiles[1].reference.clone(),
            from_component_id: "wage".to_owned(),
            to_component_id: "local-services".to_owned(),
            from_quantity: ExactQuantityV1 { atoms: 100, scale: 0 },
            to_quantity: ExactQuantityV1 { atoms: 99, scale: 0 },
            fee: None,
            method: "issuer-quote".to_owned(),
            source_reference: "quote-evidence-mismatch".to_owned(),
            issued_at: 1,
            expires_at: 20,
            accepted_by: vec!["worker".to_owned(), "guild".to_owned()],
        });
        assert_eq!(
            validate_agreement(&agreement, &profiles, NOW),
            Err(AgreementErrorV1::QuoteNotBoundToAgreement)
        );
    }

    #[test]
    fn quantity_scale_must_match_profile() {
        let (mut agreement, profiles) = base();
        agreement.components[0].quantity.scale = 2;
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::UnitMismatch));
    }

    #[test]
    fn zero_quantity_is_rejected() {
        let (mut agreement, profiles) = base();
        agreement.components[0].quantity.atoms = 0;
        assert_eq!(validate_agreement(&agreement, &profiles, NOW), Err(AgreementErrorV1::InvalidQuantity));
    }
}
