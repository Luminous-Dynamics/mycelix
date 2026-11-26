//! Directional, scoped inter-institutional cooperation agreements.
//!
//! This reference core checks an already authenticated snapshot of a negotiated
//! agreement. It does not verify signatures, charters or grantor eligibility;
//! those remain the responsibility of each party's governance and Identity
//! owners. It deliberately provides no transitive trust or downstream effects.

#![forbid(unsafe_code)]

use std::collections::HashSet;

/// Current schema version for inter-institutional cooperation agreements.
pub const FEDERATION_AGREEMENT_SCHEMA_VERSION: u16 = 1;

/// An exact local authorization decision made by one institution.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct LocalAuthorityRefV1 {
    /// Local charter policy identifier.
    pub charter_policy_id: String,
    /// Exact adopted charter revision.
    pub charter_revision: u64,
    /// Digest of the exact adopted charter policy.
    pub charter_digest: [u8; 32],
    /// Local decision or delegation record identifier.
    pub decision_id: String,
    /// Digest of the exact local authorization decision.
    pub decision_digest: [u8; 32],
}

/// One institution participating in an agreement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InstitutionPartyV1 {
    /// Canonical institution identity.
    pub institution_id: String,
    /// Local authority reference authorizing this exact agreement revision.
    pub local_authority: LocalAuthorityRefV1,
}

/// An exact data-disclosure profile owned by the relevant data controller.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct DisclosureProfileRefV1 {
    /// Stable disclosure profile identifier.
    pub profile_id: String,
    /// Monotonically versioned disclosure profile revision.
    pub revision: u64,
    /// Digest of the exact profile defining permitted fields and purpose.
    pub digest: [u8; 32],
}

/// Cooperation scope categories. These describe the agreement only; they do
/// not themselves appoint assessors, reserve assets or execute any operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum CooperationKindV1 {
    /// Recognize a specific evidence or assessment framework for a bounded use.
    EvidenceFrameworkRecognition,
    /// Recognize a specific assessor-authority profile without issuing authority.
    AssessorFrameworkRecognition,
    /// Exchange training opportunities under separately accepted learner terms.
    TrainingExchange,
    /// Participate in a named project under its independent project authority.
    ProjectParticipation,
    /// Disclose purpose-limited evidence under a pinned data profile.
    EvidenceDisclosure,
    /// Coordinate mutual-aid requests without committing another party's assets.
    MutualAidCoordination,
    /// Coordinate shared-resource requests; reservation and custody stay owner-side.
    SharedResourceCoordination,
}

/// One exact, versioned and purpose-bound cooperation scope.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct CooperationScopeV1 {
    /// Stable scope identifier within the agreement.
    pub scope_id: String,
    /// Kind of cooperation permitted by this scope.
    pub kind: CooperationKindV1,
    /// Monotonically versioned scope revision.
    pub revision: u64,
    /// Digest of the exact scope terms.
    pub digest: [u8; 32],
    /// Purpose identifier for which interoperation is permitted.
    pub purpose_id: String,
    /// Jurisdiction in which the scope is intended to apply.
    pub jurisdiction_id: String,
    /// Exact disclosure profile; this does not authorize disclosure by itself.
    pub disclosure_profile: DisclosureProfileRefV1,
}

impl CooperationScopeV1 {
    fn validate(&self) -> Result<(), AgreementErrorV1> {
        require_non_empty(&self.scope_id)?;
        require_non_empty(&self.purpose_id)?;
        require_non_empty(&self.jurisdiction_id)?;
        if self.revision == 0 || self.disclosure_profile.revision == 0 {
            return Err(AgreementErrorV1::InvalidRevision);
        }
        require_digest(&self.digest)?;
        require_non_empty(&self.disclosure_profile.profile_id)?;
        require_digest(&self.disclosure_profile.digest)
    }
}

/// One explicitly directed interoperation edge.
///
/// The edge means only that from_institution accepts the named scope from
/// to_institution. Reverse recognition requires a separate edge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DirectedCooperationEdgeV1 {
    /// Stable edge identifier within the agreement.
    pub edge_id: String,
    /// Institution that chooses to rely on or cooperate with the other party.
    pub from_institution: String,
    /// Institution whose specified framework/offer is accepted.
    pub to_institution: String,
    /// Exact scope authorizing this directed relationship.
    pub scope_id: String,
}

/// One party's consent to the exact agreement revision and its own local mandate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PartyConsentV1 {
    /// Institution giving consent.
    pub institution_id: String,
    /// Must equal the agreement's exact terms digest.
    pub terms_digest: [u8; 32],
    /// Local authorization decision used for this consent.
    pub local_authority: LocalAuthorityRefV1,
    /// Time at which the institution accepted these terms.
    pub accepted_at: u64,
    /// Withdrawal becomes effective at this timestamp, if present.
    pub withdrawn_at: Option<u64>,
}

/// Lifecycle state supplied by the agreement's trusted governance host.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AgreementStatusV1 {
    /// Draft terms; no interoperation is permitted.
    Proposed,
    /// Every named party has accepted the exact current terms.
    Active,
    /// Temporarily paused by an authorized agreement process.
    Suspended,
    /// Terminated; historical interpretation only.
    Terminated,
}

/// Immutable envelope for a bilateral or multilateral cooperation agreement.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FederationAgreementV1 {
    /// Must equal FEDERATION_AGREEMENT_SCHEMA_VERSION.
    pub schema_version: u16,
    /// Stable agreement identifier.
    pub agreement_id: String,
    /// Monotonically versioned agreement revision.
    pub revision: u64,
    /// Digest of the exact agreement terms.
    pub terms_digest: [u8; 32],
    /// Inclusive agreement start time.
    pub effective_at: u64,
    /// Exclusive expiry time.
    pub expires_at: u64,
    /// Named autonomous institutional parties.
    pub parties: Vec<InstitutionPartyV1>,
    /// Versioned scopes explicitly negotiated by the parties.
    pub scopes: Vec<CooperationScopeV1>,
    /// Directed trust edges. No reverse or transitive edge is implied.
    pub edges: Vec<DirectedCooperationEdgeV1>,
    /// Consent records, one for every named party.
    pub consents: Vec<PartyConsentV1>,
    /// Current lifecycle status from the trusted agreement host.
    pub status: AgreementStatusV1,
}

impl FederationAgreementV1 {
    /// Validate structural invariants without authenticating the host snapshot.
    pub fn validate(&self) -> Result<(), AgreementErrorV1> {
        if self.schema_version != FEDERATION_AGREEMENT_SCHEMA_VERSION {
            return Err(AgreementErrorV1::UnsupportedSchema);
        }
        require_non_empty(&self.agreement_id)?;
        if self.revision == 0 {
            return Err(AgreementErrorV1::InvalidRevision);
        }
        require_digest(&self.terms_digest)?;
        if self.expires_at <= self.effective_at {
            return Err(AgreementErrorV1::InvalidTimeRange);
        }
        if self.parties.len() < 2 {
            return Err(AgreementErrorV1::TooFewParties);
        }
        if self.scopes.is_empty() || self.edges.is_empty() {
            return Err(AgreementErrorV1::EmptyCooperationScope);
        }

        let mut party_ids = HashSet::new();
        for party in &self.parties {
            require_non_empty(&party.institution_id)?;
            if !party_ids.insert(party.institution_id.as_str()) {
                return Err(AgreementErrorV1::DuplicateParty);
            }
            validate_authority(&party.local_authority)?;
        }

        let mut scope_ids = HashSet::new();
        for scope in &self.scopes {
            scope.validate()?;
            if !scope_ids.insert(scope.scope_id.as_str()) {
                return Err(AgreementErrorV1::DuplicateScope);
            }
        }

        let mut edge_ids = HashSet::new();
        let mut edge_keys = HashSet::new();
        for edge in &self.edges {
            require_non_empty(&edge.edge_id)?;
            require_non_empty(&edge.from_institution)?;
            require_non_empty(&edge.to_institution)?;
            require_non_empty(&edge.scope_id)?;
            if edge.from_institution == edge.to_institution {
                return Err(AgreementErrorV1::SelfTrustEdge);
            }
            if !party_ids.contains(edge.from_institution.as_str())
                || !party_ids.contains(edge.to_institution.as_str())
            {
                return Err(AgreementErrorV1::EdgePartyNotInAgreement);
            }
            if !scope_ids.contains(edge.scope_id.as_str()) {
                return Err(AgreementErrorV1::UnknownScope);
            }
            if !edge_ids.insert(edge.edge_id.as_str()) {
                return Err(AgreementErrorV1::DuplicateEdgeId);
            }
            if !edge_keys.insert((
                edge.from_institution.as_str(),
                edge.to_institution.as_str(),
                edge.scope_id.as_str(),
            )) {
                return Err(AgreementErrorV1::DuplicateDirectedEdge);
            }
        }

        let mut consented = HashSet::new();
        for consent in &self.consents {
            require_non_empty(&consent.institution_id)?;
            if !party_ids.contains(consent.institution_id.as_str()) {
                return Err(AgreementErrorV1::ConsentFromNonParty);
            }
            if !consented.insert(consent.institution_id.as_str()) {
                return Err(AgreementErrorV1::DuplicateConsent);
            }
            if consent.terms_digest != self.terms_digest {
                return Err(AgreementErrorV1::ConsentDigestMismatch);
            }
            let party = self
                .parties
                .iter()
                .find(|party| party.institution_id == consent.institution_id)
                .expect("party membership was checked above");
            if consent.local_authority != party.local_authority {
                return Err(AgreementErrorV1::LocalAuthorityMismatch);
            }
            if consent.accepted_at < self.effective_at {
                return Err(AgreementErrorV1::ConsentBeforeAgreement);
            }
            if let Some(withdrawn_at) = consent.withdrawn_at {
                if withdrawn_at < consent.accepted_at {
                    return Err(AgreementErrorV1::InvalidWithdrawalTime);
                }
            }
        }

        Ok(())
    }
}

/// A request to use one direct cooperation edge for one exact purpose.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CooperationRequestV1 {
    /// Institution requesting to rely on the other party.
    pub from_institution: String,
    /// Institution whose framework/offer is being relied on.
    pub to_institution: String,
    /// Exact scope reference already pinned by the agreement.
    pub scope: CooperationScopeV1,
    /// Purpose requested by the consuming application.
    pub purpose_id: String,
    /// Jurisdiction asserted by the consuming application.
    pub jurisdiction_id: String,
    /// Evaluation time (Unix seconds).
    pub at: u64,
}

/// Why the agreement cannot authorize the requested interoperation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AgreementErrorV1 {
    /// Schema version is not supported.
    UnsupportedSchema,
    /// A required identifier is blank or padded.
    EmptyField,
    /// A required digest is the all-zero sentinel.
    MissingDigest,
    /// A revision must be greater than zero.
    InvalidRevision,
    /// Agreement validity interval is malformed.
    InvalidTimeRange,
    /// At least two autonomous parties are required.
    TooFewParties,
    /// No scopes or directed edges exist.
    EmptyCooperationScope,
    /// An institution is repeated.
    DuplicateParty,
    /// A scope identifier is repeated.
    DuplicateScope,
    /// An edge identifier is repeated.
    DuplicateEdgeId,
    /// An exact directed edge is repeated.
    DuplicateDirectedEdge,
    /// An edge references an institution outside the agreement.
    EdgePartyNotInAgreement,
    /// An edge references a scope not declared by the agreement.
    UnknownScope,
    /// An institution cannot establish a self-trust edge.
    SelfTrustEdge,
    /// A consent is supplied by an institution not named as a party.
    ConsentFromNonParty,
    /// A party has more than one consent record.
    DuplicateConsent,
    /// Consent is bound to another terms digest.
    ConsentDigestMismatch,
    /// Consent references a different local authority mandate.
    LocalAuthorityMismatch,
    /// Consent predates the agreement's effective time.
    ConsentBeforeAgreement,
    /// Withdrawal predates acceptance.
    InvalidWithdrawalTime,
    /// A local charter or decision reference is incomplete.
    InvalidLocalAuthority,
}

/// Why an interoperation request is not authorized by the exact agreement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DenialReasonV1 {
    /// Agreement structure is invalid.
    InvalidAgreement,
    /// Agreement is proposed, suspended or terminated.
    AgreementNotActive,
    /// Request is outside the agreement's validity window.
    AgreementOutsideValidity,
    /// One or more parties have not accepted the exact current terms.
    MissingPartyConsent,
    /// A party withdrew consent at or before the request time.
    PartyConsentWithdrawn,
    /// Request does not identify a direct, explicitly declared edge.
    NoDirectCooperationEdge,
    /// Scope, purpose, jurisdiction or disclosure profile differs.
    ScopeOrPurposeMismatch,
    /// Request is malformed.
    InvalidRequest,
}

/// Authorization outcome. It references the agreement edge used and causes
/// no downstream mutation or resource/economic effect.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CooperationDecisionV1 {
    /// The exact direct edge permits this limited interoperation request.
    Authorized {
        agreement_id: String,
        agreement_revision: u64,
        edge_id: String,
        scope_id: String,
    },
    /// The request is not authorized.
    Denied(DenialReasonV1),
}

/// Evaluate one exact directed request against a trusted agreement snapshot.
///
/// No graph traversal occurs. In particular, A→B and B→C do not produce A→C.
/// The caller must separately authenticate parties, local decisions, signatures,
/// status freshness and any downstream resource/evidence authority.
pub fn authorize_cooperation(
    agreement: &FederationAgreementV1,
    request: &CooperationRequestV1,
) -> CooperationDecisionV1 {
    if agreement.validate().is_err()
        || require_non_empty(&request.from_institution).is_err()
        || require_non_empty(&request.to_institution).is_err()
        || require_non_empty(&request.purpose_id).is_err()
        || require_non_empty(&request.jurisdiction_id).is_err()
        || request.scope.validate().is_err()
    {
        return CooperationDecisionV1::Denied(DenialReasonV1::InvalidRequest);
    }

    if agreement.status != AgreementStatusV1::Active {
        return CooperationDecisionV1::Denied(DenialReasonV1::AgreementNotActive);
    }
    if request.at < agreement.effective_at || request.at >= agreement.expires_at {
        return CooperationDecisionV1::Denied(DenialReasonV1::AgreementOutsideValidity);
    }

    for party in &agreement.parties {
        let Some(consent) = agreement
            .consents
            .iter()
            .find(|consent| consent.institution_id == party.institution_id)
        else {
            return CooperationDecisionV1::Denied(DenialReasonV1::MissingPartyConsent);
        };
        if consent.accepted_at > request.at {
            return CooperationDecisionV1::Denied(DenialReasonV1::MissingPartyConsent);
        }
        if consent
            .withdrawn_at
            .map(|withdrawn_at| request.at >= withdrawn_at)
            .unwrap_or(false)
        {
            return CooperationDecisionV1::Denied(DenialReasonV1::PartyConsentWithdrawn);
        }
    }

    let scope = agreement
        .scopes
        .iter()
        .find(|scope| scope.scope_id == request.scope.scope_id);
    let Some(scope) = scope else {
        return CooperationDecisionV1::Denied(DenialReasonV1::NoDirectCooperationEdge);
    };
    if scope != &request.scope
        || request.purpose_id != scope.purpose_id
        || request.jurisdiction_id != scope.jurisdiction_id
    {
        return CooperationDecisionV1::Denied(DenialReasonV1::ScopeOrPurposeMismatch);
    }

    let edge = agreement.edges.iter().find(|edge| {
        edge.from_institution == request.from_institution
            && edge.to_institution == request.to_institution
            && edge.scope_id == scope.scope_id
    });
    let Some(edge) = edge else {
        return CooperationDecisionV1::Denied(DenialReasonV1::NoDirectCooperationEdge);
    };

    CooperationDecisionV1::Authorized {
        agreement_id: agreement.agreement_id.clone(),
        agreement_revision: agreement.revision,
        edge_id: edge.edge_id.clone(),
        scope_id: scope.scope_id.clone(),
    }
}

fn validate_authority(value: &LocalAuthorityRefV1) -> Result<(), AgreementErrorV1> {
    require_non_empty(&value.charter_policy_id)?;
    require_non_empty(&value.decision_id)?;
    if value.charter_revision == 0 {
        return Err(AgreementErrorV1::InvalidRevision);
    }
    require_digest(&value.charter_digest)?;
    require_digest(&value.decision_digest)
}

fn is_non_empty_unpadded(value: &str) -> bool {
    !value.is_empty() && value.trim() == value
}

fn require_non_empty(value: &str) -> Result<(), AgreementErrorV1> {
    if is_non_empty_unpadded(value) {
        Ok(())
    } else {
        Err(AgreementErrorV1::EmptyField)
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

    const NOW: u64 = 10;

    fn authority(id: &str) -> LocalAuthorityRefV1 {
        LocalAuthorityRefV1 {
            charter_policy_id: format!("charter-{id}"),
            charter_revision: 1,
            charter_digest: [1; 32],
            decision_id: format!("decision-{id}"),
            decision_digest: [2; 32],
        }
    }

    fn scope() -> CooperationScopeV1 {
        CooperationScopeV1 {
            scope_id: "electrical-inspection-recognition".to_owned(),
            kind: CooperationKindV1::EvidenceFrameworkRecognition,
            revision: 1,
            digest: [3; 32],
            purpose_id: "water-pump-restoration".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            disclosure_profile: DisclosureProfileRefV1 {
                profile_id: "minimal-verification-receipt".to_owned(),
                revision: 1,
                digest: [4; 32],
            },
        }
    }

    fn party(id: &str) -> InstitutionPartyV1 {
        InstitutionPartyV1 {
            institution_id: id.to_owned(),
            local_authority: authority(id),
        }
    }

    fn consent(id: &str) -> PartyConsentV1 {
        PartyConsentV1 {
            institution_id: id.to_owned(),
            terms_digest: [5; 32],
            local_authority: authority(id),
            accepted_at: 1,
            withdrawn_at: None,
        }
    }

    fn edge(from: &str, to: &str, id: &str) -> DirectedCooperationEdgeV1 {
        DirectedCooperationEdgeV1 {
            edge_id: id.to_owned(),
            from_institution: from.to_owned(),
            to_institution: to.to_owned(),
            scope_id: scope().scope_id,
        }
    }

    fn agreement() -> FederationAgreementV1 {
        FederationAgreementV1 {
            schema_version: FEDERATION_AGREEMENT_SCHEMA_VERSION,
            agreement_id: "federation-agreement-1".to_owned(),
            revision: 1,
            terms_digest: [5; 32],
            effective_at: 1,
            expires_at: 100,
            parties: vec![party("institution-a"), party("institution-b")],
            scopes: vec![scope()],
            edges: vec![edge("institution-a", "institution-b", "edge-a-b")],
            consents: vec![consent("institution-a"), consent("institution-b")],
            status: AgreementStatusV1::Active,
        }
    }

    fn request(from: &str, to: &str) -> CooperationRequestV1 {
        CooperationRequestV1 {
            from_institution: from.to_owned(),
            to_institution: to.to_owned(),
            scope: scope(),
            purpose_id: "water-pump-restoration".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            at: NOW,
        }
    }

    #[test]
    fn exact_directed_edge_is_authorized() {
        assert_eq!(
            authorize_cooperation(&agreement(), &request("institution-a", "institution-b")),
            CooperationDecisionV1::Authorized {
                agreement_id: "federation-agreement-1".to_owned(),
                agreement_revision: 1,
                edge_id: "edge-a-b".to_owned(),
                scope_id: "electrical-inspection-recognition".to_owned(),
            }
        );
    }

    #[test]
    fn reverse_direction_is_not_implied() {
        assert_eq!(
            authorize_cooperation(&agreement(), &request("institution-b", "institution-a")),
            CooperationDecisionV1::Denied(DenialReasonV1::NoDirectCooperationEdge)
        );
    }

    #[test]
    fn transitive_trust_is_not_inferred() {
        let mut agreement = agreement();
        agreement.parties.push(party("institution-c"));
        agreement.consents.push(consent("institution-c"));
        agreement.edges.push(edge("institution-b", "institution-c", "edge-b-c"));

        assert_eq!(
            authorize_cooperation(&agreement, &request("institution-a", "institution-c")),
            CooperationDecisionV1::Denied(DenialReasonV1::NoDirectCooperationEdge)
        );
    }

    #[test]
    fn same_federation_membership_without_direct_edge_is_not_trust() {
        let mut agreement = agreement();
        agreement.parties.push(party("institution-c"));
        agreement.consents.push(consent("institution-c"));

        assert_eq!(
            authorize_cooperation(&agreement, &request("institution-a", "institution-c")),
            CooperationDecisionV1::Denied(DenialReasonV1::NoDirectCooperationEdge)
        );
    }

    #[test]
    fn every_party_must_consent_to_the_exact_terms_digest() {
        let mut agreement = agreement();
        agreement.consents.pop();

        assert_eq!(
            authorize_cooperation(&agreement, &request("institution-a", "institution-b")),
            CooperationDecisionV1::Denied(DenialReasonV1::MissingPartyConsent)
        );
    }

    #[test]
    fn changed_terms_require_fresh_consent() {
        let mut agreement = agreement();
        agreement.terms_digest = [6; 32];

        assert_eq!(
            authorize_cooperation(&agreement, &request("institution-a", "institution-b")),
            CooperationDecisionV1::Denied(DenialReasonV1::InvalidRequest)
        );
    }

    #[test]
    fn withdrawn_party_consent_stops_new_interoperation() {
        let mut agreement = agreement();
        agreement.consents[1].withdrawn_at = Some(9);

        assert_eq!(
            authorize_cooperation(&agreement, &request("institution-a", "institution-b")),
            CooperationDecisionV1::Denied(DenialReasonV1::PartyConsentWithdrawn)
        );
    }

    #[test]
    fn consent_withdrawal_is_not_effective_before_its_timestamp() {
        let mut agreement = agreement();
        agreement.consents[1].withdrawn_at = Some(11);

        assert!(matches!(
            authorize_cooperation(&agreement, &request("institution-a", "institution-b")),
            CooperationDecisionV1::Authorized { .. }
        ));
    }

    #[test]
    fn expired_agreement_is_denied() {
        let mut request = request("institution-a", "institution-b");
        request.at = 100;

        assert_eq!(
            authorize_cooperation(&agreement(), &request),
            CooperationDecisionV1::Denied(DenialReasonV1::AgreementOutsideValidity)
        );
    }

    #[test]
    fn suspended_or_terminated_agreement_is_denied() {
        for status in [AgreementStatusV1::Suspended, AgreementStatusV1::Terminated] {
            let mut agreement = agreement();
            agreement.status = status;
            assert_eq!(
                authorize_cooperation(&agreement, &request("institution-a", "institution-b")),
                CooperationDecisionV1::Denied(DenialReasonV1::AgreementNotActive)
            );
        }
    }

    #[test]
    fn wrong_purpose_jurisdiction_or_disclosure_profile_is_denied() {
        let mut request = request("institution-a", "institution-b");
        request.purpose_id = "unrelated-purpose".to_owned();
        assert_eq!(
            authorize_cooperation(&agreement(), &request),
            CooperationDecisionV1::Denied(DenialReasonV1::ScopeOrPurposeMismatch)
        );

        let mut request = request("institution-a", "institution-b");
        request.jurisdiction_id = "ZA-WC".to_owned();
        assert_eq!(
            authorize_cooperation(&agreement(), &request),
            CooperationDecisionV1::Denied(DenialReasonV1::ScopeOrPurposeMismatch)
        );

        let mut request = request("institution-a", "institution-b");
        request.scope.disclosure_profile.digest = [9; 32];
        assert_eq!(
            authorize_cooperation(&agreement(), &request),
            CooperationDecisionV1::Denied(DenialReasonV1::ScopeOrPurposeMismatch)
        );
    }

    #[test]
    fn malformed_duplicate_edges_fail_closed() {
        let mut agreement = agreement();
        agreement.edges.push(edge("institution-a", "institution-b", "edge-a-b-duplicate"));
        assert_eq!(
            authorize_cooperation(&agreement, &request("institution-a", "institution-b")),
            CooperationDecisionV1::Denied(DenialReasonV1::InvalidRequest)
        );
    }

    #[test]
    fn a_party_cannot_delegate_through_an_unlisted_institution() {
        let mut agreement = agreement();
        agreement.edges.push(edge("institution-a", "institution-c", "edge-a-c"));

        assert_eq!(
            agreement.validate(),
            Err(AgreementErrorV1::EdgePartyNotInAgreement)
        );
    }

    #[test]
    fn local_authority_must_match_each_party_consent() {
        let mut agreement = agreement();
        agreement.consents[0].local_authority.decision_digest = [8; 32];

        assert_eq!(
            agreement.validate(),
            Err(AgreementErrorV1::LocalAuthorityMismatch)
        );
    }
}
