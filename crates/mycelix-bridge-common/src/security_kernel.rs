// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic security decision kernel.
//!
//! This module deliberately separates cognition from authority:
//! - AdvisoryResult may describe risk, anomalies, contradictions, or
//!   recommended actions, but it cannot be converted into authorization.
//! - VerifiedCapability can only be produced after an independent
//!   verification boundary has established signature validity, revocation
//!   status, and unambiguous authority.
//! - AuthorizationDecision is produced only by policy evaluation over a
//!   verified capability and request.
//!
//! Symthaea may consume or produce advisory values around this kernel. It is
//! not a root of trust and cannot manufacture an authorization decision.

use serde::{Deserialize, Serialize};

#[cfg(feature = "identity")]
use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};

pub const MAX_SECURITY_IDENTIFIER_BYTES: usize = 512;
/// Maximum lifetime of an issued authorization permit, independent of the
/// underlying capability's absolute expiry.
pub const MAX_AUTHORIZATION_PERMIT_LIFETIME_US: u64 = 5 * 60 * 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CapabilityAction {
    Read,
    Write,
    Execute,
    Delegate,
    Admin,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Capability {
    subject: String,
    issuer: String,
    resource: String,
    actions: Vec<CapabilityAction>,
    not_before_us: u64,
    expires_at_us: u64,
    policy_version: u64,
}

/// A capability together with an Ed25519 signature over its canonical semantic
/// representation. This proves integrity of the capability bytes, not issuer
/// authorization, revocation, or policy compliance.
#[cfg(feature = "identity")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SignedCapability {
    pub capability: Capability,
    pub issuer_public_key: [u8; 32],
    pub signature: [u8; 64],
}

#[cfg(feature = "identity")]
impl SignedCapability {
    pub fn sign(capability: Capability, signing_key: &SigningKey) -> Self {
        let signature = signing_key.sign(&capability.signing_bytes());
        Self {
            capability,
            issuer_public_key: signing_key.verifying_key().to_bytes(),
            signature: signature.to_bytes(),
        }
    }

    pub fn verify_signature(&self) -> bool {
        let Ok(key) = VerifyingKey::from_bytes(&self.issuer_public_key) else {
            return false;
        };
        let signature = Signature::from_bytes(&self.signature);
        key.verify(&self.capability.signing_bytes(), &signature)
            .is_ok()
    }

    /// Verify the signature and bind it to an expected issuer key.
    ///
    /// Key possession is not institutional authorization; the caller must
    /// independently establish that this key is authorized for the issuer.
    pub fn verify_signature_from(&self, expected_public_key: [u8; 32]) -> bool {
        self.issuer_public_key == expected_public_key && self.verify_signature()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerificationEvidence {
    // Intentionally private: callers must obtain these propositions from an
    // in-crate verifier boundary rather than constructing trusted evidence
    // from arbitrary booleans.
    signature_verified: bool,
    not_revoked: bool,
    authority_unambiguous: bool,
    /// Exclusive upper bound on how long this verification evidence may authorize.
    valid_until_us: u64,
    /// Stable commitment for the exact capability the evidence verifies.
    capability_binding: [u8; 32],
    /// Opaque commitment supplied by the authority adapter for the exact
    /// authority generation/freshness state that qualified this evidence.
    authority_binding: [u8; 32],
}

impl VerificationEvidence {
    /// Test-only convenience constructor for evidence without a bounded lease.
    /// Production verification paths must use the explicit freshness-lease constructor.
    #[cfg(test)]
    pub(crate) fn new_for_capability(
        capability: &Capability,
        signature_verified: bool,
        not_revoked: bool,
        authority_unambiguous: bool,
    ) -> Self {
        Self::new_for_capability_with_valid_until(
            capability,
            signature_verified,
            not_revoked,
            authority_unambiguous,
            u64::MAX,
        )
    }

    /// Construct trusted evidence with an explicit freshness lease.
    ///
    /// The lease is an upper bound on authorization derived from this evidence;
    /// The lease is exclusive: equality with the current time is already stale.
    /// Enforcement must revalidate it before an external effect.
    pub(crate) fn new_for_capability_with_valid_until(
        capability: &Capability,
        signature_verified: bool,
        not_revoked: bool,
        authority_unambiguous: bool,
        valid_until_us: u64,
    ) -> Self {
        Self::new_for_capability_with_authority_binding_and_valid_until(
            capability,
            [0; 32],
            signature_verified,
            not_revoked,
            authority_unambiguous,
            valid_until_us,
        )
    }

    /// Construct trusted evidence with an explicit capability binding and an
    /// opaque authority-freshness commitment supplied by the authority adapter.
    ///
    /// The commitment must track the authoritative freshness domain.
    pub(crate) fn new_for_capability_with_authority_binding_and_valid_until(
        capability: &Capability,
        authority_binding: [u8; 32],
        signature_verified: bool,
        not_revoked: bool,
        authority_unambiguous: bool,
        valid_until_us: u64,
    ) -> Self {
        Self {
            signature_verified,
            not_revoked,
            authority_unambiguous,
            valid_until_us,
            capability_binding: capability.binding_digest(),
            authority_binding,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedCapability {
    capability: Capability,
    verification_valid_until_us: u64,
    authority_binding: [u8; 32],
}

/// A short-lived, non-serializable authorization permit bound to one exact
/// request. It is deliberately not constructible from advisory output or
/// from a bare Allow value.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorizationPermit {
    request: AuthorizationRequest,
    issued_at_us: u64,
    valid_until_us: u64,
    capability_binding: [u8; 32],
    authority_binding: [u8; 32],
}

/// The only request type accepted by an enforcement adapter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnforcementRequest {
    request: AuthorizationRequest,
    issued_at_us: u64,
    valid_until_us: u64,
    capability_binding: [u8; 32],
    authority_binding: [u8; 32],
}

impl AuthorizationPermit {
    pub fn request(&self) -> &AuthorizationRequest {
        &self.request
    }

    pub fn issued_at_us(&self) -> u64 {
        self.issued_at_us
    }

    pub fn valid_until_us(&self) -> u64 {
        self.valid_until_us
    }

    /// Returns whether this permit is valid at the supplied timestamp.
    ///
    /// Permit validity is inclusive at the exact expiry boundary, matching the
    /// kernel's other validity-window checks.
    pub fn is_valid_at(&self, now_us: u64) -> bool {
        now_us <= self.valid_until_us
    }
}

impl EnforcementRequest {
    /// Construct only from a permit that is still valid and whose independent
    /// verification evidence remains authoritative.
    pub fn from_permit(
        permit: AuthorizationPermit,
        evidence: VerificationEvidence,
        now_us: u64,
    ) -> Result<Self, AuthorizationDecision> {
        match revalidate_permit(&permit, evidence, now_us) {
            AuthorizationDecision::Allow => Ok(Self {
                request: permit.request,
                issued_at_us: permit.issued_at_us,
                valid_until_us: permit.valid_until_us,
                capability_binding: permit.capability_binding,
                authority_binding: permit.authority_binding,
            }),
            decision => Err(decision),
        }
    }

    pub fn request(&self) -> &AuthorizationRequest {
        &self.request
    }

    pub fn issued_at_us(&self) -> u64 {
        self.issued_at_us
    }

    pub fn valid_until_us(&self) -> u64 {
        self.valid_until_us
    }

    /// Stable commitment for the exact capability that authorized this request.
    pub fn capability_binding(&self) -> [u8; 32] {
        self.capability_binding
    }

    /// Opaque commitment for the authority generation/freshness state that
    /// qualified this enforcement request.
    pub fn authority_binding(&self) -> [u8; 32] {
        self.authority_binding
    }

    pub fn is_valid_at(&self, now_us: u64) -> bool {
        now_us <= self.valid_until_us
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizationRequest {
    subject: String,
    resource: String,
    action: CapabilityAction,
    policy_version: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorizationDecision {
    Allow,
    Deny(AuthorizationDenial),
    Indeterminate(AuthorizationIndeterminacy),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorizationDenial {
    InvalidCapability,
    RevokedCapability,
    SubjectMismatch,
    ResourceMismatch,
    ActionNotGranted,
    PolicyVersionMismatch,
    OutsideValidityWindow,
    VerificationEvidenceMismatch,
    AuthorityBindingMismatch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorizationIndeterminacy {
    AmbiguousAuthority,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AdvisoryResult {
    pub advisory_id: String,
    pub risk_signal: f32,
    pub rationale: String,
    pub recommended_action: Option<String>,
}

impl AdvisoryResult {
    pub fn new(
        advisory_id: impl Into<String>,
        risk_signal: f32,
        rationale: impl Into<String>,
        recommended_action: Option<String>,
    ) -> Self {
        Self {
            advisory_id: advisory_id.into(),
            risk_signal: if risk_signal.is_finite() {
                risk_signal.clamp(0.0, 1.0)
            } else {
                1.0
            },
            rationale: rationale.into(),
            recommended_action,
        }
    }
}

impl Capability {
    fn validate(&self) -> Result<(), &'static str> {
        if self.subject.is_empty() || self.issuer.is_empty() || self.resource.is_empty() {
            return Err("capability identifiers cannot be empty");
        }
        if self.subject.len() > MAX_SECURITY_IDENTIFIER_BYTES
            || self.issuer.len() > MAX_SECURITY_IDENTIFIER_BYTES
            || self.resource.len() > MAX_SECURITY_IDENTIFIER_BYTES
        {
            return Err("capability identifier exceeds size limit");
        }
        if self.expires_at_us < self.not_before_us {
            return Err("capability validity window is inverted");
        }
        if self.actions.is_empty() {
            return Err("capability must grant at least one action");
        }
        if self
            .actions
            .windows(2)
            .any(|pair| pair[0] as u8 >= pair[1] as u8)
        {
            return Err("capability actions must be canonical and unique");
        }
        Ok(())
    }

    /// Stable semantic bytes for cryptographic signing.
    ///
    /// Actions are canonicalized at construction, so semantically identical
    /// action sets do not acquire different signatures from vector ordering.
    /// The framing avoids dependence on JSON/map ordering and binds every
    /// authority-relevant capability field.
    pub fn signing_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(256);
        out.extend_from_slice(b"mycelix/security/capability/v1");
        frame_bytes(&mut out, self.subject.as_bytes());
        frame_bytes(&mut out, self.issuer.as_bytes());
        frame_bytes(&mut out, self.resource.as_bytes());
        out.extend_from_slice(&(self.actions.len() as u64).to_le_bytes());
        for action in &self.actions {
            out.push(*action as u8);
        }
        out.extend_from_slice(&self.not_before_us.to_le_bytes());
        out.extend_from_slice(&self.expires_at_us.to_le_bytes());
        out.extend_from_slice(&self.policy_version.to_le_bytes());
        out
    }

    /// Stable commitment used to bind verification evidence and permits to this
    /// exact capability semantics.
    pub(crate) fn binding_digest(&self) -> [u8; 32] {
        *blake3::hash(&self.signing_bytes()).as_bytes()
    }

    pub fn new(
        subject: impl Into<String>,
        issuer: impl Into<String>,
        resource: impl Into<String>,
        actions: Vec<CapabilityAction>,
        not_before_us: u64,
        expires_at_us: u64,
        policy_version: u64,
    ) -> Result<Self, &'static str> {
        let subject = subject.into();
        let issuer = issuer.into();
        let resource = resource.into();

        let mut capability = Self {
            subject,
            issuer,
            resource,
            actions,
            not_before_us,
            expires_at_us,
            policy_version,
        };

        capability.actions.sort_by_key(|action| *action as u8);
        capability.validate()?;
        Ok(capability)
    }

    pub fn subject(&self) -> &str {
        &self.subject
    }
    pub fn issuer(&self) -> &str {
        &self.issuer
    }
    pub fn resource(&self) -> &str {
        &self.resource
    }
    pub fn policy_version(&self) -> u64 {
        self.policy_version
    }
}

impl AuthorizationRequest {
    pub fn new(
        subject: impl Into<String>,
        resource: impl Into<String>,
        action: CapabilityAction,
        policy_version: u64,
    ) -> Result<Self, &'static str> {
        let subject = subject.into();
        let resource = resource.into();
        if subject.is_empty() || resource.is_empty() {
            return Err("authorization identifiers cannot be empty");
        }
        if subject.len() > MAX_SECURITY_IDENTIFIER_BYTES
            || resource.len() > MAX_SECURITY_IDENTIFIER_BYTES
        {
            return Err("authorization identifier exceeds size limit");
        }
        Ok(Self {
            subject,
            resource,
            action,
            policy_version,
        })
    }
}

/// Cross the independent verification boundary.
///
/// No AI/advisory input is accepted here by design.
fn frame_bytes(out: &mut Vec<u8>, value: &[u8]) {
    out.extend_from_slice(&(value.len() as u64).to_le_bytes());
    out.extend_from_slice(value);
}

pub fn verify_capability(
    capability: Capability,
    evidence: VerificationEvidence,
    now_us: u64,
) -> Result<VerifiedCapability, AuthorizationDecision> {
    if capability.validate().is_err() {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::InvalidCapability,
        ));
    }
    if evidence.capability_binding != capability.binding_digest() {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::VerificationEvidenceMismatch,
        ));
    }
    if !evidence.signature_verified {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::InvalidCapability,
        ));
    }
    if !evidence.not_revoked {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::RevokedCapability,
        ));
    }
    if !evidence.authority_unambiguous {
        return Err(AuthorizationDecision::Indeterminate(
            AuthorizationIndeterminacy::AmbiguousAuthority,
        ));
    }
    if now_us < capability.not_before_us
        || now_us > capability.expires_at_us
        || now_us >= evidence.valid_until_us
    {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::OutsideValidityWindow,
        ));
    }

    Ok(VerifiedCapability {
        capability,
        verification_valid_until_us: evidence.valid_until_us,
        authority_binding: evidence.authority_binding,
    })
}

/// Evaluate authorization from verified credentials and an explicit request.
///
/// This is the only public path that can produce Allow.
pub fn authorize(
    verified: &VerifiedCapability,
    request: &AuthorizationRequest,
    now_us: u64,
) -> AuthorizationDecision {
    authorize_permit(verified, request, now_us)
        .map(|_| AuthorizationDecision::Allow)
        .unwrap_or_else(|decision| decision)
}

/// Revalidate a previously issued permit at the enforcement boundary.
///
/// This closes the most important authorization TOCTOU window represented by
/// this kernel: revocation or authority ambiguity discovered after issuance
/// must prevent enforcement. The independent verifier remains responsible for
/// supplying trustworthy evidence.
pub fn revalidate_permit(
    permit: &AuthorizationPermit,
    evidence: VerificationEvidence,
    now_us: u64,
) -> AuthorizationDecision {
    if !evidence.signature_verified {
        return AuthorizationDecision::Deny(AuthorizationDenial::InvalidCapability);
    }
    if evidence.capability_binding != permit.capability_binding {
        return AuthorizationDecision::Deny(AuthorizationDenial::VerificationEvidenceMismatch);
    }
    if evidence.authority_binding != permit.authority_binding {
        return AuthorizationDecision::Deny(AuthorizationDenial::AuthorityBindingMismatch);
    }
    if !evidence.not_revoked {
        return AuthorizationDecision::Deny(AuthorizationDenial::RevokedCapability);
    }
    if !evidence.authority_unambiguous {
        return AuthorizationDecision::Indeterminate(
            AuthorizationIndeterminacy::AmbiguousAuthority,
        );
    }
    if now_us >= evidence.valid_until_us || !permit.is_valid_at(now_us) {
        return AuthorizationDecision::Deny(AuthorizationDenial::OutsideValidityWindow);
    }
    AuthorizationDecision::Allow
}

/// Evaluate authorization and, on success, mint a non-forgeable-in-module
/// permit bound to the exact request that was checked.
pub fn authorize_permit(
    verified: &VerifiedCapability,
    request: &AuthorizationRequest,
    now_us: u64,
) -> Result<AuthorizationPermit, AuthorizationDecision> {
    let c = &verified.capability;

    if now_us < c.not_before_us
        || now_us > c.expires_at_us
        || now_us >= verified.verification_valid_until_us
    {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::OutsideValidityWindow,
        ));
    }
    if c.subject != request.subject {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::SubjectMismatch,
        ));
    }
    if c.resource != request.resource {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::ResourceMismatch,
        ));
    }
    if !c.actions.contains(&request.action) {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::ActionNotGranted,
        ));
    }
    if c.policy_version != request.policy_version {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::PolicyVersionMismatch,
        ));
    }

    let permit_lifetime_us = now_us.saturating_add(MAX_AUTHORIZATION_PERMIT_LIFETIME_US);
    let valid_until_us = c
        .expires_at_us
        .min(verified.verification_valid_until_us)
        .min(permit_lifetime_us);

    Ok(AuthorizationPermit {
        request: request.clone(),
        issued_at_us: now_us,
        valid_until_us,
        capability_binding: c.binding_digest(),
        authority_binding: verified.authority_binding,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn capability() -> Capability {
        Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:ledger",
            vec![CapabilityAction::Read],
            100,
            200,
            7,
        )
        .unwrap()
    }

    fn verified() -> VerifiedCapability {
        verify_capability(
            capability(),
            VerificationEvidence::new_for_capability(&capability(), true, true, true),
            150,
        )
        .unwrap()
    }

    fn request(action: CapabilityAction) -> AuthorizationRequest {
        AuthorizationRequest::new("did:mycelix:alice", "resource:ledger", action, 7).unwrap()
    }

    #[test]
    fn capability_action_order_is_canonical_and_duplicates_rejected() {
        let first = Capability::new(
            "alice",
            "issuer",
            "ledger",
            vec![CapabilityAction::Admin, CapabilityAction::Read],
            1,
            2,
            3,
        )
        .unwrap();
        let second = Capability::new(
            "alice",
            "issuer",
            "ledger",
            vec![CapabilityAction::Read, CapabilityAction::Admin],
            1,
            2,
            3,
        )
        .unwrap();
        assert_eq!(first.signing_bytes(), second.signing_bytes());
        assert!(
            Capability::new(
                "alice",
                "issuer",
                "ledger",
                vec![CapabilityAction::Read, CapabilityAction::Read],
                1,
                2,
                3,
            )
            .is_err()
        );
    }

    #[cfg(feature = "identity")]
    #[test]
    fn signed_capability_requires_expected_issuer_key() {
        use ed25519_dalek::SigningKey;

        let key = SigningKey::from_bytes(&[7u8; 32]);
        let signed = SignedCapability::sign(capability(), &key);
        assert!(signed.verify_signature_from(key.verifying_key().to_bytes()));
        assert!(!signed.verify_signature_from([8u8; 32]));
    }

    #[cfg(feature = "identity")]
    #[test]
    fn signed_capability_cryptographically_verifies() {
        use ed25519_dalek::SigningKey;

        let capability = Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:ledger",
            vec![CapabilityAction::Read],
            100,
            200,
            7,
        )
        .unwrap();
        let key = SigningKey::from_bytes(&[7u8; 32]);
        let signed = SignedCapability::sign(capability, &key);
        assert!(signed.verify_signature());

        let mut tampered = signed.clone();
        tampered.capability = Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:ledger",
            vec![CapabilityAction::Admin],
            100,
            200,
            7,
        )
        .unwrap();
        assert!(!tampered.verify_signature());
    }

    #[test]
    fn evidence_for_another_capability_cannot_verify_capability() {
        let capability = capability();
        let other = Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:other",
            vec![CapabilityAction::Read],
            100,
            200,
            7,
        )
        .unwrap();
        let result = verify_capability(
            capability,
            VerificationEvidence::new_for_capability(&other, true, true, true),
            150,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::VerificationEvidenceMismatch
            ))
        );
    }

    #[test]
    fn valid_capability_allows_granted_action() {
        assert_eq!(
            authorize(&verified(), &request(CapabilityAction::Read), 150),
            AuthorizationDecision::Allow
        );
    }

    #[test]
    fn allow_mints_exactly_bound_enforcement_request() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();
        let enforcement = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(&capability(), true, true, true),
            150,
        )
        .unwrap();
        assert_eq!(enforcement.request(), &request(CapabilityAction::Read));
        assert_eq!(enforcement.issued_at_us(), 150);
        assert_eq!(enforcement.valid_until_us(), 200);
        assert!(enforcement.is_valid_at(200));
        assert!(!enforcement.is_valid_at(201));
    }

    #[test]
    fn verification_lease_expires_at_exact_boundary() {
        let cap = capability();
        assert_eq!(
            verify_capability(
                cap.clone(),
                VerificationEvidence::new_for_capability_with_valid_until(
                    &cap, true, true, true, 175
                ),
                175,
            )
            .unwrap_err(),
            AuthorizationDecision::Deny(AuthorizationDenial::OutsideValidityWindow)
        );
    }

    #[test]
    fn permit_lifetime_is_bounded_by_verification_freshness() {
        let cap = capability();
        let verified = verify_capability(
            cap.clone(),
            VerificationEvidence::new_for_capability_with_valid_until(&cap, true, true, true, 175),
            150,
        )
        .unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        assert_eq!(permit.valid_until_us(), 175);
        assert!(!permit.is_valid_at(176));
    }

    #[test]
    fn permit_lifetime_is_capped_independently_of_capability_expiry() {
        let long_lived = Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:ledger",
            vec![CapabilityAction::Read],
            100,
            u64::MAX,
            7,
        )
        .unwrap();
        let verified = verify_capability(
            long_lived.clone(),
            VerificationEvidence::new_for_capability(&long_lived, true, true, true),
            150,
        )
        .unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        assert_eq!(
            permit.valid_until_us(),
            150 + MAX_AUTHORIZATION_PERMIT_LIFETIME_US
        );
        assert!(!permit.is_valid_at(150 + MAX_AUTHORIZATION_PERMIT_LIFETIME_US + 1));
    }

    #[test]
    fn invalid_fresh_signature_evidence_blocks_enforcement() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();
        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(&capability(), false, true, true),
            151,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::InvalidCapability
            ))
        );
    }

    #[test]
    fn evidence_for_another_capability_cannot_revalidate_permit() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();
        let other = Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:other",
            vec![CapabilityAction::Read],
            100,
            200,
            7,
        )
        .unwrap();
        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(&other, true, true, true),
            151,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::VerificationEvidenceMismatch
            ))
        );
    }

    #[test]
    fn enforcement_rejects_evidence_at_exact_lease_boundary() {
        let cap = capability();
        let verified = verify_capability(
            cap.clone(),
            VerificationEvidence::new_for_capability_with_valid_until(&cap, true, true, true, 175),
            150,
        )
        .unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability_with_valid_until(&cap, true, true, true, 175),
            175,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow
            ))
        );
    }

    #[test]
    fn authorization_rejects_at_exact_verification_lease_boundary() {
        let cap = capability();
        let evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap, [4; 32], true, true, true, 175,
            );
        let verified = verify_capability(cap, evidence, 150).unwrap();
        assert_eq!(
            authorize_permit(&verified, &request(CapabilityAction::Read), 175),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow
            ))
        );
    }

    #[test]
    fn authority_generation_binding_blocks_revalidation_with_new_generation() {
        let cap = capability();
        let evidence_a =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap, [1; 32], true, true, true, 200,
            );
        let verified = verify_capability(cap.clone(), evidence_a, 150).unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();

        let evidence_b =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap, [2; 32], true, true, true, 200,
            );
        assert_eq!(
            revalidate_permit(&permit, evidence_b, 151),
            AuthorizationDecision::Deny(AuthorizationDenial::AuthorityBindingMismatch)
        );
    }

    #[test]
    fn authority_generation_binding_is_preserved_at_enforcement() {
        let cap = capability();
        let authority = [9; 32];
        let evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap, authority, true, true, true, 200,
            );
        let verified = verify_capability(cap.clone(), evidence, 150).unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        let enforcement = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap, authority, true, true, true, 200,
            ),
            151,
        )
        .unwrap();
        assert_eq!(enforcement.authority_binding(), authority);
    }

    #[test]
    fn revocation_after_authorization_blocks_enforcement() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();
        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(&capability(), true, false, true),
            151,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::RevokedCapability
            ))
        );
    }

    #[test]
    fn deny_mints_no_enforcement_request() {
        let result = authorize_permit(&verified(), &request(CapabilityAction::Admin), 150);
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::ActionNotGranted,
            ))
        );
    }

    #[test]
    fn advisory_cannot_authorize() {
        let advisory = AdvisoryResult::new(
            "symthaea-1",
            0.99,
            "high confidence that read is safe",
            Some("allow".into()),
        );
        assert_eq!(advisory.risk_signal, 0.99);
        // Deliberately no API from AdvisoryResult to AuthorizationDecision.
    }

    #[test]
    fn forged_signature_is_denied() {
        let result = verify_capability(
            capability(),
            VerificationEvidence::new_for_capability(&capability(), false, true, true),
            150,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::InvalidCapability
            ))
        );
    }

    #[test]
    fn revoked_capability_is_denied() {
        let result = verify_capability(
            capability(),
            VerificationEvidence::new_for_capability(&capability(), true, false, true),
            150,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::RevokedCapability
            ))
        );
    }

    #[test]
    fn ambiguous_authority_is_indeterminate_not_allow() {
        let result = verify_capability(
            capability(),
            VerificationEvidence::new_for_capability(&capability(), true, true, false),
            150,
        );
        assert_eq!(
            result,
            Err(AuthorizationDecision::Indeterminate(
                AuthorizationIndeterminacy::AmbiguousAuthority
            ))
        );
    }

    #[test]
    fn subject_mismatch_is_denied() {
        let req = AuthorizationRequest::new(
            "did:mycelix:bob",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();
        assert_eq!(
            authorize(&verified(), &req, 150),
            AuthorizationDecision::Deny(AuthorizationDenial::SubjectMismatch)
        );
    }

    #[test]
    fn ungranted_action_is_denied() {
        assert_eq!(
            authorize(&verified(), &request(CapabilityAction::Admin), 150),
            AuthorizationDecision::Deny(AuthorizationDenial::ActionNotGranted)
        );
    }

    #[test]
    fn stale_policy_is_denied() {
        let req = AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            8,
        )
        .unwrap();
        assert_eq!(
            authorize(&verified(), &req, 150),
            AuthorizationDecision::Deny(AuthorizationDenial::PolicyVersionMismatch)
        );
    }

    #[test]
    fn expired_capability_is_denied() {
        assert_eq!(
            authorize(&verified(), &request(CapabilityAction::Read), 201),
            AuthorizationDecision::Deny(AuthorizationDenial::OutsideValidityWindow)
        );
    }

    #[test]
    fn deserialized_malformed_capability_is_rejected_before_authorization() {
        let malformed: Capability = serde_json::from_str(
            r#"{
                "subject":"alice",
                "issuer":"issuer",
                "resource":"ledger",
                "actions":["Read","Read"],
                "not_before_us":1,
                "expires_at_us":2,
                "policy_version":1
            }"#,
        )
        .unwrap();
        let evidence = VerificationEvidence::new_for_capability(&malformed, true, true, true);
        assert_eq!(
            verify_capability(malformed, evidence, 1),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::InvalidCapability,
            ))
        );
    }

    #[test]
    fn malformed_capability_rejected() {
        assert!(
            Capability::new(
                "",
                "issuer",
                "resource",
                vec![CapabilityAction::Read],
                0,
                1,
                1,
            )
            .is_err()
        );
        assert!(Capability::new("subject", "issuer", "resource", vec![], 0, 1, 1).is_err());
        assert!(
            Capability::new(
                "subject",
                "issuer",
                "resource",
                vec![CapabilityAction::Read],
                2,
                1,
                1,
            )
            .is_err()
        );
    }
}
