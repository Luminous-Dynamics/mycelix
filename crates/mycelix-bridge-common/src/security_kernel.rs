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

pub const MAX_SECURITY_IDENTIFIER_BYTES: usize = 512;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerificationEvidence {
    pub signature_verified: bool,
    pub not_revoked: bool,
    pub authority_unambiguous: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedCapability {
    capability: Capability,
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

        if subject.is_empty() || issuer.is_empty() || resource.is_empty() {
            return Err("capability identifiers cannot be empty");
        }
        if subject.len() > MAX_SECURITY_IDENTIFIER_BYTES
            || issuer.len() > MAX_SECURITY_IDENTIFIER_BYTES
            || resource.len() > MAX_SECURITY_IDENTIFIER_BYTES
        {
            return Err("capability identifier exceeds size limit");
        }
        if expires_at_us < not_before_us {
            return Err("capability validity window is inverted");
        }
        if actions.is_empty() {
            return Err("capability must grant at least one action");
        }

        Ok(Self {
            subject,
            issuer,
            resource,
            actions,
            not_before_us,
            expires_at_us,
            policy_version,
        })
    }

    pub fn subject(&self) -> &str { &self.subject }
    pub fn issuer(&self) -> &str { &self.issuer }
    pub fn resource(&self) -> &str { &self.resource }
    pub fn policy_version(&self) -> u64 { self.policy_version }
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
        Ok(Self { subject, resource, action, policy_version })
    }
}

/// Cross the independent verification boundary.
///
/// No AI/advisory input is accepted here by design.
pub fn verify_capability(
    capability: Capability,
    evidence: VerificationEvidence,
    now_us: u64,
) -> Result<VerifiedCapability, AuthorizationDecision> {
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
    if now_us < capability.not_before_us || now_us > capability.expires_at_us {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::OutsideValidityWindow,
        ));
    }

    Ok(VerifiedCapability { capability })
}

/// Evaluate authorization from verified credentials and an explicit request.
///
/// This is the only public path that can produce Allow.
pub fn authorize(
    verified: &VerifiedCapability,
    request: &AuthorizationRequest,
    now_us: u64,
) -> AuthorizationDecision {
    let c = &verified.capability;

    if now_us < c.not_before_us || now_us > c.expires_at_us {
        return AuthorizationDecision::Deny(AuthorizationDenial::OutsideValidityWindow);
    }
    if c.subject != request.subject {
        return AuthorizationDecision::Deny(AuthorizationDenial::SubjectMismatch);
    }
    if c.resource != request.resource {
        return AuthorizationDecision::Deny(AuthorizationDenial::ResourceMismatch);
    }
    if !c.actions.contains(&request.action) {
        return AuthorizationDecision::Deny(AuthorizationDenial::ActionNotGranted);
    }
    if c.policy_version != request.policy_version {
        return AuthorizationDecision::Deny(AuthorizationDenial::PolicyVersionMismatch);
    }

    AuthorizationDecision::Allow
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
            VerificationEvidence {
                signature_verified: true,
                not_revoked: true,
                authority_unambiguous: true,
            },
            150,
        )
        .unwrap()
    }

    fn request(action: CapabilityAction) -> AuthorizationRequest {
        AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            action,
            7,
        )
        .unwrap()
    }

    #[test]
    fn valid_capability_allows_granted_action() {
        assert_eq!(
            authorize(&verified(), &request(CapabilityAction::Read), 150),
            AuthorizationDecision::Allow
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
            VerificationEvidence {
                signature_verified: false,
                not_revoked: true,
                authority_unambiguous: true,
            },
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
            VerificationEvidence {
                signature_verified: true,
                not_revoked: false,
                authority_unambiguous: true,
            },
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
            VerificationEvidence {
                signature_verified: true,
                not_revoked: true,
                authority_unambiguous: false,
            },
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
    fn malformed_capability_rejected() {
        assert!(Capability::new(
            "", "issuer", "resource", vec![CapabilityAction::Read], 0, 1, 1
        ).is_err());
        assert!(Capability::new(
            "subject", "issuer", "resource", vec![], 0, 1, 1
        ).is_err());
        assert!(Capability::new(
            "subject", "issuer", "resource", vec![CapabilityAction::Read], 2, 1, 1
        ).is_err());
    }
}
