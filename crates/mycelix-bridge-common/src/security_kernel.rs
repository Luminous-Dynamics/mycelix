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

use serde::{Deserialize, Serialize, de::Error as _, de::SeqAccess, de::Visitor};
use std::io::{self, Read};

#[cfg(feature = "identity")]
use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};

pub const MAX_SECURITY_IDENTIFIER_BYTES: usize = 512;
/// Maximum byte length accepted by the pre-deserialization security JSON envelope helper.
///
/// This is a parser/input-envelope bound, distinct from per-field retention bounds.
pub const MAX_SECURITY_WIRE_BYTES: usize = 512 * 1024;
/// Maximum number of actions a capability can contain.
pub const MAX_CAPABILITY_ACTIONS: usize = 5;
/// Maximum lifetime of an issued authorization permit, independent of the
/// underlying capability's absolute expiry.
pub const MAX_AUTHORIZATION_PERMIT_LIFETIME_US: u64 = 5 * 60 * 1_000_000;

/// Deserialize a security-domain JSON payload only after enforcing an outer input-size bound.
///
/// Field-level security identifiers are still bounded independently by their wire visitors.
/// This helper closes the separate parser/input-envelope gap where a JSON parser may need to
/// materialize escaped or otherwise attacker-controlled input before those visitors run.
pub fn deserialize_bounded_security_json<T>(input: &[u8]) -> Result<T, serde_json::Error>
where
    T: serde::de::DeserializeOwned,
{
    if input.len() > MAX_SECURITY_WIRE_BYTES {
        return Err(serde_json::Error::io(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "security JSON input exceeds size limit",
        )));
    }
    serde_json::from_slice(input)
}

/// Reader wrapper that prevents a security-domain JSON parser from consuming more than the
/// configured outer input envelope. The extra-byte probe distinguishes exact-limit EOF from
/// an oversized stream without buffering the whole source.
struct BoundedSecurityReader<R> {
    inner: R,
    remaining: usize,
}

impl<R> BoundedSecurityReader<R> {
    fn new(inner: R) -> Self {
        Self {
            inner,
            remaining: MAX_SECURITY_WIRE_BYTES,
        }
    }
}

impl<R: Read> Read for BoundedSecurityReader<R> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }

        if self.remaining == 0 {
            let mut probe = [0u8; 1];
            return match self.inner.read(&mut probe) {
                Ok(0) => Ok(0),
                Ok(_) => Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "security JSON input exceeds size limit",
                )),
                Err(error) => Err(error),
            };
        }

        let allowed = buf.len().min(self.remaining);
        let read = self.inner.read(&mut buf[..allowed])?;
        self.remaining -= read;
        Ok(read)
    }
}

/// Deserialize a security-domain JSON payload from a reader only after enforcing the same
/// outer input-size bound used by the slice helper.
pub fn deserialize_bounded_security_json_reader<R, T>(reader: R) -> Result<T, serde_json::Error>
where
    R: Read,
    T: serde::de::DeserializeOwned,
{
    serde_json::from_reader(BoundedSecurityReader::new(reader))
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CapabilityAction {
    Read = 1,
    Write = 2,
    Execute = 3,
    Delegate = 4,
    Admin = 5,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "CapabilityWire")]
pub struct Capability {
    subject: String,
    issuer: String,
    resource: String,
    actions: Vec<CapabilityAction>,
    not_before_us: u64,
    expires_at_us: u64,
    policy_version: u64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CapabilityWire {
    #[serde(deserialize_with = "deserialize_security_identifier")]
    subject: String,
    #[serde(deserialize_with = "deserialize_security_identifier")]
    issuer: String,
    #[serde(deserialize_with = "deserialize_security_identifier")]
    resource: String,
    #[serde(deserialize_with = "deserialize_capability_actions")]
    actions: Vec<CapabilityAction>,
    not_before_us: u64,
    expires_at_us: u64,
    policy_version: u64,
}

impl TryFrom<CapabilityWire> for Capability {
    type Error = &'static str;

    fn try_from(wire: CapabilityWire) -> Result<Self, Self::Error> {
        Self::new(
            wire.subject,
            wire.issuer,
            wire.resource,
            wire.actions,
            wire.not_before_us,
            wire.expires_at_us,
            wire.policy_version,
        )
    }
}

/// Bound security identifiers before retaining attacker-controlled wire strings.
fn deserialize_security_identifier<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    struct SecurityIdentifierVisitor;

    impl<'de> Visitor<'de> for SecurityIdentifierVisitor {
        type Value = String;

        fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            formatter.write_str("a bounded security identifier string")
        }

        fn visit_borrowed_str<E>(self, value: &'de str) -> Result<Self::Value, E>
        where
            E: serde::de::Error,
        {
            if value.len() > MAX_SECURITY_IDENTIFIER_BYTES {
                return Err(E::custom("security identifier exceeds size limit"));
            }
            Ok(value.to_owned())
        }

        fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
        where
            E: serde::de::Error,
        {
            if value.len() > MAX_SECURITY_IDENTIFIER_BYTES {
                return Err(E::custom("security identifier exceeds size limit"));
            }
            Ok(value.to_owned())
        }

        fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
        where
            E: serde::de::Error,
        {
            if value.len() > MAX_SECURITY_IDENTIFIER_BYTES {
                return Err(E::custom("security identifier exceeds size limit"));
            }
            Ok(value)
        }
    }

    deserializer.deserialize_str(SecurityIdentifierVisitor)
}

/// Bound capability action sequences before retaining an attacker-controlled
/// number of enum values from wire input.
fn deserialize_capability_actions<'de, D>(
    deserializer: D,
) -> Result<Vec<CapabilityAction>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    struct CapabilityActionsVisitor;

    impl<'de> Visitor<'de> for CapabilityActionsVisitor {
        type Value = Vec<CapabilityAction>;

        fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            formatter.write_str("a bounded capability action sequence")
        }

        fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
        where
            A: SeqAccess<'de>,
        {
            let mut actions = Vec::with_capacity(MAX_CAPABILITY_ACTIONS);
            while let Some(action) = seq.next_element()? {
                if actions.len() >= MAX_CAPABILITY_ACTIONS {
                    return Err(A::Error::custom(
                        "capability action sequence exceeds size limit",
                    ));
                }
                actions.push(action);
            }
            Ok(actions)
        }
    }

    deserializer.deserialize_seq(CapabilityActionsVisitor)
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
enum SignatureVerification {
    Verified,
    Invalid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RevocationStatus {
    Current,
    Revoked,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AuthorityResolution {
    Unambiguous,
    Ambiguous,
}

#[derive(Clone)]
pub struct VerificationEvidence {
    // Deliberately non-Copy: trusted verification evidence should not be implicitly duplicated.
    // Intentionally no Debug/PartialEq/Eq: downstream code should treat verification evidence
    // as an opaque trust hand-off rather than a value to inspect or compare outside the kernel.
    // Reuse across independent checks must be explicit (`Clone`) so evidence flow remains visible.
    // Intentionally private: callers must obtain these propositions from an
    // in-crate verifier boundary rather than constructing trusted evidence
    // from arbitrary booleans.
    signature: SignatureVerification,
    revocation: RevocationStatus,
    authority: AuthorityResolution,
    /// Exclusive upper bound on how long this verification evidence may authorize.
    valid_until_us: u64,
    /// Stable commitment for the exact capability the evidence verifies.
    capability_binding: [u8; 32],
    /// Opaque commitment supplied by the authority adapter for the exact
    /// authority generation/freshness state that qualified this evidence.
    authority_binding: [u8; 32],
}

/// Derive the bridge authority binding from the canonical current-freshness commitment.
///
/// The freshness kernel's stable digest excludes verifier timestamps and lease horizons,
/// so this binding remains stable across proof/lease refreshes that do not change the
/// semantic authority domain. The protocol/profile are committed as well, preventing a
/// digest from being interpreted under a different canonical freshness scheme.
const AUTHORITY_FRESHNESS_PROTOCOL_VERSION: &str = "mycelix-authority-freshness-v0.1";
const AUTHORITY_FRESHNESS_PROFILE: &str = "mycelix-authority-freshness-bundle-v1-blake3-framed";

/// Derive the bridge authority binding from the canonical current-freshness commitment.
///
/// The freshness protocol/profile are fixed by this bridge contract rather than supplied
/// by the caller. This prevents an adapter from accidentally interpreting the same digest
/// under a different freshness identity scheme. Dynamic proof/lease metadata remains outside
/// the binding.
fn authority_binding_from_freshness_digest(freshness_digest: [u8; 32]) -> [u8; 32] {
    // Preserve the zero sentinel used by the kernel to represent missing
    // authority freshness evidence; do not hash it into a seemingly valid
    // non-zero binding.
    if freshness_digest == [0; 32] {
        return [0; 32];
    }

    let mut hasher = blake3::Hasher::new();
    hasher.update(b"mycelix/security/authority-binding/v1");
    frame_hash_bytes(&mut hasher, AUTHORITY_FRESHNESS_PROTOCOL_VERSION.as_bytes());
    frame_hash_bytes(&mut hasher, AUTHORITY_FRESHNESS_PROFILE.as_bytes());
    frame_hash_bytes(&mut hasher, &freshness_digest);
    *hasher.finalize().as_bytes()
}

impl VerificationEvidence {
    /// Test-only convenience constructor for evidence without a bounded lease.
    /// Production verification paths must use the explicit freshness-lease constructor.
    #[cfg(test)]
    fn new_for_capability(
        capability: &Capability,
        signature: SignatureVerification,
        revocation: RevocationStatus,
        authority: AuthorityResolution,
    ) -> Self {
        Self::new_for_capability_with_authority_binding_and_valid_until(
            capability,
            [0xA5; 32],
            signature,
            revocation,
            authority,
            u64::MAX,
        )
    }

    /// Test-only convenience constructor with a synthetic canonical freshness digest.
    ///
    /// Production verification must use `new_for_capability_with_freshness_digest` so
    /// authority evidence cannot be represented by a fixed placeholder binding.
    #[cfg(test)]
    fn new_for_capability_with_valid_until(
        capability: &Capability,
        signature: SignatureVerification,
        revocation: RevocationStatus,
        authority: AuthorityResolution,
        valid_until_us: u64,
    ) -> Self {
        Self::new_for_capability_with_freshness_digest(
            capability,
            [0xA5; 32],
            signature,
            revocation,
            authority,
            valid_until_us,
        )
    }

    /// Construct trusted evidence from the canonical current-freshness digest.
    ///
    /// Production callers cannot supply an arbitrary bridge authority binding;
    /// the bridge derives that binding from the canonical freshness commitment.
    fn new_for_capability_with_freshness_digest(
        capability: &Capability,
        freshness_digest: [u8; 32],
        signature: SignatureVerification,
        revocation: RevocationStatus,
        authority: AuthorityResolution,
        valid_until_us: u64,
    ) -> Self {
        Self {
            signature,
            revocation,
            authority,
            valid_until_us,
            capability_binding: capability.binding_digest(),
            authority_binding: authority_binding_from_freshness_digest(freshness_digest),
        }
    }

    /// Test-only constructor for exercising authority-domain mismatch paths.
    #[cfg(test)]
    fn new_for_capability_with_authority_binding_and_valid_until(
        capability: &Capability,
        authority_binding: [u8; 32],
        signature: SignatureVerification,
        revocation: RevocationStatus,
        authority: AuthorityResolution,
        valid_until_us: u64,
    ) -> Self {
        Self {
            signature,
            revocation,
            authority,
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
/// from a bare Allow value. The authorization hand-off is intentionally non-duplicable: the permit is
/// consumed when creating an EnforcementRequest, so callers cannot replay the
/// same in-memory permit by cloning it. This does not claim that an application
/// cannot intentionally reuse an already-revalidated request before its expiry;
/// effect-specific idempotency belongs to the enforcement adapter.
#[must_use = "authorization permits must be consumed by EnforcementRequest::from_permit"]
pub struct AuthorizationPermit {
    request: AuthorizationRequest,
    issued_at_us: u64,
    valid_until_us: u64,
    capability_binding: [u8; 32],
    authority_binding: [u8; 32],
}

impl AuthorizationPermit {
    fn is_valid_at(&self, now_us: u64) -> bool {
        self.issued_at_us <= now_us && now_us < self.valid_until_us
    }
}

/// The only request type accepted by an enforcement adapter.
///
/// This type is intentionally not Clone: a successfully revalidated
/// enforcement request is a non-duplicable authorization hand-off. Applications
/// that need durable audit data should copy the contained non-authoritative fields
/// into a SecurityEvent instead of duplicating the enforcement capability. Whether
/// the underlying effect is idempotent or single-use remains an enforcement-layer
/// property.
#[must_use = "enforcement requests are the only effect-bound authorization hand-off"]
pub struct EnforcementRequest {
    request: AuthorizationRequest,
    issued_at_us: u64,
    revalidated_at_us: u64,
    valid_until_us: u64,
    capability_binding: [u8; 32],
    authority_binding: [u8; 32],
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
            AuthorizationOutcome::Allow => Ok(Self {
                request: permit.request,
                issued_at_us: permit.issued_at_us,
                revalidated_at_us: now_us,
                valid_until_us: permit.valid_until_us,
                capability_binding: permit.capability_binding,
                authority_binding: permit.authority_binding,
            }),
            AuthorizationOutcome::Deny(reason) => Err(AuthorizationDecision::Deny(reason)),
            AuthorizationOutcome::Indeterminate(reason) => {
                Err(AuthorizationDecision::Indeterminate(reason))
            }
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

    /// Timestamp at which permit revalidation authorized this enforcement request.
    pub fn revalidated_at_us(&self) -> u64 {
        self.revalidated_at_us
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
        self.issued_at_us <= now_us && now_us < self.valid_until_us
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "AuthorizationRequestWire")]
pub struct AuthorizationRequest {
    subject: String,
    resource: String,
    action: CapabilityAction,
    policy_version: u64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AuthorizationRequestWire {
    #[serde(deserialize_with = "deserialize_security_identifier")]
    subject: String,
    #[serde(deserialize_with = "deserialize_security_identifier")]
    resource: String,
    action: CapabilityAction,
    policy_version: u64,
}

impl TryFrom<AuthorizationRequestWire> for AuthorizationRequest {
    type Error = &'static str;

    fn try_from(wire: AuthorizationRequestWire) -> Result<Self, Self::Error> {
        Self::new(
            wire.subject,
            wire.resource,
            wire.action,
            wire.policy_version,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AuthorizationDecision {
    Deny(AuthorizationDenial),
    Indeterminate(AuthorizationIndeterminacy),
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum AuthorizationOutcome {
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
        if self.expires_at_us <= self.not_before_us {
            return Err("capability validity window must be non-empty");
        }
        if self.actions.is_empty() {
            return Err("capability must grant at least one action");
        }
        if self.actions.len() > MAX_CAPABILITY_ACTIONS {
            return Err("capability action sequence exceeds size limit");
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
    fn binding_digest(&self) -> [u8; 32] {
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

#[cfg(test)]
/// Test the checked conversion used by the future authority adapter boundary.
///
/// The authority stack uses milliseconds while the bridge kernel uses microseconds.
/// Overflow is invalid and must be handled as missing/invalid evidence.
fn authority_lease_until_us(lease_until_ms: u64) -> Option<u64> {
    lease_until_ms.checked_mul(1_000)
}

#[cfg(test)]
pub(crate) fn test_enforcement_request() -> EnforcementRequest {
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
    let evidence = VerificationEvidence::new_for_capability(
        &capability,
        SignatureVerification::Verified,
        RevocationStatus::Current,
        AuthorityResolution::Unambiguous,
    );
    let verified = verify_capability(capability.clone(), evidence, 150).unwrap();
    let request = AuthorizationRequest::new(
        "did:mycelix:alice",
        "resource:ledger",
        CapabilityAction::Read,
        7,
    )
    .unwrap();
    let permit = authorize_permit(&verified, &request, 150).unwrap();
    let enforcement_evidence = VerificationEvidence::new_for_capability(
        &capability,
        SignatureVerification::Verified,
        RevocationStatus::Current,
        AuthorityResolution::Unambiguous,
    );
    EnforcementRequest::from_permit(permit, enforcement_evidence, 150).unwrap()
}

fn frame_hash_bytes(hasher: &mut blake3::Hasher, value: &[u8]) {
    hasher.update(&(value.len() as u64).to_le_bytes());
    hasher.update(value);
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
    if evidence.signature != SignatureVerification::Verified {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::InvalidCapability,
        ));
    }
    if evidence.revocation != RevocationStatus::Current {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::RevokedCapability,
        ));
    }
    if evidence.authority != AuthorityResolution::Unambiguous {
        return Err(AuthorizationDecision::Indeterminate(
            AuthorizationIndeterminacy::AmbiguousAuthority,
        ));
    }
    if evidence.authority_binding == [0; 32] {
        return Err(AuthorizationDecision::Indeterminate(
            AuthorizationIndeterminacy::AmbiguousAuthority,
        ));
    }
    if now_us < capability.not_before_us
        || now_us >= capability.expires_at_us
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

/// Revalidate a previously issued permit at the enforcement boundary.
///
/// This closes the most important authorization TOCTOU window represented by
/// this kernel: revocation or authority ambiguity discovered after issuance
/// must prevent enforcement. The independent verifier remains responsible for
/// supplying trustworthy evidence.
fn revalidate_permit(
    permit: &AuthorizationPermit,
    evidence: VerificationEvidence,
    now_us: u64,
) -> AuthorizationOutcome {
    if evidence.signature != SignatureVerification::Verified {
        return AuthorizationOutcome::Deny(AuthorizationDenial::InvalidCapability);
    }
    if evidence.capability_binding != permit.capability_binding {
        return AuthorizationOutcome::Deny(AuthorizationDenial::VerificationEvidenceMismatch);
    }
    if evidence.authority_binding == [0; 32] {
        return AuthorizationOutcome::Indeterminate(
            AuthorizationIndeterminacy::AmbiguousAuthority,
        );
    }
    if evidence.authority_binding != permit.authority_binding {
        return AuthorizationOutcome::Deny(AuthorizationDenial::AuthorityBindingMismatch);
    }
    if evidence.revocation != RevocationStatus::Current {
        return AuthorizationOutcome::Deny(AuthorizationDenial::RevokedCapability);
    }
    if evidence.authority != AuthorityResolution::Unambiguous {
        return AuthorizationOutcome::Indeterminate(
            AuthorizationIndeterminacy::AmbiguousAuthority,
        );
    }
    if now_us >= evidence.valid_until_us || !permit.is_valid_at(now_us) {
        return AuthorizationOutcome::Deny(AuthorizationDenial::OutsideValidityWindow);
    }
    AuthorizationOutcome::Allow
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
        || now_us >= c.expires_at_us
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

    let Some(permit_lifetime_us) = now_us.checked_add(MAX_AUTHORIZATION_PERMIT_LIFETIME_US) else {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::OutsideValidityWindow,
        ));
    };
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
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
            150,
        )
        .unwrap()
    }

    fn request(action: CapabilityAction) -> AuthorizationRequest {
        AuthorizationRequest::new("did:mycelix:alice", "resource:ledger", action, 7).unwrap()
    }

    #[test]
    fn capability_action_discriminants_are_stable() {
        assert_eq!(CapabilityAction::Read as u8, 1);
        assert_eq!(CapabilityAction::Write as u8, 2);
        assert_eq!(CapabilityAction::Execute as u8, 3);
        assert_eq!(CapabilityAction::Delegate as u8, 4);
        assert_eq!(CapabilityAction::Admin as u8, 5);
    }

    #[test]
    fn capability_deserialization_is_constructor_gated() {
        let valid = serde_json::json!({
            "subject": "did:mycelix:alice",
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read"],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7
        });
        let decoded: Capability = serde_json::from_value(valid).unwrap();
        assert_eq!(decoded, capability());

        let empty_subject = serde_json::json!({
            "subject": "",
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read"],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7
        });
        assert!(serde_json::from_value::<Capability>(empty_subject).is_err());

        let empty_actions = serde_json::json!({
            "subject": "did:mycelix:alice",
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": [],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7
        });
        assert!(serde_json::from_value::<Capability>(empty_actions).is_err());

        let oversized_subject = serde_json::json!({
            "subject": "x".repeat(MAX_SECURITY_IDENTIFIER_BYTES + 1),
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read"],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7
        });
        assert!(serde_json::from_value::<Capability>(oversized_subject).is_err());

        let duplicate_actions = serde_json::json!({
            "subject": "did:mycelix:alice",
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read", "Read"],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7
        });
        assert!(serde_json::from_value::<Capability>(duplicate_actions).is_err());

        let invalid_window = serde_json::json!({
            "subject": "did:mycelix:alice",
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read"],
            "not_before_us": 200,
            "expires_at_us": 200,
            "policy_version": 7
        });
        assert!(serde_json::from_value::<Capability>(invalid_window).is_err());

        let too_many_actions = serde_json::json!({
            "subject": "did:mycelix:alice",
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read", "Write", "Execute", "Delegate", "Admin", "Read"],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7
        });
        assert!(serde_json::from_value::<Capability>(too_many_actions).is_err());

        let unknown_field = serde_json::json!({
            "subject": "did:mycelix:alice",
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read"],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7,
            "authority": "ignored-by-old-parser"
        });
        assert!(serde_json::from_value::<Capability>(unknown_field).is_err());
    }

    #[test]
    fn capability_wire_identifier_exact_limit_is_accepted() {
        let boundary = "x".repeat(MAX_SECURITY_IDENTIFIER_BYTES);
        let json = serde_json::json!({
            "subject": boundary,
            "issuer": "did:mycelix:issuer",
            "resource": "resource:ledger",
            "actions": ["Read"],
            "not_before_us": 100,
            "expires_at_us": 200,
            "policy_version": 7
        });

        let decoded: Capability = serde_json::from_value(json).unwrap();
        assert_eq!(decoded.subject().len(), MAX_SECURITY_IDENTIFIER_BYTES);
    }

    #[test]
    fn capability_constructor_rejects_excessive_action_count() {
        let actions = vec![
            CapabilityAction::Read,
            CapabilityAction::Write,
            CapabilityAction::Execute,
            CapabilityAction::Delegate,
            CapabilityAction::Admin,
            CapabilityAction::Read,
        ];

        assert!(Capability::new("alice", "issuer", "ledger", actions, 1, 2, 3).is_err());
    }

    #[test]
    fn bounded_security_json_accepts_exact_envelope_limit() {
        let mut input = serde_json::to_vec(&serde_json::json!({
            "subject": "did:mycelix:alice",
            "resource": "resource:ledger",
            "action": "Read",
            "policy_version": 7
        }))
        .unwrap();
        input.resize(MAX_SECURITY_WIRE_BYTES, b' ');

        let decoded: AuthorizationRequest = deserialize_bounded_security_json(&input).unwrap();
        assert_eq!(decoded.subject(), "did:mycelix:alice");
    }

    #[test]
    fn bounded_security_json_rejects_oversized_envelope_before_parsing() {
        let input = vec![b' '; MAX_SECURITY_WIRE_BYTES + 1];
        let error =
            deserialize_bounded_security_json::<AuthorizationRequest>(&input).unwrap_err();

        assert_eq!(error.classify(), serde_json::error::Category::Io);
    }

    #[test]
    fn bounded_security_json_reader_accepts_exact_envelope_limit() {
        let input = serde_json::to_vec(&serde_json::json!({
            "subject": "did:mycelix:alice",
            "resource": "resource:ledger",
            "action": "Read",
            "policy_version": 7
        }))
        .unwrap();
        let mut padded = input;
        padded.resize(MAX_SECURITY_WIRE_BYTES, b' ');

        let cursor = std::io::Cursor::new(padded);
        let decoded: AuthorizationRequest =
            deserialize_bounded_security_json_reader(cursor).unwrap();
        assert_eq!(decoded.subject(), "did:mycelix:alice");
    }

    #[test]
    fn bounded_security_reader_accepts_exact_budget_and_then_eof() {
        let input = vec![b'x'; MAX_SECURITY_WIRE_BYTES];
        let mut reader = BoundedSecurityReader::new(std::io::Cursor::new(input));
        let mut output = Vec::new();

        reader.read_to_end(&mut output).unwrap();
        assert_eq!(output.len(), MAX_SECURITY_WIRE_BYTES);

        let mut probe = [0u8; 1];
        assert_eq!(reader.read(&mut probe).unwrap(), 0);
    }

    #[test]
    fn bounded_security_reader_rejects_first_byte_over_budget() {
        let input = vec![b'x'; MAX_SECURITY_WIRE_BYTES + 1];
        let mut reader = BoundedSecurityReader::new(std::io::Cursor::new(input));
        let mut output = vec![0u8; MAX_SECURITY_WIRE_BYTES + 1];

        let error = reader.read_exact(&mut output).unwrap_err();
        assert_eq!(error.kind(), std::io::ErrorKind::InvalidData);
        assert_eq!(reader.remaining, 0);
        assert_eq!(output[..MAX_SECURITY_WIRE_BYTES], vec![b'x'; MAX_SECURITY_WIRE_BYTES]);
    }

    #[test]
    fn bounded_security_reader_zero_length_read_does_not_consume_input() {
        let input = vec![b'x'; 1];
        let mut reader = BoundedSecurityReader::new(std::io::Cursor::new(input));
        let mut empty = [];
        assert_eq!(reader.read(&mut empty).unwrap(), 0);

        let mut byte = [0u8; 1];
        assert_eq!(reader.read(&mut byte).unwrap(), 1);
        assert_eq!(byte[0], b'x');
    }

    #[test]
    fn bounded_security_json_reader_rejects_oversized_stream() {
        let input = serde_json::to_vec(&serde_json::json!({
            "subject": "did:mycelix:alice",
            "resource": "resource:ledger",
            "action": "Read",
            "policy_version": 7
        }))
        .unwrap();
        let mut oversized = input;
        oversized.resize(MAX_SECURITY_WIRE_BYTES + 1, b' ');

        let cursor = std::io::Cursor::new(oversized);
        let error =
            deserialize_bounded_security_json_reader::<_, AuthorizationRequest>(cursor)
                .unwrap_err();

        assert_eq!(error.classify(), serde_json::error::Category::Io);
        assert_eq!(
            error.io_error_kind(),
            Some(std::io::ErrorKind::InvalidData)
        );
    }

    #[test]
    fn authorization_request_wire_identifier_exact_limit_is_accepted() {
        let boundary = "x".repeat(MAX_SECURITY_IDENTIFIER_BYTES);
        let json = serde_json::json!({
            "subject": boundary,
            "resource": "resource:ledger",
            "action": "Read",
            "policy_version": 7
        });

        let decoded: AuthorizationRequest = serde_json::from_value(json).unwrap();
        assert_eq!(decoded.subject.len(), MAX_SECURITY_IDENTIFIER_BYTES);
    }

    #[test]
    fn authorization_request_wire_resource_exact_limit_is_accepted() {
        let boundary = "x".repeat(MAX_SECURITY_IDENTIFIER_BYTES);
        let json = serde_json::json!({
            "subject": "did:mycelix:alice",
            "resource": boundary,
            "action": "Read",
            "policy_version": 7
        });

        let decoded: AuthorizationRequest = serde_json::from_value(json).unwrap();
        assert_eq!(decoded.resource.len(), MAX_SECURITY_IDENTIFIER_BYTES);
    }

    #[test]
    fn authorization_request_wire_resource_over_limit_is_rejected() {
        let oversized = serde_json::json!({
            "subject": "did:mycelix:alice",
            "resource": "x".repeat(MAX_SECURITY_IDENTIFIER_BYTES + 1),
            "action": "Read",
            "policy_version": 7
        });

        assert!(serde_json::from_value::<AuthorizationRequest>(oversized).is_err());
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

    #[test]
    fn authority_freshness_lease_conversion_is_checked() {
        assert_eq!(authority_lease_until_us(42), Some(42_000));
        let boundary = u64::MAX / 1_000;
        assert_eq!(authority_lease_until_us(boundary), Some(boundary * 1_000));
        assert_eq!(authority_lease_until_us(boundary + 1), None);
    }

    #[test]
    fn capability_binding_commits_every_authority_relevant_field() {
        let baseline = capability();
        let baseline_digest = baseline.binding_digest();

        let mutations = [
            {
                let mut candidate = baseline.clone();
                candidate.subject.push_str(":changed");
                candidate
            },
            {
                let mut candidate = baseline.clone();
                candidate.issuer.push_str(":changed");
                candidate
            },
            {
                let mut candidate = baseline.clone();
                candidate.resource.push_str(":changed");
                candidate
            },
            {
                let mut candidate = baseline.clone();
                candidate.actions = vec![CapabilityAction::Write];
                candidate
            },
            {
                let mut candidate = baseline.clone();
                candidate.not_before_us = 101;
                candidate
            },
            {
                let mut candidate = baseline.clone();
                candidate.expires_at_us = 199;
                candidate
            },
            {
                let mut candidate = baseline.clone();
                candidate.policy_version = 8;
                candidate
            },
        ];

        for candidate in mutations {
            assert_ne!(candidate.binding_digest(), baseline_digest);
        }
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
            VerificationEvidence::new_for_capability(
                &other,
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
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
        assert!(authorize_permit(&verified(), &request(CapabilityAction::Read), 150).is_ok());
    }

    #[test]
    fn allow_mints_exactly_bound_enforcement_request() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();
        let enforcement = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
            150,
        )
        .unwrap();
        assert_eq!(enforcement.request(), &request(CapabilityAction::Read));
        assert_eq!(enforcement.issued_at_us(), 150);
        assert_eq!(enforcement.valid_until_us(), 200);
        assert!(enforcement.is_valid_at(199));
        assert!(!enforcement.is_valid_at(200));
    }

    #[test]
    fn verification_lease_expires_at_exact_boundary() {
        let cap = capability();
        assert_eq!(
            verify_capability(
                cap.clone(),
                VerificationEvidence::new_for_capability_with_valid_until(
                    &cap,
                    SignatureVerification::Verified,
                    RevocationStatus::Current,
                    AuthorityResolution::Unambiguous,
                    175,
                ),
                175,
            )
            .unwrap_err(),
            AuthorizationDecision::Deny(AuthorizationDenial::OutsideValidityWindow)
        );
    }

    #[test]
    fn capability_expiry_boundary_is_exclusive() {
        let cap = capability();
        let evidence = VerificationEvidence::new_for_capability(
            &cap,
            SignatureVerification::Verified,
            RevocationStatus::Current,
            AuthorityResolution::Unambiguous,
        );

        assert!(verify_capability(cap.clone(), evidence.clone(), cap.expires_at_us - 1).is_ok());
        assert_eq!(
            verify_capability(cap.clone(), evidence.clone(), cap.expires_at_us),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow,
            ))
        );

        let verified = verify_capability(cap.clone(), evidence, cap.expires_at_us - 1).unwrap();
        assert_eq!(
            authorize_permit(
                &verified,
                &request(CapabilityAction::Read),
                cap.expires_at_us
            ),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow,
            ))
        );
    }

    #[test]
    fn permit_lifetime_is_bounded_by_verification_freshness() {
        let cap = capability();
        let verified = verify_capability(
            cap.clone(),
            VerificationEvidence::new_for_capability_with_valid_until(
                &cap,
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                175,
            ),
            150,
        )
        .unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        assert_eq!(permit.valid_until_us, 175);
        assert!(!permit.is_valid_at(176));
    }

    #[test]
    fn permit_lifetime_overflow_fails_closed() {
        let cap = Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:ledger",
            vec![CapabilityAction::Read],
            u64::MAX - MAX_AUTHORIZATION_PERMIT_LIFETIME_US + 1,
            u64::MAX,
            7,
        )
        .unwrap();
        let evidence = VerificationEvidence::new_for_capability(
            &cap,
            SignatureVerification::Verified,
            RevocationStatus::Current,
            AuthorityResolution::Unambiguous,
        );
        let verified = verify_capability(cap, evidence, u64::MAX - 1).unwrap();
        assert_eq!(
            authorize_permit(&verified, &request(CapabilityAction::Read), u64::MAX - 1),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow,
            ))
        );
    }

    #[test]
    fn permit_expiry_boundary_is_exclusive() {
        let verified = verified();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        assert!(permit.valid_until_us > 150);
        assert!(permit.is_valid_at(permit.valid_until_us - 1));
        assert!(!permit.is_valid_at(permit.valid_until_us));

        let evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &capability(),
                [0xA5; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                permit.valid_until_us,
            );
        let permit_valid_until_us = permit.valid_until_us;
        assert_eq!(
            EnforcementRequest::from_permit(permit, evidence, permit_valid_until_us),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow,
            ))
        );
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
            VerificationEvidence::new_for_capability(
                &long_lived,
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
            150,
        )
        .unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        assert_eq!(
            permit.valid_until_us,
            150 + MAX_AUTHORIZATION_PERMIT_LIFETIME_US
        );
        assert!(!permit.is_valid_at(150 + MAX_AUTHORIZATION_PERMIT_LIFETIME_US + 1));
    }

    #[test]
    fn enforcement_rejects_time_before_permit_issuance() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();

        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
            149,
        );

        assert_eq!(
            result,
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow
            ))
        );
    }

    #[test]
    fn invalid_fresh_signature_evidence_blocks_enforcement() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();
        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Invalid,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
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
            VerificationEvidence::new_for_capability(
                &other,
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
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
            VerificationEvidence::new_for_capability_with_valid_until(
                &cap,
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                175,
            ),
            150,
        )
        .unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability_with_valid_until(
                &cap,
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                175,
            ),
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
                &cap,
                [4; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                175,
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
    fn zero_freshness_digest_fails_closed_at_verification() {
        let cap = capability();
        let evidence = VerificationEvidence::new_for_capability_with_freshness_digest(
            &cap,
            [0; 32],
            SignatureVerification::Verified,
            RevocationStatus::Current,
            AuthorityResolution::Unambiguous,
            200,
        );

        assert_eq!(
            verify_capability(cap, evidence, 150).unwrap_err(),
            AuthorizationDecision::Indeterminate(AuthorizationIndeterminacy::AmbiguousAuthority)
        );
    }

    #[test]
    fn missing_authority_binding_fails_closed_at_verification() {
        let cap = capability();
        let evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap,
                [0; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                200,
            );
        assert_eq!(
            verify_capability(cap, evidence, 150).unwrap_err(),
            AuthorizationDecision::Indeterminate(AuthorizationIndeterminacy::AmbiguousAuthority)
        );
    }

    #[test]
    fn missing_authority_binding_fails_closed_at_enforcement() {
        let cap = capability();
        let valid_evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap,
                [7; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                200,
            );
        let verified = verify_capability(cap.clone(), valid_evidence, 150).unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        let missing_evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap,
                [0; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                200,
            );
        assert_eq!(
            revalidate_permit(&permit, missing_evidence, 151),
            AuthorizationOutcome::Indeterminate(AuthorizationIndeterminacy::AmbiguousAuthority)
        );
    }

    #[test]
    fn zero_freshness_digest_remains_missing_authority_binding() {
        assert_eq!(authority_binding_from_freshness_digest([0; 32]), [0; 32]);
    }

    #[test]
    fn authority_binding_is_stable_across_lease_refresh() {
        let digest = [0x11; 32];
        let initial = authority_binding_from_freshness_digest(digest);
        let refreshed = authority_binding_from_freshness_digest(digest);
        assert_eq!(initial, refreshed);
        assert_ne!(initial, [0; 32]);
    }

    #[test]
    fn authority_binding_changes_with_freshness_domain() {
        let a = authority_binding_from_freshness_digest([0x11; 32]);
        let generation_changed = authority_binding_from_freshness_digest([0x12; 32]);
        assert_ne!(a, generation_changed);
        assert_eq!(
            AUTHORITY_FRESHNESS_PROTOCOL_VERSION,
            "mycelix-authority-freshness-v0.1"
        );
        assert_eq!(
            AUTHORITY_FRESHNESS_PROFILE,
            "mycelix-authority-freshness-bundle-v1-blake3-framed"
        );
    }

    #[test]
    fn refreshed_authority_lease_cannot_extend_existing_permit() {
        let cap = capability();
        let initial_evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap,
                [1; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                175,
            );
        let verified = verify_capability(cap.clone(), initial_evidence, 150).unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        assert_eq!(permit.valid_until_us, 175);
        assert_eq!(
            revalidate_permit(
                &permit,
                VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                    &cap,
                    [1; 32],
                    SignatureVerification::Verified,
                    RevocationStatus::Current,
                    AuthorityResolution::Unambiguous,
                    190,
                ),
                174,
            ),
            AuthorizationOutcome::Allow
        );

        let refreshed_evidence =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap,
                [1; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                190,
            );
        assert_eq!(
            revalidate_permit(&permit, refreshed_evidence, 175),
            AuthorizationOutcome::Deny(AuthorizationDenial::OutsideValidityWindow)
        );

        let refreshed_verified = verify_capability(
            cap,
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &capability(),
                [1; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                190,
            ),
            175,
        )
        .unwrap();
        let replacement_permit =
            authorize_permit(&refreshed_verified, &request(CapabilityAction::Read), 175).unwrap();
        assert_eq!(replacement_permit.issued_at_us, 175);
        assert_eq!(replacement_permit.valid_until_us, 190);
    }

    #[test]
    fn authority_generation_binding_blocks_revalidation_with_new_generation() {
        let cap = capability();
        let evidence_a =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap,
                [1; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                200,
            );
        let verified = verify_capability(cap.clone(), evidence_a, 150).unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();

        let evidence_b =
            VerificationEvidence::new_for_capability_with_authority_binding_and_valid_until(
                &cap,
                [2; 32],
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                200,
            );
        assert_eq!(
            revalidate_permit(&permit, evidence_b, 151),
            AuthorizationOutcome::Deny(AuthorizationDenial::AuthorityBindingMismatch)
        );
    }
    #[test]
    fn authority_generation_binding_is_preserved_at_enforcement() {
        let cap = capability();
        let freshness_digest = [9; 32];
        let expected_binding = authority_binding_from_freshness_digest(freshness_digest);
        let evidence = VerificationEvidence::new_for_capability_with_freshness_digest(
            &cap,
            freshness_digest,
            SignatureVerification::Verified,
            RevocationStatus::Current,
            AuthorityResolution::Unambiguous,
            200,
        );
        let verified = verify_capability(cap.clone(), evidence, 150).unwrap();
        let permit = authorize_permit(&verified, &request(CapabilityAction::Read), 150).unwrap();
        let enforcement = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability_with_freshness_digest(
                &cap,
                freshness_digest,
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
                200,
            ),
            151,
        )
        .unwrap();
        assert_eq!(enforcement.authority_binding(), expected_binding);
    }

    #[test]
    fn revocation_after_authorization_blocks_enforcement() {
        let permit = authorize_permit(&verified(), &request(CapabilityAction::Read), 150).unwrap();
        let result = EnforcementRequest::from_permit(
            permit,
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Verified,
                RevocationStatus::Revoked,
                AuthorityResolution::Unambiguous,
            ),
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
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Invalid,
                RevocationStatus::Current,
                AuthorityResolution::Unambiguous,
            ),
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
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Verified,
                RevocationStatus::Revoked,
                AuthorityResolution::Unambiguous,
            ),
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
            VerificationEvidence::new_for_capability(
                &capability(),
                SignatureVerification::Verified,
                RevocationStatus::Current,
                AuthorityResolution::Ambiguous,
            ),
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
            authorize_permit(&verified(), &req, 150),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::SubjectMismatch
            ))
        );
    }

    #[test]
    fn ungranted_action_is_denied() {
        assert_eq!(
            authorize_permit(&verified(), &request(CapabilityAction::Admin), 150),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::ActionNotGranted
            ))
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
            authorize_permit(&verified(), &req, 150),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::PolicyVersionMismatch
            ))
        );
    }

    #[test]
    fn expired_capability_is_denied() {
        assert_eq!(
            authorize_permit(&verified(), &request(CapabilityAction::Read), 201),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::OutsideValidityWindow
            ))
        );
    }

    #[test]
    fn deserialized_malformed_authorization_request_is_rejected() {
        let empty_subject: Result<AuthorizationRequest, _> = serde_json::from_str(
            r#"{
                "subject":"",
                "resource":"ledger",
                "action":"Read",
                "policy_version":7
            }"#,
        );
        assert!(empty_subject.is_err());

        let oversized_subject: Result<AuthorizationRequest, _> = serde_json::from_str(&format!(
            r#"{{"subject":"{}","resource":"ledger","action":"Read","policy_version":7}}"#,
            "x".repeat(MAX_SECURITY_IDENTIFIER_BYTES + 1)
        ));
        assert!(oversized_subject.is_err());

        let unknown_field: Result<AuthorizationRequest, _> = serde_json::from_str(
            r#"{
                "subject":"did:mycelix:alice",
                "resource":"ledger",
                "action":"Read",
                "policy_version":7,
                "authority":"ignored-by-old-parser"
            }"#,
        );
        assert!(unknown_field.is_err());
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
        let evidence = VerificationEvidence::new_for_capability(
            &malformed,
            SignatureVerification::Verified,
            RevocationStatus::Current,
            AuthorityResolution::Unambiguous,
        );
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
        assert!(
            Capability::new(
                "subject",
                "issuer",
                "resource",
                vec![CapabilityAction::Read],
                7,
                7,
                1,
            )
            .is_err()
        );
    }
}