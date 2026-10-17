//! Pure, deterministic regional-federation policy kernel.
//!
//! This module does not perform network I/O, read the system clock, interpret
//! law, or claim that a physical storage location has been independently
//! attested. Trust adapters must authenticate profiles and consent grants
//! before constructing the opaque verified wrappers exposed by this crate.

use std::collections::BTreeSet;

const SUPPORTED_SCHEMA_VERSION: u16 = 1;
const MAX_PROFILE_ID_BYTES: usize = 256;
const MAX_PROFILE_VERSION_BYTES: usize = 128;
const MAX_SCOPE_ID_BYTES: usize = 256;
const MAX_SIGNING_KEY_ID_BYTES: usize = 256;
const MAX_GRANT_ID_BYTES: usize = 256;
const MAX_RECIPIENT_ID_BYTES: usize = 512;
const MAX_PURPOSE_BYTES: usize = 512;
const MAX_SIGNATURE_BYTES: usize = 4096;
const MAX_RULES: usize = 64;
const MAX_SCOPES_PER_RULE: usize = 256;
const MAX_ACTIONS_PER_RULE: usize = 8;

/// Sensitivity class asserted by the trusted classification adapter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DataClass {
    Public,
    Community,
    Personal,
    Sensitive,
    Restricted,
}

impl DataClass {
    const fn tag(self) -> u8 {
        match self {
            Self::Public => 1,
            Self::Community => 2,
            Self::Personal => 3,
            Self::Sensitive => 4,
            Self::Restricted => 5,
        }
    }
}

/// Actions that cross a scope boundary and therefore need bilateral policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum BoundaryAction {
    Replicate,
    Export,
    Backup,
    Telemetry,
    Disclose,
}

impl BoundaryAction {
    const fn tag(self) -> u8 {
        match self {
            Self::Replicate => 1,
            Self::Export => 2,
            Self::Backup => 3,
            Self::Telemetry => 4,
            Self::Disclose => 5,
        }
    }
}

/// One rule for exactly one data class. Scope identifiers are opaque,
/// case-sensitive identifiers, not geolocation claims.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlacementRule {
    pub data_class: DataClass,
    pub allowed_actions: Vec<BoundaryAction>,
    /// Exact destination scope IDs to which the source may send this class.
    pub allowed_destination_scopes: Vec<String>,
    /// Exact source scope IDs this destination agrees to receive from.
    pub allowed_source_scopes: Vec<String>,
    pub purpose_required: bool,
    pub recipient_required: bool,
    pub consent_required: bool,
}

/// Authenticated classification claim bound to one exact object identity and
/// content digest. A caller must not be able to choose a less-sensitive class by
/// editing TransferRequest; the verifier is the authority for this claim.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClassificationClaim {
    pub schema_version: u16,
    /// Exact record/object identity; never a wildcard.
    pub resource_id: String,
    /// Optional pseudonymous data-subject identifier; required when a
    /// consent-required operation is evaluated.
    pub subject_id: Option<String>,
    /// Lowercase BLAKE3-256 digest of the exact object bytes.
    pub content_digest: String,
    pub data_class: DataClass,
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub issuer_id: String,
    pub signing_key_id: String,
    /// Lowercase BLAKE3-256 digest of canonical_payload().
    pub claim_digest: String,
    pub signature: Vec<u8>,
}

impl ClassificationClaim {
    pub fn canonical_payload(&self) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_DATA_CLASSIFICATION_V1\0");
        output.extend_from_slice(&self.schema_version.to_le_bytes());
        append_string(&mut output, &self.resource_id);
        match &self.subject_id {
            Some(subject) => {
                output.push(1);
                append_string(&mut output, subject);
            }
            None => output.push(0),
        }
        append_string(&mut output, &self.content_digest);
        output.push(self.data_class.tag());
        output.extend_from_slice(&self.valid_from_unix_ms.to_le_bytes());
        output.extend_from_slice(&self.valid_until_unix_ms.to_le_bytes());
        append_string(&mut output, &self.issuer_id);
        output
    }

    pub fn signature_message(&self) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_DATA_CLASSIFICATION_SIGNATURE_V1\0");
        append_string(&mut output, &self.issuer_id);
        append_string(&mut output, &self.signing_key_id);
        append_string(&mut output, &self.claim_digest);
        output
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClassificationVerificationFailure {
    InvalidSignature,
    UnknownIssuer,
    RevokedIssuer,
    StaleAuthority,
    UnsupportedClassificationMethod,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClassificationValidationError {
    UnsupportedSchemaVersion,
    InvalidResourceId,
    InvalidSubjectId,
    InvalidIssuer,
    InvalidSigningKeyId,
    InvalidContentDigest,
    InvalidClaimDigest,
    InvalidSignatureLength,
    InvalidValidityWindow,
    ClaimNotActive,
    VerificationFailed(ClassificationVerificationFailure),
}

/// Trusted adapter must recompute BLAKE3-256 over canonical_payload(), verify
/// signature_message(), authorize the issuer for the classification method, and
/// return a deadline bounded by signer-revocation freshness. The claim binds an
/// exact resource identity and byte digest.
pub trait ClassificationVerifier {
    fn verify_classification(
        &self,
        claim: &ClassificationClaim,
        now_ms: u64,
    ) -> Result<u64, ClassificationVerificationFailure>;
}

/// Opaque verifier-backed classification. Only this type is accepted by the
/// transfer evaluator, so a raw caller-supplied DataClass is insufficient.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedClassification {
    claim: ClassificationClaim,
    verified_at_unix_ms: u64,
    evidence_valid_until_unix_ms: u64,
}

impl VerifiedClassification {
    pub fn verify<V: ClassificationVerifier>(
        claim: ClassificationClaim,
        now_ms: u64,
        verifier: &V,
    ) -> Result<Self, ClassificationValidationError> {
        if claim.schema_version != SUPPORTED_SCHEMA_VERSION {
            return Err(ClassificationValidationError::UnsupportedSchemaVersion);
        }
        if !valid_text(&claim.resource_id, MAX_RECIPIENT_ID_BYTES) {
            return Err(ClassificationValidationError::InvalidResourceId);
        }
        if claim.subject_id.as_ref().is_some_and(|subject| {
            !valid_text(subject, MAX_DID_BYTES)
        }) {
            return Err(ClassificationValidationError::InvalidSubjectId);
        }
        if !valid_text(&claim.issuer_id, MAX_DID_BYTES) {
            return Err(ClassificationValidationError::InvalidIssuer);
        }
        if !valid_text(&claim.signing_key_id, MAX_SIGNING_KEY_ID_BYTES) {
            return Err(ClassificationValidationError::InvalidSigningKeyId);
        }
        if !valid_digest(&claim.content_digest) {
            return Err(ClassificationValidationError::InvalidContentDigest);
        }
        if !valid_digest(&claim.claim_digest) {
            return Err(ClassificationValidationError::InvalidClaimDigest);
        }
        if claim.signature.is_empty() || claim.signature.len() > MAX_SIGNATURE_BYTES {
            return Err(ClassificationValidationError::InvalidSignatureLength);
        }
        if claim.valid_from_unix_ms >= claim.valid_until_unix_ms {
            return Err(ClassificationValidationError::InvalidValidityWindow);
        }
        if !(claim.valid_from_unix_ms <= now_ms && now_ms < claim.valid_until_unix_ms) {
            return Err(ClassificationValidationError::ClaimNotActive);
        }
        let evidence_until = verifier
            .verify_classification(&claim, now_ms)
            .map_err(ClassificationValidationError::VerificationFailed)?;
        if evidence_until <= now_ms {
            return Err(ClassificationValidationError::VerificationFailed(
                ClassificationVerificationFailure::StaleAuthority,
            ));
        }
        Ok(Self {
            verified_at_unix_ms: now_ms,
            evidence_valid_until_unix_ms: evidence_until.min(claim.valid_until_unix_ms),
            claim,
        })
    }
}

/// Signed profile metadata plus canonical policy content.
///
/// The signature and digest are intentionally verified by an adapter whose
/// trust roots and cryptographic implementation are configured by deployment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegionalProfile {
    pub schema_version: u16,
    pub profile_id: String,
    pub profile_version: String,
    pub scope_id: String,
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub rules: Vec<PlacementRule>,
    /// Lowercase hex BLAKE3-256 digest of canonical_payload(). The digest
    /// algorithm is fixed for schema version 1 and is not negotiated per request.
    pub content_digest: String,
    pub signing_key_id: String,
    /// Opaque bytes interpreted only by the configured verifier.
    pub signature: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProfileValidationError {
    UnsupportedSchemaVersion,
    InvalidIdentifier,
    InvalidValidityWindow,
    InvalidDigestFormat,
    InvalidSignatureLength,
    EmptyRules,
    TooManyRules,
    DuplicateDataClassRule,
    EmptyActionList,
    TooManyActions,
    DuplicateAction,
    TooManyScopes,
    InvalidScope,
    DuplicateScope,
    ProfileNotActive,
    VerificationFailed(ProfileVerificationFailure),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProfileVerificationFailure {
    DigestMismatch,
    InvalidSignature,
    UnknownSigner,
    RevokedSigner,
    StaleTrustRoots,
    UnsupportedAlgorithm,
}

/// Deployment-owned implementation must validate the canonical digest,
/// signature, signer trust root, key validity, and revocation state. It must
/// bind the signature to signature_message(), not only to a profile ID.
pub trait ProfileVerifier {
    /// Verify canonical digest/signature, trust root, key validity and revocation.
    /// Return the deadline until those checks may still be trusted; it must not
    /// be an arbitrary far-future timestamp.
    fn verify_profile(
        &self,
        profile: &RegionalProfile,
        now_ms: u64,
    ) -> Result<u64, ProfileVerificationFailure>;
}

impl RegionalProfile {
    /// Validate structure without claiming signature authenticity.
    pub fn validate_shape(&self) -> Result<(), ProfileValidationError> {
        if self.schema_version != SUPPORTED_SCHEMA_VERSION {
            return Err(ProfileValidationError::UnsupportedSchemaVersion);
        }
        if !valid_text(&self.profile_id, MAX_PROFILE_ID_BYTES)
            || !valid_text(&self.profile_version, MAX_PROFILE_VERSION_BYTES)
            || !valid_text(&self.scope_id, MAX_SCOPE_ID_BYTES)
            || !valid_text(&self.signing_key_id, MAX_SIGNING_KEY_ID_BYTES)
        {
            return Err(ProfileValidationError::InvalidIdentifier);
        }
        if self.valid_from_unix_ms >= self.valid_until_unix_ms {
            return Err(ProfileValidationError::InvalidValidityWindow);
        }
        if self.content_digest.len() != 64
            || !self
                .content_digest
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
        {
            return Err(ProfileValidationError::InvalidDigestFormat);
        }
        if self.signature.is_empty() || self.signature.len() > MAX_SIGNATURE_BYTES {
            return Err(ProfileValidationError::InvalidSignatureLength);
        }
        if self.rules.is_empty() {
            return Err(ProfileValidationError::EmptyRules);
        }
        if self.rules.len() > MAX_RULES {
            return Err(ProfileValidationError::TooManyRules);
        }

        let mut classes = BTreeSet::new();
        for rule in &self.rules {
            if !classes.insert(rule.data_class) {
                return Err(ProfileValidationError::DuplicateDataClassRule);
            }
            if rule.allowed_actions.is_empty() {
                return Err(ProfileValidationError::EmptyActionList);
            }
            if rule.allowed_actions.len() > MAX_ACTIONS_PER_RULE {
                return Err(ProfileValidationError::TooManyActions);
            }
            let mut actions = BTreeSet::new();
            for action in &rule.allowed_actions {
                if !actions.insert(*action) {
                    return Err(ProfileValidationError::DuplicateAction);
                }
            }
            validate_scope_list(&rule.allowed_destination_scopes)?;
            validate_scope_list(&rule.allowed_source_scopes)?;
        }
        Ok(())
    }

    /// Deterministic domain-separated serialization for digest verification.
    ///
    /// Set-like vectors are sorted before encoding. Duplicate entries are still
    /// rejected by validate_shape(), so reordering cannot change profile
    /// identity and duplicate values cannot be smuggled in as aliases.
    pub fn canonical_payload(&self) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_REGIONAL_PROFILE_V1\0");
        output.extend_from_slice(&self.schema_version.to_le_bytes());
        append_string(&mut output, &self.profile_id);
        append_string(&mut output, &self.profile_version);
        append_string(&mut output, &self.scope_id);
        output.extend_from_slice(&self.valid_from_unix_ms.to_le_bytes());
        output.extend_from_slice(&self.valid_until_unix_ms.to_le_bytes());

        let mut rules: Vec<&PlacementRule> = self.rules.iter().collect();
        rules.sort_by_key(|rule| rule.data_class);
        append_u32(&mut output, rules.len() as u32);
        for rule in rules {
            output.push(rule.data_class.tag());

            let mut actions = rule.allowed_actions.clone();
            actions.sort_unstable();
            append_u32(&mut output, actions.len() as u32);
            output.extend(actions.into_iter().map(BoundaryAction::tag));

            append_sorted_strings(&mut output, &rule.allowed_destination_scopes);
            append_sorted_strings(&mut output, &rule.allowed_source_scopes);
            output.push(u8::from(rule.purpose_required));
            output.push(u8::from(rule.recipient_required));
            output.push(u8::from(rule.consent_required));
        }
        output
    }

    /// Domain-separated signature message. The signing-key identifier is
    /// included so the same digest cannot be re-labelled under another key ID.
    pub fn signature_message(&self) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_REGIONAL_PROFILE_SIGNATURE_V1\0");
        append_string(&mut output, &self.profile_id);
        append_string(&mut output, &self.signing_key_id);
        append_string(&mut output, &self.content_digest);
        output
    }

    fn active_at(&self, now_ms: u64) -> bool {
        self.valid_from_unix_ms <= now_ms && now_ms < self.valid_until_unix_ms
    }
}

/// Opaque profile wrapper: the only public constructor validates shape,
/// time validity, and the caller-supplied trusted verification adapter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedProfile {
    profile: RegionalProfile,
    verified_at_unix_ms: u64,
    evidence_valid_until_unix_ms: u64,
}

impl VerifiedProfile {
    pub fn verify<V: ProfileVerifier>(
        profile: RegionalProfile,
        now_ms: u64,
        verifier: &V,
    ) -> Result<Self, ProfileValidationError> {
        profile.validate_shape()?;
        if !profile.active_at(now_ms) {
            return Err(ProfileValidationError::ProfileNotActive);
        }
        let evidence_until = verifier
            .verify_profile(&profile, now_ms)
            .map_err(ProfileValidationError::VerificationFailed)?;
        if evidence_until <= now_ms {
            return Err(ProfileValidationError::VerificationFailed(
                ProfileVerificationFailure::StaleTrustRoots,
            ));
        }
        Ok(Self {
            verified_at_unix_ms: now_ms,
            evidence_valid_until_unix_ms: evidence_until.min(profile.valid_until_unix_ms),
            profile,
        })
    }

    pub fn profile(&self) -> &RegionalProfile {
        &self.profile
    }
}

/// Grant fields that must be authenticated exactly by the consent verifier.
/// The policy kernel does not regard possession of this struct alone as proof.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConsentGrant {
    pub schema_version: u16,
    pub grant_id: String,
    /// Exact data-subject identity issuing the grant.
    pub subject_id: String,
    /// Exact record/object identity; grants are not wildcards.
    pub resource_id: String,
    /// Consent is bound to the exact content version, not only its identifier.
    pub content_digest: String,
    pub data_class: DataClass,
    pub action: BoundaryAction,
    pub recipient_id: String,
    pub purpose: String,
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub issuer_id: String,
    pub signing_key_id: String,
    /// Lowercase BLAKE3-256 digest of canonical_payload().
    pub grant_digest: String,
    pub signature: Vec<u8>,
}

impl ConsentGrant {
    pub fn canonical_payload(&self) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_CONSENT_GRANT_V1\0");
        output.extend_from_slice(&self.schema_version.to_le_bytes());
        append_string(&mut output, &self.grant_id);
        append_string(&mut output, &self.subject_id);
        append_string(&mut output, &self.resource_id);
        append_string(&mut output, &self.content_digest);
        output.push(self.data_class.tag());
        output.push(self.action.tag());
        append_string(&mut output, &self.recipient_id);
        append_string(&mut output, &self.purpose);
        output.extend_from_slice(&self.valid_from_unix_ms.to_le_bytes());
        output.extend_from_slice(&self.valid_until_unix_ms.to_le_bytes());
        append_string(&mut output, &self.issuer_id);
        output
    }

    pub fn signature_message(&self) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_CONSENT_GRANT_SIGNATURE_V1\0");
        append_string(&mut output, &self.issuer_id);
        append_string(&mut output, &self.signing_key_id);
        append_string(&mut output, &self.grant_digest);
        output
    }
}

/// Verifies the exact signed grant, issuer authority, subject/resource/content
/// binding, data class/action, recipient/purpose, and current revocation state.
/// Must recompute grant_digest over canonical_payload() and verify signature_message().
/// Returns the deadline until which evidence remains trustworthy.
pub trait ConsentVerifier {
    fn verify_current_grant(
        &self,
        grant: &ConsentGrant,
        now_ms: u64,
    ) -> Result<u64, ConsentVerificationFailure>;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConsentVerificationFailure {
    InvalidGrant,
    InvalidSignature,
    UnknownIssuer,
    Revoked,
    StaleRevocationState,
    NotAuthorized,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConsentValidationError {
    UnsupportedSchemaVersion,
    InvalidGrantShape,
    InvalidGrantDigest,
    InvalidSigningKeyId,
    InvalidSignatureLength,
    InvalidValidityWindow,
    GrantNotActive,
    VerificationFailed(ConsentVerificationFailure),
}

/// Opaque, verifier-backed consent evidence. Evidence validity is bounded by
/// both the grant expiry and the trusted adapter's revocation-freshness deadline.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedConsentGrant {
    grant: ConsentGrant,
    verified_at_unix_ms: u64,
    evidence_valid_until_unix_ms: u64,
}

impl VerifiedConsentGrant {
    pub fn verify<V: ConsentVerifier>(
        grant: ConsentGrant,
        now_ms: u64,
        verifier: &V,
    ) -> Result<Self, ConsentValidationError> {
        if grant.schema_version != SUPPORTED_SCHEMA_VERSION {
            return Err(ConsentValidationError::UnsupportedSchemaVersion);
        }
        if !valid_text(&grant.grant_id, MAX_GRANT_ID_BYTES)
            || !valid_text(&grant.subject_id, MAX_DID_BYTES)
            || !valid_text(&grant.resource_id, MAX_RECIPIENT_ID_BYTES)
            || !valid_text(&grant.recipient_id, MAX_RECIPIENT_ID_BYTES)
            || !valid_text(&grant.purpose, MAX_PURPOSE_BYTES)
            || !valid_text(&grant.issuer_id, MAX_DID_BYTES)
            || !valid_text(&grant.signing_key_id, MAX_SIGNING_KEY_ID_BYTES)
            || !valid_digest(&grant.content_digest)
        {
            return Err(ConsentValidationError::InvalidGrantShape);
        }
        if !valid_digest(&grant.grant_digest) {
            return Err(ConsentValidationError::InvalidGrantDigest);
        }
        if grant.signature.is_empty() || grant.signature.len() > MAX_SIGNATURE_BYTES {
            return Err(ConsentValidationError::InvalidSignatureLength);
        }
        if grant.valid_from_unix_ms >= grant.valid_until_unix_ms {
            return Err(ConsentValidationError::InvalidValidityWindow);
        }
        if !(grant.valid_from_unix_ms <= now_ms && now_ms < grant.valid_until_unix_ms) {
            return Err(ConsentValidationError::GrantNotActive);
        }

        let evidence_until = verifier
            .verify_current_grant(&grant, now_ms)
            .map_err(ConsentValidationError::VerificationFailed)?;
        if evidence_until <= now_ms {
            return Err(ConsentValidationError::VerificationFailed(
                ConsentVerificationFailure::StaleRevocationState,
            ));
        }

        Ok(Self {
            verified_at_unix_ms: now_ms,
            evidence_valid_until_unix_ms: evidence_until.min(grant.valid_until_unix_ms),
            grant,
        })
    }
}

/// Bilateral agreement binding two exact regional-profile versions to a limited
/// set of data classes, actions, purposes and recipient identifiers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FederationAgreement {
    pub schema_version: u16,
    pub agreement_id: String,
    pub agreement_version: String,
    pub source_profile_id: String,
    pub source_profile_digest: String,
    pub destination_profile_id: String,
    pub destination_profile_digest: String,
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub allowed_data_classes: Vec<DataClass>,
    pub allowed_actions: Vec<BoundaryAction>,
    /// Exact purposes; wildcard matching is not permitted.
    pub allowed_purposes: Vec<String>,
    /// Exact stable service/operator identifiers; wildcard matching is not permitted.
    pub allowed_recipient_ids: Vec<String>,
    /// Lowercase hex BLAKE3-256 digest of canonical_payload().
    pub content_digest: String,
    /// Approval by an authority authorized to act for source_profile_id.
    pub source_approval_key_id: String,
    pub source_approval_signature: Vec<u8>,
    /// Approval by an authority authorized to act for destination_profile_id.
    pub destination_approval_key_id: String,
    pub destination_approval_signature: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AgreementVerificationFailure {
    DigestMismatch,
    InvalidSourceApproval,
    InvalidDestinationApproval,
    UnknownSourceApprover,
    UnknownDestinationApprover,
    RevokedSourceApprover,
    RevokedDestinationApprover,
    StaleTrustRoots,
    UnsupportedAlgorithm,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AgreementValidationError {
    UnsupportedSchemaVersion,
    InvalidIdentifier,
    InvalidValidityWindow,
    InvalidDigestFormat,
    InvalidSignatureLength,
    EmptyDataClasses,
    DuplicateDataClass,
    EmptyActions,
    DuplicateAction,
    EmptyPurposeAllowlist,
    EmptyRecipientAllowlist,
    InvalidAllowlistValue,
    DuplicateAllowlistValue,
    AgreementNotActive,
    VerificationFailed(AgreementVerificationFailure),
}

/// Deployment-owned verifier hashes canonical_payload(), verifies BOTH
/// source_approval_message() and destination_approval_message(), and checks
/// that each signing key is authorized for the corresponding exact profile ID
/// and digest. It validates key trust/revocation and returns the latest time at
/// which both approval and trust-root evidence remain trustworthy.
pub trait AgreementVerifier {
    fn verify_agreement(
        &self,
        agreement: &FederationAgreement,
        now_ms: u64,
    ) -> Result<u64, AgreementVerificationFailure>;
}

impl FederationAgreement {
    pub fn validate_shape(&self) -> Result<(), AgreementValidationError> {
        if self.schema_version != SUPPORTED_SCHEMA_VERSION {
            return Err(AgreementValidationError::UnsupportedSchemaVersion);
        }
        if !valid_text(&self.agreement_id, MAX_PROFILE_ID_BYTES)
            || !valid_text(&self.agreement_version, MAX_PROFILE_VERSION_BYTES)
            || !valid_text(&self.source_profile_id, MAX_PROFILE_ID_BYTES)
            || !valid_text(&self.destination_profile_id, MAX_PROFILE_ID_BYTES)
            || !valid_text(&self.source_approval_key_id, MAX_SIGNING_KEY_ID_BYTES)
            || !valid_text(&self.destination_approval_key_id, MAX_SIGNING_KEY_ID_BYTES)
        {
            return Err(AgreementValidationError::InvalidIdentifier);
        }
        if !valid_digest(&self.source_profile_digest)
            || !valid_digest(&self.destination_profile_digest)
            || !valid_digest(&self.content_digest)
        {
            return Err(AgreementValidationError::InvalidDigestFormat);
        }
        if self.valid_from_unix_ms >= self.valid_until_unix_ms {
            return Err(AgreementValidationError::InvalidValidityWindow);
        }
        if self.source_approval_signature.is_empty()
            || self.source_approval_signature.len() > MAX_SIGNATURE_BYTES
            || self.destination_approval_signature.is_empty()
            || self.destination_approval_signature.len() > MAX_SIGNATURE_BYTES
        {
            return Err(AgreementValidationError::InvalidSignatureLength);
        }
        if self.allowed_data_classes.is_empty() {
            return Err(AgreementValidationError::EmptyDataClasses);
        }
        let mut classes = BTreeSet::new();
        for class in &self.allowed_data_classes {
            if !classes.insert(*class) {
                return Err(AgreementValidationError::DuplicateDataClass);
            }
        }
        if self.allowed_actions.is_empty() {
            return Err(AgreementValidationError::EmptyActions);
        }
        let mut actions = BTreeSet::new();
        for action in &self.allowed_actions {
            if !actions.insert(*action) {
                return Err(AgreementValidationError::DuplicateAction);
            }
        }
        validate_allowlist(
            &self.allowed_purposes,
            MAX_PURPOSE_BYTES,
            AgreementValidationError::EmptyPurposeAllowlist,
        )?;
        validate_allowlist(
            &self.allowed_recipient_ids,
            MAX_RECIPIENT_ID_BYTES,
            AgreementValidationError::EmptyRecipientAllowlist,
        )?;
        Ok(())
    }

    /// Canonical bytes. Set-like lists are sorted and duplicates are rejected.
    pub fn canonical_payload(&self) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_FEDERATION_AGREEMENT_V1\0");
        output.extend_from_slice(&self.schema_version.to_le_bytes());
        append_string(&mut output, &self.agreement_id);
        append_string(&mut output, &self.agreement_version);
        append_string(&mut output, &self.source_profile_id);
        append_string(&mut output, &self.source_profile_digest);
        append_string(&mut output, &self.destination_profile_id);
        append_string(&mut output, &self.destination_profile_digest);
        output.extend_from_slice(&self.valid_from_unix_ms.to_le_bytes());
        output.extend_from_slice(&self.valid_until_unix_ms.to_le_bytes());

        let mut classes = self.allowed_data_classes.clone();
        classes.sort_unstable();
        append_u32(&mut output, classes.len() as u32);
        output.extend(classes.into_iter().map(DataClass::tag));

        let mut actions = self.allowed_actions.clone();
        actions.sort_unstable();
        append_u32(&mut output, actions.len() as u32);
        output.extend(actions.into_iter().map(BoundaryAction::tag));

        append_sorted_strings(&mut output, &self.allowed_purposes);
        append_sorted_strings(&mut output, &self.allowed_recipient_ids);
        output
    }

    /// Source-side approval message. The role tag prevents a source signature
    /// from being replayed as the destination's approval.
    pub fn source_approval_message(&self) -> Vec<u8> {
        self.approval_message(b"source", &self.source_approval_key_id)
    }

    /// Destination-side approval message, domain-separated from source approval.
    pub fn destination_approval_message(&self) -> Vec<u8> {
        self.approval_message(b"destination", &self.destination_approval_key_id)
    }

    fn approval_message(&self, role: &[u8], key_id: &str) -> Vec<u8> {
        let mut output = Vec::new();
        output.extend_from_slice(b"MYCELIX_FEDERATION_AGREEMENT_APPROVAL_V1\0");
        append_string(&mut output, role);
        append_string(&mut output, &self.agreement_id);
        append_string(&mut output, key_id);
        append_string(&mut output, &self.content_digest);
        output
    }

    fn active_at(&self, now_ms: u64) -> bool {
        self.valid_from_unix_ms <= now_ms && now_ms < self.valid_until_unix_ms
    }
}

/// Opaque agreement that passed structural checks, validity checks and a
/// deployment trust adapter. Both profile digests are rechecked per request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedFederationAgreement {
    agreement: FederationAgreement,
    verified_at_unix_ms: u64,
    evidence_valid_until_unix_ms: u64,
}

impl VerifiedFederationAgreement {
    pub fn verify<V: AgreementVerifier>(
        agreement: FederationAgreement,
        now_ms: u64,
        verifier: &V,
    ) -> Result<Self, AgreementValidationError> {
        agreement.validate_shape()?;
        if !agreement.active_at(now_ms) {
            return Err(AgreementValidationError::AgreementNotActive);
        }
        let evidence_until = verifier
            .verify_agreement(&agreement, now_ms)
            .map_err(AgreementValidationError::VerificationFailed)?;
        if evidence_until <= now_ms {
            return Err(AgreementValidationError::VerificationFailed(
                AgreementVerificationFailure::StaleTrustRoots,
            ));
        }
        Ok(Self {
            verified_at_unix_ms: now_ms,
            evidence_valid_until_unix_ms: evidence_until.min(agreement.valid_until_unix_ms),
            agreement,
        })
    }

    pub fn agreement(&self) -> &FederationAgreement {
        &self.agreement
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransferRequest<'a> {
    pub action: BoundaryAction,
    /// Stable identity of the exact object being transferred.
    pub resource_id: &'a str,
    /// Digest of the exact bytes being transferred; must match the verified claim.
    pub content_digest: &'a str,
    pub classification: &'a VerifiedClassification,
    pub agreement: &'a VerifiedFederationAgreement,
    pub purpose: Option<&'a str>,
    pub recipient_id: Option<&'a str>,
    pub consent: Option<&'a VerifiedConsentGrant>,
    pub evaluated_at_unix_ms: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DenialReason {
    SameScopeIsNotFederation,
    ClassificationSubjectMismatch,
    ClassificationContentMismatch,
    ClassificationNotYetValid,
    ClassificationExpired,
    ClassificationEvidenceStale,
    ClassificationVerifiedAfterDecisionTime,
    AgreementSourceProfileMismatch,
    AgreementDestinationProfileMismatch,
    AgreementNotYetValid,
    AgreementExpired,
    AgreementEvidenceStale,
    AgreementVerifiedAfterDecisionTime,
    AgreementActionNotAllowed,
    AgreementDataClassNotAllowed,
    AgreementPurposeNotAllowed,
    AgreementRecipientNotAllowed,
    SourceProfileInactive,
    DestinationProfileInactive,
    SourceProfileEvidenceStale,
    SourceProfileVerifiedAfterDecisionTime,
    DestinationProfileEvidenceStale,
    DestinationProfileVerifiedAfterDecisionTime,
    SourceRuleMissing,
    DestinationRuleMissing,
    ActionNotAllowedBySource,
    ActionNotAllowedByDestination,
    DestinationNotAllowedBySource,
    SourceNotAllowedByDestination,
    PurposeRequired,
    RecipientRequired,
    ConsentRequired,
    ConsentEvidenceStale,
    ConsentVerifiedAfterDecisionTime,
    ConsentRecipientMismatch,
    ConsentPurposeMismatch,
    ConsentSubjectMismatch,
    ConsentResourceMismatch,
    ConsentContentMismatch,
    ConsentDataClassMismatch,
    ConsentActionMismatch,
    ConsentNotYetValid,
    ConsentExpired,
}

/// A policy denial carries stable reason codes only, never the sensitive record,
/// raw credential, grant contents, or personal data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyDecision {
    pub allowed: bool,
    pub reasons: Vec<DenialReason>,
    /// Non-sensitive identifiers and versioned policy digests for audit.
    pub agreement_id: String,
    pub agreement_version: String,
    pub agreement_digest: String,
    pub source_profile_id: String,
    pub source_profile_version: String,
    pub source_profile_digest: String,
    pub destination_profile_id: String,
    pub destination_profile_version: String,
    pub destination_profile_digest: String,
    pub action: BoundaryAction,
    pub data_class: DataClass,
    pub content_digest: String,
    pub evaluated_at_unix_ms: u64,
}

impl PolicyDecision {
    fn from_reasons(
        reasons: Vec<DenialReason>,
        source: &RegionalProfile,
        destination: &RegionalProfile,
        request: &TransferRequest<'_>,
    ) -> Self {
        Self {
            allowed: reasons.is_empty(),
            reasons,
            agreement_id: request.agreement.agreement.agreement_id.clone(),
            agreement_version: request.agreement.agreement.agreement_version.clone(),
            agreement_digest: request.agreement.agreement.content_digest.clone(),
            source_profile_id: source.profile_id.clone(),
            source_profile_version: source.profile_version.clone(),
            source_profile_digest: source.content_digest.clone(),
            destination_profile_id: destination.profile_id.clone(),
            destination_profile_version: destination.profile_version.clone(),
            destination_profile_digest: destination.content_digest.clone(),
            action: request.action,
            data_class: request.classification.claim.data_class,
            content_digest: request.content_digest.to_owned(),
            evaluated_at_unix_ms: request.evaluated_at_unix_ms,
        }
    }
}

/// Evaluate one cross-scope action against both verified profiles.
///
/// This is a policy gate, not a complete authorization system: adapters must
/// enforce it again at storage/transport/backup/telemetry boundaries. Scope IDs
/// do not attest physical location; independent placement evidence is a
/// separate deployment requirement.
///
/// The caller must supply evaluated_at_unix_ms from a trusted runtime clock.
/// Never accept the evaluation timestamp from the requesting client. This pure
/// kernel intentionally reads no wall clock, so time authority remains an
/// explicit part of the adapter contract.
pub fn evaluate_transfer(
    source: &VerifiedProfile,
    destination: &VerifiedProfile,
    request: &TransferRequest<'_>,
) -> PolicyDecision {
    let mut reasons = Vec::new();
    let source_profile = source.profile();
    let destination_profile = destination.profile();
    let now_ms = request.evaluated_at_unix_ms;

    if source_profile.scope_id == destination_profile.scope_id {
        push_reason(&mut reasons, DenialReason::SameScopeIsNotFederation);
    }
    let classification = request.classification;
    let agreement = request.agreement;
    if now_ms < agreement.verified_at_unix_ms {
        push_reason(&mut reasons, DenialReason::AgreementVerifiedAfterDecisionTime);
    }
    if !agreement.agreement.active_at(now_ms) {
        if now_ms < agreement.agreement.valid_from_unix_ms {
            push_reason(&mut reasons, DenialReason::AgreementNotYetValid);
        } else {
            push_reason(&mut reasons, DenialReason::AgreementExpired);
        }
    }
    if now_ms >= agreement.evidence_valid_until_unix_ms {
        push_reason(&mut reasons, DenialReason::AgreementEvidenceStale);
    }
    if agreement.agreement.source_profile_id != source_profile.profile_id
        || agreement.agreement.source_profile_digest != source_profile.content_digest
    {
        push_reason(&mut reasons, DenialReason::AgreementSourceProfileMismatch);
    }
    if agreement.agreement.destination_profile_id != destination_profile.profile_id
        || agreement.agreement.destination_profile_digest != destination_profile.content_digest
    {
        push_reason(&mut reasons, DenialReason::AgreementDestinationProfileMismatch);
    }
    if !agreement.agreement.allowed_actions.contains(&request.action) {
        push_reason(&mut reasons, DenialReason::AgreementActionNotAllowed);
    }
    if !agreement.agreement.allowed_data_classes.contains(&classification.claim.data_class) {
        push_reason(&mut reasons, DenialReason::AgreementDataClassNotAllowed);
    }
    if request.purpose.is_none_or(|purpose| {
        !agreement.agreement.allowed_purposes.iter().any(|allowed| allowed == purpose)
    }) {
        push_reason(&mut reasons, DenialReason::AgreementPurposeNotAllowed);
    }
    if request.recipient_id.is_none_or(|recipient| {
        !agreement.agreement.allowed_recipient_ids.iter().any(|allowed| allowed == recipient)
    }) {
        push_reason(&mut reasons, DenialReason::AgreementRecipientNotAllowed);
    }
    if now_ms < classification.verified_at_unix_ms {
        push_reason(
            &mut reasons,
            DenialReason::ClassificationVerifiedAfterDecisionTime,
        );
    }
    if classification.claim.resource_id != request.resource_id {
        push_reason(&mut reasons, DenialReason::ClassificationSubjectMismatch);
    }
    if classification.claim.content_digest != request.content_digest {
        push_reason(&mut reasons, DenialReason::ClassificationContentMismatch);
    }
    if now_ms < classification.claim.valid_from_unix_ms {
        push_reason(&mut reasons, DenialReason::ClassificationNotYetValid);
    } else if now_ms >= classification.claim.valid_until_unix_ms {
        push_reason(&mut reasons, DenialReason::ClassificationExpired);
    }
    if now_ms >= classification.evidence_valid_until_unix_ms {
        push_reason(&mut reasons, DenialReason::ClassificationEvidenceStale);
    }
    if now_ms < source.verified_at_unix_ms {
        push_reason(&mut reasons, DenialReason::SourceProfileVerifiedAfterDecisionTime);
    }
    if !source_profile.active_at(now_ms) {
        push_reason(&mut reasons, DenialReason::SourceProfileInactive);
    }
    if now_ms >= source.evidence_valid_until_unix_ms {
        push_reason(&mut reasons, DenialReason::SourceProfileEvidenceStale);
    }
    if now_ms < destination.verified_at_unix_ms {
        push_reason(&mut reasons, DenialReason::DestinationProfileVerifiedAfterDecisionTime);
    }
    if !destination_profile.active_at(now_ms) {
        push_reason(&mut reasons, DenialReason::DestinationProfileInactive);
    }
    if now_ms >= destination.evidence_valid_until_unix_ms {
        push_reason(&mut reasons, DenialReason::DestinationProfileEvidenceStale);
    }

    let data_class = classification.claim.data_class;
    let source_rule = source_profile
        .rules
        .iter()
        .find(|rule| rule.data_class == data_class);
    let destination_rule = destination_profile
        .rules
        .iter()
        .find(|rule| rule.data_class == data_class);

    let Some(source_rule) = source_rule else {
        push_reason(&mut reasons, DenialReason::SourceRuleMissing);
        return PolicyDecision::from_reasons(reasons, source_profile, destination_profile, request);
    };
    let Some(destination_rule) = destination_rule else {
        push_reason(&mut reasons, DenialReason::DestinationRuleMissing);
        return PolicyDecision::from_reasons(reasons, source_profile, destination_profile, request);
    };

    if !source_rule.allowed_actions.contains(&request.action) {
        push_reason(&mut reasons, DenialReason::ActionNotAllowedBySource);
    }
    if !destination_rule.allowed_actions.contains(&request.action) {
        push_reason(&mut reasons, DenialReason::ActionNotAllowedByDestination);
    }
    if !source_rule
        .allowed_destination_scopes
        .contains(&destination_profile.scope_id)
    {
        push_reason(&mut reasons, DenialReason::DestinationNotAllowedBySource);
    }
    if !destination_rule
        .allowed_source_scopes
        .contains(&source_profile.scope_id)
    {
        push_reason(&mut reasons, DenialReason::SourceNotAllowedByDestination);
    }

    let purpose_required = source_rule.purpose_required || destination_rule.purpose_required;
    let recipient_required = source_rule.recipient_required || destination_rule.recipient_required;
    let consent_required = source_rule.consent_required || destination_rule.consent_required;

    if purpose_required && request.purpose.is_none_or(|purpose| purpose.trim().is_empty()) {
        push_reason(&mut reasons, DenialReason::PurposeRequired);
    }
    if recipient_required
        && request
            .recipient_id
            .is_none_or(|recipient| recipient.trim().is_empty())
    {
        push_reason(&mut reasons, DenialReason::RecipientRequired);
    }

    if consent_required {
        match request.consent {
            None => push_reason(&mut reasons, DenialReason::ConsentRequired),
            Some(consent) => {
                if now_ms < consent.verified_at_unix_ms {
                    push_reason(&mut reasons, DenialReason::ConsentVerifiedAfterDecisionTime);
                }
                if !(consent.grant.valid_from_unix_ms <= now_ms
                    && now_ms < consent.grant.valid_until_unix_ms)
                {
                    if now_ms < consent.grant.valid_from_unix_ms {
                        push_reason(&mut reasons, DenialReason::ConsentNotYetValid);
                    } else {
                        push_reason(&mut reasons, DenialReason::ConsentExpired);
                    }
                }
                if now_ms >= consent.evidence_valid_until_unix_ms {
                    push_reason(&mut reasons, DenialReason::ConsentEvidenceStale);
                }
                if request.recipient_id != Some(consent.grant.recipient_id.as_str()) {
                    push_reason(&mut reasons, DenialReason::ConsentRecipientMismatch);
                }
                if request.purpose != Some(consent.grant.purpose.as_str()) {
                    push_reason(&mut reasons, DenialReason::ConsentPurposeMismatch);
                }
                if classification.claim.subject_id.as_deref()
                    != Some(consent.grant.subject_id.as_str())
                {
                    push_reason(&mut reasons, DenialReason::ConsentSubjectMismatch);
                }
                if request.resource_id != consent.grant.resource_id {
                    push_reason(&mut reasons, DenialReason::ConsentResourceMismatch);
                }
                if request.content_digest != consent.grant.content_digest {
                    push_reason(&mut reasons, DenialReason::ConsentContentMismatch);
                }
                if classification.claim.data_class != consent.grant.data_class {
                    push_reason(&mut reasons, DenialReason::ConsentDataClassMismatch);
                }
                if request.action != consent.grant.action {
                    push_reason(&mut reasons, DenialReason::ConsentActionMismatch);
                }
            }
        }
    }
    PolicyDecision::from_reasons(reasons, source_profile, destination_profile, request)
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn validate_allowlist(
    values: &[String],
    max_bytes: usize,
    empty_error: AgreementValidationError,
) -> Result<(), AgreementValidationError> {
    if values.is_empty() {
        return Err(empty_error);
    }
    if values.len() > MAX_SCOPES_PER_RULE {
        return Err(AgreementValidationError::InvalidAllowlistValue);
    }
    let mut seen = BTreeSet::new();
    for value in values {
        if !valid_text(value, max_bytes) {
            return Err(AgreementValidationError::InvalidAllowlistValue);
        }
        if !seen.insert(value) {
            return Err(AgreementValidationError::DuplicateAllowlistValue);
        }
    }
    Ok(())
}

fn validate_scope_list(scopes: &[String]) -> Result<(), ProfileValidationError> {
    if scopes.len() > MAX_SCOPES_PER_RULE {
        return Err(ProfileValidationError::TooManyScopes);
    }
    let mut seen = BTreeSet::new();
    for scope in scopes {
        if !valid_text(scope, MAX_SCOPE_ID_BYTES) {
            return Err(ProfileValidationError::InvalidScope);
        }
        if !seen.insert(scope) {
            return Err(ProfileValidationError::DuplicateScope);
        }
    }
    Ok(())
}

fn valid_text(value: &str, max_bytes: usize) -> bool {
    !value.trim().is_empty()
        && value.len() <= max_bytes
        && !value.chars().any(|character| character.is_control())
}

fn append_string(output: &mut Vec<u8>, value: &str) {
    append_u32(output, value.len() as u32);
    output.extend_from_slice(value.as_bytes());
}

fn append_sorted_strings(output: &mut Vec<u8>, values: &[String]) {
    let mut sorted: Vec<&str> = values.iter().map(String::as_str).collect();
    sorted.sort_unstable();
    append_u32(output, sorted.len() as u32);
    for value in sorted {
        append_string(output, value);
    }
}

fn append_u32(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_le_bytes());
}

fn push_reason(reasons: &mut Vec<DenialReason>, reason: DenialReason) {
    if !reasons.contains(&reason) {
        reasons.push(reason);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct AcceptProfiles {
        evidence_until_ms: u64,
    }
    impl ProfileVerifier for AcceptProfiles {
        fn verify_profile(
            &self,
            _profile: &RegionalProfile,
            _now_ms: u64,
        ) -> Result<u64, ProfileVerificationFailure> {
            Ok(self.evidence_until_ms)
        }
    }

    struct RejectProfiles;
    impl ProfileVerifier for RejectProfiles {
        fn verify_profile(
            &self,
            _profile: &RegionalProfile,
            _now_ms: u64,
        ) -> Result<u64, ProfileVerificationFailure> {
            Err(ProfileVerificationFailure::InvalidSignature)
        }
    }

    struct AcceptClassification {
        evidence_until_ms: u64,
    }
    impl ClassificationVerifier for AcceptClassification {
        fn verify_classification(
            &self,
            _claim: &ClassificationClaim,
            _now_ms: u64,
        ) -> Result<u64, ClassificationVerificationFailure> {
            Ok(self.evidence_until_ms)
        }
    }

    struct RejectAgreementVerifier {
        failure: AgreementVerificationFailure,
    }
    impl AgreementVerifier for RejectAgreementVerifier {
        fn verify_agreement(
            &self,
            _agreement: &FederationAgreement,
            _now_ms: u64,
        ) -> Result<u64, AgreementVerificationFailure> {
            Err(self.failure.clone())
        }
    }

    struct AcceptAgreements {
        evidence_until_ms: u64,
    }
    impl AgreementVerifier for AcceptAgreements {
        fn verify_agreement(
            &self,
            _agreement: &FederationAgreement,
            _now_ms: u64,
        ) -> Result<u64, AgreementVerificationFailure> {
            Ok(self.evidence_until_ms)
        }
    }

    struct AcceptConsent {
        evidence_until_ms: u64,
    }
    impl ConsentVerifier for AcceptConsent {
        fn verify_current_grant(
            &self,
            _grant: &ConsentGrant,
            _now_ms: u64,
        ) -> Result<u64, ConsentVerificationFailure> {
            Ok(self.evidence_until_ms)
        }
    }

    fn rule(
        allowed_destination_scopes: &[&str],
        allowed_source_scopes: &[&str],
        consent_required: bool,
    ) -> PlacementRule {
        PlacementRule {
            data_class: DataClass::Personal,
            allowed_actions: vec![BoundaryAction::Disclose, BoundaryAction::Backup],
            allowed_destination_scopes: allowed_destination_scopes
                .iter()
                .map(|scope| (*scope).to_owned())
                .collect(),
            allowed_source_scopes: allowed_source_scopes
                .iter()
                .map(|scope| (*scope).to_owned())
                .collect(),
            purpose_required: true,
            recipient_required: true,
            consent_required,
        }
    }

    fn profile(
        scope: &str,
        destinations: &[&str],
        sources: &[&str],
    ) -> RegionalProfile {
        RegionalProfile {
            schema_version: SUPPORTED_SCHEMA_VERSION,
            profile_id: format!("profile-{scope}"),
            profile_version: "1.0.0".to_owned(),
            scope_id: scope.to_owned(),
            valid_from_unix_ms: 100,
            valid_until_unix_ms: 1_000,
            rules: vec![rule(destinations, sources, true)],
            content_digest: "a".repeat(64),
            signing_key_id: "test-key".to_owned(),
            signature: vec![1; 64],
        }
    }

    fn verified(scope: &str, destinations: &[&str], sources: &[&str]) -> VerifiedProfile {
        VerifiedProfile::verify(
            profile(scope, destinations, sources),
            200,
            &AcceptProfiles { evidence_until_ms: 900 },
        )
        .expect("fixture profile should verify through test adapter")
    }

    const TEST_CONTENT_DIGEST: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    fn verified_agreement(
        source: &VerifiedProfile,
        destination: &VerifiedProfile,
    ) -> VerifiedFederationAgreement {
        let agreement = FederationAgreement {
            schema_version: 1,
            agreement_id: "agreement-community-a-to-b".to_owned(),
            agreement_version: "1.0.0".to_owned(),
            source_profile_id: source.profile().profile_id.clone(),
            source_profile_digest: source.profile().content_digest.clone(),
            destination_profile_id: destination.profile().profile_id.clone(),
            destination_profile_digest: destination.profile().content_digest.clone(),
            valid_from_unix_ms: 100,
            valid_until_unix_ms: 900,
            allowed_data_classes: vec![DataClass::Personal],
            allowed_actions: vec![BoundaryAction::Disclose, BoundaryAction::Backup],
            allowed_purposes: vec!["credential-check".to_owned()],
            allowed_recipient_ids: vec!["service:education".to_owned()],
            content_digest: "c".repeat(64),
            source_approval_key_id: "source-agreement-test-key".to_owned(),
            source_approval_signature: vec![2; 64],
            destination_approval_key_id: "destination-agreement-test-key".to_owned(),
            destination_approval_signature: vec![3; 64],
        };
        VerifiedFederationAgreement::verify(
            agreement,
            200,
            &AcceptAgreements { evidence_until_ms: 850 },
        )
        .expect("fixture agreement should verify through test adapter")
    }

    fn verified_classification(resource_id: &str, now_ms: u64) -> VerifiedClassification {
        VerifiedClassification::verify(
            ClassificationClaim {
                schema_version: 1,
                resource_id: resource_id.to_owned(),
                subject_id: Some("did:example:subject-1".to_owned()),
                content_digest: TEST_CONTENT_DIGEST.to_owned(),
                data_class: DataClass::Personal,
                valid_from_unix_ms: 150,
                valid_until_unix_ms: 500,
                issuer_id: "issuer:classification-authority".to_owned(),
                signing_key_id: "classification-test-key".to_owned(),
                claim_digest: "d".repeat(64),
                signature: vec![4; 64],
            },
            now_ms,
            &AcceptClassification { evidence_until_ms: 300 },
        )
        .expect("fixture classification should verify through test adapter")
    }

    fn verified_consent(
        recipient: &str,
        purpose: &str,
        valid_from_ms: u64,
        valid_until_ms: u64,
        evidence_until_ms: u64,
        now_ms: u64,
    ) -> VerifiedConsentGrant {
        VerifiedConsentGrant::verify(
            ConsentGrant {
                schema_version: 1,
                grant_id: "grant-1".to_owned(),
                subject_id: "did:example:subject-1".to_owned(),
                resource_id: "record:education-credential".to_owned(),
                content_digest: TEST_CONTENT_DIGEST.to_owned(),
                data_class: DataClass::Personal,
                action: BoundaryAction::Disclose,
                recipient_id: recipient.to_owned(),
                purpose: purpose.to_owned(),
                valid_from_unix_ms: valid_from_ms,
                valid_until_unix_ms: valid_until_ms,
                issuer_id: "did:example:subject-1".to_owned(),
                signing_key_id: "consent-test-key".to_owned(),
                grant_digest: "e".repeat(64),
                signature: vec![5; 64],
            },
            now_ms,
            &AcceptConsent {
                evidence_until_ms,
            },
        )
        .expect("fixture consent should verify through test adapter")
    }

    #[test]
    fn permits_only_explicitly_authorized_bilateral_route() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent("service:education", "credential-check", 150, 500, 300, 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(decision.allowed, "unexpected denial: {:?}", decision.reasons);
        assert!(decision.reasons.is_empty());
    }

    #[test]
    fn denies_asymmetric_destination_policy() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &[]);
        let classification = verified_classification("record:education-credential", 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::SourceNotAllowedByDestination));
    }

    #[test]
    fn denies_missing_consent_even_when_route_is_allowed() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ConsentRequired));
    }

    #[test]
    fn denies_consent_bound_to_another_recipient_or_purpose() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent("service:other", "marketing", 150, 500, 300, 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ConsentRecipientMismatch));
        assert!(decision.reasons.contains(&DenialReason::ConsentPurposeMismatch));
    }

    #[test]
    fn denies_when_revocation_evidence_is_stale() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent("service:education", "credential-check", 150, 500, 220, 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ConsentEvidenceStale));
    }

    #[test]
    fn denies_after_profile_expires() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 1_000,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::SourceProfileInactive));
        assert!(decision.reasons.contains(&DenialReason::DestinationProfileInactive));
    }

    #[test]
    fn rejects_profile_that_trusted_adapter_does_not_verify() {
        let result = VerifiedProfile::verify(
            profile("community-a", &["community-b"], &[]),
            200,
            &RejectProfiles,
        );
        assert_eq!(
            result,
            Err(ProfileValidationError::VerificationFailed(
                ProfileVerificationFailure::InvalidSignature
            ))
        );
    }

    #[test]
    fn rejects_unsupported_profile_schema_before_verification() {
        let mut invalid = profile("community-a", &["community-b"], &[]);
        invalid.schema_version = SUPPORTED_SCHEMA_VERSION + 1;
        assert_eq!(
            invalid.validate_shape(),
            Err(ProfileValidationError::UnsupportedSchemaVersion)
        );
    }

    #[test]
    fn canonical_payload_is_independent_of_set_order() {
        let mut first = profile("community-a", &["community-b", "community-c"], &[]);
        first.rules[0].allowed_actions = vec![BoundaryAction::Disclose, BoundaryAction::Backup];
        let mut second = profile("community-a", &["community-c", "community-b"], &[]);
        second.rules[0].allowed_actions = vec![BoundaryAction::Backup, BoundaryAction::Disclose];
        assert_eq!(first.canonical_payload(), second.canonical_payload());
    }

    #[test]
    fn rejects_duplicate_rule_classes_and_duplicate_scopes() {
        let mut duplicate_class = profile("community-a", &["community-b"], &[]);
        duplicate_class.rules.push(duplicate_class.rules[0].clone());
        assert_eq!(
            duplicate_class.validate_shape(),
            Err(ProfileValidationError::DuplicateDataClassRule)
        );

        let mut duplicate_scope = profile("community-a", &["community-b", "community-b"], &[]);
        assert_eq!(
            duplicate_scope.validate_shape(),
            Err(ProfileValidationError::DuplicateScope)
        );
    }

    #[test]
    fn rejects_malformed_digest_and_empty_signature() {
        let mut bad_digest = profile("community-a", &["community-b"], &[]);
        bad_digest.content_digest = "not-a-digest".to_owned();
        assert_eq!(
            bad_digest.validate_shape(),
            Err(ProfileValidationError::InvalidDigestFormat)
        );

        let mut bad_signature = profile("community-a", &["community-b"], &[]);
        bad_signature.signature.clear();
        assert_eq!(
            bad_signature.validate_shape(),
            Err(ProfileValidationError::InvalidSignatureLength)
        );
    }

    #[test]
    fn missing_purpose_and_recipient_are_denied() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent("service:education", "credential-check", 150, 500, 300, 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: None,
            recipient_id: None,
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::PurposeRequired));
        assert!(decision.reasons.contains(&DenialReason::RecipientRequired));
        assert!(decision.reasons.contains(&DenialReason::ConsentRecipientMismatch));
        assert!(decision.reasons.contains(&DenialReason::ConsentPurposeMismatch));
    }
    #[test]
    fn denies_transfer_of_a_different_object_than_the_classification_claim() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:other", 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ClassificationSubjectMismatch));
    }

    #[test]
    fn denies_changed_content_after_classification() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let changed_digest = "c".repeat(64);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: &changed_digest,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ClassificationContentMismatch));
    }

    #[test]
    fn denies_when_classification_evidence_is_stale() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 300,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ClassificationEvidenceStale));
    }

    #[test]
    fn rejects_backdated_evaluation_after_profile_was_verified() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 199,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::SourceProfileVerifiedAfterDecisionTime));
        assert!(decision.reasons.contains(&DenialReason::DestinationProfileVerifiedAfterDecisionTime));
        assert!(decision.reasons.contains(&DenialReason::ClassificationVerifiedAfterDecisionTime));
        assert!(decision.reasons.contains(&DenialReason::AgreementVerifiedAfterDecisionTime));
    }

    #[test]
    fn includes_versioned_non_sensitive_policy_audit_context() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent("service:education", "credential-check", 150, 500, 300, 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(decision.allowed);
        assert_eq!(decision.source_profile_id, "profile-community-a");
        assert_eq!(decision.source_profile_version, "1.0.0");
        assert_eq!(decision.source_profile_digest, "a".repeat(64));
        assert_eq!(decision.destination_profile_id, "profile-community-b");
        assert_eq!(decision.action, BoundaryAction::Disclose);
        assert_eq!(decision.data_class, DataClass::Personal);
        assert_eq!(decision.content_digest, TEST_CONTENT_DIGEST);
        assert_eq!(decision.evaluated_at_unix_ms, 220);
    }

    #[test]
    fn bilateral_route_truth_table_only_allows_mutual_permission() {
        for source_allows in [false, true] {
            for destination_allows in [false, true] {
                let source_destinations: &[&str] = if source_allows {
                    &["community-b"]
                } else {
                    &[]
                };
                let destination_sources: &[&str] = if destination_allows {
                    &["community-a"]
                } else {
                    &[]
                };
                let source = verified("community-a", source_destinations, &[]);
                let destination = verified("community-b", &[], destination_sources);
                let classification = verified_classification("record:education-credential", 200);
                let consent = verified_consent(
                    "service:education",
                    "credential-check",
                    150,
                    500,
                    300,
                    200,
                );
                let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
                    action: BoundaryAction::Disclose,
                    resource_id: "record:education-credential",
                    content_digest: TEST_CONTENT_DIGEST,
                    classification: &classification,
            agreement: &agreement,
                    purpose: Some("credential-check"),
                    recipient_id: Some("service:education"),
                    consent: Some(&consent),
                    evaluated_at_unix_ms: 220,
                };
                let decision = evaluate_transfer(&source, &destination, &request);
                assert_eq!(
                    decision.allowed,
                    source_allows && destination_allows,
                    "source_allows={source_allows}, destination_allows={destination_allows}, reasons={:?}",
                    decision.reasons
                );
            }
        }
    }

    #[test]
    fn rejects_backdated_evaluation_after_consent_was_verified() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent(
            "service:education",
            "credential-check",
            150,
            500,
            300,
            220,
        );
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 219,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ConsentVerifiedAfterDecisionTime));
    }

    #[test]
    fn binds_policy_decision_to_profile_versions_and_non_sensitive_audit_fields() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent(
            "service:education",
            "credential-check",
            150,
            500,
            300,
            200,
        );
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(decision.allowed);
        assert_eq!(decision.agreement_id, "agreement-community-a-to-b");
        assert_eq!(decision.agreement_version, "1.0.0");
        assert_eq!(decision.agreement_digest, "c".repeat(64));
        assert_eq!(decision.source_profile_id, "profile-community-a");
        assert_eq!(decision.source_profile_version, "1.0.0");
        assert_eq!(decision.source_profile_digest, "a".repeat(64));
        assert_eq!(decision.destination_profile_id, "profile-community-b");
        assert_eq!(decision.destination_profile_digest, "a".repeat(64));
        assert_eq!(decision.action, BoundaryAction::Disclose);
        assert_eq!(decision.data_class, DataClass::Personal);
        assert_eq!(decision.content_digest, TEST_CONTENT_DIGEST);
        assert_eq!(decision.evaluated_at_unix_ms, 220);
    }

    #[test]
    fn denies_profile_when_trust_root_evidence_deadline_is_stale() {
        let result = VerifiedProfile::verify(
            profile("community-a", &["community-b"], &[]),
            200,
            &AcceptProfiles { evidence_until_ms: 200 },
        );
        assert_eq!(
            result,
            Err(ProfileValidationError::VerificationFailed(
                ProfileVerificationFailure::StaleTrustRoots
            ))
        );
    }

    #[test]
    fn denies_when_profile_trust_evidence_expires_before_profile_document() {
        let source = VerifiedProfile::verify(
            profile("community-a", &["community-b"], &[]),
            200,
            &AcceptProfiles { evidence_until_ms: 220 },
        )
        .expect("profile is active and its trust evidence is initially fresh");
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::SourceProfileEvidenceStale));
    }

    #[test]
    fn rejects_agreement_bound_to_another_profile_version() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let mut agreement = verified_agreement(&source, &destination).agreement;
        agreement.source_profile_id = "profile-community-other".to_owned();
        let agreement = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &AcceptAgreements { evidence_until_ms: 850 },
        )
        .expect("fixture verifier accepts this profile-binding negative control");
        let classification = verified_classification("record:education-credential", 200);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::AgreementSourceProfileMismatch));
    }

    #[test]
    fn denies_action_not_in_agreement_even_if_both_profiles_allow_route() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let mut agreement = verified_agreement(&source, &destination).agreement;
        agreement.allowed_actions = vec![BoundaryAction::Backup];
        let agreement = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &AcceptAgreements { evidence_until_ms: 850 },
        )
        .expect("fixture agreement verifies");
        let classification = verified_classification("record:education-credential", 200);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::AgreementActionNotAllowed));
    }

    #[test]
    fn agreement_canonical_payload_is_independent_of_allowlist_order() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let mut first = verified_agreement(&source, &destination).agreement;
        let mut second = first.clone();
        first.allowed_purposes = vec!["credential-check".to_owned(), "care-coordination".to_owned()];
        second.allowed_purposes = vec!["care-coordination".to_owned(), "credential-check".to_owned()];
        first.allowed_recipient_ids = vec!["service:education".to_owned(), "service:care".to_owned()];
        second.allowed_recipient_ids = vec!["service:care".to_owned(), "service:education".to_owned()];
        assert_eq!(first.canonical_payload(), second.canonical_payload());
    }

    #[test]
    fn rejects_agreement_when_source_profile_digest_does_not_match() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let mut agreement = verified_agreement(&source, &destination).agreement;
        agreement.source_profile_digest = "d".repeat(64);
        let agreement = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &AcceptAgreements { evidence_until_ms: 850 },
        )
        .expect("mock verifier accepts structure for the policy-binding negative control");
        let classification = verified_classification("record:education-credential", 200);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::AgreementSourceProfileMismatch));
    }

    #[test]
    fn denies_purpose_not_explicitly_authorized_by_agreement() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let mut agreement = verified_agreement(&source, &destination).agreement;
        agreement.allowed_purposes = vec!["care-coordination".to_owned()];
        let agreement = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &AcceptAgreements { evidence_until_ms: 850 },
        )
        .expect("mock verifier accepts structure for the purpose allowlist test");
        let classification = verified_classification("record:education-credential", 200);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::AgreementPurposeNotAllowed));
    }

    #[test]
    fn denies_recipient_not_explicitly_authorized_by_agreement() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let mut agreement = verified_agreement(&source, &destination).agreement;
        agreement.allowed_recipient_ids = vec!["service:care".to_owned()];
        let agreement = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &AcceptAgreements { evidence_until_ms: 850 },
        )
        .expect("mock verifier accepts structure for the recipient allowlist test");
        let classification = verified_classification("record:education-credential", 200);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: None,
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::AgreementRecipientNotAllowed));
    }

    #[test]
    fn denies_expired_agreement_even_if_source_and_destination_profiles_remain_active() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let mut agreement = verified_agreement(&source, &destination).agreement;
        agreement.valid_until_unix_ms = 220;
        let agreement = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &AcceptAgreements { evidence_until_ms: 900 },
        )
        .expect("agreement is active at verification time");
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent(
            "service:education",
            "credential-check",
            150,
            500,
            300,
            200,
        );
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::AgreementExpired));
    }

    #[test]
    fn denies_when_agreement_trust_evidence_becomes_stale() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let agreement_raw = verified_agreement(&source, &destination).agreement;
        let agreement = VerifiedFederationAgreement::verify(
            agreement_raw,
            200,
            &AcceptAgreements { evidence_until_ms: 220 },
        )
        .expect("agreement trust evidence is initially fresh");
        let classification = verified_classification("record:education-credential", 200);
        let consent = verified_consent(
            "service:education",
            "credential-check",
            150,
            500,
            300,
            200,
        );
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::AgreementEvidenceStale));
    }

    #[test]
    fn source_and_destination_approval_messages_are_domain_separated() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let agreement = verified_agreement(&source, &destination).agreement;
        let source_message = agreement.source_approval_message();
        let destination_message = agreement.destination_approval_message();
        assert_ne!(source_message, destination_message);
        assert!(source_message.starts_with(b"MYCELIX_FEDERATION_AGREEMENT_APPROVAL_V1\0"));
        assert!(destination_message.starts_with(b"MYCELIX_FEDERATION_AGREEMENT_APPROVAL_V1\0"));
    }

    #[test]
    fn rejects_agreement_when_source_approval_verification_fails() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let agreement = verified_agreement(&source, &destination).agreement;
        let result = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &RejectAgreementVerifier {
                failure: AgreementVerificationFailure::InvalidSourceApproval,
            },
        );
        assert_eq!(
            result,
            Err(AgreementValidationError::VerificationFailed(
                AgreementVerificationFailure::InvalidSourceApproval
            ))
        );
    }

    #[test]
    fn rejects_agreement_when_destination_approval_verification_fails() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let agreement = verified_agreement(&source, &destination).agreement;
        let result = VerifiedFederationAgreement::verify(
            agreement,
            200,
            &RejectAgreementVerifier {
                failure: AgreementVerificationFailure::InvalidDestinationApproval,
            },
        );
        assert_eq!(
            result,
            Err(AgreementValidationError::VerificationFailed(
                AgreementVerificationFailure::InvalidDestinationApproval
            ))
        );
    }

    #[test]
    fn denies_consent_reused_for_another_resource_or_content_version() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let mut altered = verified_consent(
            "service:education",
            "credential-check",
            150,
            500,
            300,
            200,
        )
        .grant;
        altered.resource_id = "record:another-credential".to_owned();
        altered.content_digest = "f".repeat(64);
        let consent = VerifiedConsentGrant::verify(
            altered,
            200,
            &AcceptConsent { evidence_until_ms: 300 },
        )
        .expect("mock adapter accepts structure for scope-binding tests");
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ConsentResourceMismatch));
        assert!(decision.reasons.contains(&DenialReason::ConsentContentMismatch));
    }

    #[test]
    fn denies_consent_for_another_subject_data_class_or_action() {
        let source = verified("community-a", &["community-b"], &[]);
        let destination = verified("community-b", &[], &["community-a"]);
        let classification = verified_classification("record:education-credential", 200);
        let mut altered = verified_consent(
            "service:education",
            "credential-check",
            150,
            500,
            300,
            200,
        )
        .grant;
        altered.subject_id = "did:example:another-subject".to_owned();
        altered.data_class = DataClass::Sensitive;
        altered.action = BoundaryAction::Backup;
        let consent = VerifiedConsentGrant::verify(
            altered,
            200,
            &AcceptConsent { evidence_until_ms: 300 },
        )
        .expect("mock adapter accepts structure for scope-binding tests");
        let agreement = verified_agreement(&source, &destination);
        let request = TransferRequest {
            action: BoundaryAction::Disclose,
            resource_id: "record:education-credential",
            content_digest: TEST_CONTENT_DIGEST,
            classification: &classification,
            agreement: &agreement,
            purpose: Some("credential-check"),
            recipient_id: Some("service:education"),
            consent: Some(&consent),
            evaluated_at_unix_ms: 220,
        };
        let decision = evaluate_transfer(&source, &destination, &request);
        assert!(!decision.allowed);
        assert!(decision.reasons.contains(&DenialReason::ConsentSubjectMismatch));
        assert!(decision.reasons.contains(&DenialReason::ConsentDataClassMismatch));
        assert!(decision.reasons.contains(&DenialReason::ConsentActionMismatch));
    }

}
