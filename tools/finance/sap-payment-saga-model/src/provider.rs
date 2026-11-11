//! Provider-specific idempotency and refund contract reference profile.
//!
//! This effects-disabled module converts provider documentation into explicit,
//! versioned retry/absence/refund constraints. Digests identify subjects; they
//! do not authenticate provider documents, webhooks or settlement evidence.

use sha2::{Digest, Sha256};

const CONTRACT_DOMAIN: &[u8] = b"MXSAP-PROVIDER-CONTRACT-V1\0";
const OPERATION_KEY_DOMAIN: &[u8] = b"MXSAP-PROVIDER-OPERATION-KEY-V1\0";
const OBSERVATION_DOMAIN: &[u8] = b"MXSAP-PROVIDER-OBSERVATION-V1\0";
const ABSENCE_DOMAIN: &[u8] = b"MXSAP-PROVIDER-ABSENCE-EVIDENCE-V1\0";
const MAX_FIELD_BYTES: usize = 4096;

const DAY_MS: u64 = 24 * 60 * 60 * 1_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderKeyScopeV1 {
    CompanyAccount = 1,
    MerchantAccount = 2,
    PaymentOperation = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderRegionScopeV1 {
    Global = 1,
    PerRegion = 2,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderProfileMaturityV1 {
    DocumentationBaseline = 1,
    SandboxQualified = 2,
    ProductionQualified = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderChangedPayloadBehaviorV1 {
    RejectConflict = 1,
    ReplayOriginalResponse = 2,
    Unspecified = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderInProgressBehaviorV1 {
    RetryOnlyWithTransientErrorSignal = 1,
    NeverRetryWhileInProgress = 2,
    Unspecified = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderRefundCeilingV1 {
    CapturedAmountAggregate = 1,
    ProviderReportedRemaining = 2,
    Unknown = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderAbsencePolicyV1 {
    NotSupported = 1,
    CompleteLookupAfterHorizon = 2,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderAttemptClassV1 {
    TransportTimeoutUnknown = 1,
    InProgressWithTransientSignal = 2,
    InProgressWithoutTransientSignal = 3,
    DefinitiveNoEffect = 4,
    PermanentError = 5,
    Accepted = 6,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ProviderStatusV1 {
    Unknown = 0,
    InProgress = 1,
    Accepted = 2,
    Settled = 3,
    Rejected = 4,
    Returned = 5,
    Reversed = 6,
    AuthoritativelyAbsent = 7,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProviderRetryErrorV1 {
    InvalidContract,
    InvalidOperationKey,
    ProviderScopeMismatch,
    RegionMismatch,
    ChangedRequestDigest,
    ClockMovedBackwards,
    RetentionGuaranteeElapsed,
    ProviderDoesNotSupportIdempotentPost,
    InProgressNotRetryable,
    AttemptClassNotRetryable,
    ProfileNotQualified,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProviderObservationErrorV1 {
    InvalidSubject,
    InvalidDigest,
    ScopeMismatch,
    InvalidTimestamp,
    AbsenceNotSupported,
    AbsenceHorizonNotElapsed,
    ObservationAfterNow,
    InvalidTransition,
    TerminalContradiction,
    ChangedObservation,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProviderContractV1 {
    pub provider_id: String,
    pub profile_id: String,
    pub api_version: String,
    pub account_scope: String,
    pub key_scope: ProviderKeyScopeV1,
    pub region_scope: ProviderRegionScopeV1,
    pub maximum_key_characters: u16,
    pub idempotent_post: bool,
    /// Guaranteed minimum retention. Never assume the key remains stored after it.
    pub retention_min_ms: u64,
    /// Published/observed upper end of the retention window.
    pub retention_max_ms: u64,
    pub in_progress_behavior: ProviderInProgressBehaviorV1,
    pub changed_payload_behavior: ProviderChangedPayloadBehaviorV1,
    pub partial_refunds_allowed: bool,
    pub refund_ceiling: ProviderRefundCeilingV1,
    pub absence_policy: ProviderAbsencePolicyV1,
    pub absence_visibility_horizon_ms: u64,
    pub profile_maturity: ProviderProfileMaturityV1,
    pub webhook_auth_profile: String,
}

impl ProviderContractV1 {
    pub fn validate(&self) -> Result<(), ProviderObservationErrorV1> {
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.webhook_auth_profile,
        ] {
            validate_text(value).map_err(|_| ProviderObservationErrorV1::InvalidSubject)?;
        }
        if self.idempotent_post {
            if self.maximum_key_characters == 0
                || self.retention_min_ms == 0
                || self.retention_max_ms < self.retention_min_ms
            {
                return Err(ProviderObservationErrorV1::InvalidSubject);
            }
        } else if self.retention_min_ms != 0 || self.retention_max_ms != 0 {
            return Err(ProviderObservationErrorV1::InvalidSubject);
        }
        match self.absence_policy {
            ProviderAbsencePolicyV1::NotSupported
                if self.absence_visibility_horizon_ms != 0 =>
            {
                return Err(ProviderObservationErrorV1::InvalidSubject);
            }
            ProviderAbsencePolicyV1::CompleteLookupAfterHorizon
                if self.absence_visibility_horizon_ms == 0 =>
            {
                return Err(ProviderObservationErrorV1::InvalidSubject);
            }
            _ => {}
        }
        Ok(())
    }

    pub fn canonical_digest(&self) -> Result<String, ProviderObservationErrorV1> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(CONTRACT_DOMAIN);
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
        ] {
            append_text(&mut bytes, value)?;
        }
        bytes.push(self.key_scope as u8);
        bytes.push(self.region_scope as u8);
        append_u64(&mut bytes, u64::from(self.maximum_key_characters));
        bytes.push(u8::from(self.idempotent_post));
        append_u64(&mut bytes, self.retention_min_ms);
        append_u64(&mut bytes, self.retention_max_ms);
        bytes.push(self.in_progress_behavior as u8);
        bytes.push(self.changed_payload_behavior as u8);
        bytes.push(u8::from(self.partial_refunds_allowed));
        bytes.push(self.refund_ceiling as u8);
        bytes.push(self.absence_policy as u8);
        append_u64(&mut bytes, self.absence_visibility_horizon_ms);
        bytes.push(self.profile_maturity as u8);
        append_text(&mut bytes, &self.webhook_auth_profile)?;
        Ok(sha256_hex(&bytes))
    }

    /// Public-documentation baseline for Adyen's published idempotency behavior.
    ///
    /// API version, webhook verification and authoritative-absence semantics
    /// are intentionally unpinned. This profile must not authorize live dispatch.
    pub fn adyen_documentation_baseline_v1(account_scope: impl Into<String>) -> Self {
        Self {
            provider_id: "adyen".into(),
            profile_id: "adyen-idempotency-docs-2026-11-11".into(),
            api_version: "operator-pinned-required".into(),
            account_scope: account_scope.into(),
            key_scope: ProviderKeyScopeV1::CompanyAccount,
            region_scope: ProviderRegionScopeV1::PerRegion,
            maximum_key_characters: 64,
            idempotent_post: true,
            retention_min_ms: 7 * DAY_MS,
            retention_max_ms: 14 * DAY_MS,
            in_progress_behavior:
                ProviderInProgressBehaviorV1::RetryOnlyWithTransientErrorSignal,
            changed_payload_behavior: ProviderChangedPayloadBehaviorV1::Unspecified,
            partial_refunds_allowed: true,
            refund_ceiling: ProviderRefundCeilingV1::CapturedAmountAggregate,
            absence_policy: ProviderAbsencePolicyV1::NotSupported,
            absence_visibility_horizon_ms: 0,
            profile_maturity: ProviderProfileMaturityV1::DocumentationBaseline,
            webhook_auth_profile: "operator-pinned-required".into(),
        }
    }

    pub fn require_live_dispatch(&self) -> Result<(), ProviderRetryErrorV1> {
        self.validate().map_err(|_| ProviderRetryErrorV1::InvalidContract)?;
        if self.profile_maturity != ProviderProfileMaturityV1::ProductionQualified
            || self.api_version == "operator-pinned-required"
            || self.webhook_auth_profile == "operator-pinned-required"
        {
            return Err(ProviderRetryErrorV1::ProfileNotQualified);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProviderOperationKeyV1 {
    pub provider_id: String,
    pub profile_id: String,
    pub api_version: String,
    pub account_scope: String,
    pub region: String,
    pub idempotency_key: String,
    pub request_digest: String,
    pub created_at_unix_ms: u64,
    pub contract_digest: String,
}

impl ProviderOperationKeyV1 {
    pub fn validate(
        &self,
        contract: &ProviderContractV1,
    ) -> Result<(), ProviderRetryErrorV1> {
        contract
            .validate()
            .map_err(|_| ProviderRetryErrorV1::InvalidContract)?;
        for (actual, expected) in [
            (&self.provider_id, &contract.provider_id),
            (&self.profile_id, &contract.profile_id),
            (&self.api_version, &contract.api_version),
            (&self.account_scope, &contract.account_scope),
        ] {
            if actual != expected {
                return Err(ProviderRetryErrorV1::ProviderScopeMismatch);
            }
        }
        let expected_region_shape = match contract.region_scope {
            ProviderRegionScopeV1::Global => self.region == "global",
            ProviderRegionScopeV1::PerRegion => !self.region.is_empty(),
        };
        if !expected_region_shape {
            return Err(ProviderRetryErrorV1::RegionMismatch);
        }
        if self.idempotency_key.is_empty()
            || self.idempotency_key.chars().count()
                > usize::from(contract.maximum_key_characters)
            || self.created_at_unix_ms == 0
            || !valid_digest(&self.request_digest)
            || self.contract_digest != contract.canonical_digest().map_err(|_| {
                ProviderRetryErrorV1::InvalidContract
            })?
        {
            return Err(ProviderRetryErrorV1::InvalidOperationKey);
        }
        Ok(())
    }

    pub fn canonical_digest(&self) -> Result<String, ProviderRetryErrorV1> {
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
        ] {
            validate_text(value).map_err(|_| ProviderRetryErrorV1::InvalidOperationKey)?;
        }
        if !valid_digest(&self.request_digest) || !valid_digest(&self.contract_digest) {
            return Err(ProviderRetryErrorV1::InvalidOperationKey);
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(OPERATION_KEY_DOMAIN);
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
            &self.request_digest,
            &self.contract_digest,
        ] {
            append_text(&mut bytes, value)
                .map_err(|_| ProviderRetryErrorV1::InvalidOperationKey)?;
        }
        append_u64(&mut bytes, self.created_at_unix_ms);
        Ok(sha256_hex(&bytes))
    }

    pub fn retry_same_request(
        &self,
        contract: &ProviderContractV1,
        current_account_scope: &str,
        current_region: &str,
        current_request_digest: &str,
        attempt: ProviderAttemptClassV1,
        now_unix_ms: u64,
    ) -> Result<(), ProviderRetryErrorV1> {
        self.validate(contract)?;
        if !contract.idempotent_post {
            return Err(ProviderRetryErrorV1::ProviderDoesNotSupportIdempotentPost);
        }
        if current_account_scope != self.account_scope {
            return Err(ProviderRetryErrorV1::ProviderScopeMismatch);
        }
        if contract.region_scope == ProviderRegionScopeV1::PerRegion
            && current_region != self.region
        {
            return Err(ProviderRetryErrorV1::RegionMismatch);
        }
        if current_request_digest != self.request_digest {
            return Err(ProviderRetryErrorV1::ChangedRequestDigest);
        }
        let elapsed = now_unix_ms
            .checked_sub(self.created_at_unix_ms)
            .ok_or(ProviderRetryErrorV1::ClockMovedBackwards)?;
        // Retention is deliberately bounded by the published minimum, not
        // the upper end of a range. After the minimum, presence is uncertain.
        if elapsed >= contract.retention_min_ms {
            return Err(ProviderRetryErrorV1::RetentionGuaranteeElapsed);
        }
        match attempt {
            ProviderAttemptClassV1::TransportTimeoutUnknown => Ok(()),
            ProviderAttemptClassV1::InProgressWithTransientSignal
                if contract.in_progress_behavior
                    == ProviderInProgressBehaviorV1::RetryOnlyWithTransientErrorSignal =>
            {
                Ok(())
            }
            ProviderAttemptClassV1::InProgressWithoutTransientSignal => {
                Err(ProviderRetryErrorV1::InProgressNotRetryable)
            }
            ProviderAttemptClassV1::InProgressWithTransientSignal => {
                Err(ProviderRetryErrorV1::InProgressNotRetryable)
            }
            _ => Err(ProviderRetryErrorV1::AttemptClassNotRetryable),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProviderObservationV1 {
    pub provider_id: String,
    pub profile_id: String,
    pub api_version: String,
    pub account_scope: String,
    pub region: String,
    pub idempotency_key: String,
    pub request_digest: String,
    pub status: ProviderStatusV1,
    pub provider_object_id: String,
    pub event_id: String,
    pub observed_at_unix_ms: u64,
    pub evidence_reference: String,
    pub evidence_digest: String,
}

impl ProviderObservationV1 {
    pub fn validate_against_key(
        &self,
        contract: &ProviderContractV1,
        key: &ProviderOperationKeyV1,
        now_unix_ms: u64,
    ) -> Result<(), ProviderObservationErrorV1> {
        key.validate(contract)
            .map_err(|_| ProviderObservationErrorV1::ScopeMismatch)?;
        if self.provider_id != key.provider_id
            || self.profile_id != key.profile_id
            || self.api_version != key.api_version
            || self.account_scope != key.account_scope
            || self.region != key.region
            || self.idempotency_key != key.idempotency_key
            || self.request_digest != key.request_digest
        {
            return Err(ProviderObservationErrorV1::ScopeMismatch);
        }
        if self.observed_at_unix_ms == 0 || self.observed_at_unix_ms > now_unix_ms {
            return Err(ProviderObservationErrorV1::InvalidTimestamp);
        }
        if self.status == ProviderStatusV1::AuthoritativelyAbsent {
            return Err(ProviderObservationErrorV1::AbsenceNotSupported);
        }
        if !valid_digest(&self.request_digest) || !valid_digest(&self.evidence_digest) {
            return Err(ProviderObservationErrorV1::InvalidDigest);
        }
        for value in [
            &self.provider_object_id,
            &self.event_id,
            &self.evidence_reference,
        ] {
            validate_text(value).map_err(|_| ProviderObservationErrorV1::InvalidSubject)?;
        }
        Ok(())
    }

    pub fn canonical_digest(&self) -> Result<String, ProviderObservationErrorV1> {
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
            &self.provider_object_id,
            &self.event_id,
            &self.evidence_reference,
        ] {
            validate_text(value)?;
        }
        if !valid_digest(&self.request_digest) || !valid_digest(&self.evidence_digest) {
            return Err(ProviderObservationErrorV1::InvalidDigest);
        }
        if self.observed_at_unix_ms == 0
            || self.status == ProviderStatusV1::AuthoritativelyAbsent
        {
            return Err(ProviderObservationErrorV1::InvalidSubject);
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(OBSERVATION_DOMAIN);
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
            &self.request_digest,
            &self.provider_object_id,
            &self.event_id,
            &self.evidence_reference,
            &self.evidence_digest,
        ] {
            append_text(&mut bytes, value)?;
        }
        bytes.push(self.status as u8);
        append_u64(&mut bytes, self.observed_at_unix_ms);
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProviderAbsenceEvidenceV1 {
    pub provider_id: String,
    pub profile_id: String,
    pub api_version: String,
    pub account_scope: String,
    pub region: String,
    pub idempotency_key: String,
    pub request_digest: String,
    pub lookup_reference: String,
    pub complete_through_unix_ms: u64,
    pub observed_at_unix_ms: u64,
    pub evidence_digest: String,
}

impl ProviderAbsenceEvidenceV1 {
    pub fn canonical_digest(&self) -> Result<String, ProviderObservationErrorV1> {
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
            &self.lookup_reference,
        ] {
            validate_text(value)?;
        }
        if !valid_digest(&self.request_digest) || !valid_digest(&self.evidence_digest) {
            return Err(ProviderObservationErrorV1::InvalidDigest);
        }
        if self.complete_through_unix_ms == 0 || self.observed_at_unix_ms == 0 {
            return Err(ProviderObservationErrorV1::InvalidTimestamp);
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(ABSENCE_DOMAIN);
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
            &self.request_digest,
            &self.lookup_reference,
            &self.evidence_digest,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.complete_through_unix_ms);
        append_u64(&mut bytes, self.observed_at_unix_ms);
        Ok(sha256_hex(&bytes))
    }

    pub fn validate_as_authoritative_absence(
        &self,
        contract: &ProviderContractV1,
        key: &ProviderOperationKeyV1,
        now_unix_ms: u64,
    ) -> Result<(), ProviderObservationErrorV1> {
        key.validate(contract)
            .map_err(|_| ProviderObservationErrorV1::ScopeMismatch)?;
        if contract.absence_policy != ProviderAbsencePolicyV1::CompleteLookupAfterHorizon {
            return Err(ProviderObservationErrorV1::AbsenceNotSupported);
        }
        if self.provider_id != key.provider_id
            || self.profile_id != key.profile_id
            || self.api_version != key.api_version
            || self.account_scope != key.account_scope
            || self.region != key.region
            || self.idempotency_key != key.idempotency_key
            || self.request_digest != key.request_digest
        {
            return Err(ProviderObservationErrorV1::ScopeMismatch);
        }
        if self.observed_at_unix_ms == 0
            || self.observed_at_unix_ms > now_unix_ms
            || self.complete_through_unix_ms > self.observed_at_unix_ms
        {
            return Err(ProviderObservationErrorV1::ObservationAfterNow);
        }
        let required_through = key
            .created_at_unix_ms
            .checked_add(contract.absence_visibility_horizon_ms)
            .ok_or(ProviderObservationErrorV1::InvalidTimestamp)?;
        if self.complete_through_unix_ms < required_through {
            return Err(ProviderObservationErrorV1::AbsenceHorizonNotElapsed);
        }
        if !valid_digest(&self.evidence_digest) {
            return Err(ProviderObservationErrorV1::InvalidDigest);
        }
        validate_text(&self.lookup_reference)
            .map_err(|_| ProviderObservationErrorV1::InvalidSubject)?;
        Ok(())
    }
}

pub fn validate_provider_transition_v1(
    previous: ProviderStatusV1,
    next: ProviderStatusV1,
    same_observation_digest: bool,
) -> Result<(), ProviderObservationErrorV1> {
    if previous == next && same_observation_digest {
        return Ok(());
    }
    let allowed = matches!(
        (previous, next),
        (ProviderStatusV1::Unknown, ProviderStatusV1::InProgress)
            | (ProviderStatusV1::Unknown, ProviderStatusV1::Accepted)
            | (ProviderStatusV1::Unknown, ProviderStatusV1::Settled)
            | (ProviderStatusV1::Unknown, ProviderStatusV1::Rejected)
            | (ProviderStatusV1::InProgress, ProviderStatusV1::Accepted)
            | (ProviderStatusV1::InProgress, ProviderStatusV1::Settled)
            | (ProviderStatusV1::InProgress, ProviderStatusV1::Rejected)
            | (ProviderStatusV1::Accepted, ProviderStatusV1::Settled)
            | (ProviderStatusV1::Accepted, ProviderStatusV1::Rejected)
            | (ProviderStatusV1::Settled, ProviderStatusV1::Returned)
            | (ProviderStatusV1::Settled, ProviderStatusV1::Reversed)
    );
    if allowed {
        Ok(())
    } else if matches!(
        previous,
        ProviderStatusV1::Rejected
            | ProviderStatusV1::Returned
            | ProviderStatusV1::Reversed
            | ProviderStatusV1::AuthoritativelyAbsent
    ) {
        Err(ProviderObservationErrorV1::TerminalContradiction)
    } else {
        Err(ProviderObservationErrorV1::InvalidTransition)
    }
}

fn validate_text(value: &str) -> Result<(), ProviderObservationErrorV1> {
    if value.is_empty() || value.len() > MAX_FIELD_BYTES {
        return Err(ProviderObservationErrorV1::InvalidSubject);
    }
    Ok(())
}

fn valid_digest(value: &str) -> bool {
    value.len() == 71
        && value.starts_with("sha256:")
        && value[7..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn append_text(
    output: &mut Vec<u8>,
    value: &str,
) -> Result<(), ProviderObservationErrorV1> {
    validate_text(value)?;
    let length =
        u32::try_from(value.len()).map_err(|_| ProviderObservationErrorV1::InvalidSubject)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
    Ok(())
}

fn append_u64(output: &mut Vec<u8>, value: u64) {
    output.extend_from_slice(&value.to_be_bytes());
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(71);
    output.push_str("sha256:");
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}
