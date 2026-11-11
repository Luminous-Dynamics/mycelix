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
const RETRY_PERMIT_DOMAIN: &[u8] = b"MXSAP-PROVIDER-RETRY-PERMIT-V1\0";
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
pub enum ProviderPartialRefundPolicyV1 {
    Unsupported = 1,
    Supported = 2,
    PaymentMethodDependent = 3,
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
    RetryPermitMismatch,
    RetryPermitExpired,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProviderObservationErrorV1 {
    InvalidSubject,
    InvalidDigest,
    ScopeMismatch,
    InvalidTimestamp,
    AbsenceNotSupported,
    AbsenceRequiresDedicatedEvidence,
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
    pub partial_refund_policy: ProviderPartialRefundPolicyV1,
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
        if self.partial_refund_policy != ProviderPartialRefundPolicyV1::Unsupported
            && self.partial_refund_policy != ProviderPartialRefundPolicyV1::Supported
            && self.partial_refund_policy != ProviderPartialRefundPolicyV1::PaymentMethodDependent
        {
            return Err(ProviderObservationErrorV1::InvalidSubject);
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
        bytes.push(self.partial_refund_policy as u8);
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
            partial_refund_policy: ProviderPartialRefundPolicyV1::PaymentMethodDependent,
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
        if !contract.idempotent_post {
            return Err(ProviderRetryErrorV1::ProviderDoesNotSupportIdempotentPost);
        }
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

    pub fn qualify_retry(
        &self,
        contract: &ProviderContractV1,
        current_account_scope: &str,
        current_region: &str,
        current_request_digest: &str,
        attempt: ProviderAttemptClassV1,
        now_unix_ms: u64,
    ) -> Result<ProviderRetryPermitV1, ProviderRetryErrorV1> {
        self.validate(contract)?;
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
        let valid_until_unix_ms = self
            .created_at_unix_ms
            .checked_add(contract.retention_min_ms)
            .ok_or(ProviderRetryErrorV1::InvalidOperationKey)?;
        if now_unix_ms < self.created_at_unix_ms {
            return Err(ProviderRetryErrorV1::ClockMovedBackwards);
        }
        if now_unix_ms >= valid_until_unix_ms {
            return Err(ProviderRetryErrorV1::RetentionGuaranteeElapsed);
        }
        validate_retry_attempt(contract, attempt)?;
        let permit = ProviderRetryPermitV1 {
            provider_id: self.provider_id.clone(),
            profile_id: self.profile_id.clone(),
            api_version: self.api_version.clone(),
            account_scope: self.account_scope.clone(),
            region: self.region.clone(),
            idempotency_key: self.idempotency_key.clone(),
            request_digest: self.request_digest.clone(),
            contract_digest: self.contract_digest.clone(),
            operation_key_digest: self.canonical_digest()?,
            issued_at_unix_ms: now_unix_ms,
            valid_until_unix_ms,
            attempt,
        };
        permit.validate_against(self, contract, now_unix_ms)?;
        Ok(permit)
    }

    /// Compatibility predicate for callers that only need a pass/fail result.
    /// New dispatch adapters should carry and revalidate the returned permit.
    pub fn retry_same_request(
        &self,
        contract: &ProviderContractV1,
        current_account_scope: &str,
        current_region: &str,
        current_request_digest: &str,
        attempt: ProviderAttemptClassV1,
        now_unix_ms: u64,
    ) -> Result<(), ProviderRetryErrorV1> {
        self.qualify_retry(
            contract,
            current_account_scope,
            current_region,
            current_request_digest,
            attempt,
            now_unix_ms,
        )
        .map(|_| ())
    }
}


#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProviderRetryPermitV1 {
    pub provider_id: String,
    pub profile_id: String,
    pub api_version: String,
    pub account_scope: String,
    pub region: String,
    pub idempotency_key: String,
    pub request_digest: String,
    pub contract_digest: String,
    pub operation_key_digest: String,
    pub issued_at_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub attempt: ProviderAttemptClassV1,
}

impl ProviderRetryPermitV1 {
    pub fn canonical_digest(&self) -> Result<String, ProviderRetryErrorV1> {
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
        ] {
            validate_text(value).map_err(|_| ProviderRetryErrorV1::RetryPermitMismatch)?;
        }
        if !valid_digest(&self.request_digest)
            || !valid_digest(&self.contract_digest)
            || !valid_digest(&self.operation_key_digest)
            || self.issued_at_unix_ms == 0
            || self.valid_until_unix_ms <= self.issued_at_unix_ms
        {
            return Err(ProviderRetryErrorV1::RetryPermitMismatch);
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(RETRY_PERMIT_DOMAIN);
        for value in [
            &self.provider_id,
            &self.profile_id,
            &self.api_version,
            &self.account_scope,
            &self.region,
            &self.idempotency_key,
            &self.request_digest,
            &self.contract_digest,
            &self.operation_key_digest,
        ] {
            append_text(&mut bytes, value)
                .map_err(|_| ProviderRetryErrorV1::RetryPermitMismatch)?;
        }
        append_u64(&mut bytes, self.issued_at_unix_ms);
        append_u64(&mut bytes, self.valid_until_unix_ms);
        bytes.push(self.attempt as u8);
        Ok(sha256_hex(&bytes))
    }

    pub fn validate_against(
        &self,
        key: &ProviderOperationKeyV1,
        contract: &ProviderContractV1,
        now_unix_ms: u64,
    ) -> Result<(), ProviderRetryErrorV1> {
        key.validate(contract)?;
        let expected_operation_key_digest = key.canonical_digest()?;
        let expected_valid_until = key
            .created_at_unix_ms
            .checked_add(contract.retention_min_ms)
            .ok_or(ProviderRetryErrorV1::RetryPermitMismatch)?;
        if self.provider_id != key.provider_id
            || self.profile_id != key.profile_id
            || self.api_version != key.api_version
            || self.account_scope != key.account_scope
            || self.region != key.region
            || self.idempotency_key != key.idempotency_key
            || self.request_digest != key.request_digest
            || self.contract_digest != key.contract_digest
            || self.operation_key_digest != expected_operation_key_digest
            || self.valid_until_unix_ms != expected_valid_until
        {
            return Err(ProviderRetryErrorV1::RetryPermitMismatch);
        }
        if now_unix_ms < self.issued_at_unix_ms {
            return Err(ProviderRetryErrorV1::ClockMovedBackwards);
        }
        if now_unix_ms >= self.valid_until_unix_ms {
            return Err(ProviderRetryErrorV1::RetryPermitExpired);
        }
        validate_retry_attempt(contract, self.attempt)
    }
}

fn validate_retry_attempt(
    contract: &ProviderContractV1,
    attempt: ProviderAttemptClassV1,
) -> Result<(), ProviderRetryErrorV1> {
    match attempt {
        ProviderAttemptClassV1::TransportTimeoutUnknown => Ok(()),
        ProviderAttemptClassV1::InProgressWithTransientSignal
            if contract.in_progress_behavior
                == ProviderInProgressBehaviorV1::RetryOnlyWithTransientErrorSignal =>
        {
            Ok(())
        }
        ProviderAttemptClassV1::InProgressWithoutTransientSignal
        | ProviderAttemptClassV1::InProgressWithTransientSignal => {
            Err(ProviderRetryErrorV1::InProgressNotRetryable)
        }
        _ => Err(ProviderRetryErrorV1::AttemptClassNotRetryable),
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
            return Err(ProviderObservationErrorV1::AbsenceRequiresDedicatedEvidence);
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
        if self.observed_at_unix_ms == 0 {
            return Err(ProviderObservationErrorV1::InvalidSubject);
        }
        if self.status == ProviderStatusV1::AuthoritativelyAbsent {
            return Err(ProviderObservationErrorV1::AbsenceRequiresDedicatedEvidence);
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
    if next == ProviderStatusV1::AuthoritativelyAbsent {
        return Err(ProviderObservationErrorV1::AbsenceRequiresDedicatedEvidence);
    }
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


#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 100_000_000;

    fn digest(ch: char) -> String {
        format!("sha256:{}", ch.to_string().repeat(64))
    }

    fn adyen_contract() -> ProviderContractV1 {
        ProviderContractV1::adyen_documentation_baseline_v1("company:test-account")
    }

    fn operation_key(contract: &ProviderContractV1) -> ProviderOperationKeyV1 {
        ProviderOperationKeyV1 {
            provider_id: contract.provider_id.clone(),
            profile_id: contract.profile_id.clone(),
            api_version: contract.api_version.clone(),
            account_scope: contract.account_scope.clone(),
            region: "eu".into(),
            idempotency_key: "idem-00000000-0000-4000-8000-000000000001".into(),
            request_digest: digest('a'),
            created_at_unix_ms: 1_000,
            contract_digest: contract.canonical_digest().unwrap(),
        }
    }

    fn observation(key: &ProviderOperationKeyV1) -> ProviderObservationV1 {
        ProviderObservationV1 {
            provider_id: key.provider_id.clone(),
            profile_id: key.profile_id.clone(),
            api_version: key.api_version.clone(),
            account_scope: key.account_scope.clone(),
            region: key.region.clone(),
            idempotency_key: key.idempotency_key.clone(),
            request_digest: key.request_digest.clone(),
            status: ProviderStatusV1::Accepted,
            provider_object_id: "psp:123".into(),
            event_id: "event:456".into(),
            observed_at_unix_ms: 2_000,
            evidence_reference: "provider-event:456".into(),
            evidence_digest: digest('b'),
        }
    }

    fn absence_evidence(key: &ProviderOperationKeyV1) -> ProviderAbsenceEvidenceV1 {
        ProviderAbsenceEvidenceV1 {
            provider_id: key.provider_id.clone(),
            profile_id: key.profile_id.clone(),
            api_version: key.api_version.clone(),
            account_scope: key.account_scope.clone(),
            region: key.region.clone(),
            idempotency_key: key.idempotency_key.clone(),
            request_digest: key.request_digest.clone(),
            lookup_reference: "lookup:123".into(),
            complete_through_unix_ms: 3_000,
            observed_at_unix_ms: 4_000,
            evidence_digest: digest('c'),
        }
    }

    #[test]
    fn provider_contract_v1_shared_known_answer() {
        let contract = adyen_contract();
        assert_eq!(
            contract.canonical_digest().unwrap(),
            "sha256:098d8e001c102c80bb1c6fefc9251e9a002c26cde0268d2a8e96f18d1e830822"
        );
    }

    #[test]
    fn provider_operation_key_v1_shared_known_answer() {
        let contract = adyen_contract();
        let key = operation_key(&contract);
        key.validate(&contract).unwrap();
        assert_eq!(
            key.canonical_digest().unwrap(),
            "sha256:d5730c0a62d63505fdfca8d7b7e2770dfa34d548808da7cf67ef0afb4b703dc5"
        );
    }

    #[test]
    fn provider_observation_v1_shared_known_answer() {
        let contract = adyen_contract();
        let key = operation_key(&contract);
        let observed = observation(&key);
        observed.validate_against_key(&contract, &key, NOW).unwrap();
        assert_eq!(
            observed.canonical_digest().unwrap(),
            "sha256:a559853b68900442d966c46981abf17823c25e4587b7c3815280d9f68062cdfe"
        );
    }

    #[test]
    fn provider_absence_subject_v1_shared_known_answer() {
        let contract = adyen_contract();
        let key = operation_key(&contract);
        assert_eq!(
            absence_evidence(&key).canonical_digest().unwrap(),
            "sha256:a3c2dbee60d654fcaff3191dfed77b2573fe3cd9ae5b3ad3efe8a793e99f919c"
        );
        assert_eq!(
            absence_evidence(&key).validate_as_authoritative_absence(&contract, &key, NOW),
            Err(ProviderObservationErrorV1::AbsenceNotSupported)
        );
    }

    #[test]
    fn adyen_documentation_baseline_never_authorizes_live_dispatch() {
        let contract = adyen_contract();
        assert_eq!(
            contract.require_live_dispatch(),
            Err(ProviderRetryErrorV1::ProfileNotQualified)
        );
    }

    #[test]
    fn adyen_retry_is_bound_to_same_account_region_digest_and_guaranteed_retention() {
        let contract = adyen_contract();
        let key = operation_key(&contract);
        assert_eq!(
            key.retry_same_request(
                &contract,
                "company:test-account",
                "eu",
                &key.request_digest,
                ProviderAttemptClassV1::TransportTimeoutUnknown,
                key.created_at_unix_ms + 1,
            ),
            Ok(())
        );
        assert_eq!(
            key.retry_same_request(
                &contract,
                "company:test-account",
                "eu",
                &key.request_digest,
                ProviderAttemptClassV1::InProgressWithTransientSignal,
                key.created_at_unix_ms + 1,
            ),
            Ok(())
        );
        assert_eq!(
            key.retry_same_request(
                &contract,
                "company:test-account",
                "eu",
                &key.request_digest,
                ProviderAttemptClassV1::InProgressWithoutTransientSignal,
                key.created_at_unix_ms + 1,
            ),
            Err(ProviderRetryErrorV1::InProgressNotRetryable)
        );
        assert_eq!(
            key.retry_same_request(
                &contract,
                "company:test-account",
                "us",
                &key.request_digest,
                ProviderAttemptClassV1::TransportTimeoutUnknown,
                key.created_at_unix_ms + 1,
            ),
            Err(ProviderRetryErrorV1::RegionMismatch)
        );
        assert_eq!(
            key.retry_same_request(
                &contract,
                "company:other-account",
                "eu",
                &key.request_digest,
                ProviderAttemptClassV1::TransportTimeoutUnknown,
                key.created_at_unix_ms + 1,
            ),
            Err(ProviderRetryErrorV1::ProviderScopeMismatch)
        );
        assert_eq!(
            key.retry_same_request(
                &contract,
                "company:test-account",
                "eu",
                &digest('d'),
                ProviderAttemptClassV1::TransportTimeoutUnknown,
                key.created_at_unix_ms + 1,
            ),
            Err(ProviderRetryErrorV1::ChangedRequestDigest)
        );
        assert_eq!(
            key.retry_same_request(
                &contract,
                "company:test-account",
                "eu",
                &key.request_digest,
                ProviderAttemptClassV1::TransportTimeoutUnknown,
                key.created_at_unix_ms + 7 * DAY_MS,
            ),
            Err(ProviderRetryErrorV1::RetentionGuaranteeElapsed)
        );
    }

    #[test]
    fn idempotency_key_length_uses_characters_not_utf8_bytes() {
        let mut contract = adyen_contract();
        contract.maximum_key_characters = 2;
        let mut key = operation_key(&contract);
        key.idempotency_key = "éé".into();
        key.contract_digest = contract.canonical_digest().unwrap();
        assert!(key.validate(&contract).is_ok());
        key.idempotency_key = "ééé".into();
        assert_eq!(
            key.validate(&contract),
            Err(ProviderRetryErrorV1::InvalidOperationKey)
        );
    }

    #[test]
    fn authoritative_absence_requires_complete_lookup_contract_and_horizon() {
        let mut contract = adyen_contract();
        contract.profile_id = "test-provider-complete-lookup-v1".into();
        contract.absence_policy = ProviderAbsencePolicyV1::CompleteLookupAfterHorizon;
        contract.absence_visibility_horizon_ms = 2_000;
        let key = operation_key(&contract);
        let mut evidence = absence_evidence(&key);
        evidence.complete_through_unix_ms = 2_999;
        assert_eq!(
            evidence.validate_as_authoritative_absence(&contract, &key, NOW),
            Err(ProviderObservationErrorV1::AbsenceHorizonNotElapsed)
        );
        evidence.complete_through_unix_ms = 3_000;
        assert_eq!(
            evidence.validate_as_authoritative_absence(&contract, &key, NOW),
            Ok(())
        );
        evidence.region = "us".into();
        assert_eq!(
            evidence.validate_as_authoritative_absence(&contract, &key, NOW),
            Err(ProviderObservationErrorV1::ScopeMismatch)
        );
    }

    #[test]
    fn ordinary_observation_cannot_smuggle_authoritative_absence() {
        let contract = adyen_contract();
        let key = operation_key(&contract);
        let mut observed = observation(&key);
        observed.status = ProviderStatusV1::AuthoritativelyAbsent;
        assert_eq!(
            observed.validate_against_key(&contract, &key, NOW),
            Err(ProviderObservationErrorV1::AbsenceRequiresDedicatedEvidence)
        );
        assert_eq!(
            observed.canonical_digest(),
            Err(ProviderObservationErrorV1::InvalidSubject)
        );
    }

    #[test]
    fn provider_status_replay_requires_same_observation_digest() {
        assert_eq!(
            validate_provider_transition_v1(
                ProviderStatusV1::InProgress,
                ProviderStatusV1::InProgress,
                true,
            ),
            Ok(())
        );
        assert_eq!(
            validate_provider_transition_v1(
                ProviderStatusV1::InProgress,
                ProviderStatusV1::InProgress,
                false,
            ),
            Err(ProviderObservationErrorV1::InvalidTransition)
        );
        assert_eq!(
            validate_provider_transition_v1(
                ProviderStatusV1::Settled,
                ProviderStatusV1::InProgress,
                false,
            ),
            Err(ProviderObservationErrorV1::InvalidTransition)
        );
        assert_eq!(
            validate_provider_transition_v1(
                ProviderStatusV1::Rejected,
                ProviderStatusV1::Settled,
                false,
            ),
            Err(ProviderObservationErrorV1::TerminalContradiction)
        );
    }

    #[test]
    fn provider_contract_digest_changes_when_retry_semantics_change() {
        let mut contract = adyen_contract();
        let before = contract.canonical_digest().unwrap();
        contract.region_scope = ProviderRegionScopeV1::Global;
        assert_ne!(before, contract.canonical_digest().unwrap());
    }

    #[test]
    fn provider_retry_permit_is_digest_bound_and_expires_at_guarantee_boundary() {
        let contract = adyen_contract();
        let key = operation_key(&contract);
        let permit = key
            .qualify_retry(
                &contract,
                "company:test-account",
                "eu",
                &key.request_digest,
                ProviderAttemptClassV1::TransportTimeoutUnknown,
                2_000,
            )
            .unwrap();
        assert_eq!(permit.valid_until_unix_ms, 1_000 + 7 * DAY_MS);
        let fixture = include_str!("../fixtures/sap-provider-contract-v1.kat");
        let expected = fixture
            .lines()
            .find_map(|line| line.strip_prefix("retry_permit="))
            .expect("retry permit KAT entry");
        assert_eq!(permit.canonical_digest().unwrap(), expected);
        assert_eq!(permit.validate_against(&key, &contract, 2_000), Ok(()));

        let mut changed_region = permit.clone();
        changed_region.region = "us".into();
        assert_eq!(
            changed_region.validate_against(&key, &contract, 2_000),
            Err(ProviderRetryErrorV1::RetryPermitMismatch)
        );
        assert_eq!(
            permit.validate_against(&key, &contract, 1_000 + 7 * DAY_MS),
            Err(ProviderRetryErrorV1::RetryPermitExpired)
        );
    }
}