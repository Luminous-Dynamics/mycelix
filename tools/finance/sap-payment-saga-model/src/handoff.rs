//! Crash-safe parent/child delegation handoff reference model.
//!
//! Parent and child books model separate owner-local source chains. They do not
//! share atomicity. Parent escrow remains encumbered until a child closure
//! receipt is accepted by a parent-owned release transition.

use std::collections::BTreeMap;

use sha2::{Digest, Sha256};

use super::delegated::{DelegatedExposurePolicyV1, ParentExposureScopeV1};
use super::exposure::{
    DestinationResolutionV1, ExposureBookV1, ExposureDisposition, ExposureError,
    ExposureIntentV1, ExposurePolicyV1, ExposureScopeV1,
};

const GRANT_DOMAIN: &[u8] = b"MXSAP-GRANT-TERMS-V1\0";
const PARENT_DOMAIN: &[u8] = b"MXSAP-HANDOFF-PARENT-V1\0";
const CHILD_DOMAIN: &[u8] = b"MXSAP-HANDOFF-CHILD-V1\0";
const MAX_FIELD_BYTES: usize = 4096;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum HandoffError {
    InvalidText,
    InvalidDigest,
    InvalidGrant,
    GrantConflict,
    GrantNotFound,
    AcceptanceConflict,
    RevocationConflict,
    ClosureConflict,
    ParentLimitExceeded,
    InvalidTransition,
    UnresolvedExposure,
    Exposure(ExposureError),
}

impl From<ExposureError> for HandoffError {
    fn from(value: ExposureError) -> Self {
        Self::Exposure(value)
    }
}

impl std::fmt::Display for HandoffError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for HandoffError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GrantTermsV1 {
    pub grant_id: String,
    pub parent_scope: ParentExposureScopeV1,
    pub child_scope: ExposureScopeV1,
    pub budget_minor: u64,
    pub grant_sequence: u64,
    pub parent_policy_sequence: u64,
    pub parent_policy_digest: String,
}

impl GrantTermsV1 {
    pub fn validate(&self) -> Result<(), HandoffError> {
        validate_text(&self.grant_id)?;
        for value in [
            &self.parent_scope.owner_id,
            &self.parent_scope.pool_id,
            &self.parent_scope.currency,
            &self.child_scope.owner_id,
            &self.child_scope.pool_id,
            &self.child_scope.destination_id,
            &self.child_scope.currency,
        ] {
            validate_text(value)?;
        }
        validate_digest(&self.parent_policy_digest)?;
        if self.budget_minor == 0 || self.grant_sequence == 0 || self.parent_policy_sequence == 0 {
            return Err(HandoffError::InvalidGrant);
        }
        if self.parent_scope.pool_id != self.child_scope.pool_id
            || self.parent_scope.currency != self.child_scope.currency
        {
            return Err(HandoffError::InvalidGrant);
        }
        Ok(())
    }

    pub fn canonical_digest(&self) -> Result<String, HandoffError> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(GRANT_DOMAIN);
        for value in [
            &self.grant_id,
            &self.parent_scope.owner_id,
            &self.parent_scope.pool_id,
            &self.parent_scope.currency,
            &self.child_scope.owner_id,
            &self.child_scope.pool_id,
            &self.child_scope.destination_id,
            &self.child_scope.currency,
            &self.parent_policy_digest,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.budget_minor);
        append_u64(&mut bytes, self.grant_sequence);
        append_u64(&mut bytes, self.parent_policy_sequence);
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HandoffDisposition {
    Committed,
    AlreadyCommitted,
    Observed,
    AlreadyObserved,
    Released,
    AlreadyReleased,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParentGrantStatus {
    Issued = 1,
    RevocationRequested = 2,
    Released = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeliveryKind {
    Grant = 1,
    Revocation = 2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PendingDeliveryV1 {
    pub kind: DeliveryKind,
    pub stable_delivery_id: String,
    pub grant_id: String,
    pub parent_action_hash: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GrantAcceptanceReceiptV1 {
    pub grant_id: String,
    pub grant_digest: String,
    pub parent_action_hash: String,
    pub child_owner_id: String,
    pub child_accept_action_hash: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GrantRevocationReceiptV1 {
    pub grant_id: String,
    pub grant_digest: String,
    pub parent_revoke_action_hash: String,
    pub child_owner_id: String,
    pub child_observed_action_hash: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GrantClosureReceiptV1 {
    pub grant_id: String,
    pub grant_digest: String,
    pub parent_action_hash: String,
    pub parent_revoke_action_hash: String,
    pub child_owner_id: String,
    pub child_accept_action_hash: String,
    pub child_observed_revoke_action_hash: String,
    pub child_source_chain_frontier: String,
    pub child_snapshot_digest: String,
    pub child_close_action_hash: String,
    pub zero_unresolved: bool,
}

#[derive(Clone, Debug)]
struct ParentGrantRecordV1 {
    terms: GrantTermsV1,
    grant_digest: String,
    parent_action_hash: String,
    status: ParentGrantStatus,
    acceptance: Option<GrantAcceptanceReceiptV1>,
    revoke_action_hash: Option<String>,
    revocation: Option<GrantRevocationReceiptV1>,
    closure: Option<GrantClosureReceiptV1>,
}

#[derive(Clone, Debug)]
pub struct ParentHandoffBookV1 {
    policy: DelegatedExposurePolicyV1,
    grants: BTreeMap<String, ParentGrantRecordV1>,
}

impl ParentHandoffBookV1 {
    pub fn new(policy: DelegatedExposurePolicyV1) -> Result<Self, HandoffError> {
        policy.validate().map_err(HandoffError::Exposure)?;
        Ok(Self {
            policy,
            grants: BTreeMap::new(),
        })
    }

    pub fn policy(&self) -> &DelegatedExposurePolicyV1 {
        &self.policy
    }

    pub fn issue_grant(
        &mut self,
        terms: GrantTermsV1,
        parent_action_hash: impl Into<String>,
    ) -> Result<HandoffDisposition, HandoffError> {
        terms.validate()?;
        let parent_action_hash = parent_action_hash.into();
        validate_digest(&parent_action_hash)?;
        let grant_digest = terms.canonical_digest()?;

        if let Some(existing) = self.grants.get(&terms.grant_id) {
            if existing.terms == terms && existing.grant_digest == grant_digest {
                // A retry returns the already-committed parent action. It does
                // not create a second source-chain grant or replace its hash.
                return Ok(HandoffDisposition::AlreadyCommitted);
            }
            return Err(HandoffError::GrantConflict);
        }

        if terms.parent_scope != self.policy.scope
            || terms.parent_policy_sequence != self.policy.sequence
            || terms.parent_policy_digest != self.policy.policy_digest
        {
            return Err(HandoffError::InvalidGrant);
        }
        if terms.budget_minor > self.policy.maximum_child_budget_minor {
            return Err(HandoffError::ParentLimitExceeded);
        }
        let latest_sequence = self
            .grants
            .values()
            .map(|record| record.terms.grant_sequence)
            .max()
            .unwrap_or(0);
        if terms.grant_sequence <= latest_sequence {
            return Err(HandoffError::InvalidGrant);
        }
        let allocated = self.allocated_budget_minor()?;
        let projected = allocated
            .checked_add(terms.budget_minor)
            .ok_or(HandoffError::ParentLimitExceeded)?;
        if projected > self.policy.maximum_total_minor {
            return Err(HandoffError::ParentLimitExceeded);
        }

        let grant_id = terms.grant_id.clone();
        self.grants.insert(
            grant_id,
            ParentGrantRecordV1 {
                terms,
                grant_digest,
                parent_action_hash,
                status: ParentGrantStatus::Issued,
                acceptance: None,
                revoke_action_hash: None,
                revocation: None,
                closure: None,
            },
        );
        Ok(HandoffDisposition::Committed)
    }

    /// Grant delivery remains pending until a child-authored acceptance or
    /// closure receipt is observed. Repeated delivery uses the same stable ID.
    pub fn pending_deliveries(&self) -> Vec<PendingDeliveryV1> {
        let mut output = Vec::new();
        for record in self.grants.values() {
            if record.status == ParentGrantStatus::Released {
                continue;
            }
            if record.acceptance.is_none() && record.closure.is_none() {
                output.push(PendingDeliveryV1 {
                    kind: DeliveryKind::Grant,
                    stable_delivery_id: format!("grant:{}", record.terms.grant_id),
                    grant_id: record.terms.grant_id.clone(),
                    parent_action_hash: record.parent_action_hash.clone(),
                });
            }
            if let Some(revoke_hash) = &record.revoke_action_hash {
                if record.revocation.is_none() && record.closure.is_none() {
                    output.push(PendingDeliveryV1 {
                        kind: DeliveryKind::Revocation,
                        stable_delivery_id: format!("revoke:{}", record.terms.grant_id),
                        grant_id: record.terms.grant_id.clone(),
                        parent_action_hash: revoke_hash.clone(),
                    });
                }
            }
        }
        output
    }

    pub fn observe_acceptance(
        &mut self,
        receipt: GrantAcceptanceReceiptV1,
    ) -> Result<HandoffDisposition, HandoffError> {
        validate_acceptance(&receipt)?;
        let record = self
            .grants
            .get_mut(&receipt.grant_id)
            .ok_or(HandoffError::GrantNotFound)?;
        if receipt.grant_digest != record.grant_digest
            || receipt.parent_action_hash != record.parent_action_hash
            || receipt.child_owner_id != record.terms.child_scope.owner_id
        {
            return Err(HandoffError::InvalidGrant);
        }
        match &record.acceptance {
            Some(existing) if *existing == receipt => Ok(HandoffDisposition::AlreadyObserved),
            Some(_) => Err(HandoffError::AcceptanceConflict),
            None => {
                record.acceptance = Some(receipt);
                Ok(HandoffDisposition::Observed)
            }
        }
    }

    pub fn request_revocation(
        &mut self,
        grant_id: &str,
        parent_revoke_action_hash: impl Into<String>,
    ) -> Result<HandoffDisposition, HandoffError> {
        let hash = parent_revoke_action_hash.into();
        validate_digest(&hash)?;
        let record = self.grants.get_mut(grant_id).ok_or(HandoffError::GrantNotFound)?;
        match record.status {
            ParentGrantStatus::Issued => {
                record.status = ParentGrantStatus::RevocationRequested;
                record.revoke_action_hash = Some(hash);
                Ok(HandoffDisposition::Committed)
            }
            ParentGrantStatus::RevocationRequested => {
                if record.revoke_action_hash.as_deref() == Some(hash.as_str()) {
                    Ok(HandoffDisposition::AlreadyCommitted)
                } else {
                    Err(HandoffError::RevocationConflict)
                }
            }
            ParentGrantStatus::Released => Err(HandoffError::InvalidTransition),
        }
    }

    pub fn observe_revocation(
        &mut self,
        receipt: GrantRevocationReceiptV1,
    ) -> Result<HandoffDisposition, HandoffError> {
        validate_revocation(&receipt)?;
        let record = self
            .grants
            .get_mut(&receipt.grant_id)
            .ok_or(HandoffError::GrantNotFound)?;
        if record.status != ParentGrantStatus::RevocationRequested
            || record.grant_digest != receipt.grant_digest
            || record.revoke_action_hash.as_deref() != Some(receipt.parent_revoke_action_hash.as_str())
            || record.terms.child_scope.owner_id != receipt.child_owner_id
        {
            return Err(HandoffError::InvalidGrant);
        }
        match &record.revocation {
            Some(existing) if *existing == receipt => Ok(HandoffDisposition::AlreadyObserved),
            Some(_) => Err(HandoffError::RevocationConflict),
            None => {
                record.revocation = Some(receipt);
                Ok(HandoffDisposition::Observed)
            }
        }
    }

    /// The child closure receipt itself carries the acceptance/revocation
    /// action hashes, so the parent can recover even if intermediate ACKs
    /// were lost. Production must authenticate the child chain, not trust
    /// these structurally checked fields alone.
    pub fn release_grant(
        &mut self,
        receipt: GrantClosureReceiptV1,
    ) -> Result<HandoffDisposition, HandoffError> {
        validate_closure(&receipt)?;
        let record = self
            .grants
            .get_mut(&receipt.grant_id)
            .ok_or(HandoffError::GrantNotFound)?;

        if record.status == ParentGrantStatus::Released {
            return match &record.closure {
                Some(existing) if *existing == receipt => Ok(HandoffDisposition::AlreadyReleased),
                _ => Err(HandoffError::ClosureConflict),
            };
        }
        if record.status != ParentGrantStatus::RevocationRequested
            || !receipt.zero_unresolved
            || record.grant_digest != receipt.grant_digest
            || record.parent_action_hash != receipt.parent_action_hash
            || record.revoke_action_hash.as_deref() != Some(receipt.parent_revoke_action_hash.as_str())
            || record.terms.child_scope.owner_id != receipt.child_owner_id
        {
            return Err(HandoffError::InvalidGrant);
        }
        if let Some(accepted) = &record.acceptance {
            if accepted.child_accept_action_hash != receipt.child_accept_action_hash {
                return Err(HandoffError::AcceptanceConflict);
            }
        }
        if let Some(revocation) = &record.revocation {
            if revocation.child_observed_action_hash != receipt.child_observed_revoke_action_hash {
                return Err(HandoffError::RevocationConflict);
            }
        }
        record.status = ParentGrantStatus::Released;
        record.closure = Some(receipt);
        Ok(HandoffDisposition::Released)
    }

    pub fn grant_status(&self, grant_id: &str) -> Option<ParentGrantStatus> {
        self.grants.get(grant_id).map(|record| record.status)
    }

    pub fn allocated_budget_minor(&self) -> Result<u64, HandoffError> {
        self.grants
            .values()
            .filter(|record| record.status != ParentGrantStatus::Released)
            .try_fold(0_u64, |total, record| {
                total
                    .checked_add(record.terms.budget_minor)
                    .ok_or(HandoffError::ParentLimitExceeded)
            })
    }

    pub fn available_parent_budget_minor(&self) -> Result<u64, HandoffError> {
        Ok(self
            .policy
            .maximum_total_minor
            .saturating_sub(self.allocated_budget_minor()?))
    }

    pub fn canonical_snapshot_digest(&self) -> Result<String, HandoffError> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(PARENT_DOMAIN);
        append_text(&mut bytes, &self.policy.scope.owner_id)?;
        append_text(&mut bytes, &self.policy.scope.pool_id)?;
        append_text(&mut bytes, &self.policy.scope.currency)?;
        append_u64(&mut bytes, self.policy.sequence);
        append_u64(&mut bytes, self.policy.maximum_total_minor);
        append_u64(&mut bytes, self.policy.maximum_child_budget_minor);
        append_text(&mut bytes, &self.policy.policy_digest)?;
        let count = u32::try_from(self.grants.len()).map_err(|_| HandoffError::InvalidGrant)?;
        bytes.extend_from_slice(&count.to_be_bytes());
        for record in self.grants.values() {
            append_text(&mut bytes, &record.terms.grant_id)?;
            append_text(&mut bytes, &record.grant_digest)?;
            append_text(&mut bytes, &record.parent_action_hash)?;
            bytes.push(record.status as u8);
            append_u64(&mut bytes, record.terms.grant_sequence);
            match &record.acceptance {
                Some(receipt) => {
                    bytes.push(1);
                    append_text(&mut bytes, &receipt.child_accept_action_hash)?;
                }
                None => bytes.push(0),
            }
            match &record.revoke_action_hash {
                Some(hash) => {
                    bytes.push(1);
                    append_text(&mut bytes, hash)?;
                }
                None => bytes.push(0),
            }
            match &record.revocation {
                Some(receipt) => {
                    bytes.push(1);
                    append_text(&mut bytes, &receipt.child_observed_action_hash)?;
                }
                None => bytes.push(0),
            }
            match &record.closure {
                Some(receipt) => {
                    bytes.push(1);
                    append_text(&mut bytes, &receipt.child_close_action_hash)?;
                    append_text(&mut bytes, &receipt.child_snapshot_digest)?;
                }
                None => bytes.push(0),
            }
        }
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug)]
pub struct ChildHandoffBookV1 {
    terms: GrantTermsV1,
    grant_digest: String,
    parent_action_hash: String,
    child_owner_id: String,
    acceptance: Option<GrantAcceptanceReceiptV1>,
    revocation: Option<GrantRevocationReceiptV1>,
    exposure: ExposureBookV1,
    closure: Option<GrantClosureReceiptV1>,
}

impl ChildHandoffBookV1 {
    pub fn new(
        terms: GrantTermsV1,
        parent_action_hash: impl Into<String>,
    ) -> Result<Self, HandoffError> {
        terms.validate()?;
        let parent_action_hash = parent_action_hash.into();
        validate_digest(&parent_action_hash)?;
        let grant_digest = terms.canonical_digest()?;
        let exposure_policy = ExposurePolicyV1 {
            scope: terms.child_scope.clone(),
            sequence: terms.grant_sequence,
            maximum_total_minor: terms.budget_minor,
            maximum_per_effect_minor: terms.budget_minor,
            policy_digest: grant_digest.clone(),
        };
        Ok(Self {
            child_owner_id: terms.child_scope.owner_id.clone(),
            terms,
            grant_digest,
            parent_action_hash,
            acceptance: None,
            revocation: None,
            exposure: ExposureBookV1::new(exposure_policy)?,
            closure: None,
        })
    }

    pub fn accept_grant(
        &mut self,
        parent_action_hash: &str,
        child_accept_action_hash: impl Into<String>,
    ) -> Result<(HandoffDisposition, GrantAcceptanceReceiptV1), HandoffError> {
        validate_digest(parent_action_hash)?;
        let candidate_action_hash = child_accept_action_hash.into();
        validate_digest(&candidate_action_hash)?;
        if parent_action_hash != self.parent_action_hash {
            return Err(HandoffError::InvalidGrant);
        }
        if self.revocation.is_some() || self.closure.is_some() {
            return Err(HandoffError::InvalidTransition);
        }
        if let Some(existing) = &self.acceptance {
            return Ok((HandoffDisposition::AlreadyObserved, existing.clone()));
        }
        let receipt = GrantAcceptanceReceiptV1 {
            grant_id: self.terms.grant_id.clone(),
            grant_digest: self.grant_digest.clone(),
            parent_action_hash: self.parent_action_hash.clone(),
            child_owner_id: self.child_owner_id.clone(),
            child_accept_action_hash: candidate_action_hash,
        };
        self.acceptance = Some(receipt.clone());
        Ok((HandoffDisposition::Observed, receipt))
    }

    pub fn reserve(
        &mut self,
        intent: ExposureIntentV1,
    ) -> Result<ExposureDisposition, HandoffError> {
        if self.acceptance.is_none() || self.revocation.is_some() || self.closure.is_some() {
            return Err(HandoffError::InvalidTransition);
        }
        Ok(self.exposure.reserve(intent)?)
    }

    pub fn mark_dispatch_authorized(&mut self, effect_id: &str) -> Result<(), HandoffError> {
        if self.acceptance.is_none() || self.revocation.is_some() || self.closure.is_some() {
            return Err(HandoffError::InvalidTransition);
        }
        Ok(self.exposure.mark_dispatch_authorized(effect_id)?)
    }

    pub fn mark_indeterminate(&mut self, effect_id: &str) -> Result<(), HandoffError> {
        Ok(self.exposure.mark_indeterminate(effect_id)?)
    }

    pub fn mark_committed_unsettled(
        &mut self,
        effect_id: &str,
        evidence: &DestinationResolutionV1,
    ) -> Result<(), HandoffError> {
        Ok(self.exposure.mark_committed_unsettled(effect_id, evidence)?)
    }

    pub fn mark_settled(
        &mut self,
        effect_id: &str,
        evidence: &DestinationResolutionV1,
    ) -> Result<(), HandoffError> {
        Ok(self.exposure.mark_settled(effect_id, evidence)?)
    }

    pub fn release_authoritative_absence(
        &mut self,
        effect_id: &str,
        evidence: &DestinationResolutionV1,
    ) -> Result<(), HandoffError> {
        Ok(self.exposure.release_authoritative_absence(effect_id, evidence)?)
    }

    pub fn cancel_before_dispatch(&mut self, effect_id: &str) -> Result<(), HandoffError> {
        Ok(self.exposure.cancel_before_dispatch(effect_id)?)
    }

    pub fn observe_revocation(
        &mut self,
        parent_revoke_action_hash: &str,
        child_observed_action_hash: impl Into<String>,
    ) -> Result<(HandoffDisposition, GrantRevocationReceiptV1), HandoffError> {
        validate_digest(parent_revoke_action_hash)?;
        let candidate_action_hash = child_observed_action_hash.into();
        validate_digest(&candidate_action_hash)?;
        if self.closure.is_some() {
            return Err(HandoffError::InvalidTransition);
        }
        if let Some(existing) = &self.revocation {
            if existing.parent_revoke_action_hash != parent_revoke_action_hash {
                return Err(HandoffError::RevocationConflict);
            }
            return Ok((HandoffDisposition::AlreadyObserved, existing.clone()));
        }
        let receipt = GrantRevocationReceiptV1 {
            grant_id: self.terms.grant_id.clone(),
            grant_digest: self.grant_digest.clone(),
            parent_revoke_action_hash: parent_revoke_action_hash.to_owned(),
            child_owner_id: self.child_owner_id.clone(),
            child_observed_action_hash: candidate_action_hash,
        };
        self.revocation = Some(receipt.clone());
        Ok((HandoffDisposition::Observed, receipt))
    }

    pub fn exposure_minor(&self) -> Result<u64, HandoffError> {
        Ok(self.exposure.total_exposure_minor()?)
    }

    pub fn close(
        &mut self,
        child_source_chain_frontier: impl Into<String>,
        child_close_action_hash: impl Into<String>,
    ) -> Result<(HandoffDisposition, GrantClosureReceiptV1), HandoffError> {
        if let Some(existing) = &self.closure {
            return Ok((HandoffDisposition::AlreadyCommitted, existing.clone()));
        }
        let revocation = self.revocation.as_ref().ok_or(HandoffError::InvalidTransition)?;
        let acceptance = self.acceptance.as_ref().ok_or(HandoffError::InvalidTransition)?;
        if self.exposure.total_exposure_minor()? != 0 {
            return Err(HandoffError::UnresolvedExposure);
        }
        let frontier = child_source_chain_frontier.into();
        let close_hash = child_close_action_hash.into();
        validate_digest(&frontier)?;
        validate_digest(&close_hash)?;
        let receipt = GrantClosureReceiptV1 {
            grant_id: self.terms.grant_id.clone(),
            grant_digest: self.grant_digest.clone(),
            parent_action_hash: self.parent_action_hash.clone(),
            parent_revoke_action_hash: revocation.parent_revoke_action_hash.clone(),
            child_owner_id: self.child_owner_id.clone(),
            child_accept_action_hash: acceptance.child_accept_action_hash.clone(),
            child_observed_revoke_action_hash: revocation.child_observed_action_hash.clone(),
            child_source_chain_frontier: frontier,
            child_snapshot_digest: self.exposure.canonical_snapshot_digest()?,
            child_close_action_hash: close_hash,
            zero_unresolved: true,
        };
        self.closure = Some(receipt.clone());
        Ok((HandoffDisposition::Committed, receipt))
    }

    pub fn canonical_snapshot_digest(&self) -> Result<String, HandoffError> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(CHILD_DOMAIN);
        append_text(&mut bytes, &self.terms.grant_id)?;
        append_text(&mut bytes, &self.grant_digest)?;
        append_text(&mut bytes, &self.parent_action_hash)?;
        append_text(&mut bytes, &self.child_owner_id)?;
        match &self.acceptance {
            Some(receipt) => {
                bytes.push(1);
                append_text(&mut bytes, &receipt.child_accept_action_hash)?;
            }
            None => bytes.push(0),
        }
        match &self.revocation {
            Some(receipt) => {
                bytes.push(1);
                append_text(&mut bytes, &receipt.parent_revoke_action_hash)?;
                append_text(&mut bytes, &receipt.child_observed_action_hash)?;
            }
            None => bytes.push(0),
        }
        append_text(&mut bytes, &self.exposure.canonical_snapshot_digest()?)?;
        match &self.closure {
            Some(receipt) => {
                bytes.push(1);
                append_text(&mut bytes, &receipt.child_close_action_hash)?;
                append_text(&mut bytes, &receipt.child_source_chain_frontier)?;
            }
            None => bytes.push(0),
        }
        Ok(sha256_hex(&bytes))
    }
}

fn validate_acceptance(receipt: &GrantAcceptanceReceiptV1) -> Result<(), HandoffError> {
    for value in [
        &receipt.grant_id,
        &receipt.child_owner_id,
    ] {
        validate_text(value)?;
    }
    for value in [
        &receipt.grant_digest,
        &receipt.parent_action_hash,
        &receipt.child_accept_action_hash,
    ] {
        validate_digest(value)?;
    }
    Ok(())
}

fn validate_revocation(receipt: &GrantRevocationReceiptV1) -> Result<(), HandoffError> {
    validate_text(&receipt.grant_id)?;
    validate_text(&receipt.child_owner_id)?;
    for value in [
        &receipt.grant_digest,
        &receipt.parent_revoke_action_hash,
        &receipt.child_observed_action_hash,
    ] {
        validate_digest(value)?;
    }
    Ok(())
}

fn validate_closure(receipt: &GrantClosureReceiptV1) -> Result<(), HandoffError> {
    validate_text(&receipt.grant_id)?;
    validate_text(&receipt.child_owner_id)?;
    for value in [
        &receipt.grant_digest,
        &receipt.parent_action_hash,
        &receipt.parent_revoke_action_hash,
        &receipt.child_accept_action_hash,
        &receipt.child_observed_revoke_action_hash,
        &receipt.child_source_chain_frontier,
        &receipt.child_snapshot_digest,
        &receipt.child_close_action_hash,
    ] {
        validate_digest(value)?;
    }
    if !receipt.zero_unresolved {
        return Err(HandoffError::UnresolvedExposure);
    }
    Ok(())
}

fn validate_text(value: &str) -> Result<(), HandoffError> {
    if value.is_empty() || value.len() > MAX_FIELD_BYTES {
        return Err(HandoffError::InvalidText);
    }
    Ok(())
}

fn validate_digest(value: &str) -> Result<(), HandoffError> {
    if value.len() != 71
        || !value.starts_with("sha256:")
        || !value[7..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(HandoffError::InvalidDigest);
    }
    Ok(())
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), HandoffError> {
    validate_text(value)?;
    let length = u32::try_from(value.len()).map_err(|_| HandoffError::InvalidText)?;
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
    use crate::finance_placeholder::*;

    fn digest(byte: char) -> String {
        format!("sha256:{}", byte.to_string().repeat(64))
    }

    fn parent_policy() -> DelegatedExposurePolicyV1 {
        DelegatedExposurePolicyV1 {
            scope: ParentExposureScopeV1 {
                owner_id: "owner:treasury".into(),
                pool_id: "pool:one".into(),
                currency: "SAP".into(),
            },
            sequence: 1,
            maximum_total_minor: 1_000,
            maximum_child_budget_minor: 800,
            policy_digest: digest('d'),
        }
    }

    fn terms() -> GrantTermsV1 {
        GrantTermsV1 {
            grant_id: "grant:one".into(),
            parent_scope: parent_policy().scope,
            child_scope: ExposureScopeV1 {
                owner_id: "operator:a".into(),
                pool_id: "pool:one".into(),
                destination_id: "dest:a".into(),
                currency: "SAP".into(),
            },
            budget_minor: 700,
            grant_sequence: 1,
            parent_policy_sequence: 1,
            parent_policy_digest: digest('d'),
        }
    }

    fn intent(effect_id: &str, amount: u64) -> ExposureIntentV1 {
        ExposureIntentV1 {
            scope: terms().child_scope,
            operation_id: format!("operation:{effect_id}"),
            effect_id: effect_id.into(),
            intent_digest: digest('a'),
            payload_digest: digest('b'),
            maximum_exposure_minor: amount,
        }
    }

    fn observed(effect_id: &str) -> DestinationResolutionV1 {
        DestinationResolutionV1 {
            kind: super::super::exposure::ExposureResolutionKind::ObservedUnsettledEffect,
            effect_id: effect_id.into(),
            destination_id: "dest:a".into(),
            payload_digest: digest('b'),
            profile_digest: digest('e'),
            evidence_digest: digest('f'),
        }
    }

    fn settled(effect_id: &str) -> DestinationResolutionV1 {
        DestinationResolutionV1 {
            kind: super::super::exposure::ExposureResolutionKind::FinalSettlementEvidence,
            effect_id: effect_id.into(),
            destination_id: "dest:a".into(),
            payload_digest: digest('b'),
            profile_digest: digest('c'),
            evidence_digest: digest('9'),
        }
    }

    fn absence(effect_id: &str) -> DestinationResolutionV1 {
        DestinationResolutionV1 {
            kind: super::super::exposure::ExposureResolutionKind::AuthoritativeAbsence,
            effect_id: effect_id.into(),
            destination_id: "dest:a".into(),
            payload_digest: digest('b'),
            profile_digest: digest('c'),
            evidence_digest: digest('8'),
        }
    }

    fn parent_receipt_hash() -> String { digest('1') }
    fn accept_action_hash() -> String { digest('2') }
    fn revoke_action_hash() -> String { digest('3') }
    fn observe_revoke_hash() -> String { digest('4') }
    fn child_frontier() -> String { digest('5') }
    fn close_action_hash() -> String { digest('6') }

    fn make_parent_and_child() -> (ParentHandoffBookV1, ChildHandoffBookV1) {
        let terms = terms();
        let parent_hash = parent_receipt_hash();
        let mut parent = ParentHandoffBookV1::new(parent_policy()).unwrap();
        parent.issue_grant(terms.clone(), parent_hash.clone()).unwrap();
        let child = ChildHandoffBookV1::new(terms, parent_hash).unwrap();
        (parent, child)
    }

    #[test]
    fn child_cannot_use_grant_before_acceptance_and_parent_escrow_is_atomic() {
        let (parent, mut child) = make_parent_and_child();
        assert_eq!(parent.allocated_budget_minor(), Ok(700));
        assert_eq!(parent.available_parent_budget_minor(), Ok(300));
        assert_eq!(
            child.reserve(intent("effect:1", 100)),
            Err(HandoffError::InvalidTransition)
        );
    }

    #[test]
    fn lost_acceptance_ack_keeps_parent_escrow_and_outbox_retries_same_identity() {
        let (mut parent, mut child) = make_parent_and_child();
        let pending = parent.pending_deliveries();
        assert_eq!(pending.len(), 1);
        assert_eq!(pending[0].stable_delivery_id, "grant:grant:one");

        let (_, acceptance) = child
            .accept_grant(&parent_receipt_hash(), accept_action_hash())
            .unwrap();
        // Simulate a lost response: parent has not observed the receipt.
        assert_eq!(parent.pending_deliveries().len(), 1);
        assert_eq!(parent.allocated_budget_minor(), Ok(700));

        // Child restart / duplicate delivery replays the original acceptance.
        let (disposition, replayed) = child
            .accept_grant(&parent_receipt_hash(), digest('7'))
            .unwrap();
        assert_eq!(disposition, HandoffDisposition::AlreadyObserved);
        assert_eq!(replayed, acceptance);

        child.reserve(intent("effect:1", 400)).unwrap();
        child.mark_dispatch_authorized("effect:1").unwrap();
        assert_eq!(
            parent.observe_acceptance(acceptance),
            Ok(HandoffDisposition::Observed)
        );
        assert!(parent.pending_deliveries().is_empty());
        assert_eq!(parent.allocated_budget_minor(), Ok(700));
    }

    #[test]
    fn revocation_delivery_delay_keeps_budget_held_and_child_may_only_use_bounded_grant() {
        let (mut parent, mut child) = make_parent_and_child();
        let (_, acceptance) = child
            .accept_grant(&parent_receipt_hash(), accept_action_hash())
            .unwrap();
        parent.observe_acceptance(acceptance).unwrap();

        parent.request_revocation("grant:one", revoke_action_hash()).unwrap();
        assert_eq!(parent.pending_deliveries().len(), 1);
        assert_eq!(parent.allocated_budget_minor(), Ok(700));

        // The child has not yet observed revocation; it can still use its
        // already-delegated budget, but no more than the hard child sublimit.
        child.reserve(intent("effect:1", 500)).unwrap();
        assert_eq!(
            child.reserve(intent("effect:2", 250)),
            Err(HandoffError::Exposure(ExposureError::LimitExceeded))
        );
        assert_eq!(parent.allocated_budget_minor(), Ok(700));

        let (_, observed_revoke) = child
            .observe_revocation(&revoke_action_hash(), observe_revoke_hash())
            .unwrap();
        parent.observe_revocation(observed_revoke.clone()).unwrap();
        assert!(parent.pending_deliveries().is_empty());
        assert_eq!(
            child.reserve(intent("effect:3", 10)),
            Err(HandoffError::InvalidTransition)
        );
    }

    #[test]
    fn child_close_waits_for_resolution_and_parent_release_requires_exact_receipt() {
        let (mut parent, mut child) = make_parent_and_child();
        let (_, acceptance) = child.accept_grant(&parent_receipt_hash(), accept_action_hash()).unwrap();
        parent.observe_acceptance(acceptance).unwrap();
        parent.request_revocation("grant:one", revoke_action_hash()).unwrap();
        let (_, observed_revoke) = child.observe_revocation(&revoke_action_hash(), observe_revoke_hash()).unwrap();
        parent.observe_revocation(observed_revoke).unwrap();

        child.reserve(intent("effect:1", 500)).unwrap();
        child.mark_dispatch_authorized("effect:1").unwrap();
        child.mark_indeterminate("effect:1").unwrap();
        assert_eq!(
            child.close(child_frontier(), close_action_hash()),
            Err(HandoffError::UnresolvedExposure)
        );
        child.release_authoritative_absence("effect:1", &absence("effect:1")).unwrap();
        let (_, closure) = child.close(child_frontier(), close_action_hash()).unwrap();

        let mut wrong = closure.clone();
        wrong.child_owner_id = "operator:other".into();
        assert_eq!(parent.release_grant(wrong), Err(HandoffError::InvalidGrant));
        assert_eq!(parent.allocated_budget_minor(), Ok(700));

        assert_eq!(parent.release_grant(closure.clone()), Ok(HandoffDisposition::Released));
        assert_eq!(parent.allocated_budget_minor(), Ok(0));
        assert_eq!(
            parent.release_grant(closure),
            Ok(HandoffDisposition::AlreadyReleased)
        );
    }

    #[test]
    fn child_can_resolve_already_dispatched_effect_after_revocation() {
        let (mut parent, mut child) = make_parent_and_child();
        let (_, acceptance) = child.accept_grant(&parent_receipt_hash(), accept_action_hash()).unwrap();
        parent.observe_acceptance(acceptance).unwrap();
        child.reserve(intent("effect:1", 400)).unwrap();
        child.mark_dispatch_authorized("effect:1").unwrap();
        parent.request_revocation("grant:one", revoke_action_hash()).unwrap();
        let (_, observed_revoke) = child.observe_revocation(&revoke_action_hash(), observe_revoke_hash()).unwrap();
        parent.observe_revocation(observed_revoke).unwrap();

        child.mark_committed_unsettled("effect:1", &observed("effect:1")).unwrap();
        child.mark_settled("effect:1", &settled("effect:1")).unwrap();
        let (_, closure) = child.close(child_frontier(), close_action_hash()).unwrap();
        assert_eq!(parent.release_grant(closure), Ok(HandoffDisposition::Released));
    }

    #[test]
    fn parent_restart_preserves_committed_grant_outbox_and_release_identity() {
        let (mut parent, mut child) = make_parent_and_child();
        let restarted_parent = parent.clone();
        parent = restarted_parent;
        assert_eq!(parent.pending_deliveries().len(), 1);

        let (_, acceptance) = child.accept_grant(&parent_receipt_hash(), accept_action_hash()).unwrap();
        parent.observe_acceptance(acceptance).unwrap();
        parent.request_revocation("grant:one", revoke_action_hash()).unwrap();
        let (_, revocation) = child.observe_revocation(&revoke_action_hash(), observe_revoke_hash()).unwrap();
        parent.observe_revocation(revocation).unwrap();
        let (_, closure) = child.close(child_frontier(), close_action_hash()).unwrap();

        let closure_retry = closure.clone();
        assert_eq!(parent.release_grant(closure), Ok(HandoffDisposition::Released));
        let restarted_parent = parent.clone();
        parent = restarted_parent;
        assert_eq!(
            parent.release_grant(closure_retry),
            Ok(HandoffDisposition::AlreadyReleased)
        );
    }
}
