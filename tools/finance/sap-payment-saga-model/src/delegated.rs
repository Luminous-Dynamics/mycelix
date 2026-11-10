//! Parent-escrow / child-sublimit reference protocol.
//!
//! The parent permanently encumbers each active or revoking child budget.
//! Child books can use only their allocated sublimit. Revocation blocks new
//! reservations but leaves resolution and reconciliation available.

use std::collections::BTreeMap;

use sha2::{Digest, Sha256};

use super::exposure::{
    DestinationResolutionV1, ExposureBookV1, ExposureDisposition, ExposureError,
    ExposureIntentV1, ExposurePolicyV1, ExposureResolutionKind, ExposureScopeV1,
};

const DELEGATED_DOMAIN: &[u8] = b"MXSAP-DELEGATED-EXPOSURE-V1\0";
const MAX_DELEGATED_FIELD_BYTES: usize = 4096;

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ParentExposureScopeV1 {
    pub owner_id: String,
    pub pool_id: String,
    pub currency: String,
}

impl ParentExposureScopeV1 {
    fn validate(&self) -> Result<(), ExposureError> {
        for value in [&self.owner_id, &self.pool_id, &self.currency] {
            validate_text(value)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DelegatedExposurePolicyV1 {
    pub scope: ParentExposureScopeV1,
    pub sequence: u64,
    pub maximum_total_minor: u64,
    pub maximum_child_budget_minor: u64,
    pub policy_digest: String,
}

impl DelegatedExposurePolicyV1 {
    fn validate(&self) -> Result<(), ExposureError> {
        self.scope.validate()?;
        if self.sequence == 0
            || self.maximum_total_minor == 0
            || self.maximum_child_budget_minor == 0
            || self.maximum_child_budget_minor > self.maximum_total_minor
        {
            return Err(ExposureError::InvalidPolicy);
        }
        validate_digest(&self.policy_digest)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DelegatedBudgetStatus {
    Active = 1,
    Revoking = 2,
    Closed = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DelegatedBudgetDisposition {
    Allocated,
    AlreadyAllocated,
}

#[derive(Clone, Debug)]
struct DelegatedBudgetGrant {
    grant_id: String,
    child_scope: ExposureScopeV1,
    budget_minor: u64,
    sequence: u64,
    grant_digest: String,
    status: DelegatedBudgetStatus,
    child_book: ExposureBookV1,
}

impl DelegatedBudgetGrant {
    fn same_terms(
        &self,
        child_scope: &ExposureScopeV1,
        budget_minor: u64,
        sequence: u64,
        grant_digest: &str,
    ) -> bool {
        self.child_scope == *child_scope
            && self.budget_minor == budget_minor
            && self.sequence == sequence
            && self.grant_digest == grant_digest
    }

    fn counts_against_parent(&self) -> bool {
        matches!(self.status, DelegatedBudgetStatus::Active | DelegatedBudgetStatus::Revoking)
    }
}

#[derive(Clone, Debug)]
pub struct ParentDelegatedExposureBookV1 {
    policy: DelegatedExposurePolicyV1,
    grants: BTreeMap<String, DelegatedBudgetGrant>,
}

impl ParentDelegatedExposureBookV1 {
    pub fn new(policy: DelegatedExposurePolicyV1) -> Result<Self, ExposureError> {
        policy.validate()?;
        Ok(Self {
            policy,
            grants: BTreeMap::new(),
        })
    }

    pub fn policy(&self) -> &DelegatedExposurePolicyV1 {
        &self.policy
    }

    /// Allocates a bounded child sublimit. The full sublimit remains encumbered
    /// at the parent until the grant is closed, even when child exposure is zero.
    pub fn allocate_child_budget(
        &mut self,
        grant_id: impl Into<String>,
        child_scope: ExposureScopeV1,
        budget_minor: u64,
        grant_sequence: u64,
        grant_digest: impl Into<String>,
    ) -> Result<DelegatedBudgetDisposition, ExposureError> {
        let grant_id = grant_id.into();
        let grant_digest = grant_digest.into();
        validate_text(&grant_id)?;
        validate_digest(&grant_digest)?;

        if child_scope.pool_id != self.policy.scope.pool_id
            || child_scope.currency != self.policy.scope.currency
        {
            return Err(ExposureError::ScopeMismatch);
        }
        // Reuse the exposure-policy validator for the complete child scope.
        let child_policy = ExposurePolicyV1 {
            scope: child_scope.clone(),
            sequence: grant_sequence,
            maximum_total_minor: budget_minor,
            maximum_per_effect_minor: budget_minor,
            policy_digest: grant_digest.clone(),
        };
        child_policy.validate()?;
        if budget_minor > self.policy.maximum_child_budget_minor {
            return Err(ExposureError::LimitExceeded);
        }

        if let Some(existing) = self.grants.get(&grant_id) {
            return if existing.same_terms(&child_scope, budget_minor, grant_sequence, &grant_digest) {
                Ok(DelegatedBudgetDisposition::AlreadyAllocated)
            } else {
                Err(ExposureError::ReservationConflict)
            };
        }

        let latest_sequence = self.grants.values().map(|grant| grant.sequence).max().unwrap_or(0);
        if grant_sequence <= latest_sequence {
            return Err(ExposureError::InvalidPolicy);
        }

        let allocated = self.allocated_budget_minor()?;
        let projected = allocated
            .checked_add(budget_minor)
            .ok_or(ExposureError::AmountOverflow)?;
        if projected > self.policy.maximum_total_minor {
            return Err(ExposureError::LimitExceeded);
        }

        let child_book = ExposureBookV1::new(child_policy)?;
        self.grants.insert(
            grant_id.clone(),
            DelegatedBudgetGrant {
                grant_id,
                child_scope,
                budget_minor,
                sequence: grant_sequence,
                grant_digest,
                status: DelegatedBudgetStatus::Active,
                child_book,
            },
        );
        Ok(DelegatedBudgetDisposition::Allocated)
    }

    /// Exact retries for an existing effect still resolve during revocation;
    /// a new effect cannot consume a revoking or closed grant.
    pub fn reserve(
        &mut self,
        grant_id: &str,
        intent: ExposureIntentV1,
    ) -> Result<ExposureDisposition, ExposureError> {
        let grant = self.grants.get_mut(grant_id).ok_or(ExposureError::ReservationNotFound)?;
        let existing_effect = grant.child_book.status(&intent.effect_id).is_some();
        if grant.status != DelegatedBudgetStatus::Active && !existing_effect {
            return Err(ExposureError::InvalidTransition);
        }
        if intent.scope != grant.child_scope {
            return Err(ExposureError::ScopeMismatch);
        }
        grant.child_book.reserve(intent)
    }

    pub fn mark_dispatch_authorized(
        &mut self,
        grant_id: &str,
        effect_id: &str,
    ) -> Result<(), ExposureError> {
        self.child_book_mut(grant_id)?.mark_dispatch_authorized(effect_id)
    }

    pub fn mark_indeterminate(
        &mut self,
        grant_id: &str,
        effect_id: &str,
    ) -> Result<(), ExposureError> {
        self.child_book_mut(grant_id)?.mark_indeterminate(effect_id)
    }

    pub fn mark_committed_unsettled(
        &mut self,
        grant_id: &str,
        effect_id: &str,
    ) -> Result<(), ExposureError> {
        self.child_book_mut(grant_id)?.mark_committed_unsettled(effect_id)
    }

    pub fn mark_settled(
        &mut self,
        grant_id: &str,
        effect_id: &str,
        evidence: &DestinationResolutionV1,
    ) -> Result<(), ExposureError> {
        self.child_book_mut(grant_id)?.mark_settled(effect_id, evidence)
    }

    pub fn release_authoritative_absence(
        &mut self,
        grant_id: &str,
        effect_id: &str,
        evidence: &DestinationResolutionV1,
    ) -> Result<(), ExposureError> {
        self.child_book_mut(grant_id)?.release_authoritative_absence(effect_id, evidence)
    }

    /// Stops new child reservations but does not block settlement, absence
    /// reconciliation or other resolution of existing effects.
    pub fn begin_revocation(&mut self, grant_id: &str) -> Result<(), ExposureError> {
        let grant = self.grants.get_mut(grant_id).ok_or(ExposureError::ReservationNotFound)?;
        match grant.status {
            DelegatedBudgetStatus::Active => {
                grant.status = DelegatedBudgetStatus::Revoking;
                Ok(())
            }
            DelegatedBudgetStatus::Revoking => Ok(()),
            DelegatedBudgetStatus::Closed => Err(ExposureError::InvalidTransition),
        }
    }

    /// Closes only after all child exposure is resolved. Production must replace
    /// this in-memory observation with an authenticated child close receipt.
    pub fn close_grant(&mut self, grant_id: &str) -> Result<(), ExposureError> {
        let grant = self.grants.get_mut(grant_id).ok_or(ExposureError::ReservationNotFound)?;
        if grant.status == DelegatedBudgetStatus::Closed {
            return Ok(());
        }
        if grant.status != DelegatedBudgetStatus::Revoking
            || grant.child_book.total_exposure_minor()? != 0
        {
            return Err(ExposureError::InvalidTransition);
        }
        grant.status = DelegatedBudgetStatus::Closed;
        Ok(())
    }

    pub fn grant_status(&self, grant_id: &str) -> Option<DelegatedBudgetStatus> {
        self.grants.get(grant_id).map(|grant| grant.status)
    }

    pub fn child_exposure_minor(&self, grant_id: &str) -> Result<u64, ExposureError> {
        self.grants
            .get(grant_id)
            .ok_or(ExposureError::ReservationNotFound)?
            .child_book
            .total_exposure_minor()
    }

    pub fn allocated_budget_minor(&self) -> Result<u64, ExposureError> {
        self.grants
            .values()
            .filter(|grant| grant.counts_against_parent())
            .try_fold(0_u64, |total, grant| {
                total.checked_add(grant.budget_minor).ok_or(ExposureError::AmountOverflow)
            })
    }

    pub fn available_parent_budget_minor(&self) -> Result<u64, ExposureError> {
        Ok(self
            .policy
            .maximum_total_minor
            .saturating_sub(self.allocated_budget_minor()?))
    }

    pub fn canonical_snapshot_digest(&self) -> Result<String, ExposureError> {
        let count = u32::try_from(self.grants.len()).map_err(|_| ExposureError::InvalidAmount)?;
        let mut bytes = Vec::with_capacity(256 + self.grants.len() * 256);
        bytes.extend_from_slice(DELEGATED_DOMAIN);
        for value in [
            &self.policy.scope.owner_id,
            &self.policy.scope.pool_id,
            &self.policy.scope.currency,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.policy.sequence);
        append_u64(&mut bytes, self.policy.maximum_total_minor);
        append_u64(&mut bytes, self.policy.maximum_child_budget_minor);
        append_text(&mut bytes, &self.policy.policy_digest)?;
        bytes.extend_from_slice(&count.to_be_bytes());

        for grant in self.grants.values() {
            for value in [
                &grant.grant_id,
                &grant.child_scope.owner_id,
                &grant.child_scope.pool_id,
                &grant.child_scope.destination_id,
                &grant.child_scope.currency,
                &grant.grant_digest,
            ] {
                append_text(&mut bytes, value)?;
            }
            append_u64(&mut bytes, grant.budget_minor);
            append_u64(&mut bytes, grant.sequence);
            bytes.push(grant.status as u8);
            append_text(&mut bytes, &grant.child_book.canonical_snapshot_digest()?)?;
        }

        let digest = Sha256::digest(bytes);
        let mut output = String::with_capacity(71);
        output.push_str("sha256:");
        for byte in digest {
            use std::fmt::Write as _;
            write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
        }
        Ok(output)
    }

    fn child_book_mut(&mut self, grant_id: &str) -> Result<&mut ExposureBookV1, ExposureError> {
        Ok(&mut self
            .grants
            .get_mut(grant_id)
            .ok_or(ExposureError::ReservationNotFound)?
            .child_book)
    }
}

fn validate_text(value: &str) -> Result<(), ExposureError> {
    if value.is_empty() || value.len() > MAX_DELEGATED_FIELD_BYTES {
        return Err(ExposureError::InvalidText);
    }
    Ok(())
}

fn validate_digest(value: &str) -> Result<(), ExposureError> {
    if value.len() != 71
        || !value.starts_with("sha256:")
        || !value[7..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(ExposureError::InvalidDigest);
    }
    Ok(())
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), ExposureError> {
    validate_text(value)?;
    let length = u32::try_from(value.len()).map_err(|_| ExposureError::InvalidText)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
    Ok(())
}

fn append_u64(output: &mut Vec<u8>, value: u64) {
    output.extend_from_slice(&value.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: char) -> String {
        format!("sha256:{}", byte.to_string().repeat(64))
    }

    fn parent_policy(maximum: u64) -> DelegatedExposurePolicyV1 {
        DelegatedExposurePolicyV1 {
            scope: ParentExposureScopeV1 {
                owner_id: "owner:treasury".into(),
                pool_id: "pool:one".into(),
                currency: "SAP".into(),
            },
            sequence: 1,
            maximum_total_minor: maximum,
            maximum_child_budget_minor: maximum.min(800),
            policy_digest: digest('d'),
        }
    }

    fn child_scope(owner: &str, destination: &str) -> ExposureScopeV1 {
        ExposureScopeV1 {
            owner_id: owner.into(),
            pool_id: "pool:one".into(),
            destination_id: destination.into(),
            currency: "SAP".into(),
        }
    }

    fn child_intent(scope: ExposureScopeV1, effect_id: &str, amount: u64) -> ExposureIntentV1 {
        ExposureIntentV1 {
            scope,
            operation_id: format!("operation:{effect_id}"),
            effect_id: effect_id.into(),
            intent_digest: digest('a'),
            payload_digest: digest('b'),
            maximum_exposure_minor: amount,
        }
    }

    fn final_settlement(effect_id: &str, destination: &str) -> DestinationResolutionV1 {
        DestinationResolutionV1 {
            kind: ExposureResolutionKind::FinalSettlementEvidence,
            effect_id: effect_id.into(),
            destination_id: destination.into(),
            payload_digest: digest('b'),
            profile_digest: digest('e'),
            evidence_digest: digest('f'),
        }
    }

    fn absence(effect_id: &str, destination: &str) -> DestinationResolutionV1 {
        DestinationResolutionV1 {
            kind: ExposureResolutionKind::AuthoritativeAbsence,
            effect_id: effect_id.into(),
            destination_id: destination.into(),
            payload_digest: digest('b'),
            profile_digest: digest('e'),
            evidence_digest: digest('f'),
        }
    }

    #[test]
    fn parent_encumbers_full_child_sublimits_across_destinations() {
        let mut parent = ParentDelegatedExposureBookV1::new(parent_policy(1_000)).unwrap();
        assert_eq!(
            parent.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 700, 1, digest('1')),
            Ok(DelegatedBudgetDisposition::Allocated)
        );
        assert_eq!(parent.available_parent_budget_minor(), Ok(300));
        assert_eq!(
            parent.allocate_child_budget("provider-b", child_scope("operator:b", "dest:b"), 700, 2, digest('2')),
            Err(ExposureError::LimitExceeded)
        );
        assert_eq!(parent.allocated_budget_minor(), Ok(700));
    }

    #[test]
    fn child_cannot_exceed_its_delegated_budget_even_with_multiple_effects() {
        let mut parent = ParentDelegatedExposureBookV1::new(parent_policy(1_000)).unwrap();
        parent.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 700, 1, digest('1')).unwrap();
        parent.reserve("provider-a", child_intent(child_scope("operator:a", "dest:a"), "effect:1", 500)).unwrap();
        assert_eq!(
            parent.reserve("provider-a", child_intent(child_scope("operator:a", "dest:a"), "effect:2", 250)),
            Err(ExposureError::LimitExceeded)
        );
        assert_eq!(parent.child_exposure_minor("provider-a"), Ok(500));
    }

    #[test]
    fn revocation_blocks_new_risk_but_keeps_reconciliation_available() {
        let mut parent = ParentDelegatedExposureBookV1::new(parent_policy(1_000)).unwrap();
        parent.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 700, 1, digest('1')).unwrap();
        let intent = child_intent(child_scope("operator:a", "dest:a"), "effect:1", 400);
        parent.reserve("provider-a", intent.clone()).unwrap();
        parent.begin_revocation("provider-a").unwrap();
        assert_eq!(
            parent.reserve("provider-a", child_intent(child_scope("operator:a", "dest:a"), "effect:2", 100)),
            Err(ExposureError::InvalidTransition)
        );
        // Existing exact retries resolve idempotently during revocation.
        assert_eq!(
            parent.reserve("provider-a", intent),
            Ok(ExposureDisposition::AlreadyReserved)
        );
        parent.mark_dispatch_authorized("provider-a", "effect:1").unwrap();
        parent.mark_committed_unsettled("provider-a", "effect:1").unwrap();
        parent.mark_settled("provider-a", "effect:1", &final_settlement("effect:1", "dest:a")).unwrap();
        parent.close_grant("provider-a").unwrap();
        assert_eq!(parent.available_parent_budget_minor(), Ok(1_000));
    }

    #[test]
    fn unresolved_child_effect_prevents_parent_budget_reclamation() {
        let mut parent = ParentDelegatedExposureBookV1::new(parent_policy(1_000)).unwrap();
        parent.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 700, 1, digest('1')).unwrap();
        parent.reserve("provider-a", child_intent(child_scope("operator:a", "dest:a"), "effect:1", 400)).unwrap();
        parent.mark_dispatch_authorized("provider-a", "effect:1").unwrap();
        parent.mark_indeterminate("provider-a", "effect:1").unwrap();
        parent.begin_revocation("provider-a").unwrap();
        assert_eq!(parent.close_grant("provider-a"), Err(ExposureError::InvalidTransition));
        assert_eq!(parent.available_parent_budget_minor(), Ok(300));
        parent.release_authoritative_absence("provider-a", "effect:1", &absence("effect:1", "dest:a")).unwrap();
        parent.close_grant("provider-a").unwrap();
        assert_eq!(parent.available_parent_budget_minor(), Ok(1_000));
    }

    #[test]
    fn changed_grant_terms_and_cross_pool_or_currency_are_rejected() {
        let mut parent = ParentDelegatedExposureBookV1::new(parent_policy(1_000)).unwrap();
        parent.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 700, 1, digest('1')).unwrap();
        assert_eq!(
            parent.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 600, 1, digest('1')),
            Err(ExposureError::ReservationConflict)
        );
        assert_eq!(
            parent.allocate_child_budget("provider-b", child_scope("operator:b", "dest:b"), 200, 2, digest('2')),
            Ok(DelegatedBudgetDisposition::Allocated)
        );
        assert_eq!(parent.available_parent_budget_minor(), Ok(100));
        let mut wrong_currency = child_scope("operator:b", "dest:b");
        wrong_currency.currency = "USD".into();
        assert_eq!(
            parent.allocate_child_budget("provider-c", wrong_currency, 100, 3, digest('3')),
            Err(ExposureError::ScopeMismatch)
        );
    }

    #[test]
    fn delegated_snapshot_is_insertion_order_independent() {
        let mut first = ParentDelegatedExposureBookV1::new(parent_policy(1_000)).unwrap();
        first.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 300, 1, digest('1')).unwrap();
        first.allocate_child_budget("provider-b", child_scope("operator:b", "dest:b"), 400, 2, digest('2')).unwrap();

        let mut second = ParentDelegatedExposureBookV1::new(parent_policy(1_000)).unwrap();
        second.allocate_child_budget("provider-b", child_scope("operator:b", "dest:b"), 400, 2, digest('2')).unwrap();
        second.allocate_child_budget("provider-a", child_scope("operator:a", "dest:a"), 300, 1, digest('1')).unwrap();
        assert_eq!(
            first.canonical_snapshot_digest().unwrap(),
            second.canonical_snapshot_digest().unwrap()
        );
    }
}
