//! Effect-level, liquidity-backed financial compensation reference model.
//!
//! A compensation is a new financial effect. It never rewrites the forward
//! effect. This effects-disabled model checks subjects, identities, authority
//! scope, liquidity reservations and cumulative reversible limits; it does not
//! authenticate external signatures or account balances.

use std::collections::BTreeMap;

use sha2::{Digest, Sha256};

const FORWARD_DOMAIN: &[u8] = b"MXSAP-FORWARD-EFFECT-V2\0";
const PLAN_DOMAIN: &[u8] = b"MXSAP-COMPENSATION-PLAN-V2\0";
const AUTH_DOMAIN: &[u8] = b"MXSAP-COMPENSATION-AUTH-V2\0";
const RESERVATION_DOMAIN: &[u8] = b"MXSAP-COMPENSATION-LIQUIDITY-V2\0";
const REQUEST_DOMAIN: &[u8] = b"MXSAP-COMPENSATION-REQUEST-V2\0";
const OUTCOME_DOMAIN: &[u8] = b"MXSAP-COMPENSATION-OUTCOME-V2\0";
const MAX_FIELD_BYTES: usize = 4096;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ForwardFinalityV2 {
    Indeterminate = 1,
    ObservedUnsettled = 2,
    Settled = 3,
    AuthoritativelyAbsent = 4,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ForwardEffectV2 {
    pub operation_id: String,
    pub effect_id: String,
    pub source_owner_id: String,
    pub source_account_id: String,
    pub destination_owner_id: String,
    pub destination_account_id: String,
    pub asset: String,
    pub amount_minor: u64,
    pub finality: ForwardFinalityV2,
    pub reversible_ceiling_minor: u64,
}

impl ForwardEffectV2 {
    pub fn validate(&self) -> Result<(), CompensationErrorV2> {
        for value in [
            &self.operation_id,
            &self.effect_id,
            &self.source_owner_id,
            &self.source_account_id,
            &self.destination_owner_id,
            &self.destination_account_id,
            &self.asset,
        ] {
            validate_text(value)?;
        }
        if self.source_owner_id == self.destination_owner_id
            && self.source_account_id == self.destination_account_id
        {
            return Err(CompensationErrorV2::InvalidForwardEffect);
        }
        if !matches!(
            self.finality,
            ForwardFinalityV2::Indeterminate
                | ForwardFinalityV2::ObservedUnsettled
                | ForwardFinalityV2::Settled
                | ForwardFinalityV2::AuthoritativelyAbsent
        ) {
            return Err(CompensationErrorV2::InvalidFinality);
        }
        if self.amount_minor == 0 || self.reversible_ceiling_minor > self.amount_minor {
            return Err(CompensationErrorV2::InvalidForwardEffect);
        }
        if self.finality != ForwardFinalityV2::Settled && self.reversible_ceiling_minor != 0 {
            return Err(CompensationErrorV2::ForwardNotSettled);
        }
        Ok(())
    }

    pub fn canonical_digest(&self) -> Result<String, CompensationErrorV2> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(FORWARD_DOMAIN);
        for value in [
            &self.operation_id,
            &self.effect_id,
            &self.source_owner_id,
            &self.source_account_id,
            &self.destination_owner_id,
            &self.destination_account_id,
            &self.asset,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.amount_minor);
        bytes.push(self.finality as u8);
        append_u64(&mut bytes, self.reversible_ceiling_minor);
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompensationPlanV2 {
    pub plan_id: String,
    pub forward_effect_id: String,
    pub forward_effect_digest: String,
    pub funding_owner_id: String,
    pub funding_account_id: String,
    pub beneficiary_owner_id: String,
    pub beneficiary_account_id: String,
    pub asset: String,
    pub maximum_compensation_minor: u64,
    pub policy_digest: String,
    pub recovery_reason: String,
}

impl CompensationPlanV2 {
    pub fn validate(&self, forward: &ForwardEffectV2) -> Result<(), CompensationErrorV2> {
        forward.validate()?;
        for value in [
            &self.plan_id,
            &self.forward_effect_id,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
            &self.recovery_reason,
        ] {
            validate_text(value)?;
        }
        validate_digest(&self.forward_effect_digest)?;
        validate_digest(&self.policy_digest)?;
        if forward.finality != ForwardFinalityV2::Settled {
            return Err(CompensationErrorV2::ForwardNotSettled);
        }
        if self.forward_effect_id != forward.effect_id
            || self.forward_effect_digest != forward.canonical_digest()?
            || self.beneficiary_owner_id != forward.source_owner_id
            || self.beneficiary_account_id != forward.source_account_id
            || self.asset != forward.asset
            || self.maximum_compensation_minor == 0
            || self.maximum_compensation_minor > forward.reversible_ceiling_minor
        {
            return Err(CompensationErrorV2::InvalidPlan);
        }
        Ok(())
    }

    pub fn canonical_digest(
        &self,
        forward: &ForwardEffectV2,
    ) -> Result<String, CompensationErrorV2> {
        self.validate(forward)?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(PLAN_DOMAIN);
        for value in [
            &self.plan_id,
            &self.forward_effect_id,
            &self.forward_effect_digest,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
            &self.policy_digest,
            &self.recovery_reason,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.maximum_compensation_minor);
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompensationAuthorizationV2 {
    pub capability_id: String,
    pub issuer_owner_id: String,
    pub grantee_owner_id: String,
    pub plan_digest: String,
    pub funding_owner_id: String,
    pub funding_account_id: String,
    pub beneficiary_owner_id: String,
    pub beneficiary_account_id: String,
    pub asset: String,
    pub maximum_minor: u64,
    pub nonce: u64,
    pub expires_at_unix_ms: u64,
    pub revoked: bool,
}

impl CompensationAuthorizationV2 {
    pub fn validate(
        &self,
        plan: &CompensationPlanV2,
        plan_digest: &str,
        amount_minor: u64,
        actor_owner_id: &str,
        now_unix_ms: u64,
    ) -> Result<(), CompensationErrorV2> {
        for value in [
            &self.capability_id,
            &self.issuer_owner_id,
            &self.grantee_owner_id,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
        ] {
            validate_text(value)?;
        }
        validate_digest(&self.plan_digest)?;
        if self.revoked {
            return Err(CompensationErrorV2::AuthorizationRevoked);
        }
        if self.expires_at_unix_ms <= now_unix_ms {
            return Err(CompensationErrorV2::AuthorizationExpired);
        }
        if self.grantee_owner_id != actor_owner_id
            || self.nonce == 0
            || self.maximum_minor == 0
            || amount_minor == 0
            || amount_minor > self.maximum_minor
            || self.plan_digest != plan_digest
            || self.issuer_owner_id != plan.funding_owner_id
            || self.funding_owner_id != plan.funding_owner_id
            || self.funding_account_id != plan.funding_account_id
            || self.beneficiary_owner_id != plan.beneficiary_owner_id
            || self.beneficiary_account_id != plan.beneficiary_account_id
            || self.asset != plan.asset
            || self.maximum_minor > plan.maximum_compensation_minor
        {
            return Err(CompensationErrorV2::AuthorizationScopeMismatch);
        }
        Ok(())
    }

    pub fn canonical_scope_digest(&self) -> Result<String, CompensationErrorV2> {
        for value in [
            &self.capability_id,
            &self.issuer_owner_id,
            &self.grantee_owner_id,
            &self.plan_digest,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
        ] {
            validate_text(value)?;
        }
        validate_digest(&self.plan_digest)?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(AUTH_DOMAIN);
        for value in [
            &self.capability_id,
            &self.issuer_owner_id,
            &self.grantee_owner_id,
            &self.plan_digest,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.maximum_minor);
        append_u64(&mut bytes, self.nonce);
        append_u64(&mut bytes, self.expires_at_unix_ms);
        Ok(sha256_hex(&bytes))
    }

    pub fn canonical_digest(&self) -> Result<String, CompensationErrorV2> {
        for value in [
            &self.capability_id,
            &self.issuer_owner_id,
            &self.grantee_owner_id,
            &self.plan_digest,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
        ] {
            validate_text(value)?;
        }
        validate_digest(&self.plan_digest)?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(AUTH_DOMAIN);
        for value in [
            &self.capability_id,
            &self.issuer_owner_id,
            &self.grantee_owner_id,
            &self.plan_digest,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.maximum_minor);
        append_u64(&mut bytes, self.nonce);
        append_u64(&mut bytes, self.expires_at_unix_ms);
        bytes.push(u8::from(self.revoked));
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LiquidityReservationV2 {
    pub reservation_id: String,
    pub source_owner_id: String,
    pub source_account_id: String,
    pub asset: String,
    pub amount_minor: u64,
}

impl LiquidityReservationV2 {
    pub fn validate(
        &self,
        plan: &CompensationPlanV2,
        amount_minor: u64,
    ) -> Result<(), CompensationErrorV2> {
        for value in [
            &self.reservation_id,
            &self.source_owner_id,
            &self.source_account_id,
            &self.asset,
        ] {
            validate_text(value)?;
        }
        if self.source_owner_id != plan.funding_owner_id
            || self.source_account_id != plan.funding_account_id
            || self.asset != plan.asset
            || self.amount_minor != amount_minor
            || self.amount_minor == 0
        {
            return Err(CompensationErrorV2::LiquidityReservationMismatch);
        }
        Ok(())
    }

    pub fn canonical_digest(&self) -> Result<String, CompensationErrorV2> {
        for value in [
            &self.reservation_id,
            &self.source_owner_id,
            &self.source_account_id,
            &self.asset,
        ] {
            validate_text(value)?;
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(RESERVATION_DOMAIN);
        for value in [
            &self.reservation_id,
            &self.source_owner_id,
            &self.source_account_id,
            &self.asset,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.amount_minor);
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompensationRequestV2 {
    pub compensation_id: String,
    pub actor_owner_id: String,
    pub amount_minor: u64,
    pub authorization: CompensationAuthorizationV2,
    pub liquidity_reservation: LiquidityReservationV2,
}

impl CompensationRequestV2 {
    pub fn canonical_digest(&self) -> Result<String, CompensationErrorV2> {
        validate_text(&self.compensation_id)?;
        validate_text(&self.actor_owner_id)?;
        let auth_digest = self.authorization.canonical_digest()?;
        let reservation_digest = self.liquidity_reservation.canonical_digest()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(REQUEST_DOMAIN);
        append_text(&mut bytes, &self.compensation_id)?;
        append_text(&mut bytes, &self.actor_owner_id)?;
        append_u64(&mut bytes, self.amount_minor);
        append_text(&mut bytes, &auth_digest)?;
        append_text(&mut bytes, &reservation_digest)?;
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompensationStatusV2 {
    Reserved = 1,
    DispatchAuthorized = 2,
    Indeterminate = 3,
    AuthoritativelyAbsent = 4,
    ObservedUnsettled = 5,
    Settled = 6,
    Cancelled = 7,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompensationDispositionV2 {
    Created,
    AlreadyExists,
    Applied,
    AlreadyApplied,
    Settled,
    AlreadySettled,
    Cancelled,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompensationOutcomeKindV2 {
    AuthoritativeAbsence = 1,
    ObservedUnsettled = 2,
    FinalSettlement = 3,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompensationOutcomeV2 {
    pub kind: CompensationOutcomeKindV2,
    pub compensation_id: String,
    pub compensation_effect_id: String,
    pub forward_effect_id: String,
    pub forward_effect_digest: String,
    pub plan_digest: String,
    pub amount_minor: u64,
    pub authorization_digest: String,
    pub liquidity_reservation_digest: String,
    pub funding_owner_id: String,
    pub funding_account_id: String,
    pub beneficiary_owner_id: String,
    pub beneficiary_account_id: String,
    pub asset: String,
    pub evidence_reference: String,
}

impl CompensationOutcomeV2 {
    pub fn canonical_digest(&self) -> Result<String, CompensationErrorV2> {
        for value in [
            &self.compensation_id,
            &self.compensation_effect_id,
            &self.forward_effect_id,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
            &self.evidence_reference,
        ] {
            validate_text(value)?;
        }
        validate_digest(&self.forward_effect_digest)?;
        validate_digest(&self.plan_digest)?;
        validate_digest(&self.authorization_digest)?;
        validate_digest(&self.liquidity_reservation_digest)?;
        if self.amount_minor == 0 {
            return Err(CompensationErrorV2::InvalidOutcome);
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(OUTCOME_DOMAIN);
        bytes.push(self.kind as u8);
        for value in [
            &self.compensation_id,
            &self.compensation_effect_id,
            &self.forward_effect_id,
            &self.forward_effect_digest,
            &self.plan_digest,
            &self.authorization_digest,
            &self.liquidity_reservation_digest,
            &self.funding_owner_id,
            &self.funding_account_id,
            &self.beneficiary_owner_id,
            &self.beneficiary_account_id,
            &self.asset,
            &self.evidence_reference,
        ] {
            append_text(&mut bytes, value)?;
        }
        append_u64(&mut bytes, self.amount_minor);
        Ok(sha256_hex(&bytes))
    }
}

#[derive(Clone, Debug)]
struct CompensationRecordV2 {
    request: CompensationRequestV2,
    request_digest: String,
    status: CompensationStatusV2,
    possible_effect_count: u8,
    last_outcome_digest: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompensationErrorV2 {
    InvalidText,
    InvalidDigest,
    InvalidForwardEffect,
    InvalidFinality,
    ForwardNotSettled,
    InvalidPlan,
    AuthorizationRevoked,
    AuthorizationExpired,
    AuthorizationScopeMismatch,
    CapabilityIdentityConflict,
    CallerNotAuthorized,
    AuthorizationLimitExceeded,
    LiquidityReservationMismatch,
    LiquidityReservationConflict,
    InsufficientLiquidity,
    InsufficientReversibleCeiling,
    CompensationConflict,
    CompensationNotFound,
    InvalidTransition,
    InvalidOutcome,
    OutcomeConflict,
    OutcomeContradiction,
    AmountOverflow,
}

impl std::fmt::Display for CompensationErrorV2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for CompensationErrorV2 {}

#[derive(Clone, Debug)]
pub struct CompensationBookV2 {
    forward: ForwardEffectV2,
    plan: CompensationPlanV2,
    plan_digest: String,
    initial_liquidity_minor: u64,
    records: BTreeMap<String, CompensationRecordV2>,
}

impl CompensationBookV2 {
    pub fn new(
        forward: ForwardEffectV2,
        plan: CompensationPlanV2,
        initial_liquidity_minor: u64,
    ) -> Result<Self, CompensationErrorV2> {
        plan.validate(&forward)?;
        let plan_digest = plan.canonical_digest(&forward)?;
        Ok(Self {
            forward,
            plan,
            plan_digest,
            initial_liquidity_minor,
            records: BTreeMap::new(),
        })
    }

    pub fn forward(&self) -> &ForwardEffectV2 {
        &self.forward
    }

    pub fn plan(&self) -> &CompensationPlanV2 {
        &self.plan
    }

    pub fn plan_digest(&self) -> &str {
        &self.plan_digest
    }

    pub fn reserve(
        &mut self,
        request: CompensationRequestV2,
        now_unix_ms: u64,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        if self.forward.finality != ForwardFinalityV2::Settled {
            return Err(CompensationErrorV2::ForwardNotSettled);
        }
        let request_digest = request.canonical_digest()?;
        validate_text(&request.compensation_id)?;
        validate_text(&request.actor_owner_id)?;
        if let Some(existing) = self.records.get(&request.compensation_id) {
            return if existing.request_digest == request_digest {
                Ok(CompensationDispositionV2::AlreadyExists)
            } else {
                Err(CompensationErrorV2::CompensationConflict)
            };
        }
        request.authorization.validate(
            &self.plan,
            &self.plan_digest,
            request.amount_minor,
            &request.actor_owner_id,
            now_unix_ms,
        )?;
        request
            .liquidity_reservation
            .validate(&self.plan, request.amount_minor)?;
        if self.records.values().any(|record| {
            record.request.liquidity_reservation.reservation_id
                == request.liquidity_reservation.reservation_id
        }) {
            return Err(CompensationErrorV2::LiquidityReservationConflict);
        }

        // Capability IDs are immutable identities. A changed scope cannot
        // evade a previously consumed cumulative grant ceiling.
        let capability_digest = request.authorization.canonical_scope_digest()?;
        let capability_used = self.records.values().try_fold(
            0_u64,
            |total, record| {
                if record.request.authorization.capability_id
                    != request.authorization.capability_id
                {
                    return Ok(total);
                }
                let existing_digest = record.request.authorization.canonical_scope_digest()?;
                if existing_digest != capability_digest {
                    return Err(CompensationErrorV2::CapabilityIdentityConflict);
                }
                let amount = record.request.amount_minor;
                let counted = match record.status {
                    CompensationStatusV2::Settled
                    | CompensationStatusV2::Reserved
                    | CompensationStatusV2::DispatchAuthorized
                    | CompensationStatusV2::Indeterminate
                    | CompensationStatusV2::AuthoritativelyAbsent
                    | CompensationStatusV2::ObservedUnsettled => amount,
                    CompensationStatusV2::Cancelled => 0,
                };
                total
                    .checked_add(counted)
                    .ok_or(CompensationErrorV2::AmountOverflow)
            },
        )?;
        let projected_capability_use = capability_used
            .checked_add(request.amount_minor)
            .ok_or(CompensationErrorV2::AmountOverflow)?;
        if projected_capability_use > request.authorization.maximum_minor {
            return Err(CompensationErrorV2::AuthorizationLimitExceeded);
        }

        let (settled, encumbered) = self.totals()?;
        let used = settled
            .checked_add(encumbered)
            .ok_or(CompensationErrorV2::AmountOverflow)?;
        let projected = used
            .checked_add(request.amount_minor)
            .ok_or(CompensationErrorV2::AmountOverflow)?;
        if projected > self.plan.maximum_compensation_minor {
            return Err(CompensationErrorV2::InsufficientReversibleCeiling);
        }
        if projected > self.initial_liquidity_minor {
            return Err(CompensationErrorV2::InsufficientLiquidity);
        }

        self.records.insert(
            request.compensation_id.clone(),
            CompensationRecordV2 {
                request,
                request_digest,
                status: CompensationStatusV2::Reserved,
                possible_effect_count: 0,
                last_outcome_digest: None,
            },
        );
        Ok(CompensationDispositionV2::Created)
    }

    pub fn mark_dispatch_authorized(
        &mut self,
        compensation_id: &str,
        actor_owner_id: &str,
        current_authorization: &CompensationAuthorizationV2,
        now_unix_ms: u64,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        let record = self
            .records
            .get_mut(compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        if actor_owner_id != record.request.actor_owner_id
            || actor_owner_id != current_authorization.grantee_owner_id
        {
            return Err(CompensationErrorV2::CallerNotAuthorized);
        }
        if actor_owner_id != record.request.actor_owner_id
            || actor_owner_id != current_authorization.grantee_owner_id
        {
            return Err(CompensationErrorV2::CallerNotAuthorized);
        }
        let stored_digest = record.request.authorization.canonical_scope_digest()?;
        if current_authorization.canonical_scope_digest()? != stored_digest {
            return Err(CompensationErrorV2::AuthorizationScopeMismatch);
        }
        current_authorization.validate(
            &self.plan,
            &self.plan_digest,
            record.request.amount_minor,
            actor_owner_id,
            now_unix_ms,
        )?;
        match record.status {
            CompensationStatusV2::Reserved => {
                record.status = CompensationStatusV2::DispatchAuthorized;
                Ok(CompensationDispositionV2::Applied)
            }
            CompensationStatusV2::DispatchAuthorized => {
                Ok(CompensationDispositionV2::AlreadyApplied)
            }
            _ => Err(CompensationErrorV2::InvalidTransition),
        }
    }

    pub fn mark_indeterminate(
        &mut self,
        compensation_id: &str,
        possible_effect_count: u8,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        if possible_effect_count > 1 {
            return Err(CompensationErrorV2::OutcomeContradiction);
        }
        let record = self
            .records
            .get_mut(compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        match record.status {
            CompensationStatusV2::DispatchAuthorized => {
                record.status = CompensationStatusV2::Indeterminate;
                record.possible_effect_count = possible_effect_count;
                Ok(CompensationDispositionV2::Applied)
            }
            CompensationStatusV2::Indeterminate
                if record.possible_effect_count == possible_effect_count =>
            {
                Ok(CompensationDispositionV2::AlreadyApplied)
            }
            CompensationStatusV2::Indeterminate => {
                Err(CompensationErrorV2::OutcomeConflict)
            }
            _ => Err(CompensationErrorV2::InvalidTransition),
        }
    }

    pub fn resolve_authoritative_absence(
        &mut self,
        outcome: CompensationOutcomeV2,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        self.validate_outcome(&outcome, CompensationOutcomeKindV2::AuthoritativeAbsence)?;
        let digest = outcome.canonical_digest()?;
        let record = self
            .records
            .get_mut(&outcome.compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        if record.status == CompensationStatusV2::AuthoritativelyAbsent {
            return if record.last_outcome_digest.as_deref() == Some(digest.as_str()) {
                Ok(CompensationDispositionV2::AlreadyApplied)
            } else {
                Err(CompensationErrorV2::OutcomeConflict)
            };
        }
        if record.status != CompensationStatusV2::Indeterminate {
            return Err(CompensationErrorV2::InvalidTransition);
        }
        if record.possible_effect_count != 0 {
            return Err(CompensationErrorV2::OutcomeContradiction);
        }
        record.status = CompensationStatusV2::AuthoritativelyAbsent;
        record.last_outcome_digest = Some(digest);
        Ok(CompensationDispositionV2::Applied)
    }

    pub fn retry_same_effect(
        &mut self,
        compensation_id: &str,
        actor_owner_id: &str,
        current_authorization: &CompensationAuthorizationV2,
        now_unix_ms: u64,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        let record = self
            .records
            .get_mut(compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        if record.status != CompensationStatusV2::AuthoritativelyAbsent {
            return Err(CompensationErrorV2::InvalidTransition);
        }
        let stored_digest = record.request.authorization.canonical_scope_digest()?;
        if current_authorization.canonical_scope_digest()? != stored_digest {
            return Err(CompensationErrorV2::AuthorizationScopeMismatch);
        }
        current_authorization.validate(
            &self.plan,
            &self.plan_digest,
            record.request.amount_minor,
            actor_owner_id,
            now_unix_ms,
        )?;
        record.status = CompensationStatusV2::Reserved;
        record.possible_effect_count = 0;
        record.last_outcome_digest = None;
        Ok(CompensationDispositionV2::Applied)
    }

    pub fn observe_unsettled(
        &mut self,
        outcome: CompensationOutcomeV2,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        self.validate_outcome(&outcome, CompensationOutcomeKindV2::ObservedUnsettled)?;
        let digest = outcome.canonical_digest()?;
        let record = self
            .records
            .get_mut(&outcome.compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        if record.status == CompensationStatusV2::ObservedUnsettled {
            return if record.last_outcome_digest.as_deref() == Some(digest.as_str()) {
                Ok(CompensationDispositionV2::AlreadyApplied)
            } else {
                Err(CompensationErrorV2::OutcomeConflict)
            };
        }
        if record.status != CompensationStatusV2::Indeterminate
            || record.possible_effect_count != 1
        {
            return Err(CompensationErrorV2::OutcomeContradiction);
        }
        record.status = CompensationStatusV2::ObservedUnsettled;
        record.last_outcome_digest = Some(digest);
        Ok(CompensationDispositionV2::Applied)
    }

    pub fn settle(
        &mut self,
        outcome: CompensationOutcomeV2,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        self.validate_outcome(&outcome, CompensationOutcomeKindV2::FinalSettlement)?;
        let digest = outcome.canonical_digest()?;
        let record = self
            .records
            .get_mut(&outcome.compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        if record.status == CompensationStatusV2::Settled {
            return if record.last_outcome_digest.as_deref() == Some(digest.as_str()) {
                Ok(CompensationDispositionV2::AlreadySettled)
            } else {
                Err(CompensationErrorV2::OutcomeConflict)
            };
        }
        if !matches!(
            record.status,
            CompensationStatusV2::DispatchAuthorized
                | CompensationStatusV2::Indeterminate
                | CompensationStatusV2::ObservedUnsettled
        ) {
            return Err(CompensationErrorV2::InvalidTransition);
        }
        if record.status == CompensationStatusV2::Indeterminate
            && record.possible_effect_count != 1
        {
            return Err(CompensationErrorV2::OutcomeContradiction);
        }
        record.status = CompensationStatusV2::Settled;
        record.possible_effect_count = 1;
        record.last_outcome_digest = Some(digest);
        Ok(CompensationDispositionV2::Settled)
    }

    pub fn cancel_before_dispatch(
        &mut self,
        compensation_id: &str,
        actor_owner_id: &str,
    ) -> Result<CompensationDispositionV2, CompensationErrorV2> {
        let record = self
            .records
            .get_mut(compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        if actor_owner_id != record.request.actor_owner_id
            && actor_owner_id != self.plan.funding_owner_id
        {
            return Err(CompensationErrorV2::CallerNotAuthorized);
        }
        match record.status {
            CompensationStatusV2::Reserved | CompensationStatusV2::AuthoritativelyAbsent => {
                record.status = CompensationStatusV2::Cancelled;
                Ok(CompensationDispositionV2::Cancelled)
            }
            CompensationStatusV2::Cancelled => Ok(CompensationDispositionV2::Cancelled),
            _ => Err(CompensationErrorV2::InvalidTransition),
        }
    }

    pub fn compensation_effect_id(&self, compensation_id: &str) -> Option<String> {
        self.records
            .get(compensation_id)
            .map(|_| format!("mx-compensation:{compensation_id}"))
    }

    pub fn status(&self, compensation_id: &str) -> Option<CompensationStatusV2> {
        self.records.get(compensation_id).map(|record| record.status)
    }

    pub fn settled_minor(&self) -> Result<u64, CompensationErrorV2> {
        Ok(self.totals()?.0)
    }

    pub fn encumbered_minor(&self) -> Result<u64, CompensationErrorV2> {
        Ok(self.totals()?.1)
    }

    pub fn remaining_reversible_minor(&self) -> Result<u64, CompensationErrorV2> {
        let (settled, encumbered) = self.totals()?;
        Ok(self
            .plan
            .maximum_compensation_minor
            .saturating_sub(settled.saturating_add(encumbered)))
    }

    pub fn available_liquidity_minor(&self) -> Result<u64, CompensationErrorV2> {
        let (settled, encumbered) = self.totals()?;
        Ok(self
            .initial_liquidity_minor
            .saturating_sub(settled.saturating_add(encumbered)))
    }

    fn validate_outcome(
        &self,
        outcome: &CompensationOutcomeV2,
        expected_kind: CompensationOutcomeKindV2,
    ) -> Result<(), CompensationErrorV2> {
        outcome.canonical_digest()?;
        let record = self
            .records
            .get(&outcome.compensation_id)
            .ok_or(CompensationErrorV2::CompensationNotFound)?;
        let expected_effect_id = format!("mx-compensation:{}", outcome.compensation_id);
        if outcome.kind != expected_kind
            || outcome.authorization_digest != record.request.authorization.canonical_digest()?
            || outcome.liquidity_reservation_digest
                != record.request.liquidity_reservation.canonical_digest()?
            || outcome.compensation_effect_id != expected_effect_id
            || outcome.forward_effect_id != self.forward.effect_id
            || outcome.forward_effect_digest != self.forward.canonical_digest()?
            || outcome.plan_digest != self.plan_digest
            || outcome.amount_minor != record.request.amount_minor
            || outcome.funding_owner_id != self.plan.funding_owner_id
            || outcome.funding_account_id != self.plan.funding_account_id
            || outcome.beneficiary_owner_id != self.plan.beneficiary_owner_id
            || outcome.beneficiary_account_id != self.plan.beneficiary_account_id
            || outcome.asset != self.plan.asset
        {
            return Err(CompensationErrorV2::InvalidOutcome);
        }
        Ok(())
    }

    fn totals(&self) -> Result<(u64, u64), CompensationErrorV2> {
        self.records.values().try_fold(
            (0_u64, 0_u64),
            |(settled, encumbered), record| {
                let amount = record.request.amount_minor;
                match record.status {
                    CompensationStatusV2::Settled => Ok((
                        settled.checked_add(amount).ok_or(CompensationErrorV2::AmountOverflow)?,
                        encumbered,
                    )),
                    CompensationStatusV2::Reserved
                    | CompensationStatusV2::DispatchAuthorized
                    | CompensationStatusV2::Indeterminate
                    | CompensationStatusV2::AuthoritativelyAbsent
                    | CompensationStatusV2::ObservedUnsettled => Ok((
                        settled,
                        encumbered.checked_add(amount).ok_or(CompensationErrorV2::AmountOverflow)?,
                    )),
                    CompensationStatusV2::Cancelled => Ok((settled, encumbered)),
                }
            },
        )
    }
}

fn validate_text(value: &str) -> Result<(), CompensationErrorV2> {
    if value.is_empty() || value.len() > MAX_FIELD_BYTES {
        return Err(CompensationErrorV2::InvalidText);
    }
    Ok(())
}

fn validate_digest(value: &str) -> Result<(), CompensationErrorV2> {
    if value.len() != 71
        || !value.starts_with("sha256:")
        || !value[7..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(CompensationErrorV2::InvalidDigest);
    }
    Ok(())
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), CompensationErrorV2> {
    validate_text(value)?;
    let length = u32::try_from(value.len()).map_err(|_| CompensationErrorV2::InvalidText)?;
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

    const NOW: u64 = 1_000;

    fn digest(ch: char) -> String {
        format!("sha256:{}", ch.to_string().repeat(64))
    }

    fn forward() -> ForwardEffectV2 {
        ForwardEffectV2 {
            operation_id: "operation:merchant-01".into(),
            effect_id: "effect:payment-01".into(),
            source_owner_id: "owner:merchant".into(),
            source_account_id: "account:merchant".into(),
            destination_owner_id: "owner:customer".into(),
            destination_account_id: "account:customer".into(),
            asset: "USD".into(),
            amount_minor: 100,
            finality: ForwardFinalityV2::Settled,
            reversible_ceiling_minor: 100,
        }
    }

    fn plan(forward: &ForwardEffectV2, maximum: u64) -> CompensationPlanV2 {
        CompensationPlanV2 {
            plan_id: "plan:refund-01".into(),
            forward_effect_id: forward.effect_id.clone(),
            forward_effect_digest: forward.canonical_digest().unwrap(),
            funding_owner_id: "owner:refund-treasury".into(),
            funding_account_id: "account:refund-treasury".into(),
            beneficiary_owner_id: forward.source_owner_id.clone(),
            beneficiary_account_id: forward.source_account_id.clone(),
            asset: forward.asset.clone(),
            maximum_compensation_minor: maximum,
            policy_digest: digest('d'),
            recovery_reason: "customer-refund".into(),
        }
    }

    fn authorization(plan: &CompensationPlanV2, maximum: u64) -> CompensationAuthorizationV2 {
        let plan_digest = plan.canonical_digest(&forward()).unwrap();
        CompensationAuthorizationV2 {
            capability_id: "capability:refund-01".into(),
            issuer_owner_id: plan.funding_owner_id.clone(),
            grantee_owner_id: "owner:refund-operator".into(),
            plan_digest,
            funding_owner_id: plan.funding_owner_id.clone(),
            funding_account_id: plan.funding_account_id.clone(),
            beneficiary_owner_id: plan.beneficiary_owner_id.clone(),
            beneficiary_account_id: plan.beneficiary_account_id.clone(),
            asset: plan.asset.clone(),
            maximum_minor: maximum,
            nonce: 1,
            expires_at_unix_ms: 10_000,
            revoked: false,
        }
    }

    fn book(initial_liquidity: u64, plan_maximum: u64) -> CompensationBookV2 {
        let forward = forward();
        let plan = plan(&forward, plan_maximum);
        CompensationBookV2::new(forward, plan, initial_liquidity).unwrap()
    }

    fn request(
        book: &CompensationBookV2,
        id: &str,
        amount: u64,
        reservation_id: &str,
        auth_maximum: u64,
    ) -> CompensationRequestV2 {
        CompensationRequestV2 {
            compensation_id: id.into(),
            actor_owner_id: "owner:refund-operator".into(),
            amount_minor: amount,
            authorization: authorization(&book.plan, auth_maximum),
            liquidity_reservation: LiquidityReservationV2 {
                reservation_id: reservation_id.into(),
                source_owner_id: book.plan.funding_owner_id.clone(),
                source_account_id: book.plan.funding_account_id.clone(),
                asset: book.plan.asset.clone(),
                amount_minor: amount,
            },
        }
    }

    fn outcome(
        book: &CompensationBookV2,
        request: &CompensationRequestV2,
        kind: CompensationOutcomeKindV2,
        evidence_reference: &str,
    ) -> CompensationOutcomeV2 {
        CompensationOutcomeV2 {
            kind,
            compensation_id: request.compensation_id.clone(),
            compensation_effect_id: format!("mx-compensation:{}", request.compensation_id),
            forward_effect_id: book.forward.effect_id.clone(),
            forward_effect_digest: book.forward.canonical_digest().unwrap(),
            plan_digest: book.plan_digest.clone(),
            amount_minor: request.amount_minor,
            authorization_digest: request.authorization.canonical_digest().unwrap(),
            liquidity_reservation_digest: request
                .liquidity_reservation
                .canonical_digest()
                .unwrap(),
            funding_owner_id: book.plan.funding_owner_id.clone(),
            funding_account_id: book.plan.funding_account_id.clone(),
            beneficiary_owner_id: book.plan.beneficiary_owner_id.clone(),
            beneficiary_account_id: book.plan.beneficiary_account_id.clone(),
            asset: book.plan.asset.clone(),
            evidence_reference: evidence_reference.into(),
        }
    }

    #[test]
    fn non_settled_forward_effect_cannot_create_compensation_plan() {
        let mut unsettled = forward();
        unsettled.finality = ForwardFinalityV2::ObservedUnsettled;
        unsettled.reversible_ceiling_minor = 0;
        let plan = plan(&unsettled, 0);
        assert_eq!(
            CompensationBookV2::new(unsettled, plan, 100).map(|_| ()),
            Err(CompensationErrorV2::ForwardNotSettled)
        );

        let mut absent = forward();
        absent.finality = ForwardFinalityV2::AuthoritativelyAbsent;
        absent.reversible_ceiling_minor = 0;
        let absent_plan = plan(&absent, 0);
        assert_eq!(
            CompensationBookV2::new(absent, absent_plan, 100).map(|_| ()),
            Err(CompensationErrorV2::ForwardNotSettled)
        );
    }

    #[test]
    fn partial_compensation_is_bounded_by_forward_reversible_ceiling() {
        let mut book = book(100, 100);
        let original_forward = book.forward.clone();
        let first = request(&book, "refund:1", 60, "liquidity:1", 100);
        assert_eq!(
            book.reserve(first.clone(), NOW),
            Ok(CompensationDispositionV2::Created)
        );
        assert_eq!(
            book.mark_dispatch_authorized("refund:1", "owner:refund-operator", &first.authorization, NOW),
            Ok(CompensationDispositionV2::Applied)
        );
        let settlement = outcome(
            &book,
            &first,
            CompensationOutcomeKindV2::FinalSettlement,
            "evidence:refund-1-settled",
        );
        assert_eq!(
            book.settle(settlement),
            Ok(CompensationDispositionV2::Settled)
        );
        assert_eq!(book.settled_minor(), Ok(60));
        assert_eq!(book.encumbered_minor(), Ok(0));
        assert_eq!(book.remaining_reversible_minor(), Ok(40));

        let second = request(&book, "refund:2", 41, "liquidity:2", 100);
        assert_eq!(
            book.reserve(second, NOW),
            Err(CompensationErrorV2::InsufficientReversibleCeiling)
        );
        assert_eq!(book.forward, original_forward);
        assert_eq!(book.settled_minor(), Ok(60));
    }

    #[test]
    fn indeterminate_compensation_keeps_liquidity_and_cannot_claim_absence_after_effect() {
        let mut book = book(100, 100);
        let request = request(&book, "refund:uncertain", 35, "liquidity:uncertain", 100);
        book.reserve(request.clone(), NOW).unwrap();
        book.mark_dispatch_authorized("refund:uncertain", "owner:refund-operator", &request.authorization, NOW)
            .unwrap();
        book.mark_indeterminate("refund:uncertain", 1).unwrap();

        assert_eq!(book.encumbered_minor(), Ok(35));
        let contradictory_absence = outcome(
            &book,
            &request,
            CompensationOutcomeKindV2::AuthoritativeAbsence,
            "evidence:absence-contradictory",
        );
        assert_eq!(
            book.resolve_authoritative_absence(contradictory_absence),
            Err(CompensationErrorV2::OutcomeContradiction)
        );
        assert_eq!(book.encumbered_minor(), Ok(35));

        let observed = outcome(
            &book,
            &request,
            CompensationOutcomeKindV2::ObservedUnsettled,
            "evidence:refund-observed",
        );
        assert_eq!(
            book.observe_unsettled(observed),
            Ok(CompensationDispositionV2::Applied)
        );
        assert_eq!(book.encumbered_minor(), Ok(35));
        let settlement = outcome(
            &book,
            &request,
            CompensationOutcomeKindV2::FinalSettlement,
            "evidence:refund-settled",
        );
        assert_eq!(
            book.settle(settlement),
            Ok(CompensationDispositionV2::Settled)
        );
        assert_eq!(book.encumbered_minor(), Ok(0));
        assert_eq!(book.settled_minor(), Ok(35));
    }

    #[test]
    fn authoritative_absence_allows_only_same_effect_retry() {
        let mut book = book(100, 100);
        let request = request(&book, "refund:retry", 30, "liquidity:retry", 100);
        book.reserve(request.clone(), NOW).unwrap();
        book.mark_dispatch_authorized("refund:retry", "owner:refund-operator", &request.authorization, NOW)
            .unwrap();
        book.mark_indeterminate("refund:retry", 0).unwrap();
        let absence = outcome(
            &book,
            &request,
            CompensationOutcomeKindV2::AuthoritativeAbsence,
            "evidence:refund-absent",
        );
        book.resolve_authoritative_absence(absence).unwrap();
        assert_eq!(
            book.retry_same_effect("refund:retry", "owner:refund-operator", &request.authorization, NOW),
            Ok(CompensationDispositionV2::Applied)
        );
        assert_eq!(
            book.compensation_effect_id("refund:retry").as_deref(),
            Some("mx-compensation:refund:retry")
        );
        assert_eq!(book.status("refund:retry"), Some(CompensationStatusV2::Reserved));
        assert_eq!(book.encumbered_minor(), Ok(30));
    }

    #[test]
    fn exact_request_replay_is_idempotent_even_after_capability_expiry() {
        let mut book = book(100, 100);
        let request = request(&book, "refund:replay", 20, "liquidity:replay", 100);
        assert_eq!(
            book.reserve(request.clone(), NOW),
            Ok(CompensationDispositionV2::Created)
        );
        assert_eq!(
            book.reserve(request, 20_000),
            Ok(CompensationDispositionV2::AlreadyExists)
        );
        assert_eq!(book.encumbered_minor(), Ok(20));
    }

    #[test]
    fn one_reservation_id_cannot_back_two_compensation_effects() {
        let mut book = book(100, 100);
        let first = request(&book, "refund:first", 20, "liquidity:shared", 100);
        let second = request(&book, "refund:second", 20, "liquidity:shared", 100);
        book.reserve(first, NOW).unwrap();
        assert_eq!(
            book.reserve(second, NOW),
            Err(CompensationErrorV2::LiquidityReservationConflict)
        );
        assert_eq!(book.encumbered_minor(), Ok(20));
    }

    #[test]
    fn reusable_capability_has_one_aggregate_ceiling_across_partial_refunds() {
        let mut book = book(100, 100);
        let first = request(&book, "refund:cap-1", 40, "liquidity:cap-1", 70);
        let second = request(&book, "refund:cap-2", 31, "liquidity:cap-2", 70);
        book.reserve(first, NOW).unwrap();
        assert_eq!(
            book.reserve(second, NOW),
            Err(CompensationErrorV2::AuthorizationLimitExceeded)
        );
        assert_eq!(book.encumbered_minor(), Ok(40));
    }

    #[test]
    fn acting_owner_must_match_capability_grantee() {
        let mut book = book(100, 100);
        let mut request = request(&book, "refund:wrong-actor", 20, "liquidity:wrong-actor", 100);
        request.actor_owner_id = "owner:mallory".into();
        assert_eq!(
            book.reserve(request, NOW),
            Err(CompensationErrorV2::AuthorizationScopeMismatch)
        );
        assert_eq!(book.encumbered_minor(), Ok(0));
    }

    #[test]
    fn revoked_or_expired_capability_cannot_cross_dispatch_boundary() {
        let mut book = book(100, 100);
        let request = request(&book, "refund:revoked", 20, "liquidity:revoked", 100);
        book.reserve(request.clone(), NOW).unwrap();

        let mut revoked = request.authorization.clone();
        revoked.revoked = true;
        assert_eq!(
            book.mark_dispatch_authorized("refund:revoked", "owner:refund-operator", &revoked, NOW),
            Err(CompensationErrorV2::AuthorizationRevoked)
        );
        assert_eq!(book.status("refund:revoked"), Some(CompensationStatusV2::Reserved));

        assert_eq!(
            book.mark_dispatch_authorized(
                "refund:revoked",
                "owner:refund-operator",
                &request.authorization,
                request.authorization.expires_at_unix_ms,
            ),
            Err(CompensationErrorV2::AuthorizationExpired)
        );
        assert_eq!(book.encumbered_minor(), Ok(20));
    }

    #[test]
    fn dispatch_caller_must_be_the_grantee() {
        let mut book = book(100, 100);
        let request = request(&book, "refund:caller", 20, "liquidity:caller", 100);
        book.reserve(request.clone(), NOW).unwrap();
        assert_eq!(
            book.mark_dispatch_authorized(
                "refund:caller",
                "owner:mallory",
                &request.authorization,
                NOW,
            ),
            Err(CompensationErrorV2::CallerNotAuthorized)
        );
        assert_eq!(book.status("refund:caller"), Some(CompensationStatusV2::Reserved));
        assert_eq!(book.encumbered_minor(), Ok(20));
    }

    #[test]
    fn source_liquidity_cap_and_reservation_scope_are_enforced() {
        let mut book = book(50, 100);
        let too_large = request(&book, "refund:too-large", 51, "liquidity:too-large", 100);
        assert_eq!(
            book.reserve(too_large, NOW),
            Err(CompensationErrorV2::InsufficientLiquidity)
        );

        let mut wrong_source = request(&book, "refund:wrong-source", 20, "liquidity:wrong-source", 100);
        wrong_source.liquidity_reservation.source_account_id = "account:other".into();
        assert_eq!(
            book.reserve(wrong_source, NOW),
            Err(CompensationErrorV2::LiquidityReservationMismatch)
        );
        assert_eq!(book.encumbered_minor(), Ok(0));
    }

    #[test]
    fn compensation_outcome_must_bind_authorization_reservation_and_forward_subject() {
        let mut book = book(100, 100);
        let request = request(&book, "refund:bad-outcome", 20, "liquidity:bad-outcome", 100);
        book.reserve(request.clone(), NOW).unwrap();
        book.mark_dispatch_authorized(
            "refund:bad-outcome",
            "owner:refund-operator",
            &request.authorization,
            NOW,
        )
            .unwrap();

        let mut wrong_authority = outcome(
            &book,
            &request,
            CompensationOutcomeKindV2::FinalSettlement,
            "evidence:comp-settled",
        );
        wrong_authority.authorization_digest = digest('a');
        assert_eq!(
            book.settle(wrong_authority),
            Err(CompensationErrorV2::InvalidOutcome)
        );
        assert_eq!(book.status("refund:bad-outcome"), Some(CompensationStatusV2::DispatchAuthorized));

        let mut wrong_forward = outcome(
            &book,
            &request,
            CompensationOutcomeKindV2::FinalSettlement,
            "evidence:comp-settled",
        );
        wrong_forward.forward_effect_id = "effect:other".into();
        assert_eq!(
            book.settle(wrong_forward),
            Err(CompensationErrorV2::InvalidOutcome)
        );
        assert_eq!(book.encumbered_minor(), Ok(20));
    }

    #[test]
    fn cancellation_releases_only_never_dispatched_reservation() {
        let mut book = book(100, 100);
        let request = request(&book, "refund:cancel", 25, "liquidity:cancel", 100);
        book.reserve(request.clone(), NOW).unwrap();
        assert_eq!(book.encumbered_minor(), Ok(25));
        assert_eq!(
            book.cancel_before_dispatch("refund:cancel", "owner:refund-operator"),
            Ok(CompensationDispositionV2::Cancelled)
        );
        assert_eq!(book.encumbered_minor(), Ok(0));

        let dispatched = request(&book, "refund:no-cancel-after-dispatch", 25, "liquidity:dispatched", 100);
        book.reserve(dispatched.clone(), NOW).unwrap();
        book.mark_dispatch_authorized(
            "refund:no-cancel-after-dispatch",
            "owner:refund-operator",
            &dispatched.authorization,
            NOW,
        )
        .unwrap();
        assert_eq!(
            book.cancel_before_dispatch("refund:no-cancel-after-dispatch", "owner:refund-operator"),
            Err(CompensationErrorV2::InvalidTransition)
        );
        assert_eq!(book.encumbered_minor(), Ok(25));
    }
}
