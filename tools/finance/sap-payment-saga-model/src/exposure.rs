//! Owner-scoped, effects-disabled exposure reservations.
//!
//! This reference kernel bounds aggregate maximum plausible exposure. It does
//! not establish that a supplied policy or resolution receipt is authentic.

use sha2::{Digest, Sha256};
use std::collections::BTreeMap;

const SNAPSHOT_DOMAIN: &[u8] = b"MXSAP-EXPOSURE-SNAPSHOT-V1\0";
const MAX_FIELD_BYTES: usize = 4096;

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ExposureScopeV1 {
    pub owner_id: String,
    pub pool_id: String,
    pub destination_id: String,
    pub currency: String,
}

impl ExposureScopeV1 {
    fn validate(&self) -> Result<(), ExposureError> {
        for value in [
            &self.owner_id,
            &self.pool_id,
            &self.destination_id,
            &self.currency,
        ] {
            validate_text(value)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExposurePolicyV1 {
    pub scope: ExposureScopeV1,
    pub sequence: u64,
    pub maximum_total_minor: u64,
    pub maximum_per_effect_minor: u64,
    pub policy_digest: String,
}

impl ExposurePolicyV1 {
    pub fn validate(&self) -> Result<(), ExposureError> {
        self.scope.validate()?;
        if self.sequence == 0
            || self.maximum_total_minor == 0
            || self.maximum_per_effect_minor == 0
            || self.maximum_per_effect_minor > self.maximum_total_minor
        {
            return Err(ExposureError::InvalidPolicy);
        }
        validate_digest(&self.policy_digest)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExposureIntentV1 {
    pub scope: ExposureScopeV1,
    pub operation_id: String,
    pub effect_id: String,
    pub intent_digest: String,
    pub payload_digest: String,
    /// Conservative maximum plausible exposure, in the scope currency's
    /// integer minor unit. This is not necessarily the nominal payment amount.
    pub maximum_exposure_minor: u64,
}

impl ExposureIntentV1 {
    fn validate(&self) -> Result<(), ExposureError> {
        self.scope.validate()?;
        for value in [&self.operation_id, &self.effect_id] {
            validate_text(value)?;
        }
        validate_digest(&self.intent_digest)?;
        validate_digest(&self.payload_digest)?;
        if self.maximum_exposure_minor == 0 {
            return Err(ExposureError::InvalidAmount);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ExposureStatus {
    Reserved = 1,
    DispatchAuthorized = 2,
    Indeterminate = 3,
    CommittedUnsettled = 4,
    Settled = 5,
    AuthoritativelyAbsent = 6,
    CancelledBeforeDispatch = 7,
}

impl ExposureStatus {
    fn contributes_to_limit(self) -> bool {
        matches!(
            self,
            Self::Reserved
                | Self::DispatchAuthorized
                | Self::Indeterminate
                | Self::CommittedUnsettled
        )
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExposureReservationV1 {
    pub intent: ExposureIntentV1,
    pub status: ExposureStatus,
}

impl ExposureReservationV1 {
    fn same_identity(&self, intent: &ExposureIntentV1) -> bool {
        self.intent == *intent
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExposureDisposition {
    Reserved,
    AlreadyReserved,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExposureError {
    InvalidText,
    InvalidDigest,
    InvalidPolicy,
    InvalidAmount,
    ScopeMismatch,
    ReservationConflict,
    ReservationNotFound,
    LimitExceeded,
    AmountOverflow,
    InvalidTransition,
    InvalidResolutionEvidence,
}

impl std::fmt::Display for ExposureError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for ExposureError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ExposureResolutionKind {
    FinalSettlementEvidence = 1,
    AuthoritativeAbsence = 2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DestinationResolutionV1 {
    pub kind: ExposureResolutionKind,
    pub effect_id: String,
    pub destination_id: String,
    pub payload_digest: String,
    pub profile_digest: String,
    pub evidence_digest: String,
}

impl DestinationResolutionV1 {
    fn validate_against(
        &self,
        reservation: &ExposureReservationV1,
        expected_kind: ExposureResolutionKind,
    ) -> Result<(), ExposureError> {
        validate_digest(&self.payload_digest)?;
        validate_digest(&self.profile_digest)?;
        validate_digest(&self.evidence_digest)?;
        if self.kind != expected_kind
            || self.effect_id != reservation.intent.effect_id
            || self.destination_id != reservation.intent.scope.destination_id
            || self.payload_digest != reservation.intent.payload_digest
        {
            return Err(ExposureError::InvalidResolutionEvidence);
        }
        validate_text(&self.profile_digest)?;
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ExposureBookV1 {
    policy: ExposurePolicyV1,
    reservations: BTreeMap<String, ExposureReservationV1>,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ExposureBreakdownV1 {
    pub reserved_minor: u64,
    pub dispatch_authorized_minor: u64,
    pub indeterminate_minor: u64,
    pub committed_unsettled_minor: u64,
}

impl ExposureBreakdownV1 {
    pub fn total_minor(self) -> Result<u64, ExposureError> {
        self.reserved_minor
            .checked_add(self.dispatch_authorized_minor)
            .and_then(|value| value.checked_add(self.indeterminate_minor))
            .and_then(|value| value.checked_add(self.committed_unsettled_minor))
            .ok_or(ExposureError::AmountOverflow)
    }
}

impl ExposureBookV1 {
    pub fn new(policy: ExposurePolicyV1) -> Result<Self, ExposureError> {
        policy.validate()?;
        Ok(Self {
            policy,
            reservations: BTreeMap::new(),
        })
    }

    pub fn policy(&self) -> &ExposurePolicyV1 {
        &self.policy
    }

    /// Lowering a limit never removes existing reservations. The book may
    /// remain over limit until obligations are resolved; new reservations
    /// fail closed during that period.
    pub fn replace_policy(&mut self, policy: ExposurePolicyV1) -> Result<(), ExposureError> {
        policy.validate()?;
        if policy.scope != self.policy().scope || policy.sequence <= self.policy().sequence {
            return Err(ExposureError::InvalidPolicy);
        }
        self.policy = policy;
        Ok(())
    }

    pub fn reserve(
        &mut self,
        intent: ExposureIntentV1,
    ) -> Result<ExposureDisposition, ExposureError> {
        intent.validate()?;
        let policy = self.policy();
        if intent.scope != policy.scope {
            return Err(ExposureError::ScopeMismatch);
        }
        if intent.maximum_exposure_minor > policy.maximum_per_effect_minor {
            return Err(ExposureError::LimitExceeded);
        }

        if let Some(existing) = self.reservations.get(&intent.effect_id) {
            return if existing.same_identity(&intent) {
                Ok(ExposureDisposition::AlreadyReserved)
            } else {
                Err(ExposureError::ReservationConflict)
            };
        }

        let current = self.total_exposure_minor()?;
        let projected = current
            .checked_add(intent.maximum_exposure_minor)
            .ok_or(ExposureError::AmountOverflow)?;
        if projected > policy.maximum_total_minor {
            return Err(ExposureError::LimitExceeded);
        }

        self.reservations.insert(
            intent.effect_id.clone(),
            ExposureReservationV1 {
                intent,
                status: ExposureStatus::Reserved,
            },
        );
        Ok(ExposureDisposition::Reserved)
    }

    pub fn mark_dispatch_authorized(&mut self, effect_id: &str) -> Result<(), ExposureError> {
        self.transition(
            effect_id,
            &[ExposureStatus::Reserved, ExposureStatus::DispatchAuthorized],
            ExposureStatus::DispatchAuthorized,
        )
    }

    pub fn mark_indeterminate(&mut self, effect_id: &str) -> Result<(), ExposureError> {
        self.transition(
            effect_id,
            &[
                ExposureStatus::DispatchAuthorized,
                ExposureStatus::Indeterminate,
            ],
            ExposureStatus::Indeterminate,
        )
    }

    pub fn mark_committed_unsettled(&mut self, effect_id: &str) -> Result<(), ExposureError> {
        self.transition(
            effect_id,
            &[
                ExposureStatus::DispatchAuthorized,
                ExposureStatus::Indeterminate,
                ExposureStatus::CommittedUnsettled,
            ],
            ExposureStatus::CommittedUnsettled,
        )
    }

    pub fn mark_settled(
        &mut self,
        effect_id: &str,
        evidence: &DestinationResolutionV1,
    ) -> Result<(), ExposureError> {
        let reservation = self
            .reservations
            .get(effect_id)
            .ok_or(ExposureError::ReservationNotFound)?;
        evidence.validate_against(reservation, ExposureResolutionKind::FinalSettlementEvidence)?;
        if reservation.status == ExposureStatus::Settled {
            return Ok(());
        }
        if reservation.status != ExposureStatus::CommittedUnsettled {
            return Err(ExposureError::InvalidTransition);
        }
        self.reservations
            .get_mut(effect_id)
            .expect("checked above")
            .status = ExposureStatus::Settled;
        Ok(())
    }

    pub fn release_authoritative_absence(
        &mut self,
        effect_id: &str,
        evidence: &DestinationResolutionV1,
    ) -> Result<(), ExposureError> {
        let reservation = self
            .reservations
            .get(effect_id)
            .ok_or(ExposureError::ReservationNotFound)?;
        evidence.validate_against(reservation, ExposureResolutionKind::AuthoritativeAbsence)?;
        if reservation.status == ExposureStatus::AuthoritativelyAbsent {
            return Ok(());
        }
        if !matches!(
            reservation.status,
            ExposureStatus::DispatchAuthorized | ExposureStatus::Indeterminate
        ) {
            return Err(ExposureError::InvalidTransition);
        }
        self.reservations
            .get_mut(effect_id)
            .expect("checked above")
            .status = ExposureStatus::AuthoritativelyAbsent;
        Ok(())
    }

    pub fn cancel_before_dispatch(&mut self, effect_id: &str) -> Result<(), ExposureError> {
        self.transition(
            effect_id,
            &[ExposureStatus::Reserved],
            ExposureStatus::CancelledBeforeDispatch,
        )
    }

    pub fn status(&self, effect_id: &str) -> Option<ExposureStatus> {
        self.reservations.get(effect_id).map(|r| r.status)
    }

    pub fn reservation_count(&self) -> usize {
        self.reservations.len()
    }

    pub fn breakdown_minor(&self) -> Result<ExposureBreakdownV1, ExposureError> {
        let mut totals = ExposureBreakdownV1::default();
        for reservation in self.reservations.values() {
            let amount = reservation.intent.maximum_exposure_minor;
            let bucket = match reservation.status {
                ExposureStatus::Reserved => &mut totals.reserved_minor,
                ExposureStatus::DispatchAuthorized => &mut totals.dispatch_authorized_minor,
                ExposureStatus::Indeterminate => &mut totals.indeterminate_minor,
                ExposureStatus::CommittedUnsettled => &mut totals.committed_unsettled_minor,
                ExposureStatus::Settled
                | ExposureStatus::AuthoritativelyAbsent
                | ExposureStatus::CancelledBeforeDispatch => continue,
            };
            *bucket = bucket.checked_add(amount).ok_or(ExposureError::AmountOverflow)?;
        }
        Ok(totals)
    }

    pub fn total_exposure_minor(&self) -> Result<u64, ExposureError> {
        self.breakdown_minor()?.total_minor()
    }

    pub fn headroom_minor(&self) -> Result<u64, ExposureError> {
        Ok(self
            .policy()
            .maximum_total_minor
            .saturating_sub(self.total_exposure_minor()?))
    }

    pub fn canonical_snapshot_bytes(&self) -> Result<Vec<u8>, ExposureError> {
        let policy = self.policy();
        let count =
            u32::try_from(self.reservations.len()).map_err(|_| ExposureError::InvalidAmount)?;
        let mut output = Vec::with_capacity(256 + self.reservations.len() * 256);
        output.extend_from_slice(SNAPSHOT_DOMAIN);
        for value in [
            &policy.scope.owner_id,
            &policy.scope.pool_id,
            &policy.scope.destination_id,
            &policy.scope.currency,
        ] {
            append_text(&mut output, value)?;
        }
        append_u64(&mut output, policy.sequence);
        append_u64(&mut output, policy.maximum_total_minor);
        append_u64(&mut output, policy.maximum_per_effect_minor);
        append_text(&mut output, &policy.policy_digest)?;
        output.extend_from_slice(&count.to_be_bytes());

        // BTreeMap ordering makes the snapshot independent of insertion order.
        for reservation in self.reservations.values() {
            let intent = &reservation.intent;
            for value in [
                &intent.operation_id,
                &intent.effect_id,
                &intent.intent_digest,
                &intent.payload_digest,
            ] {
                append_text(&mut output, value)?;
            }
            append_u64(&mut output, intent.maximum_exposure_minor);
            output.push(reservation.status as u8);
        }
        Ok(output)
    }

    pub fn canonical_snapshot_digest(&self) -> Result<String, ExposureError> {
        let digest = Sha256::digest(self.canonical_snapshot_bytes()?);
        let mut output = String::with_capacity(71);
        output.push_str("sha256:");
        for byte in digest {
            use std::fmt::Write as _;
            write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
        }
        Ok(output)
    }

    fn transition(
        &mut self,
        effect_id: &str,
        allowed: &[ExposureStatus],
        next: ExposureStatus,
    ) -> Result<(), ExposureError> {
        let reservation = self
            .reservations
            .get_mut(effect_id)
            .ok_or(ExposureError::ReservationNotFound)?;
        if !allowed.contains(&reservation.status) {
            return Err(ExposureError::InvalidTransition);
        }
        reservation.status = next;
        Ok(())
    }
}

fn validate_text(value: &str) -> Result<(), ExposureError> {
    if value.is_empty() || value.len() > MAX_FIELD_BYTES {
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

    fn scope() -> ExposureScopeV1 {
        ExposureScopeV1 {
            owner_id: "owner:coop-a".into(),
            pool_id: "pool:treasury".into(),
            destination_id: "dest:provider-a".into(),
            currency: "SAP".into(),
        }
    }

    fn policy(maximum_total_minor: u64) -> ExposurePolicyV1 {
        ExposurePolicyV1 {
            scope: scope(),
            sequence: 1,
            maximum_total_minor,
            maximum_per_effect_minor: maximum_total_minor.min(800),
            policy_digest: digest('d'),
        }
    }

    fn intent(effect_id: &str, maximum: u64) -> ExposureIntentV1 {
        ExposureIntentV1 {
            scope: scope(),
            operation_id: "op:1".into(),
            effect_id: effect_id.into(),
            intent_digest: digest('a'),
            payload_digest: digest('b'),
            maximum_exposure_minor: maximum,
        }
    }

    fn resolution(effect_id: &str) -> DestinationResolutionV1 {
        DestinationResolutionV1 {
            kind: ExposureResolutionKind::FinalSettlementEvidence,
            effect_id: effect_id.into(),
            destination_id: "dest:provider-a".into(),
            payload_digest: digest('b'),
            profile_digest: digest('e'),
            evidence_digest: digest('f'),
        }
    }

    #[test]
    fn exposure_breakdown_tracks_each_at_risk_state() {
        let mut book = ExposureBookV1::new(policy(2_000)).unwrap();
        book.reserve(intent("reserved", 100)).unwrap();
        book.reserve(intent("dispatch", 200)).unwrap();
        book.reserve(intent("indeterminate", 300)).unwrap();
        book.reserve(intent("committed", 400)).unwrap();
        book.mark_dispatch_authorized("dispatch").unwrap();
        book.mark_dispatch_authorized("indeterminate").unwrap();
        book.mark_indeterminate("indeterminate").unwrap();
        book.mark_dispatch_authorized("committed").unwrap();
        book.mark_committed_unsettled("committed").unwrap();
        assert_eq!(
            book.breakdown_minor().unwrap(),
            ExposureBreakdownV1 {
                reserved_minor: 100,
                dispatch_authorized_minor: 200,
                indeterminate_minor: 300,
                committed_unsettled_minor: 400,
            }
        );
        assert_eq!(book.total_exposure_minor(), Ok(1_000));
    }

    #[test]
    fn dispatch_authorization_is_idempotent_and_keeps_exposure_reserved() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        book.reserve(intent("effect:1", 600)).unwrap();
        book.mark_dispatch_authorized("effect:1").unwrap();
        book.mark_dispatch_authorized("effect:1").unwrap();
        assert_eq!(
            book.status("effect:1"),
            Some(ExposureStatus::DispatchAuthorized)
        );
        assert_eq!(book.total_exposure_minor(), Ok(600));
    }

    #[test]
    fn reservation_scope_must_match_owner_pool_destination_and_currency() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        let mut wrong = intent("effect:1", 100);
        wrong.scope.owner_id = "owner:other".into();
        assert_eq!(book.reserve(wrong), Err(ExposureError::ScopeMismatch));
        let mut wrong_currency = intent("effect:2", 100);
        wrong_currency.scope.currency = "USD".into();
        assert_eq!(
            book.reserve(wrong_currency),
            Err(ExposureError::ScopeMismatch)
        );
    }

    #[test]
    fn reservations_are_owner_scoped_and_idempotent() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        assert_eq!(
            book.reserve(intent("effect:1", 600)),
            Ok(ExposureDisposition::Reserved)
        );
        assert_eq!(
            book.reserve(intent("effect:1", 600)),
            Ok(ExposureDisposition::AlreadyReserved)
        );
        assert_eq!(book.total_exposure_minor(), Ok(600));
        assert_eq!(book.headroom_minor(), Ok(400));
        assert_eq!(
            book.reserve(intent("effect:2", 500)),
            Err(ExposureError::LimitExceeded)
        );
    }

    #[test]
    fn changed_intent_under_same_effect_identity_conflicts() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        book.reserve(intent("effect:1", 600)).unwrap();
        let mut changed = intent("effect:1", 601);
        changed.payload_digest = digest('c');
        assert_eq!(
            book.reserve(changed),
            Err(ExposureError::ReservationConflict)
        );
        assert_eq!(book.total_exposure_minor(), Ok(600));
    }

    #[test]
    fn indeterminate_effect_remains_in_aggregate_exposure() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        book.reserve(intent("effect:1", 600)).unwrap();
        book.mark_dispatch_authorized("effect:1").unwrap();
        book.mark_indeterminate("effect:1").unwrap();
        assert_eq!(book.status("effect:1"), Some(ExposureStatus::Indeterminate));
        assert_eq!(book.total_exposure_minor(), Ok(600));
        assert_eq!(
            book.reserve(intent("effect:2", 500)),
            Err(ExposureError::LimitExceeded)
        );
    }

    #[test]
    fn settlement_releases_only_after_matching_receipt_subject() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        book.reserve(intent("effect:1", 600)).unwrap();
        book.mark_dispatch_authorized("effect:1").unwrap();
        book.mark_committed_unsettled("effect:1").unwrap();
        let mut wrong = resolution("effect:1");
        wrong.payload_digest = digest('c');
        assert_eq!(
            book.mark_settled("effect:1", &wrong),
            Err(ExposureError::InvalidResolutionEvidence)
        );
        book.mark_settled("effect:1", &resolution("effect:1"))
            .unwrap();
        assert_eq!(book.total_exposure_minor(), Ok(0));
    }

    #[test]
    fn authoritative_absence_is_destination_and_payload_bound() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        book.reserve(intent("effect:1", 600)).unwrap();
        book.mark_dispatch_authorized("effect:1").unwrap();
        book.mark_indeterminate("effect:1").unwrap();
        let mut wrong = resolution("effect:1");
        wrong.destination_id = "dest:other".into();
        assert_eq!(
            book.release_authoritative_absence("effect:1", &wrong),
            Err(ExposureError::InvalidResolutionEvidence)
        );
        let mut absence = resolution("effect:1");
        absence.kind = ExposureResolutionKind::AuthoritativeAbsence;
        book.release_authoritative_absence("effect:1", &absence)
            .unwrap();
        assert_eq!(book.total_exposure_minor(), Ok(0));
    }

    #[test]
    fn lower_policy_does_not_erase_existing_exposure() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        book.reserve(intent("effect:1", 600)).unwrap();
        let mut lower = policy(500);
        lower.sequence = 2;
        book.replace_policy(lower).unwrap();
        assert_eq!(book.total_exposure_minor(), Ok(600));
        assert_eq!(book.headroom_minor(), Ok(0));
        assert_eq!(
            book.reserve(intent("effect:2", 100)),
            Err(ExposureError::LimitExceeded)
        );
    }

    #[test]
    fn cancellation_is_permitted_only_before_dispatch_authorization() {
        let mut book = ExposureBookV1::new(policy(1_000)).unwrap();
        book.reserve(intent("effect:1", 600)).unwrap();
        book.cancel_before_dispatch("effect:1").unwrap();
        assert_eq!(book.total_exposure_minor(), Ok(0));
        assert_eq!(
            book.mark_dispatch_authorized("effect:1"),
            Err(ExposureError::InvalidTransition)
        );
    }

    #[test]
    fn snapshot_is_canonical_and_insertion_order_independent() {
        let kat = include_str!("../fixtures/sap-payment-exposure-v1.kat");
        let expected = kat
            .lines()
            .find_map(|line| line.strip_prefix("snapshot_sha256="))
            .expect("shared exposure snapshot KAT exists");
        let mut first = ExposureBookV1::new(policy(1_000)).unwrap();
        first.reserve(intent("effect:1", 600)).unwrap();
        let actual = first.canonical_snapshot_digest().unwrap();
        assert_eq!(actual.strip_prefix("sha256:"), Some(expected));

        let mut second = ExposureBookV1::new(policy(1_000)).unwrap();
        second.reserve(intent("effect:2", 100)).unwrap();
        second.reserve(intent("effect:1", 600)).unwrap();
        let mut third = ExposureBookV1::new(policy(1_000)).unwrap();
        third.reserve(intent("effect:1", 600)).unwrap();
        third.reserve(intent("effect:2", 100)).unwrap();
        assert_eq!(
            second.canonical_snapshot_digest().unwrap(),
            third.canonical_snapshot_digest().unwrap()
        );
    }
}
