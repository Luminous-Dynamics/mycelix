//! Pure policy kernel for the proposed SAP-ZAR reserve perimeter.
//!
//! This crate deliberately does not call Holochain, banks, custodians, or oracles.
//! It makes arithmetic and state-transition rules executable before zome integration.
//! Callers must still authenticate evidence, validate legal authority, and bind all
//! evidence references to exact requests. This crate is not a reserve attestation.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

pub const BASIS_POINTS: u128 = 10_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReserveAssetClass {
    CashAtQualifiedCustodian,
    CentralBankMoney,
    ShortTermSovereign { remaining_days: u32 },
    CommercialBankDeposit,
    ProductiveAsset,
    LoanReceivable,
    SelfIssuedClaim,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReserveAsset {
    pub asset_id: String,
    pub class: ReserveAssetClass,
    pub gross_value_minor: u128,
    pub haircut_bps: u16,
    pub encumbered_minor: u128,
    /// True only when supported by a current, independently authenticated position statement.
    pub independently_confirmed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReservePolicy {
    pub max_sovereign_remaining_days: u32,
}

impl Default for ReservePolicy {
    fn default() -> Self {
        Self {
            max_sovereign_remaining_days: 90,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssuranceError {
    InvalidHaircut(u16),
    DuplicateAssetId(String),
    ArithmeticOverflow,
    InvalidAmount,
    IssuanceWouldBreachCoverage,
    BurnExceedsIssued,
    IdempotencyKeyConflict,
    InvalidRedemptionState {
        expected: RedemptionState,
        actual: RedemptionState,
    },
    MissingEvidenceReference,
}

impl ReserveAsset {
    /// Returns an asset's eligible reserve contribution after haircut and encumbrance.
    ///
    /// Unconfirmed, long-dated, productive, receivable, commercial-bank-deposit and
    /// self-issued claims contribute zero under the default SAP-ZAR reserve perimeter.
    /// A price quote alone never makes an asset eligible.
    pub fn eligible_value_minor(&self, policy: ReservePolicy) -> Result<u128, AssuranceError> {
        if self.haircut_bps as u128 > BASIS_POINTS {
            return Err(AssuranceError::InvalidHaircut(self.haircut_bps));
        }
        if !self.independently_confirmed || !is_reserve_eligible(self.class, policy) {
            return Ok(0);
        }
        let after_haircut = mul_div_floor(
            self.gross_value_minor,
            BASIS_POINTS - self.haircut_bps as u128,
            BASIS_POINTS,
        )?;
        Ok(after_haircut.saturating_sub(self.encumbered_minor))
    }
}

pub fn is_reserve_eligible(class: ReserveAssetClass, policy: ReservePolicy) -> bool {
    match class {
        ReserveAssetClass::CashAtQualifiedCustodian | ReserveAssetClass::CentralBankMoney => true,
        ReserveAssetClass::ShortTermSovereign { remaining_days } => {
            remaining_days <= policy.max_sovereign_remaining_days
        }
        ReserveAssetClass::CommercialBankDeposit
        | ReserveAssetClass::ProductiveAsset
        | ReserveAssetClass::LoanReceivable
        | ReserveAssetClass::SelfIssuedClaim => false,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReserveCoverage {
    pub eligible_assets_minor: u128,
    pub redeemable_liabilities_minor: u128,
    /// None means there are no redeemable liabilities; it is not reported as infinity.
    pub coverage_bps: Option<u128>,
    pub fully_covered: bool,
}

pub fn reserve_coverage(
    assets: &[ReserveAsset],
    redeemable_liabilities_minor: u128,
    policy: ReservePolicy,
) -> Result<ReserveCoverage, AssuranceError> {
    let mut seen = BTreeSet::new();
    let mut eligible_assets_minor = 0u128;

    for asset in assets {
        if asset.asset_id.is_empty() {
            return Err(AssuranceError::MissingEvidenceReference);
        }
        if !seen.insert(asset.asset_id.as_str()) {
            return Err(AssuranceError::DuplicateAssetId(asset.asset_id.clone()));
        }
        eligible_assets_minor = eligible_assets_minor
            .checked_add(asset.eligible_value_minor(policy)?)
            .ok_or(AssuranceError::ArithmeticOverflow)?;
    }

    let coverage_bps = if redeemable_liabilities_minor == 0 {
        None
    } else {
        Some(mul_div_floor(
            eligible_assets_minor,
            BASIS_POINTS,
            redeemable_liabilities_minor,
        )?)
    };

    Ok(ReserveCoverage {
        eligible_assets_minor,
        redeemable_liabilities_minor,
        coverage_bps,
        fully_covered: eligible_assets_minor >= redeemable_liabilities_minor,
    })
}

/// Checks an issuance against the post-issuance reserve/liability position.
/// The caller must separately prove that every new reserve contribution has settled.
pub fn check_issuance_coverage(
    assets_after_settlement: &[ReserveAsset],
    current_redeemable_liabilities_minor: u128,
    proposed_issuance_minor: u128,
    policy: ReservePolicy,
) -> Result<ReserveCoverage, AssuranceError> {
    if proposed_issuance_minor == 0 {
        return Err(AssuranceError::InvalidAmount);
    }
    let liabilities_after = current_redeemable_liabilities_minor
        .checked_add(proposed_issuance_minor)
        .ok_or(AssuranceError::ArithmeticOverflow)?;
    let coverage = reserve_coverage(assets_after_settlement, liabilities_after, policy)?;
    if !coverage.fully_covered {
        return Err(AssuranceError::IssuanceWouldBreachCoverage);
    }
    Ok(coverage)
}

fn mul_div_floor(value: u128, multiplier: u128, divisor: u128) -> Result<u128, AssuranceError> {
    if divisor == 0 {
        return Err(AssuranceError::ArithmeticOverflow);
    }
    let whole = (value / divisor)
        .checked_mul(multiplier)
        .ok_or(AssuranceError::ArithmeticOverflow)?;
    let remainder = (value % divisor)
        .checked_mul(multiplier)
        .ok_or(AssuranceError::ArithmeticOverflow)?;
    whole
        .checked_add(remainder / divisor)
        .ok_or(AssuranceError::ArithmeticOverflow)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SapSupply {
    pub total_issued_minor: u128,
    pub total_burned_minor: u128,
}

impl SapSupply {
    pub fn outstanding_minor(self) -> Result<u128, AssuranceError> {
        self.total_issued_minor
            .checked_sub(self.total_burned_minor)
            .ok_or(AssuranceError::BurnExceedsIssued)
    }

    pub fn issue(&mut self, amount_minor: u128) -> Result<u128, AssuranceError> {
        if amount_minor == 0 {
            return Err(AssuranceError::InvalidAmount);
        }
        self.total_issued_minor = self
            .total_issued_minor
            .checked_add(amount_minor)
            .ok_or(AssuranceError::ArithmeticOverflow)?;
        self.outstanding_minor()
    }

    pub fn burn(&mut self, amount_minor: u128) -> Result<u128, AssuranceError> {
        if amount_minor == 0 {
            return Err(AssuranceError::InvalidAmount);
        }
        let new_burned = self
            .total_burned_minor
            .checked_add(amount_minor)
            .ok_or(AssuranceError::ArithmeticOverflow)?;
        if new_burned > self.total_issued_minor {
            return Err(AssuranceError::BurnExceedsIssued);
        }
        self.total_burned_minor = new_burned;
        self.outstanding_minor()
    }
}

/// Outcome of checking an idempotency key against an exact request digest.
/// An exact retry must return the previously stored receipt/result, not execute again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdempotencyDisposition {
    NewRequest,
    ExactReplay,
}

/// Records a one-time request digest under an idempotency key digest.
///
/// The digests must be computed by a trusted canonicalization/signature layer upstream.
/// Production adapters must persist this mapping in the same authoritative transaction
/// boundary as the side effect and its receipt.
#[derive(Debug, Clone, Default)]
pub struct IdempotencyLedger {
    consumed: BTreeMap<[u8; 32], [u8; 32]>,
}

impl IdempotencyLedger {
    pub fn check_and_consume(
        &mut self,
        idempotency_key_digest: [u8; 32],
        request_payload_digest: [u8; 32],
    ) -> Result<IdempotencyDisposition, AssuranceError> {
        match self.consumed.get(&idempotency_key_digest) {
            Some(existing) if existing == &request_payload_digest => {
                Ok(IdempotencyDisposition::ExactReplay)
            }
            Some(_) => Err(AssuranceError::IdempotencyKeyConflict),
            None => {
                self.consumed
                    .insert(idempotency_key_digest, request_payload_digest);
                Ok(IdempotencyDisposition::NewRequest)
            }
        }
    }

    pub fn contains(&self, idempotency_key_digest: &[u8; 32]) -> bool {
        self.consumed.contains_key(idempotency_key_digest)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RedemptionState {
    Requested,
    Reserved,
    PayoutSubmitted,
    PayoutUnknown,
    PaidFinal,
    Burned,
    Rejected,
    Compensated,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Redemption {
    state: RedemptionState,
    payout_reference: Option<String>,
    reconciliation_reference: Option<String>,
    liability_burn_reference: Option<String>,
    compensation_reference: Option<String>,
}

impl Redemption {
    pub fn new() -> Self {
        Self {
            state: RedemptionState::Requested,
            payout_reference: None,
            reconciliation_reference: None,
            liability_burn_reference: None,
            compensation_reference: None,
        }
    }

    pub fn state(&self) -> RedemptionState {
        self.state
    }

    pub fn reserve(&mut self) -> Result<(), AssuranceError> {
        self.require_state(RedemptionState::Requested)?;
        self.state = RedemptionState::Reserved;
        Ok(())
    }

    pub fn submit_payout(&mut self) -> Result<(), AssuranceError> {
        self.require_state(RedemptionState::Reserved)?;
        self.state = RedemptionState::PayoutSubmitted;
        Ok(())
    }

    pub fn mark_payout_unknown(&mut self) -> Result<(), AssuranceError> {
        self.require_state(RedemptionState::PayoutSubmitted)?;
        self.state = RedemptionState::PayoutUnknown;
        Ok(())
    }

    /// Records a verified external finality reference. The caller must authenticate
    /// the reference against the named payment rail's finality policy.
    pub fn reconcile_paid(
        &mut self,
        external_payout_reference: String,
        reconciliation_reference: String,
    ) -> Result<(), AssuranceError> {
        if !matches!(
            self.state,
            RedemptionState::PayoutSubmitted | RedemptionState::PayoutUnknown
        ) {
            return Err(AssuranceError::InvalidRedemptionState {
                expected: RedemptionState::PayoutUnknown,
                actual: self.state,
            });
        }
        if external_payout_reference.is_empty() || reconciliation_reference.is_empty() {
            return Err(AssuranceError::MissingEvidenceReference);
        }
        self.payout_reference = Some(external_payout_reference);
        self.reconciliation_reference = Some(reconciliation_reference);
        self.state = RedemptionState::PaidFinal;
        Ok(())
    }

    /// Records independently reconciled proof that no payout occurred.
    pub fn reconcile_not_paid(
        &mut self,
        reconciliation_reference: String,
    ) -> Result<(), AssuranceError> {
        if !matches!(
            self.state,
            RedemptionState::PayoutSubmitted | RedemptionState::PayoutUnknown
        ) {
            return Err(AssuranceError::InvalidRedemptionState {
                expected: RedemptionState::PayoutUnknown,
                actual: self.state,
            });
        }
        if reconciliation_reference.is_empty() {
            return Err(AssuranceError::MissingEvidenceReference);
        }
        self.reconciliation_reference = Some(reconciliation_reference);
        self.state = RedemptionState::Rejected;
        Ok(())
    }

    /// Burns the liability only after externally reconciled payout finality.
    pub fn burn_liability(
        &mut self,
        liability_burn_reference: String,
    ) -> Result<(), AssuranceError> {
        self.require_state(RedemptionState::PaidFinal)?;
        if self.payout_reference.is_none() || liability_burn_reference.is_empty() {
            return Err(AssuranceError::MissingEvidenceReference);
        }
        self.liability_burn_reference = Some(liability_burn_reference);
        self.state = RedemptionState::Burned;
        Ok(())
    }

    /// Releases a reserved liability after a reconciled non-payment outcome.
    pub fn compensate(&mut self, compensation_reference: String) -> Result<(), AssuranceError> {
        self.require_state(RedemptionState::Rejected)?;
        if self.reconciliation_reference.is_none() || compensation_reference.is_empty() {
            return Err(AssuranceError::MissingEvidenceReference);
        }
        self.compensation_reference = Some(compensation_reference);
        self.state = RedemptionState::Compensated;
        Ok(())
    }

    fn require_state(&self, expected: RedemptionState) -> Result<(), AssuranceError> {
        if self.state != expected {
            return Err(AssuranceError::InvalidRedemptionState {
                expected,
                actual: self.state,
            });
        }
        Ok(())
    }
}

impl Default for Redemption {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cash(id: &str, gross: u128) -> ReserveAsset {
        ReserveAsset {
            asset_id: id.to_string(),
            class: ReserveAssetClass::CashAtQualifiedCustodian,
            gross_value_minor: gross,
            haircut_bps: 0,
            encumbered_minor: 0,
            independently_confirmed: true,
        }
    }

    #[test]
    fn productive_assets_and_loan_receivables_never_count_as_liquid_reserves() {
        let mut land = cash("asset-land", 10_000_000);
        land.class = ReserveAssetClass::ProductiveAsset;
        let mut loan = cash("asset-loan", 10_000_000);
        loan.class = ReserveAssetClass::LoanReceivable;
        let coverage = reserve_coverage(&[land, loan], 1, ReservePolicy::default()).unwrap();
        assert_eq!(coverage.eligible_assets_minor, 0);
        assert!(!coverage.fully_covered);
    }

    #[test]
    fn reserve_value_applies_haircut_then_encumbrance() {
        let mut asset = cash("treasury-bill", 1_000_000);
        asset.class = ReserveAssetClass::ShortTermSovereign { remaining_days: 30 };
        asset.haircut_bps = 500;
        asset.encumbered_minor = 100_000;
        assert_eq!(
            asset
                .eligible_value_minor(ReservePolicy::default())
                .unwrap(),
            850_000
        );
    }

    #[test]
    fn long_dated_sovereign_is_not_eligible_under_default_policy() {
        let mut asset = cash("long-bond", 1_000_000);
        asset.class = ReserveAssetClass::ShortTermSovereign { remaining_days: 91 };
        assert_eq!(
            asset
                .eligible_value_minor(ReservePolicy::default())
                .unwrap(),
            0
        );
    }

    #[test]
    fn unconfirmed_assets_contribute_zero_even_when_priced() {
        let mut asset = cash("unconfirmed-cash", 5_000_000);
        asset.independently_confirmed = false;
        assert_eq!(
            asset
                .eligible_value_minor(ReservePolicy::default())
                .unwrap(),
            0
        );
    }

    #[test]
    fn duplicate_asset_identity_cannot_be_double_counted() {
        let assets = [
            cash("same-custody-position", 100),
            cash("same-custody-position", 100),
        ];
        assert_eq!(
            reserve_coverage(&assets, 100, ReservePolicy::default()),
            Err(AssuranceError::DuplicateAssetId(
                "same-custody-position".to_string()
            ))
        );
    }

    #[test]
    fn issuance_must_preserve_one_hundred_percent_haircut_adjusted_coverage() {
        let assets = [cash("custody-cash", 1_000)];
        assert!(check_issuance_coverage(&assets, 900, 100, ReservePolicy::default()).is_ok());
        assert_eq!(
            check_issuance_coverage(&assets, 900, 101, ReservePolicy::default()),
            Err(AssuranceError::IssuanceWouldBreachCoverage)
        );
    }

    #[test]
    fn supply_is_issued_minus_burned_and_cannot_underflow() {
        let mut supply = SapSupply::default();
        assert_eq!(supply.issue(1_000).unwrap(), 1_000);
        assert_eq!(supply.burn(250).unwrap(), 750);
        assert_eq!(supply.burn(751), Err(AssuranceError::BurnExceedsIssued));
        assert_eq!(supply.outstanding_minor().unwrap(), 750);
    }

    #[test]
    fn idempotency_rejects_exact_replay_and_changed_payload() {
        let mut ledger = IdempotencyLedger::default();
        let key = [7_u8; 32];
        assert_eq!(
            ledger.check_and_consume(key, [1_u8; 32]),
            Ok(IdempotencyDisposition::NewRequest)
        );
        assert_eq!(
            ledger.check_and_consume(key, [1_u8; 32]),
            Ok(IdempotencyDisposition::ExactReplay)
        );
        assert_eq!(
            ledger.check_and_consume(key, [2_u8; 32]),
            Err(AssuranceError::IdempotencyKeyConflict)
        );
        assert!(ledger.contains(&key));
    }

    #[test]
    fn payout_unknown_requires_reconciliation_before_burn_or_resubmit() {
        let mut redemption = Redemption::new();
        redemption.reserve().unwrap();
        redemption.submit_payout().unwrap();
        redemption.mark_payout_unknown().unwrap();
        assert!(redemption.submit_payout().is_err());
        assert!(redemption.burn_liability("burn-1".into()).is_err());
        redemption
            .reconcile_paid("bank-payment-123".into(), "reconcile-123".into())
            .unwrap();
        redemption.burn_liability("burn-1".into()).unwrap();
        assert_eq!(redemption.state(), RedemptionState::Burned);
    }

    #[test]
    fn rejected_redemption_requires_reconciliation_and_compensation() {
        let mut redemption = Redemption::new();
        redemption.reserve().unwrap();
        redemption.submit_payout().unwrap();
        redemption.mark_payout_unknown().unwrap();
        redemption
            .reconcile_not_paid("rail-query-123".into())
            .unwrap();
        redemption
            .compensate("release-reservation-123".into())
            .unwrap();
        assert_eq!(redemption.state(), RedemptionState::Compensated);
    }

    #[test]
    fn evidence_references_are_required_for_paid_final_and_burn() {
        let mut redemption = Redemption::new();
        redemption.reserve().unwrap();
        redemption.submit_payout().unwrap();
        assert_eq!(
            redemption.reconcile_paid("".into(), "reconcile".into()),
            Err(AssuranceError::MissingEvidenceReference)
        );
        redemption
            .reconcile_paid("bank-payment".into(), "reconcile".into())
            .unwrap();
        assert_eq!(
            redemption.burn_liability("".into()),
            Err(AssuranceError::MissingEvidenceReference)
        );
    }

    #[test]
    fn one_million_deterministic_reserve_cases_preserve_bounds() {
        let policy = ReservePolicy::default();
        let mut seed = 0x9e37_79b9_7f4a_7c15_u64;
        for i in 0..1_000_000_u64 {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let gross = (seed as u128) * 17;
            let haircut = ((seed >> 32) % 10_001) as u16;
            let mut asset = cash(&format!("asset-{i}"), gross);
            asset.haircut_bps = haircut;
            asset.encumbered_minor = (seed.rotate_left(13) as u128) % (gross.saturating_add(1));
            let value = asset.eligible_value_minor(policy).unwrap();
            assert!(value <= gross);
            assert!(value <= gross.saturating_sub(asset.encumbered_minor));
        }
    }

    #[test]
    fn coverage_ratio_is_absent_when_there_are_no_liabilities() {
        let coverage = reserve_coverage(&[cash("cash", 100)], 0, ReservePolicy::default()).unwrap();
        assert_eq!(coverage.coverage_bps, None);
        assert!(coverage.fully_covered);
    }

    #[test]
    fn commercial_bank_deposits_are_not_automatically_central_bank_reserves() {
        let mut asset = cash("commercial-bank-balance", 100_000);
        asset.class = ReserveAssetClass::CommercialBankDeposit;
        assert_eq!(
            asset
                .eligible_value_minor(ReservePolicy::default())
                .unwrap(),
            0
        );
    }
}
