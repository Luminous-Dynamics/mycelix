// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![forbid(unsafe_code)]

//! Simulation-only rights-share allocation and append-only revision checks.
//!
//! This module conserves one already-defined rights pool. It does not determine
//! legal ownership, authenticate evidence references, or move money.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

/// A separately accounted rights pool. Composition and master rights must not
/// be combined unless a caller has an explicit, reviewed rule for doing so.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RightsClass {
    Composition,
    MasterRecording,
}

/// Whether a share can currently be paid to its asserted claimant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ClaimStatus {
    /// The upstream rights process says the payee/share is verified.
    Confirmed,
    /// The claim cannot yet be paid; evidence or review is still pending.
    Pending,
    /// Competing ownership/share claims exist.
    Disputed,
    /// The claim has not passed the required verification process.
    Unverified,
    /// No claimant has been identified for this share.
    Unmatched,
}

/// One share in a single rights pool. Shares must be pre-aggregated by claimant.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RightsShare {
    /// Stable ID for this claim within the rights revision.
    pub claim_id: String,
    /// Opaque claimant label. This is not an authenticated legal identity.
    pub claimant_id: Option<String>,
    /// Share of the pool in basis points; all shares must sum to exactly 10,000.
    pub share_basis_points: u16,
    pub status: ClaimStatus,
}

/// Initial rights assertion or a later correction. Evidence references are
/// provenance pointers supplied by the caller, not verified signatures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RightsRevision {
    pub pool_id: String,
    pub rights_class: RightsClass,
    pub revision_number: u64,
    pub revision_id: String,
    pub parent_revision_id: Option<String>,
    pub kind: RevisionKind,
    pub evidence_ref: String,
    pub reason_ref: String,
    pub shares: Vec<RightsShare>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RevisionKind {
    Initial,
    Correction,
}

/// In-memory append-only API for a single rights pool. No mutation, removal, or
/// replacement method is exposed; durable storage and cryptographic provenance
/// remain the responsibility of a separately reviewed persistence layer.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RightsRevisionChain {
    revisions: Vec<RightsRevision>,
}

impl RightsRevisionChain {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn revisions(&self) -> &[RightsRevision] {
        &self.revisions
    }

    pub fn current(&self) -> Option<&RightsRevision> {
        self.revisions.last()
    }

    /// Append a new revision only if its numbering, parent, pool, type, and share
    /// invariants match the existing chain. Older revisions are never rewritten.
    pub fn append(&mut self, revision: RightsRevision) -> Result<(), RightsError> {
        validate_revision(&revision)?;

        match self.revisions.last() {
            None => {
                if revision.revision_number != 1
                    || revision.parent_revision_id.is_some()
                    || revision.kind != RevisionKind::Initial
                {
                    return Err(RightsError::InvalidInitialRevision);
                }
            }
            Some(previous) => {
                let expected_number = previous
                    .revision_number
                    .checked_add(1)
                    .ok_or(RightsError::ArithmeticOverflow)?;
                if revision.revision_number != expected_number {
                    return Err(RightsError::NonSequentialRevision {
                        expected: expected_number,
                        actual: revision.revision_number,
                    });
                }
                if revision.parent_revision_id.as_deref()
                    != Some(previous.revision_id.as_str())
                {
                    return Err(RightsError::ParentRevisionMismatch);
                }
                if revision.kind != RevisionKind::Correction {
                    return Err(RightsError::InvalidRevisionKind);
                }
                if revision.pool_id != previous.pool_id {
                    return Err(RightsError::PoolIdentityChanged);
                }
                if revision.rights_class != previous.rights_class {
                    return Err(RightsError::RightsClassChanged);
                }
                if self
                    .revisions
                    .iter()
                    .any(|existing| existing.revision_id == revision.revision_id)
                {
                    return Err(RightsError::DuplicateRevisionId(
                        revision.revision_id.clone(),
                    ));
                }
            }
        }

        self.revisions.push(revision);
        Ok(())
    }
}

/// A claimant-specific escrow entry. The amount remains accounted but is not
/// represented as payable to any claimant while the claim is not confirmed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EscrowLine {
    pub claim_id: String,
    pub claimant_id: Option<String>,
    pub status: ClaimStatus,
    pub amount_minor_units: u64,
}

/// Result for one pool under one exact rights revision.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RightsAllocationReport {
    pub pool_id: String,
    pub rights_class: RightsClass,
    pub revision_id: String,
    pub denomination: String,
    pub gross_pool_minor_units: u64,
    pub paid_by_claimant_minor_units: BTreeMap<String, u64>,
    pub escrow_lines: Vec<EscrowLine>,
    pub escrow_by_status_minor_units: BTreeMap<ClaimStatus, u64>,
}

impl RightsAllocationReport {
    pub fn paid_minor_units(&self) -> Option<u64> {
        self.paid_by_claimant_minor_units
            .values()
            .try_fold(0_u64, |total, amount| total.checked_add(*amount))
    }

    pub fn escrow_minor_units(&self) -> Option<u64> {
        self.escrow_lines
            .iter()
            .try_fold(0_u64, |total, line| total.checked_add(line.amount_minor_units))
    }

    /// Exact conservation: verified payouts plus held claims equal the pool.
    pub fn reconciles(&self) -> bool {
        self.paid_minor_units()
            .zip(self.escrow_minor_units())
            .and_then(|(paid, escrow)| paid.checked_add(escrow))
            == Some(self.gross_pool_minor_units)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RightsError {
    InvalidIdentifier(&'static str, String),
    InvalidShareTotal(u32),
    ZeroShare(String),
    DuplicateClaimId(String),
    DuplicateClaimant(String),
    ConfirmedShareWithoutClaimant(String),
    UnmatchedShareHasClaimant(String),
    NoShares,
    InvalidInitialRevision,
    InvalidRevisionKind,
    NonSequentialRevision { expected: u64, actual: u64 },
    ParentRevisionMismatch,
    PoolIdentityChanged,
    RightsClassChanged,
    DuplicateRevisionId(String),
    ArithmeticOverflow,
    ConservationViolation,
}

impl fmt::Display for RightsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidIdentifier(label, value) => {
                write!(f, "{label} must be non-empty and trimmed: {value:?}")
            }
            Self::InvalidShareTotal(total) => {
                write!(f, "rights shares must total 10000 basis points, got {total}")
            }
            Self::ZeroShare(id) => write!(f, "rights share must be positive: {id:?}"),
            Self::DuplicateClaimId(id) => write!(f, "duplicate claim ID: {id:?}"),
            Self::DuplicateClaimant(id) => {
                write!(f, "claimant must be pre-aggregated within this pool: {id:?}")
            }
            Self::ConfirmedShareWithoutClaimant(id) => {
                write!(f, "confirmed share lacks a claimant: {id:?}")
            }
            Self::UnmatchedShareHasClaimant(id) => {
                write!(f, "unmatched share must not identify a claimant: {id:?}")
            }
            Self::NoShares => write!(f, "rights revision must contain at least one share"),
            Self::InvalidInitialRevision => write!(
                f,
                "first revision must be revision 1, Initial, and have no parent"
            ),
            Self::InvalidRevisionKind => write!(f, "subsequent revisions must be corrections"),
            Self::NonSequentialRevision { expected, actual } => {
                write!(f, "revision number must be {expected}, got {actual}")
            }
            Self::ParentRevisionMismatch => write!(f, "parent revision does not match current head"),
            Self::PoolIdentityChanged => write!(f, "a revision cannot change its pool ID"),
            Self::RightsClassChanged => write!(f, "a revision cannot change rights class"),
            Self::DuplicateRevisionId(id) => write!(f, "duplicate revision ID: {id:?}"),
            Self::ArithmeticOverflow => write!(f, "rights allocation arithmetic overflow"),
            Self::ConservationViolation => write!(f, "rights allocation failed pool conservation"),
        }
    }
}

impl std::error::Error for RightsError {}

/// Allocate a single currency-denominated pool according to a rights revision.
/// Only Confirmed shares are paid; other statuses are explicitly escrowed.
///
/// This calculates accounting outcomes only. It does not authorize a payout.
/// The denomination must identify one unit/currency for the whole pool.
pub fn allocate_rights_pool(
    pool_id: &str,
    denomination: &str,
    amount_minor_units: u64,
    revision: &RightsRevision,
) -> Result<RightsAllocationReport, RightsError> {
    if !valid_id(pool_id) {
        return Err(RightsError::InvalidIdentifier("pool ID", pool_id.to_owned()));
    }
    if !valid_id(denomination) {
        return Err(RightsError::InvalidIdentifier(
            "denomination",
            denomination.to_owned(),
        ));
    }
    if pool_id != revision.pool_id {
        return Err(RightsError::PoolIdentityChanged);
    }
    validate_revision(revision)?;

    let apportioned = apportion_by_basis_points(amount_minor_units, &revision.shares)?;
    let mut paid_by_claimant_minor_units = BTreeMap::new();
    let mut escrow_lines = Vec::new();
    let mut escrow_by_status_minor_units = BTreeMap::new();

    for share in &revision.shares {
        let amount = *apportioned
            .get(&share.claim_id)
            .ok_or(RightsError::ConservationViolation)?;
        if share.status == ClaimStatus::Confirmed {
            let claimant = share
                .claimant_id
                .as_ref()
                .ok_or_else(|| RightsError::ConfirmedShareWithoutClaimant(share.claim_id.clone()))?;
            let paid = paid_by_claimant_minor_units
                .entry(claimant.clone())
                .or_insert(0_u64);
            *paid = paid.checked_add(amount).ok_or(RightsError::ArithmeticOverflow)?;
        } else {
            escrow_lines.push(EscrowLine {
                claim_id: share.claim_id.clone(),
                claimant_id: share.claimant_id.clone(),
                status: share.status,
                amount_minor_units: amount,
            });
            let held = escrow_by_status_minor_units
                .entry(share.status)
                .or_insert(0_u64);
            *held = held.checked_add(amount).ok_or(RightsError::ArithmeticOverflow)?;
        }
    }

    escrow_lines.sort_by(|left, right| left.claim_id.cmp(&right.claim_id));

    let report = RightsAllocationReport {
        pool_id: revision.pool_id.clone(),
        rights_class: revision.rights_class,
        revision_id: revision.revision_id.clone(),
        denomination: denomination.to_owned(),
        gross_pool_minor_units: amount_minor_units,
        paid_by_claimant_minor_units,
        escrow_lines,
        escrow_by_status_minor_units,
    };
    if !report.reconciles() {
        return Err(RightsError::ConservationViolation);
    }
    Ok(report)
}

fn validate_revision(revision: &RightsRevision) -> Result<(), RightsError> {
    for (label, value) in [
        ("pool ID", revision.pool_id.as_str()),
        ("revision ID", revision.revision_id.as_str()),
        ("evidence reference", revision.evidence_ref.as_str()),
        ("reason reference", revision.reason_ref.as_str()),
    ] {
        if !valid_id(value) {
            return Err(RightsError::InvalidIdentifier(label, value.to_owned()));
        }
    }
    validate_shares(&revision.shares)
}

fn validate_shares(shares: &[RightsShare]) -> Result<(), RightsError> {
    if shares.is_empty() {
        return Err(RightsError::NoShares);
    }
    let mut claim_ids = BTreeSet::new();
    let mut claimant_ids = BTreeSet::new();
    let mut total_basis_points = 0_u32;

    for share in shares {
        if !valid_id(&share.claim_id) {
            return Err(RightsError::InvalidIdentifier(
                "claim ID",
                share.claim_id.clone(),
            ));
        }
        if !claim_ids.insert(share.claim_id.as_str()) {
            return Err(RightsError::DuplicateClaimId(share.claim_id.clone()));
        }
        if share.share_basis_points == 0 {
            return Err(RightsError::ZeroShare(share.claim_id.clone()));
        }
        total_basis_points = total_basis_points
            .checked_add(u32::from(share.share_basis_points))
            .ok_or(RightsError::ArithmeticOverflow)?;

        match share.claimant_id.as_deref() {
            Some(claimant) => {
                if !valid_id(claimant) {
                    return Err(RightsError::InvalidIdentifier(
                        "claimant ID",
                        claimant.to_owned(),
                    ));
                }
                if !claimant_ids.insert(claimant) {
                    return Err(RightsError::DuplicateClaimant(claimant.to_owned()));
                }
            }
            None if share.status == ClaimStatus::Confirmed => {
                return Err(RightsError::ConfirmedShareWithoutClaimant(
                    share.claim_id.clone(),
                ));
            }
            None if share.status != ClaimStatus::Unmatched => {
                // Pending/disputed/unverified claims can concern an unknown payee too.
            }
            None => {}
        }
        if share.status == ClaimStatus::Unmatched && share.claimant_id.is_some() {
            return Err(RightsError::UnmatchedShareHasClaimant(
                share.claim_id.clone(),
            ));
        }
    }

    if total_basis_points != 10_000 {
        return Err(RightsError::InvalidShareTotal(total_basis_points));
    }
    Ok(())
}

/// Largest-remainder apportionment. Claims are tie-broken by stable claim ID,
/// not input ordering, so each minor unit has a deterministic destination.
fn apportion_by_basis_points(
    amount_minor_units: u64,
    shares: &[RightsShare],
) -> Result<BTreeMap<String, u64>, RightsError> {
    let mut base = BTreeMap::new();
    let mut remainders = Vec::with_capacity(shares.len());
    let mut assigned = 0_u64;

    for share in shares {
        let numerator = u128::from(amount_minor_units) * u128::from(share.share_basis_points);
        let quotient = u64::try_from(numerator / 10_000_u128)
            .map_err(|_| RightsError::ArithmeticOverflow)?;
        let remainder = u16::try_from(numerator % 10_000_u128)
            .map_err(|_| RightsError::ArithmeticOverflow)?;

        assigned = assigned
            .checked_add(quotient)
            .ok_or(RightsError::ArithmeticOverflow)?;
        base.insert(share.claim_id.clone(), quotient);
        remainders.push((share.claim_id.clone(), remainder));
    }

    let leftover = amount_minor_units
        .checked_sub(assigned)
        .ok_or(RightsError::ArithmeticOverflow)?;
    remainders.sort_by(|(left_id, left_rem), (right_id, right_rem)| {
        right_rem.cmp(left_rem).then_with(|| left_id.cmp(right_id))
    });

    // Sum of floor remainders means the leftover is strictly less than claims.
    if leftover as usize >= remainders.len() && leftover != 0 {
        return Err(RightsError::ConservationViolation);
    }
    for (claim_id, _) in remainders.into_iter().take(leftover as usize) {
        let amount = base
            .get_mut(&claim_id)
            .ok_or(RightsError::ConservationViolation)?;
        *amount = amount.checked_add(1).ok_or(RightsError::ArithmeticOverflow)?;
    }
    Ok(base)
}

fn valid_id(value: &str) -> bool {
    !value.trim().is_empty() && value.trim() == value
}

#[cfg(test)]
mod tests {
    use super::*;

    fn share(
        claim_id: &str,
        claimant_id: Option<&str>,
        basis_points: u16,
        status: ClaimStatus,
    ) -> RightsShare {
        RightsShare {
            claim_id: claim_id.to_owned(),
            claimant_id: claimant_id.map(str::to_owned),
            share_basis_points: basis_points,
            status,
        }
    }

    fn revision(
        number: u64,
        id: &str,
        parent: Option<&str>,
        kind: RevisionKind,
        shares: Vec<RightsShare>,
    ) -> RightsRevision {
        RightsRevision {
            pool_id: "pool-1".to_owned(),
            rights_class: RightsClass::Composition,
            revision_number: number,
            revision_id: id.to_owned(),
            parent_revision_id: parent.map(str::to_owned),
            kind,
            evidence_ref: format!("evidence-{id}"),
            reason_ref: format!("reason-{id}"),
            shares,
        }
    }

    fn initial_revision() -> RightsRevision {
        revision(
            1,
            "rev-1",
            None,
            RevisionKind::Initial,
            vec![
                share("claim-a", Some("writer-a"), 3333, ClaimStatus::Confirmed),
                share("claim-b", Some("writer-b"), 3333, ClaimStatus::Disputed),
                share("claim-c", None, 3334, ClaimStatus::Unmatched),
            ],
        )
    }

    #[test]
    fn confirmed_shares_are_paid_and_disputed_or_unmatched_shares_are_escrowed() {
        let input = initial_revision();
        let report = allocate_rights_pool("pool-1", "USD-cents", 1000, &input).unwrap();

        assert_eq!(report.paid_by_claimant_minor_units["writer-a"], 333);
        assert_eq!(report.escrow_by_status_minor_units[&ClaimStatus::Disputed], 333);
        assert_eq!(report.escrow_by_status_minor_units[&ClaimStatus::Unmatched], 334);
        assert_eq!(report.escrow_minor_units(), Some(667));
        assert_eq!(report.paid_minor_units(), Some(333));
        assert!(report.reconciles());
    }

    #[test]
    fn every_rights_share_must_sum_to_exactly_ten_thousand_basis_points() {
        let mut input = initial_revision();
        input.shares[2].share_basis_points = 3333;
        assert_eq!(
            allocate_rights_pool("pool-1", "USD-cents", 1000, &input),
            Err(RightsError::InvalidShareTotal(9999))
        );
    }

    #[test]
    fn duplicate_claims_and_claimants_are_rejected() {
        let mut duplicate_claim = initial_revision();
        duplicate_claim.shares[1].claim_id = "claim-a".to_owned();
        assert!(matches!(
            allocate_rights_pool("pool-1", "USD-cents", 1, &duplicate_claim),
            Err(RightsError::DuplicateClaimId(_))
        ));

        let mut duplicate_holder = initial_revision();
        duplicate_holder.shares[1].claimant_id = Some("writer-a".to_owned());
        assert!(matches!(
            allocate_rights_pool("pool-1", "USD-cents", 1, &duplicate_holder),
            Err(RightsError::DuplicateClaimant(_))
        ));
    }

    #[test]
    fn confirmed_share_must_name_a_claimant() {
        let mut input = initial_revision();
        input.shares[0].claimant_id = None;
        assert!(matches!(
            allocate_rights_pool("pool-1", "USD-cents", 10, &input),
            Err(RightsError::ConfirmedShareWithoutClaimant(_))
        ));
    }

    #[test]
    fn unmatched_share_must_not_assert_a_claimant() {
        let mut input = initial_revision();
        input.shares[2].claimant_id = Some("guessed-writer".to_owned());
        assert!(matches!(
            allocate_rights_pool("pool-1", "USD-cents", 10, &input),
            Err(RightsError::UnmatchedShareHasClaimant(_))
        ));
    }

    #[test]
    fn mismatched_pool_and_blank_denomination_are_rejected() {
        let input = initial_revision();
        assert_eq!(
            allocate_rights_pool("other-pool", "USD-cents", 1000, &input),
            Err(RightsError::PoolIdentityChanged)
        );
        assert!(matches!(
            allocate_rights_pool("pool-1", " ", 1000, &input),
            Err(RightsError::InvalidIdentifier(_, _))
        ));
    }

    #[test]
    fn largest_remainder_is_deterministic_across_claim_input_order() {
        let input = initial_revision();
        let first = allocate_rights_pool("pool-1", "USD-cents", 1001, &input).unwrap();
        let mut reversed = initial_revision();
        reversed.shares.reverse();
        let second = allocate_rights_pool("pool-1", "USD-cents", 1001, &reversed).unwrap();

        assert_eq!(first.paid_by_claimant_minor_units, second.paid_by_claimant_minor_units);
        assert_eq!(first.escrow_lines, second.escrow_lines);
        assert_eq!(first.escrow_by_status_minor_units, second.escrow_by_status_minor_units);
        assert!(first.reconciles());
    }

    #[test]
    fn initial_revision_must_start_at_one_without_parent() {
        let mut chain = RightsRevisionChain::new();
        let bad = revision(
            2,
            "rev-2",
            Some("rev-1"),
            RevisionKind::Initial,
            initial_revision().shares,
        );
        assert_eq!(chain.append(bad), Err(RightsError::InvalidInitialRevision));
        assert!(chain.revisions().is_empty());
    }

    #[test]
    fn corrections_are_sequential_and_bound_to_the_exact_parent() {
        let mut chain = RightsRevisionChain::new();
        chain.append(initial_revision()).unwrap();

        let wrong_parent = revision(
            2,
            "rev-2",
            Some("not-rev-1"),
            RevisionKind::Correction,
            initial_revision().shares,
        );
        assert_eq!(
            chain.append(wrong_parent),
            Err(RightsError::ParentRevisionMismatch)
        );
        assert_eq!(chain.revisions().len(), 1);

        let wrong_number = revision(
            3,
            "rev-3",
            Some("rev-1"),
            RevisionKind::Correction,
            initial_revision().shares,
        );
        assert_eq!(
            chain.append(wrong_number),
            Err(RightsError::NonSequentialRevision {
                expected: 2,
                actual: 3
            })
        );
    }

    #[test]
    fn a_correction_adds_a_new_revision_and_does_not_rewrite_prior_results() {
        let first = initial_revision();
        let before = allocate_rights_pool("pool-1", "USD-cents", 1000, &first).unwrap();
        let mut chain = RightsRevisionChain::new();
        chain.append(first.clone()).unwrap();

        let corrected = revision(
            2,
            "rev-2",
            Some("rev-1"),
            RevisionKind::Correction,
            vec![
                share("claim-a", Some("writer-a"), 3333, ClaimStatus::Confirmed),
                share("claim-b", Some("writer-b"), 3333, ClaimStatus::Confirmed),
                share("claim-c", Some("writer-c"), 3334, ClaimStatus::Confirmed),
            ],
        );
        chain.append(corrected).unwrap();
        let after = allocate_rights_pool(
            "pool-1",
            "USD-cents",
            1000,
            chain.current().unwrap(),
        )
        .unwrap();

        assert_eq!(chain.revisions().len(), 2);
        assert_eq!(chain.revisions()[0], first);
        assert_eq!(before.revision_id, "rev-1");
        assert_eq!(before.paid_minor_units(), Some(333));
        assert_eq!(before.escrow_minor_units(), Some(667));
        assert_eq!(after.revision_id, "rev-2");
        assert_eq!(after.paid_minor_units(), Some(1000));
        assert_eq!(after.escrow_minor_units(), Some(0));
        assert!(after.reconciles());
    }

    #[test]
    fn corrections_cannot_change_pool_identity_or_rights_class() {
        let mut chain = RightsRevisionChain::new();
        chain.append(initial_revision()).unwrap();

        let mut wrong_pool = revision(
            2,
            "rev-2",
            Some("rev-1"),
            RevisionKind::Correction,
            initial_revision().shares.clone(),
        );
        wrong_pool.pool_id = "pool-2".to_owned();
        assert_eq!(chain.append(wrong_pool), Err(RightsError::PoolIdentityChanged));

        let mut wrong_class = revision(
            2,
            "rev-2",
            Some("rev-1"),
            RevisionKind::Correction,
            initial_revision().shares,
        );
        wrong_class.rights_class = RightsClass::MasterRecording;
        assert_eq!(chain.append(wrong_class), Err(RightsError::RightsClassChanged));
        assert_eq!(chain.revisions().len(), 1);
    }

    #[test]
    fn corrections_need_evidence_and_reason_references() {
        let mut chain = RightsRevisionChain::new();
        let mut initial = initial_revision();
        initial.evidence_ref.clear();
        assert!(matches!(
            chain.append(initial),
            Err(RightsError::InvalidIdentifier("evidence reference", _))
        ));

        chain.append(initial_revision()).unwrap();
        let mut correction = revision(
            2,
            "rev-2",
            Some("rev-1"),
            RevisionKind::Correction,
            initial_revision().shares,
        );
        correction.reason_ref = " ".to_owned();
        assert!(matches!(
            chain.append(correction),
            Err(RightsError::InvalidIdentifier("reason reference", _))
        ));
        assert_eq!(chain.revisions().len(), 1);
    }

    #[test]
    fn composition_and_master_are_separate_pool_classes() {
        let input = initial_revision();
        let report = allocate_rights_pool("pool-1", "USD-cents", 100, &input).unwrap();
        assert_eq!(report.rights_class, RightsClass::Composition);
        let mut master = initial_revision();
        master.pool_id = "master-pool".to_owned();
        master.rights_class = RightsClass::MasterRecording;
        let master_report = allocate_rights_pool("master-pool", "USD-cents", 100, &master).unwrap();
        assert_eq!(master_report.rights_class, RightsClass::MasterRecording);
    }

    #[test]
    fn conservation_holds_across_split_and_pool_amount_grid() {
        for first_bp in (1000_u16..=8000_u16).step_by(1000) {
            for second_bp in (1000_u16..=(9000_u16 - first_bp)).step_by(1000) {
                let third_bp = 10_000_u16 - first_bp - second_bp;
                let input = revision(
                    1,
                    "grid-rev",
                    None,
                    RevisionKind::Initial,
                    vec![
                        share("claim-a", Some("writer-a"), first_bp, ClaimStatus::Confirmed),
                        share("claim-b", Some("writer-b"), second_bp, ClaimStatus::Disputed),
                        share("claim-c", None, third_bp, ClaimStatus::Unmatched),
                    ],
                );
                for amount in 0_u64..=128 {
                    let report =
                        allocate_rights_pool("pool-1", "USD-cents", amount, &input).unwrap();
                    assert!(report.reconciles(), "amount={amount}, shares={first_bp}/{second_bp}/{third_bp}");
                    assert_eq!(
                        report.paid_minor_units().unwrap() + report.escrow_minor_units().unwrap(),
                        amount
                    );
                }
            }
        }
    }

    #[test]
    fn maximum_u64_pool_is_conserved_without_overflow() {
        let input = revision(
            1,
            "max-rev",
            None,
            RevisionKind::Initial,
            vec![
                share("claim-a", Some("writer-a"), 1, ClaimStatus::Disputed),
                share("claim-b", Some("writer-b"), 9999, ClaimStatus::Confirmed),
            ],
        );
        let report = allocate_rights_pool("pool-1", "USD-cents", u64::MAX, &input).unwrap();
        assert_eq!(
            report.paid_minor_units().unwrap() + report.escrow_minor_units().unwrap(),
            u64::MAX
        );
        assert!(report.reconciles());
    }
}
