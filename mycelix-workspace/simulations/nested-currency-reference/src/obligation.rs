//! Sequential reference model for canonical-obligation admission.
//!
//! This is not a verifier, production ledger, or distributed uniqueness protocol.
//! A trusted obligation marker is only constructible inside this crate so future
//! adapters must cross an explicit internal trust boundary. Callers must invoke
//! that constructor only after authenticating the principal, canonical payload,
//! and verifier receipt. This module does not implement that authentication.
//!
//! This model intentionally has no settlement/release operation.

use std::collections::BTreeMap;

pub type PrincipalInstrument = (String, String);

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AdmissionError {
    EmptyIdentity,
    ZeroAmount,
    InvalidPrincipalCap,
    MissingPrincipalInstrumentCap,
    ConflictingObligationId,
    ConflictingCanonicalObligation,
    PrincipalInstrumentCapExceeded,
    Overflow,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObligationClaim {
    pub obligation_id: String,
    pub principal_id: String,
    pub instrument_id: String,
    pub shard_id: String,
    pub epoch: u64,
    pub sequence: u64,
    pub amount_units: u64,
    /// Digest of canonical underlying work, not a display label.
    pub payload_digest: [u8; 32],
    pub verifier_version: String,
    pub verifier_receipt_digest: [u8; 32],
}

/// Marker passed to admission after a trusted verifier has approved a claim.
/// This marker is not itself cryptographic proof.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerifiedObligation(ObligationClaim);

impl VerifiedObligation {
    /// Trust boundary for a future adapter inside this crate.
    ///
    /// Never call merely because a submitted hash parses or a claimant asserts
    /// that evidence is verified. The caller must already authenticate and
    /// authorize the evidence. No cryptography is implemented in this module.
    #[allow(dead_code)] // Reserved for the internal verifier adapter.
    pub(crate) fn from_trusted_verifier(
        claim: ObligationClaim,
    ) -> Result<Self, AdmissionError> {
        if claim.obligation_id.trim().is_empty()
            || claim.principal_id.trim().is_empty()
            || claim.instrument_id.trim().is_empty()
            || claim.shard_id.trim().is_empty()
            || claim.verifier_version.trim().is_empty()
        {
            return Err(AdmissionError::EmptyIdentity);
        }
        if claim.amount_units == 0 {
            return Err(AdmissionError::ZeroAmount);
        }
        Ok(Self(claim))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AdmissionReceipt {
    pub obligation_id: String,
    pub principal_id: String,
    pub instrument_id: String,
    pub amount_units: u64,
    pub payload_digest: [u8; 32],
    pub original_epoch: u64,
    pub verifier_version: String,
    pub verifier_receipt_digest: [u8; 32],
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AdmissionOutcome {
    Accepted(AdmissionReceipt),
    IdempotentReplay(AdmissionReceipt),
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct AcceptedObligation {
    principal_id: String,
    instrument_id: String,
    amount_units: u64,
    payload_digest: [u8; 32],
    receipt: AdmissionReceipt,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ObligationBook {
    principal_caps: BTreeMap<PrincipalInstrument, u64>,
    accepted: BTreeMap<String, AcceptedObligation>,
    canonical_id_by_payload_digest: BTreeMap<[u8; 32], String>,
    outstanding_by_principal_instrument: BTreeMap<PrincipalInstrument, u64>,
}

impl ObligationBook {
    /// Configures explicit principal/instrument caps. Zero disables new
    /// obligations for that pair; a missing cap fails closed.
    pub fn new(
        principal_caps: BTreeMap<PrincipalInstrument, u64>,
    ) -> Result<Self, AdmissionError> {
        if principal_caps.keys().any(|(principal, instrument)| {
            principal.trim().is_empty() || instrument.trim().is_empty()
        }) {
            return Err(AdmissionError::InvalidPrincipalCap);
        }
        Ok(Self {
            principal_caps,
            accepted: BTreeMap::new(),
            outstanding_by_principal_instrument: BTreeMap::new(),
        })
    }

    /// Admit one independently verified claim. Obligation IDs are globally
    /// unique across shard aliases and epochs. Exact replay returns the original
    /// receipt; conflicting immutable payload is rejected without mutation.
    pub fn admit(
        &mut self,
        verified: VerifiedObligation,
    ) -> Result<AdmissionOutcome, AdmissionError> {
        let claim = verified.0;
        let payload = (
            claim.principal_id.clone(),
            claim.instrument_id.clone(),
            claim.amount_units,
            claim.payload_digest,
        );

        if let Some(prior) = self.accepted.get(&claim.obligation_id) {
            let prior_payload = (
                prior.principal_id.clone(),
                prior.instrument_id.clone(),
                prior.amount_units,
                prior.payload_digest,
            );
            if payload == prior_payload {
                return Ok(AdmissionOutcome::IdempotentReplay(prior.receipt.clone()));
            }
            return Err(AdmissionError::ConflictingObligationId);
        }

        if let Some(canonical_id) = self.canonical_id_by_payload_digest.get(&claim.payload_digest) {
            let canonical = self
                .accepted
                .get(canonical_id)
                .expect("payload-digest index must point to an accepted obligation")
                .clone();
            let canonical_payload = (
                canonical.principal_id.clone(),
                canonical.instrument_id.clone(),
                canonical.amount_units,
                canonical.payload_digest,
            );
            if payload != canonical_payload {
                return Err(AdmissionError::ConflictingCanonicalObligation);
            }
            // Persist the alias ID, but preserve the original receipt and do not
            // increase exposure or the unique-obligation count.
            let receipt = canonical.receipt;
            self.accepted.insert(
                claim.obligation_id,
                AcceptedObligation {
                    principal_id: claim.principal_id,
                    instrument_id: claim.instrument_id,
                    amount_units: claim.amount_units,
                    payload_digest: claim.payload_digest,
                    receipt: receipt.clone(),
                },
            );
            return Ok(AdmissionOutcome::IdempotentReplay(receipt));
        }

        let scope = (claim.principal_id.clone(), claim.instrument_id.clone());
        let cap = self
            .principal_caps
            .get(&scope)
            .copied()
            .ok_or(AdmissionError::MissingPrincipalInstrumentCap)?;
        let used = self
            .outstanding_by_principal_instrument
            .get(&scope)
            .copied()
            .unwrap_or(0);
        let next_used = used
            .checked_add(claim.amount_units)
            .ok_or(AdmissionError::Overflow)?;
        if next_used > cap {
            return Err(AdmissionError::PrincipalInstrumentCapExceeded);
        }

        let receipt = AdmissionReceipt {
            obligation_id: claim.obligation_id.clone(),
            principal_id: claim.principal_id.clone(),
            instrument_id: claim.instrument_id.clone(),
            amount_units: claim.amount_units,
            payload_digest: claim.payload_digest,
            original_epoch: claim.epoch,
            verifier_version: claim.verifier_version,
            verifier_receipt_digest: claim.verifier_receipt_digest,
        };
        let accepted = AcceptedObligation {
            principal_id: claim.principal_id,
            instrument_id: claim.instrument_id,
            amount_units: claim.amount_units,
            payload_digest: claim.payload_digest,
            receipt: receipt.clone(),
        };

        // All checks precede mutation: rejection leaves the ID namespace and
        // principal exposure unchanged.
        self.canonical_id_by_payload_digest
            .insert(claim.payload_digest, claim.obligation_id.clone());
        self.accepted.insert(claim.obligation_id, accepted);
        self.outstanding_by_principal_instrument.insert(scope, next_used);
        Ok(AdmissionOutcome::Accepted(receipt))
    }

    pub fn accepted_count(&self) -> usize {
        // Submitted aliases do not inflate the count of unique obligations.
        self.canonical_id_by_payload_digest.len()
    }

    pub fn outstanding_for(&self, principal_id: &str, instrument_id: &str) -> u64 {
        self.outstanding_by_principal_instrument
            .get(&(principal_id.to_owned(), instrument_id.to_owned()))
            .copied()
            .unwrap_or(0)
    }

    pub fn receipt_for(&self, obligation_id: &str) -> Option<&AdmissionReceipt> {
        self.accepted.get(obligation_id).map(|accepted| &accepted.receipt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn book(cap: u64) -> ObligationBook {
        ObligationBook::new(BTreeMap::from([(
            ("principal-A".to_owned(), "instrument-1".to_owned()),
            cap,
        )]))
        .unwrap()
    }

    fn verified(
        id: &str,
        principal: &str,
        instrument: &str,
        shard: &str,
        epoch: u64,
        sequence: u64,
        amount: u64,
        payload: u8,
    ) -> Result<VerifiedObligation, AdmissionError> {
        VerifiedObligation::from_trusted_verifier(ObligationClaim {
            obligation_id: id.to_owned(),
            principal_id: principal.to_owned(),
            instrument_id: instrument.to_owned(),
            shard_id: shard.to_owned(),
            epoch,
            sequence,
            amount_units: amount,
            payload_digest: [payload; 32],
            verifier_version: "test-verifier-v1".to_owned(),
            verifier_receipt_digest: [payload.wrapping_add(1); 32],
        })
    }

    #[test]
    fn exact_replay_across_shards_and_epochs_returns_original_receipt() {
        let mut book = book(10_000);
        let first = book
            .admit(verified("ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 4_000, 7).unwrap())
            .unwrap();
        let retry = book
            .admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-9", 99, 900, 4_000, 7,
            ).unwrap())
            .unwrap();
        let AdmissionOutcome::Accepted(original) = first else {
            panic!("first submission should be accepted");
        };
        let AdmissionOutcome::IdempotentReplay(replayed) = retry else {
            panic!("second submission should replay");
        };
        assert_eq!(original, replayed);
        assert_eq!(book.accepted_count(), 1);
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 4_000);
    }

    #[test]
    fn same_canonical_work_under_new_id_is_idempotent() {
        let mut book = book(1_000);
        let first = book
            .admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-a", 1, 1, 100, 7,
            ).unwrap())
            .unwrap();
        let alias = book
            .admit(verified(
                "ob-alias", "principal-A", "instrument-1", "s-b", 99, 800, 100, 7,
            ).unwrap())
            .unwrap();
        let AdmissionOutcome::Accepted(original) = first else {
            panic!("first claim should be accepted");
        };
        let AdmissionOutcome::IdempotentReplay(replayed) = alias else {
            panic!("same canonical work should replay");
        };
        assert_eq!(original, replayed);
        assert_eq!(book.accepted_count(), 1);
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 100);
        assert_eq!(book.receipt_for("ob-alias"), Some(&original));
        assert_eq!(
            book.admit(verified(
                "ob-alias", "principal-A", "instrument-1", "s-c", 100, 900, 100, 8,
            ).unwrap()),
            Err(AdmissionError::ConflictingObligationId),
        );
        assert_eq!(book.accepted_count(), 1);
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 100);
    }

    #[test]
    fn same_payload_digest_with_conflicting_amount_or_principal_rejects() {
        let mut caps = BTreeMap::new();
        caps.insert(("principal-A".to_owned(), "instrument-1".to_owned()), 1_000);
        caps.insert(("principal-B".to_owned(), "instrument-1".to_owned()), 1_000);
        let mut book = ObligationBook::new(caps).unwrap();
        book.admit(verified(
            "ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 100, 7,
        ).unwrap()).unwrap();
        assert_eq!(
            book.admit(verified(
                "ob-2", "principal-A", "instrument-1", "s-1", 1, 2, 101, 7,
            ).unwrap()),
            Err(AdmissionError::ConflictingCanonicalObligation),
        );
        assert_eq!(
            book.admit(verified(
                "ob-3", "principal-B", "instrument-1", "s-2", 1, 3, 100, 7,
            ).unwrap()),
            Err(AdmissionError::ConflictingCanonicalObligation),
        );
        assert_eq!(book.accepted_count(), 1);
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 100);
        assert_eq!(book.outstanding_for("principal-B", "instrument-1"), 0);
    }

    #[test]
    fn same_id_different_payload_digest_rejects_without_mutation() {
        let mut book = book(10_000);
        book
            .admit(verified("ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 100, 7).unwrap())
            .unwrap();
        assert_eq!(
            book.admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-2", 2, 4, 100, 8,
            ).unwrap()),
            Err(AdmissionError::ConflictingObligationId),
        );
        assert_eq!(book.accepted_count(), 1);
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 100);
    }

    #[test]
    fn same_id_different_amount_rejects_without_mutation() {
        let mut book = book(10_000);
        book
            .admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 100, 7,
            ).unwrap())
            .unwrap();
        assert_eq!(
            book.admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-1", 1, 2, 101, 7,
            ).unwrap()),
            Err(AdmissionError::ConflictingObligationId),
        );
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 100);
    }

    #[test]
    fn same_id_different_principal_or_instrument_rejects() {
        let mut caps = BTreeMap::new();
        caps.insert(("principal-A".to_owned(), "instrument-1".to_owned()), 1_000);
        caps.insert(("principal-B".to_owned(), "instrument-1".to_owned()), 1_000);
        caps.insert(("principal-A".to_owned(), "instrument-2".to_owned()), 1_000);
        let mut book = ObligationBook::new(caps).unwrap();
        book
            .admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 100, 7,
            ).unwrap())
            .unwrap();
        assert_eq!(
            book.admit(verified(
                "ob-1", "principal-B", "instrument-1", "s-1", 1, 2, 100, 7,
            ).unwrap()),
            Err(AdmissionError::ConflictingObligationId),
        );
        assert_eq!(
            book.admit(verified(
                "ob-1", "principal-A", "instrument-2", "s-1", 1, 2, 100, 7,
            ).unwrap()),
            Err(AdmissionError::ConflictingObligationId),
        );
        assert_eq!(book.accepted_count(), 1);
    }

    #[test]
    fn distinct_obligations_share_principal_cap_across_shards() {
        let mut book = book(150);
        book
            .admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 100, 7,
            ).unwrap())
            .unwrap();
        assert_eq!(
            book.admit(verified(
                "ob-2", "principal-A", "instrument-1", "s-1", 1, 2, 51, 8,
            ).unwrap()),
            Err(AdmissionError::PrincipalInstrumentCapExceeded),
        );
        assert_eq!(book.accepted_count(), 1);
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 100);
        // The rejected ID is not consumed; corrected evidence can be retried.
        book
            .admit(verified("ob-2", "principal-A", "instrument-1", "s-1", 1, 2, 50, 8).unwrap())
            .unwrap();
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 150);
    }

    #[test]
    fn missing_principal_cap_fails_closed() {
        let mut book = ObligationBook::default();
        assert_eq!(
            book.admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 100, 7,
            ).unwrap()),
            Err(AdmissionError::MissingPrincipalInstrumentCap),
        );
        assert_eq!(book.accepted_count(), 0);
    }

    #[test]
    fn zero_amount_and_empty_identity_reject_before_admission() {
        assert_eq!(
            verified("ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 0, 7),
            Err(AdmissionError::ZeroAmount),
        );
        assert_eq!(
            verified("ob-1", " ", "instrument-1", "s-0", 1, 1, 100, 7),
            Err(AdmissionError::EmptyIdentity),
        );
    }

    #[test]
    fn arithmetic_overflow_rejects_without_mutation() {
        let mut book = book(u64::MAX);
        book
            .admit(verified(
                "ob-1", "principal-A", "instrument-1", "s-0", 1, 1, u64::MAX, 7,
            ).unwrap())
            .unwrap();
        assert_eq!(
            book.admit(verified("ob-2", "principal-A", "instrument-1", "s-0", 1, 2, 1, 8).unwrap()),
            Err(AdmissionError::Overflow),
        );
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), u64::MAX);
        assert_eq!(book.accepted_count(), 1);
    }

    #[test]
    fn zero_cap_explicitly_disables_obligations() {
        let mut book = book(0);
        assert_eq!(
            book.admit(verified("ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 1, 7).unwrap()),
            Err(AdmissionError::PrincipalInstrumentCapExceeded),
        );
        assert_eq!(book.accepted_count(), 0);
    }

    #[test]
    fn exposure_is_separated_by_principal_and_instrument() {
        let mut caps = BTreeMap::new();
        for key in [
            ("principal-A", "instrument-1"),
            ("principal-A", "instrument-2"),
            ("principal-B", "instrument-1"),
        ] {
            caps.insert((key.0.to_owned(), key.1.to_owned()), 1_000);
        }
        let mut book = ObligationBook::new(caps).unwrap();
        book
            .admit(verified("ob-1", "principal-A", "instrument-1", "s-0", 1, 1, 100, 1).unwrap())
            .unwrap();
        book
            .admit(verified("ob-2", "principal-A", "instrument-2", "s-1", 1, 2, 200, 2).unwrap())
            .unwrap();
        book
            .admit(verified("ob-3", "principal-B", "instrument-1", "s-2", 1, 3, 300, 3).unwrap())
            .unwrap();
        assert_eq!(book.outstanding_for("principal-A", "instrument-1"), 100);
        assert_eq!(book.outstanding_for("principal-A", "instrument-2"), 200);
        assert_eq!(book.outstanding_for("principal-B", "instrument-1"), 300);
    }
}
