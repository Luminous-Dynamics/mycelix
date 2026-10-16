//! Serial claim-identity registry for exactly-once evidence semantics.
//!
//! This is the coordinated counterpart to `fulfillment_escrow_partition`.
//! It enforces one claim identity across replicas only because all operations
//! pass through one authoritative in-memory registry. It is not available during
//! a network partition, not a distributed consensus implementation, and not a
//! cryptographic proof that the underlying physical delivery occurred.

use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimRegistryError {
    InvalidRequestId,
    InvalidClaimId,
    InvalidReplicaId,
    InvalidDeliveryDigest,
    InvalidQuantity,
    UnknownClaim,
    ConflictingClaimIdentity,
    InvalidAcceptedQuantity,
    ConflictingFinalization,
    ArithmeticOverflow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimRegistrationOutcome {
    Registered,
    ExactReplay,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimFinalizationOutcome {
    Finalized,
    ExactReplay,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimStatus {
    Pending,
    Finalized { accepted_quantity: u64 },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClaimIdentityRecord {
    pub request_id: String,
    pub claim_id: String,
    pub replica_id: String,
    pub delivery_digest: String,
    pub claimed_quantity: u64,
    pub status: ClaimStatus,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ClaimIdentityJournalEntry {
    Registered {
        request_id: String,
        claim_id: String,
        replica_id: String,
        delivery_digest: String,
        claimed_quantity: u64,
    },
    Finalized {
        claim_id: String,
        accepted_quantity: u64,
        released_quantity: u64,
    },
}

/// A single serialized authority for globally scoped claim identities.
///
/// Safety property: within this registry instance, each claim ID can bind to
/// exactly one replica, delivery digest, and claimed quantity. Exact retries are
/// idempotent; conflicting reuse fails closed. A replica unable to reach this
/// registry must remain pending or decline finalization—there is intentionally
/// no offline "assume unique" path.
#[derive(Debug, Default)]
pub struct ClaimIdentityRegistry {
    records: BTreeMap<String, ClaimIdentityRecord>,
    journal: Vec<ClaimIdentityJournalEntry>,
}

impl ClaimIdentityRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record(&self, claim_id: &str) -> Option<&ClaimIdentityRecord> {
        self.records.get(claim_id)
    }

    pub fn journal(&self) -> &[ClaimIdentityJournalEntry] {
        &self.journal
    }

    pub fn claim_count(&self) -> usize {
        self.records.len()
    }

    pub fn pending_quantity(&self) -> Result<u64, ClaimRegistryError> {
        self.records.values().try_fold(0_u64, |sum, record| {
            if record.status == ClaimStatus::Pending {
                sum.checked_add(record.claimed_quantity)
                    .ok_or(ClaimRegistryError::ArithmeticOverflow)
            } else {
                Ok(sum)
            }
        })
    }

    pub fn accepted_quantity(&self) -> Result<u64, ClaimRegistryError> {
        self.records.values().try_fold(0_u64, |sum, record| {
            if let ClaimStatus::Finalized { accepted_quantity } = record.status {
                sum.checked_add(accepted_quantity)
                    .ok_or(ClaimRegistryError::ArithmeticOverflow)
            } else {
                Ok(sum)
            }
        })
    }

    /// Register a claim identity under the shared serialized authority.
    ///
    /// Retrying the exact tuple is idempotent and does not append another event.
    /// Reusing the claim ID with any different binding is rejected, including
    /// the same delivery digest claimed by a different replica.
    pub fn register(
        &mut self,
        request_id: &str,
        claim_id: &str,
        replica_id: &str,
        delivery_digest: &str,
        claimed_quantity: u64,
    ) -> Result<ClaimRegistrationOutcome, ClaimRegistryError> {
        if request_id.trim().is_empty() {
            return Err(ClaimRegistryError::InvalidRequestId);
        }
        if claim_id.trim().is_empty() {
            return Err(ClaimRegistryError::InvalidClaimId);
        }
        if replica_id.trim().is_empty() {
            return Err(ClaimRegistryError::InvalidReplicaId);
        }
        if !is_canonical_sha256(delivery_digest) {
            return Err(ClaimRegistryError::InvalidDeliveryDigest);
        }
        if claimed_quantity == 0 {
            return Err(ClaimRegistryError::InvalidQuantity);
        }

        if let Some(existing) = self.records.get(claim_id) {
            let same_binding = existing.request_id == request_id
                && existing.replica_id == replica_id
                && existing.delivery_digest == delivery_digest
                && existing.claimed_quantity == claimed_quantity;
            return if same_binding {
                Ok(ClaimRegistrationOutcome::ExactReplay)
            } else {
                Err(ClaimRegistryError::ConflictingClaimIdentity)
            };
        }

        let record = ClaimIdentityRecord {
            request_id: request_id.to_owned(),
            claim_id: claim_id.to_owned(),
            replica_id: replica_id.to_owned(),
            delivery_digest: delivery_digest.to_owned(),
            claimed_quantity,
            status: ClaimStatus::Pending,
        };
        self.journal.push(ClaimIdentityJournalEntry::Registered {
            request_id: request_id.to_owned(),
            claim_id: claim_id.to_owned(),
            replica_id: replica_id.to_owned(),
            delivery_digest: delivery_digest.to_owned(),
            claimed_quantity,
        });
        self.records.insert(claim_id.to_owned(), record);
        Ok(ClaimRegistrationOutcome::Registered)
    }

    /// Finalize a registered claim once, releasing unaccepted quantity.
    ///
    /// Repeating the same finalization is idempotent and leaves the journal
    /// unchanged. A different finalization for an already final claim is a
    /// conflict. This method accounts for the disposition; it does not verify
    /// the source or truth of delivery evidence.
    pub fn finalize(
        &mut self,
        claim_id: &str,
        accepted_quantity: u64,
    ) -> Result<ClaimFinalizationOutcome, ClaimRegistryError> {
        let existing = self
            .records
            .get(claim_id)
            .cloned()
            .ok_or(ClaimRegistryError::UnknownClaim)?;

        match existing.status {
            ClaimStatus::Finalized {
                accepted_quantity: previous,
            } => {
                return if previous == accepted_quantity {
                    Ok(ClaimFinalizationOutcome::ExactReplay)
                } else {
                    Err(ClaimRegistryError::ConflictingFinalization)
                };
            }
            ClaimStatus::Pending => {}
        }

        if accepted_quantity > existing.claimed_quantity {
            return Err(ClaimRegistryError::InvalidAcceptedQuantity);
        }
        let released_quantity = existing
            .claimed_quantity
            .checked_sub(accepted_quantity)
            .ok_or(ClaimRegistryError::InvalidAcceptedQuantity)?;

        self.records
            .get_mut(claim_id)
            .ok_or(ClaimRegistryError::UnknownClaim)?
            .status = ClaimStatus::Finalized { accepted_quantity };
        self.journal.push(ClaimIdentityJournalEntry::Finalized {
            claim_id: claim_id.to_owned(),
            accepted_quantity,
            released_quantity,
        });
        Ok(ClaimFinalizationOutcome::Finalized)
    }
}

fn is_canonical_sha256(value: &str) -> bool {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return false;
    };
    hex.len() == 64
        && hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DIGEST_A: &str =
        "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const DIGEST_B: &str =
        "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    #[test]
    fn invalid_identity_components_fail_closed() {
        let mut registry = ClaimIdentityRegistry::new();
        assert_eq!(
            registry.register("", "claim-1", "replica-a", DIGEST_A, 1),
            Err(ClaimRegistryError::InvalidRequestId)
        );
        assert_eq!(
            registry.register("request-1", "", "replica-a", DIGEST_A, 1),
            Err(ClaimRegistryError::InvalidClaimId)
        );
        assert_eq!(
            registry.register("request-1", "claim-1", "", DIGEST_A, 1),
            Err(ClaimRegistryError::InvalidReplicaId)
        );
        assert_eq!(
            registry.register("request-1", "claim-1", "replica-a", "sha256:ABC", 1),
            Err(ClaimRegistryError::InvalidDeliveryDigest)
        );
        assert_eq!(
            registry.register("request-1", "claim-1", "replica-a", DIGEST_A, 0),
            Err(ClaimRegistryError::InvalidQuantity)
        );
        assert_eq!(registry.journal().len(), 0);
    }

    #[test]
    fn exact_registration_retry_is_idempotent() {
        let mut registry = ClaimIdentityRegistry::new();
        assert_eq!(
            registry.register("request-1", "claim-1", "replica-a", DIGEST_A, 5),
            Ok(ClaimRegistrationOutcome::Registered)
        );
        let journal_len = registry.journal().len();
        assert_eq!(
            registry.register("request-1", "claim-1", "replica-a", DIGEST_A, 5),
            Ok(ClaimRegistrationOutcome::ExactReplay)
        );
        assert_eq!(registry.journal().len(), journal_len);
        assert_eq!(registry.claim_count(), 1);
    }

    #[test]
    fn same_claim_identity_cannot_be_registered_by_two_replicas() {
        let mut registry = ClaimIdentityRegistry::new();
        registry
            .register("request-1", "external-delivery-42", "replica-a", DIGEST_A, 3)
            .unwrap();
        let journal_len = registry.journal().len();
        assert_eq!(
            registry.register("request-1", "external-delivery-42", "replica-b", DIGEST_A, 3),
            Err(ClaimRegistryError::ConflictingClaimIdentity)
        );
        assert_eq!(registry.journal().len(), journal_len);
        assert_eq!(
            registry.record("external-delivery-42").unwrap().replica_id,
            "replica-a"
        );
    }

    #[test]
    fn claim_id_cannot_be_reused_for_different_payload_or_quantity() {
        let mut registry = ClaimIdentityRegistry::new();
        registry
            .register("request-1", "claim-1", "replica-a", DIGEST_A, 5)
            .unwrap();
        assert_eq!(
            registry.register("request-1", "claim-1", "replica-a", DIGEST_B, 5),
            Err(ClaimRegistryError::ConflictingClaimIdentity)
        );
        assert_eq!(
            registry.register("request-1", "claim-1", "replica-a", DIGEST_A, 4),
            Err(ClaimRegistryError::ConflictingClaimIdentity)
        );
        assert_eq!(registry.claim_count(), 1);
    }

    #[test]
    fn claim_id_cannot_be_replayed_against_another_request() {
        let mut registry = ClaimIdentityRegistry::new();
        registry
            .register("request-1", "claim-1", "replica-a", DIGEST_A, 5)
            .unwrap();
        assert_eq!(
            registry.register("request-2", "claim-1", "replica-a", DIGEST_A, 5),
            Err(ClaimRegistryError::ConflictingClaimIdentity)
        );
        assert_eq!(registry.claim_count(), 1);
    }

    #[test]
    fn invalid_finalization_does_not_mutate_state_or_journal() {
        let mut registry = ClaimIdentityRegistry::new();
        registry.register("request-1", "claim-1", "replica-a", DIGEST_A, 5).unwrap();
        let before = registry.record("claim-1").unwrap().clone();
        let journal_len = registry.journal().len();
        assert_eq!(
            registry.finalize("claim-1", 6),
            Err(ClaimRegistryError::InvalidAcceptedQuantity)
        );
        assert_eq!(registry.record("claim-1").unwrap(), &before);
        assert_eq!(registry.journal().len(), journal_len);
    }

    #[test]
    fn finalization_retry_is_idempotent_but_conflicting_retry_is_rejected() {
        let mut registry = ClaimIdentityRegistry::new();
        registry.register("request-1", "claim-1", "replica-a", DIGEST_A, 5).unwrap();
        assert_eq!(
            registry.finalize("claim-1", 3),
            Ok(ClaimFinalizationOutcome::Finalized)
        );
        let journal_len = registry.journal().len();
        assert_eq!(
            registry.finalize("claim-1", 3),
            Ok(ClaimFinalizationOutcome::ExactReplay)
        );
        assert_eq!(registry.journal().len(), journal_len);
        assert_eq!(
            registry.finalize("claim-1", 4),
            Err(ClaimRegistryError::ConflictingFinalization)
        );
        assert_eq!(registry.accepted_quantity(), Ok(3));
        assert_eq!(registry.pending_quantity(), Ok(0));
    }

    #[test]
    fn unknown_claim_cannot_be_finalized() {
        let mut registry = ClaimIdentityRegistry::new();
        assert_eq!(
            registry.finalize("unknown", 0),
            Err(ClaimRegistryError::UnknownClaim)
        );
        assert!(registry.journal().is_empty());
    }

    #[test]
    fn deterministic_10k_claim_campaign_rejects_conflicts_without_journal_growth() {
        let mut registry = ClaimIdentityRegistry::new();

        for seed in 1_u64..=10_000 {
            let request_id = format!("request-{seed}");
            let claim_id = format!("claim-{seed}");
            let replica_id = format!("replica-{}", seed % 8);
            let quantity = seed % 97 + 1;
            let accepted = quantity / 2;

            assert_eq!(
                registry.register(&request_id, &claim_id, &replica_id, DIGEST_A, quantity),
                Ok(ClaimRegistrationOutcome::Registered),
                "seed={seed}"
            );
            assert_eq!(
                registry.register(&request_id, &claim_id, &replica_id, DIGEST_A, quantity),
                Ok(ClaimRegistrationOutcome::ExactReplay),
                "exact replay seed={seed}"
            );
            assert_eq!(
                registry.register(
                    &format!("other-{seed}"),
                    &claim_id,
                    &replica_id,
                    DIGEST_A,
                    quantity
                ),
                Err(ClaimRegistryError::ConflictingClaimIdentity),
                "cross-request replay seed={seed}"
            );
            assert_eq!(
                registry.register(
                    &request_id,
                    &claim_id,
                    &format!("other-{replica_id}"),
                    DIGEST_A,
                    quantity
                ),
                Err(ClaimRegistryError::ConflictingClaimIdentity),
                "cross-replica replay seed={seed}"
            );
            assert_eq!(
                registry.register(&request_id, &claim_id, &replica_id, DIGEST_B, quantity),
                Err(ClaimRegistryError::ConflictingClaimIdentity),
                "digest substitution seed={seed}"
            );
            assert_eq!(
                registry.register(&request_id, &claim_id, &replica_id, DIGEST_A, quantity + 1),
                Err(ClaimRegistryError::ConflictingClaimIdentity),
                "quantity substitution seed={seed}"
            );
            assert_eq!(
                registry.finalize(&claim_id, accepted),
                Ok(ClaimFinalizationOutcome::Finalized),
                "finalize seed={seed}"
            );
            assert_eq!(
                registry.finalize(&claim_id, accepted),
                Ok(ClaimFinalizationOutcome::ExactReplay),
                "finalization replay seed={seed}"
            );
            let conflicting = if accepted < quantity { accepted + 1 } else { accepted - 1 };
            assert_eq!(
                registry.finalize(&claim_id, conflicting),
                Err(ClaimRegistryError::ConflictingFinalization),
                "conflicting finalization seed={seed}"
            );
        }

        assert_eq!(registry.claim_count(), 10_000);
        assert_eq!(registry.journal().len(), 20_000);
        assert_eq!(registry.pending_quantity(), Ok(0));
        assert_eq!(
            registry.accepted_quantity(),
            Ok((1_u64..=10_000).map(|seed| (seed % 97 + 1) / 2).sum())
        );
    }

    #[test]
    fn one_identity_is_accepted_once_in_the_serial_authoritative_history() {
        let mut registry = ClaimIdentityRegistry::new();
        registry
            .register("request-1", "external-delivery-42", "replica-a", DIGEST_A, 3)
            .unwrap();
        registry.finalize("external-delivery-42", 3).unwrap();
        assert_eq!(
            registry.register("request-1", "external-delivery-42", "replica-b", DIGEST_A, 3),
            Err(ClaimRegistryError::ConflictingClaimIdentity)
        );
        assert_eq!(registry.accepted_quantity(), Ok(3));
        assert_eq!(
            registry
                .journal()
                .iter()
                .filter(|entry| matches!(
                    entry,
                    ClaimIdentityJournalEntry::Finalized {
                        claim_id,
                        accepted_quantity: 3,
                        ..
                    } if claim_id == "external-delivery-42"
                ))
                .count(),
            1
        );
    }
}
