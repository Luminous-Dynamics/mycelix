//! Conservative policy for recovering uncertain external effects.
//!
//! This module computes a decision from caller-supplied provider observations
//! and a provider idempotency contract. It does not authenticate those inputs,
//! query a provider, dispatch requests, or mutate the durable journal. Callers
//! must verify source evidence and persist the selected recovery action before
//! treating it as operational authority.

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProviderIdempotencyContract {
    /// Provider offers no established duplicate-suppression guarantee.
    Unsupported,
    /// Repeating this exact request with this exact key is guaranteed to
    /// return/reuse the provider's original result until (but not including)
    /// the specified Unix epoch second. The caller must validate this contract
    /// against the provider's current documented/contractual behavior.
    SameKeySameRequest {
        provider_profile_digest: String,
        idempotency_key: String,
        request_digest: String,
        guaranteed_until_unix_seconds: u64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProviderObservation {
    /// The provider has authoritatively established that this request applied.
    Applied {
        provider_profile_digest: String,
        request_digest: String,
        receipt_digest: String,
        evidence_digest: String,
    },
    /// The provider has authoritatively established no effect was applied.
    /// no_in_flight_attempt must only be true when the provider's evidence
    /// rules out a previous request still executing asynchronously.
    DefinitelyNotApplied {
        provider_profile_digest: String,
        request_digest: String,
        evidence_digest: String,
        observed_at_unix_seconds: u64,
        no_in_flight_attempt: bool,
    },
    /// The query could not distinguish applied from not applied.
    Unknown {
        provider_profile_digest: String,
    },
    /// Provider records disagree or evidence conflicts with the operation.
    ConflictingEvidence,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RecoveryAction {
    /// Persist/verify this receipt before advancing the local operation.
    AcknowledgeReceipt {
        receipt_digest: String,
        evidence_digest: String,
    },
    /// Retry only with this original key and the identical request payload.
    RetryWithSameKey { idempotency_key: String },
    /// The caller has evidence of no effect and no in-flight request. A new
    /// key is permissible, but the operation's stable business identity must
    /// remain unchanged and the evidence must be independently verified.
    RetryWithNewKey { no_effect_evidence_digest: String },
    /// Do not dispatch and do not claim completion; obtain better evidence.
    RemainIndeterminate { reason: &'static str },
    /// Conflicting or request-mismatched evidence requires quarantine/review.
    Quarantine { reason: &'static str },
}

/// Decide the conservative next action for a potentially interrupted effect.
///
/// now_unix_seconds and max_no_effect_observation_age_seconds are explicit
/// inputs so tests are deterministic and the caller's clock assumptions remain
/// visible. This function does not verify observations or guarantee truth; it
/// only enforces the policy over the facts supplied by a separately verified
/// adapter.
pub fn decide_recovery(
    expected_provider_profile_digest: &str,
    expected_request_digest: &str,
    observation: &ProviderObservation,
    idempotency: &ProviderIdempotencyContract,
    now_unix_seconds: u64,
    max_no_effect_observation_age_seconds: u64,
) -> RecoveryAction {
    if validate_digest(expected_provider_profile_digest).is_err() {
        return RecoveryAction::Quarantine {
            reason: "expected provider-profile digest is not canonical",
        };
    }
    if validate_digest(expected_request_digest).is_err() {
        return RecoveryAction::Quarantine {
            reason: "expected request digest is not canonical",
        };
    }

    match observation {
        ProviderObservation::Applied {
            provider_profile_digest,
            request_digest,
            receipt_digest,
            evidence_digest,
        } => {
            if provider_profile_digest != expected_provider_profile_digest {
                return RecoveryAction::Quarantine {
                    reason: "provider applied record is bound to a different provider profile",
                };
            }
            if request_digest != expected_request_digest {
                return RecoveryAction::Quarantine {
                    reason: "provider applied record is bound to a different request",
                };
            }
            if validate_digest(receipt_digest).is_err() {
                return RecoveryAction::Quarantine {
                    reason: "provider receipt digest is not canonical",
                };
            }
            if validate_digest(evidence_digest).is_err() {
                return RecoveryAction::Quarantine {
                    reason: "provider evidence digest is not canonical",
                };
            }
            RecoveryAction::AcknowledgeReceipt {
                receipt_digest: receipt_digest.clone(),
                evidence_digest: evidence_digest.clone(),
            }
        }
        ProviderObservation::DefinitelyNotApplied {
            provider_profile_digest,
            request_digest,
            evidence_digest,
            observed_at_unix_seconds,
            no_in_flight_attempt,
        } => {
            if provider_profile_digest != expected_provider_profile_digest {
                return RecoveryAction::Quarantine {
                    reason: "no-effect evidence is bound to a different provider profile",
                };
            }
            if request_digest != expected_request_digest {
                return RecoveryAction::Quarantine {
                    reason: "no-effect evidence is bound to a different request",
                };
            }
            if validate_digest(evidence_digest).is_err() {
                return RecoveryAction::Quarantine {
                    reason: "no-effect evidence digest is not canonical",
                };
            }
            if !no_in_flight_attempt {
                return RecoveryAction::RemainIndeterminate {
                    reason: "a prior request may still be in flight",
                };
            }
            if *observed_at_unix_seconds > now_unix_seconds {
                return RecoveryAction::RemainIndeterminate {
                    reason: "no-effect observation timestamp is in the future",
                };
            }
            if now_unix_seconds - *observed_at_unix_seconds
                > max_no_effect_observation_age_seconds
            {
                return RecoveryAction::RemainIndeterminate {
                    reason: "no-effect observation is too old",
                };
            }

            match usable_idempotency_key(
                idempotency,
                expected_provider_profile_digest,
                expected_request_digest,
                now_unix_seconds,
            ) {
                Ok(Some(key)) => RecoveryAction::RetryWithSameKey {
                    idempotency_key: key.to_owned(),
                },
                // Once no effect and no in-flight request are established by
                // fresh authoritative evidence, a new key is safe even if the
                // provider does not support idempotency or its retention ended.
                Ok(None) => RecoveryAction::RetryWithNewKey {
                    no_effect_evidence_digest: evidence_digest.clone(),
                },
                Err(reason) => RecoveryAction::Quarantine { reason },
            }
        }
        ProviderObservation::Unknown { provider_profile_digest } => {
            if provider_profile_digest != expected_provider_profile_digest {
                return RecoveryAction::Quarantine {
                    reason: "unknown-outcome observation is bound to a different provider profile",
                };
            }
            match usable_idempotency_key(
                idempotency,
                expected_provider_profile_digest,
                expected_request_digest,
                now_unix_seconds,
            ) {
                Ok(Some(key)) => RecoveryAction::RetryWithSameKey {
                    idempotency_key: key.to_owned(),
                },
                Ok(None) => RecoveryAction::RemainIndeterminate {
                    reason: "outcome is unknown and no live idempotency guarantee permits a retry",
                },
                Err(reason) => RecoveryAction::Quarantine { reason },
            }
        }
        ProviderObservation::ConflictingEvidence => RecoveryAction::Quarantine {
            reason: "provider evidence conflicts; automatic retry is forbidden",
        },
    }
}

/// Ok(Some(key)) means the identical request may be retried with a live key;
/// Ok(None) means no live guarantee; Err means malformed or misbound data.
fn usable_idempotency_key<'a>(
    contract: &'a ProviderIdempotencyContract,
    expected_provider_profile_digest: &str,
    expected_request_digest: &str,
    now_unix_seconds: u64,
) -> Result<Option<&'a str>, &'static str> {
    match contract {
        ProviderIdempotencyContract::Unsupported => Ok(None),
        ProviderIdempotencyContract::SameKeySameRequest {
            provider_profile_digest,
            idempotency_key,
            request_digest,
            guaranteed_until_unix_seconds,
        } => {
            if validate_digest(provider_profile_digest).is_err() {
                return Err("provider contract profile digest is not canonical");
            }
            if provider_profile_digest != expected_provider_profile_digest {
                return Err("provider idempotency contract is bound to a different provider profile");
            }
            if idempotency_key.trim().is_empty() {
                return Err("provider idempotency key is empty");
            }
            if validate_digest(request_digest).is_err() {
                return Err("provider contract request digest is not canonical");
            }
            if request_digest != expected_request_digest {
                return Err("provider idempotency contract is bound to a different request");
            }
            if now_unix_seconds >= *guaranteed_until_unix_seconds {
                return Ok(None);
            }
            Ok(Some(idempotency_key))
        }
    }
}

fn validate_digest(digest: &str) -> Result<(), ()> {
    let Some(hex) = digest.strip_prefix("sha256:") else {
        return Err(());
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const PROVIDER_A: &str =
        "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";
    const PROVIDER_B: &str =
        "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";
    const REQUEST_A: &str =
        "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const REQUEST_B: &str =
        "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const RECEIPT_A: &str =
        "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
    const EVIDENCE_A: &str =
        "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
    const NOW: u64 = 10_000;

    fn live_contract() -> ProviderIdempotencyContract {
        ProviderIdempotencyContract::SameKeySameRequest {
            provider_profile_digest: PROVIDER_A.to_owned(),
            idempotency_key: "effect-key-1".to_owned(),
            request_digest: REQUEST_A.to_owned(),
            guaranteed_until_unix_seconds: NOW + 100,
        }
    }

    fn unknown() -> ProviderObservation {
        ProviderObservation::Unknown {
            provider_profile_digest: PROVIDER_A.to_owned(),
        }
    }

    #[test]
    fn applied_matching_request_returns_exact_receipt_for_acknowledgement() {
        assert_eq!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::Applied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_A.to_owned(),
                    receipt_digest: RECEIPT_A.to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                },
                &ProviderIdempotencyContract::Unsupported,
                NOW,
                30,
            ),
            RecoveryAction::AcknowledgeReceipt {
                receipt_digest: RECEIPT_A.to_owned(),
                evidence_digest: EVIDENCE_A.to_owned(),
            },
        );
    }

    #[test]
    fn applied_record_for_different_request_is_quarantined() {
        assert_eq!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::Applied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_B.to_owned(),
                    receipt_digest: RECEIPT_A.to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                },
                &ProviderIdempotencyContract::Unsupported,
                NOW,
                30,
            ),
            RecoveryAction::Quarantine {
                reason: "provider applied record is bound to a different request",
            },
        );
    }

    #[test]
    fn malformed_receipt_digest_is_quarantined() {
        assert!(matches!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::Applied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_A.to_owned(),
                    receipt_digest: "sha256:BAD".to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                },
                &ProviderIdempotencyContract::Unsupported,
                NOW,
                30,
            ),
            RecoveryAction::Quarantine { .. },
        ));
    }

    #[test]
    fn unknown_outcome_can_retry_only_with_same_live_key_and_request() {
        assert_eq!(
            decide_recovery(PROVIDER_A, REQUEST_A, &unknown(), &live_contract(), NOW, 30),
            RecoveryAction::RetryWithSameKey {
                idempotency_key: "effect-key-1".to_owned(),
            },
        );
    }

    #[test]
    fn idempotency_guarantee_is_expired_at_exact_deadline() {
        let contract = ProviderIdempotencyContract::SameKeySameRequest {
            provider_profile_digest: PROVIDER_A.to_owned(),
            idempotency_key: "effect-key-1".to_owned(),
            request_digest: REQUEST_A.to_owned(),
            guaranteed_until_unix_seconds: NOW,
        };
        assert!(matches!(
            decide_recovery(PROVIDER_A, REQUEST_A, &unknown(), &contract, NOW, 30),
            RecoveryAction::RemainIndeterminate { .. },
        ));
    }

    #[test]
    fn unknown_outcome_never_uses_a_key_bound_to_another_request() {
        let contract = ProviderIdempotencyContract::SameKeySameRequest {
            provider_profile_digest: PROVIDER_A.to_owned(),
            idempotency_key: "old-request-key".to_owned(),
            request_digest: REQUEST_B.to_owned(),
            guaranteed_until_unix_seconds: NOW + 100,
        };
        assert!(matches!(
            decide_recovery(PROVIDER_A, REQUEST_A, &unknown(), &contract, NOW, 30),
            RecoveryAction::Quarantine { .. },
        ));
    }

    #[test]
    fn fresh_definitive_no_effect_evidence_allows_new_key_after_expiry() {
        assert_eq!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::DefinitelyNotApplied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_A.to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                    observed_at_unix_seconds: NOW - 5,
                    no_in_flight_attempt: true,
                },
                &ProviderIdempotencyContract::SameKeySameRequest {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    idempotency_key: "effect-key-1".to_owned(),
                    request_digest: REQUEST_A.to_owned(),
                    guaranteed_until_unix_seconds: NOW,
                },
                NOW,
                30,
            ),
            RecoveryAction::RetryWithNewKey {
                no_effect_evidence_digest: EVIDENCE_A.to_owned(),
            },
        );
    }

    #[test]
    fn stale_no_effect_evidence_does_not_authorize_retry() {
        assert!(matches!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::DefinitelyNotApplied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_A.to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                    observed_at_unix_seconds: NOW - 31,
                    no_in_flight_attempt: true,
                },
                &ProviderIdempotencyContract::Unsupported,
                NOW,
                30,
            ),
            RecoveryAction::RemainIndeterminate { .. },
        ));
    }

    #[test]
    fn possible_in_flight_request_does_not_authorize_fresh_key() {
        assert!(matches!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::DefinitelyNotApplied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_A.to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                    observed_at_unix_seconds: NOW - 1,
                    no_in_flight_attempt: false,
                },
                &ProviderIdempotencyContract::Unsupported,
                NOW,
                30,
            ),
            RecoveryAction::RemainIndeterminate { .. },
        ));
    }

    #[test]
    fn future_observation_timestamp_does_not_authorize_retry() {
        assert!(matches!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::DefinitelyNotApplied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_A.to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                    observed_at_unix_seconds: NOW + 1,
                    no_in_flight_attempt: true,
                },
                &ProviderIdempotencyContract::Unsupported,
                NOW,
                30,
            ),
            RecoveryAction::RemainIndeterminate { .. },
        ));
    }

    #[test]
    fn misbound_no_effect_evidence_is_quarantined() {
        assert!(matches!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::DefinitelyNotApplied {
                    provider_profile_digest: PROVIDER_A.to_owned(),
                    request_digest: REQUEST_B.to_owned(),
                    evidence_digest: EVIDENCE_A.to_owned(),
                    observed_at_unix_seconds: NOW - 1,
                    no_in_flight_attempt: true,
                },
                &ProviderIdempotencyContract::Unsupported,
                NOW,
                30,
            ),
            RecoveryAction::Quarantine { .. },
        ));
    }

    #[test]
    fn conflicting_provider_evidence_is_quarantined() {
        assert!(matches!(
            decide_recovery(
                PROVIDER_A,
                REQUEST_A,
                &ProviderObservation::ConflictingEvidence,
                &live_contract(),
                NOW,
                30,
            ),
            RecoveryAction::Quarantine { .. },
        ));
    }

    #[test]
    fn invalid_expected_request_digest_is_quarantined() {
        assert!(matches!(
            decide_recovery(
                PROVIDER_A,
                "not-a-digest",
                &unknown(),
                &live_contract(),
                NOW,
                30,
            ),
            RecoveryAction::Quarantine { .. },
        ));
    }

    #[test]
    fn malformed_idempotency_key_is_quarantined_instead_of_retried() {
        let contract = ProviderIdempotencyContract::SameKeySameRequest {
            provider_profile_digest: PROVIDER_A.to_owned(),
            idempotency_key: " ".to_owned(),
            request_digest: REQUEST_A.to_owned(),
            guaranteed_until_unix_seconds: NOW + 100,
        };
        assert!(matches!(
            decide_recovery(PROVIDER_A, REQUEST_A, &unknown(), &contract, NOW, 30),
            RecoveryAction::Quarantine { .. },
        ));
    }
}
