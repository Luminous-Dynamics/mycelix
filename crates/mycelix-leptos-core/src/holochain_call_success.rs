// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-level evidence for a successfully decoded typed Holochain call.
//!
//! A decoded zome response is not automatically mutation authority. This type
//! exists so callers can correlate a successful provider attempt without
//! promoting that completion into source-chain commit, action-hash, receipt,
//! authorization, reconciliation, or currentness evidence.

use crate::HolochainCallAttemptId;

/// One successfully decoded typed provider-call response bound to the exact
/// provider-local attempt that produced it.
///
/// `HolochainCallSuccess<T>` establishes only:
///
/// - a typed call attempt was admitted by this provider instance;
/// - the transport returned response bytes;
/// - those bytes decoded as `T`.
///
/// It does **not** establish that `T` represents a committed mutation, that a
/// source-chain action exists, that authorization remains current, or that any
/// Personal read model has reconciled the result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HolochainCallSuccess<T> {
    attempt_id: HolochainCallAttemptId,
    value: T,
}

impl<T> HolochainCallSuccess<T> {
    pub(crate) fn new(attempt_id: HolochainCallAttemptId, value: T) -> Self {
        Self { attempt_id, value }
    }

    /// Provider-local attempt identity for this decoded response.
    ///
    /// This is diagnostic/correlation identity only. It is not a transport
    /// request id, mutation receipt id, Holochain action hash, or commit proof.
    pub const fn attempt_id(&self) -> HolochainCallAttemptId {
        self.attempt_id
    }

    pub const fn value(&self) -> &T {
        &self.value
    }

    pub fn into_value(self) -> T {
        self.value
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::HolochainCallAttemptSequence;

    #[test]
    fn success_retains_exact_provider_attempt_and_decoded_value() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let attempt_id = sequence.allocate().expect("attempt id");
        let success = HolochainCallSuccess::new(attempt_id, "decoded-value".to_string());

        assert_eq!(success.attempt_id(), attempt_id);
        assert_eq!(success.value(), "decoded-value");
        assert_eq!(success.into_value(), "decoded-value");
    }

    #[test]
    fn success_attempt_identity_is_provider_local_not_domain_receipt_identity() {
        let mut sequence = HolochainCallAttemptSequence::default();
        let attempt_id = sequence.allocate().expect("attempt id");
        let success = HolochainCallSuccess::new(attempt_id, "uhCkk-action-hash-looking-value");

        assert_eq!(success.attempt_id().get(), 1);
        assert_eq!(success.value(), &"uhCkk-action-hash-looking-value");
    }
}
