// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Health-consent mutation service above the generic Personal typed semantic
//! dispatch bridge.
//!
//! This layer owns only typed dispatch and domain receipt classification.
//! Receipt recording, read-model reconciliation, and exact action observation
//! remain separate post-commit responsibilities.
//!
//! `MutationReceiptView` proves only the source-chain receipt identity returned
//! by the Health coordinator. It does not prove current Health snapshot
//! reconciliation, propagation, verification, authorization, or observation.

use mycelix_leptos_core::HolochainCtx;
use personal_leptos_types::{ConsentGrantInputView, MutationReceiptView};

use crate::mutation_diagnostic_runtime::MutationDiagnosticRuntime;
use crate::mutation_dispatch::{
    dispatch_personal_mutation_typed, PersonalMutationDispatchError,
};
use crate::mutation_state::PersonalMutationTarget;

/// Domain result of one append-only Health consent mutation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HealthConsentMutationOutcome {
    /// Source-chain receipt identity returned by `grant_consent_view`.
    ///
    /// This receipt remains historical mutation evidence until a later
    /// reconciliation/observation step establishes current read-model state.
    pub receipt: MutationReceiptView,
}

/// Failure before a Health consent domain outcome can be established.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HealthConsentMutationServiceError {
    /// The typed Personal semantic dispatch could not establish a decoded
    /// provider response. The diagnostic runtime owns the invocation evidence.
    Dispatch(PersonalMutationDispatchError),
}

impl From<PersonalMutationDispatchError> for HealthConsentMutationServiceError {
    fn from(error: PersonalMutationDispatchError) -> Self {
        Self::Dispatch(error)
    }
}

/// Classify the typed Health consent response without strengthening receipt
/// evidence into reconciliation or observation evidence.
pub fn classify_health_consent_mutation_result(
    receipt: MutationReceiptView,
) -> HealthConsentMutationOutcome {
    HealthConsentMutationOutcome { receipt }
}

/// Dispatch one append-only Health consent mutation through the typed Personal
/// semantic path.
///
/// The returned receipt is the strongest positive result exposed by this
/// service. Callers must still record the receipt and reconcile the Health
/// snapshot independently.
pub async fn submit_health_consent_mutation(
    hc: &HolochainCtx,
    diagnostics: MutationDiagnosticRuntime,
    input: ConsentGrantInputView,
) -> Result<HealthConsentMutationOutcome, HealthConsentMutationServiceError> {
    let success = dispatch_personal_mutation_typed::<_, MutationReceiptView>(
        hc,
        diagnostics,
        PersonalMutationTarget::HealthConsent,
        &input,
    )
    .await?;

    Ok(classify_health_consent_mutation_result(success.into_value()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_receipt_identity_is_preserved() {
        let outcome = classify_health_consent_mutation_result(MutationReceiptView {
            action_hash: "uhCkk-health-consent".into(),
        });

        assert_eq!(
            outcome,
            HealthConsentMutationOutcome {
                receipt: MutationReceiptView {
                    action_hash: "uhCkk-health-consent".into(),
                },
            },
        );
    }

    #[test]
    fn receipt_outcome_does_not_contain_reconciliation_claims() {
        let outcome = classify_health_consent_mutation_result(MutationReceiptView {
            action_hash: "uhCkk-health-consent".into(),
        });

        assert_eq!(outcome.receipt.action_hash, "uhCkk-health-consent");
    }
}
