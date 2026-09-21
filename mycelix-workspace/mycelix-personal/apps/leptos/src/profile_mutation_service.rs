// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Profile-specific mutation service above the generic Personal typed dispatch
//! bridge.
//!
//! This layer owns only Profile admission plus domain-result classification.
//! Receipt recording and read-model reconciliation deliberately remain separate
//! post-commit responsibilities.

use mycelix_leptos_core::HolochainCtx;
use personal_leptos_types::{
    ConditionalMutationResultView, MutationReceiptView, ProfileView,
};

use crate::context::PersonalSourceState;
use crate::mutation_diagnostic_runtime::MutationDiagnosticRuntime;
use crate::mutation_dispatch::{
    dispatch_personal_mutation_typed, PersonalMutationDispatchError,
};
use crate::mutation_state::PersonalMutationTarget;
use crate::profile_mutation::{admit_profile_mutation, ProfileMutationAdmissionError};

/// Domain-level result of one admitted conditional Profile mutation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProfileMutationOutcome {
    /// The backend reports that it created the conditional Profile action and
    /// returned its source-chain receipt identity.
    ///
    /// The caller must still record/reconcile that receipt separately.
    Committed { receipt: MutationReceiptView },
    /// The displayed CAS baseline no longer matches current backend state. No
    /// Profile mutation receipt exists for this attempt.
    Conflict { current_action_hash: Option<String> },
}

/// Failure before a domain Profile outcome can be established.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProfileMutationServiceError {
    /// The displayed Identity/Profile baseline was not established strongly
    /// enough to construct a conditional wire payload. No provider call occurs.
    Admission(ProfileMutationAdmissionError),
    /// The admitted semantic mutation could not establish a decoded provider
    /// response. Typed diagnostics are handled by the dispatch/runtime layer.
    Dispatch(PersonalMutationDispatchError),
}

impl From<ProfileMutationAdmissionError> for ProfileMutationServiceError {
    fn from(error: ProfileMutationAdmissionError) -> Self {
        Self::Admission(error)
    }
}

impl From<PersonalMutationDispatchError> for ProfileMutationServiceError {
    fn from(error: PersonalMutationDispatchError) -> Self {
        Self::Dispatch(error)
    }
}

/// Classify the backend's typed conditional result without strengthening a
/// successfully decoded response into commit or reconciliation claims.
pub fn classify_profile_mutation_result(
    result: ConditionalMutationResultView,
) -> ProfileMutationOutcome {
    match result {
        ConditionalMutationResultView::Committed { receipt } => {
            ProfileMutationOutcome::Committed { receipt }
        }
        ConditionalMutationResultView::Conflict {
            current_action_hash,
        } => ProfileMutationOutcome::Conflict {
            current_action_hash,
        },
    }
}

/// Admit and dispatch one conditional Profile mutation through the typed
/// semantic Personal path.
///
/// The strongest positive return is [`ProfileMutationOutcome::Committed`],
/// which carries only the source-chain receipt returned by the backend. It does
/// not establish that the current Personal snapshot has observed that action.
pub async fn submit_profile_mutation(
    hc: &HolochainCtx,
    diagnostics: MutationDiagnosticRuntime,
    identity_state: PersonalSourceState,
    profile_action_hash: Option<String>,
    profile: ProfileView,
) -> Result<ProfileMutationOutcome, ProfileMutationServiceError> {
    let input = admit_profile_mutation(identity_state, profile_action_hash, profile)?;

    let success = dispatch_personal_mutation_typed::<_, ConditionalMutationResultView>(
        hc,
        diagnostics,
        PersonalMutationTarget::Profile,
        &input,
    )
    .await?;

    Ok(classify_profile_mutation_result(success.into_value()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_result_preserves_receipt_without_claiming_reconciliation() {
        let outcome = classify_profile_mutation_result(
            ConditionalMutationResultView::Committed {
                receipt: MutationReceiptView {
                    action_hash: "uhCkk-profile-action".into(),
                },
            },
        );

        assert_eq!(
            outcome,
            ProfileMutationOutcome::Committed {
                receipt: MutationReceiptView {
                    action_hash: "uhCkk-profile-action".into(),
                },
            },
        );
    }

    #[test]
    fn conflict_preserves_current_action_identity() {
        let outcome = classify_profile_mutation_result(
            ConditionalMutationResultView::Conflict {
                current_action_hash: Some("uhCkk-current".into()),
            },
        );

        assert_eq!(
            outcome,
            ProfileMutationOutcome::Conflict {
                current_action_hash: Some("uhCkk-current".into()),
            },
        );
    }

    #[test]
    fn authoritative_empty_conflict_remains_distinct_from_transport_failure() {
        let outcome = classify_profile_mutation_result(
            ConditionalMutationResultView::Conflict {
                current_action_hash: None,
            },
        );

        assert_eq!(
            outcome,
            ProfileMutationOutcome::Conflict {
                current_action_hash: None,
            },
        );
    }
}
