// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Preference-specific mutation service above the generic Personal typed
//! semantic dispatch bridge.
//!
//! This layer owns existing-row admission plus domain-result classification.
//! Receipt recording, optimistic-view policy, and read-model reconciliation
//! remain separate responsibilities.

use mycelix_leptos_core::HolochainCtx;
use personal_leptos_types::{
    ConditionalMutationResultView, DataSharingPreferenceView, MutationReceiptView,
};

use crate::mutation_diagnostic_runtime::MutationDiagnosticRuntime;
use crate::mutation_dispatch::{
    dispatch_personal_mutation_typed, PersonalMutationDispatchError,
};
use crate::mutation_state::PersonalMutationTarget;
use crate::preference_mutation::{
    admit_existing_preference_mutation, PreferenceMutationAdmissionError,
};

/// Domain result for one exact existing Preference-pair replacement attempt.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PreferenceMutationOutcome {
    /// The backend reports a committed action and source-chain receipt for this
    /// exact semantic pair. The receipt still requires independent read-model
    /// reconciliation/observation before stronger current-state presentation.
    Committed {
        target: PersonalMutationTarget,
        receipt: MutationReceiptView,
    },
    /// The pair's displayed CAS baseline was stale. The backend performed no
    /// Preference mutation for this attempt.
    Conflict {
        target: PersonalMutationTarget,
        current_action_hash: Option<String>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PreferenceMutationServiceError {
    Admission(PreferenceMutationAdmissionError),
    Dispatch(PersonalMutationDispatchError),
}

impl From<PreferenceMutationAdmissionError> for PreferenceMutationServiceError {
    fn from(error: PreferenceMutationAdmissionError) -> Self {
        Self::Admission(error)
    }
}

impl From<PersonalMutationDispatchError> for PreferenceMutationServiceError {
    fn from(error: PersonalMutationDispatchError) -> Self {
        Self::Dispatch(error)
    }
}

/// Classify one successfully decoded conditional Preference response while
/// retaining the exact semantic pair that produced it.
///
/// Kept module-private so callers cannot bypass admission and tag an arbitrary
/// non-Preference `PersonalMutationTarget` as a Preference outcome.
fn classify_preference_mutation_result(
    target: PersonalMutationTarget,
    result: ConditionalMutationResultView,
) -> PreferenceMutationOutcome {
    match result {
        ConditionalMutationResultView::Committed { receipt } => {
            PreferenceMutationOutcome::Committed { target, receipt }
        }
        ConditionalMutationResultView::Conflict {
            current_action_hash,
        } => PreferenceMutationOutcome::Conflict {
            target,
            current_action_hash,
        },
    }
}

/// Admit and dispatch one conditional replacement of an existing Preference
/// row using its exact source-backed action identity and semantic pair.
pub async fn submit_existing_preference_mutation(
    hc: &HolochainCtx,
    diagnostics: MutationDiagnosticRuntime,
    target: PersonalMutationTarget,
    expected_action_hash: Option<String>,
    preference: DataSharingPreferenceView,
) -> Result<PreferenceMutationOutcome, PreferenceMutationServiceError> {
    let input = admit_existing_preference_mutation(
        target.clone(),
        expected_action_hash,
        preference,
    )?;

    let success = dispatch_personal_mutation_typed::<_, ConditionalMutationResultView>(
        hc,
        diagnostics,
        target.clone(),
        &input,
    )
    .await?;

    Ok(classify_preference_mutation_result(
        target,
        success.into_value(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target() -> PersonalMutationTarget {
        PersonalMutationTarget::preference("health", "finance")
    }

    #[test]
    fn committed_result_retains_exact_pair_and_receipt() {
        let outcome = classify_preference_mutation_result(
            target(),
            ConditionalMutationResultView::Committed {
                receipt: MutationReceiptView {
                    action_hash: "uhCkk-preference-action".into(),
                },
            },
        );

        assert_eq!(
            outcome,
            PreferenceMutationOutcome::Committed {
                target: target(),
                receipt: MutationReceiptView {
                    action_hash: "uhCkk-preference-action".into(),
                },
            },
        );
    }

    #[test]
    fn conflict_retains_exact_pair_and_current_action_identity() {
        let outcome = classify_preference_mutation_result(
            target(),
            ConditionalMutationResultView::Conflict {
                current_action_hash: Some("uhCkk-current-preference".into()),
            },
        );

        assert_eq!(
            outcome,
            PreferenceMutationOutcome::Conflict {
                target: target(),
                current_action_hash: Some("uhCkk-current-preference".into()),
            },
        );
    }

    #[test]
    fn authoritative_empty_conflict_remains_pair_scoped_domain_outcome() {
        let outcome = classify_preference_mutation_result(
            target(),
            ConditionalMutationResultView::Conflict {
                current_action_hash: None,
            },
        );

        assert_eq!(
            outcome,
            PreferenceMutationOutcome::Conflict {
                target: target(),
                current_action_hash: None,
            },
        );
    }
}
