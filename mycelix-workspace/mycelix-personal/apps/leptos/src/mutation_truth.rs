// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Presentation for committed Personal mutations that remain unobserved in the
//! current read model, plus semantic-target-scoped typed call diagnostics.

use leptos::prelude::*;
use mycelix_leptos_core::{AvailabilityState, AvailabilityStateKind, HolochainCallInvocationError};

use crate::diagnostic_copy::{personal_diagnostic_copy, PersonalDiagnosticCopy};
use crate::mutation_diagnostic_runtime::use_mutation_diagnostic_runtime;
use crate::mutation_diagnostics::PersonalMutationFailure;
use crate::mutation_ledger::use_mutation_ledger;
use crate::mutation_state::PersonalMutationTarget;

fn diagnostic_copy_for_failure(failure: &PersonalMutationFailure) -> PersonalDiagnosticCopy {
    match failure.invocation_error() {
        HolochainCallInvocationError::Failure(observation) => {
            personal_diagnostic_copy(observation)
        }
        HolochainCallInvocationError::AttemptSequenceExhausted { .. } => PersonalDiagnosticCopy {
            title: "Provider call capacity exhausted",
            summary: "Personal admitted this semantic mutation attempt, but the provider could not allocate another typed call-attempt identity in this provider lifetime.",
            phase_label: "admission",
            evidence_label: "attempt-sequence-exhausted",
        },
    }
}

fn diagnostic_description(copy: &PersonalDiagnosticCopy) -> String {
    format!(
        "{} Phase: {}. Evidence class: {}. {}",
        copy.summary,
        copy.phase_label,
        copy.evidence_label,
        PersonalDiagnosticCopy::AUTHORITY_NOTICE,
    )
}

#[component]
pub fn MutationPendingNotice(target: PersonalMutationTarget) -> impl IntoView {
    let ledger = use_mutation_ledger();
    let diagnostics = use_mutation_diagnostic_runtime();
    let target_for_items = target.clone();
    let target_for_diagnostics = target;

    view! {
        <div class="mutation-evidence-list">
            <For
                each=move || ledger.for_target(&target_for_items)
                key=|receipt| receipt.action_hash.clone()
                children=move |receipt| {
                    let title = receipt.title().to_string();
                    let description = receipt.description();
                    let action_hash = receipt.action_hash;
                    view! {
                        <AvailabilityState
                            kind=AvailabilityStateKind::Degraded
                            title=title
                            description=description
                            action={Some(view! {
                                <code class="hash-line">{action_hash}</code>
                            }.into_any())}
                        />
                    }
                }
            />

            {move || {
                diagnostics
                    .latest_failure_for(&target_for_diagnostics)
                    .map(|failure| {
                        let copy = diagnostic_copy_for_failure(&failure);
                        let title = copy.title.to_string();
                        let description = diagnostic_description(&copy);
                        let evidence = format!(
                            "{} · {}",
                            copy.phase_label,
                            copy.evidence_label,
                        );
                        view! {
                            <AvailabilityState
                                kind=AvailabilityStateKind::Degraded
                                title=title
                                description=description
                                action={Some(view! {
                                    <code class="hash-line">{evidence}</code>
                                }.into_any())}
                            />
                        }.into_any()
                    })
            }}
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mutation_diagnostics::{
        PersonalMutationAttemptSequence, PersonalMutationCallBinding,
    };

    #[test]
    fn provider_sequence_exhaustion_copy_is_typed_and_non_authoritative() {
        let binding = PersonalMutationCallBinding::for_target(PersonalMutationTarget::Profile);
        let mut sequence = PersonalMutationAttemptSequence::default();
        let attempt = sequence.admit(binding.clone()).expect("personal attempt");
        let failure = attempt
            .fail(HolochainCallInvocationError::attempt_sequence_exhausted(
                binding.role(),
                binding.zome(),
                binding.function(),
            ))
            .expect("matching provider target");

        let copy = diagnostic_copy_for_failure(&failure);
        assert_eq!(copy.phase_label, "admission");
        assert_eq!(copy.evidence_label, "attempt-sequence-exhausted");
        let description = diagnostic_description(&copy);
        assert!(description.contains("retry safety"));
        assert!(description.contains("transaction outcome"));
    }

    #[test]
    fn rendered_description_keeps_authority_nonclaim() {
        let copy = PersonalDiagnosticCopy {
            title: "Typed failure",
            summary: "Typed evidence was observed.",
            phase_label: "transport",
            evidence_label: "ribosome",
        };
        let description = diagnostic_description(&copy);
        assert!(description.contains("transport"));
        assert!(description.contains("ribosome"));
        assert!(description.contains("retry safety"));
        assert!(description.contains("transaction outcome"));
    }
}
