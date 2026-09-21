// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Presentation for committed Personal mutations that remain unobserved in the
//! current read model, plus target-scoped typed call diagnostics where the
//! mutation target maps unambiguously to one provider call target.

use leptos::prelude::*;
use mycelix_leptos_core::{
    AvailabilityState, AvailabilityStateKind, HolochainCallTarget,
};

use crate::diagnostic_copy::{personal_diagnostic_copy, PersonalDiagnosticCopy};
use crate::mutation_ledger::use_mutation_ledger;
use crate::mutation_state::PersonalMutationTarget;

fn diagnostic_call_target(target: &PersonalMutationTarget) -> Option<HolochainCallTarget> {
    match target {
        PersonalMutationTarget::Profile => Some(HolochainCallTarget::new(
            "personal",
            "identity_vault",
            "set_profile_view_if_current",
        )),
        PersonalMutationTarget::HealthConsent => Some(HolochainCallTarget::new(
            "personal",
            "health_vault",
            "grant_consent_view",
        )),
        PersonalMutationTarget::Preference { .. } => None,
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
    let hc = mycelix_leptos_core::holochain_provider::use_holochain();
    let target_for_items = target.clone();
    let target_for_diagnostics = diagnostic_call_target(&target);

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
                target_for_diagnostics
                    .as_ref()
                    .and_then(|call_target| {
                        hc.target_call_diagnostics
                            .get()
                            .latest_failure_for(call_target)
                            .cloned()
                    })
                    .map(|observation| {
                        let copy = personal_diagnostic_copy(&observation);
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

    #[test]
    fn profile_maps_to_exact_conditional_write_target() {
        let target = diagnostic_call_target(&PersonalMutationTarget::Profile)
            .expect("Profile has one exact provider call target");
        assert_eq!(target.role(), "personal");
        assert_eq!(target.zome(), "identity_vault");
        assert_eq!(target.function(), "set_profile_view_if_current");
    }

    #[test]
    fn health_consent_maps_to_exact_append_target() {
        let target = diagnostic_call_target(&PersonalMutationTarget::HealthConsent)
            .expect("Health consent has one exact provider call target");
        assert_eq!(target.role(), "personal");
        assert_eq!(target.zome(), "health_vault");
        assert_eq!(target.function(), "grant_consent_view");
    }

    #[test]
    fn preference_pair_does_not_claim_pair_specific_provider_diagnostics() {
        let target = PersonalMutationTarget::preference("health", "finance");
        assert!(diagnostic_call_target(&target).is_none());
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
