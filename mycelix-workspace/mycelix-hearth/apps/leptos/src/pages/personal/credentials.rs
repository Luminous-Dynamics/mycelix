// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Personal credential surface.
//!
//! This route deliberately fails closed until a source-backed credential/vault
//! loader is wired. It must not infer credentials from profile metadata, trust
//! presentation, Hearth membership, local browser state, or demo fixtures.

use leptos::prelude::*;

const CREDENTIALS_UNAVAILABLE_COPY: &str =
    "No source-backed credential loader is wired for this client yet. Hearth does not infer credentials or verification state from your profile, trust display, membership, or browser-local data.";

#[component]
pub fn CredentialsPage() -> impl IntoView {
    view! {
        <div class="page credentials-page">
            <h1 class="page-title">"credentials"</h1>
            <p class="page-subtitle">"verifiable claims you choose to carry"</p>

            <section aria-labelledby="credentials-source-heading">
                <h2 id="credentials-source-heading">"source status"</h2>
                <div class="availability-state availability-unavailable" role="status">
                    <div class="availability-state-header">
                        <span class="availability-state-icon" aria-hidden="true">"×"</span>
                        <div class="availability-state-copy">
                            <div class="availability-state-meta">
                                <span class="availability-state-title">"Credential records"</span>
                                <span
                                    class="status-pill availability-unavailable"
                                    aria-label="Availability: Unavailable"
                                >
                                    "Unavailable"
                                </span>
                            </div>
                            <p class="availability-state-description">
                                {CREDENTIALS_UNAVAILABLE_COPY}
                            </p>
                        </div>
                    </div>
                </div>
            </section>

            <section aria-labelledby="credentials-boundary-heading">
                <h2 id="credentials-boundary-heading">"authority boundary"</h2>
                <p>
                    "A future credential view must show exact source and verification provenance. An absent loader, an identity profile, or a trust indicator is not evidence that any credential exists or is valid."
                </p>
            </section>
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::CREDENTIALS_UNAVAILABLE_COPY;

    #[test]
    fn unavailable_copy_does_not_claim_credentials_or_verification() {
        let copy = CREDENTIALS_UNAVAILABLE_COPY.to_ascii_lowercase();
        assert!(copy.contains("no source-backed credential loader"));
        assert!(copy.contains("does not infer credentials"));
        assert!(copy.contains("verification state"));
    }
}
