// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;

/// Read-only, explicitly synthetic preview for the Personal life-continuity
/// workspace. This page has no source adapters and never persists or shares data.
fn disclosure_scope_summary(scope: &str) -> &'static str {
    match scope {
        "none" => {
            "No fields selected. A real relying service would receive no vehicle details from this preview."
        },
        "existence" => {
            "Preview: disclose only that a vehicle-service record exists. No owner, identifier, date, location, or service detail is included."
        },
        "selected" => {
            "Preview: disclose selected service categories and their evidence state only. Identifier, owner, location, and full notes stay private."
        },
        "full" => {
            "High exposure: a full record could reveal detailed history. This prototype still sends nothing; a real flow must show every exact field and recipient before consent."
        },
        _ => "No disclosure scope selected.",
    }
}

#[cfg(test)]
mod tests {
    use super::disclosure_scope_summary;

    #[test]
    fn no_disclosure_is_the_minimal_default() {
        assert!(disclosure_scope_summary("none").contains("No fields selected"));
    }

    #[test]
    fn existence_only_excludes_identifying_fields() {
        let preview = disclosure_scope_summary("existence");
        assert!(preview.contains("only that a vehicle-service record exists"));
        assert!(preview.contains("No owner, identifier"));
    }

    #[test]
    fn selected_scope_preserves_private_fields() {
        let preview = disclosure_scope_summary("selected");
        assert!(preview.contains("service categories and their evidence state only"));
        assert!(preview.contains("stay private"));
    }

    #[test]
    fn full_scope_is_explicitly_high_exposure() {
        let preview = disclosure_scope_summary("full");
        assert!(preview.contains("High exposure"));
        assert!(preview.contains("sends nothing"));
    }

    #[test]
    fn unknown_scope_fails_closed() {
        assert_eq!(
            disclosure_scope_summary("unexpected"),
            "No disclosure scope selected."
        );
    }
}

#[component]
pub fn ContinuityPage() -> impl IntoView {
    let disclosure_scope = RwSignal::new("selected".to_string());
    let disclosure_description = move || disclosure_scope_summary(&disclosure_scope.get());

    view! {
        <div class="stack-page continuity-page">
            <header class="page-header">
                <span class="page-eyebrow">Personal / Life continuity</span>
                <h1>"See what matters. Keep control of what is shared."</h1>
                <p>
                    "One private workspace for the important parts of life, with source-aware records and explicit unknowns—not a central database that claims authority over everything."
                </p>
            </header>

            <section class="continuity-demo-banner" role="note" aria-label="Prototype limitation">
                <strong>"Prototype — synthetic examples only"</strong>
                <p>
                    "Nothing here is connected to a government registry, property registry, vehicle authority, insurer, recovery provider, or real personal vault record. This page does not save, publish, or send anything."
                </p>
            </section>

            <section class="continuity-overview" aria-labelledby="continuity-overview-title">
                <div class="continuity-section-heading">
                    <div>
                        <span class="continuity-kicker">"YOUR LIFE INDEX"</span>
                        <h2 id="continuity-overview-title">"Important records, connected by evidence"</h2>
                    </div>
                    <span class="continuity-tag continuity-tag-unknown">"Integration: demo only"</span>
                </div>
                <p class="supporting-copy">
                    "A unified view should link to the rightful source, show when that source was last checked, and let you act—without copying every private document into one place."
                </p>

                <div class="continuity-card-grid">
                    <article class="vault-card continuity-card">
                        <div class="continuity-card-head">
                            <span class="continuity-icon" aria-hidden="true">"ID"</span>
                            <span class="continuity-tag continuity-tag-unknown">"Not verified"</span>
                        </div>
                        <h3>"Identity & credentials"</h3>
                        <p>"Example credential entry"</p>
                        <dl class="continuity-facts">
                            <div><dt>"Issuer / source"</dt><dd>"Synthetic fixture"</dd></div>
                            <div><dt>"Currentness"</dt><dd>"Unknown"</dd></div>
                            <div><dt>"Permission granted"</dt><dd>"None implied"</dd></div>
                        </dl>
                        <p class="continuity-boundary">"Identity proof is not automatic membership, ownership, or authority to act."</p>
                    </article>

                    <article class="vault-card continuity-card">
                        <div class="continuity-card-head">
                            <span class="continuity-icon" aria-hidden="true">"⌂"</span>
                            <span class="continuity-tag continuity-tag-unknown">"Source not connected"</span>
                        </div>
                        <h3>"Property & home"</h3>
                        <p>"Synthetic reference: DEMO-PROPERTY-01"</p>
                        <dl class="continuity-facts">
                            <div><dt>"Title / registry"</dt><dd>"Not checked"</dd></div>
                            <div><dt>"Claim / source conflict"</dt><dd>"Unknown"</dd></div>
                            <div><dt>"Legal conclusion"</dt><dd>"None"</dd></div>
                        </dl>
                        <p class="continuity-boundary">"A document, address, payment, or possession alone must not be promoted to a legal-title verdict."</p>
                    </article>

                    <article class="vault-card continuity-card">
                        <div class="continuity-card-head">
                            <span class="continuity-icon" aria-hidden="true">"↗"</span>
                            <span class="continuity-tag continuity-tag-unknown">"Not checked"</span>
                        </div>
                        <h3>"Vehicle passport"</h3>
                        <p>"Synthetic reference: DEMO-VEHICLE-01"</p>
                        <dl class="continuity-facts">
                            <div><dt>"Registration / title"</dt><dd>"Unknown"</dd></div>
                            <div><dt>"Lien / recall / inspection"</dt><dd>"Not checked"</dd></div>
                            <div><dt>"Service evidence"</dt><dd>"No source connected"</dd></div>
                        </dl>
                        <p class="continuity-boundary">"A passport is a portable view of sourced lifecycle events—not proof of title, insurance, or roadworthiness."</p>
                    </article>

                    <article class="vault-card continuity-card">
                        <div class="continuity-card-head">
                            <span class="continuity-icon" aria-hidden="true">"↻"</span>
                            <span class="continuity-tag continuity-tag-unknown">"Not integrated"</span>
                        </div>
                        <h3>"Recovery & continuity"</h3>
                        <p>"Recovery readiness checklist placeholder"</p>
                        <dl class="continuity-facts">
                            <div><dt>"Recovery options"</dt><dd>"Not assessed"</dd></div>
                            <div><dt>"Last recovery drill"</dt><dd>"No evidence"</dd></div>
                            <div><dt>"Human fallback"</dt><dd>"Needs planning"</dd></div>
                        </dl>
                        <p class="continuity-boundary">"Never place passwords, private keys, seed phrases, or recovery codes in a summary card."</p>
                    </article>
                </div>
            </section>

            <section class="vault-card continuity-disclosure" aria-labelledby="continuity-disclosure-title">
                <div class="continuity-section-heading">
                    <div>
                        <span class="continuity-kicker">"SENSITIVE INFORMATION"</span>
                        <h2 id="continuity-disclosure-title">"Preview a disclosure before it happens"</h2>
                    </div>
                    <span class="continuity-tag continuity-tag-private">"Local rehearsal only"</span>
                </div>
                <p class="supporting-copy">
                    "Select a hypothetical scope. A real disclosure must bind an authenticated recipient, exact fields, purpose, expiry, policy version, and revocation path. Selecting an option here does not issue consent or send data."
                </p>

                <div class="continuity-scope-options" role="group" aria-label="Hypothetical disclosure scope">
                    <button
                        type="button"
                        class="continuity-scope-option"
                        aria-pressed=move || disclosure_scope.get() == "none"
                        on:click=move |_| disclosure_scope.set("none".to_string())
                    >"None"</button>
                    <button
                        type="button"
                        class="continuity-scope-option"
                        aria-pressed=move || disclosure_scope.get() == "existence"
                        on:click=move |_| disclosure_scope.set("existence".to_string())
                    >"Existence only"</button>
                    <button
                        type="button"
                        class="continuity-scope-option"
                        aria-pressed=move || disclosure_scope.get() == "selected"
                        on:click=move |_| disclosure_scope.set("selected".to_string())
                    >"Selected fields"</button>
                    <button
                        type="button"
                        class="continuity-scope-option"
                        aria-pressed=move || disclosure_scope.get() == "full"
                        on:click=move |_| disclosure_scope.set("full".to_string())
                    >"Full record"</button>
                </div>

                <div class="continuity-preview" role="status" aria-live="polite" aria-atomic="true">
                    <strong>"Disclosure preview"</strong>
                    <p>{disclosure_description}</p>
                    <p class="continuity-no-effect">"No real record is shared. No permission is stored. No external request is made."</p>
                </div>
            </section>

            <section class="continuity-benchmarks" aria-labelledby="continuity-benchmarks-title">
                <span class="continuity-kicker">"WHAT GOOD LOOKS LIKE"</span>
                <h2 id="continuity-benchmarks-title">"Non-negotiable guarantees"</h2>
                <div class="continuity-benchmark-grid">
                    <div><strong>"No false certainty"</strong><p>"Unavailable, stale, disputed, empty, and unverified are distinct states."</p></div>
                    <div><strong>"No silent exposure"</strong><p>"Sensitive fields stay private unless a specific, authorized disclosure is approved."</p></div>
                    <div><strong>"Recovery is tested"</strong><p>"A backup is not enough; the owner must be able to rehearse recovery and understand its limits."</p></div>
                    <div><strong>"Authority stays with the source"</strong><p>"Mycelix organizes evidence but does not replace registries, courts, insurers, clinicians, or other legitimate authorities."</p></div>
                </div>
            </section>
        </div>
    }
}
