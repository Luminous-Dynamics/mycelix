// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;
use leptos_router::components::A;

/// Farmer-first Today screen prototype.
///
/// All records on this page are illustrative. The local preview interaction is
/// intentionally not connected to Holochain, external services, persistent
/// storage, geolocation, or Symthaea/Sol-Atlas integrations.
#[component]
pub fn FarmerPage() -> impl IntoView {
    let (show_issue_form, set_show_issue_form) = signal(false);
    let (issue_kind, set_issue_kind) = signal("Irrigation or water issue".to_string());
    let (issue_notes, set_issue_notes) = signal(String::new());
    let (draft_created, set_draft_created) = signal(false);
    let (service_step, set_service_step) = signal(0_u8);

    view! {
        <div class="page farmer-page" data-page="farmer">
            <aside class="farmer-prototype-notice" aria-label="Prototype data notice">
                <span class="farmer-notice-mark" aria-hidden="true">"DEMO"</span>
                <div>
                    <strong>"Prototype with sample data"</strong>
                    <p>"Nothing on this page is a live farm record. Draft actions stay in this screen and are not saved or sent."</p>
                </div>
            </aside>

            <header class="farmer-page-header">
                <div class="farmer-heading-copy">
                    <p class="farmer-eyebrow">"FARMER WORKSPACE / TODAY"</p>
                    <h1>"Today, on your farm"</h1>
                    <p class="farmer-lead">"See what needs attention, understand what is known, and choose your next step."</p>
                </div>
                <div class="farmer-sync-card" role="status" aria-live="polite">
                    <span class="farmer-state-symbol" aria-hidden="true">"○"</span>
                    <div>
                        <strong>"Sample workspace — no live farm data"</strong>
                        <p>"Farmer requests and provider receipts are not connected here."</p>
                    </div>
                </div>
            </header>

            <nav class="farmer-quick-actions" aria-label="Farmer quick actions">
                <button
                    type="button"
                    class="farmer-button farmer-button-primary"
                    on:click=move |_| {
                        set_show_issue_form.set(true);
                        set_draft_created.set(false);
                    }
                >
                    <span aria-hidden="true">"＋"</span>
                    " Record a problem"
                </button>
                <A class="farmer-button farmer-button-secondary" href="/food">
                    <span aria-hidden="true">"▧"</span>
                    " Food cooperative"
                </A>
                <A class="farmer-button farmer-button-secondary" href="/water">
                    <span aria-hidden="true">"≈"</span>
                    " Water resources"
                </A>
            </nav>

            <div class="farmer-primary-grid">
                <section class="farmer-panel farmer-attention-panel" aria-labelledby="farmer-attention-title">
                    <div class="farmer-section-heading">
                        <div>
                            <p class="farmer-eyebrow">"DECISIONS"</p>
                            <h2 id="farmer-attention-title">"Needs attention"</h2>
                        </div>
                        <span class="farmer-demo-tag">"Sample"</span>
                    </div>

                    <article class="farmer-attention-item">
                        <div class="farmer-item-topline">
                            <span class="farmer-status-label farmer-status-check">"Check needed"</span>
                            <span class="farmer-time-label">"Example · 07:10"</span>
                        </div>
                        <h3>"Check irrigation on the north plot"</h3>
                        <p>"A sample farmer report describes lower-than-usual water flow. The cause has not been verified."</p>
                        <div class="farmer-evidence-row">
                            <span class="farmer-evidence-label">"Evidence: reported"</span>
                            <span>"No pump reading attached"</span>
                        </div>
                        <div class="farmer-card-actions">
                            <a href="#farmer-conditions" class="farmer-text-action">"Review available information"</a>
                            <button
                                type="button"
                                class="farmer-text-action"
                                on:click=move |_| {
                                    set_show_issue_form.set(true);
                                    set_issue_kind.set("Irrigation or water issue".to_string());
                                    set_draft_created.set(false);
                                }
                            >
                                "Record details"
                            </button>
                        </div>
                    </article>

                    <article class="farmer-attention-item farmer-attention-muted">
                        <div class="farmer-item-topline">
                            <span class="farmer-status-label farmer-status-unknown">"Unknown"</span>
                            <span class="farmer-time-label">"No current reading"</span>
                        </div>
                        <h3>"Soil moisture has not been checked"</h3>
                        <p>"There is not enough current information to recommend an irrigation amount from this sample view."</p>
                        <div class="farmer-evidence-row">
                            <span class="farmer-evidence-label">"Evidence: missing"</span>
                            <span>"Ask an adviser or record a measurement"</span>
                        </div>
                    </article>

                    <div class="farmer-rule-note">
                        <span aria-hidden="true">"ⓘ"</span>
                        <p>"An alert is a reason to check, not an automatic diagnosis or instruction to activate equipment."</p>
                    </div>
                </section>

                <section class="farmer-panel" aria-labelledby="farmer-work-title">
                    <div class="farmer-section-heading">
                        <div>
                            <p class="farmer-eyebrow">"PLAN"</p>
                            <h2 id="farmer-work-title">"Today's work"</h2>
                        </div>
                        <span class="farmer-demo-tag">"Illustrative"</span>
                    </div>

                    <ol class="farmer-task-list">
                        <li class="farmer-task-item">
                            <span class="farmer-task-icon" aria-hidden="true">"01"</span>
                            <div class="farmer-task-copy">
                                <h3>"Inspect the drip line"</h3>
                                <p>"Suggested task · timing not confirmed"</p>
                                <span class="farmer-status-label farmer-status-check">"Not started"</span>
                            </div>
                        </li>
                        <li class="farmer-task-item">
                            <span class="farmer-task-icon" aria-hidden="true">"02"</span>
                            <div class="farmer-task-copy">
                                <h3>"Request pump repair help"</h3>
                                <p>"A request can be drafted here; no provider has been contacted."</p>
                                <span class="farmer-status-label farmer-status-unknown">"Not submitted"</span>
                            </div>
                        </li>
                        <li class="farmer-task-item">
                            <span class="farmer-task-icon" aria-hidden="true">"03"</span>
                            <div class="farmer-task-copy">
                                <h3>"Review water availability"</h3>
                                <p>"Check the source and time of any new reading before relying on it."</p>
                                <span class="farmer-status-label farmer-status-neutral">"When information is available"</span>
                            </div>
                        </li>
                    </ol>

                    <div class="farmer-bottom-note">
                        "A schedule is a plan, not proof that work has happened."
                    </div>
                </section>
            </div>

            <div class="farmer-secondary-grid">
                <section class="farmer-panel" id="farmer-conditions" aria-labelledby="farmer-conditions-title">
                    <div class="farmer-section-heading">
                        <div>
                            <p class="farmer-eyebrow">"FIELD CONTEXT"</p>
                            <h2 id="farmer-conditions-title">"Conditions and evidence"</h2>
                        </div>
                    </div>
                    <div class="farmer-condition-list">
                        <article class="farmer-condition-item">
                            <span class="farmer-condition-icon" aria-hidden="true">"≈"</span>
                            <div class="farmer-condition-copy">
                                <h3>"Water flow"</h3>
                                <p>"Farmer-reported · example only"</p>
                            </div>
                            <span class="farmer-status-label farmer-status-check">"Reported low"</span>
                        </article>
                        <article class="farmer-condition-item">
                            <span class="farmer-condition-icon" aria-hidden="true">"◌"</span>
                            <div class="farmer-condition-copy">
                                <h3>"Soil moisture"</h3>
                                <p>"No measurement supplied"</p>
                            </div>
                            <span class="farmer-status-label farmer-status-unknown">"Unknown"</span>
                        </article>
                        <article class="farmer-condition-item">
                            <span class="farmer-condition-icon" aria-hidden="true">"☁"</span>
                            <div class="farmer-condition-copy">
                                <h3>"Weather"</h3>
                                <p>"No live forecast loaded"</p>
                            </div>
                            <span class="farmer-status-label farmer-status-neutral">"Unavailable"</span>
                        </article>
                    </div>
                    <p class="farmer-source-footnote">"Source and freshness should appear beside every real reading. This prototype does not retrieve weather, sensor, or satellite data."</p>
                </section>

                <section class="farmer-panel" aria-labelledby="farmer-service-title">
                    <div class="farmer-section-heading">
                        <div>
                            <p class="farmer-eyebrow">"SERVICE JOURNEY"</p>
                            <h2 id="farmer-service-title">"From issue to outcome"</h2>
                        </div>
                    </div>
                    <ol class="farmer-service-steps">
                        <li class="farmer-service-step farmer-step-current">
                            <span class="farmer-step-marker" aria-hidden="true">"1"</span>
                            <div><strong>"Problem draft (demo only)"</strong><p>"The form previews draft details; nothing is stored or sent."</p></div>
                        </li>
                        <li class="farmer-service-step">
                            <span class="farmer-step-marker" aria-hidden="true">"2"</span>
                            <div><strong>"Provider receives request"</strong><p>"Not confirmed in this prototype."</p></div>
                        </li>
                        <li class="farmer-service-step">
                            <span class="farmer-step-marker" aria-hidden="true">"3"</span>
                            <div><strong>"Quote and booking"</strong><p>"No quote or booking exists in the demo."</p></div>
                        </li>
                        <li class="farmer-service-step">
                            <span class="farmer-step-marker" aria-hidden="true">"4"</span>
                            <div><strong>"Work reviewed by farmer"</strong><p>"Completion and farmer acceptance must be separate records."</p></div>
                        </li>
                    </ol>

                    <div class="farmer-workflow-demo">
                        <p class="farmer-eyebrow">"INTERACTIVE WALKTHROUGH"</p>
                        <h3>"Practice the service stages"</h3>
                        <p class="farmer-simulation-intro">"Every step below is simulated in this page only. It sends no request and creates no real booking or work receipt."</p>

                        <div class="farmer-simulation-status" role="status" aria-live="polite">
                        <span class="farmer-status-label farmer-status-neutral">
                            {move || match service_step.get() {
                                0 => "Not started",
                                1 => "Draft · demo only",
                                2 => "Submitted · no receipt yet",
                                3 => "Outcome unknown · no automatic resend",
                                4 => "Request receipt · simulated",
                                5 => "Quote received · simulated",
                                6 => "Booking · simulated",
                                7 => "Provider reports completion",
                                8 => "Farmer accepts · simulated",
                                9 => "Dispute / rework · simulated",
                                _ => "Human help selected · demo only",
                            }}
                        </span>
                        <h4>{move || match service_step.get() {
                            0 => "Nothing has been started",
                            1 => "A draft is prepared",
                            2 => "A send attempt is recorded in the simulation",
                            3 => "The request outcome is unknown",
                            4 => "The request is acknowledged",
                            5 => "A quote is available to review",
                            6 => "The booking is confirmed",
                            7 => "The provider reports the work is complete",
                            8 => "The farmer accepts the work",
                            9 => "The farmer disputes the result",
                            _ => "Continue with a person",
                        }}</h4>
                        <p>{move || match service_step.get() {
                            0 => "Start the walkthrough to see how each commitment changes state.",
                            1 => "The draft exists only in this page. Nothing has been saved or sent.",
                            2 => "The demo marks a send attempt, but no provider receipt exists yet. In a real workflow, avoid claiming the request arrived.",
                            3 => "A timeout or lost response does not prove success or failure. The safe state is unknown; do not automatically resend.",
                            4 => "This acknowledgement is simulated. No provider actually received the request.",
                            5 => "Review the full price, scope, expiry and travel terms before accepting. A quote is not a booking.",
                            6 => "This demo booking does not authorize payment, dispatch, or physical action.",
                            7 => "Provider-reported completion is not the same as farmer acceptance. Choose accept or dispute below.",
                            8 => "Acceptance is shown as a distinct decision. No real repair or payment is recorded.",
                            9 => "The demo dispute remains unresolved. It does not decide fault or guilt.",
                            _ => "This only demonstrates the human-assisted option. No person was contacted and no request was sent.",
                        }}</p>
                    </div>

                    {move || match service_step.get() {
                        0 => view! {
                            <button
                                type="button"
                                class="farmer-button farmer-button-primary"
                                on:click=move |_| set_service_step.set(1)
                            >
                                "Start walkthrough"
                            </button>
                        }.into_any(),
                        1 => view! {
                            <button
                                type="button"
                                class="farmer-button farmer-button-primary"
                                on:click=move |_| set_service_step.set(2)
                            >
                                "Simulate durable submission"
                            </button>
                        }.into_any(),
                        2 => view! {
                            <div class="farmer-form-actions">
                                <button
                                    type="button"
                                    class="farmer-button farmer-button-primary"
                                    on:click=move |_| set_service_step.set(4)
                                >
                                    "Simulate request receipt"
                                </button>
                                <button
                                    type="button"
                                    class="farmer-button farmer-button-secondary"
                                    on:click=move |_| set_service_step.set(3)
                                >
                                    "Simulate lost response"
                                </button>
                            </div>
                        }.into_any(),
                        3 => view! {
                            <div class="farmer-walkthrough-choice">
                                <p class="farmer-choice-prompt">"Which next step respects the unknown outcome?"</p>
                                <div class="farmer-form-actions">
                                    <button
                                        type="button"
                                        class="farmer-button farmer-button-primary"
                                        on:click=move |_| set_service_step.set(4)
                                    >
                                        "Simulate reconciliation finds receipt"
                                    </button>
                                    <button
                                        type="button"
                                        class="farmer-button farmer-button-secondary"
                                        on:click=move |_| set_service_step.set(10)
                                    >
                                        "Choose phone / cooperative help"
                                    </button>
                                </div>
                            </div>
                        }.into_any(),
                        4 => view! {
                            <button
                                type="button"
                                class="farmer-button farmer-button-primary"
                                on:click=move |_| set_service_step.set(5)
                            >
                                "Simulate quote received"
                            </button>
                        }.into_any(),
                        5 => view! {
                            <button
                                type="button"
                                class="farmer-button farmer-button-primary"
                                on:click=move |_| set_service_step.set(6)
                            >
                                "Simulate booking confirmation"
                            </button>
                        }.into_any(),
                        6 => view! {
                            <button
                                type="button"
                                class="farmer-button farmer-button-primary"
                                on:click=move |_| set_service_step.set(7)
                            >
                                "Simulate provider completion"
                            </button>
                        }.into_any(),
                        7 => view! {
                            <div class="farmer-walkthrough-choice">
                                <p class="farmer-choice-prompt">"What should happen after the provider reports completion?"</p>
                                <div class="farmer-form-actions">
                                    <button
                                        type="button"
                                        class="farmer-button farmer-button-primary"
                                        on:click=move |_| set_service_step.set(8)
                                    >
                                        "Simulate farmer accepts"
                                    </button>
                                    <button
                                        type="button"
                                        class="farmer-button farmer-button-secondary"
                                        on:click=move |_| set_service_step.set(9)
                                    >
                                        "Simulate dispute / rework"
                                    </button>
                                </div>
                            </div>
                        }.into_any(),
                        _ => view! {
                            <button
                                type="button"
                                class="farmer-button farmer-button-secondary"
                                on:click=move |_| set_service_step.set(0)
                            >
                                "Restart walkthrough"
                            </button>
                        }.into_any(),
                    }}
                    </div>
                </section>
            </div>

            {move || {
                if show_issue_form.get() {
                    view! {
                        <section class="farmer-panel farmer-capture-panel" aria-labelledby="farmer-capture-title">
                            <div class="farmer-section-heading">
                                <div>
                                    <p class="farmer-eyebrow">"DEMO INTERACTION"</p>
                                    <h2 id="farmer-capture-title">"Record a problem"</h2>
                                </div>
                                <button
                                    type="button"
                                    class="farmer-close-button"
                                    aria-label="Close problem form"
                                    on:click=move |_| set_show_issue_form.set(false)
                                >
                                    "Close"
                                </button>
                            </div>
                            <p class="farmer-form-intro">"Describe what you noticed. This preview stays in the current page and is not transmitted or saved to an account."</p>

                            <div class="farmer-form-grid">
                                <label class="farmer-field">
                                    <span>"Problem type"</span>
                                    <select on:change=move |ev| set_issue_kind.set(event_target_value(&ev))>
                                        <option value="Irrigation or water issue" selected>"Irrigation or water issue"</option>
                                        <option value="Pump or equipment problem">"Pump or equipment problem"</option>
                                        <option value="Crop observation">"Crop observation"</option>
                                        <option value="Other">"Other"</option>
                                    </select>
                                </label>
                                <label class="farmer-field">
                                    <span>"What did you observe? (optional)"</span>
                                    <textarea
                                        rows="3"
                                        placeholder="Example: less water reached the north row. I have not checked the filter yet."
                                        on:input=move |ev| set_issue_notes.set(event_target_value(&ev))
                                    ></textarea>
                                </label>
                            </div>

                            <div class="farmer-form-actions">
                                <button
                                    type="button"
                                    class="farmer-button farmer-button-primary"
                                    on:click=move |_| set_draft_created.set(true)
                                >
                                    "Preview demo draft"
                                </button>
                                <button
                                    type="button"
                                    class="farmer-button farmer-button-secondary"
                                    on:click=move |_| {
                                        set_show_issue_form.set(false);
                                        set_draft_created.set(false);
                                        set_issue_notes.set(String::new());
                                    }
                                >
                                    "Cancel"
                                </button>
                            </div>

                            {move || {
                                if draft_created.get() {
                                    view! {
                                        <div class="farmer-draft-preview" role="status" aria-live="polite">
                                            <p class="farmer-eyebrow">"LOCAL PREVIEW ONLY"</p>
                                            <h3>"Demo draft preview"</h3>
                                            <dl>
                                                <div><dt>"Problem type"</dt><dd>{move || issue_kind.get()}</dd></div>
                                                <div><dt>"Observation"</dt><dd>{move || {
                                                    let notes = issue_notes.get();
                                                    if notes.trim().is_empty() { "No extra details added.".to_string() } else { notes }
                                                }}</dd></div>
                                                <div><dt>"Site"</dt><dd>"Not specified"</dd></div>
                                                <div><dt>"Delivery"</dt><dd>"Not sent · no provider notified"</dd></div>
                                            </dl>
                                            <p class="farmer-important-disclaimer">"This is only an on-screen preview. It has not been saved, synced, sent, booked, or added to a real farm record."</p>
                                        </div>
                                    }.into_any()
                                } else {
                                    view! { <></> }.into_any()
                                }
                            }}
                        </section>
                    }.into_any()
                } else {
                    view! { <></> }.into_any()
                }
            }}
        </div>
    }
}
