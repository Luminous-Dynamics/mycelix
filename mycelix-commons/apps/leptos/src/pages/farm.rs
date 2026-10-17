// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Farmer/cooperative UI concept. All records are illustrative fixtures.
//! Interactions in this page remain local; nothing is written to Holochain,
//! synced, or presented as a live farm record.

use leptos::prelude::*;

#[component]
pub fn FarmPage() -> impl IntoView {
    let section = RwSignal::new(0usize);
    let water_checked = RwSignal::new(false);
    let selected_design = RwSignal::new(0usize);

    view! {
        <div class="farm-commons">
            <header class="fc-hero">
                <div class="fc-eyebrow">"MYCELIX COMMONS / FARM WORKSPACE"</div>
                <div class="fc-heading-row">
                    <div>
                        <h1>"Grow together. Keep control."</h1>
                        <p class="fc-hero-copy">"Coordinate farm work, available produce, shared equipment, and cooperative commitments — without turning your farm into public data."</p>
                    </div>
                    <div class="fc-demo-pill"><span class="fc-demo-dot"></span>"LOCAL DEMO"</div>
                </div>
                <div class="fc-trust-note" role="note">
                    <strong>"Illustrative records only."</strong>
                    <span>"This prototype is not connected to a live farm or co-op. Changes stay in this browser session and are not saved or synced."</span>
                </div>
            </header>

            <section class="fc-shell" aria-label="Farm Commons workspace">
                <nav class="fc-tabs" role="group" aria-label="Farm workspace sections">
                    <button class="fc-tab" class:active=move || section.get() == 0 aria-pressed=move || section.get() == 0 on:click=move |_| section.set(0)>"Today"</button>
                    <button class="fc-tab" class:active=move || section.get() == 1 aria-pressed=move || section.get() == 1 on:click=move |_| section.set(1)>"Production & stock"</button>
                    <button class="fc-tab" class:active=move || section.get() == 2 aria-pressed=move || section.get() == 2 on:click=move |_| section.set(2)>"Shared equipment"</button>
                    <button class="fc-tab" class:active=move || section.get() == 3 aria-pressed=move || section.get() == 3 on:click=move |_| section.set(3)>"Co-op & sharing"</button>
                    <button class="fc-tab" class:active=move || section.get() == 4 aria-pressed=move || section.get() == 4 on:click=move |_| section.set(4)>"Open designs"</button>
                </nav>

                {move || match section.get() {
                    0 => view! {
                        <section class="fc-panel" aria-label="Today's farm work">
                            <div class="fc-section-heading">
                                <div>
                                    <div class="fc-eyebrow">"DAILY BRIEF / EXAMPLE"</div>
                                    <h2>"What needs a human today?"</h2>
                                    <p>"One useful list across your work and commitments. No hidden AI priority score."</p>
                                </div>
                                <span class="fc-status-chip">"Demo fixtures"</span>
                            </div>

                            <div class="fc-summary-grid">
                                <article class="fc-summary"><span>"Tasks to review"</span><strong>"02"</strong><small>"Example work queue"</small></article>
                                <article class="fc-summary"><span>"Commitment status"</span><strong>"1 due"</strong><small>"Not a confirmed real order"</small></article>
                                <article class="fc-summary"><span>"Farm data"</span><strong>"Offline"</strong><small>"No live records connected"</small></article>
                            </div>

                            <div class="fc-task-list">
                                <article class="fc-task" class:done=move || water_checked.get()>
                                    <div class="fc-task-symbol">"W"</div>
                                    <div class="fc-task-main">
                                        <div class="fc-task-meta"><span>"FIELD CHECK"</span><span class="fc-tag">"LOW RISK"</span></div>
                                        <h3>"Verify the water line"</h3>
                                        <p>"Walk the line, confirm flow at the trough, and record what you actually observed before moving stock."</p>
                                        <div class="fc-task-bottom">
                                            <span class="fc-evidence-state">{move || if water_checked.get() { "Local demo record · not synced" } else { "Observation not recorded" }}</span>
                                            <button class="fc-primary-button" on:click=move |_| water_checked.set(!water_checked.get())>{move || if water_checked.get() { "Undo demo check" } else { "Record check" }}</button>
                                        </div>
                                    </div>
                                </article>

                                <article class="fc-task">
                                    <div class="fc-task-symbol soil-symbol">"S"</div>
                                    <div class="fc-task-main">
                                        <div class="fc-task-meta"><span>"PRODUCTION"</span><span class="fc-tag muted-tag">"NEEDS INPUT"</span></div>
                                        <h3>"Confirm the actual harvest amount"</h3>
                                        <p>"Keep forecast harvest separate from weighed or counted stock. The example batch has no verified amount yet."</p>
                                        <div class="fc-task-bottom"><span class="fc-evidence-state muted-state">"No quantity observed"</span><button class="fc-secondary-button" on:click=move |_| section.set(1)>"Open production →"</button></div>
                                    </div>
                                </article>

                                <article class="fc-task">
                                    <div class="fc-task-symbol coop-symbol">"C"</div>
                                    <div class="fc-task-main">
                                        <div class="fc-task-meta"><span>"COOPERATIVE"</span><span class="fc-tag muted-tag">"REVIEW"</span></div>
                                        <h3>"Confirm the shared-crate return"</h3>
                                        <p>"A reservation is not proof of return. Record the handoff condition and any missing or damaged crates."</p>
                                        <div class="fc-task-bottom"><span class="fc-evidence-state muted-state">"Example commitment"</span><button class="fc-secondary-button" on:click=move |_| section.set(3)>"Open commitments →"</button></div>
                                    </div>
                                </article>
                            </div>
                            <p class="fc-footnote">"The system should say what is missing and who can resolve it. It should never silently turn an estimate into an observed fact."</p>
                        </section>
                    }.into_any(),
                    1 => view! {
                        <section class="fc-panel" aria-label="Production and stock">
                            <div class="fc-section-heading">
                                <div><div class="fc-eyebrow">"PRODUCTION / TRACEABLE QUANTITIES"</div><h2>"Planned is not available."</h2><p>"Different products need different units. Forecasts, physical stock, reservations, and fulfilled amounts stay separate."</p></div>
                                <span class="fc-status-chip">"All values synthetic"</span>
                            </div>
                            <div class="fc-invariant-banner"><strong>"Invariant"</strong><span>"Offer ≠ stock · Order ≠ fulfillment · Unknown quantity ≠ zero."</span></div>
                            <div class="fc-product-grid">
                                <article class="fc-product-card">
                                    <div class="fc-product-top"><span class="fc-product-icon tomato">"P"</span><span class="fc-state state-observed">"OBSERVED · DEMO"</span></div>
                                    <h3>"Tomatoes"</h3><p>"Example batch T-014"</p><div class="fc-quantity"><strong>"6 crates"</strong><span>"Fictional count"</span></div><div class="fc-product-footer"><span>"Available in demo"</span><span class="fc-state state-observed">"Observed fixture"</span></div>
                                </article>
                                <article class="fc-product-card">
                                    <div class="fc-product-top"><span class="fc-product-icon greens">"L"</span><span class="fc-state state-planned">"PLANNED"</span></div>
                                    <h3>"Leafy greens"</h3><p>"Example batch G-008"</p><div class="fc-quantity"><strong>"12 kg"</strong><span>"Estimated future harvest"</span></div><div class="fc-product-footer"><span>"Not available stock"</span><span class="fc-state state-planned">"Estimate"</span></div>
                                </article>
                                <article class="fc-product-card">
                                    <div class="fc-product-top"><span class="fc-product-icon eggs">"E"</span><span class="fc-state state-unknown">"UNKNOWN"</span></div>
                                    <h3>"Eggs"</h3><p>"Example lot E-003"</p><div class="fc-quantity"><strong>"Not recorded"</strong><span>"Count still needed"</span></div><div class="fc-product-footer"><span>"Cannot create offer yet"</span><span class="fc-state state-unknown">"Missing evidence"</span></div>
                                </article>
                            </div>
                            <div class="fc-detail-panel"><h3>"Before an offer is shared"</h3><ul><li>"Choose a typed unit and price basis."</li><li>"Show source, time, and whether quantity was counted, weighed, estimated, or planned."</li><li>"Choose the recipient and location precision; exact farm boundaries stay private by default."</li><li>"Specify shortage, substitution, and pickup/delivery terms."</li></ul></div>
                            <p class="fc-footnote">"These cards do not represent live inventory, food-safety status, or a real producer commitment."</p>
                        </section>
                    }.into_any(),
                    2 => view! {
                        <section class="fc-panel" aria-label="Shared equipment">
                            <div class="fc-section-heading">
                                <div><div class="fc-eyebrow">"COMMON TOOLS / SERVICE FIRST"</div><h2>"Use what already works."</h2><p>"Compare borrowing, sharing, rental, manual work, repair, and purchase. Ownership is not automatically the best answer."</p></div>
                                <span class="fc-status-chip">"Example catalogue"</span>
                            </div>
                            <div class="fc-equipment-grid">
                                <article class="fc-equipment-card"><div class="fc-equip-icon">"P"</div><div class="fc-equip-status">"AVAILABLE · DEMO"</div><h3>"Solar water pump"</h3><p>"For water movement where head, flow, water quality, and power availability have been verified."</p><dl><div><dt>"Last inspection"</dt><dd>"Not recorded"</dd></div><div><dt>"Compatible fittings"</dt><dd>"Needs checking"</dd></div><div><dt>"Evidence"</dt><dd>"Catalogue fixture"</dd></div></dl><button class="fc-secondary-button" on:click=move |_| section.set(3)>"View sharing terms →"</button></article>
                                <article class="fc-equipment-card"><div class="fc-equip-icon wheel-icon">"H"</div><div class="fc-equip-status">"BOOKING NEEDED"</div><h3>"Wheel hoe"</h3><p>"Manual weed control for an appropriate crop and row spacing. Confirm fit before reserving."</p><dl><div><dt>"Availability"</dt><dd>"Unverified"</dd></div><div><dt>"Repair state"</dt><dd>"Unknown"</dd></div><div><dt>"Evidence"</dt><dd>"Catalogue fixture"</dd></div></dl><button class="fc-secondary-button" on:click=move |_| section.set(3)>"View sharing terms →"</button></article>
                                <article class="fc-equipment-card"><div class="fc-equip-icon seed-icon">"S"</div><div class="fc-equip-status">"MAINTENANCE REVIEW"</div><h3>"Seedling tray seeder"</h3><p>"Shared tools need clear handling, cleaning, missing-part, repair, and return records."</p><dl><div><dt>"Last return"</dt><dd>"Not recorded"</dd></div><div><dt>"Parts list"</dt><dd>"Not attached"</dd></div><div><dt>"Evidence"</dt><dd>"Catalogue fixture"</dd></div></dl><button class="fc-secondary-button" on:click=move |_| section.set(3)>"View sharing terms →"</button></article>
                            </div>
                            <div class="fc-notice"><strong>"Evidence boundary"</strong><span>"A booking does not prove an inspection. An inspection receipt says who checked what, when, and against which checklist; it is not an automatic safety certification."</span></div>
                        </section>
                    }.into_any(),
                    3 => view! {
                        <section class="fc-panel" aria-label="Cooperative commitments and sharing">
                            <div class="fc-section-heading">
                                <div><div class="fc-eyebrow">"CO-OP / AGREEMENTS AND HANDOFFS"</div><h2>"Make the agreement visible."</h2><p>"A commitment records intent. Fulfillment records what actually happened."</p></div>
                                <span class="fc-status-chip">"Example only"</span>
                            </div>
                            <div class="fc-commitment-list">
                                <article class="fc-commitment-card"><div><span class="fc-state state-planned">"RESERVED"</span><h3>"Reusable produce crates"</h3><p>"Example: 12 crates reserved for a collection window. Return count and condition are still unknown."</p></div><div class="fc-commitment-side"><span>"Quantity"</span><strong>"12 crates"</strong><small>"Reservation, not handoff"</small></div></article>
                                <article class="fc-commitment-card"><div><span class="fc-state state-observed">"FULFILLED · DEMO"</span><h3>"Seed exchange handoff"</h3><p>"Example: 4 packets recorded as handed over. Lot details and recipient receipt would be required in a real system."</p></div><div class="fc-commitment-side"><span>"Quantity"</span><strong>"4 packets"</strong><small>"Fictional receipt"</small></div></article>
                                <article class="fc-commitment-card"><div><span class="fc-state state-unknown">"NEEDS REVIEW"</span><h3>"Shared pump maintenance"</h3><p>"Example: a reported leak needs an assigned steward and a repair/update record before next use."</p></div><div class="fc-commitment-side"><span>"Status"</span><strong>"Open"</strong><small>"No action triggered"</small></div></article>
                            </div>
                            <div class="fc-sharing-layout">
                                <article class="fc-detail-panel"><div class="fc-eyebrow">"FARMER DATA RIGHTS"</div><h3>"Private by default"</h3><p>"Exact field boundaries, yields, costs, animal records, worker details, and household order history should not become public just because they exist in the app."</p><ul><li>"Preview every field and location precision before sharing."</li><li>"Name the recipient and purpose."</li><li>"Export your records without asking the platform."</li><li>"Revoking future access does not erase copies already received."</li></ul></article>
                                <article class="fc-detail-panel"><div class="fc-eyebrow">"EVIDENCE STATES"</div><h3>"Make unknown visible"</h3><div class="fc-legend"><span class="fc-state state-observed">"Observed"</span><small>"A person/device recorded a fact with source and time."</small></div><div class="fc-legend"><span class="fc-state state-planned">"Planned"</span><small>"An intention or estimate, not physical inventory."</small></div><div class="fc-legend"><span class="fc-state state-unknown">"Unknown"</span><small>"Evidence is missing, stale, conflicting, or unverified."</small></div></article>
                            </div>
                            <p class="fc-footnote">"No token, reputation score, model output, or simulated result should override consent, physical evidence, or a hard safety constraint."</p>
                        </section>
                    }.into_any(),
                    _ => view! {
                        <section class="fc-panel" aria-label="Open agricultural design library">
                            <div class="fc-section-heading">
                                <div>
                                    <div class="fc-eyebrow">"OPEN HARDWARE / DESIGN-TO-FIELD"</div>
                                    <h2>"A design isn't ready because it renders."</h2>
                                    <p>"Track source, license, manufacturing requirements, bill of materials, tests, repairability, and replicated builds before calling a design build-ready."</p>
                                </div>
                                <span class="fc-status-chip">"Workflow concept"</span>
                            </div>
                            <div class="fc-notice">
                                <strong>"No design here is build-qualified."</strong>
                                <span>"These are illustrative candidate briefs, not released CAD packages, certified equipment, supplier quotes, or recommendations to build or operate a machine."</span>
                            </div>
                            <div class="fc-design-grid">
                                <button class="fc-design-card" class:selected=move || selected_design.get() == 0 aria-pressed=move || selected_design.get() == 0 on:click=move |_| selected_design.set(0)>
                                    <div class="fc-design-top"><span class="fc-equip-icon wheel-icon">"M"</span><span class="fc-state state-planned">"INTAKE ONLY"</span></div>
                                    <h3>"Manual implement improvement"</h3>
                                    <p>"A repairable attachment for a repetitive, low-risk field task. Compare against the current hand tool and existing implements."</p>
                                    <div class="fc-design-meta"><span>"CAD source"</span><strong>"Not reviewed"</strong></div>
                                    <div class="fc-design-meta"><span>"BOM & sourcing"</span><strong>"Not qualified"</strong></div>
                                </button>
                                <button class="fc-design-card" class:selected=move || selected_design.get() == 1 aria-pressed=move || selected_design.get() == 1 on:click=move |_| selected_design.set(1)>
                                    <div class="fc-design-top"><span class="fc-equip-icon">"S"</span><span class="fc-state state-planned">"INTAKE ONLY"</span></div>
                                    <h3>"Low-voltage field logger"</h3>
                                    <p>"A serviceable environmental observation module. Enclosure ingress, sensor calibration, power and environmental limits remain open questions."</p>
                                    <div class="fc-design-meta"><span>"Electrical checks"</span><strong>"Not run"</strong></div>
                                    <div class="fc-design-meta"><span>"Weather test"</span><strong>"Not run"</strong></div>
                                </button>
                                <button class="fc-design-card" class:selected=move || selected_design.get() == 2 aria-pressed=move || selected_design.get() == 2 on:click=move |_| selected_design.set(2)>
                                    <div class="fc-design-top"><span class="fc-equip-icon seed-icon">"R"</span><span class="fc-state state-unknown">"RESEARCH ONLY"</span></div>
                                    <h3>"Non-contact scouting platform"</h3>
                                    <p>"A possible supervised, low-speed observation platform. No spraying, watering, autonomous tool use or physical actuation is in scope."</p>
                                    <div class="fc-design-meta"><span>"Operating domain"</span><strong>"Undefined"</strong></div>
                                    <div class="fc-design-meta"><span>"Safety case"</span><strong>"Missing"</strong></div>
                                </button>
                            </div>
                            <section class="fc-design-detail" aria-label="Selected design qualification checklist">
                                <div class="fc-eyebrow">"SELECTED DESIGN / RELEASE GATE"</div>
                                <h3>{move || match selected_design.get() {
                                    0 => "Manual implement improvement",
                                    1 => "Low-voltage field logger",
                                    _ => "Non-contact scouting platform",
                                }}</h3>
                                <p>{move || match selected_design.get() {
                                    0 => "First candidate class: optimize a defined manual service and measure whether the change helps before adding power or autonomy.",
                                    1 => "Second candidate class: test whether better evidence is worth the cost of the sensor, calibration, upkeep and connectivity.",
                                    _ => "Third candidate class: proceed only when a specific task, operating envelope, human fallback and qualified reviewer are defined.",
                                }}</p>
                                <div class="fc-checklist-grid">
                                    <div class="fc-check-item"><span>"Source revision + license"</span><strong>"Missing"</strong></div>
                                    <div class="fc-check-item"><span>"Editable CAD + drawings"</span><strong>"Missing"</strong></div>
                                    <div class="fc-check-item"><span>"Complete BOM + local quotes"</span><strong>"Missing"</strong></div>
                                    <div class="fc-check-item"><span>"Build procedure + inspection"</span><strong>"Missing"</strong></div>
                                    <div class="fc-check-item"><span>"Bench test + independent checker"</span><strong>"Not run"</strong></div>
                                    <div class="fc-check-item"><span>"Independent reproduction"</span><strong>"Not run"</strong></div>
                                </div>
                                <p class="fc-footnote">"A future release record should pin artifact digests, toolchain/configuration, reviewer disposition and the exact conditions where use is not qualified. No missing item is treated as a pass."</p>
                            </section>
                        </section>
                    }.into_any(),
                }}
            </section>
            <footer class="fc-footer"><span>"Mycelix Commons · farmer-first design concept"</span><span>"No live farm integration · local demo state only"</span></footer>
        </div>
    }
}
