// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FixtureKind {
    Conflict,
    Pending,
    Stale,
    Evidence,
    Alias,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum QueueFilter {
    Attention,
    Conflict,
    Evidence,
    All,
}

/// Small, hand-authored UX fixtures. These are deliberately NOT the frozen S0 corpus
/// and must never be represented as live logistics or authoritative inventory state.
#[derive(Clone, Copy)]
struct OrderFixture {
    id: &'static str,
    participant: &'static str,
    hub: &'static str,
    sku: &'static str,
    /// Explicit per-record source identity, always synthetic in this preview.
    source: &'static str,
    /// Synthetic event/observation time; not wall-clock freshness evidence.
    observed_at: &'static str,
    quantity: u32,
    operational: &'static str,
    evidence: &'static str,
    freshness: &'static str,
    scenario: &'static str,
    next_step: &'static str,
    next_step_reason: &'static str,
    kind: FixtureKind,
}

const HOTSPOT_AVAILABLE_UNITS: u32 = 1;

fn is_evidence_gap(order: &OrderFixture) -> bool {
    matches!(order.kind, FixtureKind::Pending | FixtureKind::Stale | FixtureKind::Evidence)
}

fn needs_attention(order: &OrderFixture) -> bool {
    match order.kind {
        FixtureKind::Conflict
        | FixtureKind::Pending
        | FixtureKind::Stale
        | FixtureKind::Evidence => true,
        FixtureKind::Alias => false,
    }
}

fn conflict_contender_count() -> usize {
    SAMPLE_ORDERS
        .iter()
        .filter(|order| order.kind == FixtureKind::Conflict)
        .count()
}

fn conflict_requested_units() -> u32 {
    SAMPLE_ORDERS
        .iter()
        .filter(|order| order.kind == FixtureKind::Conflict)
        .map(|order| order.quantity)
        .sum()
}

fn matching_order_count(query: &str, filter: QueueFilter) -> usize {
    let query = query.trim();
    SAMPLE_ORDERS
        .iter()
        .filter(|order| order_matches(order, query, filter))
        .count()
}

fn order_matches(order: &OrderFixture, query: &str, filter: QueueFilter) -> bool {
    let matches_filter = match filter {
        QueueFilter::Conflict => order.kind == FixtureKind::Conflict,
        QueueFilter::Evidence => is_evidence_gap(order),
        QueueFilter::Attention => needs_attention(order),
        QueueFilter::All => true,
    };
    let haystack = format!(
        "{} {} {} {} {} {} {} {} {} {} {} {}",
        order.id,
        order.participant,
        order.hub,
        order.sku,
        order.source,
        order.observed_at,
        order.operational,
        order.evidence,
        order.freshness,
        order.scenario,
        order.next_step,
        order.next_step_reason
    )
    .to_lowercase();
    let normalized_query = query.to_lowercase();

    matches_filter && (normalized_query.is_empty() || haystack.contains(normalized_query.as_str()))
}

const SAMPLE_ORDERS: [OrderFixture; 7] = [
    OrderFixture {
        id: "LC-0001",
        participant: "Co-op North",
        hub: "Hub East",
        sku: "SKU-0000",
        source: "Synthetic inventory snapshot INV-HUB-EAST-SKU-0000@rev-7",
        observed_at: "t=100 ms (synthetic)",
        quantity: 1,
        operational: "Unresolved conflict",
        evidence: "Conflicting reservation claims",
        freshness: "Fixture observation t=100 ms; valid through t=200 ms",
        scenario: "Shares a one-unit resource with LC-0002 and LC-0003. All three sample contenders are shown below; none is selected as the winner.",
        next_step: "Defer and request fresh reservation evidence",
        next_step_reason: "Three claims request three units while the fixture snapshot reports one available unit. No winner is selected.",
        kind: FixtureKind::Conflict,
    },
    OrderFixture {
        id: "LC-0002",
        participant: "Co-op River",
        hub: "Hub East",
        sku: "SKU-0000",
        source: "Synthetic inventory snapshot INV-HUB-EAST-SKU-0000@rev-7",
        observed_at: "t=100 ms (synthetic)",
        quantity: 1,
        operational: "Unresolved conflict",
        evidence: "Conflicting reservation claims",
        freshness: "Valid through t=200 ms; one unit available on this snapshot",
        scenario: "Shares a one-unit resource with LC-0001 and LC-0003. All three sample contenders are shown below; none is selected as the winner.",
        next_step: "Defer and request fresh reservation evidence",
        next_step_reason: "Three claims request three units while the fixture snapshot reports one available unit. No winner is selected.",
        kind: FixtureKind::Conflict,
    },
    OrderFixture {
        id: "LC-0003",
        participant: "Community Store",
        hub: "Hub East",
        sku: "SKU-0000",
        source: "Synthetic inventory snapshot INV-HUB-EAST-SKU-0000@rev-7",
        observed_at: "t=100 ms (synthetic)",
        quantity: 1,
        operational: "Unresolved conflict",
        evidence: "Conflicting reservation claims",
        freshness: "Valid through t=200 ms; one unit available on this snapshot",
        scenario: "Shares a one-unit resource with LC-0001 and LC-0002. All three sample contenders are shown below; none is selected as the winner.",
        next_step: "Defer and request fresh reservation evidence",
        next_step_reason: "Three claims request three units while the fixture snapshot reports one available unit. No winner is selected.",
        kind: FixtureKind::Conflict,
    },
    OrderFixture {
        id: "LC-0104",
        participant: "Co-op North",
        hub: "Hub South",
        sku: "SKU-0142",
        source: "Synthetic inventory snapshot INV-HUB-SOUTH-SKU-0142@rev-12",
        observed_at: "t=100 ms (synthetic)",
        quantity: 2,
        operational: "Awaiting authoritative recheck",
        evidence: "Reported claim; no effect receipt",
        freshness: "Fixture observation t=100 ms; valid through t=200 ms",
        scenario: "The offline request fits the observed snapshot, but it is not committed. It must be rechecked against fresh authoritative state.",
        next_step: "Recheck against authoritative inventory",
        next_step_reason: "The claim has no effect receipt; the observation alone does not establish a committed reservation.",
        kind: FixtureKind::Pending,
    },
    OrderFixture {
        id: "LC-0105",
        participant: "Food Network West",
        hub: "Hub West",
        sku: "SKU-0831",
        source: "Synthetic inventory snapshot INV-HUB-WEST-SKU-0831@rev-3",
        observed_at: "t=100 ms (synthetic)",
        quantity: 2,
        operational: "Rejected",
        evidence: "Stale snapshot",
        freshness: "Fixture expired at t=119 ms; reviewed at t=120 ms",
        scenario: "This example is rejected because its observed inventory snapshot is outside its valid window. A stale claim must not be presented as current stock.",
        next_step: "Refresh the inventory snapshot before reconsideration",
        next_step_reason: "The example expired at t=119 ms and was reviewed at t=120 ms, after its validity window.",
        kind: FixtureKind::Stale,
    },
    OrderFixture {
        id: "LC-SHIP-0021",
        participant: "Carrier / recipient handoff",
        hub: "Hub Central",
        sku: "Shipment SH-0021",
        source: "Synthetic carrier delivery report SH-0021-E03",
        observed_at: "t=118 ms (synthetic)",
        quantity: 1,
        operational: "Delivery reported",
        evidence: "Recipient acceptance missing",
        freshness: "Event-time fixture only; no freshness interval or recipient acceptance",
        scenario: "A carrier delivery report exists in this example, but recipient acceptance evidence does not. Delivery reported is not the same as recipient accepted.",
        next_step: "Request recipient acceptance evidence",
        next_step_reason: "A carrier delivery report is not proof that the recipient accepted the shipment.",
        kind: FixtureKind::Evidence,
    },
    OrderFixture {
        id: "LC-0110",
        participant: "Co-op River",
        hub: "Hub North",
        sku: "SKU-0310",
        source: "Synthetic idempotency record ALIAS-0110 -> PRIMARY-0110",
        observed_at: "t=121 ms (synthetic)",
        quantity: 1,
        operational: "Duplicate alias",
        evidence: "Linked to an existing logical request",
        freshness: "Fixed synthetic replay fixture; as of t=121 ms",
        scenario: "This is an idempotent replay example. It refers to a primary request and must not count a second time against inventory.",
        next_step: "Resolve to the primary request and deduplicate",
        next_step_reason: "The alias points to an existing logical request and must not count a second time against inventory.",
        kind: FixtureKind::Alias,
    },
];

#[component]
pub fn LogisticsWorkspacePage() -> impl IntoView {
    let query = RwSignal::new(String::new());
    let active_filter = RwSignal::new(QueueFilter::Attention);

    view! {
        <div class="page logistics-page">
            <div class="logistics-breadcrumb">
                <span>"Commons"</span><span aria-hidden="true">"/"</span>
                <span>"Transport"</span><span aria-hidden="true">"/"</span>
                <strong>"Logistics workspace"</strong>
            </div>

            <header class="logistics-header">
                <div>
                    <p class="logistics-eyebrow">"LOGISTICS COMMONS / OPERATOR WORKSPACE"</p>
                    <h1>"Logistics workspace"</h1>
                    <p class="page-desc">
                        "One queue for contested reservations, stale observations, and incomplete shipment evidence."
                    </p>
                </div>
                <span class="logistics-mode-pill" role="status">
                    <span class="logistics-mode-dot" aria-hidden="true"></span>
                    "SYNTHETIC UX PREVIEW"
                </span>
            </header>

            <section class="logistics-safety-notice" aria-label="Simulation data warning">
                <div class="logistics-notice-mark" aria-hidden="true">"i"</div>
                <div>
                    <strong>"Fixture data only — no live connection"</strong>
                    <p>
                        "These hand-authored examples demonstrate the interface. They are not the frozen S0 corpus, "
                        "not live inventory, and not production logistics. No mutation or booking action is available here."
                    </p>
                </div>
            </section>

            <section class="logistics-summary" aria-label="Preview summary">
                <div class="logistics-metric">
                    <span class="logistics-metric-label">"Fixture records"</span>
                    <strong>{SAMPLE_ORDERS.len().to_string()}</strong>
                    <span class="logistics-metric-note">"UI examples only"</span>
                </div>
                <div class="logistics-metric">
                    <span class="logistics-metric-label">"Conflict contenders"</span>
                    <strong>{conflict_contender_count().to_string()}</strong>
                    <span class="logistics-metric-note">"One sample unit contested"</span>
                </div>
                <div class="logistics-metric">
                    <span class="logistics-metric-label">"Evidence gaps"</span>
                    <strong>{SAMPLE_ORDERS.iter().filter(|order| is_evidence_gap(order)).count().to_string()}</strong>
                    <span class="logistics-metric-note">"Pending, stale, or incomplete"</span>
                </div>
                <div class="logistics-metric logistics-metric-muted">
                    <span class="logistics-metric-label">"Live integrations"</span>
                    <strong>"0"</strong>
                    <span class="logistics-metric-note">"Intentionally unconnected"</span>
                </div>
            </section>

            <section class="logistics-work-queue" aria-labelledby="logistics-queue-title">
                <div class="logistics-section-heading">
                    <div>
                        <h2 id="logistics-queue-title">"Work queue"</h2>
                        <p class="section-subtitle">"Open a record to inspect its state, freshness, and next safe step."</p>
                    </div>
                    <span class="logistics-readonly-label">"Read-only"</span>
                </div>

                <div class="logistics-toolbar">
                    <label class="logistics-search-label" for="logistics-search">"Search the examples"</label>
                    <input
                        id="logistics-search"
                        class="search-input logistics-search"
                        type="search"
                        prop:value=move || query.get()
                        placeholder="ID, participant, hub, SKU, state, evidence, or context"
                        autocomplete="off"
                        on:input=move |ev| query.set(event_target_value(&ev))
                    />
                </div>

                <div class="logistics-filter-bar" role="group" aria-label="Filter work queue">
                    <button
                        type="button"
                        class="logistics-filter"
                        aria-controls="logistics-record-list"
                        class:active=move || active_filter.get() == QueueFilter::Attention
                        aria-pressed=move || active_filter.get() == QueueFilter::Attention
                        on:click=move |_| active_filter.set(QueueFilter::Attention)
                    >"Needs attention" <span class="logistics-filter-count">{move || matching_order_count(&query.get(), QueueFilter::Attention).to_string()}</span></button>
                    <button
                        type="button"
                        class="logistics-filter"
                        aria-controls="logistics-record-list"
                        class:active=move || active_filter.get() == QueueFilter::Conflict
                        aria-pressed=move || active_filter.get() == QueueFilter::Conflict
                        on:click=move |_| active_filter.set(QueueFilter::Conflict)
                    >"Conflicts" <span class="logistics-filter-count">{move || matching_order_count(&query.get(), QueueFilter::Conflict).to_string()}</span></button>
                    <button
                        type="button"
                        class="logistics-filter"
                        aria-controls="logistics-record-list"
                        class:active=move || active_filter.get() == QueueFilter::Evidence
                        aria-pressed=move || active_filter.get() == QueueFilter::Evidence
                        on:click=move |_| active_filter.set(QueueFilter::Evidence)
                    >"Evidence gaps" <span class="logistics-filter-count">{move || matching_order_count(&query.get(), QueueFilter::Evidence).to_string()}</span></button>
                    <button
                        type="button"
                        class="logistics-filter"
                        aria-controls="logistics-record-list"
                        class:active=move || active_filter.get() == QueueFilter::All
                        aria-pressed=move || active_filter.get() == QueueFilter::All
                        on:click=move |_| active_filter.set(QueueFilter::All)
                    >"All examples" <span class="logistics-filter-count">{move || matching_order_count(&query.get(), QueueFilter::All).to_string()}</span></button>
                    <button
                        type="button"
                        class="logistics-filter logistics-clear-filter"
                        aria-controls="logistics-record-list"
                        on:click=move |_| {
                            query.set(String::new());
                            active_filter.set(QueueFilter::Attention);
                        }
                    >"Reset search and filter"</button>
                </div>

                <p class="logistics-results-count" aria-live="polite">
                    {move || {
                        let count = matching_order_count(&query.get(), active_filter.get());
                        format!("{count} matching fixture records")
                    }}
                </p>

                <div id="logistics-record-list" class="logistics-record-list">
                    {move || {
                        let current_query = query.get().trim().to_lowercase();
                        let current_filter = active_filter.get();
                        SAMPLE_ORDERS.iter()
                            .filter(|order| order_matches(order, &current_query, current_filter))
                            .map(|order| {
                            let order = *order;
                            let row_class = if order.kind == FixtureKind::Conflict {
                                "logistics-record conflict-record"
                            } else if order.kind == FixtureKind::Stale || order.kind == FixtureKind::Evidence {
                                "logistics-record attention-record"
                            } else {
                                "logistics-record"
                            };
                            let operational_class = if order.kind == FixtureKind::Conflict {
                                "logistics-state state-conflict"
                            } else if order.kind == FixtureKind::Stale {
                                "logistics-state state-rejected"
                            } else if order.kind == FixtureKind::Evidence {
                                "logistics-state state-incomplete"
                            } else {
                                "logistics-state state-neutral"
                            };
                            view! {
                                <details class=row_class>
                                    <summary class="logistics-record-summary">
                                        <span class="logistics-record-primary">
                                            <strong>{order.id}</strong>
                                            <span>{order.participant}</span>
                                        </span>
                                        <span class="logistics-record-resource">
                                            <strong>{order.sku}</strong>
                                            <span>{order.hub}</span>
                                        </span>
                                        <span class="logistics-record-quantity">
                                            <strong>{format!("{} unit(s)", order.quantity)}</strong>
                                            <span>"Requested quantity"</span>
                                        </span>
                                        <span class=operational_class>
                                            {order.operational}
                                        </span>
                                        <span class="logistics-expand-hint">
                                            <span class="logistics-expand-closed">"Inspect"</span>
                                            <span class="logistics-expand-open">"Hide details"</span>
                                        </span>
                                    </summary>
                                    <div class="logistics-record-detail">
                                        <div class="logistics-detail-grid">
                                            <div class="logistics-detail-cell">
                                                <span class="logistics-detail-label">"Operational state"</span>
                                                <strong>{order.operational}</strong>
                                            </div>
                                            <div class="logistics-detail-cell">
                                                <span class="logistics-detail-label">"Evidence state"</span>
                                                <strong>{order.evidence}</strong>
                                            </div>
                                            <div class="logistics-detail-cell">
                                                <span class="logistics-detail-label">"Source record"</span>
                                                <strong>{order.source}</strong>
                                            </div>
                                            <div class="logistics-detail-cell">
                                                <span class="logistics-detail-label">"Observed at (synthetic time)"</span>
                                                <strong>{order.observed_at}</strong>
                                            </div>
                                            <div class="logistics-detail-cell">
                                                <span class="logistics-detail-label">"Freshness / validity"</span>
                                                <strong>{order.freshness}</strong>
                                            </div>
                                        </div>
                                        <div class="logistics-scenario-note">
                                            <strong>"What this means"</strong>
                                            <p>{order.scenario}</p>
                                        </div>
                                        <section class="logistics-safe-step" aria-label="Suggested next safe step; illustrative only">
                                            <div>
                                                <strong>"Suggested next safe step"</strong>
                                                <p>{order.next_step}</p>
                                            </div>
                                            <div>
                                                <strong>"Why this is safer"</strong>
                                                <p>{order.next_step_reason}</p>
                                            </div>
                                            <span>"Guidance only — no action is executed by this preview."</span>
                                        </section>
                                        {if order.kind == FixtureKind::Conflict {
                                            view! {
                                                <div class="logistics-contender-panel">
                                                    <div class="logistics-contender-heading">
                                                        <strong>"Complete sample contender set"</strong>
                                                        <span>{format!("{} fixture contenders", conflict_contender_count())}</span>
                                                    </div>
                                                    <ul>
                                                        {SAMPLE_ORDERS.iter()
                                                            .filter(|candidate| candidate.kind == FixtureKind::Conflict)
                                                            .map(|candidate| {
                                                                let candidate = *candidate;
                                                                view! {
                                                                    <li>{format!(
                                                                        "{} · {} · {} / {} · quantity {}",
                                                                        candidate.id,
                                                                        candidate.participant,
                                                                        candidate.hub,
                                                                        candidate.sku,
                                                                        candidate.quantity
                                                                    )}</li>
                                                                }
                                                            })
                                                            .collect_view()}
                                                    </ul>
                                                    <p>{format!("The fixture snapshot has {} available unit; these contenders request {} units in total. The safe outcome is unresolved conflict, not a chosen winner.", HOTSPOT_AVAILABLE_UNITS, conflict_requested_units())}</p>
                                                </div>
                                            }.into_any()
                                        } else {
                                            view! { <></> }.into_any()
                                        }}
                                        <div class="logistics-detail-footer">
                                            <span>"No actions enabled in synthetic preview."</span>
                                            <span>"Source: hand-authored UX fixture"</span>
                                        </div>
                                    </div>
                                </details>
                            }
                        }).collect_view()
                    }}
                    <div
                        class="logistics-empty-state"
                        class:visible=move || {
                            let current_query = query.get().trim().to_lowercase();
                            let current_filter = active_filter.get();
                            !SAMPLE_ORDERS.iter().any(|order| {
                                order_matches(order, &current_query, current_filter)
                            })
                        }
                        role="status"
                    >
                        <strong>"No matching fixture records"</strong>
                        <p>"Try another search term or choose a broader filter. No live query was made."</p>
                    </div>
                </div>
            </section>

            <section class="logistics-state-guide" aria-labelledby="logistics-state-guide-title">
                <div>
                    <h2 id="logistics-state-guide-title">"Read the states separately"</h2>
                    <p class="section-subtitle">"A single green badge should never imply that a physical claim is true."</p>
                </div>
                <div class="logistics-state-guide-grid">
                    <div>
                        <strong>"Operational state"</strong>
                        <p>"What the workflow has reached: proposed, awaiting recheck, dispatched, delivery reported, accepted, or unresolved."</p>
                    </div>
                    <div>
                        <strong>"Evidence state"</strong>
                        <p>"What supports the claim: reported, independently checked, incomplete, stale, conflicting, or superseded."</p>
                    </div>
                    <div>
                        <strong>"Connection / freshness"</strong>
                        <p>"Whether data is live, delayed, offline, or a cached observation. This preview has no live connection."</p>
                    </div>
                </div>
            </section>

            <footer class="logistics-preview-footer">
                <span>"Synthetic UX fixture • fixed illustrative data • no external side effects"</span>
                <a href="/transport">"Return to community transport"</a>
            </footer>
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conflict_fixture_contains_all_three_contenders_and_is_oversubscribed() {
        let contenders = SAMPLE_ORDERS
            .iter()
            .filter(|order| order.kind == FixtureKind::Conflict)
            .collect::<Vec<_>>();

        assert_eq!(
            contenders.iter().map(|order| order.id).collect::<Vec<_>>(),
            vec!["LC-0001", "LC-0002", "LC-0003"]
        );
        assert!(
            contenders
                .iter()
                .all(|order| order.hub == "Hub East" && order.sku == "SKU-0000")
        );
        let requested = contenders.iter().map(|order| order.quantity).sum::<u32>();
        assert_eq!(conflict_contender_count(), contenders.len());
        assert_eq!(conflict_requested_units(), requested);
        assert_eq!(HOTSPOT_AVAILABLE_UNITS, 1);
        assert_eq!(requested, 3);
        assert!(requested > HOTSPOT_AVAILABLE_UNITS);
    }

    #[test]
    fn attention_filter_includes_pending_work_and_exceptions_but_not_aliases() {
        let visible = SAMPLE_ORDERS
            .iter()
            .filter(|order| order_matches(order, "", QueueFilter::Attention))
            .map(|order| order.id)
            .collect::<Vec<_>>();

        assert_eq!(
            visible,
            vec![
                "LC-0001",
                "LC-0002",
                "LC-0003",
                "LC-0104",
                "LC-0105",
                "LC-SHIP-0021"
            ]
        );
        assert!(!visible.contains(&"LC-0110"));
    }

    #[test]
    fn evidence_filter_includes_missing_receipt_and_stale_claims() {
        let visible = SAMPLE_ORDERS
            .iter()
            .filter(|order| order_matches(order, "", QueueFilter::Evidence))
            .map(|order| order.id)
            .collect::<Vec<_>>();

        assert_eq!(visible, vec!["LC-0104", "LC-0105", "LC-SHIP-0021"]);
        assert!(visible.contains(&"LC-0104")); // Reported request with no effect receipt.
        assert!(visible.contains(&"LC-0105")); // Stale snapshot.
        assert!(visible.contains(&"LC-SHIP-0021")); // Recipient acceptance missing.
    }

    #[test]
    fn query_is_case_insensitive_and_searches_evidence_freshness_context_and_safe_steps() {
        assert!(order_matches(&SAMPLE_ORDERS[0], "CO-OP NORTH", QueueFilter::All));
        assert!(order_matches(
            &SAMPLE_ORDERS[5],
            "RECIPIENT ACCEPTANCE",
            QueueFilter::All
        ));
        assert!(order_matches(&SAMPLE_ORDERS[0], "VALID THROUGH T=200 MS", QueueFilter::All));
        assert!(order_matches(
            &SAMPLE_ORDERS[0],
            "INV-HUB-EAST-SKU-0000@REV-7",
            QueueFilter::All
        ));
        assert!(order_matches(
            &SAMPLE_ORDERS[5],
            "T=118 MS (SYNTHETIC)",
            QueueFilter::All
        ));
        assert!(order_matches(&SAMPLE_ORDERS[3], "OFFLINE REQUEST", QueueFilter::All));
        assert!(order_matches(
            &SAMPLE_ORDERS[5],
            "REQUEST RECIPIENT ACCEPTANCE EVIDENCE",
            QueueFilter::All
        ));
        assert!(order_matches(
            &SAMPLE_ORDERS[4],
            "REFRESH THE INVENTORY SNAPSHOT",
            QueueFilter::All
        ));
        assert!(!order_matches(&SAMPLE_ORDERS[5], "does-not-exist", QueueFilter::All));
    }

    #[test]
    fn search_and_filter_must_both_match() {
        assert!(order_matches(&SAMPLE_ORDERS[0], "CO-OP NORTH", QueueFilter::Conflict));
        assert!(!order_matches(&SAMPLE_ORDERS[0], "CO-OP NORTH", QueueFilter::Evidence));
        assert!(!order_matches(&SAMPLE_ORDERS[3], "OFFLINE REQUEST", QueueFilter::Conflict));
    }

    #[test]
    fn fixture_ids_are_unique() {
        let ids = SAMPLE_ORDERS
            .iter()
            .map(|order| order.id)
            .collect::<std::collections::HashSet<_>>();

        assert_eq!(ids.len(), SAMPLE_ORDERS.len());
    }

    #[test]
    fn every_fixture_has_explicit_source_observation_and_freshness() {
        for order in &SAMPLE_ORDERS {
            assert!(order.source.starts_with("Synthetic "));
            assert!(order.observed_at.contains("t="));
            assert!(order.observed_at.contains("(synthetic)"));
            assert!(!order.freshness.trim().is_empty());
        }
    }

    #[test]
    fn every_fixture_has_a_safe_step_and_reason() {
        assert!(SAMPLE_ORDERS.iter().all(|order| !order.next_step.trim().is_empty()));
        assert!(SAMPLE_ORDERS
            .iter()
            .all(|order| !order.next_step_reason.trim().is_empty()));
    }

    #[test]
    fn next_safe_steps_respect_authority_and_evidence_boundaries() {
        let contenders = SAMPLE_ORDERS
            .iter()
            .filter(|order| order.kind == FixtureKind::Conflict)
            .collect::<Vec<_>>();
        assert_eq!(conflict_contender_count(), contenders.len());
        assert!(contenders.iter().all(|order| {
            order.next_step == "Defer and request fresh reservation evidence"
                && order.next_step_reason.contains("No winner is selected")
        }));
        assert!(SAMPLE_ORDERS[3].next_step.contains("authoritative inventory"));
        assert!(SAMPLE_ORDERS[3].next_step_reason.contains("effect receipt"));
        assert!(SAMPLE_ORDERS[4].next_step.contains("Refresh"));
        assert!(SAMPLE_ORDERS[5].next_step.contains("recipient acceptance"));
        assert!(SAMPLE_ORDERS[6].next_step.contains("deduplicate"));
    }

    #[test]
    fn queue_counts_stay_consistent_with_search_and_filter_results() {
        assert_eq!(matching_order_count("", QueueFilter::All), SAMPLE_ORDERS.len());
        assert_eq!(matching_order_count("", QueueFilter::Attention), 6);
        assert_eq!(matching_order_count("", QueueFilter::Conflict), 3);
        assert_eq!(matching_order_count("", QueueFilter::Evidence), 3);

        // Counts on filter controls must reflect the same search query as the queue.
        assert_eq!(matching_order_count("SKU-0000", QueueFilter::All), 3);
        assert_eq!(matching_order_count("SKU-0000", QueueFilter::Attention), 3);
        assert_eq!(matching_order_count("SKU-0000", QueueFilter::Conflict), 3);
        assert_eq!(matching_order_count("SKU-0000", QueueFilter::Evidence), 0);

        assert_eq!(matching_order_count("recipient acceptance", QueueFilter::All), 1);
        assert_eq!(matching_order_count("recipient acceptance", QueueFilter::Attention), 1);
        assert_eq!(matching_order_count("recipient acceptance", QueueFilter::Conflict), 0);
        assert_eq!(matching_order_count("recipient acceptance", QueueFilter::Evidence), 1);
        assert_eq!(matching_order_count("no-such-fixture", QueueFilter::All), 0);
    }

    #[test]
    fn fixture_kind_partition_is_complete_and_explicit() {
        let kind_counts = [
            FixtureKind::Conflict,
            FixtureKind::Pending,
            FixtureKind::Stale,
            FixtureKind::Evidence,
            FixtureKind::Alias,
        ]
        .map(|kind| SAMPLE_ORDERS.iter().filter(|order| order.kind == kind).count());

        assert_eq!(kind_counts, [3, 1, 1, 1, 1]);
        assert_eq!(kind_counts.iter().sum::<usize>(), SAMPLE_ORDERS.len());
        assert!(SAMPLE_ORDERS
            .iter()
            .filter(|order| order.kind == FixtureKind::Alias)
            .all(|order| !needs_attention(order)));
    }

    #[test]
    fn all_filter_includes_every_hand_authored_fixture() {
        assert_eq!(
            SAMPLE_ORDERS
                .iter()
                .filter(|order| order_matches(order, "", QueueFilter::All))
                .count(),
            SAMPLE_ORDERS.len()
        );
    }
}
