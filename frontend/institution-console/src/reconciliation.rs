use crate::domain::SecurityState;
use crate::ui::SecurityBadge;
use leptos::prelude::*;
use mycelix_finance_types::{ReconciliationCase, ReconciliationCaseStatus};

fn status_label(status: ReconciliationCaseStatus) -> &'static str {
    match status {
        ReconciliationCaseStatus::Open => "Open",
        ReconciliationCaseStatus::Matched => "Matched",
        ReconciliationCaseStatus::TimingDifference => "Timing difference",
        ReconciliationCaseStatus::Resolved => "Resolved",
        ReconciliationCaseStatus::Disputed => "Disputed",
        ReconciliationCaseStatus::Superseded => "Superseded",
    }
}

fn security_state(status: ReconciliationCaseStatus) -> SecurityState {
    match status {
        ReconciliationCaseStatus::Open => SecurityState::Observed,
        ReconciliationCaseStatus::Matched => SecurityState::Reconciled,
        ReconciliationCaseStatus::TimingDifference => SecurityState::Pending,
        ReconciliationCaseStatus::Resolved => SecurityState::Reconciled,
        ReconciliationCaseStatus::Disputed => SecurityState::Disputed,
        ReconciliationCaseStatus::Superseded => SecurityState::Superseded,
    }
}

fn demo_cases() -> Vec<ReconciliationCase> {
    vec![
        ReconciliationCase {
            case_id: "REC-1001".into(),
            scope: "daily-settlement".into(),
            source_a_ref: "processor-batch-2026-10-07-A".into(),
            source_b_ref: "core-journal-batch-2026-10-07-A".into(),
            status: ReconciliationCaseStatus::Open,
            difference_amount_minor_units: 18420,
            currency: "USD".into(),
            opened_at_micros: 1_760_000_000_000_000,
            resolved_at_micros: None,
            resolution_ref: None,
            evidence_refs: vec![
                "processor-observation-77".into(),
                "core-ledger-88".into(),
            ],
        },
        ReconciliationCase {
            case_id: "REC-1002".into(),
            scope: "custody-close",
            source_a_ref: "custodian-statement-447".into(),
            source_b_ref: "general-ledger-447".into(),
            status: ReconciliationCaseStatus::TimingDifference,
            difference_amount_minor_units: 7210,
            currency: "USD".into(),
            opened_at_micros: 1_760_000_100_000_000,
            resolved_at_micros: None,
            resolution_ref: None,
            evidence_refs: vec![
                "custodian-observation-12".into(),
                "ledger-event-99".into(),
                "settlement-calendar-4".into(),
            ],
        },
        ReconciliationCase {
            case_id: "REC-1003".into(),
            scope: "external-settlement",
            source_a_ref: "rail-ethereum-tx-abc".into(),
            source_b_ref: "settlement-journal-abc".into(),
            status: ReconciliationCaseStatus::Disputed,
            difference_amount_minor_units: 92000,
            currency: "USD".into(),
            opened_at_micros: 1_760_000_200_000_000,
            resolved_at_micros: None,
            resolution_ref: None,
            evidence_refs: vec![
                "rail-observation-abc".into(),
                "journal-event-abc".into(),
                "finality-profile-v2".into(),
            ],
        },
        ReconciliationCase {
            case_id: "REC-1004".into(),
            scope: "card-clearing",
            source_a_ref: "network-file-2026-10-06".into(),
            source_b_ref: "core-journal-2026-10-06".into(),
            status: ReconciliationCaseStatus::Resolved,
            difference_amount_minor_units: 0,
            currency: "USD".into(),
            opened_at_micros: 1_759_900_000_000_000,
            resolved_at_micros: Some(1_759_910_000_000_000),
            resolution_ref: Some("resolution-receipt-1004".into()),
            evidence_refs: vec![
                "network-file-hash-1004".into(),
                "journal-close-1004".into(),
                "resolution-evidence-1004".into(),
            ],
        },
    ]
}

#[component]
pub fn ReconciliationPage() -> impl IntoView {
    let cases = demo_cases();
    let selected = RwSignal::new("REC-1001".to_string());

    view! {
        <section class="reconciliation-page">
            <header class="page-header">
                <div>
                    <span class="eyebrow">"RECONCILIATION CONTROL"</span>
                    <h1>"Exceptions and evidence"</h1>
                    <p class="muted">
                        "Synthetic cases only. This workspace demonstrates the read/review boundary; it does not authorize financial resolution."
                    </p>
                </div>
                <div class="control-chip">
                    <SecurityBadge state=SecurityState::Observed/>
                    <span>"Evidence review"</span>
                </div>
            </header>

            <div class="reconciliation-layout">
                <section class="panel case-list" aria-labelledby="case-list-title">
                    <div class="panel-header">
                        <div>
                            <span class="eyebrow">"CASES"</span>
                            <h2 id="case-list-title">"Current queue"</h2>
                        </div>
                        <span class="case-count">{cases.len()} " cases"</span>
                    </div>

                    <div role="list">
                        <For
                            each=move || cases.clone()
                            key=|case| case.case_id.clone()
                            children=move |case| {
                                let case_id = case.case_id.clone();
                                let active = move || selected.get() == case_id;
                                view! {
                                    <button
                                        type="button"
                                        class=move || if active() { "case-row active" } else { "case-row" }
                                        on:click={
                                            let case_id = case.case_id.clone();
                                            move |_| selected.set(case_id.clone())
                                        }
                                    >
                                        <div class="case-row-main">
                                            <strong>{case.case_id.clone()}</strong>
                                            <span>{case.scope.clone()}</span>
                                        </div>
                                        <div class="case-row-meta">
                                            <span>{case.difference_amount_minor_units} " minor units · " {case.currency.clone()}</span>
                                            <span class="mini-status">{status_label(case.status)}</span>
                                        </div>
                                    </button>
                                }
                            }
                        />
                    </div>
                </section>

                <section class="panel case-detail" aria-live="polite">
                    {
                        move || {
                            cases.iter()
                                .find(|case| case.case_id == selected.get())
                                .map(|case| view! {
                                    <CaseDetail case=case.clone()/>
                                }.into_view())
                                .unwrap_or_else(|| view! {
                                    <div class="empty-state">
                                        <h2>"Case unavailable"</h2>
                                        <p>"The selected case is not present in this evidence snapshot."</p>
                                    </div>
                                }.into_view())
                        }
                    }
                </section>
            </div>
        </section>
    }
}

#[component]
fn CaseDetail(case: ReconciliationCase) -> impl IntoView {
    let resolved = matches!(case.status, ReconciliationCaseStatus::Resolved);
    let difference = case.difference_amount_minor_units;

    view! {
        <div>
            <div class="panel-header">
                <div>
                    <span class="eyebrow">"CASE"</span>
                    <h2>{case.case_id.clone()}</h2>
                    <p class="muted">{case.scope.clone()}</p>
                </div>
                <SecurityBadge state=security_state(case.status)/>
            </div>

            <div class="detail-grid">
                <Detail label="Source A" value=case.source_a_ref.clone()/>
                <Detail label="Source B" value=case.source_b_ref.clone()/>
                <Detail label="Currency" value=case.currency.clone()/>
                <Detail label="Difference (minor units)" value=difference.to_string()/>
                <Detail label="Evidence references" value=case.evidence_refs.len().to_string()/>
                <Detail
                    label="Resolution reference"
                    value=case.resolution_ref.clone().unwrap_or_else(|| "None".into())
                />
            </div>

            <div class="evidence-flow">
                <span class="flow-node">"Source A"</span>
                <span class="arrow">"→"</span>
                <span class="flow-node">"Source B"</span>
                <span class="arrow">"→"</span>
                <span class="flow-node">"Difference"</span>
                <span class="arrow">"→"</span>
                <span class="flow-node">"Evidence"</span>
                <span class="arrow">"→"</span>
                <span class="flow-node">{if resolved { "Resolution receipt" } else { "Resolution required" }}</span>
            </div>

            <div class="review-boundary">
                <strong>
                    {if resolved {
                        "Resolved case"
                    } else {
                        "Client cannot resolve this case"
                    }}
                </strong>
                <p class="muted">
                    {if resolved {
                        "A resolution reference is present in the evidence model. This screen only presents it."
                    } else {
                        "Resolution requires a separately authorized control action with evidence, policy and authority context. Selecting a case does not grant that authority."
                    }}
                </p>
                <button class="action" type="button" disabled=true>
                    {if resolved { "Resolution receipt shown above" } else { "Resolution action unavailable in read-only surface" }}
                </button>
            </div>
        </div>
    }
}

#[component]
fn Detail(label: &'static str, value: String) -> impl IntoView {
    view! {
        <div class="detail-cell">
            <span>{label}</span>
            <strong>{value}</strong>
        </div>
    }
}
