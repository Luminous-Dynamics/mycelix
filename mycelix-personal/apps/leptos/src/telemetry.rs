// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;
use mycelix_leptos_core::{AvailabilityState, AvailabilityStateKind, TelemetryLine};
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::spawn_local;

use crate::context::{
    retry_full_reconciliation, use_personal, PersonalRetryAdmission, PersonalSnapshotFreshness,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BridgeMetricsSnapshot {
    pub total_success: u64,
    pub total_errors: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConstellationVitals {
    pub identity: Option<BridgeMetricsSnapshot>,
    pub finance: Option<BridgeMetricsSnapshot>,
    pub civic: Option<BridgeMetricsSnapshot>,
    pub knowledge: Option<BridgeMetricsSnapshot>,
    pub symthaea: Option<BridgeMetricsSnapshot>,
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = ["window", "__TAURI__", "event"])]
    async fn listen(event: &str, handler: &js_sys::Function) -> JsValue;
}

fn retained_epoch_label(epoch: Option<u64>) -> String {
    epoch
        .map(|epoch| format!("Personal epoch {epoch}"))
        .unwrap_or_else(|| "no completed Personal epoch".into())
}

#[component]
fn RetryPersonalSnapshot() -> impl IntoView {
    let ctx = use_personal();
    let hc = mycelix_leptos_core::holochain_provider::use_holochain();
    let loading = ctx.loading;
    let retry_note = RwSignal::new(None::<String>);

    let click_ctx = ctx.clone();
    let click_hc = hc.clone();
    let retry = move |_| {
        let ctx = click_ctx.clone();
        let hc = click_hc.clone();
        spawn_local(async move {
            match retry_full_reconciliation(ctx, hc).await {
                PersonalRetryAdmission::Started { epoch } => retry_note.set(Some(format!(
                    "Retry admitted inside Personal epoch {epoch}. Five source groups are staging off-screen."
                ))),
                PersonalRetryAdmission::NoUsableEpoch => retry_note.set(Some(
                    "Retry was not admitted because there is no usable conductor + signer epoch."
                        .into(),
                )),
                PersonalRetryAdmission::Busy { epoch } => retry_note.set(Some(format!(
                    "Personal epoch {epoch} is already reconciling; the duplicate retry was coalesced."
                ))),
            }
        });
    };

    let disabled_hc = hc.clone();
    let readiness_hc = hc;
    view! {
        <div style="display: flex; flex-direction: column; gap: 0.55rem; align-items: flex-start;">
            <button
                class="btn btn-primary"
                type="button"
                disabled=move || loading.get() || !disabled_hc.zome_calls_ready()
                on:click=retry
            >
                {move || if loading.get() { "Reconciling..." } else { "Retry live snapshot" }}
            </button>
            <Show when=move || !readiness_hc.zome_calls_ready()>
                <p class="mini-card-meta" style="margin: 0;">
                    "Retry requires both a connected conductor and an authorized zome-call signer."
                </p>
            </Show>
            <Show when=move || retry_note.get().is_some()>
                <p class="mini-card-meta" style="margin: 0;">
                    {move || retry_note.get().unwrap_or_default()}
                </p>
            </Show>
        </div>
    }
}

#[component]
fn PersonalSnapshotProvenance() -> impl IntoView {
    let ctx = use_personal();

    view! {
        {move || {
            let retained_epoch = retained_epoch_label(ctx.snapshot_epoch.get());
            match ctx.snapshot_freshness.get() {
                PersonalSnapshotFreshness::Illustrative => view! {
                    <AvailabilityState
                        kind=AvailabilityStateKind::Mock
                        title="Illustrative Personal Snapshot"
                        description="Personal is intentionally in Demo mode. These values are fixtures, not conductor-backed evidence."
                        action={None}
                    />
                }.into_any(),
                PersonalSnapshotFreshness::AwaitingLive => view! {
                    <AvailabilityState
                        kind=AvailabilityStateKind::Degraded
                        title="No Completed Live Personal Snapshot"
                        description="No coherent five-source Live snapshot has committed yet. Retry attempts Identity, Wallet, Health, Preferences, and Activity as one frontend publication transaction."
                        action={Some(view! { <RetryPersonalSnapshot /> }.into_any())}
                    />
                }.into_any(),
                PersonalSnapshotFreshness::Refreshing => view! {
                    <AvailabilityState
                        kind=AvailabilityStateKind::Degraded
                        title="Reconciling Personal Snapshot"
                        description=format!(
                            "A coherent five-source snapshot is staging off-screen. Visible values remain tied to {retained_epoch} until every source stage completes and the admitted epoch commits."
                        )
                        action={None}
                    />
                }.into_any(),
                PersonalSnapshotFreshness::Current => view! {
                    <AvailabilityState
                        kind=AvailabilityStateKind::Live
                        title="Current Coherent Personal Snapshot"
                        description=format!(
                            "All visible aggregate Personal source values were published together from {retained_epoch}. Mutation receipts and later source-level refreshes remain separately evidenced."
                        )
                        action={None}
                    />
                }.into_any(),
                PersonalSnapshotFreshness::Stale => view! {
                    <AvailabilityState
                        kind=AvailabilityStateKind::Degraded
                        title="Stale Personal Snapshot"
                        description=format!(
                            "Visible values are retained from {retained_epoch}. They remain useful cached state, but they are not current conductor evidence for the active session."
                        )
                        action={Some(view! { <RetryPersonalSnapshot /> }.into_any())}
                    />
                }.into_any(),
            }
        }}
    }
}

#[component]
pub fn ConstellationTelemetry() -> impl IntoView {
    let cultural = crate::context::use_cultural();
    let symbols = cultural.symbols;

    let identity_history = RwSignal::new(vec![1.0; 10]);
    let finance_history = RwSignal::new(vec![1.0; 10]);
    let civic_history = RwSignal::new(vec![1.0; 10]);
    let knowledge_history = RwSignal::new(vec![1.0; 10]);
    let symthaea_history = RwSignal::new(vec![1.0; 10]);

    // THERMODYNAMIC IGNITION (Vector 2)
    let genesis_pulse = RwSignal::new(false);
    let genesis_text = RwSignal::new(String::new());

    // Use a resource to subscribe to Tauri events
    #[cfg(feature = "hydrate")]
    {
        use wasm_bindgen::closure::Closure;

        spawn_local(async move {
            let callback = Closure::wrap(Box::new(move |event: JsValue| {
                let vitals: ConstellationVitals = serde_wasm_bindgen::from_value(
                    js_sys::Reflect::get(&event, &"payload".into()).unwrap(),
                )
                .unwrap();

                // Detect Thermodynamic Genesis
                if let Some(ref m) = vitals.finance {
                    if m.total_success > 100 {
                        // Threshold for pulse
                        genesis_pulse.set(true);
                        let s = symbols.get_untracked();
                        genesis_text.set(format!(
                            "{}. 10 SAP flowed into {}.",
                            s.genesis_alias, s.hearth_alias
                        ));
                        // [Audio chime would play here]
                        set_timeout(move || genesis_pulse.set(false), 5000);
                    }
                }

                let update_history =
                    |history: RwSignal<Vec<f64>>, metrics: Option<BridgeMetricsSnapshot>| {
                        let mut current = history.get_untracked();
                        current.remove(0);
                        let health = if let Some(m) = metrics {
                            let total = m.total_success + m.total_errors;
                            if total > 0 {
                                m.total_success as f64 / total as f64
                            } else {
                                1.0
                            }
                        } else {
                            0.0 // Substrate offline
                        };
                        current.push(health);
                        history.set(current);
                    };

                update_history(identity_history, vitals.identity);
                update_history(finance_history, vitals.finance);
                update_history(civic_history, vitals.civic);
                update_history(knowledge_history, vitals.knowledge);
                update_history(symthaea_history, vitals.symthaea);
            }) as Box<dyn FnMut(JsValue)>);

            listen("constellation-vitals", callback.as_ref().unchecked_ref()).await;
            callback.forget();
        });
    }

    view! {
        // GOLDEN PULSE OVERLAY (Vector 2)
        <Show when=move || genesis_pulse.get()>
            <div style="position: fixed; inset: 0; pointer-events: none; z-index: 1000;
                        background: radial-gradient(circle, rgba(255,215,0,0.15) 0%, transparent 70%); 
                        animation: pulse-gold 3s ease-out;">
                <div style="position: absolute; bottom: 20%; width: 100%; text-align: center;
                            color: var(--md-signal); font-family: var(--md-mono); font-size: 1.2rem;
                            text-shadow: 0 0 20px rgba(255,215,0,0.5);">
                    {move || genesis_text.get()}
                </div>
            </div>
        </Show>

        <div style="margin-top: 2rem;">
            <PersonalSnapshotProvenance />
        </div>

        <section class="vault-card" style="margin-top: 1rem;">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 1.5rem;">
                <h3 style="margin: 0; font-size: 0.9rem; font-weight: 600; text-transform: uppercase; letter-spacing: 0.05em; color: var(--md-fg-muted);">
                    {move || format!("{} Liveness", symbols.get().hearth_alias)}
                </h3>
                <span style="font-size: 0.75rem; color: var(--md-signal); font-family: var(--md-mono);">
                    {move || symbols.get().orientation.clone()}
                </span>
            </div>

            <div style="display: flex; flex-direction: column; gap: 0.5rem;">
                <TelemetryLine
                    label="Identity"
                    values=identity_history.into()
                    unit="%"
                    min=0.0
                    max=1.0
                />
                <TelemetryLine
                    label="Finance"
                    values=finance_history.into()
                    unit="%"
                    min=0.0
                    max=1.0
                />
                <TelemetryLine
                    label="Civic"
                    values=civic_history.into()
                    unit="%"
                    min=0.0
                    max=1.0
                />
                <TelemetryLine
                    label="Knowledge"
                    values=knowledge_history.into()
                    unit="%"
                    min=0.0
                    max=1.0
                />
                <TelemetryLine
                    label="Symthaea (Moral)"
                    values=symthaea_history.into()
                    unit="%"
                    min=0.0
                    max=1.0
                />
            </div>

            <div style="margin-top: 1.5rem; padding-top: 1rem; border-top: 1px solid var(--md-divider, rgba(255,255,255,0.05));">
                <div style="display: flex; gap: 1.5rem; font-size: 0.7rem; color: var(--md-fg-muted); font-family: var(--md-mono);">
                    <div style="display: flex; align-items: center; gap: 0.4rem;">
                        <div style="width: 6px; height: 6px; border-radius: 50%; background: var(--md-signal);"></div>
                        "Active coordination"
                    </div>
                    <div style="display: flex; align-items: center; gap: 0.4rem;">
                        <div style="width: 6px; height: 6px; border-radius: 50%; background: var(--md-fg-muted); opacity: 0.3;"></div>
                        {move || format!("Standby {}", symbols.get().hearth_alias)}
                    </div>
                </div>
            </div>
        </section>
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retained_epoch_copy_never_invents_an_epoch() {
        assert_eq!(retained_epoch_label(None), "no completed Personal epoch");
        assert_eq!(retained_epoch_label(Some(7)), "Personal epoch 7");
    }
}
