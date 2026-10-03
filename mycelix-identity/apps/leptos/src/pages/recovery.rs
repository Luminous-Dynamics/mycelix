// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Progressive Recovery — canonical live recovery state.
//!
//! The page renders the recovery configuration actually held by the Identity
//! DNA. It never invents a self-recovery state or hard-code a time lock.

use leptos::prelude::*;
use crate::identity_context::use_identity;
use identity_leptos_types::*;

fn format_time_lock(seconds: u64) -> String {
    if seconds % (24 * 3600) == 0 {
        let days = seconds / (24 * 3600);
        return format!("{days} day{}", if days == 1 { "" } else { "s" });
    }
    if seconds % 3600 == 0 {
        let hours = seconds / 3600;
        return format!("{hours} hour{}", if hours == 1 { "" } else { "s" });
    }
    let hours = seconds / 3600;
    let minutes = (seconds % 3600) / 60;
    if hours > 0 {
        format!("{hours}h {minutes}m")
    } else {
        format!("{minutes} minutes")
    }
}

#[component]
pub fn RecoveryPage() -> impl IntoView {
    let ctx = use_identity();

    let social_config = move || ctx.recovery_config.get();
    let self_config = move || ctx.self_recovery_config.get();

    let recovery_tier = move || {
        if let Some(config) = social_config().filter(|config| config.active) {
            RecoveryTier::SocialRecovery {
                trustee_count: config.trustees.len() as u32,
                threshold: config.threshold,
            }
        } else if let Some(config) = self_config().filter(|config| config.active && !config.anchors.is_empty()) {
            RecoveryTier::SelfRecovery {
                anchor_count: config.anchors.len() as u32,
            }
        } else {
            // A zero-anchor self-recovery config exists from DID creation, but
            // it is not yet actionable. Do not present it as usable protection.
            RecoveryTier::Pending
        }
    };

    view! {
        <div class="page page-recovery">
            <h1>"Recovery"</h1>
            <p class="page-subtitle">
                "Progressive protection — your identity grows safer as recovery proofs are added"
            </p>

            {move || {
                let tier = recovery_tier();
                let strength = tier.strength();
                let pct = (strength * 100.0) as u32;
                let color = tier.css_color();
                view! {
                    <div class="stat-card" style="margin-bottom: var(--space-lg);">
                        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: var(--space-sm);">
                            <span style="font-weight: 600; font-size: var(--text-lg);">"Recovery Strength"</span>
                            <span style=format!("color: {}; font-weight: 700;", color)>{tier.label()}</span>
                        </div>
                        <div style="background: rgba(255,255,255,0.1); border-radius: var(--radius-pill); height: 8px; overflow: hidden;">
                            <div style=format!("width: {}%; height: 100%; background: {}; border-radius: var(--radius-pill); transition: width 0.5s;", pct, color) />
                        </div>
                        <div style="display: flex; justify-content: space-between; margin-top: 4px; font-size: var(--text-xs); opacity: 0.5;">
                            <span>"Self-Recovery"</span>
                            <span>"Social Recovery"</span>
                        </div>
                    </div>
                }
            }}

            // ── Self-Recovery Section ──
            <section style="margin-bottom: var(--space-xl);">
                <h2>"Self-Recovery"</h2>
                <p style="opacity: 0.7; margin-bottom: var(--space-md);">
                    "The Identity DNA creates this configuration when your DID is created. "
                    "Recovery anchors are stored as privacy-preserving identifiers; they are not raw phone numbers, emails, or biometrics."
                </p>

                {move || match self_config() {
                    Some(config) => {
                        let active = config.active;
                        let superseded = config.superseded_by_social;
                        let anchor_count = config.anchors.len();
                        let threshold = config.anchor_threshold;
                        let time_lock = format_time_lock(config.time_lock_secs);
                        view! {
                            <div>
                                <div class="stat-card" style="margin-bottom: var(--space-md);">
                                    <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: var(--space-sm);">
                                        <div>
                                            <strong>{if active { "Self-Recovery Configured" } else { "Self-Recovery Inactive" }}</strong>
                                            <br />
                                            <span style="opacity: 0.7; font-size: var(--text-sm);">
                                                {format!("{anchor_count} anchor{} enrolled; {threshold} required", if anchor_count == 1 { "" } else { "s" }, threshold)}
                                            </span>
                                        </div>
                                        <span>{if superseded { "Fallback" } else { "Active" }}</span>
                                    </div>
                                    <div style="display: flex; justify-content: space-between; margin-top: var(--space-sm); opacity: 0.7; font-size: var(--text-sm);">
                                        <span>"Time Lock"</span>
                                        <strong>{time_lock}</strong>
                                    </div>
                                </div>

                                {if config.anchors.is_empty() {
                                    view! {
                                        <div class="stat-card" style="opacity: 0.7;">
                                            <span>"No recovery anchors enrolled yet."</span>
                                            <br />
                                            <span style="font-size: var(--text-xs);">"Add a passkey, device, email, phone, or biometric anchor to make self-recovery actionable."</span>
                                        </div>
                                    }.into_any()
                                } else {
                                    view! {
                                        <div style="display: grid; grid-template-columns: repeat(auto-fill, minmax(160px, 1fr)); gap: var(--space-sm);">
                                            {config.anchors.iter().map(|anchor| {
                                                let icon = anchor.anchor_type.icon();
                                                let label = anchor.anchor_type.label();
                                                let identifier = anchor.masked_identifier.clone();
                                                view! {
                                                    <div class="stat-card" style="text-align: center;">
                                                        <span style="font-size: 1.5rem; display: block;">{icon}</span>
                                                        <strong style="display: block; margin-top: 4px;">{label}</strong>
                                                        <code style="display: block; margin-top: 4px; font-size: var(--text-xs); opacity: 0.7;">{identifier}</code>
                                                    </div>
                                                }
                                            }).collect::<Vec<_>>()}
                                        </div>
                                    }.into_any()
                                }}
                            </div>
                        }.into_any()
                    }
                    None => view! {
                        <div class="stat-card" style="opacity: 0.7;">
                            <span>"Self-recovery state is not available from the live Identity DNA."</span>
                        </div>
                    }.into_any()
                }}
            </section>

            // ── Social Recovery Section ──
            <section>
                <h2>"Social Recovery"</h2>
                {move || match social_config() {
                    Some(config) if config.active => {
                        let trustee_count = config.trustees.len();
                        let threshold = config.threshold;
                        let time_lock = format_time_lock(config.time_lock_secs);
                        view! {
                            <div>
                                <div class="stat-card" style="margin-bottom: var(--space-md);">
                                    <div style="display: flex; align-items: center; gap: var(--space-md);">
                                        <span style="font-size: 1.5rem;">"✓"</span>
                                        <div>
                                            <strong>"Social Recovery Active"</strong>
                                            <br />
                                            <span style="opacity: 0.8;">{format!("{threshold} of {trustee_count} trustees needed")}</span>
                                        </div>
                                    </div>
                                    <div style="display: flex; justify-content: space-between; margin-top: var(--space-sm); opacity: 0.7; font-size: var(--text-sm);">
                                        <span>"Time Lock"</span>
                                        <strong>{time_lock}</strong>
                                    </div>
                                </div>
                                <div style="display: grid; gap: var(--space-sm);">
                                    {config.trustees.iter().map(|did| {
                                        let short = if did.len() > 30 {
                                            format!("{}...", &did[..30])
                                        } else {
                                            did.clone()
                                        };
                                        view! {
                                            <div class="stat-card" style="display: flex; align-items: center; gap: var(--space-sm);">
                                                <span>"🛡️"</span>
                                                <span style="font-family: var(--font-mono); font-size: var(--text-sm);">{short}</span>
                                            </div>
                                        }
                                    }).collect::<Vec<_>>()}
                                </div>
                            </div>
                        }.into_any()
                    }
                    Some(_) => view! {
                        <div class="stat-card" style="opacity: 0.7;">
                            <span>"Social recovery configuration exists but is inactive."</span>
                        </div>
                    }.into_any(),
                    None => view! {
                        <div class="stat-card" style="opacity: 0.7;">
                            <div style="display: flex; align-items: center; gap: var(--space-md);">
                                <span style="font-size: 1.5rem;">"🌱"</span>
                                <div>
                                    <strong>"Social Recovery Not Configured"</strong>
                                    <br />
                                    <span style="opacity: 0.7;">"A Hearth can provide independent trustees for social recovery."</span>
                                </div>
                            </div>
                        </div>
                    }.into_any(),
                }}
            </section>
        </div>
    }
}
