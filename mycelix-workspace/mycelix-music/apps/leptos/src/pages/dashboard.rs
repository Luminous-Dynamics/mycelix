// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use leptos::prelude::*;
use mycelix_leptos_core::{SovereignRadar, SovereignRadarSize, StatCard};

#[component]
pub fn DashboardPage() -> impl IntoView {
    view! {
        <div class="page dashboard-page">
            <h1>"Artist Dashboard"</h1>

            <div class="dependency-gate" role="status">
                <strong>"Authoritative artist data unavailable"</strong>
                <span>
                    " Dashboard queries and settlement authorization are not wired to the conductor. "
                    "Unknown values are shown as dashes rather than fabricated zeroes."
                </span>
            </div>

            <div class="stats-grid">
                <StatCard label="Total Plays" value="—".to_string() />
                <StatCard label="Total Earnings" value="—".to_string() />
                <StatCard label="Unsettled Plays" value="—".to_string() />
                <StatCard label="Trust Evidence" value="Unavailable".to_string() />
            </div>

            <section class="settlement-section">
                <h2>"Settlement"</h2>
                <p>"Batch your unsettled plays for on-chain settlement."</p>
                <button class="btn btn-primary" disabled=true aria-describedby="settlement-gate">
                    "Settlement unavailable"
                </button>
                <p id="settlement-gate" class="help-text">
                    "No action can be authorized until authoritative unsettled plays and the finance dependency are available."
                </p>
            </section>

            <section class="sovereign-section">
                <h2>"Civic Profile"</h2>
                <SovereignRadar size=SovereignRadarSize::Small />
            </section>

            <section class="verification-section">
                <h2>"Verification"</h2>
                <p>"Build your trust score through community vouches."</p>
                <div class="tier-progress">
                    <div class="tier-bar">
                        <span class="tier-label">"Unverified"</span>
                        <span class="tier-label">"Community"</span>
                        <span class="tier-label">"Trusted"</span>
                        <span class="tier-label">"Platform"</span>
                    </div>
                </div>
            </section>
        </div>
    }
}
