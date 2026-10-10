// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use leptos::prelude::*;

#[component]
pub fn TransportPage() -> impl IntoView {
    view! {
        <div class="page transport-page">
            <h1>"Community Transport"</h1>
            <p class="page-desc">"Shared vehicles and routes — coordinated on the mesh."</p>

            <section class="logistics-safety-notice" aria-label="Sample data notice">
                <div class="logistics-notice-mark" aria-hidden="true">"i"</div>
                <div>
                    <strong>"Illustrative sample data — not live availability"</strong>
                    <p>"The vehicles, counts, and availability figures on this page are hard-coded UI examples. Do not use them for operational decisions."</p>
                    <a href="/transport/logistics" class="logistics-entry-link">"Open read-only Logistics workspace preview →"</a>
                </div>
            </section>
            <p class="logistics-fixture-caption">"Illustrative sample metrics"</p>

            <section class="transport-stats">
                <div class="stat-card">
                    <span class="stat-value">"67"</span>
                    <span class="stat-label">"Shared vehicles (sample)"</span>
                </div>
                <div class="stat-card">
                    <span class="stat-value">"12"</span>
                    <span class="stat-label">"Active routes (sample)"</span>
                </div>
                <div class="stat-card">
                    <span class="stat-value">"89%"</span>
                    <span class="stat-label">"Fleet availability (sample)"</span>
                </div>
            </section>

            <section class="vehicle-list">
                <h2>"Illustrative vehicle examples"</h2>
                <div class="vehicle-grid">
                    <div class="vehicle-card">
                        <h3>"Electric Van #3"</h3>
                        <p>"Range: 280km — Battery: 84%"</p>
                        <span class="status-badge good">"Example available"</span>
                    </div>
                    <div class="vehicle-card">
                        <h3>"Cargo Bike #7"</h3>
                        <p>"Max load: 150kg"</p>
                        <span class="status-badge good">"Available"</span>
                    </div>
                    <div class="vehicle-card">
                        <h3>"Shuttle Bus"</h3>
                        <p>"Route: Central — 12 seats"</p>
                        <span class="status-badge in-use">"Example in use"</span>
                    </div>
                </div>
            </section>
        </div>
    }
}
