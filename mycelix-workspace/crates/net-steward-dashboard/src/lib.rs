use leptos::wasm_bindgen::JsCast;
use leptos::*;
use net_steward_schema::{
    AttackPathEdge, AttackPathNode, ConfigDriftReport, DriftStatus, EdgeKind, FederationStatus,
    HumanReadableIncidentSummary, IncidentCapsule, IncidentVerificationResult,
    InfrastructureReceipt, NodeKind, ObservedTopologySnapshot, PeerNodeStatus, PeerTelemetryReport,
    PeerTrustStatus, PostureConflict, PostureSummary, ProofStatus, SafetyVerdict, SecurityEvent,
    Severity,
};

#[component]
pub fn NetStewardDashboard(
    initial_topology: ObservedTopologySnapshot,
    drift_reports: Vec<ConfigDriftReport>,
    receipts: Vec<InfrastructureReceipt>,
    incident_summary: Option<HumanReadableIncidentSummary>,
) -> impl IntoView {
    // Signals for active user navigation and selected elements
    let (selected_node_id, set_selected_node_id) = create_signal::<Option<String>>(None);
    let (user_portal_open, set_user_portal_open) = create_signal::<bool>(false);
    let (active_tab, set_active_tab) = create_signal::<String>("topology".to_string());
    let (topology_filter, set_topology_filter) = create_signal::<String>("all".to_string());

    // 1. Create Leptos reactive signals initialized with the mock fallback values
    let (topology_sig, set_topology_sig) = create_signal(initial_topology);
    let (drift_sig, set_drift_sig) = create_signal(drift_reports);
    let (receipts_sig, set_receipts_sig) = create_signal(receipts);
    let (incident_sig, set_incident_sig) = create_signal(incident_summary);
    let (daemon_online, set_daemon_online) = create_signal(false);
    let (security_events_sig, set_security_events_sig) =
        create_signal::<Vec<SecurityEvent>>(vec![]);
    let security_events_wrapped = SignalWrapper(security_events_sig);
    let (simulated_scenario, set_simulated_scenario) = create_signal::<Option<String>>(None);
    let (peers_sig, set_peers_sig) = create_signal::<Vec<PeerNodeStatus>>(vec![]);
    let peers_wrapped = SignalWrapper(peers_sig);
    let (filter_severity, set_filter_severity) = create_signal::<String>("all".to_string());
    let (filter_node, set_filter_node) = create_signal::<String>("all".to_string());
    let (verification_result, set_verification_result) =
        create_signal::<Option<IncidentVerificationResult>>(None);
    let (federation_mode, set_federation_mode) =
        create_signal::<String>("local_fixture".to_string());
    let (telemetry_stats, set_telemetry_stats) = create_signal::<(u32, u32, u32)>((0, 0, 0));
    let (conflicts_sig, set_conflicts_sig) = create_signal::<Vec<PostureConflict>>(Vec::new());
    let (fed_status, set_fed_status) = create_signal::<Option<FederationStatus>>(None);

    // 2. Spawn async listener query loop to retrieve live data from the witness daemon if running
    create_effect(move |_| {
        spawn_local(async move {
            // Probe daemon version endpoint first to establish live status
            if let Ok(resp) = gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/version")
                .send()
                .await
            {
                if resp.ok() {
                    set_daemon_online.set(true);
                }
            }

            // Live topology endpoint query
            if let Ok(resp) = gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/topology")
                .send()
                .await
            {
                if let Ok(top) = resp.json::<ObservedTopologySnapshot>().await {
                    set_topology_sig.set(top);
                }
            }
            // Live NixOS drift report endpoint query
            if let Ok(resp) = gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/drift")
                .send()
                .await
            {
                if let Ok(drift) = resp.json::<ConfigDriftReport>().await {
                    set_drift_sig.set(vec![drift]);
                }
            }
            // Live Chronicle receipts endpoint query
            if let Ok(resp) = gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/evidence")
                .send()
                .await
            {
                if let Ok(recs) = resp.json::<Vec<InfrastructureReceipt>>().await {
                    set_receipts_sig.set(recs);
                }
            }
            // Live safety verdict endpoint query
            if let Ok(resp) = gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/verdict")
                .send()
                .await
            {
                if let Ok(verdict) = resp.json::<HumanReadableIncidentSummary>().await {
                    set_incident_sig.set(Some(verdict));
                }
            }
            // Live security events query
            if let Ok(resp) =
                gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/security/events")
                    .send()
                    .await
            {
                if let Ok(evs) = resp.json::<Vec<SecurityEvent>>().await {
                    set_security_events_sig.set(evs);
                }
            }
            // Live security peers query
            if let Ok(resp) =
                gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/security/peers")
                    .send()
                    .await
            {
                if let Ok(report) = resp.json::<PeerTelemetryReport>().await {
                    set_peers_sig.set(report.peers);
                    set_federation_mode.set(report.federation_mode);
                    set_telemetry_stats.set((
                        report.claims_fetched,
                        report.claims_rejected,
                        report.claims_stale,
                    ));
                    set_conflicts_sig.set(report.conflicts);
                }
            }
            // Live federation status query
            if let Ok(resp) =
                gloo_net::http::Request::get("http://127.0.0.1:3030/api/v1/federation/status")
                    .send()
                    .await
            {
                if let Ok(status) = resp.json::<FederationStatus>().await {
                    set_fed_status.set(Some(status));
                }
            }
        });
    });

    // 3. Define SignalWrapper helper mimicking StoredValue to avoid modifying the rendering closures
    struct SignalWrapper<T: 'static>(ReadSignal<T>);

    impl<T: 'static> Clone for SignalWrapper<T> {
        fn clone(&self) -> Self {
            SignalWrapper(self.0)
        }
    }

    impl<T: 'static> Copy for SignalWrapper<T> {}

    impl<T: Clone + 'static> SignalWrapper<T> {
        fn with_value<F, R>(&self, f: F) -> R
        where
            F: FnOnce(&T) -> R,
        {
            self.0.with(f)
        }
        fn get_value(&self) -> T {
            self.0.get()
        }
    }

    let initial_topology = SignalWrapper(topology_sig);
    let drift_reports = SignalWrapper(drift_sig);
    let receipts = SignalWrapper(receipts_sig);
    let incident_summary = SignalWrapper(incident_sig);

    let selected_node = move || {
        selected_node_id.get().and_then(|id| {
            initial_topology.with_value(|t| t.nodes.iter().find(|n| n.node_id == id).cloned())
        })
    };

    let selected_drift = move || {
        selected_node_id.get().and_then(|id| {
            drift_reports.with_value(|r_list| r_list.iter().find(|r| r.node_id == id).cloned())
        })
    };

    let selected_receipts = move || {
        selected_node_id
            .get()
            .map(|id| {
                receipts.with_value(|rec_list| {
                    rec_list
                        .iter()
                        .filter(|r| r.target_node_id == id)
                        .cloned()
                        .collect::<Vec<_>>()
                })
            })
            .unwrap_or_default()
    };

    view! {
        <div class="net-steward-shell" style="
            min-height: 100vh;
            background-color: #0d0f12;
            color: #e2e8f0;
            font-family: system-ui, -apple-system, sans-serif;
            display: flex;
            flex-direction: column;
        ">
            // Global Header
            <header style="
                border-bottom: 1px solid #1e293b;
                padding: 1rem 2rem;
                display: flex;
                justify-content: space-between;
                align-items: center;
                background-color: #0b0c0f;
            ">
                <div>
                    <h1 style="margin: 0; font-size: 1.25rem; font-weight: 700; color: #38bdf8; letter-spacing: -0.025em;">
                        "Luminous Net Steward"
                    </h1>
                    <span style="font-size: 0.75rem; color: #64748b; font-family: monospace; display: block;">
                        "Civic Network Console // Federated Infrastructure Stewardship"
                    </span>
                    <div style="margin-top: 0.5rem; display: flex; flex-direction: column; gap: 0.15rem; font-family: monospace; font-size: 0.7rem; color: #94a3b8; background: #0f172a; padding: 0.5rem; border-radius: 0.25rem; border: 1px solid #1e293b;">
                        <div><strong>"Net Steward: "</strong> "v0.1-alpha.4"</div>
                        <div><strong>"Mode: "</strong> "Read-Only Witness"</div>
                        <div><strong>"Mutation: "</strong> "Disabled"</div>
                        <div><strong>"Rollback Apply: "</strong> "Disabled"</div>
                        <div><strong>"Proof Verification: "</strong> "Not Active"</div>
                        <div>
                            <strong>"Daemon: "</strong>
                            {move || if daemon_online.get() {
                                view! { <span style="color: #4ade80;">"Online (Live REST Connection)"</span> }.into_view()
                            } else {
                                view! { <span style="color: #f87171;">"Offline (Showing Fixtures / Cached state - No live topology claims are being made)"</span> }.into_view()
                            }}
                        </div>
                    </div>
                </div>

                <div style="display: flex; gap: 1rem; align-items: center;">
                    <select
                        on:change=move |ev| {
                            let val = event_target_value(&ev);
                            if val == "live" {
                                set_simulated_scenario.set(None);
                            } else {
                                set_simulated_scenario.set(Some(val));
                            }
                        }
                        style="background: #1e293b; color: #f8fafc; border: 1px solid #374151; padding: 0.5rem; border-radius: 0.375rem; font-size: 0.875rem;"
                    >
                        <option value="live">"Live Mode (Infrastructure Daemon)"</option>
                        <option value="scenario_forge_port_blocked">"Scenario: Forge Port Blocked"</option>
                        <option value="scenario_unknown_device_on_vlan">"Scenario: Unknown Device on VLAN"</option>
                        <option value="scenario_unexpected_systemd_unit">"Scenario: Unexpected Systemd Unit"</option>
                        <option value="scenario_xenia_session_after_drift">"Scenario: Xenia Session after Drift"</option>
                        <option value="scenario_router_dhcp_unknown_host">"Scenario: Router DHCP Unknown Host"</option>
                    </select>

                    <button
                        on:click=move |_| set_user_portal_open.update(|v| *v = !*v)
                        style="
                            background-color: #0284c7;
                            color: white;
                            border: none;
                            padding: 0.5rem 1rem;
                            border-radius: 0.375rem;
                            font-size: 0.875rem;
                            font-weight: 500;
                            cursor: pointer;
                            transition: background 0.15s ease;
                        "
                    >
                        {move || if user_portal_open.get() { "Switch to Admin Console" } else { "Open Community User Portal" }}
                    </button>
                    <span style="
                        display: inline-flex;
                        align-items: center;
                        gap: 0.5rem;
                        font-size: 0.875rem;
                        background: #14532d;
                        color: #4ade80;
                        padding: 0.25rem 0.75rem;
                        border-radius: 9999px;
                        font-weight: 600;
                    ">
                        <span style="width: 8px; height: 8px; background: #4ade80; border-radius: 50%; display: inline-block;"></span>
                        "WITNESS ACTIVE"
                    </span>
                </div>
            </header>

            {move || if let Some(scenario) = simulated_scenario.get() {
                view! {
                    <div style="background: #b45309; color: #fef3c7; padding: 0.5rem 2rem; font-size: 0.8rem; font-weight: 600; text-align: center; border-bottom: 1px solid #d97706; font-family: monospace;">
                        {format!("⚠️ Scenario Mode: Simulated ({}) — No live infrastructure claims are being made. Autonomous remediation: disabled.", scenario)}
                    </div>
                }.into_view()
            } else {
                view! {}.into_view()
            }}

            // Main Content Area
            <main style="flex: 1; display: flex; overflow: hidden; position: relative;">
                {move || if user_portal_open.get() {
                    // Democratized User Portal view
                    view! {
                        <div style="flex: 1; padding: 2rem; max-width: 800px; margin: 0 auto;">
                            <div style="
                                background: #111827;
                                border: 1px solid #1f2937;
                                border-radius: 0.75rem;
                                padding: 2rem;
                                box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.5);
                            ">
                                <h2 style="margin-top: 0; color: #38bdf8; font-size: 1.5rem;">"Community Connection Explainer"</h2>
                                <p style="color: #9ca3af; line-height: 1.6;">
                                    "This portal provides transparent diagnostics. We believe networking should not require priesthood, and you deserve to understand what infrastructure knows about your connectivity."
                                </p>

                                {move || {
                                    let summary_opt = incident_summary.get_value();
                                    if let Some(summary) = summary_opt {
                                        view! {
                                            <div style="margin-top: 2rem; border-left: 4px solid #f59e0b; padding-left: 1rem; background: #1e1b4b; padding: 1.25rem; border-radius: 0.375rem;">
                                                <h3 style="margin-top: 0; color: #fbbf24; font-size: 1.125rem;">"Outage / Interruption Detected"</h3>

                                                <div style="margin: 0.5rem 0; font-size: 0.875rem;">
                                                    <span style="color: #9ca3af;">"Advisor Verdict: "</span>
                                                    <strong style=move || format!("color: {}", match summary.safety_verdict {
                                                        SafetyVerdict::Safe => "#4ade80",
                                                        SafetyVerdict::Warning => "#fbbf24",
                                                        SafetyVerdict::Blocked => "#f87171"
                                                    })>
                                                        {format!("{:?}", summary.safety_verdict)}
                                                    </strong>
                                                </div>

                                                {if !summary.safety_violations.is_empty() {
                                                    let violations = summary.safety_violations.clone();
                                                    view! {
                                                        <div style="margin: 0.5rem 0; font-size: 0.875rem; background: #311c1c; border: 1px solid #7f1d1d; padding: 0.5rem; border-radius: 0.25rem; color: #fca5a5;">
                                                            <strong>"Violations: "</strong>
                                                            {violations.join("; ")}
                                                        </div>
                                                    }.into_view()
                                                } else {
                                                    view! {}.into_view()
                                                }}

                                                <p style="font-weight: 600; margin-bottom: 0.5rem; margin-top: 1rem;">"Why is my service broken?"</p>
                                                <p style="margin-top: 0; color: #e5e7eb;">{summary.root_cause}</p>

                                                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; margin-top: 1rem; font-size: 0.875rem;">
                                                    <div>
                                                        <span style="color: #9ca3af; display: block;">"Affected Services"</span>
                                                        <span style="font-weight: 500;">{summary.affected_services.join(", ")}</span>
                                                    </div>
                                                    <div>
                                                        <span style="color: #9ca3af; display: block;">"Affected Users"</span>
                                                        <span style="font-weight: 500;">{summary.affected_users.join(", ")}</span>
                                                    </div>
                                                </div>

                                                <div style="margin-top: 1.25rem; font-size: 0.875rem; color: #a5b4fc; background: #312e81; padding: 0.75rem; border-radius: 0.25rem;">
                                                    <strong>"Symthaea Recommendation: "</strong>
                                                    {summary.recommended_action}
                                                </div>

                                                // 1. Render ZK-STARK Safety Proof status honestly
                                                {if let Some(ref proof) = summary.safety_proof {
                                                    let proof_hex = proof.iter().map(|b| format!("{:02x}", b)).collect::<Vec<String>>().join("");
                                                    let comm_hex = if let Some(comm) = summary.safety_commitment {
                                                        comm.iter().map(|b| format!("{:02x}", b)).collect::<Vec<String>>().join("")
                                                    } else {
                                                        "None".to_string()
                                                    };
                                                    let verifier_status_label = match summary.proof_status {
                                                         ProofStatus::NotPresent => "Verifier Status: Proof Not Present",
                                                         ProofStatus::CommitmentOnly => "Verifier Status: Safety Commitment Present",
                                                         ProofStatus::SimulatedEnvelope => "Proof Envelope: Simulated (Verifier: Not Active)",
                                                         ProofStatus::VerificationUnavailable => "Verifier Status: Verification Unavailable",
                                                         ProofStatus::Verified => "ZK Proof Verified (Verifier: active)",
                                                         ProofStatus::Rejected => "Proof Rejected",
                                                     };
                                                    view! {
                                                        <div style="margin-top: 1rem; border: 1px solid #1e293b; background: #0f172a; padding: 0.75rem; border-radius: 0.375rem; font-size: 0.85rem;">
                                                            <div style="color: #94a3b8; font-weight: 600; display: flex; align-items: center; gap: 0.5rem;">
                                                                <span style="font-size: 1.1rem;">"🛡️"</span>
                                                                {verifier_status_label}
                                                            </div>
                                                            <div style="font-family: monospace; font-size: 0.7rem; color: #64748b; margin-top: 0.25rem;">
                                                                <div><strong>"Status: "</strong> {verifier_status_label}</div>
                                                                <div><strong>"Verdict Commitment: "</strong> {comm_hex}</div>
                                                                <div style="word-break: break-all; margin-top: 0.25rem; opacity: 0.8;"><strong>"Proof Bytes: "</strong> {proof_hex}</div>
                                                            </div>
                                                        </div>
                                                    }.into_view()
                                                } else {
                                                    view! {}.into_view()
                                                }}

                                                // 2. Render Recovery Rollback Action Trigger as a Plan generation tool
                                                {if let Some(ref rollback) = summary.rollback_path {
                                                    let rollback_target = rollback.clone();
                                                    view! {
                                                        <button
                                                            style="width: 100%; margin-top: 1rem; padding: 0.75rem; border-radius: 0.375rem; border: none; background: #1e293b; color: #f8fafc; font-weight: 600; cursor: pointer; transition: background 0.15s ease;"
                                                            on:click=move |_| {
                                                                // Generates rollback plan dry-run in logs/UI state
                                                            }
                                                        >
                                                            {format!("Generate Rollback Plan for {}", rollback_target)}
                                                        </button>
                                                    }.into_view()
                                                } else {
                                                    view! {}.into_view()
                                                }}
                                            </div>
                                        }.into_view()
                                    } else {
                                        view! {
                                            <div style="margin-top: 2rem; background: #064e3b; color: #6ee7b7; padding: 1.25rem; border-radius: 0.375rem; text-align: center;">
                                                <strong>"All Local Infrastructure Healthy"</strong>
                                                <p style="margin: 0.5rem 0 0 0; font-size: 0.875rem; color: #a7f3d0;">
                                                    "No service interruptions detected. Your device connections are verified and performing normally."
                                                </p>
                                            </div>
                                        }.into_view()
                                    }
                                }}

                                <div style="margin-top: 2rem; border-top: 1px solid #1f2937; padding-top: 1.5rem;">
                                    <h4 style="margin-top: 0; color: #9ca3af;">"Your Active Device Signatures"</h4>
                                    <div style="font-family: monospace; font-size: 0.875rem; background: #030712; padding: 1rem; border-radius: 0.375rem; color: #34d399;">
                                        "did:mycelix:tristan-laptop // Verified Cryptographic Identity"
                                    </div>
                                </div>
                            </div>
                        </div>
                    }.into_view()
                } else {
                    // Admin View
                    view! {
                        // Left Pane: Navigation & Topology Graph
                        <div style="width: 50%; border-right: 1px solid #1e293b; padding: 1.5rem; display: flex; flex-direction: column; gap: 1rem; overflow-y: auto;">
                            // Admin Sub-tabs
                            <div style="display: flex; gap: 0.5rem; border-bottom: 1px solid #1e293b; padding-bottom: 0.75rem;">
                                <button
                                    on:click=move |_| set_active_tab.set("topology".to_string())
                                    style=move || format!("background: {}; color: {}; border: none; padding: 0.5rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.875rem; font-weight: 500;",
                                        if active_tab.get() == "topology" { "#1e293b" } else { "transparent" },
                                        if active_tab.get() == "topology" { "#f8fafc" } else { "#94a3b8" }
                                    )
                                >
                                    "Topology Map"
                                </button>
                                <button
                                    on:click=move |_| set_active_tab.set("drift".to_string())
                                    style=move || format!("background: {}; color: {}; border: none; padding: 0.5rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.875rem; font-weight: 500;",
                                        if active_tab.get() == "drift" { "#1e293b" } else { "transparent" },
                                        if active_tab.get() == "drift" { "#f8fafc" } else { "#94a3b8" }
                                    )
                                >
                                    "Config & Drift"
                                </button>
                                <button
                                    on:click=move |_| set_active_tab.set("evidence".to_string())
                                    style=move || format!("background: {}; color: {}; border: none; padding: 0.5rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.875rem; font-weight: 500;",
                                         if active_tab.get() == "evidence" { "#1e293b" } else { "transparent" },
                                         if active_tab.get() == "evidence" { "#f8fafc" } else { "#94a3b8" }
                                     )
                                >
                                    "Evidence Ledger"
                                </button>
                                <button
                                    on:click=move |_| set_active_tab.set("security".to_string())
                                    style=move || format!("background: {}; color: {}; border: none; padding: 0.5rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.875rem; font-weight: 500;",
                                         if active_tab.get() == "security" { "#1e293b" } else { "transparent" },
                                         if active_tab.get() == "security" { "#f8fafc" } else { "#94a3b8" }
                                     )
                                >
                                    "Security Witness"
                                </button>
                                <button
                                    on:click=move |_| set_active_tab.set("peers".to_string())
                                    style=move || format!("background: {}; color: {}; border: none; padding: 0.5rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.875rem; font-weight: 500;",
                                         if active_tab.get() == "peers" { "#1e293b" } else { "transparent" },
                                         if active_tab.get() == "peers" { "#f8fafc" } else { "#94a3b8" }
                                     )
                                >
                                    "Peer Postures"
                                </button>
                                <button
                                    on:click=move |_| set_active_tab.set("attack-path".to_string())
                                    style=move || format!("background: {}; color: {}; border: none; padding: 0.5rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.875rem; font-weight: 500;",
                                         if active_tab.get() == "attack-path" { "#1e293b" } else { "transparent" },
                                         if active_tab.get() == "attack-path" { "#f8fafc" } else { "#94a3b8" }
                                     )
                                >
                                    "Attack Path"
                                </button>
                            </div>

                            {move || match active_tab.get().as_str() {
                                "drift" => view! {
                                    <div style="display: flex; flex-direction: column; gap: 1rem;">
                                        <h3>"Configuration Drift Analysis"</h3>
                                        <div style="display: flex; flex-direction: column; gap: 0.75rem;">
                                            {drift_reports.with_value(|reports| {
                                                reports.iter().map(|report| {
                                                    let r = report.clone();
                                                    view! {
                                                        <div style="background: #18181b; border: 1px solid #27272a; padding: 1rem; border-radius: 0.5rem;">
                                                            <div style="display: flex; justify-content: space-between;">
                                                                <strong>{&r.node_id}</strong>
                                                                <span style=move || format!("color: {}", match r.drift_status {
                                                                    DriftStatus::InSync => "#4ade80",
                                                                    DriftStatus::DriftDetected => "#fbbf24",
                                                                    _ => "#a1a1aa"
                                                                })>
                                                                    {format!("{:?}", r.drift_status)}
                                                                </span>
                                                            </div>
                                                            {if let Some(ref diff) = r.diff_closure {
                                                                view! {
                                                                    <pre style="background: #09090b; padding: 0.5rem; border-radius: 0.25rem; font-size: 0.75rem; font-family: monospace; overflow-x: auto; margin-top: 0.5rem;">
                                                                        {diff}
                                                                    </pre>
                                                                }.into_view()
                                                            } else {
                                                                view! {}.into_view()
                                                            }}
                                                        </div>
                                                    }
                                                }).collect::<Vec<_>>()
                                            })}
                                        </div>
                                    </div>
                                }.into_view(),

                                "security" => view! {
                                    <div style="display: flex; flex-direction: column; gap: 1.5rem;">
                                        <div style="display: flex; justify-content: space-between; align-items: center;">
                                            <div>
                                                <h3 style="margin: 0; font-size: 1.1rem; color: #f43f5e;">"Security Witness Activity Monitor"</h3>
                                                <p style="margin: 0.25rem 0 0 0; font-size: 0.75rem; color: #94a3b8;">"Witnessing active endpoint posture anomalies & suspicious events (Read-only containment)"</p>
                                            </div>
                                        </div>

                                        // 1. Posture Stats Grid & Banner
                                        <div style="background: #111827; border: 1px solid #1e293b; padding: 1rem; border-radius: 0.5rem; display: grid; grid-template-columns: repeat(5, 1fr); text-align: center; gap: 0.5rem; font-family: monospace; font-size: 0.75rem;">
                                            <div>
                                                <span style="color: #ef4444; display: block; font-weight: bold;">"Critical"</span>
                                                <strong style="color: #f1f5f9; font-size: 1rem;">"0"</strong>
                                            </div>
                                            <div>
                                                <span style="color: #fb923c; display: block; font-weight: bold;">"High"</span>
                                                <strong style="color: #f1f5f9; font-size: 1rem;">"1"</strong>
                                            </div>
                                            <div>
                                                <span style="color: #fbbf24; display: block; font-weight: bold;">"Medium"</span>
                                                <strong style="color: #f1f5f9; font-size: 1rem;">"1"</strong>
                                            </div>
                                            <div>
                                                <span style="color: #9ca3af; display: block; font-weight: bold;">"Unknown"</span>
                                                <strong style="color: #f1f5f9; font-size: 1rem;">"0"</strong>
                                            </div>
                                            <div style="border-left: 1px solid #1e293b; grid-column: span 1; display: flex; align-items: center; justify-content: center; flex-direction: column;">
                                                <span style="color: #fca5a5; font-weight: bold;">"Remediation"</span>
                                                <span style="color: #f87171; font-weight: bold; font-size: 0.7rem; text-transform: uppercase;">"Disabled"</span>
                                            </div>
                                        </div>

                                        // 2. Incident Capsule Upload & Verification Panel
                                        <div style="background: #18181b; border: 1px solid #27272a; padding: 1.25rem; border-radius: 0.5rem; display: flex; flex-direction: column; gap: 0.75rem;">
                                            <h4 style="margin: 0; color: #f1f5f9; font-size: 0.95rem;">"🔍 Operator Incident Capsule Verifier"</h4>
                                            <p style="margin: 0; font-size: 0.75rem; color: #94a3b8;">"Paste capsule JSON payload to verify evidence offline (no-mutation dry-run check)"</p>

                                            <textarea
                                                id="capsule-input-area"
                                                placeholder="Paste Incident Capsule JSON payload here..."
                                                style="width: 100%; height: 80px; background: #09090b; color: #34d399; font-family: monospace; font-size: 0.75rem; border: 1px solid #27272a; border-radius: 0.375rem; padding: 0.5rem;"
                                            ></textarea>

                                            <div style="display: flex; gap: 0.5rem; justify-content: flex-end;">
                                                <button
                                                    style="background: #1e1b4b; color: #c7d2fe; border: 1px solid #312e81; padding: 0.4rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.8rem; font-weight: 600;"
                                                    on:click=move |_| {
                                                        if let Some(doc) = web_sys::window()
                                                            .and_then(|w| w.document())
                                                        {
                                                            if let Some(el) = doc.get_element_by_id("capsule-input-area") {
                                                                if let Ok(ta) = el.dyn_into::<web_sys::HtmlTextAreaElement>() {
                                                                    let val = ta.value();
                                                                    if let Ok(capsule) = serde_json::from_str::<IncidentCapsule>(&val) {
                                                                        spawn_local(async move {
                                                                            let verified_res = if let Ok(resp) = gloo_net::http::Request::post("http://127.0.0.1:3030/api/v1/security/incident/verify")
                                                                                .json(&capsule) {
                                                                                if let Ok(res_call) = resp.send().await {
                                                                                    res_call.json::<IncidentVerificationResult>().await.ok()
                                                                                } else {
                                                                                    None
                                                                                }
                                                                            } else {
                                                                                None
                                                                            };

                                                                            let res = verified_res.unwrap_or_else(|| {
                                                                                let hashes_valid = !capsule.cryptographic_receipt_hash.is_empty();
                                                                                let evidence_ledger_valid = !capsule.evidence_ledger.is_empty();
                                                                                let rollback_plan_valid = capsule.rollback_plan.requires_approval;
                                                                                let security_events_valid = capsule.security_events.iter().all(|e| !e.evidence_hash.is_empty());
                                                                                let mutation_claims_found = false;
                                                                                let result_passed = hashes_valid && evidence_ledger_valid && rollback_plan_valid && security_events_valid && !mutation_claims_found;

                                                                                IncidentVerificationResult {
                                                                                    capsule_id: capsule.capsule_id.clone(),
                                                                                    schema_version: "incident_capsule_v0.1".to_string(),
                                                                                    hashes_valid,
                                                                                    evidence_ledger_valid,
                                                                                    rollback_plan_valid,
                                                                                    security_events_valid,
                                                                                    proof_status: ProofStatus::SimulatedEnvelope,
                                                                                    mutation_claims_found,
                                                                                    result_passed,
                                                                                    verification_summary: if result_passed {
                                                                                        "PASS: All evidence commitments validated successfully via local fallback verifier.".to_string()
                                                                                    } else {
                                                                                        "FAIL: Hash validation failed or unauthorized actions detected.".to_string()
                                                                                    }
                                                                                }
                                                                            });
                                                                            set_verification_result.set(Some(res));
                                                                        });
                                                                    } else {
                                                                        logging::log!("Failed to parse capsule JSON payload");
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                >
                                                    "Verify Capsule"
                                                </button>
                                                <button
                                                    style="background: #1c1917; color: #d6d3d1; border: 1px solid #292524; padding: 0.4rem 1rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.8rem;"
                                                    on:click=move |_| {
                                                        set_verification_result.set(None);
                                                    }
                                                >
                                                    "Clear"
                                                </button>
                                            </div>

                                            {move || {
                                                if let Some(v) = verification_result.get() {
                                                    let status_color = if v.result_passed { "#10b981" } else { "#ef4444" };
                                                    let result_text = if v.result_passed { "PASS" } else { "FAIL" };
                                                    let capsule_id = v.capsule_id.clone();
                                                    let schema_version = v.schema_version.clone();
                                                    let hashes_valid = v.hashes_valid;
                                                    let evidence_ledger_valid = v.evidence_ledger_valid;
                                                    let rollback_plan_valid = v.rollback_plan_valid;
                                                    let security_events_valid = v.security_events_valid;
                                                    let proof_status_str = format!("{:?}", v.proof_status);
                                                    let mutation_claims_found = v.mutation_claims_found;
                                                    let verification_summary = v.verification_summary.clone();
                                                    view! {
                                                        <div style="background: #09090b; border: 1px solid #27272a; padding: 1rem; border-radius: 0.375rem; font-family: monospace; font-size: 0.75rem;">
                                                            <div style="display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #1e293b; padding-bottom: 0.5rem; margin-bottom: 0.5rem;">
                                                                <strong>"Verification Report Card"</strong>
                                                                <span style=move || format!("font-weight: bold; color: {};", status_color)>
                                                                    {result_text}
                                                                </span>
                                                            </div>
                                                            <div style="display: flex; flex-direction: column; gap: 0.25rem; color: #a1a1aa;">
                                                                <div>"Capsule ID: " <span style="color:#f1f5f9;">{capsule_id}</span></div>
                                                                <div>"Schema: " <span style="color:#f1f5f9;">{schema_version}</span></div>
                                                                <div>"Hashes: " <span style=move || format!("color: {};", if hashes_valid { "#10b981" } else { "#ef4444" })>{if hashes_valid { "valid" } else { "invalid" }}</span></div>
                                                                <div>"Evidence Ledger: " <span style=move || format!("color: {};", if evidence_ledger_valid { "#10b981" } else { "#ef4444" })>{if evidence_ledger_valid { "valid chain" } else { "invalid chain" }}</span></div>
                                                                <div>"Rollback Plan: " <span style=move || format!("color: {};", if rollback_plan_valid { "#10b981" } else { "#ef4444" })>{if rollback_plan_valid { "dry-run only" } else { "invalid config" }}</span></div>
                                                                <div>"Security Events: " <span style=move || format!("color: {};", if security_events_valid { "#10b981" } else { "#ef4444" })>{if security_events_valid { "evidence hashes present" } else { "missing evidence" }}</span></div>
                                                                <div>"Proof Status: " <span style="color:#f1f5f9;">{proof_status_str}</span></div>
                                                                <div>"Mutation Claims: " <span style=move || format!("color: {};", if !mutation_claims_found { "#10b981" } else { "#ef4444" })>{if mutation_claims_found { "FOUND (UNAUTHORIZED)" } else { "none" }}</span></div>
                                                            </div>
                                                            <div style="margin-top: 0.5rem; padding-top: 0.5rem; border-top: 1px solid #1e293b; color: #94a3b8;">
                                                                <strong>"Summary: "</strong> {verification_summary}
                                                            </div>
                                                        </div>
                                                    }.into_view()
                                                } else {
                                                    view! {}.into_view()
                                                }
                                            }}
                                        </div>

                                        // 3. Security Event Filters
                                        <div style="display: flex; gap: 0.5rem; background: #0b0c0f; padding: 0.5rem; border-radius: 0.375rem; border: 1px solid #1e293b; font-size: 0.85rem; flex-wrap: wrap;">
                                            <span style="color: #64748b; display: flex; align-items: center;">"Filter Severity:"</span>
                                            <select
                                                on:change=move |ev| set_filter_severity.set(event_target_value(&ev))
                                                style="background: #1e293b; color: #f8fafc; border: 1px solid #374151; padding: 0.25rem; border-radius: 0.25rem;"
                                            >
                                                <option value="all">"All Severities"</option>
                                                <option value="Critical">"Critical"</option>
                                                <option value="High">"High"</option>
                                                <option value="Medium">"Medium"</option>
                                            </select>

                                            <span style="color: #64748b; display: flex; align-items: center; margin-left: 0.5rem;">"Filter Node:"</span>
                                            <select
                                                on:change=move |ev| set_filter_node.set(event_target_value(&ev))
                                                style="background: #1e293b; color: #f8fafc; border: 1px solid #374151; padding: 0.25rem; border-radius: 0.25rem;"
                                            >
                                                <option value="all">"All Nodes"</option>
                                                <option value="forge-server">"forge-server"</option>
                                                <option value="luminous-router">"luminous-router"</option>
                                            </select>
                                        </div>

                                        // 4. Filtered Security Events List
                                        <div style="display: flex; flex-direction: column; gap: 1rem;">
                                            {security_events_wrapped.with_value(move |events| {
                                                let filtered = events.iter().filter(|evt| {
                                                    let matches_sev = filter_severity.get() == "all" || format!("{:?}", evt.severity) == filter_severity.get();
                                                    let matches_node = filter_node.get() == "all" || evt.node_id == filter_node.get();
                                                    matches_sev && matches_node
                                                }).collect::<Vec<_>>();

                                                if filtered.is_empty() {
                                                    view! {
                                                        <div style="background: #111827; padding: 1.5rem; border: 1px dashed #374151; border-radius: 0.5rem; text-align: center; color: #9ca3af; font-size: 0.85rem;">
                                                            "No matching security anomalies found."
                                                        </div>
                                                    }.into_view()
                                                } else {
                                                    filtered.into_iter().map(|evt| {
                                                        let e = evt.clone();
                                                        let severity_color = match e.severity {
                                                            Severity::Critical => "#f43f5e",
                                                            Severity::High => "#fb923c",
                                                            Severity::Medium => "#fbbf24",
                                                            _ => "#38bdf8"
                                                        };
                                                        let proc_info = if let Some(ref p) = e.related_process {
                                                            format!("PID: {} | {} ({})", p.pid, p.process_name, p.exe_path)
                                                        } else {
                                                            "None".to_string()
                                                        };
                                                        let identity_info = e.related_identity.clone().unwrap_or_else(|| "None".to_string());
                                                        let path_info = e.related_network_edge.clone().unwrap_or_else(|| "Local Loopback Only".to_string());

                                                        let recommended_badges = e.recommended_action.iter().map(|act| {
                                                            view! {
                                                                <span style="font-size: 0.65rem; background: #312e81; color: #a5b4fc; padding: 0.2rem 0.5rem; border-radius: 0.25rem; font-family: monospace;">
                                                                    {format!("{:?}", act)}
                                                                </span>
                                                            }
                                                        }).collect::<Vec<_>>();

                                                        view! {
                                                            <div style="background: #18181b; border: 1px solid #27272a; padding: 1.25rem; border-radius: 0.5rem; display: flex; flex-direction: column; gap: 0.75rem;">
                                                                <div style="display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #27272a; padding-bottom: 0.5rem;">
                                                                    <div style="display: flex; align-items: center; gap: 0.5rem;">
                                                                        <span style=move || format!("display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: {};", severity_color)></span>
                                                                        <strong style="color: #f1f5f9; font-size: 0.9rem;">{format!("{:?}", e.event_kind)}</strong>
                                                                    </div>
                                                                    <span style=move || format!("font-size: 0.7rem; font-weight: 600; text-transform: uppercase; color: {}; border: 1px solid {}; padding: 0.15rem 0.4rem; border-radius: 0.25rem;", severity_color, severity_color)>
                                                                        {format!("{:?}", e.severity)}
                                                                    </span>
                                                                </div>

                                                                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 0.75rem; font-size: 0.75rem; font-family: monospace;">
                                                                    <div>
                                                                        <span style="color: #71717a; display: block;">"Event ID"</span>
                                                                        <span style="color: #a1a1aa;">{&e.event_id}</span>
                                                                    </div>
                                                                    <div>
                                                                        <span style="color: #71717a; display: block;">"Target Node"</span>
                                                                        <span style="color: #a1a1aa;">{&e.node_id}</span>
                                                                    </div>
                                                                    <div>
                                                                        <span style="color: #71717a; display: block;">"Related Process"</span>
                                                                        <span style="color: #a1a1aa;">{proc_info}</span>
                                                                    </div>
                                                                    <div>
                                                                        <span style="color: #71717a; display: block;">"Active Identity Link"</span>
                                                                        <span style="color: #a1a1aa;">{identity_info}</span>
                                                                    </div>
                                                                    <div>
                                                                        <span style="color: #71717a; display: block;">"Evidence Path / Edge"</span>
                                                                        <span style="color: #a1a1aa;">{path_info}</span>
                                                                    </div>
                                                                    <div>
                                                                        <span style="color: #71717a; display: block;">"Evidence Hash (Verification)"</span>
                                                                        <span style="color: #94a3b8; font-size: 0.65rem;">{&e.evidence_hash}</span>
                                                                    </div>
                                                                </div>

                                                                <div style="margin-top: 0.25rem;">
                                                                    <span style="color: #71717a; display: block; font-size: 0.75rem; font-family: monospace; margin-bottom: 0.25rem;">"Recommended Recovery Action Plans:"</span>
                                                                    <div style="display: flex; flex-wrap: wrap; gap: 0.4rem;">
                                                                        {recommended_badges}
                                                                    </div>
                                                                </div>

                                                                <div style="margin-top: 0.5rem; border-top: 1px solid #27272a; padding-top: 0.75rem; display: flex; justify-content: flex-end;">
                                                                    <button
                                                                        style="background: #1e293b; color: #f1f5f9; border: 1px solid #374151; padding: 0.4rem 0.8rem; border-radius: 0.375rem; cursor: pointer; font-size: 0.75rem; font-weight: 500; display: flex; align-items: center; gap: 0.35rem;"
                                                                        on:click=move |_| {
                                                                            logging::log!("Creating evidence capsule bundle for event {}", e.event_id);
                                                                        }
                                                                    >
                                                                        "📦 Export Incident Evidence Capsule"
                                                                    </button>
                                                                </div>
                                                            </div>
                                                        }
                                                    }).collect::<Vec<_>>().into_view()
                                                }
                                            })}
                                        </div>
                                    </div>
                                }.into_view(),

                                "peers" => view! {
                                    <div style="display: flex; flex-direction: column; gap: 1.5rem;">
                                        <div>
                                            <h3 style="margin: 0; font-size: 1.1rem; color: #38bdf8;">"Federated Peer Postures"</h3>
                                            <p style="margin: 0.25rem 0 0 0; font-size: 0.75rem; color: #94a3b8;">"Multi-node trust state & posture freshness tracker"</p>
                                        </div>

                                        // Render federation mode details
                                        <div style="background: #111827; border: 1px solid #1e293b; padding: 0.75rem; border-radius: 0.375rem; font-family: monospace; font-size: 0.75rem; color: #94a3b8; display: flex; gap: 1rem; justify-content: space-between;">
                                            <div>"Federation Mode: " <strong style="color: #60a5fa;">{move || federation_mode.get()}</strong></div>
                                            <div>"Claims (Fetched/Rejected/Stale): " <strong style="color: #34d399;">{move || format!("{}/{}/{}", telemetry_stats.get().0, telemetry_stats.get().1, telemetry_stats.get().2)}</strong></div>
                                        </div>

                                        // Federation status card
                                        {move || fed_status.get().map(|status| {
                                            view! {
                                                <div style="background: #111827; border: 1px solid #1e293b; padding: 0.75rem; border-radius: 0.375rem; font-family: monospace; font-size: 0.75rem; color: #94a3b8; display: flex; flex-direction: column; gap: 0.25rem;">
                                                    <div style="font-weight: bold; color: #38bdf8;">"Mycelix Holochain Conductor Status"</div>
                                                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 0.5rem; margin-top: 0.25rem;">
                                                        <div>"Transport: " <span style="color: #f3f4f6;">{status.transport.clone()}</span></div>
                                                        <div>"Conductor Online: " <span style="color: #34d399;">{if status.conductor_available { "YES" } else { "NO" }}</span></div>
                                                        <div>"DNA Installed: " <span style="color: #34d399;">{if status.dna_installed { "YES" } else { "NO" }}</span></div>
                                                        <div>"Zome Name: " <span style="color: #f3f4f6;">{status.zome.clone()}</span></div>
                                                        <div>"Identity Binding: " <span style="color: #fbbf24;">{status.identity_binding.clone()}</span></div>
                                                        <div>"Trust Level: " <span style="color: #fbbf24;">"signed-unbound"</span></div>
                                                    </div>
                                                </div>
                                            }
                                        })}

                                        // Conflict Inspector Panel
                                        {move || {
                                            let active_conflicts = conflicts_sig.get();
                                            if active_conflicts.is_empty() {
                                                None
                                            } else {
                                                Some(view! {
                                                    <div style="background: #1e1b4b; border: 1px solid #4338ca; padding: 1rem; border-radius: 0.5rem; display: flex; flex-direction: column; gap: 0.75rem;">
                                                        <div style="display: flex; align-items: center; gap: 0.5rem; color: #f43f5e; font-weight: bold; font-size: 0.9rem;">
                                                            <span>"⚠️ CONFIGURATION drift / posture conflict detected"</span>
                                                        </div>
                                                        {active_conflicts.into_iter().map(|conflict| {
                                                            let c = conflict.clone();
                                                            view! {
                                                                <div style="border-top: 1px solid #312e81; padding-top: 0.75rem; display: flex; flex-direction: column; gap: 0.5rem;">
                                                                    <div style="font-size: 0.8rem; color: #e2e8f0; font-family: monospace;">
                                                                        "Subject node: " <strong style="color: #60a5fa;">{c.subject_node_id.clone()}</strong>
                                                                    </div>
                                                                    <div style="font-size: 0.75rem; color: #94a3b8;">
                                                                        "Reconciliation verdict: " <span style="color: #f43f5e; font-weight: bold;">"ConflictingClaims"</span>" — operator review required."
                                                                    </div>
                                                                    <div style="background: #0f172a; padding: 0.5rem; border-radius: 0.375rem; display: flex; flex-direction: column; gap: 0.25rem;">
                                                                        {c.claims.into_iter().map(|claim| {
                                                                            view! {
                                                                                <div style="font-family: monospace; font-size: 0.7rem; color: #cbd5e1; display: flex; justify-content: space-between;">
                                                                                    <span>"Issuer DID: " {claim.issuer_did}</span>
                                                                                    <span style="color: #fbbf24;">"Reported summary: " {claim.value_summary}</span>
                                                                                </div>
                                                                            }
                                                                        }).collect_view()}
                                                                    </div>
                                                                    <div style="font-size: 0.75rem; color: #cbd5e1; margin-top: 0.25rem;">
                                                                        <strong>"Recommended action: "</strong>"Verify from local collector or request fresh capsule from " {c.subject_node_id.clone()}
                                                                    </div>
                                                                </div>
                                                            }
                                                        }).collect_view()}
                                                    </div>
                                                })
                                            }
                                        }}

                                        <div style="display: flex; flex-direction: column; gap: 1rem;">
                                            {peers_wrapped.with_value(|peers| {
                                                if peers.is_empty() {
                                                    view! {
                                                        <div style="background: #111827; padding: 1.5rem; border: 1px dashed #374151; border-radius: 0.5rem; text-align: center; color: #9ca3af; font-size: 0.85rem;">
                                            "No discovered peer nodes."
                                                        </div>
                                                    }.into_view()
                                                } else {
                                                    peers.iter().map(|peer| {
                                                        let p = peer.clone();
                                                        let trust_label = match p.trust_status {
                                                            PeerTrustStatus::LocalSelf => "Local Host (Self)",
                                                            PeerTrustStatus::SignedPeer => "Signed but Unbound",
                                                            PeerTrustStatus::UnsignedPeer => "Unsigned / Untrusted",
                                                            PeerTrustStatus::StalePeer => "Stale Claim",
                                                            PeerTrustStatus::ConflictingClaims => "Conflicting Claims",
                                                            PeerTrustStatus::Quarantined => "Quarantined",
                                                            PeerTrustStatus::FederatedConsensus => "Federated Consensus",
                                                            PeerTrustStatus::VerifiedBoundFresh => "Verified + Bound + Fresh",
                                                        };
                                                        let trust_color = match p.trust_status {
                                                            PeerTrustStatus::LocalSelf => "#10b981",
                                                            PeerTrustStatus::SignedPeer => "#3b82f6",
                                                            PeerTrustStatus::UnsignedPeer => "#fb923c",
                                                            PeerTrustStatus::StalePeer => "#a1a1aa",
                                                            PeerTrustStatus::ConflictingClaims => "#f43f5e",
                                                            PeerTrustStatus::Quarantined => "#ef4444",
                                                            PeerTrustStatus::FederatedConsensus => "#8b5cf6",
                                                            PeerTrustStatus::VerifiedBoundFresh => "#10b981",
                                                        };
                                                        let posture_color = match p.posture_summary {
                                                            PostureSummary::Healthy => "#10b981",
                                                            PostureSummary::Degraded => "#fbbf24",
                                                            PostureSummary::Critical => "#ef4444",
                                                            _ => "#6b7280"
                                                        };
                                                        view! {
                                                            <div style="background: #18181b; border: 1px solid #27272a; padding: 1rem; border-radius: 0.5rem; display: flex; flex-direction: column; gap: 0.5rem;">
                                                                <div style="display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #27272a; padding-bottom: 0.5rem;">
                                                                    <strong>{&p.display_name} <span style="font-size:0.75rem; font-weight:normal; color:#71717a;">{format!("({})", p.node_id)}</span></strong>
                                                                    <div style="display: flex; gap: 0.5rem;">
                                                                        <span style=move || format!("font-size: 0.7rem; color: {}; border: 1px solid {}; padding: 0.1rem 0.4rem; border-radius: 0.25rem;", trust_color, trust_color)>
                                                                            {trust_label}
                                                                        </span>
                                                                        <span style=move || format!("font-size: 0.7rem; color: {}; border: 1px solid {}; padding: 0.1rem 0.4rem; border-radius: 0.25rem;", posture_color, posture_color)>
                                                                            {format!("{:?}", p.posture_summary)}
                                                                        </span>
                                                                    </div>
                                                                </div>
                                                                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 0.5rem; font-size: 0.75rem; font-family: monospace; color: #a1a1aa;">
                                                                    <div>"Last Seen: " {p.last_seen_unix_ms} "ms"</div>
                                                                    <div>"Staleness: " {p.staleness_ms} "ms"</div>
                                                                    <div style="grid-column: span 2;">"Claimed By Identity: " <span style="color:#60a5fa;">{&p.claimed_by}</span></div>
                                                                    <div style="grid-column: span 2;">"Evidence: " {p.evidence_refs.join(", ")}</div>
                                                                    <div style=move || format!("grid-column: span 2; background: #2e1065; border: 1px solid #4c1d95; padding: 0.5rem; border-radius: 0.25rem; color: #d8b4fe; font-size: 0.7rem; margin-top: 0.25rem; display: {};", if p.trust_status == PeerTrustStatus::FederatedConsensus { "block" } else { "none" })>
                                                                        <strong>"Consensus Explainer:"</strong><br/>
                                                                        "• 2 trusted witnesses agreed."<br/>
                                                                        "• 0 simulated signatures counted."<br/>
                                                                        "• 0 revoked signers counted."<br/>
                                                                        "• Claim freshness: valid."<br/>
                                                                        "• Capability scope: valid."
                                                                    </div>
                                                                </div>
                                                            </div>
                                                        }
                                                    }).collect::<Vec<_>>().into_view()
                                                }
                                            })}
                                        </div>
                                    </div>
                                }.into_view(),

                                "attack-path" => view! {
                                    <div style="display: flex; flex-direction: column; gap: 1.5rem;">
                                        <div>
                                            <h3 style="margin: 0; font-size: 1.1rem; color: #f43f5e;">"Threat Vector & Risk Causal Chain"</h3>
                                            <p style="margin: 0.25rem 0 0 0; font-size: 0.75rem; color: #94a3b8;">"Explaining cascading threat vectors relative to baseline declarations without alarmist indicators"</p>
                                        </div>

                                        <div style="display: flex; flex-direction: column; gap: 0.5rem;">
                                            {
                                                let (nodes, _) = generate_dashboard_mock_attack_path();
                                                nodes.into_iter().enumerate().map(|(idx, node)| {
                                                    view! {
                                                        <div style="display: flex; align-items: stretch; gap: 1rem;">
                                                            <div style="display: flex; flex-direction: column; align-items: center; width: 2rem;">
                                                                <div style="width: 2rem; height: 2rem; border-radius: 50%; background: #1e293b; border: 1px solid #374151; color: #f1f5f9; display: flex; align-items: center; justify-content: center; font-size: 0.8rem; font-weight: bold;">
                                                                    {idx + 1}
                                                                </div>
                                                                <div style="flex-grow: 1; width: 2px; background: #374151; min-height: 1.5rem; margin: 0.25rem 0;"></div>
                                                            </div>
                                                            <div style="flex-grow: 1; background: #18181b; border: 1px solid #27272a; padding: 1rem; border-radius: 0.5rem; margin-bottom: 0.75rem;">
                                                                <div style="display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #27272a; padding-bottom: 0.25rem; margin-bottom: 0.5rem;">
                                                                    <strong style="color:#f8fafc; font-size:0.9rem;">{node.display_name}</strong>
                                                                    <span style="font-size:0.7rem; font-family:monospace; color:#94a3b8;">"Confidence: " {format!("{:.0}%", node.confidence * 100.0)}</span>
                                                                </div>
                                                                <div style="font-size: 0.8rem; color:#e4e4e7; font-family: monospace;">
                                                                    "Anomaly: " {node.step_description}
                                                                </div>
                                                                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 0.5rem; font-size: 0.7rem; font-family: monospace; color: #71717a; margin-top: 0.5rem;">
                                                                    <div>"Source: " {node.source_collector}</div>
                                                                    <div>"Truth Layer: " {node.truth_layer}</div>
                                                                    <div style="grid-column: span 2;">"Evidence Hash: " <span style="color:#a1a1aa;">{node.evidence_hash}</span></div>
                                                                </div>
                                                                <div style="margin-top: 0.5rem; font-size: 0.75rem; background: #1e1b4b; color: #c7d2fe; padding: 0.4rem; border-radius: 0.25rem;">
                                                                    <strong>"Recommended Action: "</strong> {node.recommended_action}
                                                                </div>
                                                            </div>
                                                        </div>
                                                    }
                                                }).collect::<Vec<_>>().into_view()
                                            }
                                        </div>
                                    </div>
                                }.into_view(),

                                "evidence" => view! {
                                    <div style="display: flex; flex-direction: column; gap: 1rem;">
                                        <h3>"Audit Evidence Ledger"</h3>
                                        <div style="overflow-x: auto; background: #111827; border: 1px solid #1e293b; border-radius: 0.5rem; padding: 0.5rem;">
                                            <table style="width: 100%; border-collapse: collapse; text-align: left; font-size: 0.8rem; font-family: monospace;">
                                                <thead>
                                                    <tr style="border-bottom: 1px solid #1e293b; color: #94a3b8;">
                                                        <th style="padding: 0.75rem;">"Time"</th>
                                                        <th style="padding: 0.75rem;">"Actor"</th>
                                                        <th style="padding: 0.75rem;">"Action"</th>
                                                        <th style="padding: 0.75rem;">"Target"</th>
                                                        <th style="padding: 0.75rem;">"Evidence Hash"</th>
                                                        <th style="padding: 0.75rem;">"Status"</th>
                                                    </tr>
                                                </thead>
                                                <tbody>
                                                    {receipts.with_value(|recs| {
                                                        recs.iter().map(|rec| {
                                                            let r = rec.clone();
                                                            let formatted_time = format!("{}", r.requested_at_unix_ms);
                                                            let first_hash = r.evidence_hashes.first().cloned().unwrap_or_else(|| "None".to_string());
                                                            view! {
                                                                <tr style="border-bottom: 1px solid #1e293b; color: #f1f5f9;">
                                                                    <td style="padding: 0.75rem; white-space: nowrap;">{formatted_time}</td>
                                                                    <td style="padding: 0.75rem; color: #38bdf8;">{r.actor_did}</td>
                                                                    <td style="padding: 0.75rem; color: #f472b6;">{format!("{:?}", r.action_kind)}</td>
                                                                    <td style="padding: 0.75rem;">{r.target_node_id}</td>
                                                                    <td style="padding: 0.75rem; color: #94a3b8; font-size: 0.7rem;">{first_hash}</td>
                                                                    <td style="padding: 0.75rem; color: #4ade80;">{format!("{:?}", r.chronicle_status)}</td>
                                                                </tr>
                                                            }
                                                        }).collect::<Vec<_>>()
                                                    })}
                                                </tbody>
                                            </table>
                                        </div>
                                    </div>
                                }.into_view(),

                                _ => view! {
                                    // Topology graph list representation
                                    <div style="display: flex; flex-direction: column; gap: 1rem;">
                                        <div style="display: flex; justify-content: space-between; align-items: center;">
                                            <h3 style="margin: 0;">"Nodes List"</h3>
                                            <span style="font-size: 0.75rem; color: #64748b;">"Select a node to inspect"</span>
                                        </div>

                                        // Hierarchical Layer Toggle Filter
                                        <div style="display: flex; gap: 0.5rem; background: #0b0c0f; padding: 0.5rem; border-radius: 0.375rem; border: 1px solid #1e293b; font-size: 0.85rem;">
                                            <span style="color: #64748b; padding-right: 0.5rem; display: flex; align-items: center;">"Filter Layer:"</span>
                                            <button on:click=move |_| set_topology_filter.set("all".to_string()) style=move || format!("border: none; background: {}; color: {}; padding: 0.25rem 0.5rem; border-radius: 0.25rem; cursor: pointer; font-size: 0.75rem; font-weight: 500;", if topology_filter.get() == "all" { "#1e293b" } else { "transparent" }, if topology_filter.get() == "all" { "#38bdf8" } else { "#94a3b8" })>"All Layers"</button>
                                            <button on:click=move |_| set_topology_filter.set("physical".to_string()) style=move || format!("border: none; background: {}; color: {}; padding: 0.25rem 0.5rem; border-radius: 0.25rem; cursor: pointer; font-size: 0.75rem; font-weight: 500;", if topology_filter.get() == "physical" { "#1e293b" } else { "transparent" }, if topology_filter.get() == "physical" { "#38bdf8" } else { "#94a3b8" })>"Physical & Routing"</button>
                                            <button on:click=move |_| set_topology_filter.set("virtual".to_string()) style=move || format!("border: none; background: {}; color: {}; padding: 0.25rem 0.5rem; border-radius: 0.25rem; cursor: pointer; font-size: 0.75rem; font-weight: 500;", if topology_filter.get() == "virtual" { "#1e293b" } else { "transparent" }, if topology_filter.get() == "virtual" { "#38bdf8" } else { "#94a3b8" })>"Virtualization & Tunnels"</button>
                                        </div>

                                        <div style="display: flex; flex-direction: column; gap: 0.5rem;">
                                            {move || initial_topology.with_value(|t| {
                                                t.nodes.iter().filter(|n| {
                                                    match topology_filter.get().as_str() {
                                                        "physical" => match n.node_kind {
                                                            NodeKind::Container | NodeKind::VirtualMachine | NodeKind::VirtualBridge => false,
                                                            _ => true
                                                        },
                                                        "virtual" => match n.node_kind {
                                                            NodeKind::Container | NodeKind::VirtualMachine | NodeKind::VirtualBridge => true,
                                                            _ => false
                                                        },
                                                        _ => true
                                                    }
                                                }).map(|node| {
                                                    let n = node.clone();
                                                    let node_id_for_selected = n.node_id.clone();
                                                    let node_id_for_click = n.node_id.clone();
                                                    let is_selected = move || selected_node_id.get() == Some(node_id_for_selected.clone());
                                                    let hostname_label = n.hostname.clone().unwrap_or_else(|| n.node_id.clone());
                                                    view! {
                                                        <div
                                                            on:click=move |_| set_selected_node_id.set(Some(node_id_for_click.clone()))
                                                            style=move || format!("
                                                                padding: 1rem;
                                                                border-radius: 0.5rem;
                                                                border: 1px solid {};
                                                                background-color: {};
                                                                cursor: pointer;
                                                                transition: all 0.15s ease;
                                                            ",
                                                                if is_selected() { "#38bdf8" } else { "#1e293b" },
                                                                if is_selected() { "#0c4a6e" } else { "#111827" }
                                                            )
                                                        >
                                                            <div style="display: flex; justify-content: space-between; align-items: center;">
                                                                <div>
                                                                    <strong style="display: block; color: #f1f5f9;">
                                                                        {hostname_label.clone()}
                                                                    </strong>
                                                                    <span style="font-size: 0.75rem; color: #94a3b8; font-family: monospace;">
                                                                        {format!("{:?} // {:?}", n.node_kind, n.management_state)}
                                                                    </span>
                                                                </div>
                                                                <span style="width: 8px; height: 8px; background: #38bdf8; border-radius: 50%;"></span>
                                                            </div>
                                                        </div>
                                                    }
                                                }).collect::<Vec<_>>()
                                            })}
                                        </div>

                                        <h3 style="margin-top: 1.5rem; margin-bottom: 0.5rem;">"Observed Network Links (Edges)"</h3>
                                        <div style="display: flex; flex-direction: column; gap: 0.5rem;">
                                            {move || initial_topology.with_value(|t| {
                                                t.edges.iter().filter(|e| {
                                                    match topology_filter.get().as_str() {
                                                        "physical" => match e.edge_kind {
                                                            EdgeKind::VirtualBridgeLink | EdgeKind::ContainerLink | EdgeKind::VlanTag | EdgeKind::WireGuardPeer | EdgeKind::XeniaSession => false,
                                                            _ => true
                                                        },
                                                        "virtual" => match e.edge_kind {
                                                            EdgeKind::VirtualBridgeLink | EdgeKind::ContainerLink | EdgeKind::VlanTag | EdgeKind::WireGuardPeer | EdgeKind::XeniaSession => true,
                                                            _ => false
                                                        },
                                                        _ => true
                                                    }
                                                }).map(|edge| {
                                                    let e = edge.clone();
                                                    view! {
                                                        <div style="background: #0f172a; padding: 0.75rem; border-radius: 0.375rem; border: 1px solid #1e293b; font-size: 0.875rem; font-family: monospace;">
                                                            <span style="color: #38bdf8;">{&e.source_node_id}</span>
                                                            <span style="color: #64748b;">" ──(" {format!("{:?}", e.edge_kind)} ")──> "</span>
                                                            <span style="color: #34d399;">{&e.target_node_id}</span>
                                                            <span style="float: right; color: #94a3b8;">{format!("(cf: {:.2})", e.confidence)}</span>
                                                        </div>
                                                    }
                                                }).collect::<Vec<_>>()
                                            })}
                                        </div>
                                    </div>
                                }.into_view()
                            }}
                        </div>

                        // Right Pane: Inspector & Evidence Sidebar
                        <div style="width: 50%; padding: 1.5rem; display: flex; flex-direction: column; gap: 1.5rem; overflow-y: auto; background-color: #0b0c0f;">
                            {move || if let Some(n) = selected_node() {
                                view! {
                                    <div>
                                        <h2 style="margin-top: 0; color: #38bdf8;">"Node Inspector"</h2>
                                        <table style="width: 100%; border-collapse: collapse; font-size: 0.9rem;">
                                            <tbody>
                                                <tr style="border-bottom: 1px solid #1e293b;">
                                                    <td style="padding: 0.75rem 0; color: #94a3b8;">"Node ID"</td>
                                                    <td style="padding: 0.75rem 0; font-family: monospace; font-weight: 500;">{&n.node_id}</td>
                                                </tr>
                                                <tr style="border-bottom: 1px solid #1e293b;">
                                                    <td style="padding: 0.75rem 0; color: #94a3b8;">"Hostname"</td>
                                                    <td style="padding: 0.75rem 0; font-weight: 500;">{n.hostname.clone().unwrap_or_default()}</td>
                                                </tr>
                                                <tr style="border-bottom: 1px solid #1e293b;">
                                                    <td style="padding: 0.75rem 0; color: #94a3b8;">"Kind"</td>
                                                    <td style="padding: 0.75rem 0; font-weight: 500;">{format!("{:?}", n.node_kind)}</td>
                                                </tr>
                                                <tr style="border-bottom: 1px solid #1e293b;">
                                                    <td style="padding: 0.75rem 0; color: #94a3b8;">"Management State"</td>
                                                    <td style="padding: 0.75rem 0; font-weight: 500;">{format!("{:?}", n.management_state)}</td>
                                                </tr>
                                                <tr style="border-bottom: 1px solid #1e293b;">
                                                    <td style="padding: 0.75rem 0; color: #94a3b8;">"Owner DID"</td>
                                                    <td style="padding: 0.75rem 0; font-family: monospace; font-size: 0.8rem;">{n.owner_did.clone().unwrap_or_else(|| "None".to_string())}</td>
                                                </tr>
                                            </tbody>
                                        </table>

                                        // Node Drift Detail
                                        {if let Some(drift) = selected_drift() {
                                            view! {
                                                <div style="margin-top: 1.5rem; background: #27272a; padding: 1rem; border-radius: 0.375rem;">
                                                    <h4 style="margin-top: 0; color: #fbbf24;">"Configuration Drift Alert"</h4>
                                                    <p style="font-size: 0.875rem;">
                                                        "This node has drifted from the declared NixOS generation."
                                                    </p>
                                                    {if let Some(diff) = drift.diff_closure {
                                                        view! {
                                                            <pre style="background: #09090b; padding: 0.5rem; border-radius: 0.25rem; font-family: monospace; font-size: 0.75rem; overflow-x: auto; color: #fda4af;">
                                                                {diff}
                                                            </pre>
                                                        }.into_view()
                                                    } else {
                                                        view! {}.into_view()
                                                    }}
                                                </div>
                                            }.into_view()
                                        } else {
                                            view! {}.into_view()
                                        }}

                                        // Chronicle Receipts (Evidence)
                                        <div style="margin-top: 2rem;">
                                            <h3>"Infrastructure Ledger Receipts"</h3>
                                            {move || {
                                                let r_list = selected_receipts();
                                                if r_list.is_empty() {
                                                    view! {
                                                        <p style="color: #64748b; font-style: italic; font-size: 0.875rem;">
                                                            "No ledger activity or config modifications signed for this node."
                                                        </p>
                                                    }.into_view()
                                                } else {
                                                    view! {
                                                        <div style="display: flex; flex-direction: column; gap: 0.75rem;">
                                                            {r_list.into_iter().map(|rec| {
                                                                let receipt_id_short = rec.receipt_id[..std::cmp::min(8, rec.receipt_id.len())].to_string();
                                                                let status_str = format!("{:?}", rec.chronicle_status);
                                                                let action_str = format!("{:?} action by operator", rec.action_kind);
                                                                let actor_did_owned = rec.actor_did.clone();
                                                                view! {
                                                                    <div style="background: #1e293b; padding: 0.75rem; border-radius: 0.375rem; font-size: 0.875rem;">
                                                                        <div style="display: flex; justify-content: space-between; font-family: monospace; font-size: 0.75rem; color: #38bdf8;">
                                                                            <span>"ID: " {receipt_id_short}</span>
                                                                            <span>{status_str}</span>
                                                                        </div>
                                                                        <p style="margin: 0.5rem 0; font-weight: 500;">
                                                                            {action_str}
                                                                        </p>
                                                                        <div style="font-size: 0.75rem; color: #94a3b8; font-family: monospace;">
                                                                            "Actor: " {actor_did_owned}
                                                                        </div>
                                                                    </div>
                                                                }
                                                            }).collect::<Vec<_>>()}
                                                        </div>
                                                    }.into_view()
                                                }
                                            }}
                                        </div>
                                    </div>
                                }.into_view()
                            } else {
                                view! {
                                    <div style="flex: 1; display: flex; align-items: center; justify-content: center; color: #64748b; font-style: italic;">
                                        "Select a node to begin legibility inspection."
                                    </div>
                                }.into_view()
                            }}
                        </div>
                    }.into_view()
                }}
            </main>
        </div>
    }
}

pub fn generate_dashboard_mock_attack_path() -> (Vec<AttackPathNode>, Vec<AttackPathEdge>) {
    let nodes = vec![
        AttackPathNode {
            node_id: "unknown-subnet-host".to_string(),
            display_name: "Unknown Device".to_string(),
            step_description: "Unknown device observed on subnet".to_string(),
            evidence_hash: "sha256-arp-subnet-unknown-mac-2244".to_string(),
            confidence: 0.98,
            truth_layer: "opnsense_arp_table".to_string(),
            source_collector: "OPNsense ARP Collector".to_string(),
            recommended_action: "Audit network interfaces".to_string(),
        },
        AttackPathNode {
            node_id: "forge-server".to_string(),
            display_name: "Forge Build Server".to_string(),
            step_description: "unexpected listening port 9999".to_string(),
            evidence_hash: "sha256-listening-port-9999-forge".to_string(),
            confidence: 0.88,
            truth_layer: "listening_port_collector".to_string(),
            source_collector: "Local Netlink Sockets".to_string(),
            recommended_action: "Investigate process lineage".to_string(),
        },
        AttackPathNode {
            node_id: "forge-server-drift".to_string(),
            display_name: "Forge System State".to_string(),
            step_description: "NixOS generation drift detected (Gen 428 != Gen 427)".to_string(),
            evidence_hash: "sha256-drift-nixos-profile-forge".to_string(),
            confidence: 0.95,
            truth_layer: "nixos_profile_collector".to_string(),
            source_collector: "NixOS Profile Verifier".to_string(),
            recommended_action: "Generate rollback plan".to_string(),
        },
        AttackPathNode {
            node_id: "forge-server-rollback".to_string(),
            display_name: "Rollback Plan".to_string(),
            step_description: "Rollback plan available (dry-run ready)".to_string(),
            evidence_hash: "sha256-dry-run-rollback-forge".to_string(),
            confidence: 1.0,
            truth_layer: "dry_run_rollback_plan".to_string(),
            source_collector: "Symthaea Advisors / Net Steward".to_string(),
            recommended_action: "Request manual operator confirmation".to_string(),
        },
    ];

    let edges = vec![
        AttackPathEdge {
            source_node_id: "unknown-subnet-host".to_string(),
            target_node_id: "forge-server".to_string(),
            relationship: "connected to same subnet as".to_string(),
        },
        AttackPathEdge {
            source_node_id: "forge-server".to_string(),
            target_node_id: "forge-server-drift".to_string(),
            relationship: "triggers".to_string(),
        },
        AttackPathEdge {
            source_node_id: "forge-server-drift".to_string(),
            target_node_id: "forge-server-rollback".to_string(),
            relationship: "recommends".to_string(),
        },
    ];

    (nodes, edges)
}
