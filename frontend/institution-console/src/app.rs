use leptos::prelude::*;
use leptos_meta::{provide_meta_context, MetaTags, Stylesheet, Title};
use leptos_router::{
    components::{Route, Router, Routes},
    StaticSegment,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InstitutionProfile {
    Bank,
    CreditUnion,
    PaymentInstitution,
    CorporateTreasury,
    Custodian,
    CommonsFinance,
}

impl InstitutionProfile {
    fn label(self) -> &'static str {
        match self {
            Self::Bank => "Bank",
            Self::CreditUnion => "Credit union / cooperative",
            Self::PaymentInstitution => "Payment institution / fintech",
            Self::CorporateTreasury => "Corporate treasury",
            Self::Custodian => "Custodian / market infrastructure",
            Self::CommonsFinance => "Commons / cooperative finance",
        }
    }

    fn modules(self) -> &'static [&'static str] {
        match self {
            Self::Bank => &[
                "Overview", "Journal", "Reconciliation", "Treasury",
                "Payments", "Credit", "Risk", "Compliance", "Settlement", "Evidence",
            ],
            Self::CreditUnion => &[
                "Overview", "Members", "Journal", "Reconciliation", "Treasury",
                "Payments", "Credit", "Community", "Settlement", "Evidence",
            ],
            Self::PaymentInstitution => &[
                "Overview", "Payments", "Reconciliation", "Fraud",
                "Liquidity", "Settlement", "Cases", "Evidence",
            ],
            Self::CorporateTreasury => &[
                "Overview", "Cash", "Liquidity", "Funding", "FX",
                "Counterparties", "Settlement", "Reconciliation", "Evidence",
            ],
            Self::Custodian => &[
                "Overview", "Positions", "Collateral", "Settlement",
                "Corporate actions", "Reconciliation", "Risk", "Evidence",
            ],
            Self::CommonsFinance => &[
                "Overview", "Pools", "Allocations", "Journal",
                "Reconciliation", "Treasury", "Settlement", "Evidence",
            ],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EvidenceState {
    Observed,
    Verified,
    Qualified,
    Authorized,
    Pending,
    Included,
    Finalized,
    Reconciled,
    Disputed,
    Superseded,
    Indeterminate,
}

impl EvidenceState {
    fn label(self) -> &'static str {
        match self {
            Self::Observed => "Observed",
            Self::Verified => "Verified",
            Self::Qualified => "Qualified",
            Self::Authorized => "Authorized",
            Self::Pending => "Pending",
            Self::Included => "Included",
            Self::Finalized => "Finalized",
            Self::Reconciled => "Reconciled",
            Self::Disputed => "Disputed",
            Self::Superseded => "Superseded",
            Self::Indeterminate => "Indeterminate",
        }
    }
}

pub fn shell(options: LeptosOptions) -> impl IntoView {
    view! {
        <!DOCTYPE html>
        <html lang="en">
            <head>
                <meta charset="utf-8"/>
                <meta name="viewport" content="width=device-width, initial-scale=1"/>
                <AutoReload options=options.clone() />
                <HydrationScripts options/>
                <MetaTags/>
            </head>
            <body>
                <App/>
            </body>
        </html>
    }
}

#[component]
pub fn App() -> impl IntoView {
    provide_meta_context();

    view! {
        <Stylesheet id="leptos" href="/pkg/mycelix-institution-console.css"/>
        <Title text="Mycelix Institution Console"/>
        <Router>
            <main>
                <Routes fallback=|| view! {
                    <section class="empty-state">
                        <h1>"Page not found"</h1>
                        <p>"The requested workspace does not exist."</p>
                    </section>
                }.into_view()>
                    <Route path=StaticSegment("") view=InstitutionHomePage/>
                </Routes>
            </main>
        </Router>
    }
}

#[component]
fn InstitutionHomePage() -> impl IntoView {
    let profile = RwSignal::new(InstitutionProfile::Bank);

    view! {
        <div class="console">
            <aside class="sidebar">
                <div class="brand">
                    <span class="brand-mark">"M"</span>
                    <div>
                        <strong>"Mycelix"</strong>
                        <span>"Institution Console"</span>
                    </div>
                </div>

                <label class="field-label" for="institution-profile">"Workspace profile"</label>
                <select
                    id="institution-profile"
                    class="profile-select"
                    on:change=move |event| {
                        let value = event_target_value(&event);
                        profile.set(match value.as_str() {
                            "credit-union" => InstitutionProfile::CreditUnion,
                            "payment" => InstitutionProfile::PaymentInstitution,
                            "treasury" => InstitutionProfile::CorporateTreasury,
                            "custodian" => InstitutionProfile::Custodian,
                            "commons" => InstitutionProfile::CommonsFinance,
                            _ => InstitutionProfile::Bank,
                        });
                    }
                >
                    <option value="bank">"Bank"</option>
                    <option value="credit-union">"Credit union / cooperative"</option>
                    <option value="payment">"Payment institution / fintech"</option>
                    <option value="treasury">"Corporate treasury"</option>
                    <option value="custodian">"Custodian / market infrastructure"</option>
                    <option value="commons">"Commons / cooperative finance"</option>
                </select>

                <nav class="nav">
                    <div class="nav-heading">"Workspace"</div>
                    <For
                        each=move || profile.get().modules().iter().enumerate().map(|(index, name)| (index, *name)).collect::<Vec<_>>()
                        key=|(index, _)| *index
                        children=move |(_, name)| view! {
                            <button class="nav-item" type="button">{name}</button>
                        }
                    />
                </nav>

                <div class="sidebar-footer">
                    <span class="security-dot"></span>
                    <span>"Policy controls active"</span>
                </div>
            </aside>

            <section class="workspace">
                <header class="topbar">
                    <div>
                        <span class="eyebrow">"FINANCIAL CONTROL PLANE"</span>
                        <h1>{move || profile.get().label()}</h1>
                    </div>
                    <div class="identity">
                        <span class="identity-label">"Workspace"</span>
                        <strong>"Example Financial Entity"</strong>
                        <span class="identity-meta">"Legal entity scope · configured"</span>
                    </div>
                </header>

                <div class="security-banner">
                    <div>
                        <strong>"Security state"</strong>
                        <span>"Browser state is advisory; every privileged action is re-authorized at the control boundary."</span>
                    </div>
                    <EvidenceBadge state=EvidenceState::Qualified/>
                </div>

                <section class="metrics">
                    <MetricCard title="Available liquidity" value="$18.4M" detail="Projected through current evidence frontier" />
                    <MetricCard title="Open reconciliation breaks" value="27" detail="5 high-priority · 22 ordinary" />
                    <MetricCard title="Settlement in flight" value="$4.2M" detail="3 rails · 8 obligations" />
                    <MetricCard title="Control exceptions" value="4" detail="2 awaiting human decision" />
                </section>

                <section class="grid">
                    <article class="panel large">
                        <div class="panel-header">
                            <div>
                                <span class="eyebrow">"RECONCILIATION"</span>
                                <h2>"Continuous control"</h2>
                            </div>
                            <EvidenceBadge state=EvidenceState::Reconciled/>
                        </div>
                        <div class="break-row">
                            <div>
                                <strong>"Processor ↔ Core"</strong>
                                <span>"Amount mismatch · 12 minutes ago"</span>
                            </div>
                            <span class="amount">"$18,420"</span>
                            <button class="action" type="button">"Review"</button>
                        </div>
                        <div class="break-row">
                            <div>
                                <strong>"Custodian ↔ GL"</strong>
                                <span>"Timing difference · 32 minutes ago"</span>
                            </div>
                            <span class="amount">"$7,210"</span>
                            <button class="action" type="button">"Inspect"</button>
                        </div>
                        <div class="break-row">
                            <div>
                                <strong>"Rail ↔ Settlement journal"</strong>
                                <span>"Indeterminate external outcome"</span>
                            </div>
                            <span class="amount">"$92,000"</span>
                            <button class="action warning" type="button">"Resolve"</button>
                        </div>
                    </article>

                    <article class="panel">
                        <div class="panel-header">
                            <div>
                                <span class="eyebrow">"LIQUIDITY"</span>
                                <h2>"Next 24 hours"</h2>
                            </div>
                        </div>
                        <div class="liquidity-row"><span>"Opening"</span><strong>"$24.8M"</strong></div>
                        <div class="liquidity-row"><span>"Expected inflows"</span><strong>"+$8.7M"</strong></div>
                        <div class="liquidity-row"><span>"Settlement outflows"</span><strong>"−$10.1M"</strong></div>
                        <div class="liquidity-row total"><span>"Projected closing"</span><strong>"$23.4M"</strong></div>
                        <div class="chart-placeholder">
                            <div class="chart-line"></div>
                            <span>"Evidence-backed forecast"</span>
                        </div>
                    </article>

                    <article class="panel">
                        <div class="panel-header">
                            <div>
                                <span class="eyebrow">"SETTLEMENT"</span>
                                <h2>"Active obligations"</h2>
                            </div>
                        </div>
                        <SettlementRow name="ACH / Local" amount="$1.2M" state=EvidenceState::Finalized />
                        <SettlementRow name="Ethereum / Anchor" amount="$890K" state=EvidenceState::Included />
                        <SettlementRow name="Polygon / Payments" amount="$640K" state=EvidenceState::Pending />
                        <SettlementRow name="Bank / Correspondent" amount="$1.5M" state=EvidenceState::Authorized />
                    </article>

                    <article class="panel large">
                        <div class="panel-header">
                            <div>
                                <span class="eyebrow">"EVIDENCE"</span>
                                <h2>"Why this number?"</h2>
                            </div>
                            <button class="action" type="button">"Open lineage"</button>
                        </div>
                        <div class="lineage">
                            <span class="lineage-node">"Source observations"</span>
                            <span class="arrow">"→"</span>
                            <span class="lineage-node">"Qualified evidence"</span>
                            <span class="arrow">"→"</span>
                            <span class="lineage-node">"Policy evaluation"</span>
                            <span class="arrow">"→"</span>
                            <span class="lineage-node">"Economic event"</span>
                            <span class="arrow">"→"</span>
                            <span class="lineage-node">"Settlement receipt"</span>
                        </div>
                        <p class="muted">"Every material metric is intended to retain its source frontier, policy version, model/version identity, and reconciliation state."</p>
                    </article>
                </section>
            </section>
        </div>
    }
}

#[component]
fn MetricCard(title: &'static str, value: &'static str, detail: &'static str) -> impl IntoView {
    view! {
        <article class="metric-card">
            <span class="eyebrow">{title}</span>
            <strong>{value}</strong>
            <span>{detail}</span>
        </article>
    }
}

#[component]
fn EvidenceBadge(state: EvidenceState) -> impl IntoView {
    let class = match state {
        EvidenceState::Observed => "badge observed",
        EvidenceState::Verified => "badge verified",
        EvidenceState::Qualified => "badge qualified",
        EvidenceState::Authorized => "badge authorized",
        EvidenceState::Pending => "badge pending",
        EvidenceState::Included => "badge included",
        EvidenceState::Finalized => "badge finalized",
        EvidenceState::Reconciled => "badge reconciled",
        EvidenceState::Disputed => "badge disputed",
        EvidenceState::Superseded => "badge superseded",
        EvidenceState::Indeterminate => "badge indeterminate",
    };
    view! { <span class=class>{state.label()}</span> }
}

#[component]
fn SettlementRow(name: &'static str, amount: &'static str, state: EvidenceState) -> impl IntoView {
    view! {
        <div class="settlement-row">
            <div>
                <strong>{name}</strong>
                <span>{amount}</span>
            </div>
            <EvidenceBadge state/>
        </div>
    }
}
