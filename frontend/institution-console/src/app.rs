use crate::domain::{InstitutionProfile, SecurityState, WorkspaceContext};
use crate::ui::{AuthorityNotice, SecurityBadge, SecurityLegend};
use leptos::prelude::*;
use leptos_meta::{provide_meta_context, MetaTags, Stylesheet, Title};
use crate::reconciliation::ReconciliationPage;
use leptos_router::{
    components::{A, Route, Router, Routes},
    StaticSegment,
};

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
                    <Route path=StaticSegment("reconciliation") view=ReconciliationPage/>
                </Routes>
            </main>
        </Router>
    }
}

#[cfg(feature = "demo")]
#[component]
fn InstitutionHomePage() -> impl IntoView {
    let profile = RwSignal::new(InstitutionProfile::Bank);
    let context = move || WorkspaceContext::demo(profile.get());

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

                <div class="demo-label">"DEVELOPMENT / DEMONSTRATION"</div>
                <label class="field-label" for="institution-profile">"Profile simulation"</label>
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
                    <For
                        each=|| InstitutionProfile::ALL
                        key=|profile| profile.css_key()
                        children=|profile| view! {
                            <option value=profile.css_key()>{profile.label()}</option>
                        }
                    />
                </select>
                <p class="side-note">
                    "Production profile selection must come from the authorized workspace context, not this client control."
                </p>

                <nav class="nav" aria-label="Institution modules">
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
                    <span>"Security semantics centralized"</span>
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
                        <strong>"Demonstration Entity"</strong>
                        <span class="identity-meta">"No live legal-entity binding"</span>
                    </div>
                </header>

                <AuthorityNotice context=context() />

                <div class="security-banner">
                    <div>
                        <strong>"Control boundary"</strong>
                        <span>"Browser state is advisory. Material actions must be authorized independently with principal, tenant, legal entity, capability and policy context."</span>
                    </div>
                    <SecurityBadge state=SecurityState::Qualified/>
                </div>

                <section class="metrics" aria-label="Synthetic demonstration metrics">
                    <MetricCard title="Available liquidity" value="$18.4M" detail="Synthetic · evidence frontier placeholder" />
                    <MetricCard title="Open reconciliation breaks" value="27" detail="Synthetic · 5 high-priority" />
                    <MetricCard title="Settlement in flight" value="$4.2M" detail="Synthetic · 3 rails" />
                    <MetricCard title="Control exceptions" value="4" detail="Synthetic · 2 awaiting decision" />
                </section>

                <section class="grid">
                    <article class="panel large">
                        <div class="panel-header">
                            <div>
                                <span class="eyebrow">"RECONCILIATION"</span>
                                <h2>"Continuous control"</h2>
                            </div>
                            <A class="action" href="/reconciliation">"Review queue"</A>
                        </div>
                        <div class="break-row">
                            <div>
                                <strong>"Processor ↔ Core"</strong>
                                <span>"Amount mismatch · synthetic example"</span>
                            </div>
                            <span class="amount">"$18,420"</span>
                            <button class="action" type="button">"Review"</button>
                        </div>
                        <div class="break-row">
                            <div>
                                <strong>"Custodian ↔ GL"</strong>
                                <span>"Timing difference · synthetic example"</span>
                            </div>
                            <span class="amount">"$7,210"</span>
                            <button class="action" type="button">"Inspect"</button>
                        </div>
                        <div class="break-row">
                            <div>
                                <strong>"Rail ↔ Settlement journal"</strong>
                                <span>"Indeterminate external outcome · synthetic example"</span>
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
                            <span>"Synthetic evidence-backed forecast"</span>
                        </div>
                    </article>

                    <article class="panel">
                        <div class="panel-header">
                            <div>
                                <span class="eyebrow">"SETTLEMENT"</span>
                                <h2>"Active obligations"</h2>
                            </div>
                        </div>
                        <SettlementRow name="Local rail" amount="$1.2M" state=SecurityState::Finalized />
                        <SettlementRow name="Ethereum anchor" amount="$890K" state=SecurityState::Included />
                        <SettlementRow name="Polygon rail" amount="$640K" state=SecurityState::Pending />
                        <SettlementRow name="Correspondent bank" amount="$1.5M" state=SecurityState::Authorized />
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
                        <p class="muted">
                            "A production implementation should retain source frontier, policy version, model/version identity, authority context and reconciliation state for every material metric."
                        </p>
                    </article>
                </section>

                <SecurityLegend/>
            </section>
        </div>
    }
}

#[cfg(not(feature = "demo"))]
#[component]
fn InstitutionHomePage() -> impl IntoView {
    view! {
        <section class="nonproduction-boundary">
            <span class="eyebrow">"FINANCIAL CONTROL PLANE"</span>
            <h1>"Live integration required"</h1>
            <p class="muted">
                "The default build does not expose synthetic financial balances or simulated institutional state."
            </p>
            <div class="authority-notice demo" role="alert">
                <strong>"No live financial context is configured"</strong>
                <span>
                    "The production workspace must obtain its tenant, legal entity, principal, institution profile and policy context from an authenticated authority boundary."
                </span>
            </div>
        </section>
    }
}

#[cfg(feature = "demo")]
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

#[cfg(feature = "demo")]
#[component]
fn SettlementRow(name: &'static str, amount: &'static str, state: SecurityState) -> impl IntoView {
    view! {
        <div class="settlement-row">
            <div>
                <strong>{name}</strong>
                <span>{amount}</span>
            </div>
            <SecurityBadge state/>
        </div>
    }
}
