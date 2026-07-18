use leptos::prelude::*;
use leptos::task::spawn_local;
use leptos_router::{
    components::{A, Route, Router, Routes},
    path,
};
use marketplace_client::MarketplaceClient;
use marketplace_domain::{
    ActionHash, AgentPubKey, ApproveTransactionConflictInput, CreateTransactionInput,
    DerivedReputation, FileDisputeInput, FinalizeBilateralTransactionConflictInput,
    ListingOutput, ListingStatus, MarkShippedInput, TransactionConflictApprovalOutput,
    TransactionOutput, TransactionResolution, TransactionResolutionReason,
    TransactionResolutionState, TransactionSettlementResult, TransactionStatus,
};
use std::str::FromStr;

#[cfg(all(feature = "js-bridge", feature = "dev-fixtures"))]
compile_error!("js-bridge and dev-fixtures are mutually exclusive runtime modes");

#[cfg(feature = "js-bridge")]
use marketplace_client::{BridgeConnectionInfo, JsBridgeTransport};
#[cfg(feature = "js-bridge")]
type RuntimeTransport = JsBridgeTransport;

#[cfg(feature = "dev-fixtures")]
use marketplace_client::{FixtureTransport, fixture_agent};
#[cfg(feature = "dev-fixtures")]
type RuntimeTransport = FixtureTransport;

fn main() {
    mount_to_body(App);
}

#[derive(Clone, Debug)]
enum ConnectionState {
    Connecting,
    Ready {
        installed_app_id: String,
        host_signer_available: bool,
        configured_roles: Vec<String>,
        active_roles: Vec<String>,
        agent_pub_key: AgentPubKey,
    },
    Failed(String),
}

impl ConnectionState {
    fn is_ready(&self) -> bool {
        matches!(self, Self::Ready { .. })
    }

    fn agent_pub_key(&self) -> Option<AgentPubKey> {
        match self {
            Self::Ready { agent_pub_key, .. } => Some(agent_pub_key.clone()),
            _ => None,
        }
    }

    fn has_active_role(&self, role: &str) -> bool {
        match self {
            Self::Ready { active_roles, .. } => active_roles.iter().any(|candidate| candidate == role),
            _ => false,
        }
    }

}

#[component]
fn App() -> impl IntoView {
    let connection = RwSignal::new(ConnectionState::Connecting);
    provide_context(connection);

    #[cfg(feature = "js-bridge")]
    let transport = JsBridgeTransport;
    #[cfg(feature = "dev-fixtures")]
    let transport = FixtureTransport::default();
    provide_context(transport.clone());

    #[cfg(feature = "js-bridge")]
    spawn_local(async move {
        match transport.connect().await {
            Ok(BridgeConnectionInfo {
                installed_app_id,
                host_signer_available,
                configured_roles,
                active_roles,
                agent_pub_key,
            }) => connection.set(ConnectionState::Ready {
                installed_app_id,
                host_signer_available,
                configured_roles,
                active_roles,
                agent_pub_key: AgentPubKey::new(agent_pub_key),
            }),
            Err(error) => connection.set(ConnectionState::Failed(error.to_string())),
        }
    });

    #[cfg(feature = "dev-fixtures")]
    connection.set(ConnectionState::Ready {
        installed_app_id: "fixture-preview".into(),
        host_signer_available: false,
        configured_roles: vec!["marketplace".into()],
        active_roles: vec!["marketplace".into()],
        agent_pub_key: fixture_agent(),
    });

    view! {
        <Router>
            <div class="marketplace-shell">
                <header class="marketplace-header">
                    <A href="/" attr:class="brand">"MYCELIX / MARKETPLACE"</A>
                    <nav aria-label="Primary navigation">
                        <A href="/">"Browse"</A>
                        <A href="/transactions">"Transactions"</A>
                    </nav>
                    <RuntimeBadge connection />
                </header>
                <main>
                    <RuntimeReadinessPanel connection />
                    <Routes fallback=NotFound>
                        <Route path=path!("/") view=BrowsePage />
                        <Route path=path!("/listing/:listing_id") view=ListingRoute />
                        <Route path=path!("/transactions") view=TransactionsPage />
                        <Route path=path!("/transaction/:transaction_id") view=TransactionRoute />
                    </Routes>
                </main>
            </div>
        </Router>
    }
}

#[component]
fn RuntimeBadge(connection: RwSignal<ConnectionState>) -> impl IntoView {
    move || match connection.get() {
        ConnectionState::Connecting => view! {
            <span class="runtime-badge connecting">"CONNECTING"</span>
        }
        .into_any(),
        ConnectionState::Ready {
            installed_app_id,
            active_roles,
            ..
        } => {
            #[cfg(feature = "dev-fixtures")]
            let (class_name, label) = ("runtime-badge fixture", "FIXTURE PREVIEW".to_string());
            #[cfg(not(feature = "dev-fixtures"))]
            let (class_name, label) = (
                "runtime-badge ready",
                format!("{} · {} ROLE(S)", installed_app_id, active_roles.len()),
            );
            view! { <span class=class_name>{label}</span> }.into_any()
        }
        ConnectionState::Failed(_) => view! {
            <span class="runtime-badge unavailable">"LIVE UNAVAILABLE"</span>
        }
        .into_any(),
    }
}

#[component]
fn RuntimeReadinessPanel(connection: RwSignal<ConnectionState>) -> impl IntoView {
    move || match connection.get() {
        ConnectionState::Ready {
            configured_roles,
            active_roles,
            ..
        } => {
            let deferred_roles = configured_roles
                .iter()
                .filter(|role| !active_roles.contains(role))
                .cloned()
                .collect::<Vec<_>>();
            let finance_active = active_roles.iter().any(|role| role == "finance");
            let finance_configured = configured_roles.iter().any(|role| role == "finance");
            let settlement_ready = cfg!(feature = "finance-settlement") && finance_active;
            let finance_message = if settlement_ready {
                "Finance role active; settlement calls are available to authorized buyers."
            } else if finance_active {
                "Finance role active, but this build does not enable settlement controls."
            } else if finance_configured {
                "Finance is configured but deferred or inactive; settlement remains unavailable."
            } else {
                "No Finance role is configured in this hApp; settlement remains unavailable."
            };

            view! {
                <section class="runtime-readiness" aria-labelledby="runtime-readiness-heading">
                    <div>
                        <p class="eyebrow">"DEPLOYMENT CAPABILITIES"</p>
                        <h2 id="runtime-readiness-heading">"Live role inventory"</h2>
                        <p class="muted">{finance_message}</p>
                    </div>
                    <div class="role-inventory" aria-label="Active hApp roles">
                        {active_roles.into_iter().map(|role| view! {
                            <span class="role-chip active">{format!("{role} · active")}</span>
                        }).collect_view()}
                        {deferred_roles.into_iter().map(|role| view! {
                            <span class="role-chip deferred">{format!("{role} · deferred")}</span>
                        }).collect_view()}
                    </div>
                    <p class="runtime-boundary">
                        "Arbitration contracts are present, but dispute creation stays UI-gated until concurrent-head injection evidence is archived."
                    </p>
                </section>
            }
            .into_any()
        }
        _ => ().into_any(),
    }
}

#[component]
fn BrowsePage() -> impl IntoView {
    view! {
        <section class="hero">
            <p class="eyebrow">"EVIDENCE-BEARING COMMERCE"</p>
            <h1>"Trade without hiding the basis for trust."</h1>
            <p>
                "The first Leptos vertical slice now uses one typed Rust contract from listing "
                "discovery through purchase creation and transaction recovery."
            </p>
        </section>
        <ListingBrowser />
    }
}

#[component]
fn ListingBrowser() -> impl IntoView {
    let connection = expect_context::<RwSignal<ConnectionState>>();
    let transport = expect_context::<RuntimeTransport>();
    let listings = RwSignal::new(None::<Result<Vec<ListingOutput>, String>>);
    let loading = RwSignal::new(false);
    let requested = RwSignal::new(false);

    let effect_transport = transport.clone();
    Effect::new(move |_| {
        if connection.get().is_ready() && !requested.get() {
            requested.set(true);
            request_listings(effect_transport.clone(), listings, loading);
        }
    });

    let reload_transport = transport.clone();

    view! {
        <section class="live-controls">
            <div>
                <p class="eyebrow">"VERTICAL SLICE / 01"</p>
                <h2>"Live listing discovery"</h2>
                <p class="muted">
                    {move || connection_message(connection.get())}
                </p>
            </div>
            <button
                class="primary-button"
                on:click=move |_| request_listings(reload_transport.clone(), listings, loading)
                disabled=move || loading.get() || !connection.get().is_ready()
            >
                {move || if loading.get() { "Loading…" } else { "Reload listings" }}
            </button>
        </section>
        <ConnectionAvailability connection />
        {move || match listings.get() {
            None => ().into_any(),
            Some(Ok(items)) if items.is_empty() => view! {
                <section class="availability compact">
                    <h3>"No listings returned"</h3>
                </section>
            }.into_any(),
            Some(Ok(items)) => view! {
                <ListingSection listings=items fixture=cfg!(feature = "dev-fixtures") />
            }.into_any(),
            Some(Err(message)) => view! {
                <section class="availability unavailable" role="alert">
                    <h3>"Listing call failed"</h3>
                    <p>{message}</p>
                </section>
            }.into_any(),
        }}
    }
}

fn request_listings(
    transport: RuntimeTransport,
    listings: RwSignal<Option<Result<Vec<ListingOutput>, String>>>,
    loading: RwSignal<bool>,
) {
    if loading.get_untracked() {
        return;
    }
    loading.set(true);
    listings.set(None);
    let client = MarketplaceClient::new(transport);
    spawn_local(async move {
        let result = client
            .get_all_listings()
            .await
            .map(|response| response.listings)
            .map_err(|error| error.to_string());
        listings.set(Some(result));
        loading.set(false);
    });
}

fn connection_message(connection: ConnectionState) -> String {
    match connection {
        ConnectionState::Connecting => "Opening authenticated app interface…".into(),
        ConnectionState::Ready {
            host_signer_available: true,
            active_roles,
            ..
        } => format!(
            "Connected; trusted host signer detected. Active roles: {}.",
            active_roles.join(", ")
        ),
        ConnectionState::Ready {
            host_signer_available: false,
            active_roles,
            ..
        } => format!(
            "Connected; zome calls use the configured official-client signing path. Active roles: {}.",
            active_roles.join(", ")
        ),
        ConnectionState::Failed(message) => format!("Unavailable: {message}"),
    }
}

#[component]
fn ConnectionAvailability(connection: RwSignal<ConnectionState>) -> impl IntoView {
    move || match connection.get() {
        ConnectionState::Connecting => view! {
            <section class="availability compact" role="status">
                <p>"Waiting for the Launcher/Tauri environment or explicit runtime token."</p>
            </section>
        }
        .into_any(),
        ConnectionState::Failed(message) => view! {
            <section class="availability unavailable" role="alert">
                <p class="eyebrow">"FAIL-CLOSED DEFAULT"</p>
                <h2>"Authenticated Marketplace transport unavailable"</h2>
                <p>{message}</p>
                <p>
                    "No fixture data was substituted. Configure the launcher environment or "
                    <code>"window.__MYCELIX_MARKETPLACE_CONFIG__"</code>
                    "."
                </p>
            </section>
        }
        .into_any(),
        ConnectionState::Ready { .. } => ().into_any(),
    }
}

#[component]
fn ListingSection(listings: Vec<ListingOutput>, fixture: bool) -> impl IntoView {
    view! {
        <section aria-labelledby="listings-heading">
            <div class="section-heading">
                <div>
                    <p class="eyebrow">{if fixture { "DEVELOPMENT DATA" } else { "LIVE DHT DATA" }}</p>
                    <h2 id="listings-heading">{if fixture { "Fixture listings" } else { "Marketplace listings" }}</h2>
                </div>
                {fixture.then(|| view! {
                    <p class="warning-copy">
                        "Explicit dev-fixtures build. It exercises the same typed client without representing conductor evidence."
                    </p>
                })}
            </div>
            <div class="listing-grid">
                {listings.into_iter().map(|listing| view! { <ListingCard listing /> }).collect_view()}
            </div>
        </section>
    }
}

#[component]
fn ListingCard(listing: ListingOutput) -> impl IntoView {
    let href = format!("/listing/{}", listing.listing_hash);
    let title = listing.listing.title.clone();
    let description = listing.listing.description.clone();
    let price = listing.listing.formatted_price();
    let category = listing.listing.category.label();
    let evidence = format!(
        "{:?} · {:?} · {:?}",
        listing.listing.epistemic.empirical,
        listing.listing.epistemic.normative,
        listing.listing.epistemic.materiality,
    );

    view! {
        <article class="listing-card">
            <div class="listing-image" aria-hidden="true">"PROVENANCE / IMAGE PENDING"</div>
            <div class="listing-body">
                <div class="listing-meta">
                    <span>{category}</span>
                    <span>{format!("{} available", listing.listing.quantity_available)}</span>
                </div>
                <h3><A href=href>{title}</A></h3>
                <p>{description}</p>
                <p class="evidence-chip">{evidence}</p>
                <strong class="price">{price}</strong>
            </div>
        </article>
    }
}

#[component]
fn ListingRoute() -> impl IntoView {
    let connection = expect_context::<RwSignal<ConnectionState>>();
    let transport = expect_context::<RuntimeTransport>();
    let detail = RwSignal::new(None::<Result<Option<ListingOutput>, String>>);
    let requested = RwSignal::new(false);
    let listing_hash = route_action_hash("listing");

    Effect::new(move |_| {
        if !connection.get().is_ready() || requested.get() {
            return;
        }
        requested.set(true);
        match listing_hash.clone() {
            Ok(hash) => {
                let client = MarketplaceClient::new(transport.clone());
                spawn_local(async move {
                    detail.set(Some(
                        client
                            .get_listing(&hash)
                            .await
                            .map_err(|error| error.to_string()),
                    ));
                });
            }
            Err(message) => detail.set(Some(Err(message))),
        }
    });

    view! {
        <ConnectionAvailability connection />
        {move || match detail.get() {
            None => view! {
                <section class="availability compact" role="status">
                    <p>"Loading listing from the typed ActionHash route…"</p>
                </section>
            }.into_any(),
            Some(Ok(Some(listing))) => view! { <ListingDetail listing /> }.into_any(),
            Some(Ok(None)) => view! {
                <section class="availability unavailable">
                    <h1>"Listing not found"</h1>
                    <A href="/">"Return to browse"</A>
                </section>
            }.into_any(),
            Some(Err(message)) => view! {
                <section class="availability unavailable" role="alert">
                    <h1>"Could not load listing"</h1>
                    <p>{message}</p>
                    <A href="/">"Return to browse"</A>
                </section>
            }.into_any(),
        }}
    }
}

#[component]
fn ListingDetail(listing: ListingOutput) -> impl IntoView {
    let connection = expect_context::<RwSignal<ConnectionState>>();
    let transport = expect_context::<RuntimeTransport>();
    let quantity = RwSignal::new(1_u32);
    let submitting = RwSignal::new(false);
    let purchase = RwSignal::new(None::<Result<TransactionOutput, String>>);

    let listing_hash = listing.listing_hash.clone();
    let seller = listing.seller_agent_id.clone();
    let unit_price = listing.listing.price_cents;
    let available = listing.listing.quantity_available;
    let title = listing.listing.title.clone();
    let description = listing.listing.description.clone();
    let category = listing.listing.category.label();
    let status = listing.listing.status.clone();
    let formatted_price = listing.listing.formatted_price();
    let seller_label = seller.to_string();
    let hash_label = listing_hash.to_string();
    let can_purchase_listing = status == ListingStatus::Active && available > 0;

    let seller_for_submit = seller.clone();
    let seller_for_disable = seller.clone();
    let seller_for_warning = seller.clone();
    let listing_hash_for_submit = listing_hash.clone();
    let transport_for_submit = transport.clone();
    let submit_purchase = move |_| {
        let requested_quantity = quantity.get_untracked().clamp(1, available.max(1));
        let total = match unit_price.checked_mul(u64::from(requested_quantity)) {
            Some(total) => total,
            None => {
                purchase.set(Some(Err("Purchase total overflow".into())));
                return;
            }
        };
        let input = CreateTransactionInput {
            seller: seller_for_submit.clone(),
            listing_hash: listing_hash_for_submit.clone(),
            quantity: requested_quantity,
            total_price_cents: total,
        };
        let client = MarketplaceClient::new(transport_for_submit.clone());
        submitting.set(true);
        purchase.set(None);
        spawn_local(async move {
            purchase.set(Some(
                client
                    .create_transaction(&input)
                    .await
                    .map_err(|error| error.to_string()),
            ));
            submitting.set(false);
        });
    };

    view! {
        <section class="detail-grid">
            <div class="detail-visual">"EVIDENCE IMAGE / IPFS RENDERER PENDING"</div>
            <article class="detail-copy">
                <p class="eyebrow">{category}</p>
                <h1>{title}</h1>
                <p class="detail-description">{description}</p>
                <strong class="price detail-price">{formatted_price}</strong>
                <dl class="facts">
                    <div><dt>"Listing"</dt><dd>{hash_label}</dd></div>
                    <div><dt>"Seller"</dt><dd>{seller_label}</dd></div>
                    <div><dt>"Inventory"</dt><dd>{available}</dd></div>
                    <div><dt>"Status"</dt><dd>{format!("{status:?}")}</dd></div>
                </dl>
                <div class="purchase-panel">
                    <label for="purchase-quantity">"Quantity"</label>
                    <input
                        id="purchase-quantity"
                        type="number"
                        min="1"
                        max=available.to_string()
                        prop:value=move || quantity.get().to_string()
                        on:input=move |event| {
                            let parsed = event_target_value(&event).parse::<u32>().unwrap_or(1);
                            quantity.set(parsed.clamp(1, available.max(1)));
                        }
                    />
                    <p class="purchase-total">
                        {move || format!("Total: ${}.{:02}",
                            unit_price.saturating_mul(u64::from(quantity.get())) / 100,
                            unit_price.saturating_mul(u64::from(quantity.get())) % 100,
                        )}
                    </p>
                    <button
                        class="primary-button"
                        on:click=submit_purchase
                        disabled=move || {
                            submitting.get()
                                || !connection.get().is_ready()
                                || !can_purchase_listing
                                || connection.get().agent_pub_key().as_ref() == Some(&seller_for_disable)
                        }
                    >
                        {move || if submitting.get() { "Creating…" } else { "Create purchase" }}
                    </button>
                    {move || if connection.get().agent_pub_key().as_ref() == Some(&seller_for_warning) {
                        view! { <p class="warning-copy">"You cannot purchase your own listing."</p> }.into_any()
                    } else {
                        ().into_any()
                    }}
                    {move || match purchase.get() {
                        None => ().into_any(),
                        Some(Ok(output)) => {
                            let href = format!("/transaction/{}", output.transaction_hash);
                            view! {
                                <div class="success-panel" role="status">
                                    <strong>"Pending transaction created"</strong>
                                    <p>"Open the transaction directly, then reload that route to verify DHT recovery."</p>
                                    <A href=href>"Open transaction"</A>
                                </div>
                            }.into_any()
                        }
                        Some(Err(message)) => view! {
                            <div class="error-panel" role="alert">
                                <strong>"Purchase rejected"</strong>
                                <p>{message}</p>
                            </div>
                        }.into_any(),
                    }}
                </div>
            </article>
        </section>
    }
}

#[component]
fn TransactionsPage() -> impl IntoView {
    let connection = expect_context::<RwSignal<ConnectionState>>();
    let transport = expect_context::<RuntimeTransport>();
    let transactions = RwSignal::new(None::<Result<Vec<TransactionResolution>, String>>);
    let requested = RwSignal::new(false);

    Effect::new(move |_| {
        if !connection.get().is_ready() || requested.get() {
            return;
        }
        requested.set(true);
        let client = MarketplaceClient::new(transport.clone());
        spawn_local(async move {
            transactions.set(Some(
                client
                    .get_my_transaction_resolutions()
                    .await
                    .map(|response| response.resolutions)
                    .map_err(|error| error.to_string()),
            ));
        });
    });

    view! {
        <section class="page-heading">
            <p class="eyebrow">"LIFECYCLE SLICE / 01"</p>
            <h1>"Your transactions"</h1>
            <p>
                "Every row is reduced from its original Create action to the single visible leaf. "
                "Concurrent leaves remain an explicit conflict and disable mutations."
            </p>
        </section>
        <ConnectionAvailability connection />
        {move || match transactions.get() {
            None => view! { <p class="muted">"Loading transaction revision trees…"</p> }.into_any(),
            Some(Ok(items)) if items.is_empty() => view! {
                <section class="availability compact">
                    <h2>"No transactions yet"</h2>
                    <p>"Create a purchase from an active listing."</p>
                    <A href="/">"Browse listings"</A>
                </section>
            }.into_any(),
            Some(Ok(items)) => view! {
                <div class="transaction-list">
                    {items.into_iter().map(|resolution| view! {
                        <TransactionResolutionCard resolution />
                    }).collect_view()}
                </div>
            }.into_any(),
            Some(Err(message)) => view! {
                <section class="availability unavailable" role="alert">
                    <h2>"Could not load transactions"</h2>
                    <p>{message}</p>
                </section>
            }.into_any(),
        }}
    }
}

#[component]
fn TransactionResolutionCard(resolution: TransactionResolution) -> impl IntoView {
    let href = format!("/transaction/{}", resolution.root_transaction_hash);
    let root_label = resolution.root_transaction_hash.to_string();
    let revision_count = resolution.revision_count;

    match resolution.current().cloned() {
        Some(output) => {
            let total = output.transaction.total_price_cents;
            let status = output.transaction.status.label();
            let materiality = format!("{:?}", output.transaction.epistemic.materiality);
            let projection_label = if resolution.is_auto_resolved() {
                format!("AUTO-RESOLVED · {}", resolution.reason.label())
            } else {
                format!("{} · {} REVISION(S)", status, revision_count)
            };
            let card_class = if resolution.is_auto_resolved() {
                "transaction-card auto-resolution-card"
            } else {
                "transaction-card"
            };
            view! {
                <article class=card_class>
                    <div>
                        <p class="eyebrow">{projection_label}</p>
                        <h2><A href=href>{root_label}</A></h2>
                        <p>{format!(
                            "{} · Quantity {} · ${}.{:02}",
                            status,
                            output.transaction.quantity,
                            total / 100,
                            total % 100,
                        )}</p>
                    </div>
                    <span class="evidence-chip">{materiality}</span>
                </article>
            }.into_any()
        }
        None => {
            let head_count = resolution.heads.len();
            view! {
                <article class="transaction-card conflict-card">
                    <div>
                        <p class="eyebrow conflict-copy">{format!("CONFLICT · {} HEADS", head_count)}</p>
                        <h2><A href=href>{root_label}</A></h2>
                        <p>"Concurrent transaction updates require explicit review; lifecycle actions are halted."</p>
                    </div>
                    <span class="evidence-chip conflict-copy">{format!("{} REVISIONS", revision_count)}</span>
                </article>
            }.into_any()
        }
    }
}

#[component]
fn TransactionRoute() -> impl IntoView {
    let connection = expect_context::<RwSignal<ConnectionState>>();
    let transport = expect_context::<RuntimeTransport>();
    let detail = RwSignal::new(None::<Result<Option<TransactionResolution>, String>>);
    let requested = RwSignal::new(false);
    let transaction_hash = route_action_hash("transaction");

    Effect::new(move |_| {
        if !connection.get().is_ready() || requested.get() {
            return;
        }
        requested.set(true);
        match transaction_hash.clone() {
            Ok(hash) => {
                let client = MarketplaceClient::new(transport.clone());
                spawn_local(async move {
                    detail.set(Some(
                        client
                            .get_transaction_resolution(&hash)
                            .await
                            .map_err(|error| error.to_string()),
                    ));
                });
            }
            Err(message) => detail.set(Some(Err(message))),
        }
    });

    view! {
        <ConnectionAvailability connection />
        {move || match detail.get() {
            None => view! { <p class="muted">"Reducing transaction revision tree…"</p> }.into_any(),
            Some(Ok(Some(resolution))) => view! { <TransactionDetail initial=resolution /> }.into_any(),
            Some(Ok(None)) => view! {
                <section class="availability unavailable">
                    <h1>"Transaction not found"</h1>
                    <A href="/transactions">"Return to transactions"</A>
                </section>
            }.into_any(),
            Some(Err(message)) => view! {
                <section class="availability unavailable" role="alert">
                    <h1>"Could not recover transaction"</h1>
                    <p>{message}</p>
                </section>
            }.into_any(),
        }}
    }
}

#[component]
fn TransactionDetail(initial: TransactionResolution) -> impl IntoView {
    let resolution = RwSignal::new(initial);

    move || {
        let snapshot = resolution.get();
        match snapshot.current().cloned() {
            Some(output) => view! {
                <ResolvedTransactionDetail
                    root=snapshot.root_transaction_hash
                    output
                    revision_count=snapshot.revision_count
                    projection_state=snapshot.state
                    projection_reason=snapshot.reason
                    superseded_heads=snapshot.superseded_heads
                    applied_conflict_resolutions=snapshot.applied_conflict_resolutions
                    resolution
                />
            }.into_any(),
            None => view! { <TransactionConflict resolution /> }.into_any(),
        }
    }
}

#[component]
fn TransactionConflict(resolution: RwSignal<TransactionResolution>) -> impl IntoView {
    let connection = expect_context::<RwSignal<ConnectionState>>();
    let transport = expect_context::<RuntimeTransport>();
    let approvals = RwSignal::new(None::<Result<Vec<TransactionConflictApprovalOutput>, String>>);
    let selected_head = RwSignal::new(None::<ActionHash>);
    let rationale = RwSignal::new(String::new());
    let pending = RwSignal::new(false);
    let feedback = RwSignal::new(None::<Result<String, String>>);

    let initial_root = resolution.get_untracked().root_transaction_hash;
    request_conflict_approvals(transport.clone(), initial_root, approvals);

    move || {
        let snapshot = resolution.get();
        let root_hash = snapshot.root_transaction_hash.clone();
        let root = root_hash.to_string();
        let revision_count = snapshot.revision_count;
        let heads = snapshot.heads.clone();
        let current_agent = connection.get().agent_pub_key();
        let buyer = heads.first().map(|head| head.transaction.buyer.clone());
        let seller = heads.first().map(|head| head.transaction.seller.clone());
        let is_party = current_agent
            .as_ref()
            .is_some_and(|agent| Some(agent) == buyer.as_ref() || Some(agent) == seller.as_ref());

        let approval_values = approvals
            .get()
            .and_then(Result::ok)
            .unwrap_or_default();
        let buyer_approval = buyer.as_ref().and_then(|buyer| {
            approval_values
                .iter()
                .find(|output| &output.approval.approver == buyer)
                .cloned()
        });
        let seller_approval = seller.as_ref().and_then(|seller| {
            approval_values
                .iter()
                .find(|output| &output.approval.approver == seller)
                .cloned()
        });
        let bilateral_pair = match (buyer_approval.clone(), seller_approval.clone()) {
            (Some(buyer), Some(seller))
                if buyer.approval.head_hashes == seller.approval.head_hashes
                    && buyer.approval.selected_head_hash
                        == seller.approval.selected_head_hash => Some((buyer, seller)),
            _ => None,
        };

        let refresh_transport = transport.clone();
        let refresh_root = root_hash.clone();
        let approve_transport = transport.clone();
        let approve_root = root_hash.clone();
        let finalize_transport = transport.clone();
        let finalize_root = root_hash.clone();
        let arbitration_transport = transport.clone();
        let arbitration_root = root_hash.clone();

        view! {
            <article class="transaction-detail conflict-panel" role="alert">
                <p class="eyebrow conflict-copy">"CONCURRENT UPDATE CONFLICT"</p>
                <h1>"Lifecycle halted"</h1>
                <p class="hash-copy">{root}</p>
                <p>
                    "More than one live update leaf is visible and no narrow safety rule applies. "
                    "Marketplace will not select a winner or permit another lifecycle mutation without explicit authority."
                </p>
                <p class="muted">{format!("{} records traversed · {} visible heads", revision_count, heads.len())}</p>
                <div class="conflict-heads">
                    {heads.into_iter().map(|head| {
                        let total = head.transaction.total_price_cents;
                        let head_hash = head.transaction_hash.clone();
                        let selected_for_class = head_hash.clone();
                        let selected_for_click = head_hash.clone();
                        view! {
                            <section class="conflict-head">
                                <p class="eyebrow">{head.transaction.status.label()}</p>
                                <p class="hash-copy">{head_hash.to_string()}</p>
                                <p>{format!("Quantity {} · ${}.{:02}", head.transaction.quantity, total / 100, total % 100)}</p>
                                {is_party.then(|| view! {
                                    <button
                                        class=move || if selected_head.get().as_ref() == Some(&selected_for_class) {
                                            "secondary-button selected-branch"
                                        } else {
                                            "secondary-button"
                                        }
                                        disabled=move || pending.get()
                                        on:click=move |_| selected_head.set(Some(selected_for_click.clone()))
                                    >
                                        "Select this authored branch"
                                    </button>
                                })}
                            </section>
                        }
                    }).collect_view()}
                </div>

                <section class="evidence-panel" aria-labelledby="authority-heading">
                    <p class="eyebrow">"EXPLICIT AUTHORITY"</p>
                    <h2 id="authority-heading">"Resolve without erasing branch history"</h2>
                    <p>
                        "Buyer and seller may independently approve the same existing head. "
                        "A final bilateral record is valid only when both approvals bind this exact head set."
                    </p>
                    {is_party.then(|| view! {
                        <label for="conflict-rationale">"Rationale or arbitration reason"</label>
                        <textarea
                            id="conflict-rationale"
                            maxlength="2000"
                            prop:value=move || rationale.get()
                            on:input=move |event| rationale.set(event_target_value(&event))
                        ></textarea>
                        <div class="button-row">
                            <button
                                class="primary-button"
                                disabled=move || pending.get()
                                    || selected_head.get().is_none()
                                    || rationale.get().trim().is_empty()
                                on:click=move |_| {
                                    if let Some(selected) = selected_head.get_untracked() {
                                        execute_conflict_approval(
                                            approve_transport.clone(),
                                            approve_root.clone(),
                                            selected,
                                            rationale.get_untracked(),
                                            approvals,
                                            pending,
                                            feedback,
                                        );
                                    }
                                }
                            >
                                {move || if pending.get() { "Publishing…" } else { "Publish my approval" }}
                            </button>
                            <button
                                class="secondary-button"
                                disabled=move || pending.get() || rationale.get().trim().is_empty()
                                on:click=move |_| execute_conflict_arbitration(
                                    arbitration_transport.clone(),
                                    arbitration_root.clone(),
                                    rationale.get_untracked(),
                                    pending,
                                    feedback,
                                )
                            >
                                "Open conflict-bound arbitration"
                            </button>
                        </div>
                    })}

                    {move || match approvals.get() {
                        None => view! { <p class="muted">"Loading authority evidence…"</p> }.into_any(),
                        Some(Err(message)) => view! { <p class="error-copy">{message}</p> }.into_any(),
                        Some(Ok(items)) if items.is_empty() => view! {
                            <p class="muted">"No party has published an approval for this conflict."</p>
                        }.into_any(),
                        Some(Ok(items)) => view! {
                            <div class="conflict-heads">
                                {items.into_iter().map(|output| view! {
                                    <section class="conflict-head authority-record">
                                        <p class="eyebrow">"PARTY APPROVAL"</p>
                                        <p>{format!("Approver: {}", output.approval.approver)}</p>
                                        <p>{format!("Selected: {}", output.approval.selected_head_hash)}</p>
                                        <p>{output.approval.rationale}</p>
                                        <p class="hash-copy">{output.approval_hash.to_string()}</p>
                                    </section>
                                }).collect_view()}
                            </div>
                        }.into_any(),
                    }}

                    {bilateral_pair.map(|(buyer, seller)| {
                        let transport = finalize_transport.clone();
                        let root = finalize_root.clone();
                        view! {
                            <div class="gated-action">
                                <p>"Matching buyer and seller approvals are visible for one exact branch."</p>
                                <button
                                    class="primary-button"
                                    disabled=move || pending.get() || !is_party
                                    on:click=move |_| execute_bilateral_resolution(
                                        transport.clone(),
                                        root.clone(),
                                        buyer.approval_hash.clone(),
                                        seller.approval_hash.clone(),
                                        resolution,
                                        pending,
                                        feedback,
                                    )
                                >
                                    "Finalize bilateral resolution"
                                </button>
                            </div>
                        }
                    })}

                    <button
                        class="secondary-button"
                        disabled=move || pending.get()
                        on:click=move |_| request_conflict_approvals(
                            refresh_transport.clone(),
                            refresh_root.clone(),
                            approvals,
                        )
                    >
                        "Refresh authority evidence"
                    </button>
                </section>

                {move || match feedback.get() {
                    None => ().into_any(),
                    Some(Ok(message)) => view! { <div class="success-panel" role="status"><p>{message}</p></div> }.into_any(),
                    Some(Err(message)) => view! { <div class="error-panel" role="alert"><p>{message}</p></div> }.into_any(),
                }}
                <A href="/transactions">"Back to transactions"</A>
            </article>
        }
    }
}

fn request_conflict_approvals(
    transport: RuntimeTransport,
    root: ActionHash,
    approvals: RwSignal<Option<Result<Vec<TransactionConflictApprovalOutput>, String>>>,
) {
    spawn_local(async move {
        let client = MarketplaceClient::new(transport);
        approvals.set(Some(
            client
                .get_transaction_conflict_approvals(&root)
                .await
                .map_err(|error| error.to_string()),
        ));
    });
}

fn execute_conflict_approval(
    transport: RuntimeTransport,
    root: ActionHash,
    selected_head_hash: ActionHash,
    rationale: String,
    approvals: RwSignal<Option<Result<Vec<TransactionConflictApprovalOutput>, String>>>,
    pending: RwSignal<bool>,
    feedback: RwSignal<Option<Result<String, String>>>,
) {
    if pending.get_untracked() {
        return;
    }
    pending.set(true);
    feedback.set(None);
    spawn_local(async move {
        let client = MarketplaceClient::new(transport.clone());
        let result = client
            .approve_transaction_conflict(&ApproveTransactionConflictInput {
                transaction_hash: root.clone(),
                selected_head_hash,
                rationale,
            })
            .await;
        match result {
            Ok(output) => {
                feedback.set(Some(Ok(format!(
                    "Approval published as {}. It does not resolve the conflict by itself.",
                    output.approval_hash
                ))));
                request_conflict_approvals(transport, root, approvals);
            }
            Err(error) => feedback.set(Some(Err(error.to_string()))),
        }
        pending.set(false);
    });
}

fn execute_bilateral_resolution(
    transport: RuntimeTransport,
    root: ActionHash,
    buyer_approval_hash: ActionHash,
    seller_approval_hash: ActionHash,
    resolution: RwSignal<TransactionResolution>,
    pending: RwSignal<bool>,
    feedback: RwSignal<Option<Result<String, String>>>,
) {
    if pending.get_untracked() {
        return;
    }
    pending.set(true);
    feedback.set(None);
    spawn_local(async move {
        let client = MarketplaceClient::new(transport);
        match client
            .finalize_bilateral_transaction_conflict(
                &FinalizeBilateralTransactionConflictInput {
                    transaction_hash: root,
                    buyer_approval_hash,
                    seller_approval_hash,
                    summary: "Buyer and seller selected the same authored transaction branch".into(),
                },
            )
            .await
        {
            Ok(next) if next.is_authorized_resolved() => {
                resolution.set(next);
                feedback.set(Some(Ok(
                    "Bilateral authority accepted. Original branches remain visible as evidence."
                        .into(),
                )));
            }
            Ok(next) => {
                resolution.set(next);
                feedback.set(Some(Err(
                    "Authority was recorded, but the transaction did not produce one safe projection."
                        .into(),
                )));
            }
            Err(error) => feedback.set(Some(Err(error.to_string()))),
        }
        pending.set(false);
    });
}

fn execute_conflict_arbitration(
    transport: RuntimeTransport,
    root: ActionHash,
    reason: String,
    pending: RwSignal<bool>,
    feedback: RwSignal<Option<Result<String, String>>>,
) {
    if pending.get_untracked() {
        return;
    }
    pending.set(true);
    feedback.set(None);
    spawn_local(async move {
        let client = MarketplaceClient::new(transport);
        match client
            .file_transaction_conflict_dispute(&FileDisputeInput {
                transaction_hash: root,
                reason,
                evidence_cids: Vec::new(),
            })
            .await
        {
            Ok(dispute) => feedback.set(Some(Ok(format!(
                "Conflict-bound arbitration opened at {}. Assigned arbitrators may now vote through the arbitration client.",
                dispute.dispute_hash
            )))),
            Err(error) => feedback.set(Some(Err(error.to_string()))),
        }
        pending.set(false);
    });
}

#[derive(Clone)]
enum LifecycleCommand {
    Confirm,
    Ship(String),
    Deliver,
    Cancel,
}

impl LifecycleCommand {
    const fn success_message(&self) -> &'static str {
        match self {
            Self::Confirm => "Transaction confirmed from the latest revision.",
            Self::Ship(_) => "Shipment recorded from the latest revision.",
            Self::Deliver => "Delivery confirmed from the latest revision.",
            Self::Cancel => "Transaction cancelled from the latest revision.",
        }
    }
}

fn execute_lifecycle(
    transport: RuntimeTransport,
    root: ActionHash,
    command: LifecycleCommand,
    resolution: RwSignal<TransactionResolution>,
    pending: RwSignal<bool>,
    feedback: RwSignal<Option<Result<String, String>>>,
) {
    if pending.get_untracked() {
        return;
    }
    pending.set(true);
    feedback.set(None);

    spawn_local(async move {
        let client = MarketplaceClient::new(transport);
        let success_message = command.success_message().to_string();
        let mutation = match command {
            LifecycleCommand::Confirm => client.confirm_transaction(&root).await,
            LifecycleCommand::Ship(tracking_info) => {
                client
                    .mark_shipped(&MarkShippedInput {
                        transaction_hash: root.clone(),
                        tracking_info: Some(tracking_info),
                    })
                    .await
            }
            LifecycleCommand::Deliver => client.confirm_delivery(&root).await,
            LifecycleCommand::Cancel => client.cancel_transaction(&root).await,
        };

        match mutation {
            Err(error) => feedback.set(Some(Err(error.to_string()))),
            Ok(_) => match client.get_transaction_resolution(&root).await {
                Ok(Some(next)) if next.is_conflicted() => {
                    resolution.set(next);
                    feedback.set(Some(Err(
                        "The mutation exposed concurrent heads; further actions are halted.".into(),
                    )));
                }
                Ok(Some(next)) => {
                    resolution.set(next);
                    feedback.set(Some(Ok(success_message)));
                }
                Ok(None) => feedback.set(Some(Err(
                    "Mutation succeeded, but the transaction could not be recovered.".into(),
                ))),
                Err(error) => feedback.set(Some(Err(format!(
                    "Mutation succeeded, but refresh failed: {error}"
                )))),
            },
        }
        pending.set(false);
    });
}

fn execute_settlement(
    transport: RuntimeTransport,
    root: ActionHash,
    settlement: RwSignal<Option<TransactionSettlementResult>>,
    pending: RwSignal<bool>,
    feedback: RwSignal<Option<Result<String, String>>>,
) {
    pending.set(true);
    feedback.set(None);
    spawn_local(async move {
        let client = MarketplaceClient::new(transport);
        match client.settle_transaction(&root).await {
            Ok(result) if result.settled => {
                settlement.set(Some(result));
                feedback.set(Some(Ok(
                    "Finance returned a matching Completed payment record.".into(),
                )));
            }
            Ok(result) => {
                let message = result
                    .error
                    .clone()
                    .unwrap_or_else(|| format!("Finance settlement state: {}", result.state.label()));
                settlement.set(Some(result));
                feedback.set(Some(Err(message)));
            }
            Err(error) => feedback.set(Some(Err(error.to_string()))),
        }
        pending.set(false);
    });
}

#[component]
fn ResolvedTransactionDetail(
    root: ActionHash,
    output: TransactionOutput,
    revision_count: u32,
    projection_state: TransactionResolutionState,
    projection_reason: TransactionResolutionReason,
    superseded_heads: Vec<TransactionOutput>,
    applied_conflict_resolutions: Vec<marketplace_domain::AppliedTransactionConflictResolution>,
    resolution: RwSignal<TransactionResolution>,
) -> impl IntoView {
    let connection = expect_context::<RwSignal<ConnectionState>>();
    let transport = expect_context::<RuntimeTransport>();
    let pending = RwSignal::new(false);
    let feedback = RwSignal::new(None::<Result<String, String>>);
    let tracking = RwSignal::new(output.transaction.tracking_info.clone().unwrap_or_default());
    let settlement = RwSignal::new(None::<TransactionSettlementResult>);
    let seller_reputation = RwSignal::new(None::<DerivedReputation>);

    let transaction = output.transaction;
    let current_agent = connection.get_untracked().agent_pub_key();
    let is_buyer = current_agent.as_ref() == Some(&transaction.buyer);
    let is_seller = current_agent.as_ref() == Some(&transaction.seller);
    let can_confirm = is_seller && transaction.status == TransactionStatus::Pending;
    let can_ship = is_seller && transaction.status == TransactionStatus::Confirmed;
    let can_deliver = is_buyer && transaction.status == TransactionStatus::Shipped;
    let can_cancel = (is_buyer || is_seller)
        && matches!(
            &transaction.status,
            TransactionStatus::Pending | TransactionStatus::Confirmed
        );
    let show_completion_gate = transaction.status == TransactionStatus::Delivered;
    let finance_role_available = connection.get_untracked().has_active_role("finance");
    let show_dispute_gate = !transaction.status.is_terminal();
    let total = transaction.total_price_cents;

    if transaction.status == TransactionStatus::Delivered && (is_buyer || is_seller) {
        let status_transport = transport.clone();
        let status_root = root.clone();
        spawn_local(async move {
            let client = MarketplaceClient::new(status_transport);
            if let Ok(result) = client.get_transaction_settlement_status(&status_root).await {
                settlement.set(Some(result));
            }
        });
    }
    let reputation_transport = transport.clone();
    let reputation_seller = transaction.seller.clone();
    spawn_local(async move {
        let client = MarketplaceClient::new(reputation_transport);
        if let Ok(result) = client.get_derived_reputation(&reputation_seller).await {
            seller_reputation.set(Some(result));
        }
    });

    let confirm_transport = transport.clone();
    let confirm_root = root.clone();
    let ship_transport = transport.clone();
    let ship_root = root.clone();
    let deliver_transport = transport.clone();
    let deliver_root = root.clone();
    let cancel_transport = transport.clone();
    let cancel_root = root.clone();
    let settlement_transport = transport.clone();
    let settlement_root = root.clone();

    view! {
        <article class="transaction-detail">
            <p class="eyebrow">{format!("RESOLVED · {} REVISION(S)", revision_count)}</p>
            <h1>{transaction.status.label()}</h1>
            <p class="hash-copy">{root.to_string()}</p>
            <p class="muted">{format!("Current leaf: {}", output.transaction_hash)}</p>
            {matches!(projection_state, TransactionResolutionState::AutoResolved).then(|| view! {
                <section class="evidence-panel auto-resolution-panel" role="status" aria-labelledby="auto-resolution-heading">
                    <p class="eyebrow">"SAFETY-DOMINANT PROJECTION"</p>
                    <h2 id="auto-resolution-heading">{projection_reason.label()}</h2>
                    <p>{projection_reason.explanation()}</p>
                    <p class="muted">
                        "No branch was deleted. The canonical state is an existing authored head; every safely superseded head remains visible below."
                    </p>
                    <div class="conflict-heads">
                        {superseded_heads.into_iter().map(|head| view! {
                            <section class="conflict-head superseded-head">
                                <p class="eyebrow">{format!("SUPERSEDED · {}", head.transaction.status.label())}</p>
                                <p class="hash-copy">{head.transaction_hash.to_string()}</p>
                            </section>
                        }).collect_view()}
                    </div>
                </section>
            })}
            {matches!(projection_state, TransactionResolutionState::AuthorizedResolved).then(|| view! {
                <section class="evidence-panel auto-resolution-panel" role="status" aria-labelledby="authorized-resolution-heading">
                    <p class="eyebrow">"EXPLICITLY AUTHORIZED PROJECTION"</p>
                    <h2 id="authorized-resolution-heading">{projection_reason.label()}</h2>
                    <p>{projection_reason.explanation()}</p>
                    <p class="muted">
                        "The canonical state is an existing authored branch. Every original conflict head and authority record remains auditable."
                    </p>
                    <div class="conflict-heads">
                        {applied_conflict_resolutions.into_iter().map(|evidence| view! {
                            <section class="conflict-head authority-record">
                                <p class="eyebrow">{evidence.authority.label()}</p>
                                <p>{format!("Selected branch: {}", evidence.selected_head_hash)}</p>
                                <p>{format!("Bound heads: {}", evidence.bound_head_hashes.len())}</p>
                                <p class="hash-copy">{evidence.resolution_hash.to_string()}</p>
                            </section>
                        }).collect_view()}
                    </div>
                </section>
            })}
            <dl class="facts">
                <div><dt>"Listing"</dt><dd>{transaction.listing_hash.to_string()}</dd></div>
                <div><dt>"Buyer"</dt><dd>{transaction.buyer.to_string()}</dd></div>
                <div><dt>"Seller"</dt><dd>{transaction.seller.to_string()}</dd></div>
                <div><dt>"Quantity"</dt><dd>{transaction.quantity}</dd></div>
                <div><dt>"Total"</dt><dd>{format!("${}.{:02}", total / 100, total % 100)}</dd></div>
                <div><dt>"Tracking"</dt><dd>{transaction.tracking_info.unwrap_or_else(|| "Not recorded".into())}</dd></div>
            </dl>

            {move || seller_reputation.get().map(|reputation| view! {
                <section class="evidence-panel" aria-labelledby="seller-evidence-heading">
                    <p class="eyebrow">"EVIDENCE-DERIVED SELLER HISTORY"</p>
                    <h2 id="seller-evidence-heading">
                        {format!("{} validated event(s)", reputation.event_count)}
                    </h2>
                    <p>
                        {format!(
                            "{} positive · {} negative · delivered value {} cents",
                            reputation.positive_events,
                            reputation.negative_events,
                            reputation.fulfilled_value_cents,
                        )}
                    </p>
                    <p class="muted">
                        "The displayed ratio is a transparent evidence summary, not a Byzantine-tolerance claim."
                    </p>
                </section>
            })}

            <section class="lifecycle-panel" aria-labelledby="lifecycle-heading">
                <p class="eyebrow">"AUTHORIZED NEXT ACTION"</p>
                <h2 id="lifecycle-heading">"Transaction lifecycle"</h2>

                {can_confirm.then(|| {
                    let transport = confirm_transport.clone();
                    let root = confirm_root.clone();
                    view! {
                        <button
                            class="primary-button"
                            disabled=move || pending.get()
                            on:click=move |_| execute_lifecycle(
                                transport.clone(),
                                root.clone(),
                                LifecycleCommand::Confirm,
                                resolution,
                                pending,
                                feedback,
                            )
                        >
                            {move || if pending.get() { "Updating…" } else { "Confirm transaction" }}
                        </button>
                    }
                })}

                {can_ship.then(|| {
                    let transport = ship_transport.clone();
                    let root = ship_root.clone();
                    view! {
                        <label for="tracking-info">"Tracking information"</label>
                        <input
                            id="tracking-info"
                            maxlength="256"
                            prop:value=move || tracking.get()
                            on:input=move |event| tracking.set(event_target_value(&event))
                        />
                        <button
                            class="primary-button"
                            disabled=move || pending.get() || tracking.get().trim().is_empty()
                            on:click=move |_| execute_lifecycle(
                                transport.clone(),
                                root.clone(),
                                LifecycleCommand::Ship(tracking.get_untracked().trim().to_string()),
                                resolution,
                                pending,
                                feedback,
                            )
                        >
                            {move || if pending.get() { "Updating…" } else { "Mark shipped" }}
                        </button>
                    }
                })}

                {can_deliver.then(|| {
                    let transport = deliver_transport.clone();
                    let root = deliver_root.clone();
                    view! {
                        <button
                            class="primary-button"
                            disabled=move || pending.get()
                            on:click=move |_| execute_lifecycle(
                                transport.clone(),
                                root.clone(),
                                LifecycleCommand::Deliver,
                                resolution,
                                pending,
                                feedback,
                            )
                        >
                            {move || if pending.get() { "Updating…" } else { "Confirm delivery" }}
                        </button>
                    }
                })}

                {can_cancel.then(|| {
                    let transport = cancel_transport.clone();
                    let root = cancel_root.clone();
                    view! {
                        <button
                            class="secondary-button danger-button"
                            disabled=move || pending.get()
                            on:click=move |_| execute_lifecycle(
                                transport.clone(),
                                root.clone(),
                                LifecycleCommand::Cancel,
                                resolution,
                                pending,
                                feedback,
                            )
                        >
                            "Cancel transaction"
                        </button>
                    }
                })}

                {show_completion_gate.then(|| {
                    let transport = settlement_transport.clone();
                    let root = settlement_root.clone();
                    view! {
                        <div class="gated-action">
                            <button
                                class="secondary-button"
                                disabled=move || pending.get()
                                    || !is_buyer
                                    || !cfg!(feature = "finance-settlement")
                                    || !finance_role_available
                                on:click=move |_| execute_settlement(
                                    transport.clone(),
                                    root.clone(),
                                    settlement,
                                    pending,
                                    feedback,
                                )
                            >
                                {move || if pending.get() { "Settling…" } else { "Settle with Finance" }}
                            </button>
                            <p>
                                {if !cfg!(feature = "finance-settlement") {
                                    "Settlement is build-gated until the Finance role and retry-safety scenario are included in the deployed hApp."
                                } else if !finance_role_available {
                                    "This build permits settlement, but the connected hApp has no active Finance role. No payment call will be attempted."
                                } else {
                                    "Settlement uses the stable transaction root as an idempotency key. Marketplace remains Delivered; Finance owns economic finality."
                                }}
                            </p>
                            {move || settlement.get().map(|result| view! {
                                <div class="settlement-evidence" role="status">
                                    <strong>{result.state.label()}</strong>
                                    <p>{format!("Reference: {}", result.idempotency_reference)}</p>
                                    {result.finance_payment_id.map(|id| view! { <p>{format!("Finance payment: {id}")}</p> })}
                                    {result.error.map(|error| view! { <p>{error}</p> })}
                                </div>
                            })}
                        </div>
                    }
                })}

                {show_dispute_gate.then(|| view! {
                    <div class="gated-action">
                        <button class="secondary-button" disabled=true>"Open dispute"</button>
                        <p>
                            "Dispute creation remains UI-gated until the three-agent conductor scenario passes with "
                            "the immutable arbitration reputation projection enabled."
                        </p>
                    </div>
                })}

                {move || match feedback.get() {
                    None => ().into_any(),
                    Some(Ok(message)) => view! {
                        <div class="success-panel" role="status"><p>{message}</p></div>
                    }.into_any(),
                    Some(Err(message)) => view! {
                        <div class="error-panel" role="alert"><p>{message}</p></div>
                    }.into_any(),
                }}
            </section>
            <A href="/transactions">"Back to transactions"</A>
        </article>
    }
}

fn route_action_hash(kind: &str) -> Result<ActionHash, String> {
    let window = web_sys::window().ok_or_else(|| "browser window unavailable".to_string())?;
    let path = window
        .location()
        .pathname()
        .map_err(|_| "could not read browser route".to_string())?;
    let mut segments = path.split('/').filter(|segment| !segment.is_empty());
    let route_kind = segments.next().unwrap_or_default();
    let encoded = segments.next().unwrap_or_default();
    if route_kind != kind || encoded.is_empty() {
        return Err(format!("expected /{kind}/<ActionHash> route"));
    }
    ActionHash::from_str(encoded).map_err(|error| error.to_string())
}

#[component]
fn NotFound() -> impl IntoView {
    view! {
        <section class="availability">
            <p class="eyebrow">"404"</p>
            <h1>"Route not found"</h1>
            <A href="/">"Return to Marketplace"</A>
        </section>
    }
}
