use crate::domain::{SecurityState, WorkspaceContext};
use leptos::prelude::*;

#[component]
pub fn SecurityBadge(state: SecurityState) -> impl IntoView {
    view! {
        <span class=format!("badge {}", state.css_key())>
            {state.label()}
        </span>
    }
}

#[component]
pub fn AuthorityNotice(context: WorkspaceContext) -> impl IntoView {
    if context.is_authority_bound() {
        view! {
            <div class="authority-notice bound" role="status">
                <strong>"Authority context bound"</strong>
                <span>"Tenant, legal entity, principal, profile and policy version are present."</span>
            </div>
        }.into_view()
    } else {
        view! {
            <div class="authority-notice demo" role="status">
                <strong>"Demonstration context"</strong>
                <span>"No live authority binding is present. Financial values and actions on this screen are non-production."</span>
            </div>
        }.into_view()
    }
}

#[component]
pub fn SecurityLegend() -> impl IntoView {
    view! {
        <details class="security-legend">
            <summary>"How Mycelix represents state"</summary>
            <div class="legend-grid">
                <SecurityBadge state=SecurityState::Observed/>
                <span>"source observation, not yet verified"</span>

                <SecurityBadge state=SecurityState::Verified/>
                <span>"validated against the relevant evidence rule"</span>

                <SecurityBadge state=SecurityState::Qualified/>
                <span>"passed the applicable qualification policy"</span>

                <SecurityBadge state=SecurityState::Authorized/>
                <span>"authority granted; execution may still be pending"</span>

                <SecurityBadge state=SecurityState::Included/>
                <span>"accepted by the external rail, not necessarily final"</span>

                <SecurityBadge state=SecurityState::Finalized/>
                <span>"the configured finality condition is satisfied"</span>

                <SecurityBadge state=SecurityState::Reconciled/>
                <span>"the relevant records agree under reconciliation policy"</span>

                <SecurityBadge state=SecurityState::Indeterminate/>
                <span>"outcome cannot safely be classified; do not treat as success"</span>
            </div>
        </details>
    }
}
