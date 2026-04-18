use leptos::prelude::*;
use crate::mock_data;

#[component]
pub fn CredentialsPage() -> impl IntoView {
    let credentials = mock_data::mock_credentials();
    let trust = mock_data::mock_trust_credential();

    view! {
        <div class="page credentials-page">
            <h1 class="page-title">"credential wallet"</h1>
            <p class="page-subtitle">"trust credentials and k-vector"</p>

            // Trust credential
            <section>
                <h2>"K-Vector Trust"</h2>
                <div class="trust-credential-card">
                    <div class="trust-header">
                        <span class=format!("tier-badge {}", trust.trust_tier.css_class())>
                            {trust.trust_tier.label()}
                        </span>
                        <span class="trust-subject">{trust.subject_did.clone()}</span>
                    </div>
                    <div class="trust-details">
                        <span>"Issuer: " {trust.issuer_did.clone()}</span>
                        <span>{format!("Range: {:.0}% – {:.0}%",
                            trust.trust_score_range.lower * 100.0,
                            trust.trust_score_range.upper * 100.0)}</span>
                    </div>
                </div>
            </section>

            // Stored credentials
            <section>
                <h2>"Stored Credentials"</h2>
                <div class="credential-list">
                    {credentials.iter().map(|c| {
                        let ctype = c.credential_type.label().to_string();
                        let issuer = c.issuer.clone();
                        let status = if c.revoked { "Revoked" } else { "Active" };
                        let status_class = if c.revoked { "cred-revoked" } else { "cred-active" };
                        view! {
                            <div class="credential-card">
                                <div class="credential-header">
                                    <span class="credential-type">{ctype}</span>
                                    <span class=format!("credential-status {status_class}")>{status}</span>
                                </div>
                                <span class="credential-issuer">{issuer}</span>
                            </div>
                        }
                    }).collect_view()}
                </div>
            </section>
        </div>
    }
}
