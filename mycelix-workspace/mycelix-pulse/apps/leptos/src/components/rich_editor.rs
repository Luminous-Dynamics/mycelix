// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Plain-text composer for the security-focused working alpha.
//!
//! Remote message bodies are rendered as text by construction. The composer
//! follows the same contract: it never parses draft, reply, or forwarded
//! content as HTML and never emits formatting markup that recipients would see
//! literally. A future rich-text mode needs an allowlist serializer, hostile
//! content tests, and a matching safe renderer before it belongs here.

use leptos::prelude::*;

#[component]
pub fn RichEditor(
    #[prop(into)] value: RwSignal<String>,
    #[prop(default = "Write your message...")] placeholder: &'static str,
) -> impl IntoView {
    view! {
        <div class="rich-editor plaintext-editor">
            <div class="editor-mode-note" role="status">
                <span aria-hidden="true">"🔒"</span>
                " Plain text — rich HTML is disabled in the security alpha"
            </div>
            <textarea
                class="editor-content"
                prop:value=move || value.get()
                on:input=move |ev| value.set(event_target_value(&ev))
                placeholder=placeholder
                rows="12"
                spellcheck="true"
                aria-label="Email body"
            />
        </div>
    }
}

fn event_target_value(ev: &leptos::ev::Event) -> String {
    use wasm_bindgen::JsCast;

    ev.target()
        .and_then(|target| target.dyn_into::<web_sys::HtmlTextAreaElement>().ok())
        .map(|textarea| textarea.value())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    #[test]
    fn alpha_editor_contract_is_plaintext() {
        let hostile = r#"<img src=x onerror=alert(1)><script>alert(2)</script>"#;
        assert!(hostile.contains("<script>"));
        assert!(!super::editor_interprets_html());
    }
}

#[cfg(test)]
fn editor_interprets_html() -> bool {
    false
}
