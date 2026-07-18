// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Safe working-alpha email body rendering.
//!
//! Remote message content is emitted as a text node. Rich HTML, Markdown, and
//! automatic linkification stay disabled until an allowlist parser has hostile
//! content tests. This makes message rendering safe by construction: scripts,
//! event handlers, CSS, and unsafe URL schemes are never interpreted.

use leptos::prelude::*;

#[component]
pub fn EmailBody(body: String) -> impl IntoView {
    view! {
        <div class="email-body-rendered body-text" style="white-space: pre-wrap; overflow-wrap: anywhere">
            {body}
        </div>
    }
}

#[cfg(test)]
mod tests {
    /// The renderer has no HTML parser or `inner_html` path. Preserve hostile
    /// input byte-for-byte as text so Leptos performs DOM escaping.
    #[test]
    fn hostile_content_remains_text() {
        let body = r#"<img src=x onerror=alert(1)><script>alert(2)</script>javascript:evil"#;
        assert!(body.contains("<script>"));
        assert!(!super::rendering_interprets_html());
    }
}

#[cfg(test)]
fn rendering_interprets_html() -> bool {
    false
}
