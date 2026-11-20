// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Prism — Tauri v2 desktop app.
//!
//! The Local Hub: direct URL fetching (no CORS), bundled search engine,
//! native filesystem access for persistent storage.
//!
//! Tauri commands are called from the Leptos WASM frontend via IPC.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use prism_common::{ContentZone, SafetyLevel};
use prism_dom::parse_html;
use prism_privacy::ConsentStore;
use prism_reflex::ReflexArc;
use prism_search::SearchEngine;
use serde::{Deserialize, Serialize};
use std::sync::Mutex;
use tauri::State;

const MAX_NAVIGATION_URL_BYTES: usize = 8 * 1024;
const MAX_NAVIGATION_RESPONSE_BYTES: usize = 10 * 1024 * 1024;

/// Shared application state managed by Tauri.
struct PrismState {
    search: SearchEngine,
    reflex: ReflexArc,
    client: reqwest::Client,
}

/// Result of fetching and analyzing a URL.
#[derive(Serialize)]
struct FetchResult {
    html: String,
    title: String,
    zone: String,
    safety: String,
    threat_count: usize,
}

/// Search result for the frontend.
#[derive(Serialize)]
struct SearchResultJson {
    content: String,
    sources: Vec<String>,
    empirical_level: String,
    query_similarity: f32,
    author_reputation: f32,
    tags: Vec<String>,
    rank_score: f32,
}

/// User-initiated desktop navigation. Local/LAN destinations and redirects
/// remain available by design; hosted and background fetches must use the
/// PublicEgressClient policy instead.
#[tauri::command]
async fn fetch_url(
    url: String,
    state: State<'_, Mutex<PrismState>>,
) -> Result<FetchResult, String> {
    let parsed_url = validate_user_navigation_url(&url)?;

    // Clone the client before dropping the lock (reqwest::Client is cheaply cloneable).
    let client = {
        let s = state.lock().map_err(|e| format!("Lock error: {}", e))?;
        s.client.clone()
    };

    let response = client
        .get(parsed_url.clone())
        .send()
        .await
        .map_err(|e| format!("Fetch failed: {}", e))?;

    if let Some(length) = response.content_length()
        && length > MAX_NAVIGATION_RESPONSE_BYTES as u64
    {
        return Err("Response exceeds the 10 MiB navigation limit".to_string());
    }
    let content_type = response
        .headers()
        .get(reqwest::header::CONTENT_TYPE)
        .and_then(|value| value.to_str().ok())
        .map(str::to_owned);

    let mut response = response;
    let mut bytes = Vec::new();
    while let Some(chunk) = response
        .chunk()
        .await
        .map_err(|e| format!("Read failed: {}", e))?
    {
        if bytes.len().saturating_add(chunk.len()) > MAX_NAVIGATION_RESPONSE_BYTES {
            return Err("Response exceeds the 10 MiB navigation limit".to_string());
        }
        bytes.extend_from_slice(&chunk);
    }
    let html = decode_http_text(&bytes, content_type.as_deref());

    // Sanitize
    let clean = ammonia::clean(&html);

    // Security analysis (re-acquire lock for reflex arc)
    let dom = parse_html(&html);
    let title = dom.title().unwrap_or_else(|| "Untitled".to_string());

    let s = state.lock().map_err(|e| format!("Lock error: {}", e))?;
    let pre = s.reflex.pre_fetch(&parsed_url, false, false);
    let post = s.reflex.post_parse(&dom, &pre);
    let consent = ConsentStore::new();
    let zone = consent.resolve_zone(post.zone, parsed_url.host_str().unwrap_or(""));

    Ok(FetchResult {
        html: clean,
        title,
        zone: format!("{:?}", zone),
        safety: format!("{:?}", post.safety_level),
        threat_count: post.threats.len(),
    })
}

/// Perform epistemic search.
#[tauri::command]
fn search_claims(
    query: String,
    top_k: usize,
    state: State<'_, Mutex<PrismState>>,
) -> Result<Vec<SearchResultJson>, String> {
    let state = state.lock().map_err(|e| format!("Lock error: {}", e))?;
    let results = state.search.search(&query, top_k);

    Ok(results
        .into_iter()
        .map(|r| {
            let score = r.rank_score();
            SearchResultJson {
                content: r.content,
                sources: r.sources,
                empirical_level: format!("{:?}", r.empirical_level),
                query_similarity: r.query_similarity,
                author_reputation: r.author_reputation,
                tags: r.tags,
                rank_score: score,
            }
        })
        .collect())
}

/// Get claim count.
#[tauri::command]
fn claim_count(state: State<'_, Mutex<PrismState>>) -> Result<usize, String> {
    let state = state.lock().map_err(|e| format!("Lock error: {}", e))?;
    Ok(state.search.claim_count())
}

fn decode_http_text(bytes: &[u8], content_type: Option<&str>) -> String {
    let charset = content_type.and_then(|content_type| {
        content_type.split(';').skip(1).find_map(|parameter| {
            let (name, value) = parameter.trim().split_once('=')?;
            if name.trim().eq_ignore_ascii_case("charset") {
                Some(value.trim().trim_matches('"').trim_matches('\''))
            } else {
                None
            }
        })
    });
    let encoding = charset
        .and_then(|label| encoding_rs::Encoding::for_label(label.as_bytes()))
        .unwrap_or(encoding_rs::UTF_8);
    let (decoded, _, _) = encoding.decode(bytes);
    decoded.into_owned()
}

fn validate_user_navigation_url(raw: &str) -> Result<url::Url, String> {
    if raw.len() > MAX_NAVIGATION_URL_BYTES {
        return Err("URL exceeds the 8 KiB navigation limit".to_string());
    }
    let parsed = url::Url::parse(raw).map_err(|e| format!("Bad URL: {e}"))?;
    if !matches!(parsed.scheme(), "http" | "https") {
        return Err("Only HTTP and HTTPS navigation is supported".to_string());
    }
    if !parsed.username().is_empty() || parsed.password().is_some() {
        return Err("URL credentials are not permitted".to_string());
    }
    Ok(parsed)
}

fn main() {
    env_logger::init();

    // Build search index in a background thread to avoid blocking app launch.
    // The Mutex<PrismState> is populated immediately with an empty engine,
    // then swapped to the full engine once indexing completes.
    let prism_state = PrismState {
        search: SearchEngine::new(), // empty — loads fast
        reflex: ReflexArc::new(),
        client: reqwest::Client::builder()
            .user_agent("Prism/0.3 (Tauri; +https://mycelix.net)")
            .redirect(reqwest::redirect::Policy::limited(10))
            .timeout(std::time::Duration::from_secs(30))
            .build()
            .expect("HTTP client"),
    };

    log::info!("Prism desktop starting (indexing in background)");

    tauri::Builder::default()
        .plugin(tauri_plugin_shell::init())
        .manage(Mutex::new(prism_state))
        .invoke_handler(tauri::generate_handler![
            fetch_url,
            search_claims,
            claim_count,
        ])
        .setup(|app| {
            use tauri::Manager;

            // Background: load the search index without blocking the UI.
            // Use app.handle() which is 'static and cloneable.
            let handle = app.handle().clone();
            std::thread::spawn(move || {
                let engine = SearchEngine::with_core_claims();
                let count = engine.claim_count();
                let state = handle.state::<Mutex<PrismState>>();
                if let Ok(mut s) = state.lock() {
                    s.search = engine;
                }
                log::info!("Search index loaded: {} claims", count);
            });

            // WebKitGTK layout fix (reduced delay)
            if let Some(window) = app.get_webview_window("main") {
                let w = window.clone();
                std::thread::spawn(move || {
                    std::thread::sleep(std::time::Duration::from_millis(200));
                    let _ = w.eval(
                        "document.documentElement.style.width='100vw';\
                         document.body.style.width='100vw';\
                         document.body.style.margin='0';\
                         document.body.style.overflow='hidden auto';\
                         window.dispatchEvent(new Event('resize'));",
                    );
                    if let Ok(size) = w.inner_size() {
                        let _ = w.set_size(tauri::Size::Physical(tauri::PhysicalSize::new(
                            size.width - 1,
                            size.height,
                        )));
                        std::thread::sleep(std::time::Duration::from_millis(50));
                        let _ = w.set_size(tauri::Size::Physical(tauri::PhysicalSize::new(
                            size.width,
                            size.height,
                        )));
                    }
                });
            }
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error running Prism");
}
#[cfg(test)]
mod navigation_policy_tests {
    use super::*;

    #[test]
    fn bounded_navigation_preserves_declared_legacy_charsets() {
        assert_eq!(
            decode_http_text(&[0xE9], Some("text/html; charset=windows-1252")),
            "é"
        );
        assert_eq!(decode_http_text(b"hello", None), "hello");
    }

    #[test]
    fn desktop_navigation_keeps_explicit_lan_support_but_rejects_other_schemes() {
        assert!(validate_user_navigation_url("http://127.0.0.1:8080/").is_ok());
        assert!(validate_user_navigation_url("http://192.168.1.1/").is_ok());
        assert!(validate_user_navigation_url("https://example.com/").is_ok());
        assert!(validate_user_navigation_url("file:///etc/passwd").is_err());
        assert!(validate_user_navigation_url("https://user:pass@example.com/").is_err());
    }

    #[test]
    fn desktop_navigation_bounds_url_length() {
        let raw = format!(
            "https://example.com/{}",
            "a".repeat(MAX_NAVIGATION_URL_BYTES)
        );
        assert!(validate_user_navigation_url(&raw).is_err());
    }
}
