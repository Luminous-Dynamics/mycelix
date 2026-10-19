// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use leptos::prelude::*;
use leptos_router::{
    components::{Route, Router, Routes},
    path,
};
use wasm_bindgen::JsCast;

use mycelix_leptos_core::{
    ConnectStrategy, ConnectionBadge, HolochainProviderAuto, HolochainProviderConfig,
};

use crate::components::{Nav, Player};
use crate::pages::*;
use crate::types::{RepeatMode, Song};

#[derive(Clone, Debug)]
pub struct PlayerState {
    pub current_song: RwSignal<Option<Song>>,
    pub is_playing: RwSignal<bool>,
    pub volume: RwSignal<f64>,
    pub progress: RwSignal<f64>,
    pub duration: RwSignal<f64>,
    pub queue: RwSignal<Vec<Song>>,
    pub queue_index: RwSignal<Option<usize>>,
    pub repeat_mode: RwSignal<RepeatMode>,
    pub shuffle: RwSignal<bool>,
    pub show_queue: RwSignal<bool>,
}

impl PlayerState {
    pub fn new() -> Self {
        Self {
            current_song: RwSignal::new(None),
            is_playing: RwSignal::new(false),
            volume: RwSignal::new(0.8),
            progress: RwSignal::new(0.0),
            duration: RwSignal::new(0.0),
            queue: RwSignal::new(Vec::new()),
            queue_index: RwSignal::new(None),
            repeat_mode: RwSignal::new(RepeatMode::None),
            shuffle: RwSignal::new(false),
            show_queue: RwSignal::new(false),
        }
    }

    pub fn play_song(&self, song: Song) {
        let mut q = self.queue.get_untracked();
        let idx = q
            .iter()
            .position(|s| s.song_hash == song.song_hash)
            .unwrap_or_else(|| {
                q.push(song.clone());
                self.queue.set(q.clone());
                q.len() - 1
            });
        self.queue_index.set(Some(idx));
        self.current_song.set(Some(song));
        self.progress.set(0.0);
        self.is_playing.set(true);
    }

    pub fn enqueue(&self, song: Song) {
        self.queue.update(|q| {
            if !q.iter().any(|s| s.song_hash == song.song_hash) {
                q.push(song);
            }
        });
    }

    pub fn play_all(&self, songs: Vec<Song>) {
        if songs.is_empty() {
            return;
        }
        let first = songs[0].clone();
        self.queue.set(songs);
        self.queue_index.set(Some(0));
        self.current_song.set(Some(first));
        self.progress.set(0.0);
        self.is_playing.set(true);
    }

    pub fn next(&self) {
        let q = self.queue.get_untracked();
        if q.is_empty() {
            return;
        }
        let idx = self.queue_index.get_untracked().unwrap_or(0);
        let next = match self.repeat_mode.get_untracked() {
            RepeatMode::One => Some(idx),
            RepeatMode::All => Some((idx + 1) % q.len()),
            RepeatMode::None => {
                let n = idx + 1;
                if n < q.len() { Some(n) } else { None }
            }
        };
        if let Some(i) = next {
            self.queue_index.set(Some(i));
            self.current_song.set(Some(q[i].clone()));
            self.progress.set(0.0);
            self.is_playing.set(true);
        } else {
            self.is_playing.set(false);
        }
    }

    pub fn previous(&self) {
        let q = self.queue.get_untracked();
        if q.is_empty() {
            return;
        }
        if self.progress.get_untracked() > 3.0 {
            self.progress.set(0.0);
            return;
        }
        let idx = self.queue_index.get_untracked().unwrap_or(0);
        let prev = if idx > 0 {
            idx - 1
        } else if self.repeat_mode.get_untracked() == RepeatMode::All {
            q.len() - 1
        } else {
            0
        };
        self.queue_index.set(Some(prev));
        self.current_song.set(Some(q[prev].clone()));
        self.progress.set(0.0);
        self.is_playing.set(true);
    }
}

/// Browser-local media gateway choice. None means use the deployment's
/// explicit default pool; Some("") means the listener has disabled media
/// delivery; Some(list) means use only that validated origin pool.
#[derive(Clone, Debug)]
pub struct DeliveryPreferences {
    pub gateway_origins: RwSignal<Option<String>>,
    /// False until browser-local preferences have been loaded. The player must
    /// not select a deployment gateway before that privacy choice is known.
    pub loaded: RwSignal<bool>,
}

pub const MEDIA_GATEWAY_PREFERENCE_KEY: &str = "mycelix-media-gateway-origins";

#[derive(Clone, Debug)]
pub struct ThemeState {
    pub valence: RwSignal<f64>,
    pub arousal: RwSignal<f64>,
}

impl ThemeState {
    pub fn new() -> Self {
        Self {
            valence: RwSignal::new(0.0),
            arousal: RwSignal::new(0.5),
        }
    }
}

pub fn format_time(seconds: f64) -> String {
    let s = seconds as u32;
    format!("{}:{:02}", s / 60, s % 60)
}

#[component]
pub fn App() -> impl IntoView {
    let player = PlayerState::new();
    provide_context(player.clone());
    let theme = ThemeState::new();
    provide_context(theme.clone());

    let delivery_preferences = DeliveryPreferences {
        gateway_origins: RwSignal::new(None),
        loaded: RwSignal::new(false),
    };
    provide_context(delivery_preferences.clone());

    // Load per-listener preferences only from this browser's local storage.
    // Missing/unavailable storage keeps the explicit deployment default.
    Effect::new(move |_| {
        let saved = web_sys::window()
            .and_then(|window| window.local_storage().ok().flatten())
            .and_then(|storage| {
                storage
                    .get_item(MEDIA_GATEWAY_PREFERENCE_KEY)
                    .ok()
                    .flatten()
            });
        if let Some(saved) = saved {
            delivery_preferences.gateway_origins.set(Some(saved));
        }
        // Publish readiness only after storage is read (or known unavailable),
        // so the player never emits an initial request before honoring opt-out.
        delivery_preferences.loaded.set(true);
    });

    Effect::new(move |_| {
        let v = theme.valence.get();
        let a = theme.arousal.get();
        let hue = if v >= 0.0 {
            270.0 - v * 120.0
        } else {
            270.0 - v * 30.0
        };
        let sat = 30.0 + a * 60.0;
        let light = 45.0 + a * 20.0;
        if let Some(doc) = web_sys::window().and_then(|w| w.document()) {
            if let Some(root) = doc.document_element() {
                if let Ok(el) = root.dyn_into::<web_sys::HtmlElement>() {
                    let s = el.style();
                    let _ = s.set_property("--emotion-hue", &format!("{hue:.0}"));
                    let _ = s.set_property("--emotion-saturation", &format!("{sat:.0}%"));
                    let _ = s.set_property("--emotion-lightness", &format!("{light:.0}%"));
                }
            }
        }
    });

    let provider_config = HolochainProviderConfig {
        app_id: "mycelix-music".to_string(),
        default_role: Some("music".to_string()),
        log_prefix: "[Mycelix Music]",
        connect_strategy: if cfg!(feature = "fixtures") {
            ConnectStrategy::MockOnly
        } else {
            ConnectStrategy::WebSocket
        },
        status_labels: None,
    };

    view! {
        <HolochainProviderAuto config=provider_config>
            <Router>
                <Nav />
                <main class="main-content">
                    <Routes fallback=|| view! { <div class="page"><h1>"404 — Page not found"</h1></div> }>
                        <Route path=path!("/") view=ConsciousnessPage />
                        <Route path=path!("/discover") view=DiscoverPage />
                        <Route path=path!("/artist") view=ArtistPage />
                        <Route path=path!("/dashboard") view=DashboardPage />
                        <Route path=path!("/upload") view=UploadPage />
                        <Route path=path!("/gallery") view=GalleryPage />
                        <Route path=path!("/settings") view=DeliverySettingsPage />
                        <Route path=path!("/about") view=HomePage />
                    </Routes>
                </main>
                <Player />
                <footer class="footer">
                    <ConnectionBadge />
                    {cfg!(feature = "fixtures").then(|| view! {
                        <span class="fixture-label">"Development fixtures enabled"</span>
                    })}
                    <span class="footer-text">"Mycelix Music — What does your consciousness sound like?"</span>
                </footer>
            </Router>
        </HolochainProviderAuto>
    }
}
