// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::app::{DeliveryPreferences, PlayerState};
use crate::delivery::next_candidate_index;
use leptos::prelude::*;

/// Persistent audio player bar at the bottom of the screen.
/// Uses the delivery resolver and falls back when an HTTPS gateway fails.
#[component]
pub fn Player() -> impl IntoView {
    let player = expect_context::<PlayerState>();
    let delivery_preferences = expect_context::<DeliveryPreferences>();
    let gateway_origins = delivery_preferences.gateway_origins;
    let preferences_loaded = delivery_preferences.loaded;
    let current = player.current_song;
    let is_playing = player.is_playing;
    let volume = player.volume;
    let audio_ref = NodeRef::<leptos::html::Audio>::new();
    let source_index = RwSignal::new(0usize);
    let delivery_failed = RwSignal::new(false);

    // Reset per-track route selection; a previous track's failed gateway must
    // not become the first choice for the next track.
    Effect::new(move |_| {
        let _current_asset = current.get().map(|song| song.song_hash);
        let _gateway_preference = gateway_origins.get();
        let _preferences_loaded = preferences_loaded.get();
        source_index.set(0);
        delivery_failed.set(false);
    });

    let toggle_play = move |_| {
        is_playing.update(|p| *p = !*p);
    };

    // `autoplay` is only a loading hint; changing it after mount does not
    // reliably control an existing media element. Drive the media API from
    // the reactive playback state instead.
    Effect::new(move |_| {
        let has_song = current.get().is_some();
        let should_play = is_playing.get();
        let preferences_ready = preferences_loaded.get();
        if !has_song || !preferences_ready {
            return;
        }
        if let Some(audio) = audio_ref.get() {
            if should_play {
                if audio.play().is_err() {
                    is_playing.set(false);
                }
            } else {
                let _ = audio.pause();
            }
        }
    });

    let player_for_time = player.clone();
    let update_time = move |_| {
        if let Some(audio) = audio_ref.get() {
            player_for_time.progress.set(audio.current_time());
        }
    };

    let player_for_metadata = player.clone();
    let update_metadata = move |_| {
        if let Some(audio) = audio_ref.get() {
            let duration = audio.duration();
            if duration.is_finite() {
                player_for_metadata.duration.set(duration);
            }
        }
    };

    let player_for_end = player.clone();

    let start_when_ready = move |_| {
        if preferences_loaded.get_untracked() && is_playing.get_untracked() {
            if let Some(audio) = audio_ref.get() {
                if audio.play().is_err() {
                    is_playing.set(false);
                }
            }
        }
    };

    view! {
        <div class="player-bar">
            {move || {
                if let Some(song) = current.get() {
                    let on_ended = {
                        let player_for_end = player_for_end.clone();
                        move |_| player_for_end.next()
                    };
                    let song_for_src = song.clone();
                    let song_for_status = song.clone();
                    let on_audio_error = {
                        let current = current;
                        let source_index = source_index;
                        let delivery_failed = delivery_failed;
                        let is_playing = is_playing;
                        let gateway_origins = gateway_origins;
                        let preferences_loaded = preferences_loaded;
                        move |_| {
                            // The placeholder source may emit an error while
                            // localStorage is being read; it is not a failed
                            // gateway candidate and must not advance/fail the plan.
                            if !preferences_loaded.get_untracked() {
                                return;
                            }
                            if let Some(current_song) = current.get_untracked() {
                                if let Some(next_index) = next_candidate_index(
                                    source_index.get_untracked(),
                                    current_song.audio_source_count_with_preference(
                                        gateway_origins.get_untracked().as_deref(),
                                    ),
                                ) {
                                    source_index.set(next_index);
                                } else {
                                    delivery_failed.set(true);
                                    is_playing.set(false);
                                }
                            } else {
                                delivery_failed.set(true);
                                is_playing.set(false);
                            }
                        }
                    };
                    view! {
                        <div class="player-info">
                            <span class="player-title">{song.title.clone()}</span>
                            <span class="player-duration">{song.duration_display()}</span>
                            {move || {
                                let ready = preferences_loaded.get();
                                let no_delivery_route = ready && song_for_status
                                    .audio_source_count_with_preference(gateway_origins.get().as_deref()) == 0;
                                (ready && (delivery_failed.get() || no_delivery_route)).then(|| view! {
                                    <span class="player-error" role="status">
                                        "No gateway is selected, or all selected gateways failed. Review Delivery Settings or retry later."
                                    </span>
                                })
                            }}
                        </div>
                        <div class="player-controls">
                            <button
                                class="btn-player"
                                on:click=toggle_play
                                aria-label=move || if is_playing.get() { "Pause" } else { "Play" }
                            >
                                {move || if is_playing.get() { "⏸" } else { "▶" }}
                            </button>
                        </div>
                        <audio
                            node_ref=audio_ref
                            prop:src=move || {
                                if !preferences_loaded.get() {
                                    // Do not make an outbound request until the
                                    // saved browser-local gateway preference is known.
                                    "about:blank".to_string()
                                } else {
                                    song_for_src
                                        .audio_url_at_with_preference(
                                            source_index.get(),
                                            gateway_origins.get().as_deref(),
                                        )
                                        .unwrap_or_else(|| "about:blank".to_string())
                                }
                            }
                            prop:volume=move || volume.get()
                            preload="metadata"
                            on:canplay=start_when_ready
                            on:timeupdate=update_time
                            on:loadedmetadata=update_metadata
                            on:ended=on_ended
                            on:error=on_audio_error
                        />
                    }.into_any()
                } else {
                    view! {
                        <div class="player-empty">
                            <span>"No song selected — browse "
                                <a href="/discover">"Discover"</a>
                            </span>
                        </div>
                    }.into_any()
                }
            }}
        </div>
    }
}
