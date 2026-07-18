// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::app::PlayerState;
use leptos::prelude::*;

/// Persistent audio player bar at the bottom of the screen.
/// Plays audio from IPFS gateway URLs and records plays via zome calls.
#[component]
pub fn Player() -> impl IntoView {
    let player = expect_context::<PlayerState>();
    let current = player.current_song;
    let is_playing = player.is_playing;
    let volume = player.volume;
    let audio_ref = NodeRef::<leptos::html::Audio>::new();

    let toggle_play = move |_| {
        is_playing.update(|p| *p = !*p);
    };

    // `autoplay` is only a loading hint; changing it after mount does not
    // reliably control an existing media element. Drive the media API from
    // the reactive playback state instead.
    Effect::new(move |_| {
        let has_song = current.get().is_some();
        let should_play = is_playing.get();
        if !has_song {
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
        if is_playing.get_untracked() {
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
                    view! {
                        <div class="player-info">
                            <span class="player-title">{song.title.clone()}</span>
                            <span class="player-duration">{song.duration_display()}</span>
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
                            prop:src=song.audio_url()
                            prop:volume=move || volume.get()
                            preload="metadata"
                            on:canplay=start_when_ready
                            on:timeupdate=update_time
                            on:loadedmetadata=update_metadata
                            on:ended=on_ended
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
