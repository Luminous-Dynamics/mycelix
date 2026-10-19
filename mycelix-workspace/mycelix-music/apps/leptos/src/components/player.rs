// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use super::queue::QueuePanel;
use crate::app::{PlayerState, format_time, same_audio_source};
use crate::types::RepeatMode;
use leptos::prelude::*;

/// True only when the browser has selected the expected media URL.
/// An empty currentSrc is a loading state, not proof that the selected URL is active.
fn media_source_matches_expected(current_src: &str, expected_url: &str) -> bool {
    !current_src.is_empty() && current_src == expected_url
}

/// A queued error is relevant only while the selected resource still reports an error.
fn media_error_matches_source(current_src: &str, expected_url: &str, error_present: bool) -> bool {
    error_present && media_source_matches_expected(current_src, expected_url)
}

/// Attempt playback and observe the media promise instead of treating the
/// synchronous JS call as proof that playback actually started. A stale reject
/// from an older track/play attempt must not pause a newer selection.
fn request_playback(
    audio: web_sys::HtmlAudioElement,
    player: PlayerState,
    attempt_generation: RwSignal<u64>,
) {
    let Some((expected_song_hash, expected_audio_url)) = player
        .current_song
        .get_untracked()
        .map(|song| (song.song_hash.clone(), song.audio_url()))
    else {
        return;
    };
    if !player.is_playing.get_untracked() {
        return;
    }
    // Each real play attempt supersedes older pending promises. Source
    // assignment/loading is owned by the Player effect before calling play().
    let generation = attempt_generation.get_untracked().wrapping_add(1);
    attempt_generation.set(generation);
    // A prior media error may leave the element in an unrecoverable error state.
    // Reload the already-selected URL before retrying; the new generation makes
    // any rejection from the previous load unable to stop this attempt.
    if audio.error().is_some() {
        player.progress.set(0.0);
        player.duration.set(0.0);
        audio.load();
    }
    let expected_hash_for_result = expected_song_hash.clone();
    let expected_url_for_result = expected_audio_url.clone();

    match audio.play() {
        Ok(promise) => {
            let player_for_result = player.clone();
            wasm_bindgen_futures::spawn_local(async move {
                if wasm_bindgen_futures::JsFuture::from(promise).await.is_err()
                    && attempt_generation.get_untracked() == generation
                    && player_for_result.is_playing.get_untracked()
                    && player_for_result
                        .current_song
                        .get_untracked()
                        .map(|song| (song.song_hash.clone(), song.audio_url()))
                        == Some((expected_hash_for_result, expected_url_for_result))
                {
                    player_for_result.is_playing.set(false);
                }
            });
        }
        Err(_) => {
            if attempt_generation.get_untracked() == generation
                && player.is_playing.get_untracked()
                && player
                    .current_song
                    .get_untracked()
                    .map(|song| (song.song_hash.clone(), song.audio_url()))
                    == Some((expected_song_hash, expected_audio_url))
            {
                player.is_playing.set(false);
            }
        }
    }
}

/// True only when media events belong to the song currently selected in PlayerState.
fn audio_matches_selected_source(audio: &web_sys::HtmlAudioElement, player: &PlayerState) -> bool {
    let current_src = audio.current_src();
    player
        .current_song
        .get_untracked()
        .is_some_and(|song| media_source_matches_expected(&current_src, &song.audio_url()))
}

/// The Player effect owns the src attribute; reload only when its declared
/// URL differs from the selected track. This avoids double-load races with
/// reactive attribute updates.
fn media_source_needs_load(declared_src: &str, expected_url: &str) -> bool {
    declared_src != expected_url
}

/// Return a bounded seek target only when the active resource has a known finite duration.
fn bounded_seek_target(requested_seconds: f64, media_duration: f64) -> Option<f64> {
    if !requested_seconds.is_finite() || !media_duration.is_finite() || media_duration <= 0.0 {
        return None;
    }
    Some(requested_seconds.clamp(0.0, media_duration))
}

/// Persistent audio player bar at the bottom of the screen.
/// Plays audio from IPFS gateway URLs and records plays via zome calls.
#[component]
pub fn Player() -> impl IntoView {
    let player = expect_context::<PlayerState>();
    let current = player.current_song;
    let is_playing = player.is_playing;
    let volume = player.volume;
    let audio_ref = NodeRef::<leptos::html::Audio>::new();
    // Invalidates rejected play promises from older user actions / selections.
    let play_attempt_generation = RwSignal::new(0u64);

    let toggle_play = move |_| {
        is_playing.update(|p| *p = !*p);
    };

    // Clone the state per rendered view so reactive re-renders do not consume
    // the event handlers captured by the component.
    let player_for_previous = player.clone();
    let player_for_next = player.clone();

    let show_queue = player.show_queue;
    let toggle_queue = move |_| show_queue.update(|show| *show = !*show);
    let queue = player.queue;
    let progress = player.progress;
    let duration = player.duration;
    let repeat_mode = player.repeat_mode;
    let cycle_repeat = move |_| {
        repeat_mode.update(|mode| *mode = mode.next());
    };

    // `autoplay` is only a loading hint; changing it after mount does not
    // reliably control an existing media element. Drive the media API from
    // the reactive playback state instead.
    let player_for_effect = player.clone();
    // Keep logical selection identity separately from the media URL. If a
    // different record points at the same URL, src will not change, so reset
    // the real playhead here instead of only resetting the reactive progress.
    let last_selected_identity = RwSignal::new(None::<(String, String)>);
    Effect::new(move |_| {
        let selected_song = current.get();
        let selected_identity = selected_song
            .as_ref()
            .map(|song| (song.song_hash.clone(), song.audio_url()));
        if let (Some((previous_hash, previous_url)), Some(song)) = (
            last_selected_identity.get_untracked(),
            selected_song.as_ref(),
        ) {
            let selected_url = song.audio_url();
            if previous_hash != song.song_hash && previous_url == selected_url {
                if let Some(audio) = audio_ref.get() {
                    if audio.current_src() == selected_url {
                        audio.set_current_time(0.0);
                    }
                }
            }
        }
        last_selected_identity.set(selected_identity);
        let should_play = is_playing.get();
        if let Some(audio) = audio_ref.get() {
            if let Some(song) = selected_song.as_ref() {
                let expected_url = song.audio_url();
                if media_source_needs_load(&audio.src(), &expected_url) {
                    // Imperatively own source selection before requesting play.
                    // Changing src initiates resource selection; do not call load()
                    // again here, which would abort/restart that just-started selection.
                    audio.set_src(&expected_url);
                }
                if should_play {
                    request_playback(audio, player_for_effect.clone(), play_attempt_generation);
                } else {
                    let _ = audio.pause();
                }
            } else {
                // A cleared selection stops playback, releases the previous
                // resource, and resets the physical and reactive playheads.
                let _ = audio.pause();
                let _ = audio.remove_attribute("src");
                audio.load();
                audio.set_current_time(0.0);
            }
        }
    });

    let player_for_time = player.clone();
    let update_time = move |_| {
        if let Some(audio) = audio_ref.get() {
            if audio_matches_selected_source(&audio, &player_for_time) {
                player_for_time.progress.set(audio.current_time());
            }
        }
    };

    let seek_progress = player.progress;
    let seek_audio = audio_ref;
    let player_for_seek = player.clone();
    // NodeRef and RwSignal are copyable handles; keeping this handler outside
    // the per-song render closure avoids moving a non-Copy closure on rerender.
    let seek_to = move |ev| {
        if let Ok(seconds) = event_target_value(&ev).parse::<f64>() {
            if let Some(audio) = seek_audio.get() {
                // A stale slider interaction must not seek a different track
                // while the persistent element is loading a new source.
                if audio_matches_selected_source(&audio, &player_for_seek) {
                    if let Some(target) = bounded_seek_target(seconds, audio.duration()) {
                        audio.set_current_time(target);
                        seek_progress.set(target);
                    }
                }
            }
        }
    };

    let player_for_metadata = player.clone();
    let update_metadata = move |_| {
        if let Some(audio) = audio_ref.get() {
            if audio_matches_selected_source(&audio, &player_for_metadata) {
                let duration = audio.duration();
                if duration.is_finite() {
                    player_for_metadata.duration.set(duration);
                }
            }
        }
    };

    let player_for_error = player.clone();
    let on_media_error = move |_| {
        if let Some(audio) = audio_ref.get() {
            let current_src = audio.current_src();
            let error_present = audio.error().is_some();
            let selected_url = player_for_error
                .current_song
                .get_untracked()
                .map(|song| song.audio_url());
            if selected_url.as_deref().is_some_and(|expected_url| {
                media_error_matches_source(&current_src, expected_url, error_present)
            }) {
                player_for_error.is_playing.set(false);
                player_for_error.progress.set(0.0);
                player_for_error.duration.set(0.0);
            }
        }
    };

    let player_for_ready = player.clone();
    let start_when_ready = move |_| {
        if player_for_ready.is_playing.get_untracked() {
            if let Some(audio) = audio_ref.get() {
                if audio_matches_selected_source(&audio, &player_for_ready) {
                    request_playback(audio, player_for_ready.clone(), play_attempt_generation);
                }
            }
        }
    };

    let player_for_end = player.clone();
    let audio_for_end = audio_ref;
    let on_ended = move |_| {
        let Some(audio) = audio_for_end.get() else {
            return;
        };
        // A late event for a superseded source must not advance the new song.
        if !audio.ended() || !audio_matches_selected_source(&audio, &player_for_end) {
            return;
        }
        let before_song = player_for_end.current_song.get_untracked();
        player_for_end.next_on_end();
        let after_song = player_for_end.current_song.get_untracked();
        // Repeat-one (or a one-track repeat-all queue) has the same source
        // on both sides. Rewind and explicitly restart the persistent element.
        if player_for_end.is_playing.get_untracked()
            && matches!(
                (&before_song, &after_song),
                (Some(before), Some(after)) if same_audio_source(before, after)
            )
        {
            audio.set_current_time(0.0);
            request_playback(audio, player_for_end.clone(), play_attempt_generation);
        }
    };

    view! {
        <>
            <div class="player-bar">
                {move || {
                    if let Some(song) = current.get() {
                        let previous_player = player_for_previous.clone();
                        let previous_audio = audio_ref;
                        let play_previous = move |_| {
                            let before_song = previous_player.current_song.get_untracked();
                            let restart_current =
                                previous_player.progress.get_untracked() > 3.0;
                            let was_playing = previous_player.is_playing.get_untracked();
                            previous_player.previous();
                            let after_song = previous_player.current_song.get_untracked();
                            // Previous at the start of the first track can resolve to
                            // the same source. Keep the real element in sync with the
                            // state transition, but preserve pause on the >3s restart gesture.
                            if matches!(
                                (&before_song, &after_song),
                                (Some(before), Some(after)) if same_audio_source(before, after)
                            ) {
                                if let Some(audio) = previous_audio.get() {
                                    audio.set_current_time(0.0);
                                    if !restart_current || was_playing {
                                        request_playback(
                                            audio,
                                            previous_player.clone(),
                                            play_attempt_generation,
                                        );
                                    }
                                }
                            }
                        };
                        let next_player = player_for_next.clone();
                        let next_audio = audio_ref;
                        let play_next = move |_| {
                            let before_song = next_player.current_song.get_untracked();
                            next_player.next();
                            let after_song = next_player.current_song.get_untracked();
                            // Repeat-one and a one-track repeat-all queue select the
                            // same source. Rewind and start it even if it was paused.
                            if next_player.is_playing.get_untracked()
                                && matches!(
                                    (&before_song, &after_song),
                                    (Some(before), Some(after)) if same_audio_source(before, after)
                                )
                            {
                                if let Some(audio) = next_audio.get() {
                                    audio.set_current_time(0.0);
                                    request_playback(
                                        audio,
                                        next_player.clone(),
                                        play_attempt_generation,
                                    );
                                }
                            }
                        };
                        view! {
                            <div class="player-info">
                                <span class="player-title">{song.title.clone()}</span>
                                <span class="player-duration">{song.duration_display()}</span>
                            </div>
                            <div class="player-timeline">
                                <span class="player-time">{move || format_time(progress.get())}</span>
                                <input
                                    class="player-seek"
                                    type="range"
                                    min="0"
                                    max=move || duration.get().max(0.0).to_string()
                                    step="0.1"
                                    prop:value=move || progress.get().min(duration.get().max(0.0)).max(0.0).to_string()
                                    on:input=seek_to
                                    disabled=move || duration.get() <= 0.0
                                    aria-label="Seek playback position"
                                    title="Seek playback position"
                                />
                                <span class="player-time">{move || format_time(duration.get())}</span>
                            </div>
                            <div class="player-controls">
                                <button
                                    class="btn-player-secondary"
                                    on:click=play_previous
                                    disabled=move || player.queue.get().is_empty()
                                    aria-label="Previous track"
                                    title="Previous track"
                                >
                                    "⏮"
                                </button>
                                <button
                                    class="btn-player"
                                    on:click=toggle_play
                                    aria-label=move || if is_playing.get() { "Pause" } else { "Play" }
                                    title=move || if is_playing.get() { "Pause" } else { "Play" }
                                >
                                    {move || if is_playing.get() { "⏸" } else { "▶" }}
                                </button>
                                <button
                                    class="btn-player-secondary"
                                    on:click=play_next
                                    disabled=move || player.queue.get().is_empty()
                                    aria-label="Next track"
                                    title="Next track"
                                >
                                    "⏭"
                                </button>
                                <button
                                    class=move || match repeat_mode.get() {
                                        RepeatMode::None => "btn-player-secondary repeat-off",
                                        RepeatMode::All => "btn-player-secondary repeat-active",
                                        RepeatMode::One => "btn-player-secondary repeat-active repeat-one",
                                    }
                                    on:click=cycle_repeat
                                    aria-label=move || match repeat_mode.get() {
                                        RepeatMode::None => "Repeat off",
                                        RepeatMode::All => "Repeat all",
                                        RepeatMode::One => "Repeat one",
                                    }
                                    title=move || match repeat_mode.get() {
                                        RepeatMode::None => "Repeat off",
                                        RepeatMode::All => "Repeat all",
                                        RepeatMode::One => "Repeat one",
                                    }
                                    aria-pressed=move || (repeat_mode.get() != RepeatMode::None).to_string()
                                >
                                    {move || match repeat_mode.get() {
                                        RepeatMode::None => "↻",
                                        RepeatMode::All => "↻",
                                        RepeatMode::One => "1↻",
                                    }}
                                </button>
                            </div>
                        }.into_any()
                    } else {
                        view! {
                            <div class="player-empty">
                                <span>"No song selected — browse "<a href="/discover">"Discover"</a></span>
                            </div>
                        }.into_any()
                    }
                }}
                <button
                    class="btn-player-secondary queue-toggle"
                    on:click=toggle_queue
                    aria-label="Toggle playback queue"
                    aria-expanded=move || show_queue.get().to_string()
                    title="Open playback queue"
                >
                    "Queue (" {move || queue.get().len()} ")"
                </button>
            </div>
            // Keep one media element mounted across song and queue transitions.
            // Removing src when no song is selected avoids loading the page URL,
            // and gives clear-queue transitions a stable element to pause.
            // Source selection is deliberately owned by the Player effect,
            // so src updates and play() ordering cannot race each other.
            <audio
                node_ref=audio_ref
                prop:volume=move || volume.get()
                preload="metadata"
                on:canplay=start_when_ready
                on:timeupdate=update_time
                on:loadedmetadata=update_metadata
                on:ended=on_ended
                on:error=on_media_error
            />
            <QueuePanel />
        </>
    }
}

#[cfg(test)]
mod media_source_guard_tests {
    use super::media_source_matches_expected;

    #[test]
    fn empty_current_source_is_not_playable_yet() {
        assert!(!media_source_matches_expected(
            "",
            "https://ipfs.io/ipfs/QmSong"
        ));
    }

    #[test]
    fn stale_current_source_cannot_satisfy_new_selection() {
        assert!(!media_source_matches_expected(
            "https://ipfs.io/ipfs/QmOld",
            "https://ipfs.io/ipfs/QmNew"
        ));
    }

    #[test]
    fn exact_selected_source_is_accepted() {
        assert!(media_source_matches_expected(
            "https://ipfs.io/ipfs/QmSong",
            "https://ipfs.io/ipfs/QmSong"
        ));
    }
}

#[cfg(test)]
mod media_error_guard_tests {
    use super::media_error_matches_source;

    #[test]
    fn matching_source_with_current_error_is_reported() {
        let source = "https://ipfs.io/ipfs/QmSong";
        assert!(media_error_matches_source(source, source, true));
    }

    #[test]
    fn error_from_superseded_source_is_ignored() {
        assert!(!media_error_matches_source(
            "https://ipfs.io/ipfs/QmOld",
            "https://ipfs.io/ipfs/QmNew",
            true
        ));
    }

    #[test]
    fn late_error_event_after_resource_reset_is_ignored() {
        let source = "https://ipfs.io/ipfs/QmSong";
        assert!(!media_error_matches_source(source, source, false));
    }
}

#[cfg(test)]
mod media_source_reload_tests {
    use super::media_source_needs_load;

    #[test]
    fn empty_declared_source_requires_load() {
        assert!(media_source_needs_load("", "https://ipfs.io/ipfs/QmSong"));
    }

    #[test]
    fn different_declared_source_requires_load() {
        assert!(media_source_needs_load(
            "https://ipfs.io/ipfs/QmOld",
            "https://ipfs.io/ipfs/QmNew"
        ));
    }

    #[test]
    fn matching_declared_source_does_not_reload() {
        let expected = "https://ipfs.io/ipfs/QmSong";
        assert!(!media_source_needs_load(expected, expected));
    }
}

#[cfg(test)]
mod seek_target_tests {
    use super::bounded_seek_target;

    #[test]
    fn seek_target_clamps_to_media_bounds() {
        assert_eq!(bounded_seek_target(-5.0, 120.0), Some(0.0));
        assert_eq!(bounded_seek_target(121.0, 120.0), Some(120.0));
    }

    #[test]
    fn seek_target_rejects_unknown_or_non_finite_duration() {
        assert_eq!(bounded_seek_target(10.0, f64::NAN), None);
        assert_eq!(bounded_seek_target(10.0, f64::INFINITY), None);
        assert_eq!(bounded_seek_target(10.0, 0.0), None);
    }

    #[test]
    fn seek_target_rejects_non_finite_requested_position() {
        assert_eq!(bounded_seek_target(f64::NAN, 120.0), None);
        assert_eq!(bounded_seek_target(f64::INFINITY, 120.0), None);
    }
}
