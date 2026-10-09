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

    /// Reset duration only when the selected audio source actually changes.
    /// Re-selecting the same track should keep seeking available while it restarts.
    fn prepare_track_change(&self, next_song: &Song) {
        let source_changed = self
            .current_song
            .get_untracked()
            .map_or(true, |current| current.song_hash != next_song.song_hash);
        if source_changed {
            self.duration.set(0.0);
        }
    }

    pub fn play_song(&self, song: Song) {
        let same_source = self
            .current_song
            .get_untracked()
            .map_or(false, |current| current.song_hash == song.song_hash);
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
        self.prepare_track_change(&song);
        self.current_song.set(Some(song));
        // Re-selecting the active source resumes it; resetting only the signal
        // would put the playhead at zero while the audio element keeps its time.
        if !same_source {
            self.progress.set(0.0);
        }
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
        self.prepare_track_change(&first);
        self.current_song.set(Some(first));
        self.progress.set(0.0);
        self.is_playing.set(true);
    }

    /// Advance because the user pressed Next. Repeat-one affects natural
    /// track completion, not an explicit request to skip forward.
    pub fn next(&self) {
        self.advance_queue(false);
    }

    /// Advance after the current media element naturally reaches its end.
    pub fn next_on_end(&self) {
        self.advance_queue(true);
    }

    fn advance_queue(&self, track_ended: bool) {
        let q = self.queue.get_untracked();
        if q.is_empty() {
            return;
        }
        let repeat_mode = self.repeat_mode.get_untracked();
        let next = queue_next_index(
            self.queue_index.get_untracked(),
            q.len(),
            &repeat_mode,
            track_ended,
        );
        if let Some(i) = next {
            self.queue_index.set(Some(i));
            let next_song = q[i].clone();
            self.prepare_track_change(&next_song);
            self.current_song.set(Some(next_song));
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
        let idx = queue_navigation_index(self.queue_index.get_untracked(), q.len());
        let prev = if idx > 0 {
            idx - 1
        } else if self.repeat_mode.get_untracked() == RepeatMode::All {
            q.len() - 1
        } else {
            0
        };
        self.queue_index.set(Some(prev));
        let previous_song = q[prev].clone();
        self.prepare_track_change(&previous_song);
        self.current_song.set(Some(previous_song));
        self.progress.set(0.0);
        self.is_playing.set(true);
    }

    /// Select the exact queue occurrence chosen in the queue panel.
    ///
    /// A playlist may intentionally contain the same song more than once.
    /// Content identity alone is not enough to identify a queue position.
    pub fn play_queued_song_at(&self, index: usize) {
        let q = self.queue.get_untracked();
        let Some(song) = q.get(index).cloned() else {
            return;
        };
        let same_source = self
            .current_song
            .get_untracked()
            .map_or(false, |current| current.song_hash == song.song_hash);
        self.queue_index.set(Some(index));
        self.prepare_track_change(&song);
        self.current_song.set(Some(song));
        if !same_source {
            self.progress.set(0.0);
        }
        self.is_playing.set(true);
    }

    /// Remove the exact queue occurrence chosen in the queue panel.
    ///
    /// Removing the active track selects the next item, or the previous item
    /// when the removed track was last. Removing the final queued item clears
    /// playback state; removing an earlier item only adjusts the queue index.
    pub fn remove_queued_song_at(&self, removed_index: usize) {
        let mut updated = self.queue.get_untracked();
        if removed_index >= updated.len() {
            return;
        }
        let current_index = self.queue_index.get_untracked();
        updated.remove(removed_index);
        self.queue.set(updated.clone());

        match queue_removal_action(current_index, removed_index, updated.len()) {
            QueueRemovalAction::Keep => {}
            QueueRemovalAction::ShiftIndex(index) => self.queue_index.set(Some(index)),
            QueueRemovalAction::Select(index) => {
                self.queue_index.set(Some(index));
                let next_song = updated[index].clone();
                self.prepare_track_change(&next_song);
                self.current_song.set(Some(next_song));
                self.progress.set(0.0);
            }
            QueueRemovalAction::ClearPlayback => {
                self.queue_index.set(None);
                self.current_song.set(None);
                self.is_playing.set(false);
                self.progress.set(0.0);
                self.duration.set(0.0);
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum QueueRemovalAction {
    Keep,
    ShiftIndex(usize),
    Select(usize),
    ClearPlayback,
}

/// Decide the playback-state consequence of removing one queue position.
fn queue_removal_action(
    current_index: Option<usize>,
    removed_index: usize,
    remaining_len: usize,
) -> QueueRemovalAction {
    // Empty queue means the player must not retain a stale current item,
    // even if an earlier inconsistency left queue_index unset or out of range.
    if remaining_len == 0 {
        return QueueRemovalAction::ClearPlayback;
    }

    match current_index {
        Some(index) if index == removed_index => {
            QueueRemovalAction::Select(removed_index.min(remaining_len - 1))
        }
        // Defend against a stale/out-of-range index instead of carrying it
        // forward into navigation where it could index beyond the queue.
        Some(index) if index >= remaining_len.saturating_add(1) => {
            QueueRemovalAction::Select(remaining_len - 1)
        }
        Some(index) if removed_index < index => QueueRemovalAction::ShiftIndex(index - 1),
        _ => QueueRemovalAction::Keep,
    }
}

/// Return a valid current queue index for a known non-empty queue.
fn queue_navigation_index(current_index: Option<usize>, queue_len: usize) -> usize {
    current_index.unwrap_or(0).min(queue_len.saturating_sub(1))
}

/// Select the next queue index, distinguishing a natural end from an explicit skip.
fn queue_next_index(
    current_index: Option<usize>,
    queue_len: usize,
    repeat_mode: &RepeatMode,
    track_ended: bool,
) -> Option<usize> {
    if queue_len == 0 {
        return None;
    }

    let index = queue_navigation_index(current_index, queue_len);
    if track_ended && repeat_mode == &RepeatMode::One {
        return Some(index);
    }
    if repeat_mode == &RepeatMode::All {
        return Some((index + 1) % queue_len);
    }

    let next = index + 1;
    (next < queue_len).then_some(next)
}

#[cfg(test)]
mod player_queue_tests {
    use super::{QueueRemovalAction, queue_next_index, queue_removal_action};
    use crate::types::RepeatMode;

    #[test]
    fn removing_track_before_current_shifts_index() {
        assert_eq!(
            queue_removal_action(Some(3), 1, 4),
            QueueRemovalAction::ShiftIndex(2)
        );
    }

    #[test]
    fn removing_track_after_current_keeps_selection() {
        assert_eq!(
            queue_removal_action(Some(1), 3, 3),
            QueueRemovalAction::Keep
        );
    }

    #[test]
    fn removing_current_selects_next_track_when_available() {
        assert_eq!(
            queue_removal_action(Some(1), 1, 3),
            QueueRemovalAction::Select(1)
        );
    }

    #[test]
    fn removing_last_current_selects_previous_track() {
        assert_eq!(
            queue_removal_action(Some(3), 3, 3),
            QueueRemovalAction::Select(2)
        );
    }

    #[test]
    fn removing_a_duplicate_playlist_occurrence_uses_its_exact_index() {
        // If the second of two identical songs is current, removing that row
        // selects the remaining row at index zero (rather than deleting the
        // first match by content hash).
        assert_eq!(
            queue_removal_action(Some(1), 1, 1),
            QueueRemovalAction::Select(0)
        );
        // Removing the first occurrence instead preserves the second song and
        // shifts its current index to zero.
        assert_eq!(
            queue_removal_action(Some(1), 0, 1),
            QueueRemovalAction::ShiftIndex(0)
        );
    }

    #[test]
    fn removing_final_track_clears_playback() {
        assert_eq!(
            queue_removal_action(Some(0), 0, 0),
            QueueRemovalAction::ClearPlayback
        );
    }

    #[test]
    fn removing_final_track_clears_playback_even_if_index_is_missing() {
        assert_eq!(
            queue_removal_action(None, 0, 0),
            QueueRemovalAction::ClearPlayback
        );
    }

    #[test]
    fn absent_current_index_does_not_invent_a_selection() {
        assert_eq!(
            queue_removal_action(None, 0, 1),
            QueueRemovalAction::Keep
        );
    }

    #[test]
    fn stale_current_index_recovers_to_last_remaining_track_on_removal() {
        assert_eq!(
            queue_removal_action(Some(99), 0, 2),
            QueueRemovalAction::Select(1)
        );
    }

    #[test]
    fn navigation_clamps_stale_indices_to_existing_queue() {
        assert_eq!(super::queue_navigation_index(Some(99), 3), 2);
        assert_eq!(super::queue_navigation_index(None, 3), 0);
        assert_eq!(super::queue_navigation_index(Some(99), 0), 0);
    }

    #[test]
    fn repeat_one_replays_only_on_natural_end_not_manual_next() {
        assert_eq!(
            queue_next_index(Some(0), 3, &RepeatMode::One, true),
            Some(0)
        );
        assert_eq!(
            queue_next_index(Some(0), 3, &RepeatMode::One, false),
            Some(1)
        );
    }

    #[test]
    fn manual_next_at_last_track_stops_even_with_repeat_one() {
        assert_eq!(
            queue_next_index(Some(2), 3, &RepeatMode::One, false),
            None
        );
    }

    #[test]
    fn repeat_all_wraps_at_end() {
        assert_eq!(
            queue_next_index(Some(2), 3, &RepeatMode::All, true),
            Some(0)
        );
    }

    #[test]
    fn repeat_none_stops_at_end() {
        assert_eq!(
            queue_next_index(Some(2), 3, &RepeatMode::None, true),
            None
        );
    }

    #[test]
    fn next_from_empty_queue_has_no_selection() {
        assert_eq!(
            queue_next_index(None, 0, &RepeatMode::All, true),
            None
        );
    }
}

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
