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

/// Logical track identity binds the song record and the actual media resource.
pub(crate) fn same_audio_source(left: &Song, right: &Song) -> bool {
    left.song_hash == right.song_hash && same_media_resource(left, right)
}

/// The browser's persistent audio element is keyed by its resolved media URL.
pub(crate) fn same_media_resource(left: &Song, right: &Song) -> bool {
    left.audio_url() == right.audio_url()
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

    /// Reset media duration only when the actual audio resource changes.
    /// A new song record can point to the same URL, whose metadata remains valid.
    fn prepare_track_change(&self, next_song: &Song) {
        let media_changed = self
            .current_song
            .get_untracked()
            .is_none_or(|current| !same_media_resource(&current, next_song));
        if media_changed {
            self.duration.set(0.0);
        }
    }

    pub fn play_song(&self, song: Song) {
        let same_source = self
            .current_song
            .get_untracked()
            .is_some_and(|current| same_audio_source(&current, &song));
        let mut q = self.queue.get_untracked();
        let idx = q
            .iter()
            .position(|queued| same_audio_source(queued, &song))
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
            if !q.iter().any(|queued| same_audio_source(queued, &song)) {
                q.push(song);
            }
        });
    }

    /// Clear playback selection and reset all state tied to the active track.
    /// Shared by the explicit queue-clear action and final-item removal.
    fn clear_playback(&self) {
        self.queue_index.set(None);
        self.current_song.set(None);
        self.is_playing.set(false);
        self.progress.set(0.0);
        self.duration.set(0.0);
    }

    /// Clear the queue and all playback state derived from it.
    pub fn clear_queue(&self) {
        self.queue.set(Vec::new());
        self.clear_playback();
        self.show_queue.set(false);
    }

    pub fn play_all(&self, songs: Vec<Song>) {
        if songs.is_empty() {
            return;
        }
        let first = songs[0].clone();
        let same_source = self
            .current_song
            .get_untracked()
            .is_some_and(|current| same_audio_source(&current, &first));
        self.queue.set(songs);
        self.queue_index.set(Some(0));
        self.prepare_track_change(&first);
        self.current_song.set(Some(first));
        if !same_source {
            self.progress.set(0.0);
        }
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
            .is_some_and(|current| same_audio_source(&current, &song));
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
                let same_source = self
                    .current_song
                    .get_untracked()
                    .is_some_and(|current| same_audio_source(&current, &next_song));
                self.prepare_track_change(&next_song);
                self.current_song.set(Some(next_song));
                // When another occurrence of the same song is selected, the
                // persistent audio element keeps its real playhead. Do not
                // reset only the UI progress signal in that case.
                if !same_source {
                    self.progress.set(0.0);
                }
            }
            QueueRemovalAction::ClearPlayback => self.clear_playback(),
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

    if current_index.is_none() {
        // With an unselected but non-empty queue, Next starts the first item;
        // treating None as index zero would accidentally skip to item one.
        return Some(0);
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
    use super::{
        PlayerState, QueueRemovalAction, queue_next_index, queue_removal_action, same_audio_source,
        same_media_resource,
    };
    use crate::types::{AgentPubKey, RepeatMode, Song, Timestamp};
    use leptos::prelude::Owner;

    fn test_song(song_hash: &str, ipfs_cid: &str) -> Song {
        Song {
            song_hash: song_hash.to_string(),
            title: format!("Test {ipfs_cid}"),
            artist: AgentPubKey(vec![0; 39]),
            ipfs_cid: ipfs_cid.to_string(),
            cover_cid: None,
            duration_seconds: 180,
            genres: vec!["Test".to_string()],
            strategy_id: "standard".to_string(),
            released_at: Timestamp(0),
            metadata: "{}".to_string(),
        }
    }

    #[test]
    fn source_identity_binds_song_record_and_audio_url() {
        let same = test_song("record-a", "QmSame");
        let same_again = test_song("record-a", "QmSame");
        let changed_url = test_song("record-a", "QmDifferent");
        let changed_record = test_song("record-b", "QmSame");

        assert!(same_audio_source(&same, &same_again));
        assert!(!same_audio_source(&same, &changed_url));
        assert!(!same_audio_source(&same, &changed_record));
    }

    #[test]
    fn media_resource_identity_tracks_url_independently_of_record_hash() {
        let first_record = test_song("record-a", "QmSame");
        let second_record = test_song("record-b", "QmSame");
        let changed_url = test_song("record-a", "QmOther");

        assert!(same_media_resource(&first_record, &second_record));
        assert!(!same_media_resource(&first_record, &changed_url));
        assert!(!same_audio_source(&first_record, &second_record));
    }

    #[test]
    fn selecting_new_record_for_same_url_resets_playhead_but_keeps_duration() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let first = test_song("record-a", "QmSame");
            let second = test_song("record-b", "QmSame");
            player.queue.set(vec![first.clone(), second]);
            player.current_song.set(Some(first));
            player.queue_index.set(Some(0));
            player.progress.set(42.5);
            player.duration.set(180.0);

            player.play_queued_song_at(1);

            assert_eq!(player.queue_index.get_untracked(), Some(1));
            assert_eq!(player.progress.get_untracked(), 0.0);
            assert_eq!(player.duration.get_untracked(), 180.0);
        });
    }

    #[test]
    fn playing_changed_url_with_reused_record_hash_adds_correct_queue_occurrence() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let old_source = test_song("reused-record", "QmOld");
            let revised_source = test_song("reused-record", "QmNew");
            player.queue.set(vec![old_source.clone()]);
            player.current_song.set(Some(old_source));
            player.queue_index.set(Some(0));
            player.progress.set(42.5);
            player.duration.set(180.0);

            player.play_song(revised_source);

            let queue = player.queue.get_untracked();
            assert_eq!(queue.len(), 2);
            assert_eq!(player.queue_index.get_untracked(), Some(1));
            assert_eq!(
                queue[1].audio_url(),
                "https://ipfs.io/ipfs/QmNew".to_string()
            );
            assert_eq!(
                player
                    .current_song
                    .get_untracked()
                    .map(|song| song.audio_url()),
                Some("https://ipfs.io/ipfs/QmNew".to_string())
            );
            assert_eq!(player.progress.get_untracked(), 0.0);
            assert_eq!(player.duration.get_untracked(), 0.0);
            assert!(player.is_playing.get_untracked());
        });
    }

    #[test]
    fn enqueue_deduplicates_same_source_but_not_reused_hash_with_changed_url() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let original = test_song("reused-record", "QmOld");
            let revised_source = test_song("reused-record", "QmNew");

            player.enqueue(original.clone());
            player.enqueue(original);
            player.enqueue(revised_source);

            let queue = player.queue.get_untracked();
            assert_eq!(queue.len(), 2);
            assert_eq!(queue[0].audio_url(), "https://ipfs.io/ipfs/QmOld");
            assert_eq!(queue[1].audio_url(), "https://ipfs.io/ipfs/QmNew");
        });
    }

    #[test]
    fn clear_queue_resets_selection_timing_and_visibility_together() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let song = test_song("clear-me", "QmClear");
            player.queue.set(vec![song.clone()]);
            player.current_song.set(Some(song));
            player.queue_index.set(Some(0));
            player.is_playing.set(true);
            player.progress.set(42.5);
            player.duration.set(180.0);
            player.show_queue.set(true);

            player.clear_queue();

            assert!(player.queue.get_untracked().is_empty());
            assert_eq!(
                player
                    .current_song
                    .get_untracked()
                    .map(|song| song.song_hash),
                None
            );
            assert_eq!(player.queue_index.get_untracked(), None);
            assert!(!player.is_playing.get_untracked());
            assert_eq!(player.progress.get_untracked(), 0.0);
            assert_eq!(player.duration.get_untracked(), 0.0);
            assert!(!player.show_queue.get_untracked());
        });
    }

    #[test]
    fn selecting_same_source_queue_occurrence_preserves_playhead() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let duplicate = test_song("duplicate", "QmSame");
            player.queue.set(vec![duplicate.clone(), duplicate.clone()]);
            player.current_song.set(Some(duplicate));
            player.queue_index.set(Some(0));
            player.progress.set(42.5);
            player.duration.set(180.0);

            player.play_queued_song_at(1);

            assert_eq!(player.queue_index.get_untracked(), Some(1));
            assert_eq!(player.progress.get_untracked(), 42.5);
            assert_eq!(player.duration.get_untracked(), 180.0);
            assert!(player.is_playing.get_untracked());
        });
    }

    #[test]
    fn selecting_changed_url_resets_playhead_even_if_record_hash_matches() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let first = test_song("same-record", "QmOld");
            let revised_resource = test_song("same-record", "QmNew");
            player.queue.set(vec![first.clone(), revised_resource]);
            player.current_song.set(Some(first));
            player.queue_index.set(Some(0));
            player.progress.set(42.5);
            player.duration.set(180.0);

            player.play_queued_song_at(1);

            assert_eq!(player.queue_index.get_untracked(), Some(1));
            assert_eq!(player.progress.get_untracked(), 0.0);
            assert_eq!(player.duration.get_untracked(), 0.0);
        });
    }

    #[test]
    fn removing_current_occurrence_resets_playhead_when_next_source_changes() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let current = test_song("same-record", "QmOld");
            let next = test_song("same-record", "QmNew");
            player.queue.set(vec![current.clone(), next]);
            player.current_song.set(Some(current));
            player.queue_index.set(Some(0));
            player.is_playing.set(true);
            player.progress.set(42.5);
            player.duration.set(180.0);

            player.remove_queued_song_at(0);

            assert_eq!(player.queue.get_untracked().len(), 1);
            assert_eq!(player.queue_index.get_untracked(), Some(0));
            assert_eq!(
                player
                    .current_song
                    .get_untracked()
                    .map(|song| song.audio_url()),
                Some("https://ipfs.io/ipfs/QmNew".to_string())
            );
            assert_eq!(player.progress.get_untracked(), 0.0);
            assert_eq!(player.duration.get_untracked(), 0.0);
        });
    }

    #[test]
    fn removing_same_source_duplicate_preserves_playhead() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let duplicate = test_song("duplicate", "QmSame");
            player.queue.set(vec![duplicate.clone(), duplicate.clone()]);
            player.current_song.set(Some(duplicate));
            player.queue_index.set(Some(1));
            player.is_playing.set(true);
            player.progress.set(42.5);
            player.duration.set(180.0);

            player.remove_queued_song_at(1);

            assert_eq!(player.queue.get_untracked().len(), 1);
            assert_eq!(player.queue_index.get_untracked(), Some(0));
            assert_eq!(player.progress.get_untracked(), 42.5);
            assert_eq!(player.duration.get_untracked(), 180.0);
            assert!(player.is_playing.get_untracked());
        });
    }

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
        assert_eq!(queue_removal_action(None, 0, 1), QueueRemovalAction::Keep);
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
        assert_eq!(queue_next_index(Some(2), 3, &RepeatMode::One, false), None);
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
        assert_eq!(queue_next_index(Some(2), 3, &RepeatMode::None, true), None);
    }

    #[test]
    fn next_from_unselected_queue_starts_at_first_track_for_every_repeat_mode() {
        assert_eq!(
            queue_next_index(None, 3, &RepeatMode::None, false),
            Some(0)
        );
        assert_eq!(
            queue_next_index(None, 3, &RepeatMode::All, false),
            Some(0)
        );
        assert_eq!(
            queue_next_index(None, 3, &RepeatMode::One, false),
            Some(0)
        );
    }

    #[test]
    fn manual_next_with_queued_tracks_but_no_selection_starts_first() {
        let owner = Owner::new();
        owner.with(|| {
            let player = PlayerState::new();
            let first = test_song("first", "QmFirst");
            player.enqueue(first.clone());
            player.enqueue(test_song("second", "QmSecond"));

            player.next();

            assert_eq!(player.queue_index.get_untracked(), Some(0));
            assert_eq!(
                player.current_song.get_untracked().map(|song| song.song_hash),
                Some(first.song_hash)
            );
            assert!(player.is_playing.get_untracked());
        });
    }

    #[test]
    fn next_from_empty_queue_has_no_selection() {
        assert_eq!(queue_next_index(None, 0, &RepeatMode::All, true), None);
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
