// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::components::SongCard;
use crate::types::{Song, mock_songs};
use leptos::prelude::*;
use mycelix_leptos_core::{AvailabilityState, AvailabilityStateKind};

const GENRES: &[&str] = &[
    "All",
    "Electronic",
    "Rock",
    "Indie",
    "Pop",
    "Ambient",
    "Hip-Hop",
    "Jazz",
    "Classical",
    "Folk",
];

#[component]
pub fn DiscoverPage() -> impl IntoView {
    let selected_genre = RwSignal::new("All".to_string());
    let songs = RwSignal::new(if cfg!(feature = "fixtures") {
        mock_songs()
    } else {
        Vec::new()
    });

    // Fixture data is compile-time gated and visibly labelled below. A normal
    // build never substitutes it for an unavailable conductor.
    let filtered_songs = move || {
        let genre = selected_genre.get();
        let all = songs.get();
        if genre == "All" {
            all
        } else {
            all.into_iter()
                .filter(|s| s.genres.iter().any(|g| g == &genre))
                .collect::<Vec<Song>>()
        }
    };

    let genre_chips = GENRES
        .iter()
        .map(|&g| {
            let g_str = g.to_string();
            let g_click = g_str.clone();
            view! {
                <button
                    class=move || {
                        if selected_genre.get() == g_str { "genre-chip active" } else { "genre-chip" }
                    }
                    on:click=move |_| selected_genre.set(g_click.clone())
                >
                    {g}
                </button>
            }
        })
        .collect_view();

    view! {
        <div class="page discover-page">
            <h1>"Discover"</h1>
            {if cfg!(feature = "fixtures") {
                view! {
                    <AvailabilityState
                        kind=AvailabilityStateKind::Mock
                        title="Development catalog"
                        description="These listings are fixtures. No conductor data is being shown."
                        action=None
                    />
                    <div class="genre-filters">{genre_chips}</div>
                    <div class="song-grid">
                        {move || {
                            filtered_songs()
                                .into_iter()
                                .map(|song| view! { <SongCard song=song /> })
                                .collect_view()
                        }}
                    </div>
                }.into_any()
            } else {
                view! {
                    <AvailabilityState
                        kind=AvailabilityStateKind::Unavailable
                        title="Catalog unavailable"
                        description="Connect an authorized Holochain conductor to load authoritative songs."
                        action=None
                    />
                }.into_any()
            }}
        </div>
    }
}
