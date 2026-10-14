// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Video browsing and watch UX foundation.
//!
//! This page intentionally does not fabricate a published video catalogue.
//! Synthetic cards are available only with the existing `fixtures` feature.
//! Production catalogue/zome integration is a separate step.

use std::collections::HashMap;

use leptos::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::HtmlVideoElement;

use crate::app::DeliveryPreferences;
use crate::delivery::{
    delivery_candidates_for_preference, next_candidate_index, DeliveryCandidate,
    MediaAssetManifestV1,
};

const VIDEO_KINDS: &[&str] = &[
    "All",
    "Short films",
    "Performances",
    "Documentaries",
    "Learning",
    "Field reports",
];

const RESUME_STORAGE_PREFIX: &str = "mycelix-video-progress-v1:";
const LAST_VIDEO_STORAGE_KEY: &str = "mycelix-video-last-opened-v1";

#[derive(Clone, Debug)]
struct VideoChapter {
    label: String,
    start_seconds: u64,
}

#[derive(Clone, Debug)]
struct VideoListing {
    id: String,
    title: String,
    creator: String,
    kind: String,
    description: String,
    duration_seconds: u64,
    topics: Vec<String>,
    poster_style: &'static str,
    content_cid: Option<String>,
    poster_cid: Option<String>,
    captions_cid: Option<String>,
    caption_language: Option<String>,
    transcript_cid: Option<String>,
    visual_description_cid: Option<String>,
    chapters: Vec<VideoChapter>,
}

impl VideoListing {
    fn duration_label(&self) -> String {
        format_duration(self.duration_seconds)
    }

    fn has_captions(&self) -> bool {
        self.captions_cid.is_some()
    }

    fn has_transcript(&self) -> bool {
        self.transcript_cid.is_some()
    }

    fn has_visual_description(&self) -> bool {
        self.visual_description_cid.is_some()
    }
}

fn format_duration(seconds: u64) -> String {
    let hours = seconds / 3_600;
    let minutes = (seconds % 3_600) / 60;
    let seconds = seconds % 60;
    if hours > 0 {
        format!("{hours}:{minutes:02}:{seconds:02}")
    } else {
        format!("{minutes}:{seconds:02}")
    }
}

fn resume_storage_key(video_id: &str) -> String {
    format!("{RESUME_STORAGE_PREFIX}{video_id}")
}

fn listing_candidates(
    listing: &VideoListing,
    media_kind: &str,
    cid: Option<&str>,
    preference: Option<&str>,
) -> Vec<DeliveryCandidate> {
    let Some(cid) = cid else {
        return Vec::new();
    };
    let manifest = MediaAssetManifestV1::for_ipfs(
        format!("video:{}:{media_kind}", listing.id),
        cid.to_owned(),
    );
    delivery_candidates_for_preference(&manifest, preference).unwrap_or_default()
}

fn listing_url(
    listing: &VideoListing,
    media_kind: &str,
    cid: Option<&str>,
    preference: Option<&str>,
) -> Option<String> {
    listing_candidates(listing, media_kind, cid, preference)
        .into_iter()
        .next()
        .map(|candidate| candidate.url)
}

fn fixture_videos() -> Vec<VideoListing> {
    vec![
        VideoListing {
            id: "fixture-city-that-listens".into(),
            title: "A City That Listens".into(),
            creator: "Mycelix Studio · sample creator".into(),
            kind: "Short films".into(),
            description: "A fictional short exploring how neighbourhood knowledge, public space, and local decisions might become more legible to the people who live with their consequences.".into(),
            duration_seconds: 8 * 60 + 42,
            topics: vec!["Civic imagination".into(), "Community".into()],
            poster_style: "video-poster-violet",
            content_cid: None,
            poster_cid: None,
            captions_cid: None,
            caption_language: None,
            transcript_cid: None,
            visual_description_cid: None,
            chapters: vec![
                VideoChapter { label: "A shared place".into(), start_seconds: 0 },
                VideoChapter { label: "Listening before building".into(), start_seconds: 142 },
                VideoChapter { label: "A decision people can inspect".into(), start_seconds: 326 },
            ],
        },
        VideoListing {
            id: "fixture-materials-that-grow".into(),
            title: "Materials That Grow".into(),
            creator: "Commons Lab · sample creator".into(),
            kind: "Documentaries".into(),
            description: "A fictional field documentary concept about repairable tools, biological materials, and designing with local supply chains instead of assuming every region has the same resources.".into(),
            duration_seconds: 14 * 60 + 10,
            topics: vec!["Appropriate technology".into(), "Materials".into()],
            poster_style: "video-poster-moss",
            content_cid: None,
            poster_cid: None,
            captions_cid: None,
            caption_language: None,
            transcript_cid: None,
            visual_description_cid: None,
            chapters: vec![
                VideoChapter { label: "Start with what is available".into(), start_seconds: 0 },
                VideoChapter { label: "Material choices".into(), start_seconds: 281 },
                VideoChapter { label: "Repair and reuse".into(), start_seconds: 642 },
            ],
        },
        VideoListing {
            id: "fixture-river-field-notes".into(),
            title: "Field Notes: Repairing a River".into(),
            creator: "Water Commons · sample creator".into(),
            kind: "Field reports".into(),
            description: "A fictional field-report outline following a community from observing a waterway to documenting a shared maintenance plan and its open questions.".into(),
            duration_seconds: 6 * 60 + 28,
            topics: vec!["Water".into(), "Local knowledge".into()],
            poster_style: "video-poster-blue",
            content_cid: None,
            poster_cid: None,
            captions_cid: None,
            caption_language: None,
            transcript_cid: None,
            visual_description_cid: None,
            chapters: vec![
                VideoChapter { label: "Observe the watershed".into(), start_seconds: 0 },
                VideoChapter { label: "Record what changed".into(), start_seconds: 118 },
                VideoChapter { label: "Agree on follow-up".into(), start_seconds: 267 },
            ],
        },
        VideoListing {
            id: "fixture-shared-room-session".into(),
            title: "A Live Session in a Shared Room".into(),
            creator: "Resonance Collective · sample creator".into(),
            kind: "Performances".into(),
            description: "A fictional performance listing showing how a video release can preserve credits, accessibility tracks, and a clear relationship to its makers.".into(),
            duration_seconds: 12 * 60 + 2,
            topics: vec!["Performance".into(), "Music".into()],
            poster_style: "video-poster-amber",
            content_cid: None,
            poster_cid: None,
            captions_cid: None,
            caption_language: None,
            transcript_cid: None,
            visual_description_cid: None,
            chapters: vec![
                VideoChapter { label: "Opening".into(), start_seconds: 0 },
                VideoChapter { label: "Main set".into(), start_seconds: 96 },
                VideoChapter { label: "Conversation after the set".into(), start_seconds: 574 },
            ],
        },
        VideoListing {
            id: "fixture-holochain-field-notes".into(),
            title: "Field Notes on Local-First Software".into(),
            creator: "Cooperative Systems · sample creator".into(),
            kind: "Learning".into(),
            description: "A fictional learning-video concept about data ownership, peer-to-peer collaboration, offline-first tools, and the trade-offs that a distributed system still has to make explicit.".into(),
            duration_seconds: 10 * 60 + 35,
            topics: vec!["Local-first".into(), "Software".into()],
            poster_style: "video-poster-cyan",
            content_cid: None,
            poster_cid: None,
            captions_cid: None,
            caption_language: None,
            transcript_cid: None,
            visual_description_cid: None,
            chapters: vec![
                VideoChapter { label: "The local-first idea".into(), start_seconds: 0 },
                VideoChapter { label: "Sharing without one central owner".into(), start_seconds: 195 },
                VideoChapter { label: "Failure modes and recovery".into(), start_seconds: 418 },
            ],
        },
        VideoListing {
            id: "fixture-repair-culture".into(),
            title: "The Practice of Repair".into(),
            creator: "Repair Commons · sample creator".into(),
            kind: "Documentaries".into(),
            description: "A fictional documentary concept about repair cafes, tool libraries, and the social infrastructure that makes useful things last longer.".into(),
            duration_seconds: 18 * 60 + 6,
            topics: vec!["Repair".into(), "Circular economy".into()],
            poster_style: "video-poster-rose",
            content_cid: None,
            poster_cid: None,
            captions_cid: None,
            caption_language: None,
            transcript_cid: None,
            visual_description_cid: None,
            chapters: vec![
                VideoChapter { label: "Meet the repair table".into(), start_seconds: 0 },
                VideoChapter { label: "Skills move through a community".into(), start_seconds: 356 },
                VideoChapter { label: "Make repair the easy choice".into(), start_seconds: 811 },
            ],
        },
    ]
}

/// Discover and inspect published video listings. The initial slice is
/// intentionally catalog-safe: only CID-addressed media sources are accepted,
/// playback never autostarts, and resume state remains in this browser.
#[component]
pub fn VideosPage() -> impl IntoView {
    let preferences = expect_context::<DeliveryPreferences>();
    let videos = RwSignal::new(if cfg!(feature = "fixtures") {
        fixture_videos()
    } else {
        Vec::new()
    });
    let search_query = RwSignal::new(String::new());
    let selected_kind = RwSignal::new("All".to_string());
    let selected_id = RwSignal::new(None::<String>);
    let resume_seconds = RwSignal::new(HashMap::<String, u64>::new());
    let source_index = RwSignal::new(0usize);
    let delivery_failed = RwSignal::new(false);
    let video_node = NodeRef::new();
    let last_saved_bucket = RwSignal::new(0u64);

    Effect::new(move |_| {
        // Resume positions are deliberately browser-local. They are not
        // published to Holochain, sent to a gateway, or synced across devices.
        let storage = web_sys::window()
            .and_then(|window| window.local_storage().ok().flatten());
        let mut saved = HashMap::new();
        if let Some(storage) = storage {
            for listing in videos.get_untracked() {
                if let Some(seconds) = storage
                    .get_item(&resume_storage_key(&listing.id))
                    .ok()
                    .flatten()
                    .and_then(|value| value.parse::<u64>().ok())
                {
                    if seconds < listing.duration_seconds {
                        saved.insert(listing.id.clone(), seconds);
                    }
                }
            }
            if let Some(last_id) = storage
                .get_item(LAST_VIDEO_STORAGE_KEY)
                .ok()
                .flatten()
            {
                if videos.get_untracked().iter().any(|item| item.id == last_id) {
                    selected_id.set(Some(last_id));
                }
            }
        }
        resume_seconds.set(saved);
    });

    // A source selection or gateway preference change starts a fresh bounded
    // candidate plan. Nothing is requested until preferences have loaded.
    Effect::new(move |_| {
        let _selected = selected_id.get();
        let _gateway_preference = preferences.gateway_origins.get();
        let _preferences_loaded = preferences.loaded.get();
        source_index.set(0);
        delivery_failed.set(false);
        last_saved_bucket.set(0);
    });

    let filtered_videos = move || {
        let query = search_query.get().trim().to_lowercase();
        let kind = selected_kind.get();
        videos
            .get()
            .into_iter()
            .filter(|video| {
                let matches_kind = kind == "All" || video.kind == kind;
                let matches_query = query.is_empty()
                    || video.title.to_lowercase().contains(&query)
                    || video.creator.to_lowercase().contains(&query)
                    || video.description.to_lowercase().contains(&query)
                    || video.topics.iter().any(|topic| topic.to_lowercase().contains(&query));
                matches_kind && matches_query
            })
            .collect::<Vec<_>>()
    };

    let selected_video = move || {
        let id = selected_id.get();
        videos
            .get()
            .into_iter()
            .find(|video| Some(video.id.clone()) == id)
    };

    let category_buttons = VIDEO_KINDS
        .iter()
        .map(|kind| {
            let label = (*kind).to_string();
            let label_for_class = label.clone();
            let label_for_pressed = label.clone();
            let set_label = label.clone();
            view! {
                <button
                    type="button"
                    class=move || if selected_kind.get() == label_for_class { "video-filter active" } else { "video-filter" }
                    aria-pressed=move || selected_kind.get() == label_for_pressed
                    on:click=move |_| selected_kind.set(set_label.clone())
                >
                    {kind.to_string()}
                </button>
            }
        })
        .collect_view();

    let on_video_error = move |_| {
        if let Some(video) = selected_video() {
            let count = listing_candidates(
                &video,
                "content",
                video.content_cid.as_deref(),
                preferences.gateway_origins.get_untracked().as_deref(),
            )
            .len();
            if let Some(next) = next_candidate_index(source_index.get_untracked(), count) {
                source_index.set(next);
            } else {
                delivery_failed.set(true);
            }
        } else {
            delivery_failed.set(true);
        }
    };

    let on_video_loaded = move |event: web_sys::Event| {
        if let (Some(video), Some(listing)) = (
            event.current_target().and_then(|target| target.dyn_into::<HtmlVideoElement>().ok()),
            selected_video(),
        ) {
            if let Some(saved) = resume_seconds.get_untracked().get(&listing.id).copied() {
                // Avoid seeking to the end where browsers may refuse or mark
                // the item complete. Do not autoplay after restoration.
                if saved > 15 && saved + 30 < listing.duration_seconds {
                    video.set_current_time(saved as f64);
                }
            }
        }
    };

    let on_video_timeupdate = move |event: web_sys::Event| {
        let Some(video_element) = event
            .current_target()
            .and_then(|target| target.dyn_into::<HtmlVideoElement>().ok())
        else {
            return;
        };
        let Some(listing) = selected_video() else {
            return;
        };
        let seconds = video_element.current_time().max(0.0) as u64;
        let bucket = seconds / 10;
        if bucket <= last_saved_bucket.get_untracked() || seconds >= listing.duration_seconds {
            return;
        }
        last_saved_bucket.set(bucket);
        resume_seconds.update(|saved| {
            saved.insert(listing.id.clone(), seconds);
        });
        if let Some(storage) = web_sys::window().and_then(|window| window.local_storage().ok().flatten()) {
            let _ = storage.set_item(&resume_storage_key(&listing.id), &seconds.to_string());
            let _ = storage.set_item(LAST_VIDEO_STORAGE_KEY, &listing.id);
        }
    };

    view! {
        <div class="page videos-page">
            <section class="videos-intro">
                <div>
                    <p class="video-eyebrow">"MYCELIX MEDIA"</p>
                    <h1>"Watch with context."</h1>
                    <p class="page-subtitle">
                        "Films, performances, field reports, and learning media—with credits, accessibility, and provenance close at hand."
                    </p>
                </div>
                <div class="video-principles" aria-label="Video experience principles">
                    <span>"No autoplay"</span>
                    <span>"Local resume"</span>
                    <span>"Explicit delivery"</span>
                </div>
            </section>

            {cfg!(feature = "fixtures").then(|| view! {
                <div class="video-fixture-notice" role="note">
                    <strong>"Development fixture catalog."</strong>
                    <span>" Titles and descriptions below are synthetic UX samples, not published videos. No playable media source is attached and no sample video will be requested from a gateway."</span>
                </div>
            })}

            <section class="video-discovery" aria-labelledby="video-discovery-title">
                <div class="video-section-heading">
                    <div>
                        <h2 id="video-discovery-title">"Explore video"</h2>
                        <p>"Search by title, creator, topic, or format."</p>
                    </div>
                    <label class="video-search-label" for="video-search">"Search videos"</label>
                    <input
                        id="video-search"
                        class="video-search"
                        type="search"
                        placeholder="Try “repair”, “water”, or “learning”"
                        prop:value=move || search_query.get()
                        on:input=move |event| search_query.set(event_target_value(&event))
                    />
                </div>

                <div class="video-filters" role="group" aria-label="Filter by video format">
                    {category_buttons}
                </div>

                {move || {
                    let items = filtered_videos();
                    let result_count = items.len();
                    if items.is_empty() {
                        if cfg!(feature = "fixtures") {
                            view! {
                                <div class="video-empty-state">
                                    <div class="video-empty-icon" aria-hidden="true">"⌕"</div>
                                    <h3>"No matches for this search"</h3>
                                    <p>"Try a broader term or choose a different format. Filters only search this local development fixture set."</p>
                                    <button class="btn btn-secondary" type="button" on:click=move |_| {
                                        search_query.set(String::new());
                                        selected_kind.set("All".into());
                                    }>"Clear filters"</button>
                                </div>
                            }.into_any()
                        } else {
                            view! {
                                <div class="video-empty-state">
                                    <div class="video-empty-icon" aria-hidden="true">"▹"</div>
                                    <h3>"The video catalogue is not connected yet"</h3>
                                    <p>"Mycelix Music has no authoritative video-listing adapter on this page yet. Published entries, rights metadata, posters, captions, transcripts, and their CID manifests must come from the catalogue—not synthetic production data."</p>
                                    <a class="btn btn-secondary" href="/settings">"Review delivery preferences"</a>
                                </div>
                            }.into_any()
                        }
                    } else {
                        view! {
                            <p class="video-result-count" aria-live="polite">{format!("{result_count} development listing(s)")}</p>
                            <div class="video-card-grid">
                                {items.into_iter().map(|listing| {
                                    let id = listing.id.clone();
                                    let select_id = id.clone();
                                    let title = listing.title.clone();
                                    let poster_style = listing.poster_style;
                                    let card_poster_listing = listing.clone();
                                    let card_poster_preferences = preferences.clone();
                                    let card_poster_url = Signal::derive(move || {
                                        if card_poster_preferences.loaded.get() {
                                            listing_url(
                                                &card_poster_listing,
                                                "poster",
                                                card_poster_listing.poster_cid.as_deref(),
                                                card_poster_preferences.gateway_origins.get().as_deref(),
                                            )
                                        } else {
                                            None
                                        }
                                    });
                                    let creator = listing.creator.clone();
                                    let kind = listing.kind.clone();
                                    let duration = listing.duration_label();
                                    let tags = listing.topics.iter().take(2).map(|topic| {
                                        view! { <span class="video-topic-tag">{topic.clone()}</span> }
                                    }).collect_view();
                                    let resume = resume_seconds.get().get(&id).copied();
                                    let resume_label = resume.map(|seconds| format!("Resume · {}", format_duration(seconds)));
                                    view! {
                                        <article class="video-card">
                                            <button
                                                type="button"
                                                class="video-card-open"
                                                aria-label=format!("Open video details: {title}")
                                                on:click=move |_| selected_id.set(Some(select_id.clone()))
                                            >
                                                <span class=format!("video-poster {poster_style}") aria-hidden="true">
                                                    {move || card_poster_url.get().map(|url| view! {
                                                        <img class="video-poster-image" src=url alt="" loading="lazy" decoding="async" />
                                                    }.into_any())}
                                                    <span class="video-poster-mark">"▶"</span>
                                                    <span class="video-duration">{duration.clone()}</span>
                                                </span>
                                                <span class="video-card-copy">
                                                    <span class="video-card-kind">{kind.clone()}</span>
                                                    <span class="video-card-title">{title.clone()}</span>
                                                    <span class="video-card-creator">{creator.clone()}</span>
                                                    <span class="video-card-topics">{tags}</span>
                                                    {resume_label.map(|label| view! { <span class="video-resume-badge">{label}</span> })}
                                                </span>
                                            </button>
                                        </article>
                                    }
                                }).collect_view()}
                            </div>
                        }.into_any()
                    }
                }}
            </section>

            {move || {
                selected_video().map(|listing| {
                    let id = listing.id.clone();
                    let title = listing.title.clone();
                    let creator = listing.creator.clone();
                    let kind = listing.kind.clone();
                    let duration = listing.duration_label();
                    let description = listing.description.clone();
                    let has_cid = listing.content_cid.is_some();
                    let source_url = move || {
                        selected_video().and_then(|current| {
                            if !preferences.loaded.get() {
                                return None;
                            }
                            listing_candidates(
                                &current,
                                "content",
                                current.content_cid.as_deref(),
                                preferences.gateway_origins.get().as_deref(),
                            )
                            .get(source_index.get())
                            .map(|candidate| candidate.url.clone())
                        })
                    };
                    let source_preferences = preferences.clone();
                    let source_video = listing.clone();
                    let current_source_index = source_index;
                    let source_url = Signal::derive(move || {
                        if !source_preferences.loaded.get() {
                            return None;
                        }
                        listing_candidates(
                            &source_video,
                            "content",
                            source_video.content_cid.as_deref(),
                            source_preferences.gateway_origins.get().as_deref(),
                        )
                        .get(current_source_index.get())
                        .map(|candidate| candidate.url.clone())
                    });
                    let caption_preferences = preferences.clone();
                    let caption_listing = listing.clone();
                    let caption_url = Signal::derive(move || {
                        if caption_preferences.loaded.get() {
                            listing_url(
                                &caption_listing,
                                "captions",
                                caption_listing.captions_cid.as_deref(),
                                caption_preferences.gateway_origins.get().as_deref(),
                            )
                        } else {
                            None
                        }
                    });
                    let transcript_preferences = preferences.clone();
                    let transcript_listing = listing.clone();
                    let transcript_url = Signal::derive(move || {
                        if transcript_preferences.loaded.get() {
                            listing_url(
                                &transcript_listing,
                                "transcript",
                                transcript_listing.transcript_cid.as_deref(),
                                transcript_preferences.gateway_origins.get().as_deref(),
                            )
                        } else {
                            None
                        }
                    });
                    let description_preferences = preferences.clone();
                    let description_listing = listing.clone();
                    let description_url = Signal::derive(move || {
                        if description_preferences.loaded.get() {
                            listing_url(
                                &description_listing,
                                "visual-description",
                                description_listing.visual_description_cid.as_deref(),
                                description_preferences.gateway_origins.get().as_deref(),
                            )
                        } else {
                            None
                        }
                    });
                    let chapters = listing.chapters.clone().into_iter().map(|chapter| {
                        let seconds = chapter.start_seconds;
                        let seek = move |_| {
                            if let Some(video) = video_node.get() {
                                video.set_current_time(seconds as f64);
                                // Seeking is explicit user intent; playback still requires pressing Play.
                            }
                        };
                        view! {
                            <li>
                                <button type="button" class="video-chapter" on:click=seek>
                                    <span class="video-chapter-time">{format_duration(seconds)}</span>
                                    <span>{chapter.label}</span>
                                </button>
                            </li>
                        }
                    }).collect_view();
                    let topics = listing.topics.iter().map(|topic| view! {
                        <span class="video-topic-tag">{topic.clone()}</span>
                    }).collect_view();
                    let saved_time = resume_seconds.get().get(&id).copied();
                    let progress_pct = saved_time.map(|seconds| {
                        ((seconds as f64 / listing.duration_seconds.max(1) as f64) * 100.0).clamp(0.0, 100.0)
                    }).unwrap_or(0.0);
                    let close_id = id.clone();
                    view! {
                        <section class="video-watch-panel" aria-labelledby="video-watch-title">
                            <div class="video-watch-heading">
                                <div>
                                    <p class="video-eyebrow">"NOW VIEWING · DETAILS"</p>
                                    <h2 id="video-watch-title">{title.clone()}</h2>
                                    <p class="video-watch-byline">{creator.clone()} " · " {kind.clone()} " · " {duration.clone()}</p>
                                </div>
                                <button
                                    type="button"
                                    class="video-close"
                                    aria-label="Close video details"
                                    on:click=move |_| selected_id.set(None)
                                >"Close"</button>
                            </div>

                            <div class="video-watch-layout">
                                <div class="video-main-column">
                                    {move || {
                                        if source_url.get().is_some() {
                                            let title_for_player = title.clone();
                                            let caption_language = listing.caption_language.clone().unwrap_or_else(|| "en".into());
                                            view! {
                                                <div class="video-player-frame">
                                                    <video
                                                        node_ref=video_node
                                                        class="video-element"
                                                        prop:src=move || source_url.get().unwrap_or_default()
                                                        controls
                                                        playsinline
                                                        preload="none"
                                                        aria-label=title_for_player.clone()
                                                        on:error=on_video_error
                                                        on:loadedmetadata=on_video_loaded
                                                        on:timeupdate=on_video_timeupdate
                                                    >
                                                        {move || {
                                                            if let Some(url) = caption_url.get() {
                                                                view! {
                                                                    <track kind="captions"
                                                                        src=url
                                                                        srclang=caption_language.clone()
                                                                        label="Captions"
                                                                        default
                                                                    />
                                                                }.into_any()
                                                            } else {
                                                                ().into_any()
                                                            }
                                                        }}
                                                        "Your browser does not support HTML video. Use the linked transcript when available."
                                                    </video>
                                                </div>
                                            }.into_any()
                                        } else {
                                            view! {
                                                <div class=format!("video-player-placeholder {poster_style}") role="note">
                                                    <span class="video-player-placeholder-mark" aria-hidden="true">"▶"</span>
                                                    <h3>{move || {
                                                        if !has_cid {
                                                            "Media source not attached"
                                                        } else if !preferences.loaded.get() {
                                                            "Loading delivery preferences"
                                                        } else if delivery_failed.get() {
                                                            "Video could not be loaded"
                                                        } else {
                                                            "No gateway route selected"
                                                        }
                                                    }}</h3>
                                                    <p>{move || {
                                                        if !has_cid {
                                                            "This is a UI fixture, not a published video. No network request will be made.".to_string()
                                                        } else if !preferences.loaded.get() {
                                                            "The player waits for this browser's saved gateway preference before selecting a source.".to_string()
                                                        } else if delivery_failed.get() {
                                                            "Every selected gateway failed. Review Delivery Settings or retry later.".to_string()
                                                        } else {
                                                            "Choose an explicit gateway in Delivery Settings before playing this CID-addressed video.".to_string()
                                                        }
                                                    }}</p>
                                                </div>
                                            }.into_any()
                                        }
                                    }}

                                    <p class="video-delivery-status" role="status">
                                        {move || {
                                            if !has_cid {
                                                "Fixture metadata only · playback intentionally unavailable".to_string()
                                            } else if !preferences.loaded.get() {
                                                "Loading this browser's delivery preference before selecting a source…".to_string()
                                            } else if listing_candidates(
                                                &listing,
                                                "content",
                                                listing.content_cid.as_deref(),
                                                preferences.gateway_origins.get().as_deref(),
                                            ).is_empty() {
                                                "No delivery route is selected. Configure a trusted gateway in Delivery Settings.".to_string()
                                            } else if delivery_failed.get() {
                                                "Every selected route failed. Retry later or review Delivery Settings.".to_string()
                                            } else {
                                                "Media is routed through an explicitly selected gateway. Content integrity is not verified by this UI.".to_string()
                                            }
                                        }}
                                    </p>

                                    <div class="video-watch-description">
                                        <p>{description.clone()}</p>
                                        <div class="video-topic-row">{topics}</div>
                                        <p class="video-provenance-note">
                                            "Source identity and credits should be attached to the published listing. This interface does not infer ownership from a CID."
                                        </p>
                                    </div>
                                </div>

                                <aside class="video-watch-sidebar" aria-label="Video accessibility and chapters">
                                    <section class="video-accessibility">
                                        <h3>"Accessibility"</h3>
                                        {move || {
                                            if !listing.has_captions() {
                                                view! { <p class="video-capability unavailable">"Captions not provided"</p> }.into_any()
                                            } else if caption_url.get().is_some() {
                                                view! { <p class="video-capability available">"✓ Captions track linked"</p> }.into_any()
                                            } else {
                                                view! { <p class="video-capability unavailable">"Captions asset exists, but no trusted route is available"</p> }.into_any()
                                            }
                                        }}
                                        {move || {
                                            if !listing.has_transcript() {
                                                view! { <p class="video-capability unavailable">"Transcript not provided"</p> }.into_any()
                                            } else if let Some(url) = transcript_url.get() {
                                                view! { <a class="video-resource-link" href=url target="_blank" rel="noopener noreferrer">"Open transcript ↗"</a> }.into_any()
                                            } else {
                                                view! { <p class="video-capability unavailable">"Transcript asset exists, but no trusted route is available"</p> }.into_any()
                                            }
                                        }}
                                        {move || {
                                            if !listing.has_visual_description() {
                                                view! { <p class="video-capability unavailable">"Visual description not provided"</p> }.into_any()
                                            } else if let Some(url) = description_url.get() {
                                                view! { <a class="video-resource-link" href=url target="_blank" rel="noopener noreferrer">"Open visual description ↗"</a> }.into_any()
                                            } else {
                                                view! { <p class="video-capability unavailable">"Description asset exists, but no trusted route is available"</p> }.into_any()
                                            }
                                        }}
                                        <p class="video-accessibility-note">"Accessibility assets are separate CID-addressed resources; the catalogue must publish language and format metadata with each one."</p>
                                    </section>

                                    <section class="video-chapters">
                                        <h3>"Chapters"</h3>
                                        {if listing.chapters.is_empty() {
                                            view! { <p class="video-muted">"No chapter markers published."</p> }.into_any()
                                        } else {
                                            view! { <ol class="video-chapter-list">{chapters}</ol> }.into_any()
                                        }}
                                    </section>

                                    <section class="video-resume">
                                        <h3>"Continue watching"</h3>
                                        {if let Some(seconds) = saved_time {
                                            view! {
                                                <p>"Saved here: " {format_duration(seconds)}</p>
                                                <div class="video-progress-track" role="progressbar"
                                                    aria-label="Local watch progress"
                                                    aria-valuemin="0"
                                                    aria-valuemax="100"
                                                    aria-valuenow=format!("{progress_pct:.0}")
                                                >
                                                    <span style=format!("width: {progress_pct:.0}%")></span>
                                                </div>
                                                <p class="video-muted">"Resume position stays in this browser and is never shared."</p>
                                            }.into_any()
                                        } else {
                                            view! { <p class="video-muted">"Watch progress will appear here after playback begins and is saved locally."</p> }.into_any()
                                        }}
                                    </section>
                                </aside>
                            </div>
                        </section>
                    }
                })
            }}
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duration_formatting_handles_hours_and_short_clips() {
        assert_eq!(format_duration(42), "0:42");
        assert_eq!(format_duration(8 * 60 + 42), "8:42");
        assert_eq!(format_duration(1 * 3_600 + 2 * 60 + 3), "1:02:03");
    }

    #[test]
    fn fixture_listings_never_attach_fake_playback_or_accessibility_urls() {
        for video in fixture_videos() {
            assert!(video.content_cid.is_none(), "fixture source must remain absent");
            assert!(video.poster_cid.is_none(), "fixture poster CID must remain absent");
            assert!(video.captions_cid.is_none(), "fixture captions must remain absent");
            assert!(video.transcript_cid.is_none(), "fixture transcript must remain absent");
            assert!(video.visual_description_cid.is_none(), "fixture visual description must remain absent");
        }
    }

    #[test]
    fn local_resume_key_is_namespaced_and_stable() {
        assert_eq!(resume_storage_key("film-1"), "mycelix-video-progress-v1:film-1");
    }
}
