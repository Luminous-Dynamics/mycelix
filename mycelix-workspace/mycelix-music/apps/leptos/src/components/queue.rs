// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::app::PlayerState;
use leptos::prelude::*;
use wasm_bindgen::JsCast;

/// Toggleable queue panel showing the current playlist.
#[component]
pub fn QueuePanel() -> impl IntoView {
    let player = expect_context::<PlayerState>();

    let show = player.show_queue;

    view! {
        <Show when=move || show.get()>
            <QueuePanelInner />
        </Show>
    }
}

#[component]
fn QueuePanelInner() -> impl IntoView {
    let player = expect_context::<PlayerState>();

    let clear_player = player.clone();
    let clear_queue = move |_| {
        clear_player.clear_queue();
    };

    let show_q = player.show_queue;
    let panel_ref = NodeRef::<leptos::html::Div>::new();

    // Remember the invoker when the dialog mounts. Restore focus after the
    // conditional panel unmounts (Escape, close button, or overlay click).
    let return_focus = web_sys::window()
        .and_then(|window| window.document())
        .and_then(|document| document.active_element())
        .and_then(|element| element.dyn_into::<web_sys::HtmlElement>().ok());
    on_cleanup(move || {
        if let Some(element) = return_focus {
            let _ = element.focus();
        }
    });

    // The dialog only exists while open. Move keyboard focus into it so
    // Escape and the panel's keyboard controls work immediately.
    Effect::new(move |_| {
        if let Some(panel) = panel_ref.get() {
            let _ = panel.focus();
        }
    });
    let close = move |_: web_sys::MouseEvent| {
        show_q.set(false);
    };

    let close2 = move |_: web_sys::MouseEvent| {
        show_q.set(false);
    };

    // Escape key closes the queue when the overlay has keyboard focus.
    let on_keydown = move |ev: web_sys::KeyboardEvent| {
        if ev.key() == "Escape" {
            show_q.set(false);
        }
    };

    view! {
        <>
            <div class="queue-overlay" on:click=close></div>
            <div
                class="queue-panel"
                node_ref=panel_ref
                role="dialog"
                aria-modal="true"
                aria-label="Playback queue"
                tabindex="-1"
                on:keydown=on_keydown
            >
                <div class="queue-header">
                    <div>
                        <h3>"Queue"</h3>
                        <span class="queue-subtitle">
                            {move || format!("{} tracks", player.queue.get().len())}
                        </span>
                    </div>
                    <div class="queue-actions">
                        <button class="btn-sm" on:click=clear_queue disabled=move || player.queue.get().is_empty()>
                            "Clear"
                        </button>
                        <button class="btn-sm" on:click=close2 aria-label="Close queue" title="Close queue">
                            "✕"
                        </button>
                    </div>
                </div>
                <div class="queue-list">
                    {move || {
                        let q = player.queue.get();
                        let current_idx = player.queue_index.get();
                        if q.is_empty() {
                            view! {
                                <div class="queue-empty">
                                    <p>"Your queue is empty."</p>
                                    <p>"Choose + Queue on any track to build a listening session."</p>
                                </div>
                            }.into_any()
                        } else {
                            q.into_iter()
                                .enumerate()
                                .map(|(i, song)| {
                                    let is_current = current_idx == Some(i);
                                    let class = if is_current {
                                        "queue-item current"
                                    } else {
                                        "queue-item"
                                    };
                                    let player_for_play = player.clone();
                                    let play_index = i;
                                    let play_this = move |_| {
                                        player_for_play.play_queued_song_at(play_index);
                                    };

                                    let player_for_remove = player.clone();
                                    let remove_index = i;
                                    let remove = move |_| {
                                        player_for_remove.remove_queued_song_at(remove_index);
                                    };

                                    view! {
                                        <div class=class>
                                            <button class="queue-play" on:click=play_this aria-label=format!("Play {}", song.title) title="Play track">
                                                {if is_current { "♫" } else { "▶" }}
                                            </button>
                                            <div class="queue-song-info">
                                                <span class="queue-title">{song.title.clone()}</span>
                                                <span class="queue-duration">{song.duration_display()}</span>
                                            </div>
                                            <button class="queue-remove" on:click=remove aria-label=format!("Remove {}", song.title) title="Remove from queue">
                                                "✕"
                                            </button>
                                        </div>
                                    }
                                })
                                .collect_view()
                                .into_any()
                        }
                    }}
                </div>
            </div>
        </>
    }
}
