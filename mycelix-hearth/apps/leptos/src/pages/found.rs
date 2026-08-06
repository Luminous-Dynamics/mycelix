// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! The Founding Ceremony — creating a Hearth is a threshold crossing.
//!
//! Demo mode previews the ceremony locally. Live mode creates a private clone,
//! writes the founder-authored Hearth record to that exact cell, decodes the
//! returned record, and only then reveals the First Light passage.

use hearth_leptos_types::HearthType;
use leptos::prelude::*;
use leptos::task::spawn_local;
use mycelix_leptos_core::holochain_provider::use_holochain;

use crate::components::{FlameMode, HearthFlame};
use crate::hearth_context::{request_live_refresh, use_hearth};
use crate::hearth_founding::found_private_hearth;

#[derive(Clone, Copy, PartialEq)]
enum CeremonyStage {
    Naming,
    Intention,
    FirstLight,
    Invitation,
}

#[derive(Clone, Copy, PartialEq)]
enum HearthKind {
    Chosen,
    Nuclear,
    Extended,
    Intentional,
    CoPod,
}

impl HearthKind {
    fn label(self) -> &'static str {
        match self {
            Self::Chosen => "Chosen Family",
            Self::Nuclear => "Nuclear Family",
            Self::Extended => "Extended Family",
            Self::Intentional => "Intentional Community",
            Self::CoPod => "Co-Living Pod",
        }
    }

    fn description(self) -> &'static str {
        match self {
            Self::Chosen => "The people you chose. Bonds of intention, not just blood.",
            Self::Nuclear => "Parents and children. The inner ring of care.",
            Self::Extended => "Grandparents, aunts, cousins. The full tree.",
            Self::Intentional => "A community united by shared purpose and values.",
            Self::CoPod => "Housemates, co-workers, creative collaborators.",
        }
    }

    fn icon(self) -> &'static str {
        match self {
            Self::Chosen => "\u{1F91D}",
            Self::Nuclear => "\u{1F3E0}",
            Self::Extended => "\u{1F333}",
            Self::Intentional => "\u{1F331}",
            Self::CoPod => "\u{2728}",
        }
    }

    fn hearth_type(self) -> HearthType {
        match self {
            Self::Chosen => HearthType::Chosen,
            Self::Nuclear => HearthType::Nuclear,
            Self::Extended => HearthType::Extended,
            Self::Intentional => HearthType::Intentional,
            Self::CoPod => HearthType::CoPod,
        }
    }
}

const ALL_KINDS: [HearthKind; 5] = [
    HearthKind::Chosen,
    HearthKind::Nuclear,
    HearthKind::Extended,
    HearthKind::Intentional,
    HearthKind::CoPod,
];

#[component]
pub fn FoundingCeremony() -> impl IntoView {
    let hc = use_holochain();
    let hearth = use_hearth();
    let is_demo = hc.is_mock();

    let (stage, set_stage) = signal(CeremonyStage::Naming);
    let (hearth_name, set_hearth_name) = signal(String::new());
    let (hearth_kind, set_hearth_kind) = signal(None::<HearthKind>);
    let (founding, set_founding) = signal(false);
    let (founding_error, set_founding_error) = signal(None::<String>);
    let (created_clone_id, set_created_clone_id) = signal(None::<String>);

    let hc_for_readiness = hc.clone();
    let live_ready = Memo::new(move |_| is_demo || hc_for_readiness.zome_calls_ready());
    let hc_for_submit = hc.clone();
    let hearth_for_submit = hearth.clone();

    view! {
        <div class="founding-ceremony">
            <HearthFlame mode=FlameMode::Full />

            <div class="ceremony-content">
                {move || {
                    if stage.get() != CeremonyStage::Naming {
                        return view! { <div /> }.into_any();
                    }
                    view! {
                        <div class="ceremony-stage naming-stage">
                            <h1 class="ceremony-title">"The Naming"</h1>
                            <p class="ceremony-prompt">
                                "Every hearth begins with a name."
                                <br />
                                "What will you call this place of belonging?"
                            </p>
                            <div class="naming-input-wrap">
                                <input
                                    type="text"
                                    class="naming-input"
                                    placeholder="..."
                                    maxlength="64"
                                    autofocus=true
                                    on:input=move |ev| set_hearth_name.set(event_target_value(&ev))
                                    prop:value=hearth_name
                                />
                                <div class="naming-underline" />
                            </div>
                            {move || (hearth_name.get().trim().chars().count() >= 2).then(|| view! {
                                <button
                                    class="ceremony-btn"
                                    on:click=move |_| set_stage.set(CeremonyStage::Intention)
                                >
                                    "continue"
                                </button>
                            })}
                        </div>
                    }.into_any()
                }}

                {move || {
                    if stage.get() != CeremonyStage::Intention {
                        return view! { <div /> }.into_any();
                    }
                    view! {
                        <div class="ceremony-stage intention-stage">
                            <h1 class="ceremony-title">"The Intention"</h1>
                            <p class="ceremony-prompt">
                                "What kind of family is "
                                <span class="hearth-name-echo">{hearth_name}</span>
                                "?"
                            </p>
                            <div class="intention-grid">
                                {ALL_KINDS.iter().map(|kind| {
                                    let kind = *kind;
                                    view! {
                                        <button
                                            class=move || if hearth_kind.get() == Some(kind) {
                                                "intention-card selected"
                                            } else {
                                                "intention-card"
                                            }
                                            disabled=founding
                                            on:click=move |_| set_hearth_kind.set(Some(kind))
                                        >
                                            <span class="intention-icon">{kind.icon()}</span>
                                            <span class="intention-label">{kind.label()}</span>
                                            <span class="intention-desc">{kind.description()}</span>
                                        </button>
                                    }
                                }).collect::<Vec<_>>()}
                            </div>

                            <Show when=move || founding_error.get().is_some()>
                                <p class="ceremony-error" role="alert">
                                    {move || founding_error.get().unwrap_or_default()}
                                </p>
                            </Show>

                            {move || hearth_kind.get().map(|kind| {
                                let hc = hc_for_submit.clone();
                                let hearth = hearth_for_submit.clone();
                                view! {
                                    <button
                                        class="ceremony-btn"
                                        disabled=move || {
                                            founding.get() || !live_ready.get()
                                        }
                                        on:click=move |_| {
                                            if founding.get_untracked() {
                                                return;
                                            }
                                            set_founding_error.set(None);
                                            if is_demo {
                                                set_stage.set(CeremonyStage::FirstLight);
                                                return;
                                            }

                                            set_founding.set(true);
                                            let hc = hc.clone();
                                            let hearth = hearth.clone();
                                            let name = hearth_name.get_untracked();
                                            spawn_local(async move {
                                                match found_private_hearth(&hc, name, kind.hearth_type()).await {
                                                    Ok(founded) => {
                                                        hearth.current_hearth.set(Some(founded.hearth));
                                                        set_created_clone_id.set(Some(
                                                            founded.clone_cell.clone_id,
                                                        ));
                                                        request_live_refresh(hearth, hc);
                                                        set_stage.set(CeremonyStage::FirstLight);
                                                    }
                                                    Err(error) => set_founding_error.set(Some(error)),
                                                }
                                                set_founding.set(false);
                                            });
                                        }
                                    >
                                        {move || if founding.get() {
                                            "lighting the private fire…"
                                        } else if is_demo {
                                            "preview the first light"
                                        } else {
                                            "light the private fire"
                                        }}
                                    </button>
                                }
                            })}

                            <Show when=move || !live_ready.get()>
                                <p class="ceremony-status" role="status">
                                    "Live founding waits for an authenticated conductor and authorized signer."
                                </p>
                            </Show>
                        </div>
                    }.into_any()
                }}

                {move || {
                    if stage.get() != CeremonyStage::FirstLight {
                        return view! { <div /> }.into_any();
                    }
                    view! {
                        <div class="ceremony-stage firstlight-stage">
                            <div class="candle-animation">
                                <div class="candle-wick" />
                                <div class="candle-flame-inner" />
                                <div class="candle-glow-expand" />
                            </div>
                            <h1 class="ceremony-title firstlight-title">{hearth_name}</h1>
                            <p class="ceremony-prompt firstlight-subtitle">
                                {if is_demo {
                                    "is glowing in this local preview"
                                } else {
                                    "has been lit in its private network"
                                }}
                            </p>
                            <button
                                class="ceremony-btn"
                                on:click=move |_| set_stage.set(CeremonyStage::Invitation)
                            >
                                "continue"
                            </button>
                        </div>
                    }.into_any()
                }}

                {move || {
                    if stage.get() != CeremonyStage::Invitation {
                        return view! { <div /> }.into_any();
                    }
                    let href = if is_demo {
                        "/?mode=demo".to_string()
                    } else {
                        created_clone_id
                            .get()
                            .map(|clone_id| crate::hearth_clone::live_clone_url(&clone_id))
                            .unwrap_or_else(|| "/?mode=live".into())
                    };
                    view! {
                        <div class="ceremony-stage invitation-stage">
                            <h1 class="ceremony-title">"The Invitation"</h1>
                            <p class="ceremony-prompt">"Who will you bring to this fire?"</p>
                            <div class="ceremony-truth-note" role="note">
                                {if is_demo {
                                    "Invitation is not sent in Demo mode."
                                } else {
                                    "A DID alone cannot join this private network. Share an encrypted clone handoff first, then invite the agent after they have joined."
                                }}
                            </div>
                            <div class="invitation-actions">
                                <a href=href class="ceremony-btn">
                                    {if is_demo {
                                        "enter the demo hearth"
                                    } else {
                                        "enter the private hearth"
                                    }}
                                </a>
                            </div>
                        </div>
                    }.into_any()
                }}
            </div>
        </div>
    }
}
