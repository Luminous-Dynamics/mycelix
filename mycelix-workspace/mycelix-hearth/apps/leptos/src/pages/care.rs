// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use crate::care_actions::{self, CreateCareDraft};
use crate::hearth_actions;
use crate::hearth_context::{member_name, use_hearth};
use hearth_leptos_types::*;
use leptos::prelude::*;
use wasm_bindgen::JsCast;

fn care_type_from_value(value: &str) -> CareType {
    match value {
        "Childcare" => CareType::Childcare,
        "Eldercare" => CareType::Eldercare,
        "PetCare" => CareType::PetCare,
        "MealPrep" => CareType::MealPrep,
        "Medical" => CareType::Medical,
        "Emotional" => CareType::Emotional,
        _ => CareType::Chore,
    }
}

fn recurrence_from_value(value: &str) -> Recurrence {
    match value {
        "Weekly" => Recurrence::Weekly,
        "Monthly" => Recurrence::Monthly,
        _ => Recurrence::Daily,
    }
}

#[component]
pub fn CarePage() -> impl IntoView {
    let hearth = use_hearth();

    let (composer_open, set_composer_open) = signal(false);
    let (title, set_title) = signal(String::new());
    let (description, set_description) = signal(String::new());
    let (notes, set_notes) = signal(String::new());
    let (assigned_to, set_assigned_to) = signal(String::new());
    let (care_type, set_care_type) = signal(CareType::Chore);
    let (recurrence, set_recurrence) = signal(Recurrence::Daily);

    let hearth_for_open = hearth.clone();
    let hearth_for_assignees = hearth.clone();

    view! {
        <div class="page care-page">
            <h1 class="page-title">"care board"</h1>
            <p class="page-subtitle">"schedules, swaps, and shared meals"</p>

            <button
                class="action-btn"
                type="button"
                aria-expanded=move || composer_open.get().to_string()
                aria-controls="care-task-composer"
                on:click=move |_| {
                    if assigned_to.get_untracked().is_empty() {
                        if let Some(member) = hearth_for_open
                            .members
                            .get_untracked()
                            .into_iter()
                            .find(|member| member.status == MembershipStatus::Active)
                        {
                            set_assigned_to.set(member.agent);
                        }
                    }
                    set_composer_open.update(|open| *open = !*open);
                }
            >
                {move || if composer_open.get() { "Close care task form" } else { "+ New Care Task" }}
            </button>

            <Show when=move || composer_open.get()>
                <form
                    id="care-task-composer"
                    class="care-composer"
                    aria-labelledby="care-task-composer-heading"
                    on:submit=move |event| {
                        event.prevent_default();
                        let scheduled = care_actions::create_care_schedule(CreateCareDraft {
                            care_type: care_type.get(),
                            title: title.get(),
                            description: description.get(),
                            assigned_to: assigned_to.get(),
                            recurrence: recurrence.get(),
                            notes: notes.get(),
                        });

                        if scheduled {
                            set_title.set(String::new());
                            set_description.set(String::new());
                            set_notes.set(String::new());
                            set_assigned_to.set(String::new());
                            set_care_type.set(CareType::Chore);
                            set_recurrence.set(Recurrence::Daily);
                            set_composer_open.set(false);
                        }
                    }
                >
                    <h2 id="care-task-composer-heading">"New care task"</h2>
                    <p class="task-surface-note">
                        "Submitting this form does not grant permission. In a live Hearth, the care zome independently checks civic eligibility and Hearth membership before creating anything."
                    </p>

                    <div class="form-row">
                        <label for="care-task-title">"Title"</label>
                        <input
                            id="care-task-title"
                            type="text"
                            maxlength="256"
                            required=true
                            prop:value=move || title.get()
                            on:input=move |event| {
                                if let Some(input) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlInputElement>().cloned())
                                {
                                    set_title.set(input.value());
                                }
                            }
                        />
                    </div>

                    <div class="form-row">
                        <label for="care-task-type">"Type"</label>
                        <select
                            id="care-task-type"
                            on:change=move |event| {
                                if let Some(select) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlSelectElement>().cloned())
                                {
                                    set_care_type.set(care_type_from_value(&select.value()));
                                }
                            }
                        >
                            <option value="Chore">"Chore"</option>
                            <option value="Childcare">"Childcare"</option>
                            <option value="Eldercare">"Eldercare"</option>
                            <option value="PetCare">"Pet care"</option>
                            <option value="MealPrep">"Meal prep"</option>
                            <option value="Medical">"Medical"</option>
                            <option value="Emotional">"Emotional"</option>
                        </select>
                    </div>

                    <div class="form-row">
                        <label for="care-task-assignee">"Assigned to"</label>
                        <select
                            id="care-task-assignee"
                            required=true
                            prop:value=move || assigned_to.get()
                            on:change=move |event| {
                                if let Some(select) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlSelectElement>().cloned())
                                {
                                    set_assigned_to.set(select.value());
                                }
                            }
                        >
                            <option value="">"Choose an active Hearth member"</option>
                            {move || {
                                hearth_for_assignees
                                    .members
                                    .get()
                                    .into_iter()
                                    .filter(|member| member.status == MembershipStatus::Active)
                                    .map(|member| {
                                        view! {
                                            <option value=member.agent>{member.display_name}</option>
                                        }
                                    })
                                    .collect_view()
                            }}
                        </select>
                    </div>

                    <div class="form-row">
                        <label for="care-task-recurrence">"Recurrence"</label>
                        <select
                            id="care-task-recurrence"
                            on:change=move |event| {
                                if let Some(select) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlSelectElement>().cloned())
                                {
                                    set_recurrence.set(recurrence_from_value(&select.value()));
                                }
                            }
                        >
                            <option value="Daily">"Daily"</option>
                            <option value="Weekly">"Weekly"</option>
                            <option value="Monthly">"Monthly"</option>
                        </select>
                    </div>

                    <div class="form-row">
                        <label for="care-task-description">"Description"</label>
                        <textarea
                            id="care-task-description"
                            maxlength="4096"
                            prop:value=move || description.get()
                            on:input=move |event| {
                                if let Some(textarea) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlTextAreaElement>().cloned())
                                {
                                    set_description.set(textarea.value());
                                }
                            }
                        ></textarea>
                    </div>

                    <div class="form-row">
                        <label for="care-task-notes">"Notes"</label>
                        <textarea
                            id="care-task-notes"
                            maxlength="4096"
                            prop:value=move || notes.get()
                            on:input=move |event| {
                                if let Some(textarea) = event
                                    .target()
                                    .and_then(|target| target.dyn_ref::<web_sys::HtmlTextAreaElement>().cloned())
                                {
                                    set_notes.set(textarea.value());
                                }
                            }
                        ></textarea>
                    </div>

                    <div class="vote-actions">
                        <button
                            class="action-btn"
                            type="submit"
                            disabled=move || title.get().trim().is_empty() || assigned_to.get().is_empty()
                        >
                            "Create care task"
                        </button>
                        <button
                            class="cancel-btn"
                            type="button"
                            on:click=move |_| set_composer_open.set(false)
                        >
                            "Cancel"
                        </button>
                    </div>
                </form>
            </Show>

            {move || {
                let members = hearth.members.get();
                let schedules = hearth.care_schedules.get();
                let active: Vec<_> = schedules
                    .iter()
                    .filter(|care| care.status == CareScheduleStatus::Active)
                    .cloned()
                    .collect();
                let completed: Vec<_> = schedules
                    .iter()
                    .filter(|care| care.status == CareScheduleStatus::Completed)
                    .cloned()
                    .collect();

                view! {
                    <section>
                        <h2>{format!("Active ({})", active.len())}</h2>
                        {if active.is_empty() {
                            view! {
                                <div class="empty-state">"nothing to tend right now. the hearth hums quietly."</div>
                            }
                                .into_any()
                        } else {
                            view! {
                                <div>
                                    {active
                                        .iter()
                                        .map(|care| {
                                            let assigned = member_name(&members, &care.assigned_to);
                                            let care_label = care.care_type.label().to_string();
                                            let title = care.title.clone();
                                            let desc = care.description.clone();
                                            let recurrence = format!("{:?}", care.recurrence);
                                            let hash = care.hash.clone();
                                            view! {
                                                <div class="care-card">
                                                    <div class="care-header">
                                                        <span class="care-type-badge">{care_label}</span>
                                                        <span class="care-title">{title}</span>
                                                    </div>
                                                    <p class="care-desc">{desc}</p>
                                                    <div class="care-footer">
                                                        <span class="care-assigned">{assigned}</span>
                                                        <span class="care-recurrence">{recurrence}</span>
                                                        <button
                                                            class="complete-btn"
                                                            type="button"
                                                            on:click={
                                                                let hash = hash.clone();
                                                                move |_| hearth_actions::complete_care_task(hash.clone())
                                                            }
                                                        >
                                                            "✓ Done"
                                                        </button>
                                                    </div>
                                                </div>
                                            }
                                        })
                                        .collect_view()}
                                </div>
                            }
                                .into_any()
                        }}
                    </section>

                    {(!completed.is_empty()).then(|| {
                        view! {
                            <section>
                                <h2>{format!("Completed ({})", completed.len())}</h2>
                                {completed
                                    .iter()
                                    .map(|care| {
                                        let title = care.title.clone();
                                        let care_label = care.care_type.label().to_string();
                                        view! {
                                            <div class="care-card care-completed">
                                                <span class="care-type-badge">{care_label}</span>
                                                <span class="care-title">{title}</span>
                                                <span class="care-done-mark">"✓"</span>
                                            </div>
                                        }
                                    })
                                    .collect_view()}
                            </section>
                        }
                    })}
                }
            }}
        </div>
    }
}

#[cfg(test)]
mod tests {
    use super::{care_type_from_value, recurrence_from_value};
    use hearth_leptos_types::{CareType, Recurrence};

    #[test]
    fn composer_maps_only_explicit_built_in_care_options() {
        assert_eq!(care_type_from_value("Medical"), CareType::Medical);
        assert_eq!(care_type_from_value("not-a-type"), CareType::Chore);
        assert_eq!(recurrence_from_value("Monthly"), Recurrence::Monthly);
        assert_eq!(recurrence_from_value("not-a-recurrence"), Recurrence::Daily);
    }
}
