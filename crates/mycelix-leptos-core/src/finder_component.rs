// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Accessible local Finder combobox over [`LocalFinderCatalog`].
//!
//! The component owns text entry, popup state, keyboard navigation, and
//! selection presentation only. It does not route, execute actions, query
//! remote providers, or infer authority from a selected result.

use leptos::prelude::*;

use crate::finder::FinderEntry;
use crate::finder_catalog::{FinderMatch, LocalFinderCatalog};

/// Local Finder combobox using a listbox popup and `aria-activedescendant`.
#[component]
pub fn LocalFinder(
    #[prop(into)] id: String,
    catalog: LocalFinderCatalog,
    on_select: Callback<FinderEntry>,
    #[prop(optional, default = 10)] limit: usize,
    #[prop(optional, default = "Find in Mycelix")] aria_label: &'static str,
    #[prop(optional, default = "Find people, work, and actions...")] placeholder: &'static str,
) -> impl IntoView {
    let input_id = format!("{id}-input");
    let list_id = format!("{id}-results");
    let list_id_for_keys = list_id.clone();
    let list_id_for_active = list_id.clone();
    let list_id_for_options = list_id.clone();
    let search_catalog = catalog.clone();
    let on_select_keyboard = on_select.clone();
    let on_select_pointer = on_select.clone();

    let (query, set_query) = signal(String::new());
    let (results, set_results) = signal(Vec::<FinderMatch>::new());
    let (active, set_active) = signal(None::<usize>);
    let (expanded, set_expanded) = signal(false);

    view! {
        <div
            class="local-finder"
            on:focusout=move |_| {
                set_expanded.set(false);
                set_active.set(None);
            }
        >
            <input
                id=input_id
                class="form-input finder-input"
                type="search"
                role="combobox"
                aria-label=aria_label
                aria-autocomplete="list"
                aria-controls=list_id.clone()
                aria-expanded=move || expanded.get().to_string()
                aria-activedescendant=move || {
                    if !expanded.get() {
                        return None;
                    }
                    active
                        .get()
                        .map(|index| format!("{list_id_for_active}-option-{index}"))
                }
                placeholder=placeholder
                prop:value=move || query.get()
                on:input=move |ev| {
                    let value = event_target_value(&ev);
                    let next_results = search_catalog.search(&value, limit);
                    let has_results = !next_results.is_empty();

                    set_query.set(value);
                    set_results.set(next_results);
                    set_active.set(None);
                    set_expanded.set(has_results);
                }
                on:keydown=move |ev| {
                    let len = results.get_untracked().len();
                    match ev.key().as_str() {
                        "ArrowDown" if len > 0 => {
                            ev.prevent_default();
                            let next = next_index(active.get_untracked(), len);
                            set_active.set(Some(next));
                            set_expanded.set(true);
                            scroll_active_option(&list_id_for_keys, next);
                        }
                        "ArrowUp" if len > 0 => {
                            ev.prevent_default();
                            let next = previous_index(active.get_untracked(), len);
                            set_active.set(Some(next));
                            set_expanded.set(true);
                            scroll_active_option(&list_id_for_keys, next);
                        }
                        "Enter" if expanded.get_untracked() => {
                            let selected = active.get_untracked().and_then(|index| {
                                results
                                    .get_untracked()
                                    .get(index)
                                    .map(|item| item.entry.clone())
                            });
                            if let Some(entry) = selected {
                                ev.prevent_default();
                                on_select_keyboard.run(entry);
                                set_expanded.set(false);
                                set_active.set(None);
                            }
                        }
                        "Escape" if expanded.get_untracked() => {
                            ev.prevent_default();
                            set_expanded.set(false);
                            set_active.set(None);
                        }
                        _ => {}
                    }
                }
            />

            <Show when=move || expanded.get() && !results.get().is_empty()>
                <ul
                    id=list_id
                    class="finder-results"
                    role="listbox"
                    aria-label="Finder suggestions"
                >
                    {move || {
                        let list_id = list_id_for_options.clone();
                        let on_select = on_select_pointer.clone();
                        results
                            .get()
                            .into_iter()
                            .enumerate()
                            .map(move |(index, result)| {
                                let option_id = format!("{list_id}-option-{index}");
                                let entry = result.entry;
                                let entry_for_pointer = entry.clone();
                                let on_select = on_select.clone();
                                view! {
                                    <li
                                        id=option_id
                                        class="finder-result"
                                        role="option"
                                        aria-selected=move || {
                                            (active.get() == Some(index)).to_string()
                                        }
                                        on:mousedown=move |ev| {
                                            // Keep DOM focus on the combobox while choosing
                                            // a pointer result, matching aria-activedescendant
                                            // focus management.
                                            ev.prevent_default();
                                            on_select.run(entry_for_pointer.clone());
                                            set_expanded.set(false);
                                            set_active.set(None);
                                        }
                                    >
                                        <span class="finder-result-kind">{entry.kind.label()}</span>
                                        <span class="finder-result-title">{entry.title}</span>
                                        {entry.subtitle.map(|subtitle| view! {
                                            <span class="finder-result-subtitle">{subtitle}</span>
                                        })}
                                    </li>
                                }
                            })
                            .collect_view()
                    }}
                </ul>
            </Show>
        </div>
    }
}

fn next_index(current: Option<usize>, len: usize) -> usize {
    match current {
        Some(index) if index + 1 < len => index + 1,
        Some(index) => index.min(len.saturating_sub(1)),
        None => 0,
    }
}

fn previous_index(current: Option<usize>, len: usize) -> usize {
    match current {
        Some(index) if index > 0 => index - 1,
        Some(_) => 0,
        None => len.saturating_sub(1),
    }
}

fn scroll_active_option(list_id: &str, index: usize) {
    let option_id = format!("{list_id}-option-{index}");
    set_timeout(
        move || {
            if let Some(element) = web_sys::window()
                .and_then(|window| window.document())
                .and_then(|document| document.get_element_by_id(&option_id))
            {
                element.scroll_into_view();
            }
        },
        std::time::Duration::from_millis(0),
    );
}

#[cfg(test)]
mod tests {
    use super::{next_index, previous_index};

    #[test]
    fn down_arrow_starts_at_first_and_stops_at_last() {
        assert_eq!(next_index(None, 3), 0);
        assert_eq!(next_index(Some(0), 3), 1);
        assert_eq!(next_index(Some(2), 3), 2);
    }

    #[test]
    fn up_arrow_starts_at_last_and_stops_at_first() {
        assert_eq!(previous_index(None, 3), 2);
        assert_eq!(previous_index(Some(2), 3), 1);
        assert_eq!(previous_index(Some(0), 3), 0);
    }
}
