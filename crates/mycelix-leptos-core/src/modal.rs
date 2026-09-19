// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Modal and confirmation dialog components.

use leptos::prelude::*;

/// Modal size variant.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ModalSize {
    Small,
    #[default]
    Medium,
    Large,
}

impl ModalSize {
    fn css_class(&self) -> &'static str {
        match self {
            ModalSize::Small => "modal-sm",
            ModalSize::Medium => "modal-md",
            ModalSize::Large => "modal-lg",
        }
    }
}

/// Modal dialog using the native HTML `<dialog>` top-layer primitive.
///
/// Opening with `showModal()` delegates focus containment, outside-page
/// inertness, Escape behavior, and normal invoker focus restoration to the
/// browser instead of reimplementing those semantics in Rust/JavaScript.
///
/// Backdrop click still requests closure through `on_close`; inner content
/// stops propagation so normal interaction does not dismiss the dialog.
#[component]
pub fn Modal(
    open: ReadSignal<bool>,
    on_close: Callback<()>,
    #[prop(optional, into)] title: Option<String>,
    #[prop(optional, into)] aria_label: Option<String>,
    #[prop(optional)] size: ModalSize,
    children: Children,
) -> impl IntoView {
    let size_class = size.css_class();
    let rendered = children();
    let accessible_name = aria_label
        .or_else(|| title.clone())
        .unwrap_or_else(|| "Dialog".to_string());
    let dialog_ref = NodeRef::<leptos::html::Dialog>::new();

    Effect::new(move |_| {
        let should_be_open = open.get();
        let Some(dialog) = dialog_ref.get() else {
            return;
        };

        if should_be_open {
            if !dialog.open() {
                if let Err(error) = dialog.show_modal() {
                    web_sys::console::error_1(&error);
                }
            }
        } else if dialog.open() {
            dialog.close();
        }
    });

    view! {
        <dialog
            node_ref=dialog_ref
            class="modal-backdrop"
            aria-label=accessible_name
            on:cancel=move |ev| {
                // Keep the reactive `open` signal authoritative. Prevent the
                // browser from closing independently, then request closure.
                ev.prevent_default();
                on_close.run(());
            }
            on:click=move |_| on_close.run(())
        >
            <div
                class=format!("modal-content {size_class}")
                on:click=move |ev| ev.stop_propagation()
            >
                {title.map(|t| view! {
                    <div class="modal-header">
                        <h2 class="modal-title">{t}</h2>
                    </div>
                })}
                <div class="modal-body">
                    {rendered}
                </div>
            </div>
        </dialog>
    }
}

/// Confirmation dialog.
#[component]
pub fn ConfirmDialog(
    open: ReadSignal<bool>,
    #[prop(into)] title: String,
    #[prop(into)] message: String,
    on_confirm: Callback<()>,
    on_cancel: Callback<()>,
    #[prop(optional, default = "Confirm")] confirm_label: &'static str,
    #[prop(optional)] danger: bool,
) -> impl IntoView {
    view! {
        <Modal open on_close=on_cancel title=title size=ModalSize::Small>
            <p class="confirm-message">{message}</p>
            <div class="modal-footer">
                <button class="btn btn-ghost" on:click=move |_| on_cancel.run(())>
                    "Cancel"
                </button>
                <button
                    class=if danger { "btn btn-danger" } else { "btn btn-primary" }
                    on:click=move |_| on_confirm.run(())
                >
                    {confirm_label}
                </button>
            </div>
        </Modal>
    }
}
