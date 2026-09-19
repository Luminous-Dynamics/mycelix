// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared form components for Mycelix cluster frontends.
//!
//! Uses the `.form-input`, `.form-textarea`, `.form-select`, `.form-label`
//! classes defined in base.css.

use leptos::prelude::*;

/// Stable semantic IDs shared by a field wrapper and its form control.
///
/// Passing the same value to [`FormField`] and a child control associates the
/// visible label with the control and reserves a stable ID for inline error
/// description. Existing call sites remain valid because this contract is
/// additive.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FormFieldIds {
    pub control: String,
    pub error: String,
}

impl FormFieldIds {
    pub fn new(control_id: impl Into<String>) -> Self {
        let control = control_id.into();
        let error = format!("{control}-error");
        Self { control, error }
    }
}

/// A labeled form field with optional error message.
#[component]
pub fn FormField(
    #[prop(into)] label: String,
    #[prop(optional, into)] error: Option<String>,
    #[prop(optional)] required: bool,
    #[prop(optional)] ids: Option<FormFieldIds>,
    children: Children,
) -> impl IntoView {
    let label_for = ids.as_ref().map(|ids| ids.control.clone());
    let error_id = ids.as_ref().map(|ids| ids.error.clone());

    view! {
        <div class="form-field">
            <label class="form-label" for=label_for>
                {label}
                {required.then(|| view! { <span class="form-required" aria-hidden="true">" *"</span> })}
                {required.then(|| view! { <span class="sr-only">" Required"</span> })}
            </label>
            {children()}
            {error.map(|e| view! {
                <span class="form-error" id=error_id role="alert">{e}</span>
            })}
        </div>
    }
}

/// Option for a Select component.
#[derive(Clone, Debug)]
pub struct SelectOption {
    pub value: String,
    pub label: String,
}

/// Text input.
#[component]
pub fn TextInput(
    value: ReadSignal<String>,
    on_change: Callback<String>,
    #[prop(optional)] placeholder: &'static str,
    #[prop(optional, default = "text")] input_type: &'static str,
    #[prop(optional)] disabled: bool,
    #[prop(optional)] required: bool,
    #[prop(optional)] invalid: bool,
    #[prop(optional)] ids: Option<FormFieldIds>,
) -> impl IntoView {
    let control_id = ids.as_ref().map(|ids| ids.control.clone());
    let described_by = if invalid {
        ids.as_ref().map(|ids| ids.error.clone())
    } else {
        None
    };

    view! {
        <input
            class="form-input"
            id=control_id
            type=input_type
            placeholder=placeholder
            disabled=disabled
            aria-required=required.to_string()
            aria-invalid=invalid.to_string()
            aria-describedby=described_by
            prop:value=move || value.get()
            on:input=move |ev| {
                on_change.run(event_target_value(&ev));
            }
        />
    }
}

/// Textarea.
#[component]
pub fn TextArea(
    value: ReadSignal<String>,
    on_change: Callback<String>,
    #[prop(optional, default = 4)] rows: u32,
    #[prop(optional)] placeholder: &'static str,
    #[prop(optional)] disabled: bool,
    #[prop(optional)] required: bool,
    #[prop(optional)] invalid: bool,
    #[prop(optional)] ids: Option<FormFieldIds>,
) -> impl IntoView {
    let control_id = ids.as_ref().map(|ids| ids.control.clone());
    let described_by = if invalid {
        ids.as_ref().map(|ids| ids.error.clone())
    } else {
        None
    };

    view! {
        <textarea
            class="form-textarea"
            id=control_id
            rows=rows
            placeholder=placeholder
            disabled=disabled
            aria-required=required.to_string()
            aria-invalid=invalid.to_string()
            aria-describedby=described_by
            prop:value=move || value.get()
            on:input=move |ev| {
                on_change.run(event_target_value(&ev));
            }
        ></textarea>
    }
}

/// Select dropdown.
#[component]
pub fn Select(
    value: ReadSignal<String>,
    on_change: Callback<String>,
    #[prop(into)] options: Vec<SelectOption>,
    #[prop(optional)] disabled: bool,
    #[prop(optional)] required: bool,
    #[prop(optional)] invalid: bool,
    #[prop(optional)] ids: Option<FormFieldIds>,
) -> impl IntoView {
    let control_id = ids.as_ref().map(|ids| ids.control.clone());
    let described_by = if invalid {
        ids.as_ref().map(|ids| ids.error.clone())
    } else {
        None
    };

    view! {
        <select
            class="form-select"
            id=control_id
            disabled=disabled
            aria-required=required.to_string()
            aria-invalid=invalid.to_string()
            aria-describedby=described_by
            prop:value=move || value.get()
            on:change=move |ev| {
                on_change.run(event_target_value(&ev));
            }
        >
            {options.into_iter().map(|opt| {
                let val = opt.value.clone();
                view! {
                    <option value=val>{opt.label}</option>
                }
            }).collect_view()}
        </select>
    }
}

/// Checkbox with label.
#[component]
pub fn Checkbox(
    checked: ReadSignal<bool>,
    on_toggle: Callback<bool>,
    #[prop(into)] label: String,
) -> impl IntoView {
    view! {
        <label class="form-checkbox">
            <input
                type="checkbox"
                prop:checked=move || checked.get()
                on:change=move |ev| {
                    on_toggle.run(event_target_checked(&ev));
                }
            />
            <span>{label}</span>
        </label>
    }
}

fn event_target_checked(ev: &leptos::ev::Event) -> bool {
    use wasm_bindgen::JsCast;
    ev.target()
        .and_then(|t| t.dyn_into::<web_sys::HtmlInputElement>().ok())
        .map(|el| el.checked())
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::FormFieldIds;

    #[test]
    fn field_ids_bind_control_and_error_names_deterministically() {
        let ids = FormFieldIds::new("profile-name");
        assert_eq!(ids.control, "profile-name");
        assert_eq!(ids.error, "profile-name-error");
    }
}
