// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Shared form components for Mycelix cluster frontends.
//!
//! Uses the `.form-input`, `.form-textarea`, `.form-select`, `.form-label`
//! classes defined in base.css.

use leptos::{context::Provider, prelude::*};

/// Accessibility metadata passed from FormField to its shared control components.
#[derive(Clone)]
struct FormFieldA11y {
    id: Option<String>,
    error_id: Option<String>,
    required: bool,
    invalid: bool,
}

fn merge_described_by(error_id: Option<String>, described_by: Option<String>) -> Option<String> {
    match (error_id, described_by) {
        (Some(error), Some(description)) => Some(format!("{description} {error}")),
        (Some(error), None) => Some(error),
        (None, description) => description,
    }
}

/// A labeled single-control field with an optional associated error message.
///
/// Pass an id when the error needs explicit programmatic association. TextInput,
/// TextArea, and Select inherit that id and error relationship through context.
/// Without an id, the label still wraps the control and provides implicit labeling.
/// Do not wrap Checkbox in FormField; Checkbox provides its own label.
#[component]
pub fn FormField(
    #[prop(into)] label: String,
    #[prop(optional, into)] id: Option<String>,
    #[prop(optional, into)] error: Option<String>,
    #[prop(optional)] required: bool,
    children: Children,
) -> impl IntoView {
    let invalid = error.is_some();
    let error_id = if invalid {
        id.as_ref().map(|field_id| format!("{field_id}-error"))
    } else {
        None
    };
    let context = FormFieldA11y {
        id: id.clone(),
        error_id: error_id.clone(),
        required,
        invalid,
    };

    view! {
        <Provider value=context>
            <div class="form-field">
                <label class="form-label" for=id.clone()>
                    {label}
                    {required.then(|| view! { <span class="form-required" aria-hidden="true">" *"</span> })}
                    {children()}
                </label>
                {error.map(|message| view! {
                    <span
                        class="form-error"
                        id=error_id.clone()
                        aria-live="polite"
                        aria-atomic="true"
                    >
                        {message}
                    </span>
                })}
            </div>
        </Provider>
    }
}

/// Option for a Select component.
#[derive(Clone, Debug)]
pub struct SelectOption {
    pub value: String,
    pub label: String,
}

/// Text input. Inherits label ID, required state, and error association from FormField.
#[component]
pub fn TextInput(
    value: ReadSignal<String>,
    on_change: Callback<String>,
    #[prop(optional, into)] id: Option<String>,
    #[prop(optional, into)] described_by: Option<String>,
    #[prop(optional)] placeholder: &'static str,
    #[prop(optional, default = "text")] input_type: &'static str,
    #[prop(optional)] disabled: bool,
    #[prop(optional)] required: bool,
    #[prop(optional)] invalid: bool,
) -> impl IntoView {
    let field = use_context::<FormFieldA11y>();
    let field_id = field.as_ref().and_then(|ctx| ctx.id.clone());
    let field_error_id = field.as_ref().and_then(|ctx| ctx.error_id.clone());
    let id = field_id.or(id);
    let described_by = merge_described_by(field_error_id, described_by);
    let required = required || field.as_ref().is_some_and(|ctx| ctx.required);
    let invalid = invalid || field.as_ref().is_some_and(|ctx| ctx.invalid);

    view! {
        <input
            class="form-input"
            id=id
            type=input_type
            placeholder=placeholder
            disabled=disabled
            required=required
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

/// Textarea. Inherits label ID, required state, and error association from FormField.
#[component]
pub fn TextArea(
    value: ReadSignal<String>,
    on_change: Callback<String>,
    #[prop(optional, into)] id: Option<String>,
    #[prop(optional, into)] described_by: Option<String>,
    #[prop(optional, default = 4)] rows: u32,
    #[prop(optional)] placeholder: &'static str,
    #[prop(optional)] required: bool,
    #[prop(optional)] invalid: bool,
) -> impl IntoView {
    let field = use_context::<FormFieldA11y>();
    let field_id = field.as_ref().and_then(|ctx| ctx.id.clone());
    let field_error_id = field.as_ref().and_then(|ctx| ctx.error_id.clone());
    let id = field_id.or(id);
    let described_by = merge_described_by(field_error_id, described_by);
    let required = required || field.as_ref().is_some_and(|ctx| ctx.required);
    let invalid = invalid || field.as_ref().is_some_and(|ctx| ctx.invalid);

    view! {
        <textarea
            class="form-textarea"
            id=id
            rows=rows
            placeholder=placeholder
            required=required
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

/// Select dropdown. Inherits label ID, required state, and error association from FormField.
#[component]
pub fn Select(
    value: ReadSignal<String>,
    on_change: Callback<String>,
    #[prop(into)] options: Vec<SelectOption>,
    #[prop(optional, into)] id: Option<String>,
    #[prop(optional, into)] described_by: Option<String>,
    #[prop(optional)] disabled: bool,
    #[prop(optional)] required: bool,
    #[prop(optional)] invalid: bool,
) -> impl IntoView {
    let field = use_context::<FormFieldA11y>();
    let field_id = field.as_ref().and_then(|ctx| ctx.id.clone());
    let field_error_id = field.as_ref().and_then(|ctx| ctx.error_id.clone());
    let id = field_id.or(id);
    let described_by = merge_described_by(field_error_id, described_by);
    let required = required || field.as_ref().is_some_and(|ctx| ctx.required);
    let invalid = invalid || field.as_ref().is_some_and(|ctx| ctx.invalid);

    view! {
        <select
            class="form-select"
            id=id
            disabled=disabled
            required=required
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

#[cfg(test)]
mod accessibility_tests {
    use super::merge_described_by;

    #[test]
    fn no_error_does_not_create_a_dangling_description_reference() {
        assert_eq!(merge_described_by(None, None), None);
        assert_eq!(
            merge_described_by(None, Some("help-text".to_owned())),
            Some("help-text".to_owned())
        );
    }

    #[test]
    fn error_reference_is_appended_without_losing_help_text() {
        assert_eq!(
            merge_described_by(
                Some("email-error".to_owned()),
                Some("email-help".to_owned())
            ),
            Some("email-help email-error".to_owned())
        );
    }

    #[test]
    fn error_reference_is_used_when_there_is_no_help_text() {
        assert_eq!(
            merge_described_by(Some("email-error".to_owned()), None),
            Some("email-error".to_owned())
        );
    }
}

fn event_target_checked(ev: &leptos::ev::Event) -> bool {
    use wasm_bindgen::JsCast;
    ev.target()
        .and_then(|t| t.dyn_into::<web_sys::HtmlInputElement>().ok())
        .map(|el| el.checked())
        .unwrap_or(false)
}
