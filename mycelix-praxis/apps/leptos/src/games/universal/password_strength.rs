// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Password Strength Analyzer — interactive cybersecurity game.
//!
//! Students type passwords and see real-time entropy calculations,
//! crack time estimates, and learn what makes passwords strong.

use leptos::prelude::*;

fn password_entropy(password: &str) -> f64 {
    if password.is_empty() { return 0.0; }
    let has_lower = password.chars().any(|c| c.is_ascii_lowercase());
    let has_upper = password.chars().any(|c| c.is_ascii_uppercase());
    let has_digit = password.chars().any(|c| c.is_ascii_digit());
    let has_symbol = password.chars().any(|c| !c.is_alphanumeric());

    let charset_size = if has_lower { 26 } else { 0 }
        + if has_upper { 26 } else { 0 }
        + if has_digit { 10 } else { 0 }
        + if has_symbol { 33 } else { 0 };

    let charset_size = charset_size.max(1) as f64;
    password.len() as f64 * charset_size.log2()
}

fn crack_time_label(entropy: f64) -> &'static str {
    match entropy as u32 {
        0..=20 => "Instant",
        21..=35 => "Seconds",
        36..=45 => "Minutes",
        46..=55 => "Hours",
        56..=65 => "Days",
        66..=80 => "Years",
        81..=100 => "Centuries",
        _ => "Heat death of universe",
    }
}

fn strength_label(entropy: f64) -> (&'static str, &'static str) {
    match entropy as u32 {
        0..=25 => ("Very Weak", "var(--error)"),
        26..=40 => ("Weak", "var(--warning)"),
        41..=60 => ("Moderate", "var(--mastery-yellow)"),
        61..=80 => ("Strong", "var(--mastery-green)"),
        _ => ("Very Strong", "var(--success)"),
    }
}

fn common_password_check(password: &str) -> Option<&'static str> {
    let lower = password.to_lowercase();
    let common = ["password", "123456", "qwerty", "abc123", "letmein", "admin",
                   "welcome", "monkey", "dragon", "master", "1234567890"];
    if common.iter().any(|c| lower.contains(c)) {
        Some("Contains a very common password pattern")
    } else if lower.chars().all(|c| c.is_ascii_lowercase()) && password.len() < 8 {
        Some("Only lowercase letters — add numbers and symbols")
    } else if password.chars().all(|c| c.is_ascii_digit()) {
        Some("Only numbers — easy for computers to guess")
    } else {
        None
    }
}

#[component]
pub fn PasswordStrengthGame(node_id: String) -> impl IntoView {
    let (password, set_password) = signal(String::new());

    let entropy = Memo::new(move |_| password_entropy(&password.get()));
    let strength = Memo::new(move |_| strength_label(entropy.get()));
    let crack_time = Memo::new(move |_| crack_time_label(entropy.get()));
    let warning = Memo::new(move |_| common_password_check(&password.get()));

    let entropy_bar_width = Memo::new(move |_| {
        let e = entropy.get();
        ((e / 100.0) * 100.0).min(100.0)
    });

    view! {
        <div class="game-container">
            <h3 style="font-size: 1rem; margin-bottom: 0.75rem">"Password Strength Analyzer"</h3>
            <p style="font-size: 0.8rem; color: var(--text-secondary); margin-bottom: 1rem">
                "Type a password to analyze its strength. Nothing is stored or transmitted."
            </p>

            <input
                type="text"
                placeholder="Try a password..."
                autocomplete="off"
                style="width: 100%; padding: 0.75rem 1rem; background: var(--surface); border: 2px solid var(--border); border-radius: 8px; color: var(--text); font-size: 1.1rem; font-family: monospace; outline: none"
                on:input=move |ev| set_password.set(leptos::prelude::event_target_value(&ev))
            />

            // Strength bar
            <div style="margin: 1rem 0">
                <div style="height: 8px; background: var(--border); border-radius: 4px; overflow: hidden">
                    <div style=move || format!("width: {:.0}%; height: 100%; border-radius: 4px; transition: width 0.3s, background 0.3s; background: {}", entropy_bar_width.get(), strength.get().1)></div>
                </div>
                <div style="display: flex; justify-content: space-between; margin-top: 0.5rem">
                    <span style=move || format!("font-size: 0.85rem; font-weight: 600; color: {}", strength.get().1)>
                        {move || strength.get().0}
                    </span>
                    <span style="font-size: 0.8rem; color: var(--text-secondary)">
                        "Entropy: "{move || format!("{:.0}", entropy.get())}" bits"
                    </span>
                </div>
            </div>

            // Details
            {move || if !password.get().is_empty() {
                view! {
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 0.75rem; margin-bottom: 1rem">
                        <div style="padding: 0.5rem 0.75rem; background: var(--soil-rich); border-radius: 6px">
                            <div style="font-size: 0.7rem; color: var(--text-tertiary)">"Time to crack"</div>
                            <div style="font-size: 0.9rem; font-weight: 600">{crack_time.get()}</div>
                        </div>
                        <div style="padding: 0.5rem 0.75rem; background: var(--soil-rich); border-radius: 6px">
                            <div style="font-size: 0.7rem; color: var(--text-tertiary)">"Length"</div>
                            <div style="font-size: 0.9rem; font-weight: 600">{password.get().len()}" characters"</div>
                        </div>
                    </div>
                }.into_any()
            } else {
                view! { <span></span> }.into_any()
            }}

            // Warning
            {move || warning.get().map(|w| {
                view! {
                    <div style="padding: 0.5rem 0.75rem; background: rgba(239, 68, 68, 0.08); border-left: 3px solid var(--error); border-radius: 0 6px 6px 0; font-size: 0.8rem; color: var(--error); margin-bottom: 0.75rem">
                        {w}
                    </div>
                }
            })}

            // Tips
            <div style="font-size: 0.75rem; color: var(--text-tertiary); line-height: 1.6">
                <strong style="color: var(--text-secondary)">"Tips: "</strong>
                "Use 12+ characters. Mix uppercase, lowercase, numbers, and symbols. "
                "Passphrases (like 'correct-horse-battery-staple') are both strong and memorable."
            </div>
        </div>
    }
}
