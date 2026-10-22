// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Study Page — renders lesson content for a specific CAPS topic.
//!
//! Fetches the generated lesson JSON on demand and displays:
//! - Explanation with key vocabulary
//! - Worked examples (step-by-step, expandable)
//! - Practice problems (with hints and distractors)
//! - Misconceptions
//! - Supplementary resource links

use leptos::prelude::*;
use praxis_core::{
    GET_MY_PROGRESS_FN, LEARNING_COORDINATOR_ZOME, ProgressSnapshot, ProgressSyncInput,
    ProgressSyncReceipt, SYNC_PROGRESS_FN, curriculum_node_course_id,
};
use serde::Deserialize;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::{JsFuture, spawn_local};

use crate::curriculum::{
    BktState, ProgressStatus, SelfAssessment, curriculum_graph, use_progress, use_set_progress,
};
use crate::games;
use crate::holochain::{ConnectionStatus, use_holochain};
use crate::mode::{AppMode, use_app_mode};
use crate::persistence;
use crate::social_proof;

// ============================================================
// Lesson data types (from generated JSON)
// ============================================================

#[derive(Clone, Debug, Deserialize)]
struct NodeContent {
    node_id: String,
    lesson: GeneratedLesson,
    #[serde(default)]
    flashcards: Vec<Flashcard>,
}

#[derive(Clone, Debug, Deserialize)]
struct GeneratedLesson {
    #[serde(default)]
    title: String,
    #[serde(default)]
    explanation: String,
    #[serde(default)]
    examples: Vec<WorkedExample>,
    #[serde(default)]
    practice_problems: Vec<PracticeProblem>,
    #[serde(default)]
    key_vocabulary: Vec<VocabTerm>,
    #[serde(default)]
    misconceptions: Vec<Misconception>,
}

#[derive(Clone, Debug, Deserialize)]
struct WorkedExample {
    problem: String,
    #[serde(default)]
    steps: Vec<SolutionStep>,
    answer: String,
}

#[derive(Clone, Debug, Deserialize)]
struct SolutionStep {
    instruction: String,
    result: String,
}

#[derive(Clone, Debug, Deserialize)]
struct PracticeProblem {
    question: String,
    answer: String,
    #[serde(default)]
    difficulty_permille: u16,
    #[serde(default)]
    bloom_level: String,
    #[serde(default)]
    hints: Vec<String>,
    #[serde(default)]
    explanation: String,
    #[serde(default)]
    distractors: Vec<String>,
}

fn normalized_choice(text: &str) -> String {
    text.split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .trim()
        .trim_end_matches(['.', '!', '?'])
        .to_lowercase()
}

fn choice_gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn choice_ratio_key(numerator: i128, denominator: i128) -> Option<String> {
    if denominator == 0 {
        return None;
    }
    let negative = (numerator < 0) ^ (denominator < 0);
    let numerator = numerator.unsigned_abs();
    let denominator = denominator.unsigned_abs();
    let divisor = choice_gcd(numerator, denominator);
    let sign = if negative { "-" } else { "" };
    Some(format!("{sign}{}/{}", numerator / divisor, denominator / divisor))
}

fn choice_number_key(text: &str) -> Option<String> {
    let number = text.trim();
    if let Some((numerator, denominator)) = number.split_once('/') {
        return choice_ratio_key(
            numerator.trim().parse::<i128>().ok()?,
            denominator.trim().parse::<i128>().ok()?,
        );
    }
    if let Some((whole, fractional)) = number.split_once('.') {
        if fractional.is_empty() || !fractional.chars().all(|ch| ch.is_ascii_digit()) {
            return None;
        }
        let negative = whole.starts_with('-');
        let whole_digits = whole.trim_start_matches('-');
        let whole_value = if whole_digits.is_empty() {
            0
        } else {
            whole_digits.parse::<i128>().ok()?
        };
        let denominator = 10_i128.checked_pow(u32::try_from(fractional.len()).ok()?)?;
        let fractional_value = fractional.parse::<i128>().ok()?;
        let numerator = whole_value
            .checked_mul(denominator)?
            .checked_add(fractional_value)?
            .checked_mul(if negative { -1 } else { 1 })?;
        return choice_ratio_key(numerator, denominator);
    }
    choice_ratio_key(number.parse::<i128>().ok()?, 1)
}

fn choice_numeric_value(text: &str) -> Option<(String, Option<&'static str>)> {
    let normalized = normalized_choice(text);
    const UNITS: &[(&str, &str)] = &[
        (" square centimeters", "area_cm2"),
        (" square centimetres", "area_cm2"),
        (" square meters", "area_m2"),
        (" square metres", "area_m2"),
        (" square units", "area_unit"),
        (" square unit", "area_unit"),
        (" centimeters", "length_cm"),
        (" centimetres", "length_cm"),
        (" millimeters", "length_mm"),
        (" millimetres", "length_mm"),
        (" children", "count"),
        (" child", "count"),
        (" learners", "count"),
        (" learner", "count"),
        (" students", "count"),
        (" student", "count"),
        (" objects", "count"),
        (" object", "count"),
        (" counters", "count"),
        (" counter", "count"),
        (" books", "count"),
        (" book", "count"),
        (" minutes", "time_min"),
        (" minute", "time_min"),
        (" seconds", "time_s"),
        (" second", "time_s"),
        (" hours", "time_h"),
        (" hour", "time_h"),
        (" square cm", "area_cm2"),
        (" square m", "area_m2"),
        (" cm", "length_cm"),
        (" mm", "length_mm"),
        (" kg", "mass_kg"),
        (" m", "length_m"),
    ];

    for (suffix, unit) in UNITS {
        if let Some(number) = normalized.strip_suffix(*suffix) {
            let number = number.trim();
            return choice_number_key(number).map(|key| (key, Some(*unit)));
        }
    }
    choice_number_key(&normalized).map(|key| (key, None))
}

fn numeric_choices_collide(answer: &str, distractor: &str) -> bool {
    let Some((answer_value, answer_unit)) = choice_numeric_value(answer) else {
        return false;
    };
    let Some((distractor_value, distractor_unit)) = choice_numeric_value(distractor) else {
        return false;
    };
    answer_value == distractor_value
        && (answer_unit.is_none()
            || distractor_unit.is_none()
            || answer_unit == distractor_unit)
}

fn answer_first_choices(problem: &PracticeProblem) -> Option<Vec<String>> {
    if problem.distractors.len() != 3 || problem.answer.trim().is_empty() {
        return None;
    }
    let answer_key = normalized_choice(&problem.answer);
    let mut choices = problem.distractors.clone();
    choices.push(problem.answer.clone());
    let normalized = choices
        .iter()
        .map(|choice| normalized_choice(choice))
        .collect::<Vec<_>>();
    if normalized.iter().any(String::is_empty) {
        return None;
    }
    if normalized.iter().collect::<std::collections::BTreeSet<_>>().len() != choices.len() {
        return None;
    }
    if problem.distractors.iter().any(|choice| {
        normalized_choice(choice) == answer_key || numeric_choices_collide(&problem.answer, choice)
    }) {
        return None;
    }
    // Fisher–Yates using the existing browser/WASM Math binding. Do not add a
    // Rust RNG dependency just to randomize four answer choices.
    for index in (1..choices.len()).rev() {
        let swap_index = (js_sys::Math::random() * (index + 1) as f64) as usize;
        choices.swap(index, swap_index.min(index));
    }
    Some(choices)
}

#[component]
fn PracticeProblemCard(
    problem: PracticeProblem,
    index: usize,
    node_id: String,
) -> impl IntoView {
    let choices = answer_first_choices(&problem);
    let answer = problem.answer.clone();
    let question = problem.question.clone();
    let explanation = problem.explanation.clone();
    let hints = problem.hints.clone();
    let difficulty = match problem.difficulty_permille {
        0..=300 => "Gentle soil",
        301..=600 => "Steady ground",
        601..=800 => "Rocky terrain",
        _ => "Mountain path",
    };
    let bloom_level = problem.bloom_level.clone();
    let (selected_choice, set_selected_choice) = signal(None::<String>);
    let (scored_result, set_scored_result) = signal(None::<bool>);
    let (revealed_solution, set_revealed_solution) = signal(false);
    let (self_assessment, set_self_assessment) = signal(None::<SelfAssessment>);
    let (show_hint, set_show_hint) = signal(false);
    let hint_text = hints.first().cloned().unwrap_or_default();
    let progress = use_set_progress();
    let scored_node_id = node_id.clone();
    let reflection_node_id = node_id.clone();

    view! {
        <div class="praxis-problem">
            <div class="praxis-problem-q">{index + 1}". "{question}</div>
            <div class="praxis-problem-meta">{difficulty}" | "{bloom_level}</div>
            {move || {
                // Clone owned lesson text for this reactive render so nested
                // move closures can own copies without consuming retained
                // values on a later state update.
                let answer = answer.clone();
                let explanation = explanation.clone();
                let hint_text = hint_text.clone();
                let choices_for_render = choices.clone();

                if let Some(options) = choices_for_render {
                    if let Some(is_correct) = scored_result.get() {
                    view! {
                        <div class="praxis-problem-a" style="margin-top: 0.5rem">
                            "Correct answer: "{answer.clone()}
                        </div>
                        <div role="status" style="margin-top: 0.5rem; font-size: 0.85rem; color: var(--text-secondary)">
                            {if is_correct {
                                "Correct. Your choice was recorded before the answer was revealed."
                            } else {
                                "Not correct yet. Review the explanation and try a new problem later."
                            }}
                        </div>
                        <div style="font-size: 0.8rem; color: var(--text-secondary); margin-top: 0.25rem">
                            {explanation.clone()}
                        </div>
                    }.into_any()
                } else {
                    view! {
                        <p style="font-size: 0.8rem; color: var(--text-secondary); margin: 0.5rem 0">
                            "Choose an answer before seeing the solution."
                        </p>
                        <div role="group" aria-label="Answer choices" style="display: flex; flex-direction: column; gap: 0.4rem; margin-top: 0.5rem">
                            {options.iter().map(|choice| {
                                let choice_value = choice.clone();
                                let choice_for_class = choice.clone();
                                let choice_for_click = choice.clone();
                                view! {
                                    <button
                                        type="button"
                                        class=move || {
                                            if selected_choice.get().as_deref() == Some(choice_for_class.as_str()) {
                                                "praxis-filter-btn active"
                                            } else {
                                                "praxis-filter-btn"
                                            }
                                        }
                                        aria-pressed=move || selected_choice.get().as_deref() == Some(choice_value.as_str())
                                        on:click=move |_| set_selected_choice.set(Some(choice_for_click.clone()))
                                    >{choice.clone()}</button>
                                }
                            }).collect::<Vec<_>>()}
                        </div>
                        <div style="margin-top: 0.5rem; display: flex; gap: 0.5rem">
                            <button
                                type="button"
                                class="praxis-filter-btn active"
                                disabled=move || selected_choice.get().is_none()
                                on:click={
                                    let target_node = scored_node_id.clone();
                                    let correct_answer = answer.clone();
                                    move |_| {
                                        if let Some(choice) = selected_choice.get_untracked() {
                                            let correct = normalized_choice(&choice) == normalized_choice(&correct_answer);
                                            progress.update(|store| store.record_response(&target_node, correct));
                                            set_scored_result.set(Some(correct));
                                        }
                                    }
                                }
                            >"Check answer"</button>
                            {if !hint_text.is_empty() {
                                view! {
                                    <button
                                        type="button"
                                        class="praxis-filter-btn"
                                        on:click=move |_| set_show_hint.set(true)
                                    >"Hint"</button>
                                }.into_any()
                            } else {
                                view! { <span></span> }.into_any()
                            }}
                        </div>
                        {move || if show_hint.get() {
                            view! {
                                <div style="font-size: 0.8rem; color: var(--warning); margin-top: 0.4rem; font-style: italic">
                                    {hint_text.clone()}
                                </div>
                            }.into_any()
                        } else {
                            view! { <span></span> }.into_any()
                        }}
                    }.into_any()
                }
            } else if revealed_solution.get() {
                view! {
                    <div class="praxis-problem-a" style="margin-top: 0.5rem">
                        "Solution: "{answer.clone()}
                    </div>
                    <div style="font-size: 0.8rem; color: var(--text-secondary); margin-top: 0.25rem">
                        {explanation.clone()}
                    </div>
                    {move || if let Some(reflection) = self_assessment.get() {
                        view! {
                            <div role="status" style="margin-top: 0.5rem; font-size: 0.8rem; color: var(--text-tertiary)">
                                {match reflection {
                                    SelfAssessment::FeelsUnderstood => "Reflection saved: you feel you understand this item.",
                                    SelfAssessment::NeedsMorePractice => "Reflection saved: you want more practice.",
                                }}
                                " This is self-report, not a scored response or verified mastery."
                            </div>
                        }.into_any()
                    } else {
                        view! {
                            <div style="margin-top: 0.5rem; font-size: 0.8rem; color: var(--text-secondary)">
                                "This item lacks a usable four-choice set. You can save a reflection, but it will not count as a scored response."
                            </div>
                            <div style="display: flex; gap: 0.5rem; margin-top: 0.5rem">
                                <button
                                    type="button"
                                    class="praxis-filter-btn"
                                    on:click={
                                        let target_node = reflection_node_id.clone();
                                        move |_| {
                                            progress.update(|store| {
                                                store.record_self_assessment(
                                                    &target_node,
                                                    SelfAssessment::FeelsUnderstood
                                                );
                                            });
                                            set_self_assessment.set(Some(SelfAssessment::FeelsUnderstood));
                                        }
                                    }
                                >"I feel I understand"</button>
                                <button
                                    type="button"
                                    class="praxis-filter-btn"
                                    on:click={
                                        let target_node = reflection_node_id.clone();
                                        move |_| {
                                            progress.update(|store| {
                                                store.record_self_assessment(
                                                    &target_node,
                                                    SelfAssessment::NeedsMorePractice
                                                );
                                            });
                                            set_self_assessment.set(Some(SelfAssessment::NeedsMorePractice));
                                        }
                                    }
                                >"I need more practice"</button>
                            </div>
                        }.into_any()
                    }}
                }.into_any()
            } else {
                view! {
                    <p style="font-size: 0.8rem; color: var(--text-secondary); margin: 0.5rem 0">
                        "This item does not have four distinct answer choices. Try it first, then reveal the solution; reflection will be recorded separately from scored practice."
                    </p>
                    <div style="margin-top: 0.5rem; display: flex; gap: 0.5rem">
                        <button
                            type="button"
                            class="praxis-filter-btn active"
                            on:click=move |_| set_revealed_solution.set(true)
                        >"Reveal solution"</button>
                        {if !hint_text.is_empty() {
                            view! {
                                <button
                                    type="button"
                                    class="praxis-filter-btn"
                                    on:click=move |_| set_show_hint.set(true)
                                >"Hint"</button>
                            }.into_any()
                        } else {
                            view! { <span></span> }.into_any()
                        }}
                    </div>
                    {move || if show_hint.get() {
                        view! {
                            <div style="font-size: 0.8rem; color: var(--warning); margin-top: 0.4rem; font-style: italic">
                                {hint_text.clone()}
                            </div>
                        }.into_any()
                    } else {
                        view! { <span></span> }.into_any()
                    }}
                    }.into_any()
                }
            }}
        </div>
    }
}


#[derive(Clone, Debug, Deserialize)]
struct VocabTerm {
    term: String,
    definition: String,
    #[serde(default)]
    example_usage: String,
}

#[derive(Clone, Debug, Deserialize)]
struct Misconception {
    misconception: String,
    correction: String,
    #[serde(default)]
    why_students_think_this: String,
}

#[derive(Clone, Debug, Deserialize)]
struct Flashcard {
    front: String,
    back: String,
}

// ============================================================
// Fetch lesson JSON
// ============================================================

async fn fetch_lesson(node_id: &str) -> Result<NodeContent, String> {
    let url = crate::content_manifest::primary_lesson_url(node_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| format!("No generated lesson is registered for {node_id}"))?;

    let window = web_sys::window().ok_or("no window")?;
    let resp = JsFuture::from(window.fetch_with_str(url))
        .await
        .map_err(|e| format!("fetch error: {:?}", e))?;

    let resp: web_sys::Response = resp.dyn_into().map_err(|_| "not a Response")?;

    if !resp.ok() {
        return Err(format!("HTTP {}", resp.status()));
    }

    let text = JsFuture::from(resp.text().map_err(|_| "text() failed")?)
        .await
        .map_err(|e| format!("text error: {:?}", e))?;

    let text_str = text.as_string().ok_or("not a string")?;
    serde_json::from_str(&text_str).map_err(|e| format!("JSON parse error: {}", e))
}

// ============================================================
// Study page component
// ============================================================

#[component]
pub fn StudyPage(node_id: String) -> impl IntoView {
    let progress = use_progress();
    let set_progress = use_set_progress();
    let graph = curriculum_graph();

    let node = graph.node(&node_id).cloned();
    let (active_tab, set_active_tab) = signal("explanation");

    // Fetch lesson data
    let node_id_for_fetch = node_id.clone();
    let lesson_resource = LocalResource::new(move || {
        let id = node_id_for_fetch.clone();
        async move { fetch_lesson(&id).await }
    });

    let Some(node) = node else {
        return view! {
            <div class="stub-page">
                <h2>"Topic not found"</h2>
                <p>"The topic "{node_id}" was not found in the curriculum."</p>
            </div>
        }
        .into_any();
    };

    let title = node.title.clone();
    let description = node.description.clone();
    let subdomain = node.subdomain.clone();
    let grade_label = node
        .grade_levels
        .first()
        .cloned()
        .unwrap_or_default()
        .replace("Grade", "Grade ");
    let bloom = node.bloom_level.clone();
    let hours = node.estimated_hours;
    let exam_weight = node.exam_weight.clone();
    let node_id_for_status = node_id.clone();
    let node_id_for_status2 = node_id.clone();

    view! {
        <div class="praxis-skill-map study-refuge">
            // Pomodoro timer (ambient, at top)
            <crate::study_tracker::PomodoroTimer />

            // Back link — return to prospect
            <a href="/skill-map" class="refuge-back-link">
                "\u{2190} Back to constellation"
            </a>

            // Header
            <div style="margin: 1rem 0">
                <h1 style="font-size: 1.5rem; margin-bottom: 0.5rem">{title}</h1>
                <div class="praxis-detail-meta">
                    <span class="praxis-badge praxis-badge-grade">{grade_label}</span>
                    <span class="praxis-badge praxis-badge-bloom">{bloom}</span>
                    <span class="praxis-badge praxis-badge-hours">{hours}"h estimated"</span>
                    {exam_weight.map(|ew| view! {
                        <span class="praxis-badge praxis-badge-exam">
                            "Paper "{ew.paper}": "{ew.marks}"/" {ew.total_paper_marks}" marks ("{format!("{:.0}", ew.percentage)}"%)"
                        </span>
                    })}
                </div>

                // NSC exam insight
                {social_proof::exam_insight(&node_id).map(|insight| {
                    let pct = insight.percentage_struggled;
                    let error = insight.common_error;
                    let tip = insight.examiner_tip;
                    view! {
                        <div style="margin-top: 0.75rem; padding: 0.6rem 0.75rem; border-left: 3px solid var(--info); background: rgba(59, 130, 246, 0.05); border-radius: 0 6px 6px 0; font-size: 0.8rem; line-height: 1.5">
                            <span style="color: var(--info); font-weight: 600">"Exam insight: "</span>
                            <span style="color: var(--text-secondary)">{pct}"% of candidates found this challenging. "</span>
                            <span style="color: var(--text-tertiary); font-style: italic">{tip}</span>
                        </div>
                    }
                })}
            </div>

            // Status
            <div class="praxis-status-btns">
                {
                    let id_ns = node_id_for_status.clone();
                    let id_s = node_id_for_status.clone();
                    let id_s2 = id_s.clone();
                    let id_m = node_id_for_status2.clone();
                    let id_m2 = id_m.clone();
                    view! {
                        <button
                            class=move || {
                                let s = progress.get().get(&id_ns).status;
                                if s == ProgressStatus::NotStarted { "praxis-status-btn active-not-started" } else { "praxis-status-btn" }
                            }
                            on:click={
                                let id = node_id.clone();
                                move |_| set_progress.update(|p| p.set_status(&id, ProgressStatus::NotStarted))
                            }
                        >"Not Started"</button>
                        <button
                            class=move || {
                                let s = progress.get().get(&id_s).status;
                                if s == ProgressStatus::Studying { "praxis-status-btn active-studying" } else { "praxis-status-btn" }
                            }
                            on:click={
                                let id = id_s2.clone();
                                move |_| set_progress.update(|p| p.set_status(&id, ProgressStatus::Studying))
                            }
                        >"Studying"</button>
                        <button
                            class=move || {
                                let learner_state = progress.get().get(&id_m).self_assessment;
                                if learner_state == Some(SelfAssessment::FeelsUnderstood) {
                                    "praxis-status-btn active-studying"
                                } else {
                                    "praxis-status-btn"
                                }
                            }
                            on:click={
                                let id = id_m2.clone();
                                move |_| set_progress.update(|p| {
                                    p.record_self_assessment(&id, SelfAssessment::FeelsUnderstood)
                                })
                            }
                            title="Self-assessment only; this does not verify mastery."
                        >"I feel I understand"</button>
                    }
                }
            </div>

            <LiveProgressSync node_id=node_id.clone() />

            // Tabs
            <div class="praxis-tabs" style="max-width: 600px">
                <button class=move || if active_tab.get() == "explanation" { "praxis-tab active" } else { "praxis-tab" }
                    on:click=move |_| set_active_tab.set("explanation")>"Learn"</button>
                <button class=move || if active_tab.get() == "examples" { "praxis-tab active" } else { "praxis-tab" }
                    on:click=move |_| set_active_tab.set("examples")>"Examples"</button>
                <button class=move || if active_tab.get() == "practice" { "praxis-tab active" } else { "praxis-tab" }
                    on:click=move |_| set_active_tab.set("practice")>"Practice"</button>
                <button class=move || if active_tab.get() == "pitfalls" { "praxis-tab active" } else { "praxis-tab" }
                    on:click=move |_| set_active_tab.set("pitfalls")>"Pitfalls"</button>
                <button class=move || if active_tab.get() == "connections" { "praxis-tab active" } else { "praxis-tab" }
                    on:click=move |_| set_active_tab.set("connections")
                    style="color: var(--info)"
                >"Connections"</button>
                <button class=move || if active_tab.get() == "notes" { "praxis-tab active" } else { "praxis-tab" }
                    on:click=move |_| set_active_tab.set("notes")>"Notes"</button>
                {if games::has_game(&node_id_for_status) {
                    view! {
                        <button class=move || if active_tab.get() == "explore" { "praxis-tab active" } else { "praxis-tab" }
                            on:click=move |_| set_active_tab.set("explore")
                            style="color: var(--success)"
                        >"Explore"</button>
                    }.into_any()
                } else {
                    view! { <span></span> }.into_any()
                }}
            </div>

            // Content
            <Suspense fallback=move || view! {
                <div class="card-loading">
                    <div class="skeleton-line" style="width: 80%"></div>
                    <div class="skeleton-line" style="width: 60%"></div>
                    <div class="skeleton-line" style="width: 90%"></div>
                </div>
            }>
                {move || {
                    lesson_resource.get().map(|result| {
                        match &result {
                            Ok(content) => {
                                let lesson = content.lesson.clone();
                                let explanation = lesson.explanation.clone();
                                let vocab = lesson.key_vocabulary.clone();
                                let examples = lesson.examples.clone();
                                let problems = lesson.practice_problems.clone();
                                let misconceptions = lesson.misconceptions.clone();

                                view! {
                                    // Explanation tab
                                    <div style=move || if active_tab.get() == "explanation" { "display: block" } else { "display: none" }>
                                        <div class="praxis-detail" style="margin-bottom: 1rem">
                                            <p style="font-size: 0.95rem; line-height: 1.8">{explanation.clone()}</p>
                                        </div>

                                        {if !vocab.is_empty() {
                                            view! {
                                                <div class="praxis-detail">
                                                    <h3 style="font-size: 0.9rem; font-weight: 700; margin-bottom: 0.75rem; color: var(--text-secondary)">"Key Vocabulary"</h3>
                                                    {vocab.iter().map(|v| {
                                                        let term = v.term.clone();
                                                        let def = v.definition.clone();
                                                        view! {
                                                            <div style="margin-bottom: 0.5rem; padding-bottom: 0.5rem; border-bottom: 1px solid var(--border)">
                                                                <strong>{term}</strong>
                                                                <span style="color: var(--text-secondary)">" — "{def}</span>
                                                            </div>
                                                        }
                                                    }).collect::<Vec<_>>()}
                                                </div>
                                            }.into_any()
                                        } else {
                                            view! { <span></span> }.into_any()
                                        }}
                                    </div>

                                    // Examples tab
                                    <div style=move || if active_tab.get() == "examples" { "display: block" } else { "display: none" }>
                                        {examples.iter().enumerate().map(|(i, ex)| {
                                            let problem = ex.problem.clone();
                                            let answer = ex.answer.clone();
                                            let steps = ex.steps.clone();
                                            view! {
                                                <div class="praxis-example">
                                                    <div class="praxis-example-problem">"Example "{i + 1}": "{problem}</div>
                                                    {steps.iter().map(|s| {
                                                        let instr = s.instruction.clone();
                                                        let result = s.result.clone();
                                                        view! {
                                                            <div class="praxis-example-step">
                                                                <strong>{instr}</strong>" \u{2192} "{result}
                                                            </div>
                                                        }
                                                    }).collect::<Vec<_>>()}
                                                    <div class="praxis-example-answer">{answer}</div>
                                                </div>
                                            }
                                        }).collect::<Vec<_>>()}
                                    </div>

                                    // Practice tab — BKT-adaptive active recall
                                    <div style=move || if active_tab.get() == "practice" { "display: block" } else { "display: none" }>
                                        // BKT mastery indicator
                                        {
                                            let nid = node_id_for_status.clone();
                                            move || {
                                                let p = progress.get();
                                                let bkt = p.bkt(&nid);
                                                let pct = (bkt.p_mastery * 100.0) as u32;
                                                let terrain = bkt.terrain_label();
                                                view! {
                                                    <div style="display: flex; align-items: center; gap: 0.75rem; margin-bottom: 1rem; padding: 0.5rem 0.75rem; background: var(--soil-rich); border-radius: 8px">
                                                        <div style="font-size: 0.8rem; color: var(--text-secondary)">
                                                            "Estimated understanding (BKT): "
                                                            <strong style="color: var(--text)">{pct}"%"</strong>
                                                        </div>
                                                        <div style="font-size: 0.75rem; color: var(--text-tertiary); font-style: italic">
                                                            "Recommended: "{terrain}
                                                        </div>
                                                    </div>
                                                }
                                            }
                                        }
                                        <p style="font-size: 0.8rem; color: var(--text-tertiary); margin: 0 0 1rem">"BKT is a practice estimate, not proof of durable or independent mastery. Answer choices are scored before feedback appears; reflection-only items do not update scored mastery."</p>
                                        {problems.iter().cloned().enumerate().map(|(i, problem)| {
                                            view! {
                                                <PracticeProblemCard
                                                    problem=problem
                                                    index=i
                                                    node_id=node_id_for_status.clone()
                                                />
                                            }
                                        }).collect::<Vec<_>>()}
                                    </div>

                                    // Pitfalls tab
                                    <div style=move || if active_tab.get() == "pitfalls" { "display: block" } else { "display: none" }>
                                        {if misconceptions.is_empty() {
                                            view! { <p style="color: var(--text-secondary)">"No common misconceptions documented for this topic."</p> }.into_any()
                                        } else {
                                            view! {
                                                {misconceptions.iter().map(|m| {
                                                    let mc = m.misconception.clone();
                                                    let corr = m.correction.clone();
                                                    let why = m.why_students_think_this.clone();
                                                    view! {
                                                        <div class="praxis-misconception">
                                                            <div class="praxis-misconception-wrong">{mc}</div>
                                                            <div class="praxis-misconception-right">{corr}</div>
                                                            {if !why.is_empty() {
                                                                view! { <div class="praxis-misconception-why">{why}</div> }.into_any()
                                                            } else {
                                                                view! { <span></span> }.into_any()
                                                            }}
                                                        </div>
                                                    }
                                                }).collect::<Vec<_>>()}
                                            }.into_any()
                                        }}
                                    </div>

                                    // Explore tab (game)
                                    <div style=move || if active_tab.get() == "explore" { "display: block" } else { "display: none" }>
                                        <games::GameContainer node_id=node_id.clone() />
                                    </div>

                                    // Notes tab
                                    // Connections tab — cross-subject links
                                    <div style=move || if active_tab.get() == "connections" { "display: block" } else { "display: none" }>
                                        {
                                            let graph = curriculum_graph();
                                            let neighbors = graph.cross_subject_neighbors(&node_id);
                                            if neighbors.is_empty() {
                                                view! {
                                                    <div class="praxis-detail">
                                                        <p style="color: var(--text-secondary); font-size: 0.9rem">
                                                            "No cross-subject connections mapped for this topic yet. "
                                                            "Connections show where knowledge from one subject applies in another."
                                                        </p>
                                                    </div>
                                                }.into_any()
                                            } else {
                                                view! {
                                                    <div class="praxis-detail">
                                                        <h3 style="font-size: 0.9rem; font-weight: 700; margin-bottom: 0.75rem; color: var(--text-secondary)">"Connected Across Subjects"</h3>
                                                        <p style="font-size: 0.75rem; color: var(--text-tertiary); margin-bottom: 1rem">"This topic connects to knowledge in other subjects — tap to explore."</p>
                                                        {neighbors.iter().map(|(neighbor, edge)| {
                                                            let href = format!("/study/{}", neighbor.id);
                                                            let title = neighbor.title.clone();
                                                            let subject = neighbor.subject_area.clone();
                                                            let rationale = edge.rationale.clone();
                                                            let edge_type = edge.edge_type.clone();
                                                            let grade = neighbor.grade_levels.first().cloned().unwrap_or_default();
                                                            view! {
                                                                <a href=href class="connection-card">
                                                                    <div style="display: flex; align-items: center; gap: 0.5rem; margin-bottom: 0.25rem">
                                                                        <span class="praxis-badge" style="background: var(--info); color: var(--text-on-primary); font-size: 0.65rem">{subject}</span>
                                                                        <span style="font-size: 0.65rem; color: var(--text-tertiary)">{grade}</span>
                                                                        <span style="font-size: 0.6rem; color: var(--text-tertiary); font-style: italic">{edge_type}</span>
                                                                    </div>
                                                                    <div style="font-weight: 600; font-size: 0.9rem; margin-bottom: 0.25rem">{title}</div>
                                                                    <div style="font-size: 0.8rem; color: var(--text-secondary); line-height: 1.5">{rationale}</div>
                                                                </a>
                                                            }
                                                        }).collect::<Vec<_>>()}
                                                    </div>
                                                }.into_any()
                                            }
                                        }
                                    </div>

                                    // Notes tab
                                    <div style=move || if active_tab.get() == "notes" { "display: block" } else { "display: none" }>
                                        {
                                            let notes_key = format!("praxis_notes_{}", node_id);
                                            let initial_notes = persistence::load::<String>(&notes_key).unwrap_or_default();
                                            let (notes, set_notes) = signal(initial_notes);
                                            let notes_key_save = notes_key.clone();

                                            view! {
                                                <div class="praxis-detail">
                                                    <h3 style="font-size: 0.9rem; font-weight: 700; margin-bottom: 0.5rem; color: var(--text-secondary)">"Your Notes"</h3>
                                                    <p style="font-size: 0.75rem; color: var(--text-tertiary); margin-bottom: 0.75rem">"Write anything that helps you remember. Saved automatically."</p>
                                                    <textarea
                                                        style="width: 100%; min-height: 150px; padding: 0.75rem; background: var(--bg); border: 1px solid var(--border); border-radius: 8px; color: var(--text); font-size: 0.9rem; font-family: inherit; resize: vertical; line-height: 1.6; outline: none"
                                                        placeholder="Write your notes here..."
                                                        prop:value=move || notes.get()
                                                        on:input=move |ev| {
                                                            let val = leptos::prelude::event_target_value(&ev);
                                                            set_notes.set(val.clone());
                                                            persistence::save(&notes_key_save, &val);
                                                        }
                                                    ></textarea>
                                                </div>
                                            }
                                        }
                                    </div>
                                }.into_any()
                            }
                            Err(_e) => {
                                let desc = description.clone();
                                let sub = subdomain.clone();
                                view! {
                                    <div class="praxis-detail" style="margin-top: 1rem">
                                        <div style="padding: 0.75rem; background: var(--soil-sandy); border-radius: 8px; margin-bottom: 1rem">
                                            <div style="font-size: 0.85rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 0.25rem">
                                                "This lesson is still growing"
                                            </div>
                                            <div style="font-size: 0.8rem; color: var(--text-tertiary)">
                                                "Full content isn't available yet, but here's what we know about this topic."
                                            </div>
                                        </div>
                                        <h3 style="font-size: 0.9rem; font-weight: 600; margin-bottom: 0.5rem">"Overview"</h3>
                                        <p style="font-size: 0.9rem; line-height: 1.7; color: var(--text-secondary)">
                                            {desc}
                                        </p>
                                        {if !sub.is_empty() {
                                            view! {
                                                <p style="font-size: 0.8rem; color: var(--text-tertiary); margin-top: 0.5rem">
                                                    "Part of: "{sub}
                                                </p>
                                            }.into_any()
                                        } else {
                                            view! { <span></span> }.into_any()
                                        }}
                                        <div style="margin-top: 1rem">
                                            <a href="/skill-map" style="color: var(--primary); text-decoration: none; font-size: 0.85rem">
                                                "\u{2190} Explore related topics in the constellation"
                                            </a>
                                        </div>
                                    </div>
                                }.into_any()
                            }
                        }
                    })
                }}
            </Suspense>

            // Feedback widget — "Was this helpful?"
            {
                let feedback_key = format!("praxis_feedback_{}", node_id_for_status2);
                let initial_fb = persistence::load::<String>(&feedback_key);
                let (fb_given, set_fb_given) = signal(initial_fb.is_some());
                let (show_text, set_show_text) = signal(false);
                let fb_key_up = feedback_key.clone();
                let fb_key_down = feedback_key.clone();
                let fb_key_text = feedback_key.clone();

                view! {
                    <div style="margin-top: 2rem; padding: 1rem; border-top: 1px solid var(--border); text-align: center">
                        {move || if fb_given.get() {
                            view! {
                                <p style="font-size: 0.8rem; color: var(--text-tertiary); font-style: italic">"Thanks for your feedback"</p>
                            }.into_any()
                        } else if show_text.get() {
                            let fk = fb_key_text.clone();
                            view! {
                                <p style="font-size: 0.8rem; color: var(--text-secondary); margin-bottom: 0.5rem">"What could be better?"</p>
                                <textarea
                                    style="width: 100%; max-width: 400px; min-height: 60px; padding: 0.5rem; background: var(--bg); border: 1px solid var(--border); border-radius: 6px; color: var(--text); font-size: 0.8rem; font-family: inherit; resize: vertical"
                                    placeholder="Tell us what was confusing or missing..."
                                    on:change=move |ev| {
                                        let val = leptos::prelude::event_target_value(&ev);
                                        persistence::save(&fk, &format!("feedback:{}", val));
                                        set_fb_given.set(true);
                                    }
                                ></textarea>
                            }.into_any()
                        } else {
                            let fk_up = fb_key_up.clone();
                            view! {
                                <p style="font-size: 0.8rem; color: var(--text-secondary); margin-bottom: 0.5rem">"Was this lesson helpful?"</p>
                                <div style="display: flex; gap: 0.75rem; justify-content: center">
                                    <button
                                        style="padding: 0.4rem 1rem; background: var(--soil-rich); border: 1px solid var(--mastery-green); border-radius: 6px; color: var(--mastery-green); cursor: pointer; font-family: inherit; font-size: 0.85rem"
                                        on:click=move |_| {
                                            persistence::save(&fk_up, &"helpful".to_string());
                                            set_fb_given.set(true);
                                        }
                                    >"\u{1F44D} Yes"</button>
                                    <button
                                        style="padding: 0.4rem 1rem; background: var(--surface); border: 1px solid var(--border); border-radius: 6px; color: var(--text-secondary); cursor: pointer; font-family: inherit; font-size: 0.85rem"
                                        on:click=move |_| set_show_text.set(true)
                                    >"\u{1F44E} Could be better"</button>
                                </div>
                            }.into_any()
                        }}
                    </div>
                }
            }
        </div>
    }.into_any()
}

/// Explicitly publish one local curriculum topic and confirm the canonical
/// value can be read back through the same versioned contract.
#[component]
fn LiveProgressSync(node_id: String) -> impl IntoView {
    let mode = use_app_mode();
    if mode.get_untracked() != AppMode::Live {
        return view! { <span></span> }.into_any();
    }

    let hc = use_holochain();
    let progress = use_progress();
    let (syncing, set_syncing) = signal(false);
    let (result, set_result) = signal(None::<Result<String, String>>);

    let node_id_for_click = node_id.clone();
    let hc_for_click = hc.clone();
    let on_sync = move |_| {
        if syncing.get_untracked() {
            return;
        }
        if hc_for_click.status.get_untracked() != ConnectionStatus::Connected {
            set_result.set(Some(Err(
                "Live publishing requires a connected Praxis conductor.".to_string(),
            )));
            return;
        }
        if !hc_for_click.refresh_zome_call_signing_ready() {
            set_result.set(Some(Err(
                "Live publishing requires an authorized zome-call signer.".to_string(),
            )));
            return;
        }

        let local = progress.get_untracked().get(&node_id_for_click).clone();
        let course_id = curriculum_node_course_id(&node_id_for_click);
        let progress_percent = match local.status {
            ProgressStatus::NotStarted => 0.0,
            ProgressStatus::Studying | ProgressStatus::ProvisionalMastery => {
                local.mastery_permille as f32 / 10.0
            }
            ProgressStatus::Mastered => 100.0,
        };
        let input = ProgressSyncInput {
            course_id: course_id.clone(),
            progress_percent,
            completed_items: if local.status == ProgressStatus::Mastered {
                vec![node_id_for_click.clone()]
            } else {
                Vec::new()
            },
            model_version: Some("bkt-v1".into()),
            metadata: Some(serde_json::json!({
                "source": "praxis-leptos",
                "status": local.status.label(),
            })),
        };
        let hc = hc_for_click.clone();
        set_syncing.set(true);
        set_result.set(None);

        spawn_local(async move {
            let outcome = async {
                let receipt = hc
                    .call_zome_default::<_, ProgressSyncReceipt>(
                        LEARNING_COORDINATOR_ZOME,
                        SYNC_PROGRESS_FN,
                        &input,
                    )
                    .await?;
                let snapshots = hc
                    .call_zome_default::<(), Vec<ProgressSnapshot>>(
                        LEARNING_COORDINATOR_ZOME,
                        GET_MY_PROGRESS_FN,
                        &(),
                    )
                    .await?;
                let read_back = snapshots
                    .iter()
                    .find(|snapshot| snapshot.course_id == course_id)
                    .ok_or_else(|| {
                        "The write succeeded but no read-back snapshot was found".to_string()
                    })?;

                if (read_back.progress_percent - receipt.progress.progress_percent).abs() > 0.01
                    || read_back.completed_items != receipt.progress.completed_items
                {
                    return Err(
                        "The read-back snapshot did not match the published value".to_string()
                    );
                }

                Ok(format!(
                    "Published and verified at {}%",
                    read_back.progress_percent.round()
                ))
            }
            .await;

            set_result.set(Some(outcome));
            set_syncing.set(false);
        });
    };

    view! {
        <div class="progress-sync-panel" style="margin: 0.75rem 0; padding: 0.75rem; border: 1px solid var(--border); border-radius: 8px">
            <p style="margin: 0 0 0.5rem; color: var(--text-secondary); font-size: 0.8rem">
                "This publishes this topic's progress to your Praxis source chain as a public learning record."
            </p>
            <button
                class="praxis-filter-btn"
                disabled=move || {
                    hc.status.get() != ConnectionStatus::Connected
                        || !hc.zome_call_signing_ready.get()
                        || syncing.get()
                }
                on:click=on_sync
            >
                {move || if syncing.get() { "Publishing…" } else { "Publish & verify progress" }}
            </button>
            {move || result.get().map(|outcome| match outcome {
                Ok(message) => view! {
                    <span role="status" style="margin-left: 0.6rem; color: var(--success); font-size: 0.8rem">{message}</span>
                }.into_any(),
                Err(message) => view! {
                    <span role="alert" style="display: block; margin-top: 0.5rem; color: var(--danger); font-size: 0.8rem">{message}</span>
                }.into_any(),
            })}
        </div>
    }.into_any()
}


#[cfg(test)]
mod answer_first_choice_tests {
    use super::*;

    fn problem(answer: &str, distractors: [&str; 3]) -> PracticeProblem {
        PracticeProblem {
            question: "How many more children chose soccer?".to_owned(),
            answer: answer.to_owned(),
            difficulty_permille: 250,
            bloom_level: "Apply".to_owned(),
            hints: vec!["Compare the two counts.".to_owned()],
            explanation: "Subtract the smaller count from the larger count.".to_owned(),
            distractors: distractors.into_iter().map(str::to_owned).collect(),
        }
    }

    #[test]
    fn answer_first_choices_include_one_answer_and_three_distinct_wrong_choices() {
        let item = problem("3 children", ["13 children", "2 children", "5 children"]);
        let choices = answer_first_choices(&item).expect("valid choice set should be usable");
        assert_eq!(choices.len(), 4);
        assert!(
            choices
                .iter()
                .any(|choice| normalized_choice(choice) == normalized_choice("3 children"))
        );
        assert_eq!(
            choices
                .iter()
                .map(|choice| normalized_choice(choice))
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            4
        );
    }

    #[test]
    fn answer_first_choices_reject_duplicate_or_missing_distractors() {
        assert!(answer_first_choices(&problem("3", ["1", "1", "4"])).is_none());

        let mut incomplete = problem("3", ["1", "2", "4"]);
        incomplete.distractors.pop();
        assert!(answer_first_choices(&incomplete).is_none());
    }

    #[test]
    fn answer_first_choices_reject_numeric_answer_with_omitted_unit() {
        let item = problem("3 children", ["13", "2 children", "3"]);
        assert!(numeric_choices_collide("3 children", "3"));
        assert!(answer_first_choices(&item).is_none());
    }

    #[test]
    fn answer_first_choices_reject_equivalent_fraction_and_decimal_choices() {
        assert!(numeric_choices_collide("1/2", "2/4"));
        assert!(numeric_choices_collide("0.5", "1/2"));
        assert!(!numeric_choices_collide("12 cm", "12 square units"));

        let item = problem("1/2", ["2/4", "1/3", "3/4"]);
        assert!(answer_first_choices(&item).is_none());
    }
}
