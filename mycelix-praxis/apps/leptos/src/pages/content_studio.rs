// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Local-first, beginner-friendly lesson authoring. Exported lessons remain drafts.

use crate::persistence;
use leptos::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use wasm_bindgen::{JsCast, JsValue};

const DRAFT_KEY: &str = "praxis_community_creator_draft_v1";

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PracticeDraft {
    question: String,
    answer: String,
    explanation: String,
    hints: String,
    distractors: String,
}

impl Default for PracticeDraft {
    fn default() -> Self {
        Self {
            question: String::new(),
            answer: String::new(),
            explanation: String::new(),
            hints: String::new(),
            distractors: String::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CreatorDraft {
    node_id: String,
    standard_code: String,
    title: String,
    grade_level: String,
    bloom_level: String,
    language: String,
    creator_name: String,
    license_id: String,
    rights_confirmed: bool,
    explanation: String,
    example_problem: String,
    example_steps: String,
    example_answer: String,
    general_hints: String,
    vocabulary_term: String,
    vocabulary_definition: String,
    vocabulary_example: String,
    misconception: String,
    misconception_correction: String,
    misconception_reason: String,
    source_title: String,
    source_url: String,
    source_rights_note: String,
    accessibility_notes: String,
    practices: Vec<PracticeDraft>,
}

impl Default for CreatorDraft {
    fn default() -> Self {
        Self {
            node_id: "DRAFT.MAP_ME".to_string(),
            standard_code: String::new(),
            title: String::new(),
            grade_level: "Grade5".to_string(),
            bloom_level: "Understand".to_string(),
            language: "en-ZA".to_string(),
            creator_name: String::new(),
            license_id: "UNDECIDED".to_string(),
            rights_confirmed: false,
            explanation: String::new(),
            example_problem: String::new(),
            example_steps: String::new(),
            example_answer: String::new(),
            general_hints: String::new(),
            vocabulary_term: String::new(),
            vocabulary_definition: String::new(),
            vocabulary_example: String::new(),
            misconception: String::new(),
            misconception_correction: String::new(),
            misconception_reason: String::new(),
            source_title: String::new(),
            source_url: String::new(),
            source_rights_note: String::new(),
            accessibility_notes: String::new(),
            practices: vec![PracticeDraft::default(); 3],
        }
    }
}

impl CreatorDraft {
    fn ensure_three_practices(&mut self) {
        self.practices.resize(3, PracticeDraft::default());
        self.practices.truncate(3);
    }
}

fn example_draft() -> CreatorDraft {
    CreatorDraft {
        node_id: "DRAFT.MAP_ME".to_string(),
        standard_code: "LOCAL.MATHS.AREA-PERIMETER.01".to_string(),
        title: "Area and perimeter measure different things".to_string(),
        grade_level: "Grade5".to_string(),
        bloom_level: "Understand".to_string(),
        language: "en-ZA".to_string(),
        creator_name: "Example educator".to_string(),
        license_id: "UNDECIDED".to_string(),
        rights_confirmed: false,
        explanation: "Perimeter is the total distance around the outside edge of a flat shape. Area is how much surface the shape covers. Imagine a rectangular garden that is 5 metres long and 3 metres wide. Its perimeter is 5 + 3 + 5 + 3 = 16 metres. Its area is 5 × 3 = 15 square metres. Perimeter uses length units; area uses square units. These numbers describe different properties, so they do not need to be equal.".to_string(),
        example_problem: "A rectangle is 5 m long and 3 m wide. Find its perimeter and area.".to_string(),
        example_steps: "Perimeter is the distance around the edge => 5 + 3 + 5 + 3 = 16 m\nArea measures the surface covered inside => 5 × 3 = 15 m²\nCheck that the units match the quantities => Perimeter is in metres; area is in square metres".to_string(),
        example_answer: "Perimeter = 16 m. Area = 15 m².".to_string(),
        general_hints: "Ask whether the question is about the boundary or the inside.\nPerimeter adds side lengths; area multiplies length by width for a rectangle.\nCheck that the units describe the quantity.".to_string(),
        vocabulary_term: "Perimeter".to_string(),
        vocabulary_definition: "The total distance around the outside edge of a flat shape.".to_string(),
        vocabulary_example: "A 5 m by 3 m rectangle has a perimeter of 16 m.".to_string(),
        misconception: "Area and perimeter always have the same value.".to_string(),
        misconception_correction: "They measure different properties and use different units.".to_string(),
        misconception_reason: "Both describe the same shape, so the quantities are easy to confuse.".to_string(),
        source_title: String::new(),
        source_url: String::new(),
        source_rights_note: String::new(),
        accessibility_notes: "Describe meaningful visuals in text. Do not rely on colour alone.\nNo educator review has been completed.".to_string(),
        practices: vec![
            PracticeDraft {
                question: "A rectangle is 4 cm long and 2 cm wide. What is its perimeter?".to_string(),
                answer: "12 cm".to_string(),
                explanation: "Perimeter is the total distance around the edge: 4 + 2 + 4 + 2 = 12 cm.".to_string(),
                hints: "Count the outside sides.\nAdd all four side lengths.".to_string(),
                distractors: "8 cm | 6 cm | 10 cm".to_string(),
            },
            PracticeDraft {
                question: "A rectangle is 4 cm long and 2 cm wide. What is its area?".to_string(),
                answer: "8 square centimetres".to_string(),
                explanation: "Area = length × width = 4 × 2 = 8 square centimetres.".to_string(),
                hints: "Think about rows and columns of unit squares.\nMultiply length by width.".to_string(),
                distractors: "12 square centimetres | 6 square centimetres | 8 cm".to_string(),
            },
            PracticeDraft {
                question: "A learner adds the four sides to find area. What correction would help?".to_string(),
                answer: "Adding side lengths finds perimeter; area is length multiplied by width.".to_string(),
                explanation: "The boundary is measured by adding side lengths. The area of a rectangle is found by multiplying its length and width.".to_string(),
                hints: "Think about walking around the outside.\nThink about covering the inside with square units.".to_string(),
                distractors: "Area and perimeter always have the same value. | Area is measured in seconds. | The boundary is called area.".to_string(),
            },
        ],
    }
}

#[component]
pub fn ContentStudioPage() -> impl IntoView {
    let mut initial = persistence::load::<CreatorDraft>(DRAFT_KEY).unwrap_or_default();
    initial.ensure_three_practices();
    let (draft, set_draft) = signal(initial);
    let (message, set_message) = signal(
        "Drafts autosave in this browser. This page does not submit or publish them.".to_string(),
    );
    let (errors, set_errors) = signal(Vec::<String>::new());
    let (output, set_output) = signal(String::new());

    Effect::new(move |_| {
        persistence::save(DRAFT_KEY, &draft.get());
    });

    let build = move |_| match build_lesson_pack(&draft.get_untracked()) {
        Ok(json_text) => {
            set_output.set(json_text);
            set_errors.set(Vec::new());
            set_message.set(
                "Draft pack created. Review it before sharing; schema checks do not prove lesson correctness."
                    .to_string(),
            );
        }
        Err(found) => {
            set_output.set(String::new());
            set_errors.set(found);
            set_message.set(
                "A few fields need attention before Praxis can build the draft pack.".to_string(),
            );
        },
    };

    let download = move |_| {
        let value = output.get_untracked();
        if value.is_empty() {
            set_message.set("Build the draft pack first.".to_string());
            return;
        }
        match download_json(&value) {
            Ok(()) => set_message.set(
                "Download requested. Your lesson remains a local draft until you share it for review."
                    .to_string(),
            ),
            Err(error) => set_message.set(format!(
                "Browser download was unavailable: {error}. Use Copy JSON or select the preview manually."
            )),
        }
    };

    let copy_json = move |_| {
        let value = output.get_untracked();
        if value.is_empty() {
            set_message.set("Build the draft pack first.".to_string());
            return;
        }
        let Some(window) = web_sys::window() else {
            set_message.set("Clipboard access is unavailable in this browser context.".to_string());
            return;
        };
        let promise = window.navigator().clipboard().write_text(&value);
        wasm_bindgen_futures::spawn_local(async move {
            match wasm_bindgen_futures::JsFuture::from(promise).await {
                Ok(_) => set_message.set(
                    "JSON copied. It remains a private draft; share it for review before publication."
                        .to_string(),
                ),
                Err(_) => set_message.set(
                    "Clipboard permission was denied. Select the JSON preview and copy it manually."
                        .to_string(),
                ),
            }
        });
    };

    let load_example = move |_| {
        set_draft.set(example_draft());
        set_output.set(String::new());
        set_errors.set(Vec::new());
        set_message.set(
            "Example loaded. Adapt it in your own words, map the objective, and keep it as a draft until reviewed."
                .to_string(),
        );
    };

    let download_review_brief = move |_| {
        let brief = build_review_markdown(&draft.get_untracked());
        match download_text_file(
            &brief,
            "praxis-lesson-review.md",
            "text/markdown;charset=utf-8",
        ) {
            Ok(()) => set_message.set(
                "Review brief downloaded. It is a review aid, not publication approval."
                    .to_string(),
            ),
            Err(error) => set_message.set(format!("Could not download the review brief: {error}")),
        }
    };

    let reset = move |_| {
        persistence::remove(DRAFT_KEY);
        set_draft.set(CreatorDraft::default());
        set_output.set(String::new());
        set_errors.set(Vec::new());
        set_message.set("A new blank local draft is ready.".to_string());
    };

    view! {
        <section class="content-studio" style="max-width: 980px; margin: 0 auto; padding: 1.25rem; color: var(--text)">
            <header style="margin-bottom: 1.25rem">
                <p style="color: var(--accent); font-size: 0.8rem; text-transform: uppercase; letter-spacing: 0.08em">"Community learning"</p>
                <h1 style="font-size: 2rem; margin: 0.3rem 0">"Create a lesson"</h1>
                <p style="color: var(--text-secondary); max-width: 72ch">
                    "Share what you know. Start with one idea, show how it works, then give learners a few ways to practise. Your work is saved in this browser and exported as a draft for review. Do not enter learner personal data or confidential information."
                </p>
                <div role="status" aria-live="polite" style="border-left: 3px solid var(--accent); padding: 0.65rem 0.9rem; background: var(--surface-high); border-radius: 0 8px 8px 0">
                    {move || message.get()}
                </div>
            </header>

            <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 1rem">
                <section style="background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 1rem">
                    <h2 style="margin-top: 0">"1. Who is this for?"</h2>
                    <label for="creator-title">"Lesson title"</label>
                    <input id="creator-title" type="text" maxlength="160" placeholder="e.g. Area and perimeter measure different things"
                        prop:value=move || draft.get().title
                        on:input=move |ev| set_draft.update(|d| d.title = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box" />

                    <label for="creator-grade">"Learner group / grade"</label>
                    <input id="creator-grade" type="text" placeholder="e.g. Grade5, adults, beginner technicians"
                        prop:value=move || draft.get().grade_level
                        on:input=move |ev| set_draft.update(|d| d.grade_level = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box" />

                    <label for="creator-node">"Curriculum node ID (if known)"</label>
                    <input id="creator-node" type="text" placeholder="Use an existing ID or leave DRAFT.MAP_ME"
                        prop:value=move || draft.get().node_id
                        on:input=move |ev| set_draft.update(|d| d.node_id = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.3rem; padding: 0.65rem; box-sizing: border-box" />
                    <p style="font-size: 0.8rem; color: var(--text-secondary); margin-top: 0">
                        "A curator must map placeholder IDs before a lesson can appear in the app. The export will not activate a node by itself."
                    </p>

                    <label for="creator-standard">"Learning objective / standard code"</label>
                    <input id="creator-standard" type="text" placeholder="e.g. LOCAL.MATHS.AREA.01"
                        prop:value=move || draft.get().standard_code
                        on:input=move |ev| set_draft.update(|d| d.standard_code = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box" />

                    <label for="creator-language">"Language tag"</label>
                    <input id="creator-language" type="text" placeholder="e.g. en-ZA, zu-ZA, af-ZA"
                        prop:value=move || draft.get().language
                        on:input=move |ev| set_draft.update(|d| d.language = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box" />

                    <label for="creator-bloom">"Main learning action"</label>
                    <select id="creator-bloom" prop:value=move || draft.get().bloom_level
                        on:change=move |ev| set_draft.update(|d| d.bloom_level = event_target_value(&ev))
                        style="display: block; width: 100%; margin-top: 0.35rem; padding: 0.65rem">
                        <option value="Remember">"Remember"</option>
                        <option value="Understand">"Understand"</option>
                        <option value="Apply">"Apply"</option>
                        <option value="Analyze">"Analyze"</option>
                        <option value="Evaluate">"Evaluate"</option>
                        <option value="Create">"Create"</option>
                    </select>
                </section>

                <section style="background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 1rem">
                    <h2 style="margin-top: 0">"2. Teach the idea"</h2>
                    <label for="creator-explanation">"Explanation"</label>
                    <textarea id="creator-explanation" rows="7" placeholder="Explain the idea plainly, why it matters, and how it connects to what learners know."
                        prop:value=move || draft.get().explanation
                        on:input=move |ev| set_draft.update(|d| d.explanation = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box"></textarea>

                    <label for="creator-example-problem">"Worked example: problem"</label>
                    <textarea id="creator-example-problem" rows="2" placeholder="Write one representative problem."
                        prop:value=move || draft.get().example_problem
                        on:input=move |ev| set_draft.update(|d| d.example_problem = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box"></textarea>

                    <label for="creator-example-steps">"Worked steps (one per line, use instruction => result)"</label>
                    <textarea id="creator-example-steps" rows="4" placeholder="Add the known side lengths => 5 m and 3 m&#10;Add all sides for perimeter => 5 + 3 + 5 + 3 = 16 m"
                        prop:value=move || draft.get().example_steps
                        on:input=move |ev| set_draft.update(|d| d.example_steps = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box"></textarea>

                    <label for="creator-example-answer">"Worked example: answer"</label>
                    <input id="creator-example-answer" type="text" placeholder="Give the correct answer and relevant units."
                        prop:value=move || draft.get().example_answer
                        on:input=move |ev| set_draft.update(|d| d.example_answer = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.9rem; padding: 0.65rem; box-sizing: border-box" />

                    <label for="creator-general-hints">"Progressive hints (one per line)"</label>
                    <textarea id="creator-general-hints" rows="3" placeholder="Give a gentle nudge&#10;Suggest a first step&#10;Offer a fuller explanation if needed"
                        prop:value=move || draft.get().general_hints
                        on:input=move |ev| set_draft.update(|d| d.general_hints = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0; padding: 0.65rem; box-sizing: border-box"></textarea>
                </section>
            </div>

            <section style="margin-top: 1rem; background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 1rem">
                <h2 style="margin-top: 0">"3. Add three practice questions"</h2>
                <p style="color: var(--text-secondary)">
                    "Start with a straightforward check, then application, then a question that asks learners to explain their reasoning. Each question needs one correct answer, a reason, hints and three genuinely incorrect alternatives."
                </p>
                <PracticeEditor draft=draft set_draft=set_draft index=0 />
                <PracticeEditor draft=draft set_draft=set_draft index=1 />
                <PracticeEditor draft=draft set_draft=set_draft index=2 />
            </section>

            <section style="margin-top: 1rem; background: var(--surface-high); border: 1px solid var(--border); border-radius: 12px; padding: 1rem">
                <h2 style="margin-top: 0">"Live learner preview"</h2>
                <p style="color: var(--text-secondary)">"This view updates as you type. Hints and answers stay behind a disclosure so you can inspect the learner-facing sequence. It is a presentation preview, not a proof of correctness."</p>
                {move || {
                    let value = draft.get();
                    let preview_title = if value.title.trim().is_empty() {
                        "Untitled lesson".to_string()
                    } else {
                        value.title.clone()
                    };
                    let preview_explanation = if value.explanation.trim().is_empty() {
                        "Your explanation will appear here.".to_string()
                    } else {
                        value.explanation.clone()
                    };
                    let example_problem = if value.example_problem.trim().is_empty() {
                        "Your worked-example problem will appear here.".to_string()
                    } else {
                        value.example_problem.clone()
                    };
                    let example_answer = value.example_answer.clone();
                    let parsed_steps = value
                        .example_steps
                        .lines()
                        .filter_map(|line| {
                            let (instruction, result) = line.split_once("=>")?;
                            Some((instruction.trim().to_string(), result.trim().to_string()))
                        })
                        .collect::<Vec<_>>();
                    let practice = value.practices.clone();
                    view! {
                        <article style="background: var(--surface); border-radius: 8px; padding: 1rem; max-width: 780px">
                            <p style="font-size: 0.75rem; color: var(--accent); margin: 0">"LESSON DRAFT · " {value.grade_level.clone()} " · " {value.language.clone()}</p>
                            <h3 style="margin: 0.45rem 0">{preview_title}</h3>
                            <p style="line-height: 1.65; white-space: pre-wrap">{preview_explanation}</p>
                            <h4>"Worked example"</h4>
                            <p>{example_problem}</p>
                            <ol>
                                {parsed_steps.into_iter().map(|(instruction, result)| view! {
                                    <li style="margin: 0.45rem 0">
                                        <strong>{instruction}</strong>
                                        <div style="color: var(--text-secondary)">{result}</div>
                                    </li>
                                }).collect::<Vec<_>>()}
                            </ol>
                            <p><strong>"Answer: "</strong>{example_answer}</p>
                            <h4>"Try it yourself"</h4>
                            <ol>
                                {practice.into_iter().enumerate().map(|(index, item)| {
                                    let question = item.question.clone();
                                    let answer = item.answer.clone();
                                    let explanation = item.explanation.clone();
                                    let hints = lines(&item.hints);
                                    let distractors = item.distractors
                                        .split('|')
                                        .map(str::trim)
                                        .filter(|value| !value.is_empty())
                                        .map(str::to_owned)
                                        .collect::<Vec<_>>();
                                    view! {
                                        <li style="margin: 0.9rem 0">
                                            <p style="margin-bottom: 0.35rem"><strong>{index + 1} ". "</strong>{question}</p>
                                            <ul>
                                                {distractors.into_iter().map(|choice| view! {
                                                    <li>{choice}</li>
                                                }).collect::<Vec<_>>()}
                                            </ul>
                                            <details>
                                                <summary style="cursor: pointer">"Reveal hints and explanation"</summary>
                                                <ol>
                                                    {hints.into_iter().map(|hint| view! { <li>{hint}</li> }).collect::<Vec<_>>()}
                                                </ol>
                                                <p>{explanation}</p>
                                                <p><strong>"Correct answer: "</strong>{answer}</p>
                                            </details>
                                        </li>
                                    }
                                }).collect::<Vec<_>>()}
                            </ol>
                        </article>
                    }
                }}
            </section>

            <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 1rem; margin-top: 1rem">
                <section style="background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 1rem">
                    <h2 style="margin-top: 0">"4. Vocabulary and misconceptions"</h2>
                    <label for="creator-vocab-term">"Key term"</label>
                    <input id="creator-vocab-term" type="text" prop:value=move || draft.get().vocabulary_term
                        on:input=move |ev| set_draft.update(|d| d.vocabulary_term = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box" />
                    <label for="creator-vocab-def">"Definition"</label>
                    <textarea id="creator-vocab-def" rows="2" prop:value=move || draft.get().vocabulary_definition
                        on:input=move |ev| set_draft.update(|d| d.vocabulary_definition = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box"></textarea>
                    <label for="creator-vocab-example">"Example usage"</label>
                    <input id="creator-vocab-example" type="text" prop:value=move || draft.get().vocabulary_example
                        on:input=move |ev| set_draft.update(|d| d.vocabulary_example = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box" />

                    <label for="creator-misconception">"Common misconception"</label>
                    <textarea id="creator-misconception" rows="2" prop:value=move || draft.get().misconception
                        on:input=move |ev| set_draft.update(|d| d.misconception = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box"></textarea>
                    <label for="creator-correction">"Correction"</label>
                    <textarea id="creator-correction" rows="2" prop:value=move || draft.get().misconception_correction
                        on:input=move |ev| set_draft.update(|d| d.misconception_correction = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box"></textarea>
                    <label for="creator-misconception-reason">"Why might a learner think this?"</label>
                    <textarea id="creator-misconception-reason" rows="2" prop:value=move || draft.get().misconception_reason
                        on:input=move |ev| set_draft.update(|d| d.misconception_reason = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0; padding: 0.65rem; box-sizing: border-box"></textarea>
                </section>

                <section style="background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 1rem">
                    <h2 style="margin-top: 0">"5. Attribution and sharing"</h2>
                    <label for="creator-name">"Your chosen display name"</label>
                    <input id="creator-name" type="text" prop:value=move || draft.get().creator_name
                        on:input=move |ev| set_draft.update(|d| d.creator_name = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box" />

                    <label for="creator-license">"Reuse licence"</label>
                    <select id="creator-license" prop:value=move || draft.get().license_id
                        on:change=move |ev| set_draft.update(|d| d.license_id = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem">
                        <option value="UNDECIDED">"Choose later (draft only)"</option>
                        <option value="CC0-1.0">"CC0 — waive rights where permitted"</option>
                        <option value="CC-BY-4.0">"CC BY — attribution"</option>
                        <option value="CC-BY-SA-4.0">"CC BY-SA — attribution + share alike"</option>
                        <option value="CC-BY-NC-4.0">"CC BY-NC — attribution + non-commercial"</option>
                        <option value="CC-BY-NC-SA-4.0">"CC BY-NC-SA — non-commercial + share alike"</option>
                        <option value="CC-BY-ND-4.0">"CC BY-ND — no adaptations"</option>
                        <option value="CC-BY-NC-ND-4.0">"CC BY-NC-ND — non-commercial, no adaptations"</option>
                        <option value="All-Rights-Reserved">"All rights reserved (do not publish as open content)"</option>
                    </select>
                    <p style="font-size: 0.8rem; color: var(--text-secondary)">
                        "Licences grant different permissions and may be irrevocable once applied. Choose only if you own or control the rights. "
                        <a href="https://creativecommons.org/chooser/" target="_blank" rel="noopener noreferrer">"Read the licence chooser"</a>
                    </p>

                    <label for="creator-source-title">"Source title (optional)"</label>
                    <input id="creator-source-title" type="text" prop:value=move || draft.get().source_title
                        on:input=move |ev| set_draft.update(|d| d.source_title = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box" />
                    <label for="creator-source-url">"Source URL (optional)"</label>
                    <input id="creator-source-url" type="url" prop:value=move || draft.get().source_url
                        on:input=move |ev| set_draft.update(|d| d.source_url = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box" />
                    <label for="creator-source-rights">"Source licence / rights note (optional)"</label>
                    <textarea id="creator-source-rights" rows="2" prop:value=move || draft.get().source_rights_note
                        on:input=move |ev| set_draft.update(|d| d.source_rights_note = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0 0.8rem; padding: 0.65rem; box-sizing: border-box"></textarea>

                    <label for="creator-accessibility">"Accessibility and language notes"</label>
                    <textarea id="creator-accessibility" rows="3" placeholder="Describe visual alternatives, reading level, offline requirements, or language decisions."
                        prop:value=move || draft.get().accessibility_notes
                        on:input=move |ev| set_draft.update(|d| d.accessibility_notes = event_target_value(&ev))
                        style="display: block; width: 100%; margin: 0.35rem 0; padding: 0.65rem; box-sizing: border-box"></textarea>

                    <label style="display: flex; align-items: flex-start; gap: 0.55rem; margin-top: 0.75rem">
                        <input type="checkbox" prop:checked=move || draft.get().rights_confirmed
                            on:change=move |ev| set_draft.update(|d| d.rights_confirmed = checkbox_is_checked(&ev)) />
                        <span>"I created this material or have permission to share every third-party component under the selected licence."</span>
                    </label>
                    <p style="font-size: 0.8rem; color: var(--text-secondary)">
                        "This is a rights record for a draft, not legal approval. This screen always exports private draft status; public release requires separate review."
                    </p>
                </section>
            </div>

            <section style="margin-top: 1rem; background: var(--surface-high); border: 1px solid var(--border); border-radius: 12px; padding: 1rem">
                <h2 style="margin-top: 0">"6. Build your lesson pack"</h2>
                <p style="color: var(--text-secondary)">
                    "Assessment prompts and flashcards are initially derived from the three practice questions to reduce repetitive typing. Edit them in the downloaded JSON before review. Automatic checks catch missing fields, duplicate choices and some numeric answer collisions, but a human must review meaning and correctness."
                </p>
                {move || {
                    let problems = errors.get();
                    if problems.is_empty() {
                        view! { <span></span> }.into_any()
                    } else {
                        view! {
                            <div role="alert" style="padding: 0.75rem; border: 1px solid var(--error); border-radius: 8px; margin-bottom: 0.75rem">
                                <strong>"Please fix these items:"</strong>
                                <ul>{problems.into_iter().map(|problem| view! { <li>{problem}</li> }).collect::<Vec<_>>()}</ul>
                            </div>
                        }.into_any()
                    }
                }}
                <div style="display: flex; flex-wrap: wrap; gap: 0.6rem">
                    <button class="btn-outline" on:click=load_example>"Start from example"</button>
                    <button class="btn-primary" on:click=build>"Build draft JSON"</button>
                    <button class="btn-outline" on:click=copy_json disabled=move || output.get().is_empty()>"Copy JSON"</button>
                    <button class="btn-outline" on:click=download disabled=move || output.get().is_empty()>"Download JSON"</button>
                    <button class="btn-outline" on:click=download_review_brief>"Download review brief"</button>
                    <button class="btn-outline" on:click=reset>"Start blank draft"</button>
                </div>
                {move || {
                    let current = output.get();
                    if current.is_empty() {
                        view! {
                            <p style="color: var(--text-secondary); font-size: 0.9rem">
                                "Your preview will appear here after the draft passes the basic checks. The app does not publish this content."
                            </p>
                        }.into_any()
                    } else {
                        view! {
                            <div style="margin-top: 1rem">
                                <label for="creator-json-output">"Generated lesson JSON (editable after downloading)"</label>
                                <textarea id="creator-json-output" rows="18" readonly=true prop:value=current
                                    style="display: block; width: 100%; box-sizing: border-box; margin-top: 0.35rem; padding: 0.65rem; font-family: monospace; font-size: 0.78rem"></textarea>
                            </div>
                        }.into_any()
                    }
                }}
            </section>
        </section>
    }
}

#[component]
fn PracticeEditor(
    draft: ReadSignal<CreatorDraft>,
    set_draft: WriteSignal<CreatorDraft>,
    index: usize,
) -> impl IntoView {
    let question_id = format!("creator-practice-question-{index}");
    let answer_id = format!("creator-practice-answer-{index}");
    let explanation_id = format!("creator-practice-explanation-{index}");
    let hints_id = format!("creator-practice-hints-{index}");
    let distractors_id = format!("creator-practice-distractors-{index}");

    view! {
        <details open={index == 0} style="border-top: 1px solid var(--border); padding: 0.75rem 0">
            <summary style="cursor: pointer; font-weight: 600">"Practice question "{index + 1}</summary>
            <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 0.8rem; padding-top: 0.75rem">
                <div>
                    <label for=question_id.clone()>"Question"</label>
                    <textarea id=question_id rows="3" prop:value=move || draft.get().practices[index].question.clone()
                        on:input=move |ev| set_draft.update(|d| d.practices[index].question = event_target_value(&ev))
                        style="display: block; width: 100%; box-sizing: border-box; margin-top: 0.3rem; padding: 0.6rem"></textarea>
                </div>
                <div>
                    <label for=answer_id.clone()>"Correct answer"</label>
                    <textarea id=answer_id rows="2" prop:value=move || draft.get().practices[index].answer.clone()
                        on:input=move |ev| set_draft.update(|d| d.practices[index].answer = event_target_value(&ev))
                        style="display: block; width: 100%; box-sizing: border-box; margin-top: 0.3rem; padding: 0.6rem"></textarea>
                    <label for=explanation_id.clone() style="display: block; margin-top: 0.6rem">"Why is it correct?"</label>
                    <textarea id=explanation_id rows="3" prop:value=move || draft.get().practices[index].explanation.clone()
                        on:input=move |ev| set_draft.update(|d| d.practices[index].explanation = event_target_value(&ev))
                        style="display: block; width: 100%; box-sizing: border-box; margin-top: 0.3rem; padding: 0.6rem"></textarea>
                </div>
                <div>
                    <label for=hints_id.clone()>"Hints (one per line)"</label>
                    <textarea id=hints_id rows="3" prop:value=move || draft.get().practices[index].hints.clone()
                        on:input=move |ev| set_draft.update(|d| d.practices[index].hints = event_target_value(&ev))
                        style="display: block; width: 100%; box-sizing: border-box; margin-top: 0.3rem; padding: 0.6rem"></textarea>
                    <label for=distractors_id.clone() style="display: block; margin-top: 0.6rem">"Three wrong answers (separate with | )"</label>
                    <textarea id=distractors_id rows="3" placeholder="Wrong answer A | Wrong answer B | Wrong answer C"
                        prop:value=move || draft.get().practices[index].distractors.clone()
                        on:input=move |ev| set_draft.update(|d| d.practices[index].distractors = event_target_value(&ev))
                        style="display: block; width: 100%; box-sizing: border-box; margin-top: 0.3rem; padding: 0.6rem"></textarea>
                </div>
            </div>
        </details>
    }
}

fn checkbox_is_checked(event: &leptos::ev::Event) -> bool {
    event
        .target()
        .and_then(|target| target.dyn_into::<web_sys::HtmlInputElement>().ok())
        .is_some_and(|input| input.checked())
}

fn nonempty(value: &str, label: &str, errors: &mut Vec<String>) {
    if value.trim().is_empty() {
        errors.push(format!("{label} is required."));
    }
}

fn lines(value: &str) -> Vec<String> {
    value.lines().map(str::trim).filter(|line| !line.is_empty()).map(str::to_owned).collect()
}

fn normalized_choice(value: &str) -> String {
    value
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .trim_end_matches(['.', ',', ';', ':', '!', '?'])
        .to_lowercase()
}

fn scalar_answer(value: &str) -> Option<(f64, String)> {
    let normalized = value.trim().trim_end_matches(['.', ',', ';', ':', '!', '?']);
    let mut parts = normalized.splitn(2, char::is_whitespace);
    let number = parts.next()?;
    let numeric = if let Some((numerator, denominator)) = number.split_once('/') {
        let numerator = numerator.parse::<f64>().ok()?;
        let denominator = denominator.parse::<f64>().ok()?;
        if denominator == 0.0 {
            return None;
        }
        numerator / denominator
    } else {
        number.parse::<f64>().ok()?
    };
    let unit = parts.next().unwrap_or_default().trim().to_lowercase();
    Some((numeric, unit))
}

fn numerically_equivalent(answer: &str, distractor: &str) -> bool {
    let Some((a, a_unit)) = scalar_answer(answer) else {
        return false;
    };
    let Some((b, b_unit)) = scalar_answer(distractor) else {
        return false;
    };
    (a - b).abs() < 1e-9 && (a_unit.is_empty() || b_unit.is_empty() || a_unit == b_unit)
}

fn build_lesson_pack(draft: &CreatorDraft) -> Result<String, Vec<String>> {
    let mut errors = Vec::new();
    nonempty(&draft.title, "Lesson title", &mut errors);
    nonempty(&draft.grade_level, "Learner group / grade", &mut errors);
    nonempty(&draft.standard_code, "Learning objective / standard code", &mut errors);
    nonempty(&draft.language, "Language tag", &mut errors);
    if !["Remember", "Understand", "Apply", "Analyze", "Evaluate", "Create"]
        .contains(&draft.bloom_level.as_str())
    {
        errors.push("Choose a supported main learning action.".to_string());
    }
    if ![
        "UNDECIDED", "CC0-1.0", "CC-BY-4.0", "CC-BY-SA-4.0",
        "CC-BY-NC-4.0", "CC-BY-NC-SA-4.0", "CC-BY-ND-4.0",
        "CC-BY-NC-ND-4.0", "All-Rights-Reserved"
    ]
    .contains(&draft.license_id.as_str())
    {
        errors.push("Choose a supported reuse-licence option.".to_string());
    }
    nonempty(&draft.explanation, "Explanation", &mut errors);
    if draft.explanation.trim().chars().count() < 80 {
        errors.push("The explanation needs at least 80 characters; describe the idea, not just the answer.".to_string());
    }
    nonempty(&draft.example_problem, "Worked-example problem", &mut errors);
    nonempty(&draft.example_steps, "Worked-example steps", &mut errors);
    nonempty(&draft.example_answer, "Worked-example answer", &mut errors);
    nonempty(&draft.general_hints, "Progressive hints", &mut errors);
    nonempty(&draft.vocabulary_term, "Key vocabulary term", &mut errors);
    nonempty(&draft.vocabulary_definition, "Key vocabulary definition", &mut errors);
    nonempty(&draft.vocabulary_example, "Key vocabulary example", &mut errors);
    nonempty(&draft.misconception, "Common misconception", &mut errors);
    nonempty(&draft.misconception_correction, "Misconception correction", &mut errors);
    nonempty(&draft.misconception_reason, "Why the misconception occurs", &mut errors);
    nonempty(&draft.creator_name, "Creator display name", &mut errors);
    if draft.practices.len() != 3 {
        errors.push("This first version needs exactly three practice questions.".to_string());
    }
    let unique_prompts = draft
        .practices
        .iter()
        .map(|practice| normalized_choice(&practice.question))
        .collect::<std::collections::BTreeSet<_>>();
    if unique_prompts.len() != draft.practices.len() {
        errors.push("Practice questions must test different things; duplicate prompts were found.".to_string());
    }

    let mut parsed_steps = Vec::new();
    for (index, line) in draft.example_steps.lines().filter(|line| !line.trim().is_empty()).enumerate() {
        let Some((instruction, result)) = line.split_once("=>") else {
            errors.push(format!("Worked-example step {} must use 'instruction => result'.", index + 1));
            continue;
        };
        if instruction.trim().is_empty() || result.trim().is_empty() {
            errors.push(format!("Worked-example step {} needs both an instruction and a result.", index + 1));
            continue;
        }
        parsed_steps.push(json!({
            "instruction": instruction.trim(),
            "result": result.trim()
        }));
    }
    if parsed_steps.is_empty() {
        errors.push("Add at least one worked-example step.".to_string());
    }

    let mut parsed_practice = Vec::new();
    for (index, practice) in draft.practices.iter().enumerate() {
        nonempty(&practice.question, &format!("Practice {} question", index + 1), &mut errors);
        nonempty(&practice.answer, &format!("Practice {} correct answer", index + 1), &mut errors);
        nonempty(&practice.explanation, &format!("Practice {} explanation", index + 1), &mut errors);
        let hints = lines(&practice.hints);
        if hints.is_empty() {
            errors.push(format!("Practice {} needs at least one hint.", index + 1));
        }
        let distractors: Vec<String> = practice.distractors.split('|').map(str::trim).filter(|s| !s.is_empty()).map(str::to_owned).collect();
        if distractors.len() != 3 {
            errors.push(format!("Practice {} needs exactly three wrong answers separated by |.", index + 1));
        } else {
            let answer_key = normalized_choice(&practice.answer);
            let distractor_keys: Vec<String> = distractors.iter().map(|item| normalized_choice(item)).collect();
            if distractor_keys.iter().any(|item| item == &answer_key) {
                errors.push(format!("Practice {} repeats the correct answer in its wrong-answer list.", index + 1));
            }
            if distractor_keys.iter().collect::<std::collections::BTreeSet<_>>().len() != 3 {
                errors.push(format!("Practice {} has duplicate wrong answers.", index + 1));
            }
            if distractors.iter().any(|item| numerically_equivalent(&practice.answer, item)) {
                errors.push(format!("Practice {} has a numeric answer collision (e.g. the same value with an omitted unit or equivalent fraction).", index + 1));
            }
        }
        let bloom = match index {
            0 => "Remember",
            1 => "Apply",
            _ => "Analyze",
        };
        parsed_practice.push(json!({
            "question": practice.question.trim(),
            "answer": practice.answer.trim(),
            "difficulty_permille": [250, 500, 700][index.min(2)],
            "bloom_level": bloom,
            "hints": hints,
            "explanation": practice.explanation.trim(),
            "distractors": distractors
        }));
    }

    if !draft.source_title.trim().is_empty() || !draft.source_url.trim().is_empty() || !draft.source_rights_note.trim().is_empty() {
        nonempty(&draft.source_title, "Source title", &mut errors);
        nonempty(&draft.source_url, "Source URL", &mut errors);
        nonempty(&draft.source_rights_note, "Source licence / rights note", &mut errors);
        if !(draft.source_url.starts_with("https://") || draft.source_url.starts_with("http://")) {
            errors.push("A source URL must begin with https:// or http://.".to_string());
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    let bloom_distribution = vec![
        json!(["Remember", 1]),
        json!(["Apply", 1]),
        json!(["Analyze", 1])
    ];
    let assessment_items: Vec<Value> = parsed_practice.iter().enumerate().map(|(index, practice)| {
        json!({
            "question": practice["question"],
            "item_type": "ShortAnswer",
            "correct_answer": practice["answer"],
            "points": index + 1,
            "bloom_level": practice["bloom_level"],
            "rubric": practice["explanation"],
            "hint": practice["hints"][0]
        })
    }).collect();
    let total_points: usize = (1..=assessment_items.len()).sum();
    let source_records = if draft.source_title.trim().is_empty() {
        Vec::<Value>::new()
    } else {
        vec![json!({
            "title": draft.source_title.trim(),
            "url": draft.source_url.trim(),
            "use_type": "other",
            "license_or_rights_note": draft.source_rights_note.trim()
        })]
    };
    let accessibility = lines(&draft.accessibility_notes);
    let accessibility_notes = if accessibility.is_empty() {
        vec!["No accessibility review recorded yet; describe meaningful visuals in text and avoid colour-only meaning.".to_string()]
    } else {
        accessibility
    };
    let licence_url: Option<&str> = match draft.license_id.as_str() {
        "CC0-1.0" => Some("https://creativecommons.org/publicdomain/zero/1.0/"),
        "CC-BY-4.0" => Some("https://creativecommons.org/licenses/by/4.0/"),
        "CC-BY-SA-4.0" => Some("https://creativecommons.org/licenses/by-sa/4.0/"),
        "CC-BY-NC-4.0" => Some("https://creativecommons.org/licenses/by-nc/4.0/"),
        "CC-BY-NC-SA-4.0" => Some("https://creativecommons.org/licenses/by-nc-sa/4.0/"),
        "CC-BY-ND-4.0" => Some("https://creativecommons.org/licenses/by-nd/4.0/"),
        "CC-BY-NC-ND-4.0" => Some("https://creativecommons.org/licenses/by-nc-nd/4.0/"),
        _ => None
    };
    let creator_statement = format!("Created by {}.", draft.creator_name.trim());
    let hints = lines(&draft.general_hints);
    let quality = json!({
        "coherence": 0.0,
        "hallucination_detected": false,
        "needs_human_review": true,
        "veto_count": 0,
        "review_status": "draft_requires_educator_review",
        "metrics_valid_for_current_revision": false,
        "review_notes": "Local Creator Studio draft. Review facts, answers, rights, standards alignment and accessibility; no learning efficacy has been established."
    });
    let lesson = json!({
        "standard_code": draft.standard_code.trim(),
        "title": draft.title.trim(),
        "grade_level": draft.grade_level.trim(),
        "bloom_level": draft.bloom_level,
        "explanation": draft.explanation.trim(),
        "examples": [{
            "problem": draft.example_problem.trim(),
            "steps": parsed_steps,
            "answer": draft.example_answer.trim(),
            "visual_description": null
        }],
        "practice_problems": parsed_practice,
        "hints": hints,
        "key_vocabulary": [{
            "term": draft.vocabulary_term.trim(),
            "definition": draft.vocabulary_definition.trim(),
            "example_usage": draft.vocabulary_example.trim()
        }],
        "misconceptions": [{
            "misconception": draft.misconception.trim(),
            "correction": draft.misconception_correction.trim(),
            "why_students_think_this": draft.misconception_reason.trim()
        }],
        "generation_quality": {
            "coherence": 0.0,
            "hallucination_detected": false,
            "veto_count": 0,
            "epistemic_confidence": 0.0,
            "needs_human_review": true,
            "metrics_valid_for_current_revision": false,
            "review_notes": "This was authored with Creator Studio; no independent review has been completed."
        }
    });
    let flashcards: Vec<Value> = draft.practices.iter().map(|practice| json!({
        "front": practice.question.trim(),
        "back": practice.answer.trim(),
        "tags": [draft.standard_code.trim(), draft.grade_level.trim()],
        "difficulty_permille": 400
    })).collect();
    let pack = json!({
        "$schema": "https://raw.githubusercontent.com/Luminous-Dynamics/mycelix/main/mycelix-praxis/schemas/community-lesson-v1.schema.json",
        "node_id": if draft.node_id.trim().is_empty() { "DRAFT.MAP_ME" } else { draft.node_id.trim() },
        "lesson": lesson,
        "assessment": {
            "standard_code": draft.standard_code.trim(),
            "title": format!("{} — check understanding", draft.title.trim()),
            "items": assessment_items,
            "total_points": total_points,
            "estimated_minutes": 10,
            "bloom_distribution": bloom_distribution
        },
        "flashcards": flashcards,
        "quality": quality,
        "contribution": {
            "schema_version": 1,
            "status": "draft",
            "visibility": "private_draft",
            "creator": {
                "display_name": draft.creator_name.trim(),
                "profile_url": null,
                "attribution_statement": creator_statement
            },
            "license_id": draft.license_id,
            "license_url": licence_url,
            "rights_confirmed": draft.rights_confirmed,
            "language": draft.language.trim(),
            "audience_note": format!("Intended audience: {}. Bloom target: {}.", draft.grade_level.trim(), draft.bloom_level),
            "accessibility_notes": accessibility_notes,
            "sources": source_records,
            "review_note": "Generated in local Creator Studio. This asset is a private draft. Request educator and rights review before publication.",
            "change_log": ["Initial local Creator Studio draft"]
        }
    });
    serde_json::to_string_pretty(&pack).map_err(|error| vec![format!("Could not encode draft JSON: {error}")])
}

fn build_review_markdown(draft: &CreatorDraft) -> String {
    let title = if draft.title.trim().is_empty() {
        "Untitled lesson".to_string()
    } else {
        draft.title.trim().to_string()
    };
    let mut report = format!(
        "# {title}\n\n> **PRIVATE DRAFT FOR REVIEW — NOT PUBLISHED.** This brief is a review aid, not educator approval or evidence of learning efficacy.\n\n## Intended learning\n\n- **Audience:** {}\n- **Language:** {}\n- **Learning objective / standard:** {}\n- **Curriculum node ID:** {}\n- **Primary learning action:** {}\n\n## Explanation\n\n{}\n\n## Worked example\n\n**Problem:** {}\n\n",
        draft.grade_level.trim(),
        draft.language.trim(),
        draft.standard_code.trim(),
        draft.node_id.trim(),
        draft.bloom_level.trim(),
        draft.explanation.trim(),
        draft.example_problem.trim(),
    );
    report.push_str("**Steps:**\n\n");
    let mut step_count = 0usize;
    for line in draft.example_steps.lines().filter(|line| !line.trim().is_empty()) {
        step_count += 1;
        if let Some((instruction, result)) = line.split_once("=>") {
            report.push_str(&format!(
                "{}. **{}** — {}\n",
                step_count,
                instruction.trim(),
                result.trim()
            ));
        } else {
            report.push_str(&format!("{}. {}\n", step_count, line.trim()));
        }
    }
    if step_count == 0 {
        report.push_str("_No worked steps have been added yet._\n");
    }
    report.push_str(&format!("\n**Worked answer:** {}\n", draft.example_answer.trim()));

    report.push_str("\n## Practice questions\n\n");
    for (index, practice) in draft.practices.iter().enumerate() {
        report.push_str(&format!(
            "### {}. {}\n\n**Expected answer:** {}\n\n**Why:** {}\n\n",
            index + 1,
            practice.question.trim(),
            practice.answer.trim(),
            practice.explanation.trim()
        ));
        let hints = lines(&practice.hints);
        if !hints.is_empty() {
            report.push_str("**Hints (progressively more specific):**\n");
            for hint in hints {
                report.push_str(&format!("- {hint}\n"));
            }
            report.push('\n');
        }
        let distractors = practice
            .distractors
            .split('|')
            .map(str::trim)
            .filter(|value| !value.is_empty())
            .collect::<Vec<_>>();
        if !distractors.is_empty() {
            report.push_str("**Proposed wrong-answer choices (verify each is genuinely wrong):**\n");
            for choice in distractors {
                report.push_str(&format!("- {choice}\n"));
            }
            report.push('\n');
        }
    }

    report.push_str(&format!(
        "## Vocabulary\n\n- **{}:** {}\n\n**Example usage:** {}\n",
        draft.vocabulary_term.trim(),
        draft.vocabulary_definition.trim(),
        draft.vocabulary_example.trim()
    ));
    report.push_str(&format!(
        "\n## Misconception and correction\n\n- **Misconception:** {}\n- **Correction:** {}\n- **Why it may seem plausible:** {}\n",
        draft.misconception.trim(),
        draft.misconception_correction.trim(),
        draft.misconception_reason.trim()
    ));
    report.push_str(&format!(
        "\n## Creator and reuse\n\n- **Creator attribution:** {}\n- **Reuse licence:** {}\n- **Rights confirmed by creator:** {}\n",
        draft.creator_name.trim(),
        draft.license_id,
        if draft.rights_confirmed {
            "Yes (creator attestation; review all third-party material)"
        } else {
            "No"
        }
    ));
    if !draft.source_title.trim().is_empty()
        || !draft.source_url.trim().is_empty()
        || !draft.source_rights_note.trim().is_empty()
    {
        report.push_str(&format!(
            "- **Source title:** {}\n- **Source URL:** {}\n- **Source rights/licence note:** {}\n",
            draft.source_title.trim(),
            draft.source_url.trim(),
            draft.source_rights_note.trim()
        ));
    }
    report.push_str("\n### Accessibility and language notes\n\n");
    let access_notes = lines(&draft.accessibility_notes);
    if access_notes.is_empty() {
        report.push_str("_No accessibility notes have been added._\n");
    } else {
        for note in access_notes {
            report.push_str(&format!("- {note}\n"));
        }
    }
    report.push_str(
        "\n## Review checklist\n\n- [ ] Verify facts, calculations and every answer key.\n- [ ] Check that each distractor is wrong and not a paraphrase or equivalent numeric answer.\n- [ ] Review suitability for the intended age, prior knowledge and language.\n- [ ] Verify source rights and licence compatibility.\n- [ ] Check accessibility and alternatives for visual/audio activities.\n- [ ] Map the objective to a real curriculum node.\n- [ ] Record educator review and update the content revision before publication.\n\n**Current state:** locally authored draft; no independent review or learning-outcome study is claimed.\n"
    );
    report
}

fn download_json(json_text: &str) -> Result<(), String> {
    download_text_file(json_text, "praxis-community-lesson.json", "application/json")
}

fn download_text_file(text: &str, filename: &str, mime_type: &str) -> Result<(), String> {
    let window = web_sys::window().ok_or_else(|| "no browser window".to_string())?;
    let document = window.document().ok_or_else(|| "no browser document".to_string())?;
    let chunks = js_sys::Array::new();
    chunks.push(&JsValue::from_str(text));
    let mut options = web_sys::BlobPropertyBag::new();
    options.set_type(mime_type);
    let blob = web_sys::Blob::new_with_str_sequence_and_options(&chunks, &options)
        .map_err(|error| format!("{error:?}"))?;
    let url = web_sys::Url::create_object_url_with_blob(&blob)
        .map_err(|error| format!("{error:?}"))?;
    let anchor = document
        .create_element("a")
        .map_err(|error| format!("{error:?}"))?
        .dyn_into::<web_sys::HtmlAnchorElement>()
        .map_err(|_| "could not create download link".to_string())?;
    anchor.set_href(&url);
    anchor.set_download(filename);
    if let Some(body) = document.body() {
        body.append_child(&anchor).map_err(|error| format!("{error:?}"))?;
        anchor.click();
        let _ = body.remove_child(&anchor);
    } else {
        let _ = web_sys::Url::revoke_object_url(&url);
        return Err("page body is unavailable".to_string());
    }
    let _ = web_sys::Url::revoke_object_url(&url);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_draft() -> CreatorDraft {
        let mut draft = CreatorDraft::default();
        draft.node_id = "local.test.area-perimeter".to_string();
        draft.standard_code = "LOCAL.MATHS.AREA-PERIMETER.01".to_string();
        draft.title = "Area and perimeter measure different things".to_string();
        draft.grade_level = "Grade5".to_string();
        draft.explanation = "Perimeter is the distance around the boundary. Area is the amount of surface covered inside a shape. They measure different properties and use different units.".to_string();
        draft.example_problem = "A rectangle is 5 m long and 3 m wide. Find perimeter and area.".to_string();
        draft.example_steps = "Add all four sides => 5 + 3 + 5 + 3 = 16 m\nMultiply length by width => 5 × 3 = 15 m²".to_string();
        draft.example_answer = "Perimeter = 16 m; area = 15 m².".to_string();
        draft.general_hints = "Check whether the question asks about the boundary or the inside.\nUse the units to check your result.".to_string();
        draft.vocabulary_term = "Perimeter".to_string();
        draft.vocabulary_definition = "The total distance around the outside of a flat shape.".to_string();
        draft.vocabulary_example = "The rectangle's perimeter is 16 m.".to_string();
        draft.misconception = "Area and perimeter always have the same numeric value.".to_string();
        draft.misconception_correction = "They measure different properties and use different units.".to_string();
        draft.misconception_reason = "Both describe the same shape, so they can be confused.".to_string();
        draft.creator_name = "Example educator".to_string();
        draft.practices = vec![
            PracticeDraft {
                question: "A rectangle is 4 cm by 2 cm. Find its perimeter.".to_string(),
                answer: "12 cm".to_string(),
                explanation: "Add 4 + 2 + 4 + 2 to get 12 cm.".to_string(),
                hints: "Count the outside sides.\nAdd all four side lengths.".to_string(),
                distractors: "8 cm | 6 cm | 10 cm".to_string(),
            },
            PracticeDraft {
                question: "A rectangle is 4 cm by 2 cm. Find its area.".to_string(),
                answer: "8 square centimetres".to_string(),
                explanation: "Multiply 4 cm by 2 cm to get 8 square centimetres.".to_string(),
                hints: "Count rows and columns of unit squares.\nMultiply length by width.".to_string(),
                distractors: "12 square centimetres | 6 square centimetres | 8 cm".to_string(),
            },
            PracticeDraft {
                question: "Explain why area and perimeter use different units.".to_string(),
                answer: "Perimeter is a length; area measures square units.".to_string(),
                explanation: "Adding side lengths measures a boundary, while multiplication of lengths measures surface coverage.".to_string(),
                hints: "Think about walking around the outside.\nThink about covering the inside with squares.".to_string(),
                distractors: "They always use metres | Area is measured in seconds | They always have the same value".to_string(),
            },
        ];
        draft
    }

    #[test]
    fn review_brief_is_human_readable_and_keeps_draft_review_boundaries() {
        let draft = example_draft();
        let report = build_review_markdown(&draft);
        assert!(report.starts_with("# Area and perimeter measure different things"));
        assert!(report.contains("PRIVATE DRAFT FOR REVIEW"));
        assert!(report.contains("## Practice questions"));
        assert!(report.contains("Proposed wrong-answer choices"));
        assert!(report.contains("Reuse licence: UNDECIDED"));
        assert!(report.contains("Rights confirmed by creator: No"));
        assert!(report.contains("## Review checklist"));
        assert!(report.contains("no independent review or learning-outcome study is claimed"));
    }

    #[test]
    fn example_draft_builds_without_publishing_or_claiming_validation() {
        let json_text = build_lesson_pack(&example_draft())
            .expect("the built-in example should satisfy basic authoring checks");
        let pack: Value = serde_json::from_str(&json_text).expect("example pack must be JSON");
        assert_eq!(pack["node_id"], "DRAFT.MAP_ME");
        assert_eq!(pack["contribution"]["status"], "draft");
        assert_eq!(pack["contribution"]["visibility"], "private_draft");
        assert_eq!(pack["contribution"]["license_id"], "UNDECIDED");
        assert_eq!(pack["contribution"]["rights_confirmed"], false);
        assert_eq!(pack["quality"]["needs_human_review"], true);
        assert_eq!(pack["quality"]["metrics_valid_for_current_revision"], false);
        assert_eq!(pack["lesson"]["practice_problems"].as_array().unwrap().len(), 3);
    }

    #[test]
    fn builder_exports_a_private_draft_with_review_gates() {
        let pack: Value = serde_json::from_str(&build_lesson_pack(&valid_draft()).unwrap())
            .expect("creator export must be valid JSON");
        assert_eq!(pack["contribution"]["status"], "draft");
        assert_eq!(pack["contribution"]["visibility"], "private_draft");
        assert_eq!(pack["quality"]["needs_human_review"], true);
        assert_eq!(pack["quality"]["metrics_valid_for_current_revision"], false);
        assert_eq!(pack["lesson"]["practice_problems"].as_array().unwrap().len(), 3);
        assert_eq!(pack["assessment"]["items"].as_array().unwrap().len(), 3);
        assert_eq!(pack["flashcards"].as_array().unwrap().len(), 3);
        assert_eq!(pack["lesson"]["examples"][0]["steps"].as_array().unwrap().len(), 2);
    }

    #[test]
    fn builder_rejects_correct_answers_and_numeric_equivalents_as_distractors() {
        let mut draft = valid_draft();
        draft.practices[0].distractors = "8 cm | 12 cm | 10 cm".to_string();
        assert!(
            build_lesson_pack(&draft)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("repeats the correct answer"))
        );

        draft.practices[0].distractors = "8 cm | 3 | 10 cm".to_string();
        draft.practices[0].answer = "3 children".to_string();
        assert!(
            build_lesson_pack(&draft)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("numeric answer collision"))
        );
    }

    #[test]
    fn builder_requires_steps_and_three_distinct_wrong_answers() {
        let mut draft = valid_draft();
        draft.example_steps = "A line without a result separator".to_string();
        draft.practices[1].distractors = "wrong | wrong | other".to_string();
        let errors = build_lesson_pack(&draft).unwrap_err();
        assert!(errors.iter().any(|error| error.contains("instruction => result")));
        assert!(errors.iter().any(|error| error.contains("duplicate wrong answers")));
    }


    #[test]
    fn exported_pack_round_trip_preserves_sources_rights_and_accessibility_notes() {
        let mut draft = valid_draft();
        draft.license_id = "CC-BY-4.0".to_string();
        draft.rights_confirmed = true;
        draft.source_title = "Open geometry reference".to_string();
        draft.source_url = "https://example.org/geometry".to_string();
        draft.source_rights_note = "CC BY 4.0; attribution required".to_string();
        draft.accessibility_notes =
            "Describe diagrams in text.\nDo not communicate meaning by colour alone.".to_string();

        let encoded = build_lesson_pack(&draft).expect("complete source metadata should export");
        let exported: Value = serde_json::from_str(&encoded).expect("export must be JSON");
        assert_eq!(exported["contribution"]["sources"][0]["title"], "Open geometry reference");
        assert_eq!(exported["contribution"]["sources"][0]["url"], "https://example.org/geometry");
        assert_eq!(
            exported["contribution"]["sources"][0]["license_or_rights_note"],
            "CC BY 4.0; attribution required"
        );
        assert_eq!(exported["contribution"]["license_id"], "CC-BY-4.0");
        assert_eq!(exported["contribution"]["license_url"], "https://creativecommons.org/licenses/by/4.0/");
        assert_eq!(exported["contribution"]["rights_confirmed"], true);
        assert_eq!(
            exported["contribution"]["accessibility_notes"],
            json!([
                "Describe diagrams in text.",
                "Do not communicate meaning by colour alone."
            ])
        );

        // JSON export/import through the generic value representation must preserve
        // the fields this creator workflow explicitly promises to carry.
        let reencoded = serde_json::to_string(&exported).expect("value should re-encode");
        let round_tripped: Value =
            serde_json::from_str(&reencoded).expect("re-encoded pack should parse");
        for pointer in [
            "/contribution/sources/0/title",
            "/contribution/sources/0/url",
            "/contribution/sources/0/license_or_rights_note",
            "/contribution/license_id",
            "/contribution/license_url",
            "/contribution/rights_confirmed",
            "/contribution/accessibility_notes/0",
            "/contribution/accessibility_notes/1",
        ] {
            assert_eq!(
                round_tripped.pointer(pointer),
                exported.pointer(pointer),
                "round-trip changed contract field {pointer}"
            );
        }
    }

    #[test]
    fn incomplete_or_unsafe_source_metadata_is_rejected() {
        let mut draft = valid_draft();
        draft.source_title = "Reference".to_string();

        let errors = build_lesson_pack(&draft).expect_err("partial source metadata must fail");
        assert!(errors.iter().any(|error| error.contains("Source URL")));
        assert!(errors.iter().any(|error| error.contains("Source licence / rights note")));

        draft.source_url = "javascript:alert(1)".to_string();
        draft.source_rights_note = "Rights unknown".to_string();
        let errors = build_lesson_pack(&draft).expect_err("non-HTTP source locator must fail");
        assert!(errors.iter().any(|error| error.contains("source URL must begin")));
    }

    #[test]
    fn confirming_reuse_rights_never_publishes_or_reviews_the_lesson() {
        let mut draft = valid_draft();
        draft.license_id = "CC-BY-4.0".to_string();
        draft.rights_confirmed = true;

        let encoded = build_lesson_pack(&draft).expect("valid draft should export");
        let pack: Value = serde_json::from_str(&encoded).expect("export must be JSON");

        assert_eq!(pack["contribution"]["rights_confirmed"], true);
        assert_eq!(pack["contribution"]["status"], "draft");
        assert_eq!(pack["contribution"]["visibility"], "private_draft");
        assert_eq!(pack["quality"]["needs_human_review"], true);
        assert_eq!(pack["quality"]["metrics_valid_for_current_revision"], false);
        assert_eq!(
            pack["lesson"]["generation_quality"]["needs_human_review"],
            true
        );
        assert_eq!(
            pack["lesson"]["generation_quality"]["metrics_valid_for_current_revision"],
            false
        );
        assert!(
            pack["contribution"]["review_note"]
                .as_str()
                .unwrap()
                .contains("private draft")
        );
    }

    #[test]
    fn assessment_and_flashcards_remain_bound_to_the_same_practice_items() {
        let pack: Value = serde_json::from_str(
            &build_lesson_pack(&valid_draft()).expect("valid draft should export"),
        )
        .expect("export must be JSON");

        let practice = pack["lesson"]["practice_problems"]
            .as_array()
            .expect("practice problems should be an array");
        let assessment = pack["assessment"]["items"]
            .as_array()
            .expect("assessment items should be an array");
        let flashcards = pack["flashcards"]
            .as_array()
            .expect("flashcards should be an array");

        assert_eq!(practice.len(), 3);
        assert_eq!(assessment.len(), practice.len());
        assert_eq!(flashcards.len(), practice.len());
        for index in 0..practice.len() {
            assert_eq!(
                assessment[index]["question"],
                practice[index]["question"],
                "assessment prompt drifted at index {index}"
            );
            assert_eq!(
                assessment[index]["correct_answer"],
                practice[index]["answer"],
                "assessment answer drifted at index {index}"
            );
            assert_eq!(
                flashcards[index]["front"],
                practice[index]["question"],
                "flashcard front drifted at index {index}"
            );
            assert_eq!(
                flashcards[index]["back"],
                practice[index]["answer"],
                "flashcard back drifted at index {index}"
            );
        }
    }

    #[test]
    fn blank_target_is_explicitly_exported_as_a_draft_mapping_placeholder() {
        let mut draft = valid_draft();
        draft.node_id.clear();

        let pack: Value = serde_json::from_str(
            &build_lesson_pack(&draft).expect("valid lesson draft should export"),
        )
        .expect("export must be JSON");

        assert_eq!(pack["node_id"], "DRAFT.MAP_ME");
        assert_eq!(pack["contribution"]["status"], "draft");
        assert_eq!(pack["contribution"]["visibility"], "private_draft");
        assert_eq!(pack["quality"]["needs_human_review"], true);
    }
}
