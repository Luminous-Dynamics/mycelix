// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Local-first, form-based lesson authoring. Drafts stay in browser localStorage
//! until a contributor explicitly exports the JSON for review.

use leptos::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::persistence;

const DRAFT_KEY: &str = "praxis_lesson_authoring_draft_v1";

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PracticeDraft {
    question: String,
    answer: String,
    explanation: String,
    hint: String,
    distractors: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct LessonAuthorDraft {
    node_id: String,
    title: String,
    grade_level: String,
    bloom_level: String,
    learning_objective: String,
    explanation: String,
    example_problem: String,
    example_step_one: String,
    example_step_one_result: String,
    example_step_two: String,
    example_step_two_result: String,
    example_answer: String,
    visual_description: String,
    general_hint_one: String,
    general_hint_two: String,
    vocabulary_rows: String,
    misconception_rows: String,
    practice: [PracticeDraft; 3],
    assessment_question: String,
    assessment_answer: String,
    assessment_rubric: String,
    assessment_hint: String,
    assessment_points: String,
    flashcard_front: String,
    flashcard_back: String,
    author_credit: String,
    license: String,
    source_notes: String,
}

impl Default for LessonAuthorDraft {
    fn default() -> Self {
        Self {
            node_id: "EXAMPLE.MATH.GR3.EQUAL_GROUPS".into(),
            title: "Multiplication as equal groups".into(),
            grade_level: "Grade3".into(),
            bloom_level: "Understand".into(),
            learning_objective: "Represent equal groups as multiplication and explain what each factor means.".into(),
            explanation: "Multiplication can describe equal groups. In 4 × 3, the first number tells us there are 4 groups and the second number tells us each group contains 3 objects. We can draw four circles and put three counters in each circle. Counting all the counters gives 12, so 4 × 3 = 12. Repeated addition shows the same idea: 3 + 3 + 3 + 3 = 12. Keep the two meanings separate: 4 groups of 3 is not the same as 4 + 3. Drawing or arranging objects is a useful way to check the meaning before memorising a multiplication fact.".into(),
            example_problem: "There are 4 bags with 3 oranges in each bag. How many oranges are there altogether?".into(),
            example_step_one: "Identify the number of equal groups.".into(),
            example_step_one_result: "There are 4 bags, so there are 4 groups.".into(),
            example_step_two: "Identify the number in each group and calculate the total.".into(),
            example_step_two_result: "Each bag has 3 oranges, so 3 + 3 + 3 + 3 = 4 × 3 = 12.".into(),
            example_answer: "12 oranges".into(),
            visual_description: "Draw four separate bags; put three orange circles in each bag.".into(),
            general_hint_one: "Draw one box or circle for each group.".into(),
            general_hint_two: "Put the same number of objects in every group, then count all the objects.".into(),
            vocabulary_rows: "equal groups | Groups that contain the same number of objects. | Four plates with three apples on each plate are equal groups.\nfactor | A number multiplied by another number to make a product. | In 4 × 3, the numbers 4 and 3 are factors.\nproduct | The result of a multiplication. | The product of 4 and 3 is 12.".into(),
            misconception_rows: "4 × 3 means 4 + 3. | 4 × 3 represents four groups of three: 3 + 3 + 3 + 3 = 12. | The same two numbers appear in both operations, but the operations represent different relationships.".into(),
            practice: [
                PracticeDraft {
                    question: "There are 3 plates with 5 biscuits on each plate. How many biscuits are there?".into(),
                    answer: "15 biscuits".into(),
                    explanation: "There are 3 equal groups of 5 biscuits: 5 + 5 + 5 = 15.".into(),
                    hint: "Draw three groups and place five dots in each group.".into(),
                    distractors: "8 biscuits\n10 biscuits\n20 biscuits".into(),
                },
                PracticeDraft {
                    question: "Write repeated addition for 4 groups of 2 and find the total.".into(),
                    answer: "2 + 2 + 2 + 2 = 8".into(),
                    explanation: "Four groups contain four copies of 2, so 2 + 2 + 2 + 2 = 8.".into(),
                    hint: "Write one 2 for each group, then add the four numbers.".into(),
                    distractors: "2 + 4 = 6\n4 + 2 = 6\n2 + 2 + 2 = 6".into(),
                },
                PracticeDraft {
                    question: "A learner says 6 groups of 4 means 6 + 4 = 10. Explain the error and give the correct total.".into(),
                    answer: "Six groups of four contain 24 objects: 4 + 4 + 4 + 4 + 4 + 4 = 24.".into(),
                    explanation: "The learner added the number of groups to the group size instead of counting four objects six times. Six groups of four contain 24 objects.".into(),
                    hint: "Separate the number of groups from the number of objects in each group.".into(),
                    distractors: "10 because the two numbers are added\n20 because there are six groups of four\n28 because 6 + 4 is nearly 12".into(),
                },
            ],
            assessment_question: "Describe what 4 × 3 means in an equal-groups model.".into(),
            assessment_answer: "Four groups with three objects in each group.".into(),
            assessment_rubric: "Award the point for identifying both four groups and three objects in each group.".into(),
            assessment_hint: "State how many groups there are and how many objects belong in each group.".into(),
            assessment_points: "2".into(),
            flashcard_front: "What is a product?".into(),
            flashcard_back: "The result of a multiplication.".into(),
            author_credit: "Example contributor".into(),
            license: "UNDECIDED — choose a licence before publication.".into(),
            source_notes: "Original example draft; verify all claims and record any sources before publication.".into(),
        }
    }
}

fn parse_rows(text: &str) -> Vec<Vec<String>> {
    text.lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .map(|line| line.split('|').map(|part| part.trim().to_owned()).collect())
        .collect()
}

fn lesson_pack_value(draft: &LessonAuthorDraft) -> Value {
    let practice_bloom = ["Apply", "Apply", "Analyze"];
    let practice_difficulty = [250_u16, 400_u16, 550_u16];
    let practice = draft
        .practice
        .iter()
        .enumerate()
        .map(|(index, item)| {
            let distractors = item
                .distractors
                .lines()
                .map(str::trim)
                .filter(|line| !line.is_empty())
                .map(str::to_owned)
                .collect::<Vec<_>>();
            json!({
                "question": item.question,
                "answer": item.answer,
                "difficulty_permille": practice_difficulty[index],
                "bloom_level": practice_bloom[index],
                "hints": [item.hint],
                "explanation": item.explanation,
                "distractors": distractors
            })
        })
        .collect::<Vec<_>>();
    let vocabulary = parse_rows(&draft.vocabulary_rows)
        .into_iter()
        .filter(|row| row.len() == 3)
        .map(|row| json!({
            "term": row[0],
            "definition": row[1],
            "example_usage": row[2]
        }))
        .collect::<Vec<_>>();
    let misconceptions = parse_rows(&draft.misconception_rows)
        .into_iter()
        .filter(|row| row.len() == 3)
        .map(|row| json!({
            "misconception": row[0],
            "correction": row[1],
            "why_students_think_this": row[2]
        }))
        .collect::<Vec<_>>();
    let points = draft
        .assessment_points
        .trim()
        .parse::<u32>()
        .ok()
        .filter(|value| *value > 0)
        .unwrap_or(1);
    let id = draft.node_id.trim();
    json!({
        "node_id": id,
        "lesson": {
            "standard_code": id,
            "title": draft.title,
            "grade_level": draft.grade_level,
            "bloom_level": draft.bloom_level,
            "explanation": draft.explanation,
            "examples": [{
                "problem": draft.example_problem,
                "steps": [
                    {"instruction": draft.example_step_one, "result": draft.example_step_one_result},
                    {"instruction": draft.example_step_two, "result": draft.example_step_two_result}
                ],
                "answer": draft.example_answer,
                "visual_description": draft.visual_description
            }],
            "practice_problems": practice,
            "hints": [draft.general_hint_one, draft.general_hint_two],
            "key_vocabulary": vocabulary,
            "misconceptions": misconceptions,
            "generation_quality": {
                "coherence": 0.0,
                "hallucination_detected": false,
                "needs_human_review": true,
                "veto_count": 0,
                "review_status": "draft_requires_educator_review",
                "metrics_valid_for_current_revision": false
            }
        },
        "assessment": {
            "standard_code": id,
            "title": format!("Assessment: {}", draft.title),
            "items": [{
                "question": draft.assessment_question,
                "item_type": "ShortAnswer",
                "correct_answer": draft.assessment_answer,
                "points": points,
                "bloom_level": draft.bloom_level,
                "rubric": draft.assessment_rubric,
                "hint": draft.assessment_hint
            }],
            "total_points": points,
            "estimated_minutes": 10,
            "bloom_distribution": [[draft.bloom_level, 1]]
        },
        "flashcards": [{
            "front": draft.flashcard_front,
            "back": draft.flashcard_back,
            "tags": [id, draft.grade_level, "Community authored"],
            "difficulty_permille": 250
        }],
        "quality": {
            "coherence": 0.0,
            "hallucination_detected": false,
            "needs_human_review": true,
            "veto_count": 0,
            "review_status": "draft_requires_educator_review",
            "metrics_valid_for_current_revision": false,
            "review_notes": "Local authoring draft. Subject accuracy, alignment, accessibility, rights and assessment keys require review."
        },
        "authoring": {
            "format_version": 1,
            "status": "draft",
            "learning_objective": draft.learning_objective,
            "author_credit": draft.author_credit,
            "license": draft.license,
            "source_and_attribution_notes": draft.source_notes.lines().map(str::trim).filter(|line| !line.is_empty()).collect::<Vec<_>>()
        }
    })
}

fn lesson_filename(draft: &LessonAuthorDraft) -> String {
    let slug = draft
        .node_id
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || character == '-' {
                character
            } else {
                '-'
            }
        })
        .collect::<String>()
        .trim_matches('-')
        .to_lowercase();
    if slug.is_empty() {
        "praxis-lesson-draft.json".to_owned()
    } else {
        format!("praxis-{slug}.json")
    }
}

fn lesson_pack_json(draft: &LessonAuthorDraft) -> String {
    serde_json::to_string_pretty(&lesson_pack_value(draft))
        .unwrap_or_else(|error| format!("{{\"error\":\"could not serialize draft: {error}\"}}"))
}

fn validate_draft(draft: &LessonAuthorDraft) -> Vec<String> {
    let mut findings = Vec::new();
    let required = [
        ("Objective ID", draft.node_id.trim()),
        ("Lesson title", draft.title.trim()),
        ("Grade or course level", draft.grade_level.trim()),
        ("Learning objective", draft.learning_objective.trim()),
        ("Explanation", draft.explanation.trim()),
        ("Worked example problem", draft.example_problem.trim()),
        ("Worked example answer", draft.example_answer.trim()),
        ("Assessment question", draft.assessment_question.trim()),
        ("Assessment answer", draft.assessment_answer.trim()),
        ("Flashcard front", draft.flashcard_front.trim()),
        ("Flashcard back", draft.flashcard_back.trim()),
    ];
    for (label, value) in required {
        if value.is_empty() {
            findings.push(format!("{label} is required."));
        }
    }
    if draft.explanation.trim().chars().count() < 80 {
        findings.push("Expand the explanation to at least 80 characters so it can teach the idea rather than only name it.".into());
    }
    if !matches!(
        draft.bloom_level.as_str(),
        "Remember" | "Understand" | "Apply" | "Analyze" | "Evaluate" | "Create"
    ) {
        findings.push("Choose a Bloom level: Remember, Understand, Apply, Analyze, Evaluate or Create.".into());
    }
    if draft.assessment_points.trim().parse::<u32>().ok().filter(|points| *points > 0).is_none() {
        findings.push("Assessment points must be a positive whole number.".into());
    }
    for (index, item) in draft.practice.iter().enumerate() {
        for (label, value) in [
            ("question", item.question.trim()),
            ("answer", item.answer.trim()),
            ("explanation", item.explanation.trim()),
            ("hint", item.hint.trim()),
        ] {
            if value.is_empty() {
                findings.push(format!("Practice item {} needs a {label}.", index + 1));
            }
        }
        let choices = item
            .distractors
            .lines()
            .map(str::trim)
            .filter(|choice| !choice.is_empty())
            .collect::<Vec<_>>();
        if choices.len() != 3 {
            findings.push(format!("Practice item {} must have exactly three wrong-answer choices, one per line.", index + 1));
        }
        let normalized = choices.iter().map(|choice| choice.to_lowercase()).collect::<Vec<_>>();
        if normalized.iter().collect::<std::collections::BTreeSet<_>>().len() != normalized.len() {
            findings.push(format!("Practice item {} has duplicate choices.", index + 1));
        }
        let answer = item.answer.trim().to_lowercase().trim_end_matches(['.', '!', '?']).to_owned();
        if normalized.iter().any(|choice| choice.trim_end_matches(['.', '!', '?']) == answer.as_str()) {
            findings.push(format!("Practice item {} repeats the answer among the wrong-answer choices.", index + 1));
        }
    }
    for (label, rows) in [
        ("Key vocabulary", draft.vocabulary_rows.as_str()),
        ("Misconceptions", draft.misconception_rows.as_str()),
    ] {
        for (index, row) in parse_rows(rows).iter().enumerate() {
            if row.len() != 3 || row.iter().any(String::is_empty) {
                findings.push(format!("{label} row {} must contain three non-empty fields separated by |.", index + 1));
            }
        }
    }
    findings
}

#[component]
pub fn LessonAuthorPage() -> impl IntoView {
    let initial = persistence::load::<LessonAuthorDraft>(DRAFT_KEY).unwrap_or_default();
    let (draft, set_draft) = signal(initial);
    let (notice, set_notice) = signal("Drafts are saved only in this browser. Nothing is published or uploaded.".to_owned());

    Effect::new(move |_| {
        persistence::save(DRAFT_KEY, &draft.get());
    });

    view! {
        <section style="max-width: 1120px; margin: 0 auto; padding: 1.25rem; color: var(--text)">
            <header style="margin-bottom: 1.25rem">
                <p style="color: var(--accent); font-weight: 700; letter-spacing: 0.08em; text-transform: uppercase; font-size: 0.75rem">"Community authoring"</p>
                <h1 style="margin-bottom: 0.5rem">"Create a lesson for someone else"</h1>
                <p style="max-width: 760px; color: var(--text-secondary)">"Share a skill you know. This form builds a reusable lesson pack, saves your draft locally, and lets you export a JSON file for review. You do not need to write JSON by hand."</p>
            </header>

            <div style="display: flex; gap: 0.75rem; flex-wrap: wrap; margin-bottom: 1rem">
                <button class="btn-primary" on:click=move |_| {
                    let current = draft.get();
                    let json = lesson_pack_json(&current);
                    let filename = lesson_filename(&current);
                    persistence::trigger_download(&filename, &json);
                    set_notice.set("Lesson JSON exported. Run the standalone checker and add source/rights evidence before submitting it for review.".into());
                }>"Download lesson JSON"</button>
                <button class="btn-outline" on:click=move |_| {
                    let findings = validate_draft(&draft.get());
                    if findings.is_empty() {
                        set_notice.set("Quick checks passed. This is still a draft: subject review, rights review and the full Rust checker are required.".into());
                    } else {
                        set_notice.set(format!("{} quick-check finding(s): {}", findings.len(), findings.join(" ")));
                    }
                }>"Check my draft"</button>
                <button class="btn-outline" on:click=move |_| {
                    set_draft.set(LessonAuthorDraft::default());
                    set_notice.set("Restored the sample lesson in this browser.".into());
                }>"Reset to example"</button>
            </div>
            <p role="status" aria-live="polite" style="padding: 0.75rem; border: 1px solid var(--border); border-radius: 8px; color: var(--text-secondary); margin-bottom: 1rem">{move || notice.get()}</p>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"1. What are you teaching?"</h2>
                <p style="color: var(--text-secondary)">"Use an existing objective ID if you know it. Find likely objectives in Knowledge Garden / Skill Map; do not invent an official code. Drafting is separate from curriculum activation."</p>
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 0.85rem">
                    <label>"Objective ID"
                        <input style="width: 100%" prop:value=move || draft.get().node_id.clone() on:input=move |ev| set_draft.update(|d| d.node_id = event_target_value(&ev)) />
                    </label>
                    <label>"Lesson title"
                        <input style="width: 100%" prop:value=move || draft.get().title.clone() on:input=move |ev| set_draft.update(|d| d.title = event_target_value(&ev)) />
                    </label>
                    <label>"Learner level (grade or course)"
                        <input style="width: 100%" prop:value=move || draft.get().grade_level.clone() on:input=move |ev| set_draft.update(|d| d.grade_level = event_target_value(&ev)) />
                    </label>
                    <label>"Bloom level"
                        <select style="width: 100%" prop:value=move || draft.get().bloom_level.clone() on:change=move |ev| set_draft.update(|d| d.bloom_level = event_target_value(&ev))>
                            <option value="Remember">"Remember"</option>
                            <option value="Understand">"Understand"</option>
                            <option value="Apply">"Apply"</option>
                            <option value="Analyze">"Analyze"</option>
                            <option value="Evaluate">"Evaluate"</option>
                            <option value="Create">"Create"</option>
                        </select>
                    </label>
                </div>
                <label style="display: block; margin-top: 0.85rem">"Learning objective — what should a learner be able to do?"
                    <textarea style="width: 100%; min-height: 76px" prop:value=move || draft.get().learning_objective.clone() on:input=move |ev| set_draft.update(|d| d.learning_objective = event_target_value(&ev)) />
                </label>
            </section>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"2. Explain and demonstrate"</h2>
                <label style="display: block">"Explanation (teach the idea, not just the rule)"
                    <textarea style="width: 100%; min-height: 160px" prop:value=move || draft.get().explanation.clone() on:input=move |ev| set_draft.update(|d| d.explanation = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.85rem">"Worked-example question"
                    <textarea style="width: 100%; min-height: 66px" prop:value=move || draft.get().example_problem.clone() on:input=move |ev| set_draft.update(|d| d.example_problem = event_target_value(&ev)) />
                </label>
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 0.85rem; margin-top: 0.85rem">
                    <label>"Step 1 — what to do"
                        <textarea style="width: 100%; min-height: 70px" prop:value=move || draft.get().example_step_one.clone() on:input=move |ev| set_draft.update(|d| d.example_step_one = event_target_value(&ev)) />
                    </label>
                    <label>"Step 1 — result"
                        <textarea style="width: 100%; min-height: 70px" prop:value=move || draft.get().example_step_one_result.clone() on:input=move |ev| set_draft.update(|d| d.example_step_one_result = event_target_value(&ev)) />
                    </label>
                    <label>"Step 2 — what to do"
                        <textarea style="width: 100%; min-height: 70px" prop:value=move || draft.get().example_step_two.clone() on:input=move |ev| set_draft.update(|d| d.example_step_two = event_target_value(&ev)) />
                    </label>
                    <label>"Step 2 — result"
                        <textarea style="width: 100%; min-height: 70px" prop:value=move || draft.get().example_step_two_result.clone() on:input=move |ev| set_draft.update(|d| d.example_step_two_result = event_target_value(&ev)) />
                    </label>
                </div>
                <label style="display: block; margin-top: 0.85rem">"Worked-example answer"
                    <input style="width: 100%" prop:value=move || draft.get().example_answer.clone() on:input=move |ev| set_draft.update(|d| d.example_answer = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.85rem">"Visual description / text alternative (optional)"
                    <textarea style="width: 100%; min-height: 60px" prop:value=move || draft.get().visual_description.clone() on:input=move |ev| set_draft.update(|d| d.visual_description = event_target_value(&ev)) />
                </label>
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 0.85rem; margin-top: 0.85rem">
                    <label>"General hint 1 (gentle)"
                        <textarea style="width: 100%; min-height: 60px" prop:value=move || draft.get().general_hint_one.clone() on:input=move |ev| set_draft.update(|d| d.general_hint_one = event_target_value(&ev)) />
                    </label>
                    <label>"General hint 2 (more specific)"
                        <textarea style="width: 100%; min-height: 60px" prop:value=move || draft.get().general_hint_two.clone() on:input=move |ev| set_draft.update(|d| d.general_hint_two = event_target_value(&ev)) />
                    </label>
                </div>
            </section>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"3. Create independent practice"</h2>
                <p style="color: var(--text-secondary)">"Use three different questions. Write exactly three wrong answers, one per line, for each item. Wrong answers must truly be wrong, not a reworded or unit-less version of the correct answer."</p>
                <PracticeItemEditor index=0 title="Practice 1 · Apply" draft=draft set_draft=set_draft />
                <PracticeItemEditor index=1 title="Practice 2 · Apply" draft=draft set_draft=set_draft />
                <PracticeItemEditor index=2 title="Practice 3 · Diagnose a mistake" draft=draft set_draft=set_draft />
            </section>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"4. Assess understanding and support recall"</h2>
                <label style="display: block">"Assessment question"
                    <textarea style="width: 100%; min-height: 70px" prop:value=move || draft.get().assessment_question.clone() on:input=move |ev| set_draft.update(|d| d.assessment_question = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.75rem">"Expected answer / answer key"
                    <textarea style="width: 100%; min-height: 70px" prop:value=move || draft.get().assessment_answer.clone() on:input=move |ev| set_draft.update(|d| d.assessment_answer = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.75rem">"Scoring rubric (especially when multiple answers are valid)"
                    <textarea style="width: 100%; min-height: 60px" prop:value=move || draft.get().assessment_rubric.clone() on:input=move |ev| set_draft.update(|d| d.assessment_rubric = event_target_value(&ev)) />
                </label>
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 0.85rem; margin-top: 0.75rem">
                    <label>"Assessment points"
                        <input type="number" min="1" step="1" style="width: 100%" prop:value=move || draft.get().assessment_points.clone() on:input=move |ev| set_draft.update(|d| d.assessment_points = event_target_value(&ev)) />
                    </label>
                    <label>"Assessment hint"
                        <input style="width: 100%" prop:value=move || draft.get().assessment_hint.clone() on:input=move |ev| set_draft.update(|d| d.assessment_hint = event_target_value(&ev)) />
                    </label>
                </div>
                <label style="display: block; margin-top: 0.75rem">"Flashcard front"
                    <input style="width: 100%" prop:value=move || draft.get().flashcard_front.clone() on:input=move |ev| set_draft.update(|d| d.flashcard_front = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.75rem">"Flashcard back"
                    <textarea style="width: 100%; min-height: 60px" prop:value=move || draft.get().flashcard_back.clone() on:input=move |ev| set_draft.update(|d| d.flashcard_back = event_target_value(&ev)) />
                </label>
            </section>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"5. Vocabulary and misconceptions (optional but useful)"</h2>
                <p style="color: var(--text-secondary)">"One row per line; separate the three fields with a vertical bar |."</p>
                <label style="display: block">"Vocabulary: term | definition | example usage"
                    <textarea style="width: 100%; min-height: 100px" prop:value=move || draft.get().vocabulary_rows.clone() on:input=move |ev| set_draft.update(|d| d.vocabulary_rows = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.75rem">"Misconception: wrong idea | correction | why learners may think it"
                    <textarea style="width: 100%; min-height: 100px" prop:value=move || draft.get().misconception_rows.clone() on:input=move |ev| set_draft.update(|d| d.misconception_rows = event_target_value(&ev)) />
                </label>
            </section>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"6. Credit, reuse rights and sources"</h2>
                <label style="display: block">"Public author credit or pseudonym"
                    <input style="width: 100%" prop:value=move || draft.get().author_credit.clone() on:input=move |ev| set_draft.update(|d| d.author_credit = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.75rem">"Intended reuse licence / rights status"
                    <input style="width: 100%" prop:value=move || draft.get().license.clone() on:input=move |ev| set_draft.update(|d| d.license = event_target_value(&ev)) />
                </label>
                <label style="display: block; margin-top: 0.75rem">"Sources and attribution notes (one per line)"
                    <textarea style="width: 100%; min-height: 90px" prop:value=move || draft.get().source_notes.clone() on:input=move |ev| set_draft.update(|d| d.source_notes = event_target_value(&ev)) />
                </label>
                <p style="color: var(--text-secondary)">"Do not upload copyrighted textbook pages, private student work, or materials you cannot redistribute. A source link is not permission. Publication still requires the lesson evidence sidecar and reviewer approval."</p>
            </section>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"Export preview"</h2>
                <p style="color: var(--text-secondary)">"This is the JSON that will be downloaded. You can inspect it, but you do not need to edit it by hand."</p>
                <textarea readonly=true aria-label="Generated lesson JSON preview" rows="18" style="width: 100%; font-family: monospace; font-size: 0.8rem" prop:value=move || lesson_pack_json(&draft.get())></textarea>
                <button class="btn-primary" style="margin-top: 0.75rem" on:click=move |_| {
                    let current = draft.get();
                    let json = lesson_pack_json(&current);
                    let filename = lesson_filename(&current);
                    persistence::trigger_download(&filename, &json);
                    set_notice.set("Exported a draft with human review still required.".into());
                }>"Download JSON for review"</button>
            </section>
            <p style="color: var(--text-tertiary); font-size: 0.85rem">"Local-first: drafts stay in this browser. Exporting creates a file; it does not publish, issue credentials, activate a curriculum node, or prove learning effectiveness."</p>
        </section>
    }
}

#[component]
fn PracticeItemEditor(
    index: usize,
    title: &'static str,
    draft: ReadSignal<LessonAuthorDraft>,
    set_draft: WriteSignal<LessonAuthorDraft>,
) -> impl IntoView {
    view! {
        <fieldset style="border: 1px solid var(--border); padding: 0.85rem; border-radius: 10px; margin: 0.75rem 0">
            <legend style="font-weight: 700">{title}</legend>
            <label style="display: block">"Question"
                <textarea style="width: 100%; min-height: 64px" prop:value=move || draft.get().practice[index].question.clone() on:input=move |ev| set_draft.update(|d| d.practice[index].question = event_target_value(&ev)) />
            </label>
            <label style="display: block; margin-top: 0.65rem">"Correct answer"
                <input style="width: 100%" prop:value=move || draft.get().practice[index].answer.clone() on:input=move |ev| set_draft.update(|d| d.practice[index].answer = event_target_value(&ev)) />
            </label>
            <label style="display: block; margin-top: 0.65rem">"Why that answer is correct"
                <textarea style="width: 100%; min-height: 60px" prop:value=move || draft.get().practice[index].explanation.clone() on:input=move |ev| set_draft.update(|d| d.practice[index].explanation = event_target_value(&ev)) />
            </label>
            <label style="display: block; margin-top: 0.65rem">"Helpful hint"
                <textarea style="width: 100%; min-height: 54px" prop:value=move || draft.get().practice[index].hint.clone() on:input=move |ev| set_draft.update(|d| d.practice[index].hint = event_target_value(&ev)) />
            </label>
            <label style="display: block; margin-top: 0.65rem">"Three wrong-answer choices (one per line)"
                <textarea style="width: 100%; min-height: 75px" prop:value=move || draft.get().practice[index].distractors.clone() on:input=move |ev| set_draft.update(|d| d.practice[index].distractors = event_target_value(&ev)) />
            </label>
        </fieldset>
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_lesson_exports_a_complete_review_required_pack() {
        let draft = LessonAuthorDraft::default();
        let value = lesson_pack_value(&draft);
        assert_eq!(value["node_id"], "EXAMPLE.MATH.GR3.EQUAL_GROUPS");
        assert_eq!(value["lesson"]["examples"].as_array().map(Vec::len), Some(1));
        assert_eq!(value["lesson"]["practice_problems"].as_array().map(Vec::len), Some(3));
        assert_eq!(value["flashcards"].as_array().map(Vec::len), Some(1));
        assert_eq!(value["assessment"]["total_points"], 2);
        assert_eq!(value["quality"]["needs_human_review"], true);
        assert_eq!(value["quality"]["metrics_valid_for_current_revision"], false);
        assert_eq!(value["authoring"]["status"], "draft");
        assert!(validate_draft(&draft).is_empty());
    }

    #[test]
    fn quick_checks_reject_answer_collisions_and_incomplete_delimited_rows() {
        let mut draft = LessonAuthorDraft::default();
        draft.practice[0].distractors = "15 biscuits\n15 biscuits\n20 biscuits".into();
        draft.vocabulary_rows = "product | definition only".into();
        let findings = validate_draft(&draft);
        assert!(findings.iter().any(|finding| finding.contains("duplicate choices")));
        assert!(findings.iter().any(|finding| finding.contains("repeats the answer")));
        assert!(findings.iter().any(|finding| finding.contains("Key vocabulary row 1")));
    }

    #[test]
    fn downloaded_filename_is_safe_and_has_a_fallback() {
        let mut draft = LessonAuthorDraft::default();
        assert_eq!(lesson_filename(&draft), "praxis-example-math-gr3-equal-groups.json");
        draft.node_id = "///".into();
        assert_eq!(lesson_filename(&draft), "praxis-lesson-draft.json");
    }
}
