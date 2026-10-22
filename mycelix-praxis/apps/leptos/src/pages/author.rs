// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Local-first, form-based lesson authoring. Drafts stay in browser localStorage
//! until a contributor explicitly exports the JSON for review.

use leptos::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use wasm_bindgen::{JsCast, closure::Closure};

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

impl PracticeDraft {
    fn blank() -> Self {
        Self {
            question: String::new(),
            answer: String::new(),
            explanation: String::new(),
            hint: String::new(),
            distractors: String::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct LessonAuthorDraft {
    node_id: String,
    title: String,
    grade_level: String,
    language: String,
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
    /// Original imported content so remixing preserves material this form does not edit.
    #[serde(default)]
    source_pack: Option<Value>,
}

impl Default for LessonAuthorDraft {
    fn default() -> Self {
        Self {
            node_id: "EXAMPLE.MATH.GR3.EQUAL_GROUPS".into(),
            title: "Multiplication as equal groups".into(),
            grade_level: "Grade3".into(),
            language: "en-ZA".into(),
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
            source_pack: None,
        }
    }
}

fn blank_lesson_draft() -> LessonAuthorDraft {
    let mut draft = LessonAuthorDraft::default();
    draft.node_id.clear();
    draft.title.clear();
    draft.grade_level.clear();
    draft.learning_objective.clear();
    draft.explanation.clear();
    draft.example_problem.clear();
    draft.example_step_one.clear();
    draft.example_step_one_result.clear();
    draft.example_step_two.clear();
    draft.example_step_two_result.clear();
    draft.example_answer.clear();
    draft.visual_description.clear();
    draft.general_hint_one.clear();
    draft.general_hint_two.clear();
    draft.vocabulary_rows.clear();
    draft.misconception_rows.clear();
    draft.practice = std::array::from_fn(|_| PracticeDraft::blank());
    draft.assessment_question.clear();
    draft.assessment_answer.clear();
    draft.assessment_rubric.clear();
    draft.assessment_hint.clear();
    draft.flashcard_front.clear();
    draft.flashcard_back.clear();
    draft.author_credit.clear();
    draft.license = "UNDECIDED — confirm you have rights before publication.".into();
    draft.source_notes.clear();
    draft.source_pack = None;
    draft
}

fn hands_on_activity_draft() -> LessonAuthorDraft {
    let mut draft = LessonAuthorDraft::default();
    draft.node_id = "COMMUNITY.STEM.PAPER_BRIDGE".into();
    draft.title = "Design and test a paper bridge".into();
    draft.grade_level = "Grade6".into();
    draft.language = "en-ZA".into();
    draft.bloom_level = "Evaluate".into();
    draft.learning_objective = "Design a paper bridge across a fixed gap, test it with equal loads, record the result, and justify one evidence-based design change.".into();
    draft.explanation = "Engineers solve problems by making a design, testing it, and improving it using evidence. For a fair paper-bridge test, keep the paper type, bridge span, supports, and loading method the same. Change one design feature at a time, such as folding the paper into a beam, so you can reason about what caused a difference. Put identical coins on the bridge one at a time, stop when it bends too far, and record the load. Do not compare tests that used different spans or different kinds of load. A stronger design is one that performs better under the same test conditions, not simply one that looks stronger.".into();
    draft.example_problem = "Two paper bridges span the same 15 cm gap. How can you test whether folding the paper improves its strength?".into();
    draft.example_step_one = "Choose the one design feature to compare and keep other conditions the same.".into();
    draft.example_step_one_result = "Use the same paper, 15 cm span, supports, and identical coins; change only whether the paper is flat or folded.".into();
    draft.example_step_two = "Add equal loads gradually and record the load at which each bridge bends.".into();
    draft.example_step_two_result = "Test each design the same way, record the number of coins, and compare the results.".into();
    draft.example_answer = "Compare the number of identical coins each bridge supports before bending, while keeping the span and materials the same.".into();
    draft.visual_description = "Side view of two paper bridges between identical supports over a labelled 15 cm gap; one bridge is flat and the other folded.".into();
    draft.general_hint_one = "Which conditions must stay the same for the comparison to be fair?".into();
    draft.general_hint_two = "Change one design feature, add identical coins one at a time, and record the load for each design.".into();
    draft.practice = [
        PracticeDraft {
            question: "Which plan is the fairest test of whether a fold strengthens a paper bridge?".into(),
            answer: "Use the same paper and span, change only the fold, and add identical coins.".into(),
            explanation: "Keeping the materials, span, and loading method the same lets the fold be the main changed variable.".into(),
            hint: "Find the plan that changes one design feature and keeps the other conditions constant.".into(),
            distractors: "Use thicker paper only for the folded bridge.\nGive one bridge a shorter span.\nPut large coins on one bridge and small coins on the other.".into(),
        },
        PracticeDraft {
            question: "Bridge A supports 12 identical coins and Bridge B supports 8 before bending. How many more coins did A support?".into(),
            answer: "4 more coins".into(),
            explanation: "Subtract the smaller count from the larger count: 12 − 8 = 4.".into(),
            hint: "Compare the two counts by subtracting 8 from 12.".into(),
            distractors: "4 fewer coins.\n20 more coins.\nThe same number of coins.".into(),
        },
        PracticeDraft {
            question: "A bridge bends after five coins. What is the best next step in an engineering investigation?".into(),
            answer: "Record the result, change one design feature, and repeat the same test.".into(),
            explanation: "A recorded result supports comparison; changing one feature at a time helps explain why performance changed.".into(),
            hint: "Keep the evidence, make one controlled change, and test under the same conditions.".into(),
            distractors: "Change the paper, span, and coin sizes together.\nIgnore the result and claim the design always fails.\nUse a different gap and compare the counts directly.".into(),
        },
    ];
    draft.assessment_question = "Explain how you would fairly compare a flat-paper bridge with a folded-paper bridge.".into();
    draft.assessment_answer = "Use the same paper, supports, span, and identical coins; change only the fold; repeat the loading procedure; and record the load at which each bridge bends.".into();
    draft.assessment_rubric = "Award credit for identifying the changed variable, at least two controlled conditions, a consistent loading procedure, and recording/comparing results.".into();
    draft.assessment_hint = "Name what you change, what you keep the same, and what you measure.".into();
    draft.assessment_points = "4".into();
    draft.flashcard_front = "What makes an engineering test fair?".into();
    draft.flashcard_back = "Change one main feature at a time, keep the other conditions consistent, and measure each design the same way.".into();
    draft.vocabulary_rows = "variable | A factor that can change in an investigation. | The fold shape is the variable being tested.\nspan | The distance a bridge must cross. | Both bridges cross the same 15 cm span.\nload | The force or weight placed on a structure. | Add identical coins as a repeatable load.".into();
    draft.misconception_rows = "A folded bridge is always stronger, so there is no need to test it. | Test both designs under the same conditions and compare recorded results. | Familiar designs can behave differently depending on materials and span.\nChange several things at once to make the bridge stronger. | Change one feature at a time when investigating which change helped. | Multiple changes make the cause of a result difficult to identify.".into();
    draft.author_credit = "Community contributor".into();
    draft.license = "UNDECIDED — choose a reuse licence only after confirming rights.".into();
    draft.source_notes = "Starter template content; contributor must verify subject accuracy, add any sources used, and confirm redistribution rights.".into();
    draft.source_pack = None;
    draft
}

fn parse_rows(text: &str) -> Vec<Vec<String>> {
    text.lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .map(|line| line.split('|').map(|part| part.trim().to_owned()).collect())
        .collect()
}

fn value_text(value: Option<&Value>, fallback: &str) -> String {
    value.and_then(Value::as_str)
        .map(str::to_owned)
        .unwrap_or_else(|| fallback.to_owned())
}

fn pack_to_draft(value: &Value) -> Result<LessonAuthorDraft, String> {
    let Some(lesson) = value.get("lesson").filter(|value| value.is_object()) else {
        return Err("That file is not a Praxis lesson pack: the lesson object is missing.".into());
    };
    let mut draft = LessonAuthorDraft::default();
    // Never mix sample content into an imported pack. Missing source fields remain
    // blank so contributors can see and fill the gaps before export.
    draft.node_id.clear();
    draft.title.clear();
    draft.grade_level.clear();
    draft.language.clear();
    draft.bloom_level.clear();
    draft.learning_objective.clear();
    draft.explanation.clear();
    draft.example_problem.clear();
    draft.example_step_one.clear();
    draft.example_step_one_result.clear();
    draft.example_step_two.clear();
    draft.example_step_two_result.clear();
    draft.example_answer.clear();
    draft.visual_description.clear();
    draft.general_hint_one.clear();
    draft.general_hint_two.clear();
    draft.vocabulary_rows.clear();
    draft.misconception_rows.clear();
    draft.practice = std::array::from_fn(|_| PracticeDraft::blank());
    draft.assessment_question.clear();
    draft.assessment_answer.clear();
    draft.assessment_rubric.clear();
    draft.assessment_hint.clear();
    draft.assessment_points.clear();
    draft.flashcard_front.clear();
    draft.flashcard_back.clear();
    draft.author_credit.clear();
    draft.license = "UNDECIDED — confirm you have rights before publication.".into();
    draft.source_notes.clear();
    draft.source_pack = Some(value.clone());
    draft.node_id = value_text(
        value.get("node_id").or_else(|| lesson.get("standard_code")),
        &draft.node_id,
    );
    draft.title = value_text(lesson.get("title"), &draft.title);
    draft.grade_level = value_text(lesson.get("grade_level"), &draft.grade_level);
    draft.bloom_level = value_text(lesson.get("bloom_level"), &draft.bloom_level);
    draft.explanation = value_text(lesson.get("explanation"), &draft.explanation);

    if let Some(example) = lesson
        .get("examples")
        .and_then(Value::as_array)
        .and_then(|items| items.first())
    {
        draft.example_problem = value_text(example.get("problem"), &draft.example_problem);
        draft.example_answer = value_text(example.get("answer"), &draft.example_answer);
        draft.visual_description =
            value_text(example.get("visual_description"), &draft.visual_description);
        if let Some(steps) = example.get("steps").and_then(Value::as_array) {
            if let Some(step) = steps.first() {
                draft.example_step_one =
                    value_text(step.get("instruction"), &draft.example_step_one);
                draft.example_step_one_result =
                    value_text(step.get("result"), &draft.example_step_one_result);
            }
            if let Some(step) = steps.get(1) {
                draft.example_step_two =
                    value_text(step.get("instruction"), &draft.example_step_two);
                draft.example_step_two_result =
                    value_text(step.get("result"), &draft.example_step_two_result);
            }
        }
    }

    if let Some(hints) = lesson.get("hints").and_then(Value::as_array) {
        if let Some(hint) = hints.first().and_then(Value::as_str) {
            draft.general_hint_one = hint.to_owned();
        }
        if let Some(hint) = hints.get(1).and_then(Value::as_str) {
            draft.general_hint_two = hint.to_owned();
        }
    }

    if let Some(items) = lesson.get("practice_problems").and_then(Value::as_array) {
        for (index, target) in draft.practice.iter_mut().enumerate() {
            let Some(item) = items.get(index) else {
                continue;
            };
            target.question = value_text(item.get("question"), &target.question);
            target.answer = value_text(item.get("answer"), &target.answer);
            target.explanation = value_text(item.get("explanation"), &target.explanation);
            target.hint = item
                .get("hints")
                .and_then(Value::as_array)
                .and_then(|hints| hints.first())
                .and_then(Value::as_str)
                .unwrap_or(&target.hint)
                .to_owned();
            if let Some(choices) = item.get("distractors").and_then(Value::as_array) {
                target.distractors = choices
                    .iter()
                    .filter_map(Value::as_str)
                    .map(str::to_owned)
                    .collect::<Vec<_>>()
                    .join("\n");
            }
        }
    }

    if let Some(items) = lesson.get("key_vocabulary").and_then(Value::as_array) {
        draft.vocabulary_rows = items
            .iter()
            .map(|item| format!(
                "{} | {} | {}",
                value_text(item.get("term"), ""),
                value_text(item.get("definition"), ""),
                value_text(item.get("example_usage"), "")
            ))
            .collect::<Vec<_>>()
            .join("\n");
    }
    if let Some(items) = lesson.get("misconceptions").and_then(Value::as_array) {
        draft.misconception_rows = items
            .iter()
            .map(|item| format!(
                "{} | {} | {}",
                value_text(item.get("misconception"), ""),
                value_text(item.get("correction"), ""),
                value_text(item.get("why_students_think_this"), "")
            ))
            .collect::<Vec<_>>()
            .join("\n");
    }

    if let Some(item) = value.pointer("/assessment/items")
        .and_then(Value::as_array)
        .and_then(|items| items.first())
    {
        draft.assessment_question = value_text(item.get("question"), &draft.assessment_question);
        draft.assessment_answer = value_text(item.get("correct_answer"), &draft.assessment_answer);
        draft.assessment_rubric = value_text(item.get("rubric"), &draft.assessment_rubric);
        draft.assessment_hint = value_text(item.get("hint"), &draft.assessment_hint);
        if let Some(points) = item.get("points").and_then(Value::as_u64) {
            draft.assessment_points = points.to_string();
        }
    }

    if let Some(card) = value.get("flashcards")
        .and_then(Value::as_array)
        .and_then(|cards| cards.first())
    {
        draft.flashcard_front = value_text(card.get("front"), &draft.flashcard_front);
        draft.flashcard_back = value_text(card.get("back"), &draft.flashcard_back);
    }

    if let Some(authoring) = value.get("authoring") {
        draft.learning_objective = value_text(
            authoring.get("learning_objective"),
            &draft.learning_objective,
        );
        draft.author_credit = value_text(authoring.get("author_credit"), &draft.author_credit);
        draft.license = value_text(authoring.get("license"), &draft.license);
        draft.language = value_text(authoring.get("language"), &draft.language);
        if let Some(notes) = authoring.get("source_and_attribution_notes") {
            draft.source_notes = match notes {
                Value::String(note) => note.clone(),
                Value::Array(items) => items.iter().filter_map(Value::as_str).collect::<Vec<_>>().join("\n"),
                _ => draft.source_notes.clone(),
            };
        }
    }

    draft.source_notes = format!(
        "Imported as a new local draft; verify reuse rights and review the new revision.\n{}",
        draft.source_notes
    );
    Ok(draft)
}

fn preserve_unedited_array_tail(
    edited: &mut Value,
    original: Option<&Value>,
    editable_count: usize,
) {
    let Some(edited_items) = edited.as_array_mut() else {
        return;
    };
    let Some(original_items) = original.and_then(Value::as_array) else {
        return;
    };
    edited_items.extend(original_items.iter().skip(editable_count).cloned());
}

fn merge_object_preserving_unknown(source: Option<&Value>, replacement: &Value) -> Value {
    let mut object = source
        .and_then(Value::as_object)
        .cloned()
        .unwrap_or_default();
    if let Some(replacement) = replacement.as_object() {
        for (key, value) in replacement {
            object.insert(key.clone(), value.clone());
        }
    }
    Value::Object(object)
}

fn merge_imported_pack(original: &Value, mut generated: Value) -> Value {
    let original_lesson = original.get("lesson");
    let mut lesson = generated["lesson"].clone();
    preserve_unedited_array_tail(
        &mut lesson["examples"],
        original_lesson.and_then(|value| value.get("examples")),
        1,
    );
    preserve_unedited_array_tail(
        &mut lesson["practice_problems"],
        original_lesson.and_then(|value| value.get("practice_problems")),
        3,
    );
    preserve_unedited_array_tail(
        &mut lesson["hints"],
        original_lesson.and_then(|value| value.get("hints")),
        2,
    );
    generated["lesson"] = merge_object_preserving_unknown(original_lesson, &lesson);

    let original_assessment = original.get("assessment");
    let mut assessment = generated["assessment"].clone();
    preserve_unedited_array_tail(
        &mut assessment["items"],
        original_assessment.and_then(|value| value.get("items")),
        1,
    );
    let assessment_items = assessment["items"]
        .as_array()
        .cloned()
        .unwrap_or_default();
    let total_points = assessment_items
        .iter()
        .filter_map(|item| item.get("points").and_then(Value::as_u64))
        .sum::<u64>();
    assessment["total_points"] = json!(total_points);
    let mut bloom_counts = std::collections::BTreeMap::<String, u64>::new();
    for item in &assessment_items {
        if let Some(level) = item.get("bloom_level").and_then(Value::as_str) {
            *bloom_counts.entry(level.to_owned()).or_default() += 1;
        }
    }
    assessment["bloom_distribution"] = Value::Array(
        bloom_counts
            .into_iter()
            .map(|(level, count)| json!([level, count]))
            .collect(),
    );
    generated["assessment"] =
        merge_object_preserving_unknown(original_assessment, &assessment);

    let mut flashcards = generated["flashcards"].clone();
    preserve_unedited_array_tail(&mut flashcards, original.get("flashcards"), 1);
    generated["flashcards"] = flashcards;

    let merged_quality =
        merge_object_preserving_unknown(original.get("quality"), &generated["quality"]);
    generated["quality"] = merged_quality;
    let merged_authoring =
        merge_object_preserving_unknown(original.get("authoring"), &generated["authoring"]);
    generated["authoring"] = merged_authoring;

    let mut root = original.as_object().cloned().unwrap_or_default();
    if let Some(replacement) = generated.as_object() {
        for (key, value) in replacement {
            root.insert(key.clone(), value.clone());
        }
    }
    Value::Object(root)
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
    let generated = json!({
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
            "language": draft.language,
            "author_credit": draft.author_credit,
            "license": draft.license,
            "source_and_attribution_notes": draft.source_notes.lines().map(str::trim).filter(|line| !line.is_empty()).collect::<Vec<_>>()
        }
    });
    match draft.source_pack.as_ref() {
        Some(original) => merge_imported_pack(original, generated),
        None => generated,
    }
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

fn author_gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn author_rational_key(numerator: i128, denominator: i128) -> Option<String> {
    if denominator == 0 {
        return None;
    }
    let negative = (numerator < 0) ^ (denominator < 0);
    let numerator_magnitude = numerator.unsigned_abs();
    let denominator_magnitude = denominator.unsigned_abs();
    let divisor = author_gcd(numerator_magnitude, denominator_magnitude);
    let sign = if negative { "-" } else { "" };
    Some(format!(
        "{sign}{}/{}",
        numerator_magnitude / divisor,
        denominator_magnitude / divisor
    ))
}

fn author_numeric_key(text: &str) -> Option<(String, Option<&'static str>)> {
    let normalized = text
        .trim()
        .to_lowercase()
        .trim_end_matches(['.', '!', '?'])
        .trim()
        .to_owned();

    const UNITS: &[(&str, &str)] = &[
        (" square centimetres", "area_cm2"),
        (" square centimeters", "area_cm2"),
        (" square metres", "area_m2"),
        (" square meters", "area_m2"),
        (" square units", "area_unit"),
        (" square unit", "area_unit"),
        (" square cm", "area_cm2"),
        (" square m", "area_m2"),
        (" centimetres", "length_cm"),
        (" centimeters", "length_cm"),
        (" millimetres", "length_mm"),
        (" millimeters", "length_mm"),
        (" kilograms", "mass_kg"),
        (" learners", "count"),
        (" learner", "count"),
        (" children", "count"),
        (" child", "count"),
        (" counters", "count"),
        (" counter", "count"),
        (" objects", "count"),
        (" object", "count"),
        (" students", "count"),
        (" student", "count"),
        (" books", "count"),
        (" book", "count"),
        (" desks", "count"),
        (" desk", "count"),
        (" cm", "length_cm"),
        (" mm", "length_mm"),
        (" m", "length_m"),
        (" kg", "mass_kg"),
    ];

    let mut number_text = normalized.as_str();
    let mut unit = None;
    for (suffix, canonical_unit) in UNITS {
        if let Some(number) = normalized.strip_suffix(*suffix) {
            number_text = number.trim();
            unit = Some(*canonical_unit);
            break;
        }
    }

    let (numerator, denominator) = if let Some((left, right)) = number_text.split_once('/') {
        (left.trim().parse::<i128>().ok()?, right.trim().parse::<i128>().ok()?)
    } else {
        (number_text.parse::<i128>().ok()?, 1)
    };
    Some((author_rational_key(numerator, denominator)?, unit))
}

fn author_numeric_choices_collide(answer: &str, choice: &str) -> bool {
    let Some((answer_value, answer_unit)) = author_numeric_key(answer) else {
        return false;
    };
    let Some((choice_value, choice_unit)) = author_numeric_key(choice) else {
        return false;
    };
    answer_value == choice_value
        && (answer_unit.is_none() || choice_unit.is_none() || answer_unit == choice_unit)
}

fn validate_draft(draft: &LessonAuthorDraft) -> Vec<String> {
    let mut findings = Vec::new();
    let required = [
        ("Objective ID", draft.node_id.trim()),
        ("Lesson title", draft.title.trim()),
        ("Grade or course level", draft.grade_level.trim()),
        ("Language / locale", draft.language.trim()),
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
        } else if choices
            .iter()
            .any(|choice| author_numeric_choices_collide(item.answer.trim(), choice))
        {
            findings.push(format!(
                "Practice item {} may repeat the correct numeric answer with a different unit or equivalent fraction; review the wrong-answer choices.",
                index + 1
            ));
        }
    }
    let license = draft.license.to_lowercase();
    if license.contains("undecided") {
        findings.push(
            "Before publication, choose the actual reuse licence/rights status and record sources in the evidence sidecar."
                .into(),
        );
    } else if license.contains("all rights reserved")
        || license.contains("do not redistribute")
    {
        findings.push(
            "This rights status does not permit public redistribution. Keep the export private or obtain permission/a compatible licence before sharing it as an open lesson."
                .into(),
        );
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
    let mut objective_options = crate::curriculum::curriculum_graph()
        .nodes
        .iter()
        .map(|node| (node.id.clone(), node.title.clone()))
        .collect::<Vec<_>>();
    objective_options.sort_by(|left, right| left.0.cmp(&right.0));

    Effect::new(move |_| {
        persistence::save(DRAFT_KEY, &draft.get());
    });

    view! {
        <section style="max-width: 1120px; margin: 0 auto; padding: 1.25rem; color: var(--text)">
            <header style="margin-bottom: 1.25rem">
                <p style="color: var(--accent); font-weight: 700; letter-spacing: 0.08em; text-transform: uppercase; font-size: 0.75rem">"Community authoring"</p>
                <h1 style="margin-bottom: 0.5rem">"Create a lesson for someone else"</h1>
                <p style="max-width: 760px; color: var(--text-secondary)">"Share a skill you know. This form builds a reusable lesson pack, saves your draft locally, and lets you export a JSON file for review. You do not need to write JSON by hand."</p>
                <p style="margin-top: 0.5rem"><a href="https://github.com/Luminous-Dynamics/mycelix/blob/docs/praxis-k12-college-learning-quality-2026-10-21/mycelix-praxis/docs/community-authoring.md">"Read the community authoring quick-start"</a></p>
            </header>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"Start with a template"</h2>
                <p style="max-width: 760px; color: var(--text-secondary)">"You do not need to start with a blank page. Choose a teaching pattern, replace the sample with what you know, and run the quick checks. Templates are examples—not verified curriculum or approved lessons."</p>
                <div style="display: flex; gap: 0.75rem; flex-wrap: wrap">
                    <button class="btn-primary" on:click=move |_| {
                        set_draft.set(LessonAuthorDraft::default());
                        set_notice.set("Started the worked-example lesson template. Replace the sample with your own teaching and run the checks.".into());
                    }>"Explain a concept"</button>
                    <button class="btn-outline" on:click=move |_| {
                        set_draft.set(hands_on_activity_draft());
                        set_notice.set("Started a hands-on investigation template. Verify the science and adapt the activity to the learner's materials and setting.".into());
                    }>"Build a hands-on activity"</button>
                    <button class="btn-outline" on:click=move |_| {
                        set_draft.set(blank_lesson_draft());
                        set_notice.set("Started a blank draft. Fill in the required fields; your progress stays in this browser.".into());
                    }>"Start from blank"</button>
                </div>
                <p style="margin-top: 0.75rem; color: var(--text-tertiary); font-size: 0.85rem">"Choosing a template replaces the current form contents. If you need to preserve this draft first, download it for review before switching."</p>
            </section>

            <section style="padding: 0.85rem; border: 1px solid var(--border); border-radius: 10px; margin-bottom: 1rem">
                <h2>"Remix an existing lesson"</h2>
                <p style="color: var(--text-secondary)">"Choose a Praxis lesson JSON file. It is read in this browser and mapped into the form; it is not uploaded or published. The imported content becomes a new draft and still needs full review."</p>
                <label style="display: block">"Choose a lesson-pack JSON file"
                    <input
                        type="file"
                        accept=".json,application/json"
                        aria-label="Import an existing Praxis lesson JSON file"
                        on:change=move |event| {
                            let Some(target) = event.target() else {
                                set_notice.set("Could not access the selected file input.".into());
                                return;
                            };
                            let Ok(input) = target.dyn_into::<web_sys::HtmlInputElement>() else {
                                set_notice.set("Could not read the selected file input.".into());
                                return;
                            };
                            let Some(files) = input.files() else {
                                set_notice.set("No file list was available.".into());
                                return;
                            };
                            let Some(file) = files.item(0) else {
                                return;
                            };
                            let filename = file.name();
                            let reader = match web_sys::FileReader::new() {
                                Ok(reader) => reader,
                                Err(_) => {
                                    set_notice.set("Your browser could not open the selected file.".into());
                                    return;
                                }
                            };
                            let reader_for_load = reader.clone();
                            let set_draft_after_load = set_draft;
                            let set_notice_after_load = set_notice;
                            let onload = Closure::<dyn FnMut(web_sys::Event)>::new(move |_| {
                                let Some(text) = reader_for_load
                                    .result()
                                    .ok()
                                    .and_then(|result| result.as_string())
                                else {
                                    set_notice_after_load.set("The selected file could not be read as text.".into());
                                    return;
                                };
                                let parsed = match serde_json::from_str::<Value>(&text) {
                                    Ok(value) => value,
                                    Err(error) => {
                                        set_notice_after_load.set(format!("Invalid JSON: {error}"));
                                        return;
                                    }
                                };
                                match pack_to_draft(&parsed) {
                                    Ok(mut imported) => {
                                        imported.source_notes = format!(
                                            "Imported from {filename}; check source attribution and redistribution rights.\n{}",
                                            imported.source_notes
                                        );
                                        set_draft_after_load.set(imported);
                                        set_notice_after_load.set(format!(
                                            "Imported {filename} as a new local draft. Check all fields and run the standalone checker after export."
                                        ));
                                    }
                                    Err(error) => set_notice_after_load.set(error),
                                }
                            });
                            reader.set_onloadend(Some(onload.as_ref().unchecked_ref()));
                            let blob: &web_sys::Blob = file.unchecked_ref();
                            if reader.read_as_text(blob).is_err() {
                                set_notice.set("Could not start reading the selected file.".into());
                                return;
                            }
                            onload.forget();
                        }
                    />
                </label>
            </section>

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
            <p role="status" aria-live="polite" style="padding: 0.75rem; border: 1px solid var(--border); border-radius: 8px; color: var(--text-secondary); margin-bottom: 0.5rem">
                {move || {
                    let findings = validate_draft(&draft.get());
                    if findings.is_empty() {
                        "Live quick check: no obvious structural blockers. Content accuracy, rights, accessibility and educator review still matter.".to_owned()
                    } else {
                        format!("Live quick check: {} item(s) to complete or review. Select “Check my draft” for the detailed checklist.", findings.len())
                    }
                }}
            </p>
            <p role="status" aria-live="polite" style="padding: 0.75rem; border: 1px solid var(--border); border-radius: 8px; color: var(--text-secondary); margin-bottom: 1rem">{move || notice.get()}</p>

            <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
                <h2>"1. What are you teaching?"</h2>
                <p style="color: var(--text-secondary)">"Start typing an objective ID or choose a title from the suggestions. If no objective fits, enter a clearly provisional ID and document the gap; never invent an official standards code. Drafting does not activate a curriculum node."</p>
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 0.85rem">
                    <label>"Objective ID — search existing objectives or enter a provisional ID"
                        <input
                            list="praxis-objective-options"
                            aria-label="Curriculum objective ID"
                            style="width: 100%"
                            prop:value=move || draft.get().node_id.clone()
                            on:input=move |ev| set_draft.update(|d| d.node_id = event_target_value(&ev))
                        />
                        <datalist id="praxis-objective-options">
                            {objective_options.iter().map(|(id, title)| {
                                let id = id.clone();
                                let title = title.clone();
                                view! { <option value=id label=title></option> }
                            }).collect::<Vec<_>>()}
                        </datalist>
                    </label>
                    <label>"Lesson title"
                        <input style="width: 100%" prop:value=move || draft.get().title.clone() on:input=move |ev| set_draft.update(|d| d.title = event_target_value(&ev)) />
                    </label>
                    <label>"Learner level (grade or course)"
                        <input style="width: 100%" prop:value=move || draft.get().grade_level.clone() on:input=move |ev| set_draft.update(|d| d.grade_level = event_target_value(&ev)) />
                    </label>
                    <label>"Language / locale (BCP 47)"
                        <input
                            style="width: 100%"
                            placeholder="en-ZA, zu-ZA, af-ZA"
                            aria-label="Lesson language or locale"
                            prop:value=move || draft.get().language.clone()
                            on:input=move |ev| set_draft.update(|d| d.language = event_target_value(&ev))
                        />
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
                    <input
                        list="praxis-license-options"
                        aria-label="Intended reuse licence or rights status"
                        placeholder="Choose a licence or enter a verified custom rights status"
                        style="width: 100%"
                        prop:value=move || draft.get().license.clone()
                        on:input=move |ev| set_draft.update(|d| d.license = event_target_value(&ev))
                    />
                    <datalist id="praxis-license-options">
                        <option value="CC BY 4.0" label="Creative Commons Attribution — allow reuse with credit" />
                        <option value="CC BY-SA 4.0" label="Attribution + adaptations shared under the same licence" />
                        <option value="CC BY-NC 4.0" label="Attribution + non-commercial reuse only; narrower reuse" />
                        <option value="CC0 1.0" label="Public-domain dedication where legally possible" />
                        <option value="Public domain (verified)" label="Use only when the public-domain status is verified" />
                        <option value="All rights reserved — do not redistribute" label="Not cleared for public redistribution" />
                        <option value="UNDECIDED — choose a licence before publication." label="Keep as draft until rights are clear" />
                    </datalist>
                </label>
                <p style="margin-top: 0.35rem; color: var(--text-secondary)">"Choose only a licence you have the authority to grant. Check every included source and image for compatible reuse rights; a URL is not permission. This helper is not legal advice."</p>
                <label style="display: block; margin-top: 0.75rem">"Sources and attribution notes (one per line)"
                    <textarea style="width: 100%; min-height: 90px" prop:value=move || draft.get().source_notes.clone() on:input=move |ev| set_draft.update(|d| d.source_notes = event_target_value(&ev)) />
                </label>
                <p style="color: var(--text-secondary)">"Do not upload copyrighted textbook pages, private student work, or materials you cannot redistribute. A source link is not permission. Publication still requires the lesson evidence sidecar and reviewer approval."</p>
            </section>

            <LessonLearnerPreview draft=draft />

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
fn LessonLearnerPreview(draft: ReadSignal<LessonAuthorDraft>) -> impl IntoView {
    view! {
        <section style="padding: 1rem; border: 1px solid var(--border); border-radius: 12px; margin-bottom: 1rem">
            <h2>"Preview as a learner"</h2>
            <p style="color: var(--text-secondary)">"Review the lesson in plain language before exporting. Practice answer keys and feedback stay collapsed in this preview so you can first read each question as a learner would. Imported items not edited by this form remain preserved in the exported pack."</p>
            <article style="padding: 0.85rem; border: 1px solid var(--border); border-radius: 10px">
                <h3>{move || {
                    let title = draft.get().title;
                    if title.trim().is_empty() { "Your lesson title will appear here".to_owned() } else { title }
                }}</h3>
                <p style="color: var(--text-secondary)">{move || {
                    let current = draft.get();
                    format!("{} · {}", current.grade_level, current.language)
                }}</p>
                <h4>"What you will learn"</h4>
                <p>{move || {
                    let objective = draft.get().learning_objective;
                    if objective.trim().is_empty() { "Add a learning objective describing what someone will be able to do.".to_owned() } else { objective }
                }}</p>
                <h4>"Explanation"</h4>
                <p style="white-space: pre-wrap">{move || {
                    let explanation = draft.get().explanation;
                    if explanation.trim().is_empty() { "Your explanation will appear here.".to_owned() } else { explanation }
                }}</p>
                <h4>"Worked example"</h4>
                <p style="font-weight: 600">{move || {
                    let problem = draft.get().example_problem;
                    if problem.trim().is_empty() { "Add a worked-example question.".to_owned() } else { problem }
                }}</p>
                <ol>
                    <li>
                        <p>{move || draft.get().example_step_one.clone()}</p>
                        <p style="color: var(--text-secondary)">{move || draft.get().example_step_one_result.clone()}</p>
                    </li>
                    <li>
                        <p>{move || draft.get().example_step_two.clone()}</p>
                        <p style="color: var(--text-secondary)">{move || draft.get().example_step_two_result.clone()}</p>
                    </li>
                </ol>
                <details>
                    <summary>"Show worked-example answer"</summary>
                    <p>{move || draft.get().example_answer.clone()}</p>
                    <p style="color: var(--text-secondary)">{move || draft.get().visual_description.clone()}</p>
                </details>
            </article>

            <h3 style="margin-top: 1rem">"Try these questions"</h3>
            {move || {
                draft.get().practice.iter().enumerate().map(|(index, item)| {
                    let question = item.question.clone();
                    let answer = item.answer.clone();
                    let explanation = item.explanation.clone();
                    let hint = item.hint.clone();
                    let distractors = item.distractors.clone();
                    view! {
                        <article style="padding: 0.85rem; border: 1px solid var(--border); border-radius: 10px; margin: 0.75rem 0">
                            <h4>{format!("Practice {}", index + 1)}</h4>
                            <p>{if question.trim().is_empty() { "Add a practice question.".to_owned() } else { question }}</p>
                            <details>
                                <summary>"Show author answer and feedback"</summary>
                                <p><strong>"Expected answer: "</strong>{answer}</p>
                                <p><strong>"Why: "</strong>{explanation}</p>
                                <p><strong>"Hint: "</strong>{hint}</p>
                                <p><strong>"Wrong-answer choices: "</strong><span style="white-space: pre-wrap">{distractors}</span></p>
                            </details>
                        </article>
                    }
                }).collect_view()
            }}

            <h3>"Check understanding"</h3>
            <p style="font-weight: 600">{move || draft.get().assessment_question.clone()}</p>
            <details>
                <summary>"Show expected answer and rubric"</summary>
                <p>{move || draft.get().assessment_answer.clone()}</p>
                <p>{move || draft.get().assessment_rubric.clone()}</p>
                <p style="color: var(--text-secondary)">{move || draft.get().assessment_hint.clone()}</p>
            </details>
            <h3>"Remember"</h3>
            <p><strong>{move || draft.get().flashcard_front.clone()}</strong></p>
            <details>
                <summary>"Show flashcard back"</summary>
                <p>{move || draft.get().flashcard_back.clone()}</p>
            </details>
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
        assert_eq!(value["authoring"]["language"], "en-ZA");
        assert_eq!(value["quality"]["needs_human_review"], true);
        assert_eq!(value["quality"]["metrics_valid_for_current_revision"], false);
        assert_eq!(value["authoring"]["status"], "draft");
        let findings = validate_draft(&draft);
        assert!(
            findings.iter().any(|finding| finding.contains("choose the actual reuse licence")),
            "the sample should make unresolved reuse rights visible"
        );
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
    fn imported_lesson_pack_becomes_a_remixable_local_draft() {
        let original = LessonAuthorDraft::default();
        let serialized = lesson_pack_value(&original);
        let imported = pack_to_draft(&serialized).expect("exported pack should import");
        assert_eq!(imported.node_id, original.node_id);
        assert_eq!(imported.title, original.title);
        assert_eq!(imported.practice[0].question, original.practice[0].question);
        assert_eq!(imported.practice[0].distractors, original.practice[0].distractors);
        assert!(imported.source_notes.contains("Imported as a new local draft"));
        assert!(imported.license.contains("UNDECIDED"));
    }

    #[test]
    fn import_rejects_json_without_a_lesson_object() {
        let error = pack_to_draft(&serde_json::json!({"node_id":"example"}))
            .expect_err("arbitrary JSON must not be treated as a lesson pack");
        assert!(error.contains("lesson object is missing"));
    }

    #[test]
    fn remix_export_preserves_unedited_items_and_unknown_metadata() {
        let mut original = lesson_pack_value(&LessonAuthorDraft::default());
        original["lesson"]["examples"]
            .as_array_mut()
            .unwrap()
            .push(json!({
                "problem": "Unedited second example",
                "steps": [{"instruction":"Keep this step","result":"Keep this result"}],
                "answer": "Second answer"
            }));
        original["lesson"]["practice_problems"]
            .as_array_mut()
            .unwrap()
            .push(json!({
                "question":"Unedited fourth practice item",
                "answer":"Keep the answer",
                "explanation":"Keep the explanation",
                "distractors":["A","B","C"]
            }));
        original["assessment"]["items"]
            .as_array_mut()
            .unwrap()
            .push(json!({
                "question":"Unedited second assessment item",
                "correct_answer":"Keep assessment answer",
                "points":4,
                "bloom_level":"Apply"
            }));
        original["flashcards"]
            .as_array_mut()
            .unwrap()
            .push(json!({"front":"Unedited second card","back":"Keep the card"}));
        original["lesson"]["localization_note"] = json!("Preserve this unknown field");

        let mut draft = pack_to_draft(&original).expect("lesson pack should import");
        draft.title = "Remixed title".into();
        let exported = lesson_pack_value(&draft);

        assert_eq!(exported["lesson"]["title"], "Remixed title");
        assert_eq!(exported["lesson"]["examples"].as_array().unwrap().len(), 2);
        assert_eq!(
            exported["lesson"]["examples"][1]["problem"],
            "Unedited second example"
        );
        assert_eq!(exported["lesson"]["practice_problems"].as_array().unwrap().len(), 4);
        assert_eq!(
            exported["lesson"]["practice_problems"][3]["question"],
            "Unedited fourth practice item"
        );
        assert_eq!(exported["assessment"]["items"].as_array().unwrap().len(), 2);
        assert_eq!(exported["assessment"]["total_points"], 6);
        assert_eq!(exported["flashcards"].as_array().unwrap().len(), 2);
        assert_eq!(
            exported["lesson"]["localization_note"],
            "Preserve this unknown field"
        );
    }

    #[test]
    fn import_does_not_fill_missing_practice_items_with_sample_content() {
        let mut original = lesson_pack_value(&LessonAuthorDraft::default());
        let first_practice = original["lesson"]["practice_problems"][0].clone();
        original["lesson"]["practice_problems"] = json!([first_practice]);
        let draft = pack_to_draft(&original).expect("one-item lesson should import");
        assert_eq!(
            draft.practice[1].question,
            "",
            "missing items must be blank, not copied from the sample lesson"
        );
        assert_eq!(draft.practice[2].question, "");
    }

    #[test]
    fn author_quick_check_blocks_public_redistribution_when_rights_are_reserved() {
        let mut draft = LessonAuthorDraft::default();
        draft.license = "All rights reserved — do not redistribute".into();
        let findings = validate_draft(&draft);
        assert!(findings.iter().any(|finding| {
            finding.contains("does not permit public redistribution")
        }));
    }

    #[test]
    fn author_quick_check_detects_numeric_answer_collisions() {
        assert!(author_numeric_choices_collide("3 children", "3"));
        assert!(author_numeric_choices_collide("1/2", "2/4"));
        assert!(!author_numeric_choices_collide("12 cm", "12 square units"));

        let mut draft = LessonAuthorDraft::default();
        draft.practice[0].answer = "3 children".into();
        draft.practice[0].distractors = "13\n2\n3".into();
        let findings = validate_draft(&draft);
        assert!(findings.iter().any(|finding| {
            finding.contains("different unit or equivalent fraction")
        }));
    }

    #[test]
    fn blank_starter_contains_no_sample_lesson_text() {
        let draft = blank_lesson_draft();
        assert!(draft.node_id.is_empty());
        assert!(draft.title.is_empty());
        assert!(draft.learning_objective.is_empty());
        assert!(draft.explanation.is_empty());
        assert!(draft.practice.iter().all(|item| item.question.is_empty()));
        assert!(draft.source_pack.is_none());
        assert!(draft.license.contains("UNDECIDED"));
        let findings = validate_draft(&draft);
        assert!(findings.iter().any(|finding| finding.contains("Lesson title is required")));
        assert!(findings.iter().any(|finding| finding.contains("Practice item 1 needs a question")));
    }

    #[test]
    fn hands_on_activity_template_has_concrete_investigation_steps_and_valid_practice() {
        let draft = hands_on_activity_draft();
        assert_eq!(draft.node_id, "COMMUNITY.STEM.PAPER_BRIDGE");
        assert!(draft.learning_objective.contains("test it with equal loads"));
        assert!(draft.example_step_one_result.contains("same paper"));
        assert!(draft.example_step_two_result.contains("record"));
        assert_eq!(draft.practice.len(), 3);
        assert!(draft.practice.iter().all(|item| {
            !item.question.is_empty()
                && !item.answer.is_empty()
                && item.distractors.lines().filter(|line| !line.trim().is_empty()).count() == 3
        }));
        let findings = validate_draft(&draft);
        assert_eq!(
            findings.len(),
            1,
            "template should pass structural checks but keep the unresolved licence visible: {findings:?}"
        );
        assert!(findings[0].contains("choose the actual reuse licence"));
    }

    #[test]
    fn downloaded_filename_is_safe_and_has_a_fallback() {
        let mut draft = LessonAuthorDraft::default();
        assert_eq!(lesson_filename(&draft), "praxis-example-math-gr3-equal-groups.json");
        draft.node_id = "///".into();
        assert_eq!(lesson_filename(&draft), "praxis-lesson-draft.json");
    }
}
