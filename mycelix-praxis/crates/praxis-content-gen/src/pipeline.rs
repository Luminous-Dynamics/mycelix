// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Content generation pipeline orchestration.
//!
//! The [`ContentPipeline`] takes a [`StandardInput`] and produces complete
//! teaching materials (lessons, assessments, flashcards) by making multiple
//! calls to a [`ContentGenerator`] with appropriate [`ContentChannels`].

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::channels::ContentChannels;
use crate::templates;
use crate::types::*;

/// Trait for content generators.
///
/// Implementations convert [`ContentChannels`] + context into generated text.
/// The real implementation maps channels to Broca's 43-channel ThoughtChannels;
/// for development, use [`MockGenerator`](crate::mock::MockGenerator).
pub trait ContentGenerator {
    /// Generate text given channel configuration, context prompt, and token budget.
    fn generate(
        &self,
        channels: &ContentChannels,
        context: &str,
        max_tokens: usize,
    ) -> GenerationOutput;
}

/// Output from a single generation call.
#[derive(Clone, Debug)]
pub struct GenerationOutput {
    /// Generated text
    pub text: String,
    /// Coherence score (0.0-1.0)
    pub coherence: f32,
    /// Whether hallucination was detected
    pub hallucination_flag: bool,
    /// Number of self-correction veto events
    pub veto_count: u32,
}

/// Input describing a standard to generate content for.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StandardInput {
    /// Standard code (e.g., "3.OA.A.1" or "CCSS.MATH.CONTENT.3.OA.A.1")
    pub code: String,
    /// Human-readable description
    pub description: String,
    /// Target grade level (e.g., "Grade3")
    pub grade_level: String,
    /// Subject domain (e.g., "Operations & Algebraic Thinking")
    pub domain: String,
    /// What the student already knows
    pub prerequisites: Vec<String>,
}

/// Errors from the content generation pipeline.
#[derive(Debug, thiserror::Error)]
pub enum ContentGenError {
    /// All retry attempts produced low-coherence, hallucination-flagged, or
    /// known-placeholder content.
    #[error("content quality below threshold after {0} attempts")]
    QualityBelowThreshold(u32),
    /// A parsed practice item failed deterministic answer/distractor validation.
    #[error("practice problem failed validation after {attempts} attempts: {reason}")]
    InvalidPracticeProblem { attempts: u32, reason: &'static str },
    /// The generator returned empty text.
    #[error("generator returned empty text")]
    EmptyGeneration,
}

/// Content generation pipeline.
///
/// Orchestrates multiple generation calls to produce complete teaching
/// materials from a single standard definition.
pub struct ContentPipeline<G: ContentGenerator> {
    generator: G,
    /// Minimum coherence to accept (default: 0.6)
    quality_threshold: f32,
    /// Maximum retry attempts on low quality (default: 3)
    max_retries: u32,
}

impl<G: ContentGenerator> ContentPipeline<G> {
    /// Create a new pipeline with default quality settings.
    pub fn new(generator: G) -> Self {
        Self {
            generator,
            quality_threshold: 0.6,
            max_retries: 3,
        }
    }

    /// Set the minimum coherence threshold for accepting generated content.
    pub fn with_quality_threshold(mut self, threshold: f32) -> Self {
        self.quality_threshold = threshold.clamp(0.0, 1.0);
        self
    }

    /// Set the maximum retry count.
    pub fn with_max_retries(mut self, retries: u32) -> Self {
        self.max_retries = retries;
        self
    }

    /// Parse grade ordinal from grade_level string (e.g., "Grade3" -> 4).
    fn grade_ordinal(grade_level: &str) -> u8 {
        match grade_level {
            "PreK" => 0,
            "Kindergarten" => 1,
            s if s.starts_with("Grade") => s[5..].parse::<u8>().unwrap_or(6) + 1,
            "College" => 14,
            "Undergraduate" => 15,
            "Graduate" => 16,
            "Doctoral" => 17,
            "PostDoctoral" => 18,
            "Professional" => 19,
            "Adult" => 20,
            _ => 8, // default to middle school
        }
    }

    /// Generate a complete lesson for a standard.
    pub fn generate_lesson(
        &self,
        standard: &StandardInput,
    ) -> Result<GeneratedLesson, ContentGenError> {
        let grade_ord = Self::grade_ordinal(&standard.grade_level);

        // 1. Main explanation
        let explanation_output = self.generate_with_retry(
            &ContentChannels::teaching_factual().for_grade(grade_ord),
            &templates::lesson_context(standard),
            512,
        )?;

        // 2. Worked examples (3)
        let examples = (0..3)
            .map(|i| self.generate_worked_example(standard, grade_ord, i))
            .collect::<Result<Vec<_>, _>>()?;

        // 3. Practice problems (7, spread across difficulty levels)
        let problems = (0..7)
            .map(|i| {
                let difficulty = ((i as u16 + 1) * 1000) / 8; // ~125, 250, ..., 875
                self.generate_practice_problem(standard, grade_ord, difficulty)
            })
            .collect::<Result<Vec<_>, _>>()?;

        // 4. Progressive hints (3 levels)
        let hints = (1..=3u8)
            .map(|level| self.generate_hint(standard, grade_ord, level))
            .collect::<Result<Vec<_>, _>>()?;

        // 5. Vocabulary
        let vocab_output = self.generate_with_retry(
            &ContentChannels::teaching_factual().for_grade(grade_ord),
            &templates::vocabulary_context(standard),
            256,
        )?;
        let key_vocabulary = parse_vocabulary(&vocab_output.text);

        // 6. Misconceptions
        let misconception_output = self.generate_with_retry(
            &ContentChannels::misconception_correction().for_grade(grade_ord),
            &templates::misconception_context(standard),
            256,
        )?;
        let misconceptions = parse_misconceptions(&misconception_output.text);

        // 7. Compute aggregate quality
        let all_coherences = [
            explanation_output.coherence,
            vocab_output.coherence,
            misconception_output.coherence,
        ];
        let avg_coherence = all_coherences.iter().sum::<f32>() / all_coherences.len() as f32;
        let any_hallucination = explanation_output.hallucination_flag
            || vocab_output.hallucination_flag
            || misconception_output.hallucination_flag;
        let total_vetos = explanation_output.veto_count
            + vocab_output.veto_count
            + misconception_output.veto_count;

        // Determine Bloom's level from the standard code heuristic
        let bloom_level = infer_bloom_level(&standard.code, &standard.description);

        Ok(GeneratedLesson {
            standard_code: standard.code.clone(),
            title: format!("{}: {}", standard.code, truncate(&standard.description, 60)),
            grade_level: standard.grade_level.clone(),
            bloom_level,
            explanation: explanation_output.text,
            examples,
            practice_problems: problems,
            hints,
            key_vocabulary,
            misconceptions,
            generation_quality: GenerationQuality {
                coherence: avg_coherence,
                hallucination_detected: any_hallucination,
                veto_count: total_vetos,
                epistemic_confidence: avg_coherence, // proxy
                needs_human_review: avg_coherence < self.quality_threshold || any_hallucination,
            },
        })
    }

    /// Generate an assessment for a standard.
    pub fn generate_assessment(
        &self,
        standard: &StandardInput,
        num_items: usize,
    ) -> Result<GeneratedAssessment, ContentGenError> {
        let grade_ord = Self::grade_ordinal(&standard.grade_level);
        let blooms = [
            "Remember",
            "Understand",
            "Apply",
            "Analyze",
            "Evaluate",
            "Create",
        ];

        let items: Vec<AssessmentItem> = (0..num_items)
            .map(|i| {
                let bloom = blooms[i % blooms.len()];
                self.generate_assessment_item(standard, grade_ord, bloom)
            })
            .collect::<Result<Vec<_>, _>>()?;

        let total_points: u32 = items.iter().map(|it| it.points).sum();

        // Count Bloom's distribution
        let mut bloom_counts = std::collections::HashMap::new();
        for item in &items {
            *bloom_counts.entry(item.bloom_level.clone()).or_insert(0u32) += 1;
        }
        let bloom_distribution: Vec<(String, u32)> = bloom_counts.into_iter().collect();

        Ok(GeneratedAssessment {
            standard_code: standard.code.clone(),
            title: format!("Assessment: {}", standard.description),
            items,
            total_points,
            estimated_minutes: (num_items as u32) * 3, // ~3 min per item
            bloom_distribution,
        })
    }

    /// Generate flashcards for a standard.
    pub fn generate_flashcards(
        &self,
        standard: &StandardInput,
        num_cards: usize,
    ) -> Result<Vec<GeneratedFlashcard>, ContentGenError> {
        let grade_ord = Self::grade_ordinal(&standard.grade_level);

        (0..num_cards)
            .map(|i| {
                let difficulty = ((i as u16 + 1) * 1000) / (num_cards as u16 + 1);
                let output = self.generate_with_retry(
                    &ContentChannels::practice_problem().for_grade(grade_ord),
                    &templates::flashcard_context(standard),
                    128,
                )?;

                let (front, back) = split_flashcard(&output.text);
                Ok(GeneratedFlashcard {
                    front,
                    back,
                    tags: vec![
                        standard.code.clone(),
                        standard.grade_level.clone(),
                        standard.domain.clone(),
                    ],
                    difficulty_permille: difficulty,
                })
            })
            .collect()
    }

    /// Generate content for all standards in a curriculum.
    pub fn generate_curriculum(
        &self,
        standards: &[StandardInput],
    ) -> Result<Vec<GeneratedLesson>, ContentGenError> {
        standards.iter().map(|s| self.generate_lesson(s)).collect()
    }

    // ---- Internal helpers ----

    fn generate_with_retry(
        &self,
        channels: &ContentChannels,
        context: &str,
        max_tokens: usize,
    ) -> Result<GenerationOutput, ContentGenError> {
        for _attempt in 0..self.max_retries {
            let output = self.generator.generate(channels, context, max_tokens);
            if output.text.is_empty() {
                continue;
            }
            if output.coherence >= self.quality_threshold
                && !output.hallucination_flag
                && !is_placeholder_content(&output.text)
            {
                return Ok(output);
            }
        }
        Err(ContentGenError::QualityBelowThreshold(self.max_retries))
    }

    fn generate_worked_example(
        &self,
        standard: &StandardInput,
        grade_ord: u8,
        index: usize,
    ) -> Result<WorkedExample, ContentGenError> {
        let output = self.generate_with_retry(
            &ContentChannels::worked_example().for_grade(grade_ord),
            &templates::worked_example_context(standard, index),
            384,
        )?;

        Ok(parse_worked_example(&output.text))
    }

    fn generate_practice_problem(
        &self,
        standard: &StandardInput,
        grade_ord: u8,
        difficulty: u16,
    ) -> Result<PracticeProblem, ContentGenError> {
        let channels = ContentChannels::practice_problem().for_grade(grade_ord);
        let context = templates::practice_problem_context(standard, difficulty);
        let mut last_reason = "no generation attempt was made";
        let mut last_failure_was_quality = false;

        for _attempt in 0..self.max_retries {
            // Keep the outer attempt budget truthful: wrapping generate_with_retry
            // here would turn N practice attempts into as many as N squared model calls.
            let output = self.generator.generate(&channels, &context, 256);
            if output.text.trim().is_empty()
                || output.coherence < self.quality_threshold
                || output.hallucination_flag
                || is_placeholder_content(&output.text)
            {
                last_failure_was_quality = true;
                continue;
            }

            let problem = parse_practice_problem(&output.text, difficulty);
            match validate_practice_problem(&problem) {
                Ok(()) => return Ok(problem),
                Err(reason) => {
                    last_reason = reason;
                    last_failure_was_quality = false;
                }
            }
        }

        if last_failure_was_quality {
            Err(ContentGenError::QualityBelowThreshold(self.max_retries))
        } else {
            Err(ContentGenError::InvalidPracticeProblem {
                attempts: self.max_retries,
                reason: last_reason,
            })
        }
    }

    fn generate_hint(
        &self,
        standard: &StandardInput,
        grade_ord: u8,
        level: u8,
    ) -> Result<String, ContentGenError> {
        let output = self.generate_with_retry(
            &ContentChannels::hint(level).for_grade(grade_ord),
            &templates::hint_context(standard, level),
            128,
        )?;
        Ok(output.text)
    }

    fn generate_assessment_item(
        &self,
        standard: &StandardInput,
        grade_ord: u8,
        bloom: &str,
    ) -> Result<AssessmentItem, ContentGenError> {
        let output = self.generate_with_retry(
            &ContentChannels::practice_problem().for_grade(grade_ord),
            &templates::assessment_item_context(standard, bloom),
            256,
        )?;

        Ok(parse_assessment_item(&output.text, bloom))
    }
}

/// Conservatively reject known template/fallback strings that are structurally
/// non-empty but do not teach or assess the declared skill.
///
/// This is a deterministic guardrail, not a semantic proof of correctness.
/// Passing it does not mean content is accurate, aligned, accessible, or
/// pedagogically effective; those claims need separate checks and review.
fn is_placeholder_content(text: &str) -> bool {
    let normalized = text
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .to_lowercase();

    if normalized.is_empty() {
        return true;
    }
    let core = normalized.trim_end_matches(['.', '!', '?']).trim();

    let exact_placeholders = [
        "grade 3 mathematics content",
        "a common mistake in grade 3 mathematics",
        "the correct way",
        "used in grade 3 maths",
        "lorem ipsum",
        "todo",
        "placeholder",
        "answer",
    ];
    if exact_placeholders.contains(&core) {
        return true;
    }

    let template_phrases = [
        "answer: see worked solution",
        "what is important about grade 3 mathematics?",
        "a key word in this topic",
        "try the method for grade 3 mathematics.",
        "start with what you know about grade 3 mathematics.",
        "let's explore this concept together!",
        "every expert was once a beginner.",
    ];

    template_phrases
        .iter()
        .any(|phrase| normalized.contains(phrase))
        || (normalized.contains("solve a grade ") && normalized.contains(" mathematics problem"))
        || (core.starts_with("explain grade ") && core.contains(" mathematics"))
        || (core.starts_with("grade ") && core.ends_with(" mathematics content"))
}
// ---- Parsing helpers ----

/// Parse vocabulary from generated text. In mock mode, text is pre-formatted
/// as "term: definition | example" lines. With real Broca, a more sophisticated
/// parser would be needed.
fn parse_vocabulary(text: &str) -> Vec<VocabularyTerm> {
    text.lines()
        .filter(|l| l.contains(':'))
        .filter_map(|line| {
            let mut parts = line.splitn(2, ':');
            let term = parts.next()?.trim().to_string();
            let rest = parts.next()?.trim();

            let (definition, example) = if rest.contains('|') {
                let mut sub = rest.splitn(2, '|');
                (
                    sub.next().unwrap_or("").trim().to_string(),
                    sub.next().unwrap_or("").trim().to_string(),
                )
            } else {
                (rest.to_string(), String::new())
            };

            if term.is_empty() {
                return None;
            }

            Some(VocabularyTerm {
                term,
                definition,
                example_usage: example,
            })
        })
        .collect()
}

/// Parse a misconception block. Mock format: "WRONG: ... | RIGHT: ... | WHY: ..."
fn parse_misconceptions(text: &str) -> Vec<Misconception> {
    // Simple approach: split on double newlines for multiple misconceptions
    let blocks: Vec<&str> = if text.contains("\n\n") {
        text.split("\n\n").collect()
    } else {
        vec![text]
    };

    blocks
        .iter()
        .filter(|b| !b.trim().is_empty())
        .map(|block| {
            let mut misconception = String::new();
            let mut correction = String::new();
            let mut why = String::new();

            for line in block.lines() {
                let line = line.trim();
                if let Some(rest) = line.strip_prefix("WRONG:") {
                    misconception = rest.trim().to_string();
                } else if let Some(rest) = line.strip_prefix("RIGHT:") {
                    correction = rest.trim().to_string();
                } else if let Some(rest) = line.strip_prefix("WHY:") {
                    why = rest.trim().to_string();
                } else if misconception.is_empty() {
                    misconception = line.to_string();
                }
            }

            Misconception {
                misconception,
                correction,
                why_students_think_this: why,
            }
        })
        .collect()
}

/// Parse a worked example from generated text.
fn parse_worked_example(text: &str) -> WorkedExample {
    let lines: Vec<&str> = text.lines().collect();

    let problem = lines.first().unwrap_or(&"").to_string();
    let answer = lines.last().unwrap_or(&"").to_string();

    let steps: Vec<SolutionStep> = lines
        .iter()
        .skip(1)
        .take(lines.len().saturating_sub(2))
        .filter(|l| !l.trim().is_empty())
        .map(|line| {
            let parts: Vec<&str> = line.splitn(2, "->").collect();
            if parts.len() == 2 {
                SolutionStep {
                    instruction: parts[0].trim().to_string(),
                    result: parts[1].trim().to_string(),
                }
            } else {
                SolutionStep {
                    instruction: line.trim().to_string(),
                    result: String::new(),
                }
            }
        })
        .collect();

    WorkedExample {
        problem,
        steps,
        answer,
        visual_description: None,
    }
}

/// Parse a practice problem from generated text.
fn parse_practice_problem(text: &str, difficulty: u16) -> PracticeProblem {
    let lines: Vec<&str> = text.lines().collect();

    PracticeProblem {
        question: lines.first().unwrap_or(&"").to_string(),
        answer: lines.get(1).unwrap_or(&"").to_string(),
        difficulty_permille: difficulty,
        bloom_level: "Apply".to_string(),
        hints: lines.iter().skip(4).map(|l| l.to_string()).collect(),
        explanation: lines.get(2).unwrap_or(&"").to_string(),
        distractors: lines
            .get(3)
            .map(|l| l.split(',').map(|s| s.trim().to_string()).collect())
            .unwrap_or_default(),
    }
}

fn normalize_answer_choice(text: &str) -> String {
    text.split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .trim()
        .trim_end_matches(['.', '!', '?'])
        .to_lowercase()
}

fn greatest_common_divisor(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn rational_key(numerator: i128, denominator: i128) -> Option<String> {
    if denominator == 0 {
        return None;
    }
    let sign = if (numerator < 0) ^ (denominator < 0) {
        "-"
    } else {
        ""
    };
    let numerator_magnitude = numerator.unsigned_abs();
    let denominator_magnitude = denominator.unsigned_abs();
    let divisor = greatest_common_divisor(numerator_magnitude, denominator_magnitude);
    Some(format!(
        "{sign}{}/{}",
        numerator_magnitude / divisor,
        denominator_magnitude / divisor
    ))
}

fn scalar_number_key(token: &str) -> Option<String> {
    if let Some(percent) = token.strip_suffix('%') {
        let (numerator, denominator) = if let Some((left, right)) = percent.split_once('/') {
            let numerator = left.parse::<i128>().ok()?;
            let denominator = right.parse::<i128>().ok()?.checked_mul(100)?;
            (numerator, denominator)
        } else {
            (percent.parse::<i128>().ok()?, 100)
        };
        return rational_key(numerator, denominator);
    }
    if let Some((left, right)) = token.split_once('/') {
        return rational_key(left.parse::<i128>().ok()?, right.parse::<i128>().ok()?);
    }
    if let Ok(integer) = token.parse::<i128>() {
        return rational_key(integer, 1);
    }
    if let Some((whole, fraction)) = token.split_once('.') {
        if fraction.is_empty() || !fraction.chars().all(|char| char.is_ascii_digit()) {
            return None;
        }
        let sign = if whole.starts_with('-') { -1 } else { 1 };
        let whole_digits = whole.trim_start_matches('-');
        let whole_value = if whole_digits.is_empty() {
            0
        } else {
            whole_digits.parse::<i128>().ok()?
        };
        let denominator = 10_i128.checked_pow(u32::try_from(fraction.len()).ok()?)?;
        let fraction_value = fraction.parse::<i128>().ok()?;
        let numerator = whole_value
            .checked_mul(denominator)?
            .checked_add(fraction_value)?
            .checked_mul(sign)?;
        return rational_key(numerator, denominator);
    }
    None
}

fn scalar_answer_key(text: &str) -> Option<(String, Option<&'static str>)> {
    let normalized = normalize_answer_choice(text);
    let units: [(&str, &str); 27] = [
        ("square centimetres", "area_cm2"),
        ("square centimeters", "area_cm2"),
        ("square metres", "area_m2"),
        ("square meters", "area_m2"),
        ("square units", "area_unit"),
        ("square unit", "area_unit"),
        ("square cm", "area_cm2"),
        ("square m", "area_m2"),
        ("centimetres", "length_cm"),
        ("centimeters", "length_cm"),
        ("millimetres", "length_mm"),
        ("millimeters", "length_mm"),
        ("kilograms", "mass_kg"),
        ("kilometres", "length_km"),
        ("kilometers", "length_km"),
        ("minutes", "time_min"),
        ("seconds", "time_s"),
        ("hours", "time_h"),
        ("learners", "count"),
        ("children", "count"),
        ("counters", "count"),
        ("objects", "count"),
        ("oranges", "count"),
        ("students", "count"),
        ("books", "count"),
        ("cm", "length_cm"),
        ("mm", "length_mm"),
    ];
    for (unit, canonical_unit) in units {
        if let Some(number) = normalized.strip_suffix(unit)
            && (number.is_empty() || number.ends_with(' '))
        {
            let number = number.trim();
            if let Some(key) = scalar_number_key(number) {
                return Some((key, Some(canonical_unit)));
            }
        }
    }
    let tokens = normalized.split_whitespace().collect::<Vec<_>>();
    if tokens.len() == 2 {
        let unit = match tokens[1] {
            "learner" | "child" | "student" | "object" | "counter" | "orange" | "book" | "desk"
            | "cone" | "car" | "star" | "slice" | "group" | "groups" => Some("count"),
            "m" => Some("length_m"),
            "kg" => Some("mass_kg"),
            _ => None,
        };
        if let Some(unit) = unit {
            return scalar_number_key(tokens[0]).map(|key| (key, Some(unit)));
        }
    }
    if tokens.len() == 1 {
        return scalar_number_key(tokens[0]).map(|key| (key, None));
    }
    None
}

fn numeric_answer_choices_collide(answer: &str, distractor: &str) -> bool {
    let Some((answer_value, answer_unit)) = scalar_answer_key(answer) else {
        return false;
    };
    let Some((distractor_value, distractor_unit)) = scalar_answer_key(distractor) else {
        return false;
    };
    answer_value == distractor_value
        && (answer_unit.is_none() || distractor_unit.is_none() || answer_unit == distractor_unit)
}

fn validate_practice_problem(problem: &PracticeProblem) -> Result<(), &'static str> {
    if problem.question.trim().is_empty() {
        return Err("question is empty");
    }
    if problem.answer.trim().is_empty() {
        return Err("correct answer is empty");
    }
    if problem.explanation.trim().is_empty() {
        return Err("explanation is empty");
    }
    if problem.distractors.len() != 3 {
        return Err("exactly three distractors are required");
    }

    let normalized_answer = normalize_answer_choice(&problem.answer);
    let normalized_distractors = problem
        .distractors
        .iter()
        .map(|distractor| normalize_answer_choice(distractor))
        .collect::<Vec<_>>();
    if normalized_distractors.iter().any(String::is_empty) {
        return Err("a distractor is empty");
    }
    if normalized_distractors
        .iter()
        .any(|distractor| distractor == &normalized_answer)
    {
        return Err("the correct answer also appears as a distractor");
    }
    if problem
        .distractors
        .iter()
        .any(|distractor| numeric_answer_choices_collide(&problem.answer, distractor))
    {
        return Err("a distractor is numerically equivalent to the correct answer");
    }

    let distinct_distractors = normalized_distractors.iter().collect::<BTreeSet<_>>();
    if distinct_distractors.len() != normalized_distractors.len() {
        return Err("duplicate distractors were generated");
    }

    Ok(())
}

/// Parse an assessment item from generated text.
fn parse_assessment_item(text: &str, bloom: &str) -> AssessmentItem {
    let lines: Vec<&str> = text.lines().collect();
    let question = lines.first().unwrap_or(&"").to_string();
    let answer = lines.get(1).unwrap_or(&"").to_string();

    AssessmentItem {
        question,
        item_type: AssessmentItemType::ShortAnswer,
        correct_answer: answer,
        points: match bloom {
            "Remember" => 1,
            "Understand" => 2,
            "Apply" => 2,
            "Analyze" => 3,
            "Evaluate" => 3,
            "Create" => 4,
            _ => 1,
        },
        bloom_level: bloom.to_string(),
        rubric: None,
        hint: lines.get(2).map(|s| s.to_string()),
    }
}

/// Split flashcard text into front/back (separated by " | " or newline).
fn split_flashcard(text: &str) -> (String, String) {
    if text.contains(" | ") {
        let mut parts = text.splitn(2, " | ");
        (
            parts.next().unwrap_or("").to_string(),
            parts.next().unwrap_or("").to_string(),
        )
    } else {
        let lines: Vec<&str> = text.lines().collect();
        (
            lines.first().unwrap_or(&"").to_string(),
            lines.get(1).unwrap_or(&"").to_string(),
        )
    }
}

/// Infer Bloom's level from standard code and description.
fn infer_bloom_level(code: &str, description: &str) -> String {
    let desc_lower = description.to_lowercase();
    if desc_lower.contains("create")
        || desc_lower.contains("design")
        || desc_lower.contains("construct")
    {
        "Create".to_string()
    } else if desc_lower.contains("evaluate")
        || desc_lower.contains("judge")
        || desc_lower.contains("justify")
    {
        "Evaluate".to_string()
    } else if desc_lower.contains("analyze")
        || desc_lower.contains("compare")
        || desc_lower.contains("distinguish")
    {
        "Analyze".to_string()
    } else if desc_lower.contains("solve")
        || desc_lower.contains("apply")
        || desc_lower.contains("use")
        || desc_lower.contains("determine")
        || desc_lower.contains("multiply")
        || desc_lower.contains("divide")
    {
        "Apply".to_string()
    } else if desc_lower.contains("interpret")
        || desc_lower.contains("explain")
        || desc_lower.contains("understand")
        || desc_lower.contains("describe")
        || desc_lower.contains("represent")
    {
        "Understand".to_string()
    } else if code.contains(".7") || desc_lower.contains("fluently") || desc_lower.contains("know")
    {
        "Remember".to_string()
    } else {
        "Understand".to_string()
    }
}

/// Truncate a string to at most `max_len` characters, appending "..." if truncated.
fn truncate(s: &str, max_len: usize) -> String {
    if s.chars().count() <= max_len {
        s.to_string()
    } else {
        let truncated: String = s.chars().take(max_len.saturating_sub(3)).collect();
        format!("{truncated}...")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockGenerator;

    fn sample_standard() -> StandardInput {
        StandardInput {
            code: "3.OA.A.1".to_string(),
            description: "Interpret products of whole numbers".to_string(),
            grade_level: "Grade3".to_string(),
            domain: "Operations & Algebraic Thinking".to_string(),
            prerequisites: vec!["Addition within 100".to_string()],
        }
    }

    #[test]
    fn test_generate_lesson_full() {
        let pipeline = ContentPipeline::new(MockGenerator);
        let lesson = pipeline.generate_lesson(&sample_standard()).unwrap();

        assert_eq!(lesson.standard_code, "3.OA.A.1");
        assert!(!lesson.explanation.is_empty());
        assert_eq!(lesson.examples.len(), 3);
        assert_eq!(lesson.practice_problems.len(), 7);
        assert_eq!(lesson.hints.len(), 3);
        assert!(!lesson.key_vocabulary.is_empty());
        assert!(!lesson.misconceptions.is_empty());
        assert!(!lesson.generation_quality.needs_human_review);
        assert!(!lesson.generation_quality.hallucination_detected);
    }

    #[test]
    fn test_generate_assessment() {
        let pipeline = ContentPipeline::new(MockGenerator);
        let assessment = pipeline.generate_assessment(&sample_standard(), 6).unwrap();

        assert_eq!(assessment.standard_code, "3.OA.A.1");
        assert_eq!(assessment.items.len(), 6);
        assert!(assessment.total_points > 0);
        assert!(assessment.estimated_minutes > 0);
        assert!(!assessment.bloom_distribution.is_empty());
    }

    #[test]
    fn test_generate_flashcards() {
        let pipeline = ContentPipeline::new(MockGenerator);
        let cards = pipeline.generate_flashcards(&sample_standard(), 5).unwrap();

        assert_eq!(cards.len(), 5);
        for card in &cards {
            assert!(!card.front.is_empty());
            assert!(!card.back.is_empty());
            assert!(card.difficulty_permille <= 1000);
            assert!(!card.tags.is_empty());
        }
    }

    #[test]
    fn test_generate_curriculum_multiple() {
        let pipeline = ContentPipeline::new(MockGenerator);
        let standards = vec![
            sample_standard(),
            StandardInput {
                code: "3.NF.A.1".to_string(),
                description: "Understand a fraction 1/b as the quantity formed by 1 part"
                    .to_string(),
                grade_level: "Grade3".to_string(),
                domain: "Number & Operations - Fractions".to_string(),
                prerequisites: vec!["Equal parts".to_string()],
            },
        ];

        let lessons = pipeline.generate_curriculum(&standards).unwrap();
        assert_eq!(lessons.len(), 2);
        assert_eq!(lessons[0].standard_code, "3.OA.A.1");
        assert_eq!(lessons[1].standard_code, "3.NF.A.1");
    }

    #[test]
    fn placeholder_detector_rejects_non_empty_generic_fallbacks() {
        for text in [
            "Solve a Grade 3 Mathematics problem.",
            "Explain Grade 3 Mathematics.",
            "Answer: See worked solution",
            "What is important about Grade 3 Mathematics?",
            "Grade 3 mathematics content",
            "Grade 3 mathematics content.",
            "A key word in this topic",
            "A common mistake in Grade 3 Mathematics.",
            "The correct way.",
            "Answer.",
            "Let's explore this concept together! Mathematics is about finding patterns and solving problems. We'll start with something you already know and build from there. Remember: every expert was once a beginner.",
        ] {
            assert!(
                is_placeholder_content(text),
                "expected placeholder to be rejected: {text}"
            );
        }

        assert!(!is_placeholder_content(
            "Multiplication represents equal groups. Four groups of three objects make twelve objects: 3 + 3 + 3 + 3 = 12, so 4 × 3 = 12."
        ));
        assert!(!is_placeholder_content("42"));
        assert!(!is_placeholder_content(
            "A useful explanation tells learners how to check their answer."
        ));
        assert!(!is_placeholder_content(
            "Answers vary; any valid rectangle with perimeter 16 cm is acceptable."
        ));
    }

    #[test]
    fn practice_problem_validation_rejects_answer_key_collisions_and_duplicate_choices() {
        let mut problem = PracticeProblem {
            question: "What is 6 × 7?".to_string(),
            answer: "42".to_string(),
            difficulty_permille: 250,
            bloom_level: "Apply".to_string(),
            hints: vec!["Use a known fact.".to_string()],
            explanation: "6 groups of 7 make 42.".to_string(),
            distractors: vec!["36".to_string(), " 42 ".to_string(), "48".to_string()],
        };
        assert_eq!(
            validate_practice_problem(&problem),
            Err("the correct answer also appears as a distractor")
        );

        problem.distractors = vec!["36".to_string(), " 36 ".to_string(), "48".to_string()];
        assert_eq!(
            validate_practice_problem(&problem),
            Err("duplicate distractors were generated")
        );

        problem.distractors = vec!["36".to_string(), "45".to_string(), "48".to_string()];
        assert!(validate_practice_problem(&problem).is_ok());
    }

    #[test]
    fn practice_problem_validation_rejects_missing_fields_and_incomplete_choices() {
        let valid = PracticeProblem {
            question: "What is 6 × 7?".to_string(),
            answer: "42".to_string(),
            difficulty_permille: 250,
            bloom_level: "Apply".to_string(),
            hints: vec!["Use a known fact.".to_string()],
            explanation: "Six groups of seven make 42.".to_string(),
            distractors: vec!["36".to_string(), "45".to_string(), "48".to_string()],
        };

        let mut invalid = valid.clone();
        invalid.question.clear();
        assert_eq!(
            validate_practice_problem(&invalid),
            Err("question is empty")
        );

        let mut invalid = valid.clone();
        invalid.answer.clear();
        assert_eq!(
            validate_practice_problem(&invalid),
            Err("correct answer is empty")
        );

        let mut invalid = valid.clone();
        invalid.explanation.clear();
        assert_eq!(
            validate_practice_problem(&invalid),
            Err("explanation is empty")
        );

        let mut invalid = valid.clone();
        invalid.distractors.pop();
        assert_eq!(
            validate_practice_problem(&invalid),
            Err("exactly three distractors are required")
        );

        let mut invalid = valid;
        invalid.distractors[0] = " ".to_string();
        assert_eq!(
            validate_practice_problem(&invalid),
            Err("a distractor is empty")
        );
    }

    #[test]
    fn practice_problem_validation_rejects_equivalent_numbers_with_missing_units() {
        let problem = PracticeProblem {
            question: "How many more children chose soccer?".to_string(),
            answer: "3 children".to_string(),
            difficulty_permille: 250,
            bloom_level: "Apply".to_string(),
            hints: vec!["Compare the two counts.".to_string()],
            explanation: "8 minus 5 is 3 children.".to_string(),
            distractors: vec!["13".to_string(), "2".to_string(), "3".to_string()],
        };
        assert_eq!(
            validate_practice_problem(&problem),
            Err("a distractor is numerically equivalent to the correct answer")
        );

        assert!(numeric_answer_choices_collide("1/2", "2/4"));
        assert!(numeric_answer_choices_collide("20 cm", "20"));
        assert!(!numeric_answer_choices_collide("12 cm", "12 square units"));
    }

    #[test]
    fn generation_retries_practice_items_with_invalid_distractors() {
        use std::sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        };

        struct InvalidChoiceGenerator {
            calls: Arc<AtomicUsize>,
        }

        impl ContentGenerator for InvalidChoiceGenerator {
            fn generate(&self, _: &ContentChannels, _: &str, _: usize) -> GenerationOutput {
                self.calls.fetch_add(1, Ordering::SeqCst);
                GenerationOutput {
                    text: "What is 6 × 7?\\n42\\nSix groups of seven make 42.\\n36, 42, 48\\nUse a known fact.\\nCheck by repeated addition.".to_string(),
                    coherence: 0.99,
                    hallucination_flag: false,
                    veto_count: 0,
                }
            }
        }

        let calls = Arc::new(AtomicUsize::new(0));
        let pipeline = ContentPipeline::new(InvalidChoiceGenerator {
            calls: Arc::clone(&calls),
        })
        .with_max_retries(2);
        let result = pipeline.generate_practice_problem(&sample_standard(), 4, 250);
        assert!(matches!(
            result,
            Err(ContentGenError::InvalidPracticeProblem {
                attempts: 2,
                reason: _
            })
        ));
        assert_eq!(
            calls.load(Ordering::SeqCst),
            2,
            "each configured practice retry must make exactly one generator call"
        );
    }

    #[test]
    fn high_coherence_does_not_override_placeholder_content() {
        struct PlaceholderGenerator;

        impl ContentGenerator for PlaceholderGenerator {
            fn generate(&self, _: &ContentChannels, _: &str, _: usize) -> GenerationOutput {
                GenerationOutput {
                    text: "Solve a Grade 3 Mathematics problem.".to_string(),
                    coherence: 0.99,
                    hallucination_flag: false,
                    veto_count: 0,
                }
            }
        }

        let pipeline = ContentPipeline::new(PlaceholderGenerator).with_max_retries(2);
        assert!(pipeline.generate_lesson(&sample_standard()).is_err());
    }

    #[test]
    fn unknown_caps_standard_fails_closed_instead_of_publishing_mock_fallback() {
        let standard = StandardInput {
            code: "CAPS.Mathematics.Gr3.MUL".to_string(),
            description: "Times tables 2-10; multiply two-digit numbers by one digit; division"
                .to_string(),
            grade_level: "Grade3".to_string(),
            domain: "Mathematics".to_string(),
            prerequisites: vec!["Addition".to_string()],
        };

        let pipeline = ContentPipeline::new(MockGenerator);
        assert!(pipeline.generate_lesson(&standard).is_err());
    }

    #[test]
    fn test_quality_threshold_rejection() {
        /// Generator that always returns low quality.
        struct LowQualityGen;
        impl ContentGenerator for LowQualityGen {
            fn generate(&self, _: &ContentChannels, _: &str, _: usize) -> GenerationOutput {
                GenerationOutput {
                    text: "bad content".to_string(),
                    coherence: 0.2,
                    hallucination_flag: false,
                    veto_count: 0,
                }
            }
        }

        let pipeline = ContentPipeline::new(LowQualityGen);
        let result = pipeline.generate_lesson(&sample_standard());
        assert!(result.is_err());
    }

    #[test]
    fn test_hallucination_rejection() {
        /// Generator that always flags hallucination.
        struct HallucinatingGen;
        impl ContentGenerator for HallucinatingGen {
            fn generate(&self, _: &ContentChannels, _: &str, _: usize) -> GenerationOutput {
                GenerationOutput {
                    text: "hallucinated content".to_string(),
                    coherence: 0.9,
                    hallucination_flag: true,
                    veto_count: 1,
                }
            }
        }

        let pipeline = ContentPipeline::new(HallucinatingGen);
        let result = pipeline.generate_lesson(&sample_standard());
        assert!(result.is_err());
    }

    #[test]
    fn test_grade_ordinal_parsing() {
        assert_eq!(ContentPipeline::<MockGenerator>::grade_ordinal("PreK"), 0);
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("Kindergarten"),
            1
        );
        assert_eq!(ContentPipeline::<MockGenerator>::grade_ordinal("Grade1"), 2);
        assert_eq!(ContentPipeline::<MockGenerator>::grade_ordinal("Grade3"), 4);
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("Grade12"),
            13
        );
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("College"),
            14
        );
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("Undergraduate"),
            15
        );
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("Graduate"),
            16
        );
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("Doctoral"),
            17
        );
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("PostDoctoral"),
            18
        );
        assert_eq!(
            ContentPipeline::<MockGenerator>::grade_ordinal("Professional"),
            19
        );
        assert_eq!(ContentPipeline::<MockGenerator>::grade_ordinal("Adult"), 20);
    }

    #[test]
    fn test_infer_bloom_level() {
        assert_eq!(
            infer_bloom_level("3.OA.A.1", "Interpret products"),
            "Understand"
        );
        assert_eq!(
            infer_bloom_level("3.OA.A.3", "Solve word problems"),
            "Apply"
        );
        assert_eq!(
            infer_bloom_level("3.OA.C.7", "Fluently multiply and divide"),
            "Apply"
        );
        assert_eq!(
            infer_bloom_level(
                "3.OA.C.7",
                "Know from memory all products of two one-digit numbers"
            ),
            "Remember"
        );
        assert_eq!(
            infer_bloom_level("3.OA.A.4", "Determine the unknown"),
            "Apply"
        );
    }

    #[test]
    fn test_truncate() {
        assert_eq!(truncate("hello", 10), "hello");
        assert_eq!(truncate("hello world this is long", 10), "hello w...");
    }

    #[test]
    fn test_parse_vocabulary() {
        let text = "product: The result of multiplication | The product of 3 and 4 is 12.\n\
                    factor: A number being multiplied | 3 and 4 are factors of 12.";
        let vocab = parse_vocabulary(text);
        assert_eq!(vocab.len(), 2);
        assert_eq!(vocab[0].term, "product");
        assert!(vocab[0].definition.contains("result of multiplication"));
        assert!(vocab[0].example_usage.contains("product of 3"));
    }

    #[test]
    fn test_parse_misconceptions() {
        let text = "WRONG: Multiplication always makes numbers bigger.\n\
                    RIGHT: Multiplication by 1 gives the same number, and by 0 gives 0.\n\
                    WHY: Students only see multiplication with numbers greater than 1 in early examples.";
        let ms = parse_misconceptions(text);
        assert_eq!(ms.len(), 1);
        assert!(ms[0].misconception.contains("bigger"));
        assert!(ms[0].correction.contains("same number"));
        assert!(ms[0].why_students_think_this.contains("early examples"));
    }

    #[test]
    fn test_split_flashcard() {
        let (f, b) = split_flashcard("What is 3 x 4? | 12, because 3 groups of 4 = 12.");
        assert!(f.contains("3 x 4"));
        assert!(b.contains("12"));

        let (f2, b2) = split_flashcard("What is 3 x 4?\n12");
        assert!(f2.contains("3 x 4"));
        assert_eq!(b2, "12");
    }

    #[test]
    fn test_with_quality_threshold() {
        let pipeline = ContentPipeline::new(MockGenerator)
            .with_quality_threshold(0.9)
            .with_max_retries(5);
        // MockGenerator returns 0.85 coherence, so this should still pass
        // because the mock is designed to be acceptable
        let lesson = pipeline.generate_lesson(&sample_standard());
        // With 0.9 threshold and 0.85 mock coherence, this should fail
        assert!(lesson.is_err());
    }
}
