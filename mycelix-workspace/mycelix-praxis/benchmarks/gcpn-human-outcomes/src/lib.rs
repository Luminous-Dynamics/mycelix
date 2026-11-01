//! Synthetic human-usable warning-message evaluation.
//! This scores abstract task comprehension and recognition of message-stated
//! intent. It does not measure actual protective behaviour, trust, lives
//! saved, or real-world message performance. Fixture values are invented.

use std::collections::HashSet;

pub const FIXTURE: &str = include_str!("../fixtures/message-comprehension-v0.1.csv");
pub const HEADER: &str = "scenario_id,profile_id,message_variant,trials,accessible_trials,comprehension_correct,action_intent_correct,uncertainty_understood,false_certainty,median_response_seconds,p90_response_seconds";
pub const EXPECTED_FNV1A64: u64 = 0xf784_d7e5_9e19_e31f;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum InteractionProfile {
    StandardSmartphone, LowBandwidthText, ScreenReader,
    SignLanguageVideo, AudioOnlyRadio, LanguageMismatch,
}
impl InteractionProfile {
    pub const ALL: [Self; 6] = [
        Self::StandardSmartphone, Self::LowBandwidthText, Self::ScreenReader,
        Self::SignLanguageVideo, Self::AudioOnlyRadio, Self::LanguageMismatch,
    ];
    pub fn parse(v: &str) -> Result<Self, String> {
        match v {
            "standard_smartphone" => Ok(Self::StandardSmartphone),
            "low_bandwidth_text" => Ok(Self::LowBandwidthText),
            "screen_reader" => Ok(Self::ScreenReader),
            "sign_language_video" => Ok(Self::SignLanguageVideo),
            "audio_only_radio" => Ok(Self::AudioOnlyRadio),
            "language_mismatch" => Ok(Self::LanguageMismatch),
            _ => Err(format!("unknown interaction profile: {v}")),
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::StandardSmartphone => "standard_smartphone",
            Self::LowBandwidthText => "low_bandwidth_text",
            Self::ScreenReader => "screen_reader",
            Self::SignLanguageVideo => "sign_language_video",
            Self::AudioOnlyRadio => "audio_only_radio",
            Self::LanguageMismatch => "language_mismatch",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum MessageVariant { DenseTechnical, PlainLanguage, AccessibleMultimodal }
impl MessageVariant {
    pub const ALL: [Self; 3] = [Self::DenseTechnical, Self::PlainLanguage, Self::AccessibleMultimodal];
    pub fn parse(v: &str) -> Result<Self, String> {
        match v {
            "dense_technical" => Ok(Self::DenseTechnical),
            "plain_language" => Ok(Self::PlainLanguage),
            "accessible_multimodal" => Ok(Self::AccessibleMultimodal),
            _ => Err(format!("unknown message variant: {v}")),
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::DenseTechnical => "dense_technical",
            Self::PlainLanguage => "plain_language",
            Self::AccessibleMultimodal => "accessible_multimodal",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TrialCell {
    pub scenario_id: String,
    pub profile: InteractionProfile,
    pub variant: MessageVariant,
    pub trials: u32,
    pub accessible_trials: u32,
    pub comprehension_correct: u32,
    pub action_intent_correct: u32,
    pub uncertainty_understood: u32,
    pub false_certainty: u32,
    pub median_response_seconds: u32,
    pub p90_response_seconds: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ProfileSummary {
    pub profile: InteractionProfile,
    pub variant: MessageVariant,
    pub trials: u64,
    pub accessible_trials: u64,
    pub comprehension_correct: u64,
    pub action_intent_correct: u64,
    pub uncertainty_understood: u64,
    pub false_certainty: u64,
    pub accessibility_rate: f64,
    pub comprehension_given_access: Option<f64>,
    pub effective_comprehension: f64,
    pub action_intent_given_access: Option<f64>,
    pub effective_action_intent: f64,
    pub uncertainty_given_access: Option<f64>,
    pub effective_uncertainty_understanding: f64,
    pub false_certainty_given_access: Option<f64>,
    pub median_response_seconds: u32,
    pub p90_response_seconds: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct VariantSummary {
    pub variant: MessageVariant,
    pub profile_count: u32,
    pub trials: u64,
    pub accessible_trials: u64,
    pub comprehension_correct: u64,
    pub action_intent_correct: u64,
    pub uncertainty_understood: u64,
    pub false_certainty: u64,
    pub accessibility_rate: f64,
    pub comprehension_given_access: Option<f64>,
    pub effective_comprehension: f64,
    pub action_intent_given_access: Option<f64>,
    pub effective_action_intent: f64,
    pub uncertainty_given_access: Option<f64>,
    pub effective_uncertainty_understanding: f64,
    pub false_certainty_given_access: Option<f64>,
    pub minimum_profile_effective_comprehension: f64,
    pub maximum_profile_effective_comprehension: f64,
    pub effective_comprehension_profile_gap: f64,
    pub minimum_profile_effective_action_intent: f64,
    pub maximum_profile_p90_seconds: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EvaluationReport {
    pub rows: Vec<TrialCell>,
    pub profile_summaries: Vec<ProfileSummary>,
    pub variant_summaries: Vec<VariantSummary>,
    pub fixture_fingerprint: u64,
}
impl EvaluationReport {
    pub fn all_counting_invariants_hold(&self) -> bool {
        self.rows.iter().all(|r| {
            r.trials > 0 && r.accessible_trials <= r.trials
                && r.comprehension_correct <= r.accessible_trials
                && r.action_intent_correct <= r.accessible_trials
                && r.uncertainty_understood <= r.accessible_trials
                && r.false_certainty <= r.accessible_trials
                && r.p90_response_seconds >= r.median_response_seconds
        })
    }
    /// Frozen fixture integrity, not a real-world performance certification.
    pub fn fixture_oracle_matches(&self) -> bool {
        if self.rows.len() != 18 || self.profile_summaries.len() != 18
            || self.variant_summaries.len() != 3
            || self.fixture_fingerprint != EXPECTED_FNV1A64
            || !self.all_counting_invariants_hold() {
            return false;
        }
        self.variant_summaries.iter().any(|s| {
            s.variant == MessageVariant::AccessibleMultimodal
                && s.trials == 600 && s.accessible_trials == 530
                && s.comprehension_correct == 484 && s.action_intent_correct == 452
                && s.uncertainty_understood == 422 && s.false_certainty == 87
                && close(s.effective_comprehension, 484.0 / 600.0)
                && close(s.minimum_profile_effective_comprehension, 0.55)
                && close(s.effective_comprehension_profile_gap, 0.39)
                && s.maximum_profile_p90_seconds == 34
        })
    }
}
fn close(a: f64, b: f64) -> bool { (a - b).abs() <= 1e-12 }
fn parse_u32(v: &str, field: &str, line: usize) -> Result<u32, String> {
    v.parse::<u32>().map_err(|_| format!("line {line}: invalid {field}: {v}"))
}

pub fn parse_fixture(input: &str) -> Result<Vec<TrialCell>, String> {
    let mut lines = input.lines();
    if lines.next() != Some(HEADER) { return Err("fixture header mismatch".into()); }
    let mut rows = Vec::new();
    let mut seen = HashSet::new();
    for (i, line) in lines.enumerate() {
        let n = i + 2;
        if line.trim().is_empty() { return Err(format!("line {n}: empty rows are forbidden")); }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 11 { return Err(format!("line {n}: expected 11 columns")); }
        if f[0] != "core_message_task_v1" {
            return Err(format!("line {n}: inconsistent semantic task identity"));
        }
        let profile = InteractionProfile::parse(f[1])?;
        let variant = MessageVariant::parse(f[2])?;
        if !seen.insert((profile, variant)) { return Err(format!("line {n}: duplicate profile/variant")); }
        let row = TrialCell {
            scenario_id: f[0].to_string(), profile, variant,
            trials: parse_u32(f[3], "trials", n)?,
            accessible_trials: parse_u32(f[4], "accessible_trials", n)?,
            comprehension_correct: parse_u32(f[5], "comprehension_correct", n)?,
            action_intent_correct: parse_u32(f[6], "action_intent_correct", n)?,
            uncertainty_understood: parse_u32(f[7], "uncertainty_understood", n)?,
            false_certainty: parse_u32(f[8], "false_certainty", n)?,
            median_response_seconds: parse_u32(f[9], "median_response_seconds", n)?,
            p90_response_seconds: parse_u32(f[10], "p90_response_seconds", n)?,
        };
        if row.trials == 0 || row.accessible_trials > row.trials
            || row.comprehension_correct > row.accessible_trials
            || row.action_intent_correct > row.accessible_trials
            || row.uncertainty_understood > row.accessible_trials
            || row.false_certainty > row.accessible_trials
            || row.p90_response_seconds < row.median_response_seconds {
            return Err(format!("line {n}: counting or latency invariant failed"));
        }
        rows.push(row);
    }
    if rows.len() != 18 { return Err(format!("expected 18 rows, got {}", rows.len())); }
    for p in InteractionProfile::ALL {
        for v in MessageVariant::ALL {
            if !seen.contains(&(p, v)) {
                return Err(format!("missing profile/variant cell: {}/{}", p.label(), v.label()));
            }
        }
    }
    Ok(rows)
}

fn ratio(n: u64, d: u64) -> Option<f64> {
    if d == 0 { None } else { Some(n as f64 / d as f64) }
}
fn profile_summary(r: &TrialCell) -> ProfileSummary {
    let t = r.trials as u64;
    let a = r.accessible_trials as u64;
    let c = r.comprehension_correct as u64;
    let i = r.action_intent_correct as u64;
    let u = r.uncertainty_understood as u64;
    let f = r.false_certainty as u64;
    ProfileSummary {
        profile: r.profile, variant: r.variant, trials: t, accessible_trials: a,
        comprehension_correct: c, action_intent_correct: i,
        uncertainty_understood: u, false_certainty: f,
        accessibility_rate: ratio(a, t).unwrap_or(0.0),
        comprehension_given_access: ratio(c, a),
        effective_comprehension: ratio(c, t).unwrap_or(0.0),
        action_intent_given_access: ratio(i, a),
        effective_action_intent: ratio(i, t).unwrap_or(0.0),
        uncertainty_given_access: ratio(u, a),
        effective_uncertainty_understanding: ratio(u, t).unwrap_or(0.0),
        false_certainty_given_access: ratio(f, a),
        median_response_seconds: r.median_response_seconds,
        p90_response_seconds: r.p90_response_seconds,
    }
}
fn variant_summary(v: MessageVariant, rows: &[TrialCell], ps: &[ProfileSummary]) -> VariantSummary {
    let selected: Vec<&TrialCell> = rows.iter().filter(|r| r.variant == v).collect();
    let selected_p: Vec<&ProfileSummary> = ps.iter().filter(|s| s.variant == v).collect();
    let sum = |get: fn(&TrialCell) -> u32| -> u64 { selected.iter().map(|r| get(r) as u64).sum() };
    let trials = sum(|r| r.trials);
    let accessible = sum(|r| r.accessible_trials);
    let comprehension = sum(|r| r.comprehension_correct);
    let action = sum(|r| r.action_intent_correct);
    let uncertainty = sum(|r| r.uncertainty_understood);
    let false_certainty = sum(|r| r.false_certainty);
    let min_c = selected_p.iter().map(|s| s.effective_comprehension).fold(f64::INFINITY, f64::min);
    let max_c = selected_p.iter().map(|s| s.effective_comprehension).fold(f64::NEG_INFINITY, f64::max);
    let min_a = selected_p.iter().map(|s| s.effective_action_intent).fold(f64::INFINITY, f64::min);
    let max_p90 = selected_p.iter().map(|s| s.p90_response_seconds).max().unwrap_or(0);
    VariantSummary {
        variant: v, profile_count: selected_p.len() as u32,
        trials, accessible_trials: accessible, comprehension_correct: comprehension,
        action_intent_correct: action, uncertainty_understood: uncertainty, false_certainty,
        accessibility_rate: ratio(accessible, trials).unwrap_or(0.0),
        comprehension_given_access: ratio(comprehension, accessible),
        effective_comprehension: ratio(comprehension, trials).unwrap_or(0.0),
        action_intent_given_access: ratio(action, accessible),
        effective_action_intent: ratio(action, trials).unwrap_or(0.0),
        uncertainty_given_access: ratio(uncertainty, accessible),
        effective_uncertainty_understanding: ratio(uncertainty, trials).unwrap_or(0.0),
        false_certainty_given_access: ratio(false_certainty, accessible),
        minimum_profile_effective_comprehension: min_c,
        maximum_profile_effective_comprehension: max_c,
        effective_comprehension_profile_gap: max_c - min_c,
        minimum_profile_effective_action_intent: min_a,
        maximum_profile_p90_seconds: max_p90,
    }
}
pub fn evaluate(rows: Vec<TrialCell>, fingerprint: u64) -> EvaluationReport {
    let profiles: Vec<ProfileSummary> = rows.iter().map(profile_summary).collect();
    let variants = MessageVariant::ALL.iter().map(|v| variant_summary(*v, &rows, &profiles)).collect();
    EvaluationReport { rows, profile_summaries: profiles, variant_summaries: variants, fixture_fingerprint: fingerprint }
}
pub fn fixture_report() -> Result<EvaluationReport, String> {
    let rows = parse_fixture(FIXTURE)?;
    let report = evaluate(rows, fnv1a64(FIXTURE.as_bytes()));
    if !report.fixture_oracle_matches() { return Err("frozen fixture oracle did not reproduce".into()); }
    Ok(report)
}
pub fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf29ce484222325_u64;
    for b in bytes { h ^= *b as u64; h = h.wrapping_mul(0x100000001b3); }
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frozen_fixture_and_cross_product_oracle_reproduce() {
        let r = fixture_report().unwrap();
        assert_eq!(r.rows.len(), 18);
        assert_eq!(r.profile_summaries.len(), 18);
        assert_eq!(r.variant_summaries.len(), 3);
        assert_eq!(r.fixture_fingerprint, EXPECTED_FNV1A64);
        assert!(r.fixture_oracle_matches());
        assert!(r.all_counting_invariants_hold());
    }

    #[test]
    fn inaccessible_trials_remain_in_effective_denominator() {
        let r = fixture_report().unwrap();
        let s = r.variant_summaries.iter().find(|s| s.variant == MessageVariant::AccessibleMultimodal).unwrap();
        assert_eq!(s.accessible_trials, 530);
        assert_eq!(s.comprehension_correct, 484);
        assert!(close(s.effective_comprehension, 484.0 / 600.0));
        assert!(close(s.comprehension_given_access.unwrap(), 484.0 / 530.0));
        assert!(s.effective_comprehension < s.comprehension_given_access.unwrap());
    }

    #[test]
    fn pooled_score_does_not_hide_worst_profile_or_latency_tail() {
        let r = fixture_report().unwrap();
        let s = r.variant_summaries.iter().find(|s| s.variant == MessageVariant::AccessibleMultimodal).unwrap();
        assert!(close(s.minimum_profile_effective_comprehension, 0.55));
        assert!(close(s.effective_comprehension_profile_gap, 0.39));
        assert_eq!(s.maximum_profile_p90_seconds, 34);
    }

    #[test]
    fn negative_control_rejects_outcome_above_accessible_denominator() {
        let old = "core_message_task_v1,standard_smartphone,accessible_multimodal,100,100,94,91,88,5,9,17";
        let new = "core_message_task_v1,standard_smartphone,accessible_multimodal,100,100,101,91,88,5,9,17";
        let mutated = FIXTURE.replace(old, new);
        assert_ne!(mutated, FIXTURE);
        assert!(parse_fixture(&mutated).is_err());
    }

    #[test]
    fn negative_control_rejects_duplicate_profile_variant_pair() {
        let row = FIXTURE.lines().nth(1).unwrap();
        let mutated = format!("{FIXTURE}{row}\n");
        assert_ne!(mutated, FIXTURE);
        assert!(parse_fixture(&mutated).is_err());
    }

    #[test]
    fn negative_control_rejects_p90_below_median() {
        let mutated = FIXTURE.replace("100,100,94,91,88,5,9,17", "100,100,94,91,88,5,10,9");
        assert_ne!(mutated, FIXTURE);
        assert!(parse_fixture(&mutated).is_err());
    }

    #[test]
    fn negative_control_rejects_semantic_task_mismatch() {
        let mutated = FIXTURE.replace("core_message_task_v1,language_mismatch,accessible_multimodal", "other_task,language_mismatch,accessible_multimodal");
        assert_ne!(mutated, FIXTURE);
        assert!(parse_fixture(&mutated).is_err());
    }

    #[test]
    fn schema_has_no_personal_or_demographic_columns() {
        assert_eq!(FIXTURE.lines().next(), Some(HEADER));
        assert_eq!(HEADER.split(',').count(), 11);
    }
}
