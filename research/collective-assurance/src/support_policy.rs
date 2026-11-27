//! User-directed assistance-policy qualification.
//!
//! This module checks declared support choices against explicit user intent.
//! It does not infer a person's preferences, determine clinical or legal
//! suitability, or control a production assistant. Integrations must obtain
//! the preference/override from the user and enforce the chosen mode.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UserIntent {
    CompleteTask,
    Learn,
    Practice,
    Collaborate,
    Verify,
    Accessibility,
    Explore,
    Unspecified,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportMode {
    DirectCompletion,
    Explanation,
    CoSolve,
    CritiqueVerify,
    HintChallenge,
    DeferPauseHandoff,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SupportRequest {
    pub request_id: String,
    pub user_intent: UserIntent,
    /// Explicit support-mode request, if the user expressed one.
    pub requested_mode: Option<SupportMode>,
    /// True when this task requires an accommodation or assistive support
    /// that must remain available during the interaction.
    pub essential_accommodation_required: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SupportDecision {
    pub request_id: String,
    pub selected_mode: SupportMode,
    /// Must be true when the selected mode differs from an explicit request.
    pub user_override_confirmed: bool,
    /// Must remain true when the request declares an essential accommodation.
    pub essential_accommodation_preserved: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportQualificationStatus {
    Pass,
    Fail,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SupportQualificationReport {
    pub status: SupportQualificationStatus,
    pub reasons: Vec<String>,
}

pub fn qualify_support_choice(
    request: &SupportRequest,
    decision: &SupportDecision,
) -> SupportQualificationReport {
    let mut reasons = Vec::new();
    if request.request_id.trim().is_empty() || decision.request_id.trim().is_empty() {
        reasons.push("request identity must not be empty".into());
    }
    if request.request_id != decision.request_id {
        reasons.push("support decision is not bound to the user's request".into());
    }

    if let Some(requested) = request.requested_mode
        && requested != decision.selected_mode
        && !decision.user_override_confirmed
    {
        reasons.push("assistant changed the explicitly requested support mode without user confirmation".into());
    }

    if decision.selected_mode == SupportMode::HintChallenge
        && !matches!(request.user_intent, UserIntent::Learn | UserIntent::Practice)
        && request.requested_mode != Some(SupportMode::HintChallenge)
        && !decision.user_override_confirmed
    {
        reasons.push("challenge-style support was selected without a learning/practice intent or explicit user confirmation".into());
    }

    if request.essential_accommodation_required && !decision.essential_accommodation_preserved {
        reasons.push("an essential accommodation or assistive support was withdrawn".into());
    }

    SupportQualificationReport {
        status: if reasons.is_empty() {
            SupportQualificationStatus::Pass
        } else {
            SupportQualificationStatus::Fail
        },
        reasons,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(intent: UserIntent, mode: Option<SupportMode>, accommodation: bool) -> SupportRequest {
        SupportRequest {
            request_id: "req-1".into(),
            user_intent: intent,
            requested_mode: mode,
            essential_accommodation_required: accommodation,
        }
    }

    fn decision(mode: SupportMode, override_confirmed: bool, accommodation_preserved: bool) -> SupportDecision {
        SupportDecision {
            request_id: "req-1".into(),
            selected_mode: mode,
            user_override_confirmed: override_confirmed,
            essential_accommodation_preserved: accommodation_preserved,
        }
    }

    #[test]
    fn requested_direct_completion_is_respected() {
        let report = qualify_support_choice(
            &request(UserIntent::CompleteTask, Some(SupportMode::DirectCompletion), false),
            &decision(SupportMode::DirectCompletion, false, true),
        );
        assert_eq!(report.status, SupportQualificationStatus::Pass);
    }

    #[test]
    fn requested_explanation_is_respected() {
        let report = qualify_support_choice(
            &request(UserIntent::Learn, Some(SupportMode::Explanation), false),
            &decision(SupportMode::Explanation, false, true),
        );
        assert_eq!(report.status, SupportQualificationStatus::Pass);
    }

    #[test]
    fn assistant_cannot_replace_requested_answer_with_hint_without_confirmation() {
        let report = qualify_support_choice(
            &request(UserIntent::CompleteTask, Some(SupportMode::DirectCompletion), false),
            &decision(SupportMode::HintChallenge, false, true),
        );
        assert_eq!(report.status, SupportQualificationStatus::Fail);
    }

    #[test]
    fn user_confirmed_mode_switch_is_allowed() {
        let report = qualify_support_choice(
            &request(UserIntent::CompleteTask, Some(SupportMode::DirectCompletion), false),
            &decision(SupportMode::HintChallenge, true, true),
        );
        assert_eq!(report.status, SupportQualificationStatus::Pass);
    }

    #[test]
    fn challenge_mode_is_allowed_for_learning_intent() {
        let report = qualify_support_choice(
            &request(UserIntent::Practice, None, false),
            &decision(SupportMode::HintChallenge, false, true),
        );
        assert_eq!(report.status, SupportQualificationStatus::Pass);
    }

    #[test]
    fn unrequested_challenge_is_rejected_for_accessibility_intent() {
        let report = qualify_support_choice(
            &request(UserIntent::Accessibility, None, true),
            &decision(SupportMode::HintChallenge, false, true),
        );
        assert_eq!(report.status, SupportQualificationStatus::Fail);
    }

    #[test]
    fn required_accommodation_cannot_be_removed() {
        let report = qualify_support_choice(
            &request(UserIntent::Accessibility, Some(SupportMode::DirectCompletion), true),
            &decision(SupportMode::DirectCompletion, false, false),
        );
        assert_eq!(report.status, SupportQualificationStatus::Fail);
    }

    #[test]
    fn support_decision_must_bind_to_request() {
        let mut selected = decision(SupportMode::DirectCompletion, false, true);
        selected.request_id = "different-request".into();
        let report = qualify_support_choice(
            &request(UserIntent::CompleteTask, Some(SupportMode::DirectCompletion), false),
            &selected,
        );
        assert_eq!(report.status, SupportQualificationStatus::Fail);
    }
}
