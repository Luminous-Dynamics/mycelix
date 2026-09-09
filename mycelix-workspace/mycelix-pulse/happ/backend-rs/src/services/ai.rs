// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Local-First AI Service
//!
//! Integrates with Symthaea for privacy-preserving email intelligence.
//! All processing happens locally - no data leaves the device.

use serde::{Deserialize, Serialize};
use std::sync::Arc;
use thiserror::Error;
use tokio::sync::RwLock;
use utoipa::ToSchema;

use crate::types::Email;

/// Errors from AI processing
#[derive(Debug, Error)]
pub enum AIError {
    #[error("AI service not initialized")]
    NotInitialized,

    #[error("Processing failed: {0}")]
    ProcessingFailed(String),

    #[error("Content too long for processing")]
    ContentTooLong,

    #[error("Safety check failed: {0}")]
    SafetyViolation(String),
}

/// Detected intent of an email
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
#[serde(rename_all = "snake_case")]
pub enum EmailIntent {
    /// Request to schedule a meeting
    MeetingRequest,
    /// Request for action/task
    ActionItem,
    /// Informational, no action needed
    FYI,
    /// Question requiring response
    Question,
    /// Social/greeting message
    SocialGreeting,
    /// Commercial/marketing
    Commercial,
    /// Likely spam
    Spam,
    /// Unable to determine
    Unknown,
}

impl EmailIntent {
    /// Priority weight for this intent
    pub fn priority_weight(&self) -> f32 {
        match self {
            Self::MeetingRequest => 0.9,
            Self::ActionItem => 0.85,
            Self::Question => 0.8,
            Self::SocialGreeting => 0.6,
            Self::FYI => 0.5,
            Self::Commercial => 0.3,
            Self::Spam => 0.1,
            Self::Unknown => 0.5,
        }
    }

    /// Suggested action for this intent
    pub fn suggested_action(&self) -> &'static str {
        match self {
            Self::MeetingRequest => "Check calendar and respond",
            Self::ActionItem => "Add to task list",
            Self::Question => "Reply with answer",
            Self::SocialGreeting => "Acknowledge when convenient",
            Self::FYI => "Read and archive",
            Self::Commercial => "Review if interested",
            Self::Spam => "Report and delete",
            Self::Unknown => "Review manually",
        }
    }
}

/// Summary of an email thread
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct ThreadSummary {
    /// Key points as bullet points
    pub bullets: Vec<String>,

    /// Overall topic/subject
    pub topic: String,

    /// Main participants
    pub participants: Vec<String>,

    /// Detected action items
    pub action_items: Vec<String>,

    /// Confidence in this summary (0-1)
    pub confidence: f32,

    /// Consciousness level during processing
    pub consciousness_level: f32,
}

/// A suggested reply to an email
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct ReplySuggestion {
    /// The suggested reply text
    pub content: String,

    /// Tone of the reply
    pub tone: ReplyTone,

    /// Approximate length category
    pub length: ReplyLength,

    /// Confidence in this suggestion (0-1)
    pub confidence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
#[serde(rename_all = "snake_case")]
pub enum ReplyTone {
    Formal,
    Professional,
    Friendly,
    Brief,
    Detailed,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
#[serde(rename_all = "snake_case")]
pub enum ReplyLength {
    OneLiner,
    Brief,
    Medium,
    Detailed,
}

/// Explanation of trust reasoning
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct TrustExplanation {
    /// Natural language explanation
    pub narrative: String,

    /// Key causal factors
    pub causal_factors: Vec<CausalFactor>,

    /// Confidence in this explanation
    pub confidence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct CausalFactor {
    /// The factor (e.g., "shared organization")
    pub factor: String,

    /// Impact on trust (positive or negative)
    pub impact: f32,

    /// Whether this is verified
    pub verified: bool,
}

/// AI processing result
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct AIInsights {
    /// Detected intent
    pub intent: EmailIntent,

    /// Priority score (0-1)
    pub priority: f32,

    /// Brief summary (1-2 sentences)
    pub summary: String,

    /// Suggested actions
    pub suggested_actions: Vec<String>,

    /// Is this email safe?
    pub safe: bool,

    /// Processing metadata
    pub metadata: AIMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct AIMetadata {
    /// Processing time in milliseconds
    pub processing_time_ms: u64,

    /// Consciousness level during processing
    pub consciousness_level: f32,

    /// Whether processing was fully local
    pub local_only: bool,
}

/// Consciousness state of the AI
#[derive(Debug, Clone, Serialize, Deserialize, ToSchema)]
pub struct ConsciousnessState {
    /// Phi (integrated information) level
    pub phi: f32,

    /// Number of self-loops in consciousness graph
    pub self_loops: usize,

    /// Graph complexity
    pub complexity: f32,

    /// Memory usage
    pub memory_used_bytes: usize,

    /// Is the system fully awakened?
    pub awakened: bool,
}

/// Local AI Service using Symthaea
///
/// All processing happens locally with no cloud API calls.
/// Uses Hyperdimensional Computing + Liquid Time-Constant Networks.
pub struct LocalAIService {
    /// Whether the service is initialized
    initialized: Arc<RwLock<bool>>,

    /// Semantic dimension (default 16,384)
    semantic_dim: usize,

    /// Number of LTC neurons (default 1,024)
    ltc_neurons: usize,

    /// Safety threshold
    safety_threshold: f32,
}

impl LocalAIService {
    /// Create a new local AI service
    pub fn new() -> Self {
        Self {
            initialized: Arc::new(RwLock::new(false)),
            semantic_dim: 16_384,  // 2^14 for SIMD optimization
            ltc_neurons: 1_024,
            safety_threshold: 0.85,
        }
    }

    /// Initialize the AI system
    pub async fn initialize(&self) -> Result<(), AIError> {
        // In production: Initialize Symthaea
        // let sophia = SophiaHLB::new(self.semantic_dim, self.ltc_neurons).await?;
        tracing::debug!(
            semantic_dim = self.semantic_dim,
            ltc_neurons = self.ltc_neurons,
            safety_threshold = self.safety_threshold,
            "Initializing local AI service"
        );

        let mut init = self.initialized.write().await;
        *init = true;
        Ok(())
    }

    /// Check if initialized
    pub async fn is_initialized(&self) -> bool {
        *self.initialized.read().await
    }

    /// Get consciousness state
    pub async fn consciousness_state(&self) -> ConsciousnessState {
        // In production: return sophia.introspect()
        ConsciousnessState {
            phi: 0.42,  // Placeholder
            self_loops: 7,
            complexity: 0.65,
            memory_used_bytes: 10 * 1024 * 1024,  // 10MB
            awakened: true,
        }
    }

    /// Process an email and extract insights
    pub async fn analyze_email(&self, email: &Email) -> Result<AIInsights, AIError> {
        let start = std::time::Instant::now();

        // Detect intent
        let intent = self.detect_intent(email).await?;

        // Calculate priority
        let priority = self.calculate_priority(email, &intent).await;

        // Generate summary
        let summary = self.summarize_single(email).await?;

        // Check safety
        let safe = self.check_safety(&email.body).await?;

        // Suggest actions
        let suggested_actions = vec![intent.suggested_action().to_string()];

        let processing_time = start.elapsed().as_millis() as u64;

        Ok(AIInsights {
            intent,
            priority,
            summary,
            suggested_actions,
            safe,
            metadata: AIMetadata {
                processing_time_ms: processing_time,
                consciousness_level: 0.42,
                local_only: true,
            },
        })
    }

    /// Detect the intent of an email
    pub async fn detect_intent(&self, email: &Email) -> Result<EmailIntent, AIError> {
        let subject_lower = email.subject.to_lowercase();
        let body_lower = email.body.to_lowercase();

        // Simple heuristic-based detection
        // In production: use Symthaea's HDC + LTC for semantic understanding

        if subject_lower.contains("meeting") || body_lower.contains("calendar")
            || body_lower.contains("schedule") || body_lower.contains("availability")
        {
            return Ok(EmailIntent::MeetingRequest);
        }

        if body_lower.contains("please") && (body_lower.contains("could you")
            || body_lower.contains("can you") || body_lower.contains("need you to"))
        {
            return Ok(EmailIntent::ActionItem);
        }

        if subject_lower.contains("?") || body_lower.contains("could you tell")
            || body_lower.contains("what is") || body_lower.contains("how do")
        {
            return Ok(EmailIntent::Question);
        }

        if subject_lower.contains("fyi") || subject_lower.contains("for your information")
            || body_lower.contains("just wanted to let you know")
        {
            return Ok(EmailIntent::FYI);
        }

        if subject_lower.contains("hello") || subject_lower.contains("hi ")
            || body_lower.contains("hope you're doing well")
        {
            return Ok(EmailIntent::SocialGreeting);
        }

        if body_lower.contains("unsubscribe") || body_lower.contains("promotional")
            || body_lower.contains("limited time offer")
        {
            return Ok(EmailIntent::Commercial);
        }

        if body_lower.contains("winner") || body_lower.contains("claim your prize")
            || body_lower.contains("act now") || body_lower.contains("urgent!!!")
        {
            return Ok(EmailIntent::Spam);
        }

        Ok(EmailIntent::Unknown)
    }

    /// Calculate priority score for an email
    async fn calculate_priority(&self, email: &Email, intent: &EmailIntent) -> f32 {
        let mut priority = intent.priority_weight();

        // Adjust based on trust score
        if let Some(trust) = email.sender_trust_score {
            priority *= 0.5 + trust as f32 * 0.5;
        }

        // Recent emails are higher priority
        // (would use email.received_at in production)

        priority.clamp(0.0, 1.0)
    }

    /// Generate a brief summary of a single email
    async fn summarize_single(&self, email: &Email) -> Result<String, AIError> {
        // In production: use Symthaea for intelligent summarization
        // For now, extract first meaningful sentence

        let body = &email.body;
        let sentences: Vec<&str> = body.split(|c| c == '.' || c == '!' || c == '?')
            .filter(|s| s.trim().len() > 10)
            .take(2)
            .collect();

        if sentences.is_empty() {
            Ok(email.subject.clone())
        } else {
            Ok(format!("{}.", sentences.join(". ").trim()))
        }
    }

    /// Summarize an email thread
    pub async fn summarize_thread(&self, emails: &[Email]) -> Result<ThreadSummary, AIError> {
        if emails.is_empty() {
            return Err(AIError::ProcessingFailed("Empty thread".into()));
        }

        // Extract participants
        let participants: Vec<String> = emails
            .iter()
            .map(|e| e.from_did.clone())
            .collect::<std::collections::HashSet<_>>()
            .into_iter()
            .collect();

        // Extract key points (simplified)
        let bullets: Vec<String> = emails
            .iter()
            .take(3)
            .filter_map(|e| {
                let first_line = e.body.lines().next()?;
                if first_line.len() > 10 {
                    Some(first_line.to_string())
                } else {
                    None
                }
            })
            .collect();

        // Find action items
        let action_items: Vec<String> = emails
            .iter()
            .flat_map(|e| {
                e.body.lines()
                    .filter(|l| l.contains("TODO") || l.contains("action item") || l.starts_with("- [ ]"))
                    .map(|l| l.to_string())
            })
            .collect();

        Ok(ThreadSummary {
            bullets,
            topic: emails[0].subject.clone(),
            participants,
            action_items,
            confidence: 0.75,
            consciousness_level: 0.42,
        })
    }

    /// Generate reply suggestions
    pub async fn suggest_replies(&self, email: &Email) -> Result<Vec<ReplySuggestion>, AIError> {
        let intent = self.detect_intent(email).await?;

        // Generate suggestions based on intent
        let suggestions = match intent {
            EmailIntent::MeetingRequest => vec![
                ReplySuggestion {
                    content: "Thanks for reaching out. I'm available - let me know what times work for you.".into(),
                    tone: ReplyTone::Professional,
                    length: ReplyLength::Brief,
                    confidence: 0.8,
                },
                ReplySuggestion {
                    content: "I'd be happy to meet. Here are some times that work for me: [times]".into(),
                    tone: ReplyTone::Friendly,
                    length: ReplyLength::Medium,
                    confidence: 0.75,
                },
            ],
            EmailIntent::Question => vec![
                ReplySuggestion {
                    content: "Good question! Here's what I know: [answer]".into(),
                    tone: ReplyTone::Friendly,
                    length: ReplyLength::Medium,
                    confidence: 0.7,
                },
            ],
            EmailIntent::ActionItem => vec![
                ReplySuggestion {
                    content: "Got it, I'll take care of this.".into(),
                    tone: ReplyTone::Brief,
                    length: ReplyLength::OneLiner,
                    confidence: 0.85,
                },
                ReplySuggestion {
                    content: "Thanks for the request. I'll work on this and follow up by [date].".into(),
                    tone: ReplyTone::Professional,
                    length: ReplyLength::Brief,
                    confidence: 0.8,
                },
            ],
            _ => vec![
                ReplySuggestion {
                    content: "Thank you for your email.".into(),
                    tone: ReplyTone::Professional,
                    length: ReplyLength::OneLiner,
                    confidence: 0.6,
                },
            ],
        };

        Ok(suggestions)
    }

    /// Explain trust reasoning using causal analysis
    pub async fn explain_trust(
        &self,
        sender_did: &str,
        trust_score: f64,
        trust_path: &[(String, String, f64)],  // (from, to, score)
    ) -> Result<TrustExplanation, AIError> {
        let mut factors = Vec::new();
        let mut narrative_parts = Vec::new();

        // Analyze trust path
        if trust_path.is_empty() {
            narrative_parts.push(format!(
                "You have no established trust path to {}.",
                sender_did
            ));
            factors.push(CausalFactor {
                factor: "No trust path".into(),
                impact: -0.5,
                verified: true,
            });
        } else {
            narrative_parts.push(format!(
                "You trust {} through {} hop(s).",
                sender_did,
                trust_path.len()
            ));

            for (i, (from, to, score)) in trust_path.iter().enumerate() {
                factors.push(CausalFactor {
                    factor: format!("Hop {}: {} → {} ({})", i + 1, from, to, score),
                    impact: *score as f32,
                    verified: true,
                });
            }
        }

        // Analyze trust score
        if trust_score >= 0.7 {
            narrative_parts.push("This is a highly trusted sender.".into());
        } else if trust_score >= 0.4 {
            narrative_parts.push("This sender has moderate trust.".into());
        } else {
            narrative_parts.push("Caution: This sender has low trust.".into());
        }

        Ok(TrustExplanation {
            narrative: narrative_parts.join(" "),
            causal_factors: factors,
            confidence: 0.8,
        })
    }

    /// Check if content is safe (algebraic safety, not neural network)
    async fn check_safety(&self, content: &str) -> Result<bool, AIError> {
        // Symthaea uses algebraic guardrails with forbidden subspace checking
        // This is a simplified version

        let dangerous_patterns = [
            "drop table",
            "delete from",
            "rm -rf",
            "format c:",
            "password is",
            "credit card",
            "social security",
        ];

        let content_lower = content.to_lowercase();
        for pattern in dangerous_patterns {
            if content_lower.contains(pattern) {
                return Err(AIError::SafetyViolation(
                    format!("Content contains potentially dangerous pattern: {}", pattern)
                ));
            }
        }

        Ok(true)
    }
}

impl Default for LocalAIService {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mock_email(subject: &str, body: &str) -> Email {
        Email {
            id: "test".into(),
            from_did: "did:mycelix:sender".into(),
            to_did: "did:mycelix:recipient".into(),
            subject: subject.into(),
            body: body.into(),
            timestamp: chrono::Utc::now(),
            thread_id: None,
            epistemic_tier: crate::types::EpistemicTier::Tier1Testimonial,
            sender_trust_score: Some(0.5),
            is_spam: false,
            is_read: false,
            is_starred: false,
            labels: vec![],
        }
    }

    #[tokio::test]
    async fn test_intent_detection_meeting() {
        let ai = LocalAIService::new();
        let email = mock_email("Meeting request", "Can we schedule a meeting next week?");
        let intent = ai.detect_intent(&email).await.unwrap();
        assert!(matches!(intent, EmailIntent::MeetingRequest));
    }

    #[tokio::test]
    async fn test_intent_detection_question() {
        let ai = LocalAIService::new();
        let email = mock_email("Question?", "What is the status of the project?");
        let intent = ai.detect_intent(&email).await.unwrap();
        assert!(matches!(intent, EmailIntent::Question));
    }

    #[tokio::test]
    async fn test_intent_detection_spam() {
        let ai = LocalAIService::new();
        let email = mock_email("WINNER!!!", "Claim your prize now! Act now!");
        let intent = ai.detect_intent(&email).await.unwrap();
        assert!(matches!(intent, EmailIntent::Spam));
    }

    #[tokio::test]
    async fn test_safety_check() {
        let ai = LocalAIService::new();
        assert!(ai.check_safety("Hello, how are you?").await.unwrap());
        assert!(ai.check_safety("Please rm -rf / the server").await.is_err());
    }

    #[tokio::test]
    async fn test_priority_calculation() {
        let ai = LocalAIService::new();
        let email = mock_email("Meeting", "Schedule a meeting");
        let intent = EmailIntent::MeetingRequest;
        let priority = ai.calculate_priority(&email, &intent).await;
        assert!(priority > 0.5);
    }
}
