// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
// Verifiable Credential test fixtures
//
// Generators for W3C Verifiable Credentials (VCs) for testing

use super::{random_hex, random_string, seeded_rng, DEFAULT_SEED};
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VerifiableCredential {
    #[serde(rename = "@context")]
    pub context: Vec<String>,
    #[serde(rename = "type")]
    pub credential_type: Vec<String>,
    pub issuer: String,
    pub issuance_date: String,
    pub expiration_date: Option<String>,
    pub credential_subject: CredentialSubject,
    pub proof: Option<Proof>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CredentialSubject {
    pub id: String,
    pub course_id: String,
    pub model_id: Option<String>,
    pub score: f64,
    pub score_band: String,
    pub skills: Vec<String>,
    pub completion_date: String,
    #[serde(flatten)]
    pub additional: std::collections::HashMap<String, Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Proof {
    #[serde(rename = "type")]
    pub proof_type: String,
    pub created: String,
    pub verification_method: String,
    pub proof_purpose: String,
    pub proof_value: String,
}

/// Credential builder for flexible test data creation
pub struct CredentialBuilder {
    seed: u64,
    credential_type: String,
    score: f64,
    include_proof: bool,
    include_expiration: bool,
    skills: Vec<String>,
}

impl Default for CredentialBuilder {
    fn default() -> Self {
        Self {
            seed: DEFAULT_SEED,
            credential_type: "EduAchievementCredential".to_string(),
            score: 85.0,
            include_proof: true,
            include_expiration: false,
            skills: vec![
                "Problem Solving".to_string(),
                "Critical Thinking".to_string(),
            ],
        }
    }
}

impl CredentialBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    pub fn credential_type(mut self, cred_type: impl Into<String>) -> Self {
        self.credential_type = cred_type.into();
        self
    }

    pub fn score(mut self, score: f64) -> Self {
        self.score = score;
        self
    }

    pub fn include_proof(mut self, include: bool) -> Self {
        self.include_proof = include;
        self
    }

    pub fn include_expiration(mut self, include: bool) -> Self {
        self.include_expiration = include;
        self
    }

    pub fn skills(mut self, skills: Vec<String>) -> Self {
        self.skills = skills;
        self
    }

    pub fn build(self) -> VerifiableCredential {
        let mut rng = seeded_rng(self.seed);

        let issuer_did = format!("did:key:z6Mk{}", random_string(&mut rng, 40));
        let subject_did = format!("did:key:z6Mkr{}", random_string(&mut rng, 40));
        let course_id = format!("course-{}", random_hex(&mut rng, 8));
        let model_id = format!("model-{}", random_hex(&mut rng, 8));

        let now = chrono::Utc::now();
        let issuance_date = now.to_rfc3339();

        let expiration_date = if self.include_expiration {
            Some((now + chrono::Duration::days(365)).to_rfc3339())
        } else {
            None
        };

        let score_band = score_to_band(self.score);

        let proof = if self.include_proof {
            Some(Proof {
                proof_type: "Ed25519Signature2020".to_string(),
                created: issuance_date.clone(),
                verification_method: format!("{}#keys-1", issuer_did),
                proof_purpose: "assertionMethod".to_string(),
                proof_value: format!("z{}", random_string(&mut rng, 86)),
            })
        } else {
            None
        };

        VerifiableCredential {
            context: vec![
                "https://www.w3.org/2018/credentials/v1".to_string(),
                "https://mycelix.network/credentials/v1".to_string(),
            ],
            credential_type: vec![
                "VerifiableCredential".to_string(),
                self.credential_type.clone(),
            ],
            issuer: issuer_did,
            issuance_date,
            expiration_date,
            credential_subject: CredentialSubject {
                id: subject_did,
                course_id,
                model_id: Some(model_id),
                score: self.score,
                score_band,
                skills: self.skills,
                completion_date: now.format("%Y-%m-%d").to_string(),
                additional: std::collections::HashMap::new(),
            },
            proof,
        }
    }
}

fn score_to_band(score: f64) -> String {
    match score {
        s if s >= 90.0 => "A",
        s if s >= 80.0 => "B",
        s if s >= 70.0 => "C",
        s if s >= 60.0 => "D",
        _ => "F",
    }
    .to_string()
}

/// Generate a random credential with default settings
pub fn random() -> VerifiableCredential {
    CredentialBuilder::new().build()
}

/// Generate a credential with specific seed
pub fn with_seed(seed: u64) -> VerifiableCredential {
    CredentialBuilder::new().seed(seed).build()
}

/// Generate a credential for a specific course
pub fn for_course(course_id: &str) -> VerifiableCredential {
    let mut cred = random();
    cred.credential_subject.course_id = course_id.to_string();
    cred
}

/// Generate a high-scoring credential (A grade)
pub fn high_score() -> VerifiableCredential {
    CredentialBuilder::new()
        .score(95.0)
        .skills(vec![
            "Advanced Problem Solving".to_string(),
            "Expert Knowledge".to_string(),
            "Innovation".to_string(),
        ])
        .build()
}

/// Generate a low-scoring credential (D grade)
pub fn low_score() -> VerifiableCredential {
    CredentialBuilder::new()
        .score(65.0)
        .skills(vec!["Basic Understanding".to_string()])
        .build()
}

/// Generate a credential without proof (unsigned)
pub fn unsigned() -> VerifiableCredential {
    CredentialBuilder::new().include_proof(false).build()
}

/// Generate a credential with expiration date
pub fn with_expiration() -> VerifiableCredential {
    CredentialBuilder::new().include_expiration(true).build()
}

/// Generate a participation credential (different type)
pub fn participation() -> VerifiableCredential {
    CredentialBuilder::new()
        .credential_type("EduParticipationCredential")
        .score(0.0)
        .skills(vec!["Active Participation".to_string()])
        .build()
}

/// Generate a completion credential (different type)
pub fn completion() -> VerifiableCredential {
    CredentialBuilder::new()
        .credential_type("EduCompletionCredential")
        .score(100.0)
        .skills(vec!["Course Completion".to_string()])
        .build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_random_credential() {
        let cred = random();
        assert!(cred.issuer.starts_with("did:key:"));
        assert!(cred.credential_subject.id.starts_with("did:key:"));
        assert!(cred.proof.is_some());
    }

    #[test]
    fn test_credential_builder() {
        let cred = CredentialBuilder::new()
            .score(92.5)
            .include_proof(false)
            .credential_type("CustomCredential")
            .build();

        assert_eq!(cred.credential_subject.score, 92.5);
        assert_eq!(cred.credential_subject.score_band, "A");
        assert!(cred.proof.is_none());
        assert!(cred
            .credential_type
            .contains(&"CustomCredential".to_string()));
    }

    #[test]
    fn test_score_bands() {
        let a_cred = CredentialBuilder::new().score(95.0).build();
        assert_eq!(a_cred.credential_subject.score_band, "A");

        let b_cred = CredentialBuilder::new().score(85.0).build();
        assert_eq!(b_cred.credential_subject.score_band, "B");

        let f_cred = CredentialBuilder::new().score(50.0).build();
        assert_eq!(f_cred.credential_subject.score_band, "F");
    }

    #[test]
    fn test_unsigned_credential() {
        let cred = unsigned();
        assert!(cred.proof.is_none());
    }

    #[test]
    fn test_with_expiration() {
        let cred = with_expiration();
        assert!(cred.expiration_date.is_some());
    }

    #[test]
    fn test_high_score() {
        let cred = high_score();
        assert!(cred.credential_subject.score >= 90.0);
        assert_eq!(cred.credential_subject.score_band, "A");
    }

    #[test]
    fn test_deterministic_generation() {
        let cred1 = with_seed(42);
        let cred2 = with_seed(42);

        assert_eq!(cred1.issuer, cred2.issuer);
        assert_eq!(cred1.credential_subject.id, cred2.credential_subject.id);
    }

    #[test]
    fn test_w3c_context() {
        let cred = random();
        assert!(cred
            .context
            .contains(&"https://www.w3.org/2018/credentials/v1".to_string()));
        assert!(cred.credential_type.contains(&"VerifiableCredential".to_string()));
    }
}
