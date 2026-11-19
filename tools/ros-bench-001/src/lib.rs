// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Dependency-free reference oracle for ROS-BENCH-001.
//!
//! This is the expected-semantics layer, not a Cedar, SpiceDB or OpenFGA adapter.
//! Unknown freshness/authority evidence is NOT_EVALUABLE, never ALLOW.

use std::fmt;

pub const CORPUS: &str =
    include_str!("../../../benchmarks/authorization/ros-bench-001/cases.tsv");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    Allow,
    Deny,
    NotEvaluable,
}

impl Decision {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Allow => "allow",
            Self::Deny => "deny",
            Self::NotEvaluable => "not_evaluable",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorizationFacts {
    pub source_current: bool,
    pub authority_known: bool,
    pub identity_bound: bool,
    pub scope_bound: bool,
    pub action_bound: bool,
    pub resource_bound: bool,
    pub purpose_bound: bool,
    pub audience_bound: bool,
    pub within_validity: bool,
    pub not_revoked: bool,
    pub parent_chain_valid: bool,
    pub depth_within_limit: bool,
    pub nonce_fresh: bool,
    pub mutation_id_unique: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Case {
    pub id: String,
    pub facts: AuthorizationFacts,
    pub expected: Decision,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CorpusError {
    MissingHeader,
    WrongColumnCount { line: usize, found: usize },
    EmptyCaseId { line: usize },
    DuplicateCaseId(String),
    InvalidBoolean { line: usize, column: &'static str, value: String },
    InvalidDecision { line: usize, value: String },
    EmptyCorpus,
}

impl fmt::Display for CorpusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for CorpusError {}

fn parse_bool(line: usize, column: &'static str, raw: &str) -> Result<bool, CorpusError> {
    match raw {
        "1" => Ok(true),
        "0" => Ok(false),
        _ => Err(CorpusError::InvalidBoolean {
            line,
            column,
            value: raw.to_owned(),
        }),
    }
}

/// Parse the frozen pipe-delimited corpus, rejecting malformed and duplicate IDs.
pub fn parse_corpus(source: &str) -> Result<Vec<Case>, CorpusError> {
    let mut lines = source.lines().filter(|line| !line.trim().is_empty());
    let header = lines.next().ok_or(CorpusError::MissingHeader)?;
    if header
        != "case_id|source_current|authority_known|identity_bound|scope_bound|action_bound|resource_bound|purpose_bound|audience_bound|within_validity|not_revoked|parent_chain_valid|depth_within_limit|nonce_fresh|mutation_id_unique|expected"
    {
        return Err(CorpusError::MissingHeader);
    }

    let mut cases = Vec::new();
    let mut ids = std::collections::BTreeSet::new();

    for (index, raw) in lines.enumerate() {
        let line = index + 2;
        let columns: Vec<&str> = raw.split('|').collect();
        if columns.len() != 16 {
            return Err(CorpusError::WrongColumnCount {
                line,
                found: columns.len(),
            });
        }
        let id = columns[0].trim();
        if id.is_empty() {
            return Err(CorpusError::EmptyCaseId { line });
        }
        if !ids.insert(id.to_owned()) {
            return Err(CorpusError::DuplicateCaseId(id.to_owned()));
        }

        let facts = AuthorizationFacts {
            source_current: parse_bool(line, "source_current", columns[1])?,
            authority_known: parse_bool(line, "authority_known", columns[2])?,
            identity_bound: parse_bool(line, "identity_bound", columns[3])?,
            scope_bound: parse_bool(line, "scope_bound", columns[4])?,
            action_bound: parse_bool(line, "action_bound", columns[5])?,
            resource_bound: parse_bool(line, "resource_bound", columns[6])?,
            purpose_bound: parse_bool(line, "purpose_bound", columns[7])?,
            audience_bound: parse_bool(line, "audience_bound", columns[8])?,
            within_validity: parse_bool(line, "within_validity", columns[9])?,
            not_revoked: parse_bool(line, "not_revoked", columns[10])?,
            parent_chain_valid: parse_bool(line, "parent_chain_valid", columns[11])?,
            depth_within_limit: parse_bool(line, "depth_within_limit", columns[12])?,
            nonce_fresh: parse_bool(line, "nonce_fresh", columns[13])?,
            mutation_id_unique: parse_bool(line, "mutation_id_unique", columns[14])?,
        };
        let expected = match columns[15] {
            "allow" => Decision::Allow,
            "deny" => Decision::Deny,
            "not_evaluable" => Decision::NotEvaluable,
            value => {
                return Err(CorpusError::InvalidDecision {
                    line,
                    value: value.to_owned(),
                });
            }
        };
        cases.push(Case {
            id: id.to_owned(),
            facts,
            expected,
        });
    }

    if cases.is_empty() {
        return Err(CorpusError::EmptyCorpus);
    }
    Ok(cases)
}

/// Evaluate only the reference semantics: unknown source/authority -> hold;
/// any known failed constraint -> deny; only complete evidence can allow.
pub fn evaluate(facts: &AuthorizationFacts) -> Decision {
    if !facts.source_current || !facts.authority_known {
        return Decision::NotEvaluable;
    }

    let all_constraints_hold = [
        facts.identity_bound,
        facts.scope_bound,
        facts.action_bound,
        facts.resource_bound,
        facts.purpose_bound,
        facts.audience_bound,
        facts.within_validity,
        facts.not_revoked,
        facts.parent_chain_valid,
        facts.depth_within_limit,
        facts.nonce_fresh,
        facts.mutation_id_unique,
    ]
    .into_iter()
    .all(|value| value);

    if all_constraints_hold {
        Decision::Allow
    } else {
        Decision::Deny
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CorpusScore {
    pub cases: usize,
    pub allow: usize,
    pub deny: usize,
    pub not_evaluable: usize,
    pub false_allows: usize,
    pub mismatches: usize,
}

pub fn score(cases: &[Case]) -> CorpusScore {
    let mut result = CorpusScore {
        cases: cases.len(),
        ..CorpusScore::default()
    };
    for case in cases {
        let actual = evaluate(&case.facts);
        match actual {
            Decision::Allow => result.allow += 1,
            Decision::Deny => result.deny += 1,
            Decision::NotEvaluable => result.not_evaluable += 1,
        }
        if actual == Decision::Allow && case.expected != Decision::Allow {
            result.false_allows += 1;
        }
        if actual != case.expected {
            result.mismatches += 1;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frozen_corpus_is_well_formed_and_has_23_cases() {
        let cases = parse_corpus(CORPUS).unwrap();
        assert_eq!(cases.len(), 23);
        let mut ids = std::collections::BTreeSet::new();
        assert!(cases.iter().all(|case| ids.insert(case.id.as_str())));
    }

    #[test]
    fn reference_oracle_matches_every_frozen_expectation() {
        let cases = parse_corpus(CORPUS).unwrap();
        let result = score(&cases);
        assert_eq!(result.mismatches, 0, "{result:?}");
        assert_eq!(result.false_allows, 0, "{result:?}");
        assert_eq!(result.allow, 2, "{result:?}");
        assert_eq!(result.deny, 18, "{result:?}");
        assert_eq!(result.not_evaluable, 3, "{result:?}");
    }

    #[test]
    fn all_known_authorization_constraints_are_individually_fail_closed() {
        let baseline = AuthorizationFacts {
            source_current: true,
            authority_known: true,
            identity_bound: true,
            scope_bound: true,
            action_bound: true,
            resource_bound: true,
            purpose_bound: true,
            audience_bound: true,
            within_validity: true,
            not_revoked: true,
            parent_chain_valid: true,
            depth_within_limit: true,
            nonce_fresh: true,
            mutation_id_unique: true,
        };

        let mut mutations: Vec<(&str, Box<dyn Fn(&mut AuthorizationFacts)>)> = vec![
            ("identity", Box::new(|f| f.identity_bound = false)),
            ("scope", Box::new(|f| f.scope_bound = false)),
            ("action", Box::new(|f| f.action_bound = false)),
            ("resource", Box::new(|f| f.resource_bound = false)),
            ("purpose", Box::new(|f| f.purpose_bound = false)),
            ("audience", Box::new(|f| f.audience_bound = false)),
            ("validity", Box::new(|f| f.within_validity = false)),
            ("revocation", Box::new(|f| f.not_revoked = false)),
            ("parent", Box::new(|f| f.parent_chain_valid = false)),
            ("depth", Box::new(|f| f.depth_within_limit = false)),
            ("nonce", Box::new(|f| f.nonce_fresh = false)),
            ("mutation", Box::new(|f| f.mutation_id_unique = false)),
        ];
        for (name, mutate) in mutations.drain(..) {
            let mut facts = baseline.clone();
            mutate(&mut facts);
            assert_eq!(evaluate(&facts), Decision::Deny, "{name}");
        }
    }

    #[test]
    fn stale_or_unknown_authority_is_not_evaluable_not_allow() {
        let mut facts = AuthorizationFacts {
            source_current: true,
            authority_known: true,
            identity_bound: true,
            scope_bound: true,
            action_bound: true,
            resource_bound: true,
            purpose_bound: true,
            audience_bound: true,
            within_validity: true,
            not_revoked: true,
            parent_chain_valid: true,
            depth_within_limit: true,
            nonce_fresh: true,
            mutation_id_unique: true,
        };
        facts.source_current = false;
        assert_eq!(evaluate(&facts), Decision::NotEvaluable);
        facts.source_current = true;
        facts.authority_known = false;
        assert_eq!(evaluate(&facts), Decision::NotEvaluable);
    }
}
