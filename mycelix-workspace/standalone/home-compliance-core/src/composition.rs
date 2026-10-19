//! Composable rule modules for global references and local building requirements.
//!
//! Composition preserves provenance and conflicts. It does not infer a legal
//! hierarchy from module ordering, numerical strictness, or geographic scope.
//! A global reference is never a binding law by virtue of being global, and a
//! narrower jurisdiction can supersede an inherited binding statement only
//! through an explicit reviewed override record. The record's identity and
//! evidence still require authentication by a production registry adapter.

use crate::regulatory::SourceApplicability;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

/// Scope is descriptive; it never automatically establishes legal precedence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RuleModuleScope {
    /// A reusable engineering reference or model code. This scope cannot bind
    /// a jurisdiction merely because a project imports it.
    GlobalReference,
    /// Canonical slash-separated jurisdiction path, e.g. `ZA/GP/Johannesburg`.
    /// Path ancestry helps validate composition direction; it is not a legal
    /// conflict-resolution rule.
    Jurisdiction(String),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RuleForce {
    BindingRequirement,
    AdoptedNotYetEffective,
    Proposal,
    Guidance,
    EngineeringBaseline,
    OwnerPreference,
    EvidenceBackedReformProposal,
}

/// Result from an external trust adapter that checks source bytes, the
/// decision's authenticity, reviewer identity, and authority to vary the
/// specific inherited rule in this jurisdiction. This crate does not implement
/// that trust adapter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OverrideVerification {
    Verified,
    AuthorityUnverified,
    DecisionNotAuthenticated,
    DecisionDoesNotAuthorizeOverride,
    EvidenceBytesNotVerified,
    VerificationUnavailable,
}

/// Trusted integration boundary for a jurisdiction-specific override registry.
/// Implementations must verify the legal decision and actual evidence bytes;
/// a string or digest-shaped value is not sufficient.
pub trait OverrideVerifier {
    fn verify_override(
        &self,
        source_module: &RuleModule,
        source_statement: &RuleStatement,
        directive: &OverrideDirective,
    ) -> OverrideVerification;
}

impl<F> OverrideVerifier for F
where
    F: Fn(&RuleModule, &RuleStatement, &OverrideDirective) -> OverrideVerification,
{
    fn verify_override(
        &self,
        source_module: &RuleModule,
        source_statement: &RuleStatement,
        directive: &OverrideDirective,
    ) -> OverrideVerification {
        self(source_module, source_statement, directive)
    }
}

/// A human-reviewed and evidence-linked request to supersede one inherited
/// binding assertion. This prototype validates metadata and digest syntax only;
/// production must authenticate the decision, reviewer, and evidence bytes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OverrideDirective {
    pub target_statement_id: String,
    pub authority_decision_id: String,
    pub reviewed_by: String,
    pub rationale: String,
    pub evidence_sha256: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RuleStatement {
    pub statement_id: String,
    pub requirement_id: String,
    /// Canonical, typed value representation supplied by a reviewed adapter.
    /// This engine compares values exactly; it does not infer semantic safety.
    pub normalized_value: String,
    pub force: RuleForce,
    pub source_clause_ref: String,
    /// Required for EvidenceBackedReformProposal; the dossier is non-binding
    /// and cannot alter active law without a separate authority adoption record.
    pub reform_dossier_id: Option<String>,
    pub overrides: Vec<OverrideDirective>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RuleModule {
    pub module_id: String,
    pub scope: RuleModuleScope,
    /// Imported modules are dependencies, not implicit higher/lower legal rank.
    pub parent_module_ids: Vec<String>,
    pub source_uri: String,
    /// SHA-256 of the exact preserved source/module artifact; syntax checked here.
    pub source_sha256: String,
    /// Must have been computed by the time-aware regulatory source evaluator.
    pub source_applicability: SourceApplicability,
    pub statements: Vec<RuleStatement>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CompositionIssue {
    EmptySelection,
    EmptyModuleId,
    DuplicateModuleId(String),
    UnknownSelectedModule(String),
    UnknownParentModule {
        module_id: String,
        parent_id: String,
    },
    DuplicateParentModule {
        module_id: String,
        parent_id: String,
    },
    InvalidModuleMetadata {
        module_id: String,
        field: &'static str,
    },
    InvalidSourceDigest(String),
    InvalidJurisdictionPath(String),
    IncompatibleParentScope {
        parent_id: String,
        child_id: String,
    },
    ModuleDependencyCycle,
    EmptyStatementId {
        module_id: String,
    },
    DuplicateStatementId(String),
    InvalidStatementMetadata {
        statement_id: String,
        field: &'static str,
    },
    BindingStatementInGlobalReference(String),
    BindingStatementFromNonApplicableSource(String),
    DuplicateOverrideTarget {
        statement_id: String,
        target_statement_id: String,
    },
    OverrideNotAuthenticated {
        statement_id: String,
        target_statement_id: String,
    },
    OverrideVerificationFailed {
        statement_id: String,
        target_statement_id: String,
        reason: OverrideVerification,
    },
    InvalidOverrideMetadata {
        statement_id: String,
        field: &'static str,
    },
    InvalidOverrideEvidenceDigest {
        statement_id: String,
    },
    UnknownOverrideTarget {
        statement_id: String,
        target_statement_id: String,
    },
    OverrideTargetNotBinding {
        statement_id: String,
        target_statement_id: String,
    },
    OverrideTargetNotInherited {
        statement_id: String,
        target_statement_id: String,
    },
    OverrideRequirementMismatch {
        statement_id: String,
        target_statement_id: String,
    },
    OverrideSourceNotBinding(String),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComposedSource {
    pub module_id: String,
    pub statement_id: String,
    pub source_clause_ref: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComposedAssertion {
    pub requirement_id: String,
    pub normalized_value: String,
    pub force: RuleForce,
    /// Identifier of an evidence-backed reform dossier, when the assertion is
    /// a non-binding reform proposal.
    pub reform_dossier_id: Option<String>,
    pub source_applicability: SourceApplicability,
    pub source: ComposedSource,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectiveBindingRequirement {
    pub requirement_id: String,
    pub normalized_value: String,
    pub contributing_sources: Vec<ComposedSource>,
    pub superseded_sources: Vec<ComposedSource>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RuleConflictCandidate {
    pub normalized_value: String,
    pub sources: Vec<ComposedSource>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RuleConflict {
    pub requirement_id: String,
    pub candidates: Vec<RuleConflictCandidate>,
}

/// A conflict is data that must be resolved; it does not get converted into an
/// arbitrary winner. A result with conflicts is not ready for a compliance gate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComposedRuleSet {
    pub selected_module_ids: Vec<String>,
    /// Deterministic parent-before-child order for the selected dependency closure.
    pub module_lineage: Vec<String>,
    pub binding_requirements: Vec<EffectiveBindingRequirement>,
    pub conflicts: Vec<RuleConflict>,
    pub non_binding_assertions: Vec<ComposedAssertion>,
    pub owner_preferences: Vec<ComposedAssertion>,
    pub superseded_sources: Vec<ComposedSource>,
}

fn valid_jurisdiction_path(path: &str) -> bool {
    nonblank(path)
        && path.split('/').all(|segment| {
            !segment.is_empty()
                && segment
                    .bytes()
                    .all(|byte| byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_')
        })
}

fn parent_scope_covers(parent: &RuleModuleScope, child: &RuleModuleScope) -> bool {
    match (parent, child) {
        (RuleModuleScope::GlobalReference, _) => true,
        (RuleModuleScope::Jurisdiction(_), RuleModuleScope::GlobalReference) => false,
        (RuleModuleScope::Jurisdiction(parent), RuleModuleScope::Jurisdiction(child)) => {
            let parent_parts: Vec<_> = parent.split('/').collect();
            let child_parts: Vec<_> = child.split('/').collect();
            parent_parts.len() <= child_parts.len()
                && parent_parts
                    .iter()
                    .zip(child_parts.iter())
                    .all(|(a, b)| a == b)
        }
    }
}

fn module_is_ancestor(
    ancestor_id: &str,
    child_id: &str,
    modules: &HashMap<&str, &RuleModule>,
) -> bool {
    if ancestor_id == child_id {
        return true;
    }
    let mut queue = VecDeque::from([child_id.to_owned()]);
    let mut visited = HashSet::new();
    while let Some(current) = queue.pop_front() {
        if !visited.insert(current.clone()) {
            continue;
        }
        let Some(module) = modules.get(current.as_str()) else {
            continue;
        };
        for parent_id in &module.parent_module_ids {
            if parent_id == ancestor_id {
                return true;
            }
            queue.push_back(parent_id.clone());
        }
    }
    false
}

/// Compose selected modules and all their ancestors.
///
/// Errors describe malformed/unverifiable module composition. Conflicting
/// binding assertions are returned separately in `conflicts`, with no chosen
/// winner. The caller must block any permit/review gate while conflicts remain.
/// Non-binding references and owner preferences are preserved separately from
/// binding requirements, allowing creativity without misrepresenting a preference
/// as law.
/// Compose metadata-only rules without permitting any proposed override to
/// displace a binding requirement. Override decisions require the verified
/// variant below; names and well-formed digest strings are not proof of authority.
pub fn compose_rule_modules(
    selected_module_ids: &[String],
    modules: &[RuleModule],
) -> Result<ComposedRuleSet, Vec<CompositionIssue>> {
    compose_rule_modules_internal(selected_module_ids, modules, None)
}

/// Compose rules with an externally trusted jurisdiction-specific override
/// verifier. The verifier must authenticate the legal decision, verify the
/// actual evidence bytes, and establish that the named authority can vary this
/// exact inherited requirement. A successful return from this function is only
/// as trustworthy as that adapter; it is not a general compliance certificate.
pub fn compose_verified_rule_modules<V: OverrideVerifier>(
    selected_module_ids: &[String],
    modules: &[RuleModule],
    verifier: &V,
) -> Result<ComposedRuleSet, Vec<CompositionIssue>> {
    compose_rule_modules_internal(selected_module_ids, modules, Some(verifier))
}

fn compose_rule_modules_internal(
    selected_module_ids: &[String],
    modules: &[RuleModule],
    override_verifier: Option<&dyn OverrideVerifier>,
) -> Result<ComposedRuleSet, Vec<CompositionIssue>> {
    if selected_module_ids.is_empty() {
        return Err(vec![CompositionIssue::EmptySelection]);
    }

    let mut issues = Vec::new();
    let mut by_id: HashMap<&str, &RuleModule> = HashMap::new();
    for module in modules {
        if !nonblank(&module.module_id) {
            issues.push(CompositionIssue::EmptyModuleId);
            continue;
        }
        if by_id.insert(module.module_id.as_str(), module).is_some() {
            issues.push(CompositionIssue::DuplicateModuleId(
                module.module_id.clone(),
            ));
        }
    }
    if !issues.is_empty() {
        return Err(issues);
    }

    let mut selected: Vec<String> = selected_module_ids.to_vec();
    selected.sort();
    selected.dedup();
    let mut closure = BTreeSet::new();
    let mut pending = selected.clone();
    while let Some(module_id) = pending.pop() {
        let Some(module) = by_id.get(module_id.as_str()).copied() else {
            issues.push(CompositionIssue::UnknownSelectedModule(module_id));
            continue;
        };
        if !closure.insert(module.module_id.clone()) {
            continue;
        }
        for parent_id in &module.parent_module_ids {
            if !by_id.contains_key(parent_id.as_str()) {
                issues.push(CompositionIssue::UnknownParentModule {
                    module_id: module.module_id.clone(),
                    parent_id: parent_id.clone(),
                });
            } else {
                pending.push(parent_id.clone());
            }
        }
        let mut seen_parents = HashSet::new();
        for parent_id in &module.parent_module_ids {
            if !seen_parents.insert(parent_id.as_str()) {
                issues.push(CompositionIssue::DuplicateParentModule {
                    module_id: module.module_id.clone(),
                    parent_id: parent_id.clone(),
                });
            }
        }
    }
    if !issues.is_empty() {
        return Err(issues);
    }

    for module_id in &closure {
        let module = by_id[module_id.as_str()];
        for (field, value) in [
            ("source_uri", module.source_uri.as_str()),
            ("module_id", module.module_id.as_str()),
        ] {
            if !nonblank(value) {
                issues.push(CompositionIssue::InvalidModuleMetadata {
                    module_id: module.module_id.clone(),
                    field,
                });
            }
        }
        if !is_sha256_hex(&module.source_sha256) {
            issues.push(CompositionIssue::InvalidSourceDigest(
                module.module_id.clone(),
            ));
        }
        if let RuleModuleScope::Jurisdiction(path) = &module.scope {
            if !valid_jurisdiction_path(path) {
                issues.push(CompositionIssue::InvalidJurisdictionPath(
                    module.module_id.clone(),
                ));
            }
        }
        for parent_id in &module.parent_module_ids {
            let parent = by_id[parent_id.as_str()];
            if !parent_scope_covers(&parent.scope, &module.scope) {
                issues.push(CompositionIssue::IncompatibleParentScope {
                    parent_id: parent_id.clone(),
                    child_id: module.module_id.clone(),
                });
            }
        }
    }
    if !issues.is_empty() {
        return Err(issues);
    }

    let mut indegree: HashMap<String, usize> = closure
        .iter()
        .map(|module_id| (module_id.clone(), 0))
        .collect();
    let mut children: HashMap<String, Vec<String>> = HashMap::new();
    for module_id in &closure {
        let module = by_id[module_id.as_str()];
        for parent_id in &module.parent_module_ids {
            *indegree
                .get_mut(module_id)
                .expect("every selected module has an indegree") += 1;
            children
                .entry(parent_id.clone())
                .or_default()
                .push(module_id.clone());
        }
    }
    for values in children.values_mut() {
        values.sort();
    }
    let mut ready: BTreeSet<String> = indegree
        .iter()
        .filter_map(|(module_id, degree)| (*degree == 0).then_some(module_id.clone()))
        .collect();
    let mut lineage = Vec::with_capacity(closure.len());
    while let Some(module_id) = ready.pop_first() {
        lineage.push(module_id.clone());
        if let Some(dependent_modules) = children.get(&module_id) {
            for child_id in dependent_modules {
                let degree = indegree
                    .get_mut(child_id)
                    .expect("every child module has an indegree");
                *degree -= 1;
                if *degree == 0 {
                    ready.insert(child_id.clone());
                }
            }
        }
    }
    if lineage.len() != closure.len() {
        return Err(vec![CompositionIssue::ModuleDependencyCycle]);
    }

    let mut statements: HashMap<&str, (&RuleModule, &RuleStatement)> = HashMap::new();
    for module_id in &lineage {
        let module = by_id[module_id.as_str()];
        for statement in &module.statements {
            if !nonblank(&statement.statement_id) {
                issues.push(CompositionIssue::EmptyStatementId {
                    module_id: module.module_id.clone(),
                });
                continue;
            }
            if statements
                .insert(statement.statement_id.as_str(), (module, statement))
                .is_some()
            {
                issues.push(CompositionIssue::DuplicateStatementId(
                    statement.statement_id.clone(),
                ));
            }
            for (field, value) in [
                ("requirement_id", statement.requirement_id.as_str()),
                ("normalized_value", statement.normalized_value.as_str()),
                ("source_clause_ref", statement.source_clause_ref.as_str()),
            ] {
                if !nonblank(value) {
                    issues.push(CompositionIssue::InvalidStatementMetadata {
                        statement_id: statement.statement_id.clone(),
                        field,
                    });
                }
            }
            if statement.force == RuleForce::EvidenceBackedReformProposal
                && !statement.reform_dossier_id.as_deref().is_some_and(nonblank)
            {
                issues.push(CompositionIssue::InvalidStatementMetadata {
                    statement_id: statement.statement_id.clone(),
                    field: "reform_dossier_id",
                });
            }
            if statement.force == RuleForce::BindingRequirement {
                if module.scope == RuleModuleScope::GlobalReference {
                    issues.push(CompositionIssue::BindingStatementInGlobalReference(
                        statement.statement_id.clone(),
                    ));
                }
                if module.source_applicability != SourceApplicability::Applicable {
                    issues.push(CompositionIssue::BindingStatementFromNonApplicableSource(
                        statement.statement_id.clone(),
                    ));
                }
            }
        }
    }
    if !issues.is_empty() {
        return Err(issues);
    }

    let mut superseded_ids = HashSet::new();
    for module_id in &lineage {
        let module = by_id[module_id.as_str()];
        for statement in &module.statements {
            if statement.overrides.is_empty() {
                continue;
            }
            if statement.force != RuleForce::BindingRequirement
                || module.scope == RuleModuleScope::GlobalReference
                || module.source_applicability != SourceApplicability::Applicable
            {
                issues.push(CompositionIssue::OverrideSourceNotBinding(
                    statement.statement_id.clone(),
                ));
                continue;
            }
            let mut seen_targets = HashSet::new();
            for directive in &statement.overrides {
                if !seen_targets.insert(directive.target_statement_id.as_str()) {
                    issues.push(CompositionIssue::DuplicateOverrideTarget {
                        statement_id: statement.statement_id.clone(),
                        target_statement_id: directive.target_statement_id.clone(),
                    });
                    continue;
                }

                for (field, value) in [
                    (
                        "target_statement_id",
                        directive.target_statement_id.as_str(),
                    ),
                    (
                        "authority_decision_id",
                        directive.authority_decision_id.as_str(),
                    ),
                    ("reviewed_by", directive.reviewed_by.as_str()),
                    ("rationale", directive.rationale.as_str()),
                ] {
                    if !nonblank(value) {
                        issues.push(CompositionIssue::InvalidOverrideMetadata {
                            statement_id: statement.statement_id.clone(),
                            field,
                        });
                    }
                }
                if !is_sha256_hex(&directive.evidence_sha256) {
                    issues.push(CompositionIssue::InvalidOverrideEvidenceDigest {
                        statement_id: statement.statement_id.clone(),
                    });
                }

                // Validate local metadata before crossing the authentication
                // boundary so malformed digests remain diagnosable even when
                // the caller supplies no verifier. This never accepts an
                // override: authentication is still mandatory below.
                let Some(verifier) = override_verifier else {
                    issues.push(CompositionIssue::OverrideNotAuthenticated {
                        statement_id: statement.statement_id.clone(),
                        target_statement_id: directive.target_statement_id.clone(),
                    });
                    continue;
                };
                let verification = verifier.verify_override(module, statement, directive);
                if verification != OverrideVerification::Verified {
                    issues.push(CompositionIssue::OverrideVerificationFailed {
                        statement_id: statement.statement_id.clone(),
                        target_statement_id: directive.target_statement_id.clone(),
                        reason: verification,
                    });
                    continue;
                }
                let Some((target_module, target)) = statements
                    .get(directive.target_statement_id.as_str())
                    .copied()
                else {
                    issues.push(CompositionIssue::UnknownOverrideTarget {
                        statement_id: statement.statement_id.clone(),
                        target_statement_id: directive.target_statement_id.clone(),
                    });
                    continue;
                };
                if target.force != RuleForce::BindingRequirement {
                    issues.push(CompositionIssue::OverrideTargetNotBinding {
                        statement_id: statement.statement_id.clone(),
                        target_statement_id: directive.target_statement_id.clone(),
                    });
                }
                if target.requirement_id != statement.requirement_id {
                    issues.push(CompositionIssue::OverrideRequirementMismatch {
                        statement_id: statement.statement_id.clone(),
                        target_statement_id: directive.target_statement_id.clone(),
                    });
                }
                if !module_is_ancestor(&target_module.module_id, &module.module_id, &by_id)
                    || !parent_scope_covers(&target_module.scope, &module.scope)
                    || target_module.scope == RuleModuleScope::GlobalReference
                {
                    issues.push(CompositionIssue::OverrideTargetNotInherited {
                        statement_id: statement.statement_id.clone(),
                        target_statement_id: directive.target_statement_id.clone(),
                    });
                } else {
                    superseded_ids.insert(target.statement_id.as_str());
                }
            }
        }
    }
    if !issues.is_empty() {
        return Err(issues);
    }

    let mut binding_groups: BTreeMap<String, BTreeMap<String, Vec<ComposedSource>>> =
        BTreeMap::new();
    let mut non_binding_assertions = Vec::new();
    let mut owner_preferences = Vec::new();
    let mut superseded_sources_by_requirement: BTreeMap<String, Vec<ComposedSource>> =
        BTreeMap::new();

    for module_id in &lineage {
        let module = by_id[module_id.as_str()];
        for statement in &module.statements {
            let source = ComposedSource {
                module_id: module.module_id.clone(),
                statement_id: statement.statement_id.clone(),
                source_clause_ref: statement.source_clause_ref.clone(),
            };
            if superseded_ids.contains(statement.statement_id.as_str()) {
                superseded_sources_by_requirement
                    .entry(statement.requirement_id.clone())
                    .or_default()
                    .push(source.clone());
            }
            let assertion = ComposedAssertion {
                requirement_id: statement.requirement_id.clone(),
                normalized_value: statement.normalized_value.clone(),
                force: statement.force,
                reform_dossier_id: statement.reform_dossier_id.clone(),
                source_applicability: module.source_applicability.clone(),
                source: source.clone(),
            };
            match statement.force {
                RuleForce::BindingRequirement
                    if !superseded_ids.contains(statement.statement_id.as_str()) =>
                {
                    binding_groups
                        .entry(statement.requirement_id.clone())
                        .or_default()
                        .entry(statement.normalized_value.clone())
                        .or_default()
                        .push(source);
                }
                RuleForce::BindingRequirement => {}
                RuleForce::OwnerPreference => owner_preferences.push(assertion),
                _ => non_binding_assertions.push(assertion),
            }
        }
    }

    let mut binding_requirements = Vec::new();
    let mut conflicts = Vec::new();
    for (requirement_id, value_groups) in binding_groups {
        if value_groups.len() > 1 {
            let candidates = value_groups
                .into_iter()
                .map(|(normalized_value, mut sources)| {
                    sources.sort_by(|left, right| {
                        (&left.module_id, &left.statement_id)
                            .cmp(&(&right.module_id, &right.statement_id))
                    });
                    RuleConflictCandidate {
                        normalized_value,
                        sources,
                    }
                })
                .collect();
            conflicts.push(RuleConflict {
                requirement_id,
                candidates,
            });
        } else if let Some((normalized_value, mut contributing_sources)) =
            value_groups.into_iter().next()
        {
            contributing_sources.sort_by(|left, right| {
                (&left.module_id, &left.statement_id).cmp(&(&right.module_id, &right.statement_id))
            });
            let mut superseded_sources = superseded_sources_by_requirement
                .remove(&requirement_id)
                .unwrap_or_default();
            superseded_sources.sort_by(|left, right| {
                (&left.module_id, &left.statement_id).cmp(&(&right.module_id, &right.statement_id))
            });
            binding_requirements.push(EffectiveBindingRequirement {
                requirement_id,
                normalized_value,
                contributing_sources,
                superseded_sources,
            });
        }
    }

    non_binding_assertions.sort_by(|left, right| {
        (
            &left.requirement_id,
            &left.source.module_id,
            &left.source.statement_id,
        )
            .cmp(&(
                &right.requirement_id,
                &right.source.module_id,
                &right.source.statement_id,
            ))
    });
    owner_preferences.sort_by(|left, right| {
        (
            &left.requirement_id,
            &left.source.module_id,
            &left.source.statement_id,
        )
            .cmp(&(
                &right.requirement_id,
                &right.source.module_id,
                &right.source.statement_id,
            ))
    });
    let mut superseded_sources: Vec<_> = superseded_sources_by_requirement
        .into_values()
        .flatten()
        .collect();
    superseded_sources.sort_by(|left, right| {
        (&left.module_id, &left.statement_id).cmp(&(&right.module_id, &right.statement_id))
    });

    Ok(ComposedRuleSet {
        selected_module_ids: selected,
        module_lineage: lineage,
        binding_requirements,
        conflicts,
        non_binding_assertions,
        owner_preferences,
        superseded_sources,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regulatory::NonBindingReason;

    fn source_status() -> SourceApplicability {
        SourceApplicability::Applicable
    }

    fn global_status() -> SourceApplicability {
        SourceApplicability::NonBinding(NonBindingReason::Guidance)
    }

    fn statement(id: &str, requirement: &str, value: &str, force: RuleForce) -> RuleStatement {
        RuleStatement {
            statement_id: id.into(),
            requirement_id: requirement.into(),
            normalized_value: value.into(),
            force,
            source_clause_ref: format!("clause:{id}"),
            reform_dossier_id: None,
            overrides: Vec::new(),
        }
    }

    fn module(
        id: &str,
        scope: RuleModuleScope,
        parents: &[&str],
        applicability: SourceApplicability,
        statements: Vec<RuleStatement>,
    ) -> RuleModule {
        RuleModule {
            module_id: id.into(),
            scope,
            parent_module_ids: parents.iter().map(|id| (*id).into()).collect(),
            source_uri: format!("https://example.invalid/{id}"),
            source_sha256: "a".repeat(64),
            source_applicability: applicability,
            statements,
        }
    }

    fn layered_modules(
        local_value: &str,
        local_overrides: Vec<OverrideDirective>,
    ) -> Vec<RuleModule> {
        let national_statement = statement(
            "national:ventilation",
            "ventilation.minimum_rate",
            "value:base",
            RuleForce::BindingRequirement,
        );
        let mut local_statement = statement(
            "municipal:ventilation",
            "ventilation.minimum_rate",
            local_value,
            RuleForce::BindingRequirement,
        );
        local_statement.overrides = local_overrides;

        vec![
            module(
                "global:ventilation-baseline",
                RuleModuleScope::GlobalReference,
                &[],
                global_status(),
                vec![statement(
                    "baseline:ventilation",
                    "ventilation.minimum_rate",
                    "reference:value",
                    RuleForce::EngineeringBaseline,
                )],
            ),
            module(
                "ZA",
                RuleModuleScope::Jurisdiction("ZA".into()),
                &["global:ventilation-baseline"],
                source_status(),
                vec![national_statement.clone()],
            ),
            module(
                "ZA/GP",
                RuleModuleScope::Jurisdiction("ZA/GP".into()),
                &["ZA"],
                source_status(),
                vec![],
            ),
            module(
                "ZA/GP/Johannesburg",
                RuleModuleScope::Jurisdiction("ZA/GP/Johannesburg".into()),
                &["ZA/GP"],
                source_status(),
                vec![local_statement],
            ),
        ]
    }

    fn reviewed_override(target: &str) -> OverrideDirective {
        OverrideDirective {
            target_statement_id: target.into(),
            authority_decision_id: "decision:municipal-reviewed-variance".into(),
            reviewed_by: "reviewer:independent-human".into(),
            rationale: "Reviewed alternative with evidence and jurisdiction authority record"
                .into(),
            evidence_sha256: "b".repeat(64),
        }
    }

    #[test]
    fn global_reference_can_be_composed_without_becoming_binding_law() {
        let modules = layered_modules("value:base", Vec::new());
        let result = compose_rule_modules(&["ZA/GP/Johannesburg".into()], &modules).unwrap();
        assert_eq!(result.binding_requirements.len(), 1);
        assert!(result.conflicts.is_empty());
        assert!(result.non_binding_assertions.iter().any(|item| {
            item.force == RuleForce::EngineeringBaseline
                && item.source.module_id == "global:ventilation-baseline"
        }));
    }

    #[test]
    fn global_reference_is_shared_without_cross_jurisdiction_rule_leakage() {
        // Synthetic IDs/values only: this tests graph isolation, not the law of
        // South Africa, Germany, or the United States.
        let modules = vec![
            module(
                "global:ventilation-reference",
                RuleModuleScope::GlobalReference,
                &[],
                global_status(),
                vec![statement(
                    "reference:ventilation",
                    "ventilation.minimum_rate",
                    "value:engineering-reference",
                    RuleForce::EngineeringBaseline,
                )],
            ),
            module(
                "ZA/GP/Johannesburg",
                RuleModuleScope::Jurisdiction("ZA/GP/Johannesburg".into()),
                &["global:ventilation-reference"],
                source_status(),
                vec![statement(
                    "za:ventilation",
                    "ventilation.minimum_rate",
                    "value:za-profile",
                    RuleForce::BindingRequirement,
                )],
            ),
            module(
                "EU/DE/Berlin",
                RuleModuleScope::Jurisdiction("EU/DE/Berlin".into()),
                &["global:ventilation-reference"],
                source_status(),
                vec![statement(
                    "de:ventilation",
                    "ventilation.minimum_rate",
                    "value:de-profile",
                    RuleForce::BindingRequirement,
                )],
            ),
            module(
                "US/CA/Los-Angeles",
                RuleModuleScope::Jurisdiction("US/CA/Los-Angeles".into()),
                &["global:ventilation-reference"],
                source_status(),
                vec![statement(
                    "us-ca:ventilation",
                    "ventilation.minimum_rate",
                    "value:us-ca-profile",
                    RuleForce::BindingRequirement,
                )],
            ),
        ];

        for (jurisdiction, expected_value, foreign_module_ids) in [
            (
                "ZA/GP/Johannesburg",
                "value:za-profile",
                vec!["EU/DE/Berlin", "US/CA/Los-Angeles"],
            ),
            (
                "EU/DE/Berlin",
                "value:de-profile",
                vec!["ZA/GP/Johannesburg", "US/CA/Los-Angeles"],
            ),
            (
                "US/CA/Los-Angeles",
                "value:us-ca-profile",
                vec!["ZA/GP/Johannesburg", "EU/DE/Berlin"],
            ),
        ] {
            let result = compose_rule_modules(&[jurisdiction.into()], &modules).unwrap();
            assert_eq!(result.binding_requirements.len(), 1);
            assert_eq!(
                result.binding_requirements[0].normalized_value,
                expected_value
            );
            assert_eq!(result.conflicts.len(), 0);
            assert!(
                result
                    .module_lineage
                    .contains(&"global:ventilation-reference".to_owned())
            );
            assert!(!result.module_lineage.iter().any(|module_id| {
                foreign_module_ids.contains(&module_id.as_str())
            }));
            assert!(result.non_binding_assertions.iter().any(|item| {
                item.source.module_id == "global:ventilation-reference"
                    && item.force == RuleForce::EngineeringBaseline
            }));
        }
    }

    #[test]
    fn jurisdiction_scope_does_not_implicitly_override_parent_rule() {
        let modules = layered_modules("value:local-different", Vec::new());
        let result = compose_rule_modules(&["ZA/GP/Johannesburg".into()], &modules).unwrap();
        assert!(result.binding_requirements.is_empty());
        assert_eq!(result.conflicts.len(), 1);
        assert_eq!(
            result.conflicts[0].requirement_id,
            "ventilation.minimum_rate"
        );
        assert_eq!(result.conflicts[0].candidates.len(), 2);
    }

    #[test]
    fn metadata_only_composition_cannot_apply_override_even_with_digest_syntax() {
        let modules = layered_modules(
            "value:local-reviewed",
            vec![reviewed_override("national:ventilation")],
        );
        let issues = compose_rule_modules(&["ZA/GP/Johannesburg".into()], &modules).unwrap_err();
        assert!(
            issues.contains(&CompositionIssue::OverrideNotAuthenticated {
                statement_id: "municipal:ventilation".into(),
                target_statement_id: "national:ventilation".into(),
            })
        );
    }

    #[test]
    fn verified_override_preserves_superseded_provenance() {
        let modules = layered_modules(
            "value:local-reviewed",
            vec![reviewed_override("national:ventilation")],
        );
        // Test double only; production must use an authenticated registry adapter.
        let result = compose_verified_rule_modules(
            &["ZA/GP/Johannesburg".into()],
            &modules,
            &|_: &RuleModule, _: &RuleStatement, _: &OverrideDirective| {
                OverrideVerification::Verified
            },
        )
        .unwrap();
        assert!(result.conflicts.is_empty());
        assert_eq!(result.binding_requirements.len(), 1);
        let effective = &result.binding_requirements[0];
        assert_eq!(effective.normalized_value, "value:local-reviewed");
        assert_eq!(effective.superseded_sources.len(), 1);
        assert_eq!(
            effective.superseded_sources[0].statement_id,
            "national:ventilation"
        );
    }

    #[test]
    fn rejected_override_verification_blocks_instead_of_selecting_local_rule() {
        let modules = layered_modules(
            "value:local-reviewed",
            vec![reviewed_override("national:ventilation")],
        );
        let result = compose_verified_rule_modules(
            &["ZA/GP/Johannesburg".into()],
            &modules,
            &|_: &RuleModule, _: &RuleStatement, _: &OverrideDirective| {
                OverrideVerification::DecisionDoesNotAuthorizeOverride
            },
        );
        assert!(result.is_err());
        assert!(
            result
                .unwrap_err()
                .contains(&CompositionIssue::OverrideVerificationFailed {
                    statement_id: "municipal:ventilation".into(),
                    target_statement_id: "national:ventilation".into(),
                    reason: OverrideVerification::DecisionDoesNotAuthorizeOverride,
                })
        );
    }

    #[test]
    fn evidence_backed_reform_reference_is_required_and_preserved() {
        let proposal = statement(
            "reform:window-alternative",
            "window.operation",
            "value:alternative",
            RuleForce::EvidenceBackedReformProposal,
        );
        let modules = vec![module(
            "ZA/proposal",
            RuleModuleScope::Jurisdiction("ZA".into()),
            &[],
            SourceApplicability::NonBinding(NonBindingReason::Proposal),
            vec![proposal],
        )];
        let issues = compose_rule_modules(&["ZA/proposal".into()], &modules).unwrap_err();
        assert!(
            issues.contains(&CompositionIssue::InvalidStatementMetadata {
                statement_id: "reform:window-alternative".into(),
                field: "reform_dossier_id",
            })
        );

        let mut proposal = statement(
            "reform:window-alternative",
            "window.operation",
            "value:alternative",
            RuleForce::EvidenceBackedReformProposal,
        );
        proposal.reform_dossier_id = Some("reform:windows:case-1".into());
        let modules = vec![module(
            "ZA/proposal",
            RuleModuleScope::Jurisdiction("ZA".into()),
            &[],
            SourceApplicability::NonBinding(NonBindingReason::Proposal),
            vec![proposal],
        )];
        let result = compose_rule_modules(&["ZA/proposal".into()], &modules).unwrap();
        assert_eq!(
            result.non_binding_assertions[0]
                .reform_dossier_id
                .as_deref(),
            Some("reform:windows:case-1")
        );
        assert!(result.binding_requirements.is_empty());
    }

    #[test]
    fn proposed_or_guidance_source_cannot_supply_binding_requirement() {
        let modules = vec![module(
            "ZA/proposal",
            RuleModuleScope::Jurisdiction("ZA".into()),
            &[],
            SourceApplicability::NonBinding(NonBindingReason::Proposal),
            vec![statement(
                "proposal:minimum-area",
                "room.minimum_area",
                "value:larger",
                RuleForce::BindingRequirement,
            )],
        )];
        let issues = compose_rule_modules(&["ZA/proposal".into()], &modules).unwrap_err();
        assert!(
            issues.contains(&CompositionIssue::BindingStatementFromNonApplicableSource(
                "proposal:minimum-area".into()
            ))
        );
    }

    #[test]
    fn global_reference_cannot_declare_binding_law() {
        let modules = vec![module(
            "global:law",
            RuleModuleScope::GlobalReference,
            &[],
            source_status(),
            vec![statement(
                "fake:binding",
                "fire.egress",
                "value:pass",
                RuleForce::BindingRequirement,
            )],
        )];
        let issues = compose_rule_modules(&["global:law".into()], &modules).unwrap_err();
        assert!(
            issues.contains(&CompositionIssue::BindingStatementInGlobalReference(
                "fake:binding".into()
            ))
        );
    }

    #[test]
    fn malformed_override_evidence_blocks_composition() {
        let modules = layered_modules(
            "value:local-reviewed",
            vec![OverrideDirective {
                evidence_sha256: "not-a-digest".into(),
                ..reviewed_override("national:ventilation")
            }],
        );
        let issues = compose_rule_modules(&["ZA/GP/Johannesburg".into()], &modules).unwrap_err();
        assert!(issues.iter().any(|issue| matches!(
            issue,
            CompositionIssue::InvalidOverrideEvidenceDigest { statement_id }
                if statement_id == "municipal:ventilation"
        )));
    }

    #[test]
    fn module_cycles_block_composition() {
        let modules = vec![
            module(
                "ZA",
                RuleModuleScope::Jurisdiction("ZA".into()),
                &["ZA/GP"],
                source_status(),
                vec![],
            ),
            module(
                "ZA/GP",
                RuleModuleScope::Jurisdiction("ZA".into()),
                &["ZA"],
                source_status(),
                vec![],
            ),
        ];
        let issues = compose_rule_modules(&["ZA/GP".into()], &modules).unwrap_err();
        assert!(issues.contains(&CompositionIssue::ModuleDependencyCycle));
    }

    #[test]
    fn incompatible_parent_scope_blocks_composition() {
        let modules = vec![
            module(
                "ZA/GP",
                RuleModuleScope::Jurisdiction("ZA/GP".into()),
                &[],
                source_status(),
                vec![],
            ),
            module(
                "ZA",
                RuleModuleScope::Jurisdiction("ZA".into()),
                &["ZA/GP"],
                source_status(),
                vec![],
            ),
        ];
        let issues = compose_rule_modules(&["ZA".into()], &modules).unwrap_err();
        assert!(issues.contains(&CompositionIssue::IncompatibleParentScope {
            parent_id: "ZA/GP".into(),
            child_id: "ZA".into(),
        }));
    }

    #[test]
    fn owner_preferences_remain_separate_from_binding_rules() {
        let modules = vec![
            module(
                "ZA",
                RuleModuleScope::Jurisdiction("ZA".into()),
                &[],
                source_status(),
                vec![statement(
                    "law:window",
                    "window.operation",
                    "value:required-opening",
                    RuleForce::BindingRequirement,
                )],
            ),
            module(
                "owner:design",
                RuleModuleScope::Jurisdiction("ZA".into()),
                &["ZA"],
                source_status(),
                vec![statement(
                    "owner:window-style",
                    "window.appearance",
                    "value:arched",
                    RuleForce::OwnerPreference,
                )],
            ),
        ];
        let result = compose_rule_modules(&["owner:design".into()], &modules).unwrap();
        assert_eq!(result.binding_requirements.len(), 1);
        assert_eq!(result.owner_preferences.len(), 1);
        assert_eq!(result.owner_preferences[0].normalized_value, "value:arched");
    }
}
