#![forbid(unsafe_code)]
//! Research-only checks for assurance-profile completeness and finite
//! common-cause fault-domain cut sets.
//!
//! Digest identities are declared dependency identifiers, not authenticated
//! proof of organizational or causal independence. A complete result is
//! conditional on the supplied manifest and the explicit finite attack model.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Digest(pub [u8; 32]);

impl Digest {
    pub const fn from_byte(byte: u8) -> Self {
        Self([byte; 32])
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum FaultDomainDimension {
    EvaluatorIdentity,
    EvaluatorProfileIdentity,
    ImplementationLineage,
    SpecificationLineage,
    ExpectedAnswerLineage,
    TestCorpusLineage,
    SourceEvidenceLineage,
    ToolchainDistribution,
    RepositoryControl,
    ExecutionProvider,
    RunnerImage,
    Runtime,
    InputAcquisition,
    OperatorControl,
    TrustRoot,
}

impl FaultDomainDimension {
    pub const ALL: [Self; 15] = [
        Self::EvaluatorIdentity,
        Self::EvaluatorProfileIdentity,
        Self::ImplementationLineage,
        Self::SpecificationLineage,
        Self::ExpectedAnswerLineage,
        Self::TestCorpusLineage,
        Self::SourceEvidenceLineage,
        Self::ToolchainDistribution,
        Self::RepositoryControl,
        Self::ExecutionProvider,
        Self::RunnerImage,
        Self::Runtime,
        Self::InputAcquisition,
        Self::OperatorControl,
        Self::TrustRoot,
    ];
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum DomainDisposition {
    /// A declared identity that the profile requires to differ from another
    /// role's identity for a specific separation obligation.
    Distinct(Digest),
    /// A known shared identity that is explicitly accepted for this
    /// dimension. The rationale remains part of the report.
    SharedAccepted { identity: Digest, rationale: String },
    /// A dimension excluded from this claim's threat model with a rationale.
    NotApplicable { rationale: String },
    /// Explicitly unresolved. This is never silently treated as distinct.
    Unknown,
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct SeparationRequirement {
    pub left_role: String,
    pub right_role: String,
    pub dimension: FaultDomainDimension,
}

impl SeparationRequirement {
    pub fn new(
        left_role: impl Into<String>,
        right_role: impl Into<String>,
        dimension: FaultDomainDimension,
    ) -> Result<Self, ProfileError> {
        let left_role = left_role.into();
        let right_role = right_role.into();
        if left_role.trim().is_empty() || right_role.trim().is_empty() {
            return Err(ProfileError::EmptyRoleId);
        }
        if left_role == right_role {
            return Err(ProfileError::SelfSeparation);
        }
        let (left_role, right_role) = if left_role < right_role {
            (left_role, right_role)
        } else {
            (right_role, left_role)
        };
        Ok(Self {
            left_role,
            right_role,
            dimension,
        })
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ThreatProfile {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub profile_digest: Digest,
    pub policy_digest: Digest,
    pub valid_from: u64,
    pub valid_until: Option<u64>,
    pub required_roles: BTreeSet<String>,
    pub required_separations: BTreeSet<SeparationRequirement>,
    /// One entry for every required role. Each role manifest should account
    /// for all 15 dimensions using an explicit disposition.
    pub role_domains: BTreeMap<String, BTreeMap<FaultDomainDimension, DomainDisposition>>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProfileError {
    EmptyClaimId,
    EmptyRequiredRoles,
    EmptyRoleId,
    SelfSeparation,
    SeparationRoleNotRequired,
    NonCanonicalSeparation,
    SharedDimensionRequiresRationale,
    NotApplicableRequiresRationale,
    UnexpectedRoleManifest,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProfileDisposition {
    CompleteForDeclaredThreatModel,
    Incomplete,
    NotYetEffective,
    Expired,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ProfileReason {
    RequiredRoleManifestMissing,
    RequiredDimensionMissing,
    UnknownDimension,
    RequiredSeparationUnknown,
    RequiredSeparationShared,
    RequiredSeparationNotApplicable,
    RequiredSeparationIdentityEqual,
    SharedIdentityNotObserved,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SeparationFinding {
    pub requirement: SeparationRequirement,
    pub distinct_identities_observed: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SharedDependencyFinding {
    pub role: String,
    pub dimension: FaultDomainDimension,
    pub identity: Digest,
    pub rationale: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObservedSharedDependency {
    pub dimension: FaultDomainDimension,
    pub identity: Digest,
    pub roles: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProfileReport {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub profile_digest: Digest,
    pub policy_digest: Digest,
    pub disposition: ProfileDisposition,
    pub reasons: Vec<ProfileReason>,
    pub separation_findings: Vec<SeparationFinding>,
    pub accepted_shared_dependencies: Vec<SharedDependencyFinding>,
    /// Duplicate identities across roles, whether or not the input manifest
    /// explicitly labelled them as accepted shared dependencies.
    pub observed_shared_dependencies: Vec<ObservedSharedDependency>,
    pub unknown_dimensions: Vec<(String, FaultDomainDimension)>,
    pub not_applicable_dimensions: Vec<(String, FaultDomainDimension, String)>,
}

pub fn check_profile(profile: &ThreatProfile, now: u64) -> Result<ProfileReport, ProfileError> {
    if profile.claim_id.trim().is_empty() {
        return Err(ProfileError::EmptyClaimId);
    }
    if profile.required_roles.is_empty() {
        return Err(ProfileError::EmptyRequiredRoles);
    }
    if profile.required_roles.iter().any(|role| role.trim().is_empty()) {
        return Err(ProfileError::EmptyRoleId);
    }
    if profile
        .role_domains
        .keys()
        .any(|role| !profile.required_roles.contains(role))
    {
        return Err(ProfileError::UnexpectedRoleManifest);
    }
    for requirement in &profile.required_separations {
        if requirement.left_role == requirement.right_role {
            return Err(ProfileError::SelfSeparation);
        }
        if requirement.left_role >= requirement.right_role {
            return Err(ProfileError::NonCanonicalSeparation);
        }
        if !profile.required_roles.contains(&requirement.left_role)
            || !profile.required_roles.contains(&requirement.right_role)
        {
            return Err(ProfileError::SeparationRoleNotRequired);
        }
    }

    let mut reasons = BTreeSet::new();
    let mut separation_findings = Vec::new();
    let mut accepted_shared_dependencies = Vec::new();
    let mut unknown_dimensions = Vec::new();
    let mut not_applicable_dimensions = Vec::new();

    for role in &profile.required_roles {
        let Some(domains) = profile.role_domains.get(role) else {
            reasons.insert(ProfileReason::RequiredRoleManifestMissing);
            continue;
        };

        for dimension in FaultDomainDimension::ALL {
            match domains.get(&dimension) {
                None => {
                    reasons.insert(ProfileReason::RequiredDimensionMissing);
                }
                Some(DomainDisposition::Distinct(_)) => {}
                Some(DomainDisposition::SharedAccepted {
                    identity,
                    rationale,
                }) => {
                    if rationale.trim().is_empty() {
                        return Err(ProfileError::SharedDimensionRequiresRationale);
                    }
                    accepted_shared_dependencies.push(SharedDependencyFinding {
                        role: role.clone(),
                        dimension,
                        identity: *identity,
                        rationale: rationale.clone(),
                    });
                }
                Some(DomainDisposition::NotApplicable { rationale }) => {
                    if rationale.trim().is_empty() {
                        return Err(ProfileError::NotApplicableRequiresRationale);
                    }
                    not_applicable_dimensions.push((
                        role.clone(),
                        dimension,
                        rationale.clone(),
                    ));
                }
                Some(DomainDisposition::Unknown) => {
                    reasons.insert(ProfileReason::UnknownDimension);
                    unknown_dimensions.push((role.clone(), dimension));
                }
            }
        }
    }

    for requirement in &profile.required_separations {
        let left = profile
            .role_domains
            .get(&requirement.left_role)
            .and_then(|domains| domains.get(&requirement.dimension));
        let right = profile
            .role_domains
            .get(&requirement.right_role)
            .and_then(|domains| domains.get(&requirement.dimension));

        let distinct_identities_observed = match (left, right) {
            (Some(DomainDisposition::Distinct(left)), Some(DomainDisposition::Distinct(right))) => {
                if left == right {
                    reasons.insert(ProfileReason::RequiredSeparationIdentityEqual);
                    false
                } else {
                    true
                }
            }
            (Some(DomainDisposition::SharedAccepted { .. }), _)
            | (_, Some(DomainDisposition::SharedAccepted { .. })) => {
                reasons.insert(ProfileReason::RequiredSeparationShared);
                false
            }
            (Some(DomainDisposition::NotApplicable { .. }), _)
            | (_, Some(DomainDisposition::NotApplicable { .. })) => {
                reasons.insert(ProfileReason::RequiredSeparationNotApplicable);
                false
            }
            _ => {
                reasons.insert(ProfileReason::RequiredSeparationUnknown);
                false
            }
        };
        separation_findings.push(SeparationFinding {
            requirement: requirement.clone(),
            distinct_identities_observed,
        });
    }

    let mut identity_roles =
        BTreeMap::<(FaultDomainDimension, Digest), BTreeSet<String>>::new();
    for (role, domains) in &profile.role_domains {
        for (dimension, disposition) in domains {
            let identity = match disposition {
                DomainDisposition::Distinct(identity)
                | DomainDisposition::SharedAccepted { identity, .. } => Some(*identity),
                DomainDisposition::NotApplicable { .. } | DomainDisposition::Unknown => None,
            };
            if let Some(identity) = identity {
                identity_roles
                    .entry((*dimension, identity))
                    .or_default()
                    .insert(role.clone());
            }
        }
    }
    let declared_shared_keys: BTreeSet<_> = profile
        .role_domains
        .values()
        .flat_map(|domains| domains.iter())
        .filter_map(|(dimension, disposition)| match disposition {
            DomainDisposition::SharedAccepted { identity, .. } => {
                Some((*dimension, *identity))
            }
            _ => None,
        })
        .collect();
    if declared_shared_keys.iter().any(|key| {
        identity_roles
            .get(key)
            .is_none_or(|roles| roles.len() < 2)
    }) {
        reasons.insert(ProfileReason::SharedIdentityNotObserved);
    }
    let observed_shared_dependencies = identity_roles
        .into_iter()
        .filter_map(|((dimension, identity), roles)| {
            (roles.len() > 1).then(|| ObservedSharedDependency {
                dimension,
                identity,
                roles: roles.into_iter().collect(),
            })
        })
        .collect();

    let disposition = if now < profile.valid_from {
        ProfileDisposition::NotYetEffective
    } else if profile.valid_until.is_some_and(|expiry| now > expiry) {
        ProfileDisposition::Expired
    } else if reasons.is_empty() {
        ProfileDisposition::CompleteForDeclaredThreatModel
    } else {
        ProfileDisposition::Incomplete
    };

    Ok(ProfileReport {
        claim_id: profile.claim_id.clone(),
        subject_digest: profile.subject_digest,
        profile_digest: profile.profile_digest,
        policy_digest: profile.policy_digest,
        disposition,
        reasons: reasons.into_iter().collect(),
        separation_findings,
        accepted_shared_dependencies,
        observed_shared_dependencies,
        unknown_dimensions,
        not_applicable_dimensions,
    })
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct FaultDomainKey {
    pub dimension: FaultDomainDimension,
    pub identity: Digest,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RoleDependencies {
    pub role: String,
    pub known_domains: BTreeSet<FaultDomainKey>,
    pub unknown_dimensions: BTreeSet<FaultDomainDimension>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DependencyManifest {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub manifest_digest: Digest,
    pub attack_model_digest: Digest,
    pub roles: Vec<RoleDependencies>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CutSetDisposition {
    Complete,
    Incomplete,
    BudgetExceeded,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CutSetError {
    EmptyClaimId,
    EmptyRoles,
    EmptyRoleId,
    DuplicateRoleId,
    ZeroStateBudget,
    CounterOverflow,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CutSetReport {
    pub claim_id: String,
    pub subject_digest: Digest,
    pub manifest_digest: Digest,
    pub attack_model_digest: Digest,
    pub disposition: CutSetDisposition,
    /// Minimal hitting sets under the declared model: compromising any key in
    /// the set invalidates every required role. Empty for incomplete/budget
    /// exceeded results; partial cut sets are never published as complete.
    pub minimal_cut_sets: Vec<BTreeSet<FaultDomainKey>>,
    pub minimum_cut_cardinality: Option<usize>,
    pub states_examined: usize,
    pub unresolved_roles: Vec<(String, Vec<FaultDomainDimension>)>,
}

fn cut_set_report(
    manifest: &DependencyManifest,
    disposition: CutSetDisposition,
    minimal_cut_sets: Vec<BTreeSet<FaultDomainKey>>,
    minimum_cut_cardinality: Option<usize>,
    states_examined: usize,
    unresolved_roles: Vec<(String, Vec<FaultDomainDimension>)>,
) -> CutSetReport {
    CutSetReport {
        claim_id: manifest.claim_id.clone(),
        subject_digest: manifest.subject_digest,
        manifest_digest: manifest.manifest_digest,
        attack_model_digest: manifest.attack_model_digest,
        disposition,
        minimal_cut_sets,
        minimum_cut_cardinality,
        states_examined,
        unresolved_roles,
    }
}

pub fn minimal_cut_sets(
    manifest: &DependencyManifest,
    max_states: usize,
) -> Result<CutSetReport, CutSetError> {
    if manifest.claim_id.trim().is_empty() {
        return Err(CutSetError::EmptyClaimId);
    }
    if manifest.roles.is_empty() {
        return Err(CutSetError::EmptyRoles);
    }
    if max_states == 0 {
        return Err(CutSetError::ZeroStateBudget);
    }

    let roles = &manifest.roles;
    let mut seen_roles = BTreeSet::new();
    for role in roles {
        if role.role.trim().is_empty() {
            return Err(CutSetError::EmptyRoleId);
        }
        if !seen_roles.insert(role.role.as_str()) {
            return Err(CutSetError::DuplicateRoleId);
        }
    }

    let mut unresolved_roles = Vec::new();
    for role in roles {
        if !role.unknown_dimensions.is_empty() || role.known_domains.is_empty() {
            let mut unresolved: Vec<_> = role.unknown_dimensions.iter().copied().collect();
            if role.known_domains.is_empty() && unresolved.is_empty() {
                unresolved = FaultDomainDimension::ALL.to_vec();
            }
            unresolved_roles.push((role.role.clone(), unresolved));
        }
    }
    if !unresolved_roles.is_empty() {
        unresolved_roles.sort();
        return Ok(cut_set_report(
            manifest,
            CutSetDisposition::Incomplete,
            Vec::new(),
            None,
            0,
            unresolved_roles,
        ));
    }

    // Incremental minimal hitting-set construction. Each role contributes a
    // dependency set; every candidate must hit it. Supersets are pruned after
    // each role, preserving all minimal solutions without enumerating 2^N
    // subsets of the entire domain universe.
    let mut candidates = vec![BTreeSet::<FaultDomainKey>::new()];
    let mut states_examined = 0usize;

    for role in roles {
        let mut expanded = BTreeSet::<BTreeSet<FaultDomainKey>>::new();
        for candidate in candidates {
            if candidate
                .iter()
                .any(|domain| role.known_domains.contains(domain))
            {
                expanded.insert(candidate);
                continue;
            }

            for domain in &role.known_domains {
                states_examined = states_examined
                    .checked_add(1)
                    .ok_or(CutSetError::CounterOverflow)?;
                if states_examined > max_states {
                    return Ok(cut_set_report(
                        manifest,
                        CutSetDisposition::BudgetExceeded,
                        Vec::new(),
                        None,
                        states_examined,
                        Vec::new(),
                    ));
                }
                let mut next = candidate.clone();
                next.insert(*domain);
                expanded.insert(next);
            }
        }

        let mut ordered: Vec<_> = expanded.into_iter().collect();
        ordered.sort_by(|left, right| {
            left.len()
                .cmp(&right.len())
                .then_with(|| left.cmp(right))
        });
        let mut minimal = Vec::<BTreeSet<FaultDomainKey>>::new();
        for candidate in ordered {
            if !minimal.iter().any(|existing| existing.is_subset(&candidate)) {
                minimal.push(candidate);
            }
        }
        candidates = minimal;
    }

    candidates.sort_by(|left, right| {
        left.len()
            .cmp(&right.len())
            .then_with(|| left.cmp(right))
    });
    let minimum_cut_cardinality = candidates.first().map(|set| set.len());

    Ok(cut_set_report(
        manifest,
        CutSetDisposition::Complete,
        candidates,
        minimum_cut_cardinality,
        states_examined,
        Vec::new(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_distinct_roles(
        roles: &[&str],
    ) -> BTreeMap<String, BTreeMap<FaultDomainDimension, DomainDisposition>> {
        roles
            .iter()
            .enumerate()
            .map(|(role_index, role)| {
                let dimensions = FaultDomainDimension::ALL
                    .into_iter()
                    .enumerate()
                    .map(|(dimension_index, dimension)| {
                        (
                            dimension,
                            DomainDisposition::Distinct(Digest::from_byte(
                                (role_index * 16 + dimension_index + 1) as u8,
                            )),
                        )
                    })
                    .collect();
                ((*role).to_owned(), dimensions)
            })
            .collect()
    }

    fn profile() -> ThreatProfile {
        let roles = BTreeSet::from(["formal".to_owned(), "independent".to_owned()]);
        let required_separations = BTreeSet::from([
            SeparationRequirement::new(
                "formal",
                "independent",
                FaultDomainDimension::ImplementationLineage,
            )
            .expect("valid separation"),
            SeparationRequirement::new(
                "formal",
                "independent",
                FaultDomainDimension::ExpectedAnswerLineage,
            )
            .expect("valid separation"),
        ]);
        ThreatProfile {
            claim_id: "claim-1".to_owned(),
            subject_digest: Digest::from_byte(1),
            profile_digest: Digest::from_byte(2),
            policy_digest: Digest::from_byte(3),
            valid_from: 10,
            valid_until: Some(100),
            required_roles: roles,
            required_separations,
            role_domains: all_distinct_roles(&["formal", "independent"]),
        }
    }

    fn dependency_manifest(roles: Vec<RoleDependencies>) -> DependencyManifest {
        DependencyManifest {
            claim_id: "portfolio-cut-claim".to_owned(),
            subject_digest: Digest::from_byte(1),
            manifest_digest: Digest::from_byte(2),
            attack_model_digest: Digest::from_byte(3),
            roles,
        }
    }

    fn domain(dimension: FaultDomainDimension, identity: u8) -> FaultDomainKey {
        FaultDomainKey {
            dimension,
            identity: Digest::from_byte(identity),
        }
    }

    fn role_dependencies(
        role: &str,
        known: impl IntoIterator<Item = FaultDomainKey>,
    ) -> RoleDependencies {
        RoleDependencies {
            role: role.to_owned(),
            known_domains: known.into_iter().collect(),
            unknown_dimensions: BTreeSet::new(),
        }
    }

    #[test]
    fn complete_profile_requires_all_15_dimensions_and_distinct_pairs() {
        let report = check_profile(&profile(), 50).expect("valid profile");
        assert_eq!(
            report.disposition,
            ProfileDisposition::CompleteForDeclaredThreatModel
        );
        assert_eq!(report.separation_findings.len(), 2);
        assert!(
            report
                .separation_findings
                .iter()
                .all(|finding| finding.distinct_identities_observed)
        );
    }

    #[test]
    fn missing_dimension_makes_profile_incomplete() {
        let mut candidate = profile();
        candidate
            .role_domains
            .get_mut("formal")
            .expect("role exists")
            .remove(&FaultDomainDimension::TrustRoot);
        let report = check_profile(&candidate, 50).expect("well-formed but incomplete");
        assert_eq!(report.disposition, ProfileDisposition::Incomplete);
        assert!(report.reasons.contains(&ProfileReason::RequiredDimensionMissing));
    }

    #[test]
    fn unknown_dimension_is_never_treated_as_distinct() {
        let mut candidate = profile();
        candidate
            .role_domains
            .get_mut("formal")
            .expect("role exists")
            .insert(
                FaultDomainDimension::ImplementationLineage,
                DomainDisposition::Unknown,
            );
        let report = check_profile(&candidate, 50).expect("well-formed but incomplete");
        assert_eq!(report.disposition, ProfileDisposition::Incomplete);
        assert!(report.reasons.contains(&ProfileReason::UnknownDimension));
        assert!(report.reasons.contains(&ProfileReason::RequiredSeparationUnknown));
    }

    #[test]
    fn accepted_shared_unrequired_domain_remains_visible() {
        let mut candidate = profile();
        candidate
            .role_domains
            .get_mut("formal")
            .expect("role exists")
            .insert(
                FaultDomainDimension::ExecutionProvider,
                DomainDisposition::SharedAccepted {
                    identity: Digest::from_byte(9),
                    rationale: "both roles use the same hosted provider".to_owned(),
                },
            );
        candidate
            .role_domains
            .get_mut("independent")
            .expect("role exists")
            .insert(
                FaultDomainDimension::ExecutionProvider,
                DomainDisposition::SharedAccepted {
                    identity: Digest::from_byte(9),
                    rationale: "same declared provider".to_owned(),
                },
            );
        let report = check_profile(&candidate, 50).expect("valid rationale");
        assert_eq!(report.disposition, ProfileDisposition::CompleteForDeclaredThreatModel);
        assert_eq!(report.accepted_shared_dependencies.len(), 2);
        assert_eq!(report.observed_shared_dependencies.len(), 1);
        assert_eq!(
            report.observed_shared_dependencies[0].roles,
            vec!["formal".to_owned(), "independent".to_owned()]
        );
        // ExecutionProvider is not a mandatory separation in this profile.
    }


    #[test]
    fn equal_known_identity_is_reported_as_shared_even_without_label() {
        let mut candidate = profile();
        let shared = Digest::from_byte(99);
        for role in ["formal", "independent"] {
            candidate
                .role_domains
                .get_mut(role)
                .expect("role exists")
                .insert(
                    FaultDomainDimension::ExecutionProvider,
                    DomainDisposition::Distinct(shared),
                );
        }
        let report = check_profile(&candidate, 50).expect("valid shape");
        assert_eq!(report.disposition, ProfileDisposition::CompleteForDeclaredThreatModel);
        assert_eq!(report.observed_shared_dependencies.len(), 1);
        assert_eq!(
            report.observed_shared_dependencies[0].identity,
            shared
        );
    }


    #[test]
    fn accepted_shared_label_requires_a_common_identity_across_roles() {
        let mut candidate = profile();
        candidate
            .role_domains
            .get_mut("formal")
            .expect("role exists")
            .insert(
                FaultDomainDimension::ExecutionProvider,
                DomainDisposition::SharedAccepted {
                    identity: Digest::from_byte(90),
                    rationale: "declared common provider".to_owned(),
                },
            );
        candidate
            .role_domains
            .get_mut("independent")
            .expect("role exists")
            .insert(
                FaultDomainDimension::ExecutionProvider,
                DomainDisposition::SharedAccepted {
                    identity: Digest::from_byte(91),
                    rationale: "declared common provider".to_owned(),
                },
            );
        let report = check_profile(&candidate, 50).expect("valid shape");
        assert_eq!(report.disposition, ProfileDisposition::Incomplete);
        assert!(
            report
                .reasons
                .contains(&ProfileReason::SharedIdentityNotObserved)
        );
    }

    #[test]
    fn shared_identity_on_a_mandatory_separation_blocks_profile() {
        let mut candidate = profile();
        let same = Digest::from_byte(77);
        for role in ["formal", "independent"] {
            candidate
                .role_domains
                .get_mut(role)
                .expect("role exists")
                .insert(
                    FaultDomainDimension::ImplementationLineage,
                    DomainDisposition::Distinct(same),
                );
        }
        let report = check_profile(&candidate, 50).expect("valid shape");
        assert_eq!(report.disposition, ProfileDisposition::Incomplete);
        assert!(
            report
                .reasons
                .contains(&ProfileReason::RequiredSeparationIdentityEqual)
        );
    }

    #[test]
    fn shared_or_not_applicable_rationale_must_be_nonempty() {
        let mut candidate = profile();
        candidate
            .role_domains
            .get_mut("formal")
            .expect("role exists")
            .insert(
                FaultDomainDimension::ExecutionProvider,
                DomainDisposition::SharedAccepted {
                    identity: Digest::from_byte(9),
                    rationale: "  ".to_owned(),
                },
            );
        assert_eq!(
            check_profile(&candidate, 50).expect_err("empty rationale rejected"),
            ProfileError::SharedDimensionRequiresRationale
        );
    }

    #[test]
    fn profile_expiration_is_explicit() {
        let candidate = profile();
        assert_eq!(
            check_profile(&candidate, 101).expect("valid profile").disposition,
            ProfileDisposition::Expired
        );
        assert_eq!(
            check_profile(&candidate, 9).expect("valid profile").disposition,
            ProfileDisposition::NotYetEffective
        );
    }

    #[test]
    fn self_separation_is_rejected_at_construction() {
        assert_eq!(
            SeparationRequirement::new(
                "formal",
                "formal",
                FaultDomainDimension::TrustRoot
            )
            .expect_err("self separation invalid"),
            ProfileError::SelfSeparation
        );
    }

    #[test]
    fn shared_dependency_yields_singleton_and_independent_pair_cut_sets() {
        let shared_ci = domain(FaultDomainDimension::ExecutionProvider, 9);
        let left_impl = domain(FaultDomainDimension::ImplementationLineage, 1);
        let right_impl = domain(FaultDomainDimension::ImplementationLineage, 2);

        let roles = vec![
            role_dependencies("left", [shared_ci, left_impl]),
            role_dependencies("right", [shared_ci, right_impl]),
        ];
        let report = minimal_cut_sets(&dependency_manifest(roles.clone()), 100).expect("valid graph");

        assert_eq!(report.disposition, CutSetDisposition::Complete);
        assert_eq!(report.claim_id, "portfolio-cut-claim");
        assert_eq!(report.subject_digest, Digest::from_byte(1));
        assert_eq!(report.manifest_digest, Digest::from_byte(2));
        assert_eq!(report.attack_model_digest, Digest::from_byte(3));
        assert_eq!(report.minimum_cut_cardinality, Some(1));
        assert_eq!(report.minimal_cut_sets.len(), 2);
        assert!(report.minimal_cut_sets.contains(&BTreeSet::from([shared_ci])));
        assert!(
            report
                .minimal_cut_sets
                .contains(&BTreeSet::from([left_impl, right_impl]))
        );
    }

    #[test]
    fn three_role_cut_sets_are_exact_and_deterministic() {
        let shared = domain(FaultDomainDimension::TrustRoot, 9);
        let a = domain(FaultDomainDimension::ImplementationLineage, 1);
        let b = domain(FaultDomainDimension::ImplementationLineage, 2);
        let c = domain(FaultDomainDimension::ImplementationLineage, 3);
        let roles = vec![
            role_dependencies("a", [shared, a]),
            role_dependencies("b", [shared, b]),
            role_dependencies("c", [shared, c]),
        ];
        let first = minimal_cut_sets(&roles, 100).expect("valid graph");
        let second = minimal_cut_sets(&roles, 100).expect("replay");
        assert_eq!(first, second);
        assert_eq!(first.minimal_cut_sets.len(), 2);
        assert!(first.minimal_cut_sets.contains(&BTreeSet::from([shared])));
        assert!(
            first
                .minimal_cut_sets
                .contains(&BTreeSet::from([a, b, c]))
        );
    }

    #[test]
    fn unknown_dependency_returns_no_numeric_cut_claim() {
        let mut left = role_dependencies(
            "left",
            [domain(FaultDomainDimension::ImplementationLineage, 1)],
        );
        left.unknown_dimensions
            .insert(FaultDomainDimension::TrustRoot);
        let right = role_dependencies(
            "right",
            [domain(FaultDomainDimension::ImplementationLineage, 2)],
        );
        let report = minimal_cut_sets(&dependency_manifest(vec![left, right]), 100).expect("valid graph");
        assert_eq!(report.disposition, CutSetDisposition::Incomplete);
        assert!(report.minimal_cut_sets.is_empty());
        assert_eq!(report.minimum_cut_cardinality, None);
    }

    #[test]
    fn empty_dependency_manifest_is_incomplete_not_zero_cut() {
        let report = minimal_cut_sets(&dependency_manifest(vec![role_dependencies("empty", [])]), 100)
            .expect("well-formed but incomplete");
        assert_eq!(report.disposition, CutSetDisposition::Incomplete);
        assert_eq!(report.minimum_cut_cardinality, None);
    }

    #[test]
    fn state_budget_exhaustion_discards_partial_cut_sets() {
        let roles = vec![
            role_dependencies(
                "left",
                [
                    domain(FaultDomainDimension::ImplementationLineage, 1),
                    domain(FaultDomainDimension::SpecificationLineage, 2),
                ],
            ),
            role_dependencies(
                "right",
                [
                    domain(FaultDomainDimension::ToolchainDistribution, 3),
                    domain(FaultDomainDimension::ExecutionProvider, 4),
                ],
            ),
        ];
        let report = minimal_cut_sets(&dependency_manifest(roles), 1).expect("valid bounded graph");
        assert_eq!(report.disposition, CutSetDisposition::BudgetExceeded);
        assert!(report.minimal_cut_sets.is_empty());
        assert_eq!(report.minimum_cut_cardinality, None);
    }

    #[test]
    fn duplicate_role_ids_are_rejected() {
        let duplicate = role_dependencies(
            "same",
            [domain(FaultDomainDimension::TrustRoot, 1)],
        );
        assert_eq!(
            minimal_cut_sets(&dependency_manifest(vec![duplicate.clone(), duplicate]), 100)
                .expect_err("duplicate role invalid"),
            CutSetError::DuplicateRoleId
        );
    }

    #[test]
    fn reversed_separation_constructor_normalizes_role_order() {
        let forward = SeparationRequirement::new(
            "a",
            "b",
            FaultDomainDimension::TrustRoot,
        )
        .expect("forward");
        let reverse = SeparationRequirement::new(
            "b",
            "a",
            FaultDomainDimension::TrustRoot,
        )
        .expect("reverse");
        assert_eq!(forward, reverse);
    }
}
