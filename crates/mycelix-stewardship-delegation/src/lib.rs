// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Structural scoped delegation-chain candidates.
//!
//! STEW-013A proves only topology and non-amplifying scope under a conservative
//! v1 algebra. It does not prove root authority, principal authentication,
//! currentness truth, runtime authorization, capability, or enforcement.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{
    CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdentityV1,
};

pub const SCOPED_DELEGATION_CHAIN_PROFILE_V1: &str =
    "mycelix/scoped-delegation-chain/v1";
pub const MAX_DELEGATION_SCOPE_REFS_V1: usize = 32;
pub const MAX_DELEGATION_EVIDENCE_REFS_V1: usize = 32;
pub const MAX_DELEGATION_CHAIN_DEPTH_V1: usize = 16;

macro_rules! typed_ref {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(CanonicalIdV1);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
                CanonicalIdV1::new(value).map(Self)
            }

            pub fn as_str(&self) -> &str {
                self.0.as_str()
            }
        }
    };
}

typed_ref!(DelegationIdV1, "Opaque delegation record identifier; not a content commitment.");
typed_ref!(DelegatorPrincipalRefV1, "Opaque delegator principal reference; not authentication.");
typed_ref!(DelegatePrincipalRefV1, "Opaque delegate principal reference; not authentication.");
typed_ref!(DelegationAuthoritySourceRefV1, "Opaque reference to the root authority source claimed to be delegated.");
typed_ref!(DelegationDomainRefV1, "Opaque stewardship-domain scope reference.");
typed_ref!(DelegationActionRefV1, "Opaque delegated-action scope reference.");
typed_ref!(DelegationPurposeRefV1, "Opaque delegated purpose/context scope reference.");
typed_ref!(DelegationSubdelegationProfileRefV1, "Opaque external profile governing subdelegation; unresolved in STEW-013A.");
typed_ref!(DelegationMandateEvidenceRefV1, "Opaque evidence reference supporting the delegation mandate/assertion.");
typed_ref!(DelegationCurrentnessEvidenceRefV1, "Opaque evidence reference supporting delegation currentness/revocation state.");
typed_ref!(DelegationBindingEvidenceRefV1, "Opaque evidence reference binding delegator/delegate/scope/source fields.");

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum DelegationCurrentnessAssertionV1 {
    AssertedCurrent,
    AssertedRevoked,
    AssertedSuperseded,
    AssertedExpired,
    Indeterminate,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DelegationSubdelegationModeV1 {
    Forbidden,
    ExactScopeOnly,
    NarrowerScopeOnly,
    ProfileGoverned(DelegationSubdelegationProfileRefV1),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DelegationScopeErrorV1 {
    EmptyDomainScope,
    EmptyActionScope,
    EmptyPurposeScope,
    TooManyDomainRefs,
    TooManyActionRefs,
    TooManyPurposeRefs,
    DuplicateDomainRef,
    DuplicateActionRef,
    DuplicatePurposeRef,
}

impl fmt::Display for DelegationScopeErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyDomainScope => "delegation domain scope must be non-empty",
            Self::EmptyActionScope => "delegation action scope must be non-empty",
            Self::EmptyPurposeScope => "delegation purpose scope must be non-empty",
            Self::TooManyDomainRefs => "too many delegation domain refs",
            Self::TooManyActionRefs => "too many delegation action refs",
            Self::TooManyPurposeRefs => "too many delegation purpose refs",
            Self::DuplicateDomainRef => "duplicate delegation domain ref",
            Self::DuplicateActionRef => "duplicate delegation action ref",
            Self::DuplicatePurposeRef => "duplicate delegation purpose ref",
        };
        f.write_str(message)
    }
}

macro_rules! scope_list {
    ($name:ident, $ref_name:ident, $empty:ident, $many:ident, $duplicate:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name(Vec<$ref_name>);

        impl $name {
            pub fn new(mut refs: Vec<$ref_name>) -> Result<Self, DelegationScopeErrorV1> {
                if refs.is_empty() {
                    return Err(DelegationScopeErrorV1::$empty);
                }
                if refs.len() > MAX_DELEGATION_SCOPE_REFS_V1 {
                    return Err(DelegationScopeErrorV1::$many);
                }
                refs.sort();
                if refs.windows(2).any(|pair| pair[0] == pair[1]) {
                    return Err(DelegationScopeErrorV1::$duplicate);
                }
                Ok(Self(refs))
            }

            pub fn as_slice(&self) -> &[$ref_name] {
                &self.0
            }

            pub fn len(&self) -> usize {
                self.0.len()
            }

            pub fn is_empty(&self) -> bool {
                self.0.is_empty()
            }

            fn is_subset_of(&self, parent: &Self) -> bool {
                self.0.iter().all(|item| parent.0.contains(item))
            }
        }
    };
}

scope_list!(DelegationDomainScopeV1, DelegationDomainRefV1, EmptyDomainScope, TooManyDomainRefs, DuplicateDomainRef);
scope_list!(DelegationActionScopeV1, DelegationActionRefV1, EmptyActionScope, TooManyActionRefs, DuplicateActionRef);
scope_list!(DelegationPurposeScopeV1, DelegationPurposeRefV1, EmptyPurposeScope, TooManyPurposeRefs, DuplicatePurposeRef);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelegationScopeV1 {
    target: StewardedSubjectIdentityV1,
    domains: DelegationDomainScopeV1,
    actions: DelegationActionScopeV1,
    purposes: DelegationPurposeScopeV1,
}

impl DelegationScopeV1 {
    pub fn new(
        target: StewardedSubjectIdentityV1,
        domains: DelegationDomainScopeV1,
        actions: DelegationActionScopeV1,
        purposes: DelegationPurposeScopeV1,
    ) -> Self {
        Self { target, domains, actions, purposes }
    }

    pub fn target(&self) -> &StewardedSubjectIdentityV1 { &self.target }
    pub fn domains(&self) -> &DelegationDomainScopeV1 { &self.domains }
    pub fn actions(&self) -> &DelegationActionScopeV1 { &self.actions }
    pub fn purposes(&self) -> &DelegationPurposeScopeV1 { &self.purposes }

    fn is_subset_of(&self, parent: &Self) -> bool {
        self.target == parent.target
            && self.domains.is_subset_of(&parent.domains)
            && self.actions.is_subset_of(&parent.actions)
            && self.purposes.is_subset_of(&parent.purposes)
    }

    fn is_strictly_narrower_than(&self, parent: &Self) -> bool {
        self.is_subset_of(parent)
            && (self.domains.len() < parent.domains.len()
                || self.actions.len() < parent.actions.len()
                || self.purposes.len() < parent.purposes.len())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelegationPartiesV1 {
    delegator: DelegatorPrincipalRefV1,
    delegate: DelegatePrincipalRefV1,
}

impl DelegationPartiesV1 {
    pub fn new(delegator: DelegatorPrincipalRefV1, delegate: DelegatePrincipalRefV1) -> Self {
        Self { delegator, delegate }
    }
    pub fn delegator(&self) -> &DelegatorPrincipalRefV1 { &self.delegator }
    pub fn delegate(&self) -> &DelegatePrincipalRefV1 { &self.delegate }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DelegationEvidenceErrorV1 {
    NoMandateEvidence,
    NoCurrentnessEvidence,
    NoBindingEvidence,
    TooManyMandateEvidenceRefs,
    TooManyCurrentnessEvidenceRefs,
    TooManyBindingEvidenceRefs,
    DuplicateMandateEvidenceRef,
    DuplicateCurrentnessEvidenceRef,
    DuplicateBindingEvidenceRef,
}

impl fmt::Display for DelegationEvidenceErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::NoMandateEvidence => "delegation mandate evidence must be non-empty",
            Self::NoCurrentnessEvidence => "delegation currentness evidence must be non-empty",
            Self::NoBindingEvidence => "delegation binding evidence must be non-empty",
            Self::TooManyMandateEvidenceRefs => "too many delegation mandate evidence refs",
            Self::TooManyCurrentnessEvidenceRefs => "too many delegation currentness evidence refs",
            Self::TooManyBindingEvidenceRefs => "too many delegation binding evidence refs",
            Self::DuplicateMandateEvidenceRef => "duplicate delegation mandate evidence ref",
            Self::DuplicateCurrentnessEvidenceRef => "duplicate delegation currentness evidence ref",
            Self::DuplicateBindingEvidenceRef => "duplicate delegation binding evidence ref",
        };
        f.write_str(message)
    }
}

macro_rules! evidence_plane {
    ($name:ident, $ref_name:ident, $empty:ident, $many:ident, $duplicate:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name(Vec<$ref_name>);
        impl $name {
            pub fn new(refs: Vec<$ref_name>) -> Result<Self, DelegationEvidenceErrorV1> {
                if refs.is_empty() { return Err(DelegationEvidenceErrorV1::$empty); }
                if refs.len() > MAX_DELEGATION_EVIDENCE_REFS_V1 { return Err(DelegationEvidenceErrorV1::$many); }
                for (index, reference) in refs.iter().enumerate() {
                    if refs[..index].contains(reference) { return Err(DelegationEvidenceErrorV1::$duplicate); }
                }
                Ok(Self(refs))
            }
            pub fn as_slice(&self) -> &[$ref_name] { &self.0 }
            pub fn len(&self) -> usize { self.0.len() }
            pub fn is_empty(&self) -> bool { self.0.is_empty() }
        }
    };
}

evidence_plane!(DelegationMandateEvidenceRefsV1, DelegationMandateEvidenceRefV1, NoMandateEvidence, TooManyMandateEvidenceRefs, DuplicateMandateEvidenceRef);
evidence_plane!(DelegationCurrentnessEvidenceRefsV1, DelegationCurrentnessEvidenceRefV1, NoCurrentnessEvidence, TooManyCurrentnessEvidenceRefs, DuplicateCurrentnessEvidenceRef);
evidence_plane!(DelegationBindingEvidenceRefsV1, DelegationBindingEvidenceRefV1, NoBindingEvidence, TooManyBindingEvidenceRefs, DuplicateBindingEvidenceRef);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelegationEvidencePlanesV1 {
    mandate: DelegationMandateEvidenceRefsV1,
    currentness: DelegationCurrentnessEvidenceRefsV1,
    binding: DelegationBindingEvidenceRefsV1,
}

impl DelegationEvidencePlanesV1 {
    pub fn new(
        mandate: DelegationMandateEvidenceRefsV1,
        currentness: DelegationCurrentnessEvidenceRefsV1,
        binding: DelegationBindingEvidenceRefsV1,
    ) -> Self { Self { mandate, currentness, binding } }
    pub fn mandate(&self) -> &DelegationMandateEvidenceRefsV1 { &self.mandate }
    pub fn currentness(&self) -> &DelegationCurrentnessEvidenceRefsV1 { &self.currentness }
    pub fn binding(&self) -> &DelegationBindingEvidenceRefsV1 { &self.binding }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScopedDelegationRecordErrorV1 { SelfDelegation }

impl fmt::Display for ScopedDelegationRecordErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self { Self::SelfDelegation => f.write_str("self-delegation is not admitted by the v1 profile") }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScopedDelegationRecordV1 {
    delegation_id: DelegationIdV1,
    parent_delegation_id: Option<DelegationIdV1>,
    parties: DelegationPartiesV1,
    authority_source_ref: DelegationAuthoritySourceRefV1,
    scope: DelegationScopeV1,
    subdelegation_mode: DelegationSubdelegationModeV1,
    currentness: DelegationCurrentnessAssertionV1,
    evidence_planes: DelegationEvidencePlanesV1,
}

impl ScopedDelegationRecordV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        delegation_id: DelegationIdV1,
        parent_delegation_id: Option<DelegationIdV1>,
        parties: DelegationPartiesV1,
        authority_source_ref: DelegationAuthoritySourceRefV1,
        scope: DelegationScopeV1,
        subdelegation_mode: DelegationSubdelegationModeV1,
        currentness: DelegationCurrentnessAssertionV1,
        evidence_planes: DelegationEvidencePlanesV1,
    ) -> Result<Self, ScopedDelegationRecordErrorV1> {
        if parties.delegator().as_str() == parties.delegate().as_str() {
            return Err(ScopedDelegationRecordErrorV1::SelfDelegation);
        }
        Ok(Self { delegation_id, parent_delegation_id, parties, authority_source_ref, scope, subdelegation_mode, currentness, evidence_planes })
    }
    pub fn delegation_id(&self) -> &DelegationIdV1 { &self.delegation_id }
    pub fn parent_delegation_id(&self) -> Option<&DelegationIdV1> { self.parent_delegation_id.as_ref() }
    pub fn parties(&self) -> &DelegationPartiesV1 { &self.parties }
    pub fn authority_source_ref(&self) -> &DelegationAuthoritySourceRefV1 { &self.authority_source_ref }
    pub fn scope(&self) -> &DelegationScopeV1 { &self.scope }
    pub fn subdelegation_mode(&self) -> &DelegationSubdelegationModeV1 { &self.subdelegation_mode }
    pub const fn currentness(&self) -> DelegationCurrentnessAssertionV1 { self.currentness }
    pub fn evidence_planes(&self) -> &DelegationEvidencePlanesV1 { &self.evidence_planes }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScopedDelegationChainErrorV1 {
    EmptyChain,
    TooDeep,
    DuplicateDelegationId,
    MissingParent,
    MultipleRoots,
    BranchingChain,
    DisconnectedOrCyclic,
    PrincipalContinuityMismatch,
    AuthoritySourceMismatch,
    TargetScopeMismatch,
    DomainScopeBroadening,
    ActionScopeBroadening,
    PurposeScopeBroadening,
    SubdelegationForbidden,
    ExactScopeRequired,
    StrictNarrowingRequired,
    SubdelegationModeBroadening,
    ProfileGovernedSubdelegationUnresolved,
}

impl fmt::Display for ScopedDelegationChainErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyChain => "delegation chain must contain at least one record",
            Self::TooDeep => "delegation chain exceeds the v1 depth bound",
            Self::DuplicateDelegationId => "duplicate delegation ID",
            Self::MissingParent => "delegation record references a missing parent",
            Self::MultipleRoots => "delegation chain must contain exactly one root",
            Self::BranchingChain => "v1 candidate is a linear chain and rejects multiple children for one parent",
            Self::DisconnectedOrCyclic => "delegation records are disconnected or cyclic",
            Self::PrincipalContinuityMismatch => "child delegator does not equal parent delegate",
            Self::AuthoritySourceMismatch => "child changes the delegated root authority source",
            Self::TargetScopeMismatch => "child changes the exact delegated target",
            Self::DomainScopeBroadening => "child broadens delegated domain scope",
            Self::ActionScopeBroadening => "child broadens delegated action scope",
            Self::PurposeScopeBroadening => "child broadens delegated purpose scope",
            Self::SubdelegationForbidden => "parent delegation forbids another hop",
            Self::ExactScopeRequired => "parent permits only exact-scope subdelegation",
            Self::StrictNarrowingRequired => "parent requires a strictly narrower child scope",
            Self::SubdelegationModeBroadening => "child subdelegation mode broadens the parent's explicit mode",
            Self::ProfileGovernedSubdelegationUnresolved => "profile-governed subdelegation cannot authorize another hop in STEW-013A",
        };
        f.write_str(message)
    }
}

fn validate_mode_and_scope(parent: &ScopedDelegationRecordV1, child: &ScopedDelegationRecordV1) -> Result<(), ScopedDelegationChainErrorV1> {
    if child.scope().target() != parent.scope().target() { return Err(ScopedDelegationChainErrorV1::TargetScopeMismatch); }
    if !child.scope().domains().is_subset_of(parent.scope().domains()) { return Err(ScopedDelegationChainErrorV1::DomainScopeBroadening); }
    if !child.scope().actions().is_subset_of(parent.scope().actions()) { return Err(ScopedDelegationChainErrorV1::ActionScopeBroadening); }
    if !child.scope().purposes().is_subset_of(parent.scope().purposes()) { return Err(ScopedDelegationChainErrorV1::PurposeScopeBroadening); }

    match parent.subdelegation_mode() {
        DelegationSubdelegationModeV1::Forbidden => Err(ScopedDelegationChainErrorV1::SubdelegationForbidden),
        DelegationSubdelegationModeV1::ExactScopeOnly => {
            if child.scope() != parent.scope() { return Err(ScopedDelegationChainErrorV1::ExactScopeRequired); }
            match child.subdelegation_mode() {
                DelegationSubdelegationModeV1::Forbidden | DelegationSubdelegationModeV1::ExactScopeOnly => Ok(()),
                DelegationSubdelegationModeV1::NarrowerScopeOnly | DelegationSubdelegationModeV1::ProfileGoverned(_) => Err(ScopedDelegationChainErrorV1::SubdelegationModeBroadening),
            }
        }
        DelegationSubdelegationModeV1::NarrowerScopeOnly => {
            if !child.scope().is_strictly_narrower_than(parent.scope()) { return Err(ScopedDelegationChainErrorV1::StrictNarrowingRequired); }
            match child.subdelegation_mode() {
                DelegationSubdelegationModeV1::Forbidden | DelegationSubdelegationModeV1::NarrowerScopeOnly => Ok(()),
                DelegationSubdelegationModeV1::ExactScopeOnly | DelegationSubdelegationModeV1::ProfileGoverned(_) => Err(ScopedDelegationChainErrorV1::SubdelegationModeBroadening),
            }
        }
        DelegationSubdelegationModeV1::ProfileGoverned(_) => Err(ScopedDelegationChainErrorV1::ProfileGovernedSubdelegationUnresolved),
    }
}

fn validate_parent_child(parent: &ScopedDelegationRecordV1, child: &ScopedDelegationRecordV1) -> Result<(), ScopedDelegationChainErrorV1> {
    if child.parties().delegator().as_str() != parent.parties().delegate().as_str() { return Err(ScopedDelegationChainErrorV1::PrincipalContinuityMismatch); }
    if child.authority_source_ref() != parent.authority_source_ref() { return Err(ScopedDelegationChainErrorV1::AuthoritySourceMismatch); }
    validate_mode_and_scope(parent, child)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScopedDelegationChainCandidateV1 { records: Vec<ScopedDelegationRecordV1> }

impl ScopedDelegationChainCandidateV1 {
    pub fn new(records: Vec<ScopedDelegationRecordV1>) -> Result<Self, ScopedDelegationChainErrorV1> {
        if records.is_empty() { return Err(ScopedDelegationChainErrorV1::EmptyChain); }
        if records.len() > MAX_DELEGATION_CHAIN_DEPTH_V1 { return Err(ScopedDelegationChainErrorV1::TooDeep); }
        for (index, record) in records.iter().enumerate() {
            if records[..index].iter().any(|previous| previous.delegation_id() == record.delegation_id()) { return Err(ScopedDelegationChainErrorV1::DuplicateDelegationId); }
        }
        for record in &records {
            if let Some(parent_id) = record.parent_delegation_id() {
                if !records.iter().any(|candidate| candidate.delegation_id() == parent_id) { return Err(ScopedDelegationChainErrorV1::MissingParent); }
            }
        }
        let roots: Vec<_> = records.iter().filter(|record| record.parent_delegation_id().is_none()).collect();
        if roots.len() != 1 { return Err(ScopedDelegationChainErrorV1::MultipleRoots); }

        let mut ordered = Vec::with_capacity(records.len());
        let mut current = (*roots[0]).clone();
        loop {
            if ordered.iter().any(|existing: &ScopedDelegationRecordV1| existing.delegation_id() == current.delegation_id()) { return Err(ScopedDelegationChainErrorV1::DisconnectedOrCyclic); }
            ordered.push(current.clone());
            let children: Vec<_> = records.iter().filter(|candidate| candidate.parent_delegation_id() == Some(current.delegation_id())).collect();
            if children.len() > 1 { return Err(ScopedDelegationChainErrorV1::BranchingChain); }
            let Some(child) = children.first() else { break; };
            validate_parent_child(&current, child)?;
            current = (**child).clone();
        }
        if ordered.len() != records.len() { return Err(ScopedDelegationChainErrorV1::DisconnectedOrCyclic); }
        Ok(Self { records: ordered })
    }

    pub fn records(&self) -> &[ScopedDelegationRecordV1] { &self.records }
    pub fn root(&self) -> &ScopedDelegationRecordV1 { &self.records[0] }
    pub fn terminal(&self) -> &ScopedDelegationRecordV1 { self.records.last().expect("validated delegation chain is non-empty") }
    pub fn root_authority_source_ref(&self) -> &DelegationAuthoritySourceRefV1 { self.root().authority_source_ref() }
    pub fn terminal_delegate(&self) -> &DelegatePrincipalRefV1 { self.terminal().parties().delegate() }
    pub fn effective_scope(&self) -> &DelegationScopeV1 { self.terminal().scope() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1, RevisionIdV1, StewardedSubjectIdV1};

    fn target() -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:delegation:1").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new("representation:dataset:1").unwrap(),
            kind: RepresentationKindV1::Dataset,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [19; 32]),
        }
    }

    fn scope(actions: &[&str], purposes: &[&str]) -> DelegationScopeV1 {
        DelegationScopeV1::new(
            target(),
            DelegationDomainScopeV1::new(vec![DelegationDomainRefV1::new("domain:stewardship").unwrap()]).unwrap(),
            DelegationActionScopeV1::new(actions.iter().map(|value| DelegationActionRefV1::new(*value).unwrap()).collect()).unwrap(),
            DelegationPurposeScopeV1::new(purposes.iter().map(|value| DelegationPurposeRefV1::new(*value).unwrap()).collect()).unwrap(),
        )
    }

    fn evidence() -> DelegationEvidencePlanesV1 {
        DelegationEvidencePlanesV1::new(
            DelegationMandateEvidenceRefsV1::new(vec![DelegationMandateEvidenceRefV1::new("evidence:mandate:1").unwrap()]).unwrap(),
            DelegationCurrentnessEvidenceRefsV1::new(vec![DelegationCurrentnessEvidenceRefV1::new("evidence:currentness:1").unwrap()]).unwrap(),
            DelegationBindingEvidenceRefsV1::new(vec![DelegationBindingEvidenceRefV1::new("evidence:binding:1").unwrap()]).unwrap(),
        )
    }

    fn record(id: &str, parent: Option<&str>, delegator: &str, delegate: &str, scope: DelegationScopeV1, mode: DelegationSubdelegationModeV1) -> ScopedDelegationRecordV1 {
        ScopedDelegationRecordV1::new(
            DelegationIdV1::new(id).unwrap(),
            parent.map(|value| DelegationIdV1::new(value).unwrap()),
            DelegationPartiesV1::new(DelegatorPrincipalRefV1::new(delegator).unwrap(), DelegatePrincipalRefV1::new(delegate).unwrap()),
            DelegationAuthoritySourceRefV1::new("authority-source:1").unwrap(),
            scope,
            mode,
            DelegationCurrentnessAssertionV1::AssertedCurrent,
            evidence(),
        ).unwrap()
    }

    #[test]
    fn input_order_does_not_choose_chain_order() {
        let root = record("delegation:root", None, "principal:root", "principal:mid", scope(&["action:a", "action:b"], &["purpose:a", "purpose:b"]), DelegationSubdelegationModeV1::NarrowerScopeOnly);
        let child = record("delegation:child", Some("delegation:root"), "principal:mid", "principal:terminal", scope(&["action:a"], &["purpose:a"]), DelegationSubdelegationModeV1::Forbidden);
        let chain = ScopedDelegationChainCandidateV1::new(vec![child, root]).unwrap();
        assert_eq!(chain.root().delegation_id().as_str(), "delegation:root");
        assert_eq!(chain.terminal().delegation_id().as_str(), "delegation:child");
    }

    #[test]
    fn scope_lists_canonicalize_permutations() {
        let left = DelegationActionScopeV1::new(vec![DelegationActionRefV1::new("action:b").unwrap(), DelegationActionRefV1::new("action:a").unwrap()]).unwrap();
        let right = DelegationActionScopeV1::new(vec![DelegationActionRefV1::new("action:a").unwrap(), DelegationActionRefV1::new("action:b").unwrap()]).unwrap();
        assert_eq!(left, right);
        assert_eq!(left.as_slice()[0].as_str(), "action:a");
        assert_eq!(left.as_slice()[1].as_str(), "action:b");
    }

    #[test]
    fn exact_scope_mode_accepts_semantic_equality_after_canonicalization() {
        let root = record("delegation:root", None, "principal:root", "principal:mid", scope(&["action:b", "action:a"], &["purpose:b", "purpose:a"]), DelegationSubdelegationModeV1::ExactScopeOnly);
        let child = record("delegation:child", Some("delegation:root"), "principal:mid", "principal:terminal", scope(&["action:a", "action:b"], &["purpose:a", "purpose:b"]), DelegationSubdelegationModeV1::Forbidden);
        let chain = ScopedDelegationChainCandidateV1::new(vec![root, child]).unwrap();
        assert_eq!(chain.root().scope(), chain.terminal().scope());
    }

    #[test]
    fn permutation_cannot_masquerade_as_strict_narrowing() {
        let root = record("delegation:root", None, "principal:root", "principal:mid", scope(&["action:a", "action:b"], &["purpose:a", "purpose:b"]), DelegationSubdelegationModeV1::NarrowerScopeOnly);
        let child = record("delegation:child", Some("delegation:root"), "principal:mid", "principal:terminal", scope(&["action:b", "action:a"], &["purpose:b", "purpose:a"]), DelegationSubdelegationModeV1::Forbidden);
        assert_eq!(ScopedDelegationChainCandidateV1::new(vec![root, child]), Err(ScopedDelegationChainErrorV1::StrictNarrowingRequired));
    }

    #[test]
    fn broadening_child_scope_rejects() {
        let root = record("delegation:root", None, "principal:root", "principal:mid", scope(&["action:a"], &["purpose:a"]), DelegationSubdelegationModeV1::NarrowerScopeOnly);
        let child = record("delegation:child", Some("delegation:root"), "principal:mid", "principal:terminal", scope(&["action:a", "action:b"], &["purpose:a"]), DelegationSubdelegationModeV1::Forbidden);
        assert_eq!(ScopedDelegationChainCandidateV1::new(vec![root, child]), Err(ScopedDelegationChainErrorV1::ActionScopeBroadening));
    }

    #[test]
    fn exact_scope_mode_rejects_narrowed_child() {
        let root = record("delegation:root", None, "principal:root", "principal:mid", scope(&["action:a", "action:b"], &["purpose:a"]), DelegationSubdelegationModeV1::ExactScopeOnly);
        let child = record("delegation:child", Some("delegation:root"), "principal:mid", "principal:terminal", scope(&["action:a"], &["purpose:a"]), DelegationSubdelegationModeV1::Forbidden);
        assert_eq!(ScopedDelegationChainCandidateV1::new(vec![root, child]), Err(ScopedDelegationChainErrorV1::ExactScopeRequired));
    }

    #[test]
    fn profile_governed_parent_cannot_self_resolve() {
        let root = record("delegation:root", None, "principal:root", "principal:mid", scope(&["action:a"], &["purpose:a"]), DelegationSubdelegationModeV1::ProfileGoverned(DelegationSubdelegationProfileRefV1::new("profile:external").unwrap()));
        let child = record("delegation:child", Some("delegation:root"), "principal:mid", "principal:terminal", scope(&["action:a"], &["purpose:a"]), DelegationSubdelegationModeV1::Forbidden);
        assert_eq!(ScopedDelegationChainCandidateV1::new(vec![root, child]), Err(ScopedDelegationChainErrorV1::ProfileGovernedSubdelegationUnresolved));
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(SCOPED_DELEGATION_CHAIN_PROFILE_V1, "mycelix/scoped-delegation-chain/v1");
    }
}
