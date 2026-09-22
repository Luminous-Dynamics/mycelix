// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fixture-scoped cross-repository investigation identity mapping.
//!
//! This crate validates and resolves exact correspondence only. It has no transport,
//! serialization, EPI admission, Holochain, network, filesystem, database, search/tool
//! execution, Xenia, OPSEC permit, target admission, lease, connector, or action API.

use std::collections::BTreeSet;
use std::error::Error;
use std::fmt;

pub const IDENTITY_MAP_PROFILE_V1: &str =
    "mycelix:symthaea:investigation-identity-map:v0.1";
pub const MAX_REF_BYTES: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdentityMapAuthorityV1 {
    FixtureCorrespondenceOnly,
}

#[derive(Clone, PartialEq, Eq)]
pub enum IdentityMapError {
    InvalidReference {
        role: &'static str,
        reason: &'static str,
    },
    DuplicateBindingId,
    SourceSubjectMismatch,
    DestinationSubjectMismatch,
    NonOneToOneUnsupported,
    SourceIdentityConflict,
    DestinationIdentityConflict,
    DuplicateLocalOnlyIdentity,
    DuplicateNoExportIdentity,
    LocalOnlyMappedConflict,
    NoExportMappedConflict,
    LocalOnlyNoExportConflict,
    ProtectedContentMapped,
}

impl fmt::Display for IdentityMapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidReference { role, reason } => {
                write!(f, "invalid {role} reference: {reason}")
            }
            Self::DuplicateBindingId => write!(f, "duplicate binding identity: <redacted>"),
            Self::SourceSubjectMismatch => write!(f, "binding source subject mismatch"),
            Self::DestinationSubjectMismatch => write!(f, "binding destination subject mismatch"),
            Self::NonOneToOneUnsupported => {
                write!(f, "v1 correspondence requires one-to-one binding")
            }
            Self::SourceIdentityConflict => write!(f, "conflicting source identity binding"),
            Self::DestinationIdentityConflict => {
                write!(f, "conflicting destination identity binding")
            }
            Self::DuplicateLocalOnlyIdentity => {
                write!(f, "duplicate destination-local-only identity: <redacted>")
            }
            Self::DuplicateNoExportIdentity => {
                write!(f, "duplicate no-export identity: <redacted>")
            }
            Self::LocalOnlyMappedConflict => {
                write!(f, "destination identity is both mapped and local-only")
            }
            Self::NoExportMappedConflict => {
                write!(f, "destination identity is both mapped and no-export")
            }
            Self::LocalOnlyNoExportConflict => {
                write!(f, "destination identity is both local-only and no-export")
            }
            Self::ProtectedContentMapped => {
                write!(f, "no-export-by-policy identity claims raw content mapped")
            }
        }
    }
}

impl fmt::Debug for IdentityMapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

impl Error for IdentityMapError {}

fn validate_ref(role: &'static str, value: &str) -> Result<(), IdentityMapError> {
    if value.is_empty() {
        return Err(IdentityMapError::InvalidReference {
            role,
            reason: "empty",
        });
    }
    if value.len() > MAX_REF_BYTES {
        return Err(IdentityMapError::InvalidReference {
            role,
            reason: "too long",
        });
    }
    if !value.bytes().all(|byte| (0x21..=0x7e).contains(&byte)) {
        return Err(IdentityMapError::InvalidReference {
            role,
            reason: "graphic ASCII only",
        });
    }
    Ok(())
}

macro_rules! role_ref {
    ($name:ident) => {
        #[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, IdentityMapError> {
                let value = value.into();
                validate_ref(stringify!($name), &value)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}(<redacted>)", stringify!($name))
            }
        }
    };
}

role_ref!(RepositoryRef);
role_ref!(CommitRef);
role_ref!(ProfileRef);
role_ref!(RoleRef);
role_ref!(LocalIdRef);
role_ref!(BindingId);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SubjectIdentityV1 {
    pub repository: RepositoryRef,
    pub head: CommitRef,
    pub profile: ProfileRef,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LocalIdentityV1 {
    pub role: RoleRef,
    pub local_id: LocalIdRef,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct IdentityEndpointV1 {
    pub subject: SubjectIdentityV1,
    pub local: LocalIdentityV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BindingKindV1 {
    CrossRepoCorrespondence,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CorrespondenceV1 {
    pub binding_id: BindingId,
    pub binding_profile: ProfileRef,
    pub source: IdentityEndpointV1,
    pub destination: IdentityEndpointV1,
    pub binding_kind: BindingKindV1,
    pub one_to_one: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DestinationLocalOnlyV1 {
    pub local: LocalIdentityV1,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NoExportByPolicyV1 {
    pub local: LocalIdentityV1,
    pub raw_content_mapped: bool,
}

#[derive(Clone, PartialEq, Eq)]
pub struct IdentityMapV1 {
    pub profile: &'static str,
    pub source_subject: SubjectIdentityV1,
    pub destination_subject: SubjectIdentityV1,
    pub correspondences: Vec<CorrespondenceV1>,
    pub destination_local_only: Vec<DestinationLocalOnlyV1>,
    pub no_export_by_policy: Vec<NoExportByPolicyV1>,
}

impl IdentityMapV1 {
    pub fn new(
        source_subject: SubjectIdentityV1,
        destination_subject: SubjectIdentityV1,
        correspondences: Vec<CorrespondenceV1>,
        destination_local_only: Vec<DestinationLocalOnlyV1>,
        no_export_by_policy: Vec<NoExportByPolicyV1>,
    ) -> Result<Self, IdentityMapError> {
        let mut binding_ids = BTreeSet::new();
        let mut source_endpoints = BTreeSet::new();
        let mut destination_endpoints = BTreeSet::new();
        let mut mapped_destination_locals = BTreeSet::new();

        for binding in &correspondences {
            if !binding_ids.insert(binding.binding_id.clone()) {
                return Err(IdentityMapError::DuplicateBindingId);
            }
            if binding.source.subject != source_subject {
                return Err(IdentityMapError::SourceSubjectMismatch);
            }
            if binding.destination.subject != destination_subject {
                return Err(IdentityMapError::DestinationSubjectMismatch);
            }
            if !binding.one_to_one {
                return Err(IdentityMapError::NonOneToOneUnsupported);
            }
            if !source_endpoints.insert(binding.source.clone()) {
                return Err(IdentityMapError::SourceIdentityConflict);
            }
            if !destination_endpoints.insert(binding.destination.clone()) {
                return Err(IdentityMapError::DestinationIdentityConflict);
            }
            mapped_destination_locals.insert(binding.destination.local.clone());
        }

        let mut local_only = BTreeSet::new();
        for record in &destination_local_only {
            if !local_only.insert(record.local.clone()) {
                return Err(IdentityMapError::DuplicateLocalOnlyIdentity);
            }
            if mapped_destination_locals.contains(&record.local) {
                return Err(IdentityMapError::LocalOnlyMappedConflict);
            }
        }

        let mut no_export = BTreeSet::new();
        for record in &no_export_by_policy {
            if record.raw_content_mapped {
                return Err(IdentityMapError::ProtectedContentMapped);
            }
            if !no_export.insert(record.local.clone()) {
                return Err(IdentityMapError::DuplicateNoExportIdentity);
            }
            if mapped_destination_locals.contains(&record.local) {
                return Err(IdentityMapError::NoExportMappedConflict);
            }
            if local_only.contains(&record.local) {
                return Err(IdentityMapError::LocalOnlyNoExportConflict);
            }
        }

        Ok(Self {
            profile: IDENTITY_MAP_PROFILE_V1,
            source_subject,
            destination_subject,
            correspondences,
            destination_local_only,
            no_export_by_policy,
        })
    }

    /// Resolve only an exact source subject + role + local identity tuple.
    ///
    /// There is intentionally no spelling-only lookup API.
    pub fn lookup_destination(
        &self,
        source_subject: &SubjectIdentityV1,
        source_role: &RoleRef,
        source_local_id: &LocalIdRef,
    ) -> Option<&IdentityEndpointV1> {
        self.correspondences
            .iter()
            .find(|binding| {
                &binding.source.subject == source_subject
                    && &binding.source.local.role == source_role
                    && &binding.source.local.local_id == source_local_id
            })
            .map(|binding| &binding.destination)
    }

    pub fn authority_scope(&self) -> IdentityMapAuthorityV1 {
        IdentityMapAuthorityV1::FixtureCorrespondenceOnly
    }

    pub fn admission_authority(&self) -> bool {
        false
    }

    pub fn collection_authority(&self) -> bool {
        false
    }

    pub fn execution_authority(&self) -> bool {
        false
    }
}

impl fmt::Debug for IdentityMapV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("IdentityMapV1")
            .field("profile", &self.profile)
            .field("correspondence_count", &self.correspondences.len())
            .field("local_only_count", &self.destination_local_only.len())
            .field("no_export_count", &self.no_export_by_policy.len())
            .field("authority", &self.authority_scope())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rr(value: &str) -> RepositoryRef {
        RepositoryRef::new(value).unwrap()
    }

    fn cr(value: &str) -> CommitRef {
        CommitRef::new(value).unwrap()
    }

    fn pr(value: &str) -> ProfileRef {
        ProfileRef::new(value).unwrap()
    }

    fn role(value: &str) -> RoleRef {
        RoleRef::new(value).unwrap()
    }

    fn lid(value: &str) -> LocalIdRef {
        LocalIdRef::new(value).unwrap()
    }

    fn source_subject() -> SubjectIdentityV1 {
        SubjectIdentityV1 {
            repository: rr("Luminous-Dynamics/symthaea"),
            head: cr("90267527cba13a1e0be8c424ebce03f1f5726e46"),
            profile: pr("symthaea:closed-world-investigation-loop:v1"),
        }
    }

    fn destination_subject() -> SubjectIdentityV1 {
        SubjectIdentityV1 {
            repository: rr("Luminous-Dynamics/mycelix"),
            head: cr("59dcabafd28b2020f09abe4544d731ab720143d1"),
            profile: pr("mycelix:epi:investigation-capsule:v1"),
        }
    }

    fn endpoint(subject: SubjectIdentityV1, role_name: &str, local_id: &str) -> IdentityEndpointV1 {
        IdentityEndpointV1 {
            subject,
            local: LocalIdentityV1 {
                role: role(role_name),
                local_id: lid(local_id),
            },
        }
    }

    fn binding(
        id: &str,
        source_role: &str,
        source_id: &str,
        destination_role: &str,
        destination_id: &str,
    ) -> CorrespondenceV1 {
        CorrespondenceV1 {
            binding_id: BindingId::new(id).unwrap(),
            binding_profile: pr("exact-fixture-correspondence:v1"),
            source: endpoint(source_subject(), source_role, source_id),
            destination: endpoint(destination_subject(), destination_role, destination_id),
            binding_kind: BindingKindV1::CrossRepoCorrespondence,
            one_to_one: true,
        }
    }

    fn fixture_bindings() -> Vec<CorrespondenceV1> {
        let mut values = vec![
            binding("MAP:ART:A1", "ArtifactProjectionRef", "artifact:A1", "ArtifactRecordV1", "AR1"),
            binding("MAP:ART:A2", "ArtifactProjectionRef", "artifact:A2", "ArtifactRecordV1", "AR2"),
            binding("MAP:ART:A3", "ArtifactProjectionRef", "artifact:A3", "ArtifactRecordV1", "AR3"),
            binding("MAP:FRONTIER:F2", "FrontierRef", "F2", "FrontierRecordV1", "F2"),
            binding(
                "MAP:METHOD:T_WEB_PUBLIC_TOPK",
                "ToolProfileRef",
                "T_WEB_PUBLIC_TOPK",
                "MethodologySelectedProfileRef",
                "T_WEB_PUBLIC_TOPK",
            ),
            CorrespondenceV1 {
                binding_id: BindingId::new("MAP:SUBJECT:LOOP:F2").unwrap(),
                binding_profile: pr("external-subject-recording:v1"),
                source: endpoint(
                    source_subject(),
                    "ClosedWorldInvestigationRecordV1",
                    "symthaea:closed-world-investigation-loop:v1",
                ),
                destination: endpoint(
                    destination_subject(),
                    "ExternalCandidateRecordV1",
                    "SYMCAND:F2",
                ),
                binding_kind: BindingKindV1::CrossRepoCorrespondence,
                one_to_one: true,
            },
        ];

        for proposal in ["D1", "D2", "D3", "D4", "D5"] {
            values.push(binding(
                &format!("MAP:PROPOSAL:{proposal}"),
                "InformationProposalId",
                proposal,
                "PlannerHistoryProposalRef",
                proposal,
            ));
        }
        values
    }

    fn local_only(role_name: &str, local_id: &str) -> DestinationLocalOnlyV1 {
        DestinationLocalOnlyV1 {
            local: LocalIdentityV1 {
                role: role(role_name),
                local_id: lid(local_id),
            },
        }
    }

    fn fixture_map() -> IdentityMapV1 {
        IdentityMapV1::new(
            source_subject(),
            destination_subject(),
            fixture_bindings(),
            vec![
                local_only("FrontierRecordV1", "F1"),
                local_only("ArtifactRecordV1", "AR4"),
                local_only("AssumptionAssessmentV1", "ASSUMP1:F1"),
                local_only("AssumptionAssessmentV1", "ASSUMP1:F2"),
                local_only("DependencyGroupV1", "DEP:G1"),
                local_only("PresentationProjectionV1", "ATLAS:F2"),
            ],
            vec![NoExportByPolicyV1 {
                local: LocalIdentityV1 {
                    role: role("ProtectedOmissionV1"),
                    local_id: lid("OMIT1"),
                },
                raw_content_mapped: false,
            }],
        )
        .unwrap()
    }

    #[test]
    fn exact_subject_role_and_local_id_resolve_fixture_correspondence() {
        let map = fixture_map();
        let destination = map
            .lookup_destination(
                &source_subject(),
                &role("ArtifactProjectionRef"),
                &lid("artifact:A1"),
            )
            .unwrap();

        assert_eq!(destination.local.role.as_str(), "ArtifactRecordV1");
        assert_eq!(destination.local.local_id.as_str(), "AR1");
        assert_eq!(
            map.authority_scope(),
            IdentityMapAuthorityV1::FixtureCorrespondenceOnly
        );
        assert!(!map.admission_authority());
        assert!(!map.collection_authority());
        assert!(!map.execution_authority());
    }

    #[test]
    fn same_spelling_under_wrong_role_does_not_match() {
        let map = fixture_map();
        assert!(map
            .lookup_destination(&source_subject(), &role("InformationProposalId"), &lid("F2"))
            .is_none());
    }

    #[test]
    fn subject_head_substitution_does_not_match() {
        let map = fixture_map();
        let mut substituted = source_subject();
        substituted.head = cr("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        assert!(map
            .lookup_destination(&substituted, &role("FrontierRef"), &lid("F2"))
            .is_none());
    }

    #[test]
    fn source_conflict_rejects() {
        let mut bindings = fixture_bindings();
        bindings.push(binding(
            "MAP:CONFLICT",
            "ArtifactProjectionRef",
            "artifact:A1",
            "ArtifactRecordV1",
            "AR9",
        ));
        assert_eq!(
            IdentityMapV1::new(
                source_subject(),
                destination_subject(),
                bindings,
                vec![],
                vec![]
            ),
            Err(IdentityMapError::SourceIdentityConflict)
        );
    }

    #[test]
    fn mapped_identity_cannot_be_local_only() {
        let result = IdentityMapV1::new(
            source_subject(),
            destination_subject(),
            fixture_bindings(),
            vec![local_only("ArtifactRecordV1", "AR1")],
            vec![],
        );
        assert_eq!(result, Err(IdentityMapError::LocalOnlyMappedConflict));
    }

    #[test]
    fn protected_no_export_content_cannot_be_marked_mapped() {
        let result = IdentityMapV1::new(
            source_subject(),
            destination_subject(),
            fixture_bindings(),
            vec![],
            vec![NoExportByPolicyV1 {
                local: LocalIdentityV1 {
                    role: role("ProtectedOmissionV1"),
                    local_id: lid("OMIT1"),
                },
                raw_content_mapped: true,
            }],
        );
        assert_eq!(result, Err(IdentityMapError::ProtectedContentMapped));
    }

    #[test]
    fn default_debug_redacts_local_identity_values() {
        let value = lid("sensitive-investigation-target");
        let rendered = format!("{value:?}");
        assert!(rendered.contains("<redacted>"));
        assert!(!rendered.contains("sensitive-investigation-target"));

        let map = fixture_map();
        let rendered_map = format!("{map:?}");
        assert!(!rendered_map.contains("artifact:A1"));
        assert!(!rendered_map.contains("T_WEB_PUBLIC_TOPK"));
    }
}
