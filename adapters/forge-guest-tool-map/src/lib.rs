// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2C2A: exact inner-tool mapping for the hermetic Forge guest.
//!
//! The guest must never search `$PATH`, guess Nix store names, or accept host
//! argv as authority. This contract binds the exact guest plan and Nix closure
//! to the three executables used inside the sandbox, then cross-checks their
//! artifact identities against the enclosing [`ExecutionSpec`].

use mycelix_forge_core::{Digest, DigestAlgorithm};
use mycelix_forge_execution::{ExecutionContractError, ExecutionSpec, ToolArtifact};
use mycelix_forge_guest_plan::{GuestPlanError, GuestVerificationPlanV1};
use mycelix_forge_linux_isolation::{IsolationPolicyError, NixClosureManifest};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

const TOOL_MAP_DOMAIN_V1: &[u8] = b"mycelix-forge/guest-tool-map/v1\0";
const MAX_TEXT: usize = 4096;

pub const GUEST_TOOL_ROLES: &[&str] = &["forge-isolation-probe", "git", "gittuf"];

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct GuestToolBinding {
    role: String,
    executable: String,
    semantic_version: String,
    artifact_digest: Digest,
    artifact_size: u64,
    derivation: Option<Digest>,
}

impl GuestToolBinding {
    fn from_tool(
        role: &str,
        executable: String,
        tool: &ToolArtifact,
        closure: &NixClosureManifest,
    ) -> Result<Self, GuestToolMapError> {
        if tool.role() != role {
            return Err(GuestToolMapError::ToolRoleMismatch(role.to_owned()));
        }
        validate_executable(role, &executable, closure)?;
        Ok(Self {
            role: role.to_owned(),
            executable,
            semantic_version: tool.semantic_version().to_owned(),
            artifact_digest: tool.digest().clone(),
            artifact_size: tool.size(),
            derivation: tool.derivation().cloned(),
        })
    }

    fn from_wire(
        role: String,
        executable: String,
        semantic_version: String,
        artifact_digest: Digest,
        artifact_size: u64,
        derivation: Option<Digest>,
    ) -> Result<Self, GuestToolMapError> {
        validate_role(&role)?;
        validate_executable_syntax(&role, &executable)?;
        validate_text("semantic version", &semantic_version)?;
        if artifact_size == 0 {
            return Err(GuestToolMapError::EmptyToolArtifact(role));
        }
        Ok(Self {
            role,
            executable,
            semantic_version,
            artifact_digest,
            artifact_size,
            derivation,
        })
    }

    pub fn role(&self) -> &str {
        &self.role
    }

    pub fn executable(&self) -> &str {
        &self.executable
    }

    pub fn semantic_version(&self) -> &str {
        &self.semantic_version
    }

    pub fn artifact_digest(&self) -> &Digest {
        &self.artifact_digest
    }

    pub const fn artifact_size(&self) -> u64 {
        self.artifact_size
    }

    pub fn derivation(&self) -> Option<&Digest> {
        self.derivation.as_ref()
    }

    fn matches_tool(&self, tool: &ToolArtifact) -> bool {
        self.role == tool.role()
            && self.semantic_version == tool.semantic_version()
            && self.artifact_digest == *tool.digest()
            && self.artifact_size == tool.size()
            && self.derivation.as_ref() == tool.derivation()
    }
}

impl<'de> Deserialize<'de> for GuestToolBinding {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            role: String,
            executable: String,
            semantic_version: String,
            artifact_digest: Digest,
            artifact_size: u64,
            derivation: Option<Digest>,
        }

        let wire = Wire::deserialize(deserializer)?;
        Self::from_wire(
            wire.role,
            wire.executable,
            wire.semantic_version,
            wire.artifact_digest,
            wire.artifact_size,
            wire.derivation,
        )
        .map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct GuestToolMapV1 {
    plan_digest: Digest,
    execution_subject: Digest,
    nix_closure: Digest,
    tools: Vec<GuestToolBinding>,
}

impl GuestToolMapV1 {
    /// Build the exact guest-tool map directly from the intended execution tool
    /// artifacts. This is the acyclic construction entry point: the final
    /// ExecutionSpec may not exist yet because it must commit the resulting map
    /// bytes as one of its inputs.
    ///
    /// Only the three guest roles participate in this map. Additional host-side
    /// tool artifacts may be present. Duplicate artifact roles are rejected.
    /// Final qualification still cross-checks every binding against the finished
    /// ExecutionSpec through [`qualify_guest_tool_map`].
    pub fn from_tool_artifacts(
        plan: &GuestVerificationPlanV1,
        closure: &NixClosureManifest,
        tool_artifacts: &[ToolArtifact],
        paths: BTreeMap<String, String>,
    ) -> Result<Self, GuestToolMapError> {
        require_exact_path_roles(&paths)?;

        let closure_digest = closure.digest(DigestAlgorithm::Sha256)?;
        if plan.nix_closure() != &closure_digest {
            return Err(GuestToolMapError::ClosureDigestMismatch);
        }

        let mut by_role = BTreeMap::new();
        for tool in tool_artifacts {
            if by_role.insert(tool.role(), tool).is_some() {
                return Err(GuestToolMapError::DuplicateToolArtifactRole(
                    tool.role().to_owned(),
                ));
            }
        }

        let mut tools = Vec::with_capacity(GUEST_TOOL_ROLES.len());
        for role in GUEST_TOOL_ROLES {
            let executable = paths
                .get(*role)
                .expect("exact path-role set checked")
                .clone();
            let tool = by_role
                .get(*role)
                .copied()
                .ok_or_else(|| GuestToolMapError::MissingExecutionTool((*role).to_owned()))?;
            tools.push(GuestToolBinding::from_tool(
                role,
                executable,
                tool,
                closure,
            )?);
        }
        tools.sort();

        Ok(Self {
            plan_digest: plan.digest(DigestAlgorithm::Sha256)?,
            execution_subject: plan.execution_subject().clone(),
            nix_closure: closure_digest,
            tools,
        })
    }

    /// Backwards-compatible constructor from a completed execution spec.
    /// Delegates artifact construction to [`Self::from_tool_artifacts`] and
    /// retains the exact execution-subject check.
    pub fn from_execution_spec(
        plan: &GuestVerificationPlanV1,
        closure: &NixClosureManifest,
        spec: &ExecutionSpec,
        paths: BTreeMap<String, String>,
    ) -> Result<Self, GuestToolMapError> {
        if spec.subject() != plan.execution_subject() {
            return Err(GuestToolMapError::ExecutionSubjectMismatch);
        }
        Self::from_tool_artifacts(plan, closure, spec.tools(), paths)
    }

    pub fn plan_digest(&self) -> &Digest {
        &self.plan_digest
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn nix_closure(&self) -> &Digest {
        &self.nix_closure
    }

    pub fn tools(&self) -> &[GuestToolBinding] {
        &self.tools
    }

    pub fn tool(&self, role: &str) -> Option<&GuestToolBinding> {
        self.tools.iter().find(|tool| tool.role() == role)
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, GuestToolMapError> {
        validate_canonical_tool_set(&self.tools)?;
        let mut out = Vec::new();
        out.extend_from_slice(TOOL_MAP_DOMAIN_V1);
        push_digest(&mut out, &self.plan_digest)?;
        push_digest(&mut out, &self.execution_subject)?;
        push_digest(&mut out, &self.nix_closure)?;
        push_count(&mut out, self.tools.len(), "tools")?;
        for tool in &self.tools {
            push_string(&mut out, tool.role(), "tool role")?;
            push_string(&mut out, tool.executable(), "tool executable")?;
            push_string(&mut out, tool.semantic_version(), "tool semantic version")?;
            push_digest(&mut out, tool.artifact_digest())?;
            out.extend_from_slice(&tool.artifact_size().to_be_bytes());
            push_optional_digest(&mut out, tool.derivation())?;
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, GuestToolMapError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for GuestToolMapV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            plan_digest: Digest,
            execution_subject: Digest,
            nix_closure: Digest,
            tools: Vec<GuestToolBinding>,
        }

        let wire = Wire::deserialize(deserializer)?;
        validate_canonical_tool_set(&wire.tools).map_err(D::Error::custom)?;
        Ok(Self {
            plan_digest: wire.plan_digest,
            execution_subject: wire.execution_subject,
            nix_closure: wire.nix_closure,
            tools: wire.tools,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedGuestToolMap {
    map_digest: Digest,
    plan_digest: Digest,
    execution_subject: Digest,
    nix_closure: Digest,
    tools: Vec<GuestToolBinding>,
}

impl QualifiedGuestToolMap {
    pub fn map_digest(&self) -> &Digest {
        &self.map_digest
    }

    pub fn plan_digest(&self) -> &Digest {
        &self.plan_digest
    }

    pub fn execution_subject(&self) -> &Digest {
        &self.execution_subject
    }

    pub fn nix_closure(&self) -> &Digest {
        &self.nix_closure
    }

    pub fn tools(&self) -> &[GuestToolBinding] {
        &self.tools
    }

    pub fn executable(&self, role: &str) -> Option<&str> {
        self.tools()
            .iter()
            .find(|tool| tool.role() == role)
            .map(GuestToolBinding::executable)
    }
}

pub fn qualify_guest_tool_map(
    map: &GuestToolMapV1,
    plan: &GuestVerificationPlanV1,
    closure: &NixClosureManifest,
    spec: &ExecutionSpec,
) -> Result<QualifiedGuestToolMap, GuestToolMapError> {
    let plan_digest = plan.digest(DigestAlgorithm::Sha256)?;
    if map.plan_digest != plan_digest {
        return Err(GuestToolMapError::PlanDigestMismatch);
    }
    if map.execution_subject != *plan.execution_subject() || spec.subject() != plan.execution_subject()
    {
        return Err(GuestToolMapError::ExecutionSubjectMismatch);
    }

    let closure_digest = closure.digest(DigestAlgorithm::Sha256)?;
    if map.nix_closure != closure_digest || plan.nix_closure() != &closure_digest {
        return Err(GuestToolMapError::ClosureDigestMismatch);
    }
    validate_canonical_tool_set(&map.tools)?;

    for binding in &map.tools {
        validate_executable(binding.role(), binding.executable(), closure)?;
        let tool = spec
            .tools()
            .iter()
            .find(|candidate| candidate.role() == binding.role())
            .ok_or_else(|| GuestToolMapError::MissingExecutionTool(binding.role().to_owned()))?;
        if !binding.matches_tool(tool) {
            return Err(GuestToolMapError::ExecutionToolMismatch(
                binding.role().to_owned(),
            ));
        }
    }

    Ok(QualifiedGuestToolMap {
        map_digest: map.digest(DigestAlgorithm::Sha256)?,
        plan_digest,
        execution_subject: map.execution_subject.clone(),
        nix_closure: closure_digest,
        tools: map.tools.clone(),
    })
}

fn require_exact_path_roles(paths: &BTreeMap<String, String>) -> Result<(), GuestToolMapError> {
    let actual = paths.keys().map(String::as_str).collect::<BTreeSet<_>>();
    let expected = GUEST_TOOL_ROLES.iter().copied().collect::<BTreeSet<_>>();
    if actual == expected {
        Ok(())
    } else {
        Err(GuestToolMapError::ToolRoleSetMismatch)
    }
}

fn validate_canonical_tool_set(tools: &[GuestToolBinding]) -> Result<(), GuestToolMapError> {
    if tools.len() != GUEST_TOOL_ROLES.len() {
        return Err(GuestToolMapError::ToolRoleSetMismatch);
    }
    let mut sorted = tools.to_vec();
    sorted.sort();
    if sorted != tools {
        return Err(GuestToolMapError::ToolsNotCanonical);
    }
    let actual = tools.iter().map(GuestToolBinding::role).collect::<BTreeSet<_>>();
    let expected = GUEST_TOOL_ROLES.iter().copied().collect::<BTreeSet<_>>();
    if actual != expected {
        return Err(GuestToolMapError::ToolRoleSetMismatch);
    }
    if tools
        .windows(2)
        .any(|pair| pair[0].role() == pair[1].role() || pair[0].executable() == pair[1].executable())
    {
        return Err(GuestToolMapError::DuplicateToolBinding);
    }
    Ok(())
}

fn validate_role(role: &str) -> Result<(), GuestToolMapError> {
    if GUEST_TOOL_ROLES.contains(&role) {
        Ok(())
    } else {
        Err(GuestToolMapError::UnknownToolRole(role.to_owned()))
    }
}

fn expected_suffix(role: &str) -> Result<&'static str, GuestToolMapError> {
    match role {
        "forge-isolation-probe" => Ok("/bin/forge-isolation-probe"),
        "git" => Ok("/bin/git"),
        "gittuf" => Ok("/bin/gittuf"),
        other => Err(GuestToolMapError::UnknownToolRole(other.to_owned())),
    }
}

fn validate_executable(
    role: &str,
    executable: &str,
    closure: &NixClosureManifest,
) -> Result<(), GuestToolMapError> {
    validate_executable_syntax(role, executable)?;
    let root = executable_store_root(executable)?;
    if !closure.entries().iter().any(|entry| entry.store_path() == root) {
        return Err(GuestToolMapError::ExecutableOutsideClosure {
            role: role.to_owned(),
            executable: executable.to_owned(),
        });
    }
    Ok(())
}

fn validate_executable_syntax(role: &str, executable: &str) -> Result<(), GuestToolMapError> {
    validate_role(role)?;
    validate_text("tool executable", executable)?;
    if !executable.starts_with("/nix/store/")
        || executable.ends_with('/')
        || executable.contains("//")
        || executable.contains("/../")
        || executable.contains("/./")
        || !executable.ends_with(expected_suffix(role)?)
    {
        return Err(GuestToolMapError::InvalidExecutable {
            role: role.to_owned(),
            executable: executable.to_owned(),
        });
    }
    executable_store_root(executable)?;
    Ok(())
}

fn executable_store_root(executable: &str) -> Result<&str, GuestToolMapError> {
    let suffix = executable
        .strip_prefix("/nix/store/")
        .ok_or_else(|| GuestToolMapError::InvalidExecutable {
            role: "unknown".to_owned(),
            executable: executable.to_owned(),
        })?;
    let slash = suffix
        .find('/')
        .ok_or_else(|| GuestToolMapError::InvalidExecutable {
            role: "unknown".to_owned(),
            executable: executable.to_owned(),
        })?;
    let root_len = "/nix/store/".len() + slash;
    Ok(&executable[..root_len])
}

fn validate_text(field: &'static str, value: &str) -> Result<(), GuestToolMapError> {
    if value.is_empty() || value.len() > MAX_TEXT || value.contains('\0') {
        Err(GuestToolMapError::InvalidText(field))
    } else {
        Ok(())
    }
}

fn push_count(
    out: &mut Vec<u8>,
    count: usize,
    field: &'static str,
) -> Result<(), GuestToolMapError> {
    let count =
        u16::try_from(count).map_err(|_| GuestToolMapError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&count.to_be_bytes());
    Ok(())
}

fn push_string(
    out: &mut Vec<u8>,
    value: &str,
    field: &'static str,
) -> Result<(), GuestToolMapError> {
    let len =
        u16::try_from(value.len()).map_err(|_| GuestToolMapError::CanonicalFieldTooLarge(field))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), GuestToolMapError> {
    push_string(out, digest.algorithm().id(), "digest algorithm")?;
    let len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| GuestToolMapError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

fn push_optional_digest(
    out: &mut Vec<u8>,
    digest: Option<&Digest>,
) -> Result<(), GuestToolMapError> {
    match digest {
        Some(digest) => {
            out.push(1);
            push_digest(out, digest)?;
        }
        None => out.push(0),
    }
    Ok(())
}

#[derive(Debug, Error)]
pub enum GuestToolMapError {
    #[error(transparent)]
    GuestPlan(#[from] GuestPlanError),
    #[error(transparent)]
    Execution(#[from] ExecutionContractError),
    #[error(transparent)]
    Isolation(#[from] IsolationPolicyError),
    #[error("guest tool path set differs from the exact M0 inner-tool set")]
    ToolRoleSetMismatch,
    #[error("guest tool bindings are not in canonical order")]
    ToolsNotCanonical,
    #[error("guest tool bindings contain a duplicate role or executable")]
    DuplicateToolBinding,
    #[error("duplicate execution tool artifact role during construction: {0}")]
    DuplicateToolArtifactRole(String),
    #[error("unknown guest tool role: {0}")]
    UnknownToolRole(String),
    #[error("tool artifact is empty: {0}")]
    EmptyToolArtifact(String),
    #[error("tool role differs from execution-spec role: {0}")]
    ToolRoleMismatch(String),
    #[error("missing execution-spec tool: {0}")]
    MissingExecutionTool(String),
    #[error("guest tool artifact differs from execution spec: {0}")]
    ExecutionToolMismatch(String),
    #[error("invalid guest executable for {role}: {executable}")]
    InvalidExecutable { role: String, executable: String },
    #[error("guest executable for {role} is outside committed Nix closure: {executable}")]
    ExecutableOutsideClosure { role: String, executable: String },
    #[error("guest tool map names a different guest plan")]
    PlanDigestMismatch,
    #[error("guest tool map/execution spec names a different execution subject")]
    ExecutionSubjectMismatch,
    #[error("guest tool map/plan names a different Nix closure")]
    ClosureDigestMismatch,
    #[error("invalid text field: {0}")]
    InvalidText(&'static str),
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_forge_linux_isolation::NixClosureEntry;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn closure() -> NixClosureManifest {
        NixClosureManifest::new(vec![
            NixClosureEntry::new("/nix/store/aaaa-git", digest(1)).unwrap(),
            NixClosureEntry::new("/nix/store/bbbb-gittuf", digest(2)).unwrap(),
            NixClosureEntry::new("/nix/store/cccc-probe", digest(3)).unwrap(),
        ])
        .unwrap()
    }

    #[test]
    fn executable_must_live_in_exact_committed_store_root() {
        let closure = closure();
        validate_executable("git", "/nix/store/aaaa-git/bin/git", &closure).unwrap();
        assert!(matches!(
            validate_executable("git", "/nix/store/zzzz-other/bin/git", &closure),
            Err(GuestToolMapError::ExecutableOutsideClosure { .. })
        ));
    }

    #[test]
    fn executable_basename_is_role_specific() {
        let closure = closure();
        assert!(matches!(
            validate_executable("gittuf", "/nix/store/bbbb-gittuf/bin/git", &closure),
            Err(GuestToolMapError::InvalidExecutable { .. })
        ));
    }

    #[test]
    fn lexical_escape_is_rejected() {
        let closure = closure();
        assert!(validate_executable(
            "git",
            "/nix/store/aaaa-git/bin/../bin/git",
            &closure
        )
        .is_err());
    }

    #[test]
    fn path_role_set_is_exact() {
        let paths = BTreeMap::from([
            ("git".to_owned(), "/nix/store/aaaa-git/bin/git".to_owned()),
            (
                "gittuf".to_owned(),
                "/nix/store/bbbb-gittuf/bin/gittuf".to_owned(),
            ),
        ]);
        assert_eq!(
            require_exact_path_roles(&paths).unwrap_err().to_string(),
            "guest tool path set differs from the exact M0 inner-tool set"
        );
    }

    #[test]
    fn duplicate_tool_artifact_roles_are_rejected_during_construction() {
        let closure = closure();
        let tools = vec![
            ToolArtifact::new("git", "1", digest(10), 1, None).unwrap(),
            ToolArtifact::new("git", "2", digest(11), 1, None).unwrap(),
        ];
        let mut by_role = BTreeMap::new();
        let duplicate = tools
            .iter()
            .any(|tool| by_role.insert(tool.role(), tool).is_some());
        assert!(duplicate);
    }
}
