//! Static capability policy for hash-pinned integration WASM materializers.
//!
//! The policy is intentionally stricter than general WebAssembly. A v0.1
//! materializer may have no imports, tables, elements, exception tags, or start
//! function; exactly one bounded 32-bit linear memory; and only the `memory` +
//! `materialize` exports. With no imports, the module has no host route to
//! network, clock, randomness, filesystem, environment, or secrets.
//!
//! Internal globals are permitted because ordinary Rust/C WebAssembly toolchains
//! commonly use an internal stack-pointer global. They cannot be imported or
//! exported under this policy, and the eventual execution theorem MUST create a
//! fresh module instance for every materialization so internal mutable state is
//! never reused across requests.
//!
//! This still does not execute the module or prove instruction-level
//! determinism. Fuel, runtime configuration, ABI signature and output semantics
//! remain separate later theorems.

use mycelix_institutional_core::Digest32;
use mycelix_integration_materializer_artifact::QualifiedWasmMaterializerArtifact;
use thiserror::Error;
use wasmparser::{Encoding, ExternalKind, Parser, Payload, Validator};

pub const POLICY_PROFILE: &str =
    "mycelix-integration-materializer-static-policy-v1-blake3-framed";
const DOMAIN_POLICY: &[u8] = b"mycelix/integration/materializer-static-policy/v1";
const MAX_LINEAR_MEMORY_PAGES: u64 = 256; // 16 MiB at the standard 64 KiB page size.

#[derive(Debug)]
pub struct QualifiedWasmMaterializerPolicy {
    artifact: QualifiedWasmMaterializerArtifact,
    initial_memory_pages: u64,
    maximum_memory_pages: u64,
    materialize_function_index: u32,
    policy_digest: Digest32,
}

impl QualifiedWasmMaterializerPolicy {
    pub fn artifact(&self) -> &QualifiedWasmMaterializerArtifact {
        &self.artifact
    }

    pub fn initial_memory_pages(&self) -> u64 {
        self.initial_memory_pages
    }

    pub fn maximum_memory_pages(&self) -> u64 {
        self.maximum_memory_pages
    }

    pub fn materialize_function_index(&self) -> u32 {
        self.materialize_function_index
    }

    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn policy_profile(&self) -> &'static str {
        POLICY_PROFILE
    }

    pub const fn no_imports_verified_here(&self) -> bool {
        true
    }

    pub const fn no_host_io_via_imports_verified_here(&self) -> bool {
        true
    }

    pub const fn no_tables_or_elements_verified_here(&self) -> bool {
        true
    }

    pub const fn bounded_linear_memory_verified_here(&self) -> bool {
        true
    }

    pub const fn no_start_function_verified_here(&self) -> bool {
        true
    }

    pub const fn narrow_export_surface_verified_here(&self) -> bool {
        true
    }

    pub const fn wasm_structural_validity_verified_here(&self) -> bool {
        true
    }

    /// Runtime requirement, not a claim that this static qualifier instantiated
    /// or executed the module.
    pub const fn fresh_instance_per_materialization_required_by_policy(&self) -> bool {
        true
    }

    pub const fn materialize_signature_verified_here(&self) -> bool {
        false
    }

    pub const fn instruction_determinism_verified_here(&self) -> bool {
        false
    }

    pub const fn fuel_bound_enforced_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_wasm_materializer_policy(
    artifact: QualifiedWasmMaterializerArtifact,
) -> Result<QualifiedWasmMaterializerPolicy, MaterializerPolicyError> {
    let facts = analyze_module(artifact.module_bytes())?;
    let policy_digest = policy_digest(&artifact, &facts);
    Ok(QualifiedWasmMaterializerPolicy {
        artifact,
        initial_memory_pages: facts.initial_memory_pages,
        maximum_memory_pages: facts.maximum_memory_pages,
        materialize_function_index: facts.materialize_function_index,
        policy_digest,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PolicyFacts {
    initial_memory_pages: u64,
    maximum_memory_pages: u64,
    materialize_function_index: u32,
}

fn analyze_module(bytes: &[u8]) -> Result<PolicyFacts, MaterializerPolicyError> {
    // Structural validation comes first so section indices/types cannot be
    // smuggled through a parser-only policy scan.
    Validator::new().validate_all(bytes)?;

    let mut saw_module_version = false;
    let mut memory: Option<(u64, u64)> = None;
    let mut memory_export = false;
    let mut materialize_export: Option<u32> = None;
    let mut export_count = 0usize;

    for payload in Parser::new(0).parse_all(bytes) {
        match payload? {
            Payload::Version { encoding, .. } => {
                if encoding != Encoding::Module {
                    return Err(MaterializerPolicyError::ComponentModelForbidden);
                }
                saw_module_version = true;
            }
            Payload::ImportSection(reader) => {
                if reader.count() != 0 {
                    return Err(MaterializerPolicyError::ImportsForbidden);
                }
            }
            Payload::TableSection(reader) => {
                if reader.count() != 0 {
                    return Err(MaterializerPolicyError::TablesForbidden);
                }
            }
            Payload::ElementSection(reader) => {
                if reader.count() != 0 {
                    return Err(MaterializerPolicyError::ElementsForbidden);
                }
            }
            Payload::MemorySection(reader) => {
                for item in reader {
                    let memory_type = item?;
                    if memory.is_some() {
                        return Err(MaterializerPolicyError::MultipleMemoriesForbidden);
                    }
                    if memory_type.memory64 || memory_type.shared {
                        return Err(MaterializerPolicyError::UnsupportedMemoryKind);
                    }
                    if memory_type.page_size_log2.is_some() {
                        return Err(MaterializerPolicyError::CustomPageSizeForbidden);
                    }
                    let maximum = memory_type
                        .maximum
                        .ok_or(MaterializerPolicyError::MemoryMaximumRequired)?;
                    if memory_type.initial > maximum || maximum > MAX_LINEAR_MEMORY_PAGES {
                        return Err(MaterializerPolicyError::MemoryBoundExceeded);
                    }
                    memory = Some((memory_type.initial, maximum));
                }
            }
            Payload::ExportSection(reader) => {
                for item in reader {
                    let export = item?;
                    export_count += 1;
                    match (export.name, export.kind) {
                        ("memory", ExternalKind::Memory) => {
                            if memory_export || export.index != 0 {
                                return Err(MaterializerPolicyError::InvalidMemoryExport);
                            }
                            memory_export = true;
                        }
                        ("materialize", ExternalKind::Func) => {
                            if materialize_export.replace(export.index).is_some() {
                                return Err(MaterializerPolicyError::DuplicateMaterializeExport);
                            }
                        }
                        _ => return Err(MaterializerPolicyError::UnexpectedExport),
                    }
                }
            }
            Payload::StartSection { .. } => {
                return Err(MaterializerPolicyError::StartFunctionForbidden);
            }
            Payload::TagSection(reader) => {
                if reader.count() != 0 {
                    return Err(MaterializerPolicyError::ExceptionTagsForbidden);
                }
            }
            Payload::TypeSection(_)
            | Payload::FunctionSection(_)
            | Payload::GlobalSection(_)
            | Payload::DataCountSection { .. }
            | Payload::DataSection(_)
            | Payload::CodeSectionStart { .. }
            | Payload::CodeSectionEntry(_)
            | Payload::CustomSection(_)
            | Payload::End(_) => {}
            _ => return Err(MaterializerPolicyError::UnsupportedSection),
        }
    }

    if !saw_module_version {
        return Err(MaterializerPolicyError::NotCoreModule);
    }
    let (initial_memory_pages, maximum_memory_pages) =
        memory.ok_or(MaterializerPolicyError::ExactlyOneMemoryRequired)?;
    if !memory_export {
        return Err(MaterializerPolicyError::MemoryExportRequired);
    }
    let materialize_function_index =
        materialize_export.ok_or(MaterializerPolicyError::MaterializeExportRequired)?;
    if export_count != 2 {
        return Err(MaterializerPolicyError::UnexpectedExport);
    }

    Ok(PolicyFacts {
        initial_memory_pages,
        maximum_memory_pages,
        materialize_function_index,
    })
}

fn policy_digest(artifact: &QualifiedWasmMaterializerArtifact, facts: &PolicyFacts) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_POLICY);
    frame(&mut h, POLICY_PROFILE.as_bytes());
    frame(&mut h, &artifact.qualification_digest().0);
    frame(&mut h, &facts.initial_memory_pages.to_le_bytes());
    frame(&mut h, &facts.maximum_memory_pages.to_le_bytes());
    frame(&mut h, &facts.materialize_function_index.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum MaterializerPolicyError {
    #[error("materializer bytes are not a validated core WebAssembly module")]
    NotCoreModule,
    #[error("WebAssembly component-model materializers are not permitted in v0.1")]
    ComponentModelForbidden,
    #[error("materializer imports are forbidden")]
    ImportsForbidden,
    #[error("materializer tables are forbidden")]
    TablesForbidden,
    #[error("materializer element segments are forbidden")]
    ElementsForbidden,
    #[error("materializer exception tags are forbidden")]
    ExceptionTagsForbidden,
    #[error("materializer start functions are forbidden")]
    StartFunctionForbidden,
    #[error("materializer must define exactly one linear memory")]
    ExactlyOneMemoryRequired,
    #[error("materializer memory must be 32-bit, unshared and standard-page")]
    UnsupportedMemoryKind,
    #[error("materializer custom WebAssembly page sizes are forbidden")]
    CustomPageSizeForbidden,
    #[error("materializer memory must declare a finite maximum")]
    MemoryMaximumRequired,
    #[error("materializer memory exceeds the v0.1 bounded-memory policy")]
    MemoryBoundExceeded,
    #[error("multiple materializer memories are forbidden")]
    MultipleMemoriesForbidden,
    #[error("materializer must export memory index 0 as `memory`")]
    MemoryExportRequired,
    #[error("materializer `memory` export is invalid or duplicated")]
    InvalidMemoryExport,
    #[error("materializer must export one function named `materialize`")]
    MaterializeExportRequired,
    #[error("materializer exports `materialize` more than once")]
    DuplicateMaterializeExport,
    #[error("materializer has an export outside the v0.1 ABI surface")]
    UnexpectedExport,
    #[error("materializer uses a WebAssembly section outside the v0.1 policy")]
    UnsupportedSection,
    #[error(transparent)]
    Wasm(#[from] wasmparser::BinaryReaderError),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn push_section(module: &mut Vec<u8>, id: u8, payload: &[u8]) {
        assert!(payload.len() < 128);
        module.push(id);
        module.push(payload.len() as u8);
        module.extend_from_slice(payload);
    }

    fn module(
        with_import: bool,
        bounded_memory: bool,
        with_start: bool,
        extra_export: bool,
        with_internal_global: bool,
    ) -> Vec<u8> {
        let mut module = b"\0asm\x01\0\0\0".to_vec();

        // type[0] = () -> ()
        push_section(&mut module, 1, &[1, 0x60, 0, 0]);

        if with_import {
            // import env.clock as function type 0
            push_section(
                &mut module,
                2,
                &[1, 3, b'e', b'n', b'v', 5, b'c', b'l', b'o', b'c', b'k', 0, 0],
            );
        }

        // One locally defined function, type 0.
        push_section(&mut module, 3, &[1, 0]);

        // One 32-bit unshared memory, optionally with finite maximum 2 pages.
        if bounded_memory {
            push_section(&mut module, 5, &[1, 1, 1, 2]);
        } else {
            push_section(&mut module, 5, &[1, 0, 1]);
        }

        if with_internal_global {
            // One internal mutable i32 global initialized to zero. This models
            // toolchain stack-pointer state; it is never exported and later
            // execution must instantiate a fresh module for every request.
            push_section(&mut module, 6, &[1, 0x7f, 1, 0x41, 0, 0x0b]);
        }

        let materialize_index = u8::from(with_import);
        let mut exports = vec![
            if extra_export { 3 } else { 2 },
            6, b'm', b'e', b'm', b'o', b'r', b'y', 2, 0,
            11, b'm', b'a', b't', b'e', b'r', b'i', b'a', b'l', b'i', b'z', b'e', 0,
            materialize_index,
        ];
        if extra_export {
            exports.extend_from_slice(&[5, b'd', b'e', b'b', b'u', b'g', 0, materialize_index]);
        }
        push_section(&mut module, 7, &exports);

        if with_start {
            push_section(&mut module, 8, &[materialize_index]);
        }

        // One body: no locals, end.
        push_section(&mut module, 10, &[1, 2, 0, 0x0b]);
        module
    }

    #[test]
    fn narrow_import_free_bounded_module_qualifies() {
        let facts = analyze_module(&module(false, true, false, false, false)).unwrap();
        assert_eq!(facts.initial_memory_pages, 1);
        assert_eq!(facts.maximum_memory_pages, 2);
        assert_eq!(facts.materialize_function_index, 0);
    }

    #[test]
    fn internal_global_is_compatible_with_fresh_instance_policy() {
        let facts = analyze_module(&module(false, true, false, false, true)).unwrap();
        assert_eq!(facts.materialize_function_index, 0);
    }

    #[test]
    fn any_import_fails_closed() {
        assert!(matches!(
            analyze_module(&module(true, true, false, false, false)),
            Err(MaterializerPolicyError::ImportsForbidden)
        ));
    }

    #[test]
    fn unbounded_memory_fails_closed() {
        assert!(matches!(
            analyze_module(&module(false, false, false, false, false)),
            Err(MaterializerPolicyError::MemoryMaximumRequired)
        ));
    }

    #[test]
    fn start_function_fails_closed() {
        assert!(matches!(
            analyze_module(&module(false, true, true, false, false)),
            Err(MaterializerPolicyError::StartFunctionForbidden)
        ));
    }

    #[test]
    fn extra_export_fails_closed() {
        assert!(matches!(
            analyze_module(&module(false, true, false, true, false)),
            Err(MaterializerPolicyError::UnexpectedExport)
        ));
    }
}
