//! Deterministic instruction-subset policy for canonical WASM materializers.
//!
//! This theorem composes the exact ABI from the previous tranche with a stricter
//! wasmparser feature set. Floating point, SIMD/relaxed-SIMD, threads/atomics,
//! reference/GC features, exceptions, tail calls, memory64 and multi-memory are
//! disabled at validation time. Module-side `memory.grow` is additionally
//! forbidden so materializer-visible memory size changes are host-controlled.
//!
//! The result is still a static code theorem. Fresh instantiation, fuel, host
//! input placement, output bounds and provider payload materialization remain
//! runtime obligations.

use mycelix_institutional_core::Digest32;
use mycelix_integration_materializer_abi::QualifiedWasmMaterializerAbi;
use thiserror::Error;
use wasmparser::{Operator, Parser, Payload, Validator, WasmFeatures};

pub const DETERMINISM_PROFILE: &str =
    "mycelix-integration-materializer-determinism-v1-integer-linear-memory";
const DOMAIN_DETERMINISM: &[u8] = b"mycelix/integration/materializer-determinism/v1";

#[derive(Debug)]
pub struct QualifiedDeterministicWasmMaterializer {
    abi: QualifiedWasmMaterializerAbi,
    determinism_digest: Digest32,
}

impl QualifiedDeterministicWasmMaterializer {
    pub fn abi(&self) -> &QualifiedWasmMaterializerAbi {
        &self.abi
    }

    pub fn determinism_digest(&self) -> Digest32 {
        self.determinism_digest
    }

    pub fn determinism_profile(&self) -> &'static str {
        DETERMINISM_PROFILE
    }

    pub const fn floating_point_forbidden_here(&self) -> bool {
        true
    }

    pub const fn simd_forbidden_here(&self) -> bool {
        true
    }

    pub const fn threads_and_atomics_forbidden_here(&self) -> bool {
        true
    }

    pub const fn references_and_gc_forbidden_here(&self) -> bool {
        true
    }

    pub const fn exceptions_and_tail_calls_forbidden_here(&self) -> bool {
        true
    }

    pub const fn module_memory_grow_forbidden_here(&self) -> bool {
        true
    }

    pub const fn deterministic_instruction_subset_verified_here(&self) -> bool {
        true
    }

    pub const fn fresh_instance_enforced_here(&self) -> bool {
        false
    }

    pub const fn fuel_bound_enforced_here(&self) -> bool {
        false
    }

    pub const fn input_region_bounds_verified_here(&self) -> bool {
        false
    }

    pub const fn output_region_bounds_verified_here(&self) -> bool {
        false
    }

    pub const fn end_to_end_materialization_determinism_verified_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_deterministic_wasm_materializer(
    abi: QualifiedWasmMaterializerAbi,
) -> Result<QualifiedDeterministicWasmMaterializer, MaterializerDeterminismError> {
    validate_deterministic_subset(abi.policy().artifact().module_bytes())?;
    let determinism_digest = determinism_digest(&abi);
    Ok(QualifiedDeterministicWasmMaterializer {
        abi,
        determinism_digest,
    })
}

fn deterministic_features() -> WasmFeatures {
    let mut features = WasmFeatures::default();
    features.remove(
        WasmFeatures::FLOATS
            | WasmFeatures::SIMD
            | WasmFeatures::RELAXED_SIMD
            | WasmFeatures::THREADS
            | WasmFeatures::TAIL_CALL
            | WasmFeatures::REFERENCE_TYPES
            | WasmFeatures::MULTI_MEMORY
            | WasmFeatures::EXCEPTIONS
            | WasmFeatures::MEMORY64
            | WasmFeatures::COMPONENT_MODEL
            | WasmFeatures::FUNCTION_REFERENCES
            | WasmFeatures::GC
            | WasmFeatures::GC_TYPES,
    );
    features
}

fn validate_deterministic_subset(bytes: &[u8]) -> Result<(), MaterializerDeterminismError> {
    Validator::new_with_features(deterministic_features()).validate_all(bytes)?;

    for payload in Parser::new(0).parse_all(bytes) {
        if let Payload::CodeSectionEntry(body) = payload? {
            let mut operators = body.get_operators_reader()?;
            while !operators.eof() {
                if matches!(operators.read()?, Operator::MemoryGrow { .. }) {
                    return Err(MaterializerDeterminismError::ModuleMemoryGrowForbidden);
                }
            }
        }
    }
    Ok(())
}

fn determinism_digest(abi: &QualifiedWasmMaterializerAbi) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_DETERMINISM);
    frame(&mut h, DETERMINISM_PROFILE.as_bytes());
    frame(&mut h, &abi.abi_digest().0);
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum MaterializerDeterminismError {
    #[error("materializer module may not execute memory.grow")]
    ModuleMemoryGrowForbidden,
    #[error("materializer violates the deterministic WebAssembly feature subset: {0}")]
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

    fn base_module(code_body: &[u8]) -> Vec<u8> {
        let mut module = b"\0asm\x01\0\0\0".to_vec();
        // materialize: (i32, i32) -> i64
        push_section(&mut module, 1, &[1, 0x60, 2, 0x7f, 0x7f, 1, 0x7e]);
        push_section(&mut module, 3, &[1, 0]);
        push_section(&mut module, 5, &[1, 1, 1, 2]);
        push_section(
            &mut module,
            7,
            &[
                2,
                6, b'm', b'e', b'm', b'o', b'r', b'y', 2, 0,
                11, b'm', b'a', b't', b'e', b'r', b'i', b'a', b'l', b'i', b'z', b'e', 0, 0,
            ],
        );
        let mut code = vec![1, code_body.len() as u8];
        code.extend_from_slice(code_body);
        push_section(&mut module, 10, &code);
        module
    }

    #[test]
    fn integer_only_module_qualifies() {
        // locals=0; i64.const 0; end
        let module = base_module(&[0, 0x42, 0, 0x0b]);
        validate_deterministic_subset(&module).unwrap();
    }

    #[test]
    fn floating_point_instruction_fails_closed() {
        // locals=0; f32.const 0; drop; i64.const 0; end
        let module = base_module(&[0, 0x43, 0, 0, 0, 0, 0x1a, 0x42, 0, 0x0b]);
        assert!(matches!(
            validate_deterministic_subset(&module),
            Err(MaterializerDeterminismError::Wasm(_))
        ));
    }

    #[test]
    fn module_memory_grow_fails_closed() {
        // locals=0; i32.const 1; memory.grow 0; drop; i64.const 0; end
        let module = base_module(&[0, 0x41, 1, 0x40, 0, 0x1a, 0x42, 0, 0x0b]);
        assert!(matches!(
            validate_deterministic_subset(&module),
            Err(MaterializerDeterminismError::ModuleMemoryGrowForbidden)
        ));
    }

    #[test]
    fn ordinary_integer_memory_access_remains_allowed() {
        // locals=0; i32.const 0; i32.load align=2 offset=0; drop; i64.const 0; end
        let module = base_module(&[0, 0x41, 0, 0x28, 2, 0, 0x1a, 0x42, 0, 0x0b]);
        validate_deterministic_subset(&module).unwrap();
    }
}
