//! Canonical function/memory ABI for statically qualified WASM materializers.
//!
//! v0.1 deliberately uses no host imports and exactly one exported linear memory.
//! The host grows fresh-instance memory to append canonical command bytes, then
//! calls:
//!
//! ```text
//! materialize(input_ptr: i32, input_len: i32) -> i64
//! ```
//!
//! The returned i64 is interpreted as an unsigned 64-bit packed output region:
//! high 32 bits = output pointer, low 32 bits = output length. Bounds, output
//! size, instruction determinism, fuel and fresh-instance execution remain later
//! runtime theorems.

use mycelix_institutional_core::Digest32;
use mycelix_integration_materializer_policy::QualifiedWasmMaterializerPolicy;
use thiserror::Error;
use wasmparser::{ExternalKind, FuncType, Parser, Payload, ValType};

pub const ABI_PROFILE: &str =
    "mycelix-integration-materializer-abi-v1-i32-i32-to-packed-i64";
pub const OUTPUT_REGION_PACKING_PROFILE: &str =
    "mycelix-integration-materializer-output-region-v1-ptr-hi32-len-lo32";
const DOMAIN_ABI: &[u8] = b"mycelix/integration/materializer-abi/v1";

/// Non-deserializable proof that the exact `materialize` export selected by the
/// static policy has the canonical v0.1 function signature.
#[derive(Debug)]
pub struct QualifiedWasmMaterializerAbi {
    policy: QualifiedWasmMaterializerPolicy,
    materialize_function_index: u32,
    materialize_type_index: u32,
    abi_digest: Digest32,
}

impl QualifiedWasmMaterializerAbi {
    pub fn policy(&self) -> &QualifiedWasmMaterializerPolicy {
        &self.policy
    }

    pub fn materialize_function_index(&self) -> u32 {
        self.materialize_function_index
    }

    pub fn materialize_type_index(&self) -> u32 {
        self.materialize_type_index
    }

    pub fn abi_digest(&self) -> Digest32 {
        self.abi_digest
    }

    pub fn abi_profile(&self) -> &'static str {
        ABI_PROFILE
    }

    pub fn output_region_packing_profile(&self) -> &'static str {
        OUTPUT_REGION_PACKING_PROFILE
    }

    pub const fn exact_materialize_signature_verified_here(&self) -> bool {
        true
    }

    pub const fn exported_memory_required_here(&self) -> bool {
        true
    }

    pub const fn canonical_output_packing_bound_here(&self) -> bool {
        true
    }

    pub const fn input_region_bounds_verified_here(&self) -> bool {
        false
    }

    pub const fn output_region_bounds_verified_here(&self) -> bool {
        false
    }

    pub const fn fresh_instance_enforced_here(&self) -> bool {
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

pub fn qualify_wasm_materializer_abi(
    policy: QualifiedWasmMaterializerPolicy,
) -> Result<QualifiedWasmMaterializerAbi, MaterializerAbiError> {
    let expected_function_index = policy.materialize_function_index();
    let facts = analyze_abi(policy.artifact().module_bytes(), expected_function_index)?;
    let abi_digest = abi_digest(&policy, &facts);
    Ok(QualifiedWasmMaterializerAbi {
        policy,
        materialize_function_index: facts.function_index,
        materialize_type_index: facts.type_index,
        abi_digest,
    })
}

/// Decode the canonical packed materializer result without asserting that the
/// returned region is in bounds. The runtime must validate the resulting region
/// against the fresh instance's exact memory before reading any bytes.
pub fn decode_output_region(value: i64) -> (u32, u32) {
    let bits = value as u64;
    ((bits >> 32) as u32, bits as u32)
}

/// Pure helper primarily for conformance tests and reference materializers.
pub fn encode_output_region(pointer: u32, length: u32) -> i64 {
    (((pointer as u64) << 32) | u64::from(length)) as i64
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AbiFacts {
    function_index: u32,
    type_index: u32,
}

fn analyze_abi(bytes: &[u8], expected_function_index: u32) -> Result<AbiFacts, MaterializerAbiError> {
    let mut types = Vec::<FuncType>::new();
    let mut function_type_indices = Vec::<u32>::new();
    let mut exported_materialize_index: Option<u32> = None;
    let mut exported_memory_index: Option<u32> = None;

    for payload in Parser::new(0).parse_all(bytes) {
        match payload? {
            Payload::TypeSection(reader) => {
                types = reader
                    .into_iter_err_on_gc_types()
                    .collect::<Result<Vec<_>, _>>()?;
            }
            Payload::ImportSection(reader) => {
                if reader.count() != 0 {
                    return Err(MaterializerAbiError::ImportsUnexpected);
                }
            }
            Payload::FunctionSection(reader) => {
                function_type_indices = reader.collect::<Result<Vec<_>, _>>()?;
            }
            Payload::ExportSection(reader) => {
                for item in reader {
                    let export = item?;
                    match (export.name, export.kind) {
                        ("materialize", ExternalKind::Func) => {
                            if exported_materialize_index.replace(export.index).is_some() {
                                return Err(MaterializerAbiError::DuplicateMaterializeExport);
                            }
                        }
                        ("memory", ExternalKind::Memory) => {
                            if exported_memory_index.replace(export.index).is_some() {
                                return Err(MaterializerAbiError::DuplicateMemoryExport);
                            }
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    let function_index =
        exported_materialize_index.ok_or(MaterializerAbiError::MaterializeExportMissing)?;
    if function_index != expected_function_index {
        return Err(MaterializerAbiError::MaterializeFunctionIndexMismatch {
            policy: expected_function_index,
            observed: function_index,
        });
    }
    if exported_memory_index != Some(0) {
        return Err(MaterializerAbiError::MemoryExportMismatch);
    }

    // The static policy forbids imports, so function index N is local function N.
    let type_index = *function_type_indices
        .get(function_index as usize)
        .ok_or(MaterializerAbiError::FunctionIndexOutOfBounds)?;
    let function_type = types
        .get(type_index as usize)
        .ok_or(MaterializerAbiError::TypeIndexOutOfBounds)?;
    if function_type.params() != [ValType::I32, ValType::I32]
        || function_type.results() != [ValType::I64]
    {
        return Err(MaterializerAbiError::SignatureMismatch);
    }

    Ok(AbiFacts {
        function_index,
        type_index,
    })
}

fn abi_digest(policy: &QualifiedWasmMaterializerPolicy, facts: &AbiFacts) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_ABI);
    frame(&mut h, ABI_PROFILE.as_bytes());
    frame(&mut h, OUTPUT_REGION_PACKING_PROFILE.as_bytes());
    frame(&mut h, &policy.policy_digest().0);
    frame(&mut h, &facts.function_index.to_le_bytes());
    frame(&mut h, &facts.type_index.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum MaterializerAbiError {
    #[error("imports unexpectedly appeared after static materializer qualification")]
    ImportsUnexpected,
    #[error("materialize export is missing")]
    MaterializeExportMissing,
    #[error("materialize export is duplicated")]
    DuplicateMaterializeExport,
    #[error("memory export is duplicated")]
    DuplicateMemoryExport,
    #[error("exported memory is not memory index 0")]
    MemoryExportMismatch,
    #[error("materialize function index differs from the static policy: policy {policy}, observed {observed}")]
    MaterializeFunctionIndexMismatch { policy: u32, observed: u32 },
    #[error("materialize function index is outside the defined function section")]
    FunctionIndexOutOfBounds,
    #[error("materialize type index is outside the function type section")]
    TypeIndexOutOfBounds,
    #[error("materialize must have exact signature (i32, i32) -> i64")]
    SignatureMismatch,
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

    fn module(params: &[u8], result: u8) -> Vec<u8> {
        let mut module = b"\0asm\x01\0\0\0".to_vec();

        let mut type_payload = vec![1, 0x60, params.len() as u8];
        type_payload.extend_from_slice(params);
        type_payload.extend_from_slice(&[1, result]);
        push_section(&mut module, 1, &type_payload);
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

        let instruction = if result == 0x7e { 0x42 } else { 0x41 };
        push_section(&mut module, 10, &[1, 4, 0, instruction, 0, 0x0b]);
        module
    }

    #[test]
    fn exact_i32_i32_to_i64_signature_qualifies() {
        let facts = analyze_abi(&module(&[0x7f, 0x7f], 0x7e), 0).unwrap();
        assert_eq!(facts.function_index, 0);
        assert_eq!(facts.type_index, 0);
    }

    #[test]
    fn wrong_parameter_shape_fails_closed() {
        assert!(matches!(
            analyze_abi(&module(&[0x7f], 0x7e), 0),
            Err(MaterializerAbiError::SignatureMismatch)
        ));
    }

    #[test]
    fn wrong_result_type_fails_closed() {
        assert!(matches!(
            analyze_abi(&module(&[0x7f, 0x7f], 0x7f), 0),
            Err(MaterializerAbiError::SignatureMismatch)
        ));
    }

    #[test]
    fn policy_function_index_substitution_fails_closed() {
        assert!(matches!(
            analyze_abi(&module(&[0x7f, 0x7f], 0x7e), 1),
            Err(MaterializerAbiError::MaterializeFunctionIndexMismatch {
                policy: 1,
                observed: 0,
            })
        ));
    }

    #[test]
    fn output_region_packing_round_trips() {
        let encoded = encode_output_region(0x1234_5678, 0x90ab_cdef);
        assert_eq!(decode_output_region(encoded), (0x1234_5678, 0x90ab_cdef));
    }
}
