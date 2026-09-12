//! Fresh-instance, fuel-bounded execution for already-qualified integration
//! materializers.
//!
//! This crate is deliberately provider-neutral and network-free. It consumes the
//! exact deterministic WASM theorem plus a same-call preexecution admission,
//! verifies they refer to the same provider profile/release/command, and only
//! then materializes captured provider request bytes.
//!
//! It still does not cross `DispatchStarted`, call a provider, or grant execution
//! authority. Those remain obligations of the final native effect-start theorem.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::ContentCommitment;
use mycelix_integration_materializer_abi::decode_output_region;
use mycelix_integration_materializer_determinism::QualifiedDeterministicWasmMaterializer;
use mycelix_integration_preexecution_admission::QualifiedIntegrationPreexecutionAdmission;
use thiserror::Error;
use wasmi::{
    CompilationMode, Config, Engine, Linker, Module, Store, StoreLimits, StoreLimitsBuilder,
};

pub const RUNTIME_PROFILE: &str =
    "mycelix-integration-materializer-runtime-v1-wasmi2-fresh-fuel-bounded";
const DOMAIN_MATERIALIZATION: &[u8] = b"mycelix/integration/materializer-runtime/v1";
const WASM_PAGE_BYTES: usize = 65_536;
const MAX_CANONICAL_INPUT_BYTES: usize = 1024 * 1024;
const MIN_FUEL: u64 = 1_000;
const MAX_FUEL: u64 = 100_000_000;

#[derive(Debug)]
struct HostState {
    limits: StoreLimits,
}

/// Captured provider request bytes produced under the exact deterministic
/// materializer + preexecution identities supplied to `materialize_provider_request`.
#[derive(Debug)]
pub struct MaterializedProviderRequest {
    bytes: Vec<u8>,
    input_commitment: ContentCommitment,
    output_commitment: ContentCommitment,
    admission_digest: Digest32,
    determinism_digest: Digest32,
    materialization_digest: Digest32,
    fuel_limit: u64,
    fuel_consumed: u64,
    memory_pages_after: u64,
}

impl MaterializedProviderRequest {
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn input_commitment(&self) -> &ContentCommitment {
        &self.input_commitment
    }

    pub fn output_commitment(&self) -> &ContentCommitment {
        &self.output_commitment
    }

    pub fn admission_digest(&self) -> Digest32 {
        self.admission_digest
    }

    pub fn determinism_digest(&self) -> Digest32 {
        self.determinism_digest
    }

    pub fn materialization_digest(&self) -> Digest32 {
        self.materialization_digest
    }

    pub fn fuel_limit(&self) -> u64 {
        self.fuel_limit
    }

    pub fn fuel_consumed(&self) -> u64 {
        self.fuel_consumed
    }

    pub fn memory_pages_after(&self) -> u64 {
        self.memory_pages_after
    }

    pub const fn fresh_instance_enforced_here(&self) -> bool {
        true
    }

    pub const fn fuel_bound_enforced_here(&self) -> bool {
        true
    }

    pub const fn no_host_imports_exposed_here(&self) -> bool {
        true
    }

    pub const fn exact_authorized_input_verified_here(&self) -> bool {
        true
    }

    pub const fn input_region_bounds_verified_here(&self) -> bool {
        true
    }

    pub const fn output_region_bounds_verified_here(&self) -> bool {
        true
    }

    pub const fn signed_output_size_bound_enforced_here(&self) -> bool {
        true
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        true
    }

    pub const fn dispatch_started_here(&self) -> bool {
        false
    }

    pub const fn provider_call_started_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Execute one exact qualified materializer in a brand-new interpreter store.
///
/// `canonical_command_bytes` must be the exact canonical command preimage already
/// bound by the preexecution admission. The materializer artifact must be the
/// exact release/profile pair selected by that same admission.
pub fn materialize_provider_request(
    materializer: &QualifiedDeterministicWasmMaterializer,
    admission: &QualifiedIntegrationPreexecutionAdmission,
    canonical_command_bytes: &[u8],
    fuel_limit: u64,
) -> Result<MaterializedProviderRequest, MaterializerRuntimeError> {
    validate_fuel(fuel_limit)?;
    if canonical_command_bytes.is_empty()
        || canonical_command_bytes.len() > MAX_CANONICAL_INPUT_BYTES
    {
        return Err(MaterializerRuntimeError::InvalidInputSize);
    }

    let binding = admission.execution_binding();
    let artifact = materializer.abi().policy().artifact();
    if artifact.release_commitment() != binding.materializer_release() {
        return Err(MaterializerRuntimeError::MaterializerReleaseMismatch);
    }
    if artifact.provider_profile_commitment() != binding.provider_profile_commitment() {
        return Err(MaterializerRuntimeError::ProviderProfileMismatch);
    }

    let input_commitment = ContentCommitment::sha256(canonical_command_bytes);
    if &input_commitment != binding.command_commitment() {
        return Err(MaterializerRuntimeError::CommandCommitmentMismatch);
    }

    let max_memory_pages = materializer.abi().policy().maximum_memory_pages();
    let max_memory_bytes_u64 = max_memory_pages
        .checked_mul(WASM_PAGE_BYTES as u64)
        .ok_or(MaterializerRuntimeError::MemoryLimitOverflow)?;
    let max_memory_bytes = usize::try_from(max_memory_bytes_u64)
        .map_err(|_| MaterializerRuntimeError::MemoryLimitOverflow)?;

    let mut config = Config::default();
    config
        .consume_fuel(true)
        .allow_start_fn(false)
        .floats(false)
        .wasm_tail_call(false)
        .wasm_reference_types(false)
        .wasm_memory64(false)
        .wasm_multi_memory(false)
        .compilation_mode(CompilationMode::Eager);
    let engine = Engine::new(&config);
    let module = Module::new(&engine, artifact.module_bytes())
        .map_err(|error| MaterializerRuntimeError::Engine(error.to_string()))?;

    let limits = StoreLimitsBuilder::new()
        .memory_size(max_memory_bytes)
        .instances(1)
        .memories(1)
        .tables(0)
        .build();
    let mut store = Store::new(&engine, HostState { limits });
    store.limiter(|state| &mut state.limits);
    store
        .set_fuel(fuel_limit)
        .map_err(|error| MaterializerRuntimeError::Engine(error.to_string()))?;

    // Intentionally empty linker: a qualified v0.1 materializer has no imports,
    // and this runtime defines no host capabilities even if an unqualified module
    // somehow reached Wasmi.
    let linker = Linker::<HostState>::new(&engine);
    let instance = linker
        .instantiate_and_start(&mut store, &module)
        .map_err(|error| MaterializerRuntimeError::Engine(error.to_string()))?;
    let memory = instance
        .get_memory(&store, "memory")
        .ok_or(MaterializerRuntimeError::MemoryExportMissing)?;
    let materialize = instance
        .get_typed_func::<(i32, i32), i64>(&store, "materialize")
        .map_err(|error| MaterializerRuntimeError::Engine(error.to_string()))?;

    // Place input after all module-initialized memory. This avoids overwriting
    // data segments or toolchain globals/stacks. Only the host may grow memory;
    // #662 rejects module-side memory.grow.
    let input_ptr = memory.data_size(&store);
    let required_end = input_ptr
        .checked_add(canonical_command_bytes.len())
        .ok_or(MaterializerRuntimeError::InputRegionOverflow)?;
    if required_end > max_memory_bytes {
        return Err(MaterializerRuntimeError::InputExceedsQualifiedMemory);
    }
    let current_pages = memory.size(&store);
    let required_pages = div_ceil(required_end, WASM_PAGE_BYTES) as u64;
    if required_pages > current_pages {
        memory
            .grow(&mut store, required_pages - current_pages)
            .map_err(|error| MaterializerRuntimeError::Memory(error.to_string()))?;
    }
    memory
        .write(&mut store, input_ptr, canonical_command_bytes)
        .map_err(|error| MaterializerRuntimeError::Memory(error.to_string()))?;

    let input_ptr_i32 = i32::try_from(input_ptr)
        .map_err(|_| MaterializerRuntimeError::InputPointerOutOfAbiRange)?;
    let input_len_i32 = i32::try_from(canonical_command_bytes.len())
        .map_err(|_| MaterializerRuntimeError::InputLengthOutOfAbiRange)?;
    let packed = materialize
        .call(&mut store, (input_ptr_i32, input_len_i32))
        .map_err(|error| MaterializerRuntimeError::Execution(error.to_string()))?;

    let (output_ptr_u32, output_len_u32) = decode_output_region(packed);
    if output_len_u32 > binding.max_payload_bytes() {
        return Err(MaterializerRuntimeError::OutputExceedsSignedProviderBound {
            observed: output_len_u32,
            maximum: binding.max_payload_bytes(),
        });
    }
    let output_ptr = output_ptr_u32 as usize;
    let output_len = output_len_u32 as usize;
    let output_end = output_ptr
        .checked_add(output_len)
        .ok_or(MaterializerRuntimeError::OutputRegionOverflow)?;
    let memory_size = memory.data_size(&store);
    if output_end > memory_size {
        return Err(MaterializerRuntimeError::OutputRegionOutOfBounds {
            pointer: output_ptr_u32,
            length: output_len_u32,
            memory_size,
        });
    }

    let mut bytes = vec![0_u8; output_len];
    memory
        .read(&store, output_ptr, &mut bytes)
        .map_err(|error| MaterializerRuntimeError::Memory(error.to_string()))?;
    let remaining_fuel = store
        .get_fuel()
        .map_err(|error| MaterializerRuntimeError::Engine(error.to_string()))?;
    let fuel_consumed = fuel_limit.saturating_sub(remaining_fuel);
    let memory_pages_after = memory.size(&store);
    let output_commitment = ContentCommitment::sha256(&bytes);
    let materialization_digest = materialization_digest(
        admission.admission_digest(),
        materializer.determinism_digest(),
        &input_commitment,
        &output_commitment,
        fuel_limit,
        fuel_consumed,
        memory_pages_after,
    );

    Ok(MaterializedProviderRequest {
        bytes,
        input_commitment,
        output_commitment,
        admission_digest: admission.admission_digest(),
        determinism_digest: materializer.determinism_digest(),
        materialization_digest,
        fuel_limit,
        fuel_consumed,
        memory_pages_after,
    })
}

fn validate_fuel(fuel_limit: u64) -> Result<(), MaterializerRuntimeError> {
    if !(MIN_FUEL..=MAX_FUEL).contains(&fuel_limit) {
        return Err(MaterializerRuntimeError::InvalidFuelLimit);
    }
    Ok(())
}

fn div_ceil(value: usize, divisor: usize) -> usize {
    value / divisor + usize::from(value % divisor != 0)
}

#[allow(clippy::too_many_arguments)]
fn materialization_digest(
    admission: Digest32,
    determinism: Digest32,
    input: &ContentCommitment,
    output: &ContentCommitment,
    fuel_limit: u64,
    fuel_consumed: u64,
    memory_pages_after: u64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_MATERIALIZATION);
    frame(&mut h, RUNTIME_PROFILE.as_bytes());
    frame(&mut h, &admission.0);
    frame(&mut h, &determinism.0);
    frame(&mut h, &input.digest);
    frame(&mut h, &output.digest);
    frame(&mut h, &fuel_limit.to_le_bytes());
    frame(&mut h, &fuel_consumed.to_le_bytes());
    frame(&mut h, &memory_pages_after.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum MaterializerRuntimeError {
    #[error("canonical materializer input must be non-empty and at most 1 MiB")]
    InvalidInputSize,
    #[error("materializer fuel limit is outside the v0.1 policy")]
    InvalidFuelLimit,
    #[error("materializer artifact release does not match the admitted provider profile")]
    MaterializerReleaseMismatch,
    #[error("materializer artifact provider profile does not match the admitted provider profile")]
    ProviderProfileMismatch,
    #[error("canonical materializer input does not match the admitted command commitment")]
    CommandCommitmentMismatch,
    #[error("qualified memory bound overflowed the host representation")]
    MemoryLimitOverflow,
    #[error("materializer does not export memory")]
    MemoryExportMissing,
    #[error("canonical input region arithmetic overflow")]
    InputRegionOverflow,
    #[error("canonical input does not fit the materializer's qualified memory maximum")]
    InputExceedsQualifiedMemory,
    #[error("canonical input pointer does not fit the i32 ABI")]
    InputPointerOutOfAbiRange,
    #[error("canonical input length does not fit the i32 ABI")]
    InputLengthOutOfAbiRange,
    #[error("materializer output exceeds signed provider bound: observed {observed}, maximum {maximum}")]
    OutputExceedsSignedProviderBound { observed: u32, maximum: u32 },
    #[error("materializer output region arithmetic overflow")]
    OutputRegionOverflow,
    #[error("materializer output region is outside fresh-instance memory: ptr {pointer}, len {length}, memory {memory_size}")]
    OutputRegionOutOfBounds {
        pointer: u32,
        length: u32,
        memory_size: usize,
    },
    #[error("Wasmi engine/module error: {0}")]
    Engine(String),
    #[error("Wasmi linear-memory error: {0}")]
    Memory(String),
    #[error("materializer execution trapped: {0}")]
    Execution(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fuel_policy_is_bounded() {
        assert!(validate_fuel(MIN_FUEL).is_ok());
        assert!(validate_fuel(MAX_FUEL).is_ok());
        assert!(matches!(
            validate_fuel(MIN_FUEL - 1),
            Err(MaterializerRuntimeError::InvalidFuelLimit)
        ));
        assert!(matches!(
            validate_fuel(MAX_FUEL + 1),
            Err(MaterializerRuntimeError::InvalidFuelLimit)
        ));
    }

    #[test]
    fn integer_page_rounding_is_stable() {
        assert_eq!(div_ceil(1, WASM_PAGE_BYTES), 1);
        assert_eq!(div_ceil(WASM_PAGE_BYTES, WASM_PAGE_BYTES), 1);
        assert_eq!(div_ceil(WASM_PAGE_BYTES + 1, WASM_PAGE_BYTES), 2);
    }
}
