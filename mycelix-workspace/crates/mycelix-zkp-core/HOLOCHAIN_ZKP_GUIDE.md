> This guide is an integration pattern, not a deployment certification. See `CAPABILITY_MATRIX.md`.

# Adding ZKP to Any Holochain hApp

## Quick Start

```rust
// 1. Add to your zome's Cargo.toml:
// mycelix-zkp-core = { path = "path/to/crates/mycelix-zkp-core" }

// 2. Generate a proof (client-side, native Rust):
use mycelix_zkp_core::circuits::range_proof::{prove_range, verify_range};

let proof = prove_range(value, min, max, commitment)?;

// 3. Submit to zome via callZome:
// callZome("your_bridge", "submit_proof", { proof_bytes: proof.to_bytes() })

// 4. Verify off-chain:
verify_range(proof, min, max, commitment)?;
```

## Architecture

```
Client (native)              Holochain Zome (WASM)         Off-chain Verifier
┌──────────────┐            ┌─────────────────┐           ┌──────────────┐
│ prove_range() │  callZome  │ validate_struct()│  DHT read │ verify_range()│
│ Dilithium sign│ ─────────→ │ store on DHT    │ ────────→ │ Ed25519 sign  │
│               │            │ link entries     │           │ store attest  │
└──────────────┘            └─────────────────┘           └──────────────┘
```

**Why off-chain verification?** Winterfell CAN compile to WASM (no_std),
but it's untested in Holochain's wasmer runtime. The off-chain pattern avoids trusting an unverified Wasmer integration. This
archive does not include the referenced external conductor evidence.

## Step-by-Step Integration

### 1. Choose your proof type

| Use Case | Circuit | Backend | AND Constraints |
|---|---|---|---|
| Value in range | `range_proof` | Winterfell | ~32 per 16-bit value |
| Consciousness tier | `consciousness` | Winterfell | ~32 (46ms) |
| HDC/Binius examples | — | Binius | Reserved only; not implemented in this crate |

### 2. Add domain tag

```rust
// In mycelix-zkp-core/src/domain.rs, add:
pub fn tag_your_cluster() -> DomainTag {
    DomainTag::new("YourCluster", "YourProofType", 1)
}
```

### 3. Create proof entry type (integrity zome)

```rust
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct YourProofEntry {
    pub proof_bytes: Vec<u8>,        // STARK proof
    pub commitment: Vec<u8>,         // 32-byte SHA-256
    pub domain_tag: String,          // "ZTML:YourCluster:..."
    pub generated_at: Timestamp,
    pub verification_record_hash: Option<ActionHash>,
    // Store an append-only SupplyVerificationRecord separately; never a bare bool.
}
```

### 4. Add submission extern (coordinator zome)

```rust
#[hdk_extern]
pub fn submit_proof(input: SubmitProofInput) -> ExternResult<Record> {
    // Validate structure
    if input.proof_bytes.is_empty() { return Err(...) }
    if input.proof_bytes.len() > 500_000 { return Err(...) }
    if input.commitment.len() != 32 { return Err(...) }

    // Create entry
    let entry = YourProofEntry { ... };
    let hash = create_entry(&EntryTypes::YourProof(entry))?;

    // Link to relevant entity
    create_link(entity_hash, hash.clone(), LinkTypes::EntityToProof, ())?;

    get(hash, GetOptions::default())
}
```

### 5. Off-chain verifier

```rust
// Native Rust binary (not WASM):
use mycelix_zkp_core::circuits::range_proof::verify_range;
use winterfell::Proof;

let proof = Proof::from_bytes(&proof_bytes)?;
verify_range(proof, min, max, commitment)?;

// Construct an append-only verification record containing the proof hash,
// statement commitment, verifier identity/version, policy hash, backend,
// explicit outcome, and timestamp. Sign its canonical digest, then submit it.
```

### 6. Consciousness gating (optional)

```rust
// If your operation requires consciousness tier:
use mycelix_zkp_core::consciousness::{prove_consciousness_tier, ConsciousnessProofRequest, ConsciousnessTier};

let request = ConsciousnessProofRequest {
    phi_score: 0.55,
    required_tier: ConsciousnessTier::Steward,
    agent_did: "did:mycelix:agent001".to_string(),
};
let proof = prove_consciousness_tier(&request)?;
// Submit proof.proof_bytes alongside your operation
```

## Available Circuits

| Circuit | Location | Tests | Measured |
|---|---|---|---|
| Range proof | `src/circuits/range_proof.rs` | 6 | 27ms (32-bit) |
| Winterfell XOR | `src/circuits/winterfell_xor.rs` | 4 | 23.8s (16Kbit) |
| Consciousness | `src/consciousness.rs` | 5 | 46ms |
| PoGQ (FL) | `src/pogq.rs` | 8 | 382ns sim |
| Dilithium5 | `src/dilithium.rs` | Feature-gated tests | No fresh measurement in this archive |

## Feature Flags

```toml
[dependencies]
mycelix-zkp-core = { path = "...", features = ["backend-winterfell"] }
# Options: backend-winterfell, backend-miden, backend-risc0 (structural only), dilithium, full
```

- `backend-winterfell`: circuit-specific Winterfell dependencies
- `backend-miden`: circuit-specific Miden 0.23.5 dependencies
- `backend-risc0`: structural compatibility adapter only; no verifier dependency
- `dilithium`: Dilithium5 envelope authentication
- `full`: Winterfell + structural RISC0 adapter + Dilithium; it does not include Miden
