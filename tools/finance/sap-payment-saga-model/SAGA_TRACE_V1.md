# Deterministic Finance Saga Trace Profile v1

**Status:** frozen initial generator contract; not yet the million-trace qualification.

## Purpose

This profile is the language-neutral input contract for model-based payment saga testing. Rust and Go must emit byte-identical traces for the same seed and trace index. The trace is a reproducible test input, not an oracle: the expected disposition must come from separately declared safety invariants and independently checked outcomes.

## Canonical encoding

All integers are unsigned and big-endian. The fixed domain separator is the UTF-8 byte sequence:

`MXSAP-SAGA-TRACE-V1\0`

The header is:

| Field | Encoding |
|---|---|
| Domain | Exact domain bytes above |
| Seed | u64 |
| Trace index | u64 |
| Scenario | u8, values 1 through 8 |
| Operation nonce | u64 |
| Command count | u32, 1 through 64 |

Each command is exactly 20 bytes:

| Field | Encoding |
|---|---|
| Opcode | u8 |
| Owner | u8 |
| Fault | u8 |
| Reserved | u8, must be zero |
| Subject | u64 |
| Value | u64 |

Opcode values are fixed: 1 CreateIntent, 2 Authorize, 3 PrepareLocal, 4 CommitLocal, 5 PrepareTreasury, 6 DispatchTreasury, 7 MarkIndeterminate, 8 ReconcileTreasury, 9 RetrySameEffect, 10 CommitTreasury, 11 CheckpointPayments, 12 CheckpointTreasury, 13 MutateIntent, 14 DuplicateDelivery, 15 MutateEvent, 16 StaleCheckpoint, 17 ForkCheckpoint, 18 Compensate.

Owner values are fixed: 1 Sender, 2 Payments, 3 Treasury, 4 Adversary.

Fault values are fixed: 0 None, 1 ResponseLostBeforeEffect, 2 ResponseLostAfterEffect, 3 LookupUncertain, 4 ReceiptMismatch, 5 StaleFrontier, 6 WrongOwner, 7 PayloadMutation, 8 DuplicateDelivery, 9 ForkedCheckpoint, 10 MissingDependency, 11 AuthoritativeAbsence.

The trace digest is SHA-256 over the full canonical byte string, rendered as lowercase `sha256:<64 hex>`.

## Deterministic generator

The generator uses SplitMix64 with modulo-(2^{64}) wrapping arithmetic.

Seed state:

`state = seed XOR (trace_index * 0xD6E8FEB86659FD93)`

Each draw performs, in order:

1. `state = state + 0x9E3779B97F4A7C15`
2. `z = (state XOR (state >> 30)) * 0xBF58476D1CE4E5B9`
3. `z = (z XOR (z >> 27)) * 0x94D049BB133111EB`
4. Return `z XOR (z >> 31)`

All multiplication and addition wrap modulo (2^{64}).

Exactly six draws are consumed, in this order: scenario, operation nonce, effect nonce, checkpoint nonce, amount, fee.

- Scenario = first draw modulo 8 plus 1.
- Amount = fifth draw modulo 1,000,000 plus 1.
- Fee = sixth draw modulo 1,001.
- All other draws become the corresponding nonce values.

The generator emits one of eight bounded templates: normal completion/checkpointing; pre-effect acknowledgement loss, destination-qualified authoritative absence and same-effect recovery; post-effect acknowledgement loss and receipt reconciliation; unresolved external outcome; immutable-intent mutation; duplicate delivery plus event mutation; stale/forked owner checkpoint; and separately identified compensation.

The trace grammar and draw order are versioned. Any change requires a new profile version and new known-answer vectors.

## Test discipline

- The shared fixture `fixtures/saga-trace-v1.kat` pins the exact canonical bytes for a primary vector plus independent SHA-256 known answers for all eight scenario templates.
- Rust and Go tests independently regenerate the primary byte vector and all eight scenario digests; the digest values were calculated independently with Python's fixed-width integer/struct encoding and hashlib.
- A 4,096-seed deterministic smoke corpus checks repeatability and scenario coverage.
- This initial tranche does not yet execute the commands against the saga models, check economic invariants per trace, minimize counterexamples, or run one million traces.
- No provider is called and no production zome or ledger state is mutated.
