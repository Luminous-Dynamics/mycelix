# MCRB Pre-Dispatch Operation Contracts v0.1

**Status:** Proposed contract registry; not signed, reviewed or benchmark-qualified  
**Tracking:** [#6392 — MCRB-OP-PROPOSITION-001](https://github.com/Luminous-Dynamics/mycelix/issues/6392)  
**Parent:** [#6322 — Platform Stewardship and MCRB](https://github.com/Luminous-Dynamics/mycelix/issues/6322)

## 1. Purpose

MCRB must freeze the exact proposition for each operation **before dispatch**.

The benchmark must not infer success from a caller's outcome label, a generic API acknowledgement or a post-hoc description of what the operation was intended to do. It must first know:

1. which scenario and operation class are being exercised;
2. what exact proposition makes that operation a qualified result;
3. what attempt and effect states are possible;
4. what evidence is sufficient for that proposition;
5. which source roles and independence requirements apply;
6. whether cross-host observation, complete action coverage and idempotent reconciliation are mandatory.

A versioned operation-contract registry provides that frozen definition. It is not itself authority to execute an operation and it does not prove that the outcome happened.

## 2. Artifacts

The current candidate artifacts are:

- [Scenario manifest](MCRB_V0_1_SCENARIO_MANIFEST.json) — freezes the benchmark and declares the operation-contract registry as a required dependency.
- [Operation-contract registry](MCRB_OPERATION_CONTRACTS_V0_1.json) — four illustrative profiles for local append/read, remote DHT publication, capability revocation and external effects.
- [Registry JSON Schema](MCRB_OPERATION_CONTRACT_REGISTRY_V0_1.schema.json).
- [Offer-ledger JSON Schema](MCRB_OFFER_LEDGER_V0_1.schema.json).
- [Outcome-attestation schema](MCRB_OUTCOME_ATTESTATION_V0_1.schema.json).
- [Outcome-ledger schema](MCRB_OUTCOME_LEDGER_V0_1.schema.json).

The registry's `source_scenario_manifest_git_blob_sha` must equal the exact Git blob SHA of the frozen scenario manifest. The registry artifact digest is SHA-256 over the exact committed registry bytes and must be supplied to the verifier from protected run configuration. The operation-contract digest is a separate, domain-separated digest of one typed contract. Do not substitute one digest for another.

## 3. Separate the operation proposition from the outcome class

Every contract declares an expected disposition and expected QOPS outcome class.

| Expected disposition | Expected QOPS outcome | Effect proposition |
|---|---|---|
| `must-apply` | `QualifiedSuccess` | Exact expected effect is applied |
| `must-observe` | `QualifiedSuccess` | Exact record/effect is observed at the required boundary |
| `must-deny` | `CorrectDenial` | Protected effect is confirmed not applied after the required policy barrier |
| `must-reconcile` | `Unknown` | The operation remains unresolved until a reconciliation record closes it |
| `diagnostic-only` | `Unknown` | Diagnostic data cannot contribute to the QOPS numerator |

The following terminal outcome classes remain distinct:

- `QualifiedSuccess`
- `CorrectDenial`
- `UnexpectedDenial`
- `Failed`
- `Unknown`
- `DuplicateEffect`

**Correct denial is not a business success.** A revocation test may correctly pass its security proposition when the protected effect is confirmed not applied. That outcome is classified as `CorrectDenial`; it is not counted in the QOPS qualified-business-success numerator.

A timeout, local acceptance or provider acknowledgement cannot by itself establish the effect proposition.

## 4. Attempt state and effect state are independent axes

### Attempt state

- `not-dispatched`
- `dispatched`
- `accepted-at-local-boundary`
- `timed-out`
- `reconciled`
- `unsupported`

### Effect state

- `applied`
- `confirmed-not-applied`
- `partial`
- `unknown`
- `not-attempted`

Examples:

- `dispatched + unknown`: the request may have taken effect; reconcile before retry.
- `accepted-at-local-boundary + unknown`: local API acceptance is not a remote observation.
- `timed-out + unknown`: timeout is not proof of failure or non-application.
- `reconciled + confirmed-not-applied`: may establish a correct denial or confirmed non-effect, depending on the pre-dispatch contract.
- `reconciled + partial`: blocks QOPS unless the contract contains an explicit expected-action set and a separate qualified partial-effect proposition.

Unknown and partial states always block QOPS and require reconciliation. Prior attestations remain append-only; reconciliation adds a new record rather than rewriting history.

## 5. Evidence policy

Each operation contract declares:

- allowed and required evidence kinds;
- allowed source roles;
- whether an independent observer is required;
- the minimum distinct observer-source count;
- whether cross-host visibility is required;
- whether complete evidence coverage is required;
- whether idempotency is required;
- what counts as expected versus unexpected denial.

Evidence kinds include application records, provider acknowledgements, provider status queries, independent observations, policy decisions, failure traces, reconciliation and cross-peer reads.

A source's role, identity and dependency closure are established by the separately verified source trust policy. A caller cannot make two sources independent by assigning them different names. A provider acknowledgement remains provider evidence; it is not independent evidence of the external effect.

For remote DHT publication, a local zome-call response cannot satisfy the proposition that an independent remote peer observed the exact record digest and author.

## 6. Offer-ledger binding

Every offer entry is frozen before dispatch and binds:

- operation ID and offer sequence;
- request digest;
- idempotency-key SHA-256 commitment, if present;
- operation-contract ID and version;
- domain-separated operation-contract digest;
- scenario ID and operation class;
- run/generator/workload/parameter/clock context.

The offer envelope binds the exact operation-contract registry SHA-256 digest. The canonical offer-ledger digest commits both the registry digest and every per-operation contract reference.

Raw idempotency keys are not serialized into the public offer ledger. The execution component keeps the raw retry key; the benchmark artifact carries only its digest commitment.

The Rust implementation returns an opaque `ValidatedOperationContractRegistry` after structural validation against protected registry and scenario-manifest digests. `verify_offer_ledger` now requires that capability; there is no contract-free verifier path that can mint a verified denominator.

## 7. Outcome-attestation binding

Each outcome attestation and outcome-ledger entry must bind the same:

- operation-contract registry digest;
- operation-contract ID, version and digest;
- operation ID and sequence;
- request digest and idempotency-key digest;
- run, scenario, source, runtime and topology context;
- attempt state, effect state and outcome class;
- evidence references and trust-policy context.

The outcome verifier derives the allowed outcome and required evidence from the frozen contract. It must not accept a contract substitution or a caller-supplied post-hoc success proposition.

## 8. Content-binding algorithm

The current Rust candidate computes a contract digest from the typed contract projection using the domain separator:

```text
MYCELIX-MCRB-OPERATION-CONTRACT\0v0.1\0
```

and the exact serialized typed contract bytes:

```text
SHA256(domain_separator || serde_json::to_vec(typed_contract))
```

The registry digest is:

```text
SHA256(exact committed registry artifact bytes)
```

The scenario manifest Git blob SHA is a Git object identity, not a SHA-256 content digest. Keep the identities in separate fields.

## 9. Required negative controls

- Weaken a scenario-required cross-host visibility policy.
- Change a contract's expected QOPS outcome after observing results.
- Relabel a provider ACK as an independent observation.
- Use a local zome-call response for a remote DHT observation proposition.
- Use a correct-denial outcome for an operation that was expected to apply.
- Mark an operation not-dispatched when dispatch evidence exists.
- Mark a timeout as confirmed-not-applied without reconciliation.
- Reuse an operation ID under a different request or contract digest.
- Omit an idempotency-key digest for a contract that requires it.
- Reuse an idempotency-key digest within the prohibited uniqueness scope.
- Provide partial evidence for a complete-coverage proposition.
- Use source roles or evidence kinds that the contract does not allow.
- Mutate the registry or one contract while recomputing only the offer-ledger digest.
- Remove the registry digest from the run receipt or outcome ledger.

## 10. Implementation and claim ceiling

The current MCRB metric-integrity source now:

- parses and validates the registry against its exact scenario-manifest Git blob;
- computes registry and per-contract digests;
- validates disposition/outcome consistency and fail-closed Unknown/partial policies;
- requires the validated registry when verifying an offer ledger;
- binds contract identity and digest into each offer;
- retains expected outcome class, request digest and idempotency-key digest in the opaque verified-offer capability;
- propagates the registry digest into QOPS reports and outcome/receipt schemas.

The separately signed and independently verified outcome ledger tracked by #6357 is not yet implemented. Caller-provided outcome labels remain diagnostic only. No Cargo test, Clippy, CI pass, live conductor, independent effect observation or eligible QOPS rate is claimed by this specification/source change.

The registry remains `proposed-not-executed`. A protected policy decision and exact-head qualification are required before any operation contract is used to support a benchmark-performance claim.


## 11. Idempotency-key privacy

The offer ledger carries an SHA-256 commitment, not the raw idempotency key. Producers use the Rust helper `idempotency_key_commitment(raw_key)` with:

```text
SHA256(
  "MYCELIX-MCRB-IDEMPOTENCY-KEY\\0v0.1\\0"
  || u64_be(raw_key.len())
  || raw_key
)
```

The raw key remains with the operation execution adapter. It is not included in offer-ledger JSON, outcome attestations, public traces or receipts. The commitment is for binding and deduplication; it does not encrypt the key and should not be used for low-entropy secrets beyond the explicitly scoped idempotency token.
