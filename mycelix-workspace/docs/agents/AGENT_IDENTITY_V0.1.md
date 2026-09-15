# AGENT-002 — Stable Agent / Runtime-Instance Identity v0.1

Status: **executable identity theorem; explicitly non-authorizing**

Parent theorem: exact qualified AGENT/generic-authority convergence `6163a80c8398fac01fbd506515c11f60cee48320`, run `34881344813` attempt 1 PASS.

AGENT-002 consumes the exact successful parent convergence receipt by byte digest. Git ancestry alone is not treated as sufficient qualification evidence.

## The theorem

```text
existing generic stable PrincipalId
        +
exact bounded RuntimeInstanceId
        ->
QualifiedAgentRuntimeIdentityV1
```

The positive result establishes only structural validity plus the exact canonical semantic binding between those two identities.

It does not prove that the runtime exists, is currently running, is authentic, controls the principal's key material, runs a claimed model, has a particular software state, or has authority to perform any action.

## Deliberately absent concepts

AGENT-002 does not introduce:

- `AgentPrincipalId`;
- a one-field stable-agent principal wrapper;
- a mandatory `AgentControllerBinding`;
- a caller-asserted runtime epoch/generation;
- runtime attestation/model provenance;
- authority/currentness/delegation semantics;
- Mission/intent semantics; or
- action/effect permission.

Actual delegator/root-mandator provenance remains owned by the already-qualified generic grant/delegation stack. Runtime/model/software assurance belongs to AGENT-003. Current agent authority belongs to AGENT-006.

## RuntimeInstanceId

The runtime instance is a new opaque identifier domain with an invariant-preserving type boundary from day one.

It is at most 512 UTF-8 bytes and rejects empty values, leading/trailing whitespace, and control characters. The implementation rejects invalid input rather than silently trimming or normalizing it.

Accepted bytes are exact. There is no Unicode normalization or case folding. NFC and decomposed forms that render similarly remain different identifiers and produce different canonical digests.

## Canonical profile

The normative byte contract is:

```text
raw bytes: "mycelix/agent/runtime-instance/v1"
frame("mycelix-agent-runtime-instance-v1-blake3-framed-semantic")
frame("mycelix-agent-identity-v0.1")
frame(exact PrincipalId UTF-8 bytes)
frame(exact RuntimeInstanceId UTF-8 bytes)

frame(x) = u64_le(len(x)) || x
hash = BLAKE3-256
```

Independent vector:

```text
agent               = did:example:agent-alpha
runtime instance    = runtime:host-a:proc-7
digest              = 466beee862b306834aa76be027eb9c9f9629e6e550e1bdd5f033a7fec1398a9e
```

The qualification oracle independently reconstructs the full preimage and BLAKE3 digest in pure Python without invoking the production Rust implementation.

## Construction boundary

Transport input `AgentRuntimeInstanceClaimV1` is serializable/deserializable. `RuntimeInstanceId` deserialization calls the invariant-preserving constructor.

The positive `QualifiedAgentRuntimeIdentityV1` has private fields and implements neither `Serialize` nor `Deserialize`. Consumers cannot deserialize a stored green token into a positive identity theorem; they rerun qualification.

The inherited `PrincipalId` still permits tuple/deserialization bypass in this ancestry, so the qualifier deliberately reruns `PrincipalId::new` over the exact supplied bytes before producing a positive result.

## Qualification scope

The exact-head qualification must prove:

- exact parent `6163a80c8398fac01fbd506515c11f60cee48320`;
- exact parent convergence receipt bytes and digest;
- strict changed-path scope;
- exact two-package isolated Cargo census (`mycelix-institutional-core`, `mycelix-agent-identity`);
- frozen dependency lock;
- production Rust golden vector == independent pure-Python vector;
- malformed legacy PrincipalId bypass fails closed;
- malformed runtime IDs fail construction and deserialization;
- agent substitution and runtime substitution change identity;
- restart can change runtime identity without changing stable PrincipalId;
- Unicode normalization is not silently introduced;
- rustfmt on the new crate;
- tests and strict Clippy under Rust 1.98.1;
- immutable evidence checkout; and
- a postflight-only successful AGENT-002 receipt.

## Non-claims

```text
AGENT-002 PASS != proof of principal key possession
AGENT-002 PASS != AI classification
AGENT-002 PASS != runtime authenticity
AGENT-002 PASS != runtime attestation
AGENT-002 PASS != current agent authority
AGENT-002 PASS != effect authority
AGENT-002 PASS != full agent security
```
