# Mycelix AGENT-002 Identity Kernel v0.1 — Normative Invariants

Status: **structural/canonical identity only; non-authorizing**

The kernel proves exactly one relationship:

```text
existing stable PrincipalId
        +
exact bounded RuntimeInstanceId
        ->
canonical runtime-instance binding identity
```

It does not prove principal key possession, AI classification, runtime existence, runtime authenticity, model identity, attestation, currentness, authority, or effect permission.

## Stable principal reuse

The stable agent is the existing institutional `PrincipalId`. `AgentPrincipalId`, `HumanPrincipalId`, and one-field agent-principal wrappers are forbidden in this tranche.

Because the inherited v0.1 `PrincipalId` tuple field/deserializer can bypass `PrincipalId::new`, the AGENT-002 qualifier re-runs that constructor over the exact supplied bytes before qualification.

## Runtime instance identifier

`RuntimeInstanceId` is a genuinely new domain because a process/workload/runtime instance is not an authority-holder principal.

The type has a private field and invariant-preserving custom deserialization. It rejects:

- empty identifiers;
- identifiers over 512 UTF-8 bytes;
- leading/trailing whitespace rather than trimming it; and
- Unicode control characters.

Accepted bytes are preserved exactly. There is no case folding or Unicode normalization. Visually equivalent NFC/decomposed Unicode spellings are distinct identities unless a future profile explicitly defines normalization.

A restart is represented by a different runtime-instance ID. An unauthenticated numeric epoch/generation is not part of AGENT-002.

## Canonical digest

Profile: `mycelix-agent-runtime-instance-v1-blake3-framed-semantic`

Domain separator: raw bytes `mycelix/agent/runtime-instance/v1`

After the raw domain separator, every field is framed as:

```text
u64_le(byte_length) || exact_bytes
```

Field order is exactly:

1. profile string;
2. protocol version `mycelix-agent-identity-v0.1`;
3. stable `PrincipalId` exact bytes;
4. `RuntimeInstanceId` exact bytes.

Hash: BLAKE3-256.

Changing the domain, field set/order, framing, normalization policy, or semantic inclusion rules requires a new profile/domain.

## Positive result boundary

`QualifiedAgentRuntimeIdentityV1` has private fields and implements neither `Serialize` nor `Deserialize`.

A serialized claim/digest is evidence material, not a positive capability. Consumers reconstruct the positive result only by rerunning the qualifier.

## Explicit non-claims

```text
PrincipalId != proof of possession
PrincipalId != AI classification
runtime instance identity != runtime authenticity
runtime identity != runtime attestation
runtime identity != authority
AGENT-002 PASS != current agent authority
AGENT-002 PASS != effect authority
AGENT-002 PASS != full agent security
```
