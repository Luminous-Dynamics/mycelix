# IG-007S1 — Threshold-signing authority counterexamples

Issue: #965

Parent: IG-007S0 / draft #964

## Purpose

IG-007S1 freezes deterministic source-contract counterexamples for the threshold-signing authority gaps tracked by #959 and #960.

It does not submit a live signature, perform a cryptographic attack, or mutate governance state.

## Bound profile

```text
profile id     mycelix-threshold-signing-observed-fca2c107-v1
profile SHA    c15dfd860b759747938af2a13129d729fa0af1e75284418c9ea6b9c172f643ac
authority      ObservedSourceBound
production     fca2c107a1ea5108823ce617ba4111b6f7f77230
```

## Corpus identity

```text
schema         mycelix-threshold-signing-counterexamples-v1
authority      MeasurementOnly
SHA-256        0f6532ae8e2c2e421da625592dbb3b38aa2b90c5342f46f3a305bdbec89b0269
counterexamples 5
issues         #959, #960
```

The SHA-256 is deterministic corpus identity only.

## CE-SIG-01 — structural zero-byte signature fixture

The fixture reproduces the exact source-test shape recorded by S0:

```text
algorithm            Ecdsa
signature            64 zero bytes
signed-content hash  32-byte nonzero fixture
signer count         1
signers              [1]
verified             false
```

The observed pure validator result is:

`AcceptedByCheckSignatureValidity`.

The evidence class remains:

`StructuralValidatorTest`.

The fixture explicitly records:

`cryptographic_validity = NotEstablished`.

Structural byte shape is not cryptographic authorization.

## CE-SIG-02 — stored `verified` flag is not a theorem

The source-bound profile records:

`verified_field_recomputed = NoneObserved`.

S1 therefore compares otherwise identical modeled records with `verified=false` and `verified=true` and freezes:

`VerifiedFlagDoesNotAffectObservedStructuralValidity`.

This does not say the two records are equally authoritative. It demonstrates that the observed pure structural validator does not derive cryptographic truth from that stored flag.

## CE-SIG-03 — committee authority is not reconstructed

S1 uses a deliberately contrasting context:

```text
modeled committee threshold = 3
fixture signer count        = 1
```

and preserves the exact S0 observations:

```text
committee lookup                    NoneObserved
active/epoch check                  NoneObserved
committee threshold check           NoneObserved
qualified signer membership check   NoneObserved
committee scope check               NoneObserved
```

Frozen result:

`NoObservedCommitteeAuthorityPredicateInSignatureCreateValidation`.

This is not a live threshold-bypass claim.

## CE-SIG-04 — association is not authorization

For `ProposalToSignature`, S0 records:

```text
create validation                         UnconditionalValidObserved
exact subject authorization reconstruction NoneObserved
```

Frozen result:

`LinkDoesNotEstablishObservedProposalSignatureAuthorization`.

A graph/link association therefore cannot be promoted into an exact signature-subject theorem by the research model.

## CE-SIG-05 — producer/consumer API mismatch

Consumers expect:

```text
get_proposal_signature
get_committee
```

The frozen producer coordinator surface observes only:

```text
create_committee
```

and records both expected query externs as `NoneObserved`.

Frozen result:

`ExpectedSignatureQueryContractAbsentFromObservedProducer`.

The fixture does not assert a particular Holochain runtime error variant.

## Qualification

The exact-head workflow:

1. checks out the exact S1 product head;
2. binds the four S0 source blobs;
3. syntax-compiles S0 and S1 independent scripts;
4. revalidates S0 twice byte-identically;
5. runs the S1 self-test twice byte-identically;
6. emits the S1 corpus twice byte-identically;
7. asserts the exact profile/corpus commitments and CE-SIG-01..05 results;
8. verifies checkout immutability.

A hosted PASS would establish deterministic source-contract measurement only.

## Successor boundary

A corrected threshold-signing subsystem should move authority to a content-bound object such as `VerifiedThresholdAuthorization` whose positive state is derived from exact committee generation, scope, signer/threshold policy, signed subject, algorithm/PQ policy, and cryptographic evidence.

Historical S0/S1 evidence remains attached to the frozen old source subject rather than being rewritten.

## Non-claims

IG-007S1 establishes no forged-signature acceptance, cryptographic break, live threshold bypass, deployment exploit, deployment currentness, or governance safety result.
