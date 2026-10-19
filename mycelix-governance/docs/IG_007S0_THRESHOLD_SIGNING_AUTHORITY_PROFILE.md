# IG-007S0 — Observed threshold-signing authority profile

## Scope

IG-007S0 freezes the source-visible threshold-signing authority that proposal and execution paths currently depend on.

It covers P0s #959 and #960 without repairing production.

## Source subject

The profile binds the production tree represented by:

`fca2c107a1ea5108823ce617ba4111b6f7f77230`

Current main `31ede2365b81365bb119cd9351b2739119974130` has the same Git tree for these sources.

## Bound files

```text
threshold-signing coordinator
3449df8b03a4dd1774a5f22756d06931c72855b2

threshold-signing integrity
3fec8344635600c044a494fa72bbbfe408fbe5ec

proposals coordinator consumer
eb8358353ee259ef9c3b46617a61d3439f1c714c

execution coordinator consumer
3dbb8a8f69b377e494ccf24164c94bd80f54e0ef
```

## Profile identity

```text
id        mycelix-threshold-signing-observed-fca2c107-v1
revision  1
authority ObservedSourceBound
SHA-256   c15dfd860b759747938af2a13129d729fa0af1e75284418c9ea6b9c172f643ac
```

## Producer API observation

The exact threshold-signing coordinator source directory contains one `lib.rs` file.

Observed extern surface in that file:

`create_committee`

Consumers elsewhere expect:

```text
get_proposal_signature
get_committee
```

Those query externs are not observed in the exact coordinator module.

This is a source API-contract observation. It does not assert a particular Holochain runtime error variant.

## ThresholdSignature integrity theorem actually observed

Create validation calls:

`check_signature_validity`

Observed checks include:

- signer count is positive;
- signer count matches signers-list length;
- signed-content hash is non-empty;
- signature byte fields meet algorithm-specific presence/length requirements.

Not source-visibly reconstructed during signature creation:

```text
entry-author authority
cryptographic signature verification
committee existence
committee active state / epoch
committee threshold
qualified signer membership
committee scope
exact signed-subject governance authorization
verified-field derivation
```

ThresholdSignature updates are rejected, so the record is immutable after creation. Immutability does not itself make the original authority claim valid.

## Structural test observation

The integrity test helper builds an ECDSA ThresholdSignature with:

```text
signature            64 zero bytes
signed_content_hash  32 nonzero fixture bytes
signer_count         1
signers              [1]
verified             false
```

The minimum-length ECDSA test expects this fixture to pass `check_signature_validity`.

IG-007S0 therefore classifies this as:

`StructuralValidatorTest`

It does not call the fixture a cryptographically valid threshold signature.

## ProposalToSignature link

Integrity accepts creation of the proposal-to-signature link without source-visible reconstruction that the linked signature cryptographically/semantically authorizes that exact proposal subject.

A link is therefore an association observation, not an authorization theorem.

## Why this matters for #904

Execution currently wants to use threshold-signature state as an authority input.

A correct execution authorization receipt cannot safely be defined as:

```text
signature exists + verified bool
```

unless the signature subsystem first establishes what those facts mean.

The successor should provide a content-bound `VerifiedThresholdAuthorization` derived from committee, epoch, scope, signer, threshold, cryptographic and subject evidence.

## Relationship to #959

Adding `get_proposal_signature` before #960 is fixed would expose weakly qualified records through a nicer API.

The correct dependency is:

```text
strong signature authority theorem
-> typed query surface
-> execution authorization consumption
```

## Non-claims

No live forged signature, cryptographic break, deployment exploit, or governance-safety claim.