# Mycelix Governance Constitution v0.1

This crate defines the portable constitutional trust-root and lineage contract for binding Mycelix governance.

## Core rule

**Constitutional genesis is defined by the network, not by the first writer.**

A Holochain governance DNA should place a `ConstitutionGenesisManifest` in DNA properties. DNA properties are integrity modifiers, so changing the manifest changes the DNA hash and creates a different governance network.

Any participant may publish the exact matching genesis statement for discovery. Publishing it grants no special authority. A different genesis statement is invalid for that network.

## State model

A constitutional statement commits:

- network identity;
- institution identity;
- constitution identity and sequential version;
- exact parent statement digest for non-genesis versions;
- exact rulebook id/version/digest/profile;
- exact charter digest/profile;
- exact governance-parameter digest/profile;
- exact amendment-policy digest/profile;
- the binding-vote protocol/profile authorized to ratify the next transition;
- the threshold/institutional authorization profile required after the vote;
- effective time.

Authorization evidence is deliberately outside the statement digest so the child state can be committed by an authorization without creating a circular hash.

## Transition model

Every transition binds:

`exact parent state`
→ `exact child state`
→ `parent amendment policy`
→ `proposal`
→ `binding tally digest/profile`
→ `threshold authorization digest/profile`
→ `amendment payload digest/profile`
→ `authorization time`
→ `non-zero replay nonce`

The parent determines which binding-vote and threshold-authority profiles are valid for changing it. A child may change those rules only after the transition governed by the old rules succeeds.

## Fork handling

`project_verified_lineage()` never chooses between competing constitutional children by timestamp, DHT arrival order, author, reputation, Phi, stake, or model output.

If two different verified children exist for the same current parent, projection fails with `AmbiguousConstitutionalFork`. Resolution must happen through an explicit higher-level recovery/constitutional process rather than an implementation-specific tie-break.

## Verification boundary

This crate performs structural/canonical binding only. It does not verify Holochain records, signatures, threshold proofs, binding tallies, credential chains, or Xenia receipts.

`TransitionVerificationEvidence` must be produced only after the host verifies the exact external artifacts. A deserialized value is not authority merely because its fields are populated.

## Intended Holochain adapter

The governance DNA adapter should:

1. deserialize `ConstitutionGenesisManifest` from immutable DNA properties;
2. reject binding governance when the property is absent or malformed;
3. validate a persisted genesis statement against the exact DNA manifest;
4. store later statements/transitions append-only;
5. resolve binding tally and threshold authorization through fail-closed verifier boundaries;
6. expose only a verified-current-constitution endpoint to binding governance consumers;
7. treat the legacy mutable `constitution` zome as non-authoritative until migrated.

## Non-goals

- no privileged founder/bootstrap key;
- no first-writer-wins genesis;
- no timestamp-based fork selection;
- no Phi/reputation/stake/model score as constitutional authority;
- no implicit trust in proof-reference strings;
- no dependency on Holochain or Symthaea in the portable kernel.
