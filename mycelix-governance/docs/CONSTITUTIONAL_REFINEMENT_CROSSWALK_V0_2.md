# MYC-CONST-003CR2 — Constitutional Refinement Crosswalk v0.2

Status: **draft successor evidence contract**. This tranche does not modify or reinterpret the frozen CR1 semantic subject.

Parent CR1 semantic subject: `4edb56bd3c36ddc6277e2de5e0584a65e8c99b3c`.

Parent CR1 qualified verifier/run/artifact:

- verifier `651eb0c05fa70012b7eebc4e1497cb7cacf7c641`;
- run `35319268716`;
- artifact `sha256:b8bacbf0467e23fe033042691acdeee732f532c5e6774715d26dadaecad415b1`.

## Why v0.2 is needed

CR1 correctly froze a closed refinement vocabulary and a qualified lifecycle-status projection slice. It also deliberately left B4 ClaimBinding, C3 quiescence and runtime crash consistency pending.

Those subjects later evolved on separate semantic branches. In particular, the frozen CR1 tree does not contain the later B4 `constitutional-consumption/src/binding.rs` source.

Therefore a successor refinement validator must not assume:

```text
current working tree == every qualified semantic subject
```

CR2 instead binds every relationship side to:

```text
exact semantic commit
+ exact repository path
+ exact Git blob SHA
```

and reconstructs the bytes using Git object lookup.

## Evidence qualification is not refinement qualification

CR2 makes a distinction that was implicit in CR1:

```text
qualified formal/reference subject
            !=
qualified concrete ↔ formal refinement mapping
```

A subject can have a valid exact-head qualification receipt while its correspondence to another implementation layer remains unknown.

The v0.2 manifest therefore assigns each qualified subject an independent `mapping_status`.

### Inherited qualified mapping

`MYC-CONST-003B2` retains the seven lifecycle status relationships already qualified by CR1:

- `lifecycle.absent`;
- `lifecycle.pending`;
- `lifecycle.blocked`;
- `lifecycle.finalized`;
- `lifecycle.rejected`;
- `lifecycle.revoked_closed`;
- `lifecycle.halted`.

CR2 does not copy those rows. It binds the exact parent CR1 manifest blob and requires the inherited relationship census to match it exactly.

### Candidate B4 mapping

B4 ClaimBinding is now a genuinely qualified semantic subject:

- semantic `037f61c15ff367a1518d98f7acc8ec0fa962c2f6`;
- verifier `508e81ea93bd49692f54b234e20146dfe101afa3`;
- run `35344734164`;
- artifact `sha256:d0cf0a406f22cae5d8b7ff9a66a9c3a0610c073e1c84f1cd38d1255367b2c6f2`.

CR2 adds one candidate projection relationship between Rust `ClaimBinding` and Alloy `Binding`.

The Rust struct contains:

```text
schema_version       metadata/profile field
claim_id             semantic
 envelope_digest      semantic
nonce                semantic
use_index            semantic
jurisdiction         semantic
matter               semantic
target_digest         semantic
payload_digest        semantic
budget_id             semantic
```

The Alloy `Binding` signature contains the same nine semantic dimensions but intentionally does not model the Rust serialization/profile `schema_version` field.

The validator parses both exact Git objects and requires the following complete one-to-one semantic field correspondence:

```text
claim_id         -> claimId
envelope_digest  -> envelope
nonce            -> nonce
use_index        -> useIndex
jurisdiction     -> jurisdiction
matter           -> matter
target_digest    -> target
payload_digest   -> payload
budget_id        -> budget
```

A renamed, omitted, duplicated or newly introduced semantic field therefore requires an explicit crosswalk decision.

The relationship remains `candidate` in the semantic manifest. It becomes evidence-backed refinement only if the CR2 exact-head qualifier itself passes. The manifest is not allowed to self-label its own new mapping `qualified`.

## Qualified but unmapped subjects

### C3 quiescence

C3 is qualified at:

- semantic `b9bb91353788aeb858c4e52422a87e2401d60a0e`;
- verifier `18309d680a5d630bbb7f5443ab103ade9239a557`;
- run `35314384788`;
- artifact `sha256:d26e566503fc1223d879d69ecdb4cf6978fc62664576ed04973067393c8feaab`.

That proves the bounded quiescence/stall model within its stated assumptions. It does **not** prove that a concrete Rust or Holochain implementation computes `ResolutionObligation`, `ProtocolStall`, `AwaitingExternalEvidence`, or the other classification predicates correctly.

CR2 therefore records C3 as `qualified` evidence with `mapping_status: unmapped`.

### D1A effect outbox

D1A is qualified at:

- semantic `15b9c89adf0ac3c6c5a73681614d6bfcd368820a`;
- verifier `70fbe906834fdef1ba69a80066dbbbaef200156f`;
- run `35320314578`;
- artifact `sha256:ce9d7176bd1040ef552fa587e44ae8801a13b0363c0013d5428e5f95ff60430c`.

This qualifies the abstract crash-consistent effect-outbox model. The audited production execution coordinator was previously observed to dispatch effects before durable execution evidence and therefore is not silently promoted into a refinement relationship.

D1A is also recorded as `qualified` evidence with `mapping_status: unmapped`.

## Pending subjects

The v0.2 manifest keeps the following separate from qualified evidence:

- D1C effect ledger — run `35349600389` queued;
- D1D-E0 durable event contract — run `35357940683` queued;
- D1D-E1A event admission — run `35360097507` queued.

A queued or prepared verifier head is provenance, not qualification evidence.

## Validator theorem

`validate_refinement_crosswalk_v2.py` requires:

1. exact parent CR1 semantic/verifier/run/artifact identity;
2. exact parent manifest Git blob re-derivation;
3. exact inherited CR1 relationship census;
4. exact qualified-subject receipt identities for B2/B4/C3/D1A;
5. no relationship rows for subjects marked `unmapped`;
6. exactly one new candidate B4 relationship;
7. exact concrete/formal semantic heads and Git blob identities;
8. independent Rust `ClaimBinding` field parsing;
9. independent Alloy `Binding` field parsing;
10. complete duplicate-free nine-field semantic correspondence;
11. `schema_version` remains metadata rather than being invented in Alloy;
12. pending D1C/E0/E1A identities cannot be promoted without a new receipt;
13. required non-claims remain present.

Its self-test mutates those boundaries and requires every mutation to fail.

## Next refinement work

CR2 deliberately exposes two real obligations instead of hiding them:

### C3 concrete classification refinement

A later tranche must identify or implement a concrete readiness/quiescence classifier and map its actual predicates and transition-enablement semantics to the C3 TLA+ model.

### D1A runtime refinement

A later tranche must map the crash-consistent outbox state machine to concrete durable runtime commit points. D1C can become part of that proof only after D1C itself earns hosted qualification.

## Non-claims

CR2 does not claim whole-program verification, C3 concrete refinement, D1A runtime refinement, D1C qualification, event persistence qualification, Holochain persistence, deployment currentness, or qualification of CR2 itself until its exact-head verifier passes.
