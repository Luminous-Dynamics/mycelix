# STEW-000A — Restricted Traditional-Knowledge Fail-Closed Repair Preregistration v0.1

Status: preregistration / security repair boundary

Parent: STEW-000

## Purpose

Preregister the narrow repair for `mycelix-commons/zomes/water-wisdom` so the current public Holochain entry path cannot continue accepting plaintext that is merely labelled `CommunityOnly`, `ElderApproved`, or `Sacred`.

This repair deliberately does not attempt to design the final protected-content system. Its job is to stop the current semantic mismatch from creating additional exposure while preserving the existing public-use path.

## Audited current behavior

The current `TraditionalPractice` model defines:

```text
Public
CommunityOnly
ElderApproved
Sacred
```

and stores a full `description` in the same `TraditionalPractice` application entry.

The integrity zome currently accepts every access-level variant. Its test corpus explicitly contains assertions equivalent to:

```text
practice_all_access_levels_accepted
practice_sacred_access_level_accepted
practice_all_access_levels_valid
```

The coordinator currently creates the entry before checking any confidentiality mechanism, links every practice to general/type/recorder indexes, and exposes broad practice queries. Only the extra `public_practices` index is conditioned on `AccessLevel::Public`.

Therefore the enum currently expresses intent/metadata, not a confidentiality theorem.

## Security theorem

Until a separately qualified protected-content architecture exists:

```text
current public TraditionalPractice entry path
+ AccessLevel::Public
-> admissible subject to existing validation
```

but:

```text
current public TraditionalPractice entry path
+ (CommunityOnly | ElderApproved | Sacred)
-> INVALID
```

This MUST be enforced in the integrity zome.

```text
coordinator rejection alone
!= security boundary
```

A custom or modified coordinator must not be able to publish restricted-labelled plaintext through the existing public entry definition.

## Update theorem

The repair must cover update as well as create.

```text
create Public -> update Sacred plaintext
```

must not bypass the admission firewall.

The target integrity behavior is therefore:

```text
StoreEntry(Create TraditionalPractice)
    -> author binding
    -> public-entry access firewall
    -> payload validation

StoreEntry(Update TraditionalPractice)
    -> updater / recorder binding as appropriate
    -> public-entry access firewall
    -> payload validation needed by this boundary
```

The existing `RegisterUpdate` original-author check remains relevant but is not a substitute for validating the updated entry payload.

## Suggested implementation shape

Prefer one reusable pure predicate so create and update cannot drift:

```rust
fn validate_public_practice_access(access_level: &AccessLevel) -> ValidateCallbackResult {
    match access_level {
        AccessLevel::Public => ValidateCallbackResult::Valid,
        AccessLevel::CommunityOnly
        | AccessLevel::ElderApproved
        | AccessLevel::Sacred => ValidateCallbackResult::Invalid(
            "Restricted traditional knowledge cannot be published as plaintext through the public TraditionalPractice entry path".into(),
        ),
    }
}
```

Exact implementation syntax may vary, but create and update MUST share the same semantic predicate.

## Coordinator defense in depth

The official coordinator should also reject restricted plaintext before `create_entry` / `update_entry` to provide an immediate user-facing error rather than relying on lower-level validation failure.

This is defense in depth only.

```text
coordinator check
!= integrity admission
```

## Official query hardening

Legacy records may already exist with restricted labels. Official read surfaces should stop presenting those records as ordinary public catalog results.

At minimum, the repair should review and harden:

```text
get_practices_by_type
get_all_practices
get_public_practices
```

Desired application behavior:

```text
ordinary public discovery API
=> returns only records whose decoded TraditionalPractice.access_level == Public
```

`get_all_practices` should either:

1. become a compatibility alias for the public projection; or
2. be deprecated/replaced by an explicitly named administrative/migration-only surface that still cannot create a confidentiality claim.

No public API should imply that `CommunityOnly`, `ElderApproved`, or `Sacred` content is safe to display merely because a caller promises to filter it later.

## Legacy-record theorem

Query filtering is NOT retroactive secrecy.

```text
legacy restricted-labelled record
+ removed UI/index exposure
!= erased DHT plaintext
!= confidentiality restored
```

The repair may reduce accidental exposure through official applications. It cannot establish that already-published bytes are unavailable to peers that previously received them or to alternative clients capable of resolving their hashes/history.

Any migration report must preserve this distinction.

## No silent relabeling

The repair must not automatically rewrite restricted-labelled legacy content to `Public` merely to make validation pass.

```text
historical restricted label
-> public relabel without steward authority
```

would destroy evidence of the original intended restriction.

Legacy material should instead remain identifiable for later audit/migration.

## No plaintext migration yet

This repair does not invent a temporary private storage system.

For restricted new submissions the safe result is:

```text
reject
```

rather than:

```text
accept somewhere else with an unqualified confidentiality story
```

The later STEW-020 protected-content program will own encryption/key-distribution/storage/currentness semantics.

## Tests to reverse

Existing tests that assert acceptance of all access levels must be replaced with an explicit closed-world corpus.

Required create vectors:

```text
Public         -> PASS
CommunityOnly  -> REJECT
ElderApproved  -> REJECT
Sacred         -> REJECT
```

The prior minimal fixture using `Sacred` should be changed to `Public` so unrelated minimal-payload validation remains meaningful.

Required update vector:

```text
valid public entry
-> update payload access_level = Sacred
-> REJECT
```

If update validation is factored through a pure helper, test both the helper's closed-world mapping and the actual update path.

## Source invariants

A dedicated source/invariant gate should fail if future code reintroduces the old behavior.

Suggested predicates:

```text
1. restricted variants appear in an explicit rejection arm;
2. create path invokes the restricted-access predicate;
3. update path invokes the same restricted-access predicate;
4. no test named/structured as "all access levels accepted" remains;
5. public discovery surfaces do not intentionally return restricted-labelled records;
6. no comment claims an access label itself provides confidentiality;
```

## Qualification profile

The eventual implementation PR should qualify at least:

```text
cargo fmt --check
cargo check -p water_wisdom_integrity -p water_wisdom
cargo test -p water_wisdom_integrity -p water_wisdom
cargo clippy -p water_wisdom_integrity -p water_wisdom --all-targets -- -D warnings
```

plus any repository-standard exact-head / wasm32 qualification already required for Commons zomes.

Qualification MUST bind the exact proposed head. An ancestor run or source-only inspection is not implementation PASS.

## Migration/audit follow-up

After the admission firewall is implemented, a separate read-only audit should answer:

```text
How many historical TraditionalPractice records are labelled:
- CommunityOnly?
- ElderApproved?
- Sacred?

Which action hashes / authors / timestamps are involved?
Which official indexes currently reference them?
```

That audit must avoid republishing protected descriptions in logs or evidence artifacts. Counts and identifiers should be preferred over content.

No automated disclosure, deletion, relabeling, or redistribution should follow from the audit alone.

## Interoperability direction

The future stewardship policy should not mint community cultural authority from Mycelix alone.

Where communities already use external governance/protocol systems, Mycelix should support references to those externally controlled identifiers and policies.

In particular, Local Contexts Traditional Knowledge / Biocultural Labels are designed to let Indigenous communities express provenance, protocols, and permissions through community-controlled processes. Mycelix should interoperate by recording externally issued identifiers/metadata where authorized, not by fabricating or impersonating a community's Local Contexts labels.

## Deliberate non-claims

STEW-000A does not establish:

- confidentiality for any previously published record;
- a protected-content transport;
- encryption or key custody;
- community membership verification;
- elder/cultural-authority verification;
- legal ownership or copyright;
- Indigenous/community consent;
- successful deletion from the DHT;
- erasure from peers/backups;
- a right to inspect restricted legacy content;
- a final Local Contexts integration;
- AI training/use permission.

It establishes one narrow safety goal only:

```text
while TraditionalPractice is a public plaintext entry path,
restricted-labelled new plaintext must fail closed.
```
