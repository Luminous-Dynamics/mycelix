# Mycelix Governance Executor Lineage v0.1 — Normative Invariants

Status: **pure composition/qualification kernel; external effects remain disabled**

This crate closes one narrow compatibility gap left by the earlier executor-designation tranche: delegated executor authority must be proven by an exact current delegation lineage, not by a non-empty string.

## 1. Caller delegation strings are not authority

`VerifiedAuthorityGrant.delegation_verification_ref` from the older executor-designation API is compatibility metadata only in this layer.

For delegated grants this crate first reconstructs the exact current lineage from already-qualified `QualifiedDelegationEdge` values against the exact executor grant bytes.

Only after successful lineage reconstruction does it clone the grant receipt and overwrite the legacy string with a deterministic marker derived from the exact lineage digest/profile.

The caller-supplied string is never used to decide authority.

## 2. Direct and delegated grants are disjoint

A direct grant (`delegated_from == None`) must use `DelegationLineageEvidence::Direct`.

A delegated grant must provide `DelegationLineageEvidence::Delegated` with an exact root grant and complete current edge set.

Missing lineage for a delegated grant denies. Supplying lineage for a direct grant also denies.

## 3. Exact target grant bytes are re-qualified

The lineage is reconstructed with:

`qualify_complete_delegation_lineage(root_grant, executor_grant, edges, now)`

The target is therefore the exact `AuthorityGrant` object used by executor qualification, not merely a textual grant ID.

The grant is also canonicalized under:

`mycelix-authority-grant-v1-blake3-framed-semantic`

The final executor authority commits that exact grant digest/profile.

## 4. Delegation policy/revocation semantics are inherited, not reinvented

Every supplied edge is already non-deserializable qualified authority from PR #77 and therefore carries:

- exact canonical parent/child identity;
- exact current grant/delegation generations;
- exact current qualified delegation-policy authority;
- attenuation checks;
- explicit re-delegation permission; and
- bounded current lease.

This crate does not define another delegation policy model.

## 5. Legacy executor qualification is contained

After exact lineage reconstruction, the crate invokes the earlier `qualify_executor_designation` using a sanitized grant receipt.

For delegated grants the compatibility marker is deterministically derived from the qualified lineage. For direct grants the legacy delegation field is forced to `None`.

A forged caller string therefore cannot make the new final authority qualify.

## 6. Exact executor designation semantics are canonicalized

The new authority computes a stable semantic digest over the exact `ExecutorDesignation` fields, including:

- designation ID;
- executor principal;
- authority grant ID;
- proposal;
- exact actions digest/profile;
- threshold authorization ref;
- institution/jurisdiction/rulebook;
- required capability;
- exact source kind/reference/proof;
- issue/expiry times; and
- designation proof ref.

Dynamic verifier invocation metadata is excluded.

## 7. Final authority identity binds all critical semantic domains

`QualifiedLineageBoundExecutorAuthority.authority_digest` commits:

- exact proposal;
- exact executable action digest/profile;
- exact threshold-authorization ref;
- exact executor principal;
- exact canonical authority-grant digest/profile;
- exact executor-designation semantic digest/profile;
- exact executor-authority record ref;
- institution/jurisdiction/rulebook;
- exact capability scope;
- exact semantic validity horizon; and
- for delegated authority: exact root grant ID, lineage digest/profile and depth.

Changing any of those semantics changes authority identity.

## 8. Dynamic freshness remains outside stable identity

`verified_at_ms` and the reusable `lease_until_ms` are not hashed into stable executor authority.

For delegated authority the lease is bounded by the minimum of executor semantic validity and the current delegation-lineage lease.

Same-generation re-verification may therefore refresh dynamic evidence without minting a new semantic authority identity.

## 9. Qualified authority is not application data

`QualifiedLineageBoundExecutorAuthority` has private fields and intentionally does not implement `Deserialize`.

Application payloads cannot mint it.

## 10. This is not yet complete current executor freshness

This tranche closes delegated-lineage semantics. It does **not yet** replace all current-authority freshness requirements for direct grants, threshold authorizations, or executor designations.

A later current-executor verifier should compose PR #74 generation-bound freshness for those exact subjects before lifecycle authority is enabled.

## 11. Advisory systems remain outside authority

Phi, reputation, stake, consciousness/model scores, proposal authorship, committee membership, caller identity, and advisory recommendations cannot substitute for exact institutional grant + designation + threshold + lineage authority.

## 12. Runtime/effects remain fail-closed

No Holochain verifier or external-effect adapter is provisioned by this crate.

A future lifecycle verifier may consume this authority only after the remaining current-freshness/provider contracts are satisfied.
