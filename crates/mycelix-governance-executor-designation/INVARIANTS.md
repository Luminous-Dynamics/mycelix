# Governance Executor Designation v0.1

Status: **normative pure semantic kernel**

This crate reuses `mycelix-institutional-core::AuthorityGrant` as the underlying permission model. It does not create a second generic capability system.

## Why a scoped designation is necessary

`AuthorityGrant` intentionally binds holder, institution, jurisdiction, roles, capabilities, rulebook, source, delegation and lifetime, but it does not bind one exact proposal/resource/action digest.

Therefore a broad grant containing `governance.execute` cannot by itself authorize every proposal in an institution.

`ExecutorDesignation` attenuates one verified grant to:

- one exact proposal;
- one exact executable-action digest;
- one exact digest profile;
- one exact qualified threshold-authorization reference;
- one exact executor principal;
- one required capability;
- the same institution / jurisdiction / rulebook; and
- a bounded lifetime no longer than either the grant or threshold authorization.

## Three independently verified facts

Qualification consumes:

1. `VerifiedThresholdAuthorization` — a runtime wrapper over PR #69's `QualifiedThresholdAuthorization`, explicitly joining the action-digest profile and immutable threshold-authorization reference;
2. `VerifiedAuthorityGrant` — the generic institutional grant plus exact grant/proof/delegation verification provenance; and
3. `VerifiedExecutorDesignation` — the exact scoped designation plus exact source/designation proof verification provenance.

No generic `authorized=true` / `verified=true` field is accepted.

## Grant constraints

The underlying AuthorityGrant must:

- be structurally valid and active now;
- belong to the same institution/jurisdiction/rulebook as the qualified threshold authority;
- be held by the designated executor;
- contain the exact capability required by the designation;
- match the designation's exact `authority_grant_id`; and
- have its exact `grant_proof_ref` independently verified.

When the grant is delegated, a separate non-empty delegation verification reference is mandatory.

## Designation constraints

The scoped designation must:

- bind the same proposal/action digest/profile as threshold authority;
- bind the same immutable threshold-authorization reference;
- use the same institution/jurisdiction/rulebook;
- be issued no earlier than the referenced threshold authorization was verified;
- remain entirely inside the underlying grant lifetime;
- remain entirely inside the threshold authorization lifetime;
- carry an explicit AuthoritySourceRef; and
- have both its source proof and designation attestation independently verified.

## Output

`QualifiedExecutorDesignation` contains the exact executor principal and an immutable `executor_authority_ref` (the designation record identity) for later use by PR #65 / PR #64.

It remains a semantic authorization fact, not a lifecycle claim and not an external-effect permit.

The Holochain runtime may further require `executor_principal` to be a `did:mycelix:` identity because v0.1 uses one exact source chain as the local claim-serialization fence. This pure crate stays identifier/substrate-neutral.

## Score separation

Phi, reputation, stake, model recommendations and consciousness metrics are not inputs. They cannot designate an executor or satisfy a missing capability.

## Effect safety remains separate

Being the correct executor does not prove that the external effect is safe to issue. The lifecycle verifier must still independently qualify the exact effect-safety policy/adapter before claim/effect execution.
