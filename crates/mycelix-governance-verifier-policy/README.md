# Mycelix Governance Verifier Policy v0.1

Pure Rust policy kernel for the trust decisions behind binding civic governance.

This crate does **not** verify signatures, query Holochain, call trust registries, or inspect Symthaea scores. It consumes already-verified evidence from adapters and answers whether that evidence satisfies an exact institution/jurisdiction/rulebook policy.

## Core rule

> Cryptographic validity is necessary evidence, not institutional authority.

A self-issued W3C VC can be perfectly well signed and still carry zero civic authority. Likewise, five observer identities controlled through one operational/trust domain are not five independent confirmations.

## Credential eligibility

`EligibilityVerifierPolicy` binds:

- institution and optional jurisdiction;
- exact rulebook id/version/digest;
- the civic right being evaluated;
- accepted credential type(s) and schema(s);
- exact issuer DID **or** an exact trust-registry/profile/relationship requirement;
- optional privacy-preserving predicate requirements;
- any/all/N-of-M evidence composition;
- optional requirement for distinct credentials across rules;
- maximum age of verification and revocation evidence;
- validity interval;
- institutional authority/proof that adopted the policy.

`VerifiedCredentialEvidence` carries only the output of a real credential-verification adapter: subject, issuer, type/schema, validity window, verification/revocation timestamps, receipt references, issuer-trust evidence, and verified predicates.

The policy evaluator never accepts reputation, Phi, stake, wealth, model confidence, or advisory score as a civic-right source.

### Native Mycelix VC adapter

The existing Identity VC coordinator is a good first adapter because its `verify_credential` path already verifies signature, expiration, proof purpose, issuer DID shape, and revocation status fail-closed.

However, Identity intentionally allows ordinary agents to issue credentials. The governance verifier must therefore apply this crate's exact issuer/trust policy after cryptographic verification rather than treating `valid: true` as governance authority.

For governance use, schema handling must also be strict. A credential issued while schema validation was skipped must not satisfy a rule that requires an exact governance schema unless the verifier independently validates that schema contract.

## Finality independence

`BallotFinalityPolicy` binds:

- exact ballot-set digest profile;
- minimum observer count;
- minimum independent trust-domain count;
- maximum observers one trust domain can contribute;
- allowed trust domains;
- optional exact observer allowlist;
- observation/finalization window;
- institutional authority/proof adopting the policy.

`evaluate_ballot_set_finality` rejects:

- wrong ballot-set digest/profile;
- duplicate observer identities;
- unapproved trust domains;
- unapproved observers;
- observations outside the rulebook window;
- too many observers from one trust domain;
- insufficient observers;
- insufficient independent trust domains.

This intentionally implements:

> distinct observer IDs != independent evidence

## Adapter contract

The future `governance_rights_verifier` Holochain zome should be a thin orchestration layer around this policy kernel plus credential/trust/finality adapters.

It should:

1. resolve the exact rulebook-bound verifier policy;
2. cryptographically verify credentials/presentations and revocation state;
3. verify issuer authority directly or through an accepted trust-registry adapter;
4. convert only those successful checks into `VerifiedCredentialEvidence`;
5. evaluate eligibility through this crate;
6. issue a receipt bound to the exact proposal/election/principal/snapshot;
7. verify observer authorization and signatures for the exact ballot-set digest;
8. evaluate finality through this crate;
9. issue a checkpoint receipt bound to the exact finality policy and ballot set.

Missing policy, stale evidence, unknown revocation state, unavailable trust registry, unknown issuer, missing observer authorization, or dependency failure must deny rather than degrade.

## Non-goals

- no proprietary identity protocol;
- no new DID method;
- no score-based personhood;
- no governance token requirement;
- no live reputation lookup during tallying;
- no assumption that one process/key/endpoint equals one independent trust domain;
- no claim that a receipt string is cryptographic proof until an adapter verifies it.
