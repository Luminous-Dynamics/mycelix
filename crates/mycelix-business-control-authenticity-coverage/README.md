# Mycelix Business Control Authenticity Coverage

`mycelix-business-control-authenticity-coverage` proves that every control statement used by a full-interval control-coverage theorem has a current externally verified authenticity receipt under one preregistered verifier policy.

It does **not** verify cryptography itself. Verification remains owned by an external authority domain such as Xenia or Identity.

## Preregistered verifier policy

The authenticity coverage plan is frozen no later than the underlying control-coverage plan and binds:

- the exact control-coverage plan digest;
- claimed control issuer and business scope;
- full qualification interval;
- verifier domain;
- verification method; and
- verification policy digest.

Provider signing credentials may rotate during a long pilot; each external receipt therefore binds its exact credential and credential epoch independently.

## Per-window proof

Every control window must have exactly one authenticity entry. For each window the verifier checks that:

- the control contract/statement/reconciliation digests are the ones admitted by control coverage;
- the authenticity receipt is bound to the statement's exact source-document digest;
- the claimed issuer is the preregistered control source;
- verifier domain/method/policy match the preregistered verifier policy;
- verification did not predate issuance of the control statement;
- the current verifier epoch, credential epoch and revocation frontier still match; and
- verifier-receipt and authenticity-binding digests are not reused across windows.

The resulting window evidence retains the statement/document, external receipt, credential epoch, revocation frontier and revalidation time.

## Limitation evolution

Only complete authenticity coverage may support:

`limitation:control-source-authenticity-unverified:v1`
→ `limitation:control-source-issuer-authority-unverified:v1`

This is intentionally narrow. Authenticating who issued a control report does not prove that the issuer was institutionally authorized to define the economic fact, nor that the report corresponds to physical reality.

## Safety

This crate is read-only, performs no signature verification, grants no authority, and cannot mutate a provider or business system.
