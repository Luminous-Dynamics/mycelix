# Historical Execution Lifecycle Verification

Status: **required follow-on before effect execution**

The lifecycle stack distinguishes two questions that MUST NOT be conflated:

1. **May a new execution claim/effect happen now?**
2. **Was a historical lifecycle event authorized when it happened?**

PR #65 currently implements the first question. `resolve_execution_domain` requires fresh, non-expired threshold authorization, executor designation, and effect-safety policy. That is correct for `ReadyAuthorized` / `Claimed` admission and for any future pre-effect recheck.

It is not sufficient as the only audit verifier.

## Historical authority must survive expiry

After a successfully completed attempt, later expiry/revocation of the committee lease, executor grant, or effect-safety policy must prevent **new** execution but must not erase the ability to verify the historical facts that:

- the exact authorization existed at the event time;
- the exact executor was designated at the event time;
- the exact effect-safety profile qualified the attempt at the event time; and
- the completion/failure/uncertainty evidence binds the exact attempt.

Therefore a later tranche MUST introduce a historical/as-of verifier that verifies immutable authorization receipts against their event-time validity windows and revocation generations rather than requiring them to be valid at current wall-clock time.

## No carry-forward

Historical verification can establish that an old event was valid **then**. It can never re-enable execution **now**.

A historical receipt MUST NOT satisfy:

- `resolve_execution_domain` for a new claim;
- a Ready promotion;
- a current effect-safety check; or
- a future retry/reconciliation action requiring new authority.

## Suggested API split

Live path:

`resolve_execution_domain(proposal_id)`
→ requires all dynamic authority valid now.

Historical path:

`verify_execution_domain_as_of(domain_digest, authority_receipts, at_ms)`
→ verifies immutable receipts were valid for that exact domain at `at_ms`.

Historical lifecycle projection should then qualify stored terminal events with the as-of verifier while the live claim/effect path continues to use current verification.

## Revocation semantics

The historical verifier must distinguish:

- ordinary expiry after the event;
- revocation that became effective after the event;
- evidence that the grant was invalid/revoked before the event; and
- retroactively discovered forgery/invalid signature.

Only the latter two invalidate the historical event itself. Later ordinary expiry or later revocation prevents future use but does not rewrite history.

## Relation to `Uncertain`

`Uncertain` remains a historical terminal fact until governed reconciliation produces a new explicitly authorized lineage/version. Historical verification must never auto-convert uncertainty into retry authority.

## Merge discipline

No external-effect adapter should be enabled until both live current-authority verification and durable historical/as-of verification semantics are specified and adversarially tested.
