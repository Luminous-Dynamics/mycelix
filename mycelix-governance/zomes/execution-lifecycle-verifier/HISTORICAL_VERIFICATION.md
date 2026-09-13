# Historical Execution Lifecycle Verification

Status: **required follow-on before effect execution**

The lifecycle stack distinguishes two questions that MUST NOT be conflated:

1. **May a new execution claim/effect happen now?**
2. **Was a historical lifecycle event authorized when it happened?**

The live path requires fresh, non-expired threshold authorization, executor authority, and effect-safety policy. That is correct for `ReadyAuthorized` / `Claimed` admission and for any future pre-effect recheck.

It is not sufficient as the only audit verifier.

## Historical authority must survive expiry

After a successfully completed attempt, later expiry/revocation of committee/signing authority, executor grant/delegation, or effect-safety policy must prevent **new** execution but must not erase the ability to verify the historical facts that:

- the exact authorization existed at the event time;
- the exact executor was designated and current at the event time;
- the exact delegation lineage/policy, when present, was current at the event time;
- the exact effect-safety profile qualified the attempt at the event time; and
- completion/failure/uncertainty evidence binds the exact attempt.

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
→ verifies immutable receipts and the exact authority generations effective at `at_ms`.

Historical lifecycle projection should qualify stored terminal events with the as-of verifier while the live claim/effect path continues to use current verification.

## Generation-aware revocation semantics

The historical verifier must distinguish:

- ordinary expiry after the event;
- revocation/supersession generation that became effective after the event;
- evidence that the authority was already invalid/revoked before the event;
- retroactively discovered forgery/invalid signature/proof; and
- ambiguous or missing historical generation evidence.

Only authority invalidity that applies at/before the historical event, or proof that the purported historical evidence was never valid, invalidates that event itself. Later ordinary expiry or later revocation prevents future use but does not rewrite history.

For delegated executor authority, historical replay must reconstruct the exact grant/policy/delegation generations effective at the event time rather than using today's current-lineage qualifier.

For threshold authority, historical replay must evaluate the exact signing policy, committee/key epoch and revocation generation effective at signature/event time.

For direct executor authority, historical replay still requires the exact grant and executor-designation generations effective at the event time; absence of a delegation chain is not absence of revocation semantics.

## Historical proof immutability vs current-state providers

Historical verification must consume immutable authority/proof records plus an authoritative as-of generation history. A provider that only exposes today's current snapshot is insufficient for audit replay.

Current-state lease expiry is irrelevant to the stable identity of historical authority; however the authenticity of the historical records and generation history must still be verified now.

## Relation to `Uncertain`

`Uncertain` remains a historical terminal fact until governed reconciliation produces a new explicitly authorized lineage/version. Historical verification must never auto-convert uncertainty into retry authority.

## Merge discipline

No external-effect adapter should be enabled until both live current-authority verification and durable historical/as-of verification semantics are specified and adversarially tested.
