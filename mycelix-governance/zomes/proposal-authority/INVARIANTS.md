# Proposal Authority Adapter Invariants

These invariants are normative for v0.1 and should be converted into
SweetConductor/adversarial tests before the adapter is marked merge-ready.

- **PAC-01 — Legacy schema stability:** `proposals_integrity::Proposal` is not
  modified by this adapter.
- **PAC-02 — Exact action identity:** the binding action digest is recomputed
  from the current proposal id and exact action JSON bytes using the profile
  named by `ACTIONS_DIGEST_PROFILE_V1`.
- **PAC-03 — Author binding:** only the committing `did:mycelix` proposal author
  may create a `ProposalAuthorityBinding`.
- **PAC-04 — Pre-activation binding:** a new context is created only for Draft
  proposals. Active proposals may only reuse a semantically equivalent context
  already committed before activation.
- **PAC-05 — Append-only:** authority entries and links cannot be updated or
  deleted.
- **PAC-06 — Stale-context safety:** changing Draft action bytes makes prior
  contexts non-matching; they remain historical and cannot regain authority by
  timestamp ordering.
- **PAC-07 — Ambiguity denial:** conflicting contexts that match the same current
  proposal bytes cause verified lookup to fail closed.
- **PAC-08 — Explicit institution:** context binds institution, optional
  jurisdiction, exact rulebook, governing body, signing-policy id and digest.
- **PAC-09 — Explicit digest profiles:** both action digest and signing-policy
  digest semantics are named; a bare 32-byte value is not treated as a universal
  identifier.
- **PAC-10 — Expiry:** authority context must remain valid beyond the voting
  period and consumers must reject it after expiry.
- **PAC-11 — No signature legitimacy claim:** this adapter does not itself prove
  the signing policy was institutionally adopted or cryptographically verified.
- **PAC-12 — Consumer fail-closed:** new binding voting/signing/execution paths
  must treat missing, stale, expired, or ambiguous context as denial.
