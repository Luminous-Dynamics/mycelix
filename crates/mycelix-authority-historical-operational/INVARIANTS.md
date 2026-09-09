# Mycelix Authority Historical Operational v0.1 — Normative Invariants

Status: **pure current-trust verification of historical operational authority; never live current authority**

This layer is the causal/historical sibling of PR #117. It exists so a protocol may verify that one exact operational authority subject was historically Active at a signer-committed causal authority-state coordinate without converting that historical fact into present execution authority.

A crucial temporal distinction is normative here:

> the current constitution-rooted operational context decides which evidence is trusted for verification **now**; it does not retroactively create the historical authority decision.

The historical authority-state transition itself commits its exact `authority_ref` and `authority_proof_ref`, and `VerifiedAuthorityStateTransition` must independently echo/verify those exact historical provenance references before this layer can use the transition.

## 1. The operational verification context is current first

The qualifier accepts only an opaque `QualifiedOperationalPolicyContext` from #116 and revalidates it at `now_ms` before inspecting historical state.

That current context governs the source/witness/coverage trust policy under which historical evidence is accepted now. A stale root/context cannot be used to verify a historical authority claim.

It does **not** mean the current constitution retroactively authorized the historical transition.

## 2. Historical transition authority provenance remains independent

Each `AuthorityStateTransition` already commits:

- exact `authority_ref`;
- exact `authority_proof_ref`; and
- exact record proof reference.

The corresponding `VerifiedAuthorityStateTransition` must match those fields exactly. Those verified historical authority/record proofs remain an independent provider-verification boundary and are part of the immutable transition identity.

The #116 current policy context does not replace or rewrite that historical provenance.

## 3. The same challenge/source/witness coverage path as #117 is mandatory

The qualifier requires:

- exact `VerifiedCoverageChallenge` for the target subject;
- exact context and coverage policy digest binding;
- exact source-head receipt;
- exact witness set and trust bindings; and
- `qualify_context_bound_coverage` success.

It then derives the `VerifiedAuthorityStateCoverage` internally from that qualified context-bound coverage.

A caller may not bypass this layer by passing a loose lower-level state-coverage receipt directly.

## 4. Historical selection is causal, not temporal

The target is selected only by:

- exact operational `AuthoritySubjectRef` from #116;
- exact authority-state generation; and
- exact authority-state transition digest.

No wall-clock `as_of`, newest-record, DHT-order, highest-local-generation or event-time heuristic is permitted.

The supplied causal coordinate is expected to come from another protocol's signed transcript. This theorem does not choose it.

## 5. Full currently covered history comes before historical selection

The context-bound coverage is converted into state-source coverage and passed into #429.

#429 re-runs the complete covered authority-state lineage before selecting the target generation/digest. A valid historical prefix cannot hide a later revoke/reactivate/supersede transition.

## 6. Historical existence is still not enough

After #429 selects the exact historical state, the theorem requires #446 `qualify_active_causal_authority_at`.

Only the exact `Active` state may qualify. `Revoked` and `Superseded` coordinates remain valid historical facts but cannot become positive operational authority.

## 7. Exact binding is rechecked after opaque qualification

Defense in depth requires the #446 result to echo exactly:

- #116 target subject;
- requested causal generation; and
- requested causal transition digest.

Any mismatch fails closed before the final historical operational capability is constructed.

## 8. Current verification policy and historical authority identity are both committed

The stable historical operational authority digest commits:

- #116 bootstrap-root qualification digest;
- #116 operational policy-context qualification digest;
- #446 exact active causal-authority digest;
- exact target generation; and
- exact target transition digest.

The first two identify the currently accepted verification policy context. The #446 identity commits the selected historical authority transition and its complete covered history.

Changing the current root/trust policy, historical causal state, or signed causal coordinate changes the qualification identity rather than silently reinterpreting the same proof.

## 9. Dynamic evidence remains separate

The stable `authority_digest` excludes the fresh challenge/source/witness proof instance.

The separate `evidence_digest` commits:

- stable historical operational authority identity;
- exact context-bound coverage evidence identity; and
- exact #429 causal projection identity.

Refreshing evidence for the same root/context/history/coordinate may change evidence provenance without creating a new stable authority identity.

## 10. Evidence reuse is bounded

The final verification time is the maximum of:

- #116 operational-context verification time; and
- #446 complete historical-source verification time.

The final validity horizon is the minimum of:

- #116 operational-context validity; and
- #446 complete-source coverage lease.

The result cannot be minted or reused after that combined horizon expires.

## 11. No current-freshness conversion

`QualifiedHistoricalOperationalAuthority` exposes no conversion to:

- `VerifiedAuthorityFreshness`;
- `QualifiedOperationalSubjectFreshness`; or
- `QualifiedCurrentOperationalAuthority`.

Historical authority cannot reactivate a signer or authorize a present external effect.

## 12. No provider or network heuristic authority

This crate contains no HDK/Holochain calls and does not infer authority from:

- DHT absence;
- local cache state;
- timestamps;
- reputation;
- Phi/consciousness;
- provider confidence; or
- model output.

Providers may supply independently verified evidence candidates to lower layers. They do not select the causal target or create a positive historical capability by assertion.

## 13. Identity convergence target

After Identity/authority ancestry convergence, each #454 transition requirement must map through #464 to one exact generic subject and then match one `QualifiedHistoricalOperationalAuthority` by exact:

- subject;
- signed authority-state generation; and
- signed authority-state transition digest.

Only then may that Identity policy transition be considered historically authorized under an independently verified historical authority decision and the currently accepted verification policy context.
