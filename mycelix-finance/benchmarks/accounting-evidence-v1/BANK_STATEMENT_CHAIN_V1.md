# FIN-BANK-CHAIN-001 — Bank statement chain integrity v1

**Status:** proposed research and conformance contract  
**Parent work:** [#5827](https://github.com/Luminous-Dynamics/mycelix/issues/5827)  
**Composes:** [#5812 CAMT evidence adapter](https://github.com/Luminous-Dynamics/mycelix/issues/5812), [#5790 evidence completeness](https://github.com/Luminous-Dynamics/mycelix/issues/5790), [#5798 assertion evidence](https://github.com/Luminous-Dynamics/mycelix/issues/5798), [#5803/#5805 workpaper DAG](https://github.com/Luminous-Dynamics/mycelix/issues/5803)

This document defines the cross-message, cross-period integrity contract for bank statement evidence. It is deliberately separate from CAMT parsing, the expected-source-population registry, accounting classification and journal projection.

## 1. Core design

A bank statement is an immutable source artifact. A statement chain is a derived, versioned claim about the relationship among those artifacts and an independently declared reporting scope.

The system must preserve three independent propositions:

1. **Artifact integrity:** these are the exact bytes received and the exact parser/profile that interpreted them.
2. **Chain continuity:** these artifacts form a sufficiently continuous sequence for this account, denomination, cutoff and expected period scope.
3. **Source authority/completeness:** the source channel and account identity are qualified, and the expected source population is sufficiently known.

Passing one proposition does not prove the others. In particular, XSD validity and exact SHA-256 preservation do not authenticate a bank sender, prove account ownership or establish that all expected statements were received.

## 2. Distinct evidence semantics

| Source evidence | What it can establish | What it cannot establish by itself |
|---|---|---|
| CAMT.052 account report | A time-bounded account-report observation under its profile | A complete booked statement or period close |
| CAMT.053 statement | The bank-reported statement entries and balance fields for its statement period | Complete account history, customer-ledger correctness or legal discharge |
| CAMT.054 debit/credit notification | A notification/advice observation | A second economic event when later booked in CAMT.053 |
| Bank-specific MUG / profile | The bank's declared use/narrowing of the ISO message | Authentication of a particular received payload |
| Authenticated transport/envelope receipt | A source/channel authentication result within its trust policy | Account ownership or truth of every field |

A notification and its later booked statement entry are two artifacts that may describe one economic observation. An intraday report may overlap a later statement. The chain representation must retain both artifacts and an explicit relationship without creating a second cash movement.

## 3. Canonical chain manifest

The eventual Draft 2020-12 manifest schema should contain at least:

- `schema_id`, `chain_id`, `chain_version`, creation time and verifier/profile digests;
- a pseudonymous account identity and effective-dated account-reference lineage evidence;
- a single denomination-profile reference for the chain;
- the expected-period/scope-manifest reference supplied by #5790;
- immutable artifact records: artifact ID, exact SHA-256, byte length, media type, source/channel reference, message definition/version, message ID, creation/capture times and validator bundle;
- statement/report records: period bounds, cutoff semantics, statement ID, sequence, page/continuation data and message family;
- balance observations: balance type, date/time, exact signed integer-string amount and denomination reference;
- explicit relations: exact replay, overlaps, predecessor/successor, notification-to-booked-entry, replaces, corrects, cancels, returns or reverses;
- a derived verdict and reason-coded limitations.

Raw XML remains outside the canonical projection as immutable source bytes; the manifest references its digest and source location. Normalization never replaces the original.

### Canonical amount and time rules

- No binary floating point, exponent notation, locale parsing or JavaScript number coercion for monetary values.
- Amounts are canonical integer strings in the exact unit admitted by the referenced denomination profile.
- Never infer a currency scale from a bank, account type, instrument, internal ledger asset or familiar currency label.
- Preserve message creation time, capture time, booking time, value date and correction time separately.
- Do not compare balances unless account identity, denomination, balance type and cutoff semantics are compatible.

## 4. Cross-period invariants

1. **Account continuity:** account references may change only through an explicit, source-backed, effective-dated lineage relation. Similarity of account strings is not identity.
2. **Balance continuity:** prior closing booked balance equals next opening booked balance only where the selected bank profile declares compatible cutoffs. Any permitted adjustment must be a separately identified source-bound item.
3. **No balancing plugs:** unexplained differences remain `balance_conflict`; the verifier may not synthesize a transaction to make the equation balance.
4. **Expected population:** periods and pages are evaluated against the expected-scope manifest, not inferred from the files received.
5. **Missing evidence:** missing period, page, sequence or source population is unknown, never zero.
6. **Overlap:** overlapping periods are not independent confirmations. Common entries are linked by lineage and counted once as economic observations.
7. **Replay:** exact bytes under the same artifact identity are idempotent. Same identity with changed bytes is a conflict unless an explicit, source-qualified revision relation exists.
8. **Revision:** a later capture timestamp does not authorize replacement. Supersession requires an explicit bank-specific relation and reason.
9. **Late correction:** later bookings, corrections, reversals and returns append successor observations. They do not silently rewrite historical receipts or workpapers.
10. **CAMT role:** CAMT.052 and CAMT.054 may enrich event context; neither independently fills a missing CAMT.053 booked statement period.
11. **Temporal determinism:** arrival order must not change the final verdict for the same frozen evidence set and pinned policies.
12. **Selective invalidation:** a changed artifact invalidates only its dependent chain nodes and downstream workpapers.

## 5. Verdicts are multi-dimensional

Do not return a single opaque confidence score. At minimum distinguish these states:

- `complete_contiguous`
- `complete_with_explicit_adjustment`
- `gap_unknown`
- `overlap_unresolved`
- `balance_conflict`
- `revision_conflict`
- `account_lineage_unknown`
- `profile_unsupported`
- `source_authentication_unqualified`

The final result should expose separate axes for artifact/schema conformance, account identity, denomination compatibility, chain continuity, expected-population completeness and source authentication. A chain may be continuous while authenticity is unqualified, or each supplied statement may balance while the expected population has a gap.

## 6. Initial adversarial and metamorphic corpus

The following are candidate-visible scenario families; expected verdicts and mutation labels belong in the evaluator-only bundle.

| ID | Scenario | Primary property |
|---|---|---|
| BSC-001 | Two adjacent statements with matching booked carry-forward | Positive continuity |
| BSC-002 | One-minor-unit opening/closing mismatch | Exact arithmetic |
| BSC-003 | Calendar dates match but cutoff instants differ | Cutoff semantics |
| BSC-004 | Exact statement replay | Idempotency |
| BSC-005 | Same message identity, changed bytes | Conflict, no overwrite |
| BSC-006 | Explicitly referenced replacement statement | Qualified supersession |
| BSC-007 | Competing versions with no supersession evidence | Revision conflict |
| BSC-008 | Overlap with one identical booked entry | One economic observation |
| BSC-009 | Overlap with changed amount or currency | Unresolved conflict |
| BSC-010 | Entire expected statement period missing | Never complete |
| BSC-011 | Sequence gap or missing page | Population gap |
| BSC-012 | CAMT.052 exists but expected CAMT.053 is absent | Report is not statement |
| BSC-013 | CAMT.054 notification has no booked successor | Notification remains advice |
| BSC-014 | CAMT.054 linked to CAMT.053 booked entry | Two artifacts, one observation |
| BSC-015 | Prior statement corrected after close | Append-only revision |
| BSC-016 | Late booking with earlier value date | Distinct temporal clocks |
| BSC-017 | Return/reversal after original entry | Successor relation |
| BSC-018 | Account reference changed without lineage evidence | Account identity unknown |
| BSC-019 | Currency registry/profile revision unsupported | No guessed scale |
| BSC-020 | Available balance used as booked closing balance | Balance-type isolation |
| BSC-021 | Explicit bank fee versus unexplained balancing plug | Source-bound adjustment |
| BSC-022 | Account identifier collision across entities | Entity isolation |
| BSC-023 | Schema-valid message on unauthenticated channel | Authority remains unqualified |
| BSC-024 | Received files parse but contradict expected scope | Completeness is external |
| BSC-025 | Same artifacts arrive in a different order | Arrival-order invariance |
| BSC-026 | Exact replay is added to an otherwise complete chain | Economic count unchanged\n| BSC-027 | One-second gap at a required statement-period boundary | Never complete\n| BSC-028 | Statement sequence skips the expected source sequence | Population gap
| BSC-029 | Exact replay includes a duplicate statement record | One statement retained
| BSC-030 | Replacement relation crosses account identity | Revision conflict
| BSC-031 | Local economic-event reference collides across accounts | Count each account-scoped event separately
| BSC-032 | Exact replay source digest but changed normalized statement projection | Revision conflict
| BSC-033 | Cyclic replacement/correction relation graph | Revision conflict |

The corpus must be synthetic, rights-cleared, immutable and split physically between candidate-visible inputs and evaluator-only oracle data. Each case needs positive/negative controls and explicit source/profile/scope preconditions.

## 7. Comparison protocol

Use the same frozen source bundle and expected-scope manifest for each candidate:

1. deterministic baseline using only pinned bank-profile rules;
2. incumbent statement import/reconciliation export, where lawful sample access exists;
3. model-only extraction and matching;
4. deterministic parser plus chain verifier;
5. parser + verifier + source-population manifest + exception analyst;
6. human accountant/bookkeeper baseline with the same evidence and scope.

Report false-complete and false-conflict rates separately. Also report overlap handling, revision disposition, late-arrival recovery, false matches, missed periods, exact replay suppression, p50/p95 runtime and fully loaded review/rework cost. Never exchange safety for higher automatic-match coverage.

## 8. Hard qualification gates

- Zero false `complete_contiguous` verdicts when any expected period/page/population is missing.
- Zero carry-forward checks across incompatible account, denomination, balance type or cutoff semantics.
- Zero source artifact overwrite or unqualified version selection.
- Zero duplicate economic observations from overlap, replay or notification-to-statement correlation.
- Zero guessed amount scale, float coercion or implicit FX.
- 100% deterministic results under artifact arrival permutations.
- 100% affected-descendant invalidation and zero unrelated-chain invalidation.
- Every unqualified source, schema, scope or identity condition is visible in the receipt.

No weighted average may offset a false-complete result or an altered amount.

## 9. Implementation sequence

1. Freeze the semantics and schema without depending on a live parser.
2. Author the candidate/evaluator-separated synthetic cases and mutation operators.
3. Implement a dependency-light Rust semantic verifier with independent tests.
4. Add metamorphic arrival-order, replay and selective-invalidation tests.
5. Compose with the #5812 CAMT evidence adapter and #5790 source-scope manifest after their versioned interfaces exist.
6. Attach exact-head format/test/Clippy checks and reproducible input/output digests.
7. Add real bank profiles only from authorized, version-pinned usage guides and read-only shadow evidence.

**No claim of live bank connectivity, account ownership, source completeness, settlement finality, legal discharge, production accounting competence or regulatory compliance follows from this synthetic contract.**
