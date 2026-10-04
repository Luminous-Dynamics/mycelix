# AC-078: Explicit Decision Tally Evidence

## Purpose

AC-078 connects a substantive DecisionOutcome to explicit vote evidence instead of relying only on runtime aggregation.

## Outcome evidence

New outcomes carry:

- `tally_vote_refs`: ActionHashes of the vote records used by the finalizer;
- `tally`: deterministic `(option_index, total_weight_bp)` snapshot;
- `tally_fingerprint`: Blake2b-256 fingerprint of the canonical vote references and tally.

Legacy outcomes may omit these fields and are represented as unknown. No evidence is fabricated for historical records.

## Integrity checks

New outcomes are checked against the exact AC-072 finalization basis and:

1. vote references are unique and bounded;
2. each referenced Vote belongs to the same Decision;
3. the tally is reconstructed from the referenced Vote records;
4. the stored tally exactly matches the reconstructed tally;
5. at least one referenced option has positive weight;
6. `chosen_option` matches the deterministic winning option;
7. the fingerprint matches the normalized references and tally.

## Completeness boundary

The explicit reference set proves what vote records the outcome claims to have used. It does **not** prove that the set contains every valid vote link in the DHT, because collection-style link queries are not deterministic validation inputs. Holochain's validation guidance explicitly recommends addressable dependencies and deterministic `must_get_*` retrieval for this reason. urlHolochain validation documentationhttps://developer.holochain.org/build/validation/

Completeness therefore remains a separate runtime/network evidence property.

## Relationship to previous controls

AC-071: deterministic competing-outcome selection.
AC-072: exact Decision finalization basis.
AC-073: deterministic current Decision revision.
AC-075: Finalized > Closed > Open lifecycle precedence.
AC-077: outcome semantic fields match the basis.
AC-078: substantive result is reproducibly tied to explicit vote evidence.