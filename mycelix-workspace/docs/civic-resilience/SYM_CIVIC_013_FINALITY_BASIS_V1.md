# SYM-CIVIC-013 — substantive finality-basis boundary v1

Status: synthetic research provenance qualification only

Parent: qualified SYM-CIVIC-012 / `0fee3f3708e66a40b02420cbedfed14e33457aa9`

Canonical publication receipt anchor:
`3b9c05f6dd4cf54603516ec8af207745dd9acd826581fb445a0dbbed47f1a87a`

## Purpose

SYM-CIVIC-013 qualifies a stronger proposition than finality assertion integrity:

`AUTHENTIC != SUFFICIENT`

A positive finality basis is admissible only when the exact Decision, exact coverage/resolution evidence, exact timing inputs, exact authority profile, and exact semantic identity commitments agree.

## Qualification law

Closed appeal-window sufficiency requires exact Decision binding, exact coverage universe/scope, qualification at or after the exact appeal deadline, coverage beginning no later than rendering, coverage through the qualification cut, explicit completeness, a supported non-inclusion proof, an exactly empty observed appeal set, exact deadline provenance, and exact trusted authority binding.

Terminal appeal-resolution sufficiency requires exact Appeal/Decision/Resolution binding, filing before the exact deadline, terminal `AFFIRMED` disposition, resolution after filing, resolution no later than qualification, and exact appellate authority binding.

`AUTHENTIC` is therefore a necessary condition, not a sufficient one.

## Typed dispositions

- `REJECT_FINALITY_BASIS_PROVENANCE`
- `FINALITY_BASIS_SUFFICIENT`
- `FINALITY_BASIS_INSUFFICIENT`
- `FINALITY_BASIS_UNRESOLVED`

A sufficient substantive basis is not authorization, legal finality, judicial status, execution authority, or civic authority.

## No-Frankenstein timing

The appeal deadline is an exact asserted datum. The qualifier does not invent jurisdictional, calendar, business-day, or legal semantics.

Decision rendering time, appeal deadline, evidence cut, and qualification time are bound as exact semantic inputs. A changed semantic generation requires a new identity commitment.

The freshness-profile commitment and the evidence freshness result are deliberately separate. The exact freshness-profile commitment is a provenance/identity requirement: changing or removing it is a provenance rejection. A coverage cut that is validly bound to the profile but falls outside the required freshness boundary is substantive insufficiency, not identity rejection. This prevents a stale-evidence defect from being misclassified merely because the profile itself is authentic.

## Corpus

The machine corpus contains exactly 42 cases and no embedded disposition/oracle field.

Canonical census:

`REJECT_FINALITY_BASIS_PROVENANCE = 17`
`FINALITY_BASIS_SUFFICIENT = 5`
`FINALITY_BASIS_INSUFFICIENT = 20`
`FINALITY_BASIS_UNRESOLVED = 0`

The fixture file stores a compact case-specification DSL. The executable qualifier expands each specification into a complete candidate, constructs canonical identity commitments, and derives the disposition from semantic predicates rather than case names.

## Metamorphic obligations

The qualifier exercises semantic identity mutation, completeness removal, decisive evidence insertion, coverage shortening, stale evidence, freshness-profile commitment mutation/removal, unsupported proof type, local-empty queries, terminal disposition changes, resolution-before-filing, future-resolution evidence, and exact replay.

## Standards alignment

RFC 9942 (https://www.rfc-editor.org/rfc/rfc9942) defines COSE Receipts as signed proofs concerning VDS states at issuance and distinguishes proof types such as inclusion, consistency, disclosure, and non-inclusion.

RFC 9943 (https://www.rfc-editor.org/rfc/rfc9943) requires SCITT transparency VDSs to provide append-only, non-equivocation, and replayability, while allowing additional VDS-specific proof types.

The in-toto Bundle specification (https://github.com/in-toto/attestation/blob/main/spec/v1/bundle.md) explicitly discusses deletion, replay, injection, and order-independence hazards for collections of individually authenticated attestations.

## Relationship to SYM-CIVIC-014

This tranche establishes only the substantive finality-basis boundary. It does not yet turn coverage continuity, coverage-universe authenticity, or non-equivocation into universal theorems.

Those deeper obligations remain isolated in SYM-CIVIC-014, where completeness must be evidenced constructively through coverage-universe identity, checkpoint continuity, consistency/non-equivocation witnesses, and freshness.

## Qualification ceiling

A green exact-head run establishes only that this synthetic reference model distinguishes authenticated evidence from sufficient finality-basis evidence, binds scope/cut/timing/identity, rejects Frankenstein composition, and preserves the non-authorization boundary.

It does not establish global DHT completeness, authenticity of real Justice runtime records, institutional appellate legitimacy, legal or judicial finality, operational authorization, civic authority, or production security.

No Rust, Holochain, Finance, Business, network, credential, or execution semantics are introduced.
