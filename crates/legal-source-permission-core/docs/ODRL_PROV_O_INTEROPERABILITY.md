# Legal source permission interoperability: ODRL and PROV-O

**Status:** proposal for review. The Rust admission engine remains the normative implementation for its declared tri-state policy semantics; this document does not claim full ODRL semantics or legal compliance.

## Why use standards instead of inventing a new universal format?

W3C ODRL 2.2 represents policies with Permissions, Prohibitions, Duties, Assets, parties, actions and constraints: https://www.w3.org/TR/odrl-model/. It is a suitable interchange vocabulary for rights-policy statements, including operation and time/territory constraints. W3C PROV-O models provenance using entities, activities, agents, derivation, use, generation and attribution: https://www.w3.org/TR/prov-o/. W3C Verifiable Credentials Data Model 2.0 can be considered later for signed reviewer/issuer attestations, after key trust, issuer identity, expiry and revocation semantics are specified: https://www.w3.org/TR/vc-data-model-2.0/.

A June 2026 paper, *What Does ODRL Mean?*, highlights a fundamental boundary: an ODRL policy evaluator's verdict does not itself establish the underlying normative/legal positions, institutional competence, or who has authority to declare a violation. It proposes modeling competence-level relations alongside conduct-level permission/duty relations: https://arxiv.org/abs/2606.24344.

This supports a layered design, not replacing the local fail-closed evaluator with ODRL alone.

## Proposed mapping

| Current Mycelix field | ODRL / PROV-O correspondence | Important limit |
|---|---|---|
| Source component and immutable version | ODRL Asset; \`prov:Entity\` for exact snapshot | Asset identity does not prove ownership, authenticity, or lawful access |
| Operation, purpose, actor, territory, interval | ODRL Action, Party, Constraint and profile vocabulary | Publish a versioned profile; do not assume arbitrary ODRL actions mean the same thing across organizations |
| \`allowed\` | ODRL Permission candidate | A translated permission statement still requires trusted policy provenance and profile review |
| \`denied\` | ODRL Prohibition candidate | Do not erase internal denial reason or source-specific scope |
| \`unknown\` | Retained in Mycelix admission envelope as \`unknown\` / not admitted | Do not map missing evidence to an ODRL Permission; ODRL's policy-rule vocabulary alone is not the evidence-state model |
| Terms snapshot, profile, request, and receipt byte digests | \`prov:Entity\` identifiers / content digests | A hash establishes byte identity only, not who published it or whether it is legally operative |
| Permission-check execution | \`prov:Activity\`; \`prov:used\` source, terms, profile and request entities | Activity records show inputs and process identity, not that the conclusion is correct |
| Evaluator build and reviewer | \`prov:Agent\`, \`prov:wasAssociatedWith\`, \`prov:wasAttributedTo\` | Signed reviewer identity is only as trustworthy as key issuance, scope, revocation, and verification |
| Resulting admission receipt | Output \`prov:Entity\`, \`prov:wasGeneratedBy\`, \`prov:wasDerivedFrom\` links | Reproducibility is not a legal opinion or a universal compliance guarantee |

## Required invariants

1. Keep the engine's three states separate: \`allowed\`, \`denied\`, \`unknown\`; only a matching reviewed profile plus a successful scope/time/digest/retention check can yield \`admitted\`.
2. ODRL export is a derived view of a reviewed grant, never a new source of permission. Round-trip import must fail closed for an unsupported action, constraint, duty, or profile term.
3. Preserve the exact policy/profile version, source snapshot ID/digest when available, request digest, evaluator version, decision reason, review evidence and time of decision.
4. A reviewer approval that occurs after the requested decision time must not retroactively authorize an earlier operation.
5. Do not treat ODRL's policy statement as proof of authority or legal validity; keep publisher/rights-holder identity, reviewer's scope, and applicable law separate.
6. PROV-O provenance chains must not include raw legal text or personal/case data merely for convenience. Prefer lawful source IDs/digests and data-minimized receipts.
7. Credentials should not be introduced until issuer trust, expiry, revocation, key rotation, scope, and failure behavior are executable and tested.

## Proposed interoperability acceptance tests

- A same-profile ODRL export is deterministic and includes every explicit allowed and denied operation without broadening scope.
- Unknown grants are preserved as unknown locally and never silently exported as a permission.
- Importing an ODRL policy with unknown actions/constraints fails closed.
- Two independent implementations produce identical PROV entity/activity relationships for the same source/profile/request/receipt digests.
- Source or terms digest mismatch changes the admission to unknown and produces a provenance reason that names the mismatched entity.
- Test time-of-review backdating, revoked/expired reviewer claims, and key-rotation failures before accepting signed credentials.
- Each standards mapping is versioned; no mapping result is presented as proof of copyright ownership, legal validity, or universal regulatory compliance.

This remains a compatibility proposal until schemas and executable round-trip tests are added.