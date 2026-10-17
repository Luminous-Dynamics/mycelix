# Food Allergen Disclosure Contract V1

**Status:** proposed contract for review  
**Date:** 2026-10-17  
**Related issue:** [MYC-FOOD-ALLERGEN-001 / #5175](https://github.com/Luminous-Dynamics/mycelix/issues/5175)  
**Integration:** [Neighborhood producer cooperative / #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001)

This document defines evidence semantics and evaluation behavior for food-allergen disclosures in Mycelix Commons. It is not a regulatory interpretation, medical advice, or a claim that a software contract can certify a food as allergen-safe.

## 1. The core semantic rule

**Absence of a matching allergen flag is not evidence that a disclosure is complete.**

The existing food-distribution `Listing` stores `allergen_flags: Vec<String>`, and the coordinator's `search_allergen_safe` function filters by whether any listed string matches the caller's exclusions. The current model does not independently encode whether the ingredient/formulation declaration was reviewed and complete, whether the product or lot changed, or whether cross-contact was assessed. An empty list is therefore ambiguous.

V1 must not collapse these distinct propositions:

- no matching allergen appears in the currently recorded list;
- the complete declared ingredient/formulation scope was reviewed;
- no excluded allergen is declared within that review scope;
- cross-contact was assessed for a specified handling/process scope;
- a person with an allergy can safely consume the physical item.

Software should make the first four evidence states inspectable. It must not claim the fifth from those facts alone.

## 2. Product identity, lot identity, and evidence scope

Every disclosure assertion must bind to the narrowest identity it actually describes:

- `product_ref`: canonical product identity and product-profile version;
- `formulation_ref`: ingredient/formulation version or content digest where available;
- `lot_ref`: specific production/harvest/packing lot when the claim is lot-specific;
- `supplier_ref`: source that supplied the assertion;
- `jurisdiction_scope`: applicable regulatory/product category context, when known;
- `evidence_ref`: source record, declaration, review, certificate, or inspection reference;
- `observed_at`: when the underlying state was observed or attested;
- `recorded_at`: when the platform received the evidence;
- `reviewer_role`: source/reviewer role, without exposing unnecessary personal data;
- `validity`: current, stale, withdrawn, superseded, conflicting, or unknown;
- `correction_of`: prior assertion/event when this supersedes or corrects it.

These fields are a proposed logical contract, not a final Rust/Holochain entry layout. Existing ProductProfile/Lot/HandlingEvidence/ClaimEvidence structures in MYC-FOOD-001 should be reused where their semantics fit; do not introduce a second canonical product, lot, identity, or provenance subsystem.

A source's signature proves that the source made an assertion; it does not independently prove that the physical product matches it.

## 3. Ingredient/formulation disclosure states

Represent status independently from the list of named allergens.

| Proposed status | Meaning | Query treatment |
|---|---|---|
| `DeclaredPresent` | One or more allergens are declared for this product/formulation scope. Each item should retain its normalized identity and source/evidence reference. | Exclude from a constrained candidate set when it conflicts with the user's exclusion profile; retain the declared reasons in the result. |
| `ReviewedNoneDeclared` | A scoped ingredient/formulation declaration was reviewed and no excluded allergen is declared within that documented scope. | May pass a narrowly named **declared-allergen filter** for that scope. This does not mean zero trace presence or universal safety. |
| `UnknownOrIncomplete` | Disclosure is absent, partial, ambiguous, unsupported, or required ingredients cannot be resolved. | Return `RequiresVerification`; never silently map to no allergens. |
| `StaleOrSuperseded` | Evidence no longer corresponds to the current product/formulation/lot, or it has been superseded by a correction. | Do not use the previous clean declaration as current evidence. |
| `Conflicting` | Relevant current claims disagree or cannot be reconciled. | Return `RequiresVerification` / `ConflictingEvidence`; do not resolve by averaging or choosing the more convenient claim. |
| `NotApplicable` | Only available where a documented scope says the ingredient-disclosure question does not apply. | Must include rationale; never use as a generic unknown-state fallback. |

`ReviewedNoneDeclared` is deliberately scoped: it records an inspection of the declared ingredients/formulation. It does not establish absence of trace contamination, guarantee the correctness of the declaration, or provide person-specific medical clearance.

### Legacy migration

- A legacy non-empty `allergen_flags` list can be retained as **declared-positive hints** for the listed items, with source/currentness marked unknown unless evidence exists. It must not imply that unlisted allergens were reviewed.
- A legacy empty `allergen_flags` list maps to `UnknownOrIncomplete`, not `ReviewedNoneDeclared`.
- Do not migrate existing listings to a stronger state simply to preserve result counts or UI behavior.
- Any user-facing label must accurately distinguish a matching declaration filter from a verified product/lot disclosure.

## 4. Cross-contact is a separate evidence stream

Ingredient declarations cannot stand in for evidence about unintended transfer from equipment, facilities, utensils, transport, or handling. Cross-contact requires an independent status, scope, and evidence record.

| Proposed status | Meaning | Query treatment |
|---|---|---|
| `AssessedForScope` | A documented handling/process scope was assessed; references and the scope are available. | Report the assessed scope and date. Do not generalize beyond it. |
| `KnownRiskOrControlGap` | A potential cross-contact path or unresolved control gap is recorded. | Expose the risk/evidence; do not represent it as cleared. |
| `NotAssessed` | No cross-contact assessment is recorded for the relevant scope. | Return `RequiresVerification` when a user needs this evidence. |
| `Unknown` | Available records do not establish the cross-contact state. | Preserve unknown. |
| `StaleOrConflicting` | Evidence is stale or conflicts with other records. | Invalidate or supersede dependent conclusions. |

Do not use generic statements such as "made in a facility that handles X" as a substitute for evidence of applicable controls; do not infer absence of cross-contact merely because no risk statement was supplied. The evidence contract records what was assessed and by whom/what source, not a universal guarantee.

## 5. Query/API semantics

Avoid an API name that states more than the evidence proves. The current `search_allergen_safe` name should be deprecated or wrapped by a documented compatibility boundary after callers are inventoried.

Proposed result contract:

```text
DeclaredAllergenFilterResultV1 {
  product_ref
  product_profile_version
  lot_ref?                       // when lot-specific evidence is used
  excluded_allergen_refs
  declaration_status
  declaration_evidence_refs[]
  cross_contact_status
  cross_contact_evidence_refs[]
  evaluated_at
  disposition: ExcludedKnownConflict
             | EligibleForDeclaredAllergenFilter
             | RequiresVerification
             | ConflictingEvidence
             | NotEvaluable
  limitations[]
}
```

Names are illustrative until aligned with canonical shared types and Holochain serialization conventions.

Expected semantics:

1. A known declared conflict is excluded with the reason preserved.
2. A scoped reviewed declaration with no excluded allergen declared may be eligible for the bounded declaration-filter query.
3. Unknown, incomplete, stale, or conflicting declarations are verification-required, not implicitly clean.
4. If the use case depends on cross-contact status and that status is not established, the result remains verification-required.
5. No result disposition should be rendered as universal "safe"; human-facing copy must state the exact scope and any required verification.
6. Household allergy preferences should remain private. Return the minimal product evidence needed to explain the disposition; do not publish buyer identity or profile data to the DHT.
7. The planner must consume an explicit disposition and preserve uncertainty. It must not fill unknown evidence with model predictions.

## 6. Corrections, reformulation, and recall propagation

Any change in product formulation, supplier, lot, or relevant handling evidence must allow dependent results to be identified and recalculated.

- A new product-profile version does not inherit a clean disclosure merely because its name is unchanged.
- A replacement ingredient or supplier can introduce a declared allergen and must invalidate affected cached recommendations.
- A product/lot recall or corrected disclosure must create an auditable supersession event.
- Delayed, duplicate, or replayed events must be idempotent.
- A stale consumer-visible result must not remain described as current simply because a cache or client did not refresh.
- A correction event should be traceable without revealing private household identities more widely than necessary.

Keep event time and arrival/recording time distinct so offline/replayed corrections do not rewrite history silently.

## 7. Deterministic adversarial acceptance corpus

Use a fixed fixture manifest, stable IDs, exact expected results, and a digest. Include at least these cases:

| ID | Fixture | Expected result |
|---|---|---|
| A01 | Current reviewed scoped ingredient declaration includes an excluded allergen | `ExcludedKnownConflict` |
| A02 | Current reviewed scoped declaration has no excluded allergen declared; cross-contact assessment separately says unknown | Eligible for bounded declaration filtering, but **not** a universal safe claim; require verification where cross-contact assurance is requested |
| A03 | Empty legacy flags with no completeness/evidence state | `RequiresVerification` |
| A04 | Compound ingredient cannot be resolved to a source/formula | `RequiresVerification` |
| A05 | Cross-contact explicitly not assessed | `RequiresVerification` when relevant to the query |
| A06 | Shared-equipment risk/control gap is documented | Expose the risk; do not mark cleared |
| A07 | Product reformulated or supplier changed after prior review | Prior disclosure is stale/superseded; require current evidence |
| A08 | Current claims conflict | `ConflictingEvidence` |
| A09 | Correction/recall arrives after recommendations or order events exist | Dependent results identified, invalidated or superseded idempotently |
| A10 | Duplicate/replayed correction event | No duplicate effect; current state remains consistent |
| A11 | Known non-conflicting declaration with complete in-scope evidence | Discoverable through the bounded declared-allergen filter |
| A12 | Household preference is private | Result contains no unnecessary buyer identity/profile disclosure |

All cases need positive controls as well as negative controls. Unknown and stale values must never be converted into a clean state to make recall or result volume look better.

## 8. Benchmarks and release gates

For the frozen corpus, target:

- **0** unknown/incomplete/stale/conflicting disclosures classified as safe merely because no matching flag exists;
- **100%** of known matching declarations excluded from the constrained result set;
- **100%** of candidate results traceable to the exact product/profile and disclosure version used;
- **100%** of tested reformulation/correction/recall events invalidate or supersede dependent derived results;
- idempotent replay of every duplicate-event case;
- separate reporting of false-exclusion and false-eligibility rates;
- exact commit SHA, fixture manifest digest, attempted/passed/failed/unknown counts, and per-case outcome in the benchmark receipt.

These targets are scoped to a synthetic test corpus. They are not a real-world allergy safety certification, and passing them does not establish that a physical item matches a record.

Promotion gates:

1. **Contract gate:** status names, evidence scope, migration mapping, and API compatibility reviewed.
2. **Integrity gate:** deterministic fixtures pass at entry validation and query/result boundaries.
3. **Propagation gate:** stale/reformulated/conflicting/recall evidence invalidates derived results as declared.
4. **Privacy gate:** household profiles are not exposed as a side effect of querying products.
5. **Field-use gate:** only after local product-category and jurisdiction review, validate workflow and source quality with real operators. Do not infer field validity from synthetic tests.

## 9. Authoritative methodological anchors

- USDA FSIS, [Food Allergies: The Big 9](https://www.fsis.usda.gov/food-safety/safe-food-handling-and-preparation/food-safety-basics/food-allergies-big-9), including distinctions between ingredient declarations and cross-contact and between FDA- and FSIS-regulated product scopes.
- USDA FSIS, [Allergens — Voluntary Labeling Statements](https://www.fsis.usda.gov/guidelines/2013-0010), on truthful, appropriately scoped allergen labeling statements and the fact that advisory statements are not a substitute for good manufacturing practices.
- USDA FSIS, [Allergen Statement Q&A, updated 12 August 2026](https://ask.fsis.usda.gov/article/askFSIS-Public-Q-A-Allergen-Statement).

These sources describe US-specific contexts. A South African pilot and any other jurisdiction needs its own competent local review. This spec is about evidence semantics, not legal approval.
