# Integral OAD -> COS vertical slice v1

Status: ReferenceModelOnly / SourceRefined

This artifact turns the current public OAD/COS descriptions into a concrete, testable semantic boundary without treating those descriptions as a ratified API.

## Source status

The current public material describes OAD as a ten-module design pipeline including structured submission, collaborative refinement, material/ecological analysis, lifecycle/maintainability, feasibility simulation, labor decomposition, systems integration, optimization, certification and commons archival.

It describes COS as a nine-module production-coordination system including planning/work breakdown, labor/skill matching, resource/material management, workflow execution, capacity/throughput, distribution/access, QA/safety, inter-cooperative coordination, and transparency/audit.

Episode descriptions are evidence about what is publicly described, not proof of a ratified schema or implementation. Preserve:

episode description != Development Guide proposal != Technical Specification != ratified schema != implementation != conformance evidence

Sources: Revolution Now! Episode 60 (May 4, 2026) and Episode 61 (September 2026).

## Vertical slice

OAD design generation -> seam envelope -> transport receipt -> COS semantic admission -> COS production basis -> explicit production authorization -> execution evidence

Every transition is explicit.

### OAD design generation

The package retains design identity/generation, source-status class, source schema generation, certification state, production-profile identity, material/skill/lifecycle/ecological model generations, and supersession state.

A design that is certified but stale for the current production profile is rejected.

### OAD certification vs COS admission vs production authorization

OAD certification != COS semantic admission != production authorization != execution receipt

Certification does not grant production authority. COS admission creates a production basis, not an execution receipt. Production authorization is a separate explicit record.

### Seam boundary

The neutral seam reference model requires exact profile/schema generation, semantic subject identity, logical delivery identity, attempt identity, payload commitment, and an explicit semantic-admission receipt.

Transport/provider acceptance remains Indeterminate for semantic admission.

### Federation

Foreign origin is preserved through recognition/admission. Recognition does not localize foreign evidence.

## Failure cases

1. stale design generation
2. wrong production profile
3. uncertified design
4. certification without production authority
5. admission without authorization
6. transport acceptance without semantic admission
7. explicit semantic admission
8. foreign-origin preservation

## Traceability

| Public requirement | Mycelix reference owner | Executable witness | Formal boundary | Claim ceiling |
|---|---|---|---|---|
| OAD design intelligence feeds COS | oad_cos_admission | admission + basis tests | COS-FV-007 | semantic integration only |
| certification | OadDesignPackage.certified | certification/authority test | COS-FV-004 | certification-state separation only |
| current design generation | design/profile generation | stale-generation tests | COS-FV-005 | freshness/currentness semantics only |
| COS production planning | CosProductionBasis | basis creation test | COS-FV-007 | planning-basis semantics only |
| inter-co-op provenance | seam origin | foreign-origin test | COS-FV-010 | provenance preservation only |
| provider delivery | seam_profile | transport-vs-admission test | COS-FV-007 | receipt semantics only |
| production authority | ProductionAuthorization | explicit-match test | COS-FV-001 | authority boundary only |

## Deliberate nonclaims

This reference slice does not establish a working Integral node, OAD/COS implementation completeness, physical production performance, manufacturing or safety qualification, economic or ecological outcomes, autonomous production authority, or Integral ratification/endorsement.

## Next qualification step

Replace OadDesignPackage with an actual source-owned OAD DTO only when an exact current source/schema is available. Then bind it to the real delivery/outbox/receipt path. Do not weaken this reference model to fit an incomplete runtime.
