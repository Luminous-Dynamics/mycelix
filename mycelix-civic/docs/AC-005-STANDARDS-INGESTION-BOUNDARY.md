# AC-005 — Standards Ingestion Boundary

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-005 projects a narrow subset of externally schema-validated OCDS procurement and BODS beneficial-ownership data into AC-003 without creating a proprietary source of truth.

Supported source families in this tranche:

- OCDS packages declaring version `1.1`, with adapter semantics authored against canonical schema revision 1.1.5;
- BODS version `0.4`.

AC-005 is not a full JSON Schema validator. Every ingestion requires a source reference, claimed content digest, and an upstream validation receipt reference. That receipt is expected to bind the exact source bytes to the digest and the external schema result; AC-005 only syntax-checks the digest and validates its own projection semantics.

The in-memory adapter is bounded to 16 MiB and 50,000 top-level releases/statements. Larger publications require a future qualified streaming/JSONL path.

## Epistemic boundary

Imported data remains a claim.

Every generated AC-003 edge is created as `AssertionStatus::Declared`, including data that an external publisher labels as verified. AC-005 cannot manufacture AC-003 `Corroborated`, AC-002 findings, or AC-001 adjudication.

The intended chain is:

external publication -> upstream schema/hash validation receipt -> AC-005 projection -> AC-003 declared edge -> AC-004 measurement -> AC-002 observation/signal -> human review

not:

external publication -> automatic guilt or consequence

## Privacy and identity boundary

AC-005 deliberately refuses automatic cross-dataset identity fusion.

OCDS party IDs and BODS `recordId` values have different scopes and semantics. They remain internal linkage keys for the source standard rather than universal Mycelix identity.

For BODS, raw `recordId` values never appear in exported graph node IDs. Entity and person nodes use deterministic source-digest + statement-index pseudonyms. This prevents a publisher-controlled person `recordId` from becoming an accidental public identifier while preserving reproducibility for the exact source bytes.

BODS natural-person identifier values are never emitted as identifier bindings and are never copied into warnings or graph IDs. Natural persons remain `PrivatePersonCredential` nodes. Diagnostics identify BODS input positions by statement index rather than echoing publisher-controlled record identifiers.

BODS relationship lookup is publisher-scoped. Equal local `recordId` values from different publisher scopes cannot satisfy one another's references.

### Deny-by-default entity identifier export

Cross-dataset identifier bindings are controlled by `StandardsIngestionPolicy`.

The policy contains:

- a stable `policy_ref`;
- an explicit allowlist of schemes approved for public/legal-entity reconciliation.

OCDS and BODS entity identifiers are exported only when their scheme is allowlisted. Non-allowlisted values are suppressed; warnings may identify the scheme but never the identifier value. Natural-person identifiers remain suppressed even if their scheme appears in the allowlist.

The ingestion result records the exact `policy_ref`, making the privacy decision part of the evidence lineage.

## OCDS projection

This tranche projects:

- buyer / `procuringEntity` participation -> `ProcurementRole::ProcuringAuthority`;
- `tenderer` party role -> `ProcurementRole::Bidder`;
- award suppliers -> `ProcurementRole::Awardee`;
- allowlisted organization identifiers -> separate `EntityIdentifierBinding` records.

The contracting process is keyed by globally scoped `ocid`. Party graph identity remains scoped to `ocid`, preventing local party IDs from being mistaken for global identity.

OCDS extensions fail closed in this tranche. AC-005 does not silently discard extension semantics it has not explicitly implemented.

## BODS projection

BODS is consumed as an ordered Statement stream. Relationship references must resolve to prior entity/person records within the same publisher scope before projection.

This tranche projects only `shareholding` interests into AC-003 ownership edges.

- exact percentages are converted deterministically into basis points without floating-point authority;
- values requiring rounding are rejected;
- percentage ranges are not midpointed; AC-003 receives `share_bps = None` plus an explicit loss warning;
- start/end date strings are not coerced into AC-003 integer validity timestamps and produce a warning;
- non-shareholding interests, including control-only interests, produce unsupported-interest warnings rather than being mislabeled as ownership.

Where `beneficialOwnershipOrControl = true`, the interested party must resolve to a `PrivatePersonCredential`, matching the BODS natural-person requirement.

## Provenance

Every generated edge carries an object-scoped source reference and the claimed digest bound by the upstream validation receipt. The edge's `recorded_at` is copied directly from typed ingestion evidence; AC-005 never reconstructs metadata by parsing an externally influenced provenance string.

Length-prefixed composite keys prevent ambiguous concatenation when building deterministic source/edge references.

## Fail-closed conditions

The adapter rejects, among other cases:

- oversized in-memory sources or record counts;
- missing source, validation-receipt, or policy provenance;
- malformed digest syntax;
- empty identifier schemes in the export policy;
- unsupported OCDS/BODS versions;
- any OCDS extension in this initial tranche;
- missing or unresolved OCDS process/release/party/award references needed for projection;
- invalid BODS statement IDs or missing record IDs;
- BODS relationships whose records have not previously appeared in the same publisher scope;
- person-as-subject BODS ownership relationships;
- `beneficialOwnershipOrControl=true` assertions whose interested party is not a person credential;
- exact BODS percentages that cannot be represented in basis points without rounding;
- any generated edge that fails AC-003 validation.

## Non-goals

AC-005 does not validate the complete external standards, import every OCDS field or extension, model BODS control-only relationships as ownership, expose natural-person identity documents, destructively merge identities across datasets, infer corruption, or produce sanctions.

## Qualification gate

Before AC-005 is treated as qualified infrastructure:

1. exact-subject `cargo test --workspace` for Mycelix Civic passes;
2. rustfmt and warnings-denied Clippy pass;
3. independently schema-valid OCDS 1.1 and BODS 0.4 fixtures are frozen;
4. mutation/adversarial fixtures cover unsupported extensions, unresolved references, publisher-scope collisions, private-record-ID leakage, private-identifier leakage, identifier-policy bypass, resource-bound bypass and percentage rounding;
5. an independent adapter reproduces graph IDs, edge IDs, public identifier bindings, timestamps and exact share projections;
6. privacy review confirms private-person record IDs and identifier values cannot escape through nodes, bindings, errors, warnings or serialized results;
7. standards review rechecks projection semantics against pinned OCDS 1.1.5 and BODS 0.4 documentation;
8. downstream AC-004 tests confirm imported assertions remain observations rather than adjudicated findings.

## Next tranche

AC-006 should be a reversible, evidence-bearing public-entity identity reconciliation layer over AC-005 allowlisted bindings. It should preserve conflicting claims, never destructively merge graph nodes, never use private-person identifiers as a general join primitive, and keep fuzzy/name-based matching outside the authoritative trust path.
