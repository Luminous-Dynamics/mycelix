# aero-qif-locator-inventory

Independent, bounded, source-side inventory of QIF 3.0 trace fields for the AeroCommons Test 7/8 profile.

## Purpose

This crate answers a different question from an adapter or extractor:

> Given these exact QIF source bytes and this pinned trace profile, which trace-relevant fields are present, where are they, and which typed ID namespaces and relationship values do they carry?

The scanner consumes raw XML, source roles and QPIds. It does **not** read adapter-produced mapping rows or normalized graph outputs. Its expected locator list can be compared against a hand-enumerated oracle and passed into the Mycelix QIF ingestion envelope as `required_locators`.

## Run

    cargo fmt --manifest-path tools/aero-qif-locator-inventory/Cargo.toml -- --check
    cargo test --manifest-path tools/aero-qif-locator-inventory/Cargo.toml
    cargo clippy --manifest-path tools/aero-qif-locator-inventory/Cargo.toml --all-targets -- -D warnings

## QIF baseline

- Standard identity: QIF 3.0.0.
- QIF namespace: `http://qifstandards.org/xsd/qif3`.
- Streaming XML implementation: `quick-xml = 0.42.0`, pinned exactly.
- Input encoding profile: UTF-8 only; a different declared encoding is an explicit unsupported result.
- Element identity is namespace URI + local name (Clark name), not producer-specific prefixes.
- The initial profile is for Results and Plan documents only. Auxiliary document roles are typed but not yet part of this profile.

QIF 3.0 uses an `ExternalQIFReferences/ExternalQIFDocument` structure with a local `id`, a child QPId and an optional URI. The URI is inert provenance and is never dereferenced by this crate.

## Reference namespace model

The locator type deliberately separates:

- `source_element_id`: the @id of the source XML element;
- `local_reference_id`: a local QIF simple-content target reference when no xId is present;
- `parent_external_document_reference_id`: the local ExternalQIFDocument/@id that owns a child QPId or URI;
- `external_document_reference_id`: the simple-content local reference to an external document when a QIFReference uses xId;
- `external_xid`: the target object ID in that external QIF document.

For a QIFReference simple-content field:
- without xId, simple content is a local ID and maps to `local_reference_id`;
- with xId, simple content selects the local external-document reference and xId selects the object inside that external document.

These integers are not interchangeable. Numeric equality across separate QPIds or reference namespaces is not identity equality.

## What the profile inventories

- QIFDocument QPId and declared QIF version;
- Results ExternalQIFDocument IDs, child QPIds and optional URI metadata;
- MeasurementResultsSet, MeasurementResults, concrete CharacteristicMeasurement substitution members and CharacteristicItemId;
- InspectionStatus and per-characteristic Status, preserving both standard enum and Other* text alternatives;
- Plan CharacteristicItems, CharacteristicNominalId references, CharacteristicNominals and EntityExternalIds;
- DigitalModel external Entity IDs and their STEP persistent EntityId UUIDs.

The profile uses explicit substitution-group wildcard selectors only in the known QIF 3.0 hierarchy. It is a bounded trace profile, not a generic wildcard XML extractor.

## Independent coverage model

The checked-in `fixtures/hand-enumerated-locators-v1.json` is an independently authored expected locator set for the synthetic Results/Plan fixture. A test compares every actual locator tuple to this oracle, including IDs and document identity. It checks the exact set and cardinality, so both omitted and extra rows fail.

The scanner also verifies:
- exactly one Results document and every supplied Plan linked from Results;
- source-document QPId equals the externally supplied QPId;
- every external QPId resolves to one of the exact supplied Plan documents;
- every linked external document has exactly one QPId child; URI remains optional;
- MeasurementResults inspection-status choice is present once per MeasurementResults;
- every characteristic measurement has one CharacteristicItemId and one Status choice;
- every Plan characteristic item has a nominal reference;
- each XML array with an `n` attribute has exactly that many direct child elements;
- no duplicate QPIds, external-reference IDs or source locators.

The profile’s minimum/maximum rule counts are additional guards. They do not constitute a complete XSD validation.

## XML/security boundary

- Never dereferences QIF URIs, file paths, schema locations or XInclude.
- Rejects DTD/DOCTYPE and custom entities.
- Uses a streaming namespace-aware parser; it does not build a DOM.
- Fails closed on malformed XML, mismatched nesting, duplicate attributes, undeclared namespace prefixes, invalid IDs and unsupported encodings.
- Bounds source bytes, element count, depth, per-element attribute count/size, per-element text, total text, locator count and namespace bindings.
- Rejects assembly-path-qualified QIF references in this initial profile rather than erasing their meaning.
- The scanner performs no network or filesystem access on source references.

## Composition with the rest of AeroCommons

1. Run the pinned QIF 3.0 XSD and normative XSLT validators first; their independent stage reports are recorded in #6372.
2. Supply the exact raw Results/Plan bytes, roles, QPIds and profile to this scanner.
3. Preserve the inventory artifact, profile digest, source digests, scanner executable digest and report as evidence.
4. Compare the derived locators to the separate hand-enumerated oracle.
5. Pass `inventory_required_locators()` output into #6372 as the frozen `required_locators` set.
6. #6372 then checks that mapping rows cover this supplied inventory exactly; it does not regenerate or trust the adapter's own field count.

This avoids a circular arrangement in which the extractor under test creates both its mapping rows and the expected denominator.

## Benchmark targets (prospective, not achieved)

- 100% exact agreement with the independent hand-enumerated oracle on the supported synthetic trace profile.
- Zero missing, extra or duplicate locators.
- Zero QIF local-ID / external-document-ID / xId namespace conflation.
- 100% exact agreement for source QPId and linked Plan QPId resolution.
- Zero dropped Passed/Failed/NotEvaluated/Unknown measurement-status fields.
- 100% agreement between every array's declared `n` value and scanned direct children.
- 100% repeatable inventory digest for identical source bytes, profile and invocation.
- Zero DTD/custom entity acceptance and zero URI dereference.
- p50/p95 parse time, bytes/sec, peak memory, max depth and locator count reported separately.

Only claim schema-wide completeness when a separate version-pinned XSD/profile-derived locator inventory generator has itself been qualified against a separate oracle. The present oracle is synthetic and bounded.

## Claim ceiling

This crate proves a bounded source inventory against its declared profile and exact inputs. It does not parse every QIF 3.0 construct, validate QIF XSD/XSLT, establish that the profile covers the entire QIF standard, resolve all possible QIF reference types, parse STEP/AP242, prove geometry/PMI semantics, validate metrology, or authorize part acceptance, certification or airworthiness.
