# AEROCOMMONS-QIF-LOCATOR-INVENTORY-001 — independent QIF 3.0 trace locator inventory

Issue: https://github.com/Luminous-Dynamics/mycelix/issues/6391  
Crate: `tools/aero-qif-locator-inventory`  
Profile schema: `docs/aerocommons/AERO_QIF_TRACE_LOCATOR_PROFILE_V1.schema.json`  
Inventory schema: `docs/aerocommons/AERO_QIF_TRACE_LOCATOR_INVENTORY_V1.schema.json`

## Why this work is distinct

The ingestion envelope in #6372 already checks that each mapping row covers one locator in its frozen `required_locators` inventory, with exact manifest bytes and digest. That protects against missing mapping rows **relative to the supplied set**. It does not establish where the expected set came from.

This work makes the denominator independent. It derives a trace locator set from the raw QIF source XML and a separately versioned profile. It does not consult adapter mappings or normalized outputs. A hand-enumerated expected-locator fixture is separately checked in and compared to the derived list.

The new guarantee is:

`Raw QIF bytes + pinned profile → independent locator inventory → compare to independent oracle → freeze expected locator set → score adapter mapping completeness`

This is stronger than a self-reported extraction count, but remains limited to the explicitly supported profile.

## Current QIF 3.0 structure and semantics

The source profile is pinned to namespace `http://qifstandards.org/xsd/qif3` and versionQIF `3.0.0`.

Relevant QIF 3.0 schema facts:
- `QIFDocument` has one required QPId; `ExternalQIFReferences` is optional at the standard level and has a declared array count when present.
- Each `ExternalQIFDocument` has a required local id and child QPId, plus an optional child URI. The current trace profile deliberately requires at least one linked Plan document, although QIF permits documents without external references.
- `MeasurementResults` has a required InspectionStatus choice; each concrete CharacteristicMeasurement has a required Status choice. The scanner enforces exactly one enum-or-Other status field per measurement/result in this profile.
- A `QIFReferenceType` can reference an object in the same file or in an external file. With xId, two QIF identifiers participate: the local external-document reference ID in simple content and the external object ID in xId.
- `CharacteristicMeasurement` is a substitution-group head; concrete measurement element names occur in its place.
- Measurement and inspection status types are choices between enum and `Other...` textual alternatives.
- `CharacteristicNominalId` is a QIFReferenceFullType and may carry assembly-path qualifiers. The initial profile rejects those qualifiers instead of discarding them.

The code keeps these numeric namespaces in separate fields:
`source_element_id`, `local_reference_id`, `parent_external_document_reference_id`, `external_document_reference_id`, and `external_xid`.

Sources:
- QIF 3.0 schema browser root / QIFDocument: https://qualityinformationframework.github.io/qif3-browser/
- QIFReferenceType: https://qualityinformationframework.github.io/qif3-browser/Primitives_xsd_Complex_Type_QIFReferenceType.html
- CharacteristicItemBaseType / CharacteristicNominalId: https://qualityinformationframework.github.io/qif3-browser/Characteristics_xsd_Complex_Type_CharacteristicItemBaseType.html
- CharacteristicNominalBaseType / EntityExternalIds: https://qualityinformationframework.github.io/qif3-browser/Characteristics_xsd_Complex_Type_CharacteristicNominalBaseType.html
- CharacteristicStatusType: https://qualityinformationframework.github.io/qif3-browser/Characteristics_xsd_Complex_Type_CharacteristicStatusType.html
- quick-xml 0.42.0 NsReader: https://docs.rs/quick-xml/0.42.0/quick_xml/reader/struct.NsReader.html

## Command-line runner

The crate exposes a library and a small executable. Example invocation inside a disposable offline worker:

```text
qif-locator-inventory \
  --inventory-id campaign-2026-001 \
  --profile profiles/aero-qif-test7-8-trace-v1.json \
  --results-qpid 10000000-0000-4000-8000-000000000001 \
  --results staged/Results.qif \
  --plan 20000000-0000-4000-8000-000000000002=staged/Plan.qif \
  --inventory-out out/inventory.json \
  --required-locators-out out/required_locators.json \
  --report-out out/report.json
```

The CLI validates the profile before reading staged files, applies the profile's source-byte cap, rejects output/input path aliasing and never dereferences QIF URIs. `required_locators.json` is the exact `SourceFieldLocatorV1[]` input expected by #6372. The executable itself, arguments, runtime and outputs must be bound by an independent adapter-run receipt; the report identifies the package and parser version but is not a signed attestation.

## Parser contract

- Exact UTF-8/XML 1.0 profile. Non-UTF-8 bytes and unsupported XML declaration encodings are explicit errors; no transcoding guess.
- Namespace resolution by URI + local name. Prefix changes must not change locator identity.
- Streaming `NsReader`; no DOM and no external resolution.
- Reject DTD/DOCTYPE and custom entities, malformed/mismatched tags, unbound namespace prefixes and duplicate/malformed attributes.
- Predefined and numeric character references are handled without enabling DTD-based custom entities.
- Hard upper bounds on source bytes per document, aggregate Results/Plan bytes, number of documents, elements, depth, attributes per element, bytes per attribute, text per element, total text, locators and namespace bindings.
- For every XML element with an `n` attribute, the scanner checks that the actual direct-child element count equals `n`.
- The source QPId must equal the supplied QPId. External Plan QPIds must point to the exact set of supplied Plan documents.
- Source URI is captured as inert metadata and never fetched.
- Unsupported assembly-path reference fields fail explicitly.

## Profile coverage

The initial profile includes:
- Results QPId, ExternalQIFDocument id/QPId/optional URI;
- Results MeasurementResultsSet, MeasurementResults, concrete CharacteristicMeasurement substitutions, CharacteristicItemId;
- MeasurementResults InspectionStatus and per-characteristic Status enum/Other* branch;
- Plan QPId, CharacteristicItems, CharacteristicNominalId, CharacteristicNominals, EntityExternalIds/Id;
- Plan DigitalModel Entities and STEP EntityId persistent UUID.

The current profile expects one Results document and one or more Plan documents referenced from Results. It verifies one status choice per MeasurementResults, one status and one item reference per characteristic measurement, one nominal reference per characteristic item and exact array `n` counts.

This is a bounded Test 7/8 trace profile. It is not a QIF 3.0 schema-wide field inventory. The wildcard path components are used only to recognize specific substitution-group positions; the actual matched expanded names are preserved in every locator.

## Independent locator oracle

`fixtures/hand-enumerated-locators-v1.json` contains 25 manually enumerated locators for the current synthetic Results and Plan inputs. It records the source QPId, full stable XML path and every separate ID namespace field.

The test compares the exact set of scanner locators with the oracle, checking equal set size and exact equality. It therefore detects both omitted and unexpected locators. The oracle itself is not generated by the scanner.

The corpus intentionally includes:
- three characteristic measurements and their PASS, FAIL and OtherCharacteristicStatus values;
- both an external CharacteristicItemId reference with xId and local Plan references without xId;
- an ExternalQIFDocument URI and the same relationship without URI;
- namespace-prefix substitution;
- two separately linked Plan documents that intentionally reuse a numeric QIF ID;
- malformed XML, DTD/custom entity, unbound prefix, source QPId mismatch, unknown Plan QPId, array count mismatch, oversized attributes, unsupported encoding and assembly-path rejection.

All examples are authored synthetic content. They are not official CAx-IF Test 7/8 source packages.

## Benchmark targets (not achieved)

| Measure | Target |
|---|---|
| Synthetic hand-oracle agreement | 100% exact locator-set equality |
| Missing/extra/duplicate locators | 0 |
| Local/external/xId namespace conflation | 0 |
| Wrong/missing source QPId or Plan join | 0 false passes |
| Measurement and inspection status omission | 0 |
| Array `n` mismatch | 100% detection |
| DTD/custom entity and network | 0 acceptance / 0 dereference |
| Repeatable profile/input inventory digest | 100% |
| Performance | p50/p95, bytes/sec, peak memory and max depth published separately |

For cross-tool benchmarking, compare NIST STEP File Analyzer and Viewer 5.41, QIF community XSD/XSLT validators and CAx-IF paired Test 7/8 reference practices as separate evidence sources—not as interchangeable tools. NIST SFA analyses AP242 structure and semantic/graphic PMI; QIF XSD/XSLT checks different validity layers; this crate fills the source-locator inventory gap and should not replace either.

Official ecosystem references:
- NIST STEP File Analyzer and Viewer: https://www.nist.gov/services-resources/software/step-file-analyzer-and-viewer
- QIF community: https://github.com/QualityInformationFramework/qif-community
- QIF validation tools: https://github.com/QualityInformationFramework/qif-validation-tools
- CAx-IF recommended practices / Test 7 and Test 8: https://www.mbx-if.org/home/cax/recpractices/

## Relationship to other AeroCommons work

- #6371 QIF normalized external-reference/xId-to-STEP trace resolver.
- #6372 ingestion envelope and one-to-one source mapping inventory check.
- #6388 adjudicated QIF/AP242 multi-tool differential benchmark.
- #6391 this independent raw-source locator inventory.

No QIF XSD/XSLT conformance, AP242 PMI correctness, metrology validity, part acceptance, certification or airworthiness claim is made by this crate.
