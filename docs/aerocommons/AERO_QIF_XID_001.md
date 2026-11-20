# AeroCommons QIF external-reference trace — v1

Status: synthetic normalized trace verifier; no official CAx-IF archive is redistributed or parsed here  
Tracker: https://github.com/Luminous-Dynamics/mycelix/issues/6361  
Parent: adapter receipts #6354 / PR #6355; semantic assertions #6348 / PR #6349

## Purpose

Resolve the QIF 3.0 external-object reference path used by CAx-IF downstream STEP/QIF Test Case 8 without collapsing identifier namespaces.

This is a narrow normalized-graph verifier. It does not parse QIF XML, parse STEP/AP242, evaluate GD&T, validate measurement quality or accept a part.

## Identifier namespaces are not interchangeable

The normalized trace intentionally distinguishes:

1. Results QPId — identity of the QIF Results document.
2. ExternalQIFDocumentReference local id — a local QIF id in the Results document. A CharacteristicItemId reference points to this external-document reference.
3. Plan QPId — identity of the referenced external QIF Plan document.
4. xId — the QIF object id inside that external Plan, resolved only after the Plan identity and exact Plan bytes are pinned.
5. STEP persistent UUID — the STEP entity identity reached through the Plan external-ID mapping.

The critical resolution path is:

    Results CharacteristicMeasurement
      -> CharacteristicItemId simple-content local QIF id
      -> Results.ExternalQIFDocumentReference local id
      -> referenced Plan QPId + exact Plan artifact digest
      -> CharacteristicItemId xId (object id in that Plan)
      -> Plan CharacteristicItem
      -> Plan EntityExternalId mapping
      -> STEP persistent UUID
      -> exact STEP/AP242 source artifact

Do not compare xId directly with a STEP UUID. The same integer xId can appear in two distinct Plan documents and must resolve against the exact Plan selected by the local external-document reference. Both the Plan QPId and Plan content digest are required.

QIF 3.0 QIFReferenceType describes a local or external object reference; its optional xId identifies an object in the external QIF file. The external document reference has its own local id and carries the external QPId and optional URI. CAx-IF Test Case 8 uses the abbreviated Results form with xIds to point into the Plan instead of duplicating STEP mappings and nominal structures from Test Case 7.

## Implementation scope

The Rust crate consumes a normalized model with:
- Results QPId and content-addressed Results artifact;
- local external-document references, Plan QPIds and exact Plan content artifacts;
- measurement records with status and the pair of external-document-reference id and xId;
- Plan objects with type, characteristic identity and STEP UUID mapping;
- exact STEP source artifact and UUID/entity-kind inventory.

It verifies:
- QPId/STEP UUID syntax and uniqueness in their own namespaces;
- local external-reference IDs unique within Results;
- Plan QPId and Plan artifact exact match;
- no duplicate Plan-local xIds within one Plan;
- each xId resolves to the expected CharacteristicItem kind;
- each Plan STEP UUID resolves in the exact referenced STEP document;
- Plan STEP source artifact metadata exactly matches the pinned STEP document;
- all measurements resolve, including failed, not-evaluated and unknown states;
- exact bytes for Results, every Plan and every STEP source;
- deterministic trace ordering under measurement/input permutation.

The normalized model's characteristic_id and step_entity_kind are explicit adapter assertions, not claims that this crate parsed or verified the official XML schema or STEP EXPRESS entities.

## CAx-IF test context

The Cross-Domain Exchange for Downstream Uses Recommended Practice v1.0 describes two paired test approaches:
- Test 7 duplicates STEP UUID mapping and nominal information in QIF Results.
- Test 8 abbreviates Results and uses xId references into the Plan.

The practice is a STEP AP242 Edition 4 / QIF 3.0 downstream metrology use case. It is not a complete manufacturing-process-plan exchange specification. Do not claim that this narrow test proves all manufacturing execution or process semantics.

Official references:
- CAx-IF Cross-Domain Exchange for Downstream Uses v1.0: https://www.mbx-if.org/home/wp-content/uploads/2026/02/rec_prac_cross-domain_exchange_v10.pdf
- CAx-IF Recommended Practices and Test Cases 7/8: https://www.mbx-if.org/home/cax/recpractices/
- QIF 3.0 QIFReferenceType: https://qualityinformationframework.github.io/qif3-browser/Primitives_xsd_Complex_Type_QIFReferenceType.html
- QIF 3.0 ExternalQIFDocumentReferenceType: https://qualityinformationframework.github.io/qif3-browser/QIFDocument_xsd_Complex_Type_ExternalQIFDocumentReferenceType.html
- QIF 3.0 CharacteristicMeasurement: https://qualityinformationframework.github.io/qif3-browser/Characteristics_xsd_Element_CharacteristicMeasurement.html

## Rights boundary

Official Test 7/8 ZIP archives are linked from CAx-IF's public recommendations, but a public download link is not by itself a redistribution license. This repository currently contains only authored synthetic QIF/STEP-like payload strings and a normalized synthetic graph. Do not replace them with real archive payloads until acquisition and redistribution rights are established.

## Frozen synthetic scenario

The checked-in case has one Results document, two external Plans and one STEP document. The two Plans deliberately reuse external xId 301, but resolve it to different STEP UUIDs. Three measurements cover Passed, Failed and NotEvaluated statuses. The failed measurement remains attached to its intended characteristic and STEP entity.

The scenario is not a real CAx-IF Test 8 file and does not prove QIF parser interoperability. Its purpose is to qualify namespace and join rules before introducing licensed reference data.

## Prospective benchmark targets

These are targets, not achieved results:
- 100% agreement with an independently hand-enumerated synthetic oracle;
- zero QIF local-id, Plan xId and STEP UUID namespace conflation;
- zero successful resolution with missing, stale, ambiguous or wrong-type references;
- 100% trace coverage for all declared measurements, including failed measurements;
- deterministic normalized output under input ordering permutations;
- Test 7/Test 8 normalized trace equivalence only after official files and rights-cleared oracle data are available.

Report trace resolution coverage, wrong-join rate, unresolved reference rate by cause, stale-document rate, ID collision handling, byte-integrity failures and p50/p95 resolution latency separately.

## Claim ceiling

A successful result establishes only that the declared normalized reference graph is internally resolved and the exact supplied Results/Plan/STEP payload bytes match their declared content artifacts. It does not prove QIF XML conformance, AP242 conformance, that the normalized graph was faithfully extracted from source XML, semantic correctness of the source PMI, measurement validity, manufacturing capability, product acceptance, certification or airworthiness.
