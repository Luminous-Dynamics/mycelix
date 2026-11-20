# AeroCommons QIF external-reference trace — v1

Status: synthetic normalized trace verifier; no official CAx-IF archive is redistributed or parsed here  
Tracker: https://github.com/Luminous-Dynamics/mycelix/issues/6361  
Parent: adapter receipts #6354 / PR #6355; semantic assertions #6348 / PR #6349

## Purpose

Resolve the QIF 3.0 external-object reference path used by CAx-IF downstream STEP/QIF Test Case 8 without collapsing identifier namespaces or skipping intermediate QIF relationships.

This is a narrow normalized-graph verifier. It does not parse QIF XML, parse STEP/AP242, evaluate GD&T, validate measurement quality or accept a part.

## Identifier namespaces

The normalized trace distinguishes:

1. Results QPId — UUID identifying the QIF Results document.
2. ExternalQIFDocumentReference local id — positive local QIF id in the Results document.
3. ExternalQIFDocumentReference QPId — UUID identifying the external Plan document.
4. CharacteristicItemId simple-content value — local reference id identifying the ExternalQIFDocumentReference in Results.
5. CharacteristicItemId/@xId — positive external QIF object id selecting a CharacteristicItem in the referenced Plan.
6. CharacteristicItem/CharacteristicNominalId — local Plan id selecting that CharacteristicNominal.
7. CharacteristicNominal/EntityExternalIds/Id — local id selecting an entry in the Plan's DigitalModel external-entity table.
8. DigitalModel/Entities/Entity/EntityId — STEP persistent UUID for the external STEP entity.
9. STEP source artifact — exact AP242 source whose UUID namespace is being referenced.

The resolution chain is:

    Results.MeasurementResultsSet
      -> CharacteristicMeasurement
      -> CharacteristicItemId (local ExternalQIFDocumentReference id + @xId)
      -> Results.ExternalQIFDocumentReference
      -> external Plan QPId + exact Plan bytes
      -> Plan CharacteristicItem (xId)
      -> CharacteristicNominalId (local Plan id)
      -> Plan CharacteristicNominal
      -> EntityExternalIds/Id
      -> Product/Part/DefinitionExternal/DigitalModel/Entities/Entity
      -> Entity/EntityId (STEP UUID)
      -> exact STEP/AP242 source artifact

The xId is not the STEP UUID. It selects a QIF object inside the external Plan. The Plan's CharacteristicItem then references a CharacteristicNominal. The nominal's EntityExternalIds values reference the QIF DigitalModel/Entities/Entity/@id entries; those entries carry the STEP UUID in Entity/EntityId.

This intermediate mapping is important. Comparing xId directly to a STEP UUID, or assuming a CharacteristicItem directly contains a STEP UUID, skips a real namespace boundary and can generate false trace joins.

QPIds are UUIDs and are compared case-insensitively. QIF local ids are positive integers scoped to their own QIF document. The same xId or EntityExternalId integer in two different Plan documents is not the same object.

## Implementation scope

The Rust crate consumes a normalized model containing:
- Results QPId, exact Results artifact, local external-document references and measurement records;
- each measurement's MeasurementResultsSet id, status, CharacteristicItemId simple-content local reference id and xId;
- exact Plan QPId and content artifact;
- Plan CharacteristicItems and their required CharacteristicNominalId;
- Plan CharacteristicNominals and their EntityExternalIds list;
- the subset of Product/Part/DefinitionExternal/DigitalModel/Entities mappings needed by this trace;
- exact STEP source artifact and UUID/entity-kind inventory.

It verifies:
- QPId syntax, canonical case-insensitive equality and uniqueness;
- local external-reference IDs, measurement IDs, Plan object IDs and DigitalModel Entity IDs are positive and unique within their document scope;
- external reference QPId and exact Plan artifact agree;
- xId resolves to a CharacteristicItem in the exact selected Plan;
- CharacteristicItem's CharacteristicNominalId resolves in the same Plan;
- every nominal's EntityExternalIds value resolves through that Plan's DigitalModel entity mapping;
- each mapped UUID resolves in the exact pinned STEP document with the declared entity kind;
- Plan STEP-source artifact metadata exactly matches the pinned STEP document;
- every measurement is traced, including failed, not-evaluated and unknown outcomes;
- all Results, Plan and STEP source bytes match their declared content artifacts;
- normalized trace output is deterministic under measurement ordering changes.

A nominal may refer to multiple EntityExternalIds. The resolver preserves all of them instead of assuming every characteristic maps to exactly one STEP entity. This matters for compound or multi-entity characteristics and avoids silently truncating a trace.

The normalized step_entity_kind is an adapter-provided classification, not a claim that this crate parsed the STEP EXPRESS instance or independently established semantic PMI correctness.

## What CAx-IF Test 7 and Test 8 actually exercise

CAx-IF Cross-Domain Exchange for Downstream Uses v1.0 is scoped to STEP AP242 Edition 4 and QIF 3.0, with metrology downstream exchange as its initial focus.

- Test 7 duplicates the STEP mapping table and relevant FeatureNominal/CharacteristicNominal content into Results.
- Test 8 removes that duplicated Results content and uses external xIds on CharacteristicItems attached to CharacteristicMeasurements, across three MeasurementResultsSets. The Plan content is reused from Test 7.

The practice reports successful forward and backward trace examples, including failed measurements. It explicitly leaves QIF 3.0 schema quality compliance outside its own scope, referring users to QIF's schema and XSLT constraint checks instead. That distinction is important: this crate is a normalized graph verifier, not a QIF 3.0 conformance validator.

Official references:
- CAx-IF Cross-Domain Exchange for Downstream Uses v1.0: https://www.mbx-if.org/home/wp-content/uploads/2026/02/rec_prac_cross-domain_exchange_v10.pdf
- CAx-IF Recommended Practices and Test Cases 7/8: https://www.mbx-if.org/home/cax/recpractices/
- QIF 3.0 QIFReferenceType: https://qualityinformationframework.github.io/qif3-browser/Primitives_xsd_Complex_Type_QIFReferenceType.html
- QIF 3.0 ExternalQIFDocumentReferenceType: https://qualityinformationframework.github.io/qif3-browser/QIFDocument_xsd_Complex_Type_ExternalQIFDocumentReferenceType.html
- QIF 3.0 CharacteristicItemBaseType: https://qualityinformationframework.github.io/qif3-browser/Characteristics_xsd_Complex_Type_CharacteristicItemBaseType.html
- CAx-IF mapping of STEP UUIDs into QIF Product/Part/DefinitionExternal/DigitalModel/Entities: the same v1.0 recommended practice, section 5.4.1.

## Rights boundary

The official Test 7/8 ZIP archives are linked from CAx-IF's public recommendations, but public download does not by itself establish a redistribution license. This repository currently contains only authored synthetic QIF/STEP-like payload strings and a normalized synthetic graph. Do not replace them with official archive payloads until acquisition and redistribution rights are established.

## Frozen synthetic scenario

The checked-in case has one Results document, two external Plans and one STEP document. The two Plans deliberately reuse xId 301, and both contain a local DigitalModel Entity id 9991, but they resolve to different Plan-local nominal/STEP identity chains.

The four measurements span two result sets and include Passed, Failed, NotEvaluated and Unknown states. One characteristic nominal deliberately maps to two STEP UUIDs via separate EntityExternalIds. The failed measurement remains attached to its intended CharacteristicItem, nominal and STEP entity.

This synthetic corpus is not a real CAx-IF Test 8 package and does not prove QIF parser interoperability. It qualifies the identifier and mapping boundaries before rights-cleared reference data is introduced.

## Prospective benchmark targets

These are targets, not achieved results:
- 100% agreement with an independently hand-enumerated synthetic trace oracle;
- zero QIF local-id, Plan xId, DigitalModel EntityExternalId and STEP UUID namespace conflation;
- zero successful resolution with missing, stale, ambiguous or wrong-type references;
- 100% trace coverage for every declared measurement, including failed measurements;
- 100% preservation of multi-entity nominal mappings;
- deterministic normalized output under measurement/input ordering permutations;
- Test 7/Test 8 normalized trace equivalence only after official files and rights-cleared oracle data are available.

Report trace resolution coverage, wrong-join rate, unresolved reference rate by cause, stale-document rate, ID collision handling, byte-integrity failures and p50/p95 resolution latency separately.

## Claim ceiling

A successful result establishes only that the declared normalized QIF reference graph is internally resolved and that the exact supplied Results/Plan/STEP payload bytes match their declared content artifacts. It does not prove QIF XML schema conformance, AP242 conformance, that an adapter extracted the graph faithfully from source XML, semantic correctness of source PMI, metrology validity, manufacturing capability, product acceptance, certification or airworthiness.
