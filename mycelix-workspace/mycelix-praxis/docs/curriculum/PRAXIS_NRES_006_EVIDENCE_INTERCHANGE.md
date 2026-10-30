# PRAXIS-NRES-006: Portable evidence interchange profile

**Status:** draft interchange profile and synthetic conformance corpus. Not a published standard, certified implementation, operational data-exchange agreement or authorization to share real incident/personnel data.  
**Composes:** NRES-001–005; W3C PROV-O / PROV-DM; JSON-LD 1.1; FEMA HSEEP; IFRC preparedness and EOC practice; ISO 22301; WHO's 2026 public-health EOC framework.

## 1. Research shift: from platform-owned records to portable evidence

NRES-005 defined an exercise-to-improvement loop. NRES-006 adds a portable envelope so a course, evaluator, simulation engine, public institution or humanitarian partner can exchange the evidence without adopting the same application or database.

The goal is not a replacement for their systems. It is a narrow interchange layer carrying:
- stable objective, observation, evidence, finding, corrective-action and retest references;
- provenance relationships and source artifact references;
- explicit claim status and measurement units;
- framework crosswalks with the mapping's confidence/type declared;
- privacy, disclosure and integrity metadata; and
- extension points that preserve fields a consumer does not understand.

The envelope must keep three things separate: what was asserted, what evidence supports it, and who (or which accountable role/system) is responsible for the activity. A digital signature, hash or provenance chain establishes properties of the record; none by itself establishes that the underlying claim is true or that an action was lawful.

## 2. Standards composition and boundaries

| Reference | Existing purpose | NRES-006 use | What is not claimed |
|---|---|---|---|
| W3C PROV-O / PROV-DM | Domain-agnostic model for entities, activities, agents and derivation | Use the standard PROV vocabulary for evidence lineage and responsibility links | No complete PROV implementation or inference engine is claimed |
| JSON-LD 1.1 | JSON-compatible linked-data serialization for interoperable systems | Provide stable IRIs and a local context mapping without requiring a platform-specific database | A local context/profile is not a new W3C standard |
| FEMA HSEEP | Exercise objectives, evaluation, evidence, AAR/IP and improvement planning | Conceptual crosswalk for objectives, observations, findings and corrective actions | This is not an HSEEP template or official FEMA interchange schema |
| IFRC Preparedness for Effective Response / EOC guidance | Assessment, prioritisation, work planning, action/accountability, coordination and information management | Crosswalk to preparedness and EOC assessment concepts | Praxis does not replace an IFRC or National Society system |
| ISO 22301 | Business-continuity management system and continual improvement | Crosswalk to continuity measures, performance review and improvement records | No clause-level conformity or ISO certification is implied |
| WHO Public Health EOC Framework, 2nd ed. (2026) | Legal governance, plans, information systems/data standards, infrastructure, workforce, exercises and M&E | Crosswalk to EOC evidence and capability metadata | No WHO endorsement or health data authorization is implied |
| OASIS Common Alerting Protocol (CAP) | Standardized all-hazard alerts and public warnings | Explicitly excluded from after-action evidence exchange; may be linked as a referenced alert artifact | CAP is not used as a general exercise-evidence or learner-record schema |

The important design decision is to reuse PROV's entity/activity/agent model and JSON-LD's linked-data serialization, while adding only the domain-specific NRES fields. Existing framework mappings are marked **conceptual**, not represented as official field-for-field conformance.

## 3. Envelope structure

The profile is defined by `schemas/nres/nres-evidence-envelope-v1.schema.json`. A public example lives at `examples/courses/civic-resilience-evidence-envelope-001.json`.

Required top-level sections:
1. `@context`, `@id`, `@type`: JSON-LD identity and vocabulary.
2. `schema_version`: the NRES profile version.
3. `bundle_metadata`: scenario, synthetic/operational status, evaluation split and source benchmark.
4. `source_references`: file/record references and media types; raw bytes can be integrity-checked separately.
5. `@graph`: PROV entities, activities and agents, plus NRES-specific nodes. Every relation ID must resolve inside the graph or a declared external reference set.
6. `measurements`: typed, unit-bearing measures with numerator/denominator and per-service outcomes; no compensating total.
7. `framework_crosswalks`: named framework, source element, target element, mapping type and scope note.
8. `privacy`: classification, subject policy, purpose, disclosure and prohibited inferences.
9. `integrity`: digest scope/algorithm and signature state; the public fixture is explicitly unsigned.

## 4. Provenance semantics

The profile uses PROV-O terms as follows:
- `prov:Entity`: source fixture, evidence artifact, observation, finding, corrective action or verification record.
- `prov:Activity`: exercise cycle, improvement planning, verification or export activity.
- `prov:Agent`: organization, software or role responsible for an activity. Use role-only placeholders where naming an individual is neither necessary nor authorized.
- `prov:used`, `prov:wasGeneratedBy`, `prov:wasDerivedFrom`, and `prov:wasAssociatedWith`: connect inputs, outputs, derivations and accountable agents.

No natural-person identity is required by the public profile. A source reference is not proof of source authenticity. Production deployments must define custody, authorization, retention, redaction and cryptographic signing separately.

## 5. Data minimization and fail-closed rules

The public envelope must:
- contain no natural-person identifiers or contact details;
- not carry political affiliation, ideology, loyalty, enlistment interest or personal suitability scores;
- distinguish synthetic fixture data from observed operational evidence;
- preserve missingness and unknowns as first-class states;
- reject dangling provenance references and malformed closed-action records;
- reject unsupported composite/leaderboard/suitability scores;
- never promote an unsigned fixture or software-generated record into an authenticated institutional statement; and
- preserve unknown namespaced extensions without silently interpreting them.

Real operational evidence requires explicit governance and a lawful data-sharing basis before export. This is not a consent or legal-compliance determination.

## 6. Interoperability conformance suite

The public corpus in `examples/courses/civic-resilience-evidence-conformance-001.json` covers:
- valid import/export and graph-reference closure;
- round-trip preservation of unknown namespaced extensions;
- rejection of missing or dangling PROV references;
- rejection of a closed corrective action without independent verification and a passing retest;
- rejection of disallowed personal/political/enlistment fields;
- rejection of aggregate scores; and
- explicit preservation of unsigned/demonstration status.

Report the following measures for every adapter pair:
1. **Schema conformance:** valid envelopes accepted, invalid envelopes rejected.
2. **Round-trip field preservation:** supported fields preserved and unknown namespaced extensions retained.
3. **Provenance closure:** all referenced entities, activities and agents resolve.
4. **Claim-state fidelity:** synthetic/observed/assumption/estimate/unknown labels unchanged.
5. **Measure fidelity:** units, denominators, floor thresholds and null/missingness retained exactly.
6. **Corrective-action fidelity:** status, owner role, due cycle, verification and retest linkage preserved.
7. **Privacy leakage:** prohibited fields detected; target zero for prohibited-field export.
8. **Version compatibility:** explicit behavior for older/newer minor versions and unknown extensions.
9. **Operational overhead:** bytes per evidence item, import/export latency and memory at stated scale.
10. **Auditability:** an independent verifier can reconstruct the provenance chain and identify unsigned or synthetic data.

Do not compare adapter speed unless they perform the same transformation, use the same payload, hardware and validation rules. Do not claim standards compatibility from JSON parsing alone.

## 7. Proposed acceptance targets

These are engineering targets for the prototype, not externally recognized industry thresholds:
- 100% of in-profile valid fixtures parse and preserve declared fields.
- 100% of malformed references, unsupported profile versions and invalid verified-closure claims are rejected or explicitly quarantined.
- 100% of unknown namespaced extension fields survive a lossless round trip.
- 100% of exported measures preserve units, denominators and service-floor thresholds.
- 0 natural-person or prohibited political/enlistment/suitability fields in the public fixture.
- 0 composite resilience or person-suitability scores.
- 100% of unsigned fixtures remain labeled unsigned after import/export.
- Every import/export report records adapter version, source profile version and all warnings/losses.

A later confirmatory interoperability test should include at least two independently implemented adapters and a third-party validator. A self-round-trip in one implementation is necessary but insufficient evidence of interoperability.

## 8. Execution boundary

NRES-006 remains a public synthetic development profile. It is not enabled for operational incident data, live alerting, personnel records, deployment decisions or official credentials. The course and fixtures remain inactive in the Praxis curriculum. A responsible institution must approve any real-world exchange and the relevant data governance.

## 9. Sources

- W3C, [PROV-O: The PROV Ontology](https://www.w3.org/TR/prov-o/) and [PROV-DM: The PROV Data Model](https://www.w3.org/TR/2013/REC-prov-dm-20130430/).
- W3C, [JSON-LD 1.1](https://www.w3.org/TR/json-ld/).
- FEMA, [HSEEP](https://training.fema.gov/programs/nsec/hseep/) and [Exercise Evaluation Guides / Improvement Planning](https://preptoolkit.fema.gov/web/hseep-resources).
- IFRC, [National Society Preparedness Framework](https://www.ifrc.org/document/ifrc-national-society-preparedness-framework) and [Emergency Operations Centres Guide 2026](https://www.ifrc.org/document/ifrc-emergency-operations-centres-guide-2026).
- ISO, [ISO 22301:2019](https://www.iso.org/standard/75106.html).
- WHO, [Framework for a public health emergency operations centre, second edition (2026)](https://www.who.int/publications/b/84435).
- OASIS, [Standards Archive — Common Alerting Protocol v1.2](https://www.oasis-open.org/standards/).

References inform the profile. They do not validate, certify or endorse this implementation.
