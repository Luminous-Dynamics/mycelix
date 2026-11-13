# Mycelix SupplyChain Interoperability — 2026-11-13

**Status:** design and static mapping tranche; no standards conformance or execution result  
**Tracking:** [SUPPLYCHAIN-INTEROP-001 / #6105](https://github.com/Luminous-Dynamics/mycelix/issues/6105)  
**Active code owner:** `mycelix-workspace/sdk-ts/src/integrations/supplychain/` and `mycelix-workspace/happs/supplychain`  
**Archived repository:** `Luminous-Dynamics/mycelix-supplychain` is public but archived/read-only; do not target it for new changes.

## 1. Decision

Mycelix should become a **standards-interoperable provenance and selective-disclosure layer**, not another product-passport standard.

The first conformance target is a small EPCIS 2.0.1 profile with explicit transformations to the current Mycelix checkpoint model. W3C VC 2.0 should describe attestations over those events. SCITT/RFC 9943 transparency receipts must remain a distinct evidence layer. NIST IR 8536 provides the cross-ecosystem traceability principles and mapping frame.

No standard is considered implemented merely because a type or field has a similar name.

## 2. Evidence snapshot and claim ceiling

### Public implementation surfaces inspected

1. Root `README.md` (main HEAD at the time of inspection):
   - labels Mycelix a pre-alpha research project;
   - identifies Supplychain as a pre-alpha/schema-backend cluster with no deployed UI;
   - says many clusters have unit tests but far fewer have actual multi-agent integration tests.
2. `mycelix-workspace/sdk-ts/src/integrations/supplychain/index.ts`:
   - exports checkpoint, provenance-chain, handler-profile and chain-verification types;
   - a checkpoint includes product ID, optional batch ID, location/coordinates, handler and handler DID, a bounded action enum, numeric timestamp, evidence array, epistemic claim, and optional predecessor checkpoint ID;
   - evidence records currently carry a type string, arbitrary data, numeric timestamp and a boolean `verified`;
   - chain verification reports a boolean, integrity score, weak links, recommendations and timeline.
3. `mycelix-workspace/sdk-ts/examples/10-supplychain-integration.ts`:
   - demonstrates a coffee product journey across multiple checkpoints;
   - uses illustrative identifiers, `Date.now()`, placeholder image hashes and illustrative transaction IDs; it is a developer example, not real provenance evidence.
4. `mycelix-workspace/sdk-ts/INTEGRATION_AUDIT.md`:
   - dated 2026-02-06, reports a 624-line module, nine exports and a test file;
   - this is a historical internal audit record, not a fresh test result or independent validation.
5. The standalone `Luminous-Dynamics/mycelix-supplychain` repository:
   - repository metadata reports it is archived;
   - it contains historical API documentation for a separate event-to-VC-to-DKG implementation. That code is not an active edit target.

### Immediate evidence cautions

- A boolean field such as `verified: true` is not itself a cryptographic proof or an independently qualified observation.
- A handler DID string is not, by itself, proof that the handler controlled that identity or had authority over the claimed event.
- A numeric JavaScript timestamp does not establish EPCIS `eventTime`, time-zone offset, trusted clock source or separate repository `recordTime`.
- A `previousCheckpointId` and integrity score do not automatically establish an interoperable event graph, complete lineage, or non-equivocation.
- A `blockchain` evidence label or transaction hash is not proof that the asserted physical event happened.
- Current code status, actual test status and standard conformance are three separate questions. The present document establishes none of the latter two.

## 3. Standards ownership and mapping principles

| Owner / standard | Normative role | Mycelix use | Must not be conflated with |
|---|---|---|---|
| NIST IR 8536 (final 2026-09-09) | Manufacturing traceability meta-framework: encapsulation, linkage, interoperability, query, cryptographic provenance and selective disclosure | Organize the cross-party traceability mapping and conformance questions | A product certification, API, ledger, or mandatory single data model |
| GS1 EPCIS / CBV 2.0.1 | Physical/digital object visibility event vocabulary, capture/query semantics and normative JSON-LD/OpenAPI artefacts | Exchange manufacturing events with external systems | Mycelix-native checkpoint vocabulary or identity/credential trust |
| W3C VC Data Model 2.0 / Data Integrity 1.0 | Credential data model and securing mechanism | Issue/verify actor, product, inspection or event attestations with explicit issuer/subject/validity | EPCIS event semantics, physical truth or an organization's authority policy |
| IETF RFC 9943 / SCITT | Signed-statement transparency architecture, issuer statements and verifiable transparency | Bind externally verifiable transparency evidence to signed event/claim statements | DKG claim ID, local hash chain, VC signature alone or an unfinalized SCITT API draft |
| Mycelix identity / knowledge / supplychain | Existing local identity, epistemic claim, bridge and checkpoint capabilities | Native claims, coordination and local application behaviour | An authoritative replacement for the standards above |

Standards snapshots:
- NIST IR 8536: https://csrc.nist.gov/pubs/ir/8536/final
- GS1 EPCIS/CBV artefacts: https://ref.gs1.org/standards/epcis/artefacts
- W3C VC 2.0 Recommendations: https://www.w3.org/news/2025/the-verifiable-credentials-2-0-family-of-specifications-is-now-a-w3c-recommendation/
- RFC 9943: https://www.rfc-editor.org/info/rfc9943/
- SCITT work status: https://datatracker.ietf.org/wg/scitt/documents/

## 4. Initial semantic mapping decisions

The machine-readable mapping is in [`SUPPLYCHAIN_INTEROP_MAPPING_V0_1.json`](SUPPLYCHAIN_INTEROP_MAPPING_V0_1.json). Its rows deliberately use `candidate`, `conditional`, `not_direct`, or `unknown`; none means standard conformance.

### Important non-equivalences

- Mycelix `action` `shipped` is not a complete EPCIS event. Its mapping depends on whether the business occurrence is an ObjectEvent, TransformationEvent, AggregationEvent or another event, and on business-step/disposition semantics.
- Mycelix `productId` might identify an EPC class, instance, trade item or a local product record. EPCIS `epcList` and `quantityList` cannot be chosen until this identity granularity is explicit.
- Mycelix `batchId` does not map to one generic EPCIS core field without an adopted identifier/extension profile.
- Mycelix `location` display text is not an EPCIS readPoint or bizLocation identifier. Coordinates are not a replacement for a governed location identifier.
- Mycelix `handlerDid` is not automatically a source/destination party, EPCIS business transaction, VC issuer, holder or authorized actor. These roles must be separately declared.
- Mycelix `timestamp` does not collapse EPCIS `eventTime` and `recordTime`. Occurrence time, record time, source clock, offset and uncertainty must be distinct.
- Mycelix evidence array entries have heterogeneous meanings. EPCIS sensor information is not a universal replacement for photographs, signatures, external attestations, calibration evidence or cryptographic proofs.
- Mycelix `previousCheckpointId` is a native lineage pointer; it is not a general EPCIS event-linking field.
- A VC can attest who made a claim and under which securing mechanism; it cannot by itself prove the physical occurrence.
- A SCITT transparency receipt proves a statement's inclusion/transparency under its log/protocol profile; it does not prove the statement's semantic truth or physical observation.

## 5. First supported profile (proposed, not implemented)

Begin with a deliberately small profile:

1. **ObjectEvent** for object/batch observation at an identified point, with explicit action, business step, disposition and occurrence time.
2. **TransformationEvent** for a process converting one or more input lots/components into one or more outputs.
3. **AggregationEvent** for assembly/packing/containerization relationships between identified objects.

Defer other EPCIS event classes and unsupported extensions until there is an explicit mapping and round-trip test. Unknown CBV values, unknown fields with semantic significance, unresolved units and unknown identifier schemes must return a visible unsupported/indeterminate result or be retained opaquely with an explicit no-interpretation state. They must never be silently dropped.

## 6. Benchmark design

The development-only scenario catalogue is in [`SUPPLYCHAIN_INTEROP_SCENARIOS_V0_1.json`](SUPPLYCHAIN_INTEROP_SCENARIOS_V0_1.json). It defines intended control conditions; it is not a gold answer key, a scored test set, or a sealed holdout.

### Required measurements

- **Semantic fidelity:** normalized source-to-target-to-source event equivalence, by event class and field.
- **Provenance integrity:** original artifact digest, event identity, source/issuer and transformation lineage.
- **Temporal fidelity:** eventTime, recordTime, timezone, precision, clock source and ordering ambiguity.
- **Identity and trust:** issuer/handler/resource-owner separation, key validity, revocation, role and authorization.
- **Graph integrity:** parent/child completeness, multi-input/output transformation, corrections and equivocation.
- **Privacy:** requested-claim utility, forbidden-field leakage, correlation metadata and selective-disclosure correctness.
- **Interoperability:** capture, query, export and round-trip across two independently implemented systems.
- **Operations:** throughput, p50/p95 latency, batch atomicity, replay behaviour, resource cost and integration effort.

No scalar average may conceal invalid signatures, replay acceptance, unauthorized disclosure, or loss of a critical event relation.

### Baselines

The GS1 normative artifact intake, official JSON Schema / JSON-LD / SHACL checks, and independent server benchmark are owned by [MYC-GS1-CONFORMANCE-001 #5395](https://github.com/Luminous-Dynamics/mycelix/issues/5395). Do not create a second schema validator or server benchmark in this broad mapping workstream.

The candidate selection and exact revision pins are in [SUPPLYCHAIN_INTEROP_BASELINE_REGISTER_V0_1_2026-11-13.md](SUPPLYCHAIN_INTEROP_BASELINE_REGISTER_V0_1_2026-11-13.md):
1. Official GS1 EPCIS/CBV normative artefacts at the exact versions named by the conformance clauses.
2. FasTnT EPCIS as the first independent server candidate for a bounded compatibility profile; source inspection shows capture, event-query and named-query routes, but no proven 2.0.1 conformance.
3. OpenEPCIS Community Edition as a second server implementation candidate, subject to a pinned source revision and protocol probe.
4. Mycelix native checkpoint flow and Mycelix adapter/VC/transparency profiles.
5. NIST IR 8536 reference implementation only after its revision, licence, dependency and maturity review.

OpenEPCIS Event Sentry and its document-validation service are same-family diagnostics, not independent judges. Candidate servers must pass the normative artifact, protocol, semantic round-trip and adversarial gates before a performance comparison.

## 7. Prospective gates

These are proposed test thresholds, not achieved results:

- 100% semantic preservation within the explicitly supported profile's frozen positive fixtures.
- Zero silent loss of unsupported/unknown semantically relevant fields.
- 100% rejection of mutated signatures/proofs, wrong-subject bindings, prohibited replay/duplicate IDs, broken lineage and mismatched transparency receipts in the frozen adversarial fixtures.
- Zero seeded protected-field disclosure in the deterministic privacy suite.
- Exact reproducibility of round-trip reports for frozen inputs and tool/profile versions.
- Set performance targets only after measuring a matched independent EPCIS baseline under the same payloads, hardware, batch sizes and query workload.

## 8. Runner-independent work permitted now

1. Version the standards/capability register and semantic mapping.
2. Audit source types, tests and cluster interfaces without executing repository-controlled code.
3. Author synthetic positive, negative and ambiguous fixtures, keeping gold labels and any future holdout separate.
4. Freeze the small EPCIS event profile and non-goals.
5. Define canonicalization, source digest, ID and time semantics before adapter implementation.
6. Confirm external EPCIS test server/validator licenses and independent implementation availability.
7. Revisit security claims around issuer key custody, identity proof, revocation and selective disclosure before production-facing claims.

No code, standards conformance, CI result, external endpoint comparison or real supply-chain event was executed in this tranche. The separate baseline register adds source-inspected candidate server revisions only; no server was deployed or benchmarked.
