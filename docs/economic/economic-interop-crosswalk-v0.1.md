# Economic OS standards interoperability crosswalk v0.1

**Status:** research crosswalk and synthetic adversarial benchmark; no adapter implementation or standards certification is claimed.

## Design position: standards are semantic authorities for their own domains

Do not create a replacement mega-ontology. Valueflows, W3C PROV, GS1 EPCIS, ISO 20022 and Peppol solve different problems. The Economic OS should preserve their native identifiers and meaning, then bind the resulting evidence to FIN-EFFECT operation/effect identities and explicit claim profiles.

The architectural change is to represent an economic process as a typed graph of plans, owner effects, observations, evidence provenance and claims. A graph may connect these records, but connection is not proof: a plan does not become an event; an event does not become an authorization; a message does not become settlement; a visibility scan does not become acceptance.

## What each standard contributes

### Valueflows

Valueflows 1.0.0 is a stable major vocabulary for resource-flow economic networks, based on REA/IPO patterns. It supports economic activity with or without money, with independent agents and resource flows. Its own documentation distinguishes knowledge/planning/observation layers and describes Intent/Commitment/Event connections. It is a vocabulary, not a transport protocol, global authority registry or payment settlement engine.

Official sources: [Valueflows home](https://www.valueflo.ws/), [1.0 status](https://www.valueflo.ws/introduction/status/), [core concepts](https://www.valueflo.ws/introduction/core/), [FAQ](https://www.valueflo.ws/introduction/faq/).

**Mycelix mapping:** keep Intent/Offer/Request/Commitment as plan/obligation nodes; EconomicEvent as an observation/effect candidate; EconomicResource as resource identity and quantity context; Claim as a scoped assertion. Add FIN-EFFECT IDs as external join references, never as replacements for Valueflows IRIs or resource/event IDs.

### W3C PROV-DM

PROV-DM models provenance through Entities, Activities and Agents, including derivation, usage, generation, association and attribution. It can explain how an evidence artifact was produced and by whom/what. It does not establish the truth of the artifact, source completeness, legal authority or economic finality.

Official source: [W3C PROV-DM Recommendation](https://www.w3.org/TR/prov-dm/).

**Mycelix mapping:** each imported document, event, signed receipt or normalized message can be a PROV Entity; capture, transform, verify, reconcile and export steps can be Activities; actors and software can be Agents. Keep credential/mandate checks separate from PROV attribution. Bind source bytes, transformations, code/profile versions and timestamps.

### GS1 EPCIS 2.0

EPCIS provides a shared visibility-event data model and capture/query interfaces for events concerning physical or digital objects across enterprises. It is useful for logistics, manufacturing, custody/visibility and service/resource fulfilment. A visibility event does not by itself establish legal title, a payment, beneficiary acceptance or satisfaction of an obligation.

Official source: [GS1 EPCIS 2.0.0](https://ref.gs1.org/standards/epcis/2.0.0/).

**Mycelix mapping:** preserve eventID, eventTime, recordTime, business step, disposition, read point, business location, source and object/resource identifiers. Model out-of-order arrival using event time and record time; never substitute arrival order. Distinguish dispatch, shipping, receiving, delivery, quality inspection and beneficiary acceptance.

### ISO 20022

ISO 20022 is a versioned catalogue of financial message definitions. At the 24 November 2026 snapshot, the official catalogue displayed, among others, `pain.001.001.13` customer credit transfer initiation, `pacs.002.001.16` FI-to-FI payment status, `pacs.004.001.15` payment return, `pacs.007.001.14` payment reversal, `pacs.008.001.14` credit transfer, `pacs.028.001.07` status request and `camt.052/.053/.054` account reporting messages. The precise version and usage guide depend on the scheme and message set.

Official source: [ISO 20022 Message Definitions catalogue](https://www.iso20022.org/iso-20022-message-definitions).

**Mycelix mapping:** preserve message ID/version, scheme, business application header, sender/receiver, instruction/end-to-end/transaction IDs, account/agent, status code and original references. Initiation, status, return, reversal and account reporting remain different evidence classes. Never infer finality from a generic accepted/processed status without the rail's qualified finality profile and independent reconciliation.

### Peppol BIS Billing / EN 16931

Peppol BIS Billing is a Core Invoice Usage Specification of EN 16931. The validation model is staged: syntax, EN 16931, general CIUS and country-qualified rules. The 3.0.21 release notes state publication on 20 May 2026 and mandatory use from 17 August 2026. Profile applicability must still be evaluated for the actual country, actor, transaction and effective date.

Official sources: [Peppol BIS Billing 3.0 2025-Q4](https://docs.peppol.eu/poacc/billing/3.0/2025-Q4/bis/), [release notes](https://docs.peppol.eu/poacc/billing/3.0/upcoming/release-notes/).

**Mycelix mapping:** store the exact customization/profile IDs, BIS and validation artifact versions, validation-stage results and country pack. Invoice validity is evidence about document conformance, not evidence that goods were received, the supplier is genuine, a payable was approved or the bank settled.

## Cross-standard effect graph

| Semantic plane | Native objects | Mycelix treatment | Forbidden promotion |
|---|---|---|---|
| Plan / obligation | Valueflows Intent, Offer, Request, Commitment | Stable obligation + immutable intent profile | Plan != occurrence |
| Resource / identity | Valueflows EconomicResource; EPCIS object/asset IDs | Namespace-qualified resource identity and exact unit profile | Resource ID != ownership or availability |
| Observed event | Valueflows EconomicEvent; EPCIS visibility event | Owner-specific effect candidate and source evidence | Observation != authorization or finality |
| Provenance | PROV Entity, Activity, Agent | Evidence and transformation lineage | Attribution != mandate or truth |
| Financial message | ISO 20022 pain/pacs/camt | Message-specific evidence assertion and profile | Instruction/status != settlement |
| Invoice conformance | Peppol Invoice/CreditNote + EN 16931/CIUS | Document validity claim with versioned profile | Valid invoice != delivery/payment |
| Composite claim | FIN-EFFECT profile-bound projection | Explicit required-effect set, evidence frontier, scoped result | Scoped support != universal completion |

The machine-readable crosswalk lives in `economic-interop-crosswalk.v0.1.json`; the adversarial fixtures live in `economic-interop-adversarial-corpus.v0.1.json`.

## Benchmark programme

The 24-case corpus is synthetic and public. It is a development conformance set, not a hidden benchmark or field performance result. Create a separately authored, rights-cleared holdout before making superiority claims.

Compare matched pipelines:

1. Native standard schema/profile validation only.
2. Native mappings across standards without an effect/authority layer.
3. Mappings plus typed owner/effect claims and provenance binding.
4. Full Mycelix profile-bound projection plus bitemporal policy and completeness frontiers.
5. Blinded domain-expert adjudication on a stratified subset.

Give every candidate the same source bytes, permitted contextual data, runtime/tool versions, resource limits and time budget. Candidate systems must not receive evaluator-only expected dispositions. LLM judges may assist clarity review but cannot be the primary oracle for event truth, authority, amount, status or legal sufficiency.

Hard safety targets (not results): zero silent required-field loss; zero plan-to-observation promotions; zero false settlement, delivery, title or acceptance claims; zero duplicate economic effects from repeated observations; 100% deterministic replay for frozen evidence/temporal frontiers; unsupported or lossy mappings explicitly held.

Measure schema conformance by stage, semantic round-trip preservation, event/plan confusion, provenance completeness, event-time/record-time correctness, duplicate reconciliation, false-finality rate, evidence-read amplification, mapping latency p50/p95/p99 and human review minutes. Report safety and liveness separately.

## Industry comparison and competitive strategy

- **Valueflows** is already a direct comparator for resource-flow economic modelling. Do not claim to invent resource-flow accounting; add bounded authority/evidence, owner-specific effect identities and profile-scoped proof of particular claims.
- **W3C PROV** already supplies a general provenance model. Do not create a competing provenance ontology; extend/annotate it with Mycelix's assertion scope, evidence completeness and FIN-EFFECT bindings.
- **GS1 EPCIS** already handles physical visibility. Do not replace its capture/query semantics; connect observations to resource/commitment effects without claiming title or beneficiary acceptance from a scan.
- **ISO 20022 and Peppol** are mature financial-document/message standards. Aim for current, staged, scheme/jurisdiction-qualified adapters and strong semantic preservation, not to replace their message models.
- **BIS Project Agorá** is an advanced institutional wholesale-settlement comparator. Its May 2026 report describes a prototype with seven central banks and more than 40 regulated institutions; July 2026 real-value testing involved 28 financial institutions/central banks, around CHF 800,000 and 17 scenarios. This is not an apples-to-apples benchmark for community-resource coordination. Mycelix should not claim wholesale settlement superiority.

The defensible differentiation is a compositional assurance layer spanning money and non-money resources, linking standards-native events to stable obligations and scoped claims while retaining each domain's authority, version, source and finality semantics.

## Sources and snapshot discipline

All profile and message versions must be rechecked against official catalogues at adapter release time. The references above are the research snapshot as of 24 November 2026; a source snapshot manifest should pin exact bytes/digests before code uses any rule as normative.

Additional comparator: [BIS Project Agorá report, 27 May 2026](https://www.bis.org/publications/project-agora-shared-programmable-platform-wholesale-cross-border-payments).

## Claim ceiling

This is a semantic crosswalk and synthetic qualification programme. It is not standards certification, legal compliance, a payment-system membership, proof of title, proof of physical delivery, accounting assurance, external settlement finality or global uniqueness.
