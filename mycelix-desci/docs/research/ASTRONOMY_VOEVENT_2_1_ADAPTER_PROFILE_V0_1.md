# Astronomy Observation Profile — IVOA VOEvent 2.1 v0.1

**Parent programme:** [Mycelix SCI-OBS-001](https://github.com/Luminous-Dynamics/mycelix/issues/6141)  
**Related work:** [Symthaea SCI-OBS-SYM-001](https://github.com/Luminous-Dynamics/symthaea/issues/12075), [Symtropy CIVDYN observation integrity #4819](https://github.com/Luminous-Dynamics/symtropy/issues/4819), [Symtropy contingent sensing #4832](https://github.com/Luminous-Dynamics/symtropy/issues/4832)  
**Status:** Adapter profile and replay contract. Not an implemented parser or live broker integration.  
**Normative astronomy source:** IVOA Recommendation, VOEvent 2.1, 29 April 2026.

## 1. Purpose

Define how Mycelix DeSci can preserve and qualify source-native VOEvent 2.1 packets as scientific observation evidence without replacing the astronomy community's event protocol, broker ecosystem, catalogues or observatory operations.

The adapter is intentionally read-only in v0.1:

    source VOEvent packet bytes
            ↓
    source-specific acquisition receipt
            ↓
    byte digest + source/native identity + rights
            ↓
    strict VOEvent 2.1 validation and semantic projection
            ↓
    Mycelix DeSci evidence artifact / event references
            ↓
    Symthaea candidate triage and competing hypotheses
            ↓
    follow-up proposal (not instrument command)
            ↓
    later follow-up packet(s) and independent evaluation

The exact original packet remains the source object. The Mycelix projection is a derived representation and must always link back to the immutable source bytes and parser/profile version.

## 2. Why VOEvent 2.1 is the correct first astronomy seam

VOEvent is a structured packet format for reporting transient celestial events and supporting timely follow-up. Its 2.1 Recommendation explicitly permits packets to carry:
- who, what, where, when and how information;
- an initial hypothesis about the physical cause (the why);
- references to prior or related packets;
- follow-up observations and lifecycle context.

The Recommendation distinguishes packet roles:
- observation: reports an observation of the universe (the protocol default if role is absent);
- prediction: reports an event expected to occur;
- utility: reports observing-system information rather than an astrophysical event;
- test: test traffic, not an actual astronomical event.

The adapter must preserve both the raw role field and its effective interpretation. Absence of a role must not be rewritten as an explicitly supplied observation.

VOEvent 2.1 does not define the transport layer, broker implementation, publication rights or authentication policy. A well-formed packet is not, by itself, proof of the physical event, the publisher's identity, the truth of the initial hypothesis or the authority to request follow-up observations.

## 3. Identity and immutable source preservation

For every admitted packet, retain:
- exact source-native ivorn (publisher identifier + publisher-local event identifier);
- exact packet bytes and a cryptographic digest of the bytes actually received;
- byte length and media type;
- acquisition URI/endpoint or broker receipt reference, where available;
- source publisher identity as asserted by the packet;
- authenticated source identity and key-verification result as a separate field, where available;
- retrieval/receipt time and acquisition method;
- VOEvent version and parser/profile version;
- source corrections, citations and packet references.

Do not manufacture a global event identity by stripping the publisher component from ivorn. Do not merge two packets merely because their coordinates or timestamps are similar. Cross-matching is a derived, versioned analysis that records its candidate set, method, thresholds and uncertainty.

A mirrored, rewrapped, relayed or broker-enriched copy is not a new independent astronomical observation. Preserve broker lineage and the exact upstream packet reference.

## 4. Semantic mapping

| VOEvent content | Mycelix / downstream treatment | Claim boundary |
|---|---|---|
| ivorn | Source-native observation identity | Stable only within the publisher namespace |
| role | Preserve raw and effective role | Prediction, utility and test do not enter the observed-event population |
| Who / author metadata | Source attribution assertion | Not cryptographic authentication by itself |
| What / parameters and UCDs | Typed source-native event properties | Preserve identifiers, units, values and null/missing state |
| WhereWhen / STC coordinates | Domain-profile spatial/time representation | Preserve frame, reference position, time scale and uncertainty; no guessed UTC/ICRS conversion |
| How / instrument and observatory | Instrument/source context | Presence does not prove calibration validity |
| Why / initial scientific assessment | Candidate hypothesis / source assessment | Never promoted automatically into a verified scientific claim |
| References | Typed packet lineage and citation edges | Reference type and source packet identity must survive |
| Follow-up observations | New source packets and new observation evidence | Append; do not rewrite the earlier packet |
| Human-readable content | Source attachment / display-only content | Not a substitute for structured scientific fields |
| External content | Rights-qualified source reference | Do not fetch or redistribute automatically |

### 4.1 Time and coordinates

The adapter must preserve source time representation and scale, coordinate system/frame, reference position, units and uncertainties exactly as supplied.

Any conversion (for example, to an internal time representation or sky coordinate frame) is a separate derived operation, with:
- exact source field references;
- transform implementation and version;
- input/output values and units;
- frame/epoch/time-scale assumptions;
- propagated uncertainty;
- an explicit result state (qualified, partial, unsupported, unknown).

No implicit assumption that every time is UTC or every coordinate is ICRS is allowed.

### 4.2 Hypotheses

An initial Why assessment is evidence that the packet publisher proposed a hypothesis. It is not independent confirmation of that hypothesis.

Symthaea may retain the source hypothesis and attribution, generate competing explanations, identify contradictions and historical associations, issue falsifiable predictions, and recommend archive queries or follow-up observations.

It must not collapse source assessments, broker classifications, LLM-generated explanations and independent measurements into one corroboration count.

## 5. Mycelix DeSci composition

Use the existing canonical scientific-event kernel. Do not add a second event store, credential registry, authority engine or global trust score.

Recommended mapping:
- one stable astronomy ResearchObject for a source-native event or event thread;
- an EvidenceArtifact referencing the original packet bytes, exact digest, media type, source locator and rights/availability;
- typed evidence/attestation events for source assertion, parsing/validation, correction, review, follow-up and independent replication;
- explicit derived artifacts for parsed projections, cross-matches, model outputs and Symthaea assessments;
- versioned assessment policy for the precise claim being evaluated.

Do not create a universal VOEvent-to-AtomicClaim rule. A packet may report a candidate transient without establishing a scientific proposition beyond the fact that the publisher issued that report.

The canonical DeSci receipt establishes an auditable record of an accepted source assertion/event under the relevant authority policy. It does not independently authenticate an observatory, calibrate its instrument or verify the astrophysical event.

## 6. Rubin alert integration boundary

Rubin's first world-public alerts were issued on 24 February 2026. The first public night generated 800,000 alerts. Approximately seven million per night is the expected eventual full-stream scale, not a throughput result this project should claim today.

Rubin's current Early Science alert products and template coverage are still evolving ahead of the first full Data Release. Existing full-stream brokers already perform alert ingestion, filtering, cross-matching, photometric classification and follow-up prioritization.

Therefore:
- Do not build a competing Rubin full-stream broker.
- Do not use the 7-million/night eventual scale as a current benchmark result.
- Do not assume all prompt images or difference images are directly available under the same release/embargo policy as public alert packets.
- Use public broker products or bounded alert examples for a replay pilot.
- Preserve broker-generated classifications and annotations as derived source-attributed evidence, not as additional independent observations.
- Compare Symthaea's contribution to matched broker/expert baselines on investigation quality, not raw ingestion throughput.

## 7. First bounded replay benchmark

### 7.1 Input corpus

Start with a bounded, rights-reviewed packet set:
- normative VOEvent 2.1 positive and negative examples;
- public broker/alert examples where packet bytes and reuse terms are clear;
- historical/follow-up packet chains where the source references can be resolved;
- synthetic malformed packets only for parser negative controls (label synthetic fixtures as such).

For every corpus member, freeze retrieval date and exact URI/source, raw bytes and digest, rights and use disposition, publisher/broker/upstream lineage, VOEvent version and role, parser and projection versions, expected validation and semantic-mapping outputs, and exclusion reason for unavailable or unsupported references.

No public CI test should require a live broker or mutable URL. Keep the frozen small corpus and expected receipts in-repository only where rights permit; otherwise publish digests and retrieval instructions without redistributing packet bytes.

### 7.2 Required negative controls

1. Missing or incorrect VOEvent version.
2. Invalid or duplicate ivorn.
3. role=test incorrectly admitted as an astrophysical observation.
4. role=prediction incorrectly counted as an observed event.
5. role=utility incorrectly treated as a sky transient.
6. Missing role rewritten as explicitly supplied observation.
7. Publisher name treated as cryptographic authentication.
8. Broker-enriched packet counted as an independent source observation.
9. Mirrored/replayed packet counted more than once.
10. Two packets with nearby positions/times merged without a declared matching analysis.
11. Reference packet not found or reference type lost.
12. Why hypothesis automatically promoted to a verified claim.
13. Time scale, coordinate frame, units or uncertainty silently changed.
14. Missing external content treated as a successful fetch.
15. Rights-restricted payload fetched, copied or republished without permission.
16. Later follow-up/correction overwrites the original packet rather than appending a new event.

### 7.3 Metrics

Report separate lanes:
- byte-digest and source-native identity preservation;
- VOEvent schema/semantic conformance;
- packet role confusion and false observed-event promotion;
- exact reference resolution and lineage preservation;
- duplicate and false cross-match rates;
- coordinate/time/unit semantic-loss count;
- parser coverage and explicit unsupported-field count;
- acquisition/replay latency and resource cost;
- hypothesis calibration and prediction scores only when a separate adjudicated outcome set exists;
- follow-up decision value and cost only when candidate actions, constraints and outcome truth are frozen.

No composite scientific-truth score. A parser pass is not a discovery, and an alert classification result is not a physical measurement.

## 8. Initial baseline comparison

Compare like with like:
1. Strict source pass-through / VOEvent reference parser.
2. A public broker's published classification/annotation for the same event set.
3. Domain rule filters and catalogue cross-match baselines.
4. Human astronomer triage on a blinded, adjudicated subset.
5. Symthaea candidate triage and competing-hypothesis investigation.

Match source packets, event cutoffs, available context, rights, compute and human-review budgets. Report broker output as a broker-derived annotation with its model/version and source lineage.

The proposed improvement is not higher alert volume or a larger number of generated hypotheses. It is fewer incorrect event promotions, better-calibrated hypotheses, lower human investigation burden and better selection of follow-up observations at equal eligible resources.

## 9. Autonomy and safety boundary

v0.1 supports archive replay and read-only follow-up proposals.

It does not connect directly to telescope command systems, submit observing requests to a broker or scheduler, reserve telescope time, control observatory instruments, make public discovery announcements automatically, or claim independent scientific validation from a signed packet.

A follow-up proposal must state the scientific question, competing hypotheses, predicted outcomes, target/time window, required instrument capability, expected utility, cost/latency and stop rule. An authorized observatory operator or scientific programme decides whether to execute it.

## 10. Acceptance gates and claim ceiling

A first implementation may be called VOEvent 2.1 adapter-conformant for the frozen corpus only when:
- every admitted source packet has an exact byte digest and native ivorn;
- every packet role is preserved and the role negative controls pass;
- all mapped fields are loss-accounted;
- unsupported time/frame/reference semantics remain explicit;
- duplicate/mirror packets do not inflate source or corroboration counts;
- source hypotheses are not promoted into accepted claims;
- all rights and access failures remain explicit;
- the offline replay reproduces the same mapping and receipts from frozen inputs.

This profile is a proposed adapter design. It is not an IVOA certification, Rubin broker qualification, observatory partnership, astrophysical discovery, telescope authorization or proof of scientific truth.
