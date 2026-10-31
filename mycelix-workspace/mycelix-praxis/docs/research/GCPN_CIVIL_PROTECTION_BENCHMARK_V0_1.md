# GCPN Civil Protection Conformance Benchmark v0.1

**Status:** frozen synthetic conformance harness; non-operational research artifact.  
**Scope:** warning-record integrity, independent-source accounting, public-authority separation, local-first continuity, receiver-side delivery, accessibility accounting, source-to-alert traceability, and sovereign write boundaries.  
**Exclusions:** weapons, trajectories, interceptor logic, fire-control, target selection, launch authorization, military sensor placement, or operational alert policy.

## Research premise

The initial GCPN architecture proposal defined the network and evaluation dimensions but did not yet provide a runnable, independently checkable benchmark. This increment supplies a small deterministic oracle before attempting large-scale simulations or product comparisons.

The benchmark deliberately does not create another alert format. The Common Alerting Protocol (CAP) is already an open all-hazards exchange format. CAP 1.2 defines interoperable warning messages, including originator/message identity, urgency, severity, certainty, multilingual information, effective/expiry times, update/cancellation semantics, acknowledgements and dissemination-system reports. CAP does not specify a particular application or telecommunications method. GCPN should preserve CAP compatibility at the alert edge and add provenance/custody and federation semantics as a separate, versioned sidecar rather than fork the public alert standard.

- OASIS CAP 1.2: https://docs.oasis-open.org/emergency/cap/v1.2/CAP-v1.2.html
- ITU-T X.1303 bis (CAP 1.2): https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Documents/2020/T-REC-X.1303bis-201403-.pdf
- UNDRR CAP overview: https://www.undrr.org/early-warnings-for-all/common-alerting-protocol
- ITU Early Warnings for All, Pillar 3: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Early-Warnings-for-All-Initiative.aspx

The benchmark is aligned to the four Early Warnings for All pillars: risk knowledge, observation/forecasting, warning dissemination, and preparedness/response. A syntactically valid message is not a complete warning system if people cannot receive, understand, trust, or act on it.

## State-of-the-field comparison

| Existing capability / benchmark anchor | What it already does | GCPN's non-duplicative research question |
|---|---|---|
| CAP 1.2 / ITU-T X.1303 bis | Open all-hazards alert interchange, multi-language and multi-audience content, update/cancel semantics, channel-neutral message structure | Can every GCPN-derived alert map to CAP without losing uncertainty, provenance, authority, or correction history? |
| Early Warnings for All | Four-pillar, people-centred national and regional programme; national roadmaps and investment coordination | Can cross-border evidence exchange and local continuity be tested without overriding national ownership? |
| Multi-channel warning practice | Mobile/cell broadcast, radio, television, sirens, satellite, social channels and community infrastructure | What fraction of intended receivers actually receive and can access the message under channel loss and regional partition? |
| National alerting systems and regional mechanisms | Government-authorized warning, country-specific delivery infrastructure and local response arrangements | Can independent evaluators verify source-to-receiver lineage and distinguish message send, receiver receipt, accessibility, comprehension, and action? |

UNDRR reports that 91% of the world's population lives in a country implementing CAP; this is country-level implementation coverage, not proof that every person receives every alert. WMO's 29 October 2026 review of 35 government-validated roadmaps covers about 960 million people and identifies sustained investment, data sharing, governance, national ownership and implementation capacity as material requirements. These are standards and programmes GCPN should complement, not claim to replace.

## Paradigm shifts under test

1. **Separate authenticity from truth.** A valid signature says who signed a record and that it has not changed; it does not prove a physical measurement is correct.
2. **Count independent source families, not messages.** Five republished copies from one upstream source are not five independent witnesses.
3. **Keep authority separate from evidence.** A digitally authenticated observation does not itself authorize a public warning. A competent local authority remains accountable under its law and procedures.
4. **Preserve uncertainty instead of forcing consensus.** Unverified, contradictory, stale, and missing are meaningful states, not values to coerce into a binary result.
5. **Measure from the receiver backward.** A server-side send count is not proof of delivery, accessibility, comprehension, or action.
6. **Local-first operation.** A region with local authority and working local fallback should retain civil-warning capability when international/WAN links are down.
7. **Federated sovereignty.** One participating member must not write to another member's authority state without explicit delegation.
8. **Civilian value independent of missile defence.** The same network should support floods, fires, extreme heat, power failures, water disruption, public-health emergencies and other civil hazards.

## v0.1 runnable harness

The Rust package is dependency-free and isolated from the parent workspace:

```bash
cargo test --manifest-path benchmarks/gcpn-civil-conformance/Cargo.toml
cargo run --manifest-path benchmarks/gcpn-civil-conformance/Cargo.toml
```

The frozen input is `benchmarks/gcpn-civil-conformance/fixtures/gcpn-001-v0.1.csv`. It contains nine small synthetic cases:

- ordinary validated evidence;
- authentic record with unverified measurement;
- multiple reports sharing one upstream source family;
- contradictory evidence;
- stale evidence;
- missing evidence;
- WAN partition with local fallback and local authority;
- receiver accessibility gap;
- unauthorized cross-member mutation attempt.

The Rust tests include deliberate mutations to prove that the evaluator detects false source independence, authenticity/measurement conflation, rewritten stale/missing/contradictory states, unauthorized alerts, missing source lineage, local partition delivery loss, unauthorized cross-member writes, and malformed receiver counts.

The CSV is a tiny frozen test vector, not a representative sample of real alert traffic. Its ratios are fixture arithmetic and must never be presented as operational coverage or product performance.

## Metrics and gates

**Hard conformance gates:**

- zero false claims of source-family independence;
- zero silent changes to the oracle measurement state;
- zero authenticity-to-measurement-truth conflations;
- zero alerts emitted without a valid authority decision;
- 100% local-warning continuity in fixture cases where WAN is unavailable but local authority and fallback are available;
- 100% source-to-alert traceability for issued alerts;
- zero unauthorized remote-member state changes.

**Performance and accessibility metrics, always reported separately:**

- receiver-side delivery ratio = acknowledged intended receivers / intended receivers for issued alerts;
- accessible receiver coverage = receivers with at least one accessible format / intended receivers;
- p50/p95 end-to-end latency, measured from authorized publication to receiver acknowledgement;
- delivery/accessibility by channel, locale, language, disability-accessibility mode and connectivity tier;
- time to local service restoration after partition;
- time below agreed floors and cumulative service deficit for power, water, health, communications and shelter;
- warning calibration, false-alert/missed-event rates, abstention and uncertainty by hazard class;
- privacy exposure, retention, revocation latency and audit completeness;
- lifecycle cost per person reached with an accessible, actionable warning, not cost per message sent.

No single weighted score may hide a failed hard gate. Report denominators, missing receipts, abstentions, invalid runs, seed counts, confidence intervals, and subgroup performance. Do not select a single global threshold until policy authorities and affected communities establish the harm trade-offs.

## Baseline and SOTA comparison plan

A fair comparison must not treat CAP as a transport or a complete warning service. It is a message format. The planned comparison sequence is:

1. **Format conformance:** validate CAP 1.2 examples and required fields against the OASIS schema and relevant national profiles where publicly available. Verify lossless round-trip mapping between CAP and the GCPN provenance sidecar.
2. **Architecture archetypes:** compare a single-hub reference, a regionally federated reference, and a local-first federated reference under identical synthetic faults. These are reference architectures, not claims about named national systems.
3. **Standards-backed adapters:** add public CAP feeds/profiles and open warning-system adapters only where terms, licensing, safety and reproducibility permit. Do not scrape private or restricted infrastructure.
4. **Receiver-level trials:** only with voluntary partners, measure actual test-message delivery and accessibility using opted-in receivers. No real emergency alerts or sensitive infrastructure data in research tests.
5. **Held-out evaluation:** freeze scenario seeds and policy thresholds, seal hidden fault combinations, and have an independent evaluator reproduce the metrics.

Named operational systems should only be ranked after a compatible, independently evidenced adapter has been tested. Existing CAP/EW4All deployments must not be described as centralized or non-resilient baselines without evidence.

## Research targets (proposed, not achieved)

- 100% hard-gate conformance on the frozen synthetic suite.
- Zero false independent-source claims across adversarial dependency fixtures.
- Zero silent measurement-state changes across stale, missing and contradictory cases.
- Zero unauthorized alert issuance and zero unauthorized cross-member writes.
- 100% local continuity in every declared supported partition profile with local authority and fallback.
- 100% source-to-alert lineage for issued alerts.
- Public alert delivery and accessibility measured per receiver cohort with confidence intervals; target values set after baseline measurement and community review.
- CAP mapping has zero semantic loss for fields the profile claims to preserve; unsupported fields are explicit, not silently dropped.
- Held-out performance improvement is measured on paired scenarios against the strongest compatible baseline, with cost/compute/network budgets reported.

## Explicit limits

This harness does not implement a network, authenticate real identities, verify physical measurements, distribute alerts, model people, certify legal authority, establish interoperability with any national service, or validate any missile interception capability. It is only a deterministic synthetic conformance oracle and a foundation for later independent experiments.
