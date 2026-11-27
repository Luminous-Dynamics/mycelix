# Water Service Repair Benchmark V1

This folder contains a frozen, synthetic-only adversarial corpus for the prospective CRAFT water-reliability pilot (#6771).

- `manifest.json` records public baseline sources, their periods and claim limitations.
- `integration-map.json` maps each case to existing Mycelix semantic owners; it deliberately avoids a duplicate generic evidence/currentness service.
- `metamorphic-cases.json` defines twenty proposed counterfactual mutations, including context, freshness, independence, denominator, regulatory-boundary and KPI-definition tests.
- `../docs/WATER-SERVICE-LIFECYCLE-QUALIFICATION-V1.md` defines the customer-outcome measurement model, event chronology, cross-core handoff contract and prospective pilot protocol.
- `cases.json` contains sixteen synthetic known-answer cases.

The published 48-hour burst KPI definition is ambiguous: the 2025/26 business plan defines it as on-site response initiated, not necessarily completed repair. The benchmark therefore separates first response, repair completion, pressure restoration and water-quality release instead of treating them as one clock. It also covers statutory alternative supply during interruptions over 48 hours and extended-supply monitoring after one week.

All area references, event IDs, evidence IDs, digests and source groups are synthetic placeholders. The fixtures do not contain real household locations, asset coordinates, staff names or private municipal data.

The corpus separates provider-reported completion from physical restoration, water quality and locally authorized closure. It also tests exact-context binding, stale/superseded results, source common-cause duplication, permit/credential revocation, incomplete expected-action coverage and repeated bursts.

These are benchmark fixtures, not production safety rules. A field pilot requires local authority and community agreement, source-owner permission, privacy/security review, professionally approved water-quality criteria, and a newly measured comparable baseline.

## Public baseline context

The City of Johannesburg's FY2024/25 report recorded 23,521 bursts restored within 48 hours (70.35%, against an 85% target). Johannesburg Water's 29 October 2026 public statement described low pressure, intermittent supply and prolonged outages in constrained systems. These periods and measures must remain distinct; the published 2024/25 metric is not live status or a causal baseline for an intervention.

See issue #6771 for the pilot protocol and limitations. The metamorphic validator confirms only vector structure and seed references; it does not execute the mutations against the Rust reference cores. That requires the cross-core adapters and independent replay oracle. Do not treat structural fixture validation as end-to-end qualification.

Run the dependency-free structural fixture validation locally with:

```sh
bash mycelix-workspace/mycelix-craft/benchmarks/water-service-repair-v1/validate.sh
```


WSR-016 is a metrology negative control: Johannesburg Water's FY2024/25 annual report describes 70.35% of bursts restored within 48 hours, while its 2025/26 business plan gives a 67.46% baseline labelled 2023/24 and operationally defines the 48-hour milestone as on-site response initiated, not necessarily resolved. These measures must not be compared without reconciling period, numerator and event semantics.
