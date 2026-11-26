# Water Service Repair Benchmark V1

This folder contains a frozen, synthetic-only adversarial corpus for the prospective CRAFT water-reliability pilot (#6771).

- `manifest.json` records public baseline sources, their periods and claim limitations.
- `integration-map.json` maps each case to existing Mycelix semantic owners; it deliberately avoids a duplicate generic evidence/currentness service.
- `cases.json` contains twelve synthetic known-answer cases.

All area references, event IDs, evidence IDs, digests and source groups are synthetic placeholders. The fixtures do not contain real household locations, asset coordinates, staff names or private municipal data.

The corpus separates provider-reported completion from physical restoration, water quality and locally authorized closure. It also tests exact-context binding, stale/superseded results, source common-cause duplication, permit/credential revocation, incomplete expected-action coverage and repeated bursts.

These are benchmark fixtures, not production safety rules. A field pilot requires local authority and community agreement, source-owner permission, privacy/security review, professionally approved water-quality criteria, and a newly measured comparable baseline.

## Public baseline context

The City of Johannesburg's FY2024/25 report recorded 23,521 bursts restored within 48 hours (70.35%, against an 85% target). Johannesburg Water's 29 October 2026 public statement described low pressure, intermittent supply and prolonged outages in constrained systems. These periods and measures must remain distinct; the published 2024/25 metric is not live status or a causal baseline for an intervention.

See issue #6771 for the pilot protocol and limitations.

Run the dependency-free fixture validation locally with:

```sh
bash mycelix-workspace/mycelix-craft/benchmarks/water-service-repair-v1/validate.sh
```
