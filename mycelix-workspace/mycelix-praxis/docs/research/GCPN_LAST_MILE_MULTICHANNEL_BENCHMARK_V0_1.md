# GCPN Last-Mile Multi-Channel Receiver and Accessibility Benchmark v0.1

**Status:** deterministic synthetic benchmark; non-operational.  
**Parent:** issue #5609.  
**Channels:** cell broadcast, radio/TV, mobile data/app, local community relay.  
**Cohorts:** five fictional receiver groups, 2,700 synthetic receivers total.

## 1. Why last-mile coverage is a separate benchmark

A CAP message being valid, or a dissemination service reporting a successful send, does not prove that a person received an alert. A receiver receipt does not prove the person could access the message in a usable language or format. Accessible-format availability does not prove comprehension, trust, protective action or reduced harm.

This benchmark separates four layers:
1. **Channel dispatch / receipt:** each channel's synthetic receipt count.
2. **Unique delivery:** a receiver reached by one or more channels is counted once.
3. **Accessible-format delivery:** a receiver reached through at least one channel with a format declared accessible for that cohort.
4. **Comprehension / action / outcome:** deliberately out of scope here; these require separate voluntary, ethically reviewed studies and outcome evidence.

Multi-channel resilience is valuable because channel reach and channel adoption differ. ITU's 2026 cell-broadcast work identifies a large gap between mobile-network coverage and countries actually implementing or deploying cell-broadcast public-warning systems, with funding, institutional capacity, infrastructure and policy as barriers. The Africa Early Warnings Call to Action calls for inclusive and accessible warnings across channels including mobile, radio, television and community networks.

References:
- ITU, cell broadcast for public warning: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Publications/g20-cell-broadcast.aspx
- ITU, Addis Ababa Call to Action on Early Warnings in Africa: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/EW4ALL/Addis-Ababa-Call-to-Action-on-Early-Warnings-in-Africa.aspx
- UNDRR, Common Alerting Protocol: https://www.undrr.org/early-warnings-for-all/common-alerting-protocol
- UN Early Warnings for All: https://www.un.org/en/climatechange/early-warnings-for-all

## 2. Frozen fictional receiver cohorts

| Cohort | Intended receivers | Channel access | Accessible formats in fixture |
|---|---:|---|---|
| Urban general | 1,000 | Cell broadcast, radio/TV, mobile data | All three |
| Rural radio/community | 800 | Radio/TV, community relay | Both |
| Low-connectivity community | 500 | Community relay only | Community relay |
| Screen-reader mobile | 250 | Cell broadcast, radio/TV, mobile data | Mobile data only |
| Sign-language community | 150 | All four | Mobile data, community relay |
| **Total** | **2,700** | | |

These cohorts are invented homogeneous test groups, not estimates of any actual population. Within a cohort, the model assumes every member has the same channel access and accessible-format profile. Real-world heterogeneity, intersectionality, device status, language, literacy, gender, age, disability, geography and socioeconomic factors require further ethically governed study and must not be inferred from this fixture.

## 3. Channel model

Channel indices are fixed in this order:
1. cell broadcast;
2. radio/TV;
3. mobile data/app;
4. community relay.

A channel-level receipt is counted for each cohort member when that cohort has access to the channel and the channel is available in the scenario. A person reachable through multiple channels counts once in unique delivery. Each additional channel receipt is recorded as a duplicate channel delivery, not a new person reached.

Accessible coverage is counted when at least one available channel both reaches the cohort and provides a format declared accessible for that cohort.

The model does not simulate channel overlap within a cohort at individual granularity. It assumes cohort members are homogeneous and channel reach is complete for that cohort when its access flag and scenario-availability flag are both true. This is deliberately simple and deterministic, not an empirical telecommunications model.

## 4. Frozen fault matrix and structural results

All scenarios use the same 2,700 synthetic receiver denominator.

| Scenario | Unique reached | Accessible | Uncovered | Reached without accessible format | Duplicate channel receipts |
|---|---:|---:|---:|---:|---:|
| All channels up | 2,700 | 2,700 | 0 | 0 | 3,750 |
| Cell broadcast down | 2,700 | 2,700 | 0 | 0 | 2,350 |
| Mobile data down | 2,700 | 2,450 | 0 | 250 | 2,350 |
| Radio/TV down | 2,700 | 2,700 | 0 | 0 | 1,550 |
| Community relay down | 2,200 | 2,200 | 500 | 0 | 2,800 |
| Cell broadcast + mobile data down | 2,700 | 2,450 | 0 | 250 | 950 |
| Radio/TV + cell broadcast down | 2,700 | 2,700 | 0 | 0 | 150 |
| Grid/mobile network loss; radio/community backup | 2,700 | 2,450 | 0 | 250 | 950 |
| Only community relay | 1,450 | 1,450 | 1,250 | 0 | 0 |
| No channels | 0 | 0 | 2,700 | 0 | 0 |

These rows are exact consequences of the fixture's channel access and availability flags. They are not measured delivery coverage, availability or real-world channel reliability.

### Findings from the synthetic fixture

- With every channel active, 2,700 unique people are reached across 6,450 channel receipts. There are 3,750 extra channel receipts beyond unique reach.
- Under mobile-data loss, all 2,700 are still reached by at least one channel, but the 250-person screen-reader cohort has no remaining accessible format in this fixture. Aggregate delivery remains 100%, while accessible coverage falls to 90.74%.
- Under community-relay loss, the 500-person low-connectivity cohort is completely uncovered, despite the other 2,200 receivers being reached.
- With only community relay, 1,450 are reached and 1,250 are uncovered.
- With no channels, no one is reached.

These are diagnostic examples of why a single aggregate delivery percentage is inadequate. Channel redundancy can improve unique reach while still leaving accessibility gaps or entire cohorts uncovered.

## 5. Metrics (reported independently)

- Intended receivers (denominator).
- Unique receiver delivery = receivers reached by at least one available channel / intended receivers.
- Accessible receiver coverage = receivers reached through at least one accessible available format / intended receivers.
- Accessibility among reached = accessible receivers / unique receivers reached.
- Uncovered recipients by cohort.
- Reached recipients without accessible format by cohort.
- Channel-specific receipt counts.
- Duplicate channel receipts = channel-level receipts minus unique receivers reached.
- Cohort-level delivery and accessibility gaps.

Never infer human receipt from a server send count, or comprehension/action from a receiver transport receipt. No weighted total may hide zero coverage or an accessibility failure for a cohort.

## 6. Reproduction

From the Mycelix repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-last-mile/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-last-mile/Cargo.toml
```

The executable prints scenario-level and cohort-level outputs. The fixtures are pinned by deterministic FNV-1a regression fingerprints in the Rust test suite.

## 7. Comparison with industry and SOTA

The appropriate baseline is not a hypothetical system that only uses one channel. Compare these channel portfolios and receiver profiles under the same cohorts and fault conditions:
- cell-broadcast-only;
- mobile-data/app-only;
- radio/TV-only;
- community-relay-only;
- fixed multi-channel policy;
- adaptive multi-channel policy (future work; must have explicit lawful authority and transparent policy, not algorithmic alert authorization).

Compare on:
- unique receiver delivery rather than summed sends;
- accessible format delivery rather than nominal channel availability;
- subgroup-specific uncovered population;
- p50/p95 receiver acknowledgement in later measured trials;
- duplicate receipt burden and channel congestion in later channel models;
- lifecycle cost and operator burden;
- independent receiver trial evidence.

The current fixture establishes only deterministic accounting rules. It cannot rank national warning systems, telecom operators, cell-broadcast deployments or commercial vendors. Named system comparisons require permissioned adapters, compatible receiver definitions, equivalent channel trials and independently reviewed evidence.

## 8. Next benchmark increments

1. Replace homogeneous cohort assumptions with explicit synthetic receiver-level records while preserving privacy and reproducibility.
2. Add channel overlap and partial delivery at receiver-level granularity.
3. Add queueing and time-to-receipt only after link/service assumptions are explicitly defined.
4. Add message updates/cancellations, duplicate suppression and expiry.
5. Add multilingual/accessible CAP profiles with field-preservation checks.
6. Add compound channel faults correlated with power and connectivity outages.
7. Conduct voluntary receiver trials with informed consent, ethics/privacy review, and no real emergency-alert dispatch.
8. Measure comprehension and protective action in separate studies, not from delivery logs.

## 9. Non-claims and safety boundary

This package does not connect to mobile networks, send alerts, collect subscriber data, track actual recipients, estimate actual population coverage, model human comprehension, or direct protective action. It uses only invented cohorts and deterministic channel availability. It does not model military communications, weapons or engagement systems.
