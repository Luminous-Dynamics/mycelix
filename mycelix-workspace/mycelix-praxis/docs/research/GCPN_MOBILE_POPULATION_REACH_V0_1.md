# GCPN Privacy-Preserving Mobile-Population Warning Reach v0.1

**Status:** research specification and machine-readable DGP contract; executable engine outstanding.  
**Parent issue:** #5654.  
**Baseline:** integrated GCPN evaluation v0.6.  
**Scope:** offline synthetic aggregate population exposure, time-varying reach, mobility and privacy.

## 1. Why this track is necessary

The existing GCPN last-mile benchmark uses static synthetic receiver cohorts. That is a good first test of channel availability and duplicate receipt accounting, but people are not stationary. A warning can be geographically correct and still fail to reach people who move between home, work, transit, temporary accommodation, seasonal destinations or jurisdictions during the warning's validity window.

UNDRR, WMO, IFRC and ITU published *Early Warning Systems for Mobile Populations: Operational Guidance* on 15 July 2026. The guidance explicitly treats mobility as a persistent feature of disaster risk and calls for existing MHEWS processes to include people whose location and circumstances change. It describes mobile populations broadly, including displaced people, refugees, migrants in vulnerable situations and people affected by temporary movement. GCPN's role is to provide a privacy-preserving synthetic evaluation harness that complements existing institutions and national channels.

## 2. Paradigm shift: from static recipients to time-varying aggregate exposure

Replace a static receiver roster with an aggregate exposure model:

`cohort x coarse_zone x time_window x access_profile`.

Each cohort has a count, a transition distribution between coarse zones, time-varying channel access and non-identifying accessibility/language requirements. No person is represented by a persistent identifier. No individual GPS trace, device ID, immigration status or inferred vulnerability label is present.

A warning is evaluated against the aggregate population present in the relevant area during the warning's effective/validity period, not merely against the residential population registered there at an earlier time.

The system must distinguish:
- people who are present at a location during the warning window;
- people entering or leaving that location;
- people briefly transiting through a zone;
- cohorts with uncertain movement flows;
- people unable to move, who must not disappear from the denominator;
- people whose access channels, language or accessible-format needs are known only through voluntary/community-governed aggregate research, not inferred from network metadata.

## 3. Privacy-first synthetic model

### Allowed synthetic inputs

- coarse zone IDs without person-level coordinates;
- time-bucketed aggregate cohort counts;
- aggregate flow matrices with explicitly modelled uncertainty;
- synthetic channel availability and outage matrices by zone/time;
- synthetic language/accessibility service capabilities by cohort;
- policy-scoped authority/jurisdiction boundaries;
- aggregate dwell-time distributions and transit windows.

### Prohibited inputs and derived outputs

- persistent person/device IDs;
- per-person GPS or continuous movement traces;
- individual migration routes or identifiable journey histories;
- inferred migration, displacement, legal status, disability or vulnerability from network/device data;
- personal eligibility or recipient targeting;
- public release of small cells that could expose a synthetic individual's presence.

The benchmark should model privacy risk directly. Small-cell suppression, output coarsening and query differencing controls are candidates, but no universal minimum-cell threshold is prescribed. Thresholds require a threat model, utility analysis and independent privacy review.

## 4. Aggregate DGP families

| Family | Scenario | Important failure mode |
|---|---|---|
| Static baseline | Cohorts remain in home zones | Baseline for comparison |
| Commuter flows | Repeated home/work movement by time bucket | Static registration misses work-hour exposure |
| Transit/temporary presence | Short dwell at interchange/corridor zones | Brief exposures omitted by coarse daily snapshots |
| Seasonal flows | Time-varying destination populations | Outdated seasonal denominator |
| Sudden displacement | Uncertain movement after synthetic disruption | False precision from stale flow matrices |
| Cross-jurisdiction | Cohort moves across warning-authority boundaries | Unauthorized message issuance or missing handoff |
| Movement-constrained | Some cohorts cannot move within the warning window | Incorrect assumption that everyone can evacuate |
| Channel transition | Roaming/network outage or connectivity loss | Duplicate receipts or reach gaps at transitions |
| Accessibility/language | Distinct, non-inferred aggregate needs | Global average hides exclusion |
| Compound window | Multiple hazards and different zone validity periods | Stale or wrong-window exposure joins |

Each DGP has frozen counts, transition matrix, time buckets, channel/format capabilities and exact total-conservation invariants. Unknown flows remain unknown with explicit uncertainty; they are not filled with zero or treated as no exposure.

## 5. Exposure and reach accounting

Let (c) index a synthetic aggregate cohort, (z) a coarse zone and (t) a time bucket. Let (N_{czt}) be the aggregate cohort count, (R_{czt}) the count with evidence of a timely valid message receipt under the declared channel model, and (A_{czt}) the count with an accessible representation available.

Primary aggregate metrics should include:

- exposure-weighted timely reach: `sum(R_czt) / sum(N_czt)` over the predeclared hazard/time exposure window;
- accessible reach: `sum(A_czt) / sum(N_czt)`, with its own evidence definition;
- conditional accessible reach among cases with an accessible representation;
- effective accessible reach per intended exposed cohort;
- worst-cohort reach and cohort-to-cohort gap;
- entrant, transit and movement-constrained cohort reach separately;
- duplication of channel receipts versus unique aggregate cohort coverage;
- delay between entering the exposed zone and first valid message receipt;
- late/expired message and stale-zone mapping counts;
- coverage denominator uncertainty where aggregate flow data is incomplete.

A server-side send is not a recipient receipt. A receipt is not comprehension or action. Aggregate counts do not establish the experience of any individual.

## 6. Cross-jurisdiction authority and CAP boundary

CAP remains the alert message format. National and local authorities retain their existing authority to issue warnings for their jurisdiction and hazard remit.

The synthetic benchmark may represent:
- a source authority's declared zone/time scope;
- an incoming CAP message and its exact references/version;
- a cross-border notification or relay request;
- an explicit destination-authority acknowledgement/handoff;
- rejection of a message that would otherwise be re-issued outside the source authority's scope.

The benchmark must not turn a transport relay into a new issuing authority. No automatic re-issuance, translation into a new official alert or responder direction is introduced. Each jurisdiction's role is part of the synthetic policy fixture.

## 7. Comparisons

Compare these patterns under identical DGP trajectories:
1. static home-zone/registered-cohort reach;
2. aggregate time-window zone exposure;
3. multi-channel distribution using declared channel availability;
4. local-first community relay during network/roaming partitions;
5. cross-jurisdiction handoff with explicit authority scope.

Report results separately. Do not declare a universal architecture winner or collapse reach, accessibility, privacy, timeliness and cost into one score.

## 8. Required qualification targets (research hypotheses)

- Zero persistent personal/device identifiers in fixtures and released benchmark outputs.
- Exact conservation of aggregate cohort totals at every transition, including movement-constrained cohorts.
- Zero unauthorized cross-jurisdiction issuance in authority-handoff tests.
- 100% detection of deliberately stale/expired zone-time mappings.
- Explicit uncertainty/abstention where mobility-flow inputs are missing or weakly identified.
- Worst-cohort and entrant/transit measures always reported; no overall average without its denominator and subgroup table.
- Privacy differencing tests must detect configured disclosure attempts or suppress the release.
- Deterministic replay from pinned synthetic matrix, policy, seed and version.

These are internal design targets, not official UNDRR/WMO/IFRC/ITU standards or achieved field performance.

## 9. Integration boundary

The first deliverable is a standalone benchmark. Do not add a ninth v0.6 gate at this stage. Only consider integration after the DGP, aggregate conservation, authority handoff and privacy tests are implemented and independently reviewed.

Existing GCPN modules remain separate:
- unique accessible last-mile reach;
- message comprehension and uncertainty understanding;
- action evidence and outcome evaluation;
- CAP and authority provenance;
- topology continuity and lifecycle cost.

No synthetic movement count should be described as measured population coverage, human receipt, comprehension, action, vulnerability or lives saved.

## 10. Evidence progression

1. synthetic aggregate DGP and exact conservation oracle;
2. privacy red-team review of small-cell and differencing outputs;
3. community co-design of cohort definitions and access assumptions;
4. approved, data-minimized aggregate historical replay where legally and ethically permitted;
5. closed continuity exercises with national/local authority oversight;
6. prospective shadow evaluation without live alert decisions;
7. operational integration only under the responsible authority and legal governance.

## 11. Sources

- UNDRR/WMO/IFRC/ITU, *Early Warning Systems for mobile populations: Operational guidance* (15 July 2026): https://www.undrr.org/publication/documents-and-publications/early-warning-systems-mobile-populations-operational
- UNDRR launch event (22 July 2026), jointly developed with WMO, ITU, IFRC and IOM: https://www.undrr.org/event/launch-operational-guidance-early-warning-systems-mobile-populations
- ITU, EW4All Pillar 3 — warning dissemination and communication: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Early-Warnings-for-All-Initiative.aspx
- IFRC, Early Warnings for All — community reach, trusted/understood messages and feedback: https://www.ifrc.org/our-work/disasters-climate-and-crises/climate-smart-disaster-risk-reduction/early-warnings-all

## 12. Non-claims

This is a synthetic aggregate benchmark specification, not a population-tracking system, geolocation product, immigration or vulnerability inference tool, live alert channel, national MHEWS, cross-border warning authority, or causal impact study. It does not collect or process real individual location traces.