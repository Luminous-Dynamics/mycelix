# LKI-01 Repairability Benchmark Protocol v0.1

**Status:** Draft evaluation protocol; no physical or human trial has been performed  
**Scope:** Extend the existing lifetime-product engineering contract without mutating the frozen v0.3 qualification corpus  
**New software artifact:** [LKI_01_REPAIRABILITY_BENCHMARK_EXTENSION_V0_4_DRAFT.json](./LKI_01_REPAIRABILITY_BENCHMARK_EXTENSION_V0_4_DRAFT.json)

## 1. Paradigm shift: measure repair outcomes, not just repair potential

A parts count, fastener count, disassembly estimate, or scalar repairability index can describe design potential; none alone shows whether a human can open the product, identify a fault, obtain a part, restore the function, and keep the repair working. A 2025 observational study of 26 participants found substantial gaps between calculated and perceived repairability for two devices, identifying product opening and fault identification as important barriers. A 2025 household-appliance study applied EN 45554-style analysis and physical manual-disassembly testing across cooker hoods, hobs, combo-hobs, ovens and freestanding kitchens. These results motivate two separate evidence streams: technician repair outcomes and intended-user access/task outcomes. They do not establish an LKI-01 result.

Research:
- User-perceived repairability study (2025): https://doi.org/10.1016/j.spc.2025.05.024
- Functional-module design for repair study (2025): https://www.sciencedirect.com/science/article/pii/S2212827125002434
- ISO 14040 life-cycle assessment framework overview: https://www.iso.org/standard/37456.html
- EU Right to Repair Directive (consolidated 31 July 2026): https://eur-lex.europa.eu/eli/dir/2024/1799/2026-07-31
- IEC 60335-1 household-appliance safety standard overview: https://webstore.iec.ch/en/publication/61880

Use the applicable standard editions and conformity advice for the specific product and market. Links here are research anchors, not proof of compliance.

## 2. Separate the repair event into observable stages

For every designated repair task, record, separately:

1. **Diagnosis time:** from the declared start event to a recorded fault hypothesis or diagnostic conclusion.
2. **Active labour time:** hands-on service time, with pauses and role changes accounted for by a pre-registered rule.
3. **Parts/tool wait:** elapsed time waiting for parts, tools, access, approvals, or external service; distinguish this from active labour.
4. **Verification time:** the time used for the documented post-repair functional checks.
5. **Rework and failed attempts:** every unsuccessful attempt, damage event, repeated opening, or return to diagnosis.
6. **Total downtime:** from the declared service-unavailable start to a defined restored-service point. It is an outcome, not the sum of active labour alone.

Retain raw event timestamps where practical; document clock definitions, time zones, missingness, pauses, overlapping events and censoring. Never convert a missing phase to zero. Bind each record to the product, module, BOM and interface revisions tested.

## 3. Report metrics as a vector, not one score

At minimum publish the sample size, task definition, participant/technician qualification, attempt count, successful completion rate, median and P90 for diagnosis/active labour/wait/downtime, range or distribution, rework rate, damage rate, post-repair verification rate and recurrence window. Report uncertainty and all exclusions. A median under a service-time target cannot hide a long tail, failed jobs, structural damage or parts delays.

Keep two views separate:
- **Technician repairability:** qualified-service tasks, tools, labour, rework, verification and service continuity.
- **Human-facing accessibility:** whether intended users can identify the relevant access point, open the designated non-hazardous access feature, locate a labelled serviceable module, and understand when to stop and call a qualified technician.

Human-facing tests are not invitations to open energized or hazardous appliances. Do not ask general participants to perform mains-electric, induction-cooking, refrigerant, or other hazardous procedures.

## 4. Protocol and sample design

### Pilot

Run a formative pilot with a small, diverse group of intended users for non-hazardous opening/access/fault-recognition tasks, plus qualified technicians for actual service tasks. Use the pilot to identify confusing task instructions, task-order effects, tools/access obstacles, safety stop conditions, and measurement burden—not to make a superiority claim.

### Confirmatory comparison

Pre-register the primary endpoint, baseline, task, inclusion/exclusion rules, participant roles, sample-size rationale, repeat structure, randomisation/counterbalancing, handling of failed attempts, analysis method, and stopping criteria before collecting the confirmatory data. Use a power or precision analysis appropriate to the endpoint; “five technicians by six repetitions” may be a planning starting point, not a universal sufficient sample size. Repeated tasks by one technician are correlated observations and must not be treated as independent people.

Use a matched conventional design as baseline. If comparing alternative module layouts, keep the fault, task instructions, tools, training and operating conditions constant where possible. Record learning effects and order effects. Have reviewers assess the result against the pre-registered rule, not after seeing the preferred design.

### Follow-up and recurrence

Declare the observation window and usage exposure for each claim before the study. Record recurrence as its own event tied to the original repair and exact component/interface revision. A 30- or 90-day follow-up may be useful as an explicitly scoped early-recurrence measure where feasible; it is not evidence of a 20- or 50-year life. Any longer-lived reliability statement needs a separate data/model case, assumptions, uncertainty and field evidence.

## 5. Proposed software acceptance targets

These are evaluator targets, not physical-product goals:

| Benchmark | Proposed target | Meaning |
|---|---:|---|
| False pass for a failed non-compensable gate | 0 | No score offsets opening failure or permanent-structure damage |
| Repair-phase completeness for records claiming complete burden data | 100% | Required phases and exact revision are present |
| Verified-restoration evidence | 100% | No repair is counted as restored without scoped verification |
| Human-facing claim evidence | 100% | Missing user-task observation forces an inconclusive claim |
| Parts-continuity claim evidence | 100% | Dated evidence names the exact service part and scope |
| Known-answer replay | 100% | All 11 extension scenarios produce the specified oracle result |

For real product trials, do **not** make a 100% repair success rate the sole goal: that can encourage hiding unsuccessful attempts or excluding difficult cases. Report outcomes and uncertainty honestly; use engineering-defined limits for damage, safe access and verification as hard constraints.

## 6. CI/software extension

The extension is additive. v0.3 remains unchanged and is audited separately. The new Rust binary checks the v0.4 extension's structure and deterministic scenario outcomes; it does not simulate physical repairs or participants. Both suites should run with the committed lockfile and pinned toolchain. A passing software run establishes only that the evaluator matches the frozen synthetic cases.

## 7. LKI-01 design consequences

- Design access doors and service routes around actual task choreography, not just static module envelopes.
- Prefer modular connections that permit safe isolation and removal without dismantling unrelated permanent structure.
- Give replaceable parts legible IDs and connect each repair procedure to the exact revision and safe stop conditions.
- Make a module's parts continuity visible as a dated claim with scope, expiry and alternate strategy, rather than an undocumented promise.
- Preserve hard gates for structure damage, access, thermal/electrical safety and manufacturer's installation conditions. Pareto ranking starts only among candidates that pass declared gates.
- Build an unpowered full-scale fixture before energising appliances; then validate each certified module independently before any combined configuration.

The physical demonstrator, human study, South African parts availability, appliance conformity, 50-year life and any whole-life environmental superiority claim remain unestablished.
