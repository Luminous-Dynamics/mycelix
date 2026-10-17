# Farmer-first product experience v1

Status: product hypothesis and acceptance contract; not field-validated.
Owner: Mycelix experience/integration layer.
Related work: [MYC-FOOD-001 #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001), [Symthaea AGRI-ROBOTICS-001 #7797](https://github.com/Luminous-Dynamics/symthaea/issues/7797), [Symthaea platform screener PR #7817](https://github.com/Luminous-Dynamics/symthaea/pull/7817), [Symtropy field-robot cost issue #1814](https://github.com/Luminous-Dynamics/symtropy/issues/1814), [Sol-Atlas AGRO-CLIMATE-001 #107](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/107).

## Product thesis

Farmers and ranchers should love this suite because it gives them **more control, less busywork, clearer decisions, and visible economic value**—not because they are asked to adopt four projects or trust an unexplained AI.

The first product must solve a real recurring job before asking for sensor purchases, elaborate farm mapping, a large data migration, or continuous connectivity. Start with a farmer-owned **Farm/Ranch Daybook**: quickly record what happened, get a useful follow-up or report, and reuse the same record for work planning, costs, maintenance, cooperative fulfilment or spatial context. The farmer should never need to understand the underlying project boundaries.

This is a hypothesis to test with real operators, not a claim that the present pre-alpha codebase already provides a production-ready farm application.

## Evidence and what it does—and does not—tell us

1. Pawera et al., *Agricultural Systems* (2024), [Guidance on farmer participation in the design, testing and scaling of agricultural innovations](https://doi.org/10.1016/j.agsy.2024.104006), concludes that farmer participation should be chosen to match innovation readiness and should start early enough to influence the design. Implication: interview and co-design before building a broad feature set.
2. Yang, Ou and Maphango for FAO, [Strengthening digital agricultural extension and advisory services in smallholder farming](https://doi.org/10.4060/cc6267en), identifies infrastructure/connectivity, device and ICT maintenance cost, governance and institutional capacity as barriers. Implication: offline operation, low-cost onboarding, support and ecosystem fit are core requirements, not polish.
3. Dibbern et al., *Smart Agricultural Technology* (2024), [Main drivers and barriers to the adoption of Digital Agriculture technologies](https://doi.org/10.1016/j.atech.2024.100459), identifies economic constraints, infrastructure, technological knowledge, usefulness, ease of use, reliability, privacy and interoperability as relevant adoption factors. Implication: measure ease, trust and useful outcomes alongside model metrics.
4. A McKinsey survey of 5,500 row- and specialty-crop farmers across selected regions, [Agtech: Breaking down the farmer adoption dilemma](https://www.mckinsey.com/industries/agriculture/our-insights/agtech-breaking-down-the-farmer-adoption-dilemma) (2023), reports concerns about price, unclear ROI and data sharing, and argues for personalization, easier onboarding, trusted human support and integrated actionable insight. Its samples and estimates are not South African rancher evidence; use them as hypotheses, not local facts or pricing guarantees.
5. Grant et al., *Journal of Rural Studies* (2026), [Investigating robot acceptance in UK agriculture](https://doi.org/10.1016/j.jrurstud.2026.104096), reports that acceptance varies by sector and role and that participants preferred task assistance over robots making operational decisions. This is UK evidence, not a universal preference, but it reinforces a supervised, operator-controlled starting point.
6. A 2025 study of digital information sources among smallholder farmers in Tshwane, [South African Journal of Agricultural Extension](https://doi.org/10.17159/2413-3221/2025/v53n3a19230), reports that access to extension services was positively associated with adopting online information sources; digital information use did not show a significant agricultural-income effect in that study. Implication: fit local advisers and extension into the learning loop, and do not equate app usage with economic impact.

## The farmer-facing promise

> Record it once. See what matters next. Keep your data. Stay in charge.

The interface should use the operator's words (for example, paddock, camp, block, field, herd, water point, implement or crop—chosen by the operator), not force one agronomic vocabulary on all farm types. Farmers, ranchers and workers are domain experts and should be treated as co-designers, not as a training problem.

Avoid “AI-powered” as the primary value proposition. Do not market consciousness, autonomy or generic “optimization.” Show a concrete useful result, how it was derived, its freshness, and what remains uncertain.

## First wedge: an offline Farm/Ranch Daybook

A complete first vertical slice is narrower and more credible than a giant agricultural dashboard.

### Capture once

- Record a task, observation, incident, job, input use, repair or handoff via a compact form; optional voice/photo can be added only where it works reliably in the target language and field environment.
- Start with local records and a few example templates; sensors, satellite feeds and farm machinery integrations are optional extensions, not onboarding prerequisites.
- Link an event to a farmer-selected field/paddock, water point, asset, animal group, product lot or work item when known. Allow unknown instead of requiring fabricated detail.
- Save while offline, show clearly that the event is local/pending/synced, and synchronize idempotently later. Conflicting edits must be visible and resolvable, never silently overwritten.
- Turn the same event into follow-up tasks, a maintenance history, a cost line, a co-op fulfilment receipt or a map annotation when relevant. Never require re-keying the same facts into four separate applications.

### Three simple destinations

1. **Today:** a short, editable list of upcoming commitments and exceptions based only on declared tasks, source-backed observations and explicit rules. Each item says why it is shown, when its evidence was observed, and what could make it wrong. Farmers can reorder, snooze, dismiss or turn off suggestions.
2. **Costs and options:** compare a small number of explicit scenarios for a task (cash, labour, machine time, travel, water/energy where measured, uncertainty and exclusions). Show assumptions and units. Do not turn missing values into zero or combine different kinds of evidence into a magic score.
3. **Records to use elsewhere:** export or share a clear report for an adviser, co-op, bookkeeper, lender, insurer or buyer. Show precisely what will be shared; farm geometry, personal details, animal records and household/buyer identities are private by default.

Maps and imagery should answer a concrete question—“which water points need a visit?” or “which access route is blocked?”—rather than serve as a beautiful but passive globe. Keep lists and low-bandwidth summaries usable without map tiles.

## How the four projects fit together

The operator sees one workflow. Internally, each project retains clear ownership; the MVP may ship useful functionality before every integration is available.

| Project | What it must earn from farmers | Boundary |
| --- | --- | --- |
| **Symthaea** | Turn user-selected questions and available evidence into understandable, uncertainty-aware summaries, comparisons and reminders. Say what is known, estimated, stale, conflicting or unknown; abstain when evidence is insufficient. | Advice is not agronomic certification, veterinary diagnosis, machine-safety qualification or authority to apply chemicals, treat animals, pump water or actuate equipment. Reuse canonical farm task/safety contracts; do not duplicate Symtropy's scenario comparer. |
| **Mycelix** | Keep operational records, consent, identity/roles, sharing, corrections, provenance and cooperative handoffs understandable and farmer-controlled. Make multi-person work possible without publishing sensitive locations or commercial information. | A signed receipt establishes a recorded event under its stated semantics; it does not prove an agronomic outcome, food safety, regulatory compliance or payment unless the required evidence exists. The existing food co-op slice remains a related vertical, not the whole farmer product. |
| **Symtropy** | Show trade-offs in farm/ranch plans and service options: cost per completed task/service unit, labour, travel, downtime, input use, sensitivity to uncertain assumptions and value of missing measurements. | Compare declared scenarios; don't invent probability distributions, imply a best choice from incomplete evidence, or duplicate Symthaea's explanation layer. Economics models are not realized ROI. |
| **Sol-Atlas** | Make site, terrain, climate, water point, access, paddock/field and logistics context visible when it changes a decision. Let operators annotate and correct local context. | Every layer has a source, date, units/coordinate reference and uncertainty. A map layer is not proof of current water availability, land access, boundary/legal title, navigability or field safety. |

Prefer narrow adapters and documented contracts. No project should become a mandatory login or dependency for a simple Daybook entry. Existing issues to compose include [Mycelix food co-op #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001), [Symthaea platform screener #7817](https://github.com/Luminous-Dynamics/symthaea/pull/7817), [Symthaea robot safety #7818](https://github.com/Luminous-Dynamics/symthaea/issues/7818), [Symtropy field robot cost #1814](https://github.com/Luminous-Dynamics/symtropy/issues/1814) and [Sol-Atlas climate profile #107](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/107).

## Co-design campaign before broad implementation

### Participants and sampling

Recruit a small but deliberately varied formative cohort (target: 15–20 operators) across at least three operational archetypes:

- smallholder or mixed farm with constrained connectivity and shared services;
- crop/horticulture operation with time-sensitive field work and input/logistics costs;
- extensive livestock ranch/range operation with grazing, water, fences, animal groups and mobile work.

Recruit both enthusiastic adopters and skeptical/non-users, and include different farm roles, languages, ages, genders and levels of digital access where feasible. These numbers are a proposed discovery sample—not statistically representative evidence. Include a local extension adviser, mechanic/service provider or co-operative operator as a workflow participant, not as a replacement for farmer feedback.

### Method

1. Ask each participant to walk through one recent, costly or frustrating task using their current paper, phone, radio, spreadsheet or software. Observe the process; do not start by showing a solution.
2. Ask what decision they had to make, which evidence was trusted, what went wrong, how much time/cash was lost, which other people were involved, and what records they had to repeat.
3. Co-design paper/card prototypes with their real vocabulary. Test them in short task sessions before writing additional adapters.
4. Prototype the Daybook on the cheapest available device, with airplane-mode tasks and recovery/re-sync tests.
5. Offer a time-boxed, opt-in pilot with a baseline and exit/export path. Participants can delete/withdraw data within clearly stated legal and operational limits; do not pay for positive reviews or condition support on data sharing.
6. Review findings with participants and publish anonymized aggregate findings, including negative results and which ideas were rejected.

Do not claim “farmer validated” until the study has happened and the evidence, consent, participant characteristics and limitations are recorded.

## Acceptance benchmarks

These are proposed release gates to test and adjust with participating farmers—not achieved results or universal standards.

### Usability and usefulness

- At least 90% of first-time participants can complete the two most common Daybook flows without coaching in formative usability rounds.
- Median time to record a common event is at most 30 seconds after the participant's farm profile is set up; validate whether the task is actually common before optimizing it.
- A new operator reaches one useful result within 10 minutes without importing a full farm database or installing a sensor.
- At least 70% of pilot participants voluntarily use a core workflow weekly by week 4; report numerator, denominator, cohort and reasons for drop-off, not just DAU.
- Users can explain, in their own words, why a recommendation appeared and whether it is observed, estimated, forecast, stale or unresolved. Misunderstood high-stakes advice blocks release.

### Offline reliability and interoperability

- 100% of locally confirmed test events survive a 72-hour no-connectivity exercise, device/process restart and later synchronization; any loss is a release blocker.
- Duplicate/replayed events do not create duplicate work, inventory, fulfilment, cost or payment effects.
- Conflicts are surfaced with both source histories intact; no “last writer wins” deletion of evidence without explicit resolution.
- Supported export/re-import round-trip preserves all documented fields, identifiers, units, event times and provenance; private fields are omitted unless explicitly selected.
- Farmer can view current sharing grants, revoke future sharing through the defined protocol, and understand what has already been copied by other recipients. Never promise that revocation can erase a recipient's previously exported offline copy.

### Trust and decision quality

- Every suggestion shows provenance, freshness, major assumptions and a correction/dismiss path.
- Missing/stale/conflicting evidence results in a visible unknown or abstention, not confident fabricated detail.
- Zero automated actuation for the initial slice. Any future physical action needs its own physical-system safety boundary and qualified operating domain.
- Zero unconsented public disclosure of precise farm location, personal data or commercially sensitive records in the adversarial test suite.
- Farmers can keep using the Daybook without enabling Symthaea analysis or sharing with a co-op, buyer, lender or sponsor.

### Economic value

- Establish an operator-specific baseline before the pilot: time spent recording/reporting, travel or machine time for the selected task, rework/error rates and relevant direct costs. Do not make yield or income the sole outcome where the sample/window cannot support attribution.
- Count adoption/setup/training/support costs as well as savings. Report gross and net effects, denominator, observation period, confounders and uncertainty.
- Test whether each pilot participant can identify a recurring concrete benefit that justifies the total cost. A 3:1 benefit-to-cost bar may be tested as a hypothesis based on older selected-region agtech survey findings, but do not present it as a local universal or claim it has been reached until measured.
- Separate modeled savings from realized savings; a scenario result is not ROI.

## Explicit non-goals for v1

- A generic autonomous farm manager that makes operational decisions for the owner.
- Automatic spraying, pesticide/fertilizer application, veterinary treatment, irrigation/pumping, livestock handling, harvesting or machine control.
- Requiring continuous connectivity, proprietary sensors, paid satellite subscriptions, crypto/currency or public-ledger participation.
- An opaque readiness score, generic sustainability score or platform-wide farmer ranking.
- Publishing a farmer's exact location, herd/crop records, inventory, income or buyer relationships by default.
- Claiming higher yield, higher income, lower water use, safety, regulatory compliance or environmental benefit without appropriately designed field evidence.
- Rebranding an unimplemented/pre-alpha workflow as a production application.

## Exit gate

Do not expand the feature set until we have: (1) co-design notes and an explicit problem/wedge decision; (2) a task prototype tested by intended operators; (3) a baseline and preregistered pilot outcomes; (4) an offline/replay and export contract; (5) a privacy/sharing model; and (6) a maturity statement distinguishing mockup, local prototype, synthetic-tested workflow, multi-user-tested workflow and real field evidence.
