# Farmer Today prototype — formative usability protocol v1

**Tracking:** [FARMER-UX-003](https://github.com/Luminous-Dynamics/mycelix/issues/5200)  
**Prototype PR:** [#5201](https://github.com/Luminous-Dynamics/mycelix/pull/5201)  
**Status:** Proposed formative evaluation, not performed. No user results are claimed.  
**Scope:** Mycelix Commons `/farmer` Today page and its simulated irrigation-service walkthrough.

## 1. Research question

Can a farmer or agricultural-service participant understand the first Farmer Today screen, distinguish reported/modelled/unknown information from verified facts, and correctly explain what has and has not happened at each service-lifecycle step?

The prototype uses synthetic content and local in-page signals only. It does not write to Holochain, save a farm record, retrieve location/weather/satellite data, contact a provider, create a booking, invoke AI, or trigger equipment. The evaluation must not be described as a farm pilot or as evidence of agricultural outcomes.

## 2. Paradigm being tested

The core hypothesis is **decision-to-outcome beats dashboard-first**: people should be able to identify the next useful action and its evidence limits without needing to interpret a chart or know an internal system.

A second hypothesis is **explicit state language reduces false assumptions**. “Draft”, “request received”, “quote received”, “booking confirmed”, “provider reports completion”, “farmer accepts”, and “dispute/rework” should be understood as distinct states.

Research motivations:
- FAO's Digital Services Portfolio is adapted to local agricultural needs and includes advisory, weather/crop calendar, livestock and market information; its implementations are context-specific rather than a universal interface template: https://www.fao.org/digital-services/about/
- The 2026 *Field-Ready HCI* paper proposes (rather than empirically validates) a design model covering local language, low-literacy use, offline operation, explainable advice, trust signals and farmer/extension networks: https://doi.org/10.3390/app16146985
- WCAG 2.2 AA is the baseline to review applicable web interface accessibility; larger primary touch targets are an additional design goal. https://www.w3.org/TR/WCAG22/

These references motivate tests; they do not prove that this page fits local farmers.

## 3. Participants and limits

For the first **formative** round, recruit approximately 5–8 consenting participants, if feasible, from more than one relevant role: smallholder or market gardener, mixed/livestock farmer where available, extension/advisory worker, and irrigation or farm-service provider. This sample is for finding usability failures; it is not statistically representative and cannot support prevalence or agricultural-impact estimates.

Record only participant-chosen role grouping, device class, preferred test language, approximate experience band where volunteered, and task outcomes. Do not collect real field coordinates, production data, farm identity, financial records, or unnecessary personal information. Recording is optional; prefer anonymized notes. Make participation voluntary, provide a clear stop option, and do not connect the prototype to personal accounts or live services.

Before testing with non-English users, have native/qualified speakers review and translate the copy. Do not use unreviewed machine translations to claim localized comprehension.

## 4. Test setup

- Use a declared viewport and device/browser build; include at least one narrow mobile viewport around 360–390 CSS px and one larger screen.
- Where possible, test on an entry-level Android device and a constrained-network profile. Since this prototype does not implement persistence or offline sync, observe the **interface's disclosures**, not claimed offline correctness.
- Start at `/farmer`. Do not explain where to click; read the task prompt and observe.
- Before each session state: “This is a prototype with sample information. No actions will be sent to a real farm, provider or account.”
- Test keyboard-only operation for all visible actions, visible focus, zoom/reflow, and whether status meaning depends on color alone.
- Never ask the participant to enter real farm information. Use a made-up irrigation observation.

## 5. Tasks and success criteria

For every task, record unaided completion (yes/no), elapsed time, incorrect assumptions/actions, requests for help, and a short comprehension response. Do not count a fast action as successful if the participant misunderstood its consequence.

### Task A — Identify what needs attention
**Prompt:** “You have arrived at this screen. Tell me what you think needs attention first and what information supports that.”

**Success:** Participant identifies the irrigation report and says that its cause is not verified; understands that the soil-moisture reading is missing/unknown rather than zero or safe.

### Task B — Capture a problem preview
**Prompt:** “Describe the irrigation issue using this prototype. Then explain where that information went.”

**Success:** Participant selects/enters sample information and opens the preview; correctly states that it remains a demo preview and was not saved or sent. This is a critical comprehension gate.

### Task C — Distinguish the service states
**Prompt:** “Walk through the sample service journey. At each step, tell me what has actually been confirmed in the real world.”

**Success:** Participant distinguishes:
1. demo draft from a recorded request;
2. simulated request receipt from real provider acknowledgement;
3. quote received from booking confirmed;
4. booking from payment or work completion;
5. provider-reported completion from farmer acceptance;
6. acceptance from a dispute/rework outcome.

### Task D — Choose the outcome
**Prompt:** “The provider says the repair is complete, but you are not sure the water is flowing again. Show what you would do.”

**Success:** Participant identifies the accept/dispute choice and selects dispute/rework for the scenario; understands that the prototype does not adjudicate fault and that the state is simulated.

### Task E — Find an existing service area
**Prompt:** “Show where you would go to explore food-cooperative resources and water resources.”

**Success:** Participant reaches the Food and Water routes without losing the current task or assuming that those routes contain farm-specific live data.

### Task F — Recover and explain uncertainty
**Prompt:** “Which part of this screen is a verified measurement? Which part is a report? What is not available?”

**Success:** Participant correctly names reported flow, missing soil moisture, and unavailable weather/live data. No sample text is mistaken for a real observation or current forecast.

### Task G — Recover safely after a lost response
**Prompt:** “The app shows that a send was attempted, but the response was lost. You do not know whether the provider received it. What would you do next?”

**Success:** Participant recognizes that the outcome is unknown—not confirmed success and not confirmed failure—and does not assume the app should automatically send a duplicate. They choose a status-reconciliation path or a human-assisted channel, and understand that the walkthrough is still a simulation.

## 6. Proposed acceptance gates

These are release hypotheses, not current results:
- At least 90% of participants in the validation round complete each critical task without facilitator intervention after brief orientation.
- At least 90% distinguish a sample report, missing evidence, a simulated receipt and a confirmed agreement in the comprehension check.
- At least 90% choose reconciliation or human help, rather than assuming automatic resend, after the simulated lost-response/OutcomeUnknown branch.
- Routine problem-preview time has a target median under 30 seconds after brief orientation; also report P90, error rate and omitted information.
- Zero participants leave a session believing the prototype sent a request, booked a provider, authorized payment or verified completed work because of misleading UI wording. Any such interpretation requires copy/interaction revision before a broader pilot.
- Every critical task is keyboard-operable, focus is visible, interactive targets meet the designed size, zoom/reflow does not hide actions, and status is not conveyed by color alone.
- Report every participant's task success and failures; do not hide failures inside a single average. The small formative round cannot establish generalizable statistical claims.

## 7. Stop/revise rules

- **Immediate stop and fix:** a participant reasonably believes that a demo action sent a real request, disclosed a real location, accepted a real quote, made a payment, activated equipment, or verified a real repair.
- **Revise the screen/copy:** two or more participants make the same critical misinterpretation; a critical action cannot be found without a hint; a keyboard-only user cannot complete a task; or content is clipped at the declared narrow viewport.
- **Do not proceed to outcome claims:** usability success does not qualify agronomic advice, synchronization reliability, route accuracy, service fulfilment, income/yield changes, water savings, or causal impact.

## 8. Report format

Publish an anonymized report containing:
- exact prototype commit SHA and browser/device/viewport profile;
- language reviewed and whether the translation was human-reviewed;
- participant role mix and sample size (no unnecessary identifiers);
- per-task completion, time summaries (median/P90 where meaningful), error categories, assistance and comprehension;
- accessibility issues and their severity;
- decisions made from findings, unresolved issues, and whether acceptance gates were met;
- explicit claim ceiling: formative prototype usability only; no field or causal impact claim.

Keep this protocol versioned. Any substantive screen or copy changes between sessions require the tested commit to be recorded so results remain reproducible.
