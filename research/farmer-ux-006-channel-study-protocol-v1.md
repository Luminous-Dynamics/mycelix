# FARMER-UX-006 — formative cross-channel usability protocol v1

**Status:** Proposed protocol; no participants tested and no results claimed.  
**Contract:** [FARMER-UX-006 / issue #5207](https://github.com/Luminous-Dynamics/mycelix/issues/5207)  
**Prototype:** [Farmer Today PR #5201](https://github.com/Luminous-Dynamics/mycelix/pull/5201)

## Research question

Can a farmer and a human intermediary correctly use a shared request reference across channels while preserving who asserted a fact, who recorded it, what authority they have, and which service states are actually confirmed?

This evaluates interaction comprehension and workflow fit, not operational service quality or agricultural impact. The contract is informed by the 2025 Farm Digital study in Ghana, the June 2026 study of rural African smallholder digital services, and the FAO Digital Services Portfolio; these are motivations for local testing, not a substitute for it.

## Participants and protocol

Run a formative round with approximately 5–8 consenting participants, mixing farmer roles with extension/cooperative/service-provider roles if feasible. This sample is for finding usability failures, not estimating population prevalence. Use actual participant-preferred test languages after qualified human translation review; do not assume one language represents the region.

Use roleplay and synthetic identifiers only. Do not ask for real farm coordinates, farm names, telephone numbers, commercial terms, payment history, or personal identity documents. Recording is optional; anonymous task notes are preferred. Explain that the demo sends nothing to providers or services. Participants can stop at any time.

Where feasible, repeat tasks in app-like, voice/phone, cooperative/extension-assisted, and paper-digitized scenarios. Do not require a participant to own a smartphone or have mobile data to participate. Record device/connectivity constraints descriptively, not as deficits of the participant.

## Tasks and success criteria

### A — Keep one request across channels
Prompt: “You reported an irrigation issue in the app, but now you are speaking to a cooperative worker. Continue without making the situation worse.”

Success: participant carries forward the same logical request reference if known and does not create a second booking/dispatch merely because the channel changed.

### B — Explain the source of each claim
Present one event asserted by the farmer but recorded by an extension worker.

Success: participant can distinguish asserting actor from recording/relaying actor and can say whether onward sharing is authorized. They do not read the agent's record as a direct digital signature from the farmer.

### C — Distinguish relay from provider receipt
Prompt: “The cooperative worker says they passed the message to a provider, but no receipt is visible. What is confirmed?”

Success: participant chooses “relayed/awaiting authoritative provider receipt,” not “provider received,” “quote offered,” or “booking confirmed.”

### D — Reconcile a suspected duplicate
Present two similar request summaries with different references and no verified link.

Success: participant preserves both records, marks them as a possible duplicate and requests permitted human reconciliation. They neither auto-merge nor silently discard one.

### E — Interpret time and paper records
Present a device timestamp that conflicts with server receipt time, and a paper request digitized later.

Success: participant distinguishes claimed capture time from received/digitized/verified time and does not treat client clock order as authoritative.

### F — Respect consent revocation
Prompt: “A farmer asked a worker not to forward the request after all. What happens?”

Success: participant blocks future forwarding, understands why, and does not believe the app should automatically retry through another channel. Previously issued receipts are preserved under the declared retention policy without implying permission for new disclosure.

### G — Handle uncertainty and correction
Present a lost provider response followed by a later corrected event.

Success: participant keeps the original outcome unknown until an authoritative reconciliation exists; correction is appended/linked instead of silently rewriting history.

## Proposed gates (targets, not results)

- At least 90% of participants can explain who asserted the event, who recorded/relayed it, what authority was used, what is confirmed and what remains unknown.
- At least 90% keep the same request reference across a demonstrated channel handoff when the reference is available.
- Zero participants leave believing an unverified relay constitutes a provider acknowledgement or booking. Any such interpretation blocks expansion until the copy/flow is revised.
- Zero unauthorised forwarding in the consent-revocation task.
- Zero silent merges or drops of suspected duplicate requests.
- Track task completion, time, hints, incorrect actions, comprehension, trust/agency feedback and accessibility barriers separately. Do not hide critical failures inside a combined average.
- Report role/language/channel conditions and the exact prototype/fixture versions. Do not claim generalizability from a small formative sample.

## Stop and revise

Stop the scenario immediately if a participant believes real information has been sent, a real provider has been booked, or real personal/farm information was disclosed. Revise the flow if two participants independently make the same critical misinterpretation, if a keyboard-only participant cannot complete the task, or if the authority/consent rule is not understandable in the tested language.

## Claim ceiling

A successful formative round supports only a limited claim about comprehension of this prototype under declared test conditions. It does not establish actual channel adoption, provider availability, secure relay in production, service fulfilment, affordable access, accessibility conformance, or agricultural benefit. Those need separate technical verification and field outcome evaluation.