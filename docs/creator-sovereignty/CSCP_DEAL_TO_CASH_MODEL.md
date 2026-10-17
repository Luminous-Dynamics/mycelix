# CSCP Deal-to-Cash and Repeat-Booking Model

**Status:** Product/architecture proposal; no production implementation or measured impact claimed.
**Date:** 2026-10-17
**Parent:** [Creator Sovereignty Control Plane RFC](CREATOR_SOVEREIGNTY_CONTROL_PLANE_RFC.md)
**Companion:** [Creator Resilience and Creative Optionality](CSCP_CREATOR_RESILIENCE_MODEL.md)
**Music/rights extension:** [Creative Asset Passport and Rights Graph](CSCP_CREATIVE_ASSET_PASSPORT.md)

## Executive thesis

A potentially more defensible first wedge than generic creator analytics is helping creators manage the journey from a commercial agreement to cash actually received—and learn whether a one-off campaign becomes a repeat relationship.

The core workflow is:

**Deal proposed → terms reviewed → agreement approved → work delivered → deliverable accepted → invoice/payment claim submitted → due date → payment received → reconciled → repeat booking (or an explicitly unknown reason for no repeat).**

Each state needs evidence and explicit authority. The system should make delays, missing approval, unanswered acceptance, unresolved disputes, unpaid receivables, and repeat-booking patterns visible. It should not infer that a creator is legally entitled to payment merely because a workflow says that work was delivered; contract terms and human decisions remain authoritative.

This is deal and payment tracking, not payment custody, factoring, credit, escrow, legal practice, or a promise of fast settlement.

## Why research supports testing this wedge

### South African creator market

Humanz's *Paid to Create: The South African Creator Economy 2026* combines a survey of nearly 1,200 South African creators—including 243 verified paid creators—with more than R100 million in creator payments made through the Humanz platform over twelve months. It reports that:
- income predictability averaged 2.1/5 in the sample;
- 55% of paid creators waited more than a month to be paid or had to chase payment;
- about half of creators who received a paid campaign were not paid for a second time during that year;
- repeat campaign bookings strongly correlated with greater annual campaign earnings.

These results are from Humanz's platform-associated research; its payment data covers Humanz campaign fees, not all creator income, and its sample is not a census of South Africa's creator market. Treat the findings as a useful user-research hypothesis, not a universal rate or causal effect.

Source: [Humanz, Paid to Create: The South African Creator Economy 2026](https://www.humanz.com/south-african-creator-economy-report-2026/).

### Payout control across creators, contractors, and music earners

Trolley commissioned Global Surveyz to survey 450 respondents across 19 countries in May–June 2026: 150 contractors, 150 creators, and 150 music/royalty earners. Its published highlights report that 82% wanted more say in payout timing while 73% lacked that control; 41% cited payment delays as a reason they had left or would leave a platform. Since Trolley sells payout infrastructure, treat this as vendor-associated survey evidence, not proof that changing payout controls alone creates creator prosperity.

Source: [Trolley, How Independent Earners Want to Get Paid (2026)](https://trolley.com/resources/recipient-report-2026/).

### Cross-platform creators already operate a creator ecology

A CHI 2023 qualitative study interviewed 21 creators about multi-platform work. It describes platform prioritization, synchronization, audience transfer, and audience management as strategic work performed by creators. The sample is small and qualitative; it describes practices rather than estimating how common they are across all creators.

Source: [Multi-Platform Content Creation: The Configuration of Creator Ecology (CHI 2023)](https://doi.org/10.1145/3544548.3581106).

### Deal fit and audience trust are economic constraints too

CreatorIQ and Influencers.club's August 2026 survey of 5,095 creators across 100 regions reports that 42% of creators experience tension between the content their audience wants and the content brands request; the report says the share rises to 53% among creators with 500,000+ Instagram followers. This is survey evidence from a creator-marketing vendor and its research partner, not an independent causal estimate. It supports a design hypothesis: a deal can be financially attractive but conflict with a creator's voice, audience expectations, license boundaries, or future plans.

Source: [CreatorIQ, State of Creators 2026](https://www.creatoriq.com/press/releases/creatoriq-state-of-creators-report-2026).

The system should model **creator-defined fit constraints**, not a universal "brand fit score". Before a creator approves terms, let them record non-negotiables such as prohibited categories, required disclosure, permitted usage channels/duration, exclusivity, edit approvals, minimum compensation, and whether the deal requires a synthetic voice/likeness or use of unpublished material. Show financial terms and these constraints separately. A strong fee cannot override a prohibited category or unapproved rights scope.

**Research conclusion:** deal-to-cash workflow visibility and repeat-booking evidence are worth testing. These sources do not establish demand for our product, that it will reduce time-to-payment, or that a reminder increases a buyer's willingness to pay.

## Product principle: separate contract expectations from observed cash

Maintain two distinct ledgers/views:

1. **Obligations and milestones:** creator-confirmed agreement version, deliverables, approval criteria, due dates, fee/split, permitted usage and term, invoice/claim references, and dispute/approval state.
2. **Cash events:** money received, fees, refunds, reversals, currency, source, date, and reconciliation status.

Do not treat an agreement fee as income received. Do not treat an expected payment date as an actual event. Do not use unsigned or superseded terms as the current contract. If the source does not specify an enforceable date or condition, display it as missing rather than inventing one.

A deal's **financial value, creator-defined fit, and audience/creative trade-offs are distinct fields**. The tool may show the trade-off but must not hide it inside one opaque recommendation score. If a creator's explicit constraint conflicts with the deal, flag the conflict and require changed terms or rejection; do not let projected reach or revenue override it.

Every object should carry a stable local ID and schema version, source reference and source-recorded timestamp, creator-confirmed status and approval evidence, exact currency/amount where known, event time versus observation/import time, correction lineage, provenance/completeness status, permitted reuse scope, retention expiry, and deletion status.

## State model and transitions

### Commercial deal

Candidate states: Draft, TermsProposed, TermsAccepted, InProgress, Delivered, AcceptancePending, Accepted, Disputed, Cancelled, and Complete.

- Only the creator or an explicitly authorized role can record their acceptance of terms or a rights grant.
- A delivered file is evidence of delivery, not proof of acceptance.
- An acceptance criterion may be unknown or disputed; the application must not silently advance state.
- A later agreement version supersedes the old one only with recorded approval; history is not rewritten.
- The application must preserve whether approval was creator-entered, imported from a trusted source, or derived from a weak inference.

### Invoice or payment claim

Candidate states: NotRequired, Ready, Submitted, Due, PartiallyPaid, Paid, OverdueAccordingToRecordedTerms, Disputed, WrittenOff, and Cancelled.

- Overdue is only permitted when due-date evidence and relevant conditions are recorded; otherwise use DueDateUnknown.
- Partially paid claims must reconcile payment events to the recorded claim without counting them both as invoice value and as cash income.
- Payment received does not automatically resolve a dispute about usage rights or deliverable quality.
- If there are multiple invoices, retain separate obligations and payments rather than coercing the whole deal into one amount.
- No outbound demand, negotiation, contract acceptance, collection action, or payment transfer without human approval.

### Cash receipt and reconciliation

- Actual receipts come from creator-entered evidence or an authorized imported transaction.
- Keep the raw source amount/currency and payment reference.
- Duplicate source transactions must be idempotent; reuse of an ID with changed amount/currency must fail closed.
- Matching a cash event to an invoice is a separate reconciliation decision; don't assume a matching amount proves which invoice it settles.
- Refunds, reversals, fees, currency conversions, and split payouts have distinct types and provenance.
- If a split is unapproved or its sum does not match the agreement, show an unresolved conflict; never infer the split from reputation or contribution sentiment.

### Repeat booking

Represent the next engagement as a new deal linked to the prior campaign only when a source or human supplies the relationship. Do not call repeat booking proof of creative quality or a causal effect of the system.

Record reasons for no repeat only when explicitly supplied; use Unknown rather than inventing a narrative from silence.

## State-machine prototype: evidence, transitions, and replay

A first draft implementation exists in [PR #5179](https://github.com/Luminous-Dynamics/mycelix/pull/5179). It is a standalone Rust research crate, separate from the broader creator ledger and scenario engine.

The proposed workflow enforces:
- a first terms version followed by strictly increasing versions;
- both distinct parties explicitly approving the same current version before work starts;
- clearing both approvals when terms are revised;
- creator-only start/submission transitions and sponsor-only accept/reject transitions;
- matching acceptance to the exact pending deliverable ID;
- sequential delivery count and final-deliverable semantics;
- unique event IDs, idempotent identical replay, rejection of conflicting reuse, and monotonic event time;
- atomic transition failures that do not append a rejected event;
- replaying the event history to reconstruct state rather than trusting a persisted status field.

The prototype currently contains 14 source-level unit tests. The Rust toolchain is unavailable in the current editing environment, so those tests remain uncompiled and unexecuted; no passing result is claimed.

This state machine **does not prove that the referenced contract or delivery evidence is authentic**. Party IDs and evidence references are caller-supplied; cryptographic identity, source integrity, approval signatures, durable serialization, invoices, and cash settlement remain separate work. Accepted is not equivalent to paid.

## Candidate architecture across the four projects

- **Mycelix:** later, permissioned claims for collaboration, approval history, agreement versions, and portable provenance where a distributed claim adds clear value. Private contract terms, bank details, invoices, payment evidence, and audience data should remain private by default. Public DHT presence is not the default.
- **Symthaea:** summarize current workflow state and point out missing evidence or next steps with source links. Suggestions remain advisory. Never auto-send a payment demand, accept terms, or alter contract facts.
- **Symtropy:** optional later support for production/time-allocation simulations or interactive sponsor experiences. Not required for an MVP.
- **Sol-Atlas:** optional later support for location-aware events, tours, and sponsor activations. No individual fan geolocation or inferred demand without a defensible, consent-compatible basis.

The first implementation can be a local-first Rust record model and deterministic status engine. No integration needs to custody funds or call platform APIs to provide basic deadline and reconciliation visibility.

## Benchmark protocol

All targets below are proposed internal gates, not achieved results or legal standards.

| Metric | Proposed gate | Measurement definition / anti-gaming |
|---|---|---|
| Obligation-to-cash traceability | 100% of supported claim/cash records have provenance and an explicit state. | Unknown/incomplete states are counted, not dropped; cash is never inferred from an agreement. |
| Duplicate money records | Zero duplicate recognition of the same source transaction in golden and hostile fixtures. | Idempotency key/source reference and conflict-on-identity-reuse; independent reconciliation. |
| Invoice/due-date integrity | Zero fabricated due dates or unsupported overdue assertions in the adversarial set. | Missing acceptance conditions or due-date terms produce Unknown, not Overdue. |
| Rights correctness | Zero unauthorized rights transfers or broadened usage terms. | Separate permissions to analyze, store, publish, train on, and license material; human approval required. |
| Workflow visibility | 100% of open supported deals show the current recorded stage and next missing field/action. | Label stale imported evidence, unknown states, and data-source limitations. |
| Time-to-cash measurement | Report median, p75, and p90 elapsed time from creator-confirmed acceptance/deliverable condition to reconciled actual cash receipt. | Compute only where both endpoints are evidenced; stratify by source, contract type, currency, and period. |
| Overdue exposure | Show amount and count of claims past documented due date by currency. | Distinguish amount billed, accepted, disputed, received, and written off; never present unpaid agreement value as cash. |
| Repeat-booking rate | Report cohort-based repeat paid deal rate with explicit first-deal and follow-up windows. | Denominator is eligible first-time paid relationships with complete observation periods; censor incomplete follow-up and report sample size. |
| Admin effort | Exploratory target of 30% lower median creator-reported time on deal tracking/follow-up. | 4-week baseline then 8–12-week pilot; include setup burden and failed imports. Do not claim causation from uncontrolled data. |
| Creator usefulness | Median ≥4/5 for comprehension of current status and next step in design-partner testing. | Report full distribution and failure cases. |
| Payment outcome | Measure actual time-to-cash before and after adoption; no initial target claiming the software shortens buyers' payment cycles. | Reminders may improve follow-up visibility but cannot control buyer liquidity, disputes, banks, or contract enforcement. |
| Privacy/authority | Zero unauthorized messages, disclosures, collections, contract acceptances, rights changes, or transfers. | Fail-closed tests for revoked access, stale sources, conflicting approval, and adversarial identities. |

## Synthetic adversarial cases

1. A creator records a deal amount but has not delivered work. It must not appear as received income.
2. Work was delivered but acceptance is pending. Do not label the invoice overdue unless recorded contract conditions establish that state.
3. A deal is amended; old rate/rights terms remain in the audit trail but cannot silently override newly approved terms.
4. The same transaction is imported twice; totals remain unchanged.
5. An invoice is paid in two installments. Reconciliation shows paid-to-date and outstanding balance by currency.
6. Payment arrives with an unexpected fee or FX conversion. Preserve source evidence and show the discrepancy.
7. One payment could match two equal-value invoices. Ask for allocation or leave it unresolved.
8. A source reuses a payment ID with changed amount/currency. Fail closed.
9. A creator revokes a collaborator's permission after delivery. Past valid provenance remains auditable without granting future access.
10. A contract permits one usage term but an AI draft proposes a broader license. Block until terms match human-approved scope.
11. A creator sends a reminder but receives no response. Do not infer bad faith, legal breach, or lack of future opportunity.
12. A necessary due date or contract condition is missing. Mark status unknown.
13. A first-time deal has no complete follow-up period. Exclude it from the repeat-booking denominator until the window matures, while showing it in the total cohort.
14. Partial coverage is marked as all deals. The verifier rejects the false completeness claim.
15. A creator works in multiple currencies. Keep receivables, payments, and outstanding amounts separate unless a documented FX policy is chosen.

## Delivery sequence

1. Freeze state/authority semantics and adversarial fixtures.
2. Add local-first versioned records for deals, approved terms, invoices/payment claims, and actual cash events; leave payment execution out of scope.
3. Add an independent checker for money totals, state transitions, approved version lineage, and duplicate source events.
4. Add reconciliation, due-date visibility, and cohort-based metrics with explicit unknown handling.
5. Run usability tests on synthetic fixtures before importing real contracts or invoices.
6. Pilot with consenting creators and manually entered/imported deal records; measure baseline tracking time, follow-up burden, actual days-to-cash, and repeat booking.
7. Consider Mycelix permissioned claims only where creators demonstrably need portable collaborator/approval evidence.

## Go/no-go rule

Do not advance this feature if creators cannot distinguish expected payment from money received, if the interface can create unsupported debt/overdue claims, or if automation can change terms, contact a buyer, disclose private contract data, or move money without required explicit approval.
