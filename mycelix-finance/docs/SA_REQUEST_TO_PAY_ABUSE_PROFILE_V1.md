# South African Request-to-Pay Abuse and Requester Authenticity Profile v1

**Status:** synthetic, effects-disabled benchmark. This is not a live fraud detector, legal debt adjudicator, merchant accreditation system or provider integration.

## Paradigm shift: verify claims independently, not "trust" the request

The useful unit is not one requester reputation score. It is a chain of independently qualified claims:

`sender session authenticated`
`!= requester identity established`
`!= authority to collect for beneficiary`
`!= commercial claim substantiated`
`!= payer saw the exact material request`
`!= payer voluntarily accepted`
`!= payment instruction created`
`!= funds available / settled`

A cryptographically valid session can be compromised. A displayed name can be a trading name. A bank account-name match can be true while an invoice is false. A merchant can be genuine but lack authority to collect for the named beneficiary. A valid debt may still be disputed or not due. These claims must stay separately inspectable.

## Source-grounded current design observations

Capitec's PayShap Request FAQ (observed 4 November 2026) says the sender name and reference are displayed, only logged-in app users can send/approve, recipients can decline unknown requests without money being taken, and the requester can make two requests per 24 hours under its product guidance. That is a provider-specific product observation, not proof that the sender's display name is a legal identity or has authority to collect.

EPC's Verification of Payee v1.1 became effective 20 September 2026. It compares account identifier and payee name before eligible SEPA credit transfers and can return match, close match, no match or unable-to-check. EPC expressly states VOP messaging cannot be relied upon to identify a private or legal person. EPC's matching guidance v2.0, published 3 November 2026, addresses character sets and name variation including initials, spelling, phonetic and commercial-name variants. This is useful SOTA for account/name mismatch handling, not a proof of invoice legitimacy.

Pay.UK's Request to Pay framework is a useful user-control comparator: it describes query, pay part, pay later and decline as options. It is not a South African rule or an authority to disregard an underlying commercial obligation.

## Threat model

- Authenticated but compromised merchant credentials.
- Sender display-name impersonation and lookalike ShapIDs.
- Bank-account name match without verified legal-person identity or collector authority.
- Invoice/reference substitution, stale or fabricated commercial evidence.
- False debt-collection or legal-threat wording.
- Repeated requests after decline; unsolicited floods.
- Intermediary/agent lacks authority for the beneficiary.
- Report/decline/blocked events misinterpreted as payment instructions.
- Request metadata or transaction history reused for unrelated credit scoring, marketing or disclosure.
- Screen-reader, feature-phone and translated presentation omits material beneficiary/amount/fee/expiry fields.
- AI summary silently drops material terms.
- False-positive blocks excluding informal and small merchants without correction or appeal.

## Evidence graph

Each request should keep separate evidence nodes for:
1. transport/session authentication and credential status;
2. account identifier / displayed-name matching;
3. requester legal/merchant identity (where available);
4. explicit authority to collect for the beneficiary;
5. invoice/order/contract evidence and its provenance/currentness;
6. exact human-visible request presentation and accessibility profile;
7. payer query, decline, report, block or acceptance;
8. actual payment instruction and payment outcome.

No node should upgrade a weaker node. A name match does not mint an authority receipt; an invoice reference does not prove a debt; sender authentication does not prove that a person intended this exact request. Reuse typed receipts from LEX-NET #1215 and exact human-visible confirmation from #1336 rather than inventing another generic `verified=true`.

## User sovereignty and anti-coercion

Decline, query, report and block must remain safe actions. Do not tell the payer that declining changes the underlying legal obligation. Do not punish a payer or disclose the reason for decline to the requester unless the user chooses. A repeat after decline is an abuse signal and may warrant a user-controlled block/review; do not claim that one decline creates a universal scheme prohibition unless the participant rule says so.

A source-backed rate limit is participant/product/channel scoped. Capitec's stated two requests per 24 hours is not to be copied to other banks. For unknown profiles, record `unknown`; do not hard-reject with an invented limit.

Small and informal merchants need a correction/appeal path. A false positive should not become permanent exclusion or a public global blacklist. Do not expose a global public sender/payer reputation ledger.

## Benchmark corpus

The companion machine-readable corpus has 24 synthetic cases covering: displayed sender identity vs legal identity, account-name match vs collect authority, wrong beneficiary, missing/stale invoice evidence, unsupported legal threats, repeat after decline, source-backed volume-limit breach, suspected credential compromise, confusable aliases, ambiguous payee binding, pending user reports, decline, secondary data use, accessibility omissions, small-merchant appeal, AI-summary omission, unauthorised intermediary, payer query and unknown-participant limit.

## Qualification gates

Hard safety and epistemic gates:
- 0 sender-authentication results promoted to requester legal identity;
- 0 account-name matches promoted to collect authority or invoice legitimacy;
- 0 instructions from pending, queried, declined, reported or blocked requests;
- 0 mismatched/ambiguous/confusable beneficiaries silently accepted;
- 0 unsupported debt/legal assertions described as verified;
- 0 source-backed rate limits applied to another participant;
- 0 unknown limits converted into hard rejections;
- 0 secondary use outside declared purpose;
- 100% appeals and reports retain explicit dispositions;
- 100% required material fields preserved across supported accessible presentations;
- 0 unsubstantiated automatic "legitimacy" claims.

Outcome metrics, reported separately:
- impersonation / wrong-beneficiary catch rate;
- false-positive block and appeal reversal rate;
- confirmed scam/request abuse per 1,000 displayed requests (with independent label source);
- repeat-after-decline exposure;
- report-to-containment p50/p95/p99;
- payer comprehension and decision confidence calibration;
- accessibility semantic parity;
- merchant exclusion rate by merchant size/formality;
- complaint/support burden;
- data minimisation and secondary-use violations.

Do not make acceptance rate the objective. Higher acceptance can mean worse coercion, worse fraud or simply weaker decline UX.

## Industry comparison and SOTA target

Benchmark three distinct layers:
- PayShap Request's provider-specific sender-display, logged-in app, decline and source-backed limit behaviours.
- EPC VOP v1.1 + matching guidance v2.0 for account/name mismatch detection and character handling.
- Pay.UK's communication and payer choice options.

Mycelix should target a better *claim decomposition and recourse graph*, not claim that it has better fraud prediction without labelled real traffic. A valid evaluation requires a frozen, privacy-minimised dataset, adjudicated ground truth, independent reviewers, false-positive analysis for small merchants and a clear appeals pathway.

## Claim ceiling

All cases are synthetic. Passing demonstrates only that the schema and reference evaluator reject seeded claim-laundering errors. It does not establish a merchant's honesty, debt validity, payer comprehension, legal entitlement, commercial authority, real fraud detection or safe real-money deployment.


## Risk-observation integrity and lifecycle

Each abuse observation is bound to one synthetic requester subject, purpose, evidence issuer class, evidence state, freshness and appeal state. Observation history is append-only in the benchmark: a refuted/overturned observation remains visible as refuted history, never silently revived as an active signal. Stale, unknown, unbound or cross-subject observations cannot increase a claim ceiling or be applied to another merchant/account/credential.

The `evidence_commitment` is SHA-256 over a canonical compact JSON projection of the observation with the `evidence_commitment` field omitted. Canonicalisation recursively sorts object keys lexicographically, preserves array order, uses UTF-8 JSON strings and emits no whitespace. The Rust evaluator uses serde_json's default key-sorted object map and verifies the digest before evaluating the observation. This provides content-integrity/tamper detection only; it is **not** a signature, issuer authentication, trusted timestamp, legal finding, or proof that the observation is true.

For a real deployment, issuer authentication would require signed provenance under an authorised participant key, key rotation/revocation/currentness, replay protection, privacy/purpose controls, retention and deletion rules, due-process appeal, and independent adjudication. None of those real-world guarantees are established by the synthetic hashes in this corpus.


## 2026-11-04 Verification of Payee comparator update

The abuse benchmark now includes an EU-SEPA VOP v1.1 comparator profile and cases for `match`, `close match`, `no match` and `verification not possible`.

EPC VOP v1.1 became effective on 20 September 2026. EPC matching recommendations v2.0 were published 3 November 2026 and address character sets, spelling/transposition, initials, phonetic variants, commercial names and abbreviations. EPC explicitly states that VOP messaging cannot be relied upon to identify a private or legal person. Its FAQ says the payer must be informed of a no-match/unavailable result and retains the decision whether to proceed in the SEPA credit-transfer flow.

This is an architecture comparator only. The benchmark uses it to make four distinctions:
- `match` establishes an account/name comparison, not legal identity, invoice legitimacy or authority to collect;
- `close match` remains a bounded review outcome, not automatic equivalence;
- `no match` is an explicit warning and not, by itself, a universal proof of fraud or an automatic payer block;
- `verification not possible` is indeterminate, not a mismatch.

The EU scheme's user-choice and operating rules are not transferred to South Africa. Mycelix records the result, scheme/version, warning and payer-choice state separately. A bank's local rules and legal obligations must be qualified independently.

Sources:
- [EPC VOP scheme overview](https://www.europeanpaymentscouncil.eu/what-we-do/other-schemes/verification-payee-vop)
- [EPC VOP scheme rulebook v1.1](https://www.europeanpaymentscouncil.eu/document-library/rulebooks/verification-payee-scheme-rulebook-0), effective 20 September 2026
- [EPC recommendations for VOP matching v2.0](https://www.europeanpaymentscouncil.eu/document-library/guidance-documents/epc-recommendations-matching-processes-under-vop-scheme), published 3 November 2026
- [EPC FAQ on no-match and verification-unavailable](https://www.europeanpaymentscouncil.eu/faq/verification-payee-scheme/rulebook/what-should-psp-do-not-do-when-no-match-or-verification)
