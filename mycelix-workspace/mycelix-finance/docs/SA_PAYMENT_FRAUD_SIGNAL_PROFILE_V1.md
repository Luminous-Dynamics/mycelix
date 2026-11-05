# South African Payment Fraud-Signal Contract v1

**Status:** synthetic-only evidence and admission contract. It is not a live fraud exchange, legal authority, SARB-endorsed service or adjudication system.

## The architectural shift

This profile models cross-participant fraud intelligence as versioned, source-attributed, purpose-bound assertions, not as a universal score or a status attached to a natural person.

- Signature valid does not mean the assertion is true.
- Sender authenticated does not mean the request is legitimate.
- Suspected does not mean substantiated or officially determined.
- Signal delivered does not mean the recipient is authorised to use it.
- Repeated or mirrored reports do not become independent corroboration.

A signal authenticates the issuer and the exact statement it made. It does not adjudicate a debt, establish guilt, or authorise a payment, account closure, irreversible denial or other external effect.

## Why this matters in South Africa

SARB's PEM position paper proposes a future NPU baseline fraud-protection capability and participant reporting of suspected and actual fraudulent transactions. Its 9 April 2026 dialogue reports the framework had been developed and priority items were being scoped; the PEM working groups list a Fraud and Intelligence Information Sharing Forum. These sources establish programme direction and knowledge exchange, not a production API or general permission for private organisations to share person-level data.

ASAPP's September 2026 policy position calls for lawful purpose, permitted purposes, data quality, correction and appeal, retention and safeguards for information shared in good faith.

## Legal and privacy boundary

FSR Act section 251 addresses information sharing by the SARB and financial-sector regulators for specified statutory purposes, subject to safeguards and written authorised-official procedures. It is not a blanket legal basis for a private technology platform or every payment participant.

POPIA requires minimality, specific lawful purpose, limited retention, compatible further processing, information quality and data-subject access/correction rights. Information about the alleged commission of an offence is special personal information under section 26. Any real person-level sharing needs a controller-specific legal basis, safeguards and specialist South African privacy/regulatory review. This document does not supply those determinations.

## v1 contract

Each signal binds the issuer organisation, role, key and a separately produced verification receipt; an issuer-local opaque subject reference and data classification; event category and event/record times; assertion status, evidence class, evidence references, source-independence groups, qualitative confidence and determining authority; permitted recipients and purposes, prohibited secondary purposes, legal-basis review reference, special-information review, retention deadline and storage scope; plus correction/dispute/withdrawal state and downstream recipient notification counts.

Each recipient query carries a pairwise subject reference bound to the exact recipient plus a separately recorded subject-binding status and receipt. Unknown mappings are quarantined and mismatches are rejected. The query also binds purpose, requested action and evaluation time. The assertion records source-register references for policy/evidence context. Correction lineage identifies prior recipient IDs and notified recipient IDs, not only counts, so incomplete propagation is detectable.

The evaluator verifies structural and semantic consistency against synthetic fixtures. It does not perform production cryptographic signature verification or legal-basis review. The reviewed_for_fixture markers are test data, not legal approvals.

## Data minimisation and storage

- No raw account numbers, national IDs, cellphone numbers, credentials, full statements or identity attributes are present in the corpus.
- Subject references are issuer-local or recipient-pairwise opaque tokens. They are pseudonymous fixture identifiers, not anonymous data.
- Public-DHT storage is rejected. Hashing a subject or document must not be assumed to anonymise it.
- Identifiable evidence remains with its responsible participant unless a documented, purpose-specific disclosure is separately authorised.
- Data-subject contest/correction restricts use pending review. Correction, withdrawal and supersession must notify every prior recipient represented by the receipt ledger.
- Historical records can remain for audit under an applicable retention rule, but cannot remain an unrestricted current warning.

## Evidence and status rules

- Suspected is a report or observation, not a finding about a person.
- Corroborated requires separate source-independence groups; duplicate and mirrored copies count once.
- Issuer-substantiated records the issuer's own process, not a legal adjudication.
- Official determination requires an identified determining authority and court/official evidence class.
- Model-derived indicators remain model-derived; they cannot be relabelled as direct source observation or official determination.
- EU Verification of Payee match results and UK aggregate reimbursement results are separate comparator domains, not fraud-guilt assertions.

## Action boundary

Signals may support bounded advisory actions within the selected participant profile: a warning, step-up authentication, manual review or opening a case. They do not themselves authorise payment execution, settlement, account closure, irreversible denial, credit scoring or a legal-guilt label. Those actions require independent policy, authority, evidence and due process.

## Benchmark gates

- 100% of admitted signals bind issuer, source scope, evidence class, status, time window, recipient and permitted purpose.
- 0 invalid/unknown keys or issuer scopes silently admitted.
- 0 FSR Act section 251 used as a private platform's blanket sharing permission.
- 0 special-personal-information signal admitted without its separately reviewed synthetic fixture gate.
- 0 wrong-purpose or wrong-recipient disclosures.
- 0 raw identifiers or public-DHT signal storage.
- 0 contested, corrected, withdrawn, expired or superseded signals used as current.
- 100% correction/withdrawal notices reconcile to prior recipients.
- 0 mirrored source groups counted as independent corroboration.
- 0 single suspected signal promoted to official determination or irreversible adverse action.
- Deterministic replay of every frozen signal, policy and recipient query.

Operational studies must additionally report signal propagation time, independent corroboration, precision/recall/calibration on independently matured outcomes, false-positive harm, appeal/correction reversal rates, stale-signal use, downstream correction latency and data-exposure surface. Denominators, base rates and case maturity must be shown. Alert volume and raw decline rates are not success metrics.

## Industry comparators

1. Participant-local fraud operations are the baseline.
2. A participant-authorised shared feed can be compared only where actual operating rules and data access are documented.
3. Mycelix is evaluated as the evidence/purpose/correction layer, not as a competing fraud-scoring authority.

EU Verification of Payee is a name/account match service before payment initiation; UK APP reimbursement is a downstream customer-outcome/liability model. Neither is directly comparable to South African transaction-level fraud-intelligence sharing.

## Composes with existing Mycelix work

- #5777/#5778 own authenticated signal ingress and wire/decoder semantics.
- #5798 owns assertion-specific evidence relevance and reliability.
- #5215/#5220 own operation-scoped source-use admission and authenticated receipts.
- #5392 owns typed source freshness, uncertainty and dispute projections.
- #2437 owns policy/proof/execution authority separation.
- #5766 owns the payment recourse case and actual-return lifecycle.
- #5767/#5768 own Request-to-Pay authenticity, payer safeguards and non-coercion.

No competing trust engine, global identity graph, legal source registry or universal risk score is introduced.

## Claim ceiling

The v1 corpus contains 36 synthetic cases, including wrong-recipient pairwise-token substitution, unknown or mismatched recipient subject bindings, and a proposed-but-not-operational NPU profile that must remain quarantined.

Passing the corpus proves only the exact synthetic schema/evaluator behavior. It does not establish a lawful basis, POPIA/FSR compliance, actual fraud-detection accuracy, a national fraud service, SARB endorsement, provider liability or real-world consumer benefit. Live exchange requires participating controllers, current rules, data-sharing agreements, privacy impact assessment and specialist legal review.
