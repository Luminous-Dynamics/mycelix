# South African Payment Decision Integrity Study v1

**Status:** development protocol only. No participant study is approved or underway. No live payment or real financial data is used.

## 1. Research question

Does an evidence-qualified payment-request presentation improve a payer's ability to understand the material facts and choose an appropriate voluntary action, compared with a minimal request presentation, without increasing false alarms against adequately evidenced legitimate requests or creating undue pressure?

The goal is not to maximize acceptance, rejection, friction or a synthetic fraud score. It is decision quality under uncertainty.

## 2. Evidence basis and scope

SARB's February 2026 *National Payment System Vision 2030+ Consultation Paper*, section 15, explains that APP fraud can involve an authorised payer being deceived into initiating a valid payment. It states that effective digital identity and fraud-resistant infrastructure do not alone prevent this category. VOP, transaction-risk monitoring, user education and customer-protection frameworks may assist. The paper is a consultation paper, not a concluded policy:
https://www.resbank.co.za/content/dam/sarb/what-we-do/payments-and-settlements/consultation-documents/2026/vision-2030--/NPS%202030%2B%20Consultation%20Paper%20-%2024%20February%202026.pdf

EPC VOP Rulebook v1.1 became effective 20 September 2026. EPC matching recommendations v2.0, published 3 November 2026, address character sets, transliteration, initials, spelling, phonetic variation and commercial names. A payee match is a limited verification signal, not proof of legal identity, collection authority or claim legitimacy:
https://www.europeanpaymentscouncil.eu/document-library/rulebooks/verification-payee-scheme-rulebook-0
https://www.europeanpaymentscouncil.eu/news-insights/news/epc-publishes-updated-recommendations-matching-processes-under-vop-scheme

SARB's PEM pages describe a fraud framework and insights capability as programme work. They do not establish a current live national fraud-score API:
https://resbank.co.za/en/home/what-we-do/payments-and-settlements/pem

## 3. Hypotheses

H1: The evidence-qualified presentation improves material-fact comprehension versus the minimal presentation.

H2: The evidence-qualified presentation lowers false-safe actions on deceptive/contradictory tasks.

H3: The evidence-qualified presentation does not increase false-alarm actions on adequately evidenced legitimate tasks beyond a prospectively chosen non-inferiority margin.

H4: The treatment improves calibration between confidence and correctness and does not increase perceived pressure.

H5: Material-field comprehension is not materially worse in supported accessibility presentation modes.

These hypotheses must be preregistered. No H1–H5 result is claimed by this development corpus.

## 4. Design

Use a randomized, controlled, between-subject design:
- Arm A: minimal, generic request presentation.
- Arm B: evidence-qualified presentation that makes identity, payee match, requester authority, claim evidence, fees, amount, currency, purpose, expiry and uncertainty explicit.

Use matched task templates so both arms receive the same underlying request and evidence state. The intervention is how evidence and uncertainty are presented, not a hidden change to the transaction facts.

Randomize participants to arms. Randomize task order within arm. Avoid a within-person crossover unless a pilot establishes that learning and carryover can be controlled. Include legitimate, deceptive and ambiguous tasks in both arms.

All task entities and contexts are fictional. Participants are told clearly that this is a simulation. No provider brand, real bank account, cellphone number, ShapID, credentials, real debt or money transfer is used.

## 5. Development corpus and holdout

The 24 checked-in vignettes are open development fixtures only, not a blinded holdout set. They are balanced 8/8/8 across:
- adequately evidenced legitimate requests;
- deceptive or contradictory requests;
- insufficient-evidence or ambiguous requests.

A powered study must use a separately generated and locked holdout bank, created after the development corpus, with controlled variations and no participant access to reference answers. Include unseen task templates to test generalisation rather than memorisation.

Reference actions are based on the synthetic evidence state and material facts, not a legal opinion about whether a debt is due. A legitimate obligation can exist outside the payment interface; conversely, the payment interface cannot manufacture one.

## 6. Primary outcomes (reported separately)

1. **Material-fact comprehension:** proportion of critical facts correctly recalled/identified (beneficiary, amount, currency, fees, purpose, payee-check result, requester authority and claim evidence).
2. **Appropriate action:** proportion of decisions matching the adjudicated action: accept, decline/report, or pause/verify.
3. **False-safe rate:** accepting or proceeding when the scenario includes a material contradiction or insufficient authority/evidence.
4. **False-alarm rate:** declining/reporting adequately evidenced legitimate requests.
5. **Voluntariness/pressure:** participant-reported pressure and perceived ability to decline or verify independently.

Secondary outcomes include decision duration, confidence calibration, recognition of Match/Close Match/No Match/Unable to Verify, warning recall, fee comprehension, abandonment and accessibility parity.

Never combine these into a single weighted score. Publish the denominator, missing responses, confidence intervals, scenario mix and preregistered exclusions for each endpoint.

## 7. Analysis plan

Before recruitment:
- conduct cognitive interviews and a usability pilot using fictional tasks;
- obtain an ethics review/approval determination from an appropriate institutional review body;
- perform a prospective power analysis using a pilot-derived baseline and the minimum effect of interest;
- set a non-inferiority margin for legitimate-request false alarms;
- preregister hypotheses, primary endpoints, allocation, exclusions, missing-data handling and statistical models;
- freeze a blinded holdout set and independent scenario adjudication.

Use intention-to-treat analysis. Model participant and task variation. Report effect sizes and confidence intervals, not only p-values. Correctly account for repeated task responses nested within participants. Report language and accessibility subgroup results only when appropriately powered and privacy-safe.

## 8. Privacy, accessibility and participant safety

Collect no bank account data, cellphone numbers, ID numbers, actual payment receipts or free-text narratives that could contain personal information. Use pseudonymous study tokens and coarse optional demographic bands only where justified. Do not collect more than needed.

Participation is voluntary. Participants can stop or withdraw without penalty. Compensation must not depend on accepting, declining, detecting a scam or completing every task. Do not use frightening threats, real debt collection or distressing deception. The study must not encourage participants to distrust all payment requests.

The development set is en-ZA English only. Future Afrikaans, isiZulu, isiXhosa, Sesotho and other language versions require professional human translation, independent review, cognitive interviews and equivalence checks; machine translation alone does not qualify a version.

Screen-reader/mobile/desktop prototypes are simulated modalities. USSD/feature-phone tasks are not assertions that any particular participant or provider currently supports an RTP journey through that channel.

## 9. Proposed success criteria

Hard semantic gates:
- zero payment instructions without exact payer consent;
- zero presentation that omits material payee, amount, currency or known fee;
- zero unsupported assertion that a sender credential, name match or reference proves a legal claim;
- zero design that makes decline or independent verification materially harder than acceptance;
- zero secondary use of request data without declared purpose and authority.

Empirical success must be established prospectively:
- lower false-safe rate with a confidence interval and a minimum effect chosen before recruitment;
- false-alarm rate non-inferior within a predeclared margin;
- improved material-fact comprehension;
- no material increase in perceived pressure or task abandonment;
- no unacceptably large accessibility or language gap.

Do not select numeric effect thresholds after seeing treatment results. Determine them through pilot, power analysis and preregistration.

## 10. Claim ceiling

The current corpus is synthetic and development-only. Static validation can establish dataset structure and balance, not human comprehension, reduced fraud, fairness, lower loss or product safety. Any human-study claim must be grounded in approved recruitment, a preregistered protocol, a blinded holdout and independently reviewed analysis.
