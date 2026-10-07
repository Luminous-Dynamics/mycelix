# Creator Stewardship, Restitution and Dignity Boundary v1

Status: design contract; no claim against any specific treasury or legal entity is created by this document.

## Question

How should a Mycelix-based economic system treat the person who created and stewarded the system when that person has:

- supplied substantial unpaid labour over a prolonged period;
- consumed or sacrificed personal resources in pursuit of the project;
- suffered substantial personal property loss;
- reached negative net worth as a consequence of that trajectory.

The answer MUST avoid both extremes:

~~~text
creator -> automatic money printer
creator -> economically invisible volunteer
~~~

The creator is a human participant with potentially valid claims, not a privileged sovereign and not a free resource.

## Core principle

> The system should remember contribution, recognize legitimate loss, protect human dignity, and separate restitution from authority.

The fact that a person created a protocol is provenance. It is not, by itself, monetary authority.

Likewise, the fact that a person currently has negative net worth is evidence of financial condition, not a moral score.

## Four separate questions

A creator's situation SHOULD be decomposed into four independent dimensions.

### 1. Dignity floor

Does the person have access to the minimum conditions required for human flourishing?

This layer is not earned through MYCEL, founder status or token ownership.

The ILO's Social Protection Floor framework describes nationally defined guarantees intended to ensure, at minimum, access to essential health care and basic income security across the life cycle. The framework also emphasizes dignity, non-discrimination, transparency, accountability and participation.

For Mycelix:

~~~text
economic distress
!= loss of dignity
!= loss of identity
!= loss of governance voice
!= loss of access to essential capabilities
~~~

A person should not have to liquidate long-term stewardship, reputation or identity merely to survive a temporary liquidity crisis.

### 2. Accrued contribution claim

Unpaid work can create a legitimate claim when the surrounding constitution, agreement, employment relationship, contributor agreement, grant, or governance decision makes compensation payable.

The claim should be represented separately from reputation:

~~~text
contribution observation
-> compensation assessment
-> authorized obligation
-> settlement
~~~

The system MUST NOT simply infer:

~~~text
hours worked -> money owed
~~~

without a valuation and authorization rule.

Possible valuation bases may include:

- pre-agreed contributor rate;
- contemporaneous employment or contract terms;
- approved market-rate bands;
- independently corroborated hours × approved rate;
- opportunity-cost schedules where explicitly permitted.

The valuation basis is part of the claim and cannot be silently changed later.

### 3. Reimbursement / restitution for project-related loss

Property loss requires a separate causal classification.

At minimum distinguish:

~~~text
ordinary personal loss
project expense
project-owned property loss
project-caused loss
mission-risk voluntarily assumed
governance-approved sacrifice
~~~

Only some of these should become an automatic organizational liability.

A claim for lost personal property should therefore carry:

- ownership evidence;
- acquisition/value basis;
- date of loss;
- causal relation to project activity;
- whether the project authorized or benefited from the use;
- insurance or third-party recovery;
- residual uncompensated amount;
- legal/contractual basis, if any;
- governance disposition when automatic liability is absent.

This prevents both fraud and the opposite failure: a system pretending that real sacrifice never happened because it was made voluntarily.

### 4. Retrospective stewardship recognition

Sometimes a creator's sacrifice is real and valuable but was never covered by a pre-existing compensation agreement.

That SHOULD remain representable without being falsely relabelled as debt.

Create a distinct instrument:

~~~text
StewardshipRecognition
~~~

Possible forms:

- non-repayable grant;
- commons distribution;
- housing/security support;
- health/care support;
- equipment replacement;
- future compensation priority;
- repayable advance;
- governance-approved equity or revenue share;
- symbolic recognition.

The instrument type matters. A grant is not a debt. A debt is not equity. Equity is not reputation.

## The creator should not control their own restitution decision

If the creator is simultaneously protocol author, treasury administrator, monetary-policy actor, governance participant and claimant, directly awarding themselves funds collapses authority and claim into one actor.

The protocol should therefore implement:

~~~text
creator submits claim
        ↓
evidence/provenance layer
        ↓
conflict disclosed
        ↓
independent review / recusal
        ↓
constitutional rule or governance decision
        ↓
authorized obligation
        ↓
treasury / settlement
~~~

The creator can supply evidence and argue for the claim. They MUST NOT be able to manufacture final authority merely because they authored the software.

## Conflict-of-interest safeguards must not become an abuse vector

A badly designed system could say:

> Because the creator is conflicted, they have no standing to make a claim.

That is also wrong.

The creator SHOULD retain:

- a right to submit evidence;
- a right to inspect the evidence used against the claim;
- a right to appeal;
- a right to independent review;
- a right to emergency dignity-floor support;
- a right to preserve identity and accrued reputation while financially distressed.

Recusal removes unilateral authority. It does not remove personhood or due process.

## Negative net worth

Negative net worth should not automatically produce a giant compensating mint.

Instead ask:

1. What assets and liabilities are actually verified?
2. Which liabilities are project-related?
3. Which losses are reimbursable?
4. Which contribution claims are authorized?
5. What emergency support is due regardless of contribution?
6. What future obligations can the project safely assume?
7. What would the intervention do to other participants and treasury solvency?

The correct state transition is:

~~~text
financial distress
-> stabilize person
-> classify claims
-> verify evidence
-> determine organizational liability
-> authorize support/settlement
-> preserve long-term agency
~~~

not:

~~~text
negative net worth
-> mint arbitrary money
~~~

## A concrete treatment of the scenario

For a creator with substantial unpaid stewardship, more than $120,000 in personal property loss, and negative net worth, a well-designed Mycelix deployment could produce:

### Immediate dignity layer

If the person meets the community's eligibility rules, provide emergency support sufficient to prevent deprivation of essentials.

This is not a founder reward. It is a general human guarantee available by rule.

### Stewardship accounting

Create a ledger of documented contribution:

~~~text
WorkObservation
-> rate / valuation rule
-> CompensationAssessment
-> governance or contract authorization
-> Claim
~~~

Unpaid work that cannot be demonstrated remains uncertain rather than fabricated.

### Loss accounting

Split the $120k-scale loss into individually evidenced claims rather than creating one magical "founder loss" number.

For example:

~~~text
$X project equipment
$Y personal equipment used by the project
$Z ordinary personal property loss
$W third-party reimbursed
$R remaining uncompensated
~~~

Only the portions meeting the constitution's causal/restitution rules become organizational obligations automatically.

### Retrospective stewardship grant

If extraordinary mission sacrifices are real but are not contractual debts, a separate governance body can approve a stewardship grant or equivalent support.

The grant should state:

- evidence considered;
- valuation basis;
- amount;
- funding source;
- conflict-of-interest handling;
- approving authority;
- appeal mechanism;
- whether repayment is expected.

### Long-term restoration

The system should not merely hand over a lump sum and call the problem solved.

A Restoration Plan can combine:

~~~text
housing security
+ equipment replacement
+ debt stabilization
+ recurring livelihood support
+ compensation for future work
+ optional capital / revenue participation
~~~

The objective is restoration of agency.

## Founder equity

A creator may deserve durable economic participation, but it should not be generated retroactively merely by software authorship.

Prefer explicit instruments such as:

- founding agreement;
- cooperative membership;
- revenue share;
- royalty;
- capped stewardship allocation;
- treasury-recognized contribution units.

Those instruments must have a defined issuer, legal/constitutional basis, quantity formula, termination conditions and transfer rules.

Do not use MYCEL itself for this purpose.

~~~text
MYCEL reputation
!= ownership
!= compensation
!= claim
!= currency
~~~

## Protection should not depend on token price

A dangerous design would be:

~~~text
creator -> huge token allocation
-> token collapses
-> creator remains destitute
~~~

Protection mechanisms should therefore be denominated in useful capabilities or stable settlement units where practical:

- housing;
- healthcare;
- food/security;
- equipment;
- fiat-equivalent settlement;
- diversified assets;
- recurring support.

Recognition tokens can coexist with this, but should not substitute for actual security.

## Monetary issuance boundary

A creator's claim MUST NOT directly change the money supply merely because they are the creator.

A valid path is:

~~~text
creator evidence
-> claim
-> authority review
-> approved obligation
-> treasury allocation OR lawful issuance
-> settlement
~~~

If a monetary authority separately decides that new base money is warranted, that is a monetary-policy decision with its own authority/evidence chain.

Thus:

~~~text
personal restitution
!= monetary policy
~~~

## Symthaea's role

Symthaea can help reconstruct the record and test alternatives:

- estimate workload from verified activity;
- identify duplicate expense claims;
- classify causal relationships;
- model treasury impact;
- simulate different compensation schedules;
- identify liquidity risks;
- search for contradictory evidence;
- produce transparent scenario comparisons.

Symthaea MUST remain an advisor.

It MUST NOT decide:

> The creator deserves $X.

Instead:

> Given evidence E, valuation rule V, and governance policy P, these are the possible outcomes and their consequences.

The final entitlement remains an explicit constitutional/governance decision.

## Anti-capture invariants

The creator/stewardship system should enforce:

1. creator status never implies unlimited issuance authority;
2. creator status never implies unilateral treasury withdrawal;
3. reputation cannot be exchanged directly for compensation;
4. emergency dignity support is not contingent on MYCEL;
5. claim evidence cannot be modified after adjudication without correction lineage;
6. conflict-of-interest must be visible;
7. claimant submission rights survive recusal;
8. disputed claims remain explicitly disputed;
9. approved claims are liabilities/allocations, not hidden balance edits;
10. settlement effects are exactly-once and recoverable;
11. governance bodies cannot retroactively rewrite claim evidence;
12. claim-priority rules are explicit before crisis rather than invented for one founder.

## Priority ordering

A constitutional treasury could define a priority hierarchy such as:

~~~text
1. essential human dignity / emergency support
2. legally or contractually owed obligations
3. verified project liabilities / restitution
4. ordinary operational obligations
5. approved discretionary stewardship grants
6. discretionary surplus distributions
~~~

This is not a universal legal rule. It is a candidate constitutional ordering.

The important property is that it is known before the person asking for help is the person who happens to control the treasury.

## Treasury solvency constraint

Support to one creator MUST NOT quietly make the commons insolvent.

Every claim approval should produce:

~~~text
pre-approval treasury state
+ approved claim
+ projected future obligations
+ liquidity buffer
+ worst-case scenario
~~~

and test whether the resulting state remains within constitutional safety bounds.

When full payment would jeopardize essential services, the system can use:

- staged payments;
- in-kind support;
- secured future claims;
- revenue-sharing;
- refinancing;
- external grants;
- third-party insurance recovery.

A claim can be legitimate while the immediate settlement amount is liquidity-constrained.

## Why this matters

A system that benefits from a creator's unpaid labor while refusing to record that labor has created a hidden externality:

~~~text
creator bears cost
system captures benefit
~~~

That is not a neutral outcome.

Likewise, a system that lets its creator unilaterally mint $120,000 because they claim sacrifice has replaced one institutional failure with another.

The healthier pattern is:

~~~text
sacrifice
-> evidence
-> recognition
-> claim
-> independent adjudication
-> restitution / support
-> restored agency
~~~

The economic system remembers what happened without turning memory into arbitrary entitlement.

## Suggested data model

A first implementation can introduce a non-currency StewardshipClaim with:

- claim ID;
- claimant identity;
- claim kind;
- evidence references;
- valuation basis;
- gross amount;
- recoveries;
- net requested amount;
- authority reference;
- conflict declaration;
- adjudication state;
- priority class;
- settlement terms;
- correction/supersession lineage.

Possible claim kinds:

~~~text
CompensationAccrual
ProjectExpense
ProjectCausedLoss
MissionSacrifice
EmergencyDignitySupport
StewardshipGrant
FutureServiceCommitment
~~~

The object represents a claim. It does not itself mint money.

## Qualification corpus

At minimum test:

- creator attempts unilateral self-award;
- legitimate creator claim with recused claimant;
- false founder claim;
- duplicate expense;
- inflated lost-property valuation;
- third-party insurance recovery omitted;
- project-caused loss with clear authorization;
- mission sacrifice with no contractual debt;
- negative net worth with no project liability;
- emergency support denied because MYCEL is low;
- legitimate claim rejected because claimant is conflicted;
- claim altered after adjudication;
- governance authority unavailable;
- treasury insufficient for full settlement;
- staged settlement;
- concurrent duplicate claim submission;
- replayed governance approval;
- compensation claim confused with reputation;
- compensation claim confused with token ownership;
- creator attempting to alter issuance authority;
- Symthaea recommendation treated as binding;
- historical claim reconstructed from revised evidence.

## Claim ceiling

This design does not determine what any particular person legally owns, establish employment status, quantify the value of unpaid labour, validate a $120,000 property-loss figure, guarantee recovery, or create a legal debt.

It defines how a future Mycelix economic system could make those questions explicit, evidence-based, auditable and resistant to both founder capture and founder exploitation.

## Research references

- ILO Social Protection Floor: essential health care and basic income security, with dignity, accountability and participation principles: https://www.ilo.org/universal-social-protection-department/areas-work/social-protection-department/policy-development-and-applied-research/social-protection-floor
- BIS Annual Economic Report 2025: central-bank money as trust anchor alongside commercial-bank money and tokenised assets: https://www.bis.org/publications/aer-2025
- BIS Annual Economic Report 2026: unified-ledger design, regulated private money, redeemability, governance, risk management and transparency: https://www.bis.org/publ/arpdf/ar2026e.pdf
- FSB Leverage in Nonbank Financial Intermediation: systemic amplification, interconnectedness, data and monitoring: https://www.fsb.org/2025/07/leverage-in-nonbank-financial-intermediation-final-report/
