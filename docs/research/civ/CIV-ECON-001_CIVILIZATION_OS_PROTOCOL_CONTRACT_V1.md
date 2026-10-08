# CIV-ECON-001 — Civilization OS Protocol Contract v1

Status: research architecture / protocol contract

Parent issue: #4804

Related:
- #4771 / #4772 — Creditism protocol family
- #3343 — external economic interoperability
- #4578 — open-economy / FX settlement
- Symthaea #7188 / #7189 — socioeconomic interoperability laboratory
- Symthaea #7190 — power / agency / rights-floor laboratory
- Symthaea #7193 — stock-flow / intergenerational laboratory
- Symthaea #7046 — endogenous institutional evolution

## 1. Civilization OS is a substrate, not an ideology

Civilization OS provides a shared coordination substrate.

Economic, governance and allocation systems are versioned profiles over that substrate.

Examples include:
- debt/equity markets;
- market + UBI;
- cooperatives;
- mutual credit;
- Creditism;
- commons / polycentric systems;
- public-budget planning;
- hybrids.

No profile becomes authoritative merely because it is implemented.

## 2. Protocol stack

The substrate is divided into explicit domains:

`
Person
  ↓
Rights
  ↓
Evidence
  ↓
Resources / Capacity
  ↓
Authority
  ↓
Coordination
  ↓
Economic instruments
  ↓
Production / Logistics
  ↓
Outcomes / Externalities
  ↓
Dispute / Repair
  ↓
Institutional evolution
`

The stack is bidirectional in time:

`state -> action -> physical/social outcome -> evidence -> review -> updated state`

but domains may not silently inherit each other's authority.

## 3. Constitutional non-escalation

The kernel preserves:

`
evidence validity
    != human worth
    != political legitimacy
    != economic entitlement
    != physical permission
`

and:

`
instrument
    != entitlement
    != ownership
    != authority
    != capacity
    != valuation
    != settlement
`

A downstream profile can restrict an upstream claim.

It may not widen an upstream claim without an explicitly qualified authority transition.

## 4. Personhood and rights

Person identity is distinct from every economic and governance attribute.

Rights are modeled as constitutional constraints.

They are not balances.

They cannot be inferred from money, Credit, reputation, contribution score, employment, institutional membership, or governance rank.

Any temporary restriction must bind:
- subject;
- action class;
- legal/institutional authority;
- scope;
- reason;
- start;
- expiry;
- appeal;
- reversal.

## 5. Evidence and authority

Canonical chain:

`
observation
→ evidence
→ qualification
→ authority
→ decision
→ execution
→ outcome
`

Each transition must preserve exact:
- subject;
- attribute;
- policy generation;
- context;
- currentness;
- source/provenance;
- authority scope.

No confidence score, consensus count, AI output, reputation value or cryptographic signature creates missing authority by itself.

## 6. Physical reality boundary

Resources are first-class state:
- labor;
- materials;
- energy;
- water;
- land/space;
- housing;
- machines;
- inventories;
- logistics;
- ecological stocks;
- repair capacity.

Required distinction:

`authorization != reservation != execution != outcome`.

No economic instrument may create physical capacity merely by being issued.

Unknown capacity is not available capacity.

## 7. Resource reservation

A resource reservation binds:
- resource identity;
- quantity;
- unit;
- location/scope;
- time window;
- reserving project;
- authority;
- competing reservations;
- expiry;
- fulfillment status.

Two projects cannot silently consume the same exclusive capacity.

Reservations must be independently reconciled with execution receipts.

## 8. Economic instruments

Each economic regime declares its own instruments.

An instrument profile must specify:
- identifier;
- issuer/authority;
- unit;
- quantity semantics;
- issuance;
- deletion/settlement;
- transferability;
- ownership/claim semantics;
- expiry;
- collateralization rules;
- inheritance;
- external conversion;
- correction/reversal.

Example Creditism semantics remain external adapter logic.

The core must not equate PC, CC or Bonus with MYCEL, SAP or TEND.

## 9. Capital formation

Capital coordination is separate from currency.

A capital protocol represents:

`
proposal
→ evidence
→ authorization
→ capacity envelope
→ reservation
→ milestone
→ execution
→ inspection
→ service
→ maintenance
→ renewal / retirement
`

Financial structures may include:
- debt;
- equity;
- cooperative pools;
- public budgets;
- Community Credit;
- mutual-credit commitments;
- hybrids.

But the following remain distinct:

`financing instrument != ownership claim != debt claim != physical reservation != execution authority`.

## 10. Prices

Prices are observations/signals, not universal authority.

The kernel allows a regime to use:
- market prices;
- auctions;
- administered prices;
- queues;
- rationing;
- voting;
- matching;
- planning;
- hybrid allocation.

A price signal does not automatically determine wage, political power, housing entitlement, public purpose, rights, or ownership.

## 11. Housing

Housing is represented as physical capacity plus rights/stewardship.

At minimum:

`person + eligibility + dwelling + stewardship agreement -> residence right`.

Residence rights are distinct from:
- PC;
- ownership;
- speculative asset value;
- governance authority.

Allocation mechanisms are profile-defined and must report:
- access;
- waiting;
- segregation;
- mobility;
- gaming;
- capture.

## 12. Power and non-domination

CivOS measures power structurally as well as economically.

Required domains:
- asset concentration;
- infrastructure centrality;
- bridge/settlement centrality;
- verifier authority;
- governance agenda/veto;
- dependency;
- switching cost;
- exit availability;
- information asymmetry;
- resource chokepoint control.

A regime must not claim low inequality merely because balances are evenly distributed.

`balance equality != power equality`.

## 13. Privacy and legibility

Observation is itself a resource and a power relation.

Every observability mechanism declares:
- data collected;
- access scope;
- retention;
- disclosure;
- inference risk;
- error risk;
- surveillance concentration.

Coordination improvement does not erase privacy costs.

No global human score may be created merely because many observations exist.

## 14. Stock-flow consistency

Every modeled stock obeys:

`stock(t+1) = stock(t) + inflows - outflows + transformations + exogenous_change`.

This applies to:
- money/claims;
- physical capital;
- inventories;
- infrastructure;
- ecological stocks;
- skills;
- organizational capacity;
- obligations.

Project success cannot hide depreciation or maintenance liabilities.

## 15. Intergenerational continuity

The world state may include cohorts/generations.

Carry forward explicitly:
- assets;
- liabilities;
- environmental conditions;
- skills;
- institutional rules;
- maintenance obligations;
- unresolved claims.

Measure future feasible choices.

A present gain that permanently reduces future feasible states is an explicit intertemporal cost, not invisible bookkeeping.

## 16. Externalities

External effects should be modeled as state transitions where feasible.

Examples:
- pollution;
- congestion;
- health burden;
- infrastructure wear;
- ecological damage;
- information spillover;
- coordination burden.

Do not bury externalities in a single welfare scalar.

## 17. Interoperability airlock

Cross-regime interaction must pass through:

`
source observation
→ exact source profile
→ source authority/finality
→ conversion rule
→ settlement obligation
→ synchronized settlement where required
→ target recognition
→ reconciliation
`

Every bridge preserves:
- source identity;
- target identity;
- quantity/unit;
- authority;
- valuation/rate profile;
- settlement identity;
- finality;
- validity;
- correction lineage;
- claim ceiling.

No numeric parity implies semantic equivalence.

## 18. Failure isolation

Economic subsystems are bounded trust domains.

A compromised subsystem must not automatically:
- mint claims in another subsystem;
- rewrite external history;
- acquire unrelated governance authority;
- invalidate another subsystem's rights;
- turn observations into legal ownership.

Bridge failure must have explicit containment and contagion metrics.

## 19. Governance profiles

Every governance mechanism specifies:
- participants;
- affected actors;
- information set;
- proposal rule;
- selection rule;
- adoption rule;
- delegation;
- authority scope;
- duration;
- revocation;
- appeal;
- emergency mode;
- exit/fork;
- amendment.

The proposal generator cannot silently also become the authority deciding its adoption.

## 20. Role-specific qualification

CivOS prefers:

`role/domain evidence -> bounded qualification -> bounded authority`

over:

`global score -> universal political rank`.

A cognitive, participation or expertise metric may be useful where independently justified.

It cannot by itself:
- grant universal political authority;
- remove constitutional rights;
- mint economic claims;
- authorize physical action;
- override capacity or safety constraints.

See Symthaea #7191 / #7192 audit.

## 21. Institutional evolution

Institutional change is a protocol:

`
problem
→ observation
→ proposal
→ deliberation
→ adoption
→ implementation
→ outcome
→ review
→ amendment / repeal
`

Every institutional mutation records:
- parent profile;
- semantic delta;
- proposer;
- affected actors;
- adoption decision;
- evidence;
- implementation result;
- subsequent outcome;
- supersession/repeal.

No institution is assumed to be final.

## 22. Scientific model architecture

The socioeconomic lab should be separated into:

### Mechanism model
What rules exist?

### World model
What physical/resource state exists?

### Agent model
How do actors observe, decide and learn?

### Institutional model
How do rules evolve?

### Bridge model
How do regimes interact?

### Oracle
What is mechanically true about the simulation?

### Evaluation
What outcome vector was produced?

The oracle must not consume the model's claimed outcome.

## 23. Falsification-first campaign

Every proposed institution receives adversarial treatment before positive interpretation.

Minimum families:
- issuance overshoot;
- resource overbooking;
- verification fraud;
- governance capture;
- bridge capture;
- settlement replay;
- capital under-maintenance;
- ecological depletion;
- dependency lock-in;
- privacy collapse;
- emergency permanence;
- intergenerational extraction.

A result that only passes benign scenarios is not a strong result.

## 24. Core Civilization OS outputs

The canonical output should be a vector:

`
access
capacity
distribution
power
agency
rights
privacy
resilience
innovation
maintenance
ecology
intergenerational_state
governance
interoperability
external_dependence
`

No mandatory scalar “civilization score” exists in the kernel.

Optional normative profiles may define their own aggregations, but the underlying vector and causal decomposition remain available.

## 25. Civilization Mechanism Map

The long-term output is:

`
problem
+ physical environment
+ information regime
+ institutional regime
+ governance regime
+ economic regime
+ bridge structure
→ outcome vector
→ causal pathways
→ failure boundaries
→ transition costs
`

This is the correct scientific object.

The question is not:

`Which ideology wins?`

The question is:

`Which coordination mechanism works under which conditions, and what does it cost?`

## 26. Claim ceiling

CIV-ECON-001 is a research architecture.

It does not establish:
- universal political legitimacy;
- human flourishing as a measurable scalar;
- economic superiority;
- macroeconomic equilibrium;
- legal status;
- empirical causal validity;
- ecological sufficiency;
- historical inevitability.

The architecture exists to make these questions explicit, testable and contestable rather than silently decided by the implementation.
