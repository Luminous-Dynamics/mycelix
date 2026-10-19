# Mycelix Constitutional Authority v0.1

Status: **experimental / non-activating**

Tracking: #1185

This document defines the first machine-readable constitutional authority layer for Mycelix. It is intentionally additive: this tranche defines vocabulary and conformance rules but does **not** silently change current proposal, voting, execution, or constitutional amendment behavior.

## 1. Constitutional topology

Mycelix distinguishes four categories that must not be collapsed into one another:

1. **Constituent sovereignty** — citizens and constituent communities as the source of constitutional authority.
2. **Five constituted branches** — Deliberative, Stewardship, Justice, Integrity, and Civic Mandate.
3. **Independent guardians** — protected institutions with narrow authority, but without general sovereign power.
4. **Constitutional infrastructure** — capability validation, provenance, expiry, conformance tests, and later cross-DNA enforcement.

Constituent sovereignty is deliberately **not a sixth branch**. Branches are constituted by the constitutional order; constituent sovereignty is the authority from which that order derives.

## 2. Five branches

| Branch | Primary function | Explicitly excluded examples |
|---|---|---|
| Deliberative | law, appropriation, treaty ratification, legislative oversight | executing appropriations, adjudicating disputes, certifying elections |
| Stewardship | lawful execution, public administration, services, provisional emergency action | certifying its own mandate, constitutional adjudication, public audit |
| Justice | adjudication, constitutional review, remedies | ordinary legislation, appropriation, program administration |
| Integrity | public audit, authority-use audit, integrity investigation, findings, referral | prosecution judgment, conviction, ordinary legislation |
| Civic Mandate | elections, recalls, initiatives, sortition, eligibility, mandate certification | substantive legislation, executive administration, adjudication |

The authority model is deny-by-default. A role name or reputation tier is not sufficient to imply a constitutional power.

## 3. Guardians

Guardians are constitutionally protected but do not become extra governments.

Initial guardian vocabulary:

- Rights Defender
- Public Evidence Service
- Future Generations Guardian
- Fiscal Observatory
- Public Service Commission
- Prosecution Service

Their powers are intentionally narrow. For example, the Integrity Branch may investigate and refer; the independent Prosecution Service may initiate a prosecution; the Justice Branch adjudicates. This preserves the separation:

`investigator != prosecutor != judge`

## 4. Constituent voting equality

The existing governance stack supports MATL, stake, Phi, participation, domain reputation, quadratic credits, and composite merit weighting. Those mechanisms may remain useful for appropriate organizational, technical, cooperative, or ordinary-governance contexts.

They must not determine the base civic weight of a person in the following decision classes:

- Fundamental Rights
- Foundational Covenant
- Structural Constitution
- Civic Mandate

For these classes, v0.1 defines `VoteWeightBasis::EqualCivic` as the only constitutionally conformant base weighting.

This separates two principles that should coexist:

- **political equality** for constituent and fundamental civic authority;
- **earned qualification / bounded merit** for expert, technical, operational, and cooperative responsibilities.

This tranche does not yet alter the live voting zome. A later migration must create explicit civic/constituent vote types and transition rules rather than repurposing existing weighted vote entries in place.

## 5. Automated agents

`AuthorityPrincipal::AutomatedAgent` holds no power in the constitutional power inventory.

This does not prohibit AI or software from:

- analysis,
- simulation,
- recommendation,
- forecasting,
- audit assistance,
- bounded execution delegated through ordinary operational capabilities.

It does prohibit treating an automated agent as the direct holder of constituent or branch sovereignty. In particular, this authority layer is designed so an AI identity cannot directly satisfy a constitutional power check merely by possessing a high trust, MATL, Phi, reputation, or stake score.

## 6. Emergency expiry

The first extraordinary-power invariant is encoded now:

- `AuthorizeEmergency`
- `DeclareProvisionalEmergency`

require a hard expiry on their `ConstitutionalCapability`.

Later tranches should add the full emergency state machine, progressive renewal requirements, non-self-extension, forbidden constitutional transitions, and permissionless expiry enforcement already anticipated elsewhere in Mycelix governance design.

## 7. Capability record

The pure shared crate defines a transport-neutral `ConstitutionalCapability` containing:

- holder,
- power,
- jurisdiction,
- authority source,
- validity start,
- optional/required expiry,
- delegability,
- delegation-depth bound.

The type deliberately avoids HDK/HDI dependencies. Governance, Civic, Justice, Integrity, and other DNAs can later wrap or reference the same semantic object without introducing integrity-zome dependency cycles.

## 8. Initial conformance invariants

The v0.1 tests establish the following minimum properties:

- exactly five constituted branches;
- constituent sovereignty is not represented as a branch;
- Stewardship cannot certify its own mandate;
- Civic Mandate cannot enact substantive law;
- Integrity can refer for prosecution but cannot adjudicate;
- Justice cannot appropriate or execute public funds;
- no constituted branch can ratify the Foundational Covenant;
- automated agents hold none of the enumerated constitutional powers;
- fundamental civic decision classes reject MATL, stake, Phi, participation, reputation, quadratic-credit, and composite-merit vote weighting;
- emergency authority cannot be represented as valid without expiry;
- a capability is rejected if its holder attempts a cross-branch power escalation;
- nondelegable capabilities cannot claim residual delegation depth.

## 9. Relationship to the current Constitution

The existing Mycelix constitutional code remains authoritative during this tranche. This crate is a candidate executable interpretation layer, not a unilateral constitutional amendment.

Where this model conflicts with the ratified/recognized constitutional text, the conflict must be surfaced and resolved through the appropriate amendment or migration process before enforcement is enabled.

In particular, the equal-civic rule intentionally exposes a design conflict with current Phi/MATL/stake-weighted constitutional voting. The conflict is documented rather than silently resolved in code.

## 10. Integration sequence

### MYC-CONST-001 — authority taxonomy

This document and `crates/constitutional-authority`.

### MYC-CONST-002 — constituent sovereignty plane

Add explicit equal-civic vote/ratification types and eligibility semantics. Keep them distinct from existing weighted governance vote entries.

### MYC-CONST-003 — branch capability enforcement

Integrate the shared authority checks at branch-crossing boundaries and add cross-zome/cross-DNA conformance tests.

Only after those three are reviewed should Mycelix proceed to:

- Civic Mandate execution plane,
- Integrity Branch entries and audit-access protocol,
- independent prosecution boundary,
- Justice/Emergency/Public-Evidence trust-root separation,
- appointment/tenure kernel,
- emergency state machine,
- constituent amendment protocol,
- AI constitutional boundary enforcement,
- adversarial/property-based constitutional test suite.

## 11. Non-goals of v0.1

This tranche does not:

- rename or remove existing zomes;
- change live voting thresholds;
- migrate existing vote records;
- create a new election system;
- decide citizen eligibility;
- alter the current amendment procedure;
- activate an Integrity Branch operationally;
- grant guardians new runtime powers;
- change the current Holochain DNA manifest.

The purpose is to create a small, reviewable constitutional waist before higher-level governance machinery depends on it.

## 12. Comparative design basis

This architecture is not a literal copy of any single constitution. It draws specific lessons from several mature governance patterns:

- **Independent constitutional institutions:** South Africa's Constitution, Chapter 9, separately protects the Public Protector, Human Rights Commission, Auditor-General, and Electoral Commission; it requires independence and impartiality while retaining accountability. This supports the distinction between a sovereign branch and a constitutionally protected guardian/oversight institution. <https://www.justice.gov.za/constitution/chp09.html>
- **Fourth-branch design:** International IDEA's Constitution-Building Primer on independent regulatory and oversight institutions describes politically neutral bodies outside the traditional executive/legislative/judicial tripartite model whose purpose is democratic integrity, oversight, quality, and resilience. <https://www.idea.int/publications/catalogue/independent-regulatory-and-oversight-fourth-branch-institutions>
- **Audit independence:** INTOSAI's Mexico Declaration identifies unrestricted access to necessary information, freedom to report, and sufficient organizational/financial independence as prerequisites for effective external public audit. <https://www.intosai.org/documents.html>
- **Practical independence:** the 2026 OECD/IDI work on Supreme Audit Institutions emphasizes that legal independence can still be undermined through staffing, funding, information-access, and mandate design. This supports treating funding and access as constitutional capability questions rather than decorative independence language. <https://www.intosai.org/fileadmin/downloads/focus_areas/independence/EN_Report_IDI_OECD_Strengthening_the_Independence_of_SAIS.pdf>
- **Independent fiscal analysis:** OECD Government at a Glance 2025 reports that 29 of 38 OECD countries have at least one independent fiscal institution. This supports a Fiscal Observatory as an independent analytical guardian rather than a sovereign budget branch. <https://www.oecd.org/en/publications/government-at-a-glance-2025_0efd0bcd-en/full-report/independent-fiscal-institutions_f08e9536.html>
- **Collegial executive option:** Switzerland's Federal Council demonstrates a seven-member executive built around equal status, collegiality, consensus, and a rotating first-among-equals presidency. This is a useful design precedent for avoiding a single executive choke point. <https://www.admin.ch/en/federal-council-tasks>
- **Qualified-majority appointments:** Venice Commission work stresses that qualified majorities for sensitive appointments are intended to force broad agreement and prevent the governing majority from controlling independent offices. Mycelix should pair such thresholds with explicit anti-deadlock mechanisms rather than simply lowering legitimacy requirements after failure. <https://www.venice.coe.int/>

These references justify the **category distinctions** in v0.1; they do not automatically determine Mycelix's final thresholds, appointment mechanisms, or constitutional text. Those remain subject to Mycelix's own constituent process and evidence.
