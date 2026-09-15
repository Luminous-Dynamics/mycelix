# Mycelix Capital Constitution v1

Status: Draft design specification  
Tracking: #880 (`MYC-CAP-001`)  
Authority: Commercial/governance design only. This document grants no runtime authority and is not legal advice.

## 1. Purpose

Mycelix should be able to accept productive capital without making capital ownership equivalent to protocol authority.

The governing separation is:

```text
capital contribution
!= protocol authority
!= federation sovereignty
!= evidence truth
!= customer authority
!= technical qualification
```

Commercial success is desirable. Capture of the protocol, federation, evidence semantics, or users is not required for commercial success and should not be an implicit financing right.

## 2. Constitutional objective

The target architecture separates four economic domains:

1. **Neutral protocol / commons** — public specifications, conformance, protocol governance and designated public-good infrastructure.
2. **Mycelix Systems** — commercial products, managed infrastructure, integrations, support, assurance and customer contracts.
3. **Federation participants** — independently governed organizations, communities and institutions.
4. **Project-specific deployments** — municipal, sovereign, network or other infrastructure projects that may require project-specific financing.

No one domain inherits authority over another merely because money flows between them.

## 3. Neutral protocol boundary

A future neutral entity may steward, subject to final legal implementation:

- canonical public protocol specifications;
- protocol/profile registries;
- interoperability and conformance rules;
- certification/trademark policy where appropriate;
- governance procedures for protocol evolution;
- designated public-good/reference implementations;
- transparent compatibility and deprecation policy.

The neutral entity should not need to be the exclusive commercial vendor.

A commercial company's failure, acquisition, recapitalization or change of investors must not silently transfer neutral protocol authority unless an explicit, independently authorized constitutional transition says so.

## 4. Commercial company boundary

Mycelix Systems may own or operate commercial value including:

- managed Mycelix infrastructure;
- enterprise control planes;
- deployment automation and operations;
- commercial adapters and integrations;
- support and service-level agreements;
- assurance, certification and compliance services;
- customer-specific implementation work;
- lawful commercial licensing rights;
- proprietary operational tooling that does not redefine open protocol truth.

Investors may participate economically in Mycelix Systems without thereby becoming protocol administrators.

## 5. Federation sovereignty

A Mycelix participant remains an independently governed principal.

Participation does not imply transfer of:

- participant identity ownership;
- participant data ownership;
- institutional authority;
- internal policy authority;
- constitutional authority;
- ownership of participant-generated evidence.

Federation means interoperable trust, not one global administrator.

## 6. Capital routing

Different work should use capital suited to its risk and public/private character.

| Layer | Preferred capital classes |
|---|---|
| Fundamental research | grants, philanthropy, research support |
| Neutral/open protocol | grants, memberships, sponsorship, public support |
| Enterprise products | founder capital, customer revenue, venture/strategic equity |
| Mature recurring software | retained earnings, growth equity, debt where appropriate |
| Municipal / national DPI | procurement, development finance, blended/public finance |
| Physical/network infrastructure | project and infrastructure finance |
| Community ecosystem | memberships, sponsorship, grants |

This table is a routing policy, not a claim that any source is currently available.

## 7. Capital rights taxonomy

Any financing instrument should make rights explicit rather than implied.

A future machine-readable capital-rights profile should be able to represent at least:

- economic participation;
- board or observer rights;
- information rights;
- commercial-company voting rights;
- reserved matters;
- protocol-governance rights, if any;
- certification/trademark influence, if any;
- project-specific covenants;
- use restrictions;
- expiration/redemption conditions where applicable.

Absence of a protocol right means the investor does not receive it merely by holding an economic interest.

## 8. Anti-capture invariants

The v1 constitutional target is:

1. Investment cannot change a technical `FAIL` into `PASS`.
2. Investment cannot redefine a user's identity or data as investor-owned property.
3. Investment cannot require every federation participant to use one hosted provider.
4. A customer contract cannot privately redefine shared protocol semantics for all users.
5. A government-funded deployment cannot silently redefine global protocol policy.
6. A grant restriction cannot silently apply outside the project/entity that accepted it.
7. Acquisition or insolvency of Mycelix Systems cannot silently transfer neutral protocol authority.
8. Trademark/certification authority must follow an explicit governance path.
9. Capital rights must remain distinguishable from runtime `AuthorityGrant` semantics.
10. Material financing provenance and control rights should be auditable at an appropriate disclosure level.

## 9. Capital provenance

A future `CapitalSourceRecord` or equivalent should be able to bind:

```text
source class
recipient entity/project
instrument class
governance/control-right profile
use restrictions
effective interval
disclosure class
policy/profile revision
```

This is governance evidence. It is not accounting authority, tax automation, securities-law compliance, or a substitute for counsel.

## 10. Investor compatibility

Mycelix should choose capital as deliberately as capital chooses Mycelix.

A non-authoritative internal diligence profile may evaluate:

- investment horizon;
- infrastructure/deep-tech tolerance;
- commercial-open-source compatibility;
- protocol-neutrality compatibility;
- control appetite;
- strategic/customer value;
- follow-on capacity;
- sovereign/public-interest compatibility;
- institutional and reputational risk.

A weighted score must never override a hard constitutional incompatibility.

## 11. Legal implementation boundary

This specification does not choose the final legal form.

Possible tools may include conventional equity, a public-benefit corporation, nonprofit/foundation ownership, dual-class rights, reserved matters, steward-style instruments, contractual covenants, project vehicles, or jurisdiction-specific structures.

Qualified legal counsel should map the constitutional intent into enforceable documents. The implementation should be reviewed against this specification rather than silently redefining it.

## 12. Relationship to Mycelix runtime authority

Capital authority is not a parallel runtime authority system.

```text
capital rights
!= PrincipalId
!= AuthorityGrant
!= delegated current authority
!= effect authority
```

If corporate, foundation or investor actors perform consequential operations inside Mycelix, those operations should traverse the same qualified authority architecture as other principals.

## 13. Promotion gates

`MYC-CAP-001` should not be considered complete until:

- the entity/control boundary is diagrammed;
- a machine-readable rights vocabulary exists;
- anti-capture regression cases are frozen;
- IP ownership/licensing authority is reconciled;
- legal-counsel questions are separated from technical claims;
- the model is cross-checked against the Trust Fabric commercial projection;
- capital provenance is represented without requiring public disclosure of confidential terms.

## 14. Nonclaims

This document does not establish:

- securities, tax or corporate-law compliance;
- fiduciary suitability;
- investor return;
- valuation;
- fundraising success;
- product-market fit;
- technical correctness;
- democratic or legal legitimacy.

It defines the intended boundary: **capital may participate in economic upside without automatically purchasing the authority that makes Mycelix trustworthy.**
