# SAFRB shared-pump claim-fidelity fixture

This is a synthetic companion fixture for SAFRB scenario `SAFRB-SHARED-PUMP-2ROUTE-001` (https://github.com/Luminous-Dynamics/symtropy/pull/7543).

It exercises Mycelix-owned claim semantics, not physical flow:
- both logical route references resolve to one participant-owned physical pump identity;
- an authenticated issuer key is distinct from recognition for a declared claim purpose;
- reported capacity is distinct from verified capacity and current availability;
- title, custody, contract access and dispatch authority are distinct dispositions;
- shared-resource identity is a dependency fact, not a trust score.

The canonical physical result is owned by Symtropy. Mycelix should validate recognition, provenance, semantic fidelity and common-cause closure using existing owners (#5963/#5762); it must not recalculate max-flow or grant operational access.

**Synthetic only.** No real issuer, organization, title, measurement, credential or legal recognition is represented.
