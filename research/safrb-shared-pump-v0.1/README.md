# SAFRB shared-pump claim-fidelity fixture

This is a synthetic companion fixture for SAFRB scenario `SAFRB-SHARED-PUMP-2ROUTE-001` (https://github.com/Luminous-Dynamics/symtropy/pull/7543).

It exercises Mycelix-owned claim semantics, not physical flow:
- both logical route references resolve to one participant-owned physical pump identity;
- an authenticated issuer key is distinct from recognition for a declared claim purpose;
- reported capacity is distinct from verified capacity and current availability;
- title, custody, contract access and dispatch authority are distinct dispositions;
- shared-resource identity is a dependency fact, not a trust score.

The canonical physical result is owned by Symtropy. Mycelix should validate recognition, provenance, semantic fidelity and common-cause closure using existing owners (#5963/#5762); it must not recalculate max-flow or grant operational access.

## External alignment

Align the provenance and selective-disclosure design with **NIST IR 8536 (final, September 2026), Supply Chain Traceability: Manufacturing Meta-Framework**. It describes cryptographically verifiable links, temporally ordered traceability, participant-specific templates and selective disclosure without requiring a centralized repository. Mycelix should reuse these principles rather than create a competing generic traceability chain. Its added assurance responsibility here is purpose-specific recognition, claim semantics, common-cause dependency closure and infrastructure-service boundaries.

Reference: https://csrc.nist.gov/pubs/ir/8536/final

**Synthetic only.** No real issuer, organization, title, measurement, credential or legal recognition is represented.

## Executable test-only checks

The fixture now binds both the single-window and two-window Symtropy known answers, with per-window capacity bounds and validity intervals.

Run the independent standard-library checks:

```sh
python3 research/safrb-shared-pump-v0.1/verify_claim_fidelity.py
```

The verifier checks issuer recognition, claim scope, title/custody/contract/dispatch separation, common physical-resource identity, common-cause closure, source-scenario digests and exact capacity bounds for each time window. Its mutation controls reject eight deliberate authority, currentness, alias, dependency and temporal promotions.

This script is test-only. It does not verify cryptographic signatures, a real issuer, legal recognition, actual measurements, or a production Mycelix adapter. The companion workflow is included for execution when this workflow definition is available to the repository's Actions event configuration; no CI pass is claimed for this draft head.
