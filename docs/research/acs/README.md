# ACS Profile Contract v1

**Standard release:** OWASP Agent Control Standard v0.1.4  
**Schema namespace:** v0.1.0  
**Retrieved:** 2026-11-15  
**Status:** read-only crosswalk input; not an official ACS implementation or certification

The machine-readable manifest `ACS_PROFILE_CONTRACT_V1.json` pins:
- ACS-Core mandatory minimum hooks versus additional hooks that SHOULD be emitted when observable;
- MUST-support and SHOULD-support disposition semantics, including MODIFY fallback;
- baseline session chain/replay/signature semantics;
- the six optional profiles and their actual profile-specific requirements;
- self-declared handshake profile claims versus verified implementation evidence;
- multi-tenancy and A2A features deferred from v0.1.4 to v0.2;
- LISE read-only adapter outputs and non-authorization constraints.

Primary normative references:
- https://owaspacs.org/docs/spec/conformance/
- https://owaspacs.org/docs/spec/instrument/specification/
- https://owaspacs.org/docs/topics/faq/

This manifest is a manually curated, version-pinned research mapping. It is not the normative ACS schema set and does not replace validation against the 44 published JSON Schemas. Reconcile it against upstream changes before claiming conformance to a later ACS release.

**Claim boundary:** a manifest parse or mapping pass does not establish ACS implementation conformance, complete mediation, secure policy, tenant isolation, trusted Guardian operation or independent external effect observation.
