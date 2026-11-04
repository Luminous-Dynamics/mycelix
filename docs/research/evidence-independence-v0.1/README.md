# Mycelix evidence independence v0.1

This directory contains a research RFC and a versioned adversarial fixture contract for issue [#4950](https://github.com/Luminous-Dynamics/mycelix/issues/4950).

- EVIDENCE_INDEPENDENCE_PROFILE_RFC_V0_1.md defines a multi-dimensional, provenance-graph model and compatibility requirements.
- cases.json contains 18 synthetic cases.
- cases.schema.json defines a Draft 2020-12 structural contract.

**Not implemented:** no production event type, provenance resolver, Rust test adapter, or scientific truth verdict is introduced by these files. The fixtures are a proposed conformance contract and must be executed against an independent implementation before they are treated as test results.

Core distinction: independence is profile-specific. A second person may establish reviewer diversity; it does not establish organization independence. A second organization may analyze a mirrored dataset; it does not establish new-data replication. Missing or conflicting metadata stays indeterminate.
