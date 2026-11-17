# IPB-S6 v0.2 — authority provenance and transaction red-team cases

**Status:** synthetic development corpus. Not frozen, independently adjudicated, cryptographically validated or a performance result.

Related:
- [Issue #6130 — IPB-S6 v0.2 red-team benchmark](https://github.com/Luminous-Dynamics/mycelix/issues/6130)
- [Issue #6127 — IPB-S6 v0.1 development corpus](https://github.com/Luminous-Dynamics/mycelix/issues/6127)
- [Symthaea #12019 — integrated assurance vertical slice](https://github.com/Luminous-Dynamics/symthaea/issues/12019)
- [Mycelix PR #6131 — assurance exchange packet](https://github.com/Luminous-Dynamics/mycelix/pull/6131)

The initial eight cases cover valid-proof/untrusted-issuer, revocation TOCTOU, unknown credential status, timeout-after-submission, delayed success receipt, common-cause upstream dependencies, source competence mismatch, and source digest mismatch.

The input corpus and oracle are separate. Do not expose the oracle to systems under test. The oracle has not been independently adjudicated. Use only synthetic credentials and references.

See [PROTOCOL.md](./PROTOCOL.md) for metrics, comparator requirements, fault-injection expansion and prospective hard gates.