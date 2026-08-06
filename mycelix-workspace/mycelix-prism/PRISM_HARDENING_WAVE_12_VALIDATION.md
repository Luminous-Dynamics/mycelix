# Prism hardening Wave 12 — validation report

## Scope

This report covers the 17 ordered Wave 12 patches applied after the Wave 11
head. Wave 12 adds purpose-bound verifier-agent requests, threshold hybrid
release witnesses, self-contained transparency receipts, independent witness
re-audit, externally anchored recovery, and non-destructive evidence-retention
planning.

## Static and structural validation

The following checks passed on the final Wave 12 source tree:

- all 103 Rust source files parsed with the tree-sitter Rust grammar;
- all 29 TOML files parsed with Python `tomllib`;
- all 42 JSON files parsed;
- all 19 Python scripts compiled with `py_compile`;
- `scripts/verify-wave12-static.py` reproduced
  `security/wave12-static-evidence.json` exactly;
- `scripts/verify-supply-chain.py` reproduced
  `security/supply-chain-evidence.json` exactly;
- `git diff --check` passed;
- `git fsck --full` passed;
- the workspace contains 20 portable-core members and 838 locked packages;
- 395 Rust test, Tokio-test, and property-test annotations are present.

The Wave 12 delta changes 32 files with 4,260 insertions and 43 deletions before
this validation document.

## Security-boundary review

Manual review additionally confirmed:

- verifier-agent protocol v3 binds a fresh challenge and explicit operation
  purpose into the request ID and echoed response;
- quorum receipts require private-field verification tokens tied to exact
  witness, statement, and signature digests;
- quorum receipt schema v2 embeds the exact policy and signed witness-bundle
  bytes and commits to both file digests;
- transparency re-audit uses a separate verifier purpose and exact historical
  verification-key digests;
- recovery cannot infer a new trusted head and requires externally supplied
  statement and checkpoint roots;
- release-retention planning never marks a signed journal checkpoint for
  deletion or archival and has no mutation capability;
- cold-archive classification requires an externally anchored head, complete
  journal audit, and fresh transparency-quorum audit.

## Release-blocker accounting

Supply-chain release mode fails on exactly one declared blocker:

> `flake.lock` has not been generated, committed, semantically reviewed, and
> hybrid-signed on a Nix-enabled host.

No other static or supply-chain failure was observed.

## Replay validation

The sealed 17-patch Wave 12 series replayed cleanly onto an untouched clone of
the Wave 11 Git bundle. The complete 161-patch campaign also replayed cleanly
from baseline commit `d003022950177bd48af4427baef168123515a1db`. Both replay
lanes produced the exact final Git tree:

`11ae747beeeeff16f543ef73cdb41e2ea0bf6531`

Both replay worktrees were clean after patch application.

## Unexecuted gates

This environment does not provide Cargo, Rust, Nix, Bubblewrap, Chromium, or a
production verifier agent. Therefore this report does not claim that the
following executable gates passed:

- Rust formatting, compilation, Clippy, or test execution;
- Nix flake evaluation or closure builds;
- namespace and egress-broker adversarial tests;
- live Ed25519 and ML-DSA-65 verification;
- offline witness signing or the complete Wave 12 live ceremony;
- generation and semantic review of `flake.lock`.

Those remain mandatory on the canonical Nix-enabled release host.
