# D6U Holochain 0.7 runtime authority-boundary harness

Status: Experimental runtime harness; ReferenceModelOnly

This package is intentionally separate from the Mycelix manufacturing workspace. It pins the Holochain 0.7 test substrate instead of changing the existing manufacturing HDK/HDI stack.

Pinned runtime:

- Holochain 0.7.0
- HDK 0.7.0
- HDI 0.8.0 through the Holochain 0.7 dependency graph
- Rust 1.96.1, matching the Holochain 0.7.0 repository toolchain
- holochain_serialized_bytes 0.0.57

The integration test uses the real Sweettest conductor, real inline-zome execution, real AppRequest::CallZome messages, real ZomeCallParamsSigned signatures, and native capability-grant entries.

Observed cases targeted by the harness:

- canonical payload accepted;
- authorized but semantically rejected payload;
- authenticated but D6S-commitment-inconsistent payload;
- invalid call signature;
- author-grant execution;
- explicit assigned capability;
- wrong capability secret;
- capability revocation;
- provenance mismatch using a valid secret assigned to another agent;
- nonce replay;
- expired invocation;
- wrong zome;
- wrong function;
- wrong cell.

The reference matrix contains two cases deliberately outside this harness:

- isolated authenticated-but-not-yet-authorized state;
- distinct stale/older-nonce state.

Blocked provenance is now exercised through Holochain's system-level P2P block mechanism.

As a supplemental substrate check, the runtime also exercises Holochain 0.7's excessive-future-expiry rejection. This is not substituted for the canonical `nonce-stale` reference case.

## Protocol-level outcome classes

The runtime log binds each supported case to one expected protocol-level outcome:

- `accepted`: app-interface call reached successful zome execution;
- `semantic-rejected`: authorization succeeded and the probe deliberately rejected the payload semantically;
- `d6s-commitment-mismatch`: authorization succeeded and the frozen D6S commitment check rejected the mutated payload;
- `authentication-failed`: the app interface returned `ZomeCallAuthenticationFailed` for the invalid signature case;
- `authorization-failed`: the app interface returned `ZomeCallUnauthorized` for capability, provenance, nonce, or expiry failures;
- `routing-failed`: the app interface rejected an invalid zome, missing function, or missing cell before zome execution.

These outcome classes are verified against the frozen manifest by `scripts/integral/verify_d6u_runtime_evidence.py`.

Each supplemental substrate check emits two separate records: a `D6U_RUNTIME_WITNESS` line containing the error returned by the Holochain runtime, and a `D6U_SUBSTRATE_CHECK` line asserting that the expected witness was observed. The verifier requires the witness and PASS records to cover the same declared checks.

A successful run is runtime evidence for this fixture only. It does not establish Mycelix semantic truth, legal authority, production safety, physical outcomes, or actuation authority.

Claim ceiling: ReferenceModelOnly.
