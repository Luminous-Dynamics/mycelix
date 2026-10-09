# Signed execution-evidence receipts research v1

Status: research-only protocol for execution provenance; no qualification decision is made here.

This layer makes the result of the research verifier stack auditable as a separate execution event. It addresses a final distinction:

    repository contains a verifier
        !=
    verifier ran over these exact inputs
        !=
    the hosted workflow completed successfully
        !=
    the system is qualified

## Source workflow boundary

The provenance regression workflow retains read-only repository permissions. After all upstream verifier steps, Python/Node report diffs, audit-bundle checks, and exact-head checks succeed, a deterministic receipt builder:

- validates that each expected Python/Node report exists;
- validates the expected case count, unique case identities and empty failure list;
- requires paired Python and Node report bytes to be identical;
- records SHA-256 hashes of all 28 reports;
- records the deterministic generated-corpus pair;
- binds the receipt to repository, workflow/ref, checked-out commit, event SHA, run ID, run number and run attempt;
- explicitly records `hosted_qualification_pass_claimed=false` and `qualification_authority=false`.

The receipt is uploaded only on source-workflow success. Each uploaded evidence artifact is named with the source run attempt, so retrying a workflow run cannot overwrite or accidentally reuse an artifact from an earlier attempt. The source workflow itself has no attestation-writing permission.

## Separate attestation workflow

A separate `workflow_run` workflow is restricted to successful **push** runs for `main` from the same repository. It rejects pull-request runs, other branches, failed/cancelled runs, mismatched commit/run identifiers, malformed or substituted reports, mismatched Python/Node outputs, and claim-ceiling weakening.

It downloads the artifact by exact source run ID, verifies every receipt/report hash, checks the source workflow-run event metadata, and independently emits an in-toto-style custom predicate in Python and Node. The two predicate outputs must be byte-identical.

Only then does the separate workflow use GitHub's OIDC/Sigstore-backed artifact-attestation action to attest the aggregate receipt. Signing authority is therefore not granted to the PR test workflow.

## What the attestation says

The subject is the deterministic execution receipt. The custom predicate binds:

- exact source workflow identity and commit;
- run ID, run number and attempt;
- each report pair's hash, schema and case count;
- the receipt hash;
- explicit limits on what the execution proves.

The attestation states that the upstream evidence-generation workflow completed successfully and that the listed report outputs were validated. It does **not** convert the research bundle into a qualification pass, prove real-world witness independence, or assert complete SCITT interoperability or live-network convergence.

## Trigger timing

A `workflow_run` workflow must exist on the default branch before it will run for upstream workflow completions. Therefore this PR's own queued or successful pull-request run will not emit the signed execution attestation. After this change is merged, a later successful push-to-main run of the research workflow can produce the attested receipt.

No future result is presumed. The attestation step can fail, be unavailable, or remain unexecuted; the signed receipt is evidence only when actually emitted and independently verified.

## Claim ceiling

Research-only.

Demonstrated by the intended flow:
- exact report input hashing;
- source-run metadata binding;
- two independent receipt validators;
- a separately permissioned GitHub/Sigstore attestation step.

Not demonstrated until a real main-branch run emits and verifies an attestation:
- that the attestation workflow actually completed;
- external verification by an independent relying party;
- complete SCITT interoperability;
- production VDS availability or network-wide gossip convergence;
- private-key custody or real-world organizational independence.

References:
- GitHub artifact attestations: https://docs.github.com/en/actions/security-for-github-actions/using-artifact-attestations
- `actions/attest`: https://github.com/actions/attest
- RFC 9942 COSE receipts: https://www.rfc-editor.org/rfc/rfc9942.html
