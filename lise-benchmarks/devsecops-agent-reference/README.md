# LISE DevSecOps agent identity and effect qualification profile

**Status:** synthetic reference model, based on a research profile aligned to the NIST NCCoE DevSecOps-first use case announced September 29, 2026. This is not a NIST certification or an actual GitHub authorization integration.

The benchmark separates capabilities that are often blurred in agentic coding workflows:

- inspect repository / create branch / push commit / open PR;
- approve PR / merge protected branch;
- create release / sign artifact;
- obtain CI workload identity / access secrets;
- deploy to staging / deploy to production;
- reconcile a release/deployment timeout.

A code agent's permission to prepare a patch never implies approval, merge, release or deploy permission.

## Research basis

NIST NCCoE's September 29, 2026 update says its first implementation use case will demonstrate software and AI-agent identity, authentication and authorization in the software development lifecycle with the DevSecOps project. It reports more than 600 public comments and additional use cases will be scoped later.

- https://www.nist.gov/news-events/news/2026/09/comments-software-and-agentic-ai-identity-concept-paper
- https://www.nccoe.nist.gov/projects/software-and-si-agent-identity-and-authorization

Related agent authorization and side-effect benchmarks:
- AuthorizationBench: https://arxiv.org/abs/2610.27548
- SABench: https://arxiv.org/abs/2610.34167

## Run

    cargo fmt --manifest-path lise-benchmarks/devsecops-agent-reference/Cargo.toml --check
    cargo check --all-targets --manifest-path lise-benchmarks/devsecops-agent-reference/Cargo.toml
    cargo test --all-targets --manifest-path lise-benchmarks/devsecops-agent-reference/Cargo.toml
    cargo run --manifest-path lise-benchmarks/devsecops-agent-reference/Cargo.toml -- \
      lise-benchmarks/devsecops-agent-reference/fixtures/cases.json \
      lise-benchmarks/devsecops-agent-reference/fixtures/oracle.json \
      /tmp/devsecops-reference-report.json

## Core invariants

1. Agent principal identity is distinct from a human reviewer and from a CI workload identity.
2. Grants bind exact principal, action, repository, branch, environment, purpose, validity and policy digest.
3. Approval binds the exact current PR head, repository, branch, action, environment, policy digest and a qualified, distinct human reviewer.
4. Production deployment requires CI workload identity with exact subject, audience, repository, workflow reference and workflow SHA.
5. Artifact attestation binds subject digest, commit, builder identity, workflow reference/SHA and trusted signer.
6. Unknown external release/deployment outcomes hold for reconciliation; blind retries are reported as explicit violations.
7. Issue/README/build-log prompt injection is untrusted context and cannot extend a grant.
8. The evaluator always reports \`dispatch_eligible=false\`.

## Cases

The 16 candidate-visible cases and separate evaluator oracle cover feature-branch authoring, self-approval, stale approvals, workload audience substitution, valid production deploy, wrong artifact digest, unknown deployment outcome, blind retry, prompt injection, authority revocation and policy drift.

## Claim ceiling

The fixtures use synthetic grants, policies, approvals, workload identities and attestations. This crate does not authenticate a GitHub actor, inspect a live repository, enforce branch protection, validate a Sigstore/SLSA attestation, read a CI token or perform a deployment. The benchmark reference is not a production security verdict.


Workload identity now includes a separately trusted OIDC issuer, exact audience, repository, workflow reference/SHA, observation time and action-time validity interval. The expiry negative control ensures a previously valid CI workload token cannot authorize a deployment after its validity window.