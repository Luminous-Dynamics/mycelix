# FORGE-008B — Concrete Git source adapter

`forge-git-source` implements the FORGE-008A `ProposalSourceVerifier` contract with exact Git plumbing.

The adapter never invokes a shell. The process runner requires an absolute Git executable path, clears the ambient environment, disables system/global Git configuration, terminal prompting, replacement refs and lazy object fetching, and pins the exact reported Git version into its adapter identity.

For one exact proposal it verifies:

```text
repository object format == proposal object format
base object type          == commit
proposed object type      == commit
resulting object type     == tree
proposed^{tree}           == proposal.resulting_tree
merge-base --is-ancestor base proposed == success
```

The resulting FORGE-008A observation commits to the exact content-addressed `(base, proposed, tree)` source subject plus deterministic ancestry/tree evidence commitments. `ProposalSourceVerifier::verify_source` re-runs the Git inspection and refuses any mismatch with the supplied observation.

## Claim boundary

This adapter proves Git semantic relationships for exact object IDs. It does **not** prove that the repository/object database is portable or self-contained and does not prohibit object alternates as an independent storage mechanism. Missing history or lazily unavailable objects fail the semantic checks; portable closure is established separately by Forge's bundle/offline-evidence stack.

A later hermetic profile can execute this adapter with the exact Nix-qualified Git artifact already used by the M0 verifier.
