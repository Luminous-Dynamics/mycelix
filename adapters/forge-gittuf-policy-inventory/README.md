# mycelix-forge-gittuf-policy-inventory

FORGE-004D2C2 implements a complete structural trust-method inventory of the exact active gittuf v0.16.0 policy state.

It does not parse `gittuf policy list-principals` human output. The collector uses Git plumbing, inventories the complete policy tree shape, enumerates every metadata blob, decodes each DSSE payload, and parses the pinned v0.1/v0.2 policy schemas.

## Exact policy binding

The collector snapshots `refs/gittuf/policy`, reuses FORGE-004B's `gittuf_policy_subject_commitment`, and requires that commitment to equal the supplied `RepositoryPolicyState.policy_digest`. The policy ref is read again after the complete scan; movement fails closed.

The receipt binds both the exact policy commit tree and its `metadata` subtree.

## Local-only policy tree

A gittuf policy commit can contain controller metadata as additional top-level subtrees. The first local-only profile therefore requires the policy commit tree to contain **exactly one** entry:

- `metadata/` as a Git tree.

Any sibling entry fails closed. Inside `metadata/`, every entry must be a regular `*.json` blob. `root.json` and `targets.json` are mandatory; every additional blob is treated as delegated targets metadata and scanned.

## Schema-aware complete inventory

The DSSE `payloadType` must be exactly `application/vnd.gittuf+json`.

For v0.1 metadata, key maps are inventoried. For v0.2 metadata, both direct-key principals and `Person` principals are supported, including every key nested in a person's `keys` map. Unknown principal shapes, key types, malformed schema-version fields, unsupported metadata versions, malformed DSSE/base64, nested metadata trees, and unexpected files fail closed.

The inventory is intentionally conservative: every declared principal/key contributes its verification method, even if currently unused by a rule.

Recognized key types are:

- `ssh` -> SSH;
- `gpg` -> GPG;
- `sigstore-oidc` -> Sigstore.

A single Sigstore method is rejected by FORGE-004D2C1's local-key profile.

## External policy features

The first hermetic local-key profile additionally rejects non-empty root state for:

- `propagations`;
- `multiRepository`;
- `hooks`.

These can introduce remote policy authority or executable policy logic and require separate hermetic profiles.

## Object integrity boundary

The collector operates on exact Git object IDs using `rev-parse`, `ls-tree -z`, and `cat-file blob`, with lazy fetch and ambient Git configuration disabled. Final hermetic composition must run a pinned Git `fsck --full --strict` preflight on the exact policy/repository subject before inventory and verification; D2C2 itself does not turn Git object validation into a second policy-parser responsibility.

## Receipt

`PolicyInventoryReceipt` binds:

- exact Forge repository-policy-state digest;
- exact gittuf policy-tip object;
- exact policy tree object;
- exact metadata tree object;
- every metadata blob object;
- every decoded payload digest;
- per-file trust-method set;
- final `PolicyTrustInventory` commitment.

## Claim boundary

This collector proves what trust methods are structurally present in one exact local policy tree. It does not by itself prove full Git history, kernel isolation, runtime NAR closure, executable provenance, or final repository `OfflineEvidence`; those claims are composed separately.
