# CI-GOV-001F — GitHub Actions immutable-reference inventory v1

Status: read-only inventory contract. This tranche does not modify workflow behavior, update dependencies, or enforce pinning.

## Governing theorem

`action tag/name != immutable action identity`

`immutable action identity != trusted action semantics`

`pinned action != qualified workflow`

The purpose of this tranche is to make the repository's workflow dependency surface measurable before migration. It classifies executable `uses:` references without resolving tags, contacting GitHub, editing workflows, or claiming that an immutable reference is safe.

## Scope

The v1 scanner walks `.github/workflows/**/*.yml` and `.github/workflows/**/*.yaml` in the checked-out repository and classifies actual YAML `uses:` keys. Comment-only lines are ignored. YAML block-scalar bodies such as `run: |` / `run: >` are indentation-tracked and excluded, so shell/script text containing a string that looks like `uses:` does not become an action dependency. Quoted `"uses"` / `'uses'` mapping keys and simple flow-style step mappings such as `- { name: checkout, uses: actions/checkout@v4 }` are also recognized.

The scanner recognizes these families:

- `LocalAction` — repository-local `./...` action;
- `LocalReusableWorkflow` — repository-local reusable workflow;
- `ImmutableExternalAction` — external action pinned to a full 40-hex Git commit SHA;
- `MutableExternalAction` — external action using a tag, branch, shortened SHA, or other mutable/non-full-SHA ref;
- `ImmutableReusableWorkflow` — external reusable workflow pinned to a full 40-hex Git commit SHA;
- `MutableReusableWorkflow` — external reusable workflow using another ref form;
- `ImmutableDockerImage` — `docker://...@sha256:<64-hex>`;
- `MutableDockerImage` — Docker reference without an immutable SHA-256 digest;
- `InvalidUsesRef` — an unrecognized or malformed executable `uses:` value.

This classification is deliberately syntactic. It does not resolve whether a SHA corresponds to a published release, whether an action repository is trustworthy, or whether the action performs network/download operations at runtime.

## Inventory-only first tranche

The frozen manifest sets `enforcement_enabled = false`.

Therefore:

`InventoryOnly PASS != repository action-pinning PASS`

The scanner may report mutable references while still exiting successfully in inventory mode. Enforcement is a separate future profile change requiring explicit review.

The implementation also contains a dormant fail-closed enforcement mode. While the manifest keeps enforcement disabled, asking for enforcement returns `EnforcementNotEnabled` rather than silently treating inventory as compliance.

## Parsing boundary

The scanner is intentionally zero-network and standard-library only. It scans YAML text for real `uses:` mapping keys rather than searching arbitrary substrings. It ignores full-line comments, strips inline YAML comments only when they occur outside quoted values, skips indentation-delimited block-scalar bodies, recognizes quoted block-style keys, and recognizes the bounded flow-map form exercised by the frozen self-test.

This is not a general YAML parser. Unsupported/ambiguous executable reference forms classify as `InvalidUsesRef` when detected; they are not guessed into compliance. A future enforcement tranche should prefer a real YAML parser or independently prove the accepted syntax subset before claiming complete workflow-language coverage.

## Update discipline

Future migration should proceed in bounded steps:

1. freeze and review inventory behavior;
2. pin `actions/checkout` consistently;
3. pin remaining external actions in bounded groups;
4. enable static enforcement only after the reviewed workflow surface is migrated;
5. keep human-readable release/tag information adjacent to immutable executable refs;
6. perform online provenance lookup, if desired, in a separate bounded update tool—not in the offline verifier.

## Nonclaims

A CI-GOV-001F inventory result does not establish action safety, publisher identity, tag-to-SHA provenance, GitHub runner integrity, artifact integrity, transitive dependency integrity, workflow semantic correctness, product correctness, reproducibility, or qualification of any workflow.

It proves only how the frozen scanner classifies the checked-out `uses:` surface under the frozen profile.