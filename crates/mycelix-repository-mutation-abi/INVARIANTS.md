# QUAL-001G3A — Repository Mutation Subject Shape V1

This crate is the first implementation slice of #1574.

## Establishes

`qualify_repository_mutation_subject_v1(...)` may produce a non-deserializable positive
`QualifiedRepositoryMutationSubjectV1` only when:

- the exact V1 subject profile is selected;
- repository identity is explicit and bounded;
- the target is a canonical `refs/heads/...` ref;
- unsafe/ambiguous repository paths are rejected;
- the mutation set is non-empty and bounded;
- each changed path is unique after canonical ordering;
- Add/Replace/Delete field presence is internally consistent;
- replacement is not a no-op Git-blob identity;
- expected old ref state is the commit parent by construction;
- deterministic commit parent matches the exact expected old ref state;
- deterministic commit tree matches the declared candidate tree;
- the candidate tree differs from the old tree;
- canonical subject bytes are independent of caller mutation ordering.

## Typed distinctions

The ABI deliberately keeps these types distinct:

- `GitSha1ObjectId`
- `RawFileSha256`
- `SemanticCommitmentDigest`
- `SemanticContentCommitmentV1`
- repository identity
- target ref
- repository path
- deterministic commit metadata

Equal-looking bytes do not make those commitment kinds substitutable.

## Does not establish

G3A does **not** establish:

- a cryptographic candidate commitment;
- SHA-256 computation over the canonical subject;
- Git blob computation from file bytes;
- Git tree reconstruction;
- Git commit-object reconstruction;
- artifact transport identity;
- repository publication authority;
- a Git ref update;
- publication commit truth/recovery;
- semantic correctness of candidate product bytes;
- product qualification;
- provider-request materializer safety.

Those remain successors under #1292, #1394, #1483, #1573 and #1574.

## Authority rule

The positive subject object is intentionally incapable of granting repository writes.

```text
QualifiedRepositoryMutationSubjectV1
-/-> repository write authority
-/-> publication
-/-> product qualification
```

## Migration

Frozen/in-flight workflows such as #1547, #1550 and #1553 are historical pilots.
Do not rewrite them solely to adopt this ABI. Successor workflows may converge after
this subject-shape theorem receives independent qualification.
