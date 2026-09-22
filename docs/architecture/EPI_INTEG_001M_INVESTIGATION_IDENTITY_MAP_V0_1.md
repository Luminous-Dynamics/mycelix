# EPI-INTEG-001M — explicit investigation identity map v0.1

Status: **FROZEN CORPUS / NOT EXECUTED / NOT QUALIFIED / NOT PASS**

This subject freezes fixture correspondence between the prepared Symthaea closed-world investigator and the prepared Mycelix EPI-012A record capsule.

## Exact subjects

Symthaea source subject:

- repository: `Luminous-Dynamics/symthaea`
- head: `90267527cba13a1e0be8c424ebce03f1f5726e46`
- profile: `symthaea:closed-world-investigation-loop:v1`

Mycelix destination subject:

- repository: `Luminous-Dynamics/mycelix`
- head: `59dcabafd28b2020f09abe4544d731ab720143d1`
- profile: `mycelix:epi:investigation-capsule:v1`

## Governing theorem

```text
same spelling != same identity

repository-local ID
+ local role
+ subject/profile
!= another repository's local ID
```

For example:

```text
Symthaea FrontierRef("F2")
!= Mycelix FrontierRecordV1::id("F2")
```

until this exact fixture correspondence is applied.

## Binding classes

### CrossRepoCorrespondence

A typed, fixture-scoped assertion that one local identity corresponds to another local identity for this exact synthetic investigation.

It does **not** establish semantic equality outside the fixture, truth, evidence admission, collection authority, or execution authority.

### MycelixLocalOnly

A destination identity intentionally has no Symthaea counterpart in this fixture. Examples include historical F1 state and Mycelix-only protected-omission records.

### NoExportByPolicy

A protected destination record or raw detail is intentionally not mapped/exported. Absence of a mapping is explicit policy state, not negative evidence.

## Frozen correspondence set

The machine-readable fixture freezes explicit bindings for:

- `artifact:A1/A2/A3` -> `AR1/AR2/AR3`;
- Symthaea `F2` -> Mycelix `F2`;
- planner proposals `D1..D5` -> the corresponding Mycelix recorded planner-history proposal refs;
- selected methodology `T_WEB_PUBLIC_TOPK` -> the Mycelix selected methodology profile ref;
- the exact Symthaea closed-world loop subject -> Mycelix `SYMCAND:F2` as an external candidate record.

The following are explicitly **not** invented as cross-repo mappings:

- Mycelix F1;
- AR4 maintenance artifact;
- assumption ledger records;
- DEP:G1 as a canonical Mycelix dependency record;
- protected omission `OMIT1` / protected raw content;
- Sol-Atlas projection `ATLAS:F2`.

## Identity binding fields

Every correspondence binds:

```text
binding_id
binding_profile
source_repo
source_subject_head
source_subject_profile
source_role
source_local_id
destination_repo
destination_subject_head
destination_subject_profile
destination_role
destination_local_id
binding_kind
```

## Authority ceiling

```text
CrossRepoCorrespondence
!= canonical EPI admission
!= evidence relation
!= factual truth
!= source authenticity
!= identity proof outside fixture
!= search authority
!= execution authority
```

The fixture carries:

```text
admission_authority = false
collection_authority = false
execution_authority = false
```

## Required hostile checks

A qualifier must reject or explicitly detect:

1. source repository/head/profile substitution;
2. destination repository/head/profile substitution;
3. source-role substitution with unchanged local text;
4. destination-role substitution with unchanged local text;
5. one source role+ID mapped to conflicting destinations;
6. one destination role+ID mapped from conflicting sources where one-to-one is required;
7. silent spelling-based acceptance of an unmapped identity;
8. attempted mapping of protected raw content excluded by policy;
9. a correspondence interpreted as an EPI admission;
10. any change to the exact subject heads without a new mapping version.

## Nonclaims

This corpus does not qualify either repository, establish interoperability beyond the exact fixture, prove semantic truth, admit evidence, authorize collection, or authorize execution.
