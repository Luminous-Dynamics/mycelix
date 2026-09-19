# mycelix-stewardship-policy-set

STEW-012D assesses the structural shape of multiple STEW-012B policy-authority **candidates** for one exact representation and one requested action.

It deliberately preserves symmetry:

```text
unverified Permit != authorization
unverified Prohibit != final denial
unverified DeniedUnspecified != final denial
```

The crate can report uniform candidate agreement, divergent permission requirements, mixed non-permission classifications, or mixed permission/non-permission classifications. It does not choose an authority, apply precedence, satisfy constraints/duties, or issue runtime capabilities.

All candidates in one assessment must target the same exact STEW-001 representation. Duplicate binding IDs and over-broad candidate sets are rejected.
