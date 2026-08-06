# Canonical source closure

Mycelix Pulse is developed inside a larger Luminous Dynamics source tree. The
restricted-alpha repository still has three intentional path dependencies from
the Leptos application into sibling components:

- `mycelix-leptos-core`
- `mycelix-leptos-client`
- `mycelix-crypto`

They are declared in `config/canonical-source-closure.json`. The declaration
check catches new hidden sibling dependencies and stale declarations without
requiring Cargo or network access:

```sh
just check-source-closure
```

A build or release must use the strict mode, which additionally requires every
path target to be present:

```sh
just check-source-closure-present
```

A standalone archive that reports missing sibling dependencies is reviewable,
but it is not a source-complete build artifact. Release evidence must preserve
the JSON output of the strict audit alongside compiler and packaging results.
