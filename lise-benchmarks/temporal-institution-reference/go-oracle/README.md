# Independent Go replay

This directory contains a separately implemented Go replay of the v0.1 temporal-institution transition contract. It is deliberately outside the Rust crate and uses only the Go standard library.

The test suite reads the frozen Rust fixture inputs from `../fixtures/scenarios.json` and compares its independently computed disposition and reason code with `../fixtures/oracle.json`. It also checks strict JSON decoding, unique one-to-one case identity, oversight restoration evidence, unknown emergency expiry, hard-violation precedence and rejection of unknown transitions.

Run locally:

```sh
go test -race ./...
go vet ./...
```

This cross-language replay is a second implementation check, not an independent adjudication of the policy oracle, a formal proof, an institution inventory, or a production enforcement mechanism. Both implementations remain scoped to the finite synthetic contract and its explicit omissions.
