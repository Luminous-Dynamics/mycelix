# Adversarial fixtures

All fixtures are synthetic and deterministic. They contain no real worker, employer, assessor, credential, work order or event.

Run the package test suite from this directory's parent Craft workspace:

```sh
cargo test -p craft-work-evidence-v1
```

The JSON corpus is the declared fixture inventory; the Rust test maps each mutation identifier to executable signed/unsigned test inputs. Adding a fixture requires adding the corresponding verifier mutation and updating the corpus cardinality assertion.
