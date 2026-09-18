# QCAP v1 canonical vectors

These vectors pin the runner-neutral QCAP v1 canonical identity law independently of the Python dispatcher.

Registered vector:

- capsule commitment: `81699b3150a24fee586e679c473e8777fc4a672cfb2f8f2ac924929d93a287f8`
- receipt commitment: `659f9c66ad8496bf3ef9bddf0db1298bd271ab2d0b1175d0dc0bdcf7e3884339`
- fixture: `qcap-v1-vector-001.json`
- independent verifier: `verify_qcap_vectors.py`

The fixture deliberately covers UTF-8 values (`café`, `雪`, `é`), dictionary insertion-order independence, integer-only canonical numeric encoding, capsule domain separation, receipt domain separation, and a complete conjunctive PASS receipt.

The verifier MUST remain independent: it does not import `qualification/runner/qcap.py`. A future Rust, Nix, or other implementation can use these commitments as interoperability vectors.

The independent verifier prints `independent_qcap_v1_vectors=PASS` only when both capsule and receipt commitments reproduce exactly.
