# Economic Fabric Runtime Reference

This crate-free Rust reference is intentionally runtime-neutral. It is a semantic witness for the Economic Fabric V1 contract, not a ledger, currency engine, payment processor, accounting system, or financial authority.

## Purpose

It provides deterministic transition and settlement predicates that adapters can refine.

## Claim ceiling

Passing the tests demonstrates only the encoded semantic boundaries. It does not establish financial correctness, legal compliance, security, economic fairness, or real-world performance.

## Refinement target

A production implementation must map each predicate to an exact production symbol, executable test, formal artifact, source commit, and reproducible build identity before any obligation can be considered formally closed.
