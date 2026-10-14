# AC-056 — Multi-Jurisdiction Economic Context

## Purpose

An Economic OS event cannot always belong to exactly one jurisdiction.

A cross-border transaction may simultaneously involve:

- an origin jurisdiction;
- a destination jurisdiction;
- a settlement jurisdiction or monetary system;
- a reporting jurisdiction.

Forcing such an event through one policy profile would erase information that
matters for taxes, capital controls, payments, reporting, exchange rates, and
legal authority.

AC-056 therefore makes policy context a first-class, repeatable event binding.

## Policy context binding

Each context contains:

- role;
- versioned profile reference;
- exact policy-profile fingerprint.

The event envelope requires at least one context.

Structural roles such as Origin, Destination, Settlement, and Reporting may each
occur at most once. Additional explicitly labeled contexts can use Other.

## Why this matters for interoperability

BIS Project Agorá is explicitly exploring multi-currency programmable
cross-border payments and demonstrated atomic multi-currency settlement in its
2026 real-value testing. That is precisely the kind of environment where a
single-jurisdiction event model becomes insufficient.

The 2025 SNA and BPM7 also exist as coordinated but distinct international
statistical standards. A single economic event can therefore need different
interpretations depending on which reporting system is consuming it.

Mycelix should preserve the event once and allow multiple standards and
jurisdictions to produce their own representations.

## Envelope versioning

AC-056 changes the Economic OS envelope fingerprint domain from V1 to V2.

V2 replaces the singular policy-profile binding with a sorted set of explicit
policy contexts.

The event's semantic identity therefore includes the complete jurisdictional
context set.

## Core invariant

> One economic event may have multiple legitimate policy contexts, and none of
> those contexts should erase the others.

## What this does not do

AC-056 does not:

- determine which jurisdiction has legal priority;
- resolve tax or regulatory conflicts automatically;
- determine exchange rates;
- declare monetary sovereignty;
- create cross-border settlement by itself;
- assert that two legal systems recognize the same economic interpretation.

Those are functions of policy profiles, authority systems, and adapters.

## Research references

- BIS Project Agorá, updated May 2026:
  https://www.bis.org/project/agora
- UN System of National Accounts 2025:
  https://unstats.un.org/unsd/nationalaccount/sna2025.asp
- IMF BPM7:
  https://www.imf.org/en/publications/policy-papers/issues/2025/07/10/release-of-new-standards-for-macroeconomic-statistics-bpm7-568453
