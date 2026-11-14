# IPB-S6 — multi-asset stewardship seed corpus v0.1

**Status:** development corpus; synthetic; not independently adjudicated or frozen for comparative claims  
**As-of:** 2026-11-14T00:00:00Z

## Purpose

Test whether a system can keep asset claims separate, respect source competence and freshness, preserve contested evidence, reconstruct as-of history, and block consequential actions when authority or external outcomes are unknown.

This is not an asset registry, title service, valuation engine, insurance system, family-office ERP, custody platform or legal opinion.

## Files

- schema.json — structural shape of scenario inputs.
- cases.json — fourteen synthetic scenario inputs, without expected answers embedded.
- oracle.json — separately stored reference outcomes and prohibited inferences.
- oracle-schema.json — strict shape of the separate reference oracle.
- PROTOCOL.md — comparator arms, scoring and freeze requirements.

## Scenarios

Healthy multi-claim reporting; custody/title confusion; stale valuation; unknown handover outcome; unresolved encumbrance; rights restrictions; insurance uncertainty; provenance conflict; partial source outage; cross-entity authority; succession gaps; common-cause dependencies; correction/as-of reconstruction; and duplicate identity collisions.

## Safe interpretation

A source's competence is claim- and profile-specific. A source may support custody without supporting title, or schedule listing without proving current coverage. Unavailable evidence is not a negative fact. A successful request receipt is not proof that a physical or financial side effect occurred.

All entities, assets, people, documents, amounts and circumstances are fictional. Do not substitute private client records or real security-sensitive locations into this public corpus.

## Current evidence state

The files are authored and machine-readable. The seed corpus has not yet been checked by an independent JSON Schema validator or reviewed by domain adjudicators. The oracle is an engineering reference for synthetic workflow tests, not adjudicated legal or factual truth. It must not be used to claim superiority over industry systems until the preregistered protocol is followed.
