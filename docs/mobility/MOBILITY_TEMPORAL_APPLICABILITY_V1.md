# Mobility Commons — Temporal Configuration Applicability V1

## Purpose

An AppliesTo relationship between a configuration revision and a physical artifact is an identity/lineage assertion. It must not be interpreted as timeless or currently effective merely because the edge exists.

MOBILITY-COMMONS-026 adds explicit temporal scope.

## Interval semantics

ApplicabilityInterval is a half-open interval [start, end).

- start is included.
- bounded end is excluded.
- end must be greater than start.
- end omitted means the interval is open-ended.
- adjacent intervals do not overlap.
- overlap is deterministic.

## Identity binding

TemporalConfigurationApplicability requires:

- ConfigurationRevision as the configuration identity;
- PhysicalArtifact as the artifact identity;
- an exact AppliesTo edge whose source equals the configuration;
- an exact AppliesTo edge whose target equals the artifact;
- a valid applicability interval.

Holochain protocol hashes remain protocol metadata and cannot silently become native engineering identities.

## Boundary

Temporal scope answers when an applicability assertion is scoped. It does not answer whether the configuration is physically safe, conforming, certified, regulatorily accepted, or measurement-true.

This is consistent with NIST work showing that digital threads need internal temporal associations across heterogeneous lifecycle data, including explicit timestamp intervals for manufacturing execution. It also complements NIST's lifecycle-wide digital-thread work on traceability, interoperability, and trustworthy digital-twin/manufacturing data.

Qualification remains semantic and structural only.
