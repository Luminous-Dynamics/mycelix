# Craft Decision Closure V1

- **Work item:** CRAFT-DECISION-CLOSURE-001 (#6756)
- **Status:** Reference core; domain mapping and host qualification pending
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Research position

Gupta (2026), “Regulatory evidentiary closure as a formal framework for citation faithfulness in large language models,” *Discover Artificial Intelligence* 6, article 1919, published 20 November 2026, makes a critical distinction:

- source existence asks whether a cited source is recoverable from the authorized corpus;
- claim support asks whether that source supports one atomic proposition under the domain's rules;
- conclusion-relative closure asks whether the cited evidence supports every material element needed for the overall decision, including exceptions and scope conditions.

The paper shows that claim support does not imply closure. For example, a cited policy can support “annual review is required” while not supporting “the organization is compliant” because evidence of implementation, timing, authority, exceptions and documentation is still missing.

The article is a theoretical framework, not a validated audit instrument. It explicitly identifies inter-rater reliability of material-element mapping, time-to-audit benchmarking, field evaluation and dependency-aware closure as future work. This core is an engineering reference for testing that direction; it is not an implementation of an established operational standard.

Reference: https://doi.org/10.1007/s44163-026-02305-w

## 2. Three non-equivalent layers

A decision packet must keep these predicates separate:

1. **Existence:** a source is present, recoverable and authorized for the decision.
2. **Claim support:** a source supports the exact claim at its asserted strength and scope.
3. **Decision closure:** the collection of claim-support results covers every material element required for the conclusion.

Passing layer 2 does not imply layer 3. A set of individually supported statements can still omit the implementation evidence, authority, timing, exceptions, population or documentation required for the overall conclusion.

The upstream Claim Support core evaluates source/claim support. Decision Closure consumes its digest-bound results; it does not redo semantic entailment or trust an LLM rationale.

## 3. Decision context

A decision context binds:
- stable decision identifier and exact conclusion digest;
- domain and jurisdiction;
- decision/evaluation time;
- authorized evidence corpus identifier and exact snapshot digest;
- domain standard identifier, revision and digest;
- reviewer roles required to approve the material-element mapping;
- minimum number of distinct reviewers per required role;
- maximum permitted age of an atomic claim-support result.

The host must authenticate the corpus, domain standard, reviewer authority, legal/clinical/safety competence, freshness and revocation state. A corpus digest alone does not make a source authoritative.

## 4. Material-element mapping

A material element is a condition that must be established before the conclusion can be warranted. The mapping should include, where applicable:

- rule/policy existence and applicability;
- subject/population and jurisdiction scope;
- implementation or performance of required action;
- authority of the actor, operator or assessor;
- timing and freshness;
- exceptions, contraindications and conditional requirements;
- monitoring and review;
- documentation and record retention;
- corrective action and remediation;
- domain-specific elements.

The mapping records each element's statement, type, applicability state, prerequisite dependencies, minimum number of independent support-result references and the evidence references assessed against it.

Applicability is explicit:
- Applicable: evidence is required.
- NotApplicable: a digest-pinned local/competent authority decision records why it does not apply.
- Unresolved: never counts as covered and requires review.

A not-applicable decision is not a model inference or a missing-data shortcut. Its explicit decision ID and digest must equal the local authority reference's decision ID and digest. The host must authenticate the decision and its authority.

## 5. Multiple defensible mappings and inter-rater reliability

The mapping itself is an expert judgment. The same conclusion may have multiple defensible decompositions, for example:
- one element for documentation versus separate elements for document creation, maintenance and availability;
- one element for overall risk assessment versus distinct risk-analysis and risk-management elements;
- one combined clinical criterion versus separate indication, contraindication and population conditions.

The contract permits multiple mappings, each with an exact digest and independent approvals. A positive decision-level result requires every currently supplied, approved mapping to be closed. If one mapping is closed and another is open, the result is MappingSensitive; the system must not select the more favorable decomposition.

Each required reviewer role has a minimum distinct-reviewer quorum. The default pilot target is at least two independent reviewers for each required mapping role. This is a qualification policy, not a claim that two reviewers guarantee correctness. Measure:
- element inclusion/omission agreement;
- split/merge agreement;
- applicability and exception agreement;
- reviewer disagreement and adjudication burden;
- closure-verdict sensitivity across defensible mappings.

Record the mapping and reviewer decisions so disagreement is observable rather than hidden.

## 6. Claim-support inputs

Each element references one or more claim-support result references, each binding:
- support result identifier and digest;
- exact claim identifier, claim digest and proposal digest;
- domain and jurisdiction;
- authorized corpus digest;
- support disposition;
- evaluation time and optional expiry.

Only current DirectSupport results count. The decision owner sets a positive maximum support age; a result outside that window cannot close an element even if its source, claim and standard digests still match. PartialSupport, Contradiction, HumanReviewRequired and InsufficientEvidence never count as positive coverage. Any contradiction blocks closure. A wrong domain, jurisdiction or corpus is a packet integrity error, not a soft warning.

The host must verify that the referenced result was produced by the Claim Support contract and that all required independent reviewer roles approved it. Map dispositions exactly: Supported → DirectSupport; BlockedByContradiction → Contradiction; HumanReviewRequired → HumanReviewRequired; InsufficientEvidence → InsufficientEvidence. Never coerce a non-Supported result into DirectSupport. This core treats the support result as an external input; it does not authenticate the upstream result digest itself.

## 7. Dependencies and scope conditions

Material elements may depend on prerequisite elements. For example:
- an implementation claim may depend on an applicable rule;
- evidence of a valid inspection may depend on assessor authority;
- a conclusion about a clinical intervention may depend on indication and population scope;
- exception handling may depend on a threshold or triggering condition.

The dependency relation must be acyclic. The reference implementation is tested against exhaustive enumeration of every directed graph on four elements (4,096 edge configurations), compared with an independent transitive-closure cycle oracle. A dependent element cannot count as satisfied while any prerequisite is missing, contradicted, unresolved or excluded without authority. This extends the unordered coverage model discussed in the paper and is a conservative V1 treatment of dependency-sensitive decisions.

Exceptions and scope limitations should be represented as elements or explicit conditions, not left as free-text notes that a coverage calculation can omit.

## 8. Closure dispositions

- **Closed:** every required element is supported in every currently approved mapping, and dependencies are satisfied.
- **OpenMissingElements:** at least one required element lacks sufficient support.
- **BlockedByContradiction:** at least one material element has contradictory support.
- **BlockedByPrerequisite:** a required dependency is not satisfied.
- **MappingSensitive:** approved alternative mappings produce different closure verdicts.
- **MappingReviewRequired:** the mapping, applicability or reviewer quorum is incomplete.
- **InsufficientMapping:** no mapping is available.

Closure is binary at the conclusion level: partial coverage is not a partial pass. Element-level results expose the coverage gap and tell reviewers what is missing.

## 9. Authority and execution boundaries

- Need Intake preserves community intent, sources, assumptions, alternatives and unresolved questions.
- Claim Support evaluates atomic source/claim support.
- Decision Closure evaluates conclusion-relative material-element coverage.
- Identity authenticates people, organizations and signing principals.
- Governance owners define and authorize local decision processes.
- Work Evidence and professional/statutory bodies own evidence and assessor qualification.
- Safety owners define and review safety constraints.
- Project Formation evaluates team/work-package coverage after relevant approval.
- Resource owners control availability, custody and release.
- Finance and issuers control economic effects.

The Decision Closure core does not make a legal, clinical, financial, compliance, professional, statutory or safety determination. It computes coverage against a host-supplied, domain-authority-reviewed mapping and exact support-result references. It must never start work, issue a permit, certify compliance or authorize payment.

## 10. Versioning and reproducibility

The host computes a canonical mapping digest and binds each review to that exact digest. A material change to elements, dependencies, applicability, scope, reviewer roles, standard revision or evidence references creates a new mapping revision and invalidates prior reviews.

The reference core checks the presence and equality of host-supplied digests; it does not canonicalize or hash input bytes, validate a signature, authenticate the source corpus or prove that the mapping contains every domain requirement.
