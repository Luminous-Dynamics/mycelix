# EMB-DATA-PRIVACY-001T — Synthetic Smart-Glasses Privacy/Purpose Corpus v0.1

Status: SOURCE CANDIDATE / NOT QUALIFIED / NOT PASS

Parent program: EMB-DATA-PRIVACY-001 / #2948
Embodied-data program: EMB-DATA-000 / #2923
Terms/rights composition: EMB-DATA-RIGHTS-001A / #2937
Preregistration: EMB-DATA-PRIVACY-001T / #2949

## Purpose

Freeze fully synthetic semantics for human-subject privacy/consent/purpose review before implementing any evaluator. The corpus models smart-glasses and egocentric research situations without reproducing real human data, consent forms, provider legal text, credentials, sensitive attributes or runtime code.

## Core theorem

`CompatibleWithReviewedPrivacyPurposeProfile` is a narrow evidence assessment only.

It is not consent legally established, ethics/IRB approval, jurisdiction-independent privacy compliance, legal permission, training execution authority, publication/model-release authority, identity/biometric authority, humanoid actuation authority, or proof that re-identification is impossible.

## Closed assessment vocabulary

V0.1 uses exactly:

- `CompatibleWithReviewedPrivacyPurposeProfile`
- `IncompatibleWithReviewedPrivacyPurposeProfile`
- `HumanReviewRequired`
- `ConsentEvidenceUnavailable`
- `ConsentOrPurposeProfileExpiredOrSuperseded`
- `PurposeOutOfReviewedScope`
- `EvidenceConflict`

There is intentionally no `PrivacyCompliant`, `ConsentValid`, `SafeToTrain`, `SafeToPublish` or `LegallyAllowed` state.

## Closed evidence vocabularies

The synthetic evidence inputs are also closed rather than arbitrary strings.

Participant evidence states:

- `Supported`
- `Unknown`
- `Unavailable`

Bystander/nonparticipant evidence states:

- `Supported`
- `Unknown`
- `NotApplicable`
- `HandledByReviewedTransformation`
- `NotPresentInRequestedRepresentation`

Currentness states:

- `Current`
- `Expired`
- `Superseded`

Purpose dispositions:

- `Compatible`
- `Incompatible`
- `Unknown`
- `OutOfScope`

Transform-review states:

- `ReviewedForDeclaredDerivedOutput`

Unknown/new values fail visible/closed rather than being treated as permissive defaults.

## Purpose vocabulary

Purposes are independent: PerceptionResearch, ActionForecasting, GazeAffordanceResearch, ImitationLearning, RobotPolicyTraining, BenchmarkEvaluation, HumanBehaviorResearch, PublicVisualization, RawDataSharing, DerivedRepresentationSharing, IdentityRecognition, BiometricIdentification and SensitiveAttributeInference.

The last three are primarily restriction/audit categories. Presence in the vocabulary does not create a positive default.

## Modality vocabulary

V0.1 models RgbVideo, Audio, Gaze, HeadPose, BodyPose, HandPose, ObjectPose, Depth, Imu, SpatialMap, DerivedEmbedding and DerivedPoseOnly.

## Data-minimization theorem

Source availability is not a requirement to ingest. A request must declare the modalities needed for the exact purpose. Unrequested modalities may remain excluded.

`available modality != requested modality != admitted modality`

A requested modality must be either a source modality or an exact output of the reviewed transform profile named in the case. Derived modalities cannot appear by naming convention alone.

## Participant/bystander theorem

Primary participant/wearer evidence and bystander/nonparticipant evidence are independent.

`wearer evidence != bystander evidence`

Unknown bystander evidence blocks a positive assessment when the requested representation includes the unsupported human modality. A separately reviewed transformation profile may support a narrower derived representation, but that assessment applies only to its declared output and does not certify the transformation universally.

## Transformation theorem

Redaction, de-identification, pose extraction and other transforms retain exact provenance.

`derived representation != source obligations erased`

A provider- or reviewer-described de-identification transform never becomes `reidentification_impossible=true` in this corpus.

## Split-role firewall

`EvaluationOnly` and `BenchmarkHoldout` cannot silently become policy-training data even when a generic purpose field would otherwise look compatible. Split role remains an independent restriction.

## Evidence conflict

Contradictory reviewed evidence for the same exact subject and same exact purpose is `EvidenceConflict` when distinct evidence records are current and neither supersedes the other.

Each contradictory evidence record binds its own exact subject, purpose, evidence ref, currentness and disposition. Changing one record to another subject converts the problem into ordinary multi-source composition; it no longer satisfies the conflict theorem.

## Multi-source composition

Every source remains separately attributable. A positive composite requires every applicable source to satisfy the requested purpose under its own reviewed evidence. Obligations union rather than disappear. One source cannot wash another source's deletion, review, lineage, redaction or sharing restrictions.

## Terms/privacy separation

A terms compatibility result from EMB-DATA-RIGHTS-001A cannot manufacture this assessment.

`terms-compatible != privacy-purpose-compatible`

The reverse is also true. Both are independent prerequisites where applicable.

## Authority ceiling

The fixture globally fixes false: legal compliance authority, universal consent-validity authority, training execution authority, publication authority, physical execution authority and re-identification-impossibility authority.

No positive privacy-purpose assessment can create publication or humanoid execution authority.

## Frozen 20 cases

C01 wearer evidence supported but bystander evidence unknown for raw video -> HumanReviewRequired.

C02 same synthetic source with a reviewed bystander-handling transform and gaze/hand/object derived request -> narrow compatible result; raw RGB excluded.

C03 gaze/hand/object purpose explicitly excludes unneeded raw audio and IMU.

C04 PublicVisualization outside reviewed purpose -> PurposeOutOfReviewedScope.

C05 RobotPolicyTraining outside a research-only purpose profile -> PurposeOutOfReviewedScope.

C06 missing participant consent evidence -> ConsentEvidenceUnavailable.

C07 expired profile -> ConsentOrPurposeProfileExpiredOrSuperseded.

C08 exact subject/version mismatch -> PurposeOutOfReviewedScope.

C09 two distinct current evidence refs bind the same exact subject/purpose but opposite dispositions -> EvidenceConflict.

C10 wearer evidence cannot satisfy unknown bystander audio evidence.

C11 reviewed de-identification/pose transform retains provenance and an explicit `ReidentificationImpossible` nonclaim.

C12 DerivedPoseOnly retains source privacy lineage and deletion-on-revocation obligation.

C13 BenchmarkHoldout blocks RobotPolicyTraining despite a compatible generic purpose field.

C14 caller IdentityRecognition request cannot rewrite incompatible reviewed evidence.

C15 BiometricIdentification unknown -> HumanReviewRequired, never permissive default.

C16 SensitiveAttributeInference incompatible -> IncompatibleWithReviewedPrivacyPurposeProfile.

C17 derived-representation sharing may be reviewed compatible while raw sharing remains incompatible.

C18 multi-source compatible composition unions deletion/review/lineage obligations.

C19 an external terms-compatible reference cannot compensate for unknown privacy evidence.

C20 a positive gaze-affordance assessment still requires separate publication and physical-execution admission.

## Downstream implementation rule

The future evaluator should be a small deterministic component with no provider SDK, browser, HTTP, filesystem, database, Holochain, credential, model-training, LLM or robot-control dependencies.

Positive assessment objects should be constructor-controlled. Arbitrary serialized input must not mint a positive assessment token.

## Claim ceiling

A future PASS may establish only deterministic semantics over this exact synthetic privacy/consent/purpose corpus. It does not establish any real person's consent, privacy compliance, ethics approval, de-identification effectiveness, legal permission, training/publication authority or physical robot authority.
