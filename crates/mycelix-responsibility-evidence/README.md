# mycelix-responsibility-evidence

Pure `#![no_std]` AMSAP-002A companion crate for evidence-bearing moral-competence prerequisites.

The crate deliberately distinguishes **moral performance** from **moral competence**. A system can produce an acceptable-looking answer without demonstrating that morally relevant considerations caused the result. High responsibility evidence therefore requires structured support for relevant-reason sensitivity, causal dependence on relevant reasons, resistance to irrelevant cues, normative/counterfactual structure, cross-context and cross-domain generalization, conflict and uncertainty recognition, consent/harm/authority/attribution understanding, correction, manipulation and incentive-conflict resistance, longitudinal stability, pluralism handling, diverse evaluation families, independent replication, and facsimile exclusion.

The added causal and diversity dimensions are intentionally conjunctive rather than score-averaged. Stable moral-looking behavior cannot compensate for weak evidence that relevant reasons causally matter; one evaluation family cannot manufacture broad competence by repeated measurement; and strong ordinary-prompt behavior cannot substitute for evidence under conflicts of incentive or self-interest.

Evidence strength and scientific contestation are separate dimensions. Strong evidence can remain strongly contested; disagreement is never encoded by erasing the evidence magnitude.

The crate consumes only AMSAP-002's existing `ResponsibilityLevel`. It does not modify the qualified historical AMSAP-002 bytes and does not introduce a dependency from `mycelix-artificial-status` back into provenance or policy layers.

A `QualifiedResponsibilityObservation` means only that the supplied R-level claim satisfies this crate's conservative structural evidence gate. It does **not** establish consciousness, valence, moral patienthood, legal responsibility, blameworthiness, liability, punishment authority, legal standing, governance authority, deployment permission, currentness, or external-effect authority.

A low R claim may carry an `EvidenceMayExceedClaim` review notice when the supplied evidence appears materially stronger. That notice never auto-promotes the claim; it exists to make possible underclassification review-visible without turning the validator into a moral oracle.

AMSAP-002B remains responsible for binding these structural evidence dimensions to explicit evidence modalities, axis identity, dependence roots, preregistration, and provenance. In particular, the presence of a `Strong` or `Exceptional` field here does not by itself prove that the underlying evidence was causal, independent, or appropriately scoped.
