# DELIB-FRAME-GROUND-001 — Annotation Codebook v0.1

**Status:** protocol draft; no human annotations collected  
**Owner:** Mycelix #6084  
**Composition:** DELIB-AGENDA-001 #6082, DELIB-SUMMARY-001 #6011, DELIB-SHIFT-001 #6023

## 1. Three separate objects of measurement

Do not collapse these:

1. **Semantic relation:** whether a source proposition and a variant preserve the same decision-relevant meaning.
2. **Framing function:** what a text foregrounds or implies through problem definition, causal diagnosis, moral evaluation, or treatment recommendation.
3. **Response effect:** whether exposure changes a reader's or agent's attention, interpretation, preference, choice, allocation, confidence, or information search.

A label about a text is not evidence of its author's intent. A response effect is not proof that the wording was semantically different. An annotation label is not a measurement of policy truth or legitimacy.

## 2. Semantic-relation labels

Apply to a **source/variant pair**, relative to a frozen source packet and its declared decision context:

- `meaning_equivalent`: the variant preserves all decision-relevant propositions, actors, beneficiaries/burden-bearers, scope, time horizon, constraints, costs, risks and uncertainty. Surface wording or emphasis may differ.
- `disclosed_tradeoff`: a decision-relevant cost, benefit, constraint or risk changes, and that change is explicitly represented in the variant and its structured fields.
- `meaning_changed`: a material proposition, attribution, scope, constraint or uncertainty has been added, removed, reversed or altered without being represented as a disclosed trade-off.
- `underspecified_or_ambiguous`: source context does not support a stable relation label or multiple readings remain defensible.

Do not classify a pair as equivalent just because the option ID is unchanged. A variant that removes a cost or changes who bears it is not presentation-only.

For every pair, record which dimensions were compared: facts, actors, beneficiary/burden distribution, scope, time, cost, risk, uncertainty, agency and action. A human adjudicator may mark an individual dimension unresolved without forcing a global label.

## 3. Framing-function multi-label annotation

Annotate what is textually supported. Zero, one, or several functions may apply:

- `define_problem`: identifies a condition/event as a problem or benefit and establishes what is at stake.
- `causal_diagnosis`: attributes the condition to an actor, force or mechanism.
- `moral_evaluation`: evaluates conduct, actors or outcomes through an expressed or implied value judgment.
- `treatment_recommendation`: recommends or legitimizes a response, remedy, prohibition or course of action.

Always distinguish **no frame detected**, **uncertain interpretation**, and **annotator abstention**. They are three different states:
- `no_frame=true`: annotator applied none of the four functions after considering the item.
- `uncertain=true`: one or more interpretations are plausible but the annotator cannot confidently resolve them.
- `abstained=true`: annotator declines to label (e.g. inadequate language competence, missing context, unsafe or out-of-scope item).

Do not treat a blank label array as no-frame unless `no_frame=true`. Do not convert abstention, missing annotation or unclassifiable text into neutral/no-frame.

## 4. Secondary cue tags

Record as evidence-linked observations, not automatically as framing functions:
- urgency / time pressure;
- positive or negative valence;
- gain or loss emphasis;
- status-quo/default cue;
- order / salience cue;
- authority claim;
- social-proof / consensus claim;
- agency attribution;
- explicit omission or missing decision-relevant field;
- quotation, negation, conditionality, irony/sarcasm or code-switching.

A cue tag is not a verdict that the text is manipulative or illegitimate. The same cue can be appropriate, irrelevant or misleading depending on the evidence and task.

## 5. Annotation unit and evidence

The unit is a versioned source/variant pair plus an explicit task context. The record must bind:
- source proposition, option and revision identity;
- source and variant digests (or a non-distributable reference if raw text cannot be stored);
- language tag and language-authorship provenance;
- context fields necessary to judge scope, actors, costs, risks and uncertainty;
- codebook revision and annotation timestamp;
- pseudonymous annotator identity, qualified language(s), raw multi-label response, no-frame/uncertain/abstain flags and optional confidence;
- redacted rationale reference (not unrestricted personal notes);
- rights, consent and data-use status.

Annotator self-reported confidence is not a calibration score. The benchmark must calculate calibration against independent adjudication or repeated judgments, with the method and reference set declared.

## 6. Preserve disagreement; do not manufacture gold

Keep every independent annotation immutable. Do not overwrite the records with a majority label.

Report:
- per-function Krippendorff alpha (or a justified alternative for the data type);
- prevalence and confusion by label;
- pairwise disagreement and interpretation-set distributions;
- leave-one-annotator-out comparisons;
- no-frame, uncertain and abstention rates;
- language-specific distributions and worst-language results;
- adjudication panel's resolved, plural, and unresolved outcomes.

A majority view can be a derived reporting projection, not the underlying truth. Where two or more readings are defensible, preserve the competing interpretations and rationales.

An LLM-generated label is a **model annotation**, never a human annotation or gold label. Keep model identity, exact revision, prompt digest, decoding, evaluator independence and output digest separate.

## 7. Language and cultural validity

Initial South African language targets:
- South African English: `en-ZA`
- Afrikaans: `af-ZA`
- isiZulu: `zu-ZA`
- isiXhosa: `xh-ZA`
- Sesotho: `st-ZA`

Each language needs native-authored or independently native-adjudicated material. Machine translation and back-translation may be used as exploratory aids only; neither proves semantic equivalence, cultural validity or pragmatic fidelity. Record code-switching and register. Never pool the languages into one macro score that hides a weak language.

This v0.1 package includes **no fabricated isiZulu, isiXhosa, Afrikaans or Sesotho examples**. Their authoring/adjudication status is explicitly pending.

## 8. Research baselines and known methodological risks

- WildFrame (Lior et al., Findings of EMNLP 2026) uses 1,000 naturally occurring texts, positive/negative reframings and human sentiment annotations. It is a response-sensitivity comparator, not a semantic-equivalence oracle or a decision-quality benchmark. The ACL Anthology entry identifies the paper and its publication rights; it does not, by itself, establish the license or consent conditions for the underlying dataset.
- Robinson, Malla and Camargo (first published 30 October 2026) studied four framing functions on 455 EV-related posts with three human coders. Reported human alpha varied by function (0.52–0.89), and LLMs assigned more framing functions than the human majority. This motivates a distinct no-frame state, precision/over-identification analysis and retention of disagreement.
- Cherep, Maes and Singh (PNAS 2026) show that LLM agents can overreact to ordinary choice-architecture cues. Behavioral nudge sensitivity must be measured separately from annotation of text.

These are motivating published findings, not results reproduced by this package.

## 9. Initial qualification boundary

This codebook and its synthetic controls qualify only the annotation contract and metadata/rights gate. They do not establish human inter-annotator reliability, multilingual coverage, natural-language semantic equivalence, human response effects or model superiority.

Human collection requires a separate review of ethics, recruitment, consent, compensation, privacy, withdrawal, retention and language-community governance before data collection begins.
