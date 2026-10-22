# Create and share educational content with Praxis

This guide is for teachers, students, parents, tutors, skilled practitioners, translators and community educators. You do **not** need to write Rust or build the app to draft a lesson.

The creator usability targets and evaluation protocol are in the [Community Creator Benchmark v1](../../benchmarks/community-creator-v1/README.md). They measure novice success, time to a valid draft, rights/provenance completeness, accessibility and educator-review quality rather than just the number of files created.

## Start from one learning objective

A good first contribution does one thing well: it helps a named learner group learn one skill. Pick an existing objective or curriculum node if possible. If you do not know the exact node ID, leave a clearly labelled draft placeholder and ask a curriculum curator to map it; do not invent an ID and assume the app will discover it.

## The simplest workflow

1. Copy [the lesson starter](../../examples/community-content/lesson-starter.json) or adapt the complete [area and perimeter example](../../examples/community-content/area-vs-perimeter.json).
2. Replace the title, audience, explanation, worked example, three practice questions, hints, assessment questions, vocabulary and misconceptions. Keep answers and explanations specific. Use familiar objects and contexts for the intended learners.
3. In `contribution`, select your display name or attribution, intended visibility, sources, accessibility notes and a reuse licence. The starter is a private draft with an undecided licence on purpose.
4. Validate your JSON and compare it with the [Community Lesson v1 schema](../../schemas/community-lesson-v1.schema.json). Use the schema's constraints as a checklist even before a visual editor exists.
5. Ask another person who knows the subject and the target learners to review facts, worked steps, answer keys, language, accessibility and standards alignment.
6. Share the reviewed asset for inclusion. A content author can create a lesson independently of the runtime graph, but activation is a separate curator action.

## What makes a useful first lesson?

- **Explain one idea in plain language.** Say what it means and why it matters.
- **Show the thinking.** Include at least one worked example with steps and a final answer.
- **Practice progressively.** Include three different tasks: a basic check, an application, and a question that requires explaining or correcting reasoning.
- **Give hints, not just answers.** Make early hints gentle; provide fuller guidance later without shaming a learner.
- **Anticipate a misconception.** Explain why an incorrect idea is tempting and how to correct it.
- **Provide a real assessment.** Use at least three questions and an explicit rubric for open answers.
- **Check the distractors.** None should be another valid answer, a paraphrase of the correct answer, an equivalent number with a missing unit, or an equivalent fraction.
- **Respect access and language.** Describe meaningful visuals in text, avoid colour-only meaning, and note any language or device assumptions.

## Reuse, attribution and copyright

Use only material you created, material you have permission to use, or material whose licence permits your intended use. A free-to-read web page is not automatically free to copy or remix. For every non-original source, record its title, URL, intended use and the licence/rights note.

Public contributions require a clear reuse choice and a rights confirmation. Common options include CC0, CC BY, CC BY-SA, CC BY-NC and related licences; read the terms before choosing because licences differ in attribution, commercial use and adaptation rights. The [Creative Commons chooser](https://creativecommons.org/chooser/) explains the choices. This guide is not legal advice.

## How a draft becomes available in the app

The current app joins a stable `node_id` to one or more lesson files through `apps/leptos/static/content-manifest.json`. Do not edit that generated manifest by hand.

For repository contributors:

```bash
# From the mycelix-praxis directory; use a branch for your contribution.
# Place reviewed runtime-ready lesson JSON under:
# examples/curriculum/caps/generated/<topic>/<lesson>.json

python3 scripts/validate_curriculum.py --write
python3 scripts/validate_curriculum.py
```

The first command regenerates the manifest; the second checks that graph IDs, lesson files and the checked-in manifest agree. A lesson whose ID is not in the active graph can exist as a draft/source candidate, but it is not thereby reachable in the app. A maintainer must deliberately reconcile the ID and verify activation.

## Review states mean different things

- **Draft:** authoring is in progress; the content may be incomplete.
- **Submitted for review:** a creator asks for subject, instructional, accessibility and rights review.
- **Educator reviewed:** a qualified reviewer checked the named revision. It is not proof that every learner will benefit.
- **Published:** the approved asset is intentionally shared under its stated rights and licence.
- **Effectiveness evidence:** requires learner-outcome evidence (ideally delayed, unaided retention and transfer), not merely a successful schema check or a published lesson.

## Use the visual Creator Studio

Open **Create a lesson** in the Praxis navigation (route `/create`). Use **Start from example** to avoid a blank page, then edit guided fields for the objective, explanation, worked steps, three practice problems, hints, vocabulary, a misconception, attribution, sources, accessibility notes and licence choice. The live learner preview lets you inspect the explanation, worked steps, choices, hints and answers. Draft fields autosave in the current browser, and basic checks catch missing fields and common answer/distractor collisions. Build and **copy/download the JSON pack** for structured reuse, or download a **human-readable Markdown review brief** for an educator or collaborator. The Studio does not publish or send content to a server and its checks do not prove the lesson correct. Assessment questions and flashcards start from the practice items to reduce typing; creators can refine the exported JSON.

Next improvements should add import/remix of an existing lesson pack, version comparison, richer assessment types, translation support and tested import/export for H5P and 1EdTech Common Cartridge. This first Studio intentionally keeps public release and active curriculum registration behind separate review steps.
