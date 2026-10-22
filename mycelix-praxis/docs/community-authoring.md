# Create and share a Praxis lesson

Praxis should let people share useful knowledge without requiring them to be programmers or professional instructional designers. This guide describes the current contribution path and what is still required before a lesson can be released.

## Create a first draft

1. Open Praxis and choose **Create lesson** in the navigation, or visit \`/author\`.
2. Choose **Explain a concept**, **Build a hands-on activity**, or **Start from blank**. You can also import a Praxis lesson-pack JSON file to make a remix.
3. Describe what the learner should be able to do. Use everyday language first; choose an existing curriculum objective only when you can verify the match. If no matching objective is available, use a clearly provisional community ID—never invent an official standards code.
4. Explain the idea and show a worked example. Then add three practice questions. Each practice item needs an answer, a useful explanation, a hint, and three genuinely incorrect answer choices.
5. Add an assessment question and expected answer, one flashcard, and (where useful) vocabulary and common misconceptions.
6. Record author credit or a pseudonym, your intended licence/rights status, and sources. A public URL by itself does not mean the source can legally be reused.
7. Read **Preview as a learner** to check whether the explanation, worked example, practice questions and assessment make sense in sequence. Practice answers stay behind expandable author-only details.
8. Watch the live quick-check count and select **Check my draft** for the detailed list. Correct the issues you can; ask a subject expert for any content claim you cannot verify.
9. Use the licence suggestions only if you are authorized to grant that licence. “All rights reserved” blocks the lesson from being treated as a publicly redistributable resource; use the draft privately or obtain permission before sharing.
10. Download the JSON and keep it as a local draft. Export is **not** publication: it does not activate a curriculum node, issue a credential, or establish that learners have mastered the topic.

Draft content is saved in this browser. The authoring page does not upload the draft as a side effect. Download a copy before switching templates if you need to preserve your current work. Imported content is treated as a remixable draft and still needs review. The learner preview focuses on fields edited in the form; unedited imported items are preserved in the export even when they are not all shown in that preview.

## What makes a good community lesson?

A strong small lesson does not need polished graphics. It should make five things clear:

- **Purpose:** what the learner should be able to explain, make, perform, or decide.
- **Teaching:** an accurate explanation or demonstration that goes beyond stating the answer.
- **Practice:** a realistic task, a hint that helps without doing all the work, and feedback that explains why.
- **Evidence:** a short way to see whether the learner can perform the target skill, preferably without assistance.
- **Rights and context:** who contributed it, which sources were used, what can be reused, which language and learner level it targets, and what still needs checking.

Choose the content style that suits the knowledge: a worked example, a local-language explanation, a practical investigation, a repair procedure, a worked calculation, a conversation exercise, an argument with evidence, or a real-world project. Templates are starting points; they are not verified answers.

## Review and publication

The JSON file should be submitted with the appropriate lesson evidence sidecar and validation result. A reviewer should check subject accuracy, curriculum alignment, assessment keys, accessibility, age suitability, privacy and source/licence compatibility. Review applies to the exact content digest that was examined; edits after review must trigger revalidation.

Do not upload copyrighted textbook pages, private student work, personally identifying learner histories, or materials you cannot redistribute. Do not claim that a lesson is accredited, independently evaluated or proven effective unless the corresponding evidence exists.

Praxis is working toward an evidence-bearing contribution workflow: draft → local checks → review request → human review → optional pilot → approved reusable lesson. The current /author page implements the local-first authoring/export portion, not a public one-click publishing system.

## Usability goals

The [CreatorBench v1 proposal](../benchmarks/creatorbench-v1/README.md) defines proposed tests for novice task completion, time to export, first-pass validation, accessible authoring, safe remixing and rights comprehension. Proposed targets are not yet measured results. Praxis should iterate from observed contributor success and reviewer workload rather than assuming the current form is already easy enough.
