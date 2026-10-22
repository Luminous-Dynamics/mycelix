# Create and share a Praxis lesson

Praxis should let a classroom teacher, parent, student, tradesperson, researcher, librarian, community educator or practitioner turn useful knowledge into a teachable lesson—without requiring them to be a programmer or an AI expert.

This folder provides both a **form-based authoring screen** at `/author` and a **copy-edit-validate-review** workflow for contributors who prefer files. The screen autosaves to local browser storage and exports a JSON draft; it does not upload or publish content. The file workflow remains the most reproducible path for PR review.

## Start in 10 minutes

1. **Choose one capability.** Write what the learner should be able to do after the lesson. Prefer an observable outcome such as “compare two fractions with equal-sized wholes” over a broad topic such as “fractions.”
2. **Copy the starter.** Copy [examples/lesson-pack.json](examples/lesson-pack.json) to a new file, for example `authoring/drafts/fractions-compare.json`. Do not edit the source example directly.
3. **Replace the example content.** Keep the JSON structure and replace the objective ID, explanation, worked examples, questions, answers, hints, misconceptions, assessment and flashcards with your own work. Write for a real learner, not for the schema.
4. **Check it.** From the `mycelix-praxis` directory run:

   ```sh
   cargo run -p praxis-curriculum-audit -- --check-lesson authoring/drafts/fractions-compare.json --format=markdown
   ```

   The checker reports a BLAKE3 digest for the exact bytes it checked. It blocks known generic placeholders, missing lesson parts, answer/distractor collisions, incomplete distractor sets, duplicated choices and assessment point-total mismatches. The command returns non-zero when blockers remain.
5. **Review the actual teaching.** Ask someone with relevant subject knowledge to check every worked step and answer. Then ask an educator familiar with the target learners and curriculum to check alignment, age/register, accessibility, and whether the questions measure the stated objective. The automated check is a floor, not a teacher.
6. **Share a pull request.** Include the lesson JSON plus its evidence sidecar, source/rights record, and any accessibility assets. Submit drafts for review; do not add them directly to the active content manifest or claim that they are approved.

## What every complete lesson should teach

- **Target outcome:** one to three observable learner actions tied to an existing curriculum objective where possible. Do not invent an official standards code.
- **Explanation:** define the idea, explain why it works, and show when it is useful. Start with concrete objects or a worked representation when that helps.
- **Worked example:** at least one realistic problem, meaningful steps, and a checked final result. “Read the question, calculate, check” is a process reminder, not a worked example.
- **Independent practice:** at least three distinct questions that test the target skill at different levels. Include common mistakes and near-transfer, not just copies with different numbers.
- **Hints:** progressive help that gives the learner a useful next step before revealing the full answer. Full solutions are useful after an attempt or in worked examples; they should not be the only response to a request for a first hint.
- **Feedback/answer keys:** explain why the answer is right and diagnose likely wrong reasoning. Check units and accept equivalent valid answers. Wrong-answer options must genuinely be wrong; do not repeat the key in another form.
- **Assessment:** align items with the same outcome, provide scoring/rubric information when answers vary, and ensure `total_points` is exactly the sum of the item points.
- **Recall support:** include at least one useful flashcard. Make the question meaningful without depending on a picture or context the learner cannot see.
- **Accessibility:** meaningful text alternatives for visuals, keyboard-operable activities, readable structure, simple language where appropriate, and no video/audio-only route to a required concept. Preserve the same learning target while changing the mode of access.
- **Uncertainty:** mark open questions, local assumptions, and unverified claims rather than inventing citations or pretending you have checked something you have not.
- **Human review:** leave `quality.needs_human_review` set to `true`, `review_status` at `draft_requires_educator_review`, and `metrics_valid_for_current_revision` set to `false`. Editing a lesson invalidates inherited approval/evidence.

## Use sources and respect rights

You can author from personal expertise, open resources, public-domain sources, licensed materials, or original examples. Keep a source record for external factual claims and record the specific revision used. Do not paste paid textbook chapters, copyrighted worksheets, proprietary test-bank questions, private student work, or images/music you do not have permission to redistribute.

A source link is not automatically permission to copy it. Before publication, contributors must state the actual rights status and attribution in the [Lesson Evidence Record v1](../schemas/lesson-evidence-record-v1.schema.json). The starter says `UNDECIDED` intentionally: choose a license only when you have authority to apply it to all contributed material. If rights are unclear, link to the source and write original explanations/examples instead.

## Find or propose an objective

First search existing graph IDs and assets, then link the lesson to the best-fitting objective. Reuse a stable ID only when the target capability is the same. If the graph is wrong, incomplete, or not active in the app, tell reviewers; do not smuggle graph changes into a lesson-only PR. Curriculum activation and lesson quality are separately reviewed.

## How review works

A contribution can move through evidence stages:

1. **Draft:** written and editable; not yet approved.
2. **Machine-validated:** schema/structure and known-defect checks run on the exact digest.
3. **Human-reviewed:** subject accuracy, objective alignment, assessment, accessibility, safety, rights and age appropriateness reviewed for that digest.
4. **Pilot-tested:** learners tried the material under an appropriate, consented process; report what changed and what did not.
5. **Independently evaluated:** a separate evaluator checked the stated claim with a documented method. Learning-efficacy language requires learner-outcome evidence, not a clean content checker.

A digest tells reviewers which bytes were reviewed; it does not prove a lesson is true or effective. Meaningful edits create a new digest and require the relevant checks again.

## Minimal submission checklist

- [ ] The objective and intended learner level are explicit.
- [ ] Every worked solution and answer key has been checked.
- [ ] Three or more distinct practice items and progressive hints are included.
- [ ] No distractor is equivalent to the correct answer; scoring is internally consistent.
- [ ] Source, attribution, rights status and license decision are documented.
- [ ] Language, accessibility, age/safety and relevant local context have been reviewed.
- [ ] The exact lesson digest and review scope are recorded in the evidence sidecar.
- [ ] Claims are limited to the evidence actually collected; student records are not committed.
- [ ] The standalone checker passes, and the lesson remains a draft until reviewed.

## What happens next

The initial `/author` screen now provides section-by-section editing, JSON preview, basic quick checks, browser-local autosave, explicit JSON download, and **import of an existing lesson-pack JSON file for remixing**. Import is local to the browser; the file is not uploaded. The importer preserves unedited extra examples, practice items, assessment items, flashcards and unknown metadata on export, and leaves missing required fields blank instead of inserting sample text. The form currently edits the first worked example, first assessment item, first flashcard and first three practice items; remaining imported items are preserved but not yet editable in the form. Next improvements are a searchable objective picker, add/remove controls for every section, source/evidence sidecar generation, standard OER/LTI/H5P import where rights permit, and full static validation results in the screen. Publishing remains a reviewed action, never an automatic side effect of saving.
