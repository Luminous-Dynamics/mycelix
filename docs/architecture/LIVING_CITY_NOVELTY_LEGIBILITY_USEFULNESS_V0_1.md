# Living-City: Balancing Novelty, Surprise, Legibility, and Usefulness (review schema v0.1; routing policy v0.2)

**Status:** proposed evaluation protocol, not a calibrated measure or completed community review.  
**Scope:** concept selection and design exploration for Living-City, with Johannesburg/Highveld as an initial context.  
**Core rule:** a concept may be unusual without being good, familiar without being bad, and useful without being novel. Never collapse these into one creativity score.

## Research basis and limits

- Han, Forbes & Schaefer (2021), *An exploration of how creativity, functionality, and aesthetics are related in design*, reports evidence that novelty, usefulness, and surprise are distinct dimensions in design-creativity evaluation. Their experiment involved expert ratings of 45 product-design samples; this informs the rubric but does not validate a civic-specific review protocol: https://doi.org/10.1007/s00163-021-00366-9
- Guo et al. (2022), *Divergent Thinking and Evaluative Skill: A Meta-Analysis*, synthesizes 96 effect sizes from 20 studies and 3,019 participants. It finds a modest association between divergent thinking and evaluation skill (r = .13, 95% CI [.07, .20]) and reports moderation by task and context. Generating ideas and judging ideas are related but distinct capabilities: https://doi.org/10.1002/jocb.539
- *Evaluating Creativity: How Idea Context and Rater Personality Affect Considerations of Novelty and Usefulness* (2022) studies both alternative-use responses and real urban-planning suggestions; this supports specifying the idea context and evaluator perspective rather than assuming one universal novelty/usefulness weighting: https://doi.org/10.1080/10400419.2022.2125721
- Yu et al. (2025), *The Effect of Novelty and Usefulness on Creative Idea Adoption: An S-Shaped Relationships*, reports a positive relationship between usefulness and adoption and a context-dependent, S-shaped relationship for novelty. Adoption should be evaluated separately from perceived creativity: https://doi.org/10.1002/jocb.70059
- *Rater effects in creativity assessment: A mixed methods investigation* (2015) found inconsistent ratings and low dependability coefficients across three rater groups, supporting explicit attention to disagreement rather than assuming aggregate ratings are stable: https://doi.org/10.1016/j.tsc.2014.10.004
- Davison & Rowden (2026), *Unjust by design?*, analyses how urban-design governance and design standards can reproduce exclusion and displacement; it reinforces treating justice and affected-group voice as constraints rather than as creativity trade-offs: https://doi.org/10.1016/j.progress.2026.101034
- A 2026 South African study of placemaking governance reports risks of tokenistic participation and identifies co-design, co-stewardship, adaptive governance, and sustained relationships as practical mechanisms; its Bellville single case is informative, not a universal causal result: https://doi.org/10.1007/s44327-026-00274-4
- A 2025 engineering-design evaluation study found greater novice-rater reliability when ideas included sketches than when represented by text alone; that result supports testing representation effects rather than assuming a text-only score is format-neutral: https://doi.org/10.1017/pds.2025.10092
- A January 2026 study of three public spaces along Johannesburg’s Fox Street reports that temporary activities such as performances and pop-up markets helped activate space, but impacts could be short-lived without institutional support; it also reports mixed public views shaped by safety and accessibility: https://doi.org/10.1016/j.cities.2025.106436

These studies motivate separation of dimensions; they do not validate the Living-City thresholds below. The initial 1–5 anchors and routing rules are hypotheses to test and revise with local reviewers and outcome data.

## Do not use “odd” as a penalty

This contract names **surprise** as the measured dimension and treats “oddity” as informal language for perceived expectation-violation. It is descriptive, not a judgment of cultural legitimacy, quality, or safety.

- **Novelty:** how different the proposal is from an explicit, dated and inspectable comparison set. With no adequate reference set, novelty is not assessed, not automatically high.
- **Surprise/oddity:** how much the idea violates the expectations of a specified audience in a specified context. Novelty and surprise are related but not identical.
- **Legibility:** whether people can understand what the idea does, how it is expected to work, who benefits, what changes in everyday use, and how it can be questioned or reversed.
- **Usefulness:** how well it addresses the named problem for the named people; identify benefits and costs by group rather than relying only on aggregate claims.
- **Context fit and legitimacy:** whether the proposal is grounded in local conditions and authorship; this is not the same as whether it feels familiar to outside reviewers.
- **Feasibility:** engineering, costs, utilities, permissions, logistics, operations, maintainability, and credible ownership.
- **Delight/meaning:** resident-defined pleasure, identity, curiosity, or cultural meaning. Visual novelty is not a substitute for lived meaning.

The seven dimensions use ordinal 1–5 ratings with written rationales. Do not average them into a single number. Display dimension profiles, differences between reviewer groups, and dissent.

### Control presentation bias before scoring

Do not ask one concept to compete as a polished rendering against another represented only by a paragraph. Before group scoring, prepare a comparable, accessible design-card pack for every candidate: (1) the same-length plain-language problem and mechanism statement; (2) one annotated sketch or spatial/service diagram where relevant; (3) a short everyday-use scenario; (4) intended beneficiaries and plausible burdens; (5) dependencies, uncertainty, and comparator links; and (6) who will operate, maintain, pay for, and stop or reverse the intervention. Provide alt text and text equivalents so visual presentation does not exclude reviewers.

Record the representation format and fidelity in the review notes. Where feasible, test presentation effects with a balanced subset shown in text-only and text-plus-sketch form, without treating a sketch as evidence of feasibility. A change in understanding after viewing a diagram should prompt explanation or usability testing, not be mistaken for a change in a concept’s underlying novelty. This protocol is a design safeguard informed by one engineering-design study, not a universal finding about all reviewers or all civic concepts.

For public-space concepts, the design card must also show the post-pilot path: named steward/operator, plausible recurring funding, repair and maintenance responsibilities, feedback loop, and stop/transfer conditions. A successful temporary activation is not proof that the intervention can be sustained. The Johannesburg Fox Street study motivates this check, but does not by itself establish how any particular Johannesburg project would perform.

## The routing policy

Recommendations are non-compensatory. Group-level medians are one summary signal, never a substitute for the individual group profiles. The routing policy is versioned separately from the review schema: this change retains review schema v0.1 and advances routing policy to **v0.2**. A routing label is advisory only.

1. **Any hard-gate failure → stop or reframe.** A high novelty or usefulness rating cannot compensate for life-safety, accessibility, rights/consent, environmental/land feasibility, operations/maintenance, anti-displacement, or privacy failure.
2. **Contested reviews or major cross-perspective disagreement → hold for evidence.** In v0.2, if the range between perspective-level group ratings is at least three points on any decision-critical dimension—legibility, usefulness, context fit, or feasibility—the validator returns `hold_for_evidence`, even when the median is favorable. A large cross-perspective range on any scored dimension must be preserved in `dissent_summary`; it may not be smoothed away. This is a deliberately conservative governance tripwire, not a statistically validated cut-off, and it does not imply that the lower-scoring group is wrong.
3. **Any unknown hard gate → hold for evidence before routing onward.** A failed gate always takes precedence. `not_applicable` is allowed only where the rationale justifies it.
4. **High surprise plus low legibility, with no higher-priority hold condition, → prototype for comprehension testing.** Use a paper, diagram, model, simulation, or usability prototype to discover what people understand—not construction or a live public-space trial.
5. **Incomplete reference set, unresolved evidence reference, insufficient stakeholder perspectives, or low median legibility/usefulness/context fit/feasibility → hold for evidence.**
6. **Only if hard gates are cleared, the comparison set is ready, evidence references resolve, the required perspectives are represented, and the disagreement tripwire is clear can a record become eligible for human pilot review.** This is not permission to build. Relevant authorities and affected people still decide whether a reversible pilot is appropriate.
7. **Contested feedback remains contested.** Preserve reasoning and dissent; do not turn disagreement into a low average or erase minority views.

The three-point range rule is versioned so it can be changed after real calibration. It is not derived from the cited studies, it is not a claim of statistical significance, and the first deployment must inspect both the group profiles and written rationales. Do not expose sensitive small-panel scores publicly; suppress them where details could identify respondents.

For a concept affecting transport or informal trade, include operators and livelihood stakeholders. For a built intervention, include those responsible for maintenance. Store group summaries without individual names or raw individual response data; suppress public numeric group scores where small panels could make respondents identifiable.

## A practical portfolio strategy

Use three lanes when generating concepts, without assigning universal percentage weights:

| Lane | Purpose | Example for a Johannesburg civic corridor |
|---|---|---|
| Reliable improvement | Improve a known need with established mechanisms | Safe crossing, continuous step-free access, repaired drainage, dependable transit connection |
| Adjacent novelty | Recombine proven parts in a locally useful way | A shaded market/transit threshold that supports cooling, trading, seating, rainwater management, and accessible transfers |
| Radical exploration | Explore unfamiliar spatial or social possibilities without pretending feasibility is established | A modular, community-programmed civic canopy whose uses change between daily gathering, markets, shade, and emergency service |

All three lanes remain useful. Novel ideas are not automatically better; familiar improvements are not automatically boring or inferior. The point is to explore a broad space and evaluate it honestly.

## What the review should ask

Reviewers should first consider a design card containing the problem, intended beneficiaries, mechanism, maintenance owner, source/evidence status, trade-offs, test plan, and reversal/stop conditions. Then rate the separate dimensions.

- What is genuinely different from the reference set, and what is borrowed or recombined?
- What surprises each reviewer group—and is that surprise intriguing, delightful, confusing, threatening, or culturally inappropriate?
- Can people explain how it works in their own words?
- Who benefits, who may be burdened, and which groups might be excluded?
- What is the smallest reversible test that can answer the biggest uncertainty?
- What evidence or local review would change your mind?

Use embeddings and similarity search only to help discover comparisons. A similarity model cannot establish cultural meaning, real-world value, or novelty in a community context by itself.

## Mycelix / Symthaea / Sol-Atlas responsibilities

- **Symthaea:** generate alternatives across the three lanes, explain mechanism and assumptions, identify counterexamples, and suggest falsifiable questions. It must not assign cultural permission or approve a concept.
- **Mycelix:** hold versioned reviews, reference-set provenance, rights/consent state, reviewer-group summaries, dissent, and change history.
- **Sol-Atlas:** show comparison sets and separate dimension profiles; make “not assessed” visually different from a low score; expose reasons behind a route; never show a single overall creativity badge.
- **Symtropy:** later simulate only explicit and versioned mechanisms. Simulated outcomes cannot substitute for legibility testing, consent, safety review, or measured pilot results.

## Calibration and acceptance

Before treating this as stable:
1. Run the schema and semantic validator on the exact head.
2. Have diverse reviewers score varied low-risk concepts, including familiar, adjacent, and radical examples; pay affected local participants.
3. Check inter-rater disagreement and whether scores mean the same thing across groups; report within-group and between-group differences, not only a pooled median.
4. Run sensitivity checks across plausible disagreement thresholds; review false holds and missed conflicts before changing the provisional three-point tripwire.
5. Test whether routing identifies confusing concepts for explanation tests without systematically filtering out ideas that are merely unfamiliar to powerful reviewers.
6. After any real pilot, compare predicted usefulness and feasibility with observed outcomes, maintenance costs, distributional effects, and resident feedback; retain null and harmful outcomes.
7. Version changes to rating anchors or route rules; never silently compare scores collected under different versions.

The goal is not to maximize novelty or minimize oddity. It is to preserve room for transformative ideas while ensuring that surprise earns explanation, usefulness earns evidence, and no amount of creative appeal overrides dignity, safety, rights, or community agency.
