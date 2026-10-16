# CIV-BOOT-003 — Primitive-to-modern technology pathways (research pilot v0.1)

**Status:** conceptual graph and research protocol; no edge is a verified local production recipe  
**Parent:** [Mycelix CIV-BOOT-002 / Humanity Bootstrap Atlas #3807](https://github.com/Luminous-Dynamics/mycelix/issues/3807)  
**Tracking issue:** [CIV-BOOT-003 #5063](https://github.com/Luminous-Dynamics/mycelix/issues/5063)  
**Companion:** [CIV-BOOT-000 #3808](https://github.com/Luminous-Dynamics/mycelix/issues/3808)

## 1. Why use Lewis Dartnell?

Use Lewis Dartnell's *The Knowledge: How to Rebuild Our World from Scratch* (2014) as a broad orientation and bibliography seed. Its subject range—from agriculture, food and materials through power, transport, communications, advanced chemistry and scientific knowledge—is highly relevant to the Atlas:
- [Google Books bibliographic entry](https://books.google.com/books/about/The_Knowledge.html?id=_uBdngEACAAJ)
- [Open Library record and table of contents](https://openlibrary.org/books/OL27160263M/The_Knowledge)
- [The Guardian's 2014 review](https://www.theguardian.com/books/2014/apr/20/the-knowledge-review-armageddon-lewis-dartnell) discusses its limits as a practical guide and the conditions its thought experiment assumes.

**Use the book as a map of questions—not as an authoritative manufacturing manual.** Verify technical transitions against specialist sources, standards, locally relevant resource/energy data, experiments, and practitioners. Do not copy substantial prose or illustrations; capture original summaries, bibliographic references and testable claims. Respect copyright, source-access limits, and community restrictions on traditional knowledge.

## 2. The key paradigm shift

A historical sequence is useful for orientation but insufficient for planning. Build a **multi-pathway capability graph** that can answer different questions for different starting inventories:

- What can a low-tool community reproduce with local materials and skills?
- What can a salvage-rich community keep operating, repair and extend?
- What modern modules can a community adopt immediately without pretending it can manufacture them?
- Which prerequisites are genuinely hard constraints, and which historical stages can be bypassed?
- What capability remains dependent on external parts, reference standards, specialist labor, power, reagents, or infrastructure?
- Which next measurement or experiment would most efficiently distinguish between competing routes?

The graph must distinguish at least these states:

```text
can use
≠ can maintain
≠ can repair
≠ can make a replacement part
≠ can reproduce the productive equipment
≠ can renew the measurement/tooling/knowledge chain
≠ can transfer the capability to an independent node
```

## 3. Initial demonstration graph

The companion file `CIV_BOOT_TECH_PATHWAYS_V0_1.json` proposes a small graph covering materials, measurement, mechanical power, machine tools, electricity, electronics repair, computation and technical knowledge preservation. Every edge starts at E0 (conceptual) until source review and appropriate evidence are attached.

### Path A — Low-tool foundation to precision manufacturing

```text
basic resource knowledge + hand tools + craft skill
             ├── materials shaping and joining
             └── repeatable length/mass/time references
                         ↓
         material/process control + cutting tools
                         ↓
         rigid frames + slides/bearings + workholding
                         ↓
       geometric measurement + machine-tool capability
                         ↓
        repeatable parts, repairs and better tooling
                         ↓
      progressively improved productive equipment
```

This is not a claim that every community must follow that order. Measurement can develop early and support many parallel technologies; specific routes vary with ores, wood, ceramics, energy, climate, trade, inherited tools and local skills. Toolmaking and measurement form a feedback loop rather than a one-way ladder.

### Path B — Mechanical power to electrical capability

```text
manual / animal / water / wind power (site-dependent alternatives)
                         ↓
       mechanical transmission and useful work
                         ↓
    conductive material + magnetic / moving machinery
                         ↓
    insulation + bearings + measurement + protection
                         ↓
    electricity generation / conversion / safe distribution
                         ↓
      motors, controls, communications and useful services
```

Local electrical capacity is not the same as a locally reproducible electrical industry. Track wiring, insulation, switchgear, batteries, power electronics, calibration, and replacement dependencies separately. Safety review is mandatory before any real-world electrical work.

### Path C — Leapfrog to present-day computing

```text
surviving / imported computer, microcontroller or communication module
                         +
      qualified power + storage + documentation + trained users
                         ↓
     usable modern computation / communications capability
                         ↓
      maintenance, diagnostics and component-level repair
                         ↓
  some local subsystem reproduction where evidence supports it
```

A community need not reproduce silicon to benefit from existing digital capability. Conversely, possessing a computer does not mean the community can repair it, replace its battery or power supply, reproduce the storage medium, or manufacture its semiconductor devices.

The semiconductor-fabrication branch should therefore be a distinct, much deeper dependency subgraph for ultra-pure feedstocks, controlled chemical processes, vacuum, deposition/etch, lithography, precision metrology, contamination control, process equipment, energy and skilled maintenance. Keep this at capability/dependency level until authoritative, safe sources and a bounded qualification scope exist; do not imply that primitive tooling leads directly to economically viable chip fabrication.

### Path D — Preserve and expand knowledge

```text
observation + counting + shared units + records
                         ↓
       diagrams, procedures and reproducible tests
                         ↓
   experimentation + reference standards + skill transmission
                         ↓
  duplicateable archives and dependable learning pathways
                         ↓
     digital archives/build systems where sustainable
```

The knowledge system is infrastructure. Manuals alone do not preserve tacit skills; include worked examples, inspection criteria, failure reports, teaching materials and opportunities for practitioners to demonstrate competence. UNESCO emphasizes the continued transmission of craft skills and the active role of their communities:
- [UNESCO: Traditional craftsmanship](https://ich.unesco.org/en/traditional-craftsmanship-00057)
- [UNESCO: Transmission](https://ich.unesco.org/en/transmission-00078)

## 4. Starting profiles to compare

| Profile | Initial condition | Planning question |
|---|---|---|
| L — Low-tool | Manual/primitive tools, local resource knowledge, limited infrastructure | Which foundations can be demonstrated and sustained locally? |
| S — Salvage | Modern machines/devices exist; spares, power, calibration or supply are uncertain | Which existing capabilities can be preserved, repaired or extended before they degrade? |
| M — Modern leapfrog | Selected modern modules, outside technical knowledge and partial trade | Which routes skip historic intermediate technologies while making remaining dependencies explicit? |

These are scenario families—not claims that a community naturally falls into a single type. Each instance needs a site-specific inventory, skills census, energy and material constraints, ecological limits, and declared external support.

## 5. Source and evidence method

The initial source portfolio should combine:
- Dartnell as a broad secondary synthesis;
- specialist histories of machine tools and manufacturing;
- scholarship on practical mathematics, measurement and industrialization;
- engineering standards and manuals for modern technical assertions;
- local resource, energy and ecological evidence;
- practitioners' experience and community-authorized documentation of living craft knowledge;
- African and Global South technological histories, avoiding the implication that European industrialization is the universal template.

Initial anchors:
- Gragnolati / practical mathematics and precision measurement in industrialization, *The Journal of Economic History* (2022): [DOI 10.1017/S0022050722000250](https://doi.org/10.1017/S0022050722000250)
- Machine tools in historical perspective, *Journal of Economic Behavior & Organization* (1984): [DOI 10.1016/0167-2681(84)90028-3](https://doi.org/10.1016/0167-2681(84)90028-3)
- UNESCO living craft transmission: links above.

CIV-BOOT's existing evidence classes E0–E5 remain canonical. Apply evidence levels to a **specific claim under a specific profile**, not as a blanket rating for an entire technology. Keep disagreements visible. Historical occurrence is not experimental replication; simulated closure is not a physical demonstration.

## 6. Pilot acceptance tests

1. **Prerequisite integrity:** reject missing dependencies, unit mismatches, duplicate nodes and unsupported cycles.
2. **Alternative correctness:** an OR branch bypasses a historical path only when its own dependencies are satisfied.
3. **Leapfrog correctness:** salvage profiles may use extant equipment without receiving local repair or reproduction credit automatically.
4. **Renewal completeness:** report tool, wear-part, calibration, energy and knowledge dependencies needed to preserve the capability.
5. **Counterfactual behavior:** removing a necessary prerequisite cannot increase proven closure unless a declared alternative path remains.
6. **Local variation:** changing local resources, energy, ecological limits or skills changes the viable frontier without rewriting historical claims.
7. **Knowledge-loss test:** remove selected specialist skills or reference artifacts and report affected transitions and possible teaching/measurement interventions.
8. **Source disagreement test:** source conflicts remain first-class unresolved claims; the graph must not silently force consensus.
9. **Independent recomputation:** a second evaluator recomputes a small golden subgraph and rejects forged evidence/transition receipts.
10. **Coverage honesty:** report unqualified and unmapped critical dependencies; a large graph is not a complete graph.
11. **Human usability:** an independent practitioner can trace the claim from input resources through capability prerequisites to inspection evidence and a stated output.

## 7. Scope of this pilot

The JSON graph is a research seed, with conservative conceptual edges and missing-data declarations. It is not a complete Atlas, a field manual, a safe operating procedure, a cost estimate, or evidence that the pathways have been locally demonstrated.

The next implementation step is to validate the graph schema, attach claim-level source records, and select one low-risk product/capability route for an independently reviewable worked example. Do not add unsupported quantitative times, costs, yields, or probabilities merely to make the graph look complete.
