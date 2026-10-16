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



### Path E — A computer that can make more computers

This is an explicit target, not a claim that the computer can manufacture its own semiconductor dies. Compose the existing **[Symthaea COMPUTE-MFG-001 #5796](https://github.com/Luminous-Dynamics/symthaea/issues/5796)** and **[COMPUTE-MFG-001A #5798](https://github.com/Luminous-Dynamics/symthaea/issues/5798)** hardware-closure work. The Atlas supplies a source-backed pathway view; it must not fork the canonical computer/BOM closure semantics.

#### The practical, achievable meaning

A first useful milestone is a repairable low-complexity computer or controller plus a small manufacturing cell. The host computer uses a documented toolchain to create selected mechanical parts, fixtures, cable/harness assemblies, board-layout outputs, firmware images, programming jobs, and test reports. The cell assembles another computer from that work and qualified inventory. The copy boots and passes a test suite, then runs the same documented workflow to build the next copy.

A computer that does this **can produce more computers under a bounded profile**, even though its CPU, memory, some ICs, PCB substrate, solder, connectors, tooling, or measurement instruments may still be imported. This is meaningful partial closure, not semiconductor independence.

#### Replication ladder

| Level | Demonstrated capability | What it does not prove |
|---|---|---|
| C0 — Assemble | Build a declared computer from a complete, revision-pinned BOM and boot/test it | Local manufacture of any part |
| C1 — Repair | Diagnose and replace declared field-replaceable components with inspection evidence | Ability to replenish replacement ICs |
| C2 — Fabricate support | Make qualified enclosure parts, brackets, ducts, fixtures, and selected harnesses with a documented process | PCB/IC production |
| C3 — Board production | Assemble/rework a board from a purchased bare PCB and stocked ICs/passives, then electrically test it | Local bare-PCB fabrication or semiconductor closure |
| C4 — Computer-assisted clone | Host controls the qualified cell, programs and tests a second computer; the second computer independently boots its required software | A self-sustaining toolchain or chips made locally |
| C5 — Second-generation reproduction | Remove the original host from the active workflow. The first copy runs the archived toolchain and cell workflow to build and qualify another copy | Independence from imported components and manufacturing equipment |
| C6 — Dependency reduction | Demonstrate specific new local routes for a previously imported class, such as a bounded PCB, connector, enclosure, or test fixture | Any other closure dimension not tested |
| C7 — Semiconductor localization | Separately qualified semiconductor process chain with materials, process equipment, metrology, packaging/test and maintenance closure | Not an early milestone; a different industrial program |

C4/C5 must be demonstrated on real assemblies in the relevant test environment before they may be labeled physically demonstrated. Graph closure or simulation alone is insufficient.

#### The “vitamin” ledger

RepRap's project documentation explicitly discusses non-printed components as “vitamins” and different replication factors; this is a useful engineering analogy for dependencies that the machine cannot yet reproduce ([RepRap Development Pathway](https://reprap.org/wiki/Development_Pathway)). For computers, the ledger should include at least:

- compute IC/module; memory; storage; clock and reset components;
- PCB substrate and finishes; passives; connectors; cable and wire;
- power-conversion modules; cooling; fasteners and enclosure materials;
- solder/flux and assembly consumables;
- programming/debug interfaces, test loads, probes, reference instruments and calibration;
- firmware, boot artifacts, compiler/build system, CAD/CAM, drivers, licenses and offline documentation;
- manufacturing-tool wear parts, maintenance spares, and the equipment needed to renew those tools.

For every BOM line, report its exact identity/revision, quantity, source class, available stock, known substitutes, qualification state, lead-time evidence, and whether the **production toolchain itself** depends on it.

Do not collapse these into one “self-replication percentage.” Report distinct views for item count, cost (where sourced), material mass (where meaningful), criticality, unique external supply classes, stock runway under declared usage, and capability impact if the class disappears. Component count or mass alone can produce misleadingly favorable results.

#### The dependency boundary

Use purchased or salvaged ICs initially. Modern semiconductor fabrication depends on deeply specialized equipment, materials, process controls, metrology and multiple supplier tiers. NIST identifies high-purity feedstock measurement and provenance as continuing semiconductor manufacturing challenges ([NIST semiconductor metrology](https://www.nist.gov/chips/research-development-programs/metrology-program); [NIST supplier/facility overview](https://www.nist.gov/chips/vision-success-facilities-semiconductor-materials-and-manufacturing-equipment)). The Semiconductor Industry Association likewise documents multi-tier, geographically dispersed equipment/material dependencies ([SIA supply-chain overview](https://www.semiconductors.org/understanding-dependencies-in-the-u-s-semiconductor-supply-chain/)).

Therefore keep the existing semiconductor placeholder blocked until its critical process dependencies are modeled and evidence-qualified. The first goal is not to build an advanced CPU fab in a garage. It is to build a small computer-production cell that can make a useful replacement computer from an auditable stock of imported components, then steadily reduce the most consequential dependencies.

#### Acceptance benchmarks

1. **BOM closure:** every line in the declared target BOM has identity, revision, source class, quantity and status; no omitted “miscellaneous” critical items.
2. **First boot:** an independently assembled target boots a version-pinned diagnostic image and passes the profile's declared CPU/memory/storage/I/O tests.
3. **Build reproducibility:** source, compiler/toolchain, firmware and generated artifacts are versioned; independent rebuilds match the declared reproducibility properties.
4. **Manufacturing repeatability:** at least 3 consecutive copies pass the same independent functional and electrical acceptance tests; report all failed builds, rework, scrap and operator intervention.
5. **C4 host-assisted clone:** the original host manufactures or assembles the declared localizable parts, programs the clone and produces a complete test receipt.
6. **C5 host-removal test:** power down or remove the first host. The clone must independently run the preserved manufacturing workflow and build/test the next generation. Human steps and external services remain recorded, not hidden.
7. **Import-removal challenges:** remove one declared critical imported class at a time (for example CPU/module, memory, power conversion, bare PCB or programming interface) and report whether production continues, degrades, or becomes blocked. Do not fabricate substitutes or assume unknown compatibility.
8. **Toolchain-loss challenge:** remove internet/network access and verify that essential source archives, build tools, firmware, design outputs, licenses, programming interfaces and recovery instructions remain available as declared.
9. **Metrology challenge:** make the required test instrument/reference unavailable or stale; output must not pass if the profile requires that evidence.
10. **Economic/energy accounting:** measure material, consumed stock, energy, elapsed time, skilled human time, rework and equipment amortization for the defined runs. Publish raw values and scope; do not extrapolate beyond the tested profile.
11. **Independent replication:** a separate operator or node reproduces the declared C4/C5 outcome using the transfer packet and qualified inventory, with undocumented intervention explicitly counted as a failure or unresolved dependency.
12. **No hidden closure:** the report explicitly separates local mechanical fabrication, local board assembly, local bare-PCB manufacture, local IC packaging/test, and local semiconductor-die fabrication.

Initial target: **C4, followed by C5**, for one modest, repairable controller or computer profile. That is an ambitious but testable definition of “a computer that can make more computers.” C7 should remain a distinct long-range program.

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
