## 8. Contrast graph and comparator semantics

Every literature observation must be represented as a directed contrast from a precisely described control arm to a precisely described treatment arm. A high-level label such as “biochar alone” or “biochar + fertilizer” is insufficient unless the comparator's own components are known.

For each contrast, create an arm record with the known components and uncertainty:
- biochar present/absent, feedstock/type, dose, and process properties when reported;
- mineral nutrient program (N/P/K and dose) and manure/compost components when reported;
- crop, soil/site, management, application timing, and follow-up;
- measured endpoint, units, sample sizes, variance source, and source citation;
- explicit unknown/ambiguous fields.

Then assign a *contrast class* only when supported by the original paper:
1. **Incremental biochar under matched fertilizer:** treatment and comparator use the same nutrient plan, with biochar as the intended contrast.
2. **Biochar-plus-input package vs no-input/other baseline:** bundled intervention; cannot isolate a biochar effect or establish fertilizer substitution.
3. **Biochar vs untreated control:** estimates that experiment's total contrast, not performance versus best agronomic practice.
4. **Nutrient-matched substitution comparison:** suitable for a specific substitution question only if nutrient input, management, and other relevant conditions are made comparable.
5. **Unclassifiable:** the source does not specify sufficient comparator detail.

The BiocharDS paper describes the CSV as a simplified version containing target information and says the full workbook includes a `Reference` sheet and an `Explanation and unit` sheet. The paper also warns that treatment classification depends on the control treatment. Therefore, if the CSV lacks a reliable source-study key or arm details, the benchmark **must obtain the full workbook and construct a reviewed crosswalk** before training any model that claims study-level generalization or treatment-effect identification. A header heuristic from the audit tool is a discovery aid, not an approved grouping key.

This is a causal-identifiability gate: if the estimand cannot be recovered from the treatment/comparator pair, report `not_identifiable_from_source` rather than filling gaps with a model assumption.

## 9. Initial dataset strategy and acquisition gates