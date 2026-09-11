# Transaction Hospitality Qualification v0.1

A transaction-field report is evidence only for the exact tuple:

- hospitality registration digest;
- candidate and baseline model lineages;
- exact target-plan digest;
- actual-projection-spec digest;
- adapter/schema/mapping identity;
- extraction-campaign digest;
- canonical replay digest;
- transition-aware local-time schedule digest; and
- exact forecast-submission set.

A report is reconstructed from source files. The caller does not supply evaluation actuals, slice outcomes, or field-quality counts.

The theorem fails closed on missing/unplanned targets, lineage drift, connector drift, future source timestamps, replay/manifest mismatch, undeclared projection input, derived-actual overflow, target windows crossing an offset transition, and invalid field evidence.

Passing remains shadow qualification only. It does not authorize business execution.
