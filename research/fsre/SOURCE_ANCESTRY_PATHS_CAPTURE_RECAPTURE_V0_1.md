# FSRE capture–recapture source ancestry path contract

The dependence-aware capture–recapture E0 now interprets each ancestry field as a slash-separated root-to-register path, for example:

- Source A: `national-register/utility-account`
- Source B: `national-register/inspection-log`

The common `national-register` component establishes declared shared upstream ancestry and suppresses both stratum-level and pooled Lincoln–Petersen / Chapman point-estimate baselines. The exact theta-sensitivity interval remains available.

The literal `UNKNOWN` ancestry state withholds point estimates. Empty path components (such as `root//leaf`) are rejected.

Distinct declared ancestry paths do not prove statistical independence. They only avoid a declared common component; unrecorded data sharing, common selection, common capture mechanisms, or unmeasured heterogeneity may still induce dependence. LP/Chapman remain conditional baselines, never confidence intervals or authenticated estimates.

This is intentionally conservative: a shared ancestor or unknown lineage reduces what FSRE is willing to claim, rather than being converted into a compensating score.
