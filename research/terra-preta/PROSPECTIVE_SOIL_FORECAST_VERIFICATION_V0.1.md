# Prospective soil-twin forecast verification v0.1

**Status:** synthetic-only prequential forecast lock and scoring contract. It validates temporal order, identities, support and score calculations; it does not demonstrate real forecast skill.

## Paradigm

A digital twin must make predictions before observations exist, preserve the exact prediction/model/input state, and allow later independent observations to falsify it. Retrospective fit quality is not prospective skill.

V0.1 follows a one-way sequence:

1. Define target identity, method, unit, depth/support and future observation window.
2. Freeze candidate and baseline forecasts, model/code/config digests and training-data cutoff before the window begins.
3. Acquire a later reference observation with raw-source digest, method and support metadata.
4. Validate the temporal and measurement-support join.
5. Score every eligible frozen forecast exactly once; preserve invalid, missing, censored and unscorable denominators.
6. Compare the candidate against persistence and climatology on identical target IDs.

Changing a model after observing the target creates a new model/forecast version; it cannot rewrite the old forecast.

## Target and observation support

A target is uniquely identified by site, experimental unit, variable, unit, method, depth, spatial support and target time window. The observation must match these fields in v0.1. Any method/depth conversion requires a separately qualified bridge and is not implicit.

The fixture contains manufactured scalar targets. Real targets may be moisture, flux, drainage, nutrient export or activity, but each property has its own observation operator and uncertainty.

## Forecast identity and chronology

Every forecast carries:
- unique forecast ID and target ID;
- candidate / persistence / climatology role;
- model ID/version and model SHA-256;
- exact input/source SHA-256;
- immutable forecast receipt SHA-256;
- training cut-off and issue timestamps;
- held-out independence group and training independence groups;
- predictive mean / standard deviation and nominal prediction interval;
- an explicit applicability-domain state.

Required temporal order:
`training_cutoff <= issued_at < target_window_start <= target_window_end <= observed_at`.

The evaluation target's independence group must not occur in the training groups. A forecast outside its applicability domain is retained as an issued-but-abstained forecast and is not given a prediction score.

## Scores

Point forecast:
- MAE, RMSE and bias.

Predictive distribution:
- CRPS for a Normal predictive distribution (v0.1 only);
- nominal interval coverage;
- interval width;
- Winkler interval score.

Candidate skill is computed against persistence and climatology only where all forecasts cover the same target IDs. Skill is `1 - candidate_loss / baseline_loss`; positive values mean the candidate has lower loss. A zero baseline loss yields null skill and an explicit reason.

No R²-only leaderboard. Report domain abstentions, target omissions and the scored denominator. An unscored forecast is not a correct forecast.

## Validation and negative controls

- Post-window or at/after-window issue timestamp rejected.
- Training cutoff after issue rejected.
- Evaluation independence group in training groups rejected.
- Duplicate forecast receipt / forecast ID / target-role forecast rejected.
- Model/input/receipt digests must be valid SHA-256.
- Method, unit, depth or spatial-support mismatch rejected.
- Target observation timestamp must be at or after its declared window end.
- Out-of-domain forecasts remain in issuance denominator, but are excluded from scored skill with reason.
- Candidate and baselines must have identical target-ID coverage for paired skill.
- A model that loses to persistence or climatology is reported as negative skill, not promoted as validated.
- Missing/censored target observations are not fabricated as zero.
- One forecast can only be scored once against its immutable target record.

## Baselines

At minimum compare:
- persistence forecast: last available observed value before issuance;
- seasonal/climatology forecast: predeclared historical context baseline;
- candidate digital-twin model.

A fair baseline only sees information available at the candidate's issue time. All models are scored on identical target support and target IDs.

## Proposed qualification targets (not achieved)

- 100% of scored forecasts have immutable pre-outcome receipts and support-compatible later observations.
- Zero future-data leakage and zero post-outcome forecast edits.
- Nominal 90% interval coverage target 85–95% on independent site/season groups, with interval width and interval score reported.
- At least 10% relative improvement over the strongest matched baseline on predeclared primary targets, without calibration or worst-group regressions.
- Skill must persist on whole-site, season, soil-group and event holdouts; random row splits are not transfer evidence.
- Report negative skill, all abstentions and unscorable targets; do not hide them.
- Decision-grade forecast acceptance also requires model adequacy and human-authorized decision rules. Forecast correctness alone does not authorize physical action.

## Claim ceiling

All v0.1 targets and forecasts are synthetic. The contract does not validate a real soil model, instrument, field intervention or agronomic recommendation. No irrigation, amendment or fertilizer actuation is authorized.
