# Independent finite-game comparator protocol

This note defines how Mycelix's finite normal-form validator and pure-Nash solver should be checked against independent game-theory implementations.

It is a comparison protocol, not evidence that the comparisons have already been run.

## Comparator tools

- [Gambit](https://www.gambit-project.org/) / [PyGambit](https://gambitproject.readthedocs.io/): finite strategic/extensive game representation, normal-form payoff tables, pure/mixed equilibrium algorithms, and CLI tooling.
- [OpenSpiel](https://github.com/google-deepmind/open_spiel) / [documentation](https://openspiel.readthedocs.io/en/latest/): normal-form and extensive-form games, best-response and exploitability algorithms, and multiple game classes.

Pin the exact release/commit, compiler/runtime, platform and invocation for every recorded comparison. Do not use an approximate mixed-equilibrium solver as an exact pure-equilibrium oracle.

## Input equivalence contract

For each fixture, independently serialize the same game semantics:

1. Ordered players.
2. Per-player legal action/strategy sets.
3. Complete payoff function over every legal pure profile.
4. Player payoff order and utility units.
5. No chance nodes, repeated-game transitions, discounting or mixed-strategy interpretation in this v0.1 suite.
6. Exact comparison semantics for the integer-valued canonical fixtures.

For the asymmetric legal-strategy fixture, preserve different action sets by player. A converter must not silently add illegal actions merely because another player can use them.

The machine-readable source of truth is: fixtures/canonical-finite-games-v0.1.json

## Required canonical checks

| Fixture | Expected pure-strategy equilibrium result |
|---|---|
| Prisoner's Dilemma | Unique (defect, defect), payoffs (1, 1) |
| Matching Pennies | No pure-strategy equilibrium |
| Stag Hunt | (stag, stag), (4, 4); and (hare, hare), (2, 2) |
| Asymmetric legal actions | Unique (withhold, attack), payoffs (1, 2) |

Acceptance requires exact agreement on the complete equilibrium set, not only agreement on one equilibrium returned first. The no-pure-equilibrium case is a valid game result, not a solver error.

## Metamorphic and negative controls

The Mycelix harness should also enforce:

- Player permutation: permuting players and consistently permuting payoff coordinates and strategy-profile coordinates must permute the output correspondingly.
- Strategy declaration order: reordering strategy declarations must not change the equilibrium set.
- Payoff row order: reordering rows must not change results.
- Duplicate payoff row: validation must reject it.
- Missing payoff row: validation must reject the incomplete game before solving.
- Inapplicable action: validation must reject its use in a player's profile.
- Non-finite payoff: validation must reject NaN and infinities.
- Profile-space overflow / configured cap: must fail closed before unbounded enumeration.

## Result receipt

For each independent implementation, retain:

- fixture JSON SHA-256;
- exact source revision and comparator version/commit;
- input conversion digest and exported normal-form file;
- command, arguments, runtime and platform;
- raw solver output;
- normalized equilibrium set;
- Mycelix result and exact diff;
- missing/failed runs and tool limitations.

A result may be MATCH, MISMATCH, UNSUPPORTED, or NOT_RUN. NOT_RUN is never a pass.

## Claim boundary

Cross-tool agreement on these tiny known-answer games tests the implementation of the finite pure-equilibrium definition. It does not validate real human behavior, utility design, mixed/Bayesian/repeated/extensive-form equilibrium, mechanism incentives, coalition resistance, budget balance, fairness, rights, consent, legitimacy, or legal enforceability.

Broader mechanism claims belong to [Symtropy CIVDYN-MECHANISM-001 #4038](https://github.com/Luminous-Dynamics/symtropy/issues/4038) and should use matched partner populations, explicit interventions, resource-service outcomes and held-out social scenarios. CoopEval and Melting Pot are protocol comparators for that broader lane, not direct numerical score targets for this finite-game solver.