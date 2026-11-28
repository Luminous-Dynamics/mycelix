package oracle

import (
	"os"
	"reflect"
	"testing"
)

func readFixture(t *testing.T, name string) Problem {
	t.Helper()
	raw, err := os.ReadFile("../testdata/" + name + ".json")
	if err != nil {
		t.Fatal(err)
	}
	problem, err := DecodeProblem(raw)
	if err != nil {
		t.Fatal(err)
	}
	return problem
}

func TestInvariantAction(t *testing.T) {
	got, err := Evaluate(readFixture(t, "invariant"))
	if err != nil {
		t.Fatal(err)
	}
	if got.Disposition != RobustWithinExhaustiveFixture {
		t.Fatalf("disposition: %s", got.Disposition)
	}
	if !reflect.DeepEqual(got.OptimalInEveryListedWorld, []string{"repair"}) {
		t.Fatalf("identified: %v", got.OptimalInEveryListedWorld)
	}
	if !reflect.DeepEqual(got.MinimaxRegretActions, []string{"repair"}) {
		t.Fatalf("minimax regret: %v", got.MinimaxRegretActions)
	}
}

func TestNoInvariantOptimumUsesMinimaxRegretWithoutClaimingIdentification(t *testing.T) {
	got, err := Evaluate(readFixture(t, "ambiguous"))
	if err != nil {
		t.Fatal(err)
	}
	if got.Disposition != NotIdentifiedExhaustive {
		t.Fatalf("disposition: %s", got.Disposition)
	}
	if len(got.OptimalInEveryListedWorld) != 0 {
		t.Fatalf("unexpected invariant action: %v", got.OptimalInEveryListedWorld)
	}
	if !reflect.DeepEqual(got.MinimaxRegretActions, []string{"sample"}) {
		t.Fatalf("minimax regret: %v", got.MinimaxRegretActions)
	}
	if !reflect.DeepEqual(got.MinimaxLossActions, []string{"sample"}) {
		t.Fatalf("minimax loss: %v", got.MinimaxLossActions)
	}
	for _, action := range got.ActionBounds {
		if action.ActionID == "sample" &&
			(action.WorstCaseRegret == nil || *action.WorstCaseRegret != 3) {
			t.Fatalf("sample regret: %+v", action)
		}
	}
}

func TestHardSafetyVetoIsNonCompensatory(t *testing.T) {
	got, err := Evaluate(readFixture(t, "hard-safety"))
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(got.RobustSafeActions, []string{"hold"}) {
		t.Fatalf("robust-safe actions: %v", got.RobustSafeActions)
	}
	if !reflect.DeepEqual(got.MinimaxRegretActions, []string{"hold"}) {
		t.Fatalf("minimax regret: %v", got.MinimaxRegretActions)
	}
	for _, action := range got.ActionBounds {
		if action.ActionID == "release" &&
			(action.RobustSafe || action.WorstCaseRegret != nil) {
			t.Fatalf("unsafe release was scored as robust-safe: %+v", action)
		}
	}
}

func TestUnqualifiedWorldSetCannotProduceGlobalClaim(t *testing.T) {
	got, err := Evaluate(readFixture(t, "unqualified"))
	if err != nil {
		t.Fatal(err)
	}
	if got.Disposition != WorldSetCoverageNotEvaluated {
		t.Fatalf("disposition: %s", got.Disposition)
	}
	if !reflect.DeepEqual(got.OptimalInEveryListedWorld, []string{"repair"}) {
		t.Fatalf("within-set result: %v", got.OptimalInEveryListedWorld)
	}
}

func TestDeclaredSetIsConditional(t *testing.T) {
	problem := readFixture(t, "invariant")
	problem.CoverageBasis = CoverageDeclaredSet
	problem.FixtureDigest = ""
	got, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if got.Disposition != ConditionalOnDeclaredSet {
		t.Fatalf("disposition: %s", got.Disposition)
	}
}

func TestPermutationInvariance(t *testing.T) {
	problem := readFixture(t, "ambiguous")
	before, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	problem.Actions[0], problem.Actions[2] = problem.Actions[2], problem.Actions[0]
	problem.Worlds[0], problem.Worlds[1] = problem.Worlds[1], problem.Worlds[0]
	after, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(before, after) {
		t.Fatalf("permutation changed output\nbefore=%+v\nafter=%+v", before, after)
	}
}

func TestAddingWorldCannotExpandRobustSets(t *testing.T) {
	problem := readFixture(t, "invariant")
	before, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	problem.Worlds = append(problem.Worlds, World{
		ID:     "w3",
		Losses: map[string]int64{"repair": 4, "hold": 0},
	})
	after, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if !isSubset(after.RobustSafeActions, before.RobustSafeActions) {
		t.Fatalf("robust-safe set expanded: before=%v after=%v",
			before.RobustSafeActions, after.RobustSafeActions)
	}
	if !isSubset(after.OptimalInEveryListedWorld, before.OptimalInEveryListedWorld) {
		t.Fatalf("identified set expanded: before=%v after=%v",
			before.OptimalInEveryListedWorld, after.OptimalInEveryListedWorld)
	}
}

func TestStrictJSONRejectsDuplicateKeysUnknownFieldsAndTrailingData(t *testing.T) {
	good, err := os.ReadFile("../testdata/invariant.json")
	if err != nil {
		t.Fatal(err)
	}
	cases := [][]byte{
		[]byte(`{"schema_id":"one","schema_id":"two"}`),
		append(append([]byte(nil), good...), []byte(` {}`)...),
		[]byte(`{"schema_id":"mycelix.craft.decision-identifiability.v0.1","case_id":"x","coverage_basis":"fixture_exhaustive","actions":["hold"],"fallback_action":"hold","worlds":[{"id":"w","losses":{"hold":0},"surprise":true}]}`),
	}
	for i, raw := range cases {
		if _, err := DecodeProblem(raw); err == nil {
			t.Fatalf("case %d accepted", i)
		}
	}
}

func TestValidateRejectsMalformedProblems(t *testing.T) {
	base := readFixture(t, "invariant")
	mutations := []func(*Problem){
		func(problem *Problem) { problem.Actions = append(problem.Actions, "repair") },
		func(problem *Problem) {
			problem.Worlds[0].Losses = map[string]int64{"hold": 1}
		},
		func(problem *Problem) { problem.Worlds[0].Losses["repair"] = -1 },
		func(problem *Problem) { problem.Worlds[0].HardViolations = []string{"ghost"} },
		func(problem *Problem) { problem.Worlds = nil },
		func(problem *Problem) { problem.Worlds[0].HardViolations = []string{"hold"} },
		func(problem *Problem) { problem.Worlds[0].ID = problem.Worlds[1].ID },
		func(problem *Problem) { problem.FallbackAction = "ghost" },
	}
	for i, mutate := range mutations {
		problem := cloneProblem(base)
		mutate(&problem)
		if err := Validate(problem); err == nil {
			t.Fatalf("mutation %d accepted", i)
		}
	}
}

// independentSmallReference directly enumerates each action's world-by-world
// safety, optimum membership, and worst-case regret.
func independentSmallReference(problem Problem) (robust, identified, minimaxRegret []string) {
	isSafe := func(actionID string) bool {
		for _, world := range problem.Worlds {
			if contains(world.HardViolations, actionID) {
				return false
			}
		}
		return true
	}
	for _, actionID := range problem.Actions {
		if isSafe(actionID) {
			robust = append(robust, actionID)
		}
	}
	for _, actionID := range robust {
		optimalEverywhere := true
		for _, world := range problem.Worlds {
			best := int64(1 << 62)
			for _, candidate := range problem.Actions {
				if !contains(world.HardViolations, candidate) &&
					world.Losses[candidate] < best {
					best = world.Losses[candidate]
				}
			}
			if world.Losses[actionID] != best {
				optimalEverywhere = false
			}
		}
		if optimalEverywhere {
			identified = append(identified, actionID)
		}
	}

	regretByAction := make(map[string]int64)
	bestRegret := int64(1 << 62)
	for _, actionID := range robust {
		var worstRegret int64
		for _, world := range problem.Worlds {
			best := int64(1 << 62)
			for _, candidate := range problem.Actions {
				if !contains(world.HardViolations, candidate) &&
					world.Losses[candidate] < best {
					best = world.Losses[candidate]
				}
			}
			regret := world.Losses[actionID] - best
			if regret > worstRegret {
				worstRegret = regret
			}
		}
		regretByAction[actionID] = worstRegret
		if worstRegret < bestRegret {
			bestRegret = worstRegret
		}
	}
	for _, actionID := range robust {
		if regretByAction[actionID] == bestRegret {
			minimaxRegret = append(minimaxRegret, actionID)
		}
	}
	sortStrings(robust)
	sortStrings(identified)
	sortStrings(minimaxRegret)
	if robust == nil {
		robust = []string{}
	}
	if identified == nil {
		identified = []string{}
	}
	if minimaxRegret == nil {
		minimaxRegret = []string{}
	}
	return robust, identified, minimaxRegret
}


func TestDecisionFlipWitnessNamesSmallestDifferingDimension(t *testing.T) {
	got, err := Evaluate(readFixture(t, "ambiguous"))
	if err != nil {
		t.Fatal(err)
	}
	if got.DecisionFlipWitness == nil {
		t.Fatal("expected a decision-flip witness")
	}
	witness := got.DecisionFlipWitness
	if witness.WorldA != "high-risk" || witness.WorldB != "low-risk" {
		t.Fatalf("unexpected canonical witness pair: %+v", witness)
	}
	if !reflect.DeepEqual(witness.OptimalActionsA, []string{"sample"}) ||
		!reflect.DeepEqual(witness.OptimalActionsB, []string{"repair"}) {
		t.Fatalf("unexpected competing optima: %+v", witness)
	}
	if !reflect.DeepEqual(witness.DifferingDimensions, []string{"risk_state"}) {
		t.Fatalf("unexpected distinguishing dimensions: %v", witness.DifferingDimensions)
	}
}

func TestPinnedExhaustiveFixtureCannotBeRelabelledOrMutated(t *testing.T) {
	problem := readFixture(t, "invariant")
	problem.Worlds[0].Losses["repair"] = 99
	if err := Validate(problem); err == nil {
		t.Fatal("mutated fixture retained exhaustive status")
	}

	problem = readFixture(t, "invariant")
	problem.CaseID = "invented-exhaustive-case"
	if err := Validate(problem); err == nil {
		t.Fatal("unregistered case claimed exhaustive status")
	}

	problem = readFixture(t, "invariant")
	problem.FixtureDigest = "0000000000000000000000000000000000000000000000000000000000000000"
	if err := Validate(problem); err == nil {
		t.Fatal("wrong fixture digest accepted")
	}
}

func TestEvaluatorMatchesIndependentExhaustiveReference(t *testing.T) {
	for actionLoss0 := int64(0); actionLoss0 < 3; actionLoss0++ {
		for actionLoss1 := int64(0); actionLoss1 < 3; actionLoss1++ {
			for holdLoss0 := int64(0); holdLoss0 < 3; holdLoss0++ {
				for holdLoss1 := int64(0); holdLoss1 < 3; holdLoss1++ {
					for unsafe0 := 0; unsafe0 < 2; unsafe0++ {
						for unsafe1 := 0; unsafe1 < 2; unsafe1++ {
							problem := Problem{
								SchemaID:       SchemaID,
								CaseID:         "exhaustive",
								CoverageBasis:  CoverageDeclaredSet,
								Actions:        []string{"act", "hold"},
								FallbackAction: "hold",
								Worlds: []World{
									{
										ID:             "w0",
										Losses:         map[string]int64{"act": actionLoss0, "hold": holdLoss0},
										HardViolations: boolSlice(unsafe0, "act"),
									},
									{
										ID:             "w1",
										Losses:         map[string]int64{"act": actionLoss1, "hold": holdLoss1},
										HardViolations: boolSlice(unsafe1, "act"),
									},
								},
							}
							got, err := Evaluate(problem)
							if err != nil {
								t.Fatal(err)
							}
							wantSafe, wantIdentified, wantRegret := independentSmallReference(problem)
							if !reflect.DeepEqual(got.RobustSafeActions, wantSafe) ||
								!reflect.DeepEqual(got.OptimalInEveryListedWorld, wantIdentified) ||
								!reflect.DeepEqual(got.MinimaxRegretActions, wantRegret) {
								t.Fatalf("mismatch: got safe=%v identified=%v regret=%v; want safe=%v identified=%v regret=%v",
									got.RobustSafeActions, got.OptimalInEveryListedWorld, got.MinimaxRegretActions,
									wantSafe, wantIdentified, wantRegret)
							}
						}
					}
				}
			}
		}
	}
}

func boolSlice(enabled int, value string) []string {
	if enabled == 1 {
		return []string{value}
	}
	return nil
}

func cloneProblem(problem Problem) Problem {
	cloned := problem
	cloned.Actions = append([]string(nil), problem.Actions...)
	cloned.Worlds = append([]World(nil), problem.Worlds...)
	for i := range cloned.Worlds {
		losses := make(map[string]int64, len(problem.Worlds[i].Losses))
		for actionID, loss := range problem.Worlds[i].Losses {
			losses[actionID] = loss
		}
		cloned.Worlds[i].Losses = losses
		cloned.Worlds[i].HardViolations =
			append([]string(nil), problem.Worlds[i].HardViolations...)
	}
	return cloned
}

func isSubset(candidate, reference []string) bool {
	for _, value := range candidate {
		if !contains(reference, value) {
			return false
		}
	}
	return true
}

func sortStrings(values []string) {
	for i := 0; i < len(values); i++ {
		for j := i + 1; j < len(values); j++ {
			if values[j] < values[i] {
				values[i], values[j] = values[j], values[i]
			}
		}
	}
}
