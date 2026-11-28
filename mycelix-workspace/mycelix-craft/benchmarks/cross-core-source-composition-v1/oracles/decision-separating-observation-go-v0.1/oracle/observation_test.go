package oracle

import (
	"os"
	"reflect"
	"testing"
)

func loadFixture(t *testing.T, name string) Problem {
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

func TestObservationSeparatesDifferentOptimalActions(t *testing.T) {
	result, err := Evaluate(loadFixture(t, "separating"))
	if err != nil {
		t.Fatal(err)
	}
	observation := result.Observations[0]
	if !observation.UniversallySeparatingWithinListedWorlds {
		t.Fatalf("expected adaptive branch separation: %+v", observation)
	}
	if observation.OutcomeCount != 2 || observation.IdentifiedWorldCount != 2 {
		t.Fatalf("unexpected branch coverage: %+v", observation)
	}
	if !reflect.DeepEqual(observation.Branches[0].Decision.OptimalInEveryWorld, []string{"hold"}) ||
		!reflect.DeepEqual(observation.Branches[1].Decision.OptimalInEveryWorld, []string{"operate"}) {
		t.Fatalf("branch-specific decisions are wrong: %+v", observation.Branches)
	}
	if observation.Disposition != "CONDITIONALLY_DECISION_SEPARATING_WITHIN_DECLARED_SET" {
		t.Fatalf("disposition: %s", observation.Disposition)
	}
	if observation.IndependenceStatus != "NOT_EVALUATED" {
		t.Fatalf("decision partition must not claim independence: %s", observation.IndependenceStatus)
	}
}

func TestObservationThatDoesNotSeparateDecisionIsNotPromoted(t *testing.T) {
	problem := loadFixture(t, "separating")
	problem.Observations[0].OutcomeByWorld["w-high"] = "same"
	problem.Observations[0].OutcomeByWorld["w-low"] = "same"
	result, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	observation := result.Observations[0]
	if observation.UniversallySeparatingWithinListedWorlds {
		t.Fatal("same-outcome observation falsely claimed to separate the decision")
	}
	if observation.OutcomeCount != 1 || observation.IdentifiedWorldCount != 0 {
		t.Fatalf("unexpected non-separating outcome: %+v", observation)
	}
}

func TestIncompleteOutcomeMappingCannotClaimSeparation(t *testing.T) {
	problem := loadFixture(t, "separating")
	delete(problem.Observations[0].OutcomeByWorld, "w-low")
	result, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	observation := result.Observations[0]
	if observation.MappingComplete || observation.UniversallySeparatingWithinListedWorlds {
		t.Fatal("incomplete mapping was promoted")
	}
	if observation.Disposition != "OBSERVATION_MODEL_INCOMPLETE" {
		t.Fatalf("disposition: %s", observation.Disposition)
	}
	if !reflect.DeepEqual(observation.MissingWorldIDs, []string{"w-low"}) {
		t.Fatalf("missing worlds: %v", observation.MissingWorldIDs)
	}
}

func TestSharedFailureDomainDoesNotBecomeIndependence(t *testing.T) {
	problem := loadFixture(t, "separating")
	problem.Observations[0].FailureDomainIDs = []string{"lab-admin", "calibration-root"}
	problem.Observations[0].ExistingFailureDomainIDs = []string{"calibration-root", "utility-feed"}
	result, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	observation := result.Observations[0]
	if !observation.UniversallySeparatingWithinListedWorlds {
		t.Fatal("shared failure domain should not erase a decision partition")
	}
	if observation.IndependenceStatus != "SHARED_FAILURE_DOMAIN" {
		t.Fatalf("independence status: %s", observation.IndependenceStatus)
	}
	if !reflect.DeepEqual(observation.SharedFailureDomainIDs, []string{"calibration-root"}) {
		t.Fatalf("shared roots: %v", observation.SharedFailureDomainIDs)
	}
}

func TestHardSafetyExcludesUnsafeActionWithinOutcomeBranch(t *testing.T) {
	result, err := Evaluate(loadFixture(t, "hard-safety"))
	if err != nil {
		t.Fatal(err)
	}
	for _, branch := range result.Observations[0].Branches {
		if contains(branch.Decision.RobustSafeActions, "release") {
			t.Fatalf("unsafe release entered robust-safe set in branch %q", branch.Outcome)
		}
	}
}

func TestWorldAndObservationPermutationsPreserveCanonicalOutput(t *testing.T) {
	problem := loadFixture(t, "separating")
	before, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	problem.Worlds[0], problem.Worlds[1] = problem.Worlds[1], problem.Worlds[0]
	problem.Actions[0], problem.Actions[1] = problem.Actions[1], problem.Actions[0]
	after, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(before, after) {
		t.Fatalf("input permutation changed output\nbefore=%+v\nafter=%+v", before, after)
	}
}

func TestStrictJSONRejectsDuplicateKeysUnknownFieldsAndTrailingData(t *testing.T) {
	raw, err := os.ReadFile("../testdata/separating.json")
	if err != nil {
		t.Fatal(err)
	}
	cases := [][]byte{
		[]byte("{\"schema_id\":\"a\",\"schema_id\":\"b\"}"),
		append(append([]byte(nil), raw...), []byte(" {}")...),
		[]byte("{\"schema_id\":\"mycelix.craft.decision-separating-observation.v0.1\",\"case_id\":\"x\",\"scope_basis\":\"declared_set\",\"actions\":[\"hold\"],\"worlds\":[],\"observations\":[],\"surprise\":true}"),
	}
	for index, input := range cases {
		if _, err := DecodeProblem(input); err == nil {
			t.Fatalf("malformed case %d was accepted", index)
		}
	}
}
