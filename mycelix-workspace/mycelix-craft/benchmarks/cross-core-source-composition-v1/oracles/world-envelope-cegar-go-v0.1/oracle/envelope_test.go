package oracle

import (
	"os"
	"reflect"
	"testing"
)

func fixture(t *testing.T, name string) Problem {
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

func TestCoarseEnvelopeReportsSpuriousDecisionFlip(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	result, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if !result.AbstractCoverageSound {
		t.Fatal("coarse cell should soundly over-approximate the concrete fixture worlds")
	}
	if result.DomainAssignmentCount != 4 || result.ConcreteWorldCount != 3 ||
		result.AbstractWorldCount != 4 || result.SpuriousWorldCount != 1 {
		t.Fatalf("unexpected counts: %+v", result)
	}
	if !reflect.DeepEqual(result.ConcreteSummary.OptimalInEveryWorld, []string{"hold"}) {
		t.Fatalf("concrete invariant optimum: %v", result.ConcreteSummary.OptimalInEveryWorld)
	}
	if len(result.AbstractSummary.OptimalInEveryWorld) != 0 {
		t.Fatalf("coarse abstraction should be non-identifying: %v", result.AbstractSummary.OptimalInEveryWorld)
	}
	witness := result.SpuriousDecisionFlip
	if witness == nil {
		t.Fatal("expected a spurious decision-flip witness")
	}
	if !reflect.DeepEqual(witness.SpuriousAssignment, map[string]string{
		"hazard_state": "high", "sensor_state": "healthy",
	}) {
		t.Fatalf("spurious assignment: %v", witness.SpuriousAssignment)
	}
	if !reflect.DeepEqual(witness.SpuriousOptimalActions, []string{"operate"}) ||
		!reflect.DeepEqual(witness.ConcreteOptimalActions, []string{"hold"}) {
		t.Fatalf("witness actions: %+v", witness)
	}
	if !reflect.DeepEqual(witness.DifferingDimensions, []string{"sensor_state"}) {
		t.Fatalf("witness dimensions: %v", witness.DifferingDimensions)
	}
	if !reflect.DeepEqual(witness.ViolatedConstraintIDs, []string{"impossible-high-with-healthy-sensor"}) {
		t.Fatalf("violated constraints: %v", witness.ViolatedConstraintIDs)
	}
}

func TestRefinementRemovesOnlySpuriousWorldAndPreservesConcreteSemantics(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	refined, receipt, err := Refine(problem, "coarse-all", "impossible-high-with-healthy-sensor")
	if err != nil {
		t.Fatal(err)
	}
	result, err := Evaluate(refined)
	if err != nil {
		t.Fatal(err)
	}
	if result.ConcreteWorldCount != 3 || result.AbstractWorldCount != 3 || result.SpuriousWorldCount != 0 {
		t.Fatalf("refinement counts: %+v", result)
	}
	if !reflect.DeepEqual(result.AbstractSummary.OptimalInEveryWorld, []string{"hold"}) {
		t.Fatalf("refined abstract optimum: %v", result.AbstractSummary.OptimalInEveryWorld)
	}
	if !receipt.ConcreteSemanticsPreserved || !receipt.DecisionAmbiguityReduced {
		t.Fatalf("refinement receipt: %+v", receipt)
	}
	if receipt.SpuriousWorldsBefore != 1 || receipt.SpuriousWorldsAfter != 0 {
		t.Fatalf("refinement spurious counts: %+v", receipt)
	}
	if receipt.AbstractWorldsBefore != 4 || receipt.AbstractWorldsAfter != 3 {
		t.Fatalf("refinement abstract counts: %+v", receipt)
	}
	if receipt.DomainWorldDigest == "" || receipt.BeforeStateDigest == receipt.AfterStateDigest {
		t.Fatalf("missing or unchanged refinement digests: %+v", receipt)
	}
	if len(refined.Cells[0].Excluded) != 1 {
		t.Fatalf("expected exactly one sound exclusion, got %d", len(refined.Cells[0].Excluded))
	}
}

func TestGenuineDecisionFlipSurvivesRefinement(t *testing.T) {
	problem := fixture(t, "genuine-decision-flip")
	before, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if len(before.ConcreteSummary.OptimalInEveryWorld) != 0 {
		t.Fatalf("fixture should be genuinely non-identifying: %v", before.ConcreteSummary.OptimalInEveryWorld)
	}
	refined, receipt, err := Refine(problem, "coarse-all", "impossible-high-with-healthy-sensor")
	if err != nil {
		t.Fatal(err)
	}
	after, err := Evaluate(refined)
	if err != nil {
		t.Fatal(err)
	}
	if len(after.AbstractSummary.OptimalInEveryWorld) != 0 {
		t.Fatalf("refinement erased a genuine decision flip: %v", after.AbstractSummary.OptimalInEveryWorld)
	}
	if receipt.DecisionAmbiguityReduced {
		t.Fatalf("genuine ambiguity was incorrectly reported as resolved: %+v", receipt)
	}
	if after.SpuriousWorldCount != 0 {
		t.Fatalf("expected all spurious worlds removed: %d", after.SpuriousWorldCount)
	}
}

func TestUnderApproximationIsRejectedWithOmittedWorldWitness(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.Cells[0].Fixed = map[string]string{"hazard_state": "low"}
	result, err := Evaluate(problem)
	if err == nil {
		t.Fatal("abstract cell that omits a concrete high-hazard world was accepted")
	}
	if result.AbstractCoverageSound {
		t.Fatal("under-approximation was marked sound")
	}
	if len(result.OmittedConcreteAssignments) != 1 {
		t.Fatalf("expected one omitted concrete assignment: %v", result.OmittedConcreteAssignments)
	}
	if !reflect.DeepEqual(result.OmittedConcreteAssignments[0], map[string]string{
		"hazard_state": "high", "sensor_state": "failed",
	}) {
		t.Fatalf("wrong omitted assignment: %v", result.OmittedConcreteAssignments[0])
	}
}

func TestUnsupportedNarrowingIsRejectedEvenWhenNotInFixedCell(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.ScopeBasis = ScopeDeclaredAssumptions
	problem.FixtureDigest = ""
	problem.Cells[0].Excluded = []map[string]string{
		{"hazard_state": "high", "sensor_state": "failed"},
	}
	if _, err := Evaluate(problem); err == nil {
		t.Fatal("cell excluded a declared concrete world without a constraint")
	}
}

func TestMissingDimensionDefaultCannotSilentlyNarrowTheEnvelope(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.Cells[0].Fixed = map[string]string{"hazard_state": "low"}
	if _, err := Evaluate(problem); err == nil {
		t.Fatal("cell silently defaulted hazard_state to low")
	}
}

func TestContradictoryConstraintsDoNotBecomeSafeEmptyWorldSet(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.ScopeBasis = ScopeDeclaredAssumptions
	problem.FixtureDigest = ""
	problem.Constraints = append(problem.Constraints,
		ForbiddenConstraint{
			ID: "exclude-all-high",
			Forbidden: map[string]string{"hazard_state": "high"},
			EvidenceRefs: []string{"fixture:constraint:high"},
		},
		ForbiddenConstraint{
			ID: "exclude-all-low",
			Forbidden: map[string]string{"hazard_state": "low"},
			EvidenceRefs: []string{"fixture:constraint:low"},
		},
	)
	if _, err := Evaluate(problem); err == nil {
		t.Fatal("empty concrete world set was treated as a safe result")
	}
}

func TestFixtureDomainDigestPinsTheWorldUniverseButNotRefinementCells(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.Worlds[0].Losses["operate"]++
	if err := Validate(problem); err == nil {
		t.Fatal("mutated world semantics retained exact fixture status")
	}

	problem = fixture(t, "spurious-decision-flip")
	problem.Cells[0].Fixed = map[string]string{"hazard_state": "low"}
	if err := Validate(problem); err != nil {
		t.Fatalf("cell state must remain refinable independently of domain/world digest: %v", err)
	}
}

func TestUnresolvedItemsForceNotEvaluatedScope(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.ScopeBasis = ScopeDeclaredAssumptions
	problem.FixtureDigest = ""
	problem.UnresolvedItems = []string{"unobserved-source-universe"}
	if err := Validate(problem); err == nil {
		t.Fatal("unresolved source-universe coverage was labelled conditional without NotEvaluated")
	}
	problem.ScopeBasis = ScopeNotEvaluated
	if err := Validate(problem); err != nil {
		t.Fatalf("NotEvaluated scope should preserve a bounded calculable envelope: %v", err)
	}
	result, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if result.ScopeStatus != "NOT_EVALUATED" {
		t.Fatalf("scope status: %s", result.ScopeStatus)
	}
}

func TestRefineRejectsUnknownOrRepeatedConstraint(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	if _, _, err := Refine(problem, "coarse-all", "invented-constraint"); err == nil {
		t.Fatal("unknown constraint was used to refine the abstraction")
	}
	refined, _, err := Refine(problem, "coarse-all", "impossible-high-with-healthy-sensor")
	if err != nil {
		t.Fatal(err)
	}
	if _, _, err := Refine(refined, "coarse-all", "impossible-high-with-healthy-sensor"); err == nil {
		t.Fatal("repeated refinement was accepted")
	}
}

func TestPermutationInvariance(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.ScopeBasis = ScopeDeclaredAssumptions
	problem.FixtureDigest = ""
	before, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	problem.Dimensions[0], problem.Dimensions[1] = problem.Dimensions[1], problem.Dimensions[0]
	problem.Worlds[0], problem.Worlds[3] = problem.Worlds[3], problem.Worlds[0]
	after, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(before, after) {
		t.Fatalf("input permutation changed result\nbefore=%+v\nafter=%+v", before, after)
	}
}

func TestDecodeRejectsDuplicateKeysUnknownFieldsAndTrailingData(t *testing.T) {
	raw, err := os.ReadFile("../testdata/spurious-decision-flip.json")
	if err != nil {
		t.Fatal(err)
	}
	cases := [][]byte{
		[]byte("{\"schema_id\":\"one\",\"schema_id\":\"two\"}"),
		append(append([]byte(nil), raw...), []byte(" {}")...),
		[]byte("{\"schema_id\":\"mycelix.craft.world-envelope-cegar.v0.1\",\"case_id\":\"bad\",\"scope_basis\":\"declared_assumptions\",\"dimensions\":[],\"constraints\":[],\"actions\":[],\"worlds\":[],\"cells\":[],\"surprise\":true}"),
	}
	for i, input := range cases {
		if _, err := DecodeProblem(input); err == nil {
			t.Fatalf("malformed input case %d was accepted", i)
		}
	}
}

func TestDeclaredAssumptionScopeRemainsConditional(t *testing.T) {
	problem := fixture(t, "spurious-decision-flip")
	problem.ScopeBasis = ScopeDeclaredAssumptions
	problem.FixtureDigest = ""
	result, err := Evaluate(problem)
	if err != nil {
		t.Fatal(err)
	}
	if result.ScopeStatus != "CONDITIONAL_ON_DECLARED_DOMAIN_AND_CONSTRAINTS" {
		t.Fatalf("scope status: %s", result.ScopeStatus)
	}
}
