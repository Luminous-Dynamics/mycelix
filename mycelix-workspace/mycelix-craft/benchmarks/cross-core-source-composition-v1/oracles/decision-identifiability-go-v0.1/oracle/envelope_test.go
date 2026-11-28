package oracle

import (
	"os"
	"reflect"
	"testing"
)

func readFixture(t *testing.T, name string) Problem {
	t.Helper()
	raw, err := os.ReadFile("../testdata/" + name + ".json")
	if err != nil { t.Fatal(err) }
	p, err := DecodeProblem(raw)
	if err != nil { t.Fatal(err) }
	return p
}

func TestInvariantAction(t *testing.T) {
	got, err := Evaluate(readFixture(t, "invariant"))
	if err != nil { t.Fatal(err) }
	if got.Disposition != RobustWithinExhaustiveFixture { t.Fatalf("disposition: %s", got.Disposition) }
	if !reflect.DeepEqual(got.OptimalInEveryWorld, []string{"repair"}) { t.Fatalf("identified: %v", got.OptimalInEveryWorld) }
	if !reflect.DeepEqual(got.MinimaxRegretActions, []string{"repair"}) { t.Fatalf("minimax regret: %v", got.MinimaxRegretActions) }
}

func TestNoInvariantOptimumUsesMinimaxRegretWithoutClaimingIdentification(t *testing.T) {
	got, err := Evaluate(readFixture(t, "ambiguous"))
	if err != nil { t.Fatal(err) }
	if got.Disposition != NotIdentifiedExhaustive { t.Fatalf("disposition: %s", got.Disposition) }
	if len(got.OptimalInEveryWorld) != 0 { t.Fatalf("unexpected invariant action: %v", got.OptimalInEveryWorld) }
	if !reflect.DeepEqual(got.MinimaxRegretActions, []string{"sample"}) { t.Fatalf("minimax regret: %v", got.MinimaxRegretActions) }
	if !reflect.DeepEqual(got.MinimaxLossActions, []string{"sample"}) { t.Fatalf("minimax loss: %v", got.MinimaxLossActions) }
	for _, a := range got.ActionBounds {
		if a.ActionID == "sample" && (a.WorstCaseRegret == nil || *a.WorstCaseRegret != 3) { t.Fatalf("sample regret: %+v", a) }
	}
}

func TestHardSafetyVetoIsNonCompensatory(t *testing.T) {
	got, err := Evaluate(readFixture(t, "hard-safety"))
	if err != nil { t.Fatal(err) }
	if !reflect.DeepEqual(got.RobustSafeActions, []string{"hold"}) { t.Fatalf("robust-safe actions: %v", got.RobustSafeActions) }
	if !reflect.DeepEqual(got.MinimaxRegretActions, []string{"hold"}) { t.Fatalf("minimax regret: %v", got.MinimaxRegretActions) }
	for _, a := range got.ActionBounds {
		if a.ActionID == "release" && (a.RobustSafe || a.WorstCaseRegret != nil) { t.Fatalf("unsafe release was scored as robust-safe: %+v", a) }
	}
}

func TestUnqualifiedWorldSetCannotProduceGlobalClaim(t *testing.T) {
	got, err := Evaluate(readFixture(t, "unqualified"))
	if err != nil { t.Fatal(err) }
	if got.Disposition != WorldSetCoverageNotEvaluated { t.Fatalf("disposition: %s", got.Disposition) }
	if !reflect.DeepEqual(got.OptimalInEveryWorld, []string{"repair"}) { t.Fatalf("within-set result: %v", got.OptimalInEveryWorld) }
}

func TestDeclaredSetIsConditional(t *testing.T) {
	p := readFixture(t, "invariant")
	p.CoverageBasis = CoverageDeclaredSet
	got, err := Evaluate(p)
	if err != nil { t.Fatal(err) }
	if got.Disposition != ConditionalOnDeclaredSet { t.Fatalf("disposition: %s", got.Disposition) }
}

func TestPermutationInvariance(t *testing.T) {
	p := readFixture(t, "ambiguous")
	a, err := Evaluate(p); if err != nil { t.Fatal(err) }
	p.Actions[0], p.Actions[2] = p.Actions[2], p.Actions[0]
	p.Worlds[0], p.Worlds[1] = p.Worlds[1], p.Worlds[0]
	b, err := Evaluate(p); if err != nil { t.Fatal(err) }
	if !reflect.DeepEqual(a, b) { t.Fatalf("permutation changed output\nA=%+v\nB=%+v", a, b) }
}

func TestAddingWorldCannotExpandRobustSets(t *testing.T) {
	p := readFixture(t, "invariant")
	before, err := Evaluate(p); if err != nil { t.Fatal(err) }
	p.Worlds = append(p.Worlds, World{ID:"w3", Losses:map[string]int64{"repair":4,"hold":0}})
	after, err := Evaluate(p); if err != nil { t.Fatal(err) }
	if !isSubset(after.RobustSafeActions, before.RobustSafeActions) { t.Fatalf("robust-safe set expanded: before=%v after=%v", before.RobustSafeActions, after.RobustSafeActions) }
	if !isSubset(after.OptimalInEveryWorld, before.OptimalInEveryWorld) { t.Fatalf("identified set expanded: before=%v after=%v", before.OptimalInEveryWorld, after.OptimalInEveryWorld) }
}

func TestStrictJSONRejectsDuplicateKeysUnknownFieldsAndTrailingData(t *testing.T) {
	good, err := os.ReadFile("../testdata/invariant.json"); if err != nil { t.Fatal(err) }
	cases := [][]byte{
		[]byte(`{"schema_id":"one","schema_id":"two"}`),
		append(append([]byte(nil), good...), []byte(` {}`)...),
		[]byte(`{"schema_id":"mycelix.craft.decision-identifiability.v0.1","case_id":"x","coverage_basis":"fixture_exhaustive","actions":["hold"],"fallback_action":"hold","worlds":[{"id":"w","losses":{"hold":0},"surprise":true}]}`),
	}
	for i, raw := range cases { if _, err := DecodeProblem(raw); err == nil { t.Fatalf("case %d accepted", i) } }
}

func TestValidateRejectsMalformedProblems(t *testing.T) {
	base := readFixture(t, "invariant")
	mutations := []func(*Problem){
		func(p *Problem) { p.Actions = append(p.Actions, "repair") },
		func(p *Problem) { p.Worlds[0].Losses = map[string]int64{"hold":1} },
		func(p *Problem) { p.Worlds[0].Losses["repair"] = -1 },
		func(p *Problem) { p.Worlds[0].HardViolations = []string{"ghost"} },
		func(p *Problem) { p.Worlds = nil },
		func(p *Problem) { p.Worlds[0].HardViolations = []string{"hold"} },
		func(p *Problem) { p.Worlds[0].ID = p.Worlds[1].ID },
		func(p *Problem) { p.FallbackAction = "ghost" },
	}
	for i, mutate := range mutations { p := cloneProblem(base); mutate(&p); if err := Validate(p); err == nil { t.Fatalf("mutation %d accepted", i) } }
}

// independentSmallReference directly enumerates each action's world-by-world
// safety, optimum membership, max loss and max regret for a tiny fixture.
func independentSmallReference(p Problem) (robust, identified, minimaxRegret []string) {
	safe := func(a string) bool {
		for _, w := range p.Worlds { if contains(w.HardViolations, a) { return false } }
		return true
	}
	for _, a := range p.Actions { if safe(a) { robust = append(robust, a) } }
	for _, a := range robust {
		allOptimal := true
		for _, w := range p.Worlds {
			best := int64(1<<62)
			for _, b := range p.Actions { if !contains(w.HardViolations,b) && w.Losses[b] < best { best=w.Losses[b] } }
			if w.Losses[a] != best { allOptimal=false }
		}
		if allOptimal { identified=append(identified,a) }
	}
	regret := map[string]int64{}
	bestRegret := int64(1<<62)
	for _, a := range robust {
		var worst int64
		for _, w := range p.Worlds {
			best := int64(1<<62)
			for _, b := range p.Actions { if !contains(w.HardViolations,b) && w.Losses[b] < best { best=w.Losses[b] } }
			delta := w.Losses[a]-best
			if delta>worst { worst=delta }
		}
		regret[a]=worst
		if worst<bestRegret { bestRegret=worst }
	}
	for _, a := range robust { if regret[a]==bestRegret { minimaxRegret=append(minimaxRegret,a) } }
	sortStrings(robust); sortStrings(identified); sortStrings(minimaxRegret)
	if robust == nil { robust = []string{} }
	if identified == nil { identified = []string{} }
	if minimaxRegret == nil { minimaxRegret = []string{} }
	return
}

func TestEvaluatorMatchesIndependentExhaustiveReference(t *testing.T) {
	for a0:=int64(0);a0<3;a0++ { for a1:=int64(0);a1<3;a1++ { for h0:=int64(0);h0<3;h0++ { for h1:=int64(0);h1<3;h1++ {
		for unsafe0:=0;unsafe0<2;unsafe0++ { for unsafe1:=0;unsafe1<2;unsafe1++ {
			p:=Problem{SchemaID:SchemaID,CaseID:"exhaustive",CoverageBasis:CoverageFixtureExhaustive,Actions:[]string{"act","hold"},FallbackAction:"hold",Worlds:[]World{
				{ID:"w0",Losses:map[string]int64{"act":a0,"hold":h0},HardViolations:boolSlice(unsafe0,"act")},
				{ID:"w1",Losses:map[string]int64{"act":a1,"hold":h1},HardViolations:boolSlice(unsafe1,"act")},
			}}
			got,err:=Evaluate(p); if err!=nil { t.Fatal(err) }
			wantSafe,wantIdentified,wantRegret:=independentSmallReference(p)
			if !reflect.DeepEqual(got.RobustSafeActions,wantSafe)||!reflect.DeepEqual(got.OptimalInEveryWorld,wantIdentified)||!reflect.DeepEqual(got.MinimaxRegretActions,wantRegret) {
				t.Fatalf("mismatch p=%+v got=(%v,%v,%v) want=(%v,%v,%v)",p,got.RobustSafeActions,got.OptimalInEveryWorld,got.MinimaxRegretActions,wantSafe,wantIdentified,wantRegret)
			}
		} }
	} } } }
}

func boolSlice(enabled int, value string) []string { if enabled==1 { return []string{value} }; return nil }
func cloneProblem(p Problem) Problem { q:=p; q.Actions=append([]string(nil),p.Actions...); q.Worlds=append([]World(nil),p.Worlds...); for i:=range q.Worlds { m:=map[string]int64{};for k,v:=range p.Worlds[i].Losses{m[k]=v};q.Worlds[i].Losses=m;q.Worlds[i].HardViolations=append([]string(nil),p.Worlds[i].HardViolations...) };return q }
func isSubset(a,b []string) bool { for _,x:=range a { if !contains(b,x) { return false } };return true }
func sortStrings(v []string) { for i:=0;i<len(v);i++ { for j:=i+1;j<len(v);j++ { if v[j]<v[i] {v[i],v[j]=v[j],v[i]} } } }
