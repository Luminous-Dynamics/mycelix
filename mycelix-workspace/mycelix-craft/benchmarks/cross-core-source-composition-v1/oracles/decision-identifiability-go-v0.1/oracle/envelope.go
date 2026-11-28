// Package oracle evaluates decision identifiability over a finite declared
// world set. It is a test/reference oracle, not a production policy engine.
package oracle

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"sort"
)

const SchemaID = "mycelix.craft.decision-identifiability.v0.1"

var frozenFixtureDigests = map[string]string{
	"CDIE-001-invariant-action": "6fc9da3fb2c237033531454fda2d90471c9cb59a940b8b523cc746cbc8e9e7ef",
	"CDIE-002-no-invariant-optimum": "e82893357979278e83bfe97e6526837cb16bf87ac0ca79a1be7bb2895d0cb6b6",
	"CDIE-003-hard-safety-veto": "6ac5bfd06dfd73ba2328b2671d9b2ade0bc9557ad8c1261311947cee34946",
}

type CoverageBasis string

const (
	CoverageFixtureExhaustive CoverageBasis = "fixture_exhaustive"
	CoverageDeclaredSet       CoverageBasis = "declared_set"
	CoverageNotEvaluated      CoverageBasis = "not_evaluated"
)

type Problem struct {
	SchemaID       string        `json:"schema_id"`
	CaseID         string        `json:"case_id"`
	CoverageBasis  CoverageBasis `json:"coverage_basis"`
	FixtureDigest string `json:"fixture_digest,omitempty"`
	Actions        []string      `json:"actions"`
	FallbackAction string        `json:"fallback_action"`
	Worlds         []World       `json:"worlds"`
}

type World struct {
	ID             string           `json:"id"`
	Losses         map[string]int64 `json:"losses"`
	HardViolations []string         `json:"hard_violations,omitempty"`
	Attributes     map[string]string `json:"attributes,omitempty"`
}

type Disposition string

const (
	RobustWithinExhaustiveFixture Disposition = "ROBUST_WITHIN_EXHAUSTIVE_FIXTURE"
	ConditionalOnDeclaredSet      Disposition = "CONDITIONAL_ON_DECLARED_WORLD_SET"
	NotIdentifiedExhaustive       Disposition = "NOT_IDENTIFIED_WITHIN_EXHAUSTIVE_FIXTURE"
	NotIdentifiedDeclaredSet      Disposition = "NOT_IDENTIFIED_WITHIN_DECLARED_WORLD_SET"
	WorldSetCoverageNotEvaluated  Disposition = "WORLD_SET_COVERAGE_NOT_EVALUATED"
)

type ActionBounds struct {
	ActionID        string `json:"action_id"`
	RobustSafe      bool   `json:"robust_safe"`
	MinLoss         int64  `json:"min_loss"`
	MaxLoss         int64  `json:"max_loss"`
	WorstCaseLoss   *int64 `json:"worst_case_loss,omitempty"`
	WorstCaseRegret *int64 `json:"worst_case_regret,omitempty"`
}

type WorldOptimum struct {
	WorldID  string   `json:"world_id"`
	Actions  []string `json:"actions"`
	BestLoss int64    `json:"best_loss"`
}

type DecisionFlipWitness struct {
	WorldA              string   `json:"world_a"`
	OptimalActionsA     []string `json:"optimal_actions_a"`
	WorldB              string   `json:"world_b"`
	OptimalActionsB     []string `json:"optimal_actions_b"`
	DifferingDimensions []string `json:"differing_dimensions"`
}

type Result struct {
	CaseID               string         `json:"case_id"`
	CoverageBasis        CoverageBasis  `json:"coverage_basis"`
	Disposition          Disposition    `json:"disposition"`
	WorldCount           int            `json:"world_count"`
	RobustSafeActions    []string       `json:"robust_safe_actions"`
	OptimalInEveryListedWorld []string       `json:"optimal_in_every_listed_world"`
	DecisionFlipWitness       *DecisionFlipWitness `json:"decision_flip_witness,omitempty"`
	MinimaxLossActions   []string       `json:"minimax_loss_actions"`
	MinimaxRegretActions []string       `json:"minimax_regret_actions"`
	ActionBounds         []ActionBounds `json:"action_bounds"`
	WorldOptima          []WorldOptimum `json:"world_optima"`
}

// DecodeProblem rejects duplicate JSON object keys, unknown fields and trailing
// data before returning a typed problem.
func DecodeProblem(raw []byte) (Problem, error) {
	if err := validateJSON(raw); err != nil {
		return Problem{}, err
	}
	dec := json.NewDecoder(bytes.NewReader(raw))
	dec.DisallowUnknownFields()
	var p Problem
	if err := dec.Decode(&p); err != nil {
		return Problem{}, err
	}
	if err := ensureEOF(dec); err != nil {
		return Problem{}, err
	}
	if err := Validate(p); err != nil {
		return Problem{}, err
	}
	return p, nil
}

func validateJSON(raw []byte) error {
	dec := json.NewDecoder(bytes.NewReader(raw))
	if err := scanValue(dec); err != nil {
		return err
	}
	return ensureEOF(dec)
}

func scanValue(dec *json.Decoder) error {
	tok, err := dec.Token()
	if err != nil {
		return err
	}
	delim, ok := tok.(json.Delim)
	if !ok {
		return nil
	}
	switch delim {
	case '{':
		seen := make(map[string]struct{})
		for dec.More() {
			keyToken, err := dec.Token()
			if err != nil {
				return err
			}
			key, ok := keyToken.(string)
			if !ok {
				return errors.New("object key is not a string")
			}
			if _, exists := seen[key]; exists {
				return fmt.Errorf("duplicate JSON object key %q", key)
			}
			seen[key] = struct{}{}
			if err := scanValue(dec); err != nil {
				return err
			}
		}
		closeToken, err := dec.Token()
		if err != nil {
			return err
		}
		if closeToken != json.Delim('}') {
			return errors.New("malformed JSON object")
		}
	case '[':
		for dec.More() {
			if err := scanValue(dec); err != nil {
				return err
			}
		}
		closeToken, err := dec.Token()
		if err != nil {
			return err
		}
		if closeToken != json.Delim(']') {
			return errors.New("malformed JSON array")
		}
	default:
		return errors.New("unexpected JSON delimiter")
	}
	return nil
}

func ensureEOF(dec *json.Decoder) error {
	if _, err := dec.Token(); err != io.EOF {
		if err == nil {
			return errors.New("trailing JSON value")
		}
		return fmt.Errorf("trailing JSON or invalid bytes: %w", err)
	}
	return nil
}

func Validate(p Problem) error {
	if p.SchemaID != SchemaID {
		return fmt.Errorf("schema_id must equal %q", SchemaID)
	}
	if p.CaseID == "" {
		return errors.New("case_id is required")
	}
	switch p.CoverageBasis {
	case CoverageFixtureExhaustive:
		expected, ok := frozenFixtureDigests[p.CaseID]
		if !ok {
			return fmt.Errorf("fixture %q is not in the frozen exhaustive allowlist", p.CaseID)
		}
		actual, err := problemDigest(p)
		if err != nil {
			return fmt.Errorf("cannot digest fixture: %w", err)
		}
		if p.FixtureDigest == "" || p.FixtureDigest != expected || actual != expected {
			return errors.New("fixture digest does not match the frozen exhaustive fixture registry")
		}
	case CoverageDeclaredSet, CoverageNotEvaluated:
		if p.FixtureDigest != "" {
			return errors.New("fixture_digest is only valid for a pinned exhaustive fixture")
		}
	default:
		return fmt.Errorf("unsupported coverage_basis %q", p.CoverageBasis)
	}
	if len(p.Actions) == 0 {
		return errors.New("at least one action is required")
	}
	actions := make(map[string]struct{}, len(p.Actions))
	for _, id := range p.Actions {
		if id == "" {
			return errors.New("action IDs must be non-empty")
		}
		if _, exists := actions[id]; exists {
			return fmt.Errorf("duplicate action ID %q", id)
		}
		actions[id] = struct{}{}
	}
	if _, ok := actions[p.FallbackAction]; !ok || p.FallbackAction == "" {
		return errors.New("fallback_action must name a declared action")
	}
	if len(p.Worlds) == 0 {
		return errors.New("at least one compatible world is required")
	}
	worldIDs := make(map[string]struct{}, len(p.Worlds))
	for _, w := range p.Worlds {
		if w.ID == "" {
			return errors.New("world IDs must be non-empty")
		}
		if _, exists := worldIDs[w.ID]; exists {
			return fmt.Errorf("duplicate world ID %q", w.ID)
		}
		worldIDs[w.ID] = struct{}{}
		if len(w.Losses) != len(actions) {
			return fmt.Errorf("world %q must declare exactly one loss for every action", w.ID)
		}
		for action := range actions {
			loss, ok := w.Losses[action]
			if !ok {
				return fmt.Errorf("world %q is missing loss for action %q", w.ID, action)
			}
			if loss < 0 {
				return fmt.Errorf("world %q has negative loss for action %q", w.ID, action)
			}
		}
		for action := range w.Losses {
			if _, ok := actions[action]; !ok {
				return fmt.Errorf("world %q contains unknown action %q", w.ID, action)
			}
		}
		seenViolations := make(map[string]struct{}, len(w.HardViolations))
		for _, action := range w.HardViolations {
			if _, ok := actions[action]; !ok {
				return fmt.Errorf("world %q has hard violation for unknown action %q", w.ID, action)
			}
			if _, exists := seenViolations[action]; exists {
				return fmt.Errorf("world %q repeats hard violation for action %q", w.ID, action)
			}
			seenViolations[action] = struct{}{}
		}
		if _, unsafe := seenViolations[p.FallbackAction]; unsafe {
			return fmt.Errorf("fallback action %q is unsafe in world %q", p.FallbackAction, w.ID)
		}
	}
	return nil
}

func Evaluate(p Problem) (Result, error) {
	if err := Validate(p); err != nil {
		return Result{}, err
	}
	actions := append([]string(nil), p.Actions...)
	sort.Strings(actions)
	worlds := append([]World(nil), p.Worlds...)
	sort.Slice(worlds, func(i, j int) bool { return worlds[i].ID < worlds[j].ID })

	robustSafe := make([]string, 0, len(actions))
	bounds := make(map[string]ActionBounds, len(actions))
	worldOptima := make([]WorldOptimum, 0, len(worlds))
	var identified map[string]struct{}
	bestLossByWorld := make(map[string]int64, len(worlds))

	for _, action := range actions {
		minLoss, maxLoss := int64(^uint64(0)>>1), int64(0)
		isSafe := true
		for _, world := range worlds {
			loss := world.Losses[action]
			if loss < minLoss { minLoss = loss }
			if loss > maxLoss { maxLoss = loss }
			if contains(world.HardViolations, action) { isSafe = false }
		}
		bounds[action] = ActionBounds{ActionID: action, RobustSafe: isSafe, MinLoss: minLoss, MaxLoss: maxLoss}
		if isSafe { robustSafe = append(robustSafe, action) }
	}

	for _, world := range worlds {
		best := int64(^uint64(0) >> 1)
		for _, action := range actions {
			if contains(world.HardViolations, action) { continue }
			if world.Losses[action] < best { best = world.Losses[action] }
		}
		bestLossByWorld[world.ID] = best
		optimal := make([]string, 0)
		for _, action := range actions {
			if !contains(world.HardViolations, action) && world.Losses[action] == best {
				optimal = append(optimal, action)
			}
		}
		worldOptima = append(worldOptima, WorldOptimum{WorldID: world.ID, Actions: optimal, BestLoss: best})
		current := make(map[string]struct{}, len(optimal))
		for _, action := range optimal { current[action] = struct{}{} }
		if identified == nil {
			identified = current
		} else {
			for action := range identified {
				if _, ok := current[action]; !ok { delete(identified, action) }
			}
		}
	}

	optimalEveryListedWorld := keys(identified)
	decisionFlipWitness := findDecisionFlipWitness(worlds, worldOptima)
	worstLoss := make(map[string]int64, len(robustSafe))
	worstRegret := make(map[string]int64, len(robustSafe))
	for _, action := range robustSafe {
		maxLoss, maxRegret := int64(0), int64(0)
		for _, world := range worlds {
			loss := world.Losses[action]
			if loss > maxLoss { maxLoss = loss }
			regret := loss - bestLossByWorld[world.ID]
			if regret > maxRegret { maxRegret = regret }
		}
		worstLoss[action] = maxLoss
		worstRegret[action] = maxRegret
		b := bounds[action]
		b.WorstCaseLoss = intPtr(maxLoss)
		b.WorstCaseRegret = intPtr(maxRegret)
		bounds[action] = b
	}

	minimaxLoss := minimizingActions(robustSafe, worstLoss)
	minimaxRegret := minimizingActions(robustSafe, worstRegret)
	orderedBounds := make([]ActionBounds, 0, len(actions))
	for _, action := range actions { orderedBounds = append(orderedBounds, bounds[action]) }

	disposition := dispositionFor(p.CoverageBasis, len(optimalEveryListedWorld) > 0)
	return Result{
		CaseID: p.CaseID,
		CoverageBasis: p.CoverageBasis,
		Disposition: disposition,
		WorldCount: len(worlds),
		RobustSafeActions: robustSafe,
		OptimalInEveryListedWorld: optimalEveryListedWorld,
		DecisionFlipWitness: decisionFlipWitness,
		MinimaxLossActions: minimaxLoss,
		MinimaxRegretActions: minimaxRegret,
		ActionBounds: orderedBounds,
		WorldOptima: worldOptima,
	}, nil
}


func problemDigest(p Problem) (string, error) {
	p.FixtureDigest = ""
	encoded, err := json.Marshal(p)
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return hex.EncodeToString(digest[:]), nil
}

func findDecisionFlipWitness(worlds []World, optima []WorldOptimum) *DecisionFlipWitness {
	var best *DecisionFlipWitness
	for i := 0; i < len(worlds); i++ {
		for j := i + 1; j < len(worlds); j++ {
			if intersects(optima[i].Actions, optima[j].Actions) {
				continue
			}
			dimensions := differingDimensions(worlds[i].Attributes, worlds[j].Attributes)
			candidate := &DecisionFlipWitness{
				WorldA: worlds[i].ID,
				OptimalActionsA: append([]string(nil), optima[i].Actions...),
				WorldB: worlds[j].ID,
				OptimalActionsB: append([]string(nil), optima[j].Actions...),
				DifferingDimensions: dimensions,
			}
			if best == nil ||
				len(candidate.DifferingDimensions) < len(best.DifferingDimensions) ||
				(len(candidate.DifferingDimensions) == len(best.DifferingDimensions) &&
					(candidate.WorldA < best.WorldA ||
						(candidate.WorldA == best.WorldA && candidate.WorldB < best.WorldB))) {
				best = candidate
			}
		}
	}
	return best
}

func intersects(a, b []string) bool {
	for _, left := range a {
		if contains(b, left) {
			return true
		}
	}
	return false
}

func differingDimensions(a, b map[string]string) []string {
	keys := make(map[string]struct{}, len(a)+len(b))
	for key := range a { keys[key] = struct{}{} }
	for key := range b { keys[key] = struct{}{} }
	out := make([]string, 0)
	for key := range keys {
		left, leftOK := a[key]
		right, rightOK := b[key]
		if !leftOK || !rightOK || left != right {
			out = append(out, key)
		}
	}
	sort.Strings(out)
	return out
}

func dispositionFor(basis CoverageBasis, identified bool) Disposition {
	if basis == CoverageNotEvaluated { return WorldSetCoverageNotEvaluated }
	if identified {
		if basis == CoverageFixtureExhaustive { return RobustWithinExhaustiveFixture }
		return ConditionalOnDeclaredSet
	}
	if basis == CoverageFixtureExhaustive { return NotIdentifiedExhaustive }
	return NotIdentifiedDeclaredSet
}

func minimizingActions(candidates []string, metric map[string]int64) []string {
	if len(candidates) == 0 { return []string{} }
	best := int64(^uint64(0) >> 1)
	for _, action := range candidates { if metric[action] < best { best = metric[action] } }
	out := make([]string, 0)
	for _, action := range candidates { if metric[action] == best { out = append(out, action) } }
	return out
}

func contains(items []string, target string) bool {
	for _, item := range items { if item == target { return true } }
	return false
}

func keys(set map[string]struct{}) []string {
	out := make([]string, 0, len(set))
	for key := range set { out = append(out, key) }
	sort.Strings(out)
	return out
}

func intPtr(v int64) *int64 { return &v }
