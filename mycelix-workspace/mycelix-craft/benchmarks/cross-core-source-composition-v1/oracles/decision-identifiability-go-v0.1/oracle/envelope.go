// Package oracle evaluates decision identifiability over a finite declared
// world set. It is a test/reference oracle, not a production policy engine.
package oracle

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"sort"
)

const SchemaID = "mycelix.craft.decision-identifiability.v0.1"

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
	Actions        []string      `json:"actions"`
	FallbackAction string        `json:"fallback_action"`
	Worlds         []World       `json:"worlds"`
}

type World struct {
	ID             string           `json:"id"`
	Losses         map[string]int64 `json:"losses"`
	HardViolations []string         `json:"hard_violations,omitempty"`
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

type Result struct {
	CaseID               string         `json:"case_id"`
	CoverageBasis        CoverageBasis  `json:"coverage_basis"`
	Disposition          Disposition    `json:"disposition"`
	WorldCount           int            `json:"world_count"`
	RobustSafeActions    []string       `json:"robust_safe_actions"`
	OptimalInEveryWorld  []string       `json:"optimal_in_every_world"`
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
	case CoverageFixtureExhaustive, CoverageDeclaredSet, CoverageNotEvaluated:
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

	optimalEverywhere := keys(identified)
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

	disposition := dispositionFor(p.CoverageBasis, len(optimalEverywhere) > 0)
	return Result{
		CaseID: p.CaseID,
		CoverageBasis: p.CoverageBasis,
		Disposition: disposition,
		WorldCount: len(worlds),
		RobustSafeActions: robustSafe,
		OptimalInEveryWorld: optimalEverywhere,
		MinimaxLossActions: minimaxLoss,
		MinimaxRegretActions: minimaxRegret,
		ActionBounds: orderedBounds,
		WorldOptima: worldOptima,
	}, nil
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
