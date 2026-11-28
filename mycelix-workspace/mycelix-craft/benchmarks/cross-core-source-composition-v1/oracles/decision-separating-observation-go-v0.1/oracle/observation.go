// Package oracle evaluates whether an observation partitions finite possible
// worlds into branches with robustly identified safe actions. It assigns no
// probabilities and makes no source-completeness or independence claims.
package oracle

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"sort"
)

const SchemaID = "mycelix.craft.decision-separating-observation.v0.1"

type ScopeBasis string

const (
	ScopeDeclaredSet  ScopeBasis = "declared_set"
	ScopeNotEvaluated ScopeBasis = "not_evaluated"
)

type World struct {
	ID             string           "json:\"id\""
	Losses         map[string]int64 "json:\"losses\""
	HardViolations []string         "json:\"hard_violations,omitempty\""
}

type Observation struct {
	ID                      string            "json:\"id\""
	OutcomeByWorld          map[string]string "json:\"outcome_by_world\""
	AcquisitionCostUnits    int64             "json:\"acquisition_cost_units,omitempty\""
	DelayUnits              int64             "json:\"delay_units,omitempty\""
	BurdenUnits             int64             "json:\"burden_units,omitempty\""
	FailureDomainIDs        []string          "json:\"failure_domain_ids,omitempty\""
	ExistingFailureDomainIDs []string         "json:\"existing_failure_domain_ids,omitempty\""
}

type Problem struct {
	SchemaID      string        "json:\"schema_id\""
	CaseID        string        "json:\"case_id\""
	ScopeBasis    ScopeBasis    "json:\"scope_basis\""
	Actions       []string      "json:\"actions\""
	Worlds        []World       "json:\"worlds\""
	Observations  []Observation "json:\"observations\""
}

type DecisionSummary struct {
	WorldCount              int      "json:\"world_count\""
	RobustSafeActions       []string "json:\"robust_safe_actions\""
	OptimalInEveryWorld     []string "json:\"optimal_in_every_world\""
	MinimaxLossActions      []string "json:\"minimax_loss_actions\""
	MinimaxRegretActions    []string "json:\"minimax_regret_actions\""
	WorstCaseMinimaxRegret  *int64   "json:\"worst_case_minimax_regret,omitempty\""
}

type OutcomeBranch struct {
	Outcome          string          "json:\"outcome\""
	WorldIDs         []string        "json:\"world_ids\""
	WorldCount       int             "json:\"world_count\""
	Decision         DecisionSummary "json:\"decision\""
	DecisionIdentified bool           "json:\"decision_identified\""
}

type ObservationResult struct {
	ObservationID               string          "json:\"observation_id\""
	MappingComplete             bool            "json:\"mapping_complete\""
	MissingWorldIDs             []string        "json:\"missing_world_ids,omitempty\""
	UnknownWorldIDs             []string        "json:\"unknown_world_ids,omitempty\""
	OutcomeCount                int             "json:\"outcome_count\""
	Branches                    []OutcomeBranch "json:\"branches\""
	IdentifiedWorldCount        int             "json:\"identified_world_count\""
	WorldCount                  int             "json:\"world_count\""
	IdentifiedWorldCountShare   string          "json:\"identified_world_count_share\""
	UniversallySeparatingWithinListedWorlds bool `json:"universally_separating_within_listed_worlds"`
	Disposition                 string          "json:\"disposition\""
	PreObservationDecision     DecisionSummary "json:\"pre_observation_decision\""
	WorstBranchMinimaxRegret    *int64          "json:\"worst_branch_minimax_regret,omitempty\""
	PreObservationMinimaxRegret *int64          "json:\"pre_observation_minimax_regret,omitempty\""
	WorstCaseRegretReduction    *int64          "json:\"worst_case_regret_reduction,omitempty\""
	IndependenceStatus          string          "json:\"independence_status\""
	SharedFailureDomainIDs      []string        "json:\"shared_failure_domain_ids,omitempty\""
	AcquisitionCostUnits        int64           "json:\"acquisition_cost_units,omitempty\""
	DelayUnits                  int64           "json:\"delay_units,omitempty\""
	BurdenUnits                 int64           "json:\"burden_units,omitempty\""
}

type Result struct {
	CaseID             string               "json:\"case_id\""
	ScopeBasis         ScopeBasis           "json:\"scope_basis\""
	ScopeStatus        string               "json:\"scope_status\""
	WorldCount         int                  "json:\"world_count\""
	PreObservation     DecisionSummary      "json:\"pre_observation\""
	Observations       []ObservationResult  "json:\"observations\""
}

func DecodeProblem(raw []byte) (Problem, error) {
	decoder := json.NewDecoder(bytes.NewReader(raw))
	if err := rejectDuplicateKeys(raw); err != nil {
		return Problem{}, err
	}
	decoder.DisallowUnknownFields()
	var problem Problem
	if err := decoder.Decode(&problem); err != nil {
		return Problem{}, err
	}
	if err := ensureEOF(decoder); err != nil {
		return Problem{}, err
	}
	if err := Validate(problem); err != nil {
		return Problem{}, err
	}
	return problem, nil
}

func Validate(problem Problem) error {
	if problem.SchemaID != SchemaID {
		return fmt.Errorf("schema_id must equal %q", SchemaID)
	}
	if problem.CaseID == "" {
		return errors.New("case_id is required")
	}
	switch problem.ScopeBasis {
	case ScopeDeclaredSet, ScopeNotEvaluated:
	default:
		return fmt.Errorf("unsupported scope_basis %q", problem.ScopeBasis)
	}
	if len(problem.Actions) == 0 {
		return errors.New("at least one action is required")
	}
	if len(problem.Worlds) == 0 {
		return errors.New("at least one world is required")
	}
	if len(problem.Observations) == 0 {
		return errors.New("at least one candidate observation is required")
	}

	actions := make(map[string]struct{}, len(problem.Actions))
	for _, action := range problem.Actions {
		if action == "" {
			return errors.New("action IDs must be non-empty")
		}
		if _, exists := actions[action]; exists {
			return fmt.Errorf("duplicate action ID %q", action)
		}
		actions[action] = struct{}{}
	}

	worldIDs := make(map[string]World, len(problem.Worlds))
	for _, world := range problem.Worlds {
		if world.ID == "" {
			return errors.New("world IDs must be non-empty")
		}
		if _, exists := worldIDs[world.ID]; exists {
			return fmt.Errorf("duplicate world ID %q", world.ID)
		}
		if len(world.Losses) != len(actions) {
			return fmt.Errorf("world %q must declare exactly one loss per action", world.ID)
		}
		for action := range actions {
			loss, ok := world.Losses[action]
			if !ok {
				return fmt.Errorf("world %q is missing loss for action %q", world.ID, action)
			}
			if loss < 0 {
				return fmt.Errorf("world %q has negative loss for action %q", world.ID, action)
			}
		}
		for action := range world.Losses {
			if _, ok := actions[action]; !ok {
				return fmt.Errorf("world %q contains unknown action %q", world.ID, action)
			}
		}
		seenViolations := make(map[string]struct{}, len(world.HardViolations))
		for _, action := range world.HardViolations {
			if _, ok := actions[action]; !ok {
				return fmt.Errorf("world %q has unknown hard-violation action %q", world.ID, action)
			}
			if _, exists := seenViolations[action]; exists {
				return fmt.Errorf("world %q repeats hard violation %q", world.ID, action)
			}
			seenViolations[action] = struct{}{}
		}
		worldIDs[world.ID] = world
	}

	observationIDs := make(map[string]struct{}, len(problem.Observations))
	for _, observation := range problem.Observations {
		if observation.ID == "" {
			return errors.New("observation IDs must be non-empty")
		}
		if _, exists := observationIDs[observation.ID]; exists {
			return fmt.Errorf("duplicate observation ID %q", observation.ID)
		}
		observationIDs[observation.ID] = struct{}{}
		if observation.AcquisitionCostUnits < 0 || observation.DelayUnits < 0 || observation.BurdenUnits < 0 {
			return fmt.Errorf("observation %q has a negative cost/delay/burden", observation.ID)
		}
		for worldID, outcome := range observation.OutcomeByWorld {
			if _, ok := worldIDs[worldID]; !ok {
				return fmt.Errorf("observation %q maps unknown world %q", observation.ID, worldID)
			}
			if outcome == "" {
				return fmt.Errorf("observation %q has empty outcome for world %q", observation.ID, worldID)
			}
		}
		if err := uniqueNonEmpty(observation.FailureDomainIDs); err != nil {
			return fmt.Errorf("observation %q failure_domain_ids: %w", observation.ID, err)
		}
		if err := uniqueNonEmpty(observation.ExistingFailureDomainIDs); err != nil {
			return fmt.Errorf("observation %q existing_failure_domain_ids: %w", observation.ID, err)
		}
	}
	return nil
}

func Evaluate(problem Problem) (Result, error) {
	result := Result{
		CaseID:      problem.CaseID,
		ScopeBasis:  problem.ScopeBasis,
		ScopeStatus: scopeStatus(problem.ScopeBasis),
	}
	if err := Validate(problem); err != nil {
		return result, err
	}
	worlds := append([]World(nil), problem.Worlds...)
	sort.Slice(worlds, func(i, j int) bool { return worlds[i].ID < worlds[j].ID })
	result.WorldCount = len(worlds)
	result.PreObservation = summarize(problem.Actions, worlds)

	observations := append([]Observation(nil), problem.Observations...)
	sort.Slice(observations, func(i, j int) bool { return observations[i].ID < observations[j].ID })
	results := make([]ObservationResult, 0, len(observations))
	for _, observation := range observations {
		results = append(results, evaluateObservation(problem, worlds, observation, result.PreObservation))
	}
	result.Observations = results
	return result, nil
}

func evaluateObservation(problem Problem, worlds []World, observation Observation, baseline DecisionSummary) ObservationResult {
	missing := make([]string, 0)
	for _, world := range worlds {
		if _, ok := observation.OutcomeByWorld[world.ID]; !ok {
			missing = append(missing, world.ID)
		}
	}
	sort.Strings(missing)
	sharedDomains := intersectionSorted(observation.FailureDomainIDs, observation.ExistingFailureDomainIDs)
	independence := "NOT_EVALUATED"
	if len(sharedDomains) > 0 {
		independence = "SHARED_FAILURE_DOMAIN"
	}
	result := ObservationResult{
		ObservationID: observation.ID,
		MappingComplete: len(missing) == 0,
		MissingWorldIDs: missing,
		WorldCount: len(worlds),
		IdentifiedWorldCountShare: fmt.Sprintf("0/%d", len(worlds)),
		PreObservationDecision: baseline,
		IndependenceStatus: independence,
		SharedFailureDomainIDs: sharedDomains,
		AcquisitionCostUnits: observation.AcquisitionCostUnits,
		DelayUnits: observation.DelayUnits,
		BurdenUnits: observation.BurdenUnits,
	}
	if len(missing) > 0 {
		result.Disposition = "OBSERVATION_MODEL_INCOMPLETE"
		return result
	}

	branches := make(map[string][]World)
	branchIDs := make(map[string][]string)
	for _, world := range worlds {
		outcome := observation.OutcomeByWorld[world.ID]
		branches[outcome] = append(branches[outcome], world)
		branchIDs[outcome] = append(branchIDs[outcome], world.ID)
	}
	outcomes := make([]string, 0, len(branches))
	for outcome := range branches {
		outcomes = append(outcomes, outcome)
	}
	sort.Strings(outcomes)

	branchResults := make([]OutcomeBranch, 0, len(outcomes))
	identifiedWorldCount := 0
	var worstBranchRegret *int64
	universallySeparating := len(outcomes) > 0
	for _, outcome := range outcomes {
		branchWorlds := branches[outcome]
		summary := summarize(problem.Actions, branchWorlds)
		identified := len(summary.RobustSafeActions) > 0 && len(summary.OptimalInEveryWorld) > 0
		if !identified {
			universallySeparating = false
		} else {
			identifiedWorldCount += len(branchWorlds)
		}
		branchResults = append(branchResults, OutcomeBranch{
			Outcome: outcome,
			WorldIDs: append([]string(nil), branchIDs[outcome]...),
			WorldCount: len(branchWorlds),
			Decision: summary,
			DecisionIdentified: identified,
		})
		if summary.WorstCaseMinimaxRegret == nil {
			universallySeparating = false
		} else if worstBranchRegret == nil || *summary.WorstCaseMinimaxRegret > *worstBranchRegret {
			value := *summary.WorstCaseMinimaxRegret
			worstBranchRegret = &value
		}
	}
	result.Branches = branchResults
	result.OutcomeCount = len(outcomes)
	result.IdentifiedWorldCount = identifiedWorldCount
	result.IdentifiedWorldCountShare = fmt.Sprintf("%d/%d", identifiedWorldCount, len(worlds))
	result.UniversallySeparatingWithinListedWorlds = universallySeparating
	result.WorstBranchMinimaxRegret = worstBranchRegret
	result.PreObservationMinimaxRegret = baseline.WorstCaseMinimaxRegret
	if baseline.WorstCaseMinimaxRegret != nil && worstBranchRegret != nil {
		reduction := *baseline.WorstCaseMinimaxRegret - *worstBranchRegret
		result.WorstCaseRegretReduction = &reduction
	}
	switch {
	case problem.ScopeBasis == ScopeNotEvaluated:
		result.Disposition = "WORLD_SET_COVERAGE_NOT_EVALUATED"
	case universallySeparating && problem.ScopeBasis == ScopeDeclaredSet:
		result.Disposition = "CONDITIONALLY_DECISION_SEPARATING_WITHIN_DECLARED_SET"
	case universallySeparating:
		result.Disposition = "DECISION_SEPARATING_WITHIN_LISTED_WORLDS"
	default:
		result.Disposition = "NOT_UNIVERSALLY_DECISION_SEPARATING_WITHIN_LISTED_WORLDS"
	}
	return result
}

func summarize(actions []string, worlds []World) DecisionSummary {
	result := DecisionSummary{WorldCount: len(worlds)}
	ordered := append([]string(nil), actions...)
	sort.Strings(ordered)
	robustSafe := make(map[string]struct{}, len(ordered))
	identified := make(map[string]struct{}, len(ordered))
	for _, action := range ordered {
		robustSafe[action] = struct{}{}
		identified[action] = struct{}{}
	}
	bestByWorld := make([]int64, len(worlds))
	for i, world := range worlds {
		best := int64(^uint64(0) >> 1)
		for _, action := range ordered {
			if contains(world.HardViolations, action) {
				delete(robustSafe, action)
				continue
			}
			if world.Losses[action] < best {
				best = world.Losses[action]
			}
		}
		if best == int64(^uint64(0)>>1) {
			for action := range identified {
				delete(identified, action)
			}
			bestByWorld[i] = best
			continue
		}
		bestByWorld[i] = best
		for action := range identified {
			if contains(world.HardViolations, action) || world.Losses[action] != best {
				delete(identified, action)
			}
		}
	}
	result.RobustSafeActions = sortedKeys(robustSafe)
	result.OptimalInEveryWorld = sortedKeys(identified)
	worstLoss := make(map[string]int64)
	worstRegret := make(map[string]int64)
	for action := range robustSafe {
		maxLoss := int64(0)
		maxRegret := int64(0)
		valid := true
		for i, world := range worlds {
			if bestByWorld[i] == int64(^uint64(0)>>1) {
				valid = false
				break
			}
			if world.Losses[action] > maxLoss {
				maxLoss = world.Losses[action]
			}
			regret := world.Losses[action] - bestByWorld[i]
			if regret > maxRegret {
				maxRegret = regret
			}
		}
		if valid {
			worstLoss[action] = maxLoss
			worstRegret[action] = maxRegret
		}
	}
	result.MinimaxLossActions = minimize(robustSafe, worstLoss)
	result.MinimaxRegretActions = minimize(robustSafe, worstRegret)
	if len(result.MinimaxRegretActions) > 0 {
		for _, action := range result.MinimaxRegretActions {
			if value, ok := worstRegret[action]; ok {
				if result.WorstCaseMinimaxRegret == nil || value > *result.WorstCaseMinimaxRegret {
					copy := value
					result.WorstCaseMinimaxRegret = &copy
				}
			}
		}
	}
	return result
}

func intersectionSorted(a, b []string) []string {
	set := make(map[string]struct{}, len(a))
	for _, item := range a {
		set[item] = struct{}{}
	}
	out := make([]string, 0)
	for _, item := range b {
		if _, ok := set[item]; ok {
			out = append(out, item)
		}
	}
	sort.Strings(out)
	return out
}

func uniqueNonEmpty(values []string) error {
	seen := make(map[string]struct{}, len(values))
	for _, value := range values {
		if value == "" {
			return errors.New("values must be non-empty")
		}
		if _, exists := seen[value]; exists {
			return fmt.Errorf("duplicate value %q", value)
		}
		seen[value] = struct{}{}
	}
	return nil
}

func contains(values []string, needle string) bool {
	for _, value := range values {
		if value == needle {
			return true
		}
	}
	return false
}

func sortedKeys(set map[string]struct{}) []string {
	out := make([]string, 0, len(set))
	for key := range set {
		out = append(out, key)
	}
	sort.Strings(out)
	return out
}

func minimize(candidates map[string]struct{}, metric map[string]int64) []string {
	if len(candidates) == 0 || len(metric) == 0 {
		return []string{}
	}
	best := int64(^uint64(0) >> 1)
	for action := range candidates {
		value, ok := metric[action]
		if ok && value < best {
			best = value
		}
	}
	if best == int64(^uint64(0)>>1) {
		return []string{}
	}
	out := make([]string, 0)
	for action := range candidates {
		if value, ok := metric[action]; ok && value == best {
			out = append(out, action)
		}
	}
	sort.Strings(out)
	return out
}

func scopeStatus(basis ScopeBasis) string {
	if basis == ScopeDeclaredSet {
		return "CONDITIONAL_ON_DECLARED_WORLD_SET"
	}
	return "NOT_EVALUATED"
}

func rejectDuplicateKeys(raw []byte) error {
	decoder := json.NewDecoder(bytes.NewReader(raw))
	if err := scanValue(decoder); err != nil {
		return err
	}
	return ensureEOF(decoder)
}

func scanValue(decoder *json.Decoder) error {
	token, err := decoder.Token()
	if err != nil {
		return err
	}
	delimiter, ok := token.(json.Delim)
	if !ok {
		return nil
	}
	switch delimiter {
	case '{':
		seen := make(map[string]struct{})
		for decoder.More() {
			keyToken, err := decoder.Token()
			if err != nil {
				return err
			}
			key, ok := keyToken.(string)
			if !ok {
				return errors.New("JSON object key is not a string")
			}
			if _, exists := seen[key]; exists {
				return fmt.Errorf("duplicate JSON object key %q", key)
			}
			seen[key] = struct{}{}
			if err := scanValue(decoder); err != nil {
				return err
			}
		}
		closing, err := decoder.Token()
		if err != nil {
			return err
		}
		if closing != json.Delim('}') {
			return errors.New("malformed JSON object")
		}
	case '[':
		for decoder.More() {
			if err := scanValue(decoder); err != nil {
				return err
			}
		}
		closing, err := decoder.Token()
		if err != nil {
			return err
		}
		if closing != json.Delim(']') {
			return errors.New("malformed JSON array")
		}
	default:
		return errors.New("unexpected JSON delimiter")
	}
	return nil
}

func ensureEOF(decoder *json.Decoder) error {
	var extra any
	if err := decoder.Decode(&extra); err != io.EOF {
		if err == nil {
			return errors.New("trailing JSON value")
		}
		return fmt.Errorf("trailing JSON or malformed bytes: %w", err)
	}
	return nil
}
