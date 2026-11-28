// Package oracle is an independent, test-only finite-state EVSI reference.
// It does not make field recommendations or authorize operational effects.
package oracle

import (
	"errors"
	"math"
	"sort"
)

type State struct {
	ID          string  `json:"id"`
	Probability float64 `json:"probability"`
}

type Action struct {
	ID   string             `json:"id"`
	Loss map[string]float64 `json:"loss_by_state"`
}

type CostProfile struct {
	ID           string  `json:"id"`
	MoneyWeight  float64 `json:"money_weight"`
	DelayWeight  float64 `json:"delay_weight"`
	BurdenWeight float64 `json:"burden_weight"`
	RiskWeight   float64 `json:"risk_weight"`
}

type CostComponents struct {
	Money  float64 `json:"money"`
	Delay  float64 `json:"delay"`
	Burden float64 `json:"burden"`
	Risk   float64 `json:"risk"`
}

type Likelihood struct {
	ResultID string  `json:"result_id"`
	Given    float64 `json:"probability_given_state"`
}

type Observation struct {
	ID                           string                  `json:"id"`
	Available                    bool                    `json:"available"`
	InDomain                     bool                    `json:"in_domain"`
	AdmissibleForDecision        bool                    `json:"admissible_for_decision"`
	ConditionedOnCurrentEvidence bool                    `json:"conditioned_on_current_evidence"`
	Mandatory                    bool                    `json:"mandatory"`
	Costs                        CostComponents          `json:"costs"`
	LikelihoodByState            map[string][]Likelihood `json:"likelihood_by_state"`
	AllowedActionsByResult       map[string][]string     `json:"allowed_actions_by_result"`
}

type Case struct {
	ID                         string      `json:"id"`
	States                     []State     `json:"states"`
	Actions                    []Action    `json:"actions"`
	AllowedActionsBefore       []string    `json:"allowed_actions_before"`
	CostProfile                CostProfile `json:"cost_profile"`
	MandatoryEvidenceOpen      bool        `json:"mandatory_evidence_open"`
	DecisionStillIndeterminate bool        `json:"decision_still_indeterminate"`
}

type Status string

const (
	StatusRecommend        Status = "RecommendOptionalObservation"
	StatusDoNotAcquire     Status = "DoNotAcquireOptionalObservation"
	StatusAbstain          Status = "AbstainOrEscalate"
	StatusAcquireMandatory Status = "AcquireMandatoryEvidence"
)

type Report struct {
	CaseID                     string         `json:"case_id"`
	ObservationID              string         `json:"observation_id"`
	Status                     Status         `json:"status"`
	Reason                     string         `json:"reason"`
	BeforeExpectedLoss         float64        `json:"before_expected_loss,omitempty"`
	AfterExpectedLoss          float64        `json:"after_expected_loss,omitempty"`
	EVSI                       float64        `json:"evsi,omitempty"`
	EIGBits                    float64        `json:"eig_bits,omitempty"`
	Costs                      CostComponents `json:"costs"`
	NormalizedCost             float64        `json:"normalized_cost,omitempty"`
	NetDecisionValue           float64        `json:"net_decision_value,omitempty"`
	DecisionStillIndeterminate bool           `json:"decision_still_indeterminate"`
}

func finiteNonNegative(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) && v >= 0 }

func validateCase(c Case) error {
	if c.ID == "" || len(c.States) == 0 || len(c.Actions) == 0 || len(c.AllowedActionsBefore) == 0 {
		return errors.New("case requires ID, states, actions and a non-empty baseline safe-action set")
	}
	if c.CostProfile.ID == "" {
		return errors.New("cost profile ID is required")
	}
	weights := []float64{c.CostProfile.MoneyWeight, c.CostProfile.DelayWeight, c.CostProfile.BurdenWeight, c.CostProfile.RiskWeight}
	for _, w := range weights {
		if !finiteNonNegative(w) {
			return errors.New("cost weights must be finite and non-negative")
		}
	}
	stateIDs := map[string]bool{}
	total := 0.0
	for _, s := range c.States {
		if s.ID == "" || stateIDs[s.ID] || !finiteNonNegative(s.Probability) {
			return errors.New("invalid or duplicate state")
		}
		stateIDs[s.ID] = true
		total += s.Probability
	}
	if math.Abs(total-1.0) > 1e-9 {
		return errors.New("state probabilities must sum to one")
	}
	actionIDs := map[string]bool{}
	for _, a := range c.Actions {
		if a.ID == "" || actionIDs[a.ID] {
			return errors.New("invalid or duplicate action")
		}
		actionIDs[a.ID] = true
		for _, s := range c.States {
			v, ok := a.Loss[s.ID]
			if !ok || math.IsNaN(v) || math.IsInf(v, 0) {
				return errors.New("every action needs a finite loss for every state")
			}
		}
	}
	for _, id := range c.AllowedActionsBefore {
		if !actionIDs[id] {
			return errors.New("unknown baseline safe action")
		}
	}
	return nil
}

func minExpectedLoss(states []State, actions []Action, allowed []string, posterior map[string]float64) (float64, error) {
	if len(allowed) == 0 {
		return 0, errors.New("no safe action is available for this observation result")
	}
	byID := make(map[string]Action, len(actions))
	for _, a := range actions {
		byID[a.ID] = a
	}
	best := math.Inf(1)
	for _, id := range allowed {
		a, ok := byID[id]
		if !ok {
			return 0, errors.New("observation refers to an unknown action")
		}
		loss := 0.0
		for _, s := range states {
			loss += posterior[s.ID] * a.Loss[s.ID]
		}
		if loss < best {
			best = loss
		}
	}
	return best, nil
}

func entropyBits(probabilities []float64) float64 {
	h := 0.0
	for _, p := range probabilities {
		if p > 0 {
			h -= p * math.Log2(p)
		}
	}
	return h
}

// Evaluate computes one-step EVSI and EIG for one optional observation.
// Likelihoods must already be conditioned on current admitted evidence and
// relevant common-cause dependencies. If that condition is not met, it abstains.
func Evaluate(c Case, o Observation) (Report, error) {
	r := Report{CaseID: c.ID, ObservationID: o.ID, DecisionStillIndeterminate: c.DecisionStillIndeterminate, Costs: o.Costs}
	if err := validateCase(c); err != nil {
		return r, err
	}
	if o.ID == "" {
		return r, errors.New("observation ID is required")
	}
	if c.MandatoryEvidenceOpen || o.Mandatory {
		r.Status, r.Reason = StatusAcquireMandatory, "mandatory evidence is a hard constraint and is not traded against optional EVSI"
		return r, nil
	}
	if !o.Available {
		r.Status, r.Reason = StatusAbstain, "observation is unavailable"
		return r, nil
	}
	if !o.InDomain {
		r.Status, r.Reason = StatusAbstain, "observation model is out of domain"
		return r, nil
	}
	if !o.AdmissibleForDecision {
		r.Status, r.Reason = StatusAbstain, "observation is not qualified for this decision purpose"
		return r, nil
	}
	if !o.ConditionedOnCurrentEvidence {
		r.Status, r.Reason = StatusAbstain, "observation likelihood is not conditioned on current evidence and common-cause dependencies"
		return r, nil
	}
	costValues := []float64{o.Costs.Money, o.Costs.Delay, o.Costs.Burden, o.Costs.Risk}
	for _, v := range costValues {
		if !finiteNonNegative(v) {
			return r, errors.New("cost components must be finite and non-negative")
		}
	}
	profile := c.CostProfile
	r.NormalizedCost = o.Costs.Money*profile.MoneyWeight + o.Costs.Delay*profile.DelayWeight + o.Costs.Burden*profile.BurdenWeight + o.Costs.Risk*profile.RiskWeight

	prior := make(map[string]float64, len(c.States))
	priorValues := make([]float64, 0, len(c.States))
	for _, s := range c.States {
		prior[s.ID] = s.Probability
		priorValues = append(priorValues, s.Probability)
	}
	before, err := minExpectedLoss(c.States, c.Actions, c.AllowedActionsBefore, prior)
	if err != nil {
		return r, err
	}
	r.BeforeExpectedLoss = before

	resultIDs := map[string]bool{}
	resultMass := map[string]float64{}
	for _, s := range c.States {
		likelihoods, ok := o.LikelihoodByState[s.ID]
		if !ok || len(likelihoods) == 0 {
			return r, errors.New("missing observation likelihoods for state")
		}
		sum := 0.0
		seen := map[string]bool{}
		for _, l := range likelihoods {
			if l.ResultID == "" || seen[l.ResultID] || !finiteNonNegative(l.Given) || l.Given > 1 {
				return r, errors.New("invalid or duplicate observation result likelihood")
			}
			seen[l.ResultID] = true
			resultIDs[l.ResultID] = true
			sum += l.Given
			resultMass[l.ResultID] += s.Probability * l.Given
		}
		if math.Abs(sum-1.0) > 1e-9 {
			return r, errors.New("result likelihoods for each state must sum to one")
		}
	}
	if len(o.AllowedActionsByResult) != len(resultIDs) {
		return r, errors.New("every observation result must have exactly one safe-action set")
	}
	priorEntropy := entropyBits(priorValues)
	afterLoss := 0.0
	posteriorEntropy := 0.0
	orderedResults := make([]string, 0, len(resultIDs))
	for resultID := range resultIDs {
		orderedResults = append(orderedResults, resultID)
	}
	sort.Strings(orderedResults)
	for _, resultID := range orderedResults {
		mass := resultMass[resultID]
		if mass <= 0 {
			continue
		}
		posterior := make(map[string]float64, len(c.States))
		posteriorValues := make([]float64, 0, len(c.States))
		for _, s := range c.States {
			pResult := 0.0
			for _, l := range o.LikelihoodByState[s.ID] {
				if l.ResultID == resultID {
					pResult = l.Given
					break
				}
			}
			p := s.Probability * pResult / mass
			posterior[s.ID] = p
			posteriorValues = append(posteriorValues, p)
		}
		best, err := minExpectedLoss(c.States, c.Actions, o.AllowedActionsByResult[resultID], posterior)
		if err != nil {
			r.Status, r.Reason = StatusAbstain, err.Error()
			return r, nil
		}
		afterLoss += mass * best
		posteriorEntropy += mass * entropyBits(posteriorValues)
	}
	r.AfterExpectedLoss = afterLoss
	r.EVSI = before - afterLoss
	if r.EVSI < 0 && r.EVSI > -1e-9 {
		r.EVSI = 0
	}
	r.EIGBits = priorEntropy - posteriorEntropy
	if r.EIGBits < 0 && r.EIGBits > -1e-9 {
		r.EIGBits = 0
	}
	r.NetDecisionValue = r.EVSI - r.NormalizedCost
	if r.NetDecisionValue > 1e-9 {
		r.Status, r.Reason = StatusRecommend, "positive net decision value under the declared loss and cost profiles"
	} else {
		r.Status, r.Reason = StatusDoNotAcquire, "optional observation does not have positive net decision value"
	}
	return r, nil
}
