// Package oracle evaluates acquisition across a declared finite ambiguity set.
// It is a test-only reference; it does not calibrate models or authorize effects.
package oracle

import (
 "errors"
 "fmt"
 "math"
 "sort"
)

const epsilon=1e-12

type State struct { ID string `json:"id"`; Probability float64 `json:"probability"` }
type Action struct { ID string `json:"id"`; Loss map[string]float64 `json:"loss_by_state"` }
type Likelihood struct { ResultID string `json:"result_id"`; Given float64 `json:"probability"` }
type ModelScenario struct { ID string `json:"id"`; States []State `json:"states"`; LikelihoodByState map[string][]Likelihood `json:"likelihood_by_state"` }
type Case struct {
 ID string `json:"id"`
 ModelSetID string `json:"model_set_id"`
 ModelSetAdmissible bool `json:"model_set_admissible"`
 CoverageEstablished bool `json:"coverage_established"`
 SafeActions []string `json:"safe_actions"`
 Actions []Action `json:"actions"`
 Models []ModelScenario `json:"models"`
 ObservationCost float64 `json:"observation_cost"`
 ObservationAvailable bool `json:"observation_available"`
 ObservationInDomain bool `json:"observation_in_domain"`
 ObservationAdmissible bool `json:"observation_admissible"`
 MandatoryEvidenceOpen bool `json:"mandatory_evidence_open"`
}
type Status string
const (
 Recommend Status="RecommendRobustObservation"
 DoNotAcquire Status="DoNotAcquireRobustObservation"
 Abstain Status="AbstainOrEscalate"
 AcquireMandatory Status="AcquireMandatoryEvidence"
)
type ModelReport struct {
 ModelID string `json:"model_id"`
 BaselineLoss float64 `json:"baseline_loss"`
 TerminalLoss float64 `json:"terminal_loss"`
 GrossEVSI float64 `json:"gross_evsi"`
 NetEVSI float64 `json:"net_evsi"`
}
type Report struct {
 CaseID string `json:"case_id"`
 Status Status `json:"status"`
 ModelSetID string `json:"model_set_id"`
 BaselineRobustRisk float64 `json:"baseline_robust_risk"`
 TerminalWorstCaseRisk float64 `json:"terminal_worst_case_risk"`
 AcquisitionRobustRisk float64 `json:"acquisition_robust_risk"`
 RobustGrossEVSI float64 `json:"robust_gross_evsi"`
 RobustNetValue float64 `json:"robust_net_value"`
 SelectedPolicy map[string]string `json:"selected_policy,omitempty"`
 NominalPerModel []ModelReport `json:"nominal_per_model"`
 WorstCaseModelID string `json:"worst_case_model_id,omitempty"`
 ModelSetCoverageClaim string `json:"model_set_coverage_claim"`
 RealWorldCalibration string `json:"real_world_calibration"`
}
type policy struct { mapping map[string]string; key string }
func finite(x float64)bool{return !math.IsNaN(x)&&!math.IsInf(x,0)}
func nonnegative(x float64)bool{return finite(x)&&x>=0}

func validate(c Case)([]string,error){
 if c.ID==""||c.ModelSetID==""||len(c.Models)==0||len(c.Actions)==0||len(c.SafeActions)==0{return nil,errors.New("case requires model set, models, actions and safe actions")}
 if !nonnegative(c.ObservationCost){return nil,errors.New("invalid observation cost")}
 actions:=map[string]Action{}
 for _,a:=range c.Actions{if a.ID==""{return nil,errors.New("empty action ID")};if _,ok:=actions[a.ID];ok{return nil,fmt.Errorf("duplicate action %s",a.ID)};actions[a.ID]=a}
 for _,id:=range c.SafeActions{if _,ok:=actions[id];!ok{return nil,fmt.Errorf("unknown safe action %s",id)}}
 models:=map[string]bool{};stateUniverse:=map[string]bool{};results:=map[string]bool{}
 for i,m:=range c.Models{
  if m.ID==""||models[m.ID]{return nil,errors.New("empty or duplicate model ID")};models[m.ID]=true
  if len(m.States)==0{return nil,fmt.Errorf("model %s has no states",m.ID)}
  states:=map[string]bool{};sum:=0.0
  for _,s:=range m.States{
   if s.ID==""||states[s.ID]||!nonnegative(s.Probability){return nil,fmt.Errorf("invalid state in model %s",m.ID)}
   states[s.ID]=true;sum+=s.Probability
   if i==0{stateUniverse[s.ID]=true}else if !stateUniverse[s.ID]{return nil,fmt.Errorf("unknown state %s in model %s",s.ID,m.ID)}
   for _,a:=range c.Actions{v,ok:=a.Loss[s.ID];if !ok||!finite(v){return nil,fmt.Errorf("action %s lacks finite loss for state %s",a.ID,s.ID)}}
  }
  if math.Abs(sum-1)>1e-10{return nil,fmt.Errorf("model %s prior must sum to one",m.ID)}
  if len(states)!=len(stateUniverse){return nil,fmt.Errorf("model %s has different state universe",m.ID)}
  if len(m.LikelihoodByState)!=len(states){return nil,fmt.Errorf("model %s likelihood state keys mismatch",m.ID)}
  for sID:=range m.LikelihoodByState{if !states[sID]{return nil,fmt.Errorf("unknown likelihood state %s",sID)}}
  for _,s:=range m.States{
   ls,ok:=m.LikelihoodByState[s.ID];if !ok||len(ls)==0{return nil,fmt.Errorf("missing likelihoods for %s/%s",m.ID,s.ID)}
   lsSum:=0.0;seen:=map[string]bool{}
   for _,l:=range ls{if l.ResultID==""||seen[l.ResultID]||!nonnegative(l.Given)||l.Given>1{return nil,errors.New("invalid or duplicate observation likelihood")};seen[l.ResultID]=true;results[l.ResultID]=true;lsSum+=l.Given}
   if math.Abs(lsSum-1)>1e-10{return nil,fmt.Errorf("likelihoods for %s/%s do not sum to one",m.ID,s.ID)}
  }
 }
 out:=make([]string,0,len(results));for id:=range results{out=append(out,id)};sort.Strings(out)
 count:=1;for range out{if count>100000/len(c.SafeActions){return nil,errors.New("policy space exceeds 100000 deterministic policies")};count*=len(c.SafeActions)}
 return out,nil
}
func baselineForAction(c Case,m ModelScenario,id string)float64{
 loss:=0.0
 for _,s:=range m.States{for _,a:=range c.Actions{if a.ID==id{loss+=s.Probability*a.Loss[s.ID];break}}}
 return loss
}
func baselineRisk(c Case)(float64,string){
 best:=math.Inf(1);actionID:=""
 for _,id:=range c.SafeActions{worst:=0.0;for _,m:=range c.Models{v:=baselineForAction(c,m,id);if v>worst{worst=v}}
  if worst<best-epsilon||(math.Abs(worst-best)<=epsilon&&(actionID==""||id<actionID)){best,actionID=worst,id}
 }
 return best,actionID
}
func enumeratePolicies(results,actions []string)[]policy{
 out:=[]policy{};m:=map[string]string{}
 var visit func(int)
 visit=func(i int){if i==len(results){copyMap:=map[string]string{};parts:=make([]string,len(results));for j,r:=range results{copyMap[r]=m[r];parts[j]=r+"="+m[r]};out=append(out,policy{copyMap,fmt.Sprint(parts)});return};for _,a:=range actions{m[results[i]]=a;visit(i+1)}}
 visit(0);return out
}
func terminalRisk(c Case,m ModelScenario,p policy)float64{
 risk:=0.0
 for _,s:=range m.States{for _,l:=range m.LikelihoodByState[s.ID]{aid:=p.mapping[l.ResultID];for _,a:=range c.Actions{if a.ID==aid{risk+=s.Probability*l.Given*a.Loss[s.ID];break}}}}
 return risk
}
func nominalReport(c Case,m ModelScenario,results []string,policies []policy)ModelReport{
 r:=ModelReport{ModelID:m.ID};r.BaselineLoss=math.Inf(1)
 for _,a:=range c.SafeActions{v:=baselineForAction(c,m,a);if v<r.BaselineLoss{r.BaselineLoss=v}}
 r.TerminalLoss=math.Inf(1)
 for _,p:=range policies{v:=terminalRisk(c,m,p);if v<r.TerminalLoss{r.TerminalLoss=v}}
 r.GrossEVSI=r.BaselineLoss-r.TerminalLoss;r.NetEVSI=r.GrossEVSI-c.ObservationCost
 return r
}
func Evaluate(c Case)(Report,error){
 r:=Report{CaseID:c.ID,ModelSetID:c.ModelSetID,ModelSetCoverageClaim:"NotEvaluated",RealWorldCalibration:"NotEvaluated"}
 if c.MandatoryEvidenceOpen{r.Status=AcquireMandatory;return r,nil}
 if !c.ModelSetAdmissible||!c.CoverageEstablished||!c.ObservationAvailable||!c.ObservationInDomain||!c.ObservationAdmissible{r.Status=Abstain;return r,nil}
 results,err:=validate(c);if err!=nil{return r,err};if len(results)==0{return r,errors.New("no observation result labels")}
 r.ModelSetCoverageClaim="EstablishedWithinDeclaredFiniteSet"
 baseline,_:=baselineRisk(c);r.BaselineRobustRisk=baseline
 safe:=append([]string(nil),c.SafeActions...);sort.Strings(safe)
 policies:=enumeratePolicies(results,safe)
 for _,m:=range c.Models{r.NominalPerModel=append(r.NominalPerModel,nominalReport(c,m,results,policies))}
 best:=math.Inf(1);var selected policy;worstModel:=""
 for _,p:=range policies{
  worst:=math.Inf(-1);modelID:=""
  for _,m:=range c.Models{v:=terminalRisk(c,m,p);if v>worst+epsilon||(math.Abs(v-worst)<=epsilon&&(modelID==""||m.ID<modelID)){worst,modelID=v,m.ID}}
  if worst<best-epsilon||(math.Abs(worst-best)<=epsilon&&(selected.key==""||p.key<selected.key)){best,selected,worstModel=worst,p,modelID}
 }
 r.TerminalWorstCaseRisk=best;r.AcquisitionRobustRisk=best+c.ObservationCost;r.RobustGrossEVSI=baseline-best;r.RobustNetValue=baseline-r.AcquisitionRobustRisk;r.SelectedPolicy=selected.mapping;r.WorstCaseModelID=worstModel
 if r.RobustNetValue>epsilon{r.Status=Recommend}else{r.Status=DoNotAcquire}
 return r,nil
}
