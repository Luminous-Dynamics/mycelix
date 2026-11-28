
package oracle

import (
 "errors"
 "fmt"
 "math"
 "sort"
 "strings"
)

const eps=1e-12

type State struct { ID string `+"`"+`json:"id"`+"`"+`; Probability float64 `+"`"+`json:"probability"`+"`"+` }
type Action struct { ID string `+"`"+`json:"id"`+"`"+`; LossByState map[string]float64 `+"`"+`json:"loss_by_state"`+"`"+` }
type Likelihood struct { ResultID string `+"`"+`json:"result_id"`+"`"+`; Probability float64 `+"`"+`json:"probability"`+"`"+` }
type Observation struct {
 ID string `json:"id"`
 Cost float64 `json:"cost"`
 Available bool `json:"available"`
 InDomain bool `json:"in_domain"`
 Admissible bool `json:"admissible"`
 ConditionedOnLatentState bool `json:"conditioned_on_latent_state"`
 Mandatory bool `json:"mandatory"`
 LikelihoodByState map[string][]Likelihood `json:"likelihood_by_state"`
}
type Case struct {
 ID string `json:"id"`
 FactorizationProfile string `json:"factorization_profile"`
 States []State `json:"states"`
 Actions []Action `json:"actions"`
 SafeActions []string `json:"safe_actions"`
 Observations []Observation `json:"observations"`
 Budget float64 `json:"budget"`
 Horizon int `json:"horizon"`
 MandatoryEvidenceOpen bool `json:"mandatory_evidence_open"`
}
type Status string
const (
 Recommend Status="RecommendSequentialPlan"
 DoNotAcquire Status="DoNotAcquireOptionalEvidence"
 Abstain Status="AbstainOrEscalate"
 AcquireMandatory Status="AcquireMandatoryEvidence"
)
type Node struct {
 Stop bool `json:"stop"`
 ActionID string `json:"action_id,omitempty"`
 ObservationID string `json:"observation_id,omitempty"`
 Branches map[string]*Node `json:"branches,omitempty"`
 ExpectedTotalLoss float64 `json:"expected_total_loss"`
 ExpectedTerminalLoss float64 `json:"expected_terminal_loss"`
 ExpectedAcquisitionCost float64 `json:"expected_acquisition_cost"`
}
type OneStepScore struct { ObservationID string `json:"observation_id"`; EVSI float64 `json:"evsi"`; Cost float64 `json:"cost"`; NetValue float64 `json:"net_value"` }
type Report struct {
 CaseID string `json:"case_id"`
 Status Status `json:"status"`
 BaselineExpectedLoss float64 `json:"baseline_expected_loss"`
 ExpectedTotalLoss float64 `json:"expected_total_loss"`
 ExpectedTerminalLoss float64 `json:"expected_terminal_loss"`
 ExpectedAcquisitionCost float64 `json:"expected_acquisition_cost"`
 GrossEVSI float64 `json:"gross_evsi"`
 NetDecisionValue float64 `json:"net_decision_value"`
 OneStepScores []OneStepScore `json:"one_step_scores"`
 Plan *Node `json:"plan,omitempty"`
}
type result struct { total, terminal, cost float64; node *Node }
type planner struct { c Case; obs map[string]Observation; actions map[string]Action; memo map[string]result }

func finite(v float64)bool{return !math.IsNaN(v)&&!math.IsInf(v,0)}
func validate(c Case)error{
 if c.ID==""||len(c.States)==0||len(c.Actions)==0||len(c.SafeActions)==0{return errors.New("case requires ID, states, actions and safe actions")}
 if !finite(c.Budget)||c.Budget<0||c.Horizon<0||c.Horizon>len(c.Observations){return errors.New("invalid budget or horizon")}
 states:=map[string]bool{};sum:=0.0
 for _,s:=range c.States{if s.ID==""||states[s.ID]||!finite(s.Probability)||s.Probability<0{return errors.New("invalid state or prior")};states[s.ID]=true;sum+=s.Probability}
 if math.Abs(sum-1)>1e-10{return errors.New("prior must sum to one")}
 actions:=map[string]bool{}
 for _,a:=range c.Actions{if a.ID==""||actions[a.ID]{return errors.New("empty or duplicate action")};actions[a.ID]=true;for _,s:=range c.States{v,ok:=a.LossByState[s.ID];if !ok||!finite(v)||v<0{return fmt.Errorf("action %s lacks finite nonnegative loss for %s",a.ID,s.ID)}}}
 for _,id:=range c.SafeActions{if !actions[id]{return fmt.Errorf("unknown safe action %s",id)}}
 obs:=map[string]bool{}
 for _,o:=range c.Observations{if o.ID==""||obs[o.ID]||!finite(o.Cost)||o.Cost<0{return errors.New("invalid observation ID or cost")};obs[o.ID]=true;if o.Mandatory{continue};for _,s:=range c.States{ls,ok:=o.LikelihoodByState[s.ID];if !ok||len(ls)==0{return fmt.Errorf("missing likelihoods %s/%s",o.ID,s.ID)};sm:=0.0;seen:=map[string]bool{};for _,l:=range ls{if l.ResultID==""||seen[l.ResultID]||!finite(l.Probability)||l.Probability<0||l.Probability>1{return errors.New("invalid or duplicate likelihood")};seen[l.ResultID]=true;sm+=l.Probability};if math.Abs(sm-1)>1e-10{return errors.New("likelihood distribution must sum to one")}}}
 return nil
}
func(p *planner)terminal(b []float64)(float64,string){best:=math.Inf(1);id:="";for _,aID:=range p.c.SafeActions{a:=p.actions[aID];v:=0.0;for i,s:=range p.c.States{v+=b[i]*a.LossByState[s.ID]};if v<best-eps||(math.Abs(v-best)<=eps&&(id==""||aID<id)){best,id=v,aID}};return best,id}
func bkey(b []float64)string{x:=make([]string,len(b));for i,v:=range b{x[i]=fmt.Sprintf("%016x",math.Float64bits(v))};return strings.Join(x,",")}
func(p *planner)solve(b []float64,available []string,budget float64,h int)result{
 stop,action:=p.terminal(b);best:=result{stop,stop,0,&Node{Stop:true,ActionID:action,ExpectedTotalLoss:stop,ExpectedTerminalLoss:stop}}
 if h==0||len(available)==0{return best}
 key:=bkey(b)+"|"+strings.Join(available,",")+fmt.Sprintf("|%016x|%d",math.Float64bits(budget),h);if r,ok:=p.memo[key];ok{return r}
 for _,id:=range available{o:=p.obs[id];if !o.Available||!o.InDomain||!o.Admissible||!o.ConditionedOnLatentState||o.Mandatory||o.Cost>budget+eps{continue}
  masses:=map[string]float64{};likes:=map[string][]float64{}
  for i,s:=range p.c.States{for _,l:=range o.LikelihoodByState[s.ID]{masses[l.ResultID]+=b[i]*l.Probability;if likes[l.ResultID]==nil{likes[l.ResultID]=make([]float64,len(b))};likes[l.ResultID][i]=l.Probability}}
  results:=make([]string,0,len(masses));for r,m:=range masses{if m>eps{results=append(results,r)}};sort.Strings(results)
  next:=make([]string,0,len(available)-1);for _,other:=range available{if other!=id{next=append(next,other)}}
  total,terminal,cost:=o.Cost,0.0,o.Cost;branches:=map[string]*Node{}
  for _,r:=range results{m:=masses[r];post:=make([]float64,len(b));for i:=range b{post[i]=b[i]*likes[r][i]/m};child:=p.solve(post,next,budget-o.Cost,h-1);total+=m*child.total;terminal+=m*child.terminal;cost+=m*child.cost;branches[r]=child.node}
  if total<best.total-eps{node:=&Node{ObservationID:id,Branches:branches,ExpectedTotalLoss:total,ExpectedTerminalLoss:terminal,ExpectedAcquisitionCost:cost};best=result{total,terminal,cost,node}}
 }
 p.memo[key]=best;return best
}
func scoreOne(c Case,o Observation,b []float64,base float64)(OneStepScore,bool){s:=OneStepScore{ObservationID:o.ID,Cost:o.Cost};if !o.Available||!o.InDomain||!o.Admissible||!o.ConditionedOnLatentState||o.Mandatory{return s,false};mass:=map[string]float64{};joint:=map[string][]float64{};for i,st:=range c.States{for _,l:=range o.LikelihoodByState[st.ID]{mass[l.ResultID]+=b[i]*l.Probability;if joint[l.ResultID]==nil{joint[l.ResultID]=make([]float64,len(b))};joint[l.ResultID][i]=l.Probability}};acts:=map[string]Action{};for _,a:=range c.Actions{acts[a.ID]=a};for r,m:=range mass{if m<=eps{continue};post:=make([]float64,len(b));for i:=range b{post[i]=b[i]*joint[r][i]/m};best:=math.Inf(1);for _,id:=range c.SafeActions{v:=0.0;for i,st:=range c.States{v+=post[i]*acts[id].LossByState[st.ID]};if v<best{best=v}};s.EVSI+=m*(base-best)};s.NetValue=s.EVSI-s.Cost;return s,true}
func Solve(c Case)(Report,error){
 if err:=validate(c);err!=nil{return Report{},err}
 r:=Report{CaseID:c.ID}
 if c.MandatoryEvidenceOpen{r.Status=AcquireMandatory;return r,nil}
 for _,o:=range c.Observations{if o.Mandatory{r.Status=AcquireMandatory;return r,nil}}
 p:=planner{c:c,obs:map[string]Observation{},actions:map[string]Action{},memo:map[string]result{}}
 b:=make([]float64,len(c.States));for i,s:=range c.States{b[i]=s.Probability};for _,a:=range c.Actions{p.actions[a.ID]=a}
 for _,o:=range c.Observations{p.obs[o.ID]=o}
 base,_:=p.terminal(b);r.BaselineExpectedLoss=base
 if len(c.Observations)>1&&c.FactorizationProfile!="conditional_independence_given_complete_latent_state_v0.1"{r.Status=Abstain;r.ExpectedTotalLoss=base;r.ExpectedTerminalLoss=base;r.Plan=&Node{Stop:true,ExpectedTotalLoss:base,ExpectedTerminalLoss:base};return r,nil}
 valid:=[]string{};invalid:=0;for _,o:=range c.Observations{if o.Available&&o.InDomain&&o.Admissible&&o.ConditionedOnLatentState{valid=append(valid,o.ID)}else{invalid++}};sort.Strings(valid)
 for _,o:=range c.Observations{if s,ok:=scoreOne(c,o,b,base);ok{r.OneStepScores=append(r.OneStepScores,s)}};sort.Slice(r.OneStepScores,func(i,j int)bool{return r.OneStepScores[i].ObservationID<r.OneStepScores[j].ObservationID})
 if len(valid)==0&&len(c.Observations)>0&&invalid>0{r.Status=Abstain;r.ExpectedTotalLoss=base;r.ExpectedTerminalLoss=base;r.Plan=&Node{Stop:true,ExpectedTotalLoss:base,ExpectedTerminalLoss:base};return r,nil}
 best:=p.solve(b,valid,c.Budget,c.Horizon);r.ExpectedTotalLoss=best.total;r.ExpectedTerminalLoss=best.terminal;r.ExpectedAcquisitionCost=best.cost;r.GrossEVSI=base-best.terminal;r.NetDecisionValue=base-best.total;r.Plan=best.node;if r.NetDecisionValue>eps{r.Status=Recommend}else{r.Status=DoNotAcquire};return r,nil
}
func GreedyOneStep(c Case)(string,float64,error){if err:=validate(c);err!=nil{return "",0,err};p:=planner{c:c,actions:map[string]Action{}};for _,a:=range c.Actions{p.actions[a.ID]=a};b:=make([]float64,len(c.States));for i,s:=range c.States{b[i]=s.Probability};base,_:=p.terminal(b);id:="";best:=0.0;for _,o:=range c.Observations{s,ok:=scoreOne(c,o,b,base);if ok&&s.NetValue>best+eps{id,best=o.ID,s.NetValue}};return id,best,nil
}
