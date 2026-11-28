
package oracle
import("math";"testing")
func xorCase()Case{
 states:=[]State{{"00",.25},{"01",.25},{"10",.25},{"11",.25}}
 actions:=[]Action{{"A",map[string]float64{"00":0,"01":10,"10":10,"11":0}},{"B",map[string]float64{"00":10,"01":0,"10":0,"11":10}}}
 x:=Observation{ID:"observe-X",Cost:.3,Available:true,InDomain:true,Admissible:true,ConditionedOnLatentState:true,LikelihoodByState:map[string][]Likelihood{"00":{{"x0",1}},"01":{{"x0",1}},"10":{{"x1",1}},"11":{{"x1",1}}}}
 y:=Observation{ID:"observe-Y",Cost:.3,Available:true,InDomain:true,Admissible:true,ConditionedOnLatentState:true,LikelihoodByState:map[string][]Likelihood{"00":{{"y0",1}},"01":{{"y1",1}},"10":{{"y0",1}},"11":{{"y1",1}}}}
 return Case{ID:"XOR-complementarity",FactorizationProfile:"conditional_independence_given_complete_latent_state_v0.1",States:states,Actions:actions,SafeActions:[]string{"A","B"},Observations:[]Observation{x,y},Budget:.6,Horizon:2}
}
func near(t *testing.T,n string,v,w float64){t.Helper();if math.Abs(v-w)>1e-9{t.Fatalf("%s=%v want %v",n,v,w)}}
func TestComplementaryObservationsBeatGreedy(t *testing.T){c:=xorCase();id,net,e:=GreedyOneStep(c);if e!=nil{t.Fatal(e)};if id!=""||net!=0{t.Fatalf("greedy should stop, got %q %v",id,net)};r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=Recommend{t.Fatalf("status %s",r.Status)};near(t,"baseline",r.BaselineExpectedLoss,5);near(t,"terminal",r.ExpectedTerminalLoss,0);near(t,"cost",r.ExpectedAcquisitionCost,.6);near(t,"gross EVSI",r.GrossEVSI,5);near(t,"net",r.NetDecisionValue,4.4);if len(r.OneStepScores)!=2{t.Fatalf("one-step scores %#v",r.OneStepScores)};for _,s:=range r.OneStepScores{near(t,"one-step EVSI",s.EVSI,0)};if r.Plan.Stop||len(r.Plan.Branches)!=2{t.Fatalf("expected policy tree %#v",r.Plan)};for _,child:=range r.Plan.Branches{if child.Stop||child.ObservationID==""{t.Fatalf("second stage absent %#v",child)}}}
func TestUninformativeObservationNotAcquired(t *testing.T){c:=xorCase();for i:=range c.Observations{c.Observations[i].LikelihoodByState=map[string][]Likelihood{"00":{{"same",1}},"01":{{"same",1}},"10":{{"same",1}},"11":{{"same",1}}}};r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=DoNotAcquire{t.Fatalf("status %s",r.Status)};near(t,"EVSI",r.GrossEVSI,0);if !r.Plan.Stop{t.Fatal("must stop")}}
func TestBudgetInfeasible(t *testing.T){c:=xorCase();c.Budget=.5;r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=DoNotAcquire||!r.Plan.Stop{t.Fatalf("budget gate failed %#v",r)};near(t,"net",r.NetDecisionValue,0)}
func TestMandatoryPreempts(t *testing.T){c:=xorCase();c.MandatoryEvidenceOpen=true;r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=AcquireMandatory{t.Fatalf("status %s",r.Status)};c.MandatoryEvidenceOpen=false;c.Observations[0].Mandatory=true;r,e=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=AcquireMandatory{t.Fatalf("status %s",r.Status)}}
func TestUnknownDependenceAbstains(t *testing.T){c:=xorCase();c.FactorizationProfile="";r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=Abstain{t.Fatalf("status %s",r.Status)};near(t,"baseline",r.BaselineExpectedLoss,5);if r.Plan==nil||!r.Plan.Stop{t.Fatal("abstention must preserve stop baseline")}}
func TestInvalidObservationModelsAbstain(t *testing.T){c:=xorCase();c.Observations[0].ConditionedOnLatentState=false;c.Observations[1].InDomain=false;r,e:=Solve(c);if e!=nil{t.Fatal(e)};if r.Status!=Abstain{t.Fatalf("status %s",r.Status)}}
func TestBadLikelihoodRejected(t *testing.T){c:=xorCase();c.Observations[0].LikelihoodByState["00"]=[]Likelihood{{"x0",.4}};if _,e:=Solve(c);e==nil{t.Fatal("bad likelihood accepted")}}
