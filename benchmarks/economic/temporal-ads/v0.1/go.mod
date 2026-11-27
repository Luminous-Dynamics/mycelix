module mycelix-temporal-ads-corpus

go 1.23

require (
	mycelix-temporal-hindsight-oracle v0.0.0
	mycelix-temporal-scope-closure v0.0.0
)

replace mycelix-temporal-hindsight-oracle => ../../../../tools/economic/temporal-hindsight/reference/go
replace mycelix-temporal-scope-closure => ../../../../tools/economic/temporal-hindsight/reference/scopeclosure
