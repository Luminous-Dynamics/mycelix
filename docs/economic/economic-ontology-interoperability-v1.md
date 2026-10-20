# Economic ontology interoperability map

The current public Valueflows specification makes a useful architectural comparison point: an Economic Event is an **observed economic flow**, Economic Resources represent accountable/inventoried resources, and Commitments/Intents describe future or intended flows. hREA implements Valueflows concepts and exposes Economic Events, Resources, Commitments, Intents and related modules. citeturn0search4turn0search0turn0search5

Mycelix should not replace these concepts. It should provide a **provenance + assurance + settlement boundary** around them.

| External concept | Mycelix boundary | Important non-collapse |
|---|---|---|
| Valueflows EconomicEvent | EconomicEventV1 adapter | observed event != entitlement |
| Valueflows EconomicResource | Resource reference | resource identity != local ownership claim |
| Valueflows Commitment | Obligation/Commitment adapter | commitment != settlement |
| Valueflows Intent | Offer/Intent adapter | intent != execution |
| Valueflows Claim | Entitlement/Claim adapter | claim != authorization |
| Integral COS observation | EconomicEvent adapter | production observation != ITC entitlement |
| Integral ITC | Instrument adapter | ITC != generic currency |
| Mutual credit | Instrument + settlement adapter | credit limit != asset ownership |
| Accounting entry | Settlement projection | accounting projection != physical event |

## New insight

Valueflows already separates observed events from commitments and intents. Mycelix's opportunity is therefore **not** to duplicate REA semantics. It is to make the epistemic and authority boundaries explicit across heterogeneous economic runtimes: provenance, evidence validity, recognition, authorization, settlement idempotency, dispute, correction and cross-system origin.

That makes the fabric complementary rather than competitive with an economic ontology.

## Fabric test

An adapter is conformant only if:

1. source identity survives translation;
2. source origin survives translation;
3. unit identity survives translation;
4. validity survives translation;
5. evidence references survive translation;
6. semantic type does not silently widen;
7. authorization is explicit;
8. settlement is separately observable;
9. corrections preserve lineage;
10. foreign recognition does not become local issuance.

