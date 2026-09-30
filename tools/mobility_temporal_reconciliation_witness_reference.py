#!/usr/bin/env python3
"""Independent witness evaluator for temporal reconciliation."""
from dataclasses import dataclass
from enum import Enum

class C(str, Enum):
    COMPARABLE="comparable"; INCOMPARABLE="incomparable"; UNKNOWN="unknown"
class K(str, Enum):
    COMPATIBLE="compatible"; INCOMPATIBLE="incompatible"; UNKNOWN="unknown"

def overlap(a,b,c,d):
    if None in (a,b,c,d): return None
    if b < a or d < c: raise ValueError("invalid interval")
    return a <= d and c <= b

def classify(c,k,a,b,c2,d2,superseded=False,disputed=False):
    if superseded: return ("superseded",False)
    if c=="incomparable": return ("incomparable",False)
    if c=="unknown" or k=="unknown": return ("indeterminate",False)
    if k=="compatible": return ("coexistent",False)
    ov=overlap(a,b,c2,d2)
    if ov is None: return ("indeterminate",False)
    return (("conflicting" if ov else "sequential"), bool(disputed and ov))

@dataclass
class Witness:
    left_id:str
    right_id:str
    left_interval:tuple
    right_interval:tuple
    comparability:str
    compatibility:str
    superseded:bool
    disputed:bool
    classification:str
    result_disputed:bool

    def validate(self):
        if not self.left_id.strip() or not self.right_id.strip():
            raise ValueError("claim identities required")
        if self.left_id == self.right_id:
            raise ValueError("claim identities must differ")
        expected=classify(self.comparability,self.compatibility,*self.left_interval,*self.right_interval,self.superseded,self.disputed)
        if expected != (self.classification,self.result_disputed):
            raise ValueError("stored result does not reproduce from witness inputs")

def self_test():
    w=Witness("claim-a","claim-b",(0,10),(5,15),"comparable","incompatible",False,False,"conflicting",False)
    w.validate()
    bad=Witness(**{**w.__dict__,"classification":"sequential"})
    try: bad.validate()
    except ValueError: pass
    else: raise AssertionError("tampered result accepted")
    same=Witness(**{**w.__dict__,"right_id":"claim-a"})
    try: same.validate()
    except ValueError: pass
    else: raise AssertionError("same-claim witness accepted")
    disputed=Witness("claim-a","claim-b",(0,10),(5,15),"comparable","incompatible",False,True,"conflicting",True)
    disputed.validate()
    unknown=Witness("claim-a","claim-b",(0,10),(5,None),"comparable","incompatible",False,False,"indeterminate",False)
    unknown.validate()
    superseded=Witness("claim-a","claim-b",(0,10),(5,15),"comparable","incompatible",True,False,"superseded",False)
    superseded.validate()
    print("temporal reconciliation witness reference: PASS")

if __name__=="__main__":
    self_test()
