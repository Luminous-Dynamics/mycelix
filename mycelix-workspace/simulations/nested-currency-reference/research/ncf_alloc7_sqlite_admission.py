#!/usr/bin/env python3
"""NCF-ALLOC-7: local SQLite transactional admission research baseline.

This file-backed WAL model exercises actual SQLite transactions and unique indexes.
It is research code, not a Rust/Holochain production implementation or crypto verifier.
"""
from __future__ import annotations
import argparse
from contextlib import contextmanager
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
import hashlib, json, platform, sqlite3, statistics, tempfile, threading, time
from pathlib import Path
import unittest

class Decision(str, Enum):
    ACCEPTED="accepted"; REPLAY="idempotent_replay"; REJECTED="rejected"

@dataclass(frozen=True)
class Claim:
    request_id:str; principal_id:str; instrument_id:str; amount:int; payload_digest:str
    verifier_version:str="test-verifier-v1"; verifier_receipt_digest:str="a"*64
    epoch:int=1; verifier_ok:bool=True

@dataclass(frozen=True)
class Result:
    decision:Decision; reason:str; receipt:dict|None=None

def make_claim(request_id, amount=60, digest_payload=None, principal="principal-A",
               instrument="instrument-1", verifier_version="test-verifier-v1",
               verifier_receipt_digest="a"*64, epoch=1, verifier_ok=True):
    payload=digest_payload if digest_payload is not None else "work:"+request_id
    return Claim(request_id,principal,instrument,amount,hashlib.sha256(payload.encode()).hexdigest(),
                 verifier_version,verifier_receipt_digest,epoch,verifier_ok)

class SQLiteAdmission:
    """One BEGIN IMMEDIATE transaction binds dedup, cap debit, accepted row and receipt."""
    def __init__(self, db_path):
        self.db_path=db_path
        with self.connect() as c:
            c.execute("PRAGMA journal_mode=WAL")
            c.execute("PRAGMA synchronous=FULL")
            c.executescript("""
                CREATE TABLE IF NOT EXISTS principal_caps(
                  principal_id TEXT NOT NULL, instrument_id TEXT NOT NULL,
                  cap_units INTEGER NOT NULL CHECK(cap_units>=0),
                  used_units INTEGER NOT NULL DEFAULT 0 CHECK(used_units>=0 AND used_units<=cap_units),
                  PRIMARY KEY(principal_id,instrument_id));
                CREATE TABLE IF NOT EXISTS canonical_work(
                  payload_digest TEXT PRIMARY KEY, principal_id TEXT NOT NULL,
                  instrument_id TEXT NOT NULL, amount_units INTEGER NOT NULL CHECK(amount_units>0),
                  canonical_request_id TEXT NOT NULL, receipt_json TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS request_keys(
                  request_id TEXT PRIMARY KEY,
                  payload_digest TEXT NOT NULL REFERENCES canonical_work(payload_digest),
                  principal_id TEXT NOT NULL, instrument_id TEXT NOT NULL,
                  amount_units INTEGER NOT NULL CHECK(amount_units>0),
                  verifier_version TEXT NOT NULL, verifier_receipt_digest TEXT NOT NULL,
                  receipt_json TEXT NOT NULL);
                CREATE INDEX IF NOT EXISTS request_keys_by_payload ON request_keys(payload_digest);
            """)

    @contextmanager
    def connect(self):
        c=sqlite3.connect(self.db_path,timeout=10.0,isolation_level=None)
        try:
            c.execute("PRAGMA foreign_keys=ON")
            c.execute("PRAGMA synchronous=FULL")
            yield c
        finally:
            c.close()

    def set_cap(self,principal,instrument,cap):
        if not principal.strip() or not instrument.strip() or cap<0:
            raise ValueError("invalid cap identity or amount")
        with self.connect() as c:
            c.execute("BEGIN IMMEDIATE")
            c.execute("""INSERT INTO principal_caps(principal_id,instrument_id,cap_units,used_units)
              VALUES(?,?,?,0) ON CONFLICT(principal_id,instrument_id)
              DO UPDATE SET cap_units=excluded.cap_units WHERE used_units<=excluded.cap_units""",
              (principal,instrument,cap))
            row=c.execute("SELECT cap_units FROM principal_caps WHERE principal_id=? AND instrument_id=?",
                          (principal,instrument)).fetchone()
            if row is None or row[0]!=cap:
                c.rollback(); raise ValueError("cannot lower cap below already used units")
            c.commit()

    @staticmethod
    def validate(c):
        if not all(x.strip() for x in (c.request_id,c.principal_id,c.instrument_id,c.verifier_version)):
            return "empty_identity_or_verifier_version"
        if c.amount<=0 or c.epoch<0: return "invalid_amount_or_epoch"
        if len(c.payload_digest)!=64 or any(x not in "0123456789abcdef" for x in c.payload_digest):
            return "invalid_payload_digest"
        if len(c.verifier_receipt_digest)!=64 or any(x not in "0123456789abcdef" for x in c.verifier_receipt_digest):
            return "invalid_verifier_receipt_digest"
        if not c.verifier_ok: return "verifier_did_not_accept_evidence"
        if c.amount>(2**63-1): return "amount_exceeds_sqlite_integer_domain"
        return None

    def admit(self,c):
        invalid=self.validate(c)
        if invalid: return Result(Decision.REJECTED,invalid)
        try:
            with self.connect() as db:
                db.execute("BEGIN IMMEDIATE")
                prior=db.execute("""SELECT payload_digest,principal_id,instrument_id,amount_units,
                    verifier_version,verifier_receipt_digest,receipt_json
                    FROM request_keys WHERE request_id=?""",(c.request_id,)).fetchone()
                if prior is not None:
                    same=prior[:6]==(c.payload_digest,c.principal_id,c.instrument_id,c.amount,
                                     c.verifier_version,c.verifier_receipt_digest)
                    db.commit()
                    return (Result(Decision.REPLAY,"same_request_id_and_evidence",json.loads(prior[6]))
                            if same else Result(Decision.REJECTED,"request_id_payload_or_verifier_conflict",json.loads(prior[6])))
                canonical=db.execute("""SELECT principal_id,instrument_id,amount_units,
                    canonical_request_id,receipt_json FROM canonical_work WHERE payload_digest=?""",
                    (c.payload_digest,)).fetchone()
                if canonical is not None:
                    if canonical[:3]!=(c.principal_id,c.instrument_id,c.amount):
                        db.rollback()
                        return Result(Decision.REJECTED,"canonical_work_identity_or_amount_conflict",json.loads(canonical[4]))
                    db.execute("INSERT INTO request_keys VALUES(?,?,?,?,?,?,?,?)",
                      (c.request_id,c.payload_digest,c.principal_id,c.instrument_id,c.amount,
                       c.verifier_version,c.verifier_receipt_digest,canonical[4]))
                    db.commit()
                    return Result(Decision.REPLAY,"canonical_work_alias",json.loads(canonical[4]))
                cap_row=db.execute("""SELECT cap_units,used_units FROM principal_caps
                    WHERE principal_id=? AND instrument_id=?""",(c.principal_id,c.instrument_id)).fetchone()
                if cap_row is None:
                    db.rollback(); return Result(Decision.REJECTED,"missing_principal_instrument_cap")
                cap,used=cap_row; next_used=used+c.amount
                if next_used>cap:
                    db.rollback(); return Result(Decision.REJECTED,"principal_instrument_cap_exceeded")
                receipt={"request_id":c.request_id,"principal_id":c.principal_id,
                         "instrument_id":c.instrument_id,"amount_units":c.amount,
                         "payload_digest":c.payload_digest,"original_epoch":c.epoch,
                         "verifier_version":c.verifier_version,
                         "verifier_receipt_digest":c.verifier_receipt_digest}
                encoded=json.dumps(receipt,sort_keys=True,separators=(",",":"))
                db.execute("INSERT INTO canonical_work VALUES(?,?,?,?,?,?)",
                  (c.payload_digest,c.principal_id,c.instrument_id,c.amount,c.request_id,encoded))
                db.execute("INSERT INTO request_keys VALUES(?,?,?,?,?,?,?,?)",
                  (c.request_id,c.payload_digest,c.principal_id,c.instrument_id,c.amount,
                   c.verifier_version,c.verifier_receipt_digest,encoded))
                db.execute("UPDATE principal_caps SET used_units=? WHERE principal_id=? AND instrument_id=?",
                           (next_used,c.principal_id,c.instrument_id))
                db.commit()
                return Result(Decision.ACCEPTED,"accepted_atomically",receipt)
        except sqlite3.IntegrityError as exc:
            return Result(Decision.REJECTED,"storage_constraint_rejected:"+str(exc))

    def inspect(self,principal="principal-A",instrument="instrument-1"):
        with self.connect() as c:
            cap=c.execute("SELECT cap_units,used_units FROM principal_caps WHERE principal_id=? AND instrument_id=?",
                          (principal,instrument)).fetchone()
            unique=c.execute("SELECT COUNT(*),COALESCE(SUM(amount_units),0) FROM canonical_work WHERE principal_id=? AND instrument_id=?",
                             (principal,instrument)).fetchone()
            requests=c.execute("SELECT COUNT(*) FROM request_keys WHERE principal_id=? AND instrument_id=?",
                               (principal,instrument)).fetchone()[0]
            return {"cap_units":cap[0] if cap else None,"used_units":cap[1] if cap else 0,
                    "unique_obligations":unique[0],"unique_obligation_units":unique[1],
                    "request_ids_including_aliases":requests}

    def reset_for_test(self):
        with self.connect() as c:
            c.execute("BEGIN IMMEDIATE")
            c.execute("DELETE FROM request_keys"); c.execute("DELETE FROM canonical_work")
            c.execute("UPDATE principal_caps SET used_units=0")
            c.commit()

def concurrent_pair(db,ca,cb):
    gate=threading.Barrier(2)
    def worker(claim):
        gate.wait(timeout=5)
        return db.admit(claim)
    with ThreadPoolExecutor(max_workers=2) as pool:
        fa=pool.submit(worker,ca); fb=pool.submit(worker,cb)
        return fa.result(timeout=15),fb.result(timeout=15)

class SQLiteAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.db=SQLiteAdmission(str(Path(self.tmp.name)/"admission.sqlite"))
        self.db.set_cap("principal-A","instrument-1",100)
    def tearDown(self): self.tmp.cleanup()

    def test_concurrent_distinct_obligations_never_overshoot_cap(self):
        for i in range(50):
            self.db.reset_for_test()
            a=make_claim(f"a-{i}",60,f"unique-a-{i}"); b=make_claim(f"b-{i}",60,f"unique-b-{i}")
            results=concurrent_pair(self.db,a,b); snap=self.db.inspect()
            self.assertEqual(sum(r.decision==Decision.ACCEPTED for r in results),1)
            self.assertEqual(sum(r.reason=="principal_instrument_cap_exceeded" for r in results),1)
            self.assertEqual(snap["used_units"],60); self.assertLessEqual(snap["used_units"],snap["cap_units"])
            self.assertEqual(snap["unique_obligations"],1)

    def test_concurrent_same_work_under_different_request_ids_debits_once(self):
        for i in range(50):
            self.db.reset_for_test()
            a=make_claim(f"a-{i}",60,f"shared-{i}")
            b=make_claim(f"b-{i}",60,f"shared-{i}",verifier_version="verifier-v2",verifier_receipt_digest="b"*64)
            results=concurrent_pair(self.db,a,b); snap=self.db.inspect()
            self.assertEqual(sum(r.decision==Decision.ACCEPTED for r in results),1)
            self.assertEqual(sum(r.decision==Decision.REPLAY for r in results),1)
            self.assertEqual(snap["used_units"],60); self.assertEqual(snap["unique_obligations"],1)
            self.assertEqual(snap["request_ids_including_aliases"],2)

    def test_concurrent_same_request_id_conflicting_payloads_binds_one_payload(self):
        for i in range(50):
            self.db.reset_for_test()
            a=make_claim(f"shared-{i}",40,f"conflict-a-{i}"); b=make_claim(f"shared-{i}",40,f"conflict-b-{i}")
            results=concurrent_pair(self.db,a,b); snap=self.db.inspect()
            self.assertEqual(sum(r.decision==Decision.ACCEPTED for r in results),1)
            self.assertEqual(sum(r.decision==Decision.REJECTED for r in results),1)
            self.assertEqual(snap["used_units"],40); self.assertEqual(snap["unique_obligations"],1)
            self.assertEqual(snap["request_ids_including_aliases"],1)

    def test_commit_then_lost_ack_retry_returns_exact_durable_receipt(self):
        claim=make_claim("lost-ack",40,"durable-work")
        first=self.db.admit(claim); restarted=SQLiteAdmission(self.db.db_path); replay=restarted.admit(claim)
        self.assertEqual(first.decision,Decision.ACCEPTED); self.assertEqual(replay.decision,Decision.REPLAY)
        self.assertEqual(replay.receipt,first.receipt); self.assertEqual(restarted.inspect()["used_units"],40)

    def test_verifier_evidence_binds_request_id_but_alias_returns_canonical_receipt(self):
        first=self.db.admit(make_claim("original",40,"same-work"))
        alias=self.db.admit(make_claim("alias",40,"same-work",verifier_version="v2",verifier_receipt_digest="b"*64))
        self.assertEqual(first.decision,Decision.ACCEPTED); self.assertEqual(alias.decision,Decision.REPLAY)
        self.assertEqual(alias.receipt,first.receipt)
        changed=self.db.admit(make_claim("alias",40,"same-work",verifier_version="v3",verifier_receipt_digest="c"*64))
        self.assertEqual(changed.decision,Decision.REJECTED); self.assertEqual(self.db.inspect()["used_units"],40)

    def test_missing_cap_unverified_evidence_and_zero_amount_fail_closed(self):
        self.assertEqual(self.db.admit(make_claim("missing",5,"missing",principal="other")).reason,
                         "missing_principal_instrument_cap")
        self.assertEqual(self.db.admit(make_claim("bad",5,"bad",verifier_ok=False)).decision,Decision.REJECTED)
        self.assertEqual(self.db.admit(make_claim("zero",0,"zero")).decision,Decision.REJECTED)
        self.assertEqual(self.db.inspect()["used_units"],0)

    def test_liability_counter_matches_sum_of_canonical_obligations(self):
        self.db.admit(make_claim("one",20,"work-one")); self.db.admit(make_claim("two",30,"work-two"))
        self.db.admit(make_claim("alias-two",30,"work-two"))
        snap=self.db.inspect()
        self.assertEqual(snap["used_units"],snap["unique_obligation_units"])
        self.assertEqual(snap["unique_obligations"],2); self.assertEqual(snap["request_ids_including_aliases"],3)

def run_frontier(trials=50):
    results={}
    with tempfile.TemporaryDirectory() as td:
        db=SQLiteAdmission(str(Path(td)/"races.sqlite")); db.set_cap("principal-A","instrument-1",100)
        for name in ("distinct_work_cap_100","same_work_distinct_ids","same_id_conflicting_payloads"):
            outcomes={"accepted":0,"replay":0,"rejected":0,"cap_rejected":0,"invariant_failures":0}
            for i in range(trials):
                db.reset_for_test()
                if name=="distinct_work_cap_100":
                    a=make_claim(f"a-{i}",60,f"distinct-a-{i}"); b=make_claim(f"b-{i}",60,f"distinct-b-{i}")
                    exp_used,exp_unique,exp_requests=60,1,1
                elif name=="same_work_distinct_ids":
                    a=make_claim(f"a-{i}",60,f"shared-{i}"); b=make_claim(f"b-{i}",60,f"shared-{i}",verifier_version="v2",verifier_receipt_digest="b"*64)
                    exp_used,exp_unique,exp_requests=60,1,2
                else:
                    a=make_claim(f"shared-{i}",40,f"conflict-a-{i}"); b=make_claim(f"shared-{i}",40,f"conflict-b-{i}")
                    exp_used,exp_unique,exp_requests=40,1,1
                for r in concurrent_pair(db,a,b):
                    outcomes[r.decision.value]+=1
                    if r.reason=="principal_instrument_cap_exceeded": outcomes["cap_rejected"]+=1
                snap=db.inspect()
                if (snap["used_units"]!=exp_used or snap["unique_obligations"]!=exp_unique
                    or snap["request_ids_including_aliases"]!=exp_requests or snap["used_units"]>snap["cap_units"]):
                    outcomes["invariant_failures"]+=1
            results[name]={"trials":trials,"request_outcomes":outcomes,"final_state":db.inspect()}
    return results

def run_benchmark(operations=2000):
    with tempfile.TemporaryDirectory() as td:
        db=SQLiteAdmission(str(Path(td)/"bench.sqlite")); db.set_cap("principal-A","instrument-1",operations+1)
        latencies=[]; accepted=replayed=0; start=time.perf_counter()
        for i in range(operations):
            idx=i//2; claim=make_claim(f"bench-{idx}",1,f"bench-work-{idx}")
            t0=time.perf_counter_ns(); r=db.admit(claim); latencies.append((time.perf_counter_ns()-t0)/1e6)
            accepted+=r.decision==Decision.ACCEPTED; replayed+=r.decision==Decision.REPLAY
        elapsed=time.perf_counter()-start; ordered=sorted(latencies)
        pct=lambda p: ordered[min(len(ordered)-1,int((len(ordered)-1)*p))]
        return {"operations_attempted":operations,"accepted_unique":accepted,"exact_replays":replayed,
          "elapsed_seconds":round(elapsed,6),"attempts_per_second":round(operations/elapsed,2),
          "latency_ms":{"p50":round(statistics.median(ordered),4),"p95":round(pct(.95),4),"p99":round(pct(.99),4)},
          "snapshot":db.inspect(),"claim_ceiling":"Local SQLite WAL/synchronous-FULL two-transaction-per-new-work/replay throughput; not networked settlement TPS."}

def main():
    p=argparse.ArgumentParser();p.add_argument("--benchmark-operations",type=int,default=2000);args=p.parse_args()
    source_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result={"schema":"ncf-alloc7-sqlite-transactional-admission-v1",
      "status":"LOCAL_FILE_BACKED_SQLITE_RESEARCH_NOT_RUST_NOT_HOLOCHAIN_NOT_PRODUCTION",
      "source_sha256":source_sha,
      "profile":{"db":"SQLite file-backed WAL; synchronous FULL; BEGIN IMMEDIATE","concurrency_trials_per_race":50,
        "races":["distinct 60+60 at cap 100","same work distinct IDs","same ID conflicting payloads"],
        "python_version":platform.python_version(),"sqlite_version":sqlite3.sqlite_version,"platform":platform.platform()},
      "concurrency_frontier":run_frontier(50),"benchmark":run_benchmark(args.benchmark_operations),
      "claim_ceiling":"Tests exercise real local SQLite connections, transactions and unique indexes. They do not test multiple hosts, Holochain, cryptographic verification, or a Mycelix production path."}
    print(json.dumps(result,indent=2,sort_keys=True))

if __name__=="__main__": main()
