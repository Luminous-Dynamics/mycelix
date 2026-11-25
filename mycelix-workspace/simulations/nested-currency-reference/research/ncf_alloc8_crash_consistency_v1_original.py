#!/usr/bin/env python3
"""NCF-ALLOC-8: process-crash recovery and independent-process admission race study.

Research harness only. It exercises SQLite WAL on one local host. os._exit simulates
an abrupt application-process exit, NOT kernel panic, power loss, faulty storage,
network partition, multi-host consensus, or Holochain multi-agent behavior.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass, asdict
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import statistics
import subprocess
import sys
import tempfile
import time
import unittest

CRASH_EXIT = 86
PRECOMMIT_STAGES = ("after_begin", "after_canonical_insert", "after_request_insert", "after_cap_update", "before_commit")
ALL_CRASH_STAGES = PRECOMMIT_STAGES + ("after_commit_before_ack",)

@dataclass(frozen=True)
class Claim:
    request_id: str
    payload_digest: str
    principal_id: str = "principal-A"
    instrument_id: str = "instrument-1"
    amount: int = 60


def digest(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def init_db(db_path: str, cap: int = 100) -> None:
    Path(db_path).parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(db_path, timeout=10, isolation_level=None) as conn:
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=FULL")
        conn.execute("PRAGMA foreign_keys=ON")
        conn.executescript("""
        CREATE TABLE IF NOT EXISTS principal_caps (
          principal_id TEXT NOT NULL, instrument_id TEXT NOT NULL,
          cap_units INTEGER NOT NULL CHECK(cap_units >= 0),
          used_units INTEGER NOT NULL DEFAULT 0 CHECK(used_units >= 0 AND used_units <= cap_units),
          PRIMARY KEY(principal_id, instrument_id));
        CREATE TABLE IF NOT EXISTS canonical_work (
          payload_digest TEXT PRIMARY KEY, principal_id TEXT NOT NULL,
          instrument_id TEXT NOT NULL, amount_units INTEGER NOT NULL CHECK(amount_units > 0),
          canonical_request_id TEXT NOT NULL, receipt_json TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS request_keys (
          request_id TEXT PRIMARY KEY, payload_digest TEXT NOT NULL REFERENCES canonical_work(payload_digest),
          principal_id TEXT NOT NULL, instrument_id TEXT NOT NULL, amount_units INTEGER NOT NULL CHECK(amount_units > 0),
          receipt_json TEXT NOT NULL);
        """)
        conn.execute("BEGIN IMMEDIATE")
        conn.execute("INSERT INTO principal_caps(principal_id,instrument_id,cap_units,used_units) VALUES(?,?,?,0) ON CONFLICT(principal_id,instrument_id) DO UPDATE SET cap_units=excluded.cap_units WHERE used_units=0", ("principal-A", "instrument-1", cap))
        row = conn.execute("SELECT cap_units FROM principal_caps WHERE principal_id=? AND instrument_id=?", ("principal-A", "instrument-1")).fetchone()
        if row is None or row[0] != cap:
            conn.rollback()
            raise ValueError("existing cap is non-empty or differs")
        conn.commit()


def maybe_crash(requested: str | None, stage: str) -> None:
    if requested == stage:
        # Deliberately bypass Python cleanup/SQLite close hooks to model abrupt process death.
        os._exit(CRASH_EXIT)


def admit_once(db_path: str, c: Claim, crash_stage: str | None = None) -> dict:
    with sqlite3.connect(db_path, timeout=15, isolation_level=None) as conn:
        conn.execute("PRAGMA synchronous=FULL")
        conn.execute("PRAGMA foreign_keys=ON")
        conn.execute("BEGIN IMMEDIATE")
        maybe_crash(crash_stage, "after_begin")
        prior = conn.execute("SELECT payload_digest,principal_id,instrument_id,amount_units,receipt_json FROM request_keys WHERE request_id=?", (c.request_id,)).fetchone()
        if prior is not None:
            if prior[:4] == (c.payload_digest,c.principal_id,c.instrument_id,c.amount):
                conn.commit()
                return {"decision":"replay","receipt":json.loads(prior[4])}
            conn.rollback()
            return {"decision":"rejected","reason":"request_id_conflict"}
        canonical = conn.execute("SELECT principal_id,instrument_id,amount_units,receipt_json FROM canonical_work WHERE payload_digest=?", (c.payload_digest,)).fetchone()
        if canonical is not None:
            if canonical[:3] != (c.principal_id,c.instrument_id,c.amount):
                conn.rollback()
                return {"decision":"rejected","reason":"canonical_work_conflict"}
            encoded = canonical[3]
            conn.execute("INSERT INTO request_keys VALUES(?,?,?,?,?,?)", (c.request_id,c.payload_digest,c.principal_id,c.instrument_id,c.amount,encoded))
            conn.commit()
            return {"decision":"replay","receipt":json.loads(encoded)}
        cap = conn.execute("SELECT cap_units,used_units FROM principal_caps WHERE principal_id=? AND instrument_id=?", (c.principal_id,c.instrument_id)).fetchone()
        if cap is None:
            conn.rollback()
            return {"decision":"rejected","reason":"missing_cap"}
        if cap[1] + c.amount > cap[0]:
            conn.rollback()
            return {"decision":"rejected","reason":"cap_exceeded"}
        receipt = {"request_id":c.request_id,"payload_digest":c.payload_digest,"principal_id":c.principal_id,"instrument_id":c.instrument_id,"amount_units":c.amount}
        encoded = json.dumps(receipt, sort_keys=True, separators=(",", ":"))
        conn.execute("INSERT INTO canonical_work VALUES(?,?,?,?,?,?)", (c.payload_digest,c.principal_id,c.instrument_id,c.amount,c.request_id,encoded))
        maybe_crash(crash_stage, "after_canonical_insert")
        conn.execute("INSERT INTO request_keys VALUES(?,?,?,?,?,?)", (c.request_id,c.payload_digest,c.principal_id,c.instrument_id,c.amount,encoded))
        maybe_crash(crash_stage, "after_request_insert")
        conn.execute("UPDATE principal_caps SET used_units=used_units+? WHERE principal_id=? AND instrument_id=?", (c.amount,c.principal_id,c.instrument_id))
        maybe_crash(crash_stage, "after_cap_update")
        maybe_crash(crash_stage, "before_commit")
        conn.commit()
        maybe_crash(crash_stage, "after_commit_before_ack")
        return {"decision":"accepted","receipt":receipt}


def snapshot(db_path: str) -> dict:
    with sqlite3.connect(db_path, timeout=10) as conn:
        conn.execute("PRAGMA foreign_keys=ON")
        cap = conn.execute("SELECT cap_units,used_units FROM principal_caps WHERE principal_id=? AND instrument_id=?", ("principal-A","instrument-1")).fetchone()
        canonical = conn.execute("SELECT COUNT(*),COALESCE(SUM(amount_units),0) FROM canonical_work").fetchone()
        request_count = conn.execute("SELECT COUNT(*) FROM request_keys").fetchone()[0]
        broken = conn.execute("SELECT COUNT(*) FROM canonical_work c LEFT JOIN request_keys r ON r.request_id=c.canonical_request_id AND r.payload_digest=c.payload_digest WHERE r.request_id IS NULL").fetchone()[0]
        return {"cap_units":cap[0],"used_units":cap[1],"unique_obligations":canonical[0],"unique_obligation_units":canonical[1],"request_rows":request_count,"canonical_without_primary_request":broken,"integrity_check":conn.execute("PRAGMA integrity_check").fetchone()[0]}


def child_worker(db_path: str, claim_json: str, ready_path: str, release_path: str, result_path: str) -> int:
    Path(ready_path).write_text("ready", encoding="utf-8")
    deadline = time.monotonic() + 15
    while not Path(release_path).exists():
        if time.monotonic() > deadline:
            return 71
        time.sleep(0.002)
    c = Claim(**json.loads(claim_json))
    result = admit_once(db_path, c)
    Path(result_path).write_text(json.dumps(result, sort_keys=True), encoding="utf-8")
    return 0


def spawn_crash(db_path: str, c: Claim, stage: str) -> int:
    cp = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--child-crash", db_path, json.dumps(asdict(c)), stage], capture_output=True, text=True, timeout=20)
    return cp.returncode


def spawn_pair(db_path: str, claims: tuple[Claim, Claim], workdir: Path) -> tuple[list[dict], dict]:
    release = workdir / "release"
    ready_paths = [workdir / "ready-a", workdir / "ready-b"]
    result_paths = [workdir / "result-a.json", workdir / "result-b.json"]
    procs = []
    for c, ready, result in zip(claims, ready_paths, result_paths):
        args = [sys.executable, str(Path(__file__).resolve()), "--worker", db_path, json.dumps(asdict(c)), str(ready), str(release), str(result)]
        procs.append(subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True))
    deadline = time.monotonic() + 20
    while not all(p.exists() for p in ready_paths):
        if time.monotonic() > deadline:
            for p in procs: p.kill()
            raise TimeoutError("workers did not reach start barrier")
        time.sleep(0.002)
    release.touch()
    output=[]
    for p in procs:
        stdout, stderr = p.communicate(timeout=25)
        if p.returncode != 0:
            raise RuntimeError(f"worker exit={p.returncode}; stdout={stdout}; stderr={stderr}")
    for p in result_paths:
        output.append(json.loads(p.read_text(encoding="utf-8")))
    return output, snapshot(db_path)


def assert_invariants(s: dict) -> list[str]:
    errors=[]
    if s["used_units"] > s["cap_units"]: errors.append("cap_overshoot")
    if s["used_units"] != s["unique_obligation_units"]: errors.append("counter_not_sum_of_unique_work")
    if s["canonical_without_primary_request"] != 0: errors.append("canonical_without_primary_request")
    if s["integrity_check"] != "ok": errors.append("sqlite_integrity_check_failed")
    return errors


def run_campaign(crash_repetitions: int=10, race_repetitions: int=10) -> dict:
    started=time.perf_counter(); crash_rows=[]; race_rows=[]
    with tempfile.TemporaryDirectory(prefix="ncf-alloc8-") as td:
        root=Path(td)
        for stage in ALL_CRASH_STAGES:
            accepted=0; empty=0; failures=[]
            for i in range(crash_repetitions):
                d=root / f"crash-{stage}-{i}"; d.mkdir()
                db=str(d/"a.sqlite"); init_db(db)
                c=Claim(f"crash-{stage}-{i}",digest(f"work-{stage}-{i}"))
                code=spawn_crash(db,c,stage)
                s=snapshot(db); errs=assert_invariants(s)
                is_post=stage == "after_commit_before_ack"
                expected=(1,1,60) if is_post else (0,0,0)
                observed=(s["unique_obligations"],s["request_rows"],s["used_units"])
                ok=(code == CRASH_EXIT and observed == expected and not errs)
                accepted+=int(s["unique_obligations"]==1); empty+=int(s["unique_obligations"]==0)
                if not ok: failures.append({"iteration":i,"exit_code":code,"expected":expected,"observed":observed,"invariant_errors":errs})
                # Retry after recovery: pre-commit crashes should accept once; post-commit crash replays.
                retry=admit_once(db,c); after=snapshot(db); retry_expected="replay" if is_post else "accepted"
                if retry.get("decision") != retry_expected or after["used_units"] != 60 or assert_invariants(after):
                    failures.append({"iteration":i,"retry":retry,"retry_expected":retry_expected,"after_retry":after,"kind":"retry_recovery"})
            crash_rows.append({"stage":stage,"repetitions":crash_repetitions,"expected_pre_retry_state":"one_complete_obligation" if stage=="after_commit_before_ack" else "no_committed_obligation","accepted_before_retry":accepted,"empty_before_retry":empty,"failures":failures})
        for case in ("distinct_work_cap", "same_work_alias", "same_request_conflict"):
            successes=0; failures=[]; counts={}
            for i in range(race_repetitions):
                d=root / f"race-{case}-{i}"; d.mkdir()
                db=str(d/"a.sqlite"); init_db(db)
                if case=="distinct_work_cap": claims=(Claim(f"a-{i}",digest(f"work-a-{i}")),Claim(f"b-{i}",digest(f"work-b-{i}")))
                elif case=="same_work_alias": claims=(Claim(f"a-{i}",digest(f"shared-{i}")),Claim(f"b-{i}",digest(f"shared-{i}")))
                else: claims=(Claim(f"same-{i}",digest(f"work-a-{i}"),amount=40),Claim(f"same-{i}",digest(f"work-b-{i}"),amount=40))
                results,s=spawn_pair(db,claims,d)
                decisions=sorted(x["decision"] for x in results)
                if case=="distinct_work_cap": ok=decisions==["accepted","rejected"] and s["used_units"]==60 and s["unique_obligations"]==1
                elif case=="same_work_alias": ok=decisions==["accepted","replay"] and s["used_units"]==60 and s["unique_obligations"]==1 and s["request_rows"]==2
                else: ok=decisions==["accepted","rejected"] and s["used_units"]==40 and s["unique_obligations"]==1 and s["request_rows"]==1
                errs=assert_invariants(s); ok=ok and not errs
                successes+=int(ok)
                if not ok: failures.append({"iteration":i,"results":results,"snapshot":s,"invariant_errors":errs})
                for decision in decisions: counts[decision]=counts.get(decision,0)+1
            race_rows.append({"case":case,"trials":race_repetitions,"invariant_safe_trials":successes,"decision_counts":counts,"failures":failures})
    return {"campaign":"NCF-ALLOC-8","method":"local SQLite WAL + synchronous=FULL; subprocess os._exit crash injection; independent writer processes synchronized at a release barrier","environment":{"python":sys.version.split()[0],"sqlite":sqlite3.sqlite_version,"platform":sys.platform,"host":"single local filesystem/host"},"crash_trials_per_stage":crash_repetitions,"crash_stages":crash_rows,"race_trials_per_case":race_repetitions,"multiprocess_races":race_rows,"total_crash_trials":crash_repetitions*len(ALL_CRASH_STAGES),"total_multiprocess_pair_trials":race_repetitions*3,"total_invariant_failures":sum(len(x["failures"]) for x in crash_rows)+sum(len(x["failures"]) for x in race_rows),"elapsed_seconds":round(time.perf_counter()-started,4),"claim_ceiling":"Application-process crash recovery and independent-process contention on one local host only. Not kernel crash/power-loss simulation, storage-fault testing, multi-host consensus, Holochain, cryptographic verifier validation, Rust qualification, or production settlement."}


class CrashConsistencyTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix="ncf8-test-")
        self.root=Path(self.temp.name); self.db=str(self.root/"test.sqlite"); init_db(self.db)
    def tearDown(self): self.temp.cleanup()

    def test_process_crash_before_commit_recovers_no_partial_obligation(self):
        for stage in PRECOMMIT_STAGES:
            with self.subTest(stage=stage):
                d=self.root/stage; d.mkdir(); db=str(d/"a.sqlite"); init_db(db)
                c=Claim(f"req-{stage}",digest(stage))
                self.assertEqual(spawn_crash(db,c,stage),CRASH_EXIT)
                s=snapshot(db)
                self.assertEqual((s["unique_obligations"],s["request_rows"],s["used_units"]),(0,0,0))
                self.assertEqual(assert_invariants(s),[])
                self.assertEqual(admit_once(db,c)["decision"],"accepted")
                self.assertEqual(snapshot(db)["used_units"],60)

    def test_process_crash_after_commit_before_ack_recovers_and_replays(self):
        c=Claim("lost-ack",digest("lost-ack-work"))
        self.assertEqual(spawn_crash(self.db,c,"after_commit_before_ack"),CRASH_EXIT)
        s=snapshot(self.db)
        self.assertEqual((s["unique_obligations"],s["request_rows"],s["used_units"]),(1,1,60))
        retry=admit_once(self.db,c)
        self.assertEqual(retry["decision"],"replay")
        self.assertEqual(snapshot(self.db)["used_units"],60)

    def test_independent_processes_cannot_overshoot_cap(self):
        claims=(Claim("proc-a",digest("proc-work-a")),Claim("proc-b",digest("proc-work-b")))
        results,s=spawn_pair(self.db,claims,self.root)
        self.assertEqual(sorted(x["decision"] for x in results),["accepted","rejected"])
        self.assertEqual(s["used_units"],60); self.assertEqual(assert_invariants(s),[])

    def test_independent_processes_deduplicate_same_canonical_work(self):
        claims=(Claim("alias-a",digest("shared-work")),Claim("alias-b",digest("shared-work")))
        results,s=spawn_pair(self.db,claims,self.root)
        self.assertEqual(sorted(x["decision"] for x in results),["accepted","replay"])
        self.assertEqual((s["used_units"],s["unique_obligations"],s["request_rows"]),(60,1,2))

    def test_independent_processes_bind_conflicting_request_id(self):
        claims=(Claim("same-id",digest("different-a"),amount=40),Claim("same-id",digest("different-b"),amount=40))
        results,s=spawn_pair(self.db,claims,self.root)
        self.assertEqual(sorted(x["decision"] for x in results),["accepted","rejected"])
        self.assertEqual((s["used_units"],s["unique_obligations"],s["request_rows"]),(40,1,1))

    def test_integrity_counter_matches_unique_obligation_ledger(self):
        for i,amount in enumerate((10,20,30)):
            self.assertEqual(admit_once(self.db,Claim(f"r{i}",digest(f"w{i}"),amount=amount))["decision"],"accepted")
        s=snapshot(self.db)
        self.assertEqual(s["used_units"],60); self.assertEqual(s["unique_obligation_units"],60)
        self.assertEqual(s["integrity_check"],"ok"); self.assertEqual(assert_invariants(s),[])


def main() -> int:
    p=argparse.ArgumentParser()
    p.add_argument("--child-crash",nargs=3,metavar=("DB","CLAIM_JSON","STAGE"))
    p.add_argument("--worker",nargs=5,metavar=("DB","CLAIM_JSON","READY","RELEASE","RESULT"))
    p.add_argument("--campaign",action="store_true")
    p.add_argument("--crash-repetitions",type=int,default=10)
    p.add_argument("--race-repetitions",type=int,default=10)
    p.add_argument("--output",default="ncf-alloc8-crash-results.json")
    p.add_argument("--source-file",default=str(Path(__file__).resolve()))
    args=p.parse_args()
    if args.child_crash:
        db,claim_json,stage=args.child_crash
        admit_once(db,Claim(**json.loads(claim_json)),stage)
        return 0
    if args.worker:
        db,claim_json,ready,release,result=args.worker
        return child_worker(db,claim_json,ready,release,result)
    if args.campaign:
        result=run_campaign(args.crash_repetitions,args.race_repetitions)
        result["source_sha256"]=hashlib.sha256(Path(args.source_file).read_bytes()).hexdigest()
        out=Path(args.output); out.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        print(json.dumps(result,indent=2,sort_keys=True))
        return 1 if result["total_invariant_failures"] else 0
    unittest.main(argv=[sys.argv[0]])
    return 0

if __name__=="__main__":
    raise SystemExit(main())
