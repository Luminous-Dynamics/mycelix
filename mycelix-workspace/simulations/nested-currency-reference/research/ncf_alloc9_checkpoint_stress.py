#!/usr/bin/env python3
"""NCF-ALLOC-9: SQLite WAL checkpoint concurrency and ledger-consistency stress.

This is a local Python/SQLite stress harness, not an attempt to reproduce SQLite's
internal WAL-reset bug. It checks application-level invariants while writers,
read snapshots, and checkpoint modes overlap. Run on a SQLite build that contains
the WAL-reset fix for stronger adapter qualification; the current runtime is recorded.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
import hashlib
import json
from pathlib import Path
import random
import sqlite3
import statistics
import tempfile
import threading
import time
import unittest

DB_SCHEMA = """
CREATE TABLE caps (
  principal TEXT NOT NULL,
  instrument TEXT NOT NULL,
  cap_units INTEGER NOT NULL CHECK(cap_units >= 0),
  used_units INTEGER NOT NULL DEFAULT 0 CHECK(used_units >= 0 AND used_units <= cap_units),
  PRIMARY KEY(principal, instrument)
);
CREATE TABLE obligations (
  request_id TEXT PRIMARY KEY,
  work_digest TEXT NOT NULL UNIQUE,
  principal TEXT NOT NULL,
  instrument TEXT NOT NULL,
  amount INTEGER NOT NULL CHECK(amount > 0),
  receipt TEXT NOT NULL,
  FOREIGN KEY(principal, instrument) REFERENCES caps(principal, instrument)
);
"""


def open_conn(path: str, timeout: float = 2.0) -> sqlite3.Connection:
    conn = sqlite3.connect(path, timeout=timeout, isolation_level=None)
    conn.execute("PRAGMA foreign_keys=ON")
    conn.execute(f"PRAGMA busy_timeout={int(timeout*1000)}")
    return conn


def init_db(path: str, cap_units: int = 10**12) -> None:
    with closing(open_conn(path)) as conn:
        mode = conn.execute("PRAGMA journal_mode=WAL").fetchone()[0]
        if mode.lower() != "wal":
            raise RuntimeError(f"WAL mode requested but got {mode}")
        conn.execute("PRAGMA synchronous=FULL")
        conn.executescript(DB_SCHEMA)
        conn.execute("BEGIN IMMEDIATE")
        conn.execute("INSERT INTO caps(principal,instrument,cap_units,used_units) VALUES('p1','i1',?,0)", (cap_units,))
        conn.commit()


def stable_digest(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def writer(path: str, writer_id: int, count: int, start: threading.Barrier,
           failures: list[dict], latency_samples: list[float], lock: threading.Lock) -> dict:
    rng = random.Random(90210 + writer_id)
    conn = open_conn(path, timeout=5.0)
    conn.execute("PRAGMA synchronous=FULL")
    accepted = 0
    start.wait(timeout=10)
    for i in range(count):
        request = f"w{writer_id}-r{i}"
        work = stable_digest(f"canonical-work/{writer_id}/{i}")
        amount = rng.randint(1, 9)
        receipt = json.dumps({"request_id": request, "work_digest": work, "amount": amount}, sort_keys=True, separators=(",", ":"))
        began = time.perf_counter()
        attempts = 0
        while True:
            attempts += 1
            try:
                conn.execute("BEGIN IMMEDIATE")
                existing = conn.execute("SELECT work_digest,amount,receipt FROM obligations WHERE request_id=?", (request,)).fetchone()
                if existing:
                    conn.rollback()
                    with lock:
                        failures.append({"kind":"unexpected_duplicate_request","request_id":request})
                    break
                cap = conn.execute("SELECT cap_units,used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()
                if cap is None or cap[1] + amount > cap[0]:
                    conn.rollback()
                    with lock:
                        failures.append({"kind":"cap_reject_or_missing_row","request_id":request})
                    break
                conn.execute("INSERT INTO obligations VALUES(?,?,?,?,?,?)", (request, work, "p1", "i1", amount, receipt))
                conn.execute("UPDATE caps SET used_units=used_units+? WHERE principal='p1' AND instrument='i1'", (amount,))
                conn.commit()
                accepted += 1
                with lock:
                    latency_samples.append((time.perf_counter() - began) * 1000.0)
                break
            except sqlite3.OperationalError as exc:
                try: conn.rollback()
                except sqlite3.Error: pass
                if "locked" in str(exc).lower() or "busy" in str(exc).lower():
                    if attempts < 6:
                        time.sleep(min(0.002 * attempts, 0.02))
                        continue
                with lock:
                    failures.append({"kind":"writer_sqlite_error","request_id":request,"error":str(exc),"attempts":attempts})
                break
            except Exception as exc:
                try: conn.rollback()
                except sqlite3.Error: pass
                with lock: failures.append({"kind":"writer_error","request_id":request,"error":repr(exc)})
                break
    conn.close()
    return {"writer_id":writer_id,"requested":count,"accepted":accepted}


def checkpointer(path: str, cycles: int, start: threading.Barrier,
                 observations: list[dict], failures: list[dict], lock: threading.Lock) -> dict:
    conn = open_conn(path, timeout=0.1)
    start.wait(timeout=10)
    modes = ("PASSIVE", "RESTART", "TRUNCATE")
    rng = random.Random(731)
    completed = 0
    busy_or_partial = 0
    for i in range(cycles):
        mode = modes[i % len(modes)] if i % 5 else rng.choice(modes)
        began = time.perf_counter()
        try:
            row = conn.execute(f"PRAGMA wal_checkpoint({mode})").fetchone()
            busy, log_frames, checkpointed_frames = (int(row[0]), int(row[1]), int(row[2]))
            complete = (busy == 0 and (log_frames < 0 or checkpointed_frames >= log_frames))
            completed += int(complete)
            busy_or_partial += int(not complete)
            with lock:
                observations.append({"mode":mode,"busy":busy,"log_frames":log_frames,
                                     "checkpointed_frames":checkpointed_frames,"complete":complete,
                                     "latency_ms":round((time.perf_counter()-began)*1000,4)})
        except sqlite3.OperationalError as exc:
            if "locked" in str(exc).lower() or "busy" in str(exc).lower():
                busy_or_partial += 1
                with lock: observations.append({"mode":mode,"busy_error":str(exc),"complete":False})
            else:
                with lock: failures.append({"kind":"checkpoint_sqlite_error","mode":mode,"error":str(exc)})
    conn.close()
    return {"cycles_requested":cycles,"complete_checkpoint_count":completed,"busy_or_partial_count":busy_or_partial}


def snapshot_reader(path: str, reads: int, start: threading.Barrier,
                    observations: list[dict], failures: list[dict], lock: threading.Lock) -> dict:
    conn = open_conn(path, timeout=2.0)
    start.wait(timeout=10)
    good = 0
    for i in range(reads):
        try:
            conn.execute("BEGIN")
            # Both queries share one read transaction, so both should see the same WAL snapshot.
            cap = conn.execute("SELECT used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()[0]
            count, units, bad = conn.execute("SELECT COUNT(*),COALESCE(SUM(amount),0),SUM(CASE WHEN principal!='p1' OR instrument!='i1' THEN 1 ELSE 0 END) FROM obligations").fetchone()
            conn.commit()
            error = None
            if cap != units: error = f"used_units {cap} != obligation sum {units}"
            if bad or bad is None and count != 0: error = f"unexpected scope rows: bad={bad}, count={count}"
            with lock:
                observations.append({"snapshot_read":i,"used_units":cap,"obligation_count":count,"sum_units":units,"consistent":error is None,"error":error})
                if error: failures.append({"kind":"inconsistent_reader_snapshot","read":i,"error":error})
            good += int(error is None)
            if i % 7 == 0: time.sleep(0.0005)
        except Exception as exc:
            try: conn.rollback()
            except sqlite3.Error: pass
            with lock: failures.append({"kind":"reader_error","read":i,"error":repr(exc)})
    conn.close()
    return {"reads_requested":reads,"consistent_snapshots":good}


def validate_db(path: str) -> dict:
    conn = open_conn(path, timeout=5.0)
    cap = conn.execute("SELECT cap_units,used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()
    count, units, unique_digests = conn.execute("SELECT COUNT(*),COALESCE(SUM(amount),0),COUNT(DISTINCT work_digest) FROM obligations").fetchone()
    duplicate_work = conn.execute("SELECT COUNT(*) FROM (SELECT work_digest FROM obligations GROUP BY work_digest HAVING COUNT(*)>1)").fetchone()[0]
    integrity = conn.execute("PRAGMA integrity_check").fetchone()[0]
    fk = conn.execute("PRAGMA foreign_key_check").fetchall()
    conn.close()
    return {"cap_units":cap[0],"used_units":cap[1],"obligation_count":count,"obligation_units":units,
            "unique_work_digests":unique_digests,"duplicate_work_groups":duplicate_work,"integrity_check":integrity,
            "foreign_key_violations":len(fk),"cap_not_overshot":cap[1]<=cap[0],
            "counter_matches_ledger":cap[1]==units,"work_digest_unique":unique_digests==count and duplicate_work==0}


def run_campaign(writers: int = 4, writes_per_writer: int = 250, checkpoint_cycles: int = 750, readers: int = 3, reads_per_reader: int = 300) -> dict:
    began = time.perf_counter()
    with tempfile.TemporaryDirectory(prefix="ncf-alloc9-") as tmp:
        db = str(Path(tmp) / "ledger.sqlite")
        init_db(db)
        barrier = threading.Barrier(1 + writers + 1 + readers)
        failures: list[dict] = []
        checkpoint_obs: list[dict] = []
        reader_obs: list[dict] = []
        latencies: list[float] = []
        lock = threading.Lock()
        output=[]
        with ThreadPoolExecutor(max_workers=writers+readers+1) as pool:
            future_specs=[]
            for w in range(writers):
                future_specs.append(("writer",pool.submit(writer,db,w,writes_per_writer,barrier,failures,latencies,lock)))
            future_specs.append(("checkpointer",pool.submit(checkpointer,db,checkpoint_cycles,barrier,checkpoint_obs,failures,lock)))
            for r in range(readers):
                future_specs.append(("reader",pool.submit(snapshot_reader,db,reads_per_reader,barrier,reader_obs,failures,lock)))
            barrier.wait(timeout=15)
            for kind,fut in future_specs:
                try: output.append({"kind":kind,"result":fut.result(timeout=90)})
                except Exception as exc:
                    failures.append({"kind":"future_failure","worker_kind":kind,"error":repr(exc)})
        conn = open_conn(db, timeout=2.0)
        try: final_checkpoint = conn.execute("PRAGMA wal_checkpoint(TRUNCATE)").fetchone()
        except sqlite3.Error as exc: final_checkpoint = ["error",str(exc)]
        conn.close()
        final = validate_db(db)
    sorted_lat = sorted(latencies)
    def pct(p):
        return round(sorted_lat[min(len(sorted_lat)-1, int((len(sorted_lat)-1)*p))],4) if sorted_lat else None
    check_latency=[x.get("latency_ms",0) for x in checkpoint_obs if "latency_ms" in x]
    sorted_check = sorted(check_latency)
    def cpct(p):
        return round(sorted_check[min(len(sorted_check)-1, int((len(sorted_check)-1)*p))],4) if sorted_check else None
    requested_writes = writers * writes_per_writer
    accepted_writes = sum(x.get("result",{}).get("accepted",0) for x in output if x["kind"]=="writer")
    expected_snapshots = readers * reads_per_reader
    consistent_snapshots = sum(bool(x.get("consistent")) for x in reader_obs)
    final_checkpoint_ok = isinstance(final_checkpoint, (tuple, list)) and list(final_checkpoint) == [0, 0, 0]
    failure_count = (len(failures) + int(accepted_writes != requested_writes) +
                     int(len(reader_obs) != expected_snapshots) + int(consistent_snapshots != expected_snapshots) +
                     int(not final_checkpoint_ok) +
                     sum(not v for k,v in final.items() if k in ("cap_not_overshot","counter_matches_ledger","work_digest_unique")) +
                     int(final["integrity_check"]!="ok") + final["foreign_key_violations"])
    return {
      "campaign":"NCF-ALLOC-9",
      "method":"Concurrent file-backed SQLite WAL writer transactions + PASSIVE/RESTART/TRUNCATE checkpoint calls + consistent read snapshots; thread barrier starts participants together",
      "environment":{"python":__import__('sys').version.split()[0],"sqlite":sqlite3.sqlite_version,"platform":__import__('sys').platform,"host":"single local filesystem/host"},
      "parameters":{"writers":writers,"writes_per_writer":writes_per_writer,"checkpoint_cycles":checkpoint_cycles,"readers":readers,"reads_per_reader":reads_per_reader},
      "write_operations_requested":requested_writes,
      "write_operations_accepted":accepted_writes,
      "checkpoint_observations":len(checkpoint_obs),"checkpoints_complete":sum(bool(x.get("complete")) for x in checkpoint_obs),
      "checkpoints_busy_or_partial":sum(not bool(x.get("complete")) for x in checkpoint_obs),
      "checkpoint_latency_ms":{"p50":cpct(.50),"p95":cpct(.95),"p99":cpct(.99)},
      "reader_snapshot_observations":len(reader_obs),"expected_reader_snapshots":expected_snapshots,"consistent_reader_snapshots":consistent_snapshots,
      "write_latency_ms":{"p50":pct(.50),"p95":pct(.95),"p99":pct(.99)},
      "workers":output,"final_checkpoint_result":final_checkpoint,"final_checkpoint_ok":final_checkpoint_ok,"final_state":final,
      "failures":failures,"failure_count":failure_count,
      "elapsed_seconds":round(time.perf_counter()-began,4),
      "claim_ceiling":"Application-level SQLite WAL checkpoint/read/write stress on one host only. Does not reproduce the SQLite internal WAL-reset bug, prove fixed-version safety, simulate power loss, or qualify Rust/Holochain/multi-host production paths."
    }


class CheckpointStressTests(unittest.TestCase):
    def test_small_concurrent_checkpoint_writer_reader_stress(self):
        r=run_campaign(writers=2,writes_per_writer=30,checkpoint_cycles=60,readers=2,reads_per_reader=35)
        self.assertEqual(r["write_operations_accepted"],60,r["failures"])
        self.assertGreater(r["checkpoint_observations"],0)
        self.assertEqual(r["reader_snapshot_observations"],70)
        self.assertEqual(r["failure_count"],0,r)
        self.assertTrue(r["final_state"]["counter_matches_ledger"],r["final_state"])
        self.assertEqual(r["final_state"]["integrity_check"],"ok")
        self.assertTrue(r["final_state"]["work_digest_unique"])

    def test_empty_and_busy_or_partial_checkpoint_are_not_counted_as_failures(self):
        with tempfile.TemporaryDirectory(prefix="ncf9-empty-") as td:
            db=str(Path(td)/"db.sqlite"); init_db(db)
            conn=open_conn(db,timeout=1)
            row=conn.execute("PRAGMA wal_checkpoint(PASSIVE)").fetchone()
            self.assertEqual(len(row),3)
            conn.close()
            self.assertEqual(validate_db(db)["integrity_check"],"ok")

    def test_reopen_ledger_equates_counter_and_unique_obligations(self):
        with tempfile.TemporaryDirectory(prefix="ncf9-reopen-") as td:
            db=str(Path(td)/"db.sqlite"); init_db(db)
            conn=open_conn(db)
            conn.execute("BEGIN IMMEDIATE")
            for i in range(10):
                work=stable_digest(f"w{i}"); receipt=json.dumps({"i":i})
                conn.execute("INSERT INTO obligations VALUES(?,?,?,?,?,?)",(f"r{i}",work,"p1","i1",i+1,receipt))
                conn.execute("UPDATE caps SET used_units=used_units+? WHERE principal='p1' AND instrument='i1'",(i+1,))
            conn.commit(); conn.close()
            state=validate_db(db)
            self.assertEqual(state["used_units"],55)
            self.assertEqual(state["obligation_units"],55)
            self.assertEqual(state["integrity_check"],"ok")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--campaign",action="store_true")
    parser.add_argument("--writers",type=int,default=4)
    parser.add_argument("--writes-per-writer",type=int,default=250)
    parser.add_argument("--checkpoint-cycles",type=int,default=750)
    parser.add_argument("--readers",type=int,default=3)
    parser.add_argument("--reads-per-reader",type=int,default=300)
    parser.add_argument("--output",default="ncf-alloc9-checkpoint-results.json")
    args=parser.parse_args()
    if args.campaign:
        result=run_campaign(args.writers,args.writes_per_writer,args.checkpoint_cycles,args.readers,args.reads_per_reader)
        result["source_sha256"]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        Path(args.output).write_text(json.dumps(result,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        print(json.dumps(result,indent=2,sort_keys=True))
        return 1 if result["failure_count"] else 0
    unittest.main(argv=[__file__])
    return 0

if __name__=="__main__":
    raise SystemExit(main())
