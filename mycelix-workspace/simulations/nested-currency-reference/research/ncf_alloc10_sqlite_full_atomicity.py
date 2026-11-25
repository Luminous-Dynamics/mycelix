#!/usr/bin/env python3
"""NCF-ALLOC-10: SQLite FULL/disk-capacity failures must not split admission state.

This research uses PRAGMA max_page_count to deterministically trigger SQLITE_FULL.
It simulates a database-size limit, not a physically full device or failing filesystem.
"""
from __future__ import annotations
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

from ncf_alloc9_checkpoint_stress import init_db, open_conn, validate_db

MAX_PAGE_CEILING = 2_147_483_646


def work_digest(label: str) -> str:
    return hashlib.sha256(label.encode()).hexdigest()


def restrict_growth(path: str, extra_pages: int = 1) -> dict:
    with closing(open_conn(path, timeout=2.0)) as conn:
        page_count = conn.execute("PRAGMA page_count").fetchone()[0]
        requested_limit = page_count + extra_pages
        actual_limit = conn.execute(f"PRAGMA max_page_count={requested_limit}").fetchone()[0]
        if actual_limit != requested_limit:
            raise AssertionError(f"could not set deterministic page cap: requested={requested_limit}, got={actual_limit}")
        return {"page_count_before":page_count,"max_page_count":actual_limit}


def restore_growth(path: str) -> int:
    with closing(open_conn(path, timeout=2.0)) as conn:
        return conn.execute(f"PRAGMA max_page_count={MAX_PAGE_CEILING}").fetchone()[0]


def admit(path: str, request_id: str, digest: str, amount: int, receipt: str, page_limit: int | None = None) -> dict:
    """One atomic transaction, deliberately mutating cap before a later fallible insert."""
    with closing(open_conn(path, timeout=2.0)) as conn:
        conn.execute("PRAGMA synchronous=FULL")
        if page_limit is not None:
            conn.execute(f"PRAGMA max_page_count={page_limit}")
        conn.execute("BEGIN IMMEDIATE")
        try:
            existing = conn.execute("SELECT work_digest,amount,receipt FROM obligations WHERE request_id=?",(request_id,)).fetchone()
            if existing:
                if existing[0] == digest and existing[1] == amount:
                    conn.commit()
                    return {"decision":"replay","receipt":existing[2]}
                conn.rollback()
                return {"decision":"rejected","reason":"request_id_conflict"}
            cap = conn.execute("SELECT cap_units,used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()
            if cap is None or cap[1] + amount > cap[0]:
                conn.rollback()
                return {"decision":"rejected","reason":"cap_exceeded_or_missing"}
            # Force the dangerous ordering: a database-capacity error in the later insert
            # must undo this earlier liability mutation through transaction rollback.
            conn.execute("UPDATE caps SET used_units=used_units+? WHERE principal='p1' AND instrument='i1'",(amount,))
            conn.execute("INSERT INTO obligations VALUES(?,?,?,?,?,?)",(request_id,digest,"p1","i1",amount,receipt))
            conn.commit()
            return {"decision":"accepted","receipt":receipt}
        except sqlite3.OperationalError as exc:
            try: conn.rollback()
            except sqlite3.Error: pass
            return {"decision":"storage_error","error":str(exc),"sqlite_errorcode":getattr(exc,"sqlite_errorcode",None),"sqlite_errorname":getattr(exc,"sqlite_errorname",None)}
        except Exception:
            try: conn.rollback()
            except sqlite3.Error: pass
            raise


def snapshot(path: str) -> dict:
    return validate_db(path)


class SqliteFullAtomicityTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix="ncf10-full-")
        self.db=str(Path(self.temp.name)/"ledger.sqlite")
        init_db(self.db)
    def tearDown(self):
        self.temp.cleanup()

    def _force_full(self, request_id="blocked", amount=60):
        with closing(open_conn(self.db,timeout=2.0)) as conn:
            page_count=conn.execute("PRAGMA page_count").fetchone()[0]
        page_limit=page_count+1
        giant_receipt=json.dumps({"request_id":request_id,"padding":"x"*(10*1024*1024)},separators=(",",":"))
        result=admit(self.db,request_id,work_digest(request_id),amount,giant_receipt,page_limit=page_limit)
        self.assertNotEqual(result["decision"],"accepted",f"decision={result.get('decision')} error={result.get('error')}")
        self.assertEqual(result["decision"],"storage_error",f"decision={result.get('decision')} error={result.get('error')}")
        self.assertIn("full",result["error"].lower(),result)
        self.assertIn(result["sqlite_errorcode"],(sqlite3.SQLITE_FULL,sqlite3.SQLITE_IOERR),result)
        return {"page_count_before":page_count,"max_page_count":page_limit},result

    def test_sqlite_full_rolls_back_prior_counter_update(self):
        _,result=self._force_full()
        state=snapshot(self.db)
        self.assertEqual(state["used_units"],0,state)
        self.assertEqual(state["obligation_count"],0,state)
        self.assertEqual(state["integrity_check"],"ok",state)
        self.assertEqual(state["foreign_key_violations"],0,state)
        self.assertTrue(state["counter_matches_ledger"],state)
        self.assertEqual(result["sqlite_errorcode"],sqlite3.SQLITE_FULL,result)

    def test_restore_capacity_then_exact_retry_accepts_once(self):
        self._force_full(request_id="retry-me",amount=60)
        restored=restore_growth(self.db)
        self.assertGreater(restored,0)
        receipt=json.dumps({"request_id":"retry-me","payload":"small"},sort_keys=True)
        result=admit(self.db,"retry-me",work_digest("retry-me"),60,receipt)
        self.assertEqual(result["decision"],"accepted",result)
        retry=admit(self.db,"retry-me",work_digest("retry-me"),60,receipt)
        self.assertEqual(retry["decision"],"replay",retry)
        state=snapshot(self.db)
        self.assertEqual((state["used_units"],state["obligation_count"],state["obligation_units"]),(60,1,60),state)
        self.assertEqual(state["integrity_check"],"ok",state)

    def test_failed_full_attempt_does_not_mutate_prior_accepted_obligation(self):
        first=admit(self.db,"first",work_digest("first"),20,'{"first":true}')
        self.assertEqual(first["decision"],"accepted")
        self._force_full(request_id="second",amount=40)
        state=snapshot(self.db)
        self.assertEqual((state["used_units"],state["obligation_count"],state["obligation_units"]),(20,1,20),state)
        self.assertEqual(state["integrity_check"],"ok",state)
        restore_growth(self.db)
        second=admit(self.db,"second",work_digest("second"),40,'{"second":true}')
        self.assertEqual(second["decision"],"accepted",second)
        self.assertEqual(snapshot(self.db)["used_units"],60)

    def test_capacity_failure_is_distinct_from_policy_cap_rejection(self):
        state=snapshot(self.db)
        self.assertEqual(state["used_units"],0)
        cap_reject=admit(self.db,"too-large-for-policy",work_digest("policy"),10**13,'{"x":1}')
        self.assertEqual(cap_reject["decision"],"rejected",cap_reject)
        self.assertEqual(cap_reject["reason"],"cap_exceeded_or_missing")
        self.assertEqual(snapshot(self.db)["used_units"],0)
        self._force_full(request_id="storage-fail",amount=10)
        self.assertEqual(snapshot(self.db)["used_units"],0)


def main():
    unittest.main(argv=[__file__])

if __name__ == "__main__":
    main()
