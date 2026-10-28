#!/usr/bin/env python3
"""NCF-ALLOC-11: typed outcomes for durable obligation admission.

A small executable protocol reference. It distinguishes policy rejection, known
storage failure, durable acceptance/replay, and uncertain acknowledgement after
commit. Storage-failure injection uses SQLite max_page_count, not physical ENOSPC.
"""
from __future__ import annotations
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest


SCHEMA = """
CREATE TABLE caps(
 principal TEXT NOT NULL, instrument TEXT NOT NULL,
 cap_units INTEGER NOT NULL CHECK(cap_units >= 0),
 used_units INTEGER NOT NULL DEFAULT 0 CHECK(used_units >= 0 AND used_units <= cap_units),
 PRIMARY KEY(principal,instrument)
);
CREATE TABLE canonical_work(
 work_digest TEXT PRIMARY KEY, principal TEXT NOT NULL, instrument TEXT NOT NULL,
 amount INTEGER NOT NULL CHECK(amount > 0), canonical_request_id TEXT NOT NULL,
 receipt TEXT NOT NULL
);
CREATE TABLE request_keys(
 request_id TEXT PRIMARY KEY, work_digest TEXT NOT NULL REFERENCES canonical_work(work_digest),
 principal TEXT NOT NULL, instrument TEXT NOT NULL, amount INTEGER NOT NULL CHECK(amount > 0),
 receipt TEXT NOT NULL
);
"""


def open_conn(path: str, timeout: float = 2.0) -> sqlite3.Connection:
    c = sqlite3.connect(path, timeout=timeout, isolation_level=None)
    c.execute("PRAGMA foreign_keys=ON")
    c.execute(f"PRAGMA busy_timeout={int(timeout * 1000)}")
    return c


def init_db(path: str, cap_units: int = 100) -> None:
    with closing(open_conn(path)) as c:
        c.execute("PRAGMA journal_mode=WAL")
        c.execute("PRAGMA synchronous=FULL")
        c.executescript(SCHEMA)
        c.execute("BEGIN IMMEDIATE")
        c.execute("INSERT INTO caps VALUES('p1','i1',?,0)", (cap_units,))
        c.commit()


def work_digest(value: str) -> str:
    return hashlib.sha256(value.encode()).hexdigest()


def outcome(kind: str, **fields) -> dict:
    return {"kind": kind, **fields}


def submit(path: str, request_id: str, digest: str, amount: int, receipt: str,
           max_page_count: int | None = None, lose_ack_after_commit: bool = False) -> dict:
    """Outcome kinds: Accepted, PolicyRejected, StorageFailed, OutcomeUnknown."""
    with closing(open_conn(path, timeout=2.0)) as c:
        c.execute("PRAGMA synchronous=FULL")
        if max_page_count is not None:
            c.execute(f"PRAGMA max_page_count={max_page_count}")
        c.execute("BEGIN IMMEDIATE")
        try:
            prior = c.execute(
                "SELECT work_digest,principal,instrument,amount,receipt FROM request_keys WHERE request_id=?",
                (request_id,)
            ).fetchone()
            if prior is not None:
                if prior[:4] != (digest, "p1", "i1", amount):
                    c.rollback()
                    return outcome("PolicyRejected", reason="request_id_conflict", request_id=request_id)
                c.commit()
                return outcome("Accepted", receipt=prior[4], replayed=True, request_id=request_id)

            canonical = c.execute(
                "SELECT principal,instrument,amount,receipt FROM canonical_work WHERE work_digest=?",
                (digest,)
            ).fetchone()
            if canonical is not None:
                if canonical[:3] != ("p1", "i1", amount):
                    c.rollback()
                    return outcome("PolicyRejected", reason="canonical_work_conflict", request_id=request_id)
                c.execute("INSERT INTO request_keys VALUES(?,?,?,?,?,?)",
                          (request_id,digest,"p1","i1",amount,canonical[3]))
                c.commit()
                return outcome("Accepted", receipt=canonical[3], replayed=True, request_id=request_id)

            cap = c.execute("SELECT cap_units,used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()
            if cap is None:
                c.rollback()
                return outcome("PolicyRejected", reason="missing_cap", request_id=request_id)
            if amount <= 0 or cap[1] + amount > cap[0]:
                c.rollback()
                return outcome("PolicyRejected", reason="cap_exceeded_or_invalid_amount", request_id=request_id)

            # Deliberately updates counter before subsequent writes: any storage error
            # must rollback all of this transaction, not just the failing insert.
            c.execute("UPDATE caps SET used_units=used_units+? WHERE principal='p1' AND instrument='i1'", (amount,))
            c.execute("INSERT INTO canonical_work VALUES(?,?,?,?,?,?)",
                      (digest,"p1","i1",amount,request_id,receipt))
            c.execute("INSERT INTO request_keys VALUES(?,?,?,?,?,?)",
                      (request_id,digest,"p1","i1",amount,receipt))
            c.commit()
            if lose_ack_after_commit:
                return outcome("OutcomeUnknown", request_id=request_id,
                               reason="commit_may_have_succeeded_acknowledgement_lost")
            return outcome("Accepted", receipt=receipt, replayed=False, request_id=request_id)
        except sqlite3.OperationalError as exc:
            try:
                c.rollback()
            except sqlite3.Error:
                pass
            code = getattr(exc, "sqlite_errorcode", None)
            name = getattr(exc, "sqlite_errorname", None) or "SQLITE_ERROR"
            if code == sqlite3.SQLITE_FULL:
                retry_class = "retry_same_request_id_after_capacity_restored"
            elif code in (sqlite3.SQLITE_BUSY, sqlite3.SQLITE_LOCKED):
                retry_class = "retry_same_request_id_with_backoff"
            else:
                retry_class = "verify_same_request_id_before_retry"
            return outcome("StorageFailed", request_id=request_id, error=str(exc),
                           sqlite_errorcode=code, sqlite_errorname=name, retry_class=retry_class)
        except sqlite3.IntegrityError as exc:
            try:
                c.rollback()
            except sqlite3.Error:
                pass
            return outcome("PolicyRejected", reason="constraint_conflict", request_id=request_id, detail=str(exc))
        except Exception:
            try:
                c.rollback()
            except sqlite3.Error:
                pass
            raise


def snapshot(path: str) -> dict:
    with closing(open_conn(path, timeout=2.0)) as c:
        cap = c.execute("SELECT cap_units,used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()
        count, units, unique_digests = c.execute(
            "SELECT COUNT(*),COALESCE(SUM(amount),0),COUNT(DISTINCT work_digest) FROM canonical_work"
        ).fetchone()
        request_count = c.execute("SELECT COUNT(*) FROM request_keys").fetchone()[0]
        integrity = c.execute("PRAGMA integrity_check").fetchone()[0]
        foreign = c.execute("PRAGMA foreign_key_check").fetchall()
        return {
            "cap_units": cap[0], "used_units": cap[1], "canonical_count": count,
            "canonical_units": units, "unique_digests": unique_digests,
            "request_count": request_count, "integrity_check": integrity,
            "foreign_key_violations": len(foreign),
            "counter_matches_ledger": cap[1] == units,
            "work_digest_unique": count == unique_digests,
            "cap_not_overshot": cap[1] <= cap[0]
        }


def resolve_by_same_key(path: str, request_id: str, digest: str, amount: int) -> dict:
    """Resolve an unknown acknowledgement only by the original immutable request identity."""
    with closing(open_conn(path, timeout=2.0)) as c:
        row = c.execute(
            "SELECT work_digest,principal,instrument,amount,receipt FROM request_keys WHERE request_id=?",
            (request_id,)
        ).fetchone()
        if row is None:
            return outcome("OutcomeUnknown", request_id=request_id, reason="no_durable_receipt_observed")
        if row[:4] != (digest, "p1", "i1", amount):
            return outcome("PolicyRejected", request_id=request_id, reason="request_id_conflict")
        return outcome("Accepted", request_id=request_id, receipt=row[4], replayed=True)


class TypedOutcomeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="ncf11-outcomes-")
        self.db = str(Path(self.tmp.name) / "ledger.sqlite")
        init_db(self.db)
    def tearDown(self):
        self.tmp.cleanup()

    def _bounded_receipt(self, request_id: str) -> str:
        return json.dumps({"request_id":request_id,"receipt":"canonical"})

    def _force_full(self, request_id: str = "full-request", amount: int = 60) -> dict:
        with closing(open_conn(self.db)) as c:
            pages = c.execute("PRAGMA page_count").fetchone()[0]
        receipt = "x" * (10 * 1024 * 1024)
        return submit(self.db,request_id,work_digest(request_id),amount,receipt,
                      max_page_count=pages+1)

    def test_durable_commit_returns_accepted_not_before_commit(self):
        d=work_digest("accepted")
        result=submit(self.db,"r1",d,60,self._bounded_receipt("r1"))
        self.assertEqual(result["kind"],"Accepted",result)
        self.assertFalse(result["replayed"])
        self.assertEqual(snapshot(self.db)["used_units"],60)

    def test_policy_rejection_is_not_storage_failure(self):
        result=submit(self.db,"too-big",work_digest("too-big"),101,self._bounded_receipt("too-big"))
        self.assertEqual(result["kind"],"PolicyRejected",result)
        self.assertEqual(result["reason"],"cap_exceeded_or_invalid_amount")
        self.assertEqual(snapshot(self.db)["used_units"],0)

    def test_sqlite_full_returns_storage_failed_and_rolls_back_everything(self):
        result=self._force_full()
        self.assertEqual(result["kind"],"StorageFailed",result)
        self.assertEqual(result["sqlite_errorcode"],sqlite3.SQLITE_FULL,result)
        self.assertEqual(result["retry_class"],"retry_same_request_id_after_capacity_restored")
        state=snapshot(self.db)
        self.assertEqual((state["used_units"],state["canonical_count"],state["request_count"]),(0,0,0),state)
        self.assertTrue(state["counter_matches_ledger"])
        self.assertEqual(state["integrity_check"],"ok")

    def test_capacity_restore_retry_same_key_accepts_then_replays(self):
        failed=self._force_full("retry-me",60)
        self.assertEqual(failed["kind"],"StorageFailed",failed)
        with closing(open_conn(self.db)) as c:
            c.execute("PRAGMA max_page_count=2147483646")
        d=work_digest("retry-me"); receipt=self._bounded_receipt("retry-me")
        first=submit(self.db,"retry-me",d,60,receipt)
        second=submit(self.db,"retry-me",d,60,receipt)
        self.assertEqual(first["kind"],"Accepted",first)
        self.assertFalse(first["replayed"])
        self.assertEqual(second["kind"],"Accepted",second)
        self.assertTrue(second["replayed"])
        self.assertEqual(snapshot(self.db)["used_units"],60)

    def test_commit_ack_lost_returns_unknown_then_same_key_resolves(self):
        d=work_digest("lost-ack"); receipt=self._bounded_receipt("lost-ack")
        result=submit(self.db,"lost-ack",d,60,receipt,lose_ack_after_commit=True)
        self.assertEqual(result["kind"],"OutcomeUnknown",result)
        self.assertNotIn("receipt",result)
        resolved=resolve_by_same_key(self.db,"lost-ack",d,60)
        self.assertEqual(resolved["kind"],"Accepted",resolved)
        self.assertTrue(resolved["replayed"])
        self.assertEqual(snapshot(self.db)["used_units"],60)

    def test_unknown_commit_retry_with_conflicting_payload_never_mutates(self):
        d=work_digest("ambiguous-original"); receipt=self._bounded_receipt("ambiguous")
        unknown=submit(self.db,"ambiguous",d,30,receipt,lose_ack_after_commit=True)
        self.assertEqual(unknown["kind"],"OutcomeUnknown",unknown)
        conflict=submit(self.db,"ambiguous",work_digest("conflicting"),30,self._bounded_receipt("conflict"))
        self.assertEqual(conflict["kind"],"PolicyRejected",conflict)
        state=snapshot(self.db)
        self.assertEqual((state["used_units"],state["canonical_count"],state["request_count"]),(30,1,1),state)

    def test_distinct_request_id_alias_is_accepted_as_replay_without_second_debit(self):
        d=work_digest("shared-work")
        first=submit(self.db,"alias-a",d,40,self._bounded_receipt("alias-a"))
        second=submit(self.db,"alias-b",d,40,self._bounded_receipt("alias-b"))
        self.assertEqual(first["kind"],"Accepted",first)
        self.assertEqual(second["kind"],"Accepted",second)
        self.assertTrue(second["replayed"])
        self.assertEqual((snapshot(self.db)["used_units"],snapshot(self.db)["canonical_count"],snapshot(self.db)["request_count"]),(40,1,2))


if __name__ == "__main__":
    unittest.main(argv=[__file__])
