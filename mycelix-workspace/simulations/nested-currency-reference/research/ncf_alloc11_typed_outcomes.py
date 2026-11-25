#!/usr/bin/env python3
"""NCF-ALLOC-11: typed outcomes for durable obligation admission."""
from __future__ import annotations
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

SCHEMA = '''
CREATE TABLE caps(principal TEXT NOT NULL,instrument TEXT NOT NULL,cap_units INTEGER NOT NULL CHECK(cap_units>=0),used_units INTEGER NOT NULL DEFAULT 0 CHECK(used_units>=0 AND used_units<=cap_units),PRIMARY KEY(principal,instrument));
CREATE TABLE canonical_work(work_digest TEXT PRIMARY KEY,principal TEXT NOT NULL,instrument TEXT NOT NULL,amount INTEGER NOT NULL CHECK(amount>0),canonical_request_id TEXT NOT NULL,receipt TEXT NOT NULL);
CREATE TABLE request_keys(request_id TEXT PRIMARY KEY,work_digest TEXT NOT NULL REFERENCES canonical_work(work_digest),principal TEXT NOT NULL,instrument TEXT NOT NULL,amount INTEGER NOT NULL CHECK(amount>0),receipt TEXT NOT NULL);
'''

def conn(path, timeout=2.0):
    c=sqlite3.connect(path, timeout=timeout, isolation_level=None)
    c.execute('PRAGMA foreign_keys=ON')
    c.execute(f'PRAGMA busy_timeout={int(timeout*1000)}')
    return c

def init(path, cap=100):
    with closing(conn(path)) as c:
        c.execute('PRAGMA journal_mode=WAL'); c.execute('PRAGMA synchronous=FULL'); c.executescript(SCHEMA)
        c.execute('BEGIN IMMEDIATE'); c.execute("INSERT INTO caps VALUES('p1','i1',?,0)",(cap,)); c.commit()

def digest(x): return hashlib.sha256(x.encode()).hexdigest()
def result(kind, **fields): return {'kind':kind, **fields}
def receipt(req): return json.dumps({'request_id':req,'canonical':'receipt'},sort_keys=True)

def submit(path, req, work, amount, rec, max_page_count=None, lose_ack=False):
    """Outcome kinds: Accepted, PolicyRejected, StorageFailed, OutcomeUnknown."""
    with closing(conn(path)) as c:
        c.execute('PRAGMA synchronous=FULL')
        if max_page_count is not None: c.execute(f'PRAGMA max_page_count={max_page_count}')
        c.execute('BEGIN IMMEDIATE')
        try:
            old=c.execute('SELECT work_digest,principal,instrument,amount,receipt FROM request_keys WHERE request_id=?',(req,)).fetchone()
            if old:
                if old[:4] != (work,'p1','i1',amount):
                    c.rollback(); return result('PolicyRejected',request_id=req,reason='request_id_conflict')
                c.commit(); return result('Accepted',request_id=req,receipt=old[4],replayed=True)
            canonical=c.execute('SELECT principal,instrument,amount,receipt FROM canonical_work WHERE work_digest=?',(work,)).fetchone()
            if canonical:
                if canonical[:3] != ('p1','i1',amount):
                    c.rollback(); return result('PolicyRejected',request_id=req,reason='canonical_work_conflict')
                c.execute('INSERT INTO request_keys VALUES(?,?,?,?,?,?)',(req,work,'p1','i1',amount,canonical[3]))
                c.commit(); return result('Accepted',request_id=req,receipt=canonical[3],replayed=True)
            cap=c.execute("SELECT cap_units,used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()
            if cap is None:
                c.rollback(); return result('PolicyRejected',request_id=req,reason='missing_cap')
            if amount<=0 or cap[1]+amount>cap[0]:
                c.rollback(); return result('PolicyRejected',request_id=req,reason='cap_exceeded_or_invalid_amount')
            # Exercise all-or-nothing semantics: the counter changes before the fallible inserts.
            c.execute("UPDATE caps SET used_units=used_units+? WHERE principal='p1' AND instrument='i1'",(amount,))
            c.execute('INSERT INTO canonical_work VALUES(?,?,?,?,?,?)',(work,'p1','i1',amount,req,rec))
            c.execute('INSERT INTO request_keys VALUES(?,?,?,?,?,?)',(req,work,'p1','i1',amount,rec))
            c.commit()
            if lose_ack: return result('OutcomeUnknown',request_id=req,reason='commit_may_have_succeeded_acknowledgement_lost')
            return result('Accepted',request_id=req,receipt=rec,replayed=False)
        except sqlite3.OperationalError as exc:
            try: c.rollback()
            except sqlite3.Error: pass
            code=getattr(exc,'sqlite_errorcode',None); name=getattr(exc,'sqlite_errorname',None) or 'SQLITE_ERROR'
            retry='retry_same_request_id_after_capacity_restored' if code==sqlite3.SQLITE_FULL else ('retry_same_request_id_with_backoff' if code in (sqlite3.SQLITE_BUSY,sqlite3.SQLITE_LOCKED) else 'verify_same_request_id_before_retry')
            return result('StorageFailed',request_id=req,error=str(exc),sqlite_errorcode=code,sqlite_errorname=name,retry_class=retry)
        except sqlite3.IntegrityError as exc:
            try: c.rollback()
            except sqlite3.Error: pass
            return result('PolicyRejected',request_id=req,reason='constraint_conflict',detail=str(exc))
        except Exception:
            try: c.rollback()
            except sqlite3.Error: pass
            raise

def state(path):
    with closing(conn(path)) as c:
        cap=c.execute("SELECT cap_units,used_units FROM caps WHERE principal='p1' AND instrument='i1'").fetchone()
        count,units,unique=c.execute('SELECT COUNT(*),COALESCE(SUM(amount),0),COUNT(DISTINCT work_digest) FROM canonical_work').fetchone()
        reqs=c.execute('SELECT COUNT(*) FROM request_keys').fetchone()[0]
        integrity=c.execute('PRAGMA integrity_check').fetchone()[0]
        fk=len(c.execute('PRAGMA foreign_key_check').fetchall())
    return {'cap':cap[0],'used':cap[1],'count':count,'units':units,'unique':unique,'requests':reqs,'integrity':integrity,'fk':fk,'conserved':cap[1]==units,'unique_work':count==unique,'within_cap':cap[1]<=cap[0]}

def resolve(path, req, work, amount):
    """Resolve ambiguous acknowledgement only by the original immutable request identity."""
    with closing(conn(path)) as c:
        row=c.execute('SELECT work_digest,principal,instrument,amount,receipt FROM request_keys WHERE request_id=?',(req,)).fetchone()
    if row is None: return result('OutcomeUnknown',request_id=req,reason='no_durable_receipt_observed')
    if row[:4] != (work,'p1','i1',amount): return result('PolicyRejected',request_id=req,reason='request_id_conflict')
    return result('Accepted',request_id=req,receipt=row[4],replayed=True)

class TypedOutcomeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='ncf11-'); self.db=str(Path(self.tmp.name)/'ledger.sqlite'); init(self.db)
    def tearDown(self): self.tmp.cleanup()
    def fail_full(self,req='full',amount=60):
        with closing(conn(self.db)) as c: limit=c.execute('PRAGMA page_count').fetchone()[0]+1
        return submit(self.db,req,digest(req),amount,'x'*(10*1024*1024),max_page_count=limit)
    def test_durable_commit_is_accepted_only_after_commit(self):
        r=submit(self.db,'r1',digest('r1'),60,receipt('r1')); self.assertEqual(r['kind'],'Accepted',r); self.assertFalse(r['replayed']); self.assertEqual(state(self.db)['used'],60)
    def test_policy_rejection_is_not_storage_failure(self):
        r=submit(self.db,'big',digest('big'),101,receipt('big')); self.assertEqual((r['kind'],r['reason']),('PolicyRejected','cap_exceeded_or_invalid_amount')); self.assertEqual(state(self.db)['used'],0)
    def test_sqlite_full_is_typed_storage_failure_and_rolls_back(self):
        r=self.fail_full(); self.assertEqual(r['kind'],'StorageFailed',r); self.assertEqual(r['sqlite_errorcode'],sqlite3.SQLITE_FULL,r); self.assertEqual(r['retry_class'],'retry_same_request_id_after_capacity_restored'); s=state(self.db); self.assertEqual((s['used'],s['count'],s['requests']),(0,0,0),s); self.assertTrue(s['conserved']); self.assertEqual(s['integrity'],'ok')
    def test_capacity_restore_exact_retry_accepts_then_replays(self):
        self.assertEqual(self.fail_full('retry',60)['kind'],'StorageFailed')
        with closing(conn(self.db)) as c: c.execute('PRAGMA max_page_count=2147483646')
        d=digest('retry'); r1=submit(self.db,'retry',d,60,receipt('retry')); r2=submit(self.db,'retry',d,60,receipt('retry'))
        self.assertEqual(r1['kind'],'Accepted'); self.assertFalse(r1['replayed']); self.assertEqual(r2['kind'],'Accepted'); self.assertTrue(r2['replayed']); self.assertEqual(state(self.db)['used'],60)
    def test_commit_ack_lost_is_unknown_then_same_key_resolves(self):
        d=digest('lost'); r=submit(self.db,'lost',d,60,receipt('lost'),lose_ack=True); self.assertEqual(r['kind'],'OutcomeUnknown',r); self.assertNotIn('receipt',r); v=resolve(self.db,'lost',d,60); self.assertEqual(v['kind'],'Accepted',v); self.assertTrue(v['replayed']); self.assertEqual(state(self.db)['used'],60)
    def test_unknown_commit_conflict_does_not_mutate(self):
        self.assertEqual(submit(self.db,'amb',digest('orig'),30,receipt('amb'),lose_ack=True)['kind'],'OutcomeUnknown'); r=submit(self.db,'amb',digest('other'),30,receipt('other')); self.assertEqual((r['kind'],r['reason']),('PolicyRejected','request_id_conflict')); s=state(self.db); self.assertEqual((s['used'],s['count'],s['requests']),(30,1,1),s)
    def test_canonical_alias_returns_replay_without_second_debit(self):
        d=digest('shared'); first=submit(self.db,'a',d,40,receipt('a')); alias=submit(self.db,'b',d,40,receipt('b')); self.assertEqual(first['kind'],'Accepted'); self.assertEqual(alias['kind'],'Accepted'); self.assertTrue(alias['replayed']); self.assertEqual((state(self.db)['used'],state(self.db)['count'],state(self.db)['requests']),(40,1,2))

if __name__=='__main__': unittest.main(argv=[__file__])
