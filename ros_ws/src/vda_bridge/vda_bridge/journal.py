"""Durable at-most-once admission, committed BEFORE invoking ROS side effects."""
import json
import sqlite3
from pathlib import Path
from protocols.vda5050.models import dumps


def fingerprint(message):
    return json.dumps({k:v for k,v in message.items() if k not in {'headerId','timestamp'}},
                      sort_keys=True, allow_nan=False, separators=(',', ':'))


class Journal:
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True,exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS receipts(kind TEXT, id TEXT, payload TEXT, state TEXT, PRIMARY KEY(kind,id))')

    def admit(self, kind, key, payload):
        old = self.db.execute('SELECT payload FROM receipts WHERE kind=? AND id=?',(kind,key)).fetchone()
        if old:
            if fingerprint(json.loads(old[0])) != fingerprint(payload):
                raise ValueError('Duplicate ID with changed payload or stale orderUpdateId')
            return False
        with self.db:
            self.db.execute('INSERT INTO receipts VALUES(?,?,?,?)',(kind,key,dumps(payload),'ADMITTED'))
        return True

    def finish(self, kind, key, state):
        with self.db:
            self.db.execute('UPDATE receipts SET state=? WHERE kind=? AND id=?',(state,kind,key))

    def state(self, kind, key):
        row = self.db.execute('SELECT state FROM receipts WHERE kind=? AND id=?',
                              (kind, key)).fetchone()
        return row[0] if row else None

    def close(self):
        self.db.close()
