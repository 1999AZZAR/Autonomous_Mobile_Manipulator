import sqlite3
import json
import time
import uuid
import threading
import numpy as np
from typing import Optional, List, Dict


class TrainingCollector:
    def __init__(self, db_path: str = 'ml_training.db', max_frames: int = 100000):
        self.db_path = db_path
        self.max_frames = max_frames
        self._lock = threading.Lock()
        self._init_db()

    def _init_db(self):
        with self._lock, sqlite3.connect(self.db_path) as conn:
            conn.execute('''
                CREATE TABLE IF NOT EXISTS ml_training_data (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    features BLOB NOT NULL,
                    command TEXT NOT NULL,
                    reward REAL DEFAULT 0,
                    confidence REAL DEFAULT 0,
                    mode TEXT DEFAULT 'real',
                    sensor_snapshot TEXT,
                    yolo_snapshot TEXT,
                    session_id TEXT NOT NULL
                )
            ''')
            conn.execute('''
                CREATE INDEX IF NOT EXISTS idx_session
                ON ml_training_data(session_id)
            ''')
            conn.execute('''
                CREATE INDEX IF NOT EXISTS idx_timestamp
                ON ml_training_data(timestamp)
            ''')

            conn.execute('''
                CREATE TABLE IF NOT EXISTS ml_sessions (
                    id TEXT PRIMARY KEY,
                    created_at REAL NOT NULL,
                    mode TEXT NOT NULL,
                    total_frames INTEGER DEFAULT 0,
                    avg_reward REAL DEFAULT 0,
                    notes TEXT
                )
            ''')

    def new_session(self, mode: str = 'real', notes: str = '') -> str:
        session_id = str(uuid.uuid4())
        with self._lock, sqlite3.connect(self.db_path) as conn:
            conn.execute(
                'INSERT INTO ml_sessions (id, created_at, mode, notes) VALUES (?, ?, ?, ?)',
                (session_id, time.time(), mode, notes)
            )
        return session_id

    def record_frame(self, features: np.ndarray, command: str, reward: float = 0,
                     confidence: float = 0, mode: str = 'real',
                     sensor_snapshot: Optional[dict] = None,
                     yolo_snapshot: Optional[dict] = None,
                     session_id: Optional[str] = None) -> bool:
        if session_id is None:
            return False

        with self._lock, sqlite3.connect(self.db_path) as conn:
            if self._count_frames(conn) >= self.max_frames:
                self._prune_oldest(conn)

            conn.execute(
                '''INSERT INTO ml_training_data
                   (timestamp, features, command, reward, confidence,
                    mode, sensor_snapshot, yolo_snapshot, session_id)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)''',
                (time.time(), features.tobytes(), command, reward, confidence,
                 mode, json.dumps(sensor_snapshot) if sensor_snapshot else None,
                 json.dumps(yolo_snapshot) if yolo_snapshot else None,
                 session_id)
            )

            conn.execute(
                '''UPDATE ml_sessions SET total_frames = (
                    SELECT COUNT(*) FROM ml_training_data WHERE session_id = ?
                ) WHERE id = ?''',
                (session_id, session_id)
            )
        return True

    def get_frames(self, session_id: Optional[str] = None,
                   limit: int = 10000, offset: int = 0) -> List[Dict]:
        with sqlite3.connect(self.db_path) as conn:
            if session_id:
                rows = conn.execute(
                    '''SELECT * FROM ml_training_data
                       WHERE session_id = ? ORDER BY id LIMIT ? OFFSET ?''',
                    (session_id, limit, offset)
                ).fetchall()
            else:
                rows = conn.execute(
                    'SELECT * FROM ml_training_data ORDER BY id LIMIT ? OFFSET ?',
                    (limit, offset)
                ).fetchall()

        return [self._row_to_dict(r) for r in rows]

    def get_features_matrix(self, session_id: Optional[str] = None) -> tuple:
        with sqlite3.connect(self.db_path) as conn:
            if session_id:
                rows = conn.execute(
                    '''SELECT features, command FROM ml_training_data
                       WHERE session_id = ? ORDER BY id''',
                    (session_id,)
                ).fetchall()
            else:
                rows = conn.execute(
                    'SELECT features, command FROM ml_training_data ORDER BY id'
                ).fetchall()

        if not rows:
            return np.array([]), np.array([])

        X = np.frombuffer(rows[0][0], dtype=np.float32).reshape(1, -1)
        X_list = [X]
        y_list = [rows[0][1]]
        for feat_bytes, cmd in rows[1:]:
            X_list.append(np.frombuffer(feat_bytes, dtype=np.float32).reshape(1, -1))
            y_list.append(cmd)

        return np.vstack(X_list), np.array(y_list)

    def get_stats(self) -> Dict:
        with sqlite3.connect(self.db_path) as conn:
            total = conn.execute(
                'SELECT COUNT(*) FROM ml_training_data'
            ).fetchone()[0]
            sessions = conn.execute(
                'SELECT id, created_at, mode, total_frames, avg_reward FROM ml_sessions ORDER BY created_at DESC'
            ).fetchall()
            cmd_dist = conn.execute(
                'SELECT command, COUNT(*) as cnt FROM ml_training_data GROUP BY command ORDER BY cnt DESC'
            ).fetchall()

        return {
            'total_frames': total,
            'sessions': [
                {'id': s[0], 'created_at': s[1], 'mode': s[2],
                 'total_frames': s[3], 'avg_reward': s[4]}
                for s in sessions
            ],
            'command_distribution': {c: n for c, n in cmd_dist},
        }

    def delete_session(self, session_id: str):
        with self._lock, sqlite3.connect(self.db_path) as conn:
            conn.execute(
                'DELETE FROM ml_training_data WHERE session_id = ?',
                (session_id,)
            )
            conn.execute(
                'DELETE FROM ml_sessions WHERE id = ?',
                (session_id,)
            )

    def _count_frames(self, conn) -> int:
        return conn.execute(
            'SELECT COUNT(*) FROM ml_training_data'
        ).fetchone()[0]

    def _prune_oldest(self, conn):
        conn.execute('''
            DELETE FROM ml_training_data WHERE id IN (
                SELECT id FROM ml_training_data ORDER BY id LIMIT ?
            )
        ''', (self.max_frames // 10,))

    def _row_to_dict(self, row) -> Dict:
        return {
            'id': row[0], 'timestamp': row[1],
            'command': row[3], 'reward': row[4],
            'confidence': row[5], 'mode': row[6],
            'sensor_snapshot': json.loads(row[7]) if row[7] else None,
            'yolo_snapshot': json.loads(row[8]) if row[8] else None,
            'session_id': row[9],
        }
