import json
from typing import List, Dict, Optional, Tuple
from collections import defaultdict
from dataclasses import dataclass

from .training_collector import TrainingCollector


SENSOR_FEEDS = [
    'laser_left_front', 'laser_left_back', 'laser_right_front',
    'laser_right_back', 'laser_back_left', 'laser_back_right',
    'ultra_front_left', 'ultra_front_right',
]


@dataclass
class Pattern:
    conditions: List[Dict]
    action: str
    confidence: float
    count: int
    session_ids: List[str]

    def to_dict(self) -> dict:
        return {
            'conditions': self.conditions,
            'action': self.action,
            'confidence': round(self.confidence, 3),
            'count': self.count,
        }


@dataclass
class AutomationDraft:
    name: str
    trigger_type: str
    condition_match: str
    conditions: List[Dict]
    actions: List[Dict]
    source: str
    confidence: float

    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'triggerType': self.trigger_type,
            'conditionMatch': self.condition_match,
            'conditions': self.conditions,
            'actions': self.actions,
            'source': self.source,
            'confidence': round(self.confidence, 3),
        }


class PatternExtractor:
    def __init__(self, collector: TrainingCollector):
        self.collector = collector

    def analyze_session(self, session_id: str) -> List[Pattern]:
        frames = self.collector.get_frames(session_id=session_id)
        return self._extract_patterns(frames)

    def find_recurring_patterns(self, min_count: int = 5) -> List[Pattern]:
        stats = self.collector.get_stats()
        patterns_by_key = defaultdict(lambda: {
            'count': 0, 'sessions': set(), 'action': '', 'conditions': []
        })
        total_applicable = defaultdict(int)

        for session in stats.get('sessions', []):
            frames = self.collector.get_frames(
                session_id=session['id'], limit=5000
            )
            if len(frames) < 3:
                continue

            for i in range(1, len(frames)):
                prev = frames[i - 1]
                curr = frames[i]
                if curr['command'] == prev['command']:
                    continue

                snap = curr.get('sensor_snapshot')
                if not snap:
                    continue

                conditions = self._extract_conditions(snap, curr['command'])
                key = json.dumps(conditions, sort_keys=True) + curr['command']
                patterns_by_key[key]['count'] += 1
                patterns_by_key[key]['sessions'].add(curr['session_id'])
                patterns_by_key[key]['action'] = curr['command']
                patterns_by_key[key]['conditions'] = conditions

                total_applicable[key] += 1

        results = []
        for key, data in patterns_by_key.items():
            if data['count'] < min_count:
                continue
            conf = data['count'] / max(1, total_applicable[key])
            results.append(Pattern(
                conditions=data['conditions'],
                action=data['action'],
                confidence=min(1.0, conf),
                count=data['count'],
                session_ids=list(data['sessions']),
            ))

        results.sort(key=lambda p: p.count, reverse=True)
        return results

    def suggest_automation(self, pattern: Pattern,
                           prefix: str = 'ai') -> AutomationDraft:
        feed_labels = {
            'laser_left_front': 'Left Front IR',
            'laser_left_back': 'Left Back IR',
            'laser_right_front': 'Right Front IR',
            'laser_right_back': 'Right Back IR',
            'laser_back_left': 'Back Left IR',
            'laser_back_right': 'Back Right IR',
            'ultra_front_left': 'Front Left US',
            'ultra_front_right': 'Front Right US',
        }

        cmd_labels = {
            'f': 'Move Forward', 'b': 'Move Backward',
            'q': 'Move Forward-Left', 'e': 'Move Forward-Right',
            'z': 'Move Backward-Left', 'x': 'Move Backward-Right',
            't': 'Turn Left', 'y': 'Turn Right', 's': 'Stop',
        }

        cond_str = '_'.join(
            f"{c['feed'].replace('laser_', '').replace('ultra_', '')}"
            f"_{c['op']}_{int(c['threshold'])}"
            for c in pattern.conditions[:3]
        )
        name = f"{prefix}_auto_{cond_str}_to_{pattern.action}"

        return AutomationDraft(
            name=name,
            trigger_type='sensor',
            condition_match='ALL',
            conditions=[
                {
                    'feed': c['feed'],
                    'operator': c['op'],
                    'threshold': c['threshold'],
                    'label': feed_labels.get(c['feed'], c['feed']),
                }
                for c in pattern.conditions
            ],
            actions=[
                {
                    'type': 'move',
                    'value': pattern.action,
                    'label': cmd_labels.get(pattern.action, pattern.action),
                }
            ],
            source=f'pattern_extractor:{pattern.count}occurences',
            confidence=pattern.confidence,
        )

    def _extract_patterns(self, frames: List[dict]) -> List[Pattern]:
        if len(frames) < 2:
            return []

        runs = []
        current_run = [frames[0]]
        for i in range(1, len(frames)):
            if frames[i]['command'] == current_run[-1]['command']:
                current_run.append(frames[i])
            else:
                if len(current_run) >= 2:
                    runs.append(current_run)
                current_run = [frames[i]]
        if len(current_run) >= 2:
            runs.append(current_run)

        patterns = []
        for run in runs:
            cmd = run[0]['command']
            sensor_values = {feed: [] for feed in SENSOR_FEEDS}

            for frame in run:
                snap = frame.get('sensor_snapshot')
                if snap:
                    for feed in SENSOR_FEEDS:
                        val = snap.get(feed)
                        if val is not None:
                            sensor_values[feed].append(val)

            conditions = []
            for feed in SENSOR_FEEDS:
                vals = sensor_values[feed]
                if not vals:
                    continue
                min_val = min(vals)
                max_val = max(vals)

                if max_val < 300:
                    conditions.append({
                        'feed': feed, 'op': 'lt', 'threshold': max_val + 50,
                    })
                elif min_val > 1000:
                    conditions.append({
                        'feed': feed, 'op': 'gt', 'threshold': min_val - 50,
                    })

            if conditions:
                patterns.append(Pattern(
                    conditions=conditions,
                    action=cmd,
                    confidence=1.0,
                    count=len(run),
                    session_ids=[run[0].get('session_id', '')],
                ))

        return patterns

    def _extract_conditions(self, sensor_snapshot: dict,
                            command: str) -> List[Dict]:
        conditions = []
        for feed in SENSOR_FEEDS:
            val = sensor_snapshot.get(feed)
            if val is None:
                continue
            if val < 300:
                conditions.append({
                    'feed': feed, 'op': 'lt', 'threshold': 300,
                })
            elif val > 1200:
                conditions.append({
                    'feed': feed, 'op': 'gt', 'threshold': 1200,
                })

            if 'ultra' in feed and val < 500:
                conditions.append({
                    'feed': feed, 'op': 'lt', 'threshold': 500,
                })

        return conditions
