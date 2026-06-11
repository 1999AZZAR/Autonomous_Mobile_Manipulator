import json
import time
import logging
from typing import List, Optional
from datetime import datetime

from .pattern_extractor import PatternExtractor, AutomationDraft, Pattern

logger = logging.getLogger(__name__)


class RuleGenerator:
    def __init__(self, pattern_extractor: PatternExtractor,
                 min_confidence: float = 0.7,
                 rate_limit_seconds: float = 600):
        self.extractor = pattern_extractor
        self.min_confidence = min_confidence
        self.rate_limit_seconds = rate_limit_seconds
        self._last_generation_time = 0.0
        self._generated_names: set = set()

    def generate_from_patterns(self, min_count: int = 5) -> List[AutomationDraft]:
        patterns = self.extractor.find_recurring_patterns(min_count=min_count)
        drafts = []
        for p in patterns:
            if p.confidence < self.min_confidence:
                continue
            draft = self.extractor.suggest_automation(p)
            drafts.append(draft)
        return drafts

    def dry_run(self, draft: AutomationDraft, history_limit: int = 1000) -> dict:
        matches = 0
        total = 0
        samples = []

        stats = self.extractor.collector.get_stats()
        for session in stats.get('sessions', [])[:5]:
            frames = self.extractor.collector.get_frames(
                session_id=session['id'], limit=history_limit
            )
            for frame in frames:
                snap = frame.get('sensor_snapshot')
                if not snap:
                    continue
                total += 1
                if self._conditions_match(draft.conditions, snap):
                    matches += 1
                    if len(samples) < 10:
                        samples.append({
                            'timestamp': frame.get('timestamp'),
                            'actual_command': frame.get('command'),
                            'expected_action': draft.actions[0]['value'],
                        })

        return {
            'draft_name': draft.name,
            'matches': matches,
            'total_frames': total,
            'match_rate': round(matches / max(1, total), 3),
            'samples': samples,
        }

    def apply_draft(self, draft: AutomationDraft) -> Optional[dict]:
        now = time.time()
        if now - self._last_generation_time < self.rate_limit_seconds:
            logger.warning(f"Rate limited: wait {self.rate_limit_seconds - (now - self._last_generation_time):.0f}s")
            return None

        if draft.name in self._generated_names:
            logger.info(f"Duplicate rule skipped: {draft.name}")
            return None

        rule = {
            'name': draft.name,
            'enabled': True,
            'triggerType': draft.trigger_type,
            'conditionMatch': draft.condition_match,
            'conditions': draft.conditions,
            'actions': draft.actions,
            'source': draft.source,
            'aiGenerated': True,
            'confidence': draft.confidence,
            'createdAt': datetime.now().isoformat(),
        }

        self._last_generation_time = now
        self._generated_names.add(draft.name)
        return rule

    def _conditions_match(self, conditions: list, sensor_snapshot: dict) -> bool:
        for c in conditions:
            val = sensor_snapshot.get(c['feed'])
            if val is None:
                return False

            if c['op'] == 'lt' and val >= c['threshold']:
                return False
            elif c['op'] == 'gt' and val <= c['threshold']:
                return False
            elif c['op'] == 'eq' and abs(val - c['threshold']) > 0.001:
                return False
        return True
