"""
Automation API
Flask blueprint for IFTTT automation engine REST API.
"""

from flask import Blueprint, request, jsonify

import logging
import math
import time
from automation_engine import _run_async

try:
    import sys, os
    _scripts_dir = os.path.join(os.path.dirname(__file__))
    if _scripts_dir not in sys.path:
        sys.path.insert(0, _scripts_dir)
    from ml.pattern_extractor import PatternExtractor, AutomationDraft
    from ml.rule_generator import RuleGenerator
    ML_AVAILABLE = True
except ImportError:
    PatternExtractor = None
    AutomationDraft = None
    RuleGenerator = None
    ML_AVAILABLE = False

logger = logging.getLogger(__name__)

automation_bp = Blueprint('automation', __name__, url_prefix='/api')

# Global reference to automation engine (set during initialization)
_automation_engine = None


def init_automation_api(engine):
    """Initialize the automation API with the engine instance."""
    global _automation_engine
    _automation_engine = engine


def _get_engine():
    """Get the automation engine instance."""
    if _automation_engine is None:
        return None
    return _automation_engine


@automation_bp.route('/automations', methods=['GET'])
def list_automations():
    """List all automations."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        automations = _run_async(engine.db.automation.find_many(order={'createdAt': 'desc'}))
        result = []
        for a in automations:
            conditions = _run_async(engine.db.automationcondition.find_many(
                where={'automationId': a.id}
            ))
            actions = _run_async(engine.db.automationaction.find_many(
                where={'automationId': a.id},
                order={'actionOrder': 'asc'}
            ))
            result.append({
                'id': a.id,
                'name': a.name,
                'isActive': a.isActive,
                'triggerType': a.triggerType,
                'conditionMatch': a.conditionMatch,
                'elseConditionMatch': a.elseConditionMatch,
                'scheduleCron': a.scheduleCron,
                'createdAt': a.createdAt.isoformat() if a.createdAt else None,
                'updatedAt': a.updatedAt.isoformat() if a.updatedAt else None,
                'conditions': [
                    {
                        'id': c.id,
                        'feedKey': c.feedKey,
                        'operator': c.operator,
                        'value': c.value,
                        'isElse': c.isElse
                    } for c in conditions
                ],
                'actions': [
                    {
                        'id': act.id,
                        'actionType': act.actionType,
                        'actionValue': act.actionValue,
                        'delayMs': act.delayMs,
                        'webhookUrl': act.webhookUrl,
                        'triggerId': act.triggerId,
                        'actionOrder': act.actionOrder,
                        'isElse': act.isElse
                    } for act in actions
                ]
            })
        return jsonify({'automations': result})
    except Exception as e:
        logger.error(f"Error listing automations: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automations', methods=['POST'])
def create_automation():
    """Create a new automation."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        data = request.get_json()
        if not data:
            return jsonify({'error': 'No data provided'}), 400

        name = data.get('name')
        if not name:
            return jsonify({'error': 'Name is required'}), 400

        # Create automation
        automation = _run_async(engine.db.automation.create(
            data={
                'name': name,
                'isActive': data.get('isActive', True),
                'triggerType': data.get('triggerType', 'sensor'),
                'conditionMatch': data.get('conditionMatch', 'ALL'),
                'elseConditionMatch': data.get('elseConditionMatch', 'ALL'),
                'scheduleCron': data.get('scheduleCron')
            }
        ))

        # Create conditions
        conditions = data.get('conditions', [])
        for cond in conditions:
            _run_async(engine.db.automationcondition.create(
                data={
                    'automationId': automation.id,
                    'feedKey': cond['feedKey'],
                    'operator': cond['operator'],
                    'value': str(cond['value']),
                    'isElse': cond.get('isElse', False)
                }
            ))

        # Create actions
        actions = data.get('actions', [])
        for act in actions:
            _run_async(engine.db.automationaction.create(
                data={
                    'automationId': automation.id,
                    'actionType': act['actionType'],
                    'actionValue': act.get('actionValue'),
                    'delayMs': act.get('delayMs', 0),
                    'webhookUrl': act.get('webhookUrl'),
                    'triggerId': act.get('triggerId'),
                    'actionOrder': act.get('actionOrder', 0),
                    'isElse': act.get('isElse', False)
                }
            ))

        return jsonify({
            'id': automation.id,
            'name': automation.name,
            'message': 'Automation created'
        }), 201
    except Exception as e:
        logger.error(f"Error creating automation: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automations/<int:automation_id>', methods=['GET'])
def get_automation(automation_id):
    """Get a specific automation."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        automation = _run_async(engine.db.automation.find_unique(where={'id': automation_id}))
        if not automation:
            return jsonify({'error': 'Automation not found'}), 404

        conditions = _run_async(engine.db.automationcondition.find_many(
            where={'automationId': automation.id}
        ))
        actions = _run_async(engine.db.automationaction.find_many(
            where={'automationId': automation.id},
            order={'actionOrder': 'asc'}
        ))

        return jsonify({
            'id': automation.id,
            'name': automation.name,
            'isActive': automation.isActive,
            'triggerType': automation.triggerType,
            'conditionMatch': automation.conditionMatch,
            'elseConditionMatch': automation.elseConditionMatch,
            'scheduleCron': automation.scheduleCron,
            'createdAt': automation.createdAt.isoformat() if automation.createdAt else None,
            'updatedAt': automation.updatedAt.isoformat() if automation.updatedAt else None,
            'conditions': [
                {
                    'id': c.id,
                    'feedKey': c.feedKey,
                    'operator': c.operator,
                    'value': c.value,
                    'isElse': c.isElse
                } for c in conditions
            ],
            'actions': [
                {
                    'id': a.id,
                    'actionType': a.actionType,
                    'actionValue': a.actionValue,
                    'delayMs': a.delayMs,
                    'webhookUrl': a.webhookUrl,
                    'triggerId': a.triggerId,
                    'actionOrder': a.actionOrder,
                    'isElse': a.isElse
                } for a in actions
            ]
        })
    except Exception as e:
        logger.error(f"Error getting automation {automation_id}: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automations/<int:automation_id>', methods=['PUT'])
def update_automation(automation_id):
    """Update an automation."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        automation = _run_async(engine.db.automation.find_unique(where={'id': automation_id}))
        if not automation:
            return jsonify({'error': 'Automation not found'}), 404

        data = request.get_json()
        if not data:
            return jsonify({'error': 'No data provided'}), 400

        # Update automation
        update_data = {}
        if 'name' in data:
            update_data['name'] = data['name']
        if 'isActive' in data:
            update_data['isActive'] = data['isActive']
        if 'triggerType' in data:
            update_data['triggerType'] = data['triggerType']
        if 'conditionMatch' in data:
            update_data['conditionMatch'] = data['conditionMatch']
        if 'elseConditionMatch' in data:
            update_data['elseConditionMatch'] = data['elseConditionMatch']
        if 'scheduleCron' in data:
            update_data['scheduleCron'] = data['scheduleCron']

        if update_data:
            _run_async(engine.db.automation.update(
                where={'id': automation_id},
                data=update_data
            ))

        # Replace conditions if provided
        if 'conditions' in data:
            # Delete existing conditions
            _run_async(engine.db.automationcondition.delete_many(
                where={'automationId': automation_id}
            ))
            # Create new conditions
            for cond in data['conditions']:
                _run_async(engine.db.automationcondition.create(
                    data={
                        'automationId': automation_id,
                        'feedKey': cond['feedKey'],
                        'operator': cond['operator'],
                        'value': str(cond['value']),
                        'isElse': cond.get('isElse', False)
                    }
                ))

        # Replace actions if provided
        if 'actions' in data:
            # Delete existing actions
            _run_async(engine.db.automationaction.delete_many(
                where={'automationId': automation_id}
            ))
            # Create new actions
            for act in data['actions']:
                _run_async(engine.db.automationaction.create(
                    data={
                        'automationId': automation_id,
                        'actionType': act['actionType'],
                        'actionValue': act.get('actionValue'),
                        'delayMs': act.get('delayMs', 0),
                        'webhookUrl': act.get('webhookUrl'),
                        'triggerId': act.get('triggerId'),
                        'actionOrder': act.get('actionOrder', 0),
                        'isElse': act.get('isElse', False)
                    }
                ))

        return jsonify({'message': 'Automation updated'})
    except Exception as e:
        logger.error(f"Error updating automation {automation_id}: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automations/<int:automation_id>', methods=['DELETE'])
def delete_automation(automation_id):
    """Delete an automation."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        automation = _run_async(engine.db.automation.find_unique(where={'id': automation_id}))
        if not automation:
            return jsonify({'error': 'Automation not found'}), 404

        # Delete conditions and actions first (cascade should handle this, but be explicit)
        _run_async(engine.db.automationcondition.delete_many(where={'automationId': automation_id}))
        _run_async(engine.db.automationaction.delete_many(where={'automationId': automation_id}))
        _run_async(engine.db.automation.delete(where={'id': automation_id}))

        return jsonify({'message': 'Automation deleted'})
    except Exception as e:
        logger.error(f"Error deleting automation {automation_id}: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automations/<int:automation_id>/toggle', methods=['POST'])
def toggle_automation(automation_id):
    """Toggle an automation's active state."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        automation = _run_async(engine.db.automation.find_unique(where={'id': automation_id}))
        if not automation:
            return jsonify({'error': 'Automation not found'}), 404

        new_state = not automation.isActive
        _run_async(engine.db.automation.update(
            where={'id': automation_id},
            data={'isActive': new_state}
        ))

        return jsonify({
            'id': automation_id,
            'isActive': new_state,
            'message': f"Automation {'enabled' if new_state else 'disabled'}"
        })
    except Exception as e:
        logger.error(f"Error toggling automation {automation_id}: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automations/<int:automation_id>/run', methods=['POST'])
def run_automation(automation_id):
    """Manually trigger an automation."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        result = engine.trigger_manual(automation_id)
        return jsonify(result)
    except Exception as e:
        logger.error(f"Error running automation {automation_id}: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automations/<int:automation_id>/logs', methods=['GET'])
def get_automation_logs(automation_id):
    """Get execution logs for an automation."""
    engine = _get_engine()
    if not engine or not engine.db:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        limit = request.args.get('limit', 50, type=int)
        offset = request.args.get('offset', 0, type=int)

        logs = _run_async(engine.db.automationlog.find_many(
            where={'automationId': automation_id},
            order={'timestamp': 'desc'},
            take=limit,
            skip=offset
        ))

        total = _run_async(engine.db.automationlog.count(
            where={'automationId': automation_id}
        ))

        return jsonify({
            'logs': [
                {
                    'id': log.id,
                    'automationName': log.automationName,
                    'triggerReason': log.triggerReason,
                    'actionsExecuted': log.actionsExecuted,
                    'timestamp': log.timestamp.isoformat() if log.timestamp else None
                } for log in logs
            ],
            'total': total,
            'limit': limit,
            'offset': offset
        })
    except Exception as e:
        logger.error(f"Error getting logs for automation {automation_id}: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automation-feeds', methods=['GET'])
def list_feeds():
    """List available sensor feeds."""
    engine = _get_engine()
    if not engine:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        feeds = engine.get_available_feeds()
        return jsonify({'feeds': feeds})
    except Exception as e:
        logger.error(f"Error listing feeds: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automation-feeds/<key>/value', methods=['GET'])
def get_feed_value(key):
    """Get current value of a sensor feed."""
    engine = _get_engine()
    if not engine:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        value = engine.get_cached_sensor_value(key)
        return jsonify({
            'key': key,
            'value': value,
            'timestamp': time.time()
        })
    except Exception as e:
        logger.error(f"Error getting feed value for {key}: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/automation-feeds/all', methods=['GET'])
def get_all_feed_values():
    """Get current values of all sensor feeds."""
    engine = _get_engine()
    if not engine:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        values = engine.get_all_feed_values()
        return jsonify({
            'feeds': values,
            'timestamp': time.time()
        })
    except Exception as e:
        logger.error(f"Error getting all feed values: {e}")
        return jsonify({'error': str(e)}), 500


@automation_bp.route('/actions', methods=['GET'])
def list_actions():
    """List available action types."""
    engine = _get_engine()
    if not engine:
        return jsonify({'error': 'Automation engine not initialized'}), 503

    try:
        actions = engine.get_available_actions()
        return jsonify({'actions': actions})
    except Exception as e:
        logger.error(f"Error listing actions: {e}")
        return jsonify({'error': str(e)}), 500


# ============================================================
# AI Decision Engine API
# ============================================================

ai_bp = Blueprint('ai', __name__, url_prefix='/api/ai')
_ai_engine = None


def init_ai_api(engine):
    """Set the AI decision engine reference."""
    global _ai_engine
    _ai_engine = engine


@ai_bp.route('/start', methods=['POST'])
def ai_start():
    """Start the AI decision loop."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    data = request.get_json(silent=True) or {}
    result = _ai_engine.start(
        task_goal=data.get('goal', ''),
        interval=data.get('interval'),
    )
    return jsonify(result)


@ai_bp.route('/stop', methods=['POST'])
def ai_stop():
    """Stop the AI decision loop."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    result = _ai_engine.stop()
    return jsonify(result)


@ai_bp.route('/status', methods=['GET'])
def ai_status():
    """Get AI engine status."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    return jsonify(_ai_engine.get_status())


@ai_bp.route('/analyze', methods=['POST'])
def ai_analyze():
    """Run a single AI analysis cycle."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    data = request.get_json(silent=True) or {}
    result = _ai_engine.analyze_once(task_goal=data.get('goal'))
    return jsonify(result)


@ai_bp.route('/decisions', methods=['GET'])
def ai_decisions():
    """Get decision history."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    limit = request.args.get('limit', 20, type=int)
    return jsonify({'decisions': _ai_engine.get_decisions(limit)})


@ai_bp.route('/guidance', methods=['POST'])
def ai_guidance():
    """Send human guidance to the AI engine."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    data = request.get_json(silent=True) or {}
    guidance = data.get('guidance', '')
    _ai_engine.set_human_guidance(guidance)
    return jsonify({'success': True, 'guidance': guidance[:200]})


@ai_bp.route('/camera/snapshot', methods=['GET'])
def ai_camera_snapshot():
    """Get current camera frame as JPEG."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    frame = _ai_engine.camera.capture_frame()
    if frame is None:
        return jsonify({'error': 'Camera not available', 'state': _ai_engine.camera.get_state()}), 503
    from flask import Response
    return Response(frame, mimetype='image/jpeg')


@ai_bp.route('/config', methods=['POST'])
def ai_config():
    """Update AI engine settings."""
    if not _ai_engine:
        return jsonify({'error': 'AI engine not initialized'}), 503
    data = request.get_json(silent=True) or {}
    if 'interval' in data:
        _ai_engine.loop_interval = max(1.0, float(data['interval']))
    if 'backend' in data:
        _ai_engine.backend = data['backend']
    if 'model' in data:
        _ai_engine.api_model = data['model']
    if 'goal' in data:
        _ai_engine.task_goal = data['goal']
    return jsonify({'success': True, 'status': _ai_engine.get_status()})


# ============================================================
# Waypoint Memory API
# ============================================================

waypoint_bp = Blueprint('waypoints', __name__, url_prefix='/api/waypoints')
_waypoint_memory = None


def init_waypoint_api(memory):
    """Set the waypoint memory reference."""
    global _waypoint_memory
    _waypoint_memory = memory


@waypoint_bp.route('/paths', methods=['GET'])
def list_paths():
    """List all saved paths."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    return jsonify({'paths': _waypoint_memory.list_paths()})


@waypoint_bp.route('/paths', methods=['POST'])
def create_path():
    """Start recording a new path."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    data = request.get_json(silent=True) or {}
    name = data.get('name')
    if not name:
        return jsonify({'error': 'Name is required'}), 400
    result = _waypoint_memory.start_recording(name, data.get('description'))
    return jsonify(result)


@waypoint_bp.route('/paths/<int:path_id>', methods=['GET'])
def get_path(path_id):
    """Get a path with all waypoints."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    path = _waypoint_memory.get_path(path_id)
    if not path:
        return jsonify({'error': 'Path not found'}), 404
    return jsonify(path)


@waypoint_bp.route('/paths/<int:path_id>', methods=['DELETE'])
def delete_path(path_id):
    """Delete a path."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    result = _waypoint_memory.delete_path(path_id)
    return jsonify(result)


@waypoint_bp.route('/record', methods=['POST'])
def record_waypoint():
    """Record a single waypoint at current position."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    data = request.get_json(silent=True) or {}
    result = _waypoint_memory.record_waypoint(actions=data.get('actions'))
    return jsonify(result)


@waypoint_bp.route('/stop', methods=['POST'])
def stop_recording():
    """Stop recording."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    result = _waypoint_memory.stop_recording()
    return jsonify(result)


@waypoint_bp.route('/replay/<int:path_id>', methods=['POST'])
def start_replay(path_id):
    """Start replaying a saved path."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    result = _waypoint_memory.start_replay(path_id)
    return jsonify(result)


@waypoint_bp.route('/replay/stop', methods=['POST'])
def stop_replay():
    """Stop the current replay."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    _waypoint_memory.stop_replay()
    return jsonify({'success': True})


@waypoint_bp.route('/paths/<int:path_id>/waypoints', methods=['POST'])
def add_waypoints_to_path(path_id):
    """Add waypoints with explicit coordinates to an existing path."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    data = request.get_json(silent=True) or {}
    wps = data.get('waypoints', [])
    if not wps:
        return jsonify({'error': 'No waypoints provided'}), 400
    result = _waypoint_memory.add_waypoints(path_id, wps)
    return jsonify(result)


@waypoint_bp.route('/status', methods=['GET'])
def waypoint_status():
    """Get recording/replay status."""
    if not _waypoint_memory:
        return jsonify({'error': 'Waypoint memory not initialized'}), 503
    return jsonify({
        'recording': _waypoint_memory.recording,
        'current_path_id': _waypoint_memory.current_path_id,
        'replaying': _waypoint_memory.replaying,
        'replay_index': _waypoint_memory.replay_index,
    })


# ============================================================
# ML / Training API
# ============================================================

ml_bp = Blueprint('ml', __name__, url_prefix='/api/ml')
_ml_engine = None
_ml_collector = None
_ml_pattern_extractor = None
_ml_rule_generator = None


def init_ml_api(ai_engine):
    """Set ML API references from the AI engine's offline engine."""
    global _ml_engine, _ml_collector, _ml_pattern_extractor, _ml_rule_generator
    if not ML_AVAILABLE:
        return
    if ai_engine and hasattr(ai_engine, 'offline_engine') and ai_engine.offline_engine:
        _ml_engine = ai_engine.offline_engine
        _ml_collector = _ml_engine.collector
        _ml_pattern_extractor = PatternExtractor(_ml_collector)
        _ml_rule_generator = RuleGenerator(_ml_pattern_extractor)


# --- Model ---

@ml_bp.route('/model/status', methods=['GET'])
def ml_model_status():
    """Get ML model status."""
    if not _ml_engine:
        return jsonify({'loaded': False, 'error': 'Not initialized'})
    return jsonify(_ml_engine.get_model_info())


@ml_bp.route('/model/predict', methods=['POST'])
def ml_model_predict():
    """Run a single MLP prediction with given sensor data."""
    if not _ml_engine or not _ml_engine.model_loaded:
        return jsonify({'error': 'Model not loaded'}), 503
    data = request.get_json(silent=True) or {}
    sensor_data = data.get('sensor_data', {})
    camera_frame = data.get('camera_frame')
    result = _ml_engine.analyze(
        sensor_data=sensor_data,
        camera_frame=camera_frame,
        task_goal=data.get('task_goal', ''),
    )
    return jsonify(result)


@ml_bp.route('/model/load', methods=['POST'])
def ml_model_load():
    """Load a specific model by name."""
    if not _ml_engine:
        return jsonify({'error': 'Not initialized'}), 503
    data = request.get_json(silent=True) or {}
    data.get('name', 'mlp_decision')
    _ml_engine.load_model(
        model_path=data.get('model_path'),
        config_path=data.get('config_path'),
    )
    return jsonify({'success': True, ** _ml_engine.get_model_info()})


@ml_bp.route('/model/export', methods=['POST'])
def ml_model_export():
    """Export model in specified format (pt, onnx)."""
    if not _ml_engine:
        return jsonify({'error': 'Not initialized'}), 503
    data = request.get_json(silent=True) or {}
    fmt = data.get('format', 'onnx')
    if fmt == 'onnx':
        path = _ml_engine.model_path.replace('.pt', '.onnx')
        _ml_engine.model.export_onnx(path)
        return jsonify({'success': True, 'path': path, 'format': fmt})
    return jsonify({'error': f'Unsupported format: {fmt}'}), 400


# --- Training Data ---

@ml_bp.route('/data/stats', methods=['GET'])
def ml_data_stats():
    """Get training data statistics."""
    if not _ml_collector:
        return jsonify({'error': 'Collector not initialized'}), 503
    return jsonify(_ml_collector.get_stats())


@ml_bp.route('/data/export', methods=['GET'])
def ml_data_export():
    """Export training data as JSON."""
    if not _ml_collector:
        return jsonify({'error': 'Collector not initialized'}), 503
    session_id = request.args.get('session_id')
    limit = request.args.get('limit', 1000, type=int)
    frames = _ml_collector.get_frames(session_id=session_id, limit=limit)
    return jsonify({'frames': frames, 'count': len(frames)})


@ml_bp.route('/data/session/<session_id>', methods=['DELETE'])
def ml_data_delete_session(session_id):
    """Delete a training session."""
    if not _ml_collector:
        return jsonify({'error': 'Collector not initialized'}), 503
    _ml_collector.delete_session(session_id)
    return jsonify({'success': True})


# --- Patterns & Rules ---

@ml_bp.route('/patterns', methods=['GET'])
def ml_patterns():
    """Get recurring patterns from training data."""
    if not _ml_pattern_extractor:
        return jsonify({'error': 'Initializing'}), 503
    min_count = request.args.get('min_count', 5, type=int)
    patterns = _ml_pattern_extractor.find_recurring_patterns(min_count=min_count)
    return jsonify({
        'patterns': [p.to_dict() for p in patterns],
        'count': len(patterns),
    })


@ml_bp.route('/rules/generate', methods=['POST'])
def ml_rules_generate():
    """Generate automation drafts from patterns."""
    if not _ml_rule_generator:
        return jsonify({'error': 'Initializing'}), 503
    data = request.get_json(silent=True) or {}
    min_count = data.get('min_count', 5)
    drafts = _ml_rule_generator.generate_from_patterns(min_count=min_count)
    return jsonify({
        'drafts': [d.to_dict() for d in drafts],
        'count': len(drafts),
    })


@ml_bp.route('/rules/dry-run', methods=['POST'])
def ml_rules_dry_run():
    """Dry-run a specific draft against historical data."""
    if not _ml_rule_generator:
        return jsonify({'error': 'Initializing'}), 503
    data = request.get_json(silent=True) or {}
    from ml.pattern_extractor import AutomationDraft
    draft = AutomationDraft(
        name=data.get('name', 'draft'),
        trigger_type=data.get('triggerType', 'sensor'),
        condition_match=data.get('conditionMatch', 'ALL'),
        conditions=data.get('conditions', []),
        actions=data.get('actions', []),
        source='api',
        confidence=data.get('confidence', 0.7),
    )
    return jsonify(_ml_rule_generator.dry_run(draft))


@ml_bp.route('/rules/apply', methods=['POST'])
def ml_rules_apply():
    """Apply a draft as a real automation rule."""
    if not _ml_rule_generator or not _automation_engine:
        return jsonify({'error': 'Not initialized'}), 503
    data = request.get_json(silent=True) or {}
    from ml.pattern_extractor import AutomationDraft
    draft = AutomationDraft(
        name=data.get('name', 'draft'),
        trigger_type=data.get('triggerType', 'sensor'),
        condition_match=data.get('conditionMatch', 'ALL'),
        conditions=data.get('conditions', []),
        actions=data.get('actions', []),
        source='api',
        confidence=data.get('confidence', 0.7),
    )
    rule = _ml_rule_generator.apply_draft(draft)
    if not rule:
        return jsonify({'error': 'Rule rejected (duplicate or rate limited)'}), 409

    try:
        created = _run_async(_automation_engine.db.automation.create(data={
            'name': rule['name'],
            'isActive': rule['enabled'],
            'triggerType': rule['triggerType'],
            'conditionMatch': rule['conditionMatch'],
            'aiGenerated': rule['aiGenerated'],
        }))
        for cond in rule['conditions']:
            _run_async(_automation_engine.db.automationcondition.create(data={
                'automationId': created.id,
                'feedName': cond['feed'],
                'operator': cond['op'],
                'threshold': cond['threshold'],
            }))
        for act in rule['actions']:
            _run_async(_automation_engine.db.automationaction.create(data={
                'automationId': created.id,
                'actionType': act.get('type', 'move'),
                'actionValue': act.get('value', ''),
                'actionOrder': 0,
            }))
        _automation_engine.reload_rules()
        return jsonify({'success': True, 'id': created.id, 'name': rule['name']})
    except Exception as e:
        logger.error(f"Failed to apply rule: {e}")
        return jsonify({'error': str(e)}), 500


# --- Training ---

_sim_trainer_instance = None


@ml_bp.route('/train/start', methods=['POST'])
def ml_train_start():
    """Start simulation training in background thread."""
    global _sim_trainer_instance
    try:
        from ml.sim_trainer import SimulationTrainer
        from ml.mlp_model import MLPDecisionModel
        from ml.training_collector import TrainingCollector

        data = request.get_json(silent=True) or {}
        model = MLPDecisionModel()
        collector = TrainingCollector()
        trainer = SimulationTrainer(model, collector)

        scenario = data.get('scenario', 'obstacle_course')
        episodes = data.get('episodes', 50)
        result = trainer.start_training(scenario, episodes)
        if result.get('success'):
            _sim_trainer_instance = trainer
        return jsonify(result)
    except Exception as e:
        logger.error(f"Failed to start training: {e}")
        return jsonify({'error': str(e)}), 500


@ml_bp.route('/train/stop', methods=['POST'])
def ml_train_stop():
    """Stop running simulation training."""
    global _sim_trainer_instance
    if _sim_trainer_instance:
        _sim_trainer_instance.stop_training()
        _sim_trainer_instance = None
        return jsonify({'success': True})
    return jsonify({'error': 'No training running'}), 400


@ml_bp.route('/train/status', methods=['GET'])
def ml_train_status():
    """Get simulation training status."""
    global _sim_trainer_instance
    if _sim_trainer_instance:
        return jsonify(_sim_trainer_instance.get_stats())
    return jsonify({'running': False})


# --- Simulation State ---

@ml_bp.route('/sim/state', methods=['GET'])
def ml_sim_state():
    """Get SimulationEngine physics state (position, obstacles, sensors)."""
    try:
        from simulation_engine import get_engine
        sim = get_engine()
        state = sim.get_state()
        sensors = sim.get_sensors()
        return jsonify({'state': state, 'sensors': sensors})
    except ImportError:
        return jsonify({'error': 'SimulationEngine not available'}), 503


@ml_bp.route('/sim/reset', methods=['POST'])
def ml_sim_reset():
    """Reset SimulationEngine to initial state."""
    data = request.get_json(silent=True) or {}
    try:
        from simulation_engine import reset_engine
        reset_engine(data.get('obstacles'))
        return jsonify({'success': True})
    except ImportError:
        return jsonify({'error': 'SimulationEngine not available'}), 503


@ml_bp.route('/sim/step', methods=['POST'])
def ml_sim_step():
    """Run a single command step through SimulationEngine."""
    data = request.get_json(silent=True) or {}
    cmd = data.get('command', 's')
    dt = data.get('dt', 1.0/30.0)
    try:
        from simulation_engine import get_engine
        sim = get_engine()
        sim.step(cmd, dt)
        sensors = sim.get_sensors()
        return jsonify({'sensors': sensors, 'state': sim.get_state()})
    except ImportError:
        return jsonify({'error': 'SimulationEngine not available'}), 503


@ml_bp.route('/sim/obstacles', methods=['POST'])
def ml_sim_obstacles():
    """Set obstacles for SimulationEngine (synced from map)."""
    data = request.get_json(silent=True) or {}
    obstacles = data.get('obstacles', [])
    try:
        from simulation_engine import get_engine
        sim = get_engine()
        sim.set_obstacles(obstacles)
        return jsonify({'success': True, 'count': len(obstacles)})
    except ImportError:
        return jsonify({'error': 'SimulationEngine not available'}), 503


@ml_bp.route('/sim/calibrate', methods=['POST'])
def ml_sim_calibrate():
    """Calibrate SimulationEngine heading (zero current heading)."""
    try:
        from simulation_engine import get_engine
        sim = get_engine()
        sim.calibrate_heading()
        return jsonify({'success': True})
    except ImportError:
        return jsonify({'error': 'SimulationEngine not available'}), 503


# --- Navigation Goal ---

_nav_goal = None


@ml_bp.route('/navigate/goal', methods=['POST'])
def ml_nav_set_goal():
    """Set a navigation goal for the robot to navigate toward."""
    global _nav_goal
    data = request.get_json(silent=True) or {}
    x = data.get('x')
    y = data.get('y')
    if x is None or y is None:
        return jsonify({'error': 'x and y required'}), 400
    _nav_goal = {
        'x': float(x),
        'y': float(y),
        'set_at': time.time(),
        'reached': False,
        'label': data.get('label', ''),
    }
    try:
        from simulation_engine import get_engine
        sim = get_engine()
        state = sim.get_state()
        dx = float(x) - state['x']
        dy = float(y) - state['y']
        _nav_goal['distance'] = round(math.sqrt(dx*dx + dy*dy), 2)
        _nav_goal['heading_to_goal'] = round(math.degrees(math.atan2(dy, dx)), 1)
        sim.navigate_to(float(x), float(y))
    except Exception:
        _nav_goal['distance'] = 0
        _nav_goal['heading_to_goal'] = 0
    logger.info(f"Navigation goal set: ({x}, {y}), dist={_nav_goal['distance']}m")
    return jsonify({'success': True, 'goal': _nav_goal})


@ml_bp.route('/navigate/goal', methods=['GET'])
def ml_nav_get_goal():
    """Get current navigation goal and progress."""
    global _nav_goal
    if not _nav_goal:
        return jsonify({'goal': None})
    try:
        from simulation_engine import get_engine
        sim = get_engine()
        state = sim.get_state()
        dx = _nav_goal['x'] - state['x']
        dy = _nav_goal['y'] - state['y']
        remaining = round(math.sqrt(dx*dx + dy*dy), 2)
        start_dist = _nav_goal.get('distance', remaining)
        progress = round(max(0, min(100, (1 - remaining / max(start_dist, 0.01)) * 100)), 1)
        _nav_goal['remaining_distance'] = remaining
        _nav_goal['progress_pct'] = progress
        _nav_goal['heading_to_goal'] = round(math.degrees(math.atan2(dy, dx)), 1)
        _nav_goal['robot_position'] = {'x': round(state['x'], 3), 'y': round(state['y'], 3)}
        if remaining < 0.3 and not _nav_goal.get('reached'):
            _nav_goal['reached'] = True
            _nav_goal['reached_at'] = time.time()
            logger.info("Navigation goal REACHED!")
    except Exception:
        pass
    return jsonify({'goal': _nav_goal})


@ml_bp.route('/navigate/goal', methods=['DELETE'])
def ml_nav_clear_goal():
    """Clear the current navigation goal."""
    global _nav_goal
    _nav_goal = None
    return jsonify({'success': True})
