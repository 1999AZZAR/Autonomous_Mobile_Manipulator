"""
Automation API
Flask blueprint for IFTTT automation engine REST API.
"""

from flask import Blueprint, request, jsonify
import logging
import time
from automation_engine import _run_async

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


@automation_bp.route('/feeds', methods=['GET'])
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


@automation_bp.route('/feeds/<key>/value', methods=['GET'])
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


@automation_bp.route('/feeds/all', methods=['GET'])
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
