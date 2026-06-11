import time
import json
import math
import random
import threading
import numpy as np
from typing import Optional, Callable, Dict, List
from dataclasses import dataclass

from .mlp_model import MLPDecisionModel, COMMANDS
from .feature_encoder import FeatureEncoder
from .training_collector import TrainingCollector
from .trainer import MLPTrainer
from .reward import RewardCalculator


@dataclass
class ScenarioDef:
    name: str
    description: str
    robot_start: dict
    obstacles: List[dict]
    goal_conditions: dict
    max_steps: int = 500


SCENARIOS = {
    'empty_room': ScenarioDef(
        name='empty_room',
        description='Open space, learn basic movement',
        robot_start={'x': 0, 'y': -2, 'heading': 0},
        obstacles=[],
        goal_conditions={'min_distance_traveled': 3000},
        max_steps=200,
    ),
    'obstacle_course': ScenarioDef(
        name='obstacle_course',
        description='Static obstacles, learn avoidance',
        robot_start={'x': 0, 'y': -2, 'heading': 0},
        obstacles=[
            {'x': 0.5, 'y': 0, 'w': 0.3, 'h': 0.3},
            {'x': -0.5, 'y': 1, 'w': 0.3, 'h': 0.3},
            {'x': 0.3, 'y': 2, 'w': 0.3, 'h': 0.3},
        ],
        goal_conditions={'min_distance_traveled': 5000, 'max_collisions': 5},
        max_steps=500,
    ),
    'free_explore': ScenarioDef(
        name='free_explore',
        description='Random obstacles, diverse behaviors',
        robot_start={'x': 0, 'y': -2, 'heading': 0},
        obstacles=[],
        goal_conditions={'min_distance_traveled': 8000},
        max_steps=1000,
    ),
}


class SimulationTrainer:
    def __init__(self, model: MLPDecisionModel,
                 collector: Optional[TrainingCollector] = None):
        self.model = model
        self.collector = collector or TrainingCollector()
        self.encoder = FeatureEncoder()
        self.reward_calc = RewardCalculator()
        self.trainer = MLPTrainer(model)

        self.running = False
        self.paused = False
        self._episode = 0
        self._step = 0
        self._total_steps = 0
        self._epsilon = 1.0
        self._best_reward = -float('inf')
        self._episode_rewards: List[float] = []
        self._history: List[Dict] = []

        self._state_callback: Optional[Callable] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

        self._sensor_before: dict = {}
        self._last_positions: list = []

    def set_state_callback(self, cb: Callable):
        self._state_callback = cb

    def start_training(self, scenario_name: str = 'obstacle_course',
                       episodes: int = 100, strategy: str = 'epsilon-greedy'):
        if self.running:
            return {'error': 'Already running'}

        scenario = SCENARIOS.get(scenario_name)
        if not scenario:
            return {'error': f'Unknown scenario: {scenario_name}'}

        self.running = True
        self.paused = False
        self._episode = 0
        self._epsilon = 1.0
        self._episode_rewards = []
        self._history = []
        self._session_id = self.collector.new_session(
            mode='simulation',
            notes=f'scenario={scenario_name},episodes={episodes}'
        )

        self._thread = threading.Thread(
            target=self._training_loop,
            args=(scenario, episodes, strategy),
            daemon=True,
        )
        self._thread.start()
        return {'success': True, 'session_id': self._session_id}

    def stop_training(self):
        self.running = False

    def pause_training(self):
        self.paused = True

    def resume_training(self):
        self.paused = False

    def get_stats(self) -> dict:
        with self._lock:
            return {
                'running': self.running,
                'paused': self.paused,
                'episode': self._episode,
                'step': self._step,
                'total_steps': self._total_steps,
                'epsilon': round(self._epsilon, 3),
                'best_episode_reward': round(self._best_reward, 3),
                'avg_reward_last_10': round(
                    np.mean(self._episode_rewards[-10:]), 3
                ) if self._episode_rewards else 0,
                'episodes_completed': len(self._episode_rewards),
                'history': self._history[-20:],
            }

    def _training_loop(self, scenario: ScenarioDef, episodes: int,
                       strategy: str):
        epsilon_start = 1.0
        epsilon_end = 0.05
        epsilon_decay = 0.95

        for ep in range(episodes):
            if not self.running:
                break

            self._episode = ep + 1
            self._step = 0
            total_reward = 0.0
            total_distance = 0.0
            collisions = 0
            self.encoder.reset()

            sim_state = {
                'x': scenario.robot_start.get('x', 0),
                'y': scenario.robot_start.get('y', -2),
                'heading': scenario.robot_start.get('heading', 0),
                'vx': 0, 'vy': 0, 'omega': 0,
            }

            if ep == 0:
                self._generate_scenario_obstacles(scenario)

            self._sensor_before = self._generate_mock_sensors(sim_state, scenario)

            for step in range(scenario.max_steps):
                if not self.running or self.paused:
                    time.sleep(0.01)
                    continue

                self._step = step + 1
                self._total_steps += 1

                feature_vector = self.encoder.encode(
                    self._sensor_before, [],
                    self._last_command if step > 0 else None
                )

                command = self._select_action(feature_vector, strategy)

                self._last_command = command

                sim_state = self._simulate_step(sim_state, command)

                self._sensor_before = self._generate_mock_sensors(sim_state, scenario)

                pos_delta = abs(sim_state.get('_last_dx', 0)) + abs(sim_state.get('_last_dy', 0))
                total_distance += pos_delta

                reward = self.reward_calc.compute(
                    self._sensor_before, self._sensor_before,
                    command, None, pos_delta
                )
                total_reward += reward

                if reward < -1.5:
                    collisions += 1

                self.collector.record_frame(
                    features=feature_vector,
                    command=command,
                    reward=reward,
                    confidence=1.0 - self._epsilon,
                    mode='simulation',
                    sensor_snapshot=self._sensor_before,
                    session_id=self._session_id,
                )

                if step % 10 == 0:
                    self._notify_state()

            self._episode_rewards.append(total_reward)
            if total_reward > self._best_reward:
                self._best_reward = total_reward
                self.model.save_model(self._get_model_path('mlp_decision_best.pt'))

            self._history.append({
                'episode': ep + 1,
                'reward': round(total_reward, 3),
                'steps': step + 1,
                'distance': round(total_distance, 1),
                'collisions': collisions,
                'epsilon': round(self._epsilon, 3),
            })

            if strategy == 'epsilon-greedy':
                self._epsilon = max(
                    epsilon_end,
                    self._epsilon * epsilon_decay
                )

            if ep > 0 and ep % 10 == 0:
                self._retrain_on_history()

        self.running = False
        self.model.save_model(self._get_model_path('mlp_decision.pt'))

    def _select_action(self, feature_vector: np.ndarray,
                       strategy: str) -> str:
        if strategy == 'epsilon-greedy':
            if random.random() < self._epsilon:
                return random.choice(COMMANDS)
            command, conf, _ = self.model.predict(feature_vector)
            return command
        return COMMANDS[0]

    def _simulate_step(self, state: dict, command: str) -> dict:
        cmd_vel = {
            'f': (0, 1), 'b': (0, -1),
            'q': (0.7, 0.7), 'e': (-0.7, 0.7),
            'z': (0.7, -0.7), 'x': (-0.7, -0.7),
            't': (0, 0), 'y': (0, 0),
            's': (0, 0),
        }
        vx, vy = cmd_vel.get(command, (0, 0))
        speed = 0.15
        omega = 0

        if command == 't':
            omega = 60
        elif command == 'y':
            omega = -60

        heading_rad = math.radians(state['heading'])
        dx = (vx * math.cos(heading_rad) - vy * math.sin(heading_rad)) * speed
        dy = (vx * math.sin(heading_rad) + vy * math.cos(heading_rad)) * speed

        new_state = state.copy()
        new_state['x'] += dx
        new_state['y'] += dy
        new_state['heading'] = (new_state['heading'] + omega * (1 / 30)) % 360
        new_state['vx'] = vx * speed
        new_state['vy'] = vy * speed
        new_state['omega'] = omega
        new_state['_last_dx'] = dx
        new_state['_last_dy'] = dy
        return new_state

    def _generate_mock_sensors(self, state: dict, scenario: ScenarioDef) -> dict:
        import math
        sensors = {}
        for name in ['laser_left_front', 'laser_left_back', 'laser_right_front',
                      'laser_right_back', 'laser_back_left', 'laser_back_right']:
            sensors[name] = 1500
        sensors['ultra_front_left'] = 4000
        sensors['ultra_front_right'] = 4000
        sensors['line_left'] = 0
        sensors['line_center'] = 1
        sensors['line_right'] = 0
        sensors['imu_heading'] = state['heading']
        sensors['imu_pitch'] = 0
        sensors['imu_roll'] = 0

        heading_rad = math.radians(state['heading'])
        for obs in scenario.obstacles:
            dx = (obs['x'] - state['x'])
            dy = (obs['y'] - state['y'])
            dist = math.sqrt(dx * dx + dy * dy) * 1000

            angle_to_obs = math.degrees(math.atan2(dy, dx))
            rel_angle = (angle_to_obs - state['heading'] + 360) % 360

            if rel_angle < 45 or rel_angle > 315:
                sensors['laser_left_front'] = min(sensors['laser_left_front'], dist)
            elif 45 <= rel_angle < 135:
                sensors['laser_left_back'] = min(sensors['laser_left_back'], dist)
            elif 135 <= rel_angle < 225:
                sensors['laser_right_front'] = min(sensors['laser_right_front'], dist)
            else:
                sensors['laser_right_back'] = min(sensors['laser_right_back'], dist)

        return sensors

    def _generate_scenario_obstacles(self, scenario: ScenarioDef):
        if scenario.name == 'free_explore':
            import random
            scenario.obstacles = []
            for _ in range(8):
                scenario.obstacles.append({
                    'x': random.uniform(-1.5, 1.5),
                    'y': random.uniform(-0.5, 3.0),
                    'w': random.uniform(0.2, 0.5),
                    'h': random.uniform(0.2, 0.5),
                })

    def _retrain_on_history(self):
        X, y = self.collector.get_features_matrix(session_id=self._session_id)
        if len(X) < 100:
            return
        self.trainer.train(X, y, epochs=5, batch_size=32, val_split=0.1)

    def _notify_state(self):
        if self._state_callback:
            try:
                self._state_callback(self.get_stats())
            except Exception:
                pass

    def _get_model_path(self, name: str) -> str:
        import os
        base = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'models')
        os.makedirs(base, exist_ok=True)
        return os.path.join(base, name)
