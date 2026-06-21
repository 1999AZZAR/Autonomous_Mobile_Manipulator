import json
import os
import time
import logging
import threading
from typing import List, Optional, Dict, Any

logger = logging.getLogger(__name__)

DEFAULT_DIR = os.path.expanduser("~/.robot_waypoints")


class Waypoint:
    def __init__(
        self,
        x: float,
        y: float,
        heading: float = 0.0,
        actions: Optional[Dict[str, Any]] = None,
        label: str = "",
    ):
        self.x = x
        self.y = y
        self.heading = heading
        self.actions = actions or {}
        self.label = label

    def to_dict(self) -> Dict:
        return {
            "x": self.x,
            "y": self.y,
            "heading": self.heading,
            "actions": self.actions,
            "label": self.label,
        }

    @classmethod
    def from_dict(cls, d: Dict) -> "Waypoint":
        return cls(
            x=d["x"],
            y=d["y"],
            heading=d.get("heading", 0.0),
            actions=d.get("actions", {}),
            label=d.get("label", ""),
        )


class WaypointLibrary:
    def __init__(self, name: str, waypoints: Optional[List[Waypoint]] = None):
        self.name = name
        self.waypoints = waypoints or []
        self.created_at = time.time()
        self.updated_at = time.time()

    def add(self, wp: Waypoint):
        self.waypoints.append(wp)
        self.updated_at = time.time()

    def remove(self, index: int) -> bool:
        if 0 <= index < len(self.waypoints):
            self.waypoints.pop(index)
            self.updated_at = time.time()
            return True
        return False

    def update(self, index: int, wp: Waypoint) -> bool:
        if 0 <= index < len(self.waypoints):
            self.waypoints[index] = wp
            self.updated_at = time.time()
            return True
        return False

    def clear(self):
        self.waypoints.clear()
        self.updated_at = time.time()

    def to_dict(self) -> Dict:
        return {
            "name": self.name,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "count": len(self.waypoints),
        }

    @classmethod
    def from_dict(cls, d: Dict) -> "WaypointLibrary":
        lib = cls(
            name=d["name"],
            waypoints=[Waypoint.from_dict(wp) for wp in d.get("waypoints", [])],
        )
        lib.created_at = d.get("created_at", time.time())
        lib.updated_at = d.get("updated_at", time.time())
        return lib


class WaypointLibraryStore:
    def __init__(self, storage_dir: str = DEFAULT_DIR):
        self.storage_dir = storage_dir
        self._lock = threading.Lock()
        os.makedirs(self.storage_dir, exist_ok=True)

    def _path(self, name: str) -> str:
        safe = name.replace("/", "_").replace("\\", "_").replace(" ", "_")
        return os.path.join(self.storage_dir, f"{safe}.json")

    def save(self, library: WaypointLibrary) -> bool:
        try:
            with self._lock:
                path = self._path(library.name)
                with open(path, "w") as f:
                    json.dump(library.to_dict(), f, indent=2)
            logger.info(f"saved waypoint library '{library.name}' ({len(library.waypoints)} wps)")
            return True
        except Exception as e:
            logger.error(f"failed to save library '{library.name}': {e}")
            return False

    def load(self, name: str) -> Optional[WaypointLibrary]:
        try:
            path = self._path(name)
            if not os.path.exists(path):
                return None
            with self._lock:
                with open(path) as f:
                    data = json.load(f)
            return WaypointLibrary.from_dict(data)
        except Exception as e:
            logger.error(f"failed to load library '{name}': {e}")
            return None

    def delete(self, name: str) -> bool:
        try:
            path = self._path(name)
            if os.path.exists(path):
                os.remove(path)
                logger.info(f"deleted waypoint library '{name}'")
                return True
            return False
        except Exception as e:
            logger.error(f"failed to delete library '{name}': {e}")
            return False

    def list(self) -> List[Dict]:
        results = []
        try:
            os.makedirs(self.storage_dir, exist_ok=True)
            for fn in os.listdir(self.storage_dir):
                if fn.endswith(".json"):
                    name = fn[:-5]
                    path = os.path.join(self.storage_dir, fn)
                    try:
                        with open(path) as f:
                            data = json.load(f)
                        results.append({
                            "name": name,
                            "count": len(data.get("waypoints", [])),
                            "created_at": data.get("created_at", 0),
                            "updated_at": data.get("updated_at", 0),
                        })
                    except Exception:
                        results.append({"name": name, "count": 0})
        except Exception as e:
            logger.error(f"failed to list libraries: {e}")
        return sorted(results, key=lambda r: r.get("updated_at", 0), reverse=True)

    def export_to_json(self, name: str) -> Optional[str]:
        lib = self.load(name)
        if not lib:
            return None
        return json.dumps(lib.to_dict(), indent=2)

    def import_from_json(self, json_str: str) -> Optional[str]:
        try:
            data = json.loads(json_str)
            lib = WaypointLibrary.from_dict(data)
            if self.save(lib):
                return lib.name
            return None
        except Exception as e:
            logger.error(f"import failed: {e}")
            return None


_store: Optional[WaypointLibraryStore] = None


def get_store(storage_dir: str = DEFAULT_DIR) -> WaypointLibraryStore:
    global _store
    if _store is None:
        _store = WaypointLibraryStore(storage_dir)
    return _store
