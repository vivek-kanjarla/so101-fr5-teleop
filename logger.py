"""
logger.py — saves teleoperation episodes to CSV files.
"""

import os
import time
import pandas as pd
from config import LOG_DIR


class EpisodeLogger:
    def __init__(self):
        self._rows: list[dict] = []
        self._episode_id = int(time.time())
        self._recording = False

    @property
    def recording(self) -> bool:
        return self._recording

    def start(self):
        self._rows = []
        self._episode_id = int(time.time())
        self._recording = True

    def stop(self) -> str | None:
        """Stop recording and flush to disk. Returns file path or None."""
        self._recording = False
        if not self._rows:
            return None
        return self._flush()

    def log(self, timestamp: float, so101: dict[str, float], fr5: list[float]):
        if not self._recording:
            return
        row = {"timestamp": timestamp}
        for k, v in so101.items():
            row[f"so101_{k}"] = v
        for i, v in enumerate(fr5, start=1):
            row[f"fr5_j{i}"] = v
        self._rows.append(row)

    def _flush(self) -> str:
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"episode_{self._episode_id}.csv")
        pd.DataFrame(self._rows).to_csv(path, index=False)
        return path
