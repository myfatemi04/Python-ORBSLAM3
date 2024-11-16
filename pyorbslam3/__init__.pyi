"""
Type hints for the ORB-SLAM3 Python bindings
"""

from enum import Enum
from typing import List, Optional
import numpy as np


class VerbosityLevel(Enum):
    QUIET = 0
    NORMAL = 1
    VERBOSE = 2
    VERY_VERBOSE = 3
    DEBUG = 4


class Sensor(Enum):
    MONOCULAR = 0
    STEREO = 1
    RGBD = 2
    IMU_MONOCULAR = 3
    IMU_STEREO = 4
    IMU_RGBD = 5


class IMUPoint:
    def __init__(self, ax: float, ay: float, az: float, wx: float, wy: float, wz: float, t: float) -> None:
        ...


class System:
    def __init__(
        self,
        vocab_file: str,
        settings_file: str,
        sensor: Sensor,
        init_frame: int = 0,
        sequence: str = "",
    ) -> None:
        ...

    def track_stereo(
        self,
        left_img: np.ndarray,
        right_img: np.ndarray,
        timestamp: float,
        imu_meas: Optional[List[IMUPoint]] = None,
        filename: str = "",
    ) -> int:
        ...

    def track_rgbd(
        self,
        rgb_img: np.ndarray,
        depth_map: np.ndarray,
        timestamp: float,
        imu_meas: Optional[List[IMUPoint]] = None,
        filename: str = "",
    ) -> int:
        ...

    def track_monocular(
        self,
        img: np.ndarray,
        timestamp: float,
        imu_meas: Optional[List[IMUPoint]] = None,
        filename: str = "",
    ) -> int:
        ...

    def activate_localization_mode(self) -> None:
        ...

    def deactivate_localization_mode(self) -> None:
        ...

    def map_changed(self) -> bool:
        ...

    def reset(self) -> None:
        ...

    def reset_active_map(self) -> None:
        ...

    def shutdown(self) -> None:
        ...

    def is_shutdown(self) -> bool:
        ...

    def save_trajectory_tum(self, filename: str) -> None:
        ...

    def save_keyframe_trajectory_tum(self, filename: str) -> None:
        ...

    def save_trajectory_euroc(self, filename: str) -> None:
        ...

    def save_keyframe_trajectory_euroc(self, filename: str) -> None:
        ...

    def save_trajectory_kitti(self, filename: str) -> None:
        ...

    def get_tracking_state(self) -> int:
        ...

    def get_tracked_mappoints(self) -> List[int]:
        ...

    def get_tracked_keypoints(self) -> List[int]:
        ...

    def get_time_from_imu_init(self) -> float:
        ...

    def is_lost(self) -> bool:
        ...

    def is_finished(self) -> bool:
        ...

    def change_dataset(self, dataset_path: str) -> None:
        ...

    def get_image_scale(self) -> float:
        ...
