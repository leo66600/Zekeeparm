"""ChArUco board detection and pose estimation."""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from calibration.aruco_pose import MarkerPose


class ChArUcoDetector:
    """Detect a ChArUco board and estimate the board pose in camera frame.

    Args:
        squares_x: number of chessboard squares along the board X direction.
        squares_y: number of chessboard squares along the board Y direction.
        square_length_m: chessboard square side length in meters.
        marker_length_m: ArUco marker side length in meters.
        aruco_dict_id: OpenCV ArUco dictionary ID, e.g. DICT_4X4_50 = 0.
        min_corners: minimum interpolated ChArUco corners required for pose.
    """

    def __init__(
        self,
        squares_x: int,
        squares_y: int,
        square_length_m: float,
        marker_length_m: float,
        aruco_dict_id: int = 0,
        min_corners: int = 6,
    ) -> None:
        if squares_x < 2 or squares_y < 2:
            raise ValueError("ChArUco board must have at least 2x2 squares")
        if square_length_m <= 0.0 or marker_length_m <= 0.0:
            raise ValueError("ChArUco square and marker lengths must be positive")
        if marker_length_m >= square_length_m:
            raise ValueError("ChArUco marker length must be smaller than square length")

        self._min_corners = int(max(min_corners, 4))
        self._dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self._params = self._create_detector_params()
        self._board = self._create_board(
            int(squares_x),
            int(squares_y),
            float(square_length_m),
            float(marker_length_m),
            self._dict,
        )
        self._validate_dictionary_capacity()

    @property
    def board(self):
        return self._board

    @staticmethod
    def _get_chessboard_corners(board) -> np.ndarray:
        if hasattr(board, "getChessboardCorners"):
            corners = board.getChessboardCorners()
        else:
            corners = board.chessboardCorners
        return np.asarray(corners, dtype=np.float64)

    @staticmethod
    def _create_detector_params():
        if hasattr(cv2.aruco, "DetectorParameters"):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    @staticmethod
    def _create_board(
        squares_x: int,
        squares_y: int,
        square_length_m: float,
        marker_length_m: float,
        dictionary,
    ):
        if hasattr(cv2.aruco, "CharucoBoard_create"):
            return cv2.aruco.CharucoBoard_create(
                squares_x,
                squares_y,
                square_length_m,
                marker_length_m,
                dictionary,
            )
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length_m,
            marker_length_m,
            dictionary,
        )

    @staticmethod
    def _get_board_ids(board) -> np.ndarray:
        if hasattr(board, "getIds"):
            ids = board.getIds()
        else:
            ids = board.ids
        return np.asarray(ids, dtype=np.int64).reshape(-1)

    def _validate_dictionary_capacity(self) -> None:
        max_marker_id = int(np.max(self._get_board_ids(self._board)))
        dictionary_size = int(self._dict.bytesList.shape[0])
        if max_marker_id >= dictionary_size:
            raise ValueError(
                "ChArUco dictionary does not contain enough marker IDs: "
                f"board needs ID {max_marker_id}, dictionary has {dictionary_size}"
            )

    def _detect_markers(self, gray: np.ndarray):
        if hasattr(cv2.aruco, "detectMarkers"):
            return cv2.aruco.detectMarkers(
                gray,
                self._dict,
                parameters=self._params,
            )
        detector = cv2.aruco.ArucoDetector(self._dict, self._params)
        return detector.detectMarkers(gray)

    def _interpolate_charuco(
        self,
        gray: np.ndarray,
        marker_corners,
        marker_ids: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
    ):
        camera_matrix = np.asarray(K, dtype=np.float64)
        dist_coeffs = np.asarray(D, dtype=np.float64).reshape(-1, 1)
        if hasattr(cv2.aruco, "interpolateCornersCharuco"):
            return cv2.aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                self._board,
                cameraMatrix=camera_matrix,
                distCoeffs=dist_coeffs,
            )

        charuco_params = cv2.aruco.CharucoParameters()
        charuco_params.cameraMatrix = camera_matrix
        charuco_params.distCoeffs = dist_coeffs
        detector = cv2.aruco.CharucoDetector(
            self._board,
            charuco_params,
            self._params,
        )
        charuco_corners, charuco_ids, _, _ = detector.detectBoard(
            gray,
            None,
            None,
            marker_corners,
            marker_ids,
        )
        count = 0 if charuco_ids is None else len(charuco_ids)
        return count, charuco_corners, charuco_ids

    def _estimate_pose(
        self,
        charuco_corners: np.ndarray,
        charuco_ids: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
    ):
        camera_matrix = np.asarray(K, dtype=np.float64)
        dist_coeffs = np.asarray(D, dtype=np.float64).reshape(-1, 1)
        if hasattr(cv2.aruco, "estimatePoseCharucoBoard"):
            rvec = np.zeros((1, 1, 3), dtype=np.float64)
            tvec = np.zeros((1, 1, 3), dtype=np.float64)
            return cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners,
                charuco_ids,
                self._board,
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
            )

        object_points, image_points = self._board.matchImagePoints(
            charuco_corners,
            charuco_ids,
        )
        return cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
        )

    @staticmethod
    def _reprojection_metrics(
        object_points: np.ndarray,
        image_points: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
    ) -> tuple[float, float]:
        projected, _ = cv2.projectPoints(
            np.asarray(object_points, dtype=np.float64),
            np.asarray(rvec, dtype=np.float64).reshape(3),
            np.asarray(tvec, dtype=np.float64).reshape(3),
            np.asarray(K, dtype=np.float64),
            np.asarray(D, dtype=np.float64).reshape(-1, 1),
        )
        errors = np.linalg.norm(
            projected.reshape(-1, 2)
            - np.asarray(image_points, dtype=np.float64).reshape(-1, 2),
            axis=1,
        )
        return (
            float(np.sqrt(np.mean(errors**2))),
            float(np.max(errors)),
        )

    def detect(
        self,
        bgr: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
    ) -> Optional[MarkerPose]:
        """Detect the ChArUco board and return its pose in camera frame."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = self._detect_markers(gray)
        if marker_ids is None or len(marker_ids) == 0:
            return None

        count, charuco_corners, charuco_ids = self._interpolate_charuco(
            gray,
            marker_corners,
            marker_ids,
            K,
            D,
        )
        if (
            charuco_corners is None
            or charuco_ids is None
            or int(count) < self._min_corners
        ):
            return None

        ok, rvec, tvec = self._estimate_pose(
            charuco_corners,
            charuco_ids,
            K,
            D,
        )
        if not ok:
            return None

        R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3))
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        object_points = self._get_chessboard_corners(self._board)[
            np.asarray(charuco_ids, dtype=np.int64).reshape(-1)
        ]
        reprojection_rmse_px, reprojection_max_px = self._reprojection_metrics(
            object_points,
            charuco_corners,
            rvec,
            tvec,
            K,
            D,
        )
        return MarkerPose(
            id=-1,
            T_marker2cam=T,
            corner_count=int(count),
            reprojection_rmse_px=reprojection_rmse_px,
            reprojection_max_px=reprojection_max_px,
        )

    def draw_detected(
        self,
        bgr: np.ndarray,
        K: np.ndarray,
        D: np.ndarray,
        axis_length: float = 0.03,
    ) -> np.ndarray:
        """Draw detected ChArUco markers, interpolated corners, and board axes."""
        vis = bgr.copy()
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = self._detect_markers(gray)
        if marker_ids is None or len(marker_ids) == 0:
            return vis

        cv2.aruco.drawDetectedMarkers(vis, marker_corners, marker_ids)
        count, charuco_corners, charuco_ids = self._interpolate_charuco(
            gray,
            marker_corners,
            marker_ids,
            K,
            D,
        )
        if (
            charuco_corners is None
            or charuco_ids is None
            or int(count) < self._min_corners
        ):
            return vis

        cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)
        pose = self.detect(bgr, K, D)
        if pose is not None:
            rvec, _ = cv2.Rodrigues(pose.T_marker2cam[:3, :3])
            cv2.drawFrameAxes(
                vis,
                np.asarray(K, dtype=np.float64),
                np.asarray(D, dtype=np.float64).reshape(-1, 1),
                rvec,
                pose.T_marker2cam[:3, 3],
                axis_length,
            )
        return vis
