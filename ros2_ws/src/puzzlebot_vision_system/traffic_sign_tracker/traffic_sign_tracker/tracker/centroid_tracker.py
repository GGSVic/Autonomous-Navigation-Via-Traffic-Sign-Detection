"""
Temporal Data Association Module.

Implements the Hungarian Algorithm for optimal track matching. Manages object
life cycles (Birth/Death) and implements family-based filtering and traffic
light state persistence for stable navigation.
"""

from typing import List, Tuple, Optional, Set
from scipy.optimize import linear_sum_assignment
import numpy as np

from .detection import Detection
from .track import Track


class CentroidTracker:
    """
    Hungarian-based Centroid Tracker for Traffic Infrastructure.

    Coordinates the association of new detections to existing tracks. It maintains
    a persistent state of the environment, allowing the robot to 'remember'
    objects even if they are briefly occluded or missed by the detector.
    """

    def __init__(self):
        """
        Initializes the tracker with thresholds for spatial and temporal consistency.
        """
        self.tracks: List[Track] = []
        self.next_id: int = 0

        # Distance threshold (px): Max distance a sign can "move" between frames.
        self.dist_threshold: float = 55.0

        # Persistence threshold (frames): How long to keep a track after it's lost.
        self.lost_threshold: int = 10

        # Area consistency: Max relative change in size (0.5 = 50% change allowed).
        self.area_threshold: float = 0.5

        # TF State Lock: Stores the unique ID of the traffic light currently being 
        # prioritized. This is essential because the model might detect multiple 
        # TFs in the background; by locking onto the one directly in front, 
        # we prevent the robot from reacting to the wrong intersection.
        self.locked_tf_id: Optional[int] = None

    def update(self, detections: List[Detection]) -> List[Track]:
        """
        Main entry point for the tracking pipeline.

        Orchestrates the matching process:
        1. Initializes tracks if none exist.
        2. Solves the assignment problem using the Hungarian Algorithm.
        3. Updates matched tracks and creates new ones for residuals.
        4. Cleans up stale tracks and filters results for the node.
        """
        if not self.tracks:
            self._init_new_tracks(detections)
            return self._get_active_tracks()

        # Spatial association with family-based filtering.
        cost_matrix = self._compute_cost_matrix(detections)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # State synchronization.
        assigned_tracks, assigned_dets = self._apply_assignment(
            row_ind, col_ind, cost_matrix, detections
        )

        # Life-cycle management (Birth/Death).
        self._handle_unassigned(assigned_tracks, assigned_dets, detections)
        self._cleanup_tracks()

        return self._get_active_tracks()

    def _compute_cost_matrix(self, detections: List[Detection]) -> np.ndarray:
        """
        Calculates a cost matrix between existing tracks and new detections.

        The cost is primarily Euclidean distance. However, a 'hard filter' is
        applied: if the 'family' of the detection doesn't match the track,
        the cost is set to infinity. This prevents a 'Stop' sign from
        hijacking a 'Turn Right' track.
        """
        rows, cols = len(self.tracks), len(detections)
        cost_matrix = np.full((rows, cols), 999999.0)

        for i, track in enumerate(self.tracks):
            for j, det in enumerate(detections):
                # Hard Filter: Logical consistency (Family matching).
                if track.latest_detection.family == det.family:
                    dist = np.linalg.norm(
                        np.array(det.centroid) - np.array(track.latest_detection.centroid)
                    )

                    # Area consistency check to prevent matching overlapping signs.
                    area_diff = abs(track.latest_detection.area - det.area) / max(
                        track.latest_detection.area, det.area
                    )

                    if dist < self.dist_threshold and area_diff < self.area_threshold:
                        cost_matrix[i, j] = dist
        return cost_matrix

    def _apply_assignment(
        self, row_ind: np.ndarray, col_ind: np.ndarray, 
        cost_matrix: np.ndarray, detections: List[Detection]
    ) -> Tuple[Set[int], Set[int]]:
        """
        Matches detections to tracks based on the Hungarian solution.
        """
        assigned_tracks, assigned_dets = set(), set()

        for t_idx, d_idx in zip(row_ind, col_ind):
            # Final threshold validation.
            if cost_matrix[t_idx, d_idx] < self.dist_threshold:
                self.tracks[t_idx].update(detections[d_idx])
                assigned_tracks.add(t_idx)
                assigned_dets.add(d_idx)

        return assigned_tracks, assigned_dets

    def _handle_unassigned(
        self, assigned_tracks: Set[int], assigned_dets: Set[int], detections: List[Detection]
    ):
        """
        Manages residuals from the assignment process (Births and Deaths).
        """
        # Handle "Deaths" (Missed detections).
        for i in set(range(len(self.tracks))) - assigned_tracks:
            self.tracks[i].mark_missed()

        # Handle "Births" (New detections).
        for j in set(range(len(detections))) - assigned_dets:
            self.tracks.append(Track(detections[j], self.next_id))
            self.next_id += 1

    def _cleanup_tracks(self):
        """
        Hard-deletes tracks that have been lost for too long to keep the memory footprint low.
        """
        self.tracks = [t for t in self.tracks if t.lost_frames < self.lost_threshold]
    
    def _get_active_tracks(self) -> List[Track]:
        """
        Filters tracks for consumption by the robot's control logic.

        TRAFFIC LIGHT LOCKING LOGIC:
        Since the robot must navigate intersections sequentially, it 'locks' onto 
        the first Traffic Light (TF) detected in its path. This prevents the 
        controller from flickering between the current intersection and background 
        lights, ensuring it only processes the state of the relevant TF until it 
        is out of view.
        """
        valid_tracks = []
        for track in self.tracks:
            # Only return tracks currently visible in the frame.
            if track.lost_frames > 0:
                continue

            # Specialized logic for Traffic Lights.
            if track.latest_detection.family == "traffic_light":
                # Priority: Lock onto a specific TF ID until the maneuver is over.
                if self.locked_tf_id in (None, track.id):
                    self.locked_tf_id = track.id
                    valid_tracks.append(track)
            else:
                valid_tracks.append(track)

        # Release the lock if the prioritized TF is no longer in the FOV.
        if self.locked_tf_id is not None and not any(t.id == self.locked_tf_id for t in valid_tracks):
            self.locked_tf_id = None

        return valid_tracks

    def _init_new_tracks(self, detections: List[Detection]):
        """Populates the track list when no previous state exists."""
        for det in detections:
            self.tracks.append(Track(det, self.next_id))
            self.next_id += 1