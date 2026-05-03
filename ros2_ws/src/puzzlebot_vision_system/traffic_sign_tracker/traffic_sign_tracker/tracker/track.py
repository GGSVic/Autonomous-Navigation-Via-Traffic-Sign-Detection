"""
Temporal Object Tracking State.

Defines the life cycle and persistence of a tracked object. Manages temporal
metrics such as age, hit count, and visibility status to ensure robust
data association across sequential video frames.
"""

from traffic_sign_tracker.tracker.detection import Detection


class Track:
    """
    Represents the persistent identity of an object across a temporal sequence.

    This class manages the 'memory' of a detection. By maintaining counters for 
    hits and lost frames, it allows the system to filter out transient false 
    positives and handle brief occlusions or detection failures without 
    losing the object's unique ID.
    """

    def __init__(self, detection: Detection, track_id: int):
        """
        Initializes a new Track from a fresh detection.

        Args:
            detection: Initial Detection data container.
            track_id: Unique identifier for the object life cycle.
        """
        self.id: int = track_id
        
        # Latest visual and semantic state (BBox, Category, Family)
        self.latest_detection: Detection = detection 

        # Lifecycle Management Metrics
        self.age: int = 1  # Total frames since this object was first seen
        self.hits: int = 1  # Total number of frames where this object was matched
        self.lost_frames: int = 0  # Number of consecutive frames where the object disappeared

    def update(self, new_detection: Detection):
        """
        Synchronizes the track state with a newly matched detection.

        Updating 'latest_detection' ensures that downstream consumers (like 
        the FSM or PID controllers) always have the most recent spatial 
        coordinates and confidence scores while retaining the persistent ID.

        Args:
            new_detection: The current frame's matched Detection object.
        """
        self.latest_detection = new_detection
        self.lost_frames = 0
        self.hits += 1
        self.age += 1

    def mark_missed(self):
        """
        Increments the lost frame counter when no association is found.

        This allows the CentroidTracker to decide whether to keep the track 
        alive or delete it if 'lost_frames' exceeds a predefined threshold.
        """
        self.lost_frames += 1
        self.age += 1

    @property
    def is_confirmed(self) -> bool:
        """
        Heuristic to determine if a track is stable.
        
        A track is usually considered 'confirmed' after a minimum number 
        of successful hits, preventing noise from triggering robot actions.
        """
        return self.hits >= 3