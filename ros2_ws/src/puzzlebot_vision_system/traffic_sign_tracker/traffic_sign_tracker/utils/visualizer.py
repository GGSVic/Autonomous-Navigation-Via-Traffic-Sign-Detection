"""
Traffic Sign Visualization Utilities.

Centralizes all rendering logic. Maps functional categories to visual
styles (colors) to maintain consistency across debug streams.
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple
from ..tracker.track import Track


class TrafficSignVisualizer:
    """
    Visual analytics engine for perception debugging.

    This class provides standardized methods to project the tracker's internal 
    state onto the raw camera frames. By color-coding functional categories, 
    it allows for rapid verification of the FSM's perception inputs.
    """

    # BGR Pallete - Centralized for easy styling and consistency
    # Blue: Directions | Green: Traffic Lights | Red: Regulatory | Yellow: Caution
    CATEGORY_COLORS: Dict[int, Tuple[int, int, int]] = {
        1: (255, 0, 0),    # Blue (Directional)
        2: (0, 255, 0),    # Green (Traffic Lights)
        3: (0, 0, 255),    # Red (Regulatory/Stop)
        4: (0, 255, 255),  # Yellow (Caution/Warning)
        0: (255, 255, 255), # White (Undefined/Default)
    }

    TEXT_COLOR = (255, 255, 255)

    @staticmethod
    def render_tracks(image: np.ndarray, tracks: List[Track]) -> None:
        """
        Iterates and renders a collection of active tracks.
        """
        for track in tracks:
            TrafficSignVisualizer.annotate_track(image, track)

    @staticmethod
    def annotate_track(image: np.ndarray, track: Track) -> None:
        """
        Draws a single track using the color corresponding to its category.
        
        It overlays the bounding box, the track ID, the class name, and the 
        model confidence score.
        """
        det = track.latest_detection

        # Resolve color based on the detection's functional category
        color = TrafficSignVisualizer.CATEGORY_COLORS.get(det.category, (255, 255, 255))
        
        # Main Bounding Box
        cv2.rectangle(image, (det.x1, det.y1), (det.x2, det.y2), color, 2)

        # Centroid indicator (Useful for verifying tracker association)
        cv2.circle(image, det.centroid, 4, color, -1)

        # Text Label Preparation
        label = f"ID:{track.id} | {det.name} {det.confidence:.2f}"
        (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

        # Dynamic positioning to keep label within frame boundaries
        label_y = det.y1 - 10 if det.y1 > 20 else det.y1 + h + 10

        # Label Background (Solid)
        cv2.rectangle(image, (det.x1, label_y - h - 5), (det.x1 + w, label_y + 5), color, -1)

        # Text Overlay
        cv2.putText(image, label, (det.x1, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, TrafficSignVisualizer.TEXT_COLOR, 1, cv2.LINE_AA)