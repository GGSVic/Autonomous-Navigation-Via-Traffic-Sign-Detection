"""
Road Segmentation Module.

Provides a suite of static methods for isolating road elements (lines, lanes,
crosswalks) using LAB color-space segmentation and morphological reconstruction.
Includes logic for dynamic brightness correction and geometric classification
of road components based on verticality and connectivity.
"""

from __future__ import annotations

import numpy as np
import cv2

from typing import List, Tuple
from dataclasses import dataclass

from road_perception.road_segmenter.road_component import RoadComponent, ComponentType
from road_perception.utils.perception_context import PerceptionContext


class RoadSegmenter:
    """
    Orchestrates the conversion of raw RGB imagery into semantic road entities.
    
    Uses LAB color-space for lighting-invariant thresholding and connectivity-based 
    reconstruction to filter out distant or irrelevant noise.
    """

    # Threshold values for LINE segmentation (LAB color space)
    # L (Lightness), A (Green-Red), B (Blue-Yellow)
    LINE_THRESHOLD_LOW = np.array([0, 110, 0], dtype=np.uint8)
    LINE_THRESHOLD_HIGH = np.array([30, 145, 140], dtype=np.uint8)

    # Threshold values for LANE segmentation (LAB color space)
    LANE_THRESHOLD_LOW = np.array([43, 124, 135], dtype=np.uint8)
    LANE_THRESHOLD_HIGH = np.array([230, 189, 184], dtype=np.uint8)

    @dataclass(frozen=True, slots=True)
    class RoadSegmentationOutput:
        """
        Data container for the geometric infrastructure detected during the 
        segmentation and analysis pipeline.
        """
        road_mask: np.ndarray             # Combined binary mask of lines and lanes
        road_lines: List[RoadComponent]    # Vertical lane markings
        crosswalk: RoadComponent | None    # Fused horizontal marking entity

    @staticmethod
    def adjust_brightness(img: np.ndarray, target: float = 0.55) -> None:
        """
        Dynamically normalizes image brightness to a specific target level.
        
        Ensures consistent segmentation performance across varying lighting 
        conditions by scaling image intensity.

        Args:
            img: Input image (BGR). Modified in-place.
            target: Desired average brightness normalized from 0.0 to 1.0.
        """
        sample = cv2.resize(img, (32, 32))
        brightness: float = np.mean(sample) / 255

        if brightness < target:
            ratio = target / (brightness + 1e-6)
            cv2.convertScaleAbs(img, alpha=ratio, beta=0, dst=img)

    @staticmethod
    def segment_lines(img_lab: np.ndarray) -> np.ndarray:
        """
        Isolates LINE objects using calibrated LAB thresholds.

        Args:
            img_lab: Input image in LAB color space.
        Returns:
            Binary mask after a 5x5 morphological closing operation.
        """
        mask = cv2.inRange(
            img_lab,
            RoadSegmenter.LINE_THRESHOLD_LOW,
            RoadSegmenter.LINE_THRESHOLD_HIGH,
        )
        kernel = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    @staticmethod
    def segment_lanes(img_lab: np.ndarray) -> np.ndarray:
        """
        Isolates driveable LANE regions using LAB thresholds.

        Args:
            img_lab: Input image in LAB color space.
        Returns:
            Raw binary mask of detected lane surfaces.
        """
        return cv2.inRange(
            img_lab,
            RoadSegmenter.LANE_THRESHOLD_LOW,
            RoadSegmenter.LANE_THRESHOLD_HIGH,
        )

    @staticmethod
    def analyze_contours(bw_lines: np.ndarray, area_th: int = 135) -> List[RoadComponent]:
        """
        Extracts filtered RoadComponent objects from a binary mask.

        Args:
            bw_lines: Input binary mask.
            area_th: Minimum pixel area to filter out noise.
        Returns:
            List of valid RoadComponent instances.
        """
        contours, _ = cv2.findContours(bw_lines, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        road_lines: List[RoadComponent] = []
        for cnt in contours:
            try:
                line = RoadComponent(cnt, ComponentType.LINE)
                if line.area >= area_th:
                    road_lines.append(line)
            except ValueError:
                pass

        return road_lines

    @staticmethod
    def classify_lines(lines: List[RoadComponent], 
                      verticality_th: float = 0.15) -> Tuple[List[RoadComponent], List[RoadComponent]]:
        """
        Sorts components into lane markings or crosswalk candidates based on orientation.

        Args:
            lines: List of detected RoadComponent objects.
            verticality_th: Sine-based threshold to distinguish vertical lines.
        Returns:
            Tuple containing (vertical_lanes, horizontal_zebras).
        """
        lanes = [l for l in lines if l.fitline.verticality > verticality_th]
        zebras = [l for l in lines if l.fitline.verticality <= verticality_th]
        
        return lanes, zebras    
        
    @staticmethod
    def check_crosswalk(zebras: List[RoadComponent]) -> RoadComponent | None:
        """
        Fuses multiple horizontal zebra-crossing candidates into a single entity.

        Args:
            zebras: List of horizontal marking candidates.
        Returns:
            A fused RoadComponent if enough candidates are present, otherwise None.
        """
        if len(zebras) < 2:
            return None
        
        all_points: List[np.ndarray] = [l.contour for l in zebras]
        fused_cnt: np.ndarray = np.vstack(all_points).astype(np.int32)
        
        try:
            return RoadComponent(fused_cnt, ComponentType.CROSSWALK)
        except ValueError:
            return None
 
    @staticmethod
    def reconstruct_from_seed(bw: np.ndarray, seed_bw: np.ndarray) -> np.ndarray:
        """
        Performs morphological reconstruction to restore connected components that 
        intersect with a designated 'seed' region (usually the bottom of the ROI).

        Args:
            bw: Source binary mask containing all potential detections.
            seed_bw: Mask used to identify relevant components near the robot.
        Returns:
            Reconstructed binary mask containing only relevant entities.
        """
        _, labels, _, _ = cv2.connectedComponentsWithStats(bw, connectivity=8)
        reconstructed = np.zeros_like(bw)

        # Map labels that exist within the seed area
        intersecting_labels = np.unique(labels[seed_bw > 0])

        for label in intersecting_labels:
            if label != 0: # Skip background
                reconstructed[labels == label] = 255

        return reconstructed

    @staticmethod
    def detect_road_components(img_rgb: np.ndarray) -> RoadSegmenter.RoadSegmentationOutput:
        """
        Executes the complete perception pipeline:
        1. Normalizes brightness.
        2. Segments LAB masks for lines and lanes.
        3. Filters components using connectivity seeds near the robot.
        4. Classifies entities into lanes and crosswalks.
        """ 
        # --- PREPROCESSING ---
        RoadSegmenter.adjust_brightness(img_rgb)

        # --- SEGMENTATION --- 
        img_lab = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2Lab)
        raw_lines_mask = RoadSegmenter.segment_lines(img_lab)
        raw_lanes_mask = RoadSegmenter.segment_lanes(img_lab)

        # --- CONNECTIVITY RECONSTRUCTION ---
        # Keep only segments connected to the bottom of the ROI (near the robot)
        frame_h = PerceptionContext.frame_height()
        frame_w = PerceptionContext.frame_width() 

        seed = np.zeros((frame_h, frame_w), dtype=np.uint8)
        start_row = frame_h - (frame_h // 3)
        seed[start_row:frame_h, :] = 255

        lines_mask = RoadSegmenter.reconstruct_from_seed(raw_lines_mask, seed)
        lanes_mask = RoadSegmenter.reconstruct_from_seed(raw_lanes_mask, seed)

        # Merge masks and refine edges
        road_mask = cv2.bitwise_or(lines_mask, lanes_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)

        # ---- CLASSIFICATION ----
        raw_lines = RoadSegmenter.analyze_contours(lines_mask, area_th=115)
        road_lines, zebra_lines = RoadSegmenter.classify_lines(raw_lines)
        crosswalk = RoadSegmenter.check_crosswalk(zebra_lines)

        return RoadSegmenter.RoadSegmentationOutput(road_mask, road_lines, crosswalk)