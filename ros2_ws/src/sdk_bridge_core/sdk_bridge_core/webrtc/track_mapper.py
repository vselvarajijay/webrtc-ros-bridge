"""
Track mapper for identifying camera types from WebRTC tracks.

Maps SDK-specific track IDs/labels to standardized camera types
using configuration-based rules.
"""

import re
import logging
from typing import Optional, Dict, Any

logger = logging.getLogger("track_mapper")

# Standardized camera types
STANDARD_CAMERA_TYPES = ["front", "rear", "left", "right", "top", "bottom"]


class TrackMapper:
    """
    Maps WebRTC track IDs/labels to standardized camera types.
    
    Supports multiple mapping strategies:
    - track_id_contains: String contains check
    - track_id_regex: Regex pattern matching
    - track_label_contains: Label contains check
    - track_label_regex: Label regex matching
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize track mapper with configuration.
        
        Args:
            config: Dictionary with track_mappings configuration
                Example:
                {
                    "track_mappings": {
                        "front": {"track_id_contains": "1000"},
                        "rear": {"track_id_contains": "1001"}
                    }
                }
        """
        self.config = config
        self.track_mappings = config.get("track_mappings", {})
        
        # Compile regex patterns for performance
        self._compiled_patterns = {}
        for camera_type, mapping_config in self.track_mappings.items():
            if camera_type not in STANDARD_CAMERA_TYPES:
                logger.warning(f"Unknown camera type: {camera_type}, expected one of {STANDARD_CAMERA_TYPES}")
            
            compiled = {}
            if "track_id_regex" in mapping_config:
                try:
                    compiled["track_id_regex"] = re.compile(mapping_config["track_id_regex"])
                except re.error as e:
                    logger.error(f"Invalid regex for {camera_type}: {e}")
            
            if "track_label_regex" in mapping_config:
                try:
                    compiled["track_label_regex"] = re.compile(mapping_config["track_label_regex"])
                except re.error as e:
                    logger.error(f"Invalid regex for {camera_type}: {e}")
            
            if compiled:
                self._compiled_patterns[camera_type] = compiled

    def identify_camera_type(self, track_id: str, track_label: str = "") -> Optional[str]:
        """
        Identify camera type from track ID and label.
        
        Args:
            track_id: WebRTC track ID (string representation)
            track_label: WebRTC track label (optional)
            
        Returns:
            Standardized camera type ("front", "rear", etc.) or None if not matched
        """
        track_id_str = str(track_id) if track_id else ""
        track_label_str = str(track_label) if track_label else ""
        
        # Try each camera type mapping
        for camera_type, mapping_config in self.track_mappings.items():
            if self._matches_mapping(camera_type, track_id_str, track_label_str, mapping_config):
                logger.debug(f"Track {track_id_str} ({track_label_str}) mapped to {camera_type}")
                return camera_type
        
        logger.debug(f"No mapping found for track {track_id_str} ({track_label_str})")
        return None

    def _matches_mapping(
        self, 
        camera_type: str, 
        track_id: str, 
        track_label: str, 
        mapping_config: Dict[str, Any]
    ) -> bool:
        """
        Check if track matches a specific camera type mapping.
        
        Priority order:
        1. track_id_contains (fastest)
        2. track_label_contains
        3. track_id_regex
        4. track_label_regex
        """
        # Check track_id_contains
        if "track_id_contains" in mapping_config:
            search_str = str(mapping_config["track_id_contains"])
            if search_str in track_id:
                return True
        
        # Check track_label_contains
        if "track_label_contains" in mapping_config:
            search_str = str(mapping_config["track_label_contains"])
            if search_str.lower() in track_label.lower():
                return True
        
        # Check track_id_regex
        if "track_id_regex" in mapping_config:
            pattern = self._compiled_patterns.get(camera_type, {}).get("track_id_regex")
            if pattern and pattern.search(track_id):
                return True
        
        # Check track_label_regex
        if "track_label_regex" in mapping_config:
            pattern = self._compiled_patterns.get(camera_type, {}).get("track_label_regex")
            if pattern and pattern.search(track_label):
                return True
        
        return False

    def get_all_camera_types(self) -> list:
        """Get list of all configured camera types."""
        return list(self.track_mappings.keys())
