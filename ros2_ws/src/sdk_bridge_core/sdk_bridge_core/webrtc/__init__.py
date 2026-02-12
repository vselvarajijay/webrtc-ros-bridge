"""
Standardized WebRTC receiver for SDK bridges.

Provides generic WebRTC streaming infrastructure that can be used
by any SDK adapter for real-time video/audio/telemetry streaming.
"""

from .webrtc_receiver import WebRTCReceiver
from .track_mapper import TrackMapper

__all__ = ["WebRTCReceiver", "TrackMapper"]
