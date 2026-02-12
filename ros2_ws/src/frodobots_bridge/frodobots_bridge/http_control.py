"""
HTTP client for sending control commands to Frodobots SDK.
"""

import asyncio
import json
import logging
from typing import Dict, Optional

try:
    import aiohttp
    HAS_AIOHTTP = True
except ImportError:
    HAS_AIOHTTP = False
    try:
        import requests
    except ImportError:
        requests = None

logger = logging.getLogger("frodobots_http_control")


class FrodobotsHTTPControl:
    """
    HTTP client for sending control commands to Frodobots SDK.
    
    Converts standardized ROS2 control commands to Frodobots HTTP format.
    """

    def __init__(self, sdk_url: str = "http://localhost:8000"):
        """
        Initialize HTTP control client.
        
        Args:
            sdk_url: Base URL of the Frodobots SDK service
        """
        self.sdk_url = sdk_url.rstrip("/")
        self.control_endpoint = f"{self.sdk_url}/control"
        self._session = None

    async def _send_async(self, command: Dict) -> bool:
        """Send command asynchronously using aiohttp."""
        if not HAS_AIOHTTP:
            logger.error("aiohttp not available, cannot send async")
            return False
        
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    self.control_endpoint,
                    json={"command": command},
                    timeout=aiohttp.ClientTimeout(total=10.0),
                ) as response:
                    response_text = await response.text()
                    logger.info(
                        "Frodobots response: status=%s body=%s",
                        response.status,
                        response_text.strip() or "(empty)",
                    )
                    if response.status == 200:
                        logger.info("Control command sent successfully: %s", command)
                        return True
                    else:
                        logger.error(
                            "Control command failed: %s - %s",
                            response.status,
                            response_text,
                        )
                        return False
        except Exception as e:
            logger.error(
                "Error sending control command: %s: %s",
                type(e).__name__,
                str(e).strip() or repr(e),
            )
            return False

    def send_command(self, command: Dict) -> bool:
        """
        Send a control command to Frodobots SDK.
        
        Args:
            command: Command dictionary in Frodobots format
            
        Returns:
            True if command was sent successfully, False otherwise
        """
        try:
            if HAS_AIOHTTP:
                # Try async first
                loop = None
                try:
                    loop = asyncio.get_event_loop()
                except RuntimeError:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                
                if loop.is_running():
                    # If loop is running, schedule the coroutine
                    task = asyncio.create_task(self._send_async(command))
                    # Note: This is fire-and-forget, actual result won't be returned
                    return True
                else:
                    return loop.run_until_complete(self._send_async(command))
            else:
                # Fallback to requests (synchronous)
                if requests is None:
                    logger.error("Neither aiohttp nor requests available")
                    return False
                response = requests.post(
                    self.control_endpoint,
                    json={"command": command},
                    timeout=10.0,
                )
                logger.info(
                    "Frodobots response: status=%s body=%s",
                    response.status_code,
                    (response.text or "").strip() or "(empty)",
                )
                if response.status_code == 200:
                    logger.info("Control command sent successfully: %s", command)
                    return True
                else:
                    logger.error(
                        "Control command failed: %s - %s",
                        response.status_code,
                        response.text,
                    )
                    return False
        except Exception as e:
            logger.error(
                "Error sending control command: %s: %s",
                type(e).__name__,
                str(e).strip() or repr(e),
            )
            return False

    def convert_ros2_to_frodobots(self, ros2_command: Dict) -> Dict:
        """
        Convert standardized ROS2 control command to Frodobots format.
        
        Args:
            ros2_command: Standardized ROS2 command dictionary
                Expected format: {"command_type": str, "command_data": str}
                command_data should be JSON string with Frodobots command structure
        
        Returns:
            Frodobots command dictionary
        """
        try:
            # If command_data is a JSON string, parse it
            if "command_data" in ros2_command:
                command_data = ros2_command["command_data"]
                if isinstance(command_data, str):
                    try:
                        # Try to parse as JSON
                        frodobots_cmd = json.loads(command_data)
                        return frodobots_cmd
                    except json.JSONDecodeError:
                        # If not JSON, assume it's already the command structure
                        return {"command": command_data}
            
            # If command_type is provided, construct Frodobots command
            if "command_type" in ros2_command:
                return {
                    "type": ros2_command["command_type"],
                    "data": ros2_command.get("command_data", {}),
                }
            
            # Otherwise, assume the command is already in Frodobots format
            return ros2_command
            
        except Exception as e:
            logger.error(f"Error converting ROS2 command to Frodobots format: {e}")
            return ros2_command
