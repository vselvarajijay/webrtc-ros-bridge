import os
import time
import json
import re
from pyppeteer import launch
from dotenv import load_dotenv

load_dotenv()

# Configuration from environment variables with defaults
FORMAT = os.getenv("IMAGE_FORMAT", "png")
QUALITY = float(os.getenv("IMAGE_QUALITY", "1.0"))
HAS_REAR_CAMERA = os.getenv("HAS_REAR_CAMERA", "False").lower() == "true"

if FORMAT not in ["png", "jpeg", "webp"]:
    raise ValueError("Invalid image format. Supported formats: png, jpeg, webp")

if QUALITY < 0 or QUALITY > 1:
    raise ValueError("Invalid image quality. Quality should be between 0 and 1")


class BrowserService:
    def __init__(self):
        self.browser = None
        self.page = None
        self.default_viewport = {"width": 3840, "height": 2160}

    async def initialize_browser(self):
        if not self.browser:
            try:
                executable_path = os.getenv(
                    "CHROME_EXECUTABLE_PATH",
                    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
                )
                
                # Verify executable exists
                if not os.path.exists(executable_path):
                    raise FileNotFoundError(
                        f"Chromium executable not found at {executable_path}. "
                        f"Check CHROME_EXECUTABLE_PATH environment variable."
                    )
                
                self.browser = await launch(
                    executablePath=executable_path,
                    headless=True,
                    args=[
                        "--ignore-certificate-errors",
                        "--no-sandbox",
                        "--disable-setuid-sandbox",
                        "--disable-dev-shm-usage",
                        "--disable-accelerated-2d-canvas",
                        "--disable-gpu",
                        "--disable-web-security",
                        "--disable-features=IsolateOrigins,site-per-process",
                        f"--window-size={self.default_viewport['width']},{self.default_viewport['height']}",
                    ],
                )
                self.page = await self.browser.newPage()
                await self.page.setViewport(self.default_viewport)
                await self.page.setExtraHTTPHeaders(
                    {"Accept-Language": "en-US,en;q=0.9"}
                )
                await self.page.goto(
                    "http://127.0.0.1:8000/sdk", {"waitUntil": "networkidle2"}
                )
                
                # Check if page is an error page (JSON error response)
                page_content = await self.page.content()
                if '{"detail":' in page_content or '<pre>{"detail":' in page_content:
                    # Try to extract the error message from JSON
                    try:
                        # Extract JSON from page content
                        json_match = re.search(r'\{[^}]*"detail"[^}]*\}', page_content)
                        if json_match:
                            error_json = json.loads(json_match.group())
                            error_detail = error_json.get('detail', 'Unknown error')
                            error_msg = (
                                f"SDK endpoint returned an error: {error_detail}. "
                                f"This usually means SDK_API_TOKEN is not set or the mission hasn't been started. "
                                f"Check your environment variables and ensure /start-mission has been called."
                            )
                        else:
                            error_msg = (
                                f"SDK endpoint returned an error page instead of HTML. "
                                f"Check that SDK_API_TOKEN is set and the mission has been started."
                            )
                    except Exception:
                        error_msg = (
                            f"SDK endpoint returned an error page instead of HTML. "
                            f"Page content: {page_content[:500]}"
                        )
                    print(error_msg)
                    raise Exception(error_msg)
                
                # Wait for the #join button to be available before clicking
                # Add timeout and better error handling
                try:
                    await self.page.waitForSelector("#join", {"timeout": 30000})
                except Exception as e:
                    # Check if page loaded correctly by checking URL and page content
                    page_url = self.page.url
                    page_title = await self.page.title()
                    page_content = await self.page.content()
                    
                    error_msg = (
                        f"Failed to find #join button on page. "
                        f"URL: {page_url}, Title: {page_title}. "
                        f"Page might be an error page or mission not started. "
                        f"Original error: {e}"
                    )
                    print(error_msg)
                    # Print first 500 chars of page content for debugging
                    print(f"Page content preview: {page_content[:500]}")
                    raise Exception(error_msg) from e
                
                await self.page.click("#join")
                await self.page.waitForSelector("video")
                await self.page.waitForSelector("#map")
                await self.page.setViewport(self.default_viewport)

                await self.page.waitFor(2000)

                call = f"""() => {{
                    window.initializeImageParams({{
                        imageFormat: "{FORMAT}",
                        imageQuality: {QUALITY}
                    }});
                }}"""
                await self.page.evaluate(call)
            except Exception as e:
                error_type = type(e).__name__
                error_msg = f"Error initializing browser ({error_type}): {e}"
                print(error_msg)
                
                # Check if browser was created but crashed
                if self.browser:
                    try:
                        # Try to get browser version to see if it's still alive
                        await self.browser.version()
                    except Exception:
                        print("Browser closed unexpectedly - likely crashed during initialization")
                        error_msg += " (Browser crashed)"
                
                self.browser = None
                self.page = None
                await self.close_browser()
                raise Exception(error_msg) from e

    async def take_screenshot(self, video_output_folder: str, elements: list):
        await self.initialize_browser()

        dimensions = await self.page.evaluate(
            """() => {
            return {
                width: Math.max(document.documentElement.scrollWidth, window.innerWidth),
                height: Math.max(document.documentElement.scrollHeight, window.innerHeight),
            }
        }"""
        )

        if (
            dimensions["width"] > self.default_viewport["width"]
            or dimensions["height"] > self.default_viewport["height"]
        ):
            await self.page.setViewport(dimensions)

        element_map = {"front": "#player-1000", "rear": "#player-1001", "map": "#map"}

        screenshots = {}
        for name in elements:
            if name in element_map:
                element_id = element_map[name]
                output_path = f"{video_output_folder}/{name}.png"
                element = await self.page.querySelector(element_id)
                if element:
                    start_time = time.time()  # Start time
                    await element.screenshot({"path": output_path})
                    end_time = time.time()  # End time
                    elapsed_time = (
                        end_time - start_time
                    ) * 1000  # Convert to milliseconds
                    print(f"Screenshot for {name} took {elapsed_time:.2f} ms")
                    screenshots[name] = output_path
                else:
                    print(f"Element {element_id} not found")
            else:
                print(f"Invalid element name: {name}")

        return screenshots

    async def data(self) -> dict:
        await self.initialize_browser()

        bot_data = await self.page.evaluate(
            """() => {
        return window.rtm_data;
        }"""
        )

        return bot_data

    async def front(self) -> str:
        await self.initialize_browser()

        try:
            # Use window.getLastBase64Frame and handle async properly
            front_frame = await self.page.evaluate(
                """async () => {
                    if (typeof window.getLastBase64Frame === 'function') {
                        try {
                            return await window.getLastBase64Frame(1000) || null;
                        } catch (e) {
                            console.error('Error in getLastBase64Frame:', e);
                            return null;
                        }
                    }
                    console.warn('getLastBase64Frame function not available');
                    return null;
                }"""
            )
        except Exception as e:
            print(f"Error getting front frame: {e}")
            front_frame = None

        return front_frame

    async def rear(self) -> str:
        await self.initialize_browser()

        try:
            # Use window.getLastBase64Frame and handle async properly
            rear_frame = await self.page.evaluate(
                """async () => {
                    if (typeof window.getLastBase64Frame === 'function') {
                        try {
                            return await window.getLastBase64Frame(1001) || null;
                        } catch (e) {
                            console.error('Error in getLastBase64Frame:', e);
                            return null;
                        }
                    }
                    console.warn('getLastBase64Frame function not available');
                    return null;
                }"""
            )
        except Exception as e:
            print(f"Error getting rear frame: {e}")
            rear_frame = None

        return rear_frame

    async def send_message(self, message: dict):
        await self.initialize_browser()

        await self.page.evaluate(
            """(message) => {
                window.sendMessage(message);
            }""",
            message,
        )

    async def close_browser(self):
        if self.browser:
            await self.browser.close()
            self.browser = None
            self.page = None
