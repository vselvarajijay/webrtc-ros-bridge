/*
 *  These procedures use Agora Video Call SDK for Web to enable local and remote
 *  users to join and leave a Video Call channel managed by Agora Platform.
 */

/*
 *  Create an {@link https://docs.agora.io/en/Video/API%20Reference/web_ng/interfaces/iagorartcclient.html|AgoraRTCClient} instance.
 *
 * @param {string} mode - The {@link https://docs.agora.io/en/Voice/API%20Reference/web_ng/interfaces/clientconfig.html#mode| streaming algorithm} used by Agora SDK.
 * @param  {string} codec - The {@link https://docs.agora.io/en/Voice/API%20Reference/web_ng/interfaces/clientconfig.html#codec| client codec} used by the browser.
 */
var client;

/*
 * Clear the video and audio tracks used by `client` on initiation.
 */
var localTracks = {
  videoTrack: null,
  audioTrack: null,
};

/*
 * On initiation no users are connected.
 */
var remoteUsers = {};

/*
 * WebRTC bridge peer connections for forwarding Agora tracks to ROS2 bridge
 */
var bridgePeerConnections = {};

/*
 * Subscription queue to serialize subscribe calls (prevents WebRTC negotiation conflicts)
 */
var subscriptionQueue = [];
var isProcessingSubscription = false;

/*
 * On initiation. `client` is not attached to any project or channel for any specific user.
 */
var options = {
  appid: null,
  channel: null,
  uid: null,
  token: null,
};

// you can find all the agora preset video profiles here https://docs.agora.io/en/Voice/API%20Reference/web_ng/globals.html#videoencoderconfigurationpreset
var videoProfiles = [
  {
    label: "360p_7",
    detail: "480×360, 15fps, 320Kbps",
    value: "360p_7",
  },
  {
    label: "360p_8",
    detail: "480×360, 30fps, 490Kbps",
    value: "360p_8",
  },
  {
    label: "480p_1",
    detail: "640×480, 15fps, 500Kbps",
    value: "480p_1",
  },
  {
    label: "480p_2",
    detail: "640×480, 30fps, 1000Kbps",
    value: "480p_2",
  },
  {
    label: "720p_1",
    detail: "1280×720, 15fps, 1130Kbps",
    value: "720p_1",
  },
  {
    label: "720p_2",
    detail: "1280×720, 30fps, 2000Kbps",
    value: "720p_2",
  },
  {
    label: "1080p_1",
    detail: "1920×1080, 15fps, 2080Kbps",
    value: "1080p_1",
  },
  {
    label: "1080p_2",
    detail: "1920×1080, 30fps, 3000Kbps",
    value: "1080p_2",
  },
];
var curVideoProfile;
AgoraRTC.onAutoplayFailed = () => {
  alert("click to start autoplay!");
};
AgoraRTC.onMicrophoneChanged = async (changedDevice) => {
  // When plugging in a device, switch to a device that is newly plugged in.
  if (changedDevice.state === "ACTIVE") {
    localTracks.audioTrack.setDevice(changedDevice.device.deviceId);
    // Switch to an existing device when the current device is unplugged.
  } else if (
    changedDevice.device.label === localTracks.audioTrack.getTrackLabel()
  ) {
    const oldMicrophones = await AgoraRTC.getMicrophones();
    oldMicrophones[0] &&
      localTracks.audioTrack.setDevice(oldMicrophones[0].deviceId);
  }
};
AgoraRTC.onCameraChanged = async (changedDevice) => {
  // When plugging in a device, switch to a device that is newly plugged in.
  if (changedDevice.state === "ACTIVE") {
    localTracks.videoTrack.setDevice(changedDevice.device.deviceId);
    // Switch to an existing device when the current device is unplugged.
  } else if (
    changedDevice.device.label === localTracks.videoTrack.getTrackLabel()
  ) {
    const oldCameras = await AgoraRTC.getCameras();
    oldCameras[0] && localTracks.videoTrack.setDevice(oldCameras[0].deviceId);
  }
};
async function initDevices() {
  mics = await AgoraRTC.getMicrophones();
  const audioTrackLabel = localTracks.audioTrack.getTrackLabel();
  currentMic = mics.find((item) => item.label === audioTrackLabel);
  $(".mic-input").val(currentMic.label);
  $(".mic-list").empty();
  mics.forEach((mic) => {
    $(".mic-list").append(`<a class="dropdown-item" href="#">${mic.label}</a>`);
  });

  cams = await AgoraRTC.getCameras();
  const videoTrackLabel = localTracks.videoTrack.getTrackLabel();
  currentCam = cams.find((item) => item.label === videoTrackLabel);
  $(".cam-input").val(currentCam.label);
  $(".cam-list").empty();
  cams.forEach((cam) => {
    $(".cam-list").append(`<a class="dropdown-item" href="#">${cam.label}</a>`);
  });
}
async function switchCamera(label) {
  currentCam = cams.find((cam) => cam.label === label);
  $(".cam-input").val(currentCam.label);
  // switch device of local video track.
  await localTracks.videoTrack.setDevice(currentCam.deviceId);
}
async function switchMicrophone(label) {
  currentMic = mics.find((mic) => mic.label === label);
  $(".mic-input").val(currentMic.label);
  // switch device of local audio track.
  await localTracks.audioTrack.setDevice(currentMic.deviceId);
}
function initVideoProfiles() {
  videoProfiles.forEach((profile) => {
    $(".profile-list").append(
      `<a class="dropdown-item" label="${profile.label}" href="#">${profile.label}: ${profile.detail}</a>`
    );
  });
  curVideoProfile = videoProfiles.find((item) => item.label == "480p_1");
  $(".profile-input").val(`${curVideoProfile.detail}`);
}
async function changeVideoProfile(label) {
  curVideoProfile = videoProfiles.find((profile) => profile.label === label);
  $(".profile-input").val(`${curVideoProfile.detail}`);
  // change the local video track`s encoder configuration
  localTracks.videoTrack &&
    (await localTracks.videoTrack.setEncoderConfiguration(
      curVideoProfile.value
    ));
}

/*
 * When this page is called with parameters in the URL, this procedure
 * attempts to join a Video Call channel using those parameters.
 */
$(() => {
  initVideoProfiles();
  $(".profile-list").delegate("a", "click", function (e) {
    changeVideoProfile(this.getAttribute("label"));
  });
  var urlParams = new URL(location.href).searchParams;
  options.appid = urlParams.get("appid");
  options.channel = urlParams.get("channel");
  options.token = urlParams.get("token");
  options.uid = urlParams.get("uid");
  if (options.appid && options.channel) {
    $("#uid").val(options.uid);
    $("#appid").val(options.appid);
    $("#token").val(options.token);
    $("#channel").val(options.channel);
    $("#join-form").submit();
  }
});

/*
 * When a user clicks Join or Leave in the HTML form, this procedure gathers the information
 * entered in the form and calls join asynchronously. The UI is updated to match the options entered
 * by the user.
 */
$("#join-form").submit(async function (e) {
  e.preventDefault();
  $("#join").attr("disabled", true);
  try {
    client = AgoraRTC.createClient({
      mode: "rtc",
      codec: getCodec(),
    });
    options.channel = $("#channel").val();
    options.uid = Number($("#uid").val());
    options.appid = $("#appid").val();
    options.token = $("#token").val();
    await join();
    if (options.token) {
      $("#success-alert-with-token").css("display", "block");
    } else {
      $("#success-alert a").attr(
        "href",
        `index.html?appid=${options.appid}&channel=${options.channel}&token=${options.token}`
      );
      $("#success-alert").css("display", "block");
    }
  } catch (error) {
    console.error(error);
  } finally {
    $("#leave").attr("disabled", false);
  }
});

/*
 * Called when a user clicks Leave in order to exit a channel.
 */
$("#leave").click(function (e) {
  leave();
});
$("#agora-collapse").on("show.bs.collapse	", function () {
  initDevices();
});
$(".cam-list").delegate("a", "click", function (e) {
  switchCamera(this.text);
});
$(".mic-list").delegate("a", "click", function (e) {
  switchMicrophone(this.text);
});

/*
 * Join a channel, then create local video and audio tracks and publish them to the channel.
 */
async function join() {
  // Add an event listener to play remote tracks when remote user publishes.
  client.on("user-published", handleUserPublished);
  client.on("user-unpublished", handleUserUnpublished);
  options.uid = await client.join(
    options.appid,
    options.channel,
    options.token || null,
    options.uid || null
  );
  $("#captured-frames").css("display", DEBUG_MODE ? "block" : "none");
}

/*
 * Stop all local and remote tracks then leave the channel.
 */
async function leave() {
  for (trackName in localTracks) {
    var track = localTracks[trackName];
    if (track) {
      track.stop();
      track.close();
      localTracks[trackName] = undefined;
    }
  }

  // Remove remote users and player views.
  remoteUsers = {};
  // Clear subscription queue
  subscriptionQueue = [];
  isProcessingSubscription = false;
  $("#remote-playerlist").html("");

  // Close all bridge peer connections
  for (const uid in bridgePeerConnections) {
    console.log(`[WebRTC Bridge] Closing bridge connection for UID ${uid}`);
    const pc = bridgePeerConnections[uid];
    
    // Clean up video elements and canvas used for forwarding
    pc.getSenders().forEach(sender => {
      const track = sender.track;
      if (track && track._sourceVideoElement) {
        track._sourceVideoElement.pause();
        track._sourceVideoElement.srcObject = null;
        if (track._sourceVideoElement.parentNode) {
          track._sourceVideoElement.parentNode.removeChild(track._sourceVideoElement);
        }
      }
    });
    
    pc.close();
  }
  bridgePeerConnections = {};

  // leave the channel
  await client.leave();
  $("#local-player-name").text("");
  $("#join").attr("disabled", false);
  $("#leave").attr("disabled", true);
  $("#joined-setup").css("display", "none");
  console.log("client leaves channel success");
}

/*
 * Process subscription queue one at a time to prevent WebRTC negotiation conflicts
 */
async function processSubscriptionQueue() {
  if (isProcessingSubscription || subscriptionQueue.length === 0) {
    return;
  }

  isProcessingSubscription = true;

  while (subscriptionQueue.length > 0) {
    const { user, mediaType, resolve, reject } = subscriptionQueue.shift();
    const uid = user.uid;

    try {
      console.log(`Processing subscription for ${uid} ${mediaType}`);
      await client.subscribe(user, mediaType);
      console.log(`subscribe success for ${uid} ${mediaType}`);

      if (mediaType === "video") {
        const playerWidth =
          uid === 1001 ? "540px" : uid === 1000 ? "1024px" : "auto";
        const playerHeight =
          uid === 1001 ? "360px" : uid === 1000 ? "576px" : "auto";

        // Check if player already exists
        if ($(`#player-wrapper-${uid}`).length === 0) {
          const player = $(`
            <div id="player-wrapper-${uid}">
              <p class="player-name">(${uid})</p>
              <div id="player-${uid}" class="player" style="width: ${playerWidth}; height: ${playerHeight};"></div>
            </div>
          `);
          $("#remote-playerlist").append(player);
        }
        user.videoTrack.play(`player-${uid}`);

        // Check if captured frame div already exists
        if ($(`#captured-frame-${uid}`).length === 0) {
          const capturedFrameDiv = $(`
            <div id="captured-frame-${uid}" style="width: ${playerWidth}; height: ${playerHeight}; display: ${
            DEBUG_MODE ? "block" : "none"
          };">
              <p>Captured Frames (${uid})</p>
              <img id="captured-image-${uid}" style="width: 100%; height: 100%; object-fit: contain;">
              <button id="download-frame-${uid}" class="btn btn-primary mt-2">Download Frame</button>
              <button id="download-base64-${uid}" class="btn btn-secondary mt-2 ml-2">Download Base64</button>
            </div>
          `);
          $("#captured-frames").append(capturedFrameDiv);
        }

        user.videoTrack.captureEnabled = true;
        
        // Forward video track to ROS2 bridge via WebRTC
        forwardVideoTrackToBridge(user, uid).catch((error) => {
          console.error(`Failed to forward video track for UID ${uid}:`, error);
        });
      }
      if (mediaType === "audio") {
        user.audioTrack.play();
      }

      resolve();
    } catch (error) {
      console.error(`subscribe error for ${uid} ${mediaType}:`, error);
      reject(error);
    }

    // Small delay between subscriptions to let WebRTC settle
    await new Promise(r => setTimeout(r, 100));
  }

  isProcessingSubscription = false;
}

/*
 * Add the local use to a remote channel.
 * Queues subscriptions to process them sequentially.
 *
 * @param  {IAgoraRTCRemoteUser} user - The {@link  https://docs.agora.io/en/Voice/API%20Reference/web_ng/interfaces/iagorartcremoteuser.html| remote user} to add.
 * @param {trackMediaType - The {@link https://docs.agora.io/en/Voice/API%20Reference/web_ng/interfaces/itrack.html#trackmediatype | media type} to add.
 */
async function subscribe(user, mediaType) {
  return new Promise((resolve, reject) => {
    subscriptionQueue.push({ user, mediaType, resolve, reject });
    processSubscriptionQueue();
  });
}

/*
 * Add a user who has subscribed to the live channel to the local interface.
 *
 * @param  {IAgoraRTCRemoteUser} user - The {@link  https://docs.agora.io/en/Voice/API%20Reference/web_ng/interfaces/iagorartcremoteuser.html| remote user} to add.
 * @param {trackMediaType - The {@link https://docs.agora.io/en/Voice/API%20Reference/web_ng/interfaces/itrack.html#trackmediatype | media type} to add.
 */
async function handleUserPublished(user, mediaType) {
  const id = user.uid;
  remoteUsers[id] = user;
  try {
    await subscribe(user, mediaType);
  } catch (error) {
    console.error(`Failed to subscribe to user ${id} ${mediaType}:`, error);
  }
}

/*
 * Remove the user specified from the channel in the local interface.
 *
 * @param  {string} user - The {@link  https://docs.agora.io/en/Voice/API%20Reference/web_ng/interfaces/iagorartcremoteuser.html| remote user} to remove.
 */
function handleUserUnpublished(user, mediaType) {
  if (mediaType === "video") {
    const id = user.uid;
    delete remoteUsers[id];
    $(`#player-wrapper-${id}`).remove();
    
    // Close bridge peer connection if it exists
    if (bridgePeerConnections[id]) {
      console.log(`[WebRTC Bridge] Closing bridge connection for UID ${id}`);
      const pc = bridgePeerConnections[id];
      
      // Clean up video elements and canvas used for forwarding
      pc.getSenders().forEach(sender => {
        const track = sender.track;
        if (track && track._sourceVideoElement) {
          track._sourceVideoElement.pause();
          track._sourceVideoElement.srcObject = null;
          if (track._sourceVideoElement.parentNode) {
            track._sourceVideoElement.parentNode.removeChild(track._sourceVideoElement);
          }
        }
        if (track && track._sourceCanvas) {
          // Canvas cleanup handled by removing video element
        }
      });
      
      pc.close();
      delete bridgePeerConnections[id];
    }
  }
}
function getCodec() {
  var radios = document.getElementsByName("radios");
  var value;
  for (var i = 0; i < radios.length; i++) {
    if (radios[i].checked) {
      value = radios[i].value;
    }
  }
  return value;
}

async function captureFrameAsBase64(videoTrack) {
  const frame = await videoTrack.getCurrentFrameData();
  const canvas = document.createElement("canvas");
  canvas.width = frame.width;
  canvas.height = frame.height;
  const ctx = canvas.getContext("2d");
  ctx.putImageData(frame, 0, 0);
  return canvas.toDataURL(
    `image/${window.imageParams["imageFormat"]}`,
    window.imageParams["imageQuality"]
  );
}

// Add at the beginning of the file
const DEBUG_MODE = false;
const lastBase64Frames = {};

/*
 * Forward Agora video track to ROS2 bridge via WebRTC
 * @param {IAgoraRTCRemoteUser} user - The remote user with video track
 * @param {number} uid - User ID (1000 = front, 1001 = rear)
 */
async function forwardVideoTrackToBridge(user, uid) {
  // Check if bridge URL is configured
  const bridgeUrl = window.webrtcBridgeUrl;
  if (!bridgeUrl) {
    console.log("WebRTC bridge URL not configured, skipping forwarding");
    return;
  }

  // Check if connection already exists
  if (bridgePeerConnections[uid]) {
    console.log(`Bridge connection for UID ${uid} already exists`);
    return;
  }

  // Only forward front (1000) and rear (1001) cameras
  if (uid !== 1000 && uid !== 1001) {
    console.log(`Skipping forwarding for UID ${uid} (only forwarding 1000 and 1001)`);
    return;
  }

  if (!user.videoTrack) {
    console.log(`No video track for UID ${uid}`);
    return;
  }

  try {
    console.log(`Starting WebRTC forwarding for UID ${uid} to bridge ${bridgeUrl}`);

    // Create peer connection to bridge
    const pc = new RTCPeerConnection({
      iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
    });

    // Store connection
    bridgePeerConnections[uid] = pc;
    
    // Create control data channel (only for front camera UID 1000)
    let controlDataChannel = null;
    if (uid === 1000) {
      try {
        controlDataChannel = pc.createDataChannel('control', {
          ordered: true, // Ensure message ordering
        });
        
        controlDataChannel.onopen = () => {
          console.log(`[WebRTC Bridge] Control data channel opened for UID ${uid}`);
        };
        
        controlDataChannel.onclose = () => {
          console.log(`[WebRTC Bridge] Control data channel closed for UID ${uid}`);
        };
        
        controlDataChannel.onerror = (error) => {
          console.error(`[WebRTC Bridge] Control data channel error for UID ${uid}:`, error);
        };
        
        // Store control data channel globally for interception
        window.bridgeControlDataChannel = controlDataChannel;
        
        console.log(`[WebRTC Bridge] Control data channel created for UID ${uid}`);
      } catch (error) {
        console.error(`[WebRTC Bridge] Failed to create control data channel:`, error);
      }
    }

    // Handle connection state changes
    pc.onconnectionstatechange = () => {
      const state = pc.connectionState;
      console.log(`[WebRTC Bridge] Connection state for UID ${uid}: ${state}`);
      if (state === 'connected') {
        console.log(`[WebRTC Bridge] Successfully connected UID ${uid} video track to ROS2 bridge`);
      } else if (state === 'failed' || state === 'closed') {
        console.warn(`[WebRTC Bridge] Connection ${state} for UID ${uid}, cleaning up`);
        delete bridgePeerConnections[uid];
      } else if (state === 'disconnected') {
        console.warn(`[WebRTC Bridge] Connection disconnected for UID ${uid}`);
      }
    };

    // Handle ICE connection state
    pc.oniceconnectionstatechange = () => {
      const iceState = pc.iceConnectionState;
      console.log(`[WebRTC Bridge] ICE connection state for UID ${uid}: ${iceState}`);
      if (iceState === 'failed') {
        console.error(`[WebRTC Bridge] ICE connection failed for UID ${uid}`);
      } else if (iceState === 'connected' || iceState === 'completed') {
        console.log(`[WebRTC Bridge] ICE connection ${iceState} for UID ${uid}`);
      }
    };
    
    // Handle ICE gathering state
    pc.onicegatheringstatechange = () => {
      console.log(`[WebRTC Bridge] ICE gathering state for UID ${uid}: ${pc.iceGatheringState}`);
    };

    // Forward the Agora video track to the bridge
    // Try to use the Agora track directly first (it may already have the correct ID)
    // If that doesn't work or isn't compatible, use canvas-based forwarding
    let forwardedTrack;
    let videoElement = null;
    let canvas = null;
    
    // Check if Agora track ID contains the UID (for bridge identification)
    const agoraTrackId = String(user.videoTrack.id || '');
    const hasCorrectId = (uid === 1000 && agoraTrackId.includes('1000')) || 
                         (uid === 1001 && agoraTrackId.includes('1001'));
    
    if (hasCorrectId) {
      // Try to use Agora track directly - it may work if it's a standard MediaStreamTrack
      try {
        forwardedTrack = user.videoTrack;
        console.log(`Using Agora track directly for UID ${uid} (ID: ${agoraTrackId})`);
      } catch (error) {
        console.warn('Failed to use Agora track directly, using canvas fallback:', error);
        forwardedTrack = null;
      }
    }
    
    // If direct track doesn't work, use canvas-based forwarding
    if (!forwardedTrack) {
      console.log(`Using canvas-based forwarding for UID ${uid}`);
      // Create a hidden video element to play the Agora track
      videoElement = document.createElement('video');
      videoElement.style.display = 'none';
      videoElement.autoplay = true;
      videoElement.playsInline = true;
      videoElement.srcObject = new MediaStream([user.videoTrack]);
      document.body.appendChild(videoElement);
      
      // Wait for video to be ready
      await new Promise((resolve) => {
        videoElement.onloadedmetadata = () => {
          videoElement.play().then(resolve).catch(resolve);
        };
      });

      // Create canvas to capture frames
      canvas = document.createElement('canvas');
      canvas.width = videoElement.videoWidth || 640;
      canvas.height = videoElement.videoHeight || 480;
      const ctx = canvas.getContext('2d');

      // Capture stream from canvas (this creates a new track with a new ID)
      const canvasStream = canvas.captureStream(30); // 30 FPS
      forwardedTrack = canvasStream.getVideoTracks()[0];
      
      // Draw frames from video to canvas in a loop
      const drawFrame = () => {
        if (videoElement.readyState >= 2) { // HAVE_CURRENT_DATA
          ctx.drawImage(videoElement, 0, 0, canvas.width, canvas.height);
        }
        if (forwardedTrack.readyState === 'live') {
          requestAnimationFrame(drawFrame);
        }
      };
      drawFrame();

      // Store video element and canvas for cleanup
      forwardedTrack._sourceVideoElement = videoElement;
      forwardedTrack._sourceCanvas = canvas;
    }

    // Create a stream with the forwarded track and add to peer connection
    const stream = new MediaStream([forwardedTrack]);
    pc.addTrack(forwardedTrack, stream);

    // Create offer
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);

    // Send offer to bridge
    const response = await fetch(`${bridgeUrl}/offer`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        sdp: pc.localDescription.sdp,
        type: pc.localDescription.type,
      }),
    });

    if (!response.ok) {
      throw new Error(`Bridge offer failed: ${response.status} ${response.statusText}`);
    }

    const answer = await response.json();
    await pc.setRemoteDescription(new RTCSessionDescription(answer));

    console.log(`[WebRTC Bridge] SDP negotiation completed for UID ${uid}, waiting for ICE connection...`);
  } catch (error) {
    console.error(`[WebRTC Bridge] Error forwarding video track for UID ${uid} to bridge:`, error);
    // Clean up on error
    if (bridgePeerConnections[uid]) {
      const pc = bridgePeerConnections[uid];
      // Clean up video elements if they exist
      pc.getSenders().forEach(sender => {
        const track = sender.track;
        if (track && track._sourceVideoElement) {
          track._sourceVideoElement.pause();
          track._sourceVideoElement.srcObject = null;
          if (track._sourceVideoElement.parentNode) {
            track._sourceVideoElement.parentNode.removeChild(track._sourceVideoElement);
          }
        }
      });
      pc.close();
      delete bridgePeerConnections[uid];
    }
  }
}

// Function to get the latest base64 frame for a specific UID
async function getLastBase64Frame(uid) {
  const user = remoteUsers[uid];
  if (!user || !user.videoTrack || !user.videoTrack.captureEnabled) {
    return null;
  }

  const base64Frame = await captureFrameAsBase64(user.videoTrack);
  lastBase64Frames[uid] = base64Frame;
  return base64Frame;
}

function initializeImageParams({ imageFormat, imageQuality }) {
  window.imageParams = { imageFormat, imageQuality };
}
window.initializeImageParams = initializeImageParams;
window.getLastBase64Frame = getLastBase64Frame
