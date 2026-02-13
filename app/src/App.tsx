import { useState, useEffect, useRef } from 'react'
import {
  AppShell,
  Burger,
  Group,
  Text,
  NavLink,
  Stack,
  Card,
  Title,
  Grid,
  Button,
  Code,
  ScrollArea,
  Badge,
} from '@mantine/core'
import { useDisclosure } from '@mantine/hooks'

interface LogMessage {
  topic: string
  data: string
  timestamp: string
  type: string
}

const API_BASE = 'http://localhost:8001'

interface DriveState {
  linear: number
  angular: number
  lamp: number
}

const WASD_LINEAR_FWD = 0.7
const WASD_LINEAR_BACK = -0.5
const WASD_ANGULAR_LEFT = 0.6
const WASD_ANGULAR_RIGHT = -0.6

function App() {
  const [opened, { toggle }] = useDisclosure()
  const [logs, setLogs] = useState<LogMessage[]>([])
  const [isConnected, setIsConnected] = useState(false)
  const [controlStatus, setControlStatus] = useState<'idle' | 'sending' | 'sent' | 'error'>('idle')
  const [driveState, setDriveState] = useState<DriveState>({ linear: 0, angular: 0, lamp: 0 })
  const [videoError, setVideoError] = useState(false)
  const [videoMode, setVideoMode] = useState<'webrtc' | 'mjpeg'>('webrtc')
  const [videoErrorOccupancy, setVideoErrorOccupancy] = useState(false)
  const [videoModeOccupancy, setVideoModeOccupancy] = useState<'webrtc' | 'mjpeg'>('webrtc')
  const [occupancyReceivedFrames, setOccupancyReceivedFrames] = useState(false)
  const [controlMethod, setControlMethod] = useState<'webrtc' | 'http' | 'connecting'>('connecting')
  const [webrtcConnectionState, setWebrtcConnectionState] = useState<string>('new')
  const driveRef = useRef<DriveState>({ linear: 0, angular: 0, lamp: 0 })
  const keysPressedRef = useRef<Set<string>>(new Set()) // Track which keys are currently pressed
  const lastKeyEventRef = useRef<number>(0) // For safety: force stop if keyup missed (e.g. focus loss)
  const wsRef = useRef<WebSocket | null>(null)
  const logsEndRef = useRef<HTMLDivElement>(null)
  const videoRef = useRef<HTMLVideoElement>(null)
  const videoRefOccupancy = useRef<HTMLVideoElement>(null)
  const pcRef = useRef<RTCPeerConnection | null>(null)
  const pcOccupancyRef = useRef<RTCPeerConnection | null>(null)
  const dataChannelRef = useRef<RTCDataChannel | null>(null)
  const httpFallbackTimeoutRef = useRef<number | null>(null)
  const webrtcStatsRef = useRef<{ webrtc: number; http: number }>({ webrtc: 0, http: 0 })

  const sendControl = (linear: number, angular: number, lamp: number) => {
    const payload = { command: { linear, angular, lamp } }
    const dc = dataChannelRef.current
    
    // Prioritize WebRTC data channel if it's open
    if (dc?.readyState === 'open') {
      try {
        dc.send(JSON.stringify(payload))
        webrtcStatsRef.current.webrtc++
        if (controlMethod !== 'webrtc') {
          setControlMethod('webrtc')
          console.log('[WebRTC] Control command sent via WebRTC data channel')
        }
        return
      } catch (error) {
        console.warn('[WebRTC] Failed to send via data channel, falling back to HTTP:', error)
        // Fall through to HTTP fallback
      }
    }
    
    // Fallback to HTTP only if WebRTC is not available or failed
    // Use exponential backoff to avoid spamming HTTP if WebRTC is preferred
    if (controlMethod === 'webrtc' && dc?.readyState !== 'closed') {
      // WebRTC was working but temporarily unavailable, wait a bit before falling back
      if (!httpFallbackTimeoutRef.current) {
        httpFallbackTimeoutRef.current = window.setTimeout(() => {
          httpFallbackTimeoutRef.current = null
        }, 100)
        return // Skip this command, will retry next interval
      }
    }
    
    // HTTP fallback
    fetch(`${API_BASE}/api/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    })
      .then(() => {
        webrtcStatsRef.current.http++
        if (controlMethod !== 'http') {
          setControlMethod('http')
          console.log('[HTTP] Control command sent via HTTP fallback')
        }
      })
      .catch((error) => {
        console.error('[HTTP] Failed to send control command:', error)
      })
  }

  useEffect(() => {
    let cancelled = false
    let reconnectTimeout: number | null = null
    let reconnectAttempts = 0
    const MAX_RECONNECT_ATTEMPTS = Infinity // Keep trying forever
    const RECONNECT_DELAY = 3000 // 3 seconds

    const connectWebSocket = () => {
      if (cancelled) return

      try {
        const ws = new WebSocket('ws://localhost:8001/ws')
        wsRef.current = ws

        ws.onopen = () => {
          setIsConnected(true)
          reconnectAttempts = 0 // Reset on successful connection
          console.log('WebSocket connected')
        }

        ws.onmessage = (event) => {
          try {
            const message: LogMessage = JSON.parse(event.data)
            setLogs((prev) => [...prev, message].slice(-100)) // Keep last 100 logs
          } catch (error) {
            console.error('Error parsing WebSocket message:', error)
          }
        }

        ws.onerror = (error) => {
          console.error('WebSocket error:', error)
          setIsConnected(false)
        }

        ws.onclose = (event) => {
          setIsConnected(false)
          console.log('WebSocket closed', event.code, event.reason)
          
          // Attempt reconnection if not cancelled and not a normal closure
          if (!cancelled && event.code !== 1000) {
            reconnectAttempts++
            const delay = Math.min(RECONNECT_DELAY * reconnectAttempts, 30000) // Max 30s delay
            console.log(`WebSocket reconnecting in ${delay}ms (attempt ${reconnectAttempts})...`)
            reconnectTimeout = window.setTimeout(() => {
              if (!cancelled) {
                connectWebSocket()
              }
            }, delay)
          }
        }
      } catch (error) {
        console.error('Failed to create WebSocket:', error)
        setIsConnected(false)
        // Retry connection
        if (!cancelled) {
          reconnectTimeout = window.setTimeout(() => {
            if (!cancelled) {
              connectWebSocket()
            }
          }, RECONNECT_DELAY)
        }
      }
    }

    // Initial connection with small delay for React Strict Mode
    const timeoutId = setTimeout(() => {
      connectWebSocket()
    }, 100)

    return () => {
      cancelled = true
      clearTimeout(timeoutId)
      if (reconnectTimeout !== null) {
        clearTimeout(reconnectTimeout)
      }
      if (wsRef.current?.readyState === WebSocket.OPEN || wsRef.current?.readyState === WebSocket.CONNECTING) {
        wsRef.current.close()
      }
      wsRef.current = null
    }
  }, [])

  useEffect(() => {
    // Auto-scroll to bottom when new logs arrive
    logsEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [logs])

  // Poll occupancy status so we can show "Waiting for stream..." when no frames yet
  useEffect(() => {
    const check = async () => {
      try {
        const res = await fetch(`${API_BASE}/api/occupancy/status`)
        if (res.ok) {
          const data = await res.json()
          setOccupancyReceivedFrames(Boolean(data.received_frames))
        }
      } catch {
        // ignore
      }
    }
    check()
    const id = setInterval(check, 2000)
    return () => clearInterval(id)
  }, [])

  // WebRTC: video + control data channel; create offer, POST to server, attach remote track to <video>; fallback to MJPEG/HTTP on failure
  useEffect(() => {
    let cancelled = false
    let reconnectAttempts = 0
    const MAX_RECONNECT_ATTEMPTS = 3
    let reconnectTimeout: number | null = null
    let videoTrackTimeout: number | null = null
    let videoTrackReceived = false
    
    const setupWebRTC = () => {
      if (cancelled) return
      
      // Reset video track tracking for new connection
      videoTrackReceived = false
      if (videoTrackTimeout) {
        clearTimeout(videoTrackTimeout)
        videoTrackTimeout = null
      }
      
      const pc = new RTCPeerConnection({
        iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
      })
      pcRef.current = pc
      setWebrtcConnectionState('new')
      setControlMethod('connecting')

      // Create data channel BEFORE creating offer (required for proper negotiation)
      const dc = pc.createDataChannel('control', {
        ordered: true, // Ensure message ordering
      })
      dataChannelRef.current = dc

      // Request video so server includes front camera track in answer
      pc.addTransceiver('video', { direction: 'recvonly' })
      
      // Enhanced data channel event handlers with logging
      dc.onopen = () => {
        if (cancelled) return
        console.log('[WebRTC] Data channel opened successfully')
        setControlMethod('webrtc')
        reconnectAttempts = 0 // Reset on successful connection
        // Note: Don't set videoMode to 'webrtc' here - wait for video track
      }
      
      dc.onclose = () => {
        console.log('[WebRTC] Data channel closed')
        if (!cancelled) {
          dataChannelRef.current = null
          setControlMethod('http')
          // Attempt reconnection if not cancelled
          if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
            reconnectAttempts++
            console.log(`[WebRTC] Attempting reconnection (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})...`)
            reconnectTimeout = window.setTimeout(() => {
              if (!cancelled) {
                setupWebRTC()
              }
            }, 2000 * reconnectAttempts) // Exponential backoff
          } else {
            console.warn('[WebRTC] Max reconnection attempts reached, using HTTP fallback')
          }
        }
      }
      
      dc.onerror = (error) => {
        console.error('[WebRTC] Data channel error:', error)
        if (!cancelled) {
          dataChannelRef.current = null
          setControlMethod('http')
        }
      }
      
      dc.onmessage = (event) => {
        // Handle any messages from server (if needed in future)
        console.log('[WebRTC] Received message on data channel:', event.data)
      }

      // Track connection state changes
      pc.onconnectionstatechange = () => {
        const state = pc.connectionState
        setWebrtcConnectionState(state)
        console.log(`[WebRTC] Connection state: ${state}`)
        
        if (state === 'failed' || state === 'closed' || state === 'disconnected') {
          if (videoTrackTimeout) {
            clearTimeout(videoTrackTimeout)
            videoTrackTimeout = null
          }
          if (!cancelled) {
            setVideoMode('mjpeg')
            setVideoError(false)
            if (state === 'failed' && reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
              reconnectAttempts++
              console.log(`[WebRTC] Connection failed, attempting reconnection (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})...`)
              reconnectTimeout = window.setTimeout(() => {
                if (!cancelled) {
                  setupWebRTC()
                }
              }, 3000 * reconnectAttempts)
            }
          }
        } else if (state === 'connected') {
          console.log('[WebRTC] Peer connection established')
          // If connected but no video track received within 3 seconds, fall back to MJPEG
          if (!videoTrackReceived) {
            videoTrackTimeout = window.setTimeout(() => {
              if (!cancelled && !videoTrackReceived) {
                console.warn('[WebRTC] Connection established but no video track received within 3s, falling back to MJPEG')
                setVideoMode('mjpeg')
                setVideoError(false)
              }
            }, 3000)
          }
        }
      }
      
      // Track ICE connection state
      pc.oniceconnectionstatechange = () => {
        console.log(`[WebRTC] ICE connection state: ${pc.iceConnectionState}`)
      }
      
      // Track ICE gathering state
      pc.onicegatheringstatechange = () => {
        console.log(`[WebRTC] ICE gathering state: ${pc.iceGatheringState}`)
      }

      pc.ontrack = (event) => {
        if (cancelled || !videoRef.current) return
        
        // Only handle video tracks
        if (event.track.kind !== 'video') {
          console.log('[WebRTC] Non-video track received:', event.track.kind)
          return
        }
        
        console.log('[WebRTC] Front camera video track received:', event.track.kind, event.track.id)
        videoTrackReceived = true
        
        // Clear the timeout since we got a video track
        if (videoTrackTimeout) {
          clearTimeout(videoTrackTimeout)
          videoTrackTimeout = null
        }
        
        const stream = event.streams[0] ?? new MediaStream([event.track])
        
        // Ensure video element is ready
        if (videoRef.current) {
          videoRef.current.srcObject = stream
          
          // Play the video explicitly
          videoRef.current.play().then(() => {
            console.log('[WebRTC] Front camera video playing')
            setVideoMode('webrtc')
            setVideoError(false)
          }).catch((error) => {
            console.error('[WebRTC] Failed to play front camera video:', error)
            setVideoMode('mjpeg') // Fall back to MJPEG if play fails
            setVideoError(false)
          })
          
          // Monitor track state
          event.track.onended = () => {
            console.warn('[WebRTC] Front camera track ended')
            setVideoMode('mjpeg') // Fall back to MJPEG when track ends
            setVideoError(false)
          }
          
          event.track.onmute = () => {
            console.warn('[WebRTC] Front camera track muted')
          }
          
          event.track.onunmute = () => {
            console.log('[WebRTC] Front camera track unmuted')
          }
        }
      }

      const connect = async () => {
        try {
          console.log('[WebRTC] Creating offer...')
          const offer = await pc.createOffer()
          await pc.setLocalDescription(offer)
          console.log('[WebRTC] Sending offer to server...')
          
          const res = await fetch(`${API_BASE}/api/webrtc/offer`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ sdp: pc.localDescription?.sdp, type: pc.localDescription?.type }),
          })
          
          if (cancelled) return
          
          if (!res.ok) {
            const data = await res.json().catch(() => ({}))
            console.warn('[WebRTC] Offer failed:', res.status, data)
            setVideoMode('mjpeg')
            setVideoError(false)
            setControlMethod('http')
            return
          }
          
          const answer = await res.json()
          await pc.setRemoteDescription(new RTCSessionDescription(answer))
          console.log('[WebRTC] Answer received, connection establishing...')
        } catch (e) {
          if (!cancelled) {
            console.error('[WebRTC] Connection error:', e)
            setVideoMode('mjpeg')
            setVideoError(false)
            setControlMethod('http')
          }
        }
      }
      
      connect()
    }
    
    // Initial setup
    setupWebRTC()

    // Periodic health check for data channel
    const healthCheckInterval = setInterval(() => {
      if (cancelled) return
      const dc = dataChannelRef.current
      const pc = pcRef.current
      
      if (dc && pc) {
        if (dc.readyState === 'open' && pc.connectionState === 'connected') {
          // Connection is healthy
          if (controlMethod !== 'webrtc') {
            setControlMethod('webrtc')
          }
        } else if (dc.readyState === 'closed' || pc.connectionState === 'failed') {
          // Connection is down
          if (controlMethod === 'webrtc') {
            console.warn('[WebRTC] Health check: connection down, switching to HTTP')
            setControlMethod('http')
          }
        }
      }
    }, 5000) // Check every 5 seconds

    return () => {
      cancelled = true
      if (reconnectTimeout !== null) {
        clearTimeout(reconnectTimeout)
      }
      if (videoTrackTimeout !== null) {
        clearTimeout(videoTrackTimeout)
      }
      clearInterval(healthCheckInterval)
      dataChannelRef.current = null
      if (pcRef.current) {
        pcRef.current.close()
        pcRef.current = null
      }
      if (videoRef.current) {
        videoRef.current.srcObject = null
      }
    }
  }, [])

  // WebRTC: occupancy stream (annotated floor overlay); second connection, no data channel
  useEffect(() => {
    let cancelled = false
    let videoTrackReceivedOccupancy = false
    let videoTrackTimeoutOccupancy: number | null = null
    const pc = new RTCPeerConnection({
      iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
    })
    pcOccupancyRef.current = pc

    // Request video so server includes occupancy track in answer
    pc.addTransceiver('video', { direction: 'recvonly' })

    pc.ontrack = (event) => {
      if (cancelled || !videoRefOccupancy.current) return
      console.log('[WebRTC] Occupancy video track received:', event.track.kind, event.track.id, 'readyState:', event.track.readyState)
      videoTrackReceivedOccupancy = true
      if (videoTrackTimeoutOccupancy) {
        clearTimeout(videoTrackTimeoutOccupancy)
        videoTrackTimeoutOccupancy = null
      }
      const stream = event.streams[0] ?? new MediaStream([event.track])
      
      // Ensure video element is ready
      if (videoRefOccupancy.current) {
        videoRefOccupancy.current.srcObject = stream
        
        // Play the video explicitly
        videoRefOccupancy.current.play().then(() => {
          console.log('[WebRTC] Occupancy video playing successfully')
          setVideoModeOccupancy('webrtc')
          setVideoErrorOccupancy(false)
        }).catch((error) => {
          console.error('[WebRTC] Failed to play occupancy video:', error)
          setVideoModeOccupancy('mjpeg') // Fallback to MJPEG
          setVideoErrorOccupancy(false)
        })
        
        // Monitor track state
        event.track.onended = () => {
          console.warn('[WebRTC] Occupancy track ended')
          setVideoModeOccupancy('mjpeg')
          setVideoErrorOccupancy(false)
        }
        
        event.track.onmute = () => {
          console.warn('[WebRTC] Occupancy track muted')
        }
        
        event.track.onunmute = () => {
          console.log('[WebRTC] Occupancy track unmuted')
        }
      }
    }

    pc.onconnectionstatechange = () => {
      const state = pc.connectionState
      console.log(`[WebRTC Occupancy] Connection state: ${state}`)
      if (state === 'failed' || state === 'closed' || state === 'disconnected') {
        if (videoTrackTimeoutOccupancy) {
          clearTimeout(videoTrackTimeoutOccupancy)
          videoTrackTimeoutOccupancy = null
        }
        if (!cancelled) {
          console.warn(`[WebRTC Occupancy] Connection ${state}, falling back to MJPEG`)
          setVideoModeOccupancy('mjpeg')
          setVideoErrorOccupancy(false)
        }
      } else if (state === 'connected') {
        console.log('[WebRTC Occupancy] Peer connection established')
        if (!videoTrackReceivedOccupancy) {
          videoTrackTimeoutOccupancy = window.setTimeout(() => {
            if (!cancelled && !videoTrackReceivedOccupancy) {
              console.warn('[WebRTC Occupancy] Connection established but no video track within 5s, falling back to MJPEG')
              setVideoModeOccupancy('mjpeg')
              setVideoErrorOccupancy(false)
            }
            videoTrackTimeoutOccupancy = null
          }, 5000)
        }
      }
    }
    
    pc.oniceconnectionstatechange = () => {
      console.log(`[WebRTC Occupancy] ICE connection state: ${pc.iceConnectionState}`)
    }
    
    pc.onicegatheringstatechange = () => {
      console.log(`[WebRTC Occupancy] ICE gathering state: ${pc.iceGatheringState}`)
    }

    const connect = async () => {
      try {
        console.log('[WebRTC Occupancy] Creating offer...')
        const offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        console.log('[WebRTC Occupancy] Sending offer to server...')
        
        const res = await fetch(`${API_BASE}/api/webrtc/occupancy/offer`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ sdp: pc.localDescription?.sdp, type: pc.localDescription?.type }),
        })
        
        if (cancelled) return
        
        if (!res.ok) {
          const data = await res.json().catch(() => ({}))
          console.warn('[WebRTC Occupancy] Offer failed:', res.status, data)
          setVideoModeOccupancy('mjpeg')
          setVideoErrorOccupancy(false)
          return
        }
        
        const answer = await res.json()
        await pc.setRemoteDescription(new RTCSessionDescription(answer))
        console.log('[WebRTC Occupancy] Answer received, connection establishing...')
      } catch (e) {
        if (!cancelled) {
          console.error('[WebRTC Occupancy] Connection error:', e)
          setVideoModeOccupancy('mjpeg')
          setVideoErrorOccupancy(false)
        }
      }
    }
    connect()

    return () => {
      cancelled = true
      if (videoTrackTimeoutOccupancy) {
        clearTimeout(videoTrackTimeoutOccupancy)
        videoTrackTimeoutOccupancy = null
      }
      pc.close()
      pcOccupancyRef.current = null
      if (videoRefOccupancy.current) {
        videoRefOccupancy.current.srcObject = null
      }
    }
  }, [])

  const SEND_INTERVAL_MS = 10 // 100 Hz for finer control resolution
  useEffect(() => {
    const id = setInterval(() => {
      // Only send commands if keys are currently pressed
      // Zero commands are sent immediately on key release, not continuously
      if (keysPressedRef.current.size > 0) {
        lastKeyEventRef.current = Date.now() // so safety timeout does not fire while holding
        const { linear, angular, lamp } = driveRef.current
        sendControl(linear, angular, lamp)
      }
    }, SEND_INTERVAL_MS)
    return () => clearInterval(id)
  }, [])

  useEffect(() => {
    const isEditable = (el: Element | null) => {
      if (!el) return false
      const tag = (el as HTMLElement).tagName?.toLowerCase()
      if (tag === 'input' || tag === 'textarea') return true
      return (el as HTMLElement).isContentEditable === true
    }
    const SAFETY_KEY_TIMEOUT_MS = 150
    const sendStopBurst = (lamp: number) => {
      // One immediate stop so the very next command in the pipeline is stop
      sendControl(0, 0, lamp)
      // Then flood stop for 200ms so robot/dashboard keeps receiving stop (overrides any fixed-duration "go")
      const STOP_BURST_COUNT = 20
      const STOP_BURST_SPACING_MS = 10
      for (let i = 1; i <= STOP_BURST_COUNT; i++) {
        setTimeout(() => sendControl(0, 0, lamp), i * STOP_BURST_SPACING_MS)
      }
    }

    const onKeyDown = (e: KeyboardEvent) => {
      if (isEditable(document.activeElement)) return
      if (e.repeat) return // ignore key repeat; interval keeps sending
      lastKeyEventRef.current = Date.now()
      const key = e.key?.toLowerCase()
      const r = driveRef.current
      let changed = false
      if (key === 'w') {
        r.linear = WASD_LINEAR_FWD
        keysPressedRef.current.add(key)
        e.preventDefault()
        changed = true
      } else if (key === 's') {
        r.linear = WASD_LINEAR_BACK
        keysPressedRef.current.add(key)
        e.preventDefault()
        changed = true
      } else if (key === 'a') {
        r.angular = WASD_ANGULAR_LEFT
        keysPressedRef.current.add(key)
        e.preventDefault()
        changed = true
      } else if (key === 'd') {
        r.angular = WASD_ANGULAR_RIGHT
        keysPressedRef.current.add(key)
        e.preventDefault()
        changed = true
      }
      if (changed) {
        setDriveState({ ...r })
        sendControl(r.linear, r.angular, r.lamp) // immediate first command
      }
    }
    const onKeyUp = (e: KeyboardEvent) => {
      if (isEditable(document.activeElement)) return
      if (e.repeat) return
      lastKeyEventRef.current = Date.now()
      const key = e.key?.toLowerCase()
      const r = driveRef.current
      let changed = false
      if (key === 'w' || key === 's') {
        keysPressedRef.current.delete('w')
        keysPressedRef.current.delete('s')
        // Only set linear to 0 if no other linear key is pressed
        if (!keysPressedRef.current.has('w') && !keysPressedRef.current.has('s')) {
          r.linear = 0
          changed = true
        } else if (keysPressedRef.current.has('w')) {
          r.linear = WASD_LINEAR_FWD
          changed = true
        } else if (keysPressedRef.current.has('s')) {
          r.linear = WASD_LINEAR_BACK
          changed = true
        }
      } else if (key === 'a' || key === 'd') {
        keysPressedRef.current.delete('a')
        keysPressedRef.current.delete('d')
        // Only set angular to 0 if no other angular key is pressed
        if (!keysPressedRef.current.has('a') && !keysPressedRef.current.has('d')) {
          r.angular = 0
          changed = true
        } else if (keysPressedRef.current.has('a')) {
          r.angular = WASD_ANGULAR_LEFT
          changed = true
        } else if (keysPressedRef.current.has('d')) {
          r.angular = WASD_ANGULAR_RIGHT
          changed = true
        }
      }
      if (changed) {
        setDriveState({ ...r })
        const isFullStop = r.linear === 0 && r.angular === 0
        if (isFullStop) {
          sendStopBurst(r.lamp)
        } else {
          sendControl(r.linear, r.angular, r.lamp)
        }
      }
    }
    document.addEventListener('keydown', onKeyDown)
    document.addEventListener('keyup', onKeyUp)

    const safetyInterval = setInterval(() => {
      if (keysPressedRef.current.size > 0 && Date.now() - lastKeyEventRef.current > SAFETY_KEY_TIMEOUT_MS) {
        driveRef.current = { linear: 0, angular: 0, lamp: driveRef.current.lamp }
        keysPressedRef.current.clear()
        setDriveState({ linear: 0, angular: 0, lamp: driveRef.current.lamp })
        sendStopBurst(driveRef.current.lamp)
      }
    }, SAFETY_KEY_TIMEOUT_MS)

    return () => {
      document.removeEventListener('keydown', onKeyDown)
      document.removeEventListener('keyup', onKeyUp)
      clearInterval(safetyInterval)
    }
  }, [])

  useEffect(() => {
    const onBlur = () => {
      driveRef.current = { linear: 0, angular: 0, lamp: 0 }
      setDriveState({ linear: 0, angular: 0, lamp: 0 })
      sendControl(0, 0, 0)
    }
    window.addEventListener('blur', onBlur)
    return () => window.removeEventListener('blur', onBlur)
  }, [])

  const formatTimestamp = (timestamp: string) => {
    try {
      const date = new Date(timestamp)
      return date.toLocaleTimeString()
    } catch {
      return timestamp
    }
  }

  const sendFrodobotsControl = async () => {
    setControlStatus('sending')
    const payload = { command: { linear: 1, angular: 1, lamp: 0 } }
    const dc = dataChannelRef.current
    if (dc?.readyState === 'open') {
      try {
        dc.send(JSON.stringify(payload))
        setControlStatus('sent')
        setTimeout(() => setControlStatus('idle'), 2000)
      } catch {
        setControlStatus('error')
        setTimeout(() => setControlStatus('idle'), 2000)
      }
      return
    }
    try {
      const res = await fetch(`${API_BASE}/api/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      })
      const data = await res.json().catch(() => ({}))
      if (res.ok && data.ok) {
        setControlStatus('sent')
        setTimeout(() => setControlStatus('idle'), 2000)
      } else {
        setControlStatus('error')
        setTimeout(() => setControlStatus('idle'), 2000)
      }
    } catch {
      setControlStatus('error')
      setTimeout(() => setControlStatus('idle'), 2000)
    }
  }

  return (
    <AppShell
      header={{ height: 60 }}
      navbar={{
        width: 300,
        breakpoint: 'sm',
        collapsed: { mobile: !opened },
      }}
      padding="md"
    >
      <AppShell.Header>
        <Group h="100%" px="md" justify="space-between">
          <Group>
            <Burger opened={opened} onClick={toggle} hiddenFrom="sm" size="sm" />
            <Title order={3}>Dashboard</Title>
          </Group>
          <Group gap="xs">
            <Badge color={isConnected ? 'green' : 'red'} variant="filled">
              {isConnected ? 'Connected' : 'Disconnected'}
            </Badge>
            <Badge 
              color={controlMethod === 'webrtc' ? 'green' : controlMethod === 'http' ? 'orange' : 'gray'} 
              variant="filled"
              title={`Control method: ${controlMethod}. WebRTC: ${webrtcStatsRef.current.webrtc} messages, HTTP: ${webrtcStatsRef.current.http} messages`}
            >
              {controlMethod === 'webrtc' ? 'WebRTC' : controlMethod === 'http' ? 'HTTP' : 'Connecting...'}
            </Badge>
            {webrtcConnectionState !== 'connected' && controlMethod === 'connecting' && (
              <Badge color="blue" variant="light">
                {webrtcConnectionState}
              </Badge>
            )}
          </Group>
        </Group>
      </AppShell.Header>

      <AppShell.Navbar p="md">
        <Stack gap="xs">
          <NavLink label="Home" active />
          <NavLink label="Analytics" />
          <NavLink label="Settings" />
          <NavLink label="Reports" />
        </Stack>
      </AppShell.Navbar>

      <AppShell.Main>
        <Stack gap="md">
          <Card shadow="sm" padding="lg" radius="md" withBorder>
            <Title order={2} mb="md">Welcome to Your Dashboard</Title>
            <Text size="sm" c="dimmed">
              This is a simple dashboard built with Mantine AppShell and Tailwind CSS.
            </Text>
          </Card>

          <Grid>
            <Grid.Col span={{ base: 12, sm: 6, md: 4 }}>
              <Card shadow="sm" padding="lg" radius="md" withBorder>
                <Title order={4} mb="xs">Stats</Title>
                <Text size="xl" fw={700}>1,234</Text>
                <Text size="sm" c="dimmed">Total Users</Text>
              </Card>
            </Grid.Col>
            <Grid.Col span={{ base: 12, sm: 6, md: 4 }}>
              <Card shadow="sm" padding="lg" radius="md" withBorder>
                <Title order={4} mb="xs">Revenue</Title>
                <Text size="xl" fw={700}>$12,345</Text>
                <Text size="sm" c="dimmed">This Month</Text>
              </Card>
            </Grid.Col>
            <Grid.Col span={{ base: 12, sm: 6, md: 4 }}>
              <Card shadow="sm" padding="lg" radius="md" withBorder>
                <Title order={4} mb="xs">Messages</Title>
                <Text size="xl" fw={700}>{logs.length}</Text>
                <Text size="sm" c="dimmed">ROS2 Messages</Text>
              </Card>
            </Grid.Col>
          </Grid>

          <Card shadow="sm" padding="lg" radius="md" withBorder>
            <Group justify="space-between" mb="md">
              <Title order={4}>ROS2 Logs</Title>
              <Button
                variant="light"
                size="xs"
                onClick={() => setLogs([])}
              >
                Clear Logs
              </Button>
            </Group>
            <ScrollArea h={400} type="scroll">
              <Stack gap="xs">
                {logs.length === 0 ? (
                  <Text c="dimmed" ta="center" py="xl">
                    No messages yet. Start ROS2 talker to see messages here.
                  </Text>
                ) : (
                  <>
                    {logs.map((log, index) => (
                      <Card key={index} padding="sm" withBorder>
                        <Group justify="space-between" mb="xs">
                          <Badge size="sm" variant="light">
                            {log.topic}
                          </Badge>
                          <Text size="xs" c="dimmed">
                            {formatTimestamp(log.timestamp)}
                          </Text>
                        </Group>
                        <Code block>{log.data}</Code>
                      </Card>
                    ))}
                    <div ref={logsEndRef} />
                  </>
                )}
              </Stack>
            </ScrollArea>
          </Card>

          <Grid>
            <Grid.Col span={{ base: 12, md: 6 }}>
              <Card shadow="sm" padding="lg" radius="md" withBorder>
                <Title order={4} mb="xs">Front camera</Title>
                <Text size="sm" c="dimmed" mb="sm">
                  {videoMode === 'webrtc'
                    ? 'Live WebRTC stream from robot (via ROS2).'
                    : 'Live MJPEG fallback from robot (via ROS2).'}
                </Text>
                <div style={{ minHeight: 240, background: '#1a1b1e', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                  {videoError ? (
                    <Text size="sm" c="dimmed">Stream unavailable. Ensure app-server is running and ros2_app_bridge is publishing frames.</Text>
                  ) : videoMode === 'webrtc' ? (
                    <video
                      ref={videoRef}
                      autoPlay
                      playsInline
                      muted
                      style={{ maxWidth: '100%', maxHeight: 360, objectFit: 'contain' }}
                      onLoadedMetadata={() => {
                        console.log('[Video] Front camera metadata loaded')
                        setVideoError(false)
                      }}
                      onError={(e) => {
                        console.error('[Video] Front camera error:', e)
                        setVideoError(true)
                      }}
                      onPlay={() => {
                        console.log('[Video] Front camera playing')
                        setVideoError(false)
                      }}
                    />
                  ) : (
                    <img
                      src={`${API_BASE}/api/video/front/stream`}
                      alt="Front camera"
                      style={{ maxWidth: '100%', maxHeight: 360, objectFit: 'contain' }}
                      onError={(e) => {
                        console.error('[MJPEG] Front camera image load error:', e)
                        setVideoError(true)
                      }}
                      onLoad={() => {
                        console.log('[MJPEG] Front camera image loaded successfully')
                        setVideoError(false)
                      }}
                    />
                  )}
                </div>
              </Card>
            </Grid.Col>
            <Grid.Col span={{ base: 12, md: 6 }}>
              <Card shadow="sm" padding="lg" radius="md" withBorder>
                <Title order={4} mb="xs">Occupancy (floor grid overlay)</Title>
                <Text size="sm" c="dimmed" mb="sm">
                  {videoModeOccupancy === 'webrtc'
                    ? 'Live WebRTC: floor segmentation + BEV mini-map (via ROS2).'
                    : 'Live MJPEG fallback: occupancy annotated stream.'}
                </Text>
                <div style={{ minHeight: 240, background: '#1a1b1e', display: 'flex', alignItems: 'center', justifyContent: 'center', position: 'relative' }}>
                  {videoErrorOccupancy ? (
                    <Text size="sm" c="dimmed">Occupancy stream unavailable. Ensure occupancy node and ros2_app_bridge are running.</Text>
                  ) : (
                    <>
                      {videoModeOccupancy === 'webrtc' ? (
                        <video
                          ref={videoRefOccupancy}
                          autoPlay
                          playsInline
                          muted
                          style={{ maxWidth: '100%', maxHeight: 360, objectFit: 'contain' }}
                          onLoadedMetadata={() => {
                            console.log('[Video] Occupancy metadata loaded')
                            setVideoErrorOccupancy(false)
                          }}
                          onError={(e) => {
                            console.error('[Video] Occupancy error:', e)
                            setVideoErrorOccupancy(true)
                          }}
                          onPlay={() => {
                            console.log('[Video] Occupancy playing')
                            setVideoErrorOccupancy(false)
                          }}
                        />
                      ) : (
                        <img
                          src={`${API_BASE}/api/video/occupancy/stream`}
                          alt="Occupancy"
                          style={{ maxWidth: '100%', maxHeight: 360, objectFit: 'contain' }}
                          onError={(e) => {
                            console.error('[MJPEG] Occupancy image load error:', e)
                            setVideoErrorOccupancy(true)
                          }}
                          onLoad={() => {
                            console.log('[MJPEG] Occupancy image loaded successfully')
                            setVideoErrorOccupancy(false)
                          }}
                        />
                      )}
                      {!occupancyReceivedFrames && (
                        <div style={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', background: 'rgba(0,0,0,0.7)' }}>
                          <Text size="sm" c="dimmed" ta="center" maw={320}>
                            Waiting for occupancy stream… Front camera must be active so the occupancy node can process frames. Check ros2_bridge and ros2_app_bridge.
                          </Text>
                        </div>
                      )}
                    </>
                  )}
                </div>
              </Card>
            </Grid.Col>
          </Grid>

          <Card shadow="sm" padding="lg" radius="md" withBorder>
            <Title order={4} mb="xs">Drive (WASD)</Title>
            <Text size="sm" c="dimmed" mb="sm">
              W forward, S back, A left, D right. Click page to enable.
            </Text>
            <Group gap="xs" mb="xs">
              <Badge 
                size="sm" 
                color={controlMethod === 'webrtc' ? 'green' : controlMethod === 'http' ? 'orange' : 'gray'}
                variant="light"
              >
                Control: {controlMethod === 'webrtc' ? 'WebRTC' : controlMethod === 'http' ? 'HTTP' : 'Connecting'}
              </Badge>
              <Text size="xs" c="dimmed">
                (WebRTC: {webrtcStatsRef.current.webrtc}, HTTP: {webrtcStatsRef.current.http})
              </Text>
            </Group>
            <Text size="xs" c="dimmed">
              Linear: {driveState.linear.toFixed(2)} · Angular: {driveState.angular.toFixed(2)}
            </Text>
          </Card>

          <Card shadow="sm" padding="lg" radius="md" withBorder>
            <Title order={4} mb="md">Quick Actions</Title>
            <Group>
              <Button
                color="orange"
                variant="filled"
                loading={controlStatus === 'sending'}
                onClick={sendFrodobotsControl}
              >
                {controlStatus === 'sent' ? 'Sent' : controlStatus === 'error' ? 'Error' : 'Send Frodobots control'}
              </Button>
              <Button variant="filled">Create New</Button>
              <Button variant="outline">Export Data</Button>
              <Button variant="light">View Reports</Button>
            </Group>
          </Card>
        </Stack>
      </AppShell.Main>
    </AppShell>
  )
}

export default App
