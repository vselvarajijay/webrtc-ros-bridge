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
  const driveRef = useRef<DriveState>({ linear: 0, angular: 0, lamp: 0 })
  const wsRef = useRef<WebSocket | null>(null)
  const logsEndRef = useRef<HTMLDivElement>(null)
  const videoRef = useRef<HTMLVideoElement>(null)
  const pcRef = useRef<RTCPeerConnection | null>(null)
  const dataChannelRef = useRef<RTCDataChannel | null>(null)

  const sendControl = (linear: number, angular: number, lamp: number) => {
    const payload = { command: { linear, angular, lamp } }
    const dc = dataChannelRef.current
    if (dc?.readyState === 'open') {
      try {
        dc.send(JSON.stringify(payload))
      } catch {
        fetch(`${API_BASE}/api/control`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        }).catch(() => {})
      }
    } else {
      fetch(`${API_BASE}/api/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      }).catch(() => {})
    }
  }

  useEffect(() => {
    // Delay connect so React Strict Mode cleanup can cancel the first attempt (avoids "closed before connection established")
    const timeoutId = setTimeout(() => {
      const ws = new WebSocket('ws://localhost:8001/ws')
      wsRef.current = ws

      ws.onopen = () => {
        setIsConnected(true)
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

      ws.onerror = () => {
        setIsConnected(false)
      }

      ws.onclose = () => {
        setIsConnected(false)
      }
    }, 100)

    return () => {
      clearTimeout(timeoutId)
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

  // WebRTC: video + control data channel; create offer, POST to server, attach remote track to <video>; fallback to MJPEG/HTTP on failure
  useEffect(() => {
    let cancelled = false
    const pc = new RTCPeerConnection({
      iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
    })
    pcRef.current = pc

    const dc = pc.createDataChannel('control')
    dataChannelRef.current = dc
    dc.onopen = () => {
      if (!cancelled) setVideoMode('webrtc')
    }
    dc.onclose = () => {
      dataChannelRef.current = null
    }
    dc.onerror = () => {
      dataChannelRef.current = null
    }

    pc.ontrack = (event) => {
      if (cancelled || !videoRef.current) return
      const stream = event.streams[0] ?? new MediaStream([event.track])
      videoRef.current.srcObject = stream
      setVideoMode('webrtc')
      setVideoError(false)
    }

    pc.onconnectionstatechange = () => {
      if (pc.connectionState === 'failed' || pc.connectionState === 'closed') {
        if (!cancelled) setVideoMode('mjpeg')
      }
    }

    const connect = async () => {
      try {
        const offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        const res = await fetch(`${API_BASE}/api/webrtc/offer`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ sdp: pc.localDescription?.sdp, type: pc.localDescription?.type }),
        })
        if (cancelled) return
        if (!res.ok) {
          const data = await res.json().catch(() => ({}))
          console.warn('WebRTC offer failed:', res.status, data)
          setVideoMode('mjpeg')
          return
        }
        const answer = await res.json()
        await pc.setRemoteDescription(new RTCSessionDescription(answer))
      } catch (e) {
        if (!cancelled) {
          console.warn('WebRTC error:', e)
          setVideoMode('mjpeg')
        }
      }
    }
    connect()

    return () => {
      cancelled = true
      dataChannelRef.current = null
      pc.close()
      pcRef.current = null
      if (videoRef.current) {
        videoRef.current.srcObject = null
      }
    }
  }, [])

  const SEND_INTERVAL_MS = 20 // 50 Hz for real-time control (robot → ROS2 → webapp and back)
  useEffect(() => {
    const id = setInterval(() => {
      const { linear, angular, lamp } = driveRef.current
      sendControl(linear, angular, lamp)
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
    const onKeyDown = (e: KeyboardEvent) => {
      if (isEditable(document.activeElement)) return
      if (e.repeat) return // ignore key repeat; interval keeps sending
      const key = e.key?.toLowerCase()
      const r = driveRef.current
      let changed = false
      if (key === 'w') {
        r.linear = WASD_LINEAR_FWD
        e.preventDefault()
        changed = true
      } else if (key === 's') {
        r.linear = WASD_LINEAR_BACK
        e.preventDefault()
        changed = true
      } else if (key === 'a') {
        r.angular = WASD_ANGULAR_LEFT
        e.preventDefault()
        changed = true
      } else if (key === 'd') {
        r.angular = WASD_ANGULAR_RIGHT
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
      const key = e.key?.toLowerCase()
      const r = driveRef.current
      let changed = false
      if (key === 'w' || key === 's') {
        r.linear = 0
        changed = true
      } else if (key === 'a' || key === 'd') {
        r.angular = 0
        changed = true
      }
      if (changed) {
        setDriveState({ ...r })
        sendControl(r.linear, r.angular, r.lamp) // stop immediately
      }
    }
    document.addEventListener('keydown', onKeyDown)
    document.addEventListener('keyup', onKeyUp)
    return () => {
      document.removeEventListener('keydown', onKeyDown)
      document.removeEventListener('keyup', onKeyUp)
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
          <Badge color={isConnected ? 'green' : 'red'} variant="filled">
            {isConnected ? 'Connected' : 'Disconnected'}
          </Badge>
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
                />
              ) : (
                <img
                  src={`${API_BASE}/api/video/front/stream`}
                  alt="Front camera"
                  style={{ maxWidth: '100%', maxHeight: 360, objectFit: 'contain' }}
                  onError={() => setVideoError(true)}
                  onLoad={() => setVideoError(false)}
                />
              )}
            </div>
          </Card>

          <Card shadow="sm" padding="lg" radius="md" withBorder>
            <Title order={4} mb="xs">Drive (WASD)</Title>
            <Text size="sm" c="dimmed" mb="sm">
              W forward, S back, A left, D right. Click page to enable.
              {videoMode === 'webrtc' ? ' Control sent via WebRTC data channel.' : ' Control sent via HTTP.'}
            </Text>
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
