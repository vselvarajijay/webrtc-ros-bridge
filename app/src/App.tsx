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
  const driveRef = useRef<DriveState>({ linear: 0, angular: 0, lamp: 0 })
  const wsRef = useRef<WebSocket | null>(null)
  const logsEndRef = useRef<HTMLDivElement>(null)

  const sendControl = (linear: number, angular: number, lamp: number) => {
    fetch(`${API_BASE}/api/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: { linear, angular, lamp } }),
    }).catch(() => {})
  }

  useEffect(() => {
    // Connect to WebSocket
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

    ws.onerror = (error) => {
      console.error('WebSocket error:', error)
      setIsConnected(false)
    }

    ws.onclose = () => {
      setIsConnected(false)
      console.log('WebSocket disconnected')
      // Attempt to reconnect after 3 seconds
      setTimeout(() => {
        if (wsRef.current?.readyState === WebSocket.CLOSED) {
          // Reconnect logic handled by useEffect cleanup and re-run
        }
      }, 3000)
    }

    return () => {
      ws.close()
    }
  }, [])

  useEffect(() => {
    // Auto-scroll to bottom when new logs arrive
    logsEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [logs])

  const SEND_INTERVAL_MS = 25 // 40 Hz for smoother continuous motion
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
    try {
      const res = await fetch(`${API_BASE}/api/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          command: { linear: 1, angular: 1, lamp: 0 },
        }),
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
            <Title order={4} mb="xs">Drive (WASD)</Title>
            <Text size="sm" c="dimmed" mb="sm">
              W forward, S back, A left, D right. Click page to enable.
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
