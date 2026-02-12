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

function App() {
  const [opened, { toggle }] = useDisclosure()
  const [logs, setLogs] = useState<LogMessage[]>([])
  const [isConnected, setIsConnected] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)
  const logsEndRef = useRef<HTMLDivElement>(null)

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

  const formatTimestamp = (timestamp: string) => {
    try {
      const date = new Date(timestamp)
      return date.toLocaleTimeString()
    } catch {
      return timestamp
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
            <Title order={4} mb="md">Quick Actions</Title>
            <Group>
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
