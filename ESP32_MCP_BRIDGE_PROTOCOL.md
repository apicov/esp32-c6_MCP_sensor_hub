# ESP32 MCP Bridge Protocol Documentation

## Overview

The ESP32 MCP Bridge enables IoT devices to communicate with MCP servers via MQTT. Sensors publish readings as JSON messages, and the server can discover device capabilities automatically.

## MQTT Topics

### Device publishes to:

- `devices/{device_id}/capabilities` - Device capabilities (retained)
- `devices/{device_id}/sensors/{sensor_id}/data` - Sensor readings
- `devices/{device_id}/status` - Device status (online/offline)
- `devices/{device_id}/error` - Error messages

### Server subscribes to:

- `devices/+/capabilities` - Discover all devices
- `devices/+/sensors/+/data` - All sensor data
- `devices/+/status` - Device status updates
- `devices/+/error` - Error notifications

## Message Formats

### 1. Capabilities Message

Published once on connection with `retain=true`.

```json
{
  "device_id": "esp32_c6_sensor_hub",
  "firmware_version": "1.0.0",
  "device_location": "home",
  "sensors": ["scd30", "bmp280"],
  "actuators": [],
  "metadata": {
    "scd30": {
      "type": "multi_value",
      "description": "SCD30 environmental sensor (CO2, temperature, humidity)",
      "location": "home",
      "fields": {
        "co2": {
          "unit": "ppm",
          "metric_type": "co2",
          "min_range": 400,
          "max_range": 10000,
          "accuracy": 30
        },
        "temperature": {
          "unit": "°C",
          "metric_type": "temperature",
          "min_range": -40,
          "max_range": 70,
          "accuracy": 0.4
        },
        "humidity": {
          "unit": "%RH",
          "metric_type": "humidity",
          "min_range": 0,
          "max_range": 100,
          "accuracy": 3
        }
      }
    },
    "bmp280": {
      "type": "multi_value",
      "description": "BME280 environmental sensor (pressure, temperature, humidity)",
      "location": "home",
      "fields": {
        "pressure": {
          "unit": "Pa",
          "metric_type": "pressure",
          "min_range": 30000,
          "max_range": 110000,
          "accuracy": 12
        },
        "temperature": {
          "unit": "°C",
          "metric_type": "temperature",
          "min_range": -40,
          "max_range": 85,
          "accuracy": 1
        },
        "humidity": {
          "unit": "%RH",
          "metric_type": "humidity",
          "min_range": 0,
          "max_range": 100,
          "accuracy": 3
        }
      }
    }
  }
}
```

### 2. Sensor Data Message (Multi-Value)

Published at configured intervals (default 60 seconds for MQTT publish, sensors measure at their own intervals).

```json
{
  "device_id": "esp32_c6_sensor_hub",
  "sensor_id": "scd30",
  "timestamp": 123456,
  "type": "sensor",
  "action": "read",
  "value": {
    "co2": {
      "reading": 450.0,
      "unit": "ppm",
      "quality": 100
    },
    "temperature": {
      "reading": 23.5,
      "unit": "°C",
      "quality": 100
    },
    "humidity": {
      "reading": 45.2,
      "unit": "%RH",
      "quality": 100
    }
  },
  "metrics": {
    "free_heap": 50000,
    "uptime": 123456
  }
}
```

### 3. Status Message

```json
{
  "value": "online",
  "timestamp": 123456
}
```

**Possible values:** `online`, `offline`

### 4. Error Message

```json
{
  "device_id": "esp32_c6_sensor_hub",
  "timestamp": 123456,
  "value": {
    "error_type": "sensor_error",
    "message": "Failed to read sensor",
    "severity": 2
  }
}
```

**Severity levels:** 0=info, 1=warning, 2=error, 3=critical

## Python MCP Server Implementation Guide

### Required Libraries

```python
import paho.mqtt.client as mqtt
import json
```

### Basic Structure

```python
class ESP32MCPBridge:
    def __init__(self, broker_host, broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.devices = {}  # Store device capabilities
        self.client = mqtt.Client()

        # Set up callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        """Subscribe to all device topics on connection"""
        print(f"Connected with result code {rc}")
        client.subscribe("devices/+/capabilities")
        client.subscribe("devices/+/sensors/+/data")
        client.subscribe("devices/+/status")
        client.subscribe("devices/+/error")

    def on_message(self, client, userdata, msg):
        """Route messages based on topic"""
        topic_parts = msg.topic.split('/')

        if len(topic_parts) < 3:
            return

        device_id = topic_parts[1]
        message_type = topic_parts[2]

        try:
            payload = json.loads(msg.payload.decode())

            if message_type == "capabilities":
                self.handle_capabilities(device_id, payload)
            elif message_type == "sensors":
                # topic: devices/{device_id}/sensors/{sensor_id}/data
                sensor_id = topic_parts[3]
                self.handle_sensor_data(device_id, sensor_id, payload)
            elif message_type == "status":
                self.handle_status(device_id, payload)
            elif message_type == "error":
                self.handle_error(device_id, payload)

        except json.JSONDecodeError:
            print(f"Failed to decode JSON from {msg.topic}")

    def handle_capabilities(self, device_id, payload):
        """Store device capabilities"""
        self.devices[device_id] = payload
        print(f"Device {device_id} registered with {len(payload['sensors'])} sensors")

    def handle_sensor_data(self, device_id, sensor_id, payload):
        """Process sensor readings"""
        # payload contains: device_id, sensor_id, timestamp, value, metrics
        # value is a dict of field_name -> {reading, unit, quality}
        for field_name, field_data in payload['value'].items():
            reading = field_data['reading']
            unit = field_data['unit']
            # Store or process the reading
            print(f"{device_id}/{sensor_id}/{field_name}: {reading} {unit}")

    def handle_status(self, device_id, payload):
        """Handle device status changes"""
        status = payload['value']
        print(f"Device {device_id} is {status}")

    def handle_error(self, device_id, payload):
        """Handle device errors"""
        error = payload['value']
        print(f"Error from {device_id}: {error['message']} (severity: {error['severity']})")

    def start(self):
        """Connect to broker and start loop"""
        self.client.connect(self.broker_host, self.broker_port, 60)
        self.client.loop_start()
```

### Usage Example

```python
# Create bridge
bridge = ESP32MCPBridge("localhost", 1883)
bridge.start()

# Access device capabilities
device_id = "esp32_c6_sensor_hub"
if device_id in bridge.devices:
    capabilities = bridge.devices[device_id]
    for sensor_id in capabilities['sensors']:
        sensor_meta = capabilities['metadata'][sensor_id]
        print(f"Sensor: {sensor_id}")
        print(f"  Description: {sensor_meta['description']}")
        if 'fields' in sensor_meta:
            for field_name, field_meta in sensor_meta['fields'].items():
                print(f"  - {field_name}: {field_meta['min_range']}-{field_meta['max_range']} {field_meta['unit']}")
```

## Key Implementation Notes

1. **Timestamp Format**: ESP32 uses Unix timestamp (seconds since Jan 1, 1970) from DS3231 RTC. This provides accurate absolute time synchronized with the real-time clock.

2. **RTC Synchronization**: The ESP32 system time is set from the DS3231 RTC at boot. Ensure the RTC is set to correct UTC time for accurate timestamps.

3. **Retained Messages**: Capabilities are retained, so new servers get device info immediately upon subscription.

4. **Quality Field**: Always 100 by default. Can be used for sensor health/reliability indicators.

5. **Multi-Value Sensors**: A single sensor (like "scd30") publishes multiple fields in one message. Parse `value` dict to get all fields.

6. **Device Discovery**: Subscribe to `devices/+/capabilities` with QoS 1 to discover all online devices.

7. **Connection Resilience**: ESP32 auto-reconnects to WiFi/MQTT with exponential backoff. Handle temporary disconnections gracefully.

## Testing

Use mosquitto_sub to monitor all topics:

```bash
mosquitto_sub -h localhost -t 'devices/#' -v
```

## Configuration

Default ESP32 settings:
- MQTT publish interval: 60 seconds (configurable via `CONFIG_ESP32_C6_SENSOR_INTERVAL`)
- SCD30 measurement interval: 10 seconds (internal sensor reading rate)
- BMP280/BME280 measurement interval: On-demand (reads cached within 1 second)
- MQTT QoS: 0 for sensor data, 1 for status/errors
- Reconnect timeout: 10 seconds
- WiFi retry: Exponential backoff (max 60s)
- Device location: "home" (configurable in code)
- Sensor locations: "home" (configurable per sensor)
