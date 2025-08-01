# ESP32-C6 MCP Sensor Hub

This project creates a comprehensive sensor hub using an ESP32-C6 microcontroller with multiple environmental sensors, integrated through the esp_mcp_bridge library. The sensor data is published via MQTT and can be accessed through the chat application, making it a central monitoring station for environmental data.

## Features

- **SCD30 Sensor Integration**: Reads CO2 concentration, temperature, and humidity
- **BMP280/BME280 Sensor Integration**: Reads atmospheric pressure, temperature, and humidity (BME280 only)
- **Multi-Sensor Hub**: Efficiently manages multiple sensors on the same I2C bus
- **MCP Bridge Integration**: Uses the esp_mcp_bridge library for standardized MQTT communication
- **Centralized Monitoring**: Acts as a single point for multiple environmental measurements
- **WiFi Connectivity**: Connects to your WiFi network
- **MQTT Publishing**: Publishes sensor data to configurable MQTT broker
- **Chat App Compatible**: Real-time monitoring through the MCP chat application

## Hardware Setup

### Required Components
- ESP32-C6 development board
- SCD30 CO2/temperature/humidity sensor
- BMP280 or BME280 pressure/temperature(/humidity) sensor
- Jumper wires

### Wiring
Connect both sensors to the ESP32-C6 I2C bus:

```
Sensor   ESP32-C6
------   --------
VCC   -> 3.3V (both sensors)
GND   -> GND (both sensors)
SDA   -> GPIO 6 (both sensors)
SCL   -> GPIO 7 (both sensors)
```

**Note**: Both sensors share the same I2C bus. The SCD30 uses address 0x61 and BMP280/BME280 uses address 0x76 (default), so there are no address conflicts.

## Prerequisites

### ESP-IDF Setup
Ensure you have ESP-IDF v5.0 or later installed:

```bash
# Install ESP-IDF if not already installed
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
source export.sh
```

### Project Structure

This project is **not** part of the `esp_mcp_bridge` repository, but it uses the firmware components from that repository.  
You need to copy the `firmware/components` directory from `esp_mcp_bridge` into this project as shown below:

```
esp32-c6-mcp-sensor-hub/              # This project
└── components/                       # Copy from esp_mcp_bridge/firmware/components
    ├── esp_mcp_bridge/               # MCP Bridge library
    └── (sensor components)           # SCD30 and other sensor drivers
```

### Dependencies Setup

1. **Copy firmware components from esp_mcp_bridge**:

   If you only want the `firmware/components` folder, you can use a sparse checkout:

   ```bash
   git clone --filter=blob:none --sparse https://github.com/your-repo/esp_mcp_bridge.git
   cd esp_mcp_bridge
   git sparse-checkout set firmware/components
   cp -r firmware/components /path/to/your/esp32-c6-mcp-sensor-hub/
   cd ..
   rm -rf esp_mcp_bridge
   ```

   Or, if you already have the full repository:

   ```bash
   cp -r /path/to/esp_mcp_bridge/firmware/components /path/to/esp32-c6-mcp-sensor-hub/
   ```

2. **Initialize ESP-IDF submodules** (if needed):

   ```bash
   git submodule update --init --recursive
   ```

3. **The project automatically includes**:
   - `esp_mcp_bridge` library from `components/esp_mcp_bridge/`
   - SCD30 sensor driver from the components directory
   - BMP280/BME280 sensor driver from the components directory
   - Required ESP-IDF components (WiFi, MQTT, FreeRTOS, etc.)

## Configuration

Before building, configure the project settings:

```bash
idf.py menuconfig
```

Navigate to "ESP32-C6 MCP Sensor Hub Configuration" and set:
- **WiFi SSID**: Your WiFi network name
- **WiFi Password**: Your WiFi password
- **MQTT Broker URI**: MQTT broker address (e.g., `mqtt://192.168.1.100:1883`)
- **Device ID**: Unique identifier for your device
- **Sensor Reading Interval**: How often to read sensors (default: 5000ms)

### MQTT Broker Setup

You'll need an MQTT broker running to receive sensor data. Options include:

1. **Local Mosquitto Broker**:
   ```bash
   # Install mosquitto
   sudo apt install mosquitto mosquitto-clients
   
   # Start broker
   sudo systemctl start mosquitto
   
   # Test with client
   mosquitto_sub -h localhost -t "devices/+/sensors/+/data"
   ```

2. **Docker Mosquitto**:
   ```bash
   docker run -it -p 1883:1883 eclipse-mosquitto
   ```

3. **Cloud MQTT services** (AWS IoT, Google Cloud IoT, etc.)

### MCP Bridge Configuration

The MCP Bridge library is automatically configured through the CMakeLists.txt files:

- **Main CMakeLists.txt**: Includes `../firmware/components` in the component search path
- **Component dependency**: `esp_mcp_bridge` is listed as a required component
- **Auto-discovery**: ESP-IDF automatically finds and builds the library

## Building and Flashing

```bash
# Build the project
idf.py build

# Flash to ESP32-C6
idf.py -p /dev/ttyUSB0 flash monitor
```

## MQTT Topics

The device publishes sensor data to the following topics:

### SCD30 Sensor Topics
- `devices/{device_id}/sensors/co2/data` - CO2 concentration in ppm
- `devices/{device_id}/sensors/temperature/data` - Temperature in °C (SCD30)
- `devices/{device_id}/sensors/humidity/data` - Humidity in %RH (SCD30)

### BMP280/BME280 Sensor Topics
- `devices/{device_id}/sensors/pressure/data` - Atmospheric pressure in Pa
- `devices/{device_id}/sensors/temperature_bmp/data` - Temperature in °C (BMP280/BME280)
- `devices/{device_id}/sensors/humidity_bme/data` - Humidity in %RH (BME280 only)

### Device Status Topics
- `devices/{device_id}/status` - Device status (online/offline)
- `devices/{device_id}/capabilities` - Device capabilities and metadata

## Integration with Chat App

### Setting up the Chat App

1. **Navigate to the chat app directory**:
   ```bash
   cd ../chat_app
   ```

2. **Install dependencies and run** (refer to chat_app README for specific instructions)

3. **Configure MQTT connection** in the chat app to match your broker settings

### Using the Chat App

Once both the ESP32-C6 device and chat app are running, you can:

- **Query current sensor readings**: Ask about CO2, temperature, or humidity
- **Monitor sensor data in real-time**: View live updates as they're published
- **Check device status and capabilities**: See if the device is online and what sensors are available
- **View sensor metadata and specifications**: Get details about sensor ranges and accuracy

### Example Chat Commands

```
> What's the current CO2 level?
> Show me the temperature
> What's the atmospheric pressure?
> Show me humidity readings
> What sensors are available?
> Is the ESP32-C6 device online?
```

### Verification Without Chat App

You can also verify the system works using MQTT clients:

```bash
# Subscribe to all sensor data
mosquitto_sub -h your-broker-ip -t "devices/+/sensors/+/data"

# Subscribe to device status
mosquitto_sub -h your-broker-ip -t "devices/+/status"

# Subscribe to device capabilities
mosquitto_sub -h your-broker-ip -t "devices/+/capabilities"
```

## Troubleshooting

### Common Issues

1. **WiFi Connection Failed**
   - Verify SSID and password in menuconfig
   - Check WiFi signal strength
   - Ensure ESP32-C6 is within range

2. **MQTT Connection Failed**
   - Verify MQTT broker URI and port
   - Check network connectivity
   - Ensure MQTT broker is running and accessible

3. **Sensor Not Responding**
   - Check I2C wiring (SDA/SCL pins)
   - Verify sensor power (3.3V for both sensors)
   - Ensure proper I2C pull-up resistors if needed
   - Check for I2C address conflicts (shouldn't occur with these sensors)

4. **Invalid Sensor Readings**
   - Allow 2-3 minutes for SCD30 to stabilize after power-on
   - BMP280/BME280 should respond immediately after initialization
   - Check for proper ventilation around sensors
   - Verify sensors are not in direct sunlight or heat source

5. **Missing BME280 Humidity Data**
   - Humidity is only available on BME280, not BMP280
   - Check sensor marking - BME280 should be clearly labeled
   - If you have BMP280, humidity sensor won't be registered

### Debug Logs

Monitor the serial output for debugging information:

```bash
idf.py monitor
```

Look for log messages from the `ESP32_C6_SENSOR_HUB` tag for sensor-specific information. You should see messages like:
- "Detected sensor: BME280" or "Detected sensor: BMP280"
- "SCD30 Firmware Version: X.X"
- Sensor measurement updates in debug mode

## License

This project uses components from the ESP-IDF library and esp-idf-lib sensor drivers.
