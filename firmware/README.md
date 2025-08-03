# ESP32 Firmware

ESP-IDF components and examples for the MCP-MQTT bridge firmware.

## 📁 **Structure**

```
firmware/
├── components/
│   └── esp_mcp_bridge/         # Main bridge library
│       ├── include/            # Public headers
│       ├── src/                # Implementation
│       ├── Kconfig             # Configuration options
│       └── CMakeLists.txt      # Build configuration
├── examples/
│   ├── basic_example/          # Simple usage example
│   ├── advanced_example/       # Full-featured example
│   └── production_example/     # Production-ready template
├── tests/
│   ├── unit_tests/             # Component unit tests
│   └── integration_tests/      # Hardware integration tests
└── tools/
    ├── flash_tool.py           # Custom flashing utility
    └── monitor_tool.py         # Enhanced monitoring
```

## 🛠️ **Development Setup**

### **Prerequisites**
- ESP-IDF v5.1 or later
- Python 3.8+
- Git

### **Setup**
```bash
# Install ESP-IDF
cd ~/
git clone -b v5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
source export.sh

# Clone project
git clone <your-repo-url> esp32-mcp-bridge
cd esp32-mcp-bridge/firmware
```

## 🚀 **Quick Start**

### **Basic Example**
```bash
cd examples/basic_example
idf.py menuconfig    # Configure WiFi/MQTT settings
idf.py build
idf.py flash monitor
```

### **Configuration Options**
Navigate to: `Component config → MCP Bridge Configuration`

- WiFi SSID/Password
- MQTT Broker URL
- Device ID Prefix
- Sensor Publish Interval
- Security Settings
- Debug Options

## 📋 **API Overview**

### **Basic Usage**
```c
#include "esp_mcp_bridge.h"

// Initialize with default configuration
ESP_ERROR_CHECK(mcp_bridge_init_default());

// Register a temperature sensor
ESP_ERROR_CHECK(mcp_bridge_register_sensor(
    "temperature", "temperature", "°C", 
    &metadata, temp_read_cb, NULL
));

// Register an LED actuator
ESP_ERROR_CHECK(mcp_bridge_register_actuator(
    "led", "led", &metadata, led_control_cb, NULL
));

// Start the bridge
ESP_ERROR_CHECK(mcp_bridge_start());
```

### **Advanced Configuration**
```c
mcp_bridge_config_t config = {
    .wifi_ssid = "YourWiFiSSID",
    .wifi_password = "YourWiFiPassword", 
    .mqtt_broker_uri = "mqtts://broker.com:8883",
    .mqtt_username = "device_user",
    .mqtt_password = "device_pass",
    .enable_device_auth = true,
    .tls_config = {
        .enable_tls = true,
        .ca_cert_pem = ca_cert,
        .client_cert_pem = client_cert,
        .client_key_pem = client_key
    }
};

ESP_ERROR_CHECK(mcp_bridge_init(&config));
```

## 🧪 **Testing**

### **Unit Tests**
```bash
cd tests/unit_tests
idf.py build
idf.py flash monitor
```

### **Integration Tests**
```bash
cd tests/integration_tests
# Connect test hardware
idf.py build flash monitor
```

## 🔧 **Build Options**

### **Release Build**
```bash
idf.py set-target esp32
idf.py build
```

### **Debug Build**
```bash
idf.py menuconfig  # Enable debug options
idf.py build
```

### **Custom Partitions**
```bash
# Edit partitions.csv for custom partition layout
idf.py partition-table
```

## 📊 **Performance Tuning**

### **Memory Optimization**
- Enable `CONFIG_MCP_BRIDGE_OPTIMIZE_MEMORY`
- Adjust task stack sizes in menuconfig
- Monitor heap usage with `esp_get_free_heap_size()`

### **Network Optimization**
- Configure appropriate QoS levels
- Adjust MQTT keep-alive intervals
- Enable connection pooling

## 🐛 **Debugging**

### **Enable Debug Logging**
```c
esp_log_level_set("MCP_BRIDGE", ESP_LOG_VERBOSE);
```

### **Monitor Memory**
```c
ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
```

### **Core Dump Analysis**
```bash
idf.py coredump-debug
```

## 📝 **Configuration Reference**

### **Kconfig Options**
- `CONFIG_MCP_BRIDGE_WIFI_SSID`: Default WiFi SSID
- `CONFIG_MCP_BRIDGE_MQTT_BROKER_URL`: Default MQTT broker
- `CONFIG_MCP_BRIDGE_SENSOR_PUBLISH_INTERVAL`: Publishing interval
- `CONFIG_MCP_BRIDGE_ENABLE_WATCHDOG`: Enable watchdog timer
- `CONFIG_MCP_BRIDGE_MAX_SENSORS`: Maximum number of sensors
- `CONFIG_MCP_BRIDGE_MAX_ACTUATORS`: Maximum number of actuators

### **Runtime Configuration**
All Kconfig options can be overridden at runtime via the configuration structure.

## 🔗 **Related Documentation**

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [MQTT Component Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mqtt.html)
- [WiFi Component Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html) 