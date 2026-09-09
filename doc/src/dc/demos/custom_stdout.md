# Custom uptime to Stdout

In this demo, we will go through a new use case. You want to create your own measurement or use an existing measurement provided by `dc_measurements` but not exactly, you want to add a field and also modify the JSON schema. That is what we will do here: create a new plugin, inside dc_demos, another package, create a plugin and load it

We are going to take the uptime measurement, change it slightly and collect the data.

To test it, run:

```bash
ros2 launch dc_demos uptime_custom_stdout.launch.py
```

## JSON schema

Located in `dc_demos/plugins/measurements/json/uptime_custom.json`:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Uptime Custom",
  "description": "Time the system has been up. Intentionally failing to demonstrate customization and callback",
  "properties": {
    "time": {
      "description": "Time the system has been up",
      "type": "integer",
      "maximum": 0
    }
  },
  "type": "object"
}
```

It is almost the same as the standard uptime, but for the sake of the example, we will set the maximum value to 0, which will certainly make the validation fail!

## CPP code

First, we create a hpp file in `dc_demos/include/dc_demos/plugins/measurements`:

```cpp
#ifndef DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_
#define DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_

#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>

#include "dc_measurements/measurement.hpp"
#include "dc_measurements/plugins/measurements/uptime.hpp"

namespace dc_demos
{
using json = nlohmann::json;

class UptimeCustom : public dc_measurements::Uptime
{
protected:
  void onFailedValidation(json data_json) override;
};

}  // namespace dc_demos

#endif  // DC_DEMOS__PLUGINS__MEASUREMENTS__UPTIME_CUSTOM_HPP_
```

We create a new class `UptimeCustom`, which inherits from `dc_measurements::Uptime`.

```admonish info
If we wanted to start a new measurement from scratch, it would inherit from `dc_core::Measurement`.
```

The method `onFailedValidation` is not mandatory but it is here to show it is possible to trigger a custom function when the validation fails.

Then the cpp code, currently located in `dc_demos/plugins/measurements/uptime_custom.cpp`:

```cpp
#include "dc_demos/plugins/measurements/uptime_custom.hpp"

namespace dc_demos
{

void UptimeCustom::onFailedValidation(json data_json)
{
  (void)data_json;
  RCLCPP_INFO(logger_, "Callback! Validation failed for uptime custom");
}

}  // namespace dc_demos

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_demos::UptimeCustom, dc_core::Measurement)
```

We include the uptime_custom file header. Then, define the onFailedValidation function (triggered when validation fails).

The schema is picked up without any code: a Measurement validates against
`plugins/measurements/json/<plugin type>.json` from the package registering it, so `dc_demos/UptimeCustom`
reads `dc_demos/plugins/measurements/json/uptime_custom.json`. Set `json_schema_path` to point elsewhere, or
`enable_validator: false` to turn validation off.

```admonish info
Do not forget to include the pluginlib statements at the end to export the plugin class.
```

## Plugin file
Create the xml file, here will be `measurement_plugin.xml` for us, at the source of the package. It defines the plugins of the package.

```xml
<class_libraries>
    <library path="dc_uptime_custom_measurement">
        <class name="dc_demos/UptimeCustom" type="dc_demos::UptimeCustom" base_class_type="dc_core::Measurement">
            <description>
                dc_measurement_uptime_custom
            </description>
        </class>
    </library>
</class_libraries>
```

## CMakeLists.txt
Now that you have all files set up, you can add the build process to the CMakeLists.txt:

```cmake
# Measurement plugins
add_library(dc_uptime_custom_measurement SHARED
  plugins/measurements/uptime_custom.cpp
)
list(APPEND dc_measurement_plugin_libs dc_uptime_custom_measurement)

foreach(measurement_plugin ${dc_measurement_plugin_libs})
  ament_target_dependencies(${measurement_plugin} ${dependencies})
  target_link_libraries(
    ${measurement_plugin}
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
  )
  target_compile_definitions(${measurement_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

pluginlib_export_plugin_description_file(dc_core measurement_plugin.xml)

install(FILES measurement_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins/measurements/
  DESTINATION share/${PROJECT_NAME}/plugins/measurements/
)
```

It creates the library, installs and exports it.

## Console output

In the measurement server log, the plugin is detected properly

```
[component_container_isolated-1] [INFO] [1788506243.714587305] [measurement_server]: Creating measurement plugin uptime_custom: Type dc_demos/UptimeCustom, Group key: uptime, Polling interval: 5000, Debug: 0, Validator enabled: 1, Schema path: , Tags: [], Init collect: 1, Init Max measurement: 0, Include measurement name: 1, Include measurement plugin name: 0, Remote keys: , Remote prefixes: , Nest: 0, Flatten: 0, Include measurement plugin name: 0, Max measurement on condition: 0, If all condition: , If any condition: , If none condition: , Gate condition: , Buffer duration sec: 0, Post roll duration sec: 0, Cooldown sec: 0, Max flush rate hz: 0, Flush topic: /dc/flush
[component_container_isolated-1] [INFO] [1788506243.724443694] [measurement_server]: Done configuring uptime_custom
[component_container_isolated-1] [INFO] [1788506243.724997657] [measurement_server]: Looking for schema at /root/ws/install/dc_demos/share/dc_demos/plugins/measurements/json/uptime_custom.json
[component_container_isolated-1] [INFO] [1788506243.725045689] [measurement_server]: schema: {"$schema":"http://json-schema.org/draft-07/schema#","description":"Time the system has been up. Intentionally failing to demonstrate customization and callback","properties":{"time":{"description":"Time the system has been up","maximum":0,"type":"integer"}},"title":"Uptime Custom","type":"object"}
```

Then, it fails as expected:

```
[component_container_isolated-1] [ERROR] [1788506243.726002578] [measurement_server]: Validation failed: At /time of 1638628 - instance exceeds maximum of 0
[component_container_isolated-1] data={"time":1638628}
[component_container_isolated-1] [INFO] [1788506243.726050184] [measurement_server]: Callback! Validation failed for uptime custom
[component_container_isolated-1] [ERROR] [1788506248.725121430] [measurement_server]: Validation failed: At /time of 1638633 - instance exceeds maximum of 0
[component_container_isolated-1] data={"time":1638633}
[component_container_isolated-1] [INFO] [1788506248.725329122] [measurement_server]: Callback! Validation failed for uptime custom
```

Even though every Record fails validation, `dc_bridge` still ships the raw data to the
`console` Destination — validation failure only triggers `onFailedValidation`, it doesn't
drop the Record:

```
[dc_bridge-2] {"date":1788506243.7255764,"flattened":false,"host":"127.0.0.1","name":"uptime_custom","nested":false,"source_type":"fluent","tag":"dc.measurement.uptime_custom","time":1638628,"timestamp":"2026-09-04T07:17:23.725576334Z"}
```
