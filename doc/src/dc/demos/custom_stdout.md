# Custom uptime to Stdout

In this demo, we will go through a new use case. You want to create your own measurement or use an existing measurement provided by `dc_measurements` but not exactly, you want to add a field and also modify the JSON schema. That is what we will do here: create a new plugin, inside dc_demos, another package, create a plugin and load it

We are going to take the uptime measurement, change it slightly and collect the data.

To test it, run:

```bash
$ ros2 launch dc_demos uptime_custom_stdout.launch.py
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
  void setValidationSchema() override;
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

void UptimeCustom::setValidationSchema()
{
  if (enable_validator_)
  {
    validateSchema("dc_demos", "uptime_custom.json");
  }
}

}  // namespace dc_demos

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_demos::UptimeCustom, dc_core::Measurement)
```

We include the uptime_custom file header. Then, define the onFailedValidation function (triggered when validation fails) and the setValidationSchema function which sets the json with the new json schema

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
[component_container_isolated-1] [INFO] [1677715629.547448938] [measurement_server]: Creating measurement plugin uptime_custom: Type dc_demos/UptimeCustom, Group key: uptime, Polling interval: 5000, Debug: 0, Validator enabled: 1, Schema path: , Tags: [flb_stdout], Init collect: 1, Init Max measurement: 0, Include measurement name: 0, Include measurement plugin name: 0, Remote keys: , Remote prefixes: , Include measurement plugin name: 0, Max measurement on condition: 0, If all condition: , If any condition: , If none condition:
...
[component_container_isolated-1] [INFO] [1677715629.550523128] [measurement_server]: schema: {"$schema":"http://json-schema.org/draft-07/schema#","description":"Time the system has been up. Intentionally failing to demonstrate customization and callback","properties":{"time":{"description":"Time the system has been up","maximum":0,"type":"integer"}},"title":"Uptime Custom","type":"object"}
```

Then, it fails as expected:

```
[component_container_isolated-1] [ERROR] [1677715634.550778659] [measurement_server]: Validation failed: At /time of 139123 - instance exceeds maximum of 0
[component_container_isolated-1] data={"tags":["flb_stdout"],"time":139123}
[component_container_isolated-1] [INFO] [1677715634.550911115] [measurement_server]: Callback! Validation failed for uptime custom
[component_container_isolated-1] [ERROR] [1677715639.550754853] [measurement_server]: Validation failed: At /time of 139128 - instance exceeds maximum of 0
[component_container_isolated-1] data={"tags":["flb_stdout"],"time":139128}
[component_container_isolated-1] [INFO] [1677715639.550867662] [measurement_server]: Callback! Validation failed for uptime custom
```
