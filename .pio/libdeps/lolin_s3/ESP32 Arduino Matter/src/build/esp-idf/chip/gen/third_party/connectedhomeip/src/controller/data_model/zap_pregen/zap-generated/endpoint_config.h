/*
 *
 *    Copyright (c) 2022 Project CHIP Authors
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

// THIS FILE IS GENERATED BY ZAP

// Prevent multiple inclusion
#pragma once

#include <lib/core/CHIPConfig.h>
#include <app/util/endpoint-config-defines.h>


// Default values for the attributes longer than a pointer,
// in a form of a binary blob
// Separate block is generated for big-endian and little-endian cases.
#if BIGENDIAN_CPU
#define GENERATED_DEFAULTS { \
}


#else // !BIGENDIAN_CPU
#define GENERATED_DEFAULTS { \
}

#endif // BIGENDIAN_CPU

#define GENERATED_DEFAULTS_COUNT (0)

// This is an array of EmberAfAttributeMinMaxValue structures.
#define GENERATED_MIN_MAX_DEFAULT_COUNT 0
#define GENERATED_MIN_MAX_DEFAULTS { \
}


// This is an array of EmberAfAttributeMetadata structures.
#define GENERATED_ATTRIBUTE_COUNT 0
#define GENERATED_ATTRIBUTES { \
}


// clang-format off
#define GENERATED_EVENT_COUNT 0
#define GENERATED_EVENTS { \
}

// clang-format on

// Cluster function static arrays
#define GENERATED_FUNCTION_ARRAYS   \




// This is an array of EmberAfCluster structures.
#define GENERATED_CLUSTER_COUNT 66
// clang-format off
#define GENERATED_CLUSTERS { \
  { \
      /* Endpoint: 1, Cluster: Identify (client) */ \
      .clusterId = 0x00000003, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Groups (client) */ \
      .clusterId = 0x00000004, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Scenes (client) */ \
      .clusterId = 0x00000005, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: On/Off (client) */ \
      .clusterId = 0x00000006, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: On/off Switch Configuration (client) */ \
      .clusterId = 0x00000007, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Level Control (client) */ \
      .clusterId = 0x00000008, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Binary Input (Basic) (client) */ \
      .clusterId = 0x0000000F, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Descriptor (client) */ \
      .clusterId = 0x0000001D, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Binding (client) */ \
      .clusterId = 0x0000001E, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Access Control (client) */ \
      .clusterId = 0x0000001F, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Actions (client) */ \
      .clusterId = 0x00000025, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Basic Information (client) */ \
      .clusterId = 0x00000028, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: OTA Software Update Provider (client) */ \
      .clusterId = 0x00000029, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: OTA Software Update Requestor (client) */ \
      .clusterId = 0x0000002A, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Localization Configuration (client) */ \
      .clusterId = 0x0000002B, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Time Format Localization (client) */ \
      .clusterId = 0x0000002C, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Unit Localization (client) */ \
      .clusterId = 0x0000002D, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Power Source Configuration (client) */ \
      .clusterId = 0x0000002E, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Power Source (client) */ \
      .clusterId = 0x0000002F, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: General Commissioning (client) */ \
      .clusterId = 0x00000030, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Network Commissioning (client) */ \
      .clusterId = 0x00000031, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Diagnostic Logs (client) */ \
      .clusterId = 0x00000032, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: General Diagnostics (client) */ \
      .clusterId = 0x00000033, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Software Diagnostics (client) */ \
      .clusterId = 0x00000034, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Thread Network Diagnostics (client) */ \
      .clusterId = 0x00000035, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: WiFi Network Diagnostics (client) */ \
      .clusterId = 0x00000036, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Ethernet Network Diagnostics (client) */ \
      .clusterId = 0x00000037, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Bridged Device Basic Information (client) */ \
      .clusterId = 0x00000039, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Switch (client) */ \
      .clusterId = 0x0000003B, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Administrator Commissioning (client) */ \
      .clusterId = 0x0000003C, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Operational Credentials (client) */ \
      .clusterId = 0x0000003E, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Group Key Management (client) */ \
      .clusterId = 0x0000003F, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Fixed Label (client) */ \
      .clusterId = 0x00000040, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: User Label (client) */ \
      .clusterId = 0x00000041, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Boolean State (client) */ \
      .clusterId = 0x00000045, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Mode Select (client) */ \
      .clusterId = 0x00000050, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Door Lock (client) */ \
      .clusterId = 0x00000101, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Window Covering (client) */ \
      .clusterId = 0x00000102, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Barrier Control (client) */ \
      .clusterId = 0x00000103, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Pump Configuration and Control (client) */ \
      .clusterId = 0x00000200, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Thermostat (client) */ \
      .clusterId = 0x00000201, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Fan Control (client) */ \
      .clusterId = 0x00000202, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Thermostat User Interface Configuration (client) */ \
      .clusterId = 0x00000204, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Color Control (client) */ \
      .clusterId = 0x00000300, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Ballast Configuration (client) */ \
      .clusterId = 0x00000301, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Illuminance Measurement (client) */ \
      .clusterId = 0x00000400, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Temperature Measurement (client) */ \
      .clusterId = 0x00000402, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Pressure Measurement (client) */ \
      .clusterId = 0x00000403, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Flow Measurement (client) */ \
      .clusterId = 0x00000404, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Relative Humidity Measurement (client) */ \
      .clusterId = 0x00000405, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Occupancy Sensing (client) */ \
      .clusterId = 0x00000406, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Wake on LAN (client) */ \
      .clusterId = 0x00000503, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Channel (client) */ \
      .clusterId = 0x00000504, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Target Navigator (client) */ \
      .clusterId = 0x00000505, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Media Playback (client) */ \
      .clusterId = 0x00000506, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Media Input (client) */ \
      .clusterId = 0x00000507, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Low Power (client) */ \
      .clusterId = 0x00000508, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Keypad Input (client) */ \
      .clusterId = 0x00000509, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Content Launcher (client) */ \
      .clusterId = 0x0000050A, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Audio Output (client) */ \
      .clusterId = 0x0000050B, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Application Launcher (client) */ \
      .clusterId = 0x0000050C, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Application Basic (client) */ \
      .clusterId = 0x0000050D, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Account Login (client) */ \
      .clusterId = 0x0000050E, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Electrical Measurement (client) */ \
      .clusterId = 0x00000B04, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Client Monitoring (client) */ \
      .clusterId = 0x00001046, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
  { \
      /* Endpoint: 1, Cluster: Unit Testing (client) */ \
      .clusterId = 0xFFF1FC05, \
      .attributes = ZAP_ATTRIBUTE_INDEX(0), \
      .attributeCount = 0, \
      .clusterSize = 0, \
      .mask = ZAP_CLUSTER_MASK(CLIENT), \
      .functions = NULL, \
      .acceptedCommandList = nullptr, \
      .generatedCommandList = nullptr, \
      .eventList = nullptr, \
      .eventCount = 0, \
    },\
}

// clang-format on

#define ZAP_FIXED_ENDPOINT_DATA_VERSION_COUNT 0

// This is an array of EmberAfEndpointType structures.
#define GENERATED_ENDPOINT_TYPES { \
  { ZAP_CLUSTER_INDEX(0), 66, 0 }, \
}



// Largest attribute size is needed for various buffers
#define ATTRIBUTE_LARGEST (1)

static_assert(ATTRIBUTE_LARGEST <= CHIP_CONFIG_MAX_ATTRIBUTE_STORE_ELEMENT_SIZE,
              "ATTRIBUTE_LARGEST larger than expected");

// Total size of singleton attributes
#define ATTRIBUTE_SINGLETONS_SIZE (0)

// Total size of attribute storage
#define ATTRIBUTE_MAX_SIZE (0)

// Number of fixed endpoints
#define FIXED_ENDPOINT_COUNT (1)

// Array of endpoints that are supported, the data inside
// the array is the endpoint number.
#define FIXED_ENDPOINT_ARRAY { 0x0001 }

// Array of profile ids
#define FIXED_PROFILE_IDS { 0x0103 }

// Array of device types
#define FIXED_DEVICE_TYPES {{0x0016,1}}

// Array of device type offsets
#define FIXED_DEVICE_TYPE_OFFSETS { 0}

// Array of device type lengths
#define FIXED_DEVICE_TYPE_LENGTHS { 1}

// Array of endpoint types supported on each endpoint
#define FIXED_ENDPOINT_TYPES { 0 }

// Array of networks supported on each endpoint
#define FIXED_NETWORKS { 0 }

