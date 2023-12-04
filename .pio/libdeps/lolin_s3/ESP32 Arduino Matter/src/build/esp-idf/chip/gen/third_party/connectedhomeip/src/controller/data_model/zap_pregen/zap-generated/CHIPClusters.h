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

#include <app-common/zap-generated/ids/Clusters.h>
#include <app-common/zap-generated/ids/Commands.h>

#include <controller/CHIPCluster.h>
#include <lib/core/CHIPCallback.h>
#include <lib/support/Span.h>

namespace chip {
namespace Controller {

class DLL_EXPORT IdentifyCluster : public ClusterBase
{
public:
    IdentifyCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~IdentifyCluster() {}
};

class DLL_EXPORT GroupsCluster : public ClusterBase
{
public:
    GroupsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~GroupsCluster() {}
};

class DLL_EXPORT ScenesCluster : public ClusterBase
{
public:
    ScenesCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ScenesCluster() {}
};

class DLL_EXPORT OnOffCluster : public ClusterBase
{
public:
    OnOffCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~OnOffCluster() {}
};

class DLL_EXPORT OnOffSwitchConfigurationCluster : public ClusterBase
{
public:
    OnOffSwitchConfigurationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~OnOffSwitchConfigurationCluster() {}
};

class DLL_EXPORT LevelControlCluster : public ClusterBase
{
public:
    LevelControlCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~LevelControlCluster() {}
};

class DLL_EXPORT BinaryInputBasicCluster : public ClusterBase
{
public:
    BinaryInputBasicCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~BinaryInputBasicCluster() {}
};

class DLL_EXPORT DescriptorCluster : public ClusterBase
{
public:
    DescriptorCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~DescriptorCluster() {}
};

class DLL_EXPORT BindingCluster : public ClusterBase
{
public:
    BindingCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~BindingCluster() {}
};

class DLL_EXPORT AccessControlCluster : public ClusterBase
{
public:
    AccessControlCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~AccessControlCluster() {}
};

class DLL_EXPORT ActionsCluster : public ClusterBase
{
public:
    ActionsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ActionsCluster() {}
};

class DLL_EXPORT BasicInformationCluster : public ClusterBase
{
public:
    BasicInformationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~BasicInformationCluster() {}
};

class DLL_EXPORT OtaSoftwareUpdateProviderCluster : public ClusterBase
{
public:
    OtaSoftwareUpdateProviderCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~OtaSoftwareUpdateProviderCluster() {}
};

class DLL_EXPORT OtaSoftwareUpdateRequestorCluster : public ClusterBase
{
public:
    OtaSoftwareUpdateRequestorCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~OtaSoftwareUpdateRequestorCluster() {}
};

class DLL_EXPORT LocalizationConfigurationCluster : public ClusterBase
{
public:
    LocalizationConfigurationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~LocalizationConfigurationCluster() {}
};

class DLL_EXPORT TimeFormatLocalizationCluster : public ClusterBase
{
public:
    TimeFormatLocalizationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~TimeFormatLocalizationCluster() {}
};

class DLL_EXPORT UnitLocalizationCluster : public ClusterBase
{
public:
    UnitLocalizationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~UnitLocalizationCluster() {}
};

class DLL_EXPORT PowerSourceConfigurationCluster : public ClusterBase
{
public:
    PowerSourceConfigurationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~PowerSourceConfigurationCluster() {}
};

class DLL_EXPORT PowerSourceCluster : public ClusterBase
{
public:
    PowerSourceCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~PowerSourceCluster() {}
};

class DLL_EXPORT GeneralCommissioningCluster : public ClusterBase
{
public:
    GeneralCommissioningCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~GeneralCommissioningCluster() {}
};

class DLL_EXPORT NetworkCommissioningCluster : public ClusterBase
{
public:
    NetworkCommissioningCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~NetworkCommissioningCluster() {}
};

class DLL_EXPORT DiagnosticLogsCluster : public ClusterBase
{
public:
    DiagnosticLogsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~DiagnosticLogsCluster() {}
};

class DLL_EXPORT GeneralDiagnosticsCluster : public ClusterBase
{
public:
    GeneralDiagnosticsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~GeneralDiagnosticsCluster() {}
};

class DLL_EXPORT SoftwareDiagnosticsCluster : public ClusterBase
{
public:
    SoftwareDiagnosticsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~SoftwareDiagnosticsCluster() {}
};

class DLL_EXPORT ThreadNetworkDiagnosticsCluster : public ClusterBase
{
public:
    ThreadNetworkDiagnosticsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ThreadNetworkDiagnosticsCluster() {}
};

class DLL_EXPORT WiFiNetworkDiagnosticsCluster : public ClusterBase
{
public:
    WiFiNetworkDiagnosticsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~WiFiNetworkDiagnosticsCluster() {}
};

class DLL_EXPORT EthernetNetworkDiagnosticsCluster : public ClusterBase
{
public:
    EthernetNetworkDiagnosticsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~EthernetNetworkDiagnosticsCluster() {}
};

class DLL_EXPORT BridgedDeviceBasicInformationCluster : public ClusterBase
{
public:
    BridgedDeviceBasicInformationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~BridgedDeviceBasicInformationCluster() {}
};

class DLL_EXPORT SwitchCluster : public ClusterBase
{
public:
    SwitchCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~SwitchCluster() {}
};

class DLL_EXPORT AdministratorCommissioningCluster : public ClusterBase
{
public:
    AdministratorCommissioningCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~AdministratorCommissioningCluster() {}
};

class DLL_EXPORT OperationalCredentialsCluster : public ClusterBase
{
public:
    OperationalCredentialsCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~OperationalCredentialsCluster() {}
};

class DLL_EXPORT GroupKeyManagementCluster : public ClusterBase
{
public:
    GroupKeyManagementCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~GroupKeyManagementCluster() {}
};

class DLL_EXPORT FixedLabelCluster : public ClusterBase
{
public:
    FixedLabelCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~FixedLabelCluster() {}
};

class DLL_EXPORT UserLabelCluster : public ClusterBase
{
public:
    UserLabelCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~UserLabelCluster() {}
};

class DLL_EXPORT BooleanStateCluster : public ClusterBase
{
public:
    BooleanStateCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~BooleanStateCluster() {}
};

class DLL_EXPORT ModeSelectCluster : public ClusterBase
{
public:
    ModeSelectCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ModeSelectCluster() {}
};

class DLL_EXPORT DoorLockCluster : public ClusterBase
{
public:
    DoorLockCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~DoorLockCluster() {}
};

class DLL_EXPORT WindowCoveringCluster : public ClusterBase
{
public:
    WindowCoveringCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~WindowCoveringCluster() {}
};

class DLL_EXPORT BarrierControlCluster : public ClusterBase
{
public:
    BarrierControlCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~BarrierControlCluster() {}
};

class DLL_EXPORT PumpConfigurationAndControlCluster : public ClusterBase
{
public:
    PumpConfigurationAndControlCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~PumpConfigurationAndControlCluster() {}
};

class DLL_EXPORT ThermostatCluster : public ClusterBase
{
public:
    ThermostatCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ThermostatCluster() {}
};

class DLL_EXPORT FanControlCluster : public ClusterBase
{
public:
    FanControlCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~FanControlCluster() {}
};

class DLL_EXPORT ThermostatUserInterfaceConfigurationCluster : public ClusterBase
{
public:
    ThermostatUserInterfaceConfigurationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ThermostatUserInterfaceConfigurationCluster() {}
};

class DLL_EXPORT ColorControlCluster : public ClusterBase
{
public:
    ColorControlCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ColorControlCluster() {}
};

class DLL_EXPORT BallastConfigurationCluster : public ClusterBase
{
public:
    BallastConfigurationCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~BallastConfigurationCluster() {}
};

class DLL_EXPORT IlluminanceMeasurementCluster : public ClusterBase
{
public:
    IlluminanceMeasurementCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~IlluminanceMeasurementCluster() {}
};

class DLL_EXPORT TemperatureMeasurementCluster : public ClusterBase
{
public:
    TemperatureMeasurementCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~TemperatureMeasurementCluster() {}
};

class DLL_EXPORT PressureMeasurementCluster : public ClusterBase
{
public:
    PressureMeasurementCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~PressureMeasurementCluster() {}
};

class DLL_EXPORT FlowMeasurementCluster : public ClusterBase
{
public:
    FlowMeasurementCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~FlowMeasurementCluster() {}
};

class DLL_EXPORT RelativeHumidityMeasurementCluster : public ClusterBase
{
public:
    RelativeHumidityMeasurementCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~RelativeHumidityMeasurementCluster() {}
};

class DLL_EXPORT OccupancySensingCluster : public ClusterBase
{
public:
    OccupancySensingCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~OccupancySensingCluster() {}
};

class DLL_EXPORT WakeOnLanCluster : public ClusterBase
{
public:
    WakeOnLanCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~WakeOnLanCluster() {}
};

class DLL_EXPORT ChannelCluster : public ClusterBase
{
public:
    ChannelCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ChannelCluster() {}
};

class DLL_EXPORT TargetNavigatorCluster : public ClusterBase
{
public:
    TargetNavigatorCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~TargetNavigatorCluster() {}
};

class DLL_EXPORT MediaPlaybackCluster : public ClusterBase
{
public:
    MediaPlaybackCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~MediaPlaybackCluster() {}
};

class DLL_EXPORT MediaInputCluster : public ClusterBase
{
public:
    MediaInputCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~MediaInputCluster() {}
};

class DLL_EXPORT LowPowerCluster : public ClusterBase
{
public:
    LowPowerCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~LowPowerCluster() {}
};

class DLL_EXPORT KeypadInputCluster : public ClusterBase
{
public:
    KeypadInputCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~KeypadInputCluster() {}
};

class DLL_EXPORT ContentLauncherCluster : public ClusterBase
{
public:
    ContentLauncherCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ContentLauncherCluster() {}
};

class DLL_EXPORT AudioOutputCluster : public ClusterBase
{
public:
    AudioOutputCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~AudioOutputCluster() {}
};

class DLL_EXPORT ApplicationLauncherCluster : public ClusterBase
{
public:
    ApplicationLauncherCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ApplicationLauncherCluster() {}
};

class DLL_EXPORT ApplicationBasicCluster : public ClusterBase
{
public:
    ApplicationBasicCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ApplicationBasicCluster() {}
};

class DLL_EXPORT AccountLoginCluster : public ClusterBase
{
public:
    AccountLoginCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~AccountLoginCluster() {}
};

class DLL_EXPORT ElectricalMeasurementCluster : public ClusterBase
{
public:
    ElectricalMeasurementCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ElectricalMeasurementCluster() {}
};

class DLL_EXPORT ClientMonitoringCluster : public ClusterBase
{
public:
    ClientMonitoringCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~ClientMonitoringCluster() {}
};

class DLL_EXPORT UnitTestingCluster : public ClusterBase
{
public:
    UnitTestingCluster(Messaging::ExchangeManager & exchangeManager, const SessionHandle & session, EndpointId endpoint) : ClusterBase(exchangeManager, session, endpoint) {}
    ~UnitTestingCluster() {}
};

} // namespace Controller
} // namespace chip
