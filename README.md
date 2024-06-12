# go-brml
GO bindings for brml

``` 
go get github.com/BirenTechnology/go-brml
```

## Requirements
- libbiren-ml.so.0 
- libbebr.so
- libbeci.so
- libbeki.so
- libbesu.so
- libsuoa.so


```
md5sum /var/lib/biren/*

2e7f041379db09587e540ea4f483f3e9  libbebr.so
b5340c12c041170086c21e0e216778ef  libbeci.so
e35f2593f33a888ac1ee00aa51e79c56  libbeki.so
5a6d17dd19e45ee4e2eb05f3d6eeb4d1  libbesu.so
392a1fe92f14ff7c7322f76d7c3bb470  libbiren-ml.so
392a1fe92f14ff7c7322f76d7c3bb470  libbiren-ml.so.0
fce442d27ed1c5ae1382a0e532a371b0  libsupa.so
```

```
md5sum biren.ko

1bc9c6ab793d5932554e3c54b6198166  biren.ko
```

## Example
```
go run example/example.go
```

## Generate bindings
```
make gen
```


## Functions

|Name|location|binding function|description|version|
|--|--|--|--|--|
|brmlInit|init.go|Init|Initialize BRML|v0.1.0|
|brmlInitWithFlags|init.go|InitWithFlags|Initialize BRML with flags |v0.1.0
|brmlShutdown|init.go|Shutdown| Shut down BRML by releasing all GPU resources|v0.1.0
|brmlSystemGetDriverVersion|system.go|DriverVersion|Retrieves the version of the system's graphics driver|v0.1.0
|brmlSystemGetBRMLVersion|system.go|BRMLVersion|Retrieves the version of the BRML library|v0.1.0
|brmlDeviceGetMemoryInfo|device.go|MemoryInfo|Retrieves the amount of used, free and total memory available on the device|v0.2.0
|brmlDeviceGetBAR1MemoryInfo|device.go|BAR1MemoryInfo|Gets Total, Available and Used size of BAR1 memory|v0.2.0
|brmlSystemGetSupaDriverVersion|system.go|SupaDriverVersion|Retrieves the version of the CUDA driver|v0.1.0
|brmlSystemGetProcessName|system.go|ProcessName|Gets name of the process with provided process id|v0.1.0
|brmlDeviceGetFirmwareVersion|device.go|FirmwareVersion|Retrieves the version of specific firmware|v0.2.0
|brmlDeviceGetCount|device.go|DeviceCount|Retrieves the number of compute devices in the system|v0.1.0
|brmlDeviceGetHandleByIndex|device.go|HandleByIndex|Acquire the handle for a particular device, based on its index|v0.1.0
|brmlDeviceGetIndex|device.go|DeviceGetIndex|Retrieves the BRML index of this device|v0.1.0
|brmlDeviceDiscoverGpus|device.go|DiscoverGpus| Request the OS and the BIREN kernel driver to rediscover a portion of the PCI subsystem looking for GPUs that were previously removed|v0.2.0
|brmlDeviceGetHandleByPciBusId|device.go|DeviceHandleByPciBusId|Acquire the handle for a particular device, based on its PCI bus id|v0.2.0
|brmlDeviceGetPciInfo|device.go|DevicePciInfo|Retrieves the PCI attributes of this device.|v0.1.0
|brmlDeviceGetMaxPcieLinkWidth|device.go|DevicePcieLinkWidth| Retrieves the maximum PCIe link width possible with this device and system|v0.1.0
|brmlDeviceGetCurrPcieLinkWidth|device.go|DeviceCurrPcieLinkWidth|Retrieves the current PCIe link width|v0.2.0
|brmlDeviceGetMaxPcieLinkGeneration|device.go|DeviceMaxPcieLinkGeneration|Retrieves the maximum PCIe link generation possible with this device and system|v0.2.0
|brmlDeviceGetCurrPcieLinkGeneration|device.go|DeviceCurrPcieLinkGeneration|Retrieves the current PCIe link generation|v0.2.0
|brmlDeviceGetPcieThroughput|device.go|DevicePcieThroughput|Retrieve PCIe utilization information.|v0.2.0
|brmlDeviceGetVirtualizationMode|device.go|DeviceVirtualizationMode|This method is used to get the virtualization mode corresponding to the GPU.|v0.2.0
|brmlDeviceGetPcieReplayCounter|device.go|PcieReplayCounter|Retrieve the PCIe replay counter.|v0.2.0
|brmlDeviceGetGpuInfo|device.go|GpuInfo|Retrive GPU basic information e.g. product name, id, and serial etc.|v0.2.0
|brmlDeviceGetClockInfo|device.go|ClockInfo|Retrieves the current clock speeds for the device.|v0.2.0
|brmlDeviceGetMaxClockInfo|device.go|MaxClockInfo| Retrieves the maximum clock speeds for the device.|v0.2.0
|brmlDeviceGetTemperature|device.go|Temperature|Retrieves the current temperature readings for the device, in degrees C.|v0.2.0
|brmlDeviceGetTemperatureThreshold|device.go|TemperatureThreshold|Retrieves the temperature threshold for the GPU with the specified threshold type in degrees C|v0.2.0
|brmlDeviceGetPowerUsage|device.go|PowerUsage|Retrieves power usage for this GPU in milliwatts and its associated circuitry (e.g. memory)|v0.2.0
|brmlDeviceGetVoltage|device.go|Voltage|Retrieves the current voltage readings for the device, in voltage milliVolt. |v0.2.0
|brmlEventSetCreate|event.go|EventSetCreate|Create an empty set of events.|v0.2.0
|brmlDeviceRegisterEvents|event.go|RegisterEvents| Starts recording of events on a specified devices and add the events to specified \ref brmlEventSet_t|v0.2.0
|brmlDeviceGetSupportedEventTypes|device.go|SupportedEventTypes|Returns information about events supported on device|v0.2.0
|brmlEventSetWait|event.go|EventSetWait|Waits on events and delivers events|v0.2.0
|brmlEventSetFree|event.go|EventSetFree|Releases events in the set|v0.2.0
|brmlDeviceGetUtilizationRates|device.go|UtilizationRates|Retrieves the current utilization rates for the device's major subsystems.|v0.2.0
|brmlDeviceGetEncoderUtilization|device.go|EncoderUtilization|Retrieves the current utilization and sampling size in microseconds for the Encoder|v0.2.0
|brmlDeviceGetDecoderUtilization|device.go|DecoderUtilization|Retrieves the current utilization and sampling size in microseconds for the Decoder|v0.2.0
|brmlDeviceGetEncoderCapacity|device.go|DecoderCapacity|Retrieves the current capacity of the device's encoder, as a percentage of maximum encoder capacity with valid values in the range 0-100|v0.2.0
|brmlDeviceGetEncoderStats|device.go|EncoderStats|Retrieves the current encoder statistics for a given device|v0.2.0
|brmlDeviceGetEncoderSessions|device.go|EncoderSessions|Retrieves information about active encoder sessions on a target device|v0.2.0
|brmlDeviceGetEccMode|device.go|EccMode|Retrieves the current and pending ECC modes for the device|v0.2.0
|brmlDeviceGetTotalEccErrors|device.go|TotalEccErrors|Retrieves the total ECC error counts for the device|v0.2.0
|brmlDeviceGetDetailedEccErrors|device.go|DetailedEccErrors|Retrieves the detailed ECC error counts for the device|v0.2.0
|brmlDeviceGetMemoryErrorCounter|device.go|MemoryErrorCounter|Retrieves the requested memory error counter for the device|v0.2.0
|brmlDeviceClearEccErrorCounts|device.go|ClearEccErrorCounts|Clear the ECC error and other memory error counts for the device|v0.2.0
|brmlDeviceGetTotalAerErrors|device.go|TotalAerErrors|Retrieves the total AER(Advanced Error Reporting) error counts for the device|v0.2.0
|brmlDeviceGetComputeRunningProcesses|device.go|ComputeRunningProcess|Get information about processes with a compute context on a device|v0.2.0
|brmlDeviceGetProcessUtilization|device.go|ProcessUtilization|Retrieves the current utilization and process ID|v0.2.0
