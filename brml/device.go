// Copyright 2024 Shanghai Biren Technology Co., Ltd.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
package brml

import (
	"github.com/pkg/errors"
)

func MinorNumber(device Device) (int, error) {
	var mn uint32
	ret := brmlDeviceGetMinorNumber(device, &mn)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(mn), nil
}

func BoardPartNumber(device Device) (string, error) {
	partNumber := make([]byte, DEVICE_PART_NUMBER_BUFFER_SIZE)
	ret := brmlDeviceGetBoardPartNumber(device, &partNumber[0], DEVICE_PART_NUMBER_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(partNumber[:clen(partNumber)]), nil
}

func DeviceUUID(device Device) (string, error) {
	uuid := make([]byte, DEVICE_UUID_BUFFER_SIZE)
	ret := brmlDeviceGetUUID(device, &uuid[0], DEVICE_UUID_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(uuid[:clen(uuid)]), nil
}

func DeviceBoardPartNumber(device Device) (string, error) {
	boardPartNumber := make([]byte, DEFAULT_BUFFER_SIZE)
	ret := brmlDeviceGetBoardPartNumber(device, &boardPartNumber[0], DEFAULT_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(boardPartNumber[:clen(boardPartNumber)]), nil
}

func DeviceVendorName(device Device) (string, error) {
	vendorName := make([]byte, DEFAULT_BUFFER_SIZE)

	ret := brmlDeviceGetVendorName(device, &vendorName[0], DEFAULT_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(vendorName[:clen(vendorName)]), nil
}

func ViolationStatus(device Device) (int, error) {
	var policyType PerfPolicyType
	var value uint32
	ret := brmlDeviceGetViolationStatus(device, policyType, &value)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(value), nil
}

func DeviceCount() (int, error) {
	var dc uint32
	ret := brmlDeviceGetCount_v1(&dc)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(dc), nil
}

func HandleByIndex(index int) (Device, error) {
	var device Device
	ret := brmlDeviceGetHandleByIndex_v1(uint32(index), &device)
	if ret != SUCCESS {
		return device, errors.New(Error2String(ret))
	}
	return device, nil
}

func HandleByNodeID(id int) (Device, error) {
	var device Device
	ret := brmlDeviceGetHandleByNodeId(uint32(id), &device)
	if ret != SUCCESS {
		return device, errors.New(Error2String(ret))
	}
	return device, nil
}

func DeviceGetIndex(device Device) (int, error) {
	var index uint32
	ret := brmlDeviceGetIndex(device, &index)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(index), nil
}

func DeviceResetGPU(device Device) error {
	ret := brmlDeviceResetGpu(device, GPU_RESET)
	if ret != SUCCESS {
		return errors.New(Error2String(ret))
	}
	return nil
}

func DevicePciInfo(device Device) (PciInfo, error) {
	var pci PciInfo
	ret := brmlDeviceGetPciInfo_v1(device, &pci)
	if ret != SUCCESS {
		return PciInfo{}, errors.New(Error2String(ret))
	}
	return pci, nil
}

func DevicePhysicalSlotNumber(device Device) (int, error) {
	var slotNum uint32
	ret := brmlDeviceGetPhysicalSlotNumber(device, &slotNum)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(slotNum), nil
}

func DevicePcieLinkWidth(device Device) (int, error) {
	var lw uint32
	ret := brmlDeviceGetCurrPcieLinkWidth(device, &lw)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(lw), nil
}

func MemoryInfo(device Device) (Memory, error) {
	var m Memory
	ret := brmlDeviceGetMemoryInfo(device, &m)
	if ret != SUCCESS {
		return m, errors.New(Error2String(ret))
	}
	return m, nil
}

func FirmwareVersion(device Device, firmwareType GPUFirmwareType) (string, error) {
	version := make([]byte, DEFAULT_BUFFER_SIZE)

	ret := brmlDeviceGetFirmwareVersion(device, firmwareType, &version[0], DEFAULT_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(version[:clen(version)]), nil
}

func HandleByPciBusID(pciBusID string) (Device, error) {
	var device Device
	ret := brmlDeviceGetHandleByPciBusId_v1(pciBusID, &device)
	if ret != SUCCESS {
		return device, errors.New(Error2String(ret))
	}
	return device, nil
}

func CurrentPcieLinkWidth(device Device) (int, error) {
	var width uint32
	ret := brmlDeviceGetCurrPcieLinkWidth(device, &width)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(width), nil
}

func MaxPcieLinkWidth(device Device) (int, error) {
	var width uint32
	ret := brmlDeviceGetMaxPcieLinkWidth(device, &width)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(width), nil
}

func MaxPcieLinkGeneration(device Device) (int, error) {
	var lg uint32
	ret := brmlDeviceGetMaxPcieLinkGeneration(device, &lg)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(lg), nil
}

func CurrentPcieLinkGenerattion(device Device) (int, error) {
	var lg uint32
	ret := brmlDeviceGetCurrPcieLinkGeneration(device, &lg)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(lg), nil
}

func PcieThroughput(device Device, counter PcieUtilCounter) (int, error) {
	var t uint32
	ret := brmlDeviceGetPcieThroughput(device, counter, &t)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(t), nil
}

func VirtualizationMode(device Device) (int, error) {
	var gpm GpuVirtualizationMode
	ret := brmlDeviceGetVirtualizationMode(device, &gpm)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(gpm), nil
}

func PcieReplayCounter(device Device) (int, error) {
	var counter uint32
	ret := brmlDeviceGetPcieReplayCounter(device, &counter)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(counter), nil
}

func PcieReplayNumberRollovers(device Device) (int, error) {
	var num uint32
	ret := brmlDeviceGetPcieReplayNumberRollovers(device, &num)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(num), nil
}

func GetGPUInfo(device Device) (GPUInfo, error) {
	var info GPUInfo
	ret := brmlDeviceGetGpuInfo(device, &info)
	if ret != SUCCESS {
		return info, errors.New(Error2String(ret))
	}
	return info, nil
}

func ClockInfo(device Device, clockType int) (int, error) {
	var ci uint32
	ret := brmlDeviceGetClockInfo(device, ClockType(clockType), &ci)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(ci), nil
}

func MaxClockInfo(device Device, clockType int) (int, error) {
	var ci uint32
	ret := brmlDeviceGetMaxClockInfo(device, ClockType(clockType), &ci)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(ci), nil
}

func Temperature(device Device, st TemperatureSensors) (int, error) {
	var temp uint32
	ret := brmlDeviceGetTemperature(device, st, &temp)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(temp), nil
}

func TemperatureThreshold(device Device, tt TemperatureThresholds) (int, error) {
	var temp uint32
	ret := brmlDeviceGetTemperatureThreshold(device, tt, &temp)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(temp), nil
}

func PowerManagementLimit(device Device) (int, error) {
	var limit uint32
	ret := brmlDeviceGetPowerManagementLimit(device, &limit)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(limit), nil
}

func PowerManagementLimitConstraints(device Device) (int, int, error) {
	var min, max uint32
	ret := brmlDeviceGetPowerManagementLimitConstraints(device, &min, &max)
	if ret != SUCCESS {
		return 0, 0, errors.New(Error2String(ret))
	}
	return int(min), int(max), nil
}

func PowerUsage(device Device) (int, error) {
	var p uint32
	ret := brmlDeviceGetPowerUsage(device, &p)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(p), nil
}

func GetPowerSamples(device Device) (PowerSamples, error) {
	var ps PowerSamples
	ret := brmlDeviceGetPowerSamples(device, &ps)
	if ret != SUCCESS {
		return PowerSamples{}, errors.New(Error2String(ret))
	}
	return ps, nil
}

func UtilizationRates(device Device) (Utilization, error) {
	var ut Utilization
	ret := brmlDeviceGetUtilizationRates(device, &ut)
	if ret != SUCCESS {
		return ut, errors.New(Error2String(ret))
	}
	return ut, nil
}

func EncoderUtilization(device Device) (uint32, uint32, error) {
	var ut uint32
	var spu uint32
	ret := brmlDeviceGetEncoderUtilization(device, &ut, &spu)
	if ret != SUCCESS {
		return ut, spu, errors.New(Error2String(ret))
	}
	return ut, spu, nil
}

func DecoderUtilization(device Device) (uint32, uint32, error) {
	var ut uint32
	var spu uint32
	ret := brmlDeviceGetDecoderUtilization(device, &ut, &spu)
	if ret != SUCCESS {
		return ut, spu, errors.New(Error2String(ret))
	}
	return ut, spu, nil
}

func EncoderCapacity(device Device, encoderQueryType EncoderType) (uint32, error) {
	var ec uint32
	ret := brmlDeviceGetEncoderCapacity(device, encoderQueryType, &ec)
	if ret != SUCCESS {
		return ec, errors.New(Error2String(ret))
	}
	return ec, nil
}

func EccMode(device Device) (EnableState, error) {
	var state EnableState
	ret := brmlDeviceGetEccMode(device, &state)
	if ret != SUCCESS {
		return state, errors.New(Error2String(ret))
	}
	return state, nil
}

func TotalEccErrors(device Device, errorType MemoryErrorType, counterType EccCounterType) (uint64, error) {
	var eccCounts uint64
	ret := brmlDeviceGetTotalEccErrors(device, errorType, counterType, &eccCounts)
	if ret != SUCCESS {
		return eccCounts, errors.New(Error2String(ret))
	}
	return eccCounts, nil
}

func MemoryErrorCounter(device Device, errorType MemoryErrorType, counterType EccCounterType, locationType MemoryLocation) (uint64, error) {
	var count uint64
	ret := brmlDeviceGetMemoryErrorCounter(device, errorType, counterType, locationType, &count)
	if ret != SUCCESS {
		return count, errors.New(Error2String(ret))
	}
	return count, nil
}

func ClearEccErrorCounts(device Device, counterType EccCounterType) error {
	ret := brmlDeviceClearEccErrorCounts(device, counterType)
	if ret != SUCCESS {
		return errors.New(Error2String(ret))
	}
	return nil
}

func TotalAerErrors(device Device, errorType AerErrorType) (uint64, error) {
	var aerCounts uint64
	ret := brmlDeviceGetTotalAerErrors(device, errorType, &aerCounts)
	if ret != SUCCESS {
		return aerCounts, errors.New(Error2String(ret))
	}
	return aerCounts, nil
}

func HealthStatus(device Device) (GpuHealthStatus, error) {
	var hs GpuHealthStatus
	ret := brmlDeviceGetHealthStatus(device, &hs)
	if ret != SUCCESS {
		return hs, errors.New(Error2String(ret))
	}
	return hs, nil
}

func ComputeRunningProcess(device Device) (uint32, ProcessInfo, error) {
	var infoCounts uint32
	var infos ProcessInfo
	ret := brmlDeviceGetComputeRunningProcesses_v1(device, &infoCounts, &infos)
	if ret != SUCCESS {
		return infoCounts, infos, errors.New(Error2String(ret))
	}
	return infoCounts, infos, nil
}

func ProcessUtilization(device Device, lastSeenTimestamp uint64) (ProcessUtilizationSample, uint32, error) {
	var pus ProcessUtilizationSample
	var psc uint32
	ret := brmlDeviceGetProcessUtilization(device, &pus, &psc, lastSeenTimestamp)
	if ret != SUCCESS {
		return pus, psc, errors.New(Error2String(ret))
	}
	return pus, psc, nil
}

func CurrentClocksThrottleReasons(device Device) (uint64, error) {
	var reason uint64
	ret := brmlDeviceGetCurrentClocksThrottleReasons(device, &reason)
	if ret != SUCCESS {
		return reason, errors.New(Error2String(ret))
	}
	return reason, nil
}

func TopologyCommonAncestor(device Device, device2 Device) (GpuTopologyLevel, error) {
	var pathInfo GpuTopologyLevel
	ret := brmlDeviceGetTopologyCommonAncestor(device, device2, &pathInfo)
	if ret != SUCCESS {
		return pathInfo, errors.New(Error2String(ret))
	}
	return pathInfo, nil
}

func NumaNode(device Device) (int, error) {
	var n uint32
	ret := brmlDeviceGetNumaNode(device, &n)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(n), nil

}

func P2PStatus(device Device, device2 Device) (int, error) {
	var portNum uint32
	ret := brmlDeviceGetP2PStatus(device, device2, &portNum)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(portNum), nil
}

func P2PStatusV2(device Device, device2 Device) (P2pStatus, error) {
	var p2pStatus P2pStatus
	ret := brmlDeviceGetP2PStatus_v2(device, device2, &p2pStatus)
	if ret != SUCCESS {
		return p2pStatus, errors.New(Error2String(ret))
	}
	return p2pStatus, nil
}

func GetSviMode(device Device) (int, error) {
	var currentMode uint32
	var pendingMode uint32
	ret := brmlDeviceGetSviMode(device, &currentMode, &pendingMode)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(currentMode), nil
}

func SetClockFreqs(device Device, clockType ClockType, clockFreq uint32) error {
	ret := brmlDeviceSetClockFreqs(device, clockType, uint32Pointers(clockFreq))
	if ret != SUCCESS {
		return errors.New(Error2String(ret))
	}
	return nil
}

func GetGPUInstanceByID(device Device, id uint32) (Device, error) {
	var gpuInstance Device
	ret := brmlDeviceGetGpuInstanceById(device, id, &gpuInstance)
	if ret != SUCCESS {
		return gpuInstance, errors.New(Error2String(ret))
	}
	return gpuInstance, nil
}

func GetGPUNodeIds(device Device) (int, error) {
	var ids uint32
	ret := brmlDeviceGetNodeId(device, &ids)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(ids), nil
}
