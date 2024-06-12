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
package main

import (
	"fmt"

	"github.com/BirenTechnology/go-brml/brml"
)

func main() {
	err := brml.Init()
	if err != nil {
		fmt.Println(err.Error())
		return
	}
	num, err := brml.DeviceCount()
	if err != nil {
		fmt.Println(err.Error())
		return
	}

	v, err := brml.DriverVersion()
	fmt.Printf("Driver Version: %s\n", v)

	bv, _ := brml.BRMLVersion()

	fmt.Println("BRML Version: ", bv)

	sv, _ := brml.SupaDriverVersion()
	fmt.Println("SUPA Driver Version: ", sv)

	fmt.Println("GPU Count: ", num)
	fmt.Println("--------------------------------------")
	for i := 0; i < num; i++ {
		device, err := brml.HandleByIndex(i)
		if err != nil {
			fmt.Println(err.Error())
			return
		}

		sviCount, err := brml.GetSviMode(device)
		if err != nil {
			fmt.Println(err.Error())
			return
		}
		fmt.Printf("Device %d virtual instance count: %d \n", i, sviCount)

		if sviCount == 1 {
			err = deviceInfo(device)
			if err != nil {
				fmt.Println(err.Error())
				return
			}
			continue
		}

		for i := 0; i < sviCount; i++ {
			err = deviceInstanceInfo(device, i)
			if err != nil {
				fmt.Println(err)
			}
		}
		fmt.Println("--------------------------------------")
	}

	err = deviceTopo(num)
	if err != nil {
		fmt.Println(err.Error())
		return
	}
}

func deviceInfo(device brml.Device) error {
	index, err := brml.DeviceGetIndex(device)
	if err != nil {
		fmt.Println(err.Error())
		return err
	}
	fmt.Println("device index:", index)

	pciInfo, err := brml.DevicePciInfo(device)
	if err != nil {
		fmt.Println(err.Error())
		return err
	}
	fmt.Printf("Domain: %d Bus: %d Device: %d Function: %d PciDeviceId: %d PciSubSystemId: %d PciSubSystemId: %d BusId: %s\n", pciInfo.Domain, pciInfo.Bus, pciInfo.Device, pciInfo.Function, pciInfo.PciDeviceId, pciInfo.PciSubSystemId, pciInfo.PciSubSystemId, brml.B2S(pciInfo.BusId))

	// _, err := brml.HandleByPciBusID(brml.B2S(pciInfo.BusId))
	// if err != nil {
	// 	fmt.Println(err.Error())
	// 	return err
	// }

	pcieLinkWidth, err := brml.DevicePcieLinkWidth(device)
	if err != nil {
		fmt.Println(err.Error())
		return err
	}
	fmt.Printf("Pcie link width: %d\n", pcieLinkWidth)

	gpuUUID, err := brml.DeviceUUID(device)
	if err != nil {
		return err
	}

	fmt.Println("GPU UUID: ", gpuUUID)

	eccMode, err := brml.EccMode(device)
	if err != nil {
		return err
	}
	fmt.Println("Ecc Mode: ", eccMode)

	mem, err := brml.MemoryInfo(device)
	if err != nil {
		return err
	}
	fmt.Println("-----Memory Info-----")
	fmt.Printf("Used: %d, Free: %d, Total: %d\n", mem.Used, mem.Freed, mem.Total)

	boardPartNumber, err := brml.DeviceBoardPartNumber(device)
	if err != nil {
		return err
	}
	fmt.Println("Device board part number: ", boardPartNumber)

	fv, err := brml.FirmwareVersion(device, 1)
	if err != nil {
		return err
	}
	fmt.Printf("Command Processor firmware version: %s\n", fv)

	cplw, err := brml.CurrentPcieLinkWidth(device)
	if err != nil {
		return err
	}

	fmt.Println("Current PCIE Link Width: ", cplw)

	gpuTem, err := brml.Temperature(device, 0)
	if err != nil {
		return err
	}

	fmt.Printf("GPU Temperature: %d \n", gpuTem)

	utr, err := brml.UtilizationRates(device)
	if err != nil {
		return err
	}
	fmt.Printf("GPU Utilization Rates Unit: %d, Memory: %d\n", utr.Gpu, utr.Memory)

	pu, err := brml.PowerUsage(device)
	if err != nil {
		return err
	}
	fmt.Printf("Power Usage: %d millWatts\n", pu)
	return nil
}

func deviceInstanceInfo(device brml.Device, instance int) error {
	dd, err := brml.GetGPUInstanceByID(device, uint32(instance))
	if err != nil {
		return err
	}

	cardID, err := brml.GetGPUNodeIds(dd)
	if err != nil {
		return err
	}

	fmt.Printf("---- Card ID: card_%d ----\n", cardID)

	gpuUUID, err := brml.DeviceUUID(dd)

	if err != nil {
		return err
	}
	fmt.Printf("GPU UUID: %s\n", gpuUUID)
	mem, err := brml.MemoryInfo(dd)
	if err != nil {
		return err
	}
	fmt.Printf("Memory Info: Used: %d, Free: %d, Total: %d\n", mem.Used, mem.Freed, mem.Total)

	fmt.Println("----")
	return nil
}

func deviceTopo(num int) error {
	for i := 0; i < num; i++ {
		di, err := brml.HandleByIndex(i)
		if err != nil {
			return err
		}
		diNodeId, err := brml.GetGPUNodeIds(di)
		if err != nil {
			return err
		}
		for j := 0; j < i; j++ {
			dj, err := brml.HandleByIndex(j)
			if err != nil {
				return err
			}
			djNodeId, err := brml.GetGPUNodeIds(dj)
			if err != nil {
				return err
			}

			status, err := brml.P2PStatusV2(di, dj)
			if err != nil {
				return err
			}
			fmt.Printf("Device: card_%d and Device: card_%d connect with %s \n", diNodeId, djNodeId, type2String(brml.P2pLinkType(status.Type)))
		}
	}
	return nil
}

func type2String(i brml.P2pLinkType) string {
	switch int(i) {
	case 0:
		return "P2P_NO_LINK"
	case 1:
		return "P2P_INDIRECT_LINK"
	case 2:
		return "P2P_DIRECT_LINK"
	}
	return "Can't find LinkType"
}
