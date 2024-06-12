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
// Code generated by cmd/cgo -godefs; DO NOT EDIT.
// cgo -godefs types.go

package brml

import "unsafe"

const sizeofValue = unsafe.Sizeof([8]byte{})

type Value [sizeofValue]byte

type Device *_Ctype_struct_brmlDevice_st

type PciInfo struct {
	Domain		uint32
	Bus		uint32
	Device		uint32
	Function	uint32
	PciDeviceId	uint32
	PciSubSystemId	uint32
	BusId		[32]int8
}

type PciErrorCounter struct {
	EbufOverflow		[16]uint32
	EbufUnderrun		[16]uint32
	DecodeError		[16]uint32
	RunningDisparity	[16]uint32
	SkpParity		[16]uint32
	SyncHeader		[16]uint32
	RxDeassertion		[16]uint32
	CtlSkpParity		[16]uint32
	FirstRetimer		[16]uint32
	SecondeRetimer		[16]uint32
	MarginCrcParity		[16]uint32
	DetectEiInfer		uint32
	ReceiverError		uint32
	RxRecoveryReq		uint32
	NFtsTimeout		uint32
	FramingError		uint32
	DeskewError		uint32
	BadTlp			uint32
	LcrcError		uint32
	BadDllp			uint32
	ReplayNumRollover	uint32
	ReplayTimeout		uint32
	RxNakDllp		uint32
	TxNakDllp		uint32
	RetryTlp		uint32
	FcTimeout		uint32
	PositionedTlp		uint32
	EcrcError		uint32
	UnsupportReq		uint32
	CompleterAbort		uint32
	CompletionTimeout	uint32
	TotalError		uint32
}

type EccErrorCounts struct {
	DeviceMemory uint64
}

type Utilization struct {
	Gpu	uint32
	Memory	uint32
}

type GpuUtilization struct {
	Spc	uint32
	Cp	uint32
	Sdma	uint32
	Vdcp	uint32
	Vecp	uint32
	Vdec	uint32
	Venc	uint32
}

type Memory struct {
	Total	uint64
	Freed	uint64
	Used	uint64
}

type ProcessInfo struct {
	Pid		uint32
	UsedGpuMemory	uint64
	UsedHostMemory	uint64
}

type GPUInfo struct {
	Name		[96]int8
	Id		[96]int8
	Serial		[96]int8
	Board_id	[96]int8
}

type ProcessUtilizationSample struct {
	Pid		uint32
	TimeStamp	uint64
	MemUtil		uint32
	Pad_cgo_0	[4]byte
}

type EventData struct {
	Device		*_Ctype_struct_brmlDevice_st
	EventType	uint64
	EventData	uint64
}

type PowerSamples struct {
	Min	uint32
	Max	uint32
	Avg	uint32
}

type P2pStatus struct {
	Type	uint32
	Count	uint32
	Port	[8]uint32
}

type P2pThroughput struct {
	Rx_throughput	uint32
	Tx_throughput	uint32
}

type MemoryBandwidth struct {
	Read_bandwidth	uint32
	Write_bandwidth	uint32
}
