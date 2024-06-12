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

const (
	SYSTEM_PROCESS_NAME_BUFFER_SIZE = 256
)

func DriverVersion() (string, error) {
	version := make([]byte, SYSTEM_DRIVER_VERSION_BUFFER_SIZE)
	ret := brmlSystemGetDriverVersion(&version[0], SYSTEM_DRIVER_VERSION_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(version[:clen(version)]), nil
}

func UmdVersion() (int32, error) {
	var version int32
	ret := brmlSystemGetUmdDriverVersion(&version)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return version, nil
}

func BRMLVersion() (string, error) {
	version := make([]byte, SYSTEM_BRML_VERSION_BUFFER_SIZE)
	ret := brmlSystemGetBRMLVersion(&version[0], SYSTEM_BRML_VERSION_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(version[:clen(version)]), nil
}

func SupaDriverVersion() (int, error) {
	var version int32
	ret := brmlSystemGetSupaDriverVersion(&version)
	if ret != SUCCESS {
		return 0, errors.New(Error2String(ret))
	}
	return int(version), nil
}

func ProcessName(pid int) (string, error) {
	name := make([]byte, SYSTEM_PROCESS_NAME_BUFFER_SIZE)
	ret := brmlSystemGetProcessName(uint32(pid), &name[0], SYSTEM_PROCESS_NAME_BUFFER_SIZE)
	if ret != SUCCESS {
		return "", errors.New(Error2String(ret))
	}
	return string(name[:clen(name)]), nil
}
