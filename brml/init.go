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

import "fmt"

import "C"

const (
	brmlLibraryName      = "libbiren-ml.so.1"
	brmlLibraryLoadFlags = RTLD_LAZY | RTLD_GLOBAL
)

var brmlib *DynamicLibrary

func Init() error {
	lib := New(brmlLibraryName, brmlLibraryLoadFlags)
	if lib == nil {
		return fmt.Errorf("error instantiating DynamicLibrary for %s", brmlLibraryName)
	}

	err := lib.Open()
	if err != nil {
		return fmt.Errorf("error opening %s: %v", brmlLibraryName, err)
	}

	brmlib = lib
	ret := brmlInit()
	if ret != SUCCESS {
		return fmt.Errorf("error %d", ret)
	}
	return nil
}

func Shutdown() error {
	ret := brmlShutdown()
	if ret != SUCCESS {
		return fmt.Errorf("error shutdown")
	}

	err := brmlib.Close()
	if err != nil {
		return err
	}
	return nil
}
