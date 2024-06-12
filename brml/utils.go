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
	"fmt"
	"unsafe"
)

// #cgo LDFLAGS: -ldl
// #include <dlfcn.h>
import "C"

const (
	RTLD_LAZY           = C.RTLD_LAZY
	RTLD_NOW            = C.RTLD_NOW
	RTLD_GLOBAL         = C.RTLD_GLOBAL
	RTLD_LOCAL          = C.RTLD_LOCAL
	RTLD_NODELETE       = C.RTLD_NODELETE
	RTLD_NOLOAD         = C.RTLD_NOLOAD
	RTLD_DEEPBIND       = C.RTLD_DEEPBIND
	DEFAULT_BUFFER_SIZE = 64
)

type DynamicLibrary struct {
	Name   string
	Flags  int
	handle unsafe.Pointer
}

func New(name string, flags int) *DynamicLibrary {
	return &DynamicLibrary{
		Name:   name,
		Flags:  flags,
		handle: nil,
	}
}

func (dl *DynamicLibrary) Open() error {
	handle := C.dlopen(C.CString(dl.Name), C.int(dl.Flags))
	if handle == C.NULL {
		return fmt.Errorf("%s", C.GoString(C.dlerror()))
	}
	dl.handle = handle
	return nil
}

func (dl *DynamicLibrary) Close() error {
	err := C.dlclose(dl.handle)
	if err != 0 {
		return fmt.Errorf("%s", C.GoString(C.dlerror()))
	}
	return nil
}

func (dl *DynamicLibrary) Lookup(symbol string) error {
	C.dlerror() // Clear out any previous errors
	C.dlsym(dl.handle, C.CString(symbol))
	err := C.dlerror()
	if unsafe.Pointer(err) == C.NULL {
		return nil
	}
	return fmt.Errorf("%s", C.GoString(err))
}

func clen(n []byte) int {
	for i := 0; i < len(n); i++ {
		if n[i] == 0 {
			return i
		}
	}
	return len(n)
}

func Error2String(result Return) string {
	return brmlErrorString(result)
}

func B2S(bs [32]int8) string {
	b := make([]byte, len(bs))
	for i, v := range bs {
		b[i] = byte(v)
	}
	return string(b)
}

func int32Pointers(i int32) *int32 {
	return &i
}

func uint32Pointers(i uint32) *uint32 {
	return &i
}
