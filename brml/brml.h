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

/*
BRML API Reference

The BIREN GPU Management Library (BRML) is a C-based programmatic interface for monitoring and
managing various states within Biren GPUs. It is intended to be a platform for building
3rd party applications, and is also the underlying library for the brsmi tool provided by Biren.
BRML is thread-safe so it is safe to make simultaneous BRML calls from multiple threads.

API Documentation

Supported platforms:
- Linux:       64-bit

The BRML library will be found on the standard library path.
*/

#ifndef __brml_brml_h__
#define __brml_brml_h__

#ifdef __cplusplus
extern "C" {
#endif

#define DECLDIR __attribute__((visibility("default")))


/**
 * BRML API versioning support
 */
#define BRML_API_VERSION            1
#define BRML_API_VERSION_STR        "1"

/***************************************************************************************************/
/** @defgroup brmlConstants Constants
 *  @{
 */
/***************************************************************************************************/

/**
 * Special constant that some fields take when they are not available.
 * Used when only part of the struct is not available.
 *
 * Each structure explicitly states when to check for this value.
 */
#define BRML_VALUE_NOT_AVAILABLE (-1)

/**
 * PCI format string for brmlPciInfo_st::busId
 * Format followed by brmlPciInfo_st::busId (string)
 */
#define BRML_DEVICE_PCI_BUS_ID_FMT                  "%08X:%02X:%02X.%01X"

/**
 * Maximum Buffer Size for PCI BUS ID
 */
#define BRML_DEVICE_PCI_BUS_ID_BUFFER_SIZE            32

/**
 * Maximum Buffer Size for UUID
 */
#define BRML_DEVICE_UUID_BUFFER_SIZE                  80

/**
 * Maximum Buffer Size for Serial Number
 */
#define BRML_DEVICE_SERIAL_BUFFER_SIZE                32

/**
 * Maximum Buffer Size for System Driver Version \ref brmlSystemGetDriverVersion
 */
#define BRML_SYSTEM_DRIVER_VERSION_BUFFER_SIZE        80

/**
 * Maximum Buffer Size for System Firmware Version \ref brmlDeviceGetFirmwareVersion
 */
#define BRML_SYSTEM_FIRMWARE_VERSION_BUFFER_SIZE        80

/**
 * Maximum Buffer Size for System BRML Version \ref brmlSystemGetBRMLVersion
 */
#define BRML_SYSTEM_BRML_VERSION_BUFFER_SIZE          80

/**
 * Maximum Buffer Size for Device Board Part Number \ref brmlDeviceGetBoardPartNumber
 */
#define BRML_DEVICE_PART_NUMBER_BUFFER_SIZE           80

/**
 * Maximum Buffer Size for Device Name 
 */
#define BRML_DEVICE_NAME_BUFFER_SIZE                  64

/**
 * Convert from SUPA driver version number to Major version numbers
 */
#define BRML_SUPA_DRIVER_VERSION_MAJOR(v) ((v) / 1000)

/**
 * Convert from SUPA driver version number to Minor version numbers
 */
#define BRML_SUPA_DRIVER_VERSION_MINOR(v) (((v) % 1000) / 10)

/**
 * SVI mode in Disabled status
 */
#define BRML_DEVICE_SVI_DISABLE 0x0

/**
 * SVI mode in Enabled status
 */
#define BRML_DEVICE_SVI_ENABLE  0x1

/**
 * SVI mode 1, device is not splited
 */
#define BRML_DEVICE_SVI_MODE0   0x1

/**
 * SVI mode 2, device is splited into 2 instances
 */
#define BRML_DEVICE_SVI_MODE1   0x2

/**
 * SVI mode 4, device is splited into 4 instances
 */
#define BRML_DEVICE_SVI_MODE2   0x4

/** @} */

/***************************************************************************************************/
/** @defgroup brmlClocksThrottleReasons Clocks Throttle Reasons
 *  @{
 */
/***************************************************************************************************/

/** HW Slowdown
 *
 * This may be caused by:
 *   - temperature being too high
 *   - External Power Braker (e.g. the system power supply) 
 *   - Power is too high and thus Fast Trigger protection is working 
 *
 * @see brmlDeviceGetTemperature
 * @see brmlDeviceGetTemperatureThreshold
 * @see brmlDeviceGetPowerUsage
 */
#define brmlClocksThrottleReasonHwSlowdown                0x0000000000000001LL

/** HW Thermal Slowdown
 *
 * This may be caused by:
 *   - temperature being too high
 *
 * @see brmlDeviceGetTemperature
 * @see brmlDeviceGetTemperatureThreshold
 * @see brmlDeviceGetPowerUsage
 */
#define brmlClocksThrottleReasonHwThermalSlowdown         0x0000000000000002LL

/** HW Power Brake Slowdown
 *                                                                                    
 * This may be caused by:
 *   - External Power Braker (e.g. the system power supply)   
 *
 * @see brmlDeviceGetTemperature
 * @see brmlDeviceGetTemperatureThreshold
 * @see brmlDeviceGetPowerUsage
 */
#define brmlClocksThrottleReasonHwPowerBrakeSlowdown      0x0000000000000004LL

/** All Clocks Throttle Reasons supported
 * New reasons might be added to this list in the future
 */
#define brmlClocksThrottleReasonAll (brmlClocksThrottleReasonHwSlowdown \
    | brmlClocksThrottleReasonHwThermalSlowdown \
    | brmlClocksThrottleReasonHwPowerBrakeSlowdown \
    )

/** @} */

/** @defgroup brmlEventType Event Types
 * @{
 * Supported Event Types.
 */

//! Topology Event
#define brmlEventTypeTopology              0x0000000000000002LL

//! Thermal Event
#define brmlEventTypeThermal               0x0000000000000004LL

//! No Event
#define brmlEventTypeNone                  0x0000000000000000LL

//! All Events
#define brmlEventTypeAll (brmlEventTypeNone     \
    | brmlEventTypeTopology                \
    | brmlEventTypeThermal                  \
    )

/** @} */

/***************************************************************************************************/
/** @defgroup brmlDeviceEnums Device Enums
 *  @{
 */
/***************************************************************************************************/

/**
 * Enable status.
 */
typedef enum brmlEnableState_enum {
    BRML_FEATURE_DISABLED    = 0,     //!< Feature disabled
    BRML_FEATURE_ENABLED     = 1      //!< Feature enabled
} brmlEnableState_t;

/**
 * Firmware types.
 */
typedef enum brmlGPUFirmwareType {
    BRML_GPU_FIRMWARE_TYPE_CP = 1,      //!< Command Processor firmware
    BRML_GPU_FIRMWARE_TYPE_SDMA = 2,    //!< System DMA firmware
    BRML_GPU_FIRMWARE_TYPE_ENCODE = 3,  //!< encoder firmware
    BRML_GPU_FIRMWARE_TYPE_DECODE = 4,  //!< decoder firmware
    BRML_GPU_FIRMWARE_TYPE_HBM = 5,     //!< hbm firmware
    BRML_GPU_FIRMWARE_TYPE_FLASH = 6,   //!< flash firmware
    BRML_GPU_FIRMWARE_TYPE_MPU = 7,     //!< mpu firmware
    BRML_GPU_FIRMWARE_TYPE_PMU = 8,     //!< pmu firmware

    // Keep this last
    BRML_GPU_FIRMWARE_TYPE_COUNT
} brmlGPUFirmwareType_t;

/**
 * Temperature thresholds.
 */
typedef enum brmlTemperatureThresholds_enum {
    BRML_TEMPERATURE_THRESHOLD_SHUTDOWN = 0,    //!< Temperature of the GPU shutting down for HW protection
    BRML_TEMPERATURE_THRESHOLD_SLOWDOWN = 1,    //!< Temperature of the GPU beginning HW slowdown
    // Keep this last
    BRML_TEMPERATURE_THRESHOLD_COUNT
} brmlTemperatureThresholds_t;

/**
 * Temperature sensors.
 */
typedef enum brmlTemperatureSensors_enum {
    BRML_TEMPERATURE_GPU            = 0,    //!< Temperature sensor for the GPU die
    BRML_TEMPERATURE_MEMORY         = 1,    //!< Temperature sensor for the memory
    BRML_TEMPERATURE_SPC            = 2,    //!< Temperature sensor for the spc
    BRML_TEMPERATURE_BOARD_SENSOR0  = 3,    //!< Temperature sensor for the Board Sensor 0
    BRML_TEMPERATURE_BOARD_SENSOR1  = 4,    //!< Temperature sensor for the Board Sensor 1
    BRML_TEMPERATURE_BOARD_SENSOR2  = 5,    //!< Temperature sensor for the Board Sensor 2 (br104 not supported)
    BRML_TEMPERATURE_BOARD_SENSOR3  = 6,    //!< Temperature sensor for the Board Sensor 3 (br104 not supported)
    BRML_TEMPERATURE_DIODE_SENSOR0  = 7,    //!< Temperature sensor for the Diode Sensor 0
    BRML_TEMPERATURE_DIODE_SENSOR1  = 8,    //!< Temperature sensor for the Diode Sensor 1
    BRML_TEMPERATURE_DIODE_SENSOR2  = 9,    //!< Temperature sensor for the Diode Sensor 2 (br104 not supported)
    BRML_TEMPERATURE_DIODE_SENSOR3  = 10,   //!< Temperature sensor for the Diode Sensor 3 (br104 not supported)

    // Keep this last
    BRML_TEMPERATURE_COUNT
} brmlTemperatureSensors_t;

/**
 * Memory error types
 */
typedef enum brmlMemoryErrorType_enum {
    /**
     * Correctable Memory errors
     *
     * These are single bit errors for ECC errors
     */
    BRML_MEMORY_ERROR_TYPE_CORRECTED = 0,
    /**
     * Uncorrectable Memory errors
     *
     * These are double bit errors for ECC errors
     */
    BRML_MEMORY_ERROR_TYPE_UNCORRECTED = 1,


    // Keep this last
    BRML_MEMORY_ERROR_TYPE_COUNT

} brmlMemoryErrorType_t;

/**
 * AER error types
 */
typedef enum brmlAerErrorType_enum {
    BRML_AER_ERROR_TYPE_CORRECTED = 0,   //!< correctable errors
    BRML_AER_ERROR_TYPE_FATAL     = 1,   //!< fatal uncorrectable errors
    BRML_AER_ERROR_TYPE_NONFATAL  = 2,   //!< nonfatal uncorrectable errors

    // Keep this last
    BRML_AER_ERROR_TYPE_COUNT
} brmlAerErrorType_t;

/**
 * ECC counter types.
 *
 * Note: Volatile counts are reset each time the driver loads. 
 */
typedef enum brmlEccCounterType_enum {
    BRML_VOLATILE_ECC      = 0,      //!< Volatile counts are reset each time the driver loads.

    // Keep this last
    BRML_ECC_COUNTER_TYPE_COUNT
} brmlEccCounterType_t;

/**
 * Clock types.
 *
 * All speeds are in MHz.
 */
typedef enum brmlClockType_enum {
    BRML_CLOCK_SOC       = 0,        //!< SoC clock domain
    BRML_CLOCK_CORE      = 1,        //!< SPC clock domain
    BRML_CLOCK_MEM       = 2,        //!< Memory clock domain
    BRML_CLOCK_ENCODE    = 3,        //!< encoder clock domain
    BRML_CLOCK_DECODE    = 4,        //!< decoder clock domain

    // Keep this last
    BRML_CLOCK_COUNT
} brmlClockType_t;

/**
 * GPU Reset type.
 */
typedef enum brmlGpuResetType_enum {
    BRML_GPU_RESET       = 0,        //!< HOT Reset type

    // Keep this last
    BRML_RESET_COUNT
} brmlGpuResetType_t;

/**
 * BRML API return values.
 */
typedef enum brmlReturn_enum {
    BRML_SUCCESS = 0,                        //!< The operation was successful
    BRML_ERROR_UNINITIALIZED = 1,            //!< BRML was not initialized with brmlInit()
    BRML_ERROR_INVALID_ARGUMENT = 2,         //!< Argument is invalid
    BRML_ERROR_NOT_SUPPORTED = 3,            //!< Operation is not supported on target device
    BRML_ERROR_NO_PERMISSION = 4,            //!< The current user does not have permission for operation
    BRML_ERROR_NOT_FOUND = 5,                //!< The query object is not found
    BRML_ERROR_INSUFFICIENT_SIZE = 6,        //!< Size is not large enough
    BRML_ERROR_DRIVER_NOT_LOADED = 7,        //!< Driver is not loaded
    BRML_ERROR_TIMEOUT = 8,                  //!< User provided timeout passed
    BRML_ERROR_UNEXPECTED_DATA = 9,          //!< Data is unexpected
    BRML_ERROR_UNEXPECTED_SIZE = 10,         //!< Size is unexpected
    BRML_ERROR_NO_DATA = 11,                 //!< No data
    BRML_ERROR_INSUFFICIENT_RESOURCES = 12,  //!< Ran out of critical resources, other than memory
    BRML_ERROR_FILE_ERROR = 13,              //!< File operation errors
    BRML_ERROR_INTERRUPT = 14,               //!< The operation is interrupted
    BRML_ERROR_BUSY = 15,                    //!< The requested resource is in busy
    BRML_ERROR_DRIVER_VERSION_MISMATCH = 16, //!< Driver version mismatch
    BRML_ERROR_IN_USE = 17,                  //!< The GPU is currently in use
    BRML_ERROR_UNKNOWN = 0xFF                //!< Unknown error
} brmlReturn_t;

/**
 * See \ref brmlDeviceGetMemoryErrorCounter
 */
typedef enum brmlMemoryLocation_enum {
    BRML_MEMORY_LOCATION_L1_CACHE        = 0,    //!< GPU L1 Cache
    BRML_MEMORY_LOCATION_L2_CACHE        = 1,    //!< GPU L2 Cache
    BRML_MEMORY_LOCATION_DEVICE_MEMORY   = 2,    //!< GPU Device Memory
    // Keep this last
    BRML_MEMORY_LOCATION_COUNT
} brmlMemoryLocation_t;

/**
 * Level relationships within a system between two GPUs
 */
typedef enum brmlGpuLevel_enum {
    BRML_TOPOLOGY_INTERNAL           = 0,  //!< all devices are multi gpu board
    BRML_TOPOLOGY_SINGLE             = 10, //!< all devices only traverse a single PCIe switch
    BRML_TOPOLOGY_MULTIPLE           = 20, //!< all devices needn't traverse a host bridge
    BRML_TOPOLOGY_HOSTBRIDGE         = 30, //!< all devices are connected to the same host bridge
    BRML_TOPOLOGY_NODE               = 40, //!< all devices are connected to the same NUMA node but possibly multiple host bridges
    BRML_TOPOLOGY_SYSTEM             = 50, //!< all devices in the system

    // there is purposefully no COUNT here due to the need for spacing above
} brmlGpuTopologyLevel_t;

/**
 * GPU Health Status
 */
typedef enum brmlGpuHealthStatus_enum {
    BRML_HEALTH_STATUS_OK = 0,
    BRML_HEALTH_STATUS_WARNING,
    BRML_HEALTH_STATUS_CRITICAL_WARNING,
    BRML_HEALTH_STATUS_ERROR
} brmlGpuHealthStatus_t;

/**
 * Perf Policy types
 */
typedef enum brmlPerfPolicyType_enum {
    BRML_PERF_POLICY_POWER = 0,      //!< Power violations
    BRML_PERF_POLICY_THERMAL = 1,    //!< Thermal violations

    BRML_PERF_POLICY_COUNT
} brmlPerfPolicyType_t;

/**
 * PCIe utilization counters
 */
typedef enum brmlPcieUtilCounter_enum {
    BRML_PCIE_UTIL_TX_BYTES             = 0,
    BRML_PCIE_UTIL_RX_BYTES             = 1,

    // Keep this last
    BRML_PCIE_UTIL_COUNT
} brmlPcieUtilCounter_t;

/**
 * Value types
 */
typedef union brmlValue_st {
    double dVal;                    //!< The value is double
    unsigned int uiVal;             //!< The value is unsigned int
    unsigned long ulVal;            //!< The value is unsigned long
    unsigned long long ullVal;      //!< The value is unsigned long long
    signed long long sllVal;        //!< The value is signed long long
} brmlValue_t;

/**
 * GPU virtualization mode types
 */
typedef enum brmlGpuVirtualizationMode {
    BRML_GPU_VIRTUALIZATION_MODE_NONE = 0,             //!< Bare Metal GPU
    BRML_GPU_VIRTUALIZATION_MODE_PASSTHROUGH = 1,      //!< Device is associated with GPU-Passthorugh
    BRML_GPU_VIRTUALIZATION_MODE_VGPU = 2,             //!< Device is associated with vGPU inside virtual machine
    BRML_GPU_VIRTUALIZATION_MODE_HYPERVISOR_HOST = 3,  //!< Device is associated with hypervisor in vGPU mode
} brmlGpuVirtualizationMode_t;

/**
 * Encode types
 */
typedef enum brmlEncoderQueryType_enum {
    BRML_ENCODER_QUERY_H264 = 0,
    BRML_ENCODER_QUERY_HEVC = 1,
} brmlEncoderType_t;

/**
 * Compute mode.
 */
typedef enum brmlComputeMode_enum {
    BRML_COMPUTEMODE_DEFAULT           = 0,  //!< Multiple contexts are allowed per device.
    BRML_COMPUTEMODE_EXCLUSIVE_PROCESS = 1,  //!< Only one context is allowed per device, usable from multiple threads at a time
    BRML_COMPUTEMODE_PROHIBITED        = 2,  //!< Contexts are allowed per device (no compute apps).
    // Keep this last
    BRML_COMPUTEMODE_COUNT
} brmlComputeMode_t;

/**
 * P2p link types.
 */
typedef enum brmlP2pLinkType_enum {
    BRML_P2P_NO_LINK = 0,          //!< Device has no connection
    BRML_P2P_INDIRECT_LINK = 1,    //!< P2P with indriect connected
    BRML_P2P_DIRECT_LINK = 2,      //!< P2P with driect connected
} brmlP2pLinkType_t;

/**
 * Performance states.
 */
typedef enum brmlPstate_enum {
    BRML_PSTATE_0 = 0,          //!< Maximum Performance
    BRML_PSTATE_1 = 1,          //!< Performance state 1
    BRML_PSTATE_2 = 2,          //!< Performance state 2
    BRML_PSTATE_3 = 3,          //!< Performance state 3
} brmlPstate_t;

/**
 * Brlink error counters
 */
typedef enum brmlBrLinkErrorCounter_enum {
    BRML_BRLINK_ERROR_DL_REPLAY   = 0,     //!< Data link transmit replay error counter
    BRML_BRLINK_ERROR_DL_RECOVERY = 1,     //!< Data link transmit recovery error counter
    BRML_BRLINK_ERROR_DL_CRC_FLIT = 2,     //!< Data link receive flow control digit CRC error counter
    BRML_BRLINK_ERROR_DL_CRC_DATA = 3,     //!< Data link receive data CRC error counter
    // keep this be last
    BRML_BRLINK_ERROR_COUNT
} brmlBrLinkErrorCounter_t;

/** @} */

/***************************************************************************************************/
/** @defgroup brmlDeviceStructs Device Structs
 *  @{
 */
/***************************************************************************************************/

/**
 * Pointer to a GPU Device struct
 */
typedef struct brmlDevice_st* brmlDevice_t;

/**
 * PCI information about a GPU device.
 */
typedef struct brmlPciInfo_st {
    unsigned int domain;             //!< The PCI domain on which the device's bus resides, 0 to 0xffffffff
    unsigned int bus;                //!< The bus on which the device resides, 0 to 0xff
    unsigned int device;             //!< The device's id on the bus, 0 to 31
    unsigned int function;           //!< The device's function id on the bus, 0 to 7

    unsigned int pciDeviceId;        //!< The combined 16-bit device id and 16-bit vendor id
    unsigned int pciSubSystemId;     //!< The 32-bit Sub System Device ID

    char busId[BRML_DEVICE_PCI_BUS_ID_BUFFER_SIZE]; //!< The tuple domain:bus:device.function PCI identifier (&amp; NULL terminator)
} brmlPciInfo_t;

/**
 * PCI information about a GPU device.
 */
typedef struct brmlPciErrorCounter_st {
    unsigned int ebufOverflow[16];         //!< Counts of EBUF Overflow error
    unsigned int ebufUnderrun[16];         //!< Counts of EBUF Under_run error
    unsigned int decodeError[16];          //!< Counts of Decode error
    unsigned int runningDisparity[16];     //!< Counts of Running Disparity error
    unsigned int skpParity[16];            //!< Counts of SKP Parity error
    unsigned int syncHeader[16];           //!< Counts of Sync Header error
    unsigned int rxDeassertion[16];        //!< Counts of Rx De-assertion error
    unsigned int ctlSkpParity[16];         //!< Counts of CTL SKP Parity error
    unsigned int firstRetimer[16];         //!< Counts of 1st retimer error
    unsigned int secondeRetimer[16];       //!< Counts of 2nd retimer error
    unsigned int marginCrcParity[16];      //!< Counts of Margin CRC Parity error
    unsigned int detectEiInfer;            //!< Counts of Detect EI Infer error
    unsigned int receiverError;            //!< Counts of Receiver error
    unsigned int rxRecoveryReq;            //!< Counts of Rx Recovery Request error
    unsigned int nFtsTimeout;              //!< Counts of N FTS Timeout error
    unsigned int framingError;             //!< Counts of Framing error
    unsigned int deskewError;              //!< Counts of Deskew error
    unsigned int badTlp;                   //!< Counts of Bad TLP error
    unsigned int lcrcError;                //!< Counts of LCRC error
    unsigned int badDllp;                  //!< Counts of Bad DLLP error
    unsigned int replayNumRollover;        //!< Counts of Replay Num Rollover error
    unsigned int replayTimeout;            //!< Counts of Replay timeout error
    unsigned int rxNakDllp;                //!< Counts of Rx NAK DLLP error
    unsigned int txNakDllp;                //!< Counts of Tx NAK DLLP error
    unsigned int retryTlp;                 //!< Counts of Retry TLP error
    unsigned int fcTimeout;                //!< Counts of FC Timeout error
    unsigned int positionedTlp;            //!< Counts of Positioned TLP error
    unsigned int ecrcError;                //!< Counts of ECRC error
    unsigned int unsupportReq;             //!< Counts of Unsupport Request error
    unsigned int completerAbort;           //!< Counts of Completer Abort error
    unsigned int completionTimeout;        //!< Counts of Completion Timeout error

    unsigned int totalError;               //!< Total counts of PCIe error
} brmlPciErrorCounter_t;

/**
 * Detailed ECC error counts for a device.
 */
typedef struct brmlEccErrorCounts_st {
    unsigned long long deviceMemory; //!< Device memory errors
} brmlEccErrorCounts_t;

/**
 * Utilization information for a device.
 */
typedef struct brmlUtilization_st {
    unsigned int gpu;                //!< Percent of time over the past sample period during which one or more kernels was executing on the GPU
    unsigned int memory;             //!< Percent of time over the past sample period during which global (device) memory was being read or written
} brmlUtilization_t;

/**
 * Gpu utilization information.
 */
typedef struct brmlGpuUtilization_st {
    unsigned int spc;                //!< Percent of spc utilization
    unsigned int cp;                 //!< Percent of cp utilization
    unsigned int sdma;               //!< Percent of sdma utilization
    unsigned int vdcp;               //!< Percent of video decoder cp utilization
    unsigned int vecp;               //!< Percent of video encoder cp utilization
    unsigned int vdec;               //!< Percent of video decoder utilization
    unsigned int venc;               //!< Percent of video encoder utilization
} brmlGpuUtilization_t;

/**
 * Memory allocation information for a device.
 */
typedef struct brmlMemory_st {
    unsigned long long total;        //!< Total installed FB memory (in bytes)
    unsigned long long free;         //!< Unallocated FB memory (in bytes)
    unsigned long long used;         //!< Allocated FB memory (in bytes).
} brmlMemory_t;

/**
 * Information about running compute processes on the GPU
 */
typedef struct brmlProcessInfo_st {
    unsigned int        pid;                //!< Process ID
    unsigned long long  usedGpuMemory;      //!< Amount of used GPU memory in bytes.
    unsigned long long  usedHostMemory;     //!< Amount of used Host memory in bytes.
} brmlProcessInfo_t;

/**
 * Static GPU info.
 */
typedef struct brmlGPUInfo_st {
    char name[96];                      //!< Product name
    char id[96];                        //!< Product identifier
    char serial[96];                    //!< Product serial number
    char board_id[96];                  //!< Product board id
} brmlGPUInfo_t;

/**
 * Structure to store utilization value and process Id
 */
typedef struct brmlProcessUtilizationSample_st {
    unsigned int pid;                   //!< PID of process
    unsigned long long timeStamp;       //!< CPU Timestamp in microseconds
    unsigned int memUtil;               //!< Frame Buffer Memory Util Value
} brmlProcessUtilizationSample_t;

/**
 * Information about occurred event
 */
typedef struct brmlEventData_st {
    brmlDevice_t        device;             //!< Specific device where the event occurred
    unsigned long long  eventType;          //!< Specific type about the event occurred
    unsigned long long  eventData;          //!< Stores event data
} brmlEventData_t;

/**
 * Structure to hold power samples data
 */
typedef struct brmlPowerSamples_st {
    unsigned int      min;           //!< Min Power Value
    unsigned int      max;           //!< Max Power Value
    unsigned int      avg;           //!< Average Power Value
} brmlPowerSamples_t;

/**
 * Structure to hold p2p link data
 */
typedef struct brmlP2pStatus_st {
    brmlP2pLinkType_t type;    //!< P2P link type
    unsigned int count;        //!< P2P connected port count
    unsigned int port[8];      //!< P2P port list
} brmlP2pStatus_t;

/**
 * Structure to hold p2p throughput
 */
typedef struct brmlP2pThroughput_st {
    unsigned int rx_throughput; //!< P2p rx throughput
    unsigned int tx_throughput; //!< P2p tx throughput
} brmlP2pThroughput_t;

/**
 * Structure to memory bandwidth
 */
typedef struct brmlMemoryBandwidth_st {
    unsigned int read_bandwidth;   //!< GPU memory read bandwidth
    unsigned int write_bandwidth;  //!< GPU memory write bandwidth
} brmlMemoryBandwidth_t;

/** @} */

/***************************************************************************************************/
/** @defgroup brmlInitializationAndCleanup Initialization and Cleanup
 * This chapter describes the methods that handle BRML initialization and cleanup.
 * It is the user's responsibility to call \ref brmlInit() before calling any other methods, and
 * \ref brmlShutdown() once BRML is no longer being used.
 *  @{
 */
/***************************************************************************************************/

/**
 * Initialize BRML, but don't initialize any GPUs yet.
 *
 * This allows BRML to communicate with a GPU
 * when other GPUs in the system are unstable or in a bad state.  When using this API, GPUs are
 * discovered and initialized in brmlDeviceGetHandleBy* functions instead.
 *
 * This method, should be called once before invoking any other methods in the library.
 * A reference count of the number of initializations is maintained.  Shutdown only occurs
 * when the reference count reaches zero.
 *
 * @return
 *         - \ref BRML_SUCCESS                   if BRML has been properly initialized
 *         - \ref BRML_ERROR_DRIVER_NOT_LOADED   if BIREN driver is not running
 */
brmlReturn_t DECLDIR brmlInit(void);

/**
 * Shut down BRML by releasing all GPU resources previously allocated with \ref brmlInit().
 *
 * This method should be called after BRML work is done, once for each call to \ref brmlInit()
 * A reference count of the number of initializations is maintained.  Shutdown only occurs
 * when the reference count reaches zero.  For backwards compatibility, no error is reported if
 * brmlShutdown() is called more times than brmlInit().
 *
 * @return
 *         - \ref BRML_SUCCESS                 if BRML has been properly shut down
 */
brmlReturn_t DECLDIR brmlShutdown(void);

/** @} */

/***************************************************************************************************/
/** @defgroup brmlErrorReporting Error reporting
 * This chapter describes helper functions for error reporting routines.
 *  @{
 */
/***************************************************************************************************/

/**
 * Helper method for converting BRML error codes into readable strings.
 *
 * @param result                               BRML error code to convert
 *
 * @return String representation of the error.
 *
 */
const DECLDIR char* brmlErrorString(brmlReturn_t result);

/** @} */

/***************************************************************************************************/
/** @defgroup brmlSystemQueries System Queries
 * This chapter describes the queries for BRML to match the local system. These queries
 * are not device-specific.
 *  @{
 */
/***************************************************************************************************/

/**
 * Get the system's graphics driver version.
 *
 * The version identifier is an alphanumeric string.  
 * It will be at most 80 characters (including the NULL terminator).
 * See \ref brmlConstants::BRML_SYSTEM_DRIVER_VERSION_BUFFER_SIZE.
 *
 * @param version                              The driver version identifier
 * @param length                               The maximum length of the string returned in \a version
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a version has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a version is NULL
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE \a length is too small
 *         - \ref BRML_ERROR_NOT_FOUND         version information is not found
 */
brmlReturn_t DECLDIR brmlSystemGetDriverVersion(char* version, unsigned int length);

/**
 * Get the system's Umd driver version.
 *
 * It will return the current version of Umd driver.
 * If the UMD library is not found, this function will return an error.
 *
 * @param umdDriverVersion                     UMD driver version identifier 
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a umdDriverVersion has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a umdDriverVersion is NULL
 *         - \ref BRML_ERROR_NOT_FOUND         UMD library is not found
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlSystemGetUmdDriverVersion(int* umdDriverVersion);

/**
 * Get the version of the BRML library.
 *
 * It will return the current version of BRML.
 * The version identifier is an alphanumeric string.  
 * It will be at most 80 characters (including the NULL terminator).
 * See \ref brmlConstants::BRML_SYSTEM_BRML_VERSION_BUFFER_SIZE.
 *
 * @param version                              BRML version identifier
 * @param length                               The maximum length of the string returned in \a version
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a version has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a version is NULL
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE \a length is too small
 */
brmlReturn_t DECLDIR brmlSystemGetBRMLVersion(char* version, unsigned int length);

/**
 * Get the version of the SUPA driver.
 *
 * It will return the currently version of SUPA.
 * If the SUPA library is not found, this function will return an error.
 *
 * @param supaDriverVersion                    SUPA version identifier
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a supaDriverVersion has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a supaDriverVersion is NULL
 *         - \ref BRML_ERROR_NOT_FOUND         SUPA library is not found
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlSystemGetSupaDriverVersion(int* supaDriverVersion);

/**
 * Get minor number for the device.   
 * 
 * It will return the current minor number for the device,
 * which is such that the Biren device node file for each GPU.
 *
 * @param device                               The target device identifier
 * @param minorNumber                          The minor number for the device
 * @return
 *         - \ref BRML_SUCCESS                 the minor number is got successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a minorNumber is NULL
 */
brmlReturn_t DECLDIR brmlDeviceGetMinorNumber(brmlDevice_t device, unsigned int* minorNumber);

/**
 * Get the device board part number which is programmed into the board's FRU.
 * 
 * It will be at most 80 characters (including the NULL terminator).
 * See \ref brmlConstants::BRML_DEVICE_PART_NUMBER_BUFFER_SIZE.
 *
 * @param device                                The target device identifier
 * @param partNumber                            The part number for the device
 * @param length                                The maximum length of the string returned in \a partNumber
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a partNumber has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_NOT_SUPPORTED      the needed fields have not been filled
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a device is invalid or \a partNumber is NULL
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE  \a length is too small
 *         - \ref BRML_ERROR_UNKNOWN            unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetBoardPartNumber(brmlDevice_t device, char* partNumber, unsigned int length);

/**
 * Retrieves the the device product part number which is programmed into the product's InfoROM
 *
 * For all products.
 *
 * @param device                                Identifier of the target device
 * @param partNumber                            Reference to the buffer to return
 * @param length                                Length of the buffer reference
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a partNumber has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_NOT_SUPPORTED      the needed fields have not been filled
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a device is invalid or \a partNumber is NULL
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE  \a length is too small
 *         - \ref BRML_ERROR_UNKNOWN            unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetProductPartNumber(brmlDevice_t device, char* partNumber, unsigned int length);

/**
 * Get the globally unique immutable UUID associated with this device.
 * 
 * It will be at most 80 characters (including the NULL terminator).
 * See \ref brmlConstants::BRML_DEVICE_UUID_BUFFER_SIZE.
 *
 * @param device                                The target device identifier
 * @param uuid                                  The UUID of the device
 * @param length                                The maximum length of the string returned in \a uuid
 * 
 * @return
 *         - \ref BRML_SUCCESS                  \a uuid has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_NOT_SUPPORTED      the needed fields have not been filled
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a device is invalid or \a uuid is NULL
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE  \a length is too small
 *         - \ref BRML_ERROR_UNKNOWN            unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetUUID(brmlDevice_t device, char* uuid, unsigned int length);

/**
 * Get the device vendor which is programmed into the board's FRU.
 *
 * @param device                                The target device identifier
 * @param boardVendor                           The verdor name for the device
 * @param length                                The maximum length of the string returned in \a boardVendor
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a boardVendor has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_NOT_SUPPORTED      the needed fields have not been filled
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a device is invalid or \a boardVendor is NULL
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE  \a length is too small
 *         - \ref BRML_ERROR_UNKNOWN            unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetVendorName(brmlDevice_t device, char* boardVendor, unsigned int length);

/**
 * Get the amount of used, free and total memory available on the device, in bytes.
 *
 * @param device                               The target device identifier
 * @param memory                               The amount of used, free and total memory available on the device, in bytes.
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a memory is got successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, \a memory is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     this query is not supported by \a device
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 *
 */
brmlReturn_t DECLDIR brmlDeviceGetMemoryInfo(brmlDevice_t device, brmlMemory_t* memory);

/**
 * Get the device throttled status due to power or thermal constraints.
 *
 * @param device                               The target device identifier
 * @param perfPolicyType                       The current performance policy which causes GPU throttling
 * @param value                                The value of violation status  
 *
 *
 * @return
 *         - \ref BRML_SUCCESS                 violation status is got successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, \a perfPolicyType is NULL, or \a value is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     this query is not supported by the device
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 *
 */
brmlReturn_t DECLDIR brmlDeviceGetViolationStatus(brmlDevice_t device, brmlPerfPolicyType_t perfPolicyType,
                                                  unsigned int* value);

/**
 * Get the process name with provided process id
 *
 * Returned process name is cropped to provided length.
 * The name string is encoded in ANSI.
 * 
 *
 * @param pid                                  The process id
 * @param name                                 The process name
 * @param length                               The maximum length of the string returned in \a name
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a name is got successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a name is NULL
 *         - \ref BRML_ERROR_NOT_FOUND         process \a pid doesn't exists
 *         - \ref BRML_ERROR_NO_PERMISSION     the user doesn't have permission to perform this operation
 *         - \ref BRML_ERROR_UNEXPECTED_SIZE   \a length is invalid
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlSystemGetProcessName(unsigned int pid, char* name, unsigned int length);

/** @} */

/***************************************************************************************************/
/** @defgroup brmlDeviceQueries Device Queries
 * This chapter describes that queries that BRML can perform against each device.
 * In each case the device is identified with an brmlDevice_t handle. This handle can be obtained
 * by calling one of \ref brmlDeviceGetHandleByIndex(), * \ref brmlDeviceGetHandleByPciBusId().
 *  @{
 */
/***************************************************************************************************/

/**
 * Get the version of specific firmware.
 *
 * The version identifier is an alphanumeric string.  
 * It will be at most 80 characters in length(including the NULL terminator).  
 * See \ref brmlConstants::BRML_SYSTEM_FIRMWARE_VERSION_BUFFER_SIZE.
 *
 * @param device                               The target device identifier
 * @param firmwareType                         The type of the firmware
 * @param version                              The firmware version
 * @param length                               The maximum length of the string returned in \a version
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a version has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a version is NULL
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE \a length is too small
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a firmwareType is NULL
 */
brmlReturn_t DECLDIR brmlDeviceGetFirmwareVersion(brmlDevice_t device, brmlGPUFirmwareType_t firmwareType,     
                                                  char* version, int length);


/**
* Get the number of GPUs in the computer system. 
* GPUs BRML can not access will not be included.
*
* @param deviceCount                          The amount of accessible devices 
*
* @return
*         - \ref BRML_SUCCESS                 \a deviceCount has been set successfully
*         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
*         - \ref BRML_ERROR_INVALID_ARGUMENT  \a deviceCount is NULL
*/
brmlReturn_t DECLDIR brmlDeviceGetCount(unsigned int* deviceCount);

/**
 * Get the device handle through Index
 *
 * Valid indices are determined by \ref brmlDeviceGetCount() and delivered by \a deviceCount. 
 * For example, if \a deviceCount is 2, the valid indices will be 0 and 1, corresponding to GPU 0 and GPU 1.
 *   
 *
 * The Index may change during reboots. 
 * For this reason it is recommended that devices are looked up by their PCI ids or GPU UUID. 
 * See \ref brmlDeviceGetHandleByPciBusId().
 *
 * Note: The BRML index may not correlate with other APIs, such as the SUPA device index.
 *
 * @param index                                The target GPU's index, >= 0 and < \a deviceCount
 * @param device                               The device handle
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a device has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a device is invailid
 *         - \ref BRML_ERROR_NOT_FOUND          \a index does not match a device
 *
 * @see brmlDeviceGetIndex
 * @see brmlDeviceGetCount
 */
brmlReturn_t DECLDIR brmlDeviceGetHandleByIndex(unsigned int index, brmlDevice_t* device);

/**
 * Get the device handle through its board serial number.
 *
 * This number corresponds to the value printed directly on the board.
 *
 * @deprecated Since more than one GPU can exist on a single board this function is deprecated in favor
 *             of \ref brmlDeviceGetHandleByUUID.
 *             For dual GPU boards this function will return BRML_ERROR_INVALID_ARGUMENT.
 *
 * @param serial                               The board serial number
 * @param device                               The device handle of the specified board
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a device has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a serial is invalid, \a device is NULL or more than one
 *                                              device has the same serial (dual GPU boards)
 *         - \ref BRML_ERROR_NOT_FOUND          \a serial does not match a valid device on the system
 *         - \ref BRML_ERROR_UNKNOWN            unknown error occurs
 *
 * @see brmlDeviceGetHandleByUUID
 */
brmlReturn_t DECLDIR brmlDeviceGetHandleBySerial(const char* serial, brmlDevice_t* device);

/**
 * Get the handle for a particular device, based on its globally unique immutable UUID associated with each device.
 *
 * For all products.
 *
 * @param uuid                                 The target GPU's uuid
 * @param device                               The device handle
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a device has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a uuid is invalid or \a device is null
 *         - \ref BRML_ERROR_NOT_FOUND          \a uuid does not match a valid device on the system
 *         - \ref BRML_ERROR_UNKNOWN            unknown error occurs
 *
 * @see brmlDeviceGetUUID
 */
brmlReturn_t DECLDIR brmlDeviceGetHandleByUUID(const char* uuid, brmlDevice_t* device);

/**
 * Get the BRML index of this device.
 *
 * Valid indices are determined by \ref brmlDeviceGetCount() and delivered by \a deviceCount. 
 * For example, if \a deviceCount is 2, the valid indices will be 0 and 1, corresponding to GPU 0 and GPU 1.
 *
 * The Index may change during reboots. 
 * For this reason it is recommended that devices are looked up by their PCI ids or GPU UUID. 
 * See \ref brmlDeviceGetHandleByPciBusId().
 *
 * Note: The BRML index may not correlate with other APIs, such as the SUPA device index.
 *
 * @param device                               The target device identifier
 * @param index                                The BRML index of the device
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a index has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a index is NULL
 *         - \ref BRML_ERROR_NOT_FOUND         \a device is not found
 *
 * @see brmlDeviceGetHandleByIndex()
 * @see brmlDeviceGetCount()
 */
brmlReturn_t DECLDIR brmlDeviceGetIndex(brmlDevice_t device, unsigned int* index);

/**
 * Get the BRML node id of this device.
 *
 * @param device                               The target device identifier
 * @param nodeId                               The BRML node id of the device
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a get node id successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a nodeId is NULL
 *         - \ref BRML_ERROR_NOT_FOUND         \a device is not found
 */
brmlReturn_t DECLDIR brmlDeviceGetNodeId(brmlDevice_t device, unsigned int* nodeId);

/**
 * Get the device handle through node id.
 * If SVI is enabled, it will return the gpu instance which node id matches the given nodeId.
 *
 * @param nodeId                               The device node id
 * @param device                               The device handle
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a device has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invailid
 *         - \ref BRML_ERROR_NOT_FOUND         \a nodeId does not match a valid device on the system.
 */
brmlReturn_t DECLDIR brmlDeviceGetHandleByNodeId(unsigned int nodeId, brmlDevice_t* device);

/**
 * Reset the GPU node.
 * Requires root/admin permissions.
 *
 * @param device                               The target device identifier
 * @param type                                 GPU Reset type
 *
 * @return
 *         - \ref BRML_SUCCESS                 new settings has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a type is NULL
 */
brmlReturn_t DECLDIR brmlDeviceResetGpu(brmlDevice_t device, brmlGpuResetType_t type);

/**
 * Get the handle for a particular device, based on its PCI bus id.
 *
 * This value corresponds to the brmlPciInfo_t::busId returned by \ref brmlDeviceGetPciInfo().
 *
 * @param pciBusId                              The PCI bus id of the target GPU
 * @param device                                The device handle
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a device has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a pciBusId is NULL or \a device is invalid
 *         - \ref BRML_ERROR_NOT_FOUND          \a pciBusId does not match a valid device on the system
 */
brmlReturn_t DECLDIR brmlDeviceGetHandleByPciBusId(const char* pciBusId, brmlDevice_t* device);

/**
 * Get the PCI attributes of the this device.
 *
 * See \ref brmlPciInfo_t for details on the available PCI info.
 *
 * @param device                               The target device identifier
 * @param pciInfo                              The PCI info
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a pciInfo has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a pciInfo is invalid
 */
brmlReturn_t DECLDIR brmlDeviceGetPciInfo(brmlDevice_t device, brmlPciInfo_t* pciInfo);

/**
 * Get the Physical slot number of the this device.
 *
 * @param device                               The target device identifier
 * @param phySlotNum                           The Physical slot number
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a phySlotNum has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a phySlotNum is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     Physical slot number is not supported
 */
brmlReturn_t DECLDIR brmlDeviceGetPhysicalSlotNumber(brmlDevice_t device, unsigned int* phySlotNum);

/**
 * Get the maximum PCIe link width possible with this device and system
 *
 * I.E. for a device with a 16x PCIe bus width attached to a 8x PCIe system bus this function will report
 * a max link width of 8.
 *
 * @param device                               The target device identifier
 * @param maxLinkWidth                         The max PCIe link width
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a maxLinkWidth has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a maxLinkWidth is null
 *         - \ref BRML_ERROR_NOT_SUPPORTED     PCIe link information is not supported
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMaxPcieLinkWidth(brmlDevice_t device, unsigned int* maxLinkWidth);

/**
 * Get the current PCIe link width
 *
 * @param device                               The target device identifier
 * @param currLinkWidth                        The current PCIe link width
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a currLinkWidth has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a currLinkWidth is null
 *         - \ref BRML_ERROR_NOT_SUPPORTED     PCIe link information is not supported
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetCurrPcieLinkWidth(brmlDevice_t device, unsigned int* currLinkWidth);

/**
 * Get the maximum PCIe link generation possible with this device and system
 *
 * I.E. for a generation 2 PCIe device attached to a generation 1 PCIe bus the max link generation this function will
 * report is generation 1.
 *
 * @param device                               The target device identifier
 * @param maxLinkGen                           The max PCIe link generation
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a maxLinkGen has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a maxLinkGen is null
 *         - \ref BRML_ERROR_NOT_SUPPORTED     PCIe link information is not supported
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMaxPcieLinkGeneration(brmlDevice_t device, unsigned int* maxLinkGen);

/**
 * Get the current PCIe link generation
 *
 * @param device                               The target device identifier
 * @param currLinkGen                          The current PCIe link generation
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a currLinkGen has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a currLinkGen is null
 *         - \ref BRML_ERROR_NOT_SUPPORTED     PCIe link information is not supported
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetCurrPcieLinkGeneration(brmlDevice_t device, unsigned int* currLinkGen);

/**
 * Get PCIe utilization information.
 * This function is querying a byte counter over a 20ms interval and thus counts the
 *   PCIe throughput over that interval.
 *
 * This method is not supported in virtual machines running virtual GPU (vGPU).
 *
 * @param device                               The target device identifier
 * @param counter                              The specific counter type that fits \ref brmlPcieUtilCounter_t
 * @param value                                Throughput in KB/s
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a value has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a counter is invalid, or \a value is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPcieThroughput(brmlDevice_t device, brmlPcieUtilCounter_t counter,     
                                                 unsigned int* value);

/**
 * Get the PCIe replay counts.
 *
 * @param device                               The target device indentifier
 * @param value                                The PCIe replay counts
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a value has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a value is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPcieReplayCounter(brmlDevice_t device, unsigned int* value);

/**
 * Get the PCIe replay number rollover times.
 *
 * @param device                               The target device indentifier
 * @param value                                The PCIe replay number rollover times
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a value has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a value is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPcieReplayNumberRollovers(brmlDevice_t device, unsigned int* value);

/**
 * Get the PCIe group error counter.
 * 
 * @param device                               The target device indentifier
 * @param counter                              The counter's values
 * 
 * @return
 *         - \ref BRML_SUCCESS                 \a counter has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a counter is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPcieErrorCounter(brmlDevice_t device, brmlPciErrorCounter_t* counter);

/**
 * Get the virtualization mode corresponding to the GPU.
 *
 * @param device                                The target device indentifier
 * @param virtualMode                           The GPU's virtualization mode
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a virtualMode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a device is invalid or \a virtualMode is NULL
 */
brmlReturn_t DECLDIR brmlDeviceGetVirtualizationMode(brmlDevice_t device, brmlGpuVirtualizationMode_t* virtualMode);

/**
 * Get GPU basic information, e.g. product name, id, and serial number etc.
 *
 * @param device                               The target device indentifier
 * @param info                                 The GPU basic info
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a info has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a info is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 */
brmlReturn_t DECLDIR brmlDeviceGetGpuInfo(brmlDevice_t device, brmlGPUInfo_t* info);

/**
 * Get the current clock speeds for the device.
 *
 * See \ref brmlClockType_t for details on available clock information.
 *
 * @param device                               The target device identifier
 * @param type                                 The clock domain to query
 * @param clock                                The clock speed, in MHz
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a clock has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a clock is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device cannot report the specified clock
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetClockInfo(brmlDevice_t device, brmlClockType_t type, unsigned int* clock);

/**
 * Get the maximum clock speeds for the device.
 *
 * See \ref brmlClockType_t for details on available clock information.
 *
 * @param device                               The target device identifier
 * @param type                                 The clock domain to query
 * @param clock                                The maximum clock speed in MHz
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a clock has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a type is invalid or \a clock is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device cannot report the specified clock
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMaxClockInfo(brmlDevice_t device, brmlClockType_t type, unsigned int* clock);

/**
 * Set the clock freqs for the the device.
 *
 * See \ref brmlClockType_t for details on available clock information.
 *
 * Requires root/admin permissions.
 *
 * @param device                               The target device identifier
 * @param type                                 The clock domain to set
 * @param clockFreq                            The clock freqs value to set
 *
 * @return
 *         - \ref BRML_SUCCESS                 The specified clock frequence has been set to \a clockFreq successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a type is invalid, or \a clockFreq is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device cannot report the specified clock
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetClockFreqs(brmlDevice_t device, brmlClockType_t type, unsigned int* clockFreq);

/**
 * Get the current temperature readings for the device, in degrees C.
 *
 * See \ref brmlTemperatureSensors_t for details on available temperature sensors.
 *
 * Note: Only get temperature range for HBM temperature, \a temp (0b011:<85C, 0b010:85C~95C, 0b110:95~120C)
 *
 * @param device                               The target device identifier
 * @param sensorType                           The type of temperature sensor
 * @param temp                                 The temperature reading
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a temp has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a temp is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not have the specified sensor
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetTemperature(brmlDevice_t device, brmlTemperatureSensors_t sensorType,
                                              unsigned int* temp);

/**
 * Get the temperature threshold for the GPU with the specified threshold type, in degrees C.
 *
 * See \ref brmlTemperatureThresholds_t for details on available temperature thresholds.
 *
 * @param device                               The target device identifier
 * @param thresholdType                        The type of threshold value queried
 * @param temp                                 The temperature threshold
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a temp has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a thresholdType is invalid, or \a temp is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this operation
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetTemperatureThreshold(brmlDevice_t device, brmlTemperatureThresholds_t thresholdType,
                                                       unsigned int* temp);

/**
 * Get the power management limit for this GPU, in milliwatts.
 *
 * The power limit defines the upper boundary for the card's power draw. If the card's total power draw reaches this
 * limit the power management algorithm kicks in.
 *
 * This reading is only available if power management mode is supported.
 *
 * @param device                               The target device identifier
 * @param limit                                The power management limit, in milliwatts
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a limit has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a limit is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPowerManagementLimit(brmlDevice_t device, unsigned int* limit);

/**
 * Get the power threshold.
 *
 * The power limit defines the upper boundary for max power. If the card's total power draw reaches this
 * limit the power management algorithm kicks in.
 *
 * @param device                               The target device identifier
 * @param threshold                            The power threshold
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a limit has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a limit is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPowerThreshold(brmlDevice_t device, unsigned int* threshold);

/**
 * Set the power threshold.
 *
 * The power limit defines the upper boundary for max power. If the card's total power draw reaches this
 * limit the power management algorithm kicks in.
 *
 * @param device                               The target device identifier
 * @param threshold                            The power threshold need to set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a limit has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a limit is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetPowerThreshold(brmlDevice_t device, unsigned int* threshold);

/**
 * Get the power managements limit for this GPU, in milliwatts.
 *
 * @param device                               The target device identifier
 * @param minLimit                             The minimum power management limit
 * @param maxLimit                             The maximum power management limit
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a minLimit and \a maxLimit have been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or either \a minLimit or \a maxLimit is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support power readings
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPowerManagementLimitConstraints(brmlDevice_t device, unsigned int* minLimit,
                                                                  unsigned int* maxLimit);

/**
 * Get power usage for \a device, in milliwatts.
 *
 * For fully supported devices.
 *
 * @param device                               The target device identifier
 * @param power                                The power usage, in milliwatts
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a power has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a power is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPowerUsage(brmlDevice_t device, unsigned int* power);

/**
 * Get the power samples maintained in the buffer for the device.
 * 
 * @param device                               The target device identifier
 * @param power                                The power samples
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a power has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a power is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPowerSamples(brmlDevice_t device, brmlPowerSamples_t* power);

/**
 * Get current clocks throttling reasons.
 *
 * @param device                                The target device identifier
 * @param clocksThrottleReasons                 The current clocks throttle reasons
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a clocksThrottleReasons has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a clocksThrottleReasons is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 *
 * @see brmlClocksThrottleReasons
 * @see brmlDeviceGetSupportedClocksThrottleReasons
 */
brmlReturn_t DECLDIR brmlDeviceGetCurrentClocksThrottleReasons(brmlDevice_t device,
                                                               unsigned long long* clocksThrottleReasons);

/**
 * Get the average Current value for the device, in milliampere. 
 *
 * @param device                               The target device identifier
 * @param current                              The average Current value
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a current has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, \a current is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not have the specified sensor
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetCurrent(brmlDevice_t device, unsigned int* current);

/**
 * Get the current compute mode for the device.
 * See \ref brmlComputeMode_t for details on available compute modes.
 * When SVI is enabled, the \a device should be the gpu instance, \ref brmlDeviceGetGpuInstanceById.
 *
 * @param device                               The target device identifier
 * @param mode                                 The current compute mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetComputeMode(brmlDevice_t device, brmlComputeMode_t* mode);

/**
 * Set the compute mode for the device.
 *
 * The compute mode determines whether a GPU can be used for compute operations and whether it can
 * be shared across contexts.
 * See \ref brmlComputeMode_t for details on available compute modes.
 * When SVI is enabled, the \a device should be the gpu instance, \ref brmlDeviceGetGpuInstanceById.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The compute mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetComputeMode(brmlDevice_t device, brmlComputeMode_t mode);

/**
 * Get the current performance state for the device.
 * See \ref brmlPstate_t for for details on available performance states.
 *
 * @param device                               The target device identifier
 * @param pState                               The performance state
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a pState has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a pState is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetPerformanceState(brmlDevice_t device, brmlPstate_t* pState);

/**
 * Get the p2p throughput for the device.
 *
 * See \ref brmlP2pThroughput_t for details p2p throughput info.
 * See \ref brmlDeviceGetP2PStatus_v2 to get the p2p port.
 * The p2p throughput monitor should be enabled, \ref brmlDeviceSetP2pThroughputMonitorMode can be used
 * to set the mode.
 *
 * @param device                               The target device identifier
 * @param port                                 Specifies the p2p port to be queried
 * @param p2pThroughput                        The p2p throughput
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a p2pThroughput has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a port is invalid, or \a p2pThroughput is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetP2pThroughput(brmlDevice_t device, unsigned int port, brmlP2pThroughput_t* p2pThroughput);

/**
 * Get the p2p throughput monitor mode for the device.
 *
 * See \ref brmlEnableState_t for details on allowed modes.
 *
 * @param device                               The target device identifier
 * @param mode                                 The p2p throughput mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetP2pThroughputMonitorMode(brmlDevice_t device, brmlEnableState_t* mode);

/**
 * Set the p2p throughput monitor mode for the device.
 *
 * See \ref brmlEnableState_t for details on allowed modes.
 *
 * @param device                               The target device identifier
 * @param mode                                 The p2p throughput mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetP2pThroughputMonitorMode(brmlDevice_t device, brmlEnableState_t mode);

/**
 * Get the GPU memory bandwidth for the device.
 *
 * See \ref brmlMemoryBandwidth_t for details memory bandwidth info.
 * The memory bandwidth monitor should be enabled, \ref brmlDeviceSetMemoryBandwidthMonitorMode can be used
 * to set the mode.
 *
 * @param device                               The target device identifier
 * @param memBandwidth                         The memory bandwidth
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a memBandwidth has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a memBandwidth is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMemoryBandwidth(brmlDevice_t device, brmlMemoryBandwidth_t* memBandwidth);

/**
 * Get the GPU memory bandwidth monitor mode for the device.
 *
 * See \ref brmlEnableState_t for details on allowed modes.
 *
 * @param device                               The target device identifier
 * @param mode                                 The memory bandwidth mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMemoryBandwidthMonitorMode(brmlDevice_t device, brmlEnableState_t* mode);

/**
 * Set the GPU memory bandwidth monitor mode for the device.
 *
 * See \ref brmlEnableState_t for details on allowed modes.
 *
 * @param device                               The target device identifier
 * @param mode                                 The memory bandwidth mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetMemoryBandwidthMonitorMode(brmlDevice_t device, brmlEnableState_t mode);

/** @} */

/***************************************************************************************************/
/** @defgroup brmlAffinity Memory and Encoder/Decoder Queries
 *  BRML operations query infomations about Memory and Encoder/Decoder.
 *  @{
 */
/***************************************************************************************************/

/**
 * Get the requested memory error counts for the device.
 *
 * Requires ECC Mode to be enabled.
 *
 * See \ref brmlMemoryErrorType_t for a description of available memory error types.
 * See \ref brmlEccCounterType_t for a description of available counter types.
 * See \ref brmlMemoryLocation_t for a description of available counter locations.
 *
 * @param device                               The target device identifier
 * @param errorType                            The type of error
 * @param counterType                          The type of the error counter
 * @param locationType                         The type of memory
 * @param count                                The counts of ECC error
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a count has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a errorType is invalid, or \a count is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this opration
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMemoryErrorCounter(brmlDevice_t device, brmlMemoryErrorType_t errorType,
                                                     brmlEccCounterType_t counterType,
                                                     brmlMemoryLocation_t locationType, unsigned long long* count);

/**
 * Get the current utilization rates for the device.
 *
 * See \ref brmlUtilization_t for details on available utilization rates.
 *
 * @param device                               The target device identifier
 * @param utilization                          The utilization rates
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a utilization has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a utilization is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetUtilizationRates(brmlDevice_t device, brmlUtilization_t* utilization);

/**
 * Get the gpu utilization and sampling size in microseconds
 *
 * @param device                               The target device identifier
 * @param gpuUtilization                       The Gpu Utilization info
 * @param samplingPeriodUs                     Sampling period in US
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a gpuUtilization has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, \a gpuUtilization is NULL, or \a samplingPeriodUs is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetGpuUtilization(brmlDevice_t device, brmlGpuUtilization_t* gpuUtilization,
                                                 unsigned int* samplingPeriodUs);
/**
 * Get the current utilization and sampling size in microseconds for the Encoder
 *
 * @param device                               The target device identifier
 * @param utilization                          Encoder utilization info
 * @param samplingPeriodUs                     Sampling period in US
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a utilization has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, \a utilization is NULL, or \a samplingPeriodUs is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetEncoderUtilization(brmlDevice_t device, unsigned int* utilization,
                                                     unsigned int* samplingPeriodUs);

/**
 * Get the current utilization and sampling size in microseconds for the Decoder
 *
 * @param device                               The target device identifier
 * @param utilization                          The decoder utilization info
 * @param samplingPeriodUs                     Sampling period in US
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a utilization has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a utilization is NULL, or \a samplingPeriodUs is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetDecoderUtilization(brmlDevice_t device, unsigned int* utilization,
                                                     unsigned int* samplingPeriodUs);

/**
 * Get the current capacity of the device's encoder, with valid values in the range 0-100%.
 *
 * @param device                                The target device identifier
 * @param encoderQueryType                      Flag that indicates the type of encoder to query
 * @param encoderCapacity                       The encoder capacity
 *
 * @return
 *         - \ref BRML_SUCCESS                  \a encoderCapacity has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED      BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT   \a device is invalid, or \a encoderCapacity is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED      \a device does not support the encoder specified in \a encodeQueryType
 *         - \ref BRML_ERROR_UNKNOWN            unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetEncoderCapacity(brmlDevice_t device, brmlEncoderType_t encoderQueryType,
                                                   unsigned int* encoderCapacity);

/**
 * Get the current ECC mode for the device.
 *
 * Changing ECC mode requires a reboot.
 *
 * See \ref brmlEnableState_t for details on allowed modes.
 *
 * @param device                               The target device identifier
 * @param state                                The current ECC mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a state has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a state is NULL
 */
brmlReturn_t DECLDIR brmlDeviceGetEccMode(brmlDevice_t device, brmlEnableState_t* state);

/**
 * Get the total ECC error counts for the device.
 *
 * Requires ECC Mode to be enabled.
 *
 * See \ref brmlMemoryErrorType_t for a description of available error types.\n
 * See \ref brmlEccCounterType_t for a description of available counter types.
 *
 * @param device                               The target device identifier
 * @param errorType                            The type of the error
 * @param counterType                          The type of the error counter
 * @param eccCounts                            The counts of specified ECC error
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a eccCounts has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a errorType is invalid, or \a eccCounts is NULL
 *
 * @see brmlDeviceClearEccErrorCounts()
 */
brmlReturn_t DECLDIR brmlDeviceGetTotalEccErrors(brmlDevice_t device, brmlMemoryErrorType_t errorType,
                                                 brmlEccCounterType_t counterType, unsigned long long* eccCounts);

/**
 * Reset the ECC error and other memory error counts for the device.
 *
 * Requires ECC Mode to be enabled.
 * Sets all of the specified ECC counters to 0 immediately.
 *
 * Requires root/admin permissions.
 *
 * See \ref brmlMemoryErrorType_t for details on available counter types.
 *
 * @param device                               The target device identifier
 * @param counterType                          The type of errors should be reset.
 *
 * @return
 *         - \ref BRML_SUCCESS                 The error counts were reset
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a counterType is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_NO_PERMISSION     the user doesn't have permission to perform this operation
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 *
 * @see
 *      - brmlDeviceGetTotalEccErrors()
 */
brmlReturn_t DECLDIR brmlDeviceClearEccErrorCounts(brmlDevice_t device, brmlEccCounterType_t counterType);

/**
 * Get the total AER(Advanced Error Reporting) error counts for the device.
 * 
 * Requires root/admin permissions.
 *
 * @param device                               The target device identifier
 * @param errorType                            Flag that specifies the type of the AER error
 * @param aerCounts                            The specified AER error counts
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a aerCounts has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device, \a errorType is invalid, or \a aerCounts is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetTotalAerErrors(brmlDevice_t device, brmlAerErrorType_t errorType,
                                                 unsigned long long* aerCounts);

/**
 * Get the health status for the device.
 *
 * @param device                               The target device identifier
 * @param status                               The health status
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a status has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a status is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetHealthStatus(brmlDevice_t device, brmlGpuHealthStatus_t *status);

/**
 * Get information about running processes.
 *
 * This function returns information only about compute running processes (e.g. SUPA application which have
 * active context).
 * 
 * Invoking the function with *infoCount set to 0 can get the current number of running compute processes.
 * BRML_ERROR_INSUFFICIENT_SIZE or BRML_SUCCESS will be returned none are running. \a infos is allowed to
 * be NULL when calling the function.
 * When SVI is enabled, the \a device should be the gpu instance, \ref brmlDeviceGetGpuInstanceById.
 *
 * @param device                               The target device identifier
 * @param infoCount                            The number of running processes
 * @param infos                                The process information
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a infoCount and \a infos have been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE \a infos is NULL
 *         - \ref BRML_ERROR_NO_PERMISSION     the user doesn't have permission to perform this operation
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a infoCount is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     this query is not supported by \a device
 *         - \ref BRML_ERROR_NOT_FOUND         no running compute process found
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 *
 * @see \ref brmlSystemGetProcessName
 */
brmlReturn_t DECLDIR brmlDeviceGetComputeRunningProcesses(brmlDevice_t device, unsigned int* infoCount,
                                                          brmlProcessInfo_t* infos);

/** @} */

/***************************************************************************************************/
/** @defgroup brmlProcessQueries Query Process infos
 *  @{
 */
/***************************************************************************************************/

/**
 * Get the current utilization and process ID for device.
 *
 * This function reads recent utilization of memory of GPU and CPU timestamp for running processes, and return them as an array
 * of sturcture brmlProcessUtilizationSample_t pointed at by \a utilization . User should allocate the buffer of the arry. 
 *
 * User should invoke the function with \a utilization set to NULL first, to get the number of samples which can be read which
 * returned in \a processSamplesCount . Then invoke the function again with \a utilization point at the allocated buffer of size
 * processSamplesCount * sizeof(brmlProcessUtilizationSample_t).
 *
 * \a processSamplesCount will be updated with the number of samples that were actually written into \a utilization , and it
 * could be different from each operation as instances are created or destroyed.
 * When SVI is enabled, the \a device should be the gpu instance, \ref brmlDeviceGetGpuInstanceById.
 *
 * @param device                    The target device identifier
 * @param utilization               Pointer to caller-supplied buffer in which guest process utilization samples are returned
 * @param processSamplesCount       Pointer to caller-supplied array size, and returns number of processes running
 * @param lastSeenTimeStamp         Return only samples with timestamp greater than lastSeenTimeStamp

 * @return
 *         - \ref BRML_SUCCESS                 \a utilization and \a processSamplesCount have been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a processSamplesCount is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_INSUFFICIENT_SIZE \a utilization is NULL
 *         - \ref BRML_ERROR_NOT_FOUND         no running process found
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetProcessUtilization(brmlDevice_t device, brmlProcessUtilizationSample_t* utilization,
                                                     unsigned int* processSamplesCount, unsigned long long lastSeenTimeStamp);

/** @} */

/***************************************************************************************************/
/** @defgroup brmlTopology Topology Queries
* This chapter describes operations that are associated with topology.
 *  @{
 */
/***************************************************************************************************/

/**
 * Get the common ancestor for two GPUs.
 *
 * @param device1                              The first device identifier
 * @param device2                              The second device identifier
 * @param pathInfo                             The path type
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a pathInfo has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device1, or \a device2 is invalid, or \a pathInfo is NULL
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */

brmlReturn_t DECLDIR brmlDeviceGetTopologyCommonAncestor(brmlDevice_t device1, brmlDevice_t device2,
                                                         brmlGpuTopologyLevel_t* pathInfo);

/**
 * Get the list of local CPUs to which this GPU belongs.
 *
 * @param device                               The target device identifier
 * @param localCpuList                         The local cpu list
 * @param length                               The maximum length of the string returned in \a localCpuList
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a localCpuList has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a localCpuList is NULL
 */

brmlReturn_t DECLDIR brmlDeviceGetLocalCpuList(brmlDevice_t device, char* localCpuList, unsigned int length);

/**
 * Get the NUMA node to which this GPU belongs.
 *
 * @param device                               The target device identifier
 * @param numaNode                             The numa node
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a numaNode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a numaNode is NULL
 */

brmlReturn_t DECLDIR brmlDeviceGetNumaNode(brmlDevice_t device, unsigned int* numaNode);

/**
 * Get the port number of the first device in the two GPUs which have P2P connection between each other.
 *
 * If device1 has no p2p connection to device2, the \a portNum is set to 9999, means invalid port.
 * And it can only get one p2p port connection for each device. New interface see \ref brmlDeviceGetP2PStatus_v2.
 *
 * @param device1                                 The first device identifier
 * @param device2                                 The second device identifier
 * @param portNum                                 \a device1's port number
 *
 * @return
 *         - \ref BRML_SUCCESS                    \a portNum has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED        BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT     \a device1 or \a device2 is invalid, or \a portNum is NULL
 */
brmlReturn_t DECLDIR brmlDeviceGetP2PStatus(brmlDevice_t device1, brmlDevice_t device2, unsigned int* portNum);

/**
 * Get the p2p status of the first device in the two GPUs which have P2P connection between each other.
 *
 * @param device1                                 The first device identifier
 * @param device2                                 The second device identifier
 * @param p2pStatus                               \a device1's p2p status
 *
 * @return
 *         - \ref BRML_SUCCESS                    \a portNum has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED        BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT     \a device1 or \a device2 is invalid, or \a p2pStatus is NULL
 */
brmlReturn_t DECLDIR brmlDeviceGetP2PStatus_v2(brmlDevice_t device1, brmlDevice_t device2, brmlP2pStatus_t* p2pStatus);

/**
 * Get the link number to which GPU belongs.
 *
 * @param device                               The target device identifier
 * @param linkNum                              The link number
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a linkNum has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a linkNum is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetBrLinkNumber(brmlDevice_t device, unsigned int* linkNum);

/**
 * Get the PCI information for the remote node on a BrLink link.
 *
 * @param device                               The target device identifier
 * @param link                                 Specifies the BrLink link to be queried
 * @param pciInfo                              brmlPciInfo_t of the remote node for the specified link
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a pciInfo has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or \a link is invalid or \a pciInfo is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_NOT_FOUND         The remote device \a pciInfo is not found
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetBrLinkRemotePciInfo(brmlDevice_t device, unsigned int link, brmlPciInfo_t* pciInfo);

/**
 * Get the error counter value of specified link for the device.
 *
 * See \ref brmlBrLinkErrorCounter_t for details on available counter.
 * See \ref brmlDeviceGetBrLinkNumber to get the total link number.
 *
 * @param device                               The target device identifier
 * @param link                                 The specified link to be queried
 * @param counter                              The counter to be queried
 * @param value                                The error counter value
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a value has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device, link or counter is invalid or \a value is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetBrLinkErrorCounter(brmlDevice_t device, unsigned int link,
                                                     brmlBrLinkErrorCounter_t counter, unsigned long long* value);

/**
 * Get the brlink state for the device.
 *
 * See \ref brmlEnableState_t for details on available states.
 * See \ref brmlDeviceGetBrLinkNumber to get the total link number.
 *
 * @param device                               The target device identifier
 * @param link                                 The specified link to be queried
 * @param state                                The brlink state
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a state has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device or link is invalid or \a state is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetBrLinkState(brmlDevice_t device, unsigned int link, brmlEnableState_t* state);

/** @} */

/***************************************************************************************************/
/** @defgroup brmlDriverConfiguration GPU Driver Configuration
 * This chapter describes configuration functions for driver (internel use).
 *  @{
 */
/***************************************************************************************************/

/**
 * get the l2pad size for the device.
 *
 * @param device                               The identifier of the target device
 * @param size                                 The current l2pad size, in MB
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a size has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a size is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetL2padSize(brmlDevice_t device, unsigned int* size);

/**
 * Set the l2pad size for the device.
 *
 * @param device                               The identifier of the target device
 * @param size                                 The l2pad size to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a size has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a size is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetL2padSize(brmlDevice_t device, unsigned int size);

/**
 * get the dtg mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The current dtg mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetDtgMode(brmlDevice_t device, unsigned int* mode);

/**
 * Set the dtg mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The dtg mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetDtgMode(brmlDevice_t device, unsigned int mode);

/**
 * get the mcast VF number for the device.
 *
 * @param device                               The identifier of the target device
 * @param number                               The current mcast VF number
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a number has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a number is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMcastVfNumber(brmlDevice_t device, unsigned int* number);

/**
 * Set the mcast VF number for the device.
 *
 * The mcast VF number can only be set when device SVI is disabled \ref brmlDeviceSetSVIMode
 * and mcast half uma8/uma16 is not set \ref brmlDeviceSetMcastHalfUma8Mode \ref brmlDeviceSetMcastHalfUma16Mode
 *
 * @param device                               The identifier of the target device
 * @param number                               The mcast vf number to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a number has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a number is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_BUSY              \a device svi is enabled or mcast half uma8/uma16 is set
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetMcastVfNumber(brmlDevice_t device, unsigned int number);

/**
 * get the mcast half uma8 mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The current mcast half uma8 mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMcastHalfUma8Mode(brmlDevice_t device, unsigned int* mode);

/**
 * Set the mcast half uma8 mode for the device.
 *
 * The mcast half uma8 mode can only be set when device SVI is disabled \ref brmlDeviceSetSVIMode
 * and mcast half uma16 is not set \ref brmlDeviceSetMcastHalfUma16Mode
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The mcast half uma8 mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_BUSY              \a device svi is enabled or mcast half uma16 is set
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetMcastHalfUma8Mode(brmlDevice_t device, unsigned int mode);

/**
 * get the mcast half uma16 mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The current mcast half uma16 mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetMcastHalfUma16Mode(brmlDevice_t device, unsigned int* mode);

/**
 * Set the mcast half uma16 mode for the device.
 *
 * The mcast half uma16 mode can only be set when device SVI is disabled \ref brmlDeviceSetSVIMode
 * and mcast half uma8 is not set \ref brmlDeviceSetMcastHalfUma8Mode
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The mcast half uma16 mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_BUSY              \a device svi is enabled or mcast half uma8 is set
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetMcastHalfUma16Mode(brmlDevice_t device, unsigned int mode);

/**
 * get the health check cp timeout for the device.
 *
 * @param device                               The identifier of the target device
 * @param timeout                              The current timeout, in seconds
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a timeout has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a timeout is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetHealthCheckCpTimeout(brmlDevice_t device, unsigned int* timeout);

/**
 * Set the health check cp timeout for the device.
 *
 * @param device                               The identifier of the target device
 * @param timeout                              The timeout to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a timeout has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a timeout is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetHealthCheckCpTimeout(brmlDevice_t device, unsigned int timeout);

/**
 * get the health check Sdma timeout for the device.
 *
 * @param device                               The identifier of the target device
 * @param timeout                              The current timeout, in seconds
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a timeout has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a timeout is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetHealthCheckSdmaTimeout(brmlDevice_t device, unsigned int* timeout);

/**
 * Set the health check Sdma timeout for the device.
 *
 * @param device                               The identifier of the target device
 * @param timeout                              The timeout to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a timeout has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a timeout is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetHealthCheckSdmaTimeout(brmlDevice_t device, unsigned int timeout);

/**
 * get the health check mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The current mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetHealthCheckMode(brmlDevice_t device, unsigned int* mode);

/**
 * Set the health check mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_BUSY              \a device is doing hot reset
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetHealthCheckMode(brmlDevice_t device, unsigned int mode);

/**
 * get the waw mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The current mode
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been populated successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceGetWawMode(brmlDevice_t device, unsigned int* mode);

/**
 * Set the waw mode for the device.
 *
 * @param device                               The identifier of the target device
 * @param mode                                 The mode to be set
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a mode is invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetWawMode(brmlDevice_t device, unsigned int mode);

/** @} */

/*******************************************************************************************************************/
/** @defgroup brmlSecurityVirtualizationInstance GPU SVI Management
 * This chapter describes BRML operations that are associated with GPU Security Virtualization Instance management.
 *  @{
 */
/*******************************************************************************************************************/

/**
 * Get GPU SVI mode for the device.
 *
 * Changing SVI modes may require device unbind or reset. The "pending" SVI mode refers to the target mode following the
 * next activation trigger.
 *
 * @param device                                    The identifier of the target device
 * @param currentMode                               Returns the current mode, \ref BRML_DEVICE_SVI_MODE0,
 *                                                  \ref BRML_DEVICE_SVI_MODE1 or \ref BRML_DEVICE_SVI_MODE2
 * @param pendingMode                               Returns the pending mode, \ref BRML_DEVICE_SVI_MODE0,
 *                                                  \ref BRML_DEVICE_SVI_MODE1 or \ref BRML_DEVICE_SVI_MODE2
 *
 * @return
 *         - \ref BRML_SUCCESS                      \a currentMode and \a pendingMode have been get successfully
 *         - \ref BRML_ERROR_UNINITIALIZED          BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT       \a device is invalid, \a currentMode or \a pendingMode is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED          \a device does not support this SVI mode
 */
brmlReturn_t DECLDIR brmlDeviceGetSviMode(brmlDevice_t device, unsigned int* currentMode, unsigned int* pendingMode);

/**
 * Set GPU SVI mode for the device.
 *
 * Requires root/admin permissions.
 *
 * This mode determines whether a GPU instance can be created.
 *
 * @param device                               The target device identifier
 * @param mode                                 The mode to be set, \ref BRML_DEVICE_SVI_MODE0,
 *                                             \ref BRML_DEVICE_SVI_MODE1 or \ref BRML_DEVICE_SVI_MODE2
 * @param activationStatus                     The activationStatus status
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a mode has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid or \a activationStatus is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     \a device does not support this feature
 *         - \ref BRML_ERROR_UNKNOWN           unknown error occurs
 */
brmlReturn_t DECLDIR brmlDeviceSetSVIMode(brmlDevice_t device, unsigned int mode, brmlReturn_t* activationStatus);

/**
 * Get GPU instances for given instance ID.
 *
 * @param device                               The identifier of the target device
 * @param id                                   The GPU instance ID
 * @param gpuInstance                          Returns GPU instance
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a id has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  If \a device, \a id or gpuInstance are invalid
 *         - \ref BRML_ERROR_NOT_SUPPORTED     If \a device doesn't have SVI mode enabled
 *         - \ref BRML_ERROR_NO_PERMISSION     If user doesn't have permission to perform the operation
 *         - \ref BRML_ERROR_NOT_FOUND         If the GPU instance is not found.
 */
brmlReturn_t DECLDIR brmlDeviceGetGpuInstanceById(brmlDevice_t device, unsigned int id, brmlDevice_t* gpuInstance);


/**
 * Get GPU instance number.
 *
 * @param device                               The target device indentifier
 * @param number                               The GPU instance number
 *
 * @return
 *         - \ref BRML_SUCCESS                 \a number has been set successfully
 *         - \ref BRML_ERROR_UNINITIALIZED     BRML has not been successfully initialized
 *         - \ref BRML_ERROR_INVALID_ARGUMENT  \a device is invalid, or \a number is NULL
 *         - \ref BRML_ERROR_NOT_SUPPORTED     the device does not support this feature
 */
brmlReturn_t DECLDIR brmlDeviceGetInstanceNumber(brmlDevice_t device, unsigned int* number);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
