/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder.h"

#if AP_RANGEFINDER_ENABLED

#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_MaxsonarSerialLV.h"
#include "AP_RangeFinder_BBB_PRU.h"
#include "AP_RangeFinder_LightWareI2C.h"
#include "AP_RangeFinder_LightWareSerial.h"
#if AP_RANGEFINDER_BEBOP_ENABLED
#include "AP_RangeFinder_Bebop.h"
#endif
#include "AP_RangeFinder_Backend.h"
#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder_MAVLink.h"
#include "AP_RangeFinder_LeddarOne.h"
#include "AP_RangeFinder_USD1_Serial.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_TeraRanger_Serial.h"
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_VL53L1X.h"
#include "AP_RangeFinder_NMEA.h"
#include "AP_RangeFinder_Wasp.h"
#include "AP_RangeFinder_Benewake_TF02.h"
#include "AP_RangeFinder_Benewake_TF03.h"
#include "AP_RangeFinder_Benewake_TFMini.h"
#include "AP_RangeFinder_Benewake_TFMiniPlus.h"
#include "AP_RangeFinder_PWM.h"
#include "AP_RangeFinder_GYUS42v2.h"
#include "AP_RangeFinder_HC_SR04.h"
#include "AP_RangeFinder_Bebop.h"
#include "AP_RangeFinder_BLPing.h"
#include "AP_RangeFinder_DroneCAN.h"
#include "AP_RangeFinder_Lanbao.h"
#include "AP_RangeFinder_LeddarVu8.h"
#include "AP_RangeFinder_SITL.h"
#include "AP_RangeFinder_MSP.h"
#include "AP_RangeFinder_USD1_CAN.h"
#include "AP_RangeFinder_Benewake_CAN.h"
#include "AP_RangeFinder_Lua.h"
#include "AP_RangeFinder_NoopLoop.h"
#include "AP_RangeFinder_TOFSenseP_CAN.h"
#include "AP_RangeFinder_NRA24_CAN.h"
#include "AP_RangeFinder_TOFSenseF_I2C.h"
#include "AP_RangeFinder_JRE_Serial.h"
#include "AP_RangeFinder_Ainstein_LR_D1.h"
#include "AP_RangeFinder_RDS02UF.h"
#include "AP_RangeFinder_Upix_IIC.h"
#include "AP_RangeFinder_TS1224.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_InternalError/AP_InternalError.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {

	// @Group: 1_
	// @Path: AP_RangeFinder_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, RangeFinder, AP_RangeFinder_Params),

    // @Group: 1_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, RangeFinder, AP_RangeFinder_Params),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, RangeFinder, AP_RangeFinder_Params),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, RangeFinder, AP_RangeFinder_Params),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_",  60, RangeFinder, backend_var_info[3]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 33, RangeFinder, AP_RangeFinder_Params),

    // @Group: 5_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_",  34, RangeFinder, backend_var_info[4]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 35, RangeFinder, AP_RangeFinder_Params),

    // @Group: 6_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_",  36, RangeFinder, backend_var_info[5]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 37, RangeFinder, AP_RangeFinder_Params),

    // @Group: 7_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_",  38, RangeFinder, backend_var_info[6]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 39, RangeFinder, AP_RangeFinder_Params),

    // @Group: 8_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_",  40, RangeFinder, backend_var_info[7]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[8], "9_", 41, RangeFinder, AP_RangeFinder_Params),

    // @Group: 9_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_",  42, RangeFinder, backend_var_info[8]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 9
    // @Group: A_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[9], "A_", 43, RangeFinder, AP_RangeFinder_Params),

    // @Group: A_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Backend_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[9], "A_",  44, RangeFinder, backend_var_info[9]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *RangeFinder::backend_var_info[RANGEFINDER_MAX_INSTANCES];

RangeFinder::RangeFinder()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void RangeFinder::convert_params(void)
{
    // PARAMETER_CONVERSION - Added: Dec-2024 for 4.6->4.7
    for (auto &p : params) {
        p.convert_min_max_params();
    }
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
__INITFUNC__ void RangeFinder::init(enum Rotation orientation_default)
{
    convert_params();

    if (num_instances != 0) {
        // don't re-init if we've found some sensors already
        return;
    }

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        params[i].orientation.set_default(orientation_default);
    }

    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN rangefinder may already have been
            // found
            num_instances = MAX(num_instances, i+1);
        }

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
        // initialize signal_quality_pct for drivers that don't handle it.
        state[i].signal_quality_pct = SIGNAL_QUALITY_UNKNOWN;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
        }
    }
#if HAL_LOGGING_ENABLED
    Log_RFND();
#endif
}

__INITFUNC__ bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);

    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
__INITFUNC__ void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    AP_RangeFinder_Backend_Serial *(*serial_create_fn)(RangeFinder::RangeFinder_State&, AP_RangeFinder_Params&) = nullptr;

    const Type _type = (Type)params[instance].type.get();
    switch (_type) {
#if AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
    case Type::PLI2C:
    case Type::PLI2CV3:
    case Type::PLI2CV3HP:
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], params[instance], _type),
                             instance)) {
                break;
            }
        }
        break;
#endif
#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
    case Type::MBI2C: {
        uint8_t addr = AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR;
        if (params[instance].address != 0) {
            addr = params[instance].address;
        }
        FOREACH_I2C(i) {
            auto *device_ptr = hal.i2c_mgr->get_device_ptr(i, addr);
            if (_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance], params[instance],
                                                                  device_ptr),
                             instance)) {
                break;
            }
            delete device_ptr;
        }
        break;
    }
#endif
#if AP_RANGEFINDER_UPIX_IIC_ENABLED
    case Type::UPIX: {
            uint8_t addr = AP_RANGE_FINDER_UPIX_IIC_DEFAULT_ADDR;
            if(params[instance].address != 0){
                addr = params[instance].address;
            }
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_Upix_IIC::detect(state[instance], params[instance],
                                                                 hal.i2c_mgr->get_device(i, addr)),
                                 instance)) {
                    break;
                }
            }
        break;
    }
#endif
#if AP_RANGEFINDER_TS1224_ENABLED
    case Type::TS1224:
        serial_create_fn = AP_RangeFinder_TS1224::create;
        break;
#endif
#if AP_RANGEFINDER_LWI2C_ENABLED
    case Type::LWI2C:
        if (params[instance].address) {
            // the LW20 needs a long time to boot up, so we delay 1.5s here
#ifndef HAL_BUILD_AP_PERIPH
            if (!hal.util->was_watchdog_armed()) {
                hal.scheduler->delay(1500);
            }
#endif
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                             hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, params[instance].address)),
                                                             instance);
#else
            FOREACH_I2C(i) {
                if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                                     hal.i2c_mgr->get_device(i, params[instance].address)),
                                 instance)) {
                    break;
                }
            }
#endif
        }
        break;
#endif  // AP_RANGEFINDER_LWI2C_ENABLED
#if AP_RANGEFINDER_TRI2C_ENABLED
    case Type::TRI2C:
        if (params[instance].address) {
            FOREACH_I2C(i) {
                auto *device_ptr = hal.i2c_mgr->get_device_ptr(i, params[instance].address);
                if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                      device_ptr),
                                 instance)) {
                    break;
                }
                delete device_ptr;
            }
        }
        break;
#endif
    case Type::VL53L0X:
    case Type::VL53L1X_Short:
            FOREACH_I2C(i) {
#if AP_RANGEFINDER_VL53L0X_ENABLED
                if (_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address)),
                        instance)) {
                    break;
                }
#endif
#if AP_RANGEFINDER_VL53L1X_ENABLED
                if (_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                                hal.i2c_mgr->get_device(i, params[instance].address),
                                                                _type == Type::VL53L1X_Short ?  AP_RangeFinder_VL53L1X::DistanceMode::Short :
                                                                AP_RangeFinder_VL53L1X::DistanceMode::Long),
                                 instance)) {
                    break;
                }
#endif
            }
        break;
#if AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED
    case Type::BenewakeTFminiPlus: {
        uint8_t addr = TFMINIPLUS_ADDR_DEFAULT;
        if (params[instance].address != 0) {
            addr = params[instance].address;
        }
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_Benewake_TFMiniPlus::detect(state[instance], params[instance],
                                                                        hal.i2c_mgr->get_device(i, addr)),
                    instance)) {
                break;
            }
        }
        break;
    }
#endif
#if AP_RANGEFINDER_PWM_ENABLED
    case Type::PX4_PWM:
        // to ease moving from PX4 to ChibiOS we'll lie a little about
        // the backend driver...
        if (AP_RangeFinder_PWM::detect()) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
        break;
#endif
#if AP_RANGEFINDER_BBB_PRU_ENABLED
    case Type::BBB_PRU:
        if (AP_RangeFinder_BBB_PRU::detect()) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_BBB_PRU(state[instance], params[instance]), instance);
        }
        break;
#endif
#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
    case Type::LWSER:
        serial_create_fn = AP_RangeFinder_LightWareSerial::create;
        break;
#endif
#if AP_RANGEFINDER_LEDDARONE_ENABLED
    case Type::LEDDARONE:
        serial_create_fn = AP_RangeFinder_LeddarOne::create;
        break;
#endif
#if AP_RANGEFINDER_USD1_SERIAL_ENABLED
    case Type::USD1_Serial:
        serial_create_fn = AP_RangeFinder_USD1_Serial::create;
        break;
#endif
#if AP_RANGEFINDER_BEBOP_ENABLED
    case Type::BEBOP:
        if (AP_RangeFinder_Bebop::detect()) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_Bebop(state[instance], params[instance]), instance);
        }
        break;
#endif
#if AP_RANGEFINDER_MAVLINK_ENABLED
    case Type::MAVLink:
        if (AP_RangeFinder_MAVLink::detect()) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_MAVLink(state[instance], params[instance]), instance);
        }
        break;
#endif
#if AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
    case Type::MBSER:
        serial_create_fn = AP_RangeFinder_MaxsonarSerialLV::create;
        break;
#endif
#if AP_RANGEFINDER_ANALOG_ENABLED
    case Type::ANALOG:
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(params[instance])) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_analog(state[instance], params[instance]), instance);
        }
        break;
#endif
#if AP_RANGEFINDER_HC_SR04_ENABLED
    case Type::HC_SR04:
        // note that this will always come back as present if the pin is valid
        if (AP_RangeFinder_HC_SR04::detect(params[instance])) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_HC_SR04(state[instance], params[instance]), instance);
        }
        break;
#endif
#if AP_RANGEFINDER_NMEA_ENABLED
    case Type::NMEA:
        serial_create_fn = AP_RangeFinder_NMEA::create;
        break;
#endif
#if AP_RANGEFINDER_WASP_ENABLED
    case Type::WASP:
        serial_create_fn = AP_RangeFinder_Wasp::create;
        break;
#endif
#if AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
    case Type::BenewakeTF02:
        serial_create_fn = AP_RangeFinder_Benewake_TF02::create;
        break;
#endif
#if AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
    case Type::BenewakeTFmini:
        serial_create_fn = AP_RangeFinder_Benewake_TFMini::create;
        break;
#endif
#if AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
    case Type::BenewakeTF03:
        serial_create_fn = AP_RangeFinder_Benewake_TF03::create;
        break;
#endif
#if AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
    case Type::TeraRanger_Serial:
        serial_create_fn = AP_RangeFinder_TeraRanger_Serial::create;
        break;
#endif
#if AP_RANGEFINDER_PWM_ENABLED
    case Type::PWM:
        if (AP_RangeFinder_PWM::detect()) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
        break;
#endif
#if AP_RANGEFINDER_BLPING_ENABLED
    case Type::BLPing:
        serial_create_fn = AP_RangeFinder_BLPing::create;
        break;
#endif
#if AP_RANGEFINDER_LANBAO_ENABLED
    case Type::Lanbao:
        serial_create_fn = AP_RangeFinder_Lanbao::create;
        break;
#endif
#if AP_RANGEFINDER_LEDDARVU8_ENABLED
    case Type::LeddarVu8_Serial:
        serial_create_fn = AP_RangeFinder_LeddarVu8::create;
        break;
#endif

#if AP_RANGEFINDER_DRONECAN_ENABLED
    case Type::UAVCAN:
        /*
          the UAVCAN driver gets created when we first receive a
          measurement. We take the instance slot now, even if we don't
          yet have the driver
         */
        num_instances = MAX(num_instances, instance+1);
        break;
#endif

#if AP_RANGEFINDER_GYUS42V2_ENABLED
    case Type::GYUS42v2:
        serial_create_fn = AP_RangeFinder_GYUS42v2::create;
        break;
#endif

#if AP_RANGEFINDER_SIM_ENABLED
    case Type::SIM:
        _add_backend(NEW_NOTHROW AP_RangeFinder_SITL(state[instance], params[instance], instance), instance);
        break;
#endif

#if HAL_MSP_RANGEFINDER_ENABLED
    case Type::MSP:
        if (AP_RangeFinder_MSP::detect()) {
            _add_backend(NEW_NOTHROW AP_RangeFinder_MSP(state[instance], params[instance]), instance);
        }
        break;
#endif // HAL_MSP_RANGEFINDER_ENABLED

#if AP_RANGEFINDER_USD1_CAN_ENABLED
    case Type::USD1_CAN:
        _add_backend(NEW_NOTHROW AP_RangeFinder_USD1_CAN(state[instance], params[instance]), instance);
        break;
#endif
#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
    case Type::Benewake_CAN:
        _add_backend(NEW_NOTHROW AP_RangeFinder_Benewake_CAN(state[instance], params[instance]), instance);
        break;
#endif

#if AP_RANGEFINDER_LUA_ENABLED
    case Type::Lua_Scripting:
        _add_backend(NEW_NOTHROW AP_RangeFinder_Lua(state[instance], params[instance]), instance);
        break;
#endif

#if AP_RANGEFINDER_NOOPLOOP_ENABLED
    case Type::NoopLoop_P:
        serial_create_fn = AP_RangeFinder_NoopLoop::create;
        break;
#endif

#if AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
    case Type::Ainstein_LR_D1:
        serial_create_fn = AP_RangeFinder_Ainstein_LR_D1::create;
        break;
#endif

#if AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED
    case Type::TOFSenseP_CAN:
        _add_backend(NEW_NOTHROW AP_RangeFinder_TOFSenseP_CAN(state[instance], params[instance]), instance);
        break;
#endif
#if AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
  #if AP_RANGEFINDER_NRA24_CAN_ENABLED
    case Type::NRA24_CAN:
  #endif
  #if AP_RANGEFINDER_HEXSOONRADAR_ENABLED
    case Type::HEXSOON_RADAR:
  #endif
        _add_backend(NEW_NOTHROW AP_RangeFinder_NRA24_CAN(state[instance], params[instance]), instance);
        break;
#endif  // AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
#if AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
    case Type::TOFSenseF_I2C: {
        uint8_t addr = TOFSENSEP_I2C_DEFAULT_ADDR;
        if (params[instance].address != 0) {
            addr = params[instance].address;
        }
        FOREACH_I2C(i) {
            if (_add_backend(AP_RangeFinder_TOFSenseF_I2C::detect(state[instance], params[instance],
                                                                  hal.i2c_mgr->get_device(i, addr)),
                             instance)) {
                break;
            }
        }
        break;
    }
#endif
#if AP_RANGEFINDER_JRE_SERIAL_ENABLED
    case Type::JRE_Serial:
        serial_create_fn = AP_RangeFinder_JRE_Serial::create;
        break;
#endif

#if AP_RANGEFINDER_RDS02UF_ENABLED
    case Type::RDS02UF:
        serial_create_fn = AP_RangeFinder_RDS02UF::create;
        break;
#endif
    case Type::NONE:
        break;
    }

    if (serial_create_fn != nullptr) {
        if (AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance)) {
            auto *b = serial_create_fn(state[instance], params[instance]);
            if (b != nullptr) {
                _add_backend(b, instance, serial_instance++);
            }
        }
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return Status::NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(const mavlink_message_t &msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && ((Type)params[i].type.get() != Type::NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

#if HAL_MSP_RANGEFINDER_ENABLED
void RangeFinder::handle_msp(const MSP::msp_rangefinder_data_message_t &pkt)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && ((Type)params[i].type.get() == Type::MSP)) {
          drivers[i]->handle_msp(pkt);
        }
    }
}
#endif // HAL_MSP_RANGEFINDER_ENABLED

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    if ((find_instance(orientation) != nullptr)) {
        // we have a rangefinder with this orientation
        return true;
    }

    // special case for DroneCAN
    // DroneCAN rangefinder backend is not created until we receive a
    // measurement, so we need to check the params directly
#if AP_RANGEFINDER_DRONECAN_ENABLED
     for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if ((RangeFinder::Type)params[i].type.get() == RangeFinder::Type::UAVCAN) {
            if (params[i].orientation.get() == orientation) {
                return true;
            }
        }
    }
#endif

    // no rangefinder with this orientation
    return false;
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    // first try for a rangefinder that is in range
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation &&
            backend->status() == Status::Good) {
            return backend;
        }
    }
    // if none in range then return first with correct orientation
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation) {
            return backend;
        }
    }
    return nullptr;
}

float RangeFinder::distance_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance();
}

int8_t RangeFinder::signal_quality_pct_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    }
    return backend->signal_quality_pct();
}

float RangeFinder::min_distance_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;  // consider NaN
    }
    return backend->min_distance();
}

float RangeFinder::max_distance_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;  // consider NaN
    }
    return backend->max_distance();
}

float RangeFinder::ground_clearance_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;  // consider NaN
    }
    return backend->ground_clearance();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

uint32_t RangeFinder::last_reading_ms(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

// get temperature reading in C.  returns true on success and populates temp argument
bool RangeFinder::get_temp(enum Rotation orientation, float &temp) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_temp(temp);
}

#if HAL_LOGGING_ENABLED
// Write an RFND (rangefinder) packet
void RangeFinder::Log_RFND() const
{
    if (_log_rfnd_bit == uint32_t(-1)) {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit)) {
        return;
    }

    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        const AP_RangeFinder_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_RFND pkt = {
                LOG_PACKET_HEADER_INIT(LOG_RFND_MSG),
                time_us      : AP_HAL::micros64(),
                instance     : i,
                dist         : s->distance(),
                status       : (uint8_t)s->status(),
                orient       : s->orientation(),
                quality      : s->signal_quality_pct(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}
#endif  // HAL_LOGGING_ENABLED

bool RangeFinder::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if ((Type)params[i].type.get() == Type::NONE) {
            continue;
        }

        if (drivers[i] == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: Not Detected", i + 1);
            return false;
        }

        // backend-specific checks.  This might end up drivers[i]->arming_checks(...).
        switch (drivers[i]->allocated_type()) {
#if AP_RANGEFINDER_ANALOG_ENABLED || AP_RANGEFINDER_PWM_ENABLED
#if AP_RANGEFINDER_ANALOG_ENABLED
        case Type::ANALOG:
#endif
#if AP_RANGEFINDER_PWM_ENABLED
        case Type::PX4_PWM:
        case Type::PWM:
#endif
        {
            // ensure pin is configured
            if (params[i].pin == -1) {
                hal.util->snprintf(failure_msg, failure_msg_len, "RNGFND%u_PIN not set", unsigned(i + 1));
                return false;
            }
#if AP_RANGEFINDER_ANALOG_ENABLED
            if (drivers[i]->allocated_type() == Type::ANALOG) {
                // Analog backend does not use GPIO pin
                break;
            }
#endif

            // ensure that the pin we're configured to use is available
            if (!hal.gpio->valid_pin(params[i].pin)) {
                uint8_t servo_ch;
                if (hal.gpio->pin_to_servo_channel(params[i].pin, servo_ch)) {
                    hal.util->snprintf(failure_msg, failure_msg_len, "RNGFND%u_PIN=%d, set SERVO%u_FUNCTION=-1", unsigned(i + 1), int(params[i].pin.get()), unsigned(servo_ch+1));
                } else {
                    hal.util->snprintf(failure_msg, failure_msg_len, "RNGFND%u_PIN=%d invalid", unsigned(i + 1), int(params[i].pin.get()));
                }
                return false;
            }
            break;
        }
#endif  // AP_RANGEFINDER_ANALOG_ENABLED || AP_RANGEFINDER_PWM_ENABLED

#if AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
  #if AP_RANGEFINDER_NRA24_CAN_ENABLED
        case Type::NRA24_CAN:
  #endif
  #if AP_RANGEFINDER_HEXSOONRADAR_ENABLED
        case Type::HEXSOON_RADAR:
  #endif
        {
            if (drivers[i]->status() == Status::NoData) {
                // This sensor stops sending data if there is no relative motion. This will mostly happen during takeoff, before arming
                // To avoid pre-arm failure, return true even though there is no data.
                // This sensor also sends a "heartbeat" so we can differentiate between  "NoData" and "NotConnected"
                return true;
            }
            break;
        }
#endif // AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED 

        default:
            break;
        }

        switch (drivers[i]->status()) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: Not Connected", i + 1);
            return false;
        case Status::OutOfRangeLow:
        case Status::OutOfRangeHigh:
        case Status::Good:
            break;
        }
    }

    return true;
}

RangeFinder *RangeFinder::_singleton;

namespace AP {

RangeFinder *rangefinder()
{
    return RangeFinder::get_singleton();
}

}

#endif  // AP_RANGEFINDER_ENABLED
