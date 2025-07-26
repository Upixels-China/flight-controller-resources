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


#include "AP_RangeFinder_Upix_IIC.h"

#if AP_RANGEFINDER_UPIX_IIC_ENABLED

#include <utility>

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_Upix_IIC::AP_RangeFinder_Upix_IIC(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> iic_dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(iic_dev)) 
{
}


AP_RangeFinder_Backend *AP_RangeFinder_Upix_IIC::detect(RangeFinder::RangeFinder_State &_state,
																AP_RangeFinder_Params &_params,
                                                                AP_HAL::OwnPtr<AP_HAL::I2CDevice> iic_dev)
{
    if (!iic_dev) {
        return nullptr;
    }

    AP_RangeFinder_Upix_IIC *sensor
        = NEW_NOTHROW AP_RangeFinder_Upix_IIC(_state, _params, std::move(iic_dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_Upix_IIC::_init(void)
{
    dev->get_semaphore()->take_blocking();

    if (!start_reading()) {
        dev->get_semaphore()->give();
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(100);

    uint16_t reading_cm;
    if (!get_reading(reading_cm)) {
        dev->get_semaphore()->give();
        return false;
    }

    dev->get_semaphore()->give();

    dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Upix_IIC::_timer, void));

    return true;
}


// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_Upix_IIC::start_reading()
{
    uint8_t cmd = AP_RANGE_FINDER_UPIX_IIC_COMMAND_TAKE_RANGE_READING;

    // send command to take reading
    return dev->transfer(&cmd, sizeof(cmd), nullptr, 0);
}

// read - return last value measured by sensor
bool AP_RangeFinder_Upix_IIC::get_reading(uint16_t &reading_cm)
{    
    uint8_t val[6];

    // trigger a new reading
    start_reading();

    // take range reading and read back results
    bool ret = dev->transfer(nullptr, 0, (uint8_t *) &val, sizeof(val));

    if(ret)
    {
        reading_cm = (val[1] << 8) | val[2];
    }

    return ret;
}

/*
  timer called at 50Hz
*/
void AP_RangeFinder_Upix_IIC::_timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        WITH_SEMAPHORE(_sem);
        distance = d;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_Upix_IIC::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_m = distance * 0.001f;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_UPIX_IIC_ENABLED
