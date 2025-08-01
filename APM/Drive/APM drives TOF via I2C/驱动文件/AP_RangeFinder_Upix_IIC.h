#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_UPIX_IIC_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

#define AP_RANGE_FINDER_UPIX_IIC_DEFAULT_ADDR   0x52
#define AP_RANGE_FINDER_UPIX_IIC_COMMAND_TAKE_RANGE_READING 0x00

class AP_RangeFinder_Upix_IIC : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> iic_dev);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // constructor
    AP_RangeFinder_Upix_IIC(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> iic_dev);

    bool _init(void);
    void _timer(void);

    uint16_t distance;
    bool new_distance;

    // start a reading
    bool start_reading(void);    
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
};

#endif  // AP_RANGEFINDER_UPIX_IIC_ENABLED
