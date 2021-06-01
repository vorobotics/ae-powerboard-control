#ifndef UTILS_HPP
#define UTILS_HPP

#include "ros/ros.h"

class Utils
{
    public:
        enum NumberIq
        {
            I4Q8 = 0x0408,
        };

        static float ConvertFixedToFloat(uint32_t number, NumberIq iq, uint8_t decimalPlaces)
        {
            uint8_t q_part = iq & 0xff;
            uint8_t i_part = (iq >> 8) & 0xff;
            uint32_t mask = 0x01 << (i_part + q_part - 1);
            int32_t sign_value = 0;
            float result = 0.0;

            if((number & mask) == 0)
            {
                sign_value = number;
            }
            else
            {
                sign_value = number | (~(mask - 1));
            }
            result = (float)(sign_value);
            mask = 0x01 << q_part;
            result /= (float)(mask);

            result *= pow(10, decimalPlaces);
            result = roundf(result);
            result /= pow(10, decimalPlaces);

            return result;
        }
};

#endif //UTILS_HPP