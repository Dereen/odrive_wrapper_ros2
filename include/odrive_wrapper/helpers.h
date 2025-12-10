/**
* @file helpers.h
* @author Anna Zigajkova (zigajkova@jettyvision.cz)
* @brief Helper functions for odrive CAN interface
* @version 0.1
* @date 2023-05-04
*
# @copyright (c) JettyVision s.r.o in Prague 2023 - All Rights Reserved
*
*/

#ifndef ODRIVE_WRAPPER_HELPERS_H
#define ODRIVE_WRAPPER_HELPERS_H

#include <unordered_map>
#include <cstring>
#include <algorithm>
#include <stdexcept>
#include <linux/can.h>

namespace odrive_wrapper
{
    /**
     * @brief Function to check if the key is present or not using count()
     *
     * @param[in] unordered map in whith the key is searched for
     * @param[in] key the key, which presence is checked for
     */
    bool inline key_present(const std::unordered_map<int, int>& m, int key)
    {
        return m.count(key) != 0;
    }

    /**
     * @brief Extracts a 32-bit unsigned integer from an 8-bit array
     *
     * @param data Pointer to the data array (must have at least startIdx+4 bytes available)
     * @param dataSize Size of the data array in bytes
     * @param startIdx Starting index in the array (must be <= dataSize-4)
     * @param lsb True for LSB first (little-endian), false for MSB first (big-endian)
     * @return uint32_t The extracted 32-bit value
     * @throws std::out_of_range if startIdx+3 >= dataSize
     */
    uint32_t inline get32from8(const uint8_t *data, size_t dataSize, size_t startIdx, bool lsb = true)
    {
        // Bounds checking to prevent buffer overflow
        if (startIdx + 4 > dataSize)
        {
            throw std::out_of_range("get32from8: startIdx + 4 exceeds data buffer size");
        }

        if (!lsb)
            return data[startIdx] | (data[startIdx + 1] << 8) | (data[startIdx + 2] << 16) | (data[startIdx + 3] << 24);
        else
            return data[startIdx + 3] | (data[startIdx + 2] << 8) | (data[startIdx + 1] << 16) | (data[startIdx] << 24);
    }

    float inline get_float(uint32_t f)
    {
        static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
        float ret;
        std::memcpy(&ret, &f, sizeof(float));

        return ret;
    }
    /** See https://stackoverflow.com/questions/1659440/32-bit-to-16-bit-floating-point-conversion*/
    /** Fixed: Using memcpy instead of type-punning to avoid undefined behavior */
    float inline as_float(const uint x)
    {
        static_assert(sizeof(float) == sizeof(uint), "float and uint must have same size");
        float ret;
        std::memcpy(&ret, &x, sizeof(float));
        return ret;
    }

    uint inline as_uint(const float x)
    {
        static_assert(sizeof(float) == sizeof(uint), "float and uint must have same size");
        uint ret;
        std::memcpy(&ret, &x, sizeof(uint));
        return ret;
    }

    ushort inline float_to_half(const float x)
    {                                                                                                                                                                                       // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
        const uint b = as_uint(x) + 0x00001000;                                                                                                                                             // round-to-nearest-even: add last bit after truncated mantissa
        const uint e = (b & 0x7F800000) >> 23;                                                                                                                                              // exponent
        const uint m = b & 0x007FFFFF;                                                                                                                                                      // mantissa; in line below: 0x007FF000 = 0x00800000-0x00001000 = decimal indicator flag - initial rounding
        return (b & 0x80000000) >> 16 | (e > 112) * ((((e - 112) << 10) & 0x7C00) | m >> 13) | ((e < 113) & (e > 101)) * ((((0x007FF000 + m) >> (125 - e)) + 1) >> 1) | (e > 143) * 0x7FFF; // sign : normalized : denormalized : saturate
    }

    float inline half_to_float(const ushort x)
    {                                                                                                                                                        // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
        const uint e = (x & 0x7C00) >> 10;                                                                                                                   // exponent
        const uint m = (x & 0x03FF) << 13;                                                                                                                   // mantissa
        const uint v = as_uint((float)m) >> 23;                                                                                                              // evil log2 bit hack to count leading zeros in denormalized format
        return as_float((x & 0x8000) << 16 | (e != 0) * ((e + 112) << 23 | m) | ((e == 0) & (m != 0)) * ((v - 37) << 23 | ((m << (150 - v)) & 0x007FE000))); // sign : normalized : denormalized
    }

    template <typename T>
    void inline get_char_from_num(char *arr, const T var, const bool lsb)
    {
        memcpy(arr, &var, sizeof(T));

        if (lsb)
            std::reverse(arr, arr + sizeof(T));
    }

    template <typename T>
    void inline get_char_from_nums(char *arr, const T var1, const T var2, const bool lsb)
    {
        get_char_from_num<T>(arr, var1, lsb);
        get_char_from_num<T>(arr + sizeof(T), var2, lsb);
    }

    template <typename T, typename F>
    void inline get_char_from_nums(char *arr, const T var1, const F var2, const F var3, const bool lsb)
    {
        get_char_from_num<T>(arr, var1, lsb);
        get_char_from_num<F>(arr + sizeof(T), var2, lsb);
        get_char_from_num<F>(arr + sizeof(T) + sizeof(F), var3, lsb);
    }

    template <typename T>
    void inline get_char_from_nums(char *arr, const T var1, const T var2, const T var3, const bool lsb)
    {
        get_char_from_num<T>(arr, var1, lsb);
        get_char_from_num<T>(arr + sizeof(T), var2, lsb);
        get_char_from_num<T>(arr + 2 * sizeof(T), var3, lsb);
    }

    void inline get_char_for_short_float(char *arr, const float in, const bool lsb)
    {
        get_char_from_num<ushort>(arr, float_to_half(in), lsb);
    }

    bool inline check_msg_error(uint32_t header)
    {
        return CAN_ERR_FLAG & header;
    }

    template <typename T, typename... Args>
    std::unique_ptr<T> inline make_unique(Args &&...args)
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}
#endif // ODRIVE_WRAPPER_HELPERS_H
