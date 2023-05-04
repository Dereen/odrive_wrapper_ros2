 /*
 * @file helpers.h
 * @author Anna Zigajkova (zigajkova@jettyvision.cz)
 * @brief Helper functions for odrive CAN interface
 * @version 0.1
 * @date 2023-05-04
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <unordered_map>
#include <cstring>
#include <linux/can.h>


    /**
     * @brief Function to check if the key is present or not using count()
     *
     * @param[in] unordered map in whith the key is searched for
     * @param[in] key the key, which presence is checked for
     */
bool key_present(std::unordered_map<int, int> m, int key)
{
    // Key is not present
    if (m.count(key) == 0)
        return false;

    return true;
}

uint32_t get32from8(uint8_t *data, int startIdx, bool lsb=true)
{
    if (!lsb)
        return data[startIdx] | data[startIdx + 1] << 8 | data[startIdx + 2] << 16 | data[startIdx + 3] << 24;
    else
        return data[startIdx + 3] | data[startIdx + 2] << 8 | data[startIdx + 1] << 16 | data[startIdx] << 24;
}


float get_float(uint32_t f)
{
    static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
    float ret;
    std::memcpy(&ret, &f, sizeof(float));

    return ret;
}

template <typename T>
void get_char_from_num(char *arr, T var, bool lsb)
{
    memcpy(arr, &var, sizeof(T));

    if (lsb)
        std::reverse(arr, arr + sizeof(T));
}

template <typename T>
void get_char_from_nums(char *arr, T var1, T var2, bool lsb)
{
    get_char_from_num<T>(arr, var1, lsb);
    get_char_from_num<T>(arr + sizeof(T), var2, lsb);
}

template <typename T, typename F>
void get_char_from_nums(char *arr, T var1, F var2, F var3, bool lsb)
{
    get_char_from_num<T>(arr, var1, lsb);
    get_char_from_num<F>(arr + sizeof(T), var2, lsb);
    get_char_from_num<F>(arr + sizeof(T) + sizeof(F), var3, lsb);

}

template <typename T>
void get_char_from_nums(char *arr, T var1, T var2, T var3, bool lsb)
{
    get_char_from_num<T>(arr, var1, lsb);
    get_char_from_num<T>(arr + sizeof(T), var2, lsb);
    get_char_from_num<T>(arr + 2 * sizeof(T), var3, lsb);

}

bool check_msg_error(uint32_t header)
{
    return CAN_ERR_FLAG & header;
}

