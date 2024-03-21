/* by honey the codewitch

Originally derived from M5 Stack Tough Lib code

Original license follows:

MIT License

Copyright (c) 2020 M5Stack

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
namespace arduino {
#else
#include <esp_timer.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
namespace esp_idf {
#endif

template <uint16_t Width, uint16_t Height, int8_t IntPin, uint8_t Interval = 13, uint8_t Address = 0x2E>
class chsc6540 final {
    
#ifdef ARDUINO
    TwoWire& m_i2c;
#else
    constexpr static const uint8_t ACK_CHECK_EN = 0x1;
    constexpr static const uint8_t ACK_CHECK_DIS = 0x0;
    constexpr static const uint8_t ACK_VAL = 0x0;
    constexpr static const uint8_t NACK_VAL = 0x1;
    i2c_port_t m_i2c;
#endif
    uint8_t m_rotation;
    bool m_initialized;
    size_t m_touches;
    uint32_t m_read_ts;
    uint16_t m_touches_x[2], m_touches_y[2], m_touches_id[2];

    chsc6540(const chsc6540& rhs) = delete;
    chsc6540& operator=(const chsc6540& rhs) = delete;
    void do_move(chsc6540& rhs) {
        m_i2c = rhs.m_i2c;
        m_rotation = rhs.m_rotation;
        m_initialized = rhs.m_initialized;
        m_touches = rhs.m_touches;
        m_read_ts = rhs.m_read_ts;
        memcpy(m_touches_x, rhs.m_touches_x, sizeof(m_touches_x));
        memcpy(m_touches_y, rhs.m_touches_y, sizeof(m_touches_y));
        memcpy(m_touches_id, rhs.m_touches_id, sizeof(m_touches_id));
    }
    bool pressed() const {
        if(!m_initialized) {
            return false;
        }
#if ARDUINO
        return digitalRead(interrupt_pin)==LOW;
#else
        return gpio_get_level((gpio_num_t)interrupt_pin)==0;
#endif
    }
    int reg(int r) const {
#ifdef ARDUINO
        int result = 0;
        m_i2c.beginTransmission(address);
        m_i2c.write(r);
        m_i2c.endTransmission();
        m_i2c.requestFrom((uint8_t)address, (uint8_t)1);
        if (m_i2c.available()) {
            result = m_i2c.read();
        }
        return result;
#else
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, r, I2C_MASTER_ACK);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        uint8_t result;
        i2c_master_read_byte(cmd, &result, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        return result;
#endif
    }
    size_t regs(int r, uint8_t* out_data, size_t data_size) const {
#ifdef ARDUINO
        size_t result = 0;
        m_i2c.beginTransmission(address);
        m_i2c.write(r);
        m_i2c.endTransmission();
        m_i2c.requestFrom((uint8_t)address, (uint8_t)data_size);
        if (m_i2c.available()) {
            result = m_i2c.readBytes(out_data,data_size);
        }
        return result;
#else
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, r, I2C_MASTER_ACK);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        size_t result = data_size;
        i2c_master_read(cmd, out_data, size, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        return result;
#endif
    }
    void reg(int r, int value) {
#ifdef ARDUINO
        m_i2c.beginTransmission(address);
        m_i2c.write((uint8_t)r);
        m_i2c.write((uint8_t)value);
        m_i2c.endTransmission();
#else
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, r, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
#endif
    }
    bool read_all() {
       
        // Return immediately if read() is called more frequently than the
        // touch sensor updates. This prevents unnecessary I2C reads, and the
        // data can also get corrupted if reads are too close together.
        if (millis() - m_read_ts < interval) return false;
        m_read_ts = millis();
       uint8_t pts = 0;
        uint8_t p0f = 0;

        if (pressed())
        {
            uint8_t data[11];
            regs(0x02,data,sizeof(data));

            uint8_t pts = data[0];
            if (pts > 2) return false;
            if (pts) {
                // Read the data. Never mind trying to read the "weight" and
                // "size" properties or using the built-in gestures: they
                // are always set to zero.
                p0f = (data[3] >> 4) ? 1 : 0;
                m_touches_x[0] = ((data[1] << 8) | data[2]) & 0x0fff;
                m_touches_y[0] = ((data[3] << 8) | data[4]) & 0x0fff;
                if (m_touches_x[0] >= native_width || m_touches_y[0] >= native_height) return false;
                if (pts == 2) {
                    m_touches_x[1] = ((data[7] << 8) | data[8]) & 0x0fff;
                    m_touches_y[1] = ((data[9] << 8) | data[10]) & 0x0fff;
                    if (m_touches_x[1] >= native_width || m_touches_y[1] >= native_height) return false;
                }
                if(p0f) {
                    uint8_t tmp = m_touches_x[0];
                    m_touches_x[0]=m_touches_x[1];
                    m_touches_x[1]=tmp;
                    tmp = m_touches_y[0];
                    m_touches_y[0]=m_touches_y[1];
                    m_touches_y[1]=tmp;
                }
                m_touches = pts;
                return true;
            } else {
                return false;
            }
        } 
        m_touches = 0;
        return true;
    }
    
    void translate(uint16_t& x, uint16_t& y) const {
        uint16_t tmp;
        switch (m_rotation & 3) {
            case 1:
                tmp = x;
                x = y;
                y = native_width - tmp - 1;
                break;
            case 2:
                x = native_width - x - 1;
                y = native_height - y - 1;
                break;
            case 3:
                tmp = x;
                x = native_height - y - 1;
                y = tmp;
            default:
                break;
        }
    }

   public:
    constexpr static const uint16_t native_width = Width;
    constexpr static const uint16_t native_height = Height;
    constexpr static const uint8_t interval = Interval;
    constexpr static const uint8_t interrupt_pin = IntPin;
    constexpr static const uint8_t address = Address;
    chsc6540(chsc6540&& rhs) {
        do_move(rhs);
    }
    chsc6540& operator=(chsc6540&& rhs) {
        do_move(rhs);
        return *this;
    }
    chsc6540(
#ifdef ARDUINO
        TwoWire& i2c = Wire
#else
        i2c_port_t i2c = I2C_NUM_0
#endif
        ) : m_i2c(i2c), m_rotation(0), m_touches(0) {
    }

    bool initialized() const {
        return m_initialized;
    }
    bool initialize() {
        if (!m_initialized) {
#ifdef ARDUINO
            pinMode(interrupt_pin, INPUT);
#else
            gpio_config_t io_conf;
            memset(&io_conf,0,sizeof(io_conf));
            //disable interrupt
            io_conf.intr_type = GPIO_INTR_DISABLE;
            //set as output mode
            io_conf.mode = GPIO_MODE_OUTPUT;
            //bit mask of the pins that you want to set,e.g.GPIO18/19
            io_conf.pin_bit_mask = 1U<<interrupt_pin;
            //disable pull-down mode
            io_conf.pull_down_en = 0;
            //disable pull-up mode
            io_conf.pull_up_en = 0;
            //configure GPIO with the given settings
            gpio_config(&io_conf);   
#endif
            
#ifdef ARDUINO
            m_i2c.begin();
#endif
            reg(0x5A,0x5A);
            m_read_ts = 0;
            m_touches = 0;
            m_initialized = true;
        }
        return m_initialized;
    }
    uint8_t rotation() const {
        return m_rotation;
    }
    void rotation(uint8_t value) {
        m_rotation = value & 3;
    }
    uint16_t width() const {
        return m_rotation & 1 ? native_height : native_width;
    }
    uint16_t height() const {
        return m_rotation & 1 ? native_width : native_height;
    }
    size_t touches() const {
        if(!m_initialized) {
            return 0;
        }
        return m_touches;
    }
    bool xy(uint16_t* out_x, uint16_t* out_y) const {
        if(!m_initialized) {
            return false;
        }
        if(m_touches<1) {
            return false;
        }
        *out_x = m_touches_x[0];
        *out_y = m_touches_y[0];
        translate(*out_x,*out_y);
        return true;
    }
    bool xy2(uint16_t* out_x, uint16_t* out_y) const {
        if(!m_initialized) {
            return false;
        }
        if(m_touches<2) {
            return false;
        }
        *out_x = m_touches_x[1];
        *out_y = m_touches_y[1];
        translate(*out_x,*out_y);
        return true;
    }
    bool update() {
        if (!initialize()) {
            return false;
        }
        return read_all();
    }
};
}  // namespace arduino