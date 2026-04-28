/*
 * Copyright (C) 2024 Caian Benedicto <caianbene@gmail.com>
 *
 * This file is part of xinfc.
 *
 * xinfc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * xinfc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with xinfc.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <algorithm>
#include <cstring>
#include <string>

#if defined(XINFC_DUMMY_OUT)
#include <iostream>
#include <iomanip>
#include <sstream>
#endif

namespace xinfc {

struct i2c_error
{
    const char* msg;
    int eno;
    int ret;
};

class i2c_nfc_device
{
public:

    static const unsigned int max_ndef_buf_size = 160;
    static const unsigned short base_ndef_addr = 0x01;  // NT3H2111 page 1 offset

    static_assert(max_ndef_buf_size / 16 <= I2C_RDRW_IOCTL_MAX_MSGS);

private:

    std::string _bus_path;
    unsigned short _address;
    int _fd;

public:

    i2c_nfc_device(const std::string& bus, unsigned short address) :
        _bus_path("/dev/i2c-" + bus),
        _address(address),
        _fd(
#if !defined(XINFC_DUMMY_OUT)
        open(_bus_path.c_str(), O_RDWR, 0)
#else
        -1
#endif
        )
    {
#if !defined(XINFC_DUMMY_OUT)
        if (_fd < 0)
        {
            throw i2c_error { "failed to open i2c bus", errno, _fd };
        }
#endif
    }

    void set_timeout(unsigned long timeout) const
    {
#if !defined(XINFC_DUMMY_OUT)
        if (ioctl(_fd, I2C_TIMEOUT, timeout) < 0)
        {
            throw i2c_error { "failed to set i2c timeout", errno, _fd };
        }
#endif
    }

    void set_retries(unsigned long retries) const
    {
#if !defined(XINFC_DUMMY_OUT)
        const int r = ioctl(_fd, I2C_RETRIES, retries);

        if (r < 0)
        {
            throw i2c_error { "failed to set i2c retries", errno, r };
        }
#endif
    }

    void set_device_address(unsigned char address)
    {
#if !defined(XINFC_DUMMY_OUT)
        long laddress = address;
        const int r = ioctl(_fd, I2C_SLAVE, laddress);

        if (r < 0)
        {
            throw i2c_error { "failed to set i2c device address", errno, r };
        }
#endif
    }

    void close()
    {
#if !defined(XINFC_DUMMY_OUT)
        const int r = ::close(_fd);

        if (r)
        {
            throw i2c_error { "failed to close device", errno, r};
        }
#endif

        _fd = -1;
    }

    void read_ndef(unsigned char* out_buf, unsigned int size)
    {
        if (size == 0)
            return;

        if (out_buf == nullptr)
        {
            throw i2c_error { "invalid ndef buffer", 0 };
        }

        if (size > max_ndef_buf_size)
            size = max_ndef_buf_size;

        memset(out_buf, 0, size);

        // Read in 16-byte chunks
        const unsigned int chunk_size = 16;
        for (unsigned int i = 0, offset = 0; offset < size; i++, offset += chunk_size)
        {
            unsigned int read_size = std::min(chunk_size, size - offset);
            unsigned short addr = base_ndef_addr + i;

            const unsigned int read_nmsgs = 2;
            struct i2c_msg msgs[2];
            struct i2c_rdwr_ioctl_data rdwr;

            unsigned char ndef_addr_buf[1] = {
                static_cast<unsigned char>(addr & 0xFF)
            };

            msgs[0].addr = _address;
            msgs[0].flags = 0;
            msgs[0].len = sizeof(ndef_addr_buf);
            msgs[0].buf = ndef_addr_buf;

            msgs[1].addr = _address;
            msgs[1].flags = I2C_M_RD;
            msgs[1].len = read_size;
            msgs[1].buf = out_buf + offset;

            rdwr.msgs = msgs;
            rdwr.nmsgs = read_nmsgs;

#if !defined(XINFC_DUMMY_OUT)
            const int r = ioctl(_fd, I2C_RDWR, &rdwr);

            if (r != read_nmsgs)
            {
                throw i2c_error { "failed to read from i2c device", errno, r };
            }
#else
            print_rdwr(rdwr);
#endif
        }
    }

    void write_ndef_at(
        const unsigned char* buf,
        unsigned int size,
        unsigned short ndef_off
    )
    {
        if (size == 0)
            return;

        if (buf == nullptr)
        {
            throw i2c_error { "invalid ndef buffer", 0, 0 };
        }

        if (size > max_ndef_buf_size)
        {
            throw i2c_error { "invalid ndef buffer size", 0, 0 };
        }

        // Each write operation will write 16 bytes of data
        const unsigned int chunk_size = 16;
        const unsigned int write_nmsgs = ((size - 1) / chunk_size) + 1;

        struct i2c_msg msgs[write_nmsgs];
	    struct i2c_rdwr_ioctl_data rdwr;

        // Each write operation needs 1 extra byte of addressing
        unsigned char ndef_wbuf[write_nmsgs * 17];

        for (unsigned int i = 0; i < write_nmsgs; i++)
        {
            auto curr_buf_off = chunk_size*i;
            auto curr_p_buf = buf + curr_buf_off;
            auto curr_p_wbuf = ndef_wbuf + 17*i;
            auto ndef_addr = base_ndef_addr + ndef_off + i;

            auto len = std::min(size - curr_buf_off, chunk_size);

            curr_p_wbuf[0] = static_cast<unsigned char>(ndef_addr & 0xFF);

            for (unsigned int j = 0; j < len; j++)
                curr_p_wbuf[1 + j] = curr_p_buf[j];

            for (unsigned int j = len; j < chunk_size; j++)
                curr_p_wbuf[1 + j] = 0;

            auto& msg = msgs[i];

            msg.addr = _address;
            msg.flags = 0;
            msg.len = 17;  // 1 byte address + 16 bytes data
            msg.buf = curr_p_wbuf;
        }

        rdwr.msgs = msgs;
        rdwr.nmsgs = write_nmsgs;

#if !defined(XINFC_DUMMY_OUT)
        const int r = ioctl(_fd, I2C_RDWR, &rdwr);

        if (r != (int)write_nmsgs)
        {
            throw i2c_error { "failed to write to i2c device", errno, r };
        }
#else
        print_rdwr(rdwr);
#endif
    }

    ~i2c_nfc_device()
    {
        if (_fd >= 0)
            ::close(_fd);
    }

#if defined(XINFC_DUMMY_OUT)
    void print_rdwr(const i2c_rdwr_ioctl_data& rdwr)
    {
        for (unsigned int i = 0; i < rdwr.nmsgs; i++)
        {
            const auto& msg = rdwr.msgs[i];
            const auto rd = (msg.flags & I2C_M_RD) != 0;

            std::stringstream ss;

            ss << (rd ? "read " : "write ") << msg.len
               << (rd ? " from " : " to ") << "0x" << std::hex
               << std::setfill('0') << std::setw(2) << msg.addr
               << (rd ? " to " : " from ") << std::setw(8) << (void*)msg.buf;

            if (!rd)
            {
                for (int i = 0; i < msg.len; i++)
                    ss << " 0x" << std::setw(2) << (int)msg.buf[i];
            }

            std::cerr << ss.str() << std::endl;
        }
    }
#endif
};

}
