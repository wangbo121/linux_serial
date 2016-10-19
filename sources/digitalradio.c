/*
 * digitalradio.c
 *
 *  Created on: Oct 19, 2016
 *      Author: wangbo
 */


#include <stdio.h>
#include <string.h>

/*转换int或者short的字节顺序，该程序arm平台为大端模式，地面站x86架构为小端模式*/
#include <byteswap.h>

/*open() close()所需头文件*/
#include <unistd.h>

#include "digitalradio.h"
#include "uart.h"


#define RADIO_RECV_BUF_LEN 512

#define RADIO_RECV_HEAD1  0
#define RADIO_RECV_HEAD2  1
#define RADIO_RECV_LEN  2
#define RADIO_RECV_CNT  3
#define RADIO_RECV_SYSID 4
#define RADIO_RECV_TYPE 5
#define RADIO_RECV_DATA 6
#define RADIO_RECV_CHECKSUM 7

static int radio_recv_state = 0;

int radio_uart_init()
{
    uart_device.uart_name=UART_RADIO;

    uart_device.baudrate=UART_RADIO_BAUD;
    uart_device.databits=UART_RADIO_DATABITS;
    uart_device.parity=UART_RADIO_PARITY;
    uart_device.stopbits=UART_RADIO_STOPBITS;

    uart_device.uart_num=open_uart_dev(uart_device.uart_name);

    uart_device.ptr_fun=read_radio_data;

    set_uart_opt( uart_device.uart_name, \
                  uart_device.baudrate,\
                  uart_device.databits,\
                  uart_device.parity,\
                  uart_device.stopbits);

    create_uart_pthread(&uart_device);

    return 0;
}

/*
 * 函数需要修改
 * 用到了global_bool_boatpilot变量
 */
int read_radio_data(unsigned char *buf, unsigned int len)
{
    static unsigned char _buffer[RADIO_RECV_BUF_LEN];

    static unsigned char _pack_recv_type;
    static int _pack_recv_cnt = 0;
    static int _pack_recv_len = 0;
    static unsigned char _pack_recv_buf[RADIO_RECV_BUF_LEN];
    static int _pack_buf_len = 0;
    static unsigned char _checksum = 0;

    int _length;
    int i = 0;
    unsigned char c;
    unsigned char _sysid;

    memcpy(_buffer, buf, len);

    /*显示通过电台收到的数据*/
#if 1
    printf("radio data buf=\n");
    for(i=0;i<len;i++)
    {
        printf("%c",buf[i]);
    }
    printf("\n");
#endif

    _length = len;
    for (i = 0; i<_length; i++)
    {
        c = _buffer[i];
        switch (radio_recv_state)
        {
        case RADIO_RECV_HEAD1:
            if (c == 0xaa)
            {
                radio_recv_state = RADIO_RECV_HEAD2;
                _checksum = c;
            }
            break;
        case RADIO_RECV_HEAD2:
            if (c == 0x55)
            {
                radio_recv_state = RADIO_RECV_LEN;
                _checksum += c;
            }
            else
            {
                radio_recv_state = RADIO_RECV_HEAD1;
            }
            break;
        case RADIO_RECV_LEN:
            _pack_recv_len = c;
            radio_recv_state = RADIO_RECV_CNT;
            _checksum += c;
            _pack_buf_len = 0;
            break;
        case RADIO_RECV_CNT:
            _pack_recv_cnt = c;
            radio_recv_state = RADIO_RECV_SYSID;
            _checksum += c;
            _pack_buf_len = 0;
            break;
        case RADIO_RECV_SYSID:
            _sysid = c;
            if(0!=_sysid)
            {
                radio_recv_state = RADIO_RECV_TYPE;
            }
            _checksum += c;
            _pack_buf_len = 0;
            break;
        case RADIO_RECV_TYPE:
            _pack_recv_type = c;
            radio_recv_state = RADIO_RECV_DATA;
            _checksum += c;
            _pack_buf_len = 0;
            break;
        case RADIO_RECV_DATA:
            _pack_recv_buf[_pack_buf_len] = c;
            _checksum += c;
            _pack_buf_len++;
            if (_pack_buf_len >= _pack_recv_len)
            {
                radio_recv_state = RADIO_RECV_CHECKSUM;
            }
            break;
        case RADIO_RECV_CHECKSUM:
            if (_checksum == c)
            {
                switch (_pack_recv_type)
                {
                /*
                 * 经过帧头校验后的处理程序写在这里
                 */
#if 0
                case COMMAND_GCS2AP_CMD:
                    if (_pack_recv_len == sizeof(gcs2ap_cmd))
                    {
                        memcpy(&gcs2ap_cmd, _pack_recv_buf, _pack_recv_len);
                        global_bool_boatpilot.bool_get_gcs2ap_cmd = TRUE;
                    }
                    radio_recv_state = 0;
                    global_bool_boatpilot.wgcs2ap_cmd_cnt++;
                    break;
#endif
                default:
                    break;
                }
            }
            else
            {
                //send_ap2gcs_ack(_pack_recv_type, _pack_recv_cnt, 0x01);
                radio_recv_state = 0;
            }
        }
    }

    return 0;
}

int send_radio_data(unsigned char *buf, unsigned int len)
{
    uart_device.uart_name=UART_RADIO;
    send_uart_data(uart_device.uart_name, (char *)buf, len);

    return 0;
}

int radio_uart_close()
{
    uart_device.uart_name=UART_RADIO;
    close_uart_dev(uart_device.uart_name);

    return 0;
}

