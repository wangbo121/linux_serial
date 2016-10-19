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

static int radio_uart_init_real(struct T_UART_DEVICE_PROPERTY *ptr_uart_device_property);

int radio_uart_init(unsigned int uart_num)
{
    uart_device_property.uart_num=uart_num;
    uart_device_property.baudrate=UART_RADIO_BAUD;
    uart_device_property.databits=UART_RADIO_DATABITS;
    uart_device_property.parity=UART_RADIO_PARITY;
    uart_device_property.stopbits=UART_RADIO_STOPBITS;

    radio_uart_init_real(&uart_device_property);

    return 0;
}

static int radio_uart_init_real(struct T_UART_DEVICE_PROPERTY *ptr_uart_device_property)
{

    if(-1!=(uart_fd[ptr_uart_device_property->uart_num]=open_uart_dev(uart_fd[ptr_uart_device_property->uart_num], ptr_uart_device_property->uart_num)))
    {
        printf("uart_fd[UART_RADIO]=%d\n",uart_fd[ptr_uart_device_property->uart_num]);
        set_uart_opt( uart_fd[ptr_uart_device_property->uart_num], \
                      ptr_uart_device_property->baudrate,\
                      ptr_uart_device_property->databits,\
                      ptr_uart_device_property->parity,\
                      ptr_uart_device_property->stopbits);

        uart_device_pthread(ptr_uart_device_property->uart_num);

#ifdef PRINT_DEBUG_MESSAGE
        sprintf(dbg_msg,"create uart [%d] and uart_recvbuf_and_process [%d] for RADIO OK!", ptr_uart_device_property->uart_num, ptr_uart_device_property->uart_num);
        PRINT_DBGMSG();
#endif
    }

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

/*
 * 函数需要修改
 * 这个函数用到了global_bool_boatpilot.radio_send_packet_cnt
 * 和global_bool_boatpilot.bool_radio_is_locked
 */
int send_radio_data(unsigned char *buf, unsigned int len)
{
    send_uart_data(uart_fd[UART_RADIO], (char *)buf, len);

    return 0;
}

int radio_uart_close(unsigned int uart_num)
{
    close(uart_fd[uart_num]);

    return 0;
}

