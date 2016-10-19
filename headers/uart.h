/*
 * uart.h
 *
 *  Created on: Oct 19, 2016
 *      Author: wangbo
 */

#ifndef HEADERS_UART_H_
#define HEADERS_UART_H_


#define UART_DEV_TOTAL 10
#define UART_BUF_SIZE 512

struct T_UART_DEVICE_PROPERTY
{
    unsigned char uart_num;
    unsigned char databits;
    unsigned char parity;
    unsigned char stopbits;

    unsigned int baudrate;
};

extern int uart_fd[UART_DEV_TOTAL];
extern char *uart_dev[];
extern struct T_UART_DEVICE_PROPERTY uart_device_property;

int open_uart_dev(int fd, int uart_no);
int set_uart_opt(int fd, int speed, int bits, char event, int stop);
int read_uart_data(int fd, char *rcv_buf, int time_out_ms, int buf_len);
int send_uart_data(int fd, char *send_buf, int buf_len);

void uart_recvbuf_and_process(int uart_no);
int uart_device_pthread(int uart_no);

/*
 * 测试时，保存读取的数据
 */
/*int dataSave(int uart_no, int baudrate);*/


#endif /* HEADERS_UART_H_ */
