/*
 * digitalradio.h
 *
 *  Created on: Oct 19, 2016
 *      Author: wangbo
 */

#ifndef HEADERS_DIGITALRADIO_H_
#define HEADERS_DIGITALRADIO_H_

//#define UART_RADIO 6
#define UART_RADIO "/dev/ttyUSB0"

#define UART_RADIO_BAUD 9600
#define UART_RADIO_DATABITS 8 //8 data bit
#define UART_RADIO_STOPBITS 1 //1 stop bit
#define UART_RADIO_PARITY 0 //no parity

#define RADIO_MAX_WAIT_TIME 6//[s]



int radio_uart_init();

/*
 * Function:read_radio_data
 * Description:通过电台接收cmd，waypoint，link，cte，parameter等由地面站发送过来的数据
 *             传输过来的数据尽量每个都是unsigned char型，否则就要考虑大小端，然后在实现中交换位置
 *             比如目前的航点数据中的经度和纬度
 */
int read_radio_data(unsigned char *buf, unsigned int len);

/*
 * Function:send_radio_data
 * Description:通过电台发送数据，目前是发送了实时数据，发送了回传命令包，发送了回传航点
 */
int send_radio_data(unsigned char *buf, unsigned int len);

int radio_uart_close();



#endif /* HEADERS_DIGITALRADIO_H_ */
