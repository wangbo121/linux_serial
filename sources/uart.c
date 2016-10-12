/*
 * uart.c
 *
 *  Created on: 2016年5月9日
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <fcntl.h>
#include <unistd.h>

 /*set baud rate*/
#include <termios.h>

#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>

/*create uart device pthread*/
#include <pthread.h>

#include "uart.h"
#include "digitalradio.h"

char *uart_dev[]={"/dev/ttyO0",\
		          "/dev/ttyO1",\
				  "/dev/ttyO2",\
				  "/dev/ttyO3",\
				  "/dev/ttyO4",\
				  "/dev/ttyO5",\
		          "/dev/ttyUSB0",\
				  "/dev/ttyUSB1",\
				  "/dev/ttyUSB2"};

int uart_fd[10]={0};
struct T_UART_DEVICE_PROPERTY uart_device_property;

static pthread_t uart_pthrd[]={0};
static unsigned long int uart_recv_cnt[]={0};

int open_uart_dev(int fd, int uart_no)
{
	//fd = open(uart_dev[uart_no], O_RDWR | O_NOCTTY | O_NDELAY);
	fd = open(uart_dev[uart_no], O_RDWR | O_NOCTTY | O_NONBLOCK);
	printf("uart_dev[%d]  :  fd=%d",uart_no,fd);
	if (-1 == fd)
	{
		//perror("Can't Open Serial Port /dev/ttyS0");
		printf("Can't Open Serial Port %s \n",uart_dev[uart_no]);
		return(-1);
	}
	else
	{
		printf("Open %s  .....\n",uart_dev[uart_no]);
	}

	if (fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed!\n");
	}
	else
	{
		printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
	}
	if (isatty(STDIN_FILENO) == 0)
	{
		printf("standard input is not a terminal device\n");
	}
	else
	{
		printf("is a tty success!\n");
	}
	printf("open uart_dev[%d], fd-open=%d\n", uart_no,fd);

	return fd;
}

int set_uart_opt(int fd, int speed, int bits, char event, int stop)
{
	struct termios newtio, oldtio;

	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch (bits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

	switch (event)
	{
	/*odd check*/
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	/*even check*/
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 0:
			newtio.c_cflag &= ~PARENB;
			break;

	/*
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	*/
	}

	switch (speed)
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 57600:
		cfsetispeed(&newtio, B57600);
		cfsetospeed(&newtio, B57600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}

	if (stop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (stop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}

	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);

	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}

	return 0;
}

/*
 * 这个函数一定会读取到buf_len个数据到rcv_buf后才会结束，
 * 除非到达time_out_ms时间后，还没有收到buf_len个数的数据才会结束，相当与多了个缓存空间
 * 所以针对485来说，等待时间过长会收取到一个数据，因此time_out_ms不易过长
 */
int read_uart_data(int fd, char *rcv_buf, int time_out_ms, int buf_len)
{
	int retval;
	static fd_set rfds;
	struct timeval tv;
	int ret, pos;
	tv.tv_sec = time_out_ms / 1000;  //set the rcv wait time
	tv.tv_usec = (time_out_ms % 1000) * 1000;  //100000us = 0.1s

	struct stat temp_stat;

	pos = 0;

#ifdef PRINT_DEBUG_MESSAGE
	sprintf(dbg_msg,"waiting for read_uart_data fd=%d",fd);
	PRINT_DBGMSG();
#endif

	while (1)
	{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		if(-1==fstat(fd,&temp_stat))
		{
			printf("fstat %d error:%s",fd,strerror(errno));
		}

		retval = select(fd + 1, &rfds, NULL, NULL, &tv);
		//retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
		//retval = select(fd + 1, &rfds, NULL, NULL, NULL);

		if (retval == -1)
		{
			perror("select()");
			break;
		}
		else if (retval)
		{
			ret = read(fd, rcv_buf + pos, 1);
			if (-1 == ret)
			{
				break;
			}

			pos++;
			if (buf_len <= pos)
			{
				break;
			}
		}
		else
		{
			break;
		}
	}

	return pos;
}

int send_uart_data(int fd, char *send_buf, int buf_len)
{
	int ret=0;

	ret = write(fd, send_buf, buf_len);
	if (ret == -1)
	{
		printf("write device error fd = %d\n",fd);
		return -1;
	}

	return 1;
}

void uart_recvbuf_and_process(int uart_no)
{
	char buf[UART_BUF_SIZE] = { 0 };
	unsigned int read_len;



	while(1)
	{

		//if(0!=(read_len=read_uart_data(uart_fd[uart_no], buf, 1000, sizeof(buf)-1)))
		//if(-1!=(read_len=read_uart_data(uart_fd[uart_no], buf, 1000, sizeof(buf)-1)))
		/*
		 * 这个函数的等待时间可能需要修改去减少等待时间，也就是不许要缓存
		 * 读取长度可能也需要减小，因为无论是radio还是gps还是485都不需要512个字节这么大
		 * 如果时间改小后不能满足gps或者电台的数据接收了，那么就把这一据放在switch case语句中
		 */
		if(-1!=(read_len=read_uart_data(uart_fd[uart_no], buf, 200, sizeof(buf)-1)))
		//if( (read_len=read_uart_data(uart_fd[uart_no], buf, 200, sizeof(buf)-1)) >0 )
		{
			//printf("read_len=%d\n",read_len);
			//buf[read_len]='\0';
			//printf("%s\n",buf);

			uart_recv_cnt[uart_no] += read_len;
			switch (uart_no)
			{
			/*
			 * 串口处理程序写在这里
			 */
#if 1
				case UART_RADIO:
				read_radio_data((unsigned char *)buf, read_len);
				break;
#else
				case UART_GPS:
				read_gps_data(buf, read_len);
				break;

				case UART_MODBUS:
				read_modbus_data((unsigned char *)buf, read_len);
				break;
				/*
			case UART_IMU:
				getIMUData(buf, read_len);
				break;
			case UART_PWM:
				break;
			case UART_AWS:
				getAWSData(buf, read_len);
			case UART_POWER:
				getPowerData(buf, read_len);
				break;
				*/
#endif
				default:
				break;
			}
		}


	}

	return ;
}

int uart_device_pthread(int uart_no)
{
	int ret=0;

#ifdef PRINT_DEBUG_MESSAGE
	sprintf(dbg_msg,"enter uart_device_pthread %d!\n",uart_fd[uart_no]);
	PRINT_DBGMSG();
#endif


	ret = pthread_create (&uart_pthrd[uart_no],            //线程标识符指针
		  	              NULL,                            //默认属性
						  (void *)uart_recvbuf_and_process,//运行函数
						  (void *)uart_no);                 //无参数
	if (0 != ret)
	{
	   perror ("pthread create error\n");
	}

	return 0;
}
