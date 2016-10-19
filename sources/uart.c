/*
 * uart.c
 *
 *  Created on: Oct 19, 2016
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
//#include "digitalradio.h"

#define UART_DEV_TOTAL 12
#define UART_BUF_SIZE  512

struct T_UART_DEVICE uart_device;

static char *uart_dev[UART_DEV_TOTAL]={"/dev/ttyO0","/dev/ttyO1","/dev/ttyO2","/dev/ttyO3","/dev/ttyO4","/dev/ttyO5",\
                                       "/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5",\
                                      };
static int uart_fd[UART_DEV_TOTAL]={0};
static pthread_t uart_pthrd[UART_DEV_TOTAL]={0};
static unsigned int uart_recv_cnt[UART_BUF_SIZE]={0};

static int get_uart_num(char *uart_name)
{
    int i=0;
    int uart_no=0;

    for(i=0;i<UART_DEV_TOTAL;i++)
    {
        if(strcmp(uart_name,uart_dev[i])==0)
        {
            //printf("这是第i个串口=%d\n",i);
            uart_no=i;
            break;
        }
    }

    if(i==UART_DEV_TOTAL)
    {
        printf("串口名称错误，没有相应的串口\n");
        return -1;
    }
    else
    {
        return uart_no;
    }
}

static int get_uart_fd(char *uart_name)
{
    int uart_num;

    uart_num=get_uart_num(uart_name);

    if(-1!=uart_num)
    {
        return uart_fd[uart_num];
    }
    else
    {
        return -1;
    }
}

int open_uart_dev(char *uart_name)
{
    int i=0;
    int uart_no=0;
    int fd=0;

    uart_no=get_uart_num(uart_name);
    //fd = open(uart_dev[uart_no], O_RDWR | O_NOCTTY | O_NDELAY);
    //fd = open(uart_dev[uart_no], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(-1 != uart_no)
    {
        fd = open(uart_dev[uart_no], O_RDWR | O_NOCTTY | O_NONBLOCK);
    }
    else
    {
        printf("Can't Open Serial Port %s \n",uart_dev[uart_no]);
    }

    if (-1 == fd)
    {
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
    printf("uart_dev[%s]  :  uart_dev[%d]  :  fd=%d",uart_dev[uart_no],uart_no,fd);

    uart_fd[uart_no]=fd;

    return uart_no;
}

int set_uart_opt(char *uart_name, int speed, int bits, char event, int stop)
{
    int fd;
    struct termios newtio, oldtio;

    fd=get_uart_fd(uart_name);

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
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    case 0:
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        newtio.c_cflag &= ~PARENB;
        break;
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

int send_uart_data(char *uart_name, char *send_buf, int buf_len)
{
    int ret=0;
    int fd=0;

    fd=get_uart_fd(uart_name);

    ret = write(fd, send_buf, buf_len);
    if (ret == -1)
    {
        printf("write device error fd = %d\n",fd);
        return -1;
    }

    return 1;
}

/*
 * 这个函数一定会读取到buf_len个数据到rcv_buf后才会结束，
 * 除非到达time_out_ms时间后，还没有收到buf_len个数的数据才会结束，相当于多了个缓存空间
 * 所以针对485来说，等待时间过长会收取到一个数据，因此time_out_ms不易过长
 */
int read_uart_data(char *uart_name, char *rcv_buf, int time_out_ms, int buf_len)
{
    int fd=0;
    int retval;
    static fd_set rfds;
    struct timeval tv;
    int ret, pos;
    tv.tv_sec = time_out_ms / 1000;  //set the rcv wait time
    tv.tv_usec = (time_out_ms % 1000) * 1000;  //100000us = 0.1s

    struct stat temp_stat;

    fd=get_uart_fd(uart_name);
    pos = 0;

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

void uart_recvbuf_and_process(void * ptr_pthread_arg)
{
    char buf[UART_BUF_SIZE] = { 0 };
    unsigned int read_len;

    unsigned int uart_no=0;
    struct T_UART_DEVICE *ptr_uart;

    ptr_uart=(struct T_UART_DEVICE *)ptr_pthread_arg;

    uart_no=ptr_uart->uart_num;

    while(1)
    {
        if(-1!=(read_len=read_uart_data(ptr_uart->uart_name, buf, 200, sizeof(buf)-1)))
        {
#if 0
            printf("read_len=%d\n",read_len);
            buf[read_len]='\0';
            printf("%s\n",buf);
#endif
            uart_recv_cnt[uart_no] += read_len;

            //ptr_uart->ptr_fun((unsigned char*)buf,read_len);
            if(read_len>0)
            {
                ptr_uart->ptr_fun((unsigned char*)buf,read_len);
            }

        }
    }
}

int create_uart_pthread(struct T_UART_DEVICE *ptr_uart)
{
    int ret=0;
    int uart_no;

    uart_no=ptr_uart->uart_num;

    ret = pthread_create (&uart_pthrd[uart_no],            //线程标识符指针
                          NULL,                            //默认属性
                          (void *)uart_recvbuf_and_process,//运行函数
                          (void *)ptr_uart);                 //运行函数的参数
    if (0 != ret)
    {
       perror ("pthread create error\n");
    }

    return 0;
}

int close_uart_dev(char *uart_name)
{
    int fd=0;

    fd=get_uart_fd(uart_name);

    if(fd!=-1)
    {
        close(fd);
    }

    return 0;
}
