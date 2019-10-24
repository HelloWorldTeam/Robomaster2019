#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <uart.h>
#include <unistd.h>
#include <iostream>
#include <thread>

using namespace std;

/*
 * 安全读写函数
 */
UART::UART(const char *pathname, const char *pathname_bkp = NULL)
{
  init(pathname, pathname_bkp);
}

void UART::init(const char *pathname, const char *pathname_bkp)
{
  while (1) {
    fd = open(pathname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
      perror("Open UART failed!");
    }
    else break;
    fd = open(pathname_bkp, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
      perror("Open UART failed!");
    }
    else break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (uart_set(115200, 0, 8, 'N', 1) == -1) {
    fprintf(stderr, "uart set failed!\n");
    exit(EXIT_FAILURE);
  }
}

UART::~UART() {
  assert(fd);
  close(fd);
}

ssize_t UART::safe_write(const void *vptr, size_t n) {
  size_t nleft;
  ssize_t nwritten;
  const char *ptr;

  ptr = (char *)vptr;
  nleft = n;

  while (nleft > 0) {
    if ((nwritten = write(fd, ptr, nleft)) <= 0) {
      if (nwritten < 0 && errno == EINTR)
        nwritten = 0;
      else
        return -1;
    }
    nleft -= nwritten;
    ptr += nwritten;
  }
  return (n);
}

ssize_t UART::safe_read(void *vptr, size_t n) {
  size_t nleft;
  ssize_t nread;
  char *ptr;

  ptr = (char *)vptr;
  nleft = n;

  while (nleft > 0) {
    if ((nread = read(fd, ptr, nleft)) < 0) {
      if (errno == EINTR)  //被信号中断
        nread = 0;
      else
        return -1;
    } else if (nread == 0)
      break;
    nleft -= nread;
    ptr += nread;
    cout << "read: " << nread << "bytes\n";
  }
  return (n - nleft);
}

int UART::uart_set(int baude, int c_flow, int bits, char parity, int stop) {
  struct termios options;

  /*获取终端属性*/
  if (tcgetattr(fd, &options) < 0) {
    perror("tcgetattr error");
    return -1;
  }

  /*设置输入输出波特率，两者保持一致*/
  switch (baude) {
    case 4800:
      cfsetispeed(&options, B4800);
      cfsetospeed(&options, B4800);
      break;
    case 9600:
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
      break;
    case 115200:
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      break;
    case 19200:
      cfsetispeed(&options, B19200);
      cfsetospeed(&options, B19200);
      break;
    case 38400:
      cfsetispeed(&options, B38400);
      cfsetospeed(&options, B38400);
      break;
    default:
      fprintf(stderr, "Unkown baude!\n");
      return -1;
  }

  /*设置控制模式*/
  options.c_cflag |= CLOCAL;  //保证程序不占用串口
  options.c_cflag |= CREAD;   //保证程序可以从串口中读取数据

  /*设置数据流控制*/
  switch (c_flow) {
    case 0:  //不进行流控制
      options.c_cflag &= ~CRTSCTS;
      break;
    case 1:  //进行硬件流控制
      options.c_cflag |= CRTSCTS;
      break;
    case 2:  //进行软件流控制
      options.c_cflag |= IXON | IXOFF | IXANY;
      break;
    default:
      fprintf(stderr, "Unkown c_flow!\n");
      return -1;
  }

  /*设置数据位*/
  switch (bits) {
    case 5:
      options.c_cflag &= ~CSIZE;  //屏蔽其它标志位
      options.c_cflag |= CS5;
      break;
    case 6:
      options.c_cflag &= ~CSIZE;  //屏蔽其它标志位
      options.c_cflag |= CS6;
      break;
    case 7:
      options.c_cflag &= ~CSIZE;  //屏蔽其它标志位
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag &= ~CSIZE;  //屏蔽其它标志位
      options.c_cflag |= CS8;
      break;
    default:
      fprintf(stderr, "Unkown bits!\n");
      return -1;
  }

  /*设置校验位*/
  switch (parity) {
    /*无奇偶校验位*/
    case 'n':
    case 'N':
      options.c_cflag &= ~PARENB;  // PARENB：产生奇偶位，执行奇偶校验
      // options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
      break;
      /*设为空格,即停止位为2位*/
    case 's':
    case 'S':
      options.c_cflag &= ~PARENB;  // PARENB：产生奇偶位，执行奇偶校验
      options.c_cflag &= ~CSTOPB;  // CSTOPB：使用两位停止位
      break;
      /*设置奇校验*/
    case 'o':
    case 'O':
      options.c_cflag |= PARENB;  // PARENB：产生奇偶位，执行奇偶校验
      options.c_cflag |= PARODD;  // PARODD：若设置则为奇校验,否则为偶校验
      options.c_cflag |= INPCK;  // INPCK：使奇偶校验起作用
      options.c_cflag |=
          ISTRIP;  // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
      break;
      /*设置偶校验*/
    case 'e':
    case 'E':
      options.c_cflag |= PARENB;  // PARENB：产生奇偶位，执行奇偶校验
      options.c_cflag &= ~PARODD;  // PARODD：若设置则为奇校验,否则为偶校验
      options.c_cflag |= INPCK;  // INPCK：使奇偶校验起作用
      options.c_cflag |=
          ISTRIP;  // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
      break;
    default:
      fprintf(stderr, "Unkown parity!\n");
      return -1;
  }

  /*设置停止位*/
  switch (stop) {
    case 1:
      options.c_cflag &= ~CSTOPB;  // CSTOPB：使用两位停止位
      break;
    case 2:
      options.c_cflag |= CSTOPB;  // CSTOPB：使用两位停止位
      break;
    default:
      fprintf(stderr, "Unkown stop!\n");
      return -1;
  }

  /*设置输出模式为原始输出*/
  options.c_oflag &=
      ~OPOST;  // OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

  /*设置本地模式为原始模式*/
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /*
   *ICANON：允许规范模式进行输入处理
   *ECHO：允许输入字符的本地回显
   *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
   *ISIG：允许信号
   */

  /*设置等待时间和最小接受字符*/
  options.c_cc[VTIME] = 0;  //可以在select中设置
  options.c_cc[VMIN] = 1;   //最少读取一个字符

  /*如果发生数据溢出，只接受数据，但是不进行读操作*/
  tcflush(fd, TCIFLUSH);

  /*激活配置*/
  if (tcsetattr(fd, TCSANOW, &options) < 0) {
    perror("tcsetattr failed");
    return -1;
  }

  return 0;
}

int UART::uart_read(char *r_buf, size_t len) {
  ssize_t cnt = 0;
  fd_set rfds;
  struct timeval time;

  /*将文件描述符加入读描述符集合*/
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  /*设置超时为5s*/
  time.tv_sec = 5;
  time.tv_usec = 0;

  /*实现串口的多路I/O*/
  int ret = select(fd + 1, &rfds, NULL, NULL, &time);
  switch (ret) {
    case -1:
      fprintf(stderr, "select error!\n");
      return -1;
    case 0:
      fprintf(stderr, "time over!\n");
      return -1;
    default:
      cnt = read(fd, r_buf, len);
      if (cnt <= 0) {
        fprintf(stderr, "read error!\n");
        init("/dev/ttyUSB0", "/dev/ttyUSB1");
        return -1;
      }
      return cnt;
  }
}

int UART::uart_write(const char *w_buf, size_t len) {
  ssize_t cnt = 0;

  cnt = safe_write(w_buf, len);
  if (cnt == -1) {
    fprintf(stderr, "write error!\n");
    return -1;
  }

  return cnt;
}

int UART::send(double X, double Y, double Z, int type) {
  /* configure can_id and can data length */
  char frame[11];
  // X = 0.010;Y = 0.010; Z = 1.374;
  short x100 = X * 1000;
  short y100 = Y * 1000;
  short z100 = Z * 1000;
  frame[3] = (unsigned char)(x100 >> 8);
  frame[4] = (unsigned char)(x100);
  frame[5] = (unsigned char)(y100 >> 8);
  frame[6] = (unsigned char)(y100);
  frame[7] = (unsigned char)(z100 >> 8);
  frame[8] = (unsigned char)(z100);
  int tmp = 0;
  for (int i=3; i<9; ++i) tmp+=frame[i];
  frame[9] = (unsigned char)tmp;
  frame[10] = (unsigned char)(type);
  frame[0] = 0xaa;
  frame[1] = 0xbb;
  frame[2] = 0xcc;
#ifdef __SHOWING_SEND__
  printf("X Y Z is: %.3f %.3f %.3f\n", X, Y, Z);
  printf("uart send: data=");
  for (int i = 0; i < sizeof(frame); i++) printf("%x ", frame[i]);
  printf("\n");
#endif
  /* Sending data */
  int nbytes = uart_write(frame, sizeof(frame));
  //nbytes = uart_write(frame, sizeof(frame));
  if (nbytes < 0) {
    perror("Send failed");
  }
  return nbytes;
}

int UART::receive(char *buffer) {
  char frame[3] = {0};

  uart_read(frame, 1);
  if (frame[0] != 0x3f) {
    printf("Head is wrong\n");
    return 0;
  }
  
  uart_read(frame, 1);
  if (frame[0] != 0x4f) {
    printf("Head is wrong\n");
    return 0;
  }

  int nbytes = uart_read(frame, 2);
  if (nbytes <= 0) {
    printf("nbytes = %d\n", nbytes);
    return 0;
  }

  printf("uart receive: data= ");
  for (int i = 0; i < nbytes; i++) printf("%x ", frame[i]);
  printf("\n");
  memcpy(buffer, frame, nbytes);

  return nbytes;
}
