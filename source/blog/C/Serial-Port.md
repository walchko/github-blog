![](pics/c.png){width=200px}

# Serial Port

`gcc -o serial serial.c`

```c
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */

int main(){
  int fd;

  /* O_RDWR   - Read/Write access to serial port       */
  /* O_NOCTTY - No terminal will control the process   */
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);    

  if (fd == -1){
    printf("\n  Error! in Opening ttyUSB0\n\n");
    return 1;
  }
  else
    printf("\n\n  ttyUSB0 Opened Successfully\n\n");

  close(fd);
  return 0;
}
```
