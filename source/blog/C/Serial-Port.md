![](pics/c.png){width=200px}

# Serial Port

- **O_NOCTTY:** If pathname refers to a terminal device — it will not become the process's controlling terminal even if the process does not have one. O_NOFOLLOW : If pathname is a symbolic link, then the open fails. 
- **O_NONBLOCK:** When possible, the file is opened in non-blocking mode. Neither the open() nor any subsequent operations on the file descriptor which is returned will cause the calling process to wait. This option is equivalent to O_NODELAY option. 
- **O_SYNC:** The file is opened for synchronous I/O. Any write on the resulting file descriptor will block the calling process until the data has been physically written to the underlying hardware. 

`gcc -o serial serial.c`

<script src="https://gist.github.com/walchko/6d342aa34deed471d3b542fb0a9168f5.js"></script>

# Reference

- [Understanding Linux open() system call)](https://www.ibm.com/developerworks/community/blogs/58e72888-6340-46ac-b488-d31aa4058e9c/entry/understanding_linux_open_system_call?lang=en)
- [ref](https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c)
- [ref](https://github.com/rm5248/CSerial/blob/master/examples/example_full.c)
- [ref](https://github.com/rm5248/CSerial/blob/master/c_serial.h)
- [ref](https://github.com/rm5248/CSerial/blob/master/c_serial.c)
- [Serial Programming Guide for POSIX Operating Systems](https://www.cmrr.umn.edu/~strupp/serial.html#3_1)
