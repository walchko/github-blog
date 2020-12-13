---
title: Inter Process Communications with Shared Memory
date: 3 Oct 2018
image: "virtualmemory.png"
---

```
shm_fd = shm_open(name, O_CREAT | O_RDRW, 0666);
Parameters:
name: The first parameter specifies the name of the shared-memory object.
Processes that wish to access this shared memory must refer to the
object by this name.
O_CREAT | O_RDRW : The subsequent parameters specify that the shared-memory
object is to be created if it does not yet exist (O_CREAT) and that the object is
open for reading and writing (O_RDRW).

The last parameter establishes the directory permissions of the
shared-memory object.
```

<script src="https://gist.github.com/walchko/6d211a2dc8b2f5158642fb2fd65cf295.js"></script>

```bash
% make clean
rm -f cons prod

% make
gcc -std=c99 -Wall -Wextra -D_XOPEN_SOURCE=700 -o cons shm-posix-consumer.c -lrt
gcc -std=c99 -Wall -Wextra -D_XOPEN_SOURCE=700 -o prod shm-posix-producer.c -lrt

% prod
display: prod
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

display: prod
53 74 75 64 79 69 6e 67 20 4f 70 65 72 61 74 69
6e 67 20 53 79 73 74 65 6d 73 20 49 73 20 46 75
6e 21 0a 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

% prod
display: prod
53 74 75 64 79 69 6e 67 20 4f 70 65 72 61 74 69
6e 67 20 53 79 73 74 65 6d 73 20 49 73 20 46 75
6e 21 0a 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

display: prod
53 74 75 64 79 69 6e 67 20 4f 70 65 72 61 74 69
6e 67 20 53 79 73 74 65 6d 73 20 49 73 20 46 75
6e 21 0a 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

% cons
display: cons
53 74 75 64 79 69 6e 67 20 4f 70 65 72 61 74 69
6e 67 20 53 79 73 74 65 6d 73 20 49 73 20 46 75
6e 21 0a 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

Studying Operating Systems Is Fun!

% cons
cons: Shared memory failed: No such file or directory
```

# References

- [shared memory api](https://www.geeksforgeeks.org/posix-shared-memory-api/)
- [psu.edu](http://www.cse.psu.edu/~deh25/cmpsc473/notes/OSC/Processes/shm.html)
