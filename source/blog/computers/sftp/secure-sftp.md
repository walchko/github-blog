---
title: Using Secure FTP
date: 7 June 2020
---

Never use normal FTP, it isn't safe. Instead use secure FTP (`sftp`).

| Command           | Description                               |
|-------------------|-------------------------------------------|
| `get <file>`      | copy file from remote system to local     |
| `get -rP <file>`  | `r` recursive, `P` maintains permissions  |
| `put <file>`      | copy file from local system to remote, `r` recursive |
|`lls`,`lpwd`,`lcd` | operate on local system as expected       |
| `ls`, `pwd`, `cd` | operate on remote system as expected      |
| `exit`            | exit sftp                                 | 


## References

- Digital Oceans: [How To Use SFTP to Securely Transfer Files with a Remote Server](https://www.digitalocean.com/community/tutorials/how-to-use-sftp-to-securely-transfer-files-with-a-remote-server)
