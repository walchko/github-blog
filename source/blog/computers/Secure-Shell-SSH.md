---
title: Secure Shell (ssh)
date: 2012-09-10
---



![](pics/ssh.jpg)

| Command     | Example                        | Use                       |
| ----------- | ------------------------------ | ------------------------- |
| ssh         | ssh <user@email>               | login to a computer       |
| ssh-keygen  | ssh-keygen                     | generate an ssh key       |
| ssh-keygen  | ssh-keygen -lvf                | view the key finger print |
| ssh-copy-id | ssh-copy-id <user@email>       | copy key to remote server |

## Generate Key

```
kevin@dalek ~ $ ssh-keygen -t ed25519 -a 100
Generating public/private ed25519 key pair.
Enter file in which to save the key (/home/kevin/.ssh/id_ed25519): 
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/kevin/.ssh/id_ed25519.
Your public key has been saved in /home/kevin/.ssh/id_ed25519.pub.
The key fingerprint is:
SHA256:1cNtxP2990ZClsIoFek67E4S9xIdTeNtmleTJJ4B4HA kevin@dalek
The key's randomart image is:
+--[ED25519 256]--+
|       . Eo+++.o |
|        + o*.=* o|
|         +oo*o=++|
|        .oo.o*+.+|
|      ..So. o+. .|
|       o+o   ...o|
|      ..o..    oo|
|       o..      o|
|       ..      . |
+----[SHA256]-----+
```

## Get a Fingerprint

```
[kevin@Tardis ~]$ ssh-keygen -lf .ssh/test_rsa_key.pub
2048 d0:4a:98:88:95:65:6e:3c:59:7d:10:db:1d:00:10:40  kevin@tardis.local (RSA)
```

## Best Practices

- Use OpenSSH 7 or later (which is common now, Debian Stretch has 7.4, Ubuntu 16.04 has 7.2)
- Use Ed25519 keys: `ssh-keygen -t ed25519 -a 100`
	- `-a 100` specifies 100 rounds of derivation which makes the key harder to brute force
	- creates a pub/private key at `.ssh/id_ed25519[.pub]`
	- New key, not every server will support
	- Ed25519 is the EdDSA signature scheme using SHA-512 (SHA-2) and Curve25519
- Else, use RSA, 4096 bits: `ssh-keygen -t rsa -b 4096 


# References

- [Cryptographic Guide 2019](https://paragonie.com/blog/2019/03/definitive-2019-guide-cryptographic-key-sizes-and-algorithm-recommendations)
- [Seravo: Creating good ssh keys](https://seravo.fi/2019/how-to-create-good-ssh-keys)
- [StackExchange: use ed25519](https://security.stackexchange.com/a/144044)
- [wikipedia: EdDSA](https://en.wikipedia.org/wiki/EdDSA)
