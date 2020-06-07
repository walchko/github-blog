---
title: Secure Shell (ssh)
date: 2012-09-10
---


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

## Adding Key

```
Host *
  AddKeysToAgent yes
  UseKeychain yes
  IdentityFile ~/.ssh/id_ed25519
  IdentityFile ~/.ssh/id_rsa # Keep any old key files if you want
```

Then to update add key to SSH agent: `ssh-add -K ~/.ssh/id_ed25519`

## Best Practices

- **rsa:** an old algorithm based on the difficulty of factoring large numbers. A key size of at least 2048 bits is recommended for RSA; 4096 bits is better. RSA is getting old and significant advances are being made in factoring. Choosing a different algorithm may be advisable. It is quite possible the RSA algorithm will become practically breakable in the foreseeable future. All SSH clients support this algorithm.
- **dsa:** an old US government Digital Signature Algorithm. It is based on the difficulty of computing discrete logarithms. A key size of 1024 would normally be used with it. DSA in its original form is no longer recommended.
- **ecdsa:** a new Digital Signature Algorithm standarized by the US government, using elliptic curves. This is probably a good algorithm for current applications. Only three key sizes are supported: 256, 384, and 521 (sic!) bits. We would recommend always using it with 521 bits, since the keys are still small and probably more secure than the smaller keys (even though they should be safe as well). Most SSH clients now support this algorithm.
- **ed25519:** this is a new algorithm added in OpenSSH. Support for it in clients is not yet universal. Thus its use in general purpose applications may not yet be advisable.
- Use OpenSSH 7 or later (which is common now, Debian Stretch has 7.4, Ubuntu 16.04 has 7.2)
- Use Ed25519 keys: `ssh-keygen -t ed25519 -a 100`
	- `-a 100` specifies 100 rounds of derivation which makes the key harder to brute force
	- creates a pub/private key at `.ssh/id_ed25519[.pub]`
	- New key, not every server will support
	- Ed25519 is the EdDSA signature scheme using SHA-512 (SHA-2) and Curve25519
- Else, use RSA, 4096 bits: `ssh-keygen -t rsa -b 4096 

# Disabling Host Key Check

Why?? Well, when you build a lot of raspberry pi computers and the key keeps changing, it is
annoying to keep fixing it.

From the command line: `$ ssh -o "UserKnownHostsFile=/dev/null" -o "StrictHostKeyChecking=no" pi@raspberrypi.local`.

To perminately change it, create a `~/.ssh/config` file with:

```
Host *
   StrictHostKeyChecking no
   UserKnownHostsFile=/dev/null
```

Or (maybe safer)

```
Host raspberrypi.local
   StrictHostKeyChecking no
   UserKnownHostsFile=/dev/null
```

# References

- [Cryptographic Guide 2019](https://paragonie.com/blog/2019/03/definitive-2019-guide-cryptographic-key-sizes-and-algorithm-recommendations)
- [Seravo: Creating good ssh keys](https://seravo.fi/2019/how-to-create-good-ssh-keys)
- [StackExchange: use ed25519](https://security.stackexchange.com/a/144044)
- [wikipedia: EdDSA](https://en.wikipedia.org/wiki/EdDSA)
- [ssh.com keygen](https://www.ssh.com/ssh/keygen)
- Medium: [Upgrade Your SSH Key to Ed25519](https://medium.com/risan/upgrade-your-ssh-key-to-ed25519-c6e8d60d3c54)
