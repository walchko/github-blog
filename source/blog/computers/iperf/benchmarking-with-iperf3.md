---
title: iperf3
date: 2016-04-02
abstract: How to benchmark your network performance with `iperf3`
image: "https://i.pinimg.com/564x/72/0a/6a/720a6a5d265af4c589ad74a8592a4ef1.jpg"
---

[iperf3] allows you to benchmark network performance. [Github]

## Server

Start the server:

``` {.bashsession}
[kevin@Dalek ~]$ iperf3 -s -p 8282
```

## Client

Run the client to test the network:

``` {.bashsession}
[kevin@Tardis ~]$ iperf3 -c Dalek.local -p 8282
Connecting to host Dalek.local, port 8282
[  6] local 192.168.1.3 port 54673 connected to 192.168.1.2 port 8282
[ ID] Interval           Transfer     Bandwidth
[  6]   0.00-1.00   sec  22.3 MBytes   187 Mbits/sec
[  6]   1.00-2.00   sec  21.3 MBytes   178 Mbits/sec
[  6]   2.00-3.00   sec  18.4 MBytes   155 Mbits/sec
[  6]   3.00-4.00   sec  21.3 MBytes   178 Mbits/sec
[  6]   4.00-5.00   sec  20.4 MBytes   171 Mbits/sec
[  6]   5.00-6.00   sec  21.0 MBytes   176 Mbits/sec
[  6]   6.00-7.00   sec  20.5 MBytes   172 Mbits/sec
[  6]   7.00-8.00   sec  20.8 MBytes   174 Mbits/sec
[  6]   8.00-9.00   sec  20.7 MBytes   174 Mbits/sec
[  6]   9.00-10.00  sec  21.8 MBytes   183 Mbits/sec
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID] Interval           Transfer     Bandwidth
[  6]   0.00-10.00  sec   209 MBytes   175 Mbits/sec                  sender
[  6]   0.00-10.00  sec   209 MBytes   175 Mbits/sec                  receiver

iperf Done.
```

  [iperf3]: http://software.es.net/iperf/
  [Github]: https://github.com/esnet/iperf
