dpkt Cheatsheet
===============

:date: 2015-12-30
:summary: How to use ``dpkt`` to mine packets on your netowrk for info

dpkt is a python library for manipulating packets and although it is a good library it is
very poorly documented.

.. figure:: pics/TCP-IP-model-vs-OSI-model.png
   :align: center

Useful Ports
-------------

TCP
~~~

========= ============
TCP Port  Service
========= ============
21        IRC
22        SSH
25        STMP
80        Http
123       Network Time Server
443       Https
445       SMB
548       Apple File Protocol (AFP) over TCP
3689      iTunes using the iTunes Library Sharing feature
5009      Airport admin utility
9100      HP Jet Direct
10000     ??
========= ============

UDP
~~~

========= ============
UDP Port  Service
========= ============
67,68     DHCP
123       NTP
5353      mDNS, Bonjour
17500     Dropbox
========= ============


Packet Types
-------------

There are many types of packets supported and several ways to test what they are:

.. code-block:: python

    if isinstance(packet,dpkt.ethernet.ETH_TYPE_IP)
    if type(packet) == dpkt.ethernet.ETH_TYPE_IP
    if packet.p == dpkt.ethernet.ETH_TYPE_IP

dpkt.tcp.*  dpkt.udp.*  dpkt.icmp.*

================== ======== ================ ========
dpkt.ethernet.*    Val      dpkt.ip.*        Val
================== ======== ================ ========
ETH_TYPE_ARP                IP_PROTO_ICMP    1
ETH_TYPE_IP        2048     IP_PROTO_TCP     6
ETH_TYPE_IP6       34525    IP_PROTO_UDP     17
                            ICMP6            58
                            SCTP             132
================== ======== ================ ========


Ethernet (Physical & Datalink Layers)
--------------------------------------

Ethernet is a family of computer networking technologies for local area
networks (LANs).

.. code-block:: python

    ether = dpkt.ethernet.Ethernet(data)
    mac = ...

::

	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|       Ethernet destination address (first 32 bits)            |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	| Ethernet dest (last 16 bits)  |Ethernet source (first 16 bits)|
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|       Ethernet source address (last 32 bits)                  |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|        Type code              |                               |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

+---------------+-----------------------------------------------------------------------------+
| Addresses     | Description                                                                 |
+===============+=============================================================================+
| 224.0.0.251   | `Multicast DNS <http://en.wikipedia.org/wiki/Multicast_address>`_ (mDNS)    |
+---------------+-----------------------------------------------------------------------------+

IP (Internet Protocol)
-----------------------

The Internet Protocol (IP) is the principal communications protocol in the Internet
protocol suite for relaying datagrams across network boundaries. Its routing function
enables internetworking, and essentially establishes the Internet.

.. code-block:: python

    ip = dpkt.ip.IP(data)
    ip = ether.data
    ip_addr = socket.inet_ntoa(ip.src|ip.dst)


::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|Version|  IHL  |Type of Service|          Total Length         |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|         Identification        |Flags|      Fragment Offset    |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|  Time to Live |    Protocol   |         Header Checksum       |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                       Source Address                          |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Destination Address                        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Options                    |    Padding    |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

IPv6
----

::

	$ host -t AAAA www.cyberciti.biz
	www.cyberciti.biz has IPv6 address 2607:f0d0:1002:51::4

	$ ping6 2607:f0d0:1002:51::4
	PING 2607:f0d0:1002:51::4(2607:f0d0:1002:51::4) 56 data bytes
	64 bytes from 2607:f0d0:1002:51::4: icmp_seq=1 ttl=64 time=0.056 ms
	64 bytes from 2607:f0d0:1002:51::4: icmp_seq=2 ttl=64 time=0.027 ms
	64 bytes from 2607:f0d0:1002:51::4: icmp_seq=3 ttl=64 time=0.021 ms
	64 bytes from 2607:f0d0:1002:51::4: icmp_seq=4 ttl=64 time=0.023 ms


dpkt.ip6.IP6

.. code-block:: python

	class IP6(dpkt.Packet):
		__hdr__ = (
			('_v_fc_flow', 'I', 0x60000000L),
			('plen', 'H', 0),  # payload length (not including header)
			('nxt', 'B', 0),  # next header protocol
			('hlim', 'B', 0),  # hop limit
			('src', '16s', ''),
			('dst', '16s', '')
			)

:nxt: Next header type, typical values are 6 for TCP, 17 for UDP, 58 for ICMPv6, 132 for SCTP.

.. code-block:: python

    socket.inet_ntop(AF_INET6, ip.dst)
    socket.inet_pton(socket.AF_INET6, "2001:1938:26f:1:204:4bff:0:1")

    ip = eth.data
    if eth.type == dpkt.ethernet.ETH_TYPE_IP6 and ip.nxt == dpkt.ip.IP_PROTO_UDP:


UDP (User Datagram Protocol)
----------------------------

UDP uses a simple connectionless transmission model with a minimum of protocol mechanism.
It has no handshaking dialogues, and thus exposes any unreliability of the underlying
network protocol to the user's program. There is no guarantee of delivery, ordering, or
duplicate protection. UDP provides checksums for data integrity, and port numbers for
addressing different functions at the source and destination of the datagram.

::

	0        7 8     15 16    23 24    31
	 +--------+--------+--------+--------+
	 |     Source      |   Destination   |
	 |      Port       |      Port       |
	 +--------+--------+--------+--------+
	 |                 |                 |
	 |     Length      |    Checksum     |
	 +--------+--------+--------+--------+
	 |
	 |          data octets ...
	 +---------------- ...


TCP (Transmission Control Protocol)
------------------------------------

The Transmission Control Protocol (TCP) is a core protocol of the Internet Protocol Suite.
TCP provides reliable, ordered, and error-checked delivery of a stream of octets between
applications running on hosts communicating over an IP network.

.. code-block:: python

	tcp = ip.data
	port = tcp.sport|dport

::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|          Source Port          |       Destination Port        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                        Sequence Number                        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Acknowledgment Number                      |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|  Data |           |U|A|P|R|S|F|                               |
	| Offset| Reserved  |R|C|S|S|Y|I|            Window             |
	|       |           |G|K|H|T|N|N|                               |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|           Checksum            |         Urgent Pointer        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Options                    |    Padding    |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                             data                              |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+


TCP Flags
~~~~~~~~~~

.. code-block:: python

	fin_flag = ( tcp.flags & dpkt.tcp.TH_FIN ) != 0
	syn_flag = ( tcp.flags & dpkt.tcp.TH_SYN ) != 0
	rst_flag = ( tcp.flags & dpkt.tcp.TH_RST ) != 0
	psh_flag = ( tcp.flags & dpkt.tcp.TH_PUSH) != 0
	ack_flag = ( tcp.flags & dpkt.tcp.TH_ACK ) != 0
	urg_flag = ( tcp.flags & dpkt.tcp.TH_URG ) != 0
	ece_flag = ( tcp.flags & dpkt.tcp.TH_ECE ) != 0
	cwr_flag = ( tcp.flags & dpkt.tcp.TH_CWR ) != 0


DNS (Domain Name System)
-------------------------

The Domain Name System (DNS) is a hierarchical distributed naming system for computers,
services, or any resource connected to the Internet or a private network. An often-used
analogy to explain the Domain Name System is that it serves as the phone book for the
Internet by translating human-friendly computer hostnames into IP addresses. For example,
the domain name www.example.com translates to the addresses 93.184.216.119 (IPv4) and
2606:2800:220:6d:26bf:1447:1097:aa7 (IPv6).

A simple example of parsing a DNS packet

.. code-block:: python

	eth = dpkt.ethernet.Ethernet(buf)
	ip = eth.data
	udp = ip.data
	# make the dns object out of the udp data and check for it being a RR (answer)
	# and for opcode QUERY (I know, counter-intuitive)
	if udp.dport != 53 and udp.sport != 53: continue
	dns = dpkt.dns.DNS(udp.data)
	if dns.qr != dpkt.dns.DNS_R: continue
	if dns.opcode != dpkt.dns.DNS_QUERY: continue
	if dns.rcode != dpkt.dns.DNS_RCODE_NOERR: continue
	if len(dns.an) < 1: continue
	# now we're going to process and spit out responses based on record type
	# ref: http://en.wikipedia.org/wiki/List_of_DNS_record_types
	for answer in dns.an:
		if answer.type == dpkt.dns.DNS_CNAME:
			print "CNAME request", answer.name, "\tresponse", answer.cname
		elif answer.type == dpkt.dns.DNS_A:
			print "A request", answer.name, "\tresponse", socket.inet_ntoa(answer.rdata)
		elif answer.type == dpkt.dns.DNS_PTR:
			print "PTR request", answer.name, "\tresponse", answer.ptrname

Make a DNS packet

.. code-block:: python

	dns = dpkt.dns.DNS(udp.data)
	dns.op = dpkt.dns.DNS_RA
	dns.rcode = dpkt.dns.DNS_RCODE_NOERR
	dns.qr = dpkt.dns.DNS_R

	# make a record
	arr = dpkt.dns.DNS.RR()
	arr.cls = dpkt.dns.DNS_IN
	arr.type = dpkt.dns.DNS_A
	arr.name = 'paypal.com'
	arr.ip = dnet.addr('127.0.0.1').ip

	dns.an.append(arr)
	print dns
	>> DNS(an=[RR(name='paypal.com')], qd=[Q(name='paypal.com')], id=21825, op=32896)

DNS Arrays
~~~~~~~~~~~

==================== ===============
dpkt.dns.* arrays    Description
==================== ===============
qd(name='',type='')  Question
qd.name              The name that was searched, such as 'www.google.com'
qd.cls               class, dpkt.dns.DNS_IN
qd.type              dpkt.dns.DNS_A, many more options
an(name='',type='')  Answer
ns                   List of name servers for this domain. You can iterate over the list
==================== ===============

DNS Codes
~~~~~~~~~~

:qr:      Type of message, either query or response, hence Q/R: dpkt.dns.DNS_Q (0), dpkt.dns.DNS_R (1)
:opcode:  What type of query, usually standard query: dpkt.dns.DNS_QUERY (0)
:rcode:   Errors: dpkt.dns.DNS_RCODE_NOERR (0), anything else is an error
:op:      No clue what this is: dpkt.dns.DNS_RA, dpkt.dns.DNS_AA
:ar:      Authority record or additional record

DNS Questions
~~~~~~~~~~~~~~

dpkt.dns.DNS.Q(data)

:name:   Domain name
:type:   Type of query
:cls:    Class of query
:data:   Data ?


DNS Answer
~~~~~~~~~~~

dpkt keeps an array of responses in dpkt.dns.DNS.RR(data) with the fields:

:name:  Name that was queried
:type:  Type of response, see dpkt.dns.*.type table
:cls:   Class of response, usually internet addr: dpkt.dns.DNS_IN (1)
:ttl:   The number of seconds the result can be cached
:rlen:  The length of the RDATA field
:rdata: The response data. The format is dependent on the TYPE field: A(1) is IPv4 addr, CNAME(5) then a name, NS(2) is name servers, etc
:data:  Data?


================= ==== ===========
dpkt.dns.*.type        Description, assume rr is the python dpkt.dns.DNS object
================= ==== ===========
DNS_A             1    IPv4 address; data = socket.inet_ntoa(rr.ip)
DNS_AAAA          28   IPv6 address
DNS_CNAME         5    Conical name or alias
DNS_HINFO         13   OS info
DNS_MX            15   Mail server: an.mxname
DNS_NS            2    Name server info:
DNS_PTR           12   Map IP to hostname, data = rr.ptrname; ex. 10.27/1.168.192.in-addr.arpa. 1800 PTR mail.example.com.
DNS_SRV           33   Service locator; data = rr.srvname, rr.priority, rr.weight, rr.port
DNS_SOA           6    Start of Authorities, gives info about domain: admin, contact info, etc
DNS_TXT           16   Text field; data = tuple(rr.text) # Convert the list to a hashable tuple
================= ==== ===========

.. code-block:: python

	class DNS(dpkt.Packet):
		hdr__ = (
			('id', 'H', 0),
			('op', 'H', DNS_RD),  # recursive query
			# XXX - lists of query, RR objects
			('qd', 'H', []),
			('an', 'H', []),
			('ns', 'H', []),
			('ar', 'H', [])
		)

	class RR(Q):
		"""DNS resource record."""
		__hdr__ = (
			('name', '1025s', ''),
			('type', 'H', DNS_A),
			('cls', 'H', DNS_IN),
			('ttl', 'I', 0),
			('rlen', 'H', 4),
			('rdata', 's', '')
		)

	class Q(dpkt.Packet):
		"""DNS question."""
		__hdr__ = (
			('name', '1025s', ''),
			('type', 'H', DNS_A),
			('cls', 'H', DNS_IN)
		)


Some examples:

.. code-block:: python

	DNS_CACHE_FLUSH = 0x8000
	answer = dpkt.dns.DNS.RR(
		type = dpkt.dns.DNS_TXT,
		cls = dpkt.dns.DNS_IN | DNS_CACHE_FLUSH,
		ttl = 200,
		name = 'www.hello.com',
		text = 'Some text')

	ans = dpkt.dns.DNS.RR(
		type = dpkt.dns.DNS_SRV,
		cls = dpkt.dns.DNS_IN | DNS_CACHE_FLUSH,
		ttl = self._response_ttl,
		name = q.name,
		srvname = full_hostname,
		priority = priority,
		weight = weight,
		port = port)
	# The target host (srvname) requires to send an A record with its IP
	# address. We do this as if a query for it was sent.
	q = dpkt.dns.DNS.Q(name=full_hostname, type=dpkt.dns.DNS_A)
	answers = []
	for ip_addr in self._a_records[q.name]:
		answers.append(dpkt.dns.DNS.RR(
			type = dpkt.dns.DNS_A,
			cls = dpkt.dns.DNS_IN | DNS_CACHE_FLUSH,
			ttl = self._response_ttl,
			name = q.name,
			ip = ip_addr))
    [ans] + answers

	MDNS_IP_ADDR = '224.0.0.251'
	MDNS_PORT = 5353
	resp_dns = dpkt.dns.DNS(
		op = dpkt.dns.DNS_AA, # Authoritative Answer.
		rcode = dpkt.dns.DNS_RCODE_NOERR,
		an = answers)
	# This property modifies the "op" field:
	resp_dns.qr = dpkt.dns.DNS_R, # Response.
	sock.send(str(resp_dns), MDNS_IP_ADDR, MDNS_PORT)

DNSLib
-------

class RR
:rclass: ?
:rdlength: ?
:rname: ?
:rtype: ?
:ttl: ?

ARP
---

The Address Resolution Protocol (ARP) is a telecommunication protocol used for resolution
of network layer addresses into link layer addresses, a critical function in
multiple-access networks. ARP is used to convert a network address (e.g. an IPv4 address)
to a physical address such as an Ethernet address (also known as a MAC address).
`wiki <http://en.wikipedia.org/wiki/Address_Resolution_Protocol>__`

If you have the IP address, you can get the MAC address by sending an ARP message with a
broadcast MAC address (FF:FF:FF:FF:FF:FF or 00:00:00:00:00:00 (arping uses)) which every computer will read. Then the
computer with the IP address will respond with its MAC address.

=========== ===============================================
dpkt.arp.*
=========== ===============================================
op          dpkt.arp.ARP_OP_REQUEST,dpkt.arp.ARP_OP_REPLY
sha         Source hardware address
spa         Source protocol address
tha         Target hardware address
tpa         Target protocol address
=========== ===============================================


This doesn't work on Windows and OSX becuase socket.PF_PACKET isn't defined, only on
linux:

.. code-block:: python

	s = socket.socket(socket.PF_PACKET, socket.SOCK_RAW)
	s.bind(('en1', ethernet.ETH_TYPE_ARP))

.. code-block:: python

	my_mac = commands.getoutput("ifconfig " + 'en1' + "| grep ether | awk '{ print $2 }'")
	ans = commands.getoutput('arp -i en1 -l -n 192.168.1.13')

	# bulid an ARP reply
	arp_p = arp.ARP()
	arp_p.sha = eth_aton(src_mac)
	arp_p.spa = socket.inet_aton(src_ip)
	arp_p.tha = eth_aton(dst_mac)
	arp_p.tpa = socket.inet_aton(dst_ip)
	arp_p.op = arp.ARP_OP_REPLY

	packet = ethernet.Ethernet()
	packet.src = eth_aton(so_mac)
	packet.dst = eth_aton(to_mac)
	packet.data = arp_p
	packet.type = ethernet.ETH_TYPE_ARP

You can also use `arping` to find the MAC:

.. code-block:: bashsession

	[kevin@Tardis docs]$ sudo arping -c 3 192.168.1.6
	ARPING 192.168.1.6
	Timeout
	42 bytes from 40:30:04:f0:8c:50 (192.168.1.6): index=0 time=556.113 msec
	42 bytes from 40:30:04:f0:8c:50 (192.168.1.6): index=1 time=164.716 msec

	--- 192.168.1.6 statistics ---
	3 packets transmitted, 2 packets received,  33% unanswered (0 extra)
	rtt min/avg/max/std-dev = 164.716/360.415/556.113/195.698 ms

Or simplify it using other utils::

	sudo arping -c 2 192.168.1.5 | grep bytes | awk '{ print $4 }'


Decoding an ARP Packet
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

	import binascii
	def add_colons_to_mac( mac_addr ) :
		"""This function accepts a 12 hex digit string and converts it to a colon
	separated string"""
		s = list()
		for i in range(12/2) : 	# mac_addr should always be 12 chars, we work in groups of 2 chars
			s.append( mac_addr[i*2:i*2+2] )
		r = ":".join(s)
		return r

	eth = dpkt.ethernet.Ethernet(pkt)

	# should actually double check this is an ARP packet and not assume
	arp = eth.arp
	print "source protocol address", socket.inet_ntoa(arp.spa)
	print "source hardware address", add_colons_to_mac( binascii.hexlify(arp.sha) )
	print "Target protocol address", socket.inet_ntoa(arp.tpa)	#IPv4 address
	print "target hardware address", add_colons_to_mac( binascii.hexlify(arp.tha) )
	arp_cache[arp.spa] = arp.sha
	add_colons_to_mac( binascii.hexlify(arp_cache[ip]))


ICMP
-----

The Internet Control Message Protocol (ICMP) is one of the main protocols of the Internet
Protocol Suite. It is used by network devices, like routers, to send error messages
indicating, for example, that a requested service is not available or that a host or
router could not be reached.

The variable size of the ICMP packet data section has been exploited. In the well-known
"Ping of death," large or fragmented ping packets are used for denial-of-service attacks.
ICMP can also be used to create covert channels for communication, as with the LOKI
exploit.

::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Type      |     Code      |          Checksum             |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                             unused                            |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|      Internet Header + 64 bits of Original Data Datagram      |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

types

- echo reply 0
- destination unreachable 3
- echo request 8
- timestamp 13
- timestamp reply 14


ICMPv6 `RFC4443 <http://www.ietf.org/rfc/rfc4443.txt>`_
----------------------------------------------------------

The ICMPv6 messages have the following general format::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Type      |     Code      |          Checksum             |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                                                               |
	+                         Message Body                          +
	|                                                               |


ICMPv6 messages are grouped into two classes: error messages and
informational messages.  Error messages are identified as such by a
zero in the high-order bit of their message Type field values.  Thus,
error messages have message types from 0 to 127; informational
messages have message types from 128 to 255.

ICMPv6 error message types:

  1    Destination Unreachable
  2    Packet Too Big
  3    Time Exceeded
  4    Parameter Problem

ICMPv6 informational message types:

  128  Echo Request
  129  Echo Reply

ICMPv6 Fields:

Type      1 Destination Unreachable

Code      0 - No route to destination
		  1 - Communication with destination administratively prohibited
		  2 - Beyond scope of source address
		  3 - Address unreachable
		  4 - Port unreachable
		  5 - Source address failed ingress/egress policy
		  6 - Reject route to destination


ICMPv6 Informational Messages

Echo Request Message
~~~~~~~~~~~~~~~~~~~~~

::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Type      |     Code      |          Checksum             |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|           Identifier          |        Sequence Number        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Data ...
	+-+-+-+-+-


   :Type:            128
   :Code:            0
   :Identifier:      An identifier to aid in matching Echo Replies to this Echo Request.  May be zero.
   :Sequence Number: A sequence number to aid in matching Echo Replies to this Echo Request.  May be zero.
   :Data:            Zero or more octets of arbitrary data.

Echo Reply Message
~~~~~~~~~~~~~~~~~~~

::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Type      |     Code      |          Checksum             |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|           Identifier          |        Sequence Number        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Data ...
	+-+-+-+-+-

:Type:            129
:Code:            0
:Identifier:      The identifier from the invoking Echo Request message.
:Sequence Number: The sequence number from the invoking Echo Request message.
:Data:            The data from the invoking Echo Request message.


Neighbor Discovery (ND) protocol for Internet Protocol Version 6 (IPv6)
--------------------------------------------------------------------------

`RFC4861 <http://www.ietf.org/rfc/rfc4861.txt>`_

Nodes (hosts and routers) use Neighbor Discovery to determine the link-layer addresses for
neighbors known to reside on attached links and to quickly purge
cached values that become invalid.  Hosts also use Neighbor Discovery
to find neighboring routers that are willing to forward packets on
their behalf.  Finally, nodes use the protocol to actively keep track
of which neighbors are reachable and which are not, and to detect
changed link-layer addresses.

Neighbor Solicitation Message Format

Nodes send Neighbor Solicitations to request the link-layer address
of a target node while also providing their own link-layer address to
the target.  Neighbor Solicitations are multicast when the node needs
to resolve an address and unicast when the node seeks to verify the
reachability of a neighbor.

::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Type      |     Code      |          Checksum             |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                           Reserved                            |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                                                               |
	+                                                               +
	|                                                               |
	+                       Target Address                          +
	|                                                               |
	+                                                               +
	|                                                               |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|   Options ...
	+-+-+-+-+-+-+-+-+-+-+-+-

    IP Fields:

      Source Address
                     Either an address assigned to the interface from
                     which this message is sent or (if Duplicate Address
                     Detection is in progress [ADDRCONF]) the
                     unspecified address.
      Destination Address
                     Either the solicited-node multicast address
                     corresponding to the target address, or the target
                     address.
      Hop Limit      255

   ICMP Fields:

      Type           135

      Code           0

      Checksum       The ICMP checksum.  See [ICMPv6].

      Reserved       This field is unused.  It MUST be initialized to
                     zero by the sender and MUST be ignored by the
                     receiver.

      Target Address The IP address of the target of the solicitation.
                     It MUST NOT be a multicast address.

   Possible options:

      Source link-layer address
                     The link-layer address for the sender.  MUST NOT be
                     included when the source IP address is the
                     unspecified address.  Otherwise, on link layers
                     that have addresses this option MUST be included in
                     multicast solicitations and SHOULD be included in
                     unicast solicitations.


Neighbor Advertisement Message Format

A node sends Neighbor Advertisements in response to Neighbor
Solicitations and sends unsolicited Neighbor Advertisements in order
to (unreliably) propagate new information quickly.

::

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     Type      |     Code      |          Checksum             |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|R|S|O|                     Reserved                            |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                                                               |
	+                                                               +
	|                                                               |
	+                       Target Address                          +
	|                                                               |
	+                                                               +
	|                                                               |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|   Options ...
	+-+-+-+-+-+-+-+-+-+-+-+-

IP Fields:

:Source Address:  An address assigned to the interface from which the advertisement is sent.

:Destination Address: For solicited advertisements, the Source Address of an
  invoking Neighbor Solicitation or, if the solicitation's Source Address is
  the unspecified address, the all-nodes multicast address. For unsolicited
  advertisements typically the all-nodes multicast address.

:Hop Limit:      255

ICMP Fields:

:Type:           136

:Code:           0

:Checksum:       The ICMP checksum.  See [ICMPv6].

:R:              Router flag.  When set, the R-bit indicates that
			 the sender is a router.  The R-bit is used by
			 Neighbor Unreachability Detection to detect a
			 router that changes to a host.

:S:              Solicited flag.  When set, the S-bit indicates that
			 the advertisement was sent in response to a
			 Neighbor Solicitation from the Destination address.
			 The S-bit is used as a reachability confirmation
			 for Neighbor Unreachability Detection.  It MUST NOT
			 be set in multicast advertisements or in
			 unsolicited unicast advertisements.

:O:            Override flag.  When set, the O-bit indicates that
			 the advertisement should override an existing cache
			 entry and update the cached link-layer address.
			 When it is not set the advertisement will not
			 update a cached link-layer address though it will
			 update an existing Neighbor Cache entry for which
			 no link-layer address is known.  It SHOULD NOT be
			 set in solicited advertisements for anycast
			 addresses and in solicited proxy advertisements.
			 It SHOULD be set in other solicited advertisements
			 and in unsolicited advertisements.

:Reserved:     29-bit unused field.  It MUST be initialized to
			 zero by the sender and MUST be ignored by the
			 receiver.

:Target Address: For solicited advertisements, the Target Address
			 field in the Neighbor Solicitation message that
			 prompted this advertisement.  For an unsolicited
			 advertisement, the address whose link-layer address
			 has changed.  The Target Address MUST NOT be a
			 multicast address.

Possible options:

:Target link-layer address: The link-layer address for the target, i.e., the
			 sender of the advertisement.  This option MUST be
			 included on link layers that have addresses when
			 responding to multicast solicitations.  When
			 responding to a unicast Neighbor Solicitation this
			 option SHOULD be included.

			 The option MUST be included for multicast
			 solicitations in order to avoid infinite Neighbor
			 Solicitation "recursion" when the peer node does
			 not have a cache entry to return a Neighbor
			 Advertisements message.  When responding to unicast
			 solicitations, the option can be omitted since the
			 sender of the solicitation has the correct link-
			 layer address; otherwise, it would not be able to
			 send the unicast solicitation in the first place.
			 However, including the link-layer address in this
			 case adds little overhead and eliminates a
			 potential race condition where the sender deletes
			 the cached link-layer address prior to receiving a
			 response to a previous solicitation.




Multicast DNS (mDNS)
---------------------

The multicast Domain Name System (mDNS) resolves host names to IP addresses within small
networks that do not include a local name server. It is a zero configuration service,
using essentially the same programming interfaces, packet formats and operating semantics
as the unicast Domain Name System (DNS). While it is designed to be stand-alone capable,
it can work in concert with unicast DNS servers.

The mDNS Ethernet frame is a multicast UDP packet to:

* MAC address 01:00:5E:00:00:FB (for IPv4) or 33:33:00:00:00:FB (for IPv6)
* IPv4 address 224.0.0.251 or IPv6 address FF02::FB
* UDP port 5353

You can simulate mDNS request with `dig`:

.. code-block:: bashsession

	[kevin@Tardis test]$ dig -p 5353 @224.0.0.251 calculon.local

	; <<>> DiG 9.8.3-P1 <<>> -p 5353 @224.0.0.251 calculon.local
	; (1 server found)
	;; global options: +cmd
	;; Got answer:
	;; ->>HEADER<<- opcode: QUERY, status: NOERROR, id: 53097
	;; flags: qr aa; QUERY: 1, ANSWER: 1, AUTHORITY: 0, ADDITIONAL: 0

	;; QUESTION SECTION:
	;calculon.local.			IN	A

	;; ANSWER SECTION:
	calculon.local.		10	IN	A	192.168.1.17

	;; Query time: 59 msec
	;; SERVER: 192.168.1.17#5353(224.0.0.251)
	;; WHEN: Thu May 28 12:04:07 2015
	;; MSG SIZE  rcvd: 48


Active Network Mapping
-----------------------

Fast determination if a host is up: UDP - low cost to send packets

1. Send UDP packets to a port on a remote machine
2. Listen for ICMP (error, type=code=3) responses back. An unreachable port error means the host is up

Scan a host for open ports: TCP

1. For each host that is up, start the handshake process:
2. Send a SYN packet to a port
3. Wait for the ACK (port is open) and then send a RST (reset) to close out the process and move on to the next port
4. If you don't get the ACK, then the port is closed


Passive Network Mapping
------------------------

Listen for:

- DNS responses: DNS_A will match names to IPv4 addresses
- mDNS: same as above
- mDNS: DNS_SRV will match services to IPv4 addresses

Resources
---------
- `DNS record types <http://en.wikipedia.org/wiki/List_of_DNS_record_types>`_
- `dpkt doc <http://www.commercialventvac.com/dpkt.html#mozTocId839997>`_
- `dpkt-tutorial-1-icmp-echo <https://jon.oberheide.org/blog/2008/08/25/dpkt-tutorial-1-icmp-echo/>`_
- `dpkt-tutorial-2-parsing-a-pcap-file <https://jon.oberheide.org/blog/2008/10/15/dpkt-tutorial-2-parsing-a-pcap-file/>`_
- `dpkt-tutorial-3-dns-spoofing <https://jon.oberheide.org/blog/2008/12/20/dpkt-tutorial-3-dns-spoofing/>`_
- `IP Protocol headers <http://www.binarytides.com/python-packet-sniffer-code-linux/>`_
- `ICMP message <http://en.wikipedia.org/wiki/Internet_Control_Message_Protocol>`_
- `ARP message <http://en.wikipedia.org/wiki/Address_Resolution_Protocol>`_
- `mDNS message <http://en.wikipedia.org/wiki/Multicast_DNS>`_
- `unixwiz.net <http://unixwiz.net/techtips/iguide-kaminsky-dns-vuln.html>`_
- `IANA DNS types <http://www.iana.org/assignments/dns-parameters/dns-parameters.xhtml#dns-parameters-6>`_
- `Internet Engineering Task Force <http://www.ietf.org>`_


Airplay
--------

http://nto.github.io/AirPlay.html#audio-airportexpressauthentication

Misc
------

.. code-block:: python

	import AppKit
	# Create instance of OS X notification center
	notification_center = AppKit.NSUserNotificationCenter.defaultUserNotificationCenter()
	# Create new notification instance
	notification = AppKit.NSUserNotification.alloc().init()
	notification.setTitle_(title)
	notification.setSubtitle_(subtitle)
	notification.setInformativeText_(content)
	# Deliver OS notifications
	notification_center.deliverNotification_(notification)
