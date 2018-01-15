
Cyber Espionage Tools
========================

:date: 2015-12-23
:summary: Review of Cyber weapons and groups

Stuxnet
	June 2009. Computer worm that attacked SCADA and PLC (programmable logic controllers)
	targeted Iranian nuclear capabilities. The first cyber weapon targeting centerfuges with
	Siemens Step7 software. It was first introduced via USB flash drive and then spread peer-to-peer.

Duqu
	Sept 2011. New worm nearly identical to Stuxnet but with different targets (keystrokes
	and system info). One of Duqu's actions is to steal digital certificates (and corresponding
	private keys, as used in public-key cryptography) from attacked computers to help future
	viruses appear as secure software. Duqu uses a 54×54 pixel jpeg file and encrypted dummy
	files as containers to smuggle data to its command and control center. Duqu got its name
	from the prefix "~DQ" it gives to the names of files it creates. Kaspersky Labs said Duqu
	and Stuxnet both originated from 2007 and both contained a "~d" at the beginning of the
	file, thus naming them part of the Tilded platform.

Flame
	Mar 2012. Attacks Microsoft Windows computers in the Middle East for cyber espionage.
	Flame can spread to other systems over a local network (LAN) or via USB stick. It can
	record audio, screenshots, keyboard activity and network traffic. The program also
	records Skype conversations and can turn infected computers into Bluetooth beacons which
	attempt to download contact information from nearby Bluetooth-enabled devices.[7] This
	data, along with locally stored documents, is sent on to one of several command and
	control servers that are scattered around the world. The program then awaits further
	instructions from these servers. Named because the text "flame" was found in one of its
	modules. Kaspersky Labs analysed the code and said there was a strong relationship between
	Flame and Stuxnet.

Gauss
	2012. Another Duqu, Flame, Stuxnet derivative. Its mission appears to target Lebanese
	financial institutions perhaps to track banking transactions and accounts.

Duqu 2.0
	Jun 2015. This is an upgraded version of the original Duqu code. It lives entirely
	in memory and uses various new encryption techniques not found in previous versions of the
	code base. Because it lives in memory, it can be removed by rebooting, however, if there is
	one infected computer on your network, then it will spread again. Thus you need to reboot
	all computers at once in order to clear your network. This is also notable, because this
	is the first time a network security company, Kaspersky Labs, was the target of an attack.
	The attack appears to be focused on their technology, research and internal processes.

Great Cannon
	Mar 2015. The Great Cannon of China is an attack tool that is used to launch
	distributed denial-of-service attacks on websites by intercepting massive amounts of web
	traffic and redirecting them to targeted websites. While it is co-located with the Great
	Firewall, the Great Cannon is a separate offensive system, with different capabilities
	and design.

Shamoon
	2012. Targeting recent NT kernel-based versions of Microsoft Windows. The virus
	has been used for cyber espionage in the energy sector. The virus has been noted to have
	behaviour differing from other malware attacks, intended for cyber espionage. Shamoon can
	spread from an infected machine to other computers on the network. Once a system is
	infected, the virus continues to compile a list of files from specific locations on the
	system, upload them to the attacker, and erase them. Finally the virus overwrites the
	master boot record of the infected computer, making it unbootable.

Wiper
	2012. Wiper is the section of the Shamoon agent responsible for destroying data on
	the target's hard disk (or similar storage) on systems running Microsoft Windows. Wiper is
	significant on its own, as it appears to have been incorporated into more than one agent,
	is difficult to detect, and resulted in the indirect detection of the Flame agent. The
	name shamoon in fact comes from a substring detected in what appears to be one of Wiper's
	search tables.

================ ===============
Tilded Platform  Flame Platform
================ ===============
Stuxnet 2009      Flame
Stuxnet 2010      Gauss
Duqu
Duqu 2.0
================ ===============


Hacking Groups and Incidents
-----------------------------

LulzSec
	2011.  HBGary Federal attacked by anonymous because Aaron Barr tried to out them.
	LulzSec uncovered questionable activities/tactics of HBGary Fed.Hacked Sony, exposing
	their poor protection (plain text) of PSN user's credit cards numbers, home addresses,
	phone numbers, etc.

AntiSec
	2011. Attacks tied to the passing of Arizona's immigration reform bill. They
	attacked US law enforcement around the country and Stratfor, a US based intelligence
	company.

Lizard Squad
	Dec 2014. Another off shoot of Anonymous attacked gaming services
	(XBox Live & PSN) over Christmas.

Equation Group
	2015. Believed to have existed since 2001 and really part of the NSA, or
	at least tied to them. Kaspersky Labs has identified numerous malware created by them,
	EquationDrug, EquationLaser and GrayFish. These are capable of reprogramming hard disk
	drive firmware (both HDD and SDD) which is non-trivial.

Animal Farm
	See arstechnica.com

United States' Office of Personnel Management (OPM)
	Jun 2015. Largest US gov't hack exposing PII of gov't workers.




References
-----------

- https://en.wikipedia.org/wiki/Stuxnet
- https://securelist.com/blog/research/70504/the-mystery-of-duqu-2-0-a-sophisticated-cyberespionage-actor-returns/
