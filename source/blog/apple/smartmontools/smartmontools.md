---
title: `smartmontools` to Monitor Drives
date: 3 Jul 2021
---

Install the tools with `brew`:

```
brew install smartmontools
```

Right now, I only have my internal SSD (`disk0`):

```
% diskutil list
/dev/disk0 (internal, physical):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      GUID_partition_scheme                        *251.0 GB   disk0
   1:                        EFI ⁨EFI⁩                     314.6 MB   disk0s1
   2:                 Apple_APFS ⁨Container disk1⁩         250.7 GB   disk0s2

/dev/disk1 (synthesized):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      APFS Container Scheme -                      +250.7 GB   disk1
                                 Physical Store disk0s2
   1:                APFS Volume ⁨Logan - Data⁩            198.0 GB   disk1s1
   2:                APFS Volume ⁨Preboot⁩                 449.2 MB   disk1s2
   3:                APFS Volume ⁨Recovery⁩                620.0 MB   disk1s3
   4:                APFS Volume ⁨VM⁩                      4.3 GB     disk1s4
   5:                APFS Volume ⁨Logan⁩                   15.3 GB    disk1s5
   6:              APFS Snapshot ⁨com.apple.os.update-...⁩ 15.3 GB    disk1s5s1
```

Find out the disk info with:

```
% smartctl --all /dev/disk0
smartctl 7.2 2020-12-30 r5155 [Darwin 20.5.0 x86_64] (local build)
Copyright (C) 2002-20, Bruce Allen, Christian Franke, www.smartmontools.org

=== START OF INFORMATION SECTION ===
Model Number:                       APPLE SSD AP0256J
Serial Number:                      123456789
Firmware Version:                   14.17.01
PCI Vendor/Subsystem ID:            0x106b
IEEE OUI Identifier:                0x000502
Controller ID:                      0
NVMe Version:                       <1.2
Number of Namespaces:               2
Local Time is:                      Sat Jul  3 09:51:47 2021 EDT
Firmware Updates (0x02):            1 Slot
Optional Admin Commands (0x0004):   Frmw_DL
Optional NVM Commands (0x0004):     DS_Mngmt
Maximum Data Transfer Size:         256 Pages

Supported Power States
St Op     Max   Active     Idle   RL RT WL WT  Ent_Lat  Ex_Lat
 0 +     0.00W       -        -    0  0  0  0        0       0

=== START OF SMART DATA SECTION ===
SMART overall-health self-assessment test result: PASSED

SMART/Health Information (NVMe Log 0x02)
Critical Warning:                   0x00
Temperature:                        36 Celsius
Available Spare:                    90%
Available Spare Threshold:          2%
Percentage Used:                    3%
Data Units Read:                    71,099,081 [36.4 TB]
Data Units Written:                 59,342,615 [30.3 TB]
Host Read Commands:                 642,862,982
Host Write Commands:                601,427,780
Controller Busy Time:               0
Power Cycles:                       59,431
Power On Hours:                     242
Unsafe Shutdowns:                   10
Media and Data Integrity Errors:    0
Error Information Log Entries:      0

Read 1 entries from Error Information Log failed: GetLogPage failed: system=0x38, sub=0x0, code=745
```

- **Percentage Used:** this give the lifespan of the drive; so here I have used 3%
- **Available Spare:** Contains a normalized percentage (0 to 100%) of the remaining spare capacity available

# References

- [How to Check the Health of SSD in macOS](https://www.maketecheasier.com/check-ssd-health-macos/)
- [Kingston SMART Attributes](https://media.kingston.com/support/downloads/MKP_521.6_SMART-DCP1000_attribute.pdf)
