MSP430 BSL Example Source Code
Lane Westlund
Updated: 5/18/2011

This directory contains the files needed to modify and create a custom 5/6xx
Bootstrap Loader (BSL).  The directories are ordered in such a way that
files are shared between the BSLs where they can be reused.  Specific device 
BSLs will include the required BSL files, as well as their own device specific 
files.

Note: The devices listed in the project names are simply representative of the 
family supported by this BSL.  See the device specific datasheet to know which 
devices belong to a family.

Note: The BSLs built from the projects here will not be identical to those shipped 
in MSP430 devices.  The source provided here is always the newest snapshot, and is 
what should be used for new custom BSLs at the time.  Image files of shipping BSLs 
are provided in a separate directory.

Note: The directory structure used is not a requirement.  It is only done 
in this way to demonstrate code reusability between various BSLs.  A custom-made 
BSL project could simply have all files in a single directory.


Change Log:
------------------------------------------------
4/26/2011
- L.Westlund - Source Snapshot update
5/18/2011
- L.Westlund - Initial Version
------------------------------------------------

IAR version used:
IAR Assembler for MSP430
  5.20.1 (5.20.1.50214)
  C:\Program Files\IAR Systems\Embedded Workbench 6.0 Kickstart - V5.20\430\bin\a430.exe
  11/29/2010 8:32:18 PM, 2081280 bytes
  
IAR C/C++ Compiler for MSP430
  5.20.1 [Kickstart] (5.20.1.30214)
  C:\Program Files\IAR Systems\Embedded Workbench 6.0 Kickstart - V5.20\430\bin\icc430.exe
  11/29/2010 8:41:16 PM, 15781888 bytes