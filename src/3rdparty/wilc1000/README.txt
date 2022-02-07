WILC1000 firmware image is taken from the latest distribution and converted
into a raw binary file.

Prerequisites:
	Grab the latest version of ASF (version 3.51.0 as of Jan 2022):
	  https://www.microchip.com/en-us/tools-resources/develop/libraries/advanced-software-framework
	Grab 'zopfli' from:
	  https://github.com/google/zopfli
	If your system does not have a hexdump command, grab one from somewhere.

Extract ASF somewhere and grab the following file to the same place as
this README.txt file:
 <ASF-dir>/common/components/wifi/wilc/driver/include/wifi_firmware_1000b.h

Run 'make'

Finally, edit the generted header (wilc1000_fw.h) to add two #defines, as
follows:

   #define WILC_FW_BUILD_DATE	"Dec 21 2020"
   #define WILC_FW_BUILD_TIME	"22:19:00"

The actual date/time can be extracted from the firmware by running 'strings'
on wifi_firmware_1000b.bin and searching for 'Build Time'. The date/time
should be visibile around there or thereabouts.
