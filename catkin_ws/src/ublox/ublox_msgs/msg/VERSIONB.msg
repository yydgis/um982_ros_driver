# uncore bin: VERSIONB   header (aa 44 b5)
# 
# use the third sync b5 as CLASS_ID
# This message displays information about uncore ARGIC 
# 
#

uint8 CLASS_ID = 181
uint32 MESSAGE_ID = 37
uint8 CPUIDle
uint16 messageId
uint16 messageLen
uint8 TimeRef
uint8 TimeStatus
uint16 Wn
uint32 Ms
uint32 Res
uint8 Version
uint8 Leap_sec
uint16 DelayMs
#end of header 

uint32 pn_type
uint8[33] sw_version
uint8[129] model
uint8[66] Psn
uint8[33] efuse_ID
uint8[43] comp_time
#crc32

# 308+24
