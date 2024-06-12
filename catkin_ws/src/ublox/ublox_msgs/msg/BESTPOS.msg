
# uncore bin: bestops   header (aa 44 12)
# 
# use the third sync 12 as CLASS_ID
# This message displays information about uncore bestpos bin   
# 
#

uint8 CLASS_ID = 18
uint32 MESSAGE_ID = 42
uint8 UM_BIN_HEAD_LEN = 28
uint8 headlen           #fix value 0x1c
uint16 messageId         #base on different message 
uint8 messageType
uint8 reserved1         # Reserved 1 byte
uint16 messageLen       # message len
uint16 reserved2         # Reserved 2 bytes
uint8 idle
uint8 timeStaus 
uint16 gpsWeek
uint32 iTOW                # GPS time of week of the navigation epoch. [ms]
uint32 reserved3         # Reserved 4 bytes
uint16 bdOffsetGps         # 
uint16 reserved4         # Reserved 2 bytes
### end of header 

uint32 sol_status
uint32 pos_type
float64 lat
float64 lon
float64 hgt
float32 undulation  
uint32  datumId
float32 lat_std
float32 lon_std
float32 hgt_std
uint8[4] stn_id
float32 diff_age
float32 sol_age
uint8 SVs
uint8 solnSVs
uint8[3] Reserved
uint8 ext_sol_stat
uint8 Galileo_sig_mask
uint8 GGB_sig_mask
##crc 4 byte
#end of bestpos


#sol status
uint32 SOL_COMPUTED = 0
uint32 INSUFFICIENT_OBS = 1
uint32 NO_CONVERGENCE = 2
uint32 COV_TRACE = 4
# pos type
uint32 NONE = 0
uint32 FIXEDPOS = 1 
uint32 FIXEDHEIGHT = 2 
uint32 DOPPLER_VELOCITY = 8
uint32 SINGLE = 16
uint32 PSRDIFF = 17
uint32 WAAS_SBAS = 18
uint32 L1_FLOAT_L1 = 32
uint32 IONOFREE_FLOAT = 33
uint32 NARROW_FLOAT = 34
uint32 L1_INT_L1 = 48
uint32 WIDE_INT = 49
uint32 NARROW_INT = 50 
uint32 INS = 52
uint32 INS_PSRSP = 53
uint32 INS_PSRDIFF = 54
uint32 INS_RTKFLOA = 55
uint32 INS_RTKFIXED = 56
uint8 UB_BIN_HEAD_LEN = 28
