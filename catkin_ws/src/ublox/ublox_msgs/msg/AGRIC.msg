
# uncore bin: AGRIC   header (aa 44 b5)
# 
# use the third sync b5 as CLASS_ID
# This message displays information about uncore ARGIC 
# 
#

uint8 CLASS_ID = 181
uint32 MESSAGE_ID = 11276
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

uint8[4] GNSS
uint8 length            #fix 236 : from gnss to crc
uint8 Year              # UTC time         
uint8 Month
uint8 Day
uint8 Hour
uint8 Minute
uint8 Second
uint8 RTK_Status        # values: 0 1 2 4 5 
uint8 RTK_STATUS_NONE = 0
uint8 RTK_STATUS_PVT = 1
uint8 RTK_STATUS_DIFF = 2
uint8 RTK_STATUS_FIX = 4
uint8 RTK_STATUS_FLOAT = 5
uint8 Heading_Status    # values: 0 4 5 
uint8 Num_GPS_Sta
uint8 Num_BDS_Sta
uint8 Num_GLO_Sta
float32 Baseline_N
float32 Baseline_E
float32 Baseline_U
float32 Baseline_NStd
float32 Baseline_EStd
float32 Baseline_UStd
float32 Heading
float32 Pitch
float32 Roll
float32 Speed
float32 Velocity_of_North
float32 Velocity_of_East
float32 Velocity_of_Up
float32 Xigema_Vx
float32 Xigema_Vy
float32 Xigema_Vz
float64 lat
float64 lon
float64 alt
float64 ECEF_X
float64 ECEF_y
float64 ECEF_Z
float32 Xigema_lat
float32 Xigema_lon
float32 Xigema_alt
float32 Xigema_ECEF_X
float32 Xigema_ECEF_Y
float32 Xigema_ECEF_Z
float64 BASE_lat
float64 BASE_lon
float64 BASE_alt
float64 SEC_lat
float64 SEC_lon
float64 SEC_alt
uint32 GPS_WEEK_SECOND
float32 Diffage
float32 Speed_Heading
float32 Undulation
float32 Remain_float_3
float32 Remain_float_4
uint8 Num_GAL_Sta
uint8 Speed_Type
uint8 Remain_char_3
uint8 Remain_char_4

#end of AGRIC
