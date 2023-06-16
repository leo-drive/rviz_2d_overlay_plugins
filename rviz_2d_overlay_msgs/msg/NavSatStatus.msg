# Navigation Satellite fix status for any Global Navigation Satellite System

# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service

int16 GPS_POSITION_TYPE_NO_SOLUTION = 0
int16 GPS_POSITION_TYPE_UNKNOWN = 1
int16 GPS_POSITION_TYPE_SINGLE = 2
int16 GPS_POSITION_TYPE_PSRDIFF = 3
int16 GPS_POSITION_TYPE_SBAS = 4
int16 GPS_POSITION_TYPE_OMNISTAR = 5
int16 GPS_POSITION_TYPE_RTK_FLOAT = 6
int16 GPS_POSITION_TYPE_RTK_INT = 7
int16 GPS_POSITION_TYPE_PPP_FLOAT = 8
int16 GPS_POSITION_TYPE_PPP_INT = 9
int16 GPS_POSITION_TYPE_FIXED = 10

int16 gps_position_type