# The state of an object estimate used to track
# states smaller than 0 disable all updates

int8 UNKNOWN = 0
int8 PENDING = 1
int8 ACTIVE  = 2
int8 CONFIRMED = -1
int8 DISCARDED = -2
int8 APPROACHING = -3

int8 state
