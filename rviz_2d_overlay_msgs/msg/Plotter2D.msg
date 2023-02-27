uint8 LEFT = 0
uint8 RIGHT = 1
uint8 CENTER = 2
uint8 TOP = 3
uint8 BOTTOM = 4


int32 horizontal_distance # Horizontal distance from left/right border or center, depending on alignment
int32 vertical_distance # Vertical distance between from top/bottom border or center, depending on alignment


# Alignment of the overlay withing RVIZ
uint8 horizontal_alignment # one of LEFT, CENTER, RIGHT
uint8 vertical_alignment # one of TOP, CENTER, BOTTOM

uint8 width
uint8 height

float32 max_value
float32 min_value

std_msgs/ColorRGBA fg_color

string caption
float32 data