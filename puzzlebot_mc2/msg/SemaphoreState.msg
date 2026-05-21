# SemaphoreState.msg
# Estado actual del semaforo detectado por el nodo traffic_light_detector.

uint8 STATE_UNKNOWN = 0
uint8 STATE_GREEN   = 1
uint8 STATE_YELLOW  = 2
uint8 STATE_RED     = 3

std_msgs/Header header
uint8   state              # uno de los STATE_* de arriba
float32 confidence         # area del blob ganador / area total imagen (0-1)
uint32  blob_pixel_area    # area cruda del blob ganador en pixeles
