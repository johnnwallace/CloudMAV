#include "esp_camera.h"

void ws_register_connection_callback(void (*callback)(bool connected));
void ws_send_image(const camera_fb_t* im);
void socket_init();