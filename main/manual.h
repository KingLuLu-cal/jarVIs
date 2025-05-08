#ifndef MANUAL_H
#define MANUAL_H

// Function declarations
void echo_task(void *arg);
void start_bt(void);
int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

#endif // MANUAL_H