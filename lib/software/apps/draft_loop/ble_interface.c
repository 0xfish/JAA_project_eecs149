#include "ble_interface.h"


/*Initializes the BLE for the ROMI.*/
void setup_ble(enum STATES *st) {
  // Main application state
  simple_ble_app_t* simple_ble_app;
  simple_ble_app = simple_ble_init(&ble_config);
  simple_ble_add_service(&safety_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(*st), (uint32_t*)st,
      &safety_service, &driving_state_char);

  // Start Advertising
  simple_ble_adv_only_name();
}
