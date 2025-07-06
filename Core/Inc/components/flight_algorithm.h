#include <stdbool.h>
#include "system_definitions.h"
#include "system_types.h"

/// @brief (0) Main algorithm for system init
void initialize_system();

/// @brief (1) Main algorithm for initiating flight
void start_flight();

/// @brief (2) Main algorithm for apogy
void apogy();

/// @brief (3) Main algorithm for landing
void landing();

void buzz();

void read_sensors();

/// @brief Open rescue system
void open_rescue();

/// @brief Check if rescue system is open.
/// @return 1 if success, 0 otherwise.
bool check_rescue();

// Вернёт 1, если апогей
bool check_apogy();
// Вернёт 1, если приземление
bool check_landing();
// Возвращает текущую высоту от уровня моря
float get_altitude(float pressure, float temperature);

SystemState get_sys_state();

// // Сохраняет начальную высоту
// void get_start_height();
