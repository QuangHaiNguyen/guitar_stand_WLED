/**
 * @file wifi_manager.h
 * @brief WiFi Manager module for handling WiFi connections.
 *
 * This module provides functions to initialize, configure, and manage
 * WiFi connections for the application.
 *
 * @author Hai Nguyen
 * @date 2025.05.04
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Initializes the WiFi Manager.
 *
 * This function sets up the WiFi Manager, including initializing
 * necessary resources and configurations.
 *
 * @return true if initialization was successful, false otherwise.
 */
bool wifi_manager_Init(void);


bool wifi_manager_StartCaptivePortal(void);

/**
 * @brief Connects to a WiFi network.
 *
 * @param ssid The SSID of the WiFi network.
 * @param password The password of the WiFi network.
 * @return true if the connection was successful, false otherwise.
 */
bool wifi_manager_connect(const char *ssid, const char *password);

/**
 * @brief Disconnects from the current WiFi network.
 */
void wifi_manager_disconnect(void);

/**
 * @brief Checks the connection status.
 *
 * @return true if connected to a WiFi network, false otherwise.
 */
bool wifi_manager_is_connected(void);


#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H