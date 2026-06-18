#ifndef _WS2812_H
#define _WS2812_H

/**
 * @brief Initializes the WS2812 LED strip driver.
 *
 * Configures the SPI-based LED strip hardware, subscribes to the application
 * event bus, and restores the last known LED count and color data from
 * persistent storage. Falls back to defaults if no stored values are found.
 *
 * @return true if initialization succeeded, false otherwise.
 */
bool ws2812_Init(void);

/**
 * @brief Returns a pointer to the current LED color data buffer.
 *
 * The buffer is laid out as sequential RGB triplets (one per LED).
 * On success, @p len is set to @c num_leds * 3 bytes.
 *
 * @param len[out] Set to the number of valid bytes in the returned buffer.
 *                 Must not be NULL.
 * @return Pointer to the internal color data array, or NULL if @p len is NULL.
 */
uint8_t *ws2812_GetLedColor(size_t *len);

#endif