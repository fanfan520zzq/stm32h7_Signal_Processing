#ifndef KNOWN_MODEL_H
#define KNOWN_MODEL_H

#include <stdint.h>

/**
 * @brief Calculate and set the required AD9833 amplitude and frequency to achieve 
 *        the target output voltage at the end of the known model circuit.
 * 
 * @param freq_hz    Target frequency in Hz (100 - 3000)
 * @param target_vpp Target peak-to-peak voltage in Volts (1.0 - 2.0)
 */
void KnownModel_Update(uint32_t freq_hz, float target_vpp);

#endif // KNOWN_MODEL_H
