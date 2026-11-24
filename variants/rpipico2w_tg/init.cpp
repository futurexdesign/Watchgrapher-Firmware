/*
    Initialize the Pico W WiFi driver

    Copyright (c) 2022 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cyw43_wrappers.h>

// TG-VARIANT-BEGIN
// Project copy of variants/rpipico2w from framework-arduinopico 1.50501.0
// (arduino-pico 5.5.1, commit 275e73d). Only the marked lines differ from the
// original; tests/check_ble_core.sh (CORE-04) strips them and compares the
// rest byte for byte, and compares pins_arduino.h, digital.cpp and
// ../generic/common.h (included by pins_arduino.h) as copies.
//
// Why: the framework variant brings the CYW43 up from initVariant() on core 0.
// The async context of the radio is bound to the core that initialises it
// (plan F-17, F-22), so an env that wants the radio on core 1
// (TG_BLE_CORE=1, plan section 5.8) must not initialise it here. The sketch
// calls init_cyw43_wifi() from core 1 instead (src/ble_core.cpp).
// TG-VARIANT-END
extern "C" void initVariant() {
// TG-VARIANT-BEGIN
#if !(defined(TG_BLE_CORE) && TG_BLE_CORE == 1)
// TG-VARIANT-END
    init_cyw43_wifi();
// TG-VARIANT-BEGIN
#endif
// TG-VARIANT-END
}
