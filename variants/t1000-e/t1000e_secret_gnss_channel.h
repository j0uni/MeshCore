#pragma once

/**
 * T1000-E simple_repeater: 16-byte group PSK is injected at build time only.
 * See t1000e_repeater_gnss_channel_key.py — SECRET_GNSS_CHANNEL_KEY_HEX or .secret_gnss_channel_key
 *
 * The mesh region name is not a separate build input: it is derived at runtime as
 * '#' + first 14 bytes of SHA256(PSK) as lowercase hex (28 chars), so listeners can
 * compute the same name from the shared key.
 */

#if defined(T1000E_REPEATER_BUILD)
#ifndef SECRET_GNSS_CHANNEL_KEY_HEX
#error "T1000-E repeater: set SECRET_GNSS_CHANNEL_KEY_HEX (32 hex digits); see t1000e_repeater_gnss_channel_key.py."
#endif

#if defined(__cplusplus)
static_assert(sizeof(SECRET_GNSS_CHANNEL_KEY_HEX) == 33u,
              "SECRET_GNSS_CHANNEL_KEY_HEX must be exactly 32 hex characters as a C string.");
#endif

#endif
