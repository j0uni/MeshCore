# T1000-E repeater: 16-byte group PSK as 32 hex chars — never commit. Provide via:
#   export SECRET_GNSS_CHANNEL_KEY_HEX=<32_hex_digits>
#   or gitignored one-line file: .secret_gnss_channel_key
Import("env")

import os


def _key_from_file(project_dir):
    path = os.path.join(project_dir, ".secret_gnss_channel_key")
    if not os.path.isfile(path):
        return ""
    try:
        with open(path, "r", encoding="ascii", errors="ignore") as f:
            return f.readline().strip()
    except OSError:
        return ""


def _normalize_hex(s):
    s = s.strip().lower()
    if s.startswith("0x"):
        s = s[2:]
    return s


raw = os.environ.get("SECRET_GNSS_CHANNEL_KEY_HEX", "").strip()
if not raw:
    raw = _key_from_file(env["PROJECT_DIR"])

key_hex = _normalize_hex(raw)
if len(key_hex) != 32 or any(c not in "0123456789abcdef" for c in key_hex):
    print(
        "\n*** t1000e_repeater: set SECRET_GNSS_CHANNEL_KEY_HEX to exactly 32 hex digits (16-byte PSK),\n"
        "    e.g. export SECRET_GNSS_CHANNEL_KEY_HEX=0123456789abcdef0123456789abcdef\n"
        "    or add gitignored .secret_gnss_channel_key with one line.\n"
    )
    env.Exit(1)

# C string literal for fromHex(..., SECRET_GNSS_CHANNEL_KEY_HEX)
env.Append(
    CPPDEFINES=[
        ("SECRET_GNSS_CHANNEL_KEY_HEX", '\\"' + key_hex + '\\"'),
    ]
)
