A very quick and dirty uart to mqtt bridge for a DalySmart bms.

Currently supports:
 * State of charge (%)
 * Pack voltage
 * Cell Voltage (mV)
 * Temperature sensors.
 * Charge/Discharge current
 * Residual charge (mAh)
 * Min/Max Cell voltage (gives odd values)
 * MOS state (charge/discharge)
 * Pack state

Set preferences in `main/user_config.h`.
