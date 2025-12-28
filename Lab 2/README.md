# Issues

The sensor currently returns an error code causing the sensor reading to fail every time on "wake up". To try to assess it, I attempted to scan every available address, but it turned up emtpy.

```c
static void i2c_scan(i2c_master_bus_handle_t bus)
{
    printf("I2C scan:\n");
    for (int addr = 0x03; addr <= 0x77; addr++) {
        esp_err_t rc = i2c_master_probe(bus, addr, I2C_MASTER_TIMEOUT_MS);
        if (rc == ESP_OK) printf("  - Found device at 0x%02X\n", addr);
    }
    printf("Scan done.\n");
}
```

This shows me the address of 0x70 isn't connected as intended.