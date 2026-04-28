# Flashing

Build:

```bash
pio run -e esp32-c3
```

OTA flash to base station:

```bash
curl --fail-with-body -F "update=@.pio/build/esp32-c3/firmware.bin;filename=firmware.bin" http://xxx.xxx.xxx.xxx/update
```

Verify:

```bash
curl http://xxx.xxx.xxx.xxx/status
```

USB fallback:

```bash
pio run -e esp32-c3 -t upload
```
