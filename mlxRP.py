
#!/usr/bin/env python3

from smbus2 import SMBus
from mlx90614 import MLX90614
import time

if __name__ == "__main__":
    bus = SMBus(1)
    sensor = MLX90614(bus, address=0x5A)
    while True:
        ambient = int(sensor.get_ambient())
        object = int(sensor.get_object_1())
        print(f"ambient: {ambient} target: {object}")
        time.sleep(0.5)
    bus.close()
