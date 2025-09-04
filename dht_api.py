from dht import DHT22
import uasyncio as asyncio
import pheripherals
import shared_data

class SafeDHT:
    def __init__(self, pin_id: int):
        self.sensor = DHT22(pheripherals.dio[pin_id])
        self.temp = None
        self.hum = None
        self._measure()
        if self.temp == None and self.hum == None:
            raise Exception("Not responding")   
    
    def _measure(self):
        self.sensor.measure()
        self.temp = self.sensor.temperature()
        self.hum = self.sensor.humidity()
    
    def get_sensor_data(self):
        self._measure()
        return self.temp, self.hum

dht = {}        
def DHT_Init(dht_config_data):
    try:
        for i in range(1, 16):
            if dht_config_data[f'DHT{i}']['enable']:
                pin_name = dht_config_data[f'DHT{i}']['pin_name']
                pin_index = int(pin_name[3:])
                dht[i] = SafeDHT(pin_index)
                pheripherals.boot_print(f"[OK] DHT{i} Init")
            else:
                dht[i] = None
    except:
        pheripherals.boot_print("[ERR] DHT Init")


async def measure_dht_task(delay: float = 5):
    for idx in range(1, 16):
        if dht[idx]:
            temp, hum = dht[idx].get_sensor_data()
            await shared_data.dht_temp[idx].set(temp)
            await shared_data.dht_hum[idx].set(hum)
    while True:
        for idx in range(1, 16):
            if dht[idx]:
                temp, hum = dht[idx].get_sensor_data()
                await shared_data.dht_temp[idx].set(temp)
                await shared_data.dht_hum[idx].set(hum)
                await asyncio.sleep(delay)