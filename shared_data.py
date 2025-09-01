import uasyncio as asyncio
import pheripherals

def map_value(x, in_min, in_max, out_min, out_max):
    if in_min == in_max:
        return None

    ratio = (x - in_min) / (in_max - in_min)
    return out_min + (ratio * (out_max - out_min))

class SharedSimpleData:
    def __init__(self, unit=''):
        self.value = None
        self.lock = asyncio.Lock()
        self.unit = unit

    async def set(self, value):
        async with self.lock:
            self.value = value
    
    async def get(self):
        async with self.lock:
            return self.value
    
    async def get_unit(self):
        return self.unit

class SharedMappedData:
    def __init__(self, cfg):
        self.raw_value = 0
        self.lock = asyncio.Lock()
        self.cfg = cfg
        
        self.raw_min = self.cfg['map'].get('raw_min', 0)
        self.raw_max = self.cfg['map'].get('raw_max', 100)
        self.out_min = self.cfg['map'].get('out_min', 0)
        self.out_max = self.cfg['map'].get('out_max', 100)

        self.unit = self.cfg['map'].get('unit', '')

    async def set(self, raw_value):
        async with self.lock:
            self.raw_value = raw_value

    async def get(self):
        async with self.lock:
            return map_value(self.raw_value, self.raw_min, self.raw_max, self.out_min, self.out_max)

    async def get_unit(self):
        return self.unit

class SharedBinaryData:
    def __init__(self):
        self.state = None
        self.lock = asyncio.Lock()

    async def set(self, state: bool):
        async with self.lock:
            self.state = state
    
    async def get(self):
        async with self.lock:
            return self.state
         
pcb_temp = None
adci = {}
adce = {}
din = {}
dout = {}
dht_temp = {}
dht_hum = {}
def SharedData_Init(config_data):
    global pcb_temp
    try:
        pcb_temp = SharedSimpleData(unit='°C')
        for i in range(1, 5):
            cfg = config_data['AIInt'][f'AI{i}']
            adci[i] = SharedMappedData(cfg)
        for i in range(1, 5):
            cfg = config_data['AIExt'][f'AI{i}']
            adce[i] = SharedMappedData(cfg)
        for i in range(1, 17):
            cfg = config_data['DI'][f'DI{i}']
            if pheripherals.di[i].get_type() == 'counter':
                din[i] = SharedMappedData(cfg)
            else:
                din[i] = SharedBinaryData()
        for i in range(1, 17):
            cfg = config_data['DO'][f'DO{i}']
            if pheripherals.do[i].get_type() == 'pwm':
                dout[i] = SharedSimpleData()
            else:
                dout[i] = SharedBinaryData()
        for i in range(1, 16):
            dht_temp[i] = SharedSimpleData(unit='°C')
        for i in range(1, 16):
            dht_hum[i] = SharedSimpleData(unit='%')

        pheripherals.boot_print("[OK] Data Init")
    except Exception as e:
        pheripherals.boot_print(f"[ERR] Data Init: {e}")

