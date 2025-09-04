import uasyncio as asyncio
import pheripherals

def map_value(x, in_min, in_max, out_min, out_max):
    if in_min == in_max:
        return None

    ratio = (x - in_min) / (in_max - in_min)
    try:
        return (out_min + (ratio * (out_max - out_min)))
    except:
        return None
    
class SharedSimpleData:
    def __init__(self, unit='', name=''):
        self.value = None
        self.lock = asyncio.Lock()
        self.unit = unit
        self.name = name
    
    async def set(self, value):
        async with self.lock:
            self.value = value
    
    async def get(self):
        async with self.lock:
            try:
                val = round(self.value, 2)
                return f"{val:.2f}"
            except:
                return None
    
    async def get_name(self):
        return self.name
    
    async def get_unit(self):
        return self.unit

class SharedMappedData:
    def __init__(self, cfg, name=''):
        self.raw_value = 0
        self.lock = asyncio.Lock()
        self.cfg = cfg
        
        self.raw_min = self.cfg['map'].get('raw_min', 0)
        self.raw_max = self.cfg['map'].get('raw_max', 100)
        self.out_min = self.cfg['map'].get('out_min', 0)
        self.out_max = self.cfg['map'].get('out_max', 100)

        self.unit = self.cfg['map'].get('unit', '')
        self.name = name

    async def set(self, raw_value):
        async with self.lock:
            self.raw_value = raw_value

    async def get(self):
        async with self.lock:
            try:
                val = round((map_value(self.raw_value, self.raw_min, self.raw_max, self.out_min, self.out_max)), 2)
                return f"{val:.2f}"
            except:
                return None 

    async def get_name(self):
        return self.name

    async def get_unit(self):
        return self.unit

class SharedBinaryData:
    def __init__(self, name=''):
        self.state = None
        self.lock = asyncio.Lock()
        self.name = name

    async def set(self, state: bool):
        async with self.lock:
            self.state = state
    
    async def get_name(self):
        return self.name
    
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
def SharedData_Init(config_data, dht_config_data):
    global pcb_temp
    try:
        cfg_name = config_data['PCBTemp']['name']
        pcb_temp = SharedSimpleData(unit='°C', name=cfg_name)
        for i in range(1, 5):
            cfg = config_data['AIInt'][f'AI{i}']
            cfg_name = cfg['name']
            adci[i] = SharedMappedData(cfg, name=cfg_name)
        for i in range(1, 5):
            cfg = config_data['AIExt'][f'AI{i}']
            cfg_name = cfg['name']
            adce[i] = SharedMappedData(cfg, name=cfg_name)
        for i in range(1, 17):
            cfg = config_data['DI'][f'DI{i}']
            cfg_name = cfg['name']
            if pheripherals.di[i].get_type() == 'counter':
                din[i] = SharedMappedData(cfg, name=cfg_name)
            else:
                din[i] = SharedBinaryData(name=cfg_name)
        for i in range(1, 17):
            cfg = config_data['DO'][f'DO{i}']
            cfg_name = cfg['name']
            if pheripherals.do[i].get_type() == 'pwm':
                dout[i] = SharedSimpleData('%', name=cfg_name)
            else:
                dout[i] = SharedBinaryData(name=cfg_name)
        for i in range(1, 16):
            cfg = dht_config_data[f'DHT{i}']['temperature']
            cfg_name = cfg['name']
            dht_temp[i] = SharedSimpleData(unit='°C', name=cfg_name)
        for i in range(1, 16):
            cfg = dht_config_data[f'DHT{i}']['humidity']
            cfg_name = cfg['name']
            dht_hum[i] = SharedSimpleData(unit='%', name=cfg_name)

        pheripherals.boot_print("[OK] Data Init")
    except Exception as e:
        pheripherals.boot_print(f"[ERR] Data Init: {e}")


