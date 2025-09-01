from machine import SPI, I2C, UART, WDT
from pyb import Pin, Timer, CAN, RTC, ADC
import network, ntptime, utime, uos, ujson
import uasyncio as asyncio

from drivers.lm75 import LM75
from drivers.ads1115 import ADS1115
from drivers.sdcard import SDCard
from drivers.sh1106 import SH1106_I2C

def boot_print(text):
    print(text)
    try:
        pass # boot_disp.println(text)
    except:
        pass
    if '[ERR]' in text:
        utime.sleep(5)

def alert_print(text, timeout=10):
    print(text)
    if disp is not None:
        try:
            asyncio.create_task(disp.popup(text, timeout))
        except:
            pass

def disp_clear():
    boot_disp.fill(0)
    boot_disp.show()

################################################################
# OLED Display Init
class SafeOLED:
    def __init__(self, base_disp, i2c_lock, line_height=8):
        self.oled = base_disp
        self._lock = i2c_lock
        self.width = base_disp.width
        self.height = base_disp.height
        self.line_height = line_height
        self.lines = self.height // line_height

        self._buffer = ["" for _ in range(self.lines)]
        self._popup_active = False
        self._popup_task = None

    async def write_line(self, line: int, text: str):
        if not (0 <= line < self.lines):
            return
        
        async with self._lock:
            try:
                old = self._buffer[line]
                new = str(text)
                if old != new and not self._popup_active:
                    self.oled.fill_rect(0, line * self.line_height, self.width, self.line_height, 0)
                    self.oled.text(new, 0, line * self.line_height)
                    self.oled.show()
                    self._buffer[line] = new
            except:
                print("[ERR] OLED write_line")

    async def write_lines_dict(self, lines_dict: dict):
        async with self._lock:
            try:
                for row, text in lines_dict.items():
                    if 0 <= row < self.lines:
                        self._buffer[row] = str(text)

                if not self._popup_active:
                    self.oled.fill(0)
                    for i, text in enumerate(self._buffer):
                        self.oled.text(text, 0, i * self.line_height)
                    self.oled.show()
            except:
                print("[ERR] OLED write_lines_dict")

    async def refresh(self):
        async with self._lock:
            try:
                self.oled.fill(0)
                if not self._popup_active:
                    for i, text in enumerate(self._buffer):
                        self.oled.text(text, 0, i * self.line_height)
                self.oled.show()
            except:
                print("[ERR] OLED refresh")

    async def popup(self, text: str, timeout: float = None):
        if self._popup_task is not None:
            self._popup_task.cancel()
            self._popup_task = None

        async with self._lock:
            try:
                self._popup_active = True
                self.oled.fill(0)
                self.oled.text(str(text), 0, 0)
                self.oled.show()
            except:
                print("[ERR] OLED popup")

        if timeout is not None:
            self._popup_task = asyncio.create_task(self._popup_timeout_task(timeout))

    async def _popup_timeout_task(self, timeout: float):
        try:
            await asyncio.sleep(timeout)
            await self.popup_clear()
        except asyncio.CancelledError:
            pass
        finally:
            self._popup_task = None

    async def popup_clear(self):
        need_refresh = False
        async with self._lock:
            if self._popup_active:
                self._popup_active = False
                need_refresh = True

        if need_refresh:
            await self.refresh()  

boot_disp = None
disp = None
def Disp_Init():
    global boot_disp, disp
    try:
        boot_disp = SH1106_I2C(128, 64, i2c1, rotate=180)
        
        if boot_disp.begin():
            boot_print("[OK] Display Init")
            boot_disp.init_display()
        else:
            raise TypeError("Disp init Fail")
        disp = SafeOLED(boot_disp, i2c1_lock)    
    except:
        boot_print("[ERR] Display Init")

'''
example:
lines  = {
        0: "Hello",
        1: "World",
        2: "Doplnok",
        3: "Riadok 4"
    }
await pheripherals.disp.write_lines_dict(lines)
or
await pheripherals.disp.write_line(1, f'cnter: {counter}')
or
await pheripherals.disp.popup('test', 5)
'''

################################################################
# Load json files
pinmap_data = None
def PinMap_Init():
    global pinmap_data
    try:
        with open ('/flash/pinmap.json', 'r') as f:
            pinmap_data = ujson.load(f)
            boot_print("[OK] Load pinmap_data")
    except Exception as e:
        boot_print("[ERR] Load pinmap_data: ", e)

################################################################
# I2C1 for OLED display and PCB temp sensor (LM75)
i2c1 = None
i2c1_lock = None
def I2C1_Init():
    global i2c1, i2c1_lock
    try:
        i2c1 = I2C(1, freq=100000)			# sck = B8; sda = B9
        i2c1_lock = asyncio.Lock()
        boot_print("[OK] I2C1 Init")
    except Exception as e:
        boot_print("[ERR] I2C1 Init:", e)

################################################################
# Status LED Init
status_led = None
def StatusLED_Init():
    global status_led
    try:
        status_led = Pin(pinmap_data['FUNC_IO']['STATUS_LED'], Pin.OUT_PP, value=False)
        boot_print("[OK] Status LED Init")
    except Exception as e:
        boot_print("[ERR] Status LED Init:", e)

################################################################
# UART3 for WIFI modul ESP-01S
uart3 = None
uart3_en = None
uart3_lock = None
def UART3_Init():
    global uart3, uart3_en, uart3_lock
    try:
        uart3 = UART(3, baudrate=115200, bits=8, parity=None, stop=1)		# tx = D8; rx = D9
        uart3_en = Pin(pinmap_data['UART3']['EN'], Pin.OUT_PP, value=False)
        uart3_lock = asyncio.Lock()
        boot_print("[OK] UART3 Init")
    except Exception as e:
        boot_print("[ERR] UART3 Init:", e)

################################################################
# UART8 for internal RS485
uart8 = None
uart8_dir = None
uart8_lock = None
def UART8_Init():
    global uart8, uart8_dir, uart8_lock
    try:
        uart8 = UART(8, baudrate=115200)    	# tx = E1; rx = E0
        uart8_dir = Pin(pinmap_data['UART8']['DIR'], Pin.OUT_PP, value=False)
        uart8_lock = asyncio.Lock()
        boot_print("[OK] UART8 Init")
    except Exception as e:
        boot_print("[ERR] UART8 Init:", e)

################################################################
# UART2 not used
uart2 = None
uart2_lock = None
def UART2_Init(config_data):
    global uart2, uart2_lock
    try:
        if config_data['uart2']['enable']:
            uart2_lock = asyncio.Lock()
            uart2 = UART(2, baudrate=config_data['uart2']['baud'])  		# tx = D5; rx = D6; cts = D3; rts = D4
            boot_print("[OK] UART2 Init")
    except Exception as e:
        boot_print("[ERR] UART2 Init:", e)

################################################################
# UART5 for external RS485
uart5 = None
uart5_dir = None
uart5_lock = None
def UART5_Init(config_data):
    global uart5, uart5_dir, uart5_lock
    try:
        if config_data['uart5']['enable']:
            uart5 = UART(5, baudrate=config_data['uart5']['baud'])  		# tx = B6; rx = B12
            uart5_dir = Pin(pinmap_data['UART5']['DIR'], Pin.OUT_PP, value=False)
            uart5_lock = asyncio.Lock()
            boot_print("[OK] UART5 Init")
    except Exception as e:
        boot_print("[ERR] UART5 Init:", e)

################################################################
# CAN1 for external CAN
can1 = None
can1_lock = None
def CAN1_Init(config_data):
    global can1, can1_lock
    try:
        if config_data['can1']['enable']:
            bitrate = config_data['can1']['bitr']
            can1 = CAN(1, CAN.LOOPBACK)				# CAN_tx = D1; CAN_rx = D0
            can1_lock = asyncio.Lock()
            boot_print("[OK] CAN1 Init")
    except Exception as e:
        boot_print("[ERR] CAN1 Init:", e)

################################################################
# WDT Init
wdt = None
def WDT_Init():
    global wdt
    try:
        wdt = WDT(timeout=10000)
        boot_print("[OK] WDT Init")
    except Exception as e:
        boot_print("[ERR] WDT Init:", e)


################################################################
# DIO Init
dio = {}
def DIO_Init(): 
    try:
        for i in range(1, 16):
            pin_name = pinmap_data['DIO_pins_map'][f'DIO{i}']
            dio[i] = Pin(pin_name)
        boot_print("[OK] DIO Init")
    except Exception as e:
        boot_print("[ERR] DIO Init:", e)


################################################################
# Ethernet for MQTT communication
eth = None
def ETH_Init(config_data):
    global eth
    try:
        eth = network.LAN()
        eth.active(True)
        ip_address =	config_data['eth']['ip']
        if ip_address != "DHCP":
            subnet =		config_data['eth']['subnet']
            gateway = 		config_data['eth']['gateway']
            dns = 			config_data['eth']['dns']
            eth.ifconfig((ip_address, subnet, gateway, dns))
        attempts = 0
        if not eth.isconnected():
            boot_print('[INFO] ETH Connecting..')
            while not eth.isconnected() and attempts < 15:
                utime.sleep(1)
                attempts += 1

            if not eth.isconnected():
                raise Exception('Timeout')
        boot_print("[OK] ETH Init")
        boot_print("[INFO] IP:")
        boot_print(f" {eth.ifconfig()[0]}")
    except:
        boot_print("[ERR] ETH Init")


################################################################ 
# TIM1 for DO PWM (DO1 - DO4)
tim1 = None
def Timer1_Init(config_data):
    global tim1
    try:
        tim1 = Timer(1, freq=config_data['timer1']['freq'])
        boot_print("[OK] Timer1 Init")
    except Exception as e:
        boot_print("[ERR] Timer1 Init:", e)


################################################################
# TIM3 for DO PWM (DO9 - DO12)
tim3 = None
def Timer3_Init(config_data):
    global tim3
    try:
        tim3 = Timer(3, freq=config_data['timer3']['freq'])
        boot_print("[OK] Timer3 Init")
    except Exception as e:
        boot_print("[ERR] Timer3 Init:", e)


################################################################
# DO Init
class SafeDO:
    def __init__(self, pin_id: int, config_data, tim=None):
        self.pin_id = pin_id
        self.config_data = config_data
        self.pin = None
        self.tim = tim
        self.lock = asyncio.Lock()
        self.type = None # state || pwm
        self.setup()

    def setup(self):
        pin_name = pinmap_data['DO_pins_map'][f'DO{self.pin_id}']
        init_value = self.config_data['DO'][f'DO{self.pin_id}']['init_value']
        pwm_cfg = self.config_data['DO'][f'DO{self.pin_id}'].get('pwm', {})
        pwm_enabled = pwm_cfg.get('enable', False)
        pwm_duty = pwm_cfg.get('duty', 0)

        if pwm_enabled and self.tim is not None:
            pin = Pin(pin_name, Pin.OUT)
            ch = (self.pin_id - 1) % 4 + 1
            self.pin = self.tim.channel(ch, Timer.PWM, pin=pin, pulse_width_percent=pwm_duty)
            self.type = 'pwm'
        else:
            self.pin = Pin(pin_name, Pin.OUT_PP, value=init_value)
            self.type = 'state'
  
    def get_type(self):
        return self.type

    async def on(self):
        async with self.lock:
            if self.type == 'state':
                self.pin.on()
                return True
            alert_print(f'[WAR] Set DO{self.pin_id} On', 3)
            return False 

    async def off(self):
        async with self.lock:
            if self.type == 'state':
                self.pin.off()
                return True
            alert_print(f'[WAR] Set DO{self.pin_id} Off', 3)
            return False 

    async def duty(self, value: int):
        async with self.lock:
            if self.type == 'pwm':
                self.pin.pulse_width_percent(value)
                return True
            alert_print(f'[WAR] Set DO{self.pin_id} Duty', 3)
            return False 

    async def get_state(self):
        async with self.lock:
            if self.type == 'state':
                return bool(self.pin.value())
            alert_print(f'[WAR] Read DO{self.pin_id} State', 3)
            return None
    
    async def get_duty(self):
        async with self.lock:
            if self.type == 'pwm':
                return round(self.pin.pulse_width_percent())
            alert_print(f'[WAR] Read DO{self.pin_id} Duty', 3)
            return None
    
do = {}
def DO_Init(config_data):
    for i in range(1, 17):
        try:
            if 1 <= i <= 4:
                do[i] = SafeDO(i, config_data, tim=tim1)
            elif 9 <= i <= 12:
                do[i] = SafeDO(i, config_data, tim=tim3)
            else:
                do[i] = SafeDO(i, config_data, tim=None)
            boot_print(f"[OK] DO{i}")
        except:
            boot_print(f"[ERR] DO{i}")
'''
example get type:
  do[1].get_type()            return: 'state' | 'pwm'

example like Pin:
  await do[1].on()                  return: True | False
  await do[1].off()                 return: True | False
  await stat = do[1].get_state()    return: True | False | None

example like PWM: 
  await do[2].duty(75)              return: True | False
  await duty = do[2].get_duty()     return: DutyCycle | None
'''


################################################################
# DI Init
class SafeDI:
    def __init__(self, pin_id: int, config_data):
        self.pin_id = pin_id
        self.config_data = config_data
        self.do_dict = do
        self.lock = asyncio.Lock() # Because of 2 thread (frequency, states)
        self.type = None # state || counter

        self.pin = None
        self.pull_up_pin = None
        self.counter = 0
        self.irq_edge = None
        self.meas_freq = None
        self.irq_link_to_out = ""
        self.link_to_out_type = ""

        self.setup()

    def setup(self):
        cfg = self.config_data['DI'][f'DI{self.pin_id}']
        pin_name = pinmap_data['DI_pins_map'][f'DI{self.pin_id}']
        pull_up_pin_name = pinmap_data['DI_PU_pins_map'][f'DI{self.pin_id}_PU']

        pull_up_enable = cfg['pull_up_enable']

        irq_cfg = cfg['irq']  # predpokladáme, že vždy existuje
        irq_enable = irq_cfg['enable']
        self.irq_edge = irq_cfg['edge']
        self.meas_freq = irq_cfg['meas_frequency']
        self.irq_link_to_out = irq_cfg['link_to_out']
        self.link_to_out_type = irq_cfg['link_to_out_type']

        self.pull_up_pin = Pin(pull_up_pin_name, Pin.OUT_PP)
        self.pull_up_pin.value(True if pull_up_enable else False)

        self.pin = Pin(pin_name, Pin.IN)

        if irq_enable:
            trigger_type = 0
            if self.link_to_out_type == 'direct' or self.link_to_out_type == 'reverse':
                trigger_type = Pin.IRQ_FALLING | Pin.IRQ_RISING
            else:
                if self.irq_edge == "FALLING":
                    trigger_type = Pin.IRQ_FALLING
                elif self.irq_edge == "RISING":
                    trigger_type = Pin.IRQ_RISING

            self.pin.irq(trigger=trigger_type, handler=self._irq_handler)
        
        if irq_enable and self.meas_freq:
            self.type = 'counter'
        else:
            self.type = 'state'

    def _irq_handler(self, pin):
        val = pin.value()
        if (val == 1 and self.irq_edge == "RISING") or \
            (val == 0 and self.irq_edge == "FALLING"):
            self.counter += 1
            if self.counter > 1000000:
                self.counter = 0

        if self.irq_link_to_out in self.do_dict:
            output = self.do_dict[self.irq_link_to_out].pin
            if isinstance(output, Pin):
                if self.link_to_out_type == "toggle":
                    output.value(not output.value())
                elif self.link_to_out_type == "direct":
                    output.value(val)
                elif self.link_to_out_type == "reverse":
                    output.value(not val)

    def get_type(self):
        return self.type

    async def get_state(self):
        async with self.lock:
            if self.type == 'state':
                return bool(self.pin.value())
            alert_print(f'[WAR] Read DI{self.pin_id} State', 3)
            return None
    
    async def get_count(self):
        async with self.lock:
            if self.type == 'counter':
                return self.counter
            alert_print(f'[WAR] Read DI{self.pin_id} Counter', 3)
            return None

    async def reset_count(self):
        async with self.lock:
            self.counter = 0

di = {}
def DI_Init(config_data):
    for i in range(1, 17):
        try:
            di[i] = SafeDI(i, config_data)
            boot_print(f"[OK] DI{i}")
        except:
            boot_print(f"[ERR] DI{i}")
'''
example get type:
  di[1].get_type()            return: 'state' | 'counter'

example like Pin:
  await stat = di[1].get_state()    return: True | False | None

example like counter: 
  await stat = di[1].get_count()    return: NumOfPulses | None
  await stat = di[1].reset_count()
'''

################################################################
# ADC Init
class SafeADC:
    def __init__(self, pin_id: int):
        self.pin_id = pin_id
        self.pin = None
        self.pin_name = pinmap_data['ADC_pins_map'][f'ADC{self.pin_id}']
        self.pin = ADC(self.pin_name)
        
    async def get(self):
        try:
            return self.pin.read()
        except:
            alert_print(f"[ERR] Read AI{self.pin_id}")
            return None

ai = {}
def AI_Init():
    for i in range(1, 5):
        try:
            ai[i] = SafeADC(i)
            boot_print(f"[OK] AI{i}")
        except:
            boot_print(f"[ERR] AI{i}")
'''
example:
   adc = await ai[1].get()     return ADCValue
'''

################################################################
# RTC Init
class SafeRTC:
    def __init__(self):
        self.rtc = RTC()

    async def get_datetime(self):
        try:
            dt = self.rtc.datetime()
            return dt
        except:
            alert_print("[ERR] RTC get datetime:")
            return None

    async def set_datetime(self):
        try:
            global eth
            if eth and eth.isconnected():
                ntptime.settime()
                alert_print("[OK] RTC set from NTP")
                return True
            else:
                alert_print("[ERR] RTC set from NTP")
                return False
        except:
            alert_print("[ERR] RTC set from NTP")
            return False
        
rtc = None
def RTC_Init():
    global rtc
    try:
        rtc = SafeRTC()
        boot_print("[OK] RTC Init")
    except:
        boot_print("[ERR] RTC Init")
'''
example:
await pheripherals.rtc.get_datetime()
await pheripherals.rtc.set_datetime()
'''


################################################################
# LM75 Init
class SafeLM75:
    def __init__(self, i2c, i2c_lock):
        self.i2c_lock = i2c_lock
        self.sensor = LM75(i2c, 0x48)
        if not self.sensor.begin():
            raise Exception("Not responding")            
        
    async def get(self):
        async with self.i2c_lock:
            try:
                return self.sensor.read_temp()
            except:
                alert_print("[ERR] LM75 get")
                return None
lm75 = None
def LM75_Init():
    global lm75
    try:
        lm75 = SafeLM75(i2c1, i2c1_lock)
        boot_print("[OK] LM75 Init")
    except:
        boot_print("[ERR] LM75 Init")
'''
example:
await pheripherals.lm75.get()
'''


#####################################################################x
# ADS1115 Init
class SafeADS1115:
    def __init__(self):
        self.i2c = I2C(2, freq=100000)	        # sck = F1; sda = F0
        self.ads = ADS1115(self.i2c, 0x48)
        if not self.ads.begin():
            raise Exception("Not responding")            

    async def get(self, channel):
        try:
            return self.ads.read_adc(channel - 1)
        except:
            alert_print(f"[ERR] ADS1115 get AD{channel}")
            return None
        
ads1115 = None
def ADS1555_Init():
    global ads1115
    try:
        ads1115 = SafeADS1115()
        boot_print("[OK] ADS1115 Init")
    except:
        boot_print("[ERR] ADS1115 Init:")
'''
example:
await pheripherals.ads1115.get(1)
'''

#####################################################################x
# uSD Card Init
class SafeSDCard:
    def __init__(self):
        self.spi = SPI(3, baudrate=1_000_000)  # clk = B3; miso = B4; mosi = B5
        self.cs = Pin(pinmap_data['SPI3']['NSS'], Pin.OUT_PP, value=True)
        self.card = SDCard(self.spi, self.cs)
        try:
            uos.umount("/sd")
        except OSError:
            pass
        uos.mount(self.card, "/sd")
        
sd = None
def uSD_Init():
    global sd
    try:
        sd = SafeSDCard()
        boot_print("[OK] uSD mounted at /sd")
    except:
        boot_print("[ERR] uSD mount")


