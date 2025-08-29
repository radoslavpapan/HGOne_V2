from machine import SPI, I2C, UART, WDT
from pyb import Pin, Timer, CAN, RTC, ADC
import network, utime, uos, ujson
import uasyncio as asyncio

from drivers.lm75 import LM75
from drivers.ads1115 import ADS1115
from drivers.sdcard import SDCard
from drivers.sh1106 import SH1106_I2C

def boot_print(text):
    print(text)
    try:
        boot_disp.println(text)
    except:
        pass
    if '[ERR]' in text:
        utime.sleep(5)

def alert_print(text):
    print(text)
    if disp is not None:
        try:
            # spustíme popup ako task, aby neblokoval
            asyncio.create_task(disp.popup(text, 10))
        except Exception as e:
            print("[ERR] alert_print:", e)

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
            except Exception as e:
                print("[ERR] OLED write_line:", e)

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
            except Exception as e:
                print("[ERR] OLED write_lines_dict:", e)

    async def refresh(self):
        async with self._lock:
            try:
                self.oled.fill(0)
                if not self._popup_active:
                    for i, text in enumerate(self._buffer):
                        self.oled.text(text, 0, i * self.line_height)
                self.oled.show()
            except Exception as e:
                print("[ERR] OLED refresh:", e)

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
            except Exception as e:
                print("[ERR] OLED popup:", e)

        # spustíme nový timeout task, ak je timeout zadaný
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
    except Exception as e:
        boot_print("[ERR] Display Init:", e)

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
await pheripherals.popup('test', 5)
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
# SPI3 for uSD
spi3 = None
spi3_ss = None
spi3_lock = None
def SPI3_Init():
    global spi3, spi3_ss, spi3_lock
    try:
        spi3 = SPI(3, baudrate=1_000_000)  # clk = B3; miso = B4; mosi = B5
        spi3_ss = Pin(pinmap_data['SPI3']['NSS'], Pin.OUT_PP, value=True)
        spi3_lock = asyncio.Lock()
        boot_print("[OK] SPI3 Init")
    except Exception as e:
        boot_print("[ERR] SPI3 Init:", e)

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
# I2C2 for external ADC (ADS1115)
i2c2 = None
i2c2_lock = None
def I2C2_Init():
    global i2c2, i2c2_lock
    try:
        i2c2 = I2C(2, freq=100000)	        # sck = F1; sda = F0
        i2c2_lock = asyncio.Lock()
        boot_print("[OK] I2C2 Init")
    except Exception as e:
        boot_print("[ERR] I2C2 Init:", e)

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
            dio[f'DIO{i}'] = {
                "pin": Pin(pin_name),
                "lock": asyncio.Lock()
            }
        boot_print("[OK] DIO Init")
    except Exception as e:
        boot_print("[ERR] DIO Init:", e)

################################################################ 
# TIM1 for DO PWM (DO1 - DO4)
tim1 = None
def Timer1_Init(config_data):
    global tim1
    try:
        if config_data['timer1']['enable']:
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
        if config_data['timer3']['enable']:
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
        self.tim = tim
        self.pin = None
        self.lock = asyncio.Lock()
        self.setup()

    def setup(self):
        try:
            pin_name = pinmap_data['DO_pins_map'][f'DO{self.pin_id}']
            init_value = self.config_data['DO'][f'DO{self.pin_id}']['init_value']
            pwm_cfg = self.config_data['DO'][f'DO{self.pin_id}'].get('pwm', {})
            pwm_enabled = pwm_cfg.get('enable', False)
            pwm_duty = pwm_cfg.get('duty', 0)

            if pwm_enabled and self.tim is not None:
                pin = Pin(pin_name, Pin.OUT)
                ch = (self.pin_id - 1) % 4 + 1
                self.pin = self.tim.channel(ch, Timer.PWM, pin=pin, pulse_width_percent=pwm_duty)
            else:
                self.pin = Pin(pin_name, Pin.OUT_PP, value=init_value)

            boot_print(f"[OK] DO{self.pin_id} Setup")

        except Exception as e:
            boot_print(f"[ERR] DO{self.pin_id} Setup:", e)
            self.pin = None

    async def _on_set(self):
        try:
            if isinstance(self.pin, Pin):
                async with self.lock:
                    self.pin.on()
            else:
                raise TypeError("Incorrect instance")
        except Exception as e:
            alert_print("[ERR] DO On:", e)

    async def _off_set(self):
        try:
            if isinstance(self.pin, Pin):
                async with self.lock:
                    self.pin.off()
            else:
                raise TypeError("Incorrect instance")
        except Exception as e:
            alert_print("[ERR] DO Off:", e)

    async def _duty_set(self, value: int):
        try:
            if hasattr(self.pin, "pulse_width_percent"):
                async with self.lock:
                    self.pin.pulse_width_percent(value)
            else:
                raise TypeError("Incorrect instance")
        except Exception as e:
            alert_print("[ERR] DO Duty:", e)

    async def _get(self):
        try:
            async with self.lock:
                if isinstance(self.pin, Pin):
                    return bool(self.pin.value())
                else:
                    return self.pin.pulse_width_percent()
        except Exception as e:
            alert_print("[ERR] DO Get:", e)
            return None
        
    async def on(self):
        try:
            await self._on_set()
        except :
            alert_print("[ERR] DO On:")

    async def off(self):
        try:
            await self._off_set()
        except :
            alert_print("[ERR] DO Off:")

    async def duty(self, value: int):
        try:
            await self._duty_set(value)
        except:
            alert_print("[ERR] DO Duty:")

    
    async def get(self):
        try:
            val = await self._get()
            return val
        except Exception as e:
            alert_print("[ERR] DO Get:", e)
            return None
do = {}
def DO_Init(config_data):
    for i in range(1, 17):
        if 1 <= i <= 4:
            do[i] = SafeDO(i, config_data, tim=tim1)
        elif 9 <= i <= 12:
            do[i] = SafeDO(i, config_data, tim=tim3)
        else:
            do[i] = SafeDO(i, config_data, tim=None)
'''
example like Pin:
  await do[1].on()
  await stat = do[1].get()    return True/False

example like PWM: 
  await do[2].duty(75)
  await duty = do[2].get()    return DutyCycle
'''

################################################################
# DI Init
class SafeDI:
    def __init__(self, pin_id: int, config_data):
        self.pin_id = pin_id
        self.config_data = config_data
        self.do_dict = do
        self.lock = asyncio.Lock()

        self.pin = None
        self.pull_up_pin = None
        self.counter = 0
        self.irq_edge = None
        self.irq_link_to_out = ""
        self.link_to_out_type = ""

        self.setup()

    def setup(self):
        try:
            cfg = self.config_data['DI'][f'DI{self.pin_id}']
            pin_name = pinmap_data['DI_pins_map'][f'DI{self.pin_id}']
            pull_up_pin_name = pinmap_data['DI_PU_pins_map'][f'DI{self.pin_id}_PU']

            pull_up_enable = cfg['pull_up_enable']
            irq_cfg = cfg.get('irq', {})
            irq_enable = irq_cfg.get('enable', False)
            self.irq_edge = irq_cfg.get('edge', 'FALLING')
            self.irq_link_to_out = irq_cfg.get('link_to_out', '')
            self.link_to_out_type = irq_cfg.get('link_to_out_type', '')

            # Pull-up pin setup
            self.pull_up_pin = Pin(pull_up_pin_name, Pin.OUT_PP)
            self.pull_up_pin.value(True if pull_up_enable else False)

            # Input pin setup
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

            boot_print(f"[OK] DI{self.pin_id} Setup")

        except Exception as e:
            boot_print(f"[ERR] DI{self.pin_id} Setup:", e)
            self.pin = None

    def _irq_handler(self, pin):
        try:
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
        except Exception as e:
            boot_print(f"[ERR] DI{self.pin_id} IRQ:", e)

    async def get(self):
        try:
            async with self.lock:
                if self.pin:
                    return bool(self.pin.value())
                return None
        except Exception as e:
            alert_print(f"[ERR] DI{self.pin_id} Get:", e)
            return None

    async def count(self):
        async with self.lock:
            return self.counter

    async def reset_count(self):
        async with self.lock:
            self.counter = 0
di = {}
def DI_Init(config_data):
    for i in range(1, 17):
        di[i] = SafeDI(i, config_data)

################################################################
# ADC Init
class SafeADC:
    def __init__(self, pin_id: int):
        self.pin_id = pin_id
        self.lock = asyncio.Lock()
        self.pin = None
        self.setup()

    def setup(self):
        try:
            pin_name = pinmap_data['ADC_pins_map'][f'ADC{self.pin_id}']
            self.pin = ADC(pin_name)
            boot_print(f"[OK] AI{self.pin_id} Setup")
        except Exception as e:
            boot_print(f"[ERR] AI{self.pin_id} Setup:", e)
            self.pin = None

    async def get(self):
        try:
            async with self.lock:
                if self.pin is not None:
                    return self.pin.read()
                else:
                    raise Exception("Not initialized")
        except Exception as e:
            boot_print(f"[ERR] AI{self.pin_id} Read:", e)
            return None
ai = {}
def AI_Init():
    for i in range(1, 5):
        ai[i] = SafeADC(i)
'''
example:
  await adc = ai[1].get()     return ADCValue
'''

################################################################
# Ethernet for MQTT communication
eth = None
eth_lock = None
def ETH_Init(config_data):
    global eth, eth_lock
    try:
        eth = network.LAN()
        eth_lock = asyncio.Lock()
        eth.active(True)
        ip_address =	config_data['eth']['ip']
        if ip_address != "DHCP":
            subnet =		config_data['eth']['subnet']
            gateway = 		config_data['eth']['gateway']
            dns = 			config_data['eth']['dns']
            eth.ifconfig((ip_address, subnet, gateway, dns))
        attempts = 0
        if not eth.isconnected():
            boot_print('[INFO] ETH Connecting...')
            while not eth.isconnected() and attempts < 15:
                utime.sleep(1)
                attempts += 1

            if not eth.isconnected():
                raise Exception('Timeout')
        boot_print("[OK] ETH Init")
        boot_print("[INFO] IP:")
        boot_print(f" {eth.ifconfig()[0]}")
    except Exception as e:
        boot_print("[ERR] ETH Init:", e)

################################################################
# RTC Init
class SafeRTC:
    def __init__(self, rtc, rtc_lock):
        self.rtc = rtc
        self.lock = rtc_lock

    async def get_datetime(self):
        async with self.lock:
            try:
                dt = self.rtc.datetime()
                return dt
            except Exception as e:
                boot_print("[ERR] RTC get datetime:", e)
                return None
rtc = None
def RTC_Init():
    global rtc
    try:
        rtc = SafeRTC(RTC(), asyncio.Lock())
        boot_print("[OK] RTC Init")
    except Exception as e:
            boot_print("[ERR] RTC Init:", e)

################################################################
# LM75 Init
class SafeLM75:
    def __init__(self, i2c, i2c_lock, addr=0x48):
        self.i2c_lock = i2c_lock
        try:
            self.sensor = LM75(i2c, addr)
            if self.sensor.begin():
                boot_print("[OK] LM75 Init")
            else:
                raise Exception("Not responding")
        except Exception as e:
            boot_print("[ERR] LM75 Init:", e)

    async def get(self):
        async with self.i2c_lock:
            try:
                return self.sensor.read_temp()
            except Exception as e:
                boot_print("[ERR] LM75 get:", e)
                return None
lm75 = None
def LM75_Init():
    global lm75
    lm75 = SafeLM75(i2c1, i2c1_lock)

#####################################################################x
# ADS1115 Init
class SafeADS1115:
    def __init__(self, i2c, i2c_lock, addr=0x48):
        self.i2c_lock = i2c_lock
        try:
            self.ads = ADS1115(i2c, addr)
            if self.ads.begin():
                boot_print("[OK] ADS1115 Init")
            else:
                raise Exception("Not responding")
        except Exception as e:
            boot_print("[ERR] ADS1115 Init:", e)

    async def get(self, channel):
        async with self.i2c_lock:
            try:
                return self.ads.read_adc(channel - 1)
            except Exception as e:
                boot_print(f"[ERR] ADS1115 get AD{channel}:", e)
                return None
ads1115 = None
def ADS1555_Init():
    global ads1115
    ads1115 = SafeADS1115(i2c2, i2c2_lock)

#####################################################################x
# uSD Card Init
class SafeSDCard:
    def __init__(self, spi, cs_pin, spi_lock):
        self.spi = spi
        self.cs = cs_pin if isinstance(cs_pin, Pin) else Pin(cs_pin, Pin.OUT_PP, value=True)
        self.card = SDCard(self.spi, self.cs)
        self.spi_lock = spi_lock

        try:
            uos.umount("/sd")
        except OSError:
            pass
        try:
            uos.mount(self.card, "/sd")
            boot_print("[OK] uSD mounted at /sd")
        except Exception as e:
            boot_print("[ERR] uSD mount:", e)

    async def read_json(self, filepath):
        async with self.spi_lock:
            try:
                with open(filepath, "r") as f:
                    return ujson.load(f)
            except Exception as e:
                boot_print("[ERR] Read data:", e)
                return None

    async def write_json(self, filepath, data):
        async with self.spi_lock:
            try:
                with open(filepath, "w") as f:
                    ujson.dump(data, f)
                return True
            except Exception as e:
                boot_print("[ERR] Write data:", e)
                return False
sd = None
def uSD_Init():
    global sd
    sd = SafeSDCard(spi=spi3, cs_pin=spi3_ss, spi_lock=spi3_lock)

#####################################################################



