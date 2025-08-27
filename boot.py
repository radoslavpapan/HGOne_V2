from pyb import repl_uart, main
repl_uart(None) # must be turned off on begining because same uart is used for wifi (uart3)

import gc
gc.collect()
import sys
modules_to_clear = ['pheripherals', 'config']
for mod in modules_to_clear:
    if mod in sys.modules:
        del sys.modules[mod]


import pheripherals
import config

pheripherals.PinMap_Init()
pheripherals.SPI3_Init()
pheripherals.uSD_Init()

config.Config_Init()

pheripherals.I2C1_Init()
pheripherals.I2C2_Init()

pheripherals.Disp_Init()

pheripherals.StatusLED_Init()

pheripherals.UART3_Init()
pheripherals.UART8_Init()
pheripherals.UART2_Init(config.config_data)
pheripherals.UART5_Init(config.config_data)

pheripherals.CAN1_Init(config.config_data)

# pheripherals.WDT_Init()

pheripherals.DIO_Init()

pheripherals.Timer1_Init(config.config_data)
pheripherals.Timer3_Init(config.config_data)

pheripherals.DO_Init(config.config_data)
pheripherals.DI_Init(config.config_data)

pheripherals.AI_Init()

pheripherals.RTC_Init()

# pheripherals.ETH_Init(config.config_data)

pheripherals.LM75_Init()

pheripherals. ADS1555_Init()

print('')

main('main.py')

