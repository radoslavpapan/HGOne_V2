#temp part - only for programming
import gc

gc.collect()
import sys
modules_to_clear = ['pheripherals', 'config', 'drivers.ads1115', 'drivers.lm75', 'drivers.sh1106', 'drivers.sdcard', 'services', 'shared_data', 'app_services', 'mqtt']
for mod in modules_to_clear:
    if mod in sys.modules:
        del sys.modules[mod]

exec(open('boot.py').read())

#Start main.py
import utime
import uasyncio as asyncio
import services, app_services

    
async def main():
    # Basic services
#     wd_task = asyncio.create_task(services.wdt_refresh_task())
    pcb_temp_task = asyncio.create_task(services.measure_pcb_temp_task())
    in_freq_task = asyncio.create_task(services.measure_in_frequency_task())
    external_adc_task = asyncio.create_task(services.measure_external_adc_task())
    internal_adc_task = asyncio.create_task(services.measure_internal_adc_task())
    di_read_task = asyncio.create_task(services.read_di_stat_task())
    do_read_task = asyncio.create_task(services.read_do_stat_task())
    oled_task = asyncio.create_task(run_oled_task())
    
    read_dht_task = asyncio.create_task(app_services.read_dht_values_task())
    check_mqtt_task = asyncio.create_task(app_services.check_mqtt_subscribe_task())
    publish_mqtt_task = asyncio.create_task(app_services.publish_mqtt_values_task())
    await asyncio.sleep(2)
    
    
    while True:
        pheripherals.status_led.on()
        await asyncio.sleep(0.25)
        pheripherals.status_led.off()
        await asyncio.sleep(0.5)
    
    
try:
    asyncio.run(main())
except:
    from machine import soft_reset
    soft_reset()    
# except Exception as e:
#     print(e)


