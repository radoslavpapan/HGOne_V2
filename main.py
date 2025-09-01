exec(open('boot.py').read())

import utime
import uasyncio as asyncio
import pheripherals
import shared_data
import services
import dht_api
import mqtt_api

async def main():
    asyncio.create_task(services.measure_pcb_temp_task(5))
    
    for i in range(1, 5):
        asyncio.create_task(services.measure_adce_task(i, 2))
    
    for i in range(1, 5):
        asyncio.create_task(services.measure_adci_task(i, 2))
    
    for i in range(1, 17):
        if pheripherals.di[i].get_type()  == 'counter':
            asyncio.create_task(services.measure_di_freq_task(i, 1))
        else:
            asyncio.create_task(services.measure_di_state_task(i, 0.25))
    
    for i in range(1, 17):
        if pheripherals.do[i].get_type()  == 'pwm':
            asyncio.create_task(services.measure_do_duty_task(i, 10))
        else:
            asyncio.create_task(services.measure_do_state_task(i, 0.5))
    
    for i in range(1, 16):
        if dht_api.dht[i] is not None:
            asyncio.create_task(dht_api.measure_dht_task(i, 3))
    
    asyncio.create_task(mqtt_api.check_mqtt_subscribe_task())
    
    for topic, interval, func in mqtt_api.publish_topics:
        asyncio.create_task(mqtt_api.publish_mqtt_task(topic, interval, func))

    await asyncio.sleep(1)
    while True:
        try:      
            # Blink status LED
            pheripherals.status_led.on()
            await asyncio.sleep(0.25)
            pheripherals.status_led.off()
            await asyncio.sleep(0.25)
        except Exception as e:
            pheripherals.alert_print(f"[ERR] main loop print: {e}")
            await asyncio.sleep(1)
        
try:
    asyncio.run(main())
except:
    from machine import soft_reset
    soft_reset()
# except Exception as e:
#     print(e)

