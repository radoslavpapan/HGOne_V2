exec(open('boot.py').read()) # Must be commented with real Run

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
            asyncio.create_task(services.measure_di_state_task(i, 0.15))
    
    for i in range(1, 17):
        if pheripherals.do[i].get_type()  == 'pwm':
            asyncio.create_task(services.measure_do_duty_task(i, 1))
        else:
            asyncio.create_task(services.measure_do_state_task(i, 0.5))
    
    asyncio.create_task(dht_api.measure_dht_task(5))
    
    asyncio.create_task(mqtt_api.check_mqtt_subscribe_task(0.1))
    
    for topic, interval, func in mqtt_api.publish_topics:
        asyncio.create_task(mqtt_api.publish_mqtt_task(topic, interval, func))

    await asyncio.sleep(1)
    while True: 
#             pheripherals.wdt.feed()
            pheripherals.status_led.on()
            await asyncio.sleep(0.25)
            pheripherals.status_led.off()
            await asyncio.sleep(0.25)
        
try:
    asyncio.run(main())
except:
    from machine import soft_reset
    soft_reset()
# except Exception as e:
#     print(e)

