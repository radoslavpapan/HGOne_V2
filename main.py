import utime
import uasyncio as asyncio
import pheripherals

exec(open('boot.py').read())

# config.Config_Update(["timer1", "enable"], True)
# config.Config_Update(["timer1", "freq"], 5)
# 
# config.Config_Update(["DO", "DO1", "init_value"], False)
# config.Config_Update(["DO", "DO1", "pwm", "enable"], True)
# config.Config_Update(["DO", "DO1", "pwm", "duty"], 50)

# config.Config_Update(["timer3", "enable"], False)
# 
# config.Config_Update(["DO", "DO9", "init_value"], False)
# config.Config_Update(["DO", "DO9", "pwm", "enable"], False)
# config.Config_Update(["DO", "DO9", "pwm", "duty"], 50)

# config.Config_Update(["DI", "DI1", "pull_up_enable"], False)
# config.Config_Update(["DI", "DI1", "irq", "edge"], "RISING")
# config.Config_Update(["DI", "DI1", "irq", "link_to_out"], 4)
# config.Config_Update(["DI", "DI1", "irq", "link_to_out_type"], "direct")

async def main():
#     for i in range(1, 17):
#         stat = await pheripherals.di[i].get()
#         print(f'DI{i}: {stat}')
#     for i in range(1, 5):
#         stat = await pheripherals.ai[i].get()
#         print(f'AI{i}: {stat}')
#     
#     cur_time = await pheripherals.rtc.get_datetime()
#     print(cur_time)
#     
#     cur_pcb_temp = await pheripherals.lm75.get()
#     print(cur_pcb_temp)
#     
#     for i in range(1, 5):
#         stat = await pheripherals.ads1115.get(i)
#         print(f'AIExt{i}: {stat}')
    
#     for i in range(1, 100):
#         await pheripherals.do[9].duty(i)
#         await asyncio.sleep(0.005)
#         print(f"Hodnota: {i}", end="\r")
#     for i in range(100, 1, -1):
#         await pheripherals.do[9].duty(i)
#         await asyncio.sleep(0.005)
#         print(f"Hodnota: {i}", end="\r")
#         
#     for i in range(1, 17):
#         await pheripherals.do[i].on()
#         stat = await pheripherals.do[i].get()
#         print(f'DO{i}: {stat}')
#     for i in range(1, 17):
#         await pheripherals.do[i].off()
#         stat = await pheripherals.do[i].get()
#         print(f'DO{i}: {stat}')
    
    while True:
        counter = await pheripherals.di[1].count()
        print(f"\rDI1 Counter: {counter:<10}", end="")
        await pheripherals.di[1].reset_count()
        await asyncio.sleep(1)
#         pheripherals.status_led.on()
#         await asyncio.sleep(0.25)
#         pheripherals.status_led.off()
#         await asyncio.sleep(0.5)

    await asyncio.sleep(0.5)

try:
    asyncio.run(main())
except:
    from machine import soft_reset
    soft_reset()
# except Exception as e:
#     print(e)