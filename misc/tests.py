# Test SharedData:
await shared_data.pcb_temp.set(10)
print(await shared_data.pcb_temp.get(), await shared_data.pcb_temp.get_unit())

await shared_data.adci[1].set(1024)
print(await shared_data.adci[1].get(), await shared_data.adci[1].get_unit())

await shared_data.adce[1].set(1024)
print(await shared_data.adce[1].get(), await shared_data.adce[1].get_unit())

await shared_data.din[2].set(True)
print(await shared_data.din[2].get())
await shared_data.din[1].set(10)
print(await shared_data.din[1].get())

await shared_data.dout[1].set(100)
print(await shared_data.dout[1].get())


# Test ADS1115:
for i in range(1, 5):
    print(f'AII{i} Ext: ', await pheripherals.ads1115.get(i))

# Test LM75:
print('PCB Temp: ', await pheripherals.lm75.get(), '°C')

# Test AIInt:
for i in range(1, 5):
    print(f'AII{i} Int: ', await pheripherals.ai[i].get())

# Test RTC:
print(await pheripherals.rtc.get_datetime())
print(await pheripherals.rtc.set_datetime())
print(await pheripherals.rtc.get_datetime())

# Test DO:
for i in range(1, 17):
    print(f'DO{i}: ', await pheripherals.do[i].get_type())
 
for i in range(1, 17):
    print(f'Set DO{i} on: ', await pheripherals.do[i].on())
    print(f'Read DO{i} on: ', await pheripherals.do[i].get_state())
await asyncio.sleep(1) 
for i in range(1, 17):
    print(f'Set DO{i} on: ', await pheripherals.do[i].off())
    print(f'Read DO{i} off: ', await pheripherals.do[i].get_state())

for i in range(0, 101):
    print('DO1: ', f'Set duty {i}: ', await pheripherals.do[1].duty(i), f'\tRead duty: ', await pheripherals.do[1].get_duty(), end="\r")
    await asyncio.sleep(0.05) 
for i in range(100, 0, -1):
    print('DO1: ', f'Set duty {i}: ', await pheripherals.do[1].duty(i), f'\tRead duty: ', await pheripherals.do[1].get_duty(), end="\r")
    await asyncio.sleep(0.05)

# Tests DI:
for i in range(1, 17):
    print(f'DI{i}: ', await pheripherals.di[i].get_type())
for i in range(1, 17):
    print(f'Read DI{i} state: ', await pheripherals.di[i].get_state())

awaittime = 2
for i in range(5):
    await pheripherals.di[1].reset_count()
    await asyncio.sleep(awaittime)
    cnt = await pheripherals.di[1].get_count()
    print('Pulses: ', cnt, 'Freq: ', cnt/awaittime, end="\r")
print('')
for i in range(5):
    await pheripherals.di[2].reset_count()
    await asyncio.sleep(awaittime)
    cnt = await pheripherals.di[2].get_count()
    try:
        print('Pulses: ', cnt, 'Freq: ', cnt/awaittime, end="\r")
    except:
        print('returned', cnt)

# Tests OLED:
lines  = {
        0: "Hello",
        1: "World",
        2: "Doplnok",
        3: "Riadok 4"
    }
await pheripherals.disp.write_lines_dict(lines)

await pheripherals.disp.write_line(1, f'cnter: {counter}')

await pheripherals.disp.popup('test', 5)