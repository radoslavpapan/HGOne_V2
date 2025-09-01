import uasyncio as asyncio
import shared_data
import pheripherals

async def measure_pcb_temp_task(delay: float = 1):
    while True:
        temp = await pheripherals.lm75.get()
        await shared_data.pcb_temp.set(temp)
        await asyncio.sleep(delay)

async def measure_adce_task(idx: int, delay: float = 1):
    while True:
        raw = await pheripherals.ads1115.get(idx)
        await shared_data.adce[idx].set(raw)
        await asyncio.sleep(delay)

async def measure_adci_task(idx: int, delay: float = 1):
    while True:
        raw = await pheripherals.ai[idx].get()
        await shared_data.adci[idx].set(raw)
        await asyncio.sleep(delay)

async def measure_di_freq_task(idx: int, delay: float = 1):
    while True:
        count = await pheripherals.di[idx].get_count()
        freq = count / delay
        await shared_data.din[idx].set(freq)
        await pheripherals.di[idx].reset_count()
        await asyncio.sleep(delay)

async def measure_di_state_task(idx: int, delay: float = 1):
    while True:
        state = await pheripherals.di[idx].get_state()
        await shared_data.din[idx].set(state)
        await asyncio.sleep(delay)

async def measure_do_state_task(idx: int, delay: float = 1):
    while True:
        state = await pheripherals.do[idx].get_state()
        await shared_data.dout[idx].set(state)
        await asyncio.sleep(delay)
        
async def measure_do_duty_task(idx: int, delay: float = 1):
    while True:
        state = await pheripherals.do[idx].get_duty()
        await shared_data.dout[idx].set(state)
        await asyncio.sleep(delay)        
        
        
        
        
        