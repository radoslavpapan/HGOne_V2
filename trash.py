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
                if self.link_to_out_type == 'direct':
                    trigger_type = Pin.IRQ_FALLING | Pin.IRQ_RISING
                else:
                    if self.irq_edge == "FALLING":
                        trigger_type = Pin.IRQ_FALLING
                    elif self.irq_edge == "RISING":
                        trigger_type = Pin.IRQ_RISING

                self.pin.irq(trigger=trigger_type, handler=self._irq_handler)

            print(f"[OK] DI{self.pin_id} Setup")

        except Exception as e:
            print(f"[ERR] DI{self.pin_id} Setup:", e)
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
            print(f"[ERR] DI{self.pin_id} IRQ:", e)

    async def get(self):
        try:
            async with self.lock:
                if self.pin:
                    return bool(self.pin.value())
                return None
        except Exception as e:
            print(f"[ERR] DI{self.pin_id} Get:", e)
            return None

    async def count(self):
        async with self.lock:
            return self.counter

    async def reset_count(self):
        async with self.lock:
            self.counter = 0