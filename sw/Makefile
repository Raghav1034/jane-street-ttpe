# Assemble the example programs into images for the host (RP2350 / MicroPython) and for $readmemh.
#   make images                      all six examples -> build/<name>.hex  .bin  .py (PROGRAM = [...])
#   make images ASMFLAGS="-D BIT=100"   pass assembler -D options to every program (unused symbols are ignored)
#   make test                        sw unit tests (python3 -I -m unittest discover -s tests)
#   make clean
# Defaults: UART 115200 baud (BIT=434), I2C 100 kHz (TL=TH=250), see the header of each .asm for parameters.

PY       ?= python3
ASMFLAGS ?=
BUILD    ?= build
EXAMPLES := uart_tx uart_rx spi_master i2c_master usb_rx usb_tx

.PHONY: images test clean

images: $(foreach e,$(EXAMPLES),$(BUILD)/$(e).hex)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.hex: examples/%.asm ttpe_asm.py ttpe_isa.py | $(BUILD)
	$(PY) ttpe_asm.py $< -o $@ --bin $(BUILD)/$*.bin --mpy $(BUILD)/$*.py $(ASMFLAGS)

test:
	$(PY) -I -m unittest discover -s tests

clean:
	rm -rf $(BUILD)
