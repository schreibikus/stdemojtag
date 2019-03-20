# stdemojtag

stdemojtag is a JTAG adapter firmware for STM32L476 Discovery boards.

Hardcoded PINs:

```
PD9 -> JTAG_TRST (JTAG Reset)
PD10 -> JTAG_TCK
PD11 -> JTAG_TMS
PD12 -> JTAG_TDI
PD13 -> JTAG_TDO
PD14 -> POR_B (System Reset)
```

## OpenOCD

OpenOCD in this repository supports this JTAG adapter board. The driver is called stdemo.

Using OpenOCD:

```
telnet localhost 4444
Trying 127.0.0.1...
Connected to localhost.
Escape character is '^]'.
Open On-Chip Debugger
> halt
imx8mq.a53.0 cluster 0 core 0 multi core
target halted in AArch64 state due to debug-request, current mode: EL3H
cpsr: 0x400003cd pc: 0x8ddc
MMU: disabled, D-Cache: disabled, I-Cache: enabled
>
```

## openocd.cfg example

```
source [find interface/stdemo.cfg]
source [find board/nxp_mcimx8m-evk.cfg]
```

## Some docs

 * [OpenOCD User's Guide: General Commands](http://openocd.org/doc/html/General-Commands.html)

