# STM32F446RE → I²C-1602 LCD Demo

This project demonstrates bit-banged I²C (via CMSIS register writes, **no HAL**) to drive a 16×2 HD44780-compatible LCD through a PCF8574 I²C I/O expander. On reset it centers and prints **I LOVE BUBU**.

---

## File Layout

LCD/
├─ .cproject
├─ .gitignore
├─ LCD.ioc
├─ Core/
│ ├─ Inc/
│ │ └─ main.h
│ └─ Src/
│ └─ main.c ← all application code here
├─ Drivers/
└─ …

css
Copy
Edit

All of the code lives in:
Core/Src/main.c

markdown
Copy
Edit

---

## Build & Flash

1. Open `LCD.ioc` in STM32CubeIDE.  
2. Let Cube generate the project.  
3. Replace `Core/Src/main.c` with the provided version.  
4. Build (Project → Build) and Debug/Run on the NUCLEO-F446RE.

---

## How it works

I used **CMSIS** (direct `RCC->…`, `GPIOB->…`, `I2C1->…`) to:

- **`SystemClock_Config()`**  
  Sets up the PLL to run the F446 at 84 MHz (APB1 = 42 MHz).

- **`GPIO_Init()`**  
  Configures PB6/PB7 as I²C1 SCL/SDA: AF4 open-drain, pull-up, high speed.

- **`I2C_Init()`**  
  Soft-resets I2C1 and programs:  
  - `CR2.FREQ = 16` MHz  
  - `CCR = 0x50` for 100 kHz  
  - `TRISE = 17`  
  - Enables `PE`

- **`TIM4_ms_Delay(ms)`**  
  A blocking millisecond delay via TIM4.

- **`I2C_Start()`, `I2C_Send_Addr()`, `I2C_Write()`, `I2C_Stop()`**  
  Polling-based routines for I²C Master transfers:
  1. Generate START → wait `SR1.SB`  
  2. Send 8-bit address → wait `SR1.ADDR` (clear it)  
  3. Send data → wait `SR1.TXE`/`SR1.BTF`  
  4. Generate STOP

- **`LCD_Init()`**  
  HD44780 4-bit initialization:
  1. Wait > 40 ms  
  2. 3× `0x30` (8-bit reset)  
  3. `0x20` → switch to 4-bit  
  4. Function-set, display-off, clear, entry-mode (increment/no-shift), display-on

- **`LCD_Cursor(r,c)`**  
  Sets DDRAM address to row `r` (0 or 1), column `c`.

- **`LCD_Write_Cmd(addr,reg,byte)`**  
  Sends one command byte in two 4-bit bursts (EN=1→0).

- **`LCD_Write_Data(addr,reg,byte)`**  
  Same as _Write_Cmd_ but with `RS=1` for data writes.

---

## Notes

- **No HAL I²C** or HAL GPIO—everything is register-level CMSIS.  
- Power the LCD backpack at **3.3 V** so I²C pull-ups match the STM32’s I/O.  
- Adjust the initial `TIM4_ms_Delay(100)` in `main()` if your module needs more power-on settling.
- Enjoy!
