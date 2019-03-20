/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define SEND_BUFFER_LEN 128
#define RECV_BUFFER_LEN 128

#define COMNUM 3 /* Count of commands */
#define COM_RESET "reset"
#define COM_STARTJTAG "startjtag"
#define COM_STOPJTAG "stopjtag"

#define DEV_INT_COMMAND_RESET           0x01 // COM_RESET
#define DEV_INT_COMMAND_STARTJTAG       0x02 // COM_STARTJTAG
#define DEV_INT_COMMAND_STOPJTAG        0x03 // COM_STOPJTAG

#define DEV_INT_COMMAND_SET_LED         0x10 // set LED command 0 0 0
#define DEV_INT_COMMAND_SET_TCK_TMS_TDI 0x11 // set TCK TMS TDI 0 0 1
#define DEV_INT_COMMAND_SET_TRST_SRST   0x12 // set TRST SRST 0 1 0
#define DEV_INT_COMMAND_SET_GET_DATA    0x13 // set get data 0 1 1
#define DEV_INT_COMMAND_TO_DEV_DATA     0x14 // set get data 1 0 0
#define DEV_INT_COMMAND_RUN_TEST        0x15 // run test 1 0 1
#define DEV_INT_COMMAND_TMS_MOVE        0x16 // set TMS 1 1 0
#define DEV_INT_COMMAND_GET_TDO         0x17 // get TDO 1 1 1


#define DEV_INT_PROCESS_COMMAND         0xA0 // process command


/*
 * 8bit command semantic
 *-----------
 *  7 - bit one
 *-----------
 * 6,5,4 - command
 * 0 0 0 - set LED value(bit 1)
 * 0 0 1 - set values TCK(bit 3) TMS(bit 2) TDI(bit 1)
 * 0 1 0 - set values TRST(bit 2) SRST(bit 1)
 * 0 1 1 - recv and send data. bit(3,2,1) - presence flag of length byte
 * 1 0 0 - recv data. bit(3,2,1) - presence flag of length byte
 * 1 0 1 - run test. bit(3,2,1) - presence flag of length byte
 * 1 1 0 - tms move. bit(3,2,1) - length of write in bits
 * 1 1 1 - get value TDO. !!!!! not used !!!!!
 *-----------
 * 3,2,1 - data
 *-----------
 * 0 - parity bit
 *-----------
 */

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;

static uint8_t recv_buffer[RECV_BUFFER_LEN]; /* Buffer for receiving data */
static volatile uint8_t recv_addr_start, recv_addr_end; /* Write and Read pointers of circular bufffer */
static uint8_t send_buffer[SEND_BUFFER_LEN]; /* Buffer for sending data */
static volatile uint8_t send_addr_start, send_addr_end; /* Write and Read pointers of circular buffer */
static volatile uint8_t send_running; /* Sending data  */

static volatile uint32_t time; /* time delay in miliseconds */
static volatile uint8_t time_delay; /* running time delay */

static volatile uint8_t nlen; /* length of current command */
static volatile uint8_t comrun[COMNUM]; /* for parsing commands */
static volatile const char *commands[COMNUM] = { COM_RESET, COM_STARTJTAG, COM_STOPJTAG };
static volatile uint8_t comlen[COMNUM] = { 5, 9, 8 }; /* length every command */

static uint8_t in_jtag_state; /* Only work short command and stopjtag command */
static uint8_t jtag_port_data;
static uint8_t jtag_port_wait_integer; // size of integger in bytes
static uint32_t jtag_port_integer; // integer for command

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

static UART_HandleTypeDef uartHandle;

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    if (time_delay) /* running delay algo */
    {
        if (time)
            time --;
        else
            time_delay = 0;
    }
    HAL_IncTick();
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&uartHandle);
}

static void UART_RxISR(UART_HandleTypeDef *huart)
{
    uint16_t uhMask = huart->Mask;
    uint16_t uhdata = (uint16_t)READ_REG(huart->Instance->RDR);
    uint8_t addr = 0x7F & recv_addr_end;

    recv_buffer[addr] = (uint8_t)(uhdata & (uint8_t)uhMask);
    addr ++;
    if (addr == RECV_BUFFER_LEN)
        recv_addr_end = (~recv_addr_end) & 0x80;
    else
        recv_addr_end++;
}

static void UART_TxISR(UART_HandleTypeDef *huart)
{
    uint8_t addr;
    if (send_addr_end != send_addr_start)
    {
        addr = send_addr_start & 0x7F;
        huart->Instance->TDR = send_buffer[addr];
        addr ++;
        if (addr == SEND_BUFFER_LEN)
            send_addr_start = (~send_addr_start) & 0x80;
        else
            send_addr_start++;
    }
    else
    {
        send_running = 0;
        CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE); /* Disable interrupt. Internal buffer is empty */
    }
}

HAL_StatusTypeDef HAL_UART_Confugure(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if(huart)
    {
        huart->Instance = USART2;
        huart->Init.BaudRate = 115200;
        huart->Init.WordLength = UART_WORDLENGTH_8B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_NONE;
        huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart->Init.Mode = UART_MODE_TX_RX;

        if((status = HAL_UART_Init(huart)) == HAL_OK)
        {
            /* Process Locked */
            __HAL_LOCK(huart);

            huart->RxISR = UART_RxISR;
            huart->TxISR = UART_TxISR;

            /* Computation of UART mask to apply to RDR register */
            UART_MASK_COMPUTATION(huart);

            huart->ErrorCode = HAL_UART_ERROR_NONE;

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
            /* Process Unlocked */
            __HAL_UNLOCK(huart);

            /* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
            SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);

            HAL_NVIC_SetPriority(USART2_IRQn, 0x04, 0);
            HAL_NVIC_EnableIRQ(USART2_IRQn);               // Enable Interrupt

            status = HAL_OK;
        }
    }

    return status;
}

/* Get byte from receive buffer */
static uint8_t recv_char(uint8_t *symb)
{
    uint8_t addr;

    if(recv_addr_start != recv_addr_end)
    {
        addr = recv_addr_start & 0x7F;
        *symb = recv_buffer[addr];
        addr++;
        if (addr == RECV_BUFFER_LEN)
            recv_addr_start = (~recv_addr_start) & 0x80;
        else
            recv_addr_start ++;
        return 1;
    }
    return 0;
}

/* function of sending data */
static uint8_t send_data(uint8_t *buffer, uint8_t size)
{
    uint8_t count, addr, buf;
    uint8_t change_addr = 0;

    if((send_addr_end & 0x80 ) == (send_addr_start & 0x80))
    {
        count = send_addr_end - send_addr_start;
        count = SEND_BUFFER_LEN - count;
    }
    else
        count = (send_addr_start & 0x7F) - (send_addr_end & 0x7F);
    if (count <= size)
        return 0;
    addr = send_addr_end & 0x7F;
    for(buf = 0;buf < size; buf++)
    {
        send_buffer[addr] = buffer[buf];
        addr++;
        if (addr == SEND_BUFFER_LEN)
        {
            addr = 0;
            change_addr = 1;
        }
    }
    send_addr_end =(change_addr)?(((~send_addr_end)&0x80) | addr):(((send_addr_end)&0x80) | addr);
    if (!send_running)
    {
        send_running = 1;
        SET_BIT(uartHandle.Instance->CR1, USART_CR1_TXEIE); /* If interrupt disabled, enable UART interrupt. */
    }
    return 1;
}

/* Set delay in miliseconds */
static void set_delay(uint32_t msec)
{
    time = msec + 1;
    time_delay = 1;
}

static void reset_check_command(void)
{
    uint8_t i = 0;

    for(; i < COMNUM; i++)
          comrun[i] = 1;
    nlen = 0;
}

static unsigned int parity_even_bit(unsigned int v)
{
	/* unsigned int ov = v; */
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	return (0x6996 >> v) & 1;
}

static uint8_t get_short_command(uint8_t symb)
{
    uint8_t command = 0;

    if(symb & 0x80)
    {
        if(parity_even_bit(symb & 0xFE) == (symb & 1))
        {
            jtag_port_data = symb;
            switch(symb & 0x70)
            {
                case 0x00: // LED
                    command = DEV_INT_COMMAND_SET_LED;
                    break;
                case 0x10:
                    command = DEV_INT_COMMAND_SET_TCK_TMS_TDI;
                    break;
                case 0x20:
                    command = DEV_INT_COMMAND_SET_TRST_SRST;
                    break;
                case 0x30:
                    command = DEV_INT_COMMAND_SET_GET_DATA;
                    break;
                case 0x40:
                    command = DEV_INT_COMMAND_TO_DEV_DATA;
                    break;
                case 0x50:
                    command = DEV_INT_COMMAND_RUN_TEST;
                    break;
                case 0x60:
                    command = DEV_INT_COMMAND_TMS_MOVE;
                    break;
                case 0x70:
                    command = DEV_INT_COMMAND_GET_TDO;
                    break;
                default:
                    break;
            }
        }
    }

    return command;
}

/* Get command */
static uint8_t get_command(uint8_t symb)
{
    uint8_t run_command = 0;
    uint8_t reload_state = 0;
    uint8_t i = 0;

    if(in_jtag_state) /* check only last command "stopjtag" */
        i = COMNUM - 1;

    for (; i < COMNUM; i ++)
    {
        if (comrun[i] == 1)
        {
            if (commands[i][nlen] != symb)
                comrun[i] = 0;
            else
            {
                if(nlen + 1 == comlen [i])
                    run_command = i+1;
                else
                    reload_state = 1;
            }
        }
    }
    nlen ++;

    if (!reload_state)
        reset_check_command();

    return run_command;
}

/* set TCK TMS TDI */
static void set_tck_tms_tdi(uint8_t tck, uint8_t tms, uint8_t tdi)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (tck) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PD10 -> JTAG_TCK
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, (tms) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PD11 -> JTAG_TMS
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, (tdi) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PD12 -> JTAG_TDI
}

/* set  TRST SRST */
static void set_trst_srst(uint8_t trst, uint8_t srst)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, (trst) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PD9 -> JTAG_TRST (JTAG Reset)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, (srst) ? GPIO_PIN_SET : GPIO_PIN_RESET); // PD14 -> POR_B (System Reset)
}

static uint8_t get_tdo(void)
{
    uint8_t tdo = 0;

    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)) // PD13 -> JTAG_TDO
        tdo = 1;

    return tdo;
}

static void recv_scan_data(uint8_t send)
{
    uint32_t count_bits = 0;
    uint8_t tms;
    uint8_t val;
    uint8_t tdi;
    uint8_t bitcounter;
    uint8_t sendbyte;
    uint8_t bcval;

    while(1)
    {
        if(recv_char(&sendbyte))
        {
            for(bitcounter = 0; (bitcounter < 8) && (count_bits < jtag_port_integer); bitcounter ++)
            {
                tms = (count_bits == (jtag_port_integer - 1)) ? 1 : 0;
                bcval = 1 << bitcounter;
                tdi = 0;
                if(sendbyte & bcval)
                    tdi = 1;
                set_tck_tms_tdi(0, tms, tdi);
                val = get_tdo();
                set_tck_tms_tdi(1, tms, tdi);

                if (val)
                    sendbyte |= bcval;
                else
                    sendbyte &= ~bcval;
                count_bits++;
            }

            if(send)
                while(!send_data(&sendbyte, 1));

            if(count_bits >= jtag_port_integer)
                break;
        }
    }
}

static void run_test(void)
{
    uint32_t count_bits;

    for(count_bits = 0; (count_bits < jtag_port_integer); count_bits ++)
    {
        set_tck_tms_tdi(0, 0, 0);
        set_tck_tms_tdi(1, 0, 0);
    }
    set_tck_tms_tdi(0, 0, 0);
}
#if 0
static void send_scan_data(void)
{
    uint32_t count_bits;
    uint8_t tms;
    uint8_t val;
    uint8_t bitcounter = 0;
    uint8_t sendbyte = 0;
    uint8_t bcval;

    for (count_bits = 0; count_bits < jtag_port_integer; count_bits++)
    {
        tms = (count_bits == (jtag_port_integer - 1)) ? 1 : 0;
        bcval = 1 << bitcounter;

        set_tck_tms_tdi(0, tms, 0);
        val = get_tdo();
        set_tck_tms_tdi(1, tms, 0);

        if (val)
            sendbyte |= bcval;
        else
            sendbyte &= ~bcval;

        bitcounter ++;
        if(bitcounter >= 8)
        {
            bitcounter = 0;
            while(!send_data(&sendbyte, 1));
        }
    }

    if(bitcounter)
        while(!send_data(&sendbyte, 1));
}
#endif
static void tms_move(void)
{
    uint8_t byte;
    uint8_t i, count_bits, tms;

    while(1)
    {
        if(recv_char(&byte))
        {
            count_bits = (jtag_port_data & 0x0E) >> 1;
            tms = 0;
            for (i = 0; i < count_bits; i++)
            {
                tms = (byte >> i) & 1;
                set_tck_tms_tdi(0, tms, 0);
                set_tck_tms_tdi(1, tms, 0);
            }
            set_tck_tms_tdi(0, tms, 0);
            break;
        }
    }
}


/* exec command */
static void exec_command(uint8_t command)
{
    switch(command)
    {
        case DEV_INT_COMMAND_RESET: // reset
            NVIC_SystemReset();
            break;
        case DEV_INT_COMMAND_STARTJTAG: // start jtag
            in_jtag_state = 1;
            command = 'e'; // enable jtag commands
            send_data(&command, 1);
            break;
        case DEV_INT_COMMAND_STOPJTAG:
            in_jtag_state = 0;
            command = 'd'; // disable jtag commands
            send_data(&command, 1);
            break;
        case DEV_INT_COMMAND_SET_LED:
            HAL_GPIO_WritePin(LED4_GPIO_PORT, LED4_PIN, (jtag_port_data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case DEV_INT_COMMAND_SET_TCK_TMS_TDI:
            set_tck_tms_tdi(jtag_port_data & 0x08, jtag_port_data & 0x04, jtag_port_data & 0x02);
            break;
        case DEV_INT_COMMAND_SET_TRST_SRST:
            set_trst_srst(jtag_port_data & 0x04, jtag_port_data & 0x02);
            break;
        case DEV_INT_COMMAND_SET_GET_DATA:
        case DEV_INT_COMMAND_TO_DEV_DATA:
        case DEV_INT_COMMAND_RUN_TEST:
            jtag_port_wait_integer = 0;
            jtag_port_integer = 0;
            if(jtag_port_data & 0x08)
                jtag_port_wait_integer ++;
            if(jtag_port_data & 0x04)
                jtag_port_wait_integer ++;
            if(jtag_port_data & 0x02)
                jtag_port_wait_integer ++;
            break;
        case DEV_INT_COMMAND_TMS_MOVE:
            tms_move();
            break;
        case DEV_INT_COMMAND_GET_TDO:
            if(get_tdo())
                command = '1';
            else
                command = '0';
            send_data(&command, 1);
            break;
        case DEV_INT_PROCESS_COMMAND:
            switch(jtag_port_data & 0x70)
            {
                case 0x30: // DEV_INT_COMMAND_SET_GET_DATA
                    recv_scan_data(1);
                    break;
                case 0x40: // DEV_INT_COMMAND_TO_DEV_DATA
                    recv_scan_data(0);
                    break;
                case 0x50: // DEV_INT_COMMAND_RUN_TEST
                    run_test();
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

int main(void)
{
   uint32_t start_tick = 0;
  /* This sample code shows how to use GPIO HAL API to toggle LED4 and LED5 IOs
    in an infinite loop. */

  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 4 MHz */
  SystemClock_Config();

  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
  LED4_GPIO_CLK_ENABLE();
  LED5_GPIO_CLK_ENABLE();

  /* -2- Configure IO in output push-pull mode to drive external LEDs */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = LED4_PIN;
  HAL_GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LED5_PIN;
  HAL_GPIO_Init(LED5_GPIO_PORT, &GPIO_InitStruct);

#if 0
  PD9 -> JTAG_TRST (JTAG Reset) // to up
  PD10 -> JTAG_TCK // no resist
  PD11 -> JTAG_TMS
  PD12 -> JTAG_TDI
  PD13 -> JTAG_TDO
  PD14 -> POR_B (System Reset) // no resist
#endif
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); // PD9 -> JTAG_TRST (JTAG Reset)
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // PD14 -> POR_B (System Reset)

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  if(HAL_UART_Confugure(&uartHandle) == HAL_OK)
  {
    /* -3- Toggle IO in an infinite loop */
    while (1)
    {
        uint8_t byte, command;

        if(recv_char(&byte))
        {
            if(in_jtag_state)
            {
                if(jtag_port_wait_integer)
                {
                    jtag_port_integer <<= 8;
                    jtag_port_integer |= byte;
                    jtag_port_wait_integer --;
                    if(jtag_port_wait_integer == 0)
                        command = DEV_INT_PROCESS_COMMAND;
                    else
                        command = 0;
                }
                else
                {
                    command = get_short_command(byte);
                    if(command == 0)
                        command = get_command(byte);
                }
            }
            else
                command = get_command(byte);

            if(command)
                exec_command(command);
            set_delay(100);
        }
        else
        {
            __asm__("wfi"); // enter to sleep mode
        }

        if (!time_delay)
        {
            reset_check_command();
            set_delay(5000);
        }
        if((HAL_GetTick() - start_tick) >= 500)
        {
            HAL_GPIO_TogglePin(LED5_GPIO_PORT, LED5_PIN);
            start_tick = HAL_GetTick();
        }
    }
    HAL_UART_DeInit(&uartHandle);
  }

  return 1;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* The following clock configuration sets the Clock configuration sets after System reset                */
  /* It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig */
  /* and to be eventually adapted to new clock configuration                                               */

  /* MSI is enabled after System reset at 4Mhz, PLL not used */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  /* Set 0 Wait State flash latency for 4Mhz */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
