#include "brushless.h"
#include "tim.h"
#include "dma.h"

// Buffer size: 16 data bits + 48 reset bits = 64 elements
#define DSHOT_BUFFER_SIZE 64

static uint32_t motor1_buffer[DSHOT_BUFFER_SIZE];
static uint32_t motor2_buffer[DSHOT_BUFFER_SIZE];

static void DShot_PrepareBuffer(uint16_t value, uint32_t *buffer)
{
    uint16_t packet;
    uint16_t checksum = 0;
    
    packet = value << 1; // Telemetry bit = 0
    
    uint16_t temp_packet = packet;
    for (int i = 0; i < 3; i++)
    {
        checksum ^= (temp_packet & 0x0F);
        temp_packet >>= 4;
    }
    checksum &= 0x0F;
    packet = (packet << 4) | checksum;

    uint32_t temp_buf[16];
    for (int i = 0; i < 16; i++)
    {
        if (packet & (0x8000 >> i))
        {
            temp_buf[i] = 118; 
        }
        else
        {
            temp_buf[i] = 58;  
        }
    }

    for (int i = 0; i < 16; i++)
    {
        buffer[i] = temp_buf[i];
    }
}

void Brushless_Init(void)
{
    for (int i = 0; i < DSHOT_BUFFER_SIZE; i++) {
        motor1_buffer[i] = 0;
        motor2_buffer[i] = 0;
    }
    
    DShot_PrepareBuffer(0, motor1_buffer);
    DShot_PrepareBuffer(0, motor2_buffer);
    
    __HAL_TIM_MOE_ENABLE(&htim1);
    
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, motor1_buffer, DSHOT_BUFFER_SIZE);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, motor2_buffer, DSHOT_BUFFER_SIZE);
}

void Brushless_SetSpeed(uint16_t motor1, uint16_t motor2)
{
    static uint16_t last_m1 = 0xFFFF;
    static uint16_t last_m2 = 0xFFFF;

    if (motor1 > DSHOT_MAX_VALUE) motor1 = DSHOT_MAX_VALUE;
    if (motor2 > DSHOT_MAX_VALUE) motor2 = DSHOT_MAX_VALUE;

    if (motor1 != last_m1) {
        DShot_PrepareBuffer(motor1, motor1_buffer);
        last_m1 = motor1;
    }
    if (motor2 != last_m2) {
        DShot_PrepareBuffer(motor2, motor2_buffer);
        last_m2 = motor2;
    }
}
