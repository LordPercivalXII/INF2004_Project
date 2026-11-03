/* Standalone LSM303DLHC diagnosis. No robot tasks or motor outputs.
 * Grove 3: GP4 SDA, GP5 SCL. Flash the robot UF2 to restore the car.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define SDA_PIN 4U
#define SCL_PIN 5U
#define TRANSFER_TIMEOUT_US 20000U

static bool
read_registers(uint8_t address, uint8_t reg, uint8_t * data, size_t count)
{
    int const written = i2c_write_timeout_us(i2c0, address, &reg, 1U,
                                             true, TRANSFER_TIMEOUT_US);
    if (written != 1)
    {
        printf("SDK: addr=0x%02x register write failed=%d\n", address, written);
        return false;
    }
    int const received = i2c_read_timeout_us(i2c0, address, data, count,
                                             false, TRANSFER_TIMEOUT_US);
    if (received != (int) count)
    {
        printf("SDK: addr=0x%02x read failed=%d\n", address, received);
        return false;
    }
    return true;
}

static bool
write_register(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t const data[2] = {reg, value};
    int const result = i2c_write_timeout_us(i2c0, address, data, 2U,
                                            false, TRANSFER_TIMEOUT_US);
    if (result != 2)
    {
        printf("SDK: addr=0x%02x config reg=0x%02x failed=%d\n",
               address, reg, result);
    }
    return result == 2;
}

static int16_t
accel_word(uint8_t const * bytes)
{
    return (int16_t) (((uint16_t) bytes[1] << 8U) | bytes[0]);
}

int
main(void)
{
    stdio_init_all();
    i2c_init(i2c0, 100000U);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    while (!stdio_usb_connected())
    {
        sleep_ms(100);
    }
    sleep_ms(500);

    for (;;)
    {
        unsigned int found = 0U;
        bool accel_present = false;
        bool mag_present = false;
        printf("\nSDK IMU TEST: independent driver, GP4/GP5, 100 kHz\n");
        printf("SDK: idle SDA=%u SCL=%u (normally both 1)\n",
               gpio_get(SDA_PIN), gpio_get(SCL_PIN));
        for (uint8_t candidate = 0x08U; candidate <= 0x77U; candidate++)
        {
            uint8_t byte;
            int const result = i2c_read_timeout_us(i2c0, candidate, &byte,
                                        1U, false, TRANSFER_TIMEOUT_US);
            if (result == 1)
            {
                printf("SDK: device ACK at 0x%02x\n", candidate);
                found++;
                accel_present = accel_present || (candidate == 0x19U);
                mag_present = mag_present || (candidate == 0x1EU);
            }
        }
        printf("SDK: scan found %u device(s)\n", found);
        if (accel_present)
        {
            uint8_t identity;
            if (read_registers(0x19U, 0x0FU, &identity, 1U))
            {
                printf("SDK: 0x19 WHO_AM_I=0x%02x "
                       "(0x33 matches several ST accelerometers)\n", identity);
            }
        }
        if (mag_present)
        {
            uint8_t identity[3];
            if (read_registers(0x1EU, 0x0AU, identity, sizeof(identity)))
            {
                printf("SDK: 0x1e ID_A/B/C=0x%02x,0x%02x,0x%02x "
                       "(0x48,0x34,0x33 is H43)\n",
                       identity[0], identity[1], identity[2]);
            }
        }
        if (accel_present && mag_present)
        {
            uint8_t accel_id = 0;
            uint8_t mag_id[3] = {0};
            if (read_registers(0x19U, 0x0FU, &accel_id, 1U) &&
                read_registers(0x1EU, 0x0AU, mag_id, 3U) &&
                (accel_id == 0x33U) && (mag_id[0] == 0x48U) &&
                (mag_id[1] == 0x34U) && (mag_id[2] == 0x33U) &&
                write_register(0x19U, 0x23U, 0x98U) &&
                write_register(0x19U, 0x20U, 0x57U) &&
                write_register(0x1EU, 0x00U, 0x18U) &&
                write_register(0x1EU, 0x01U, 0x20U) &&
                write_register(0x1EU, 0x02U, 0x00U))
            {
                printf("SDK: LSM303DLHC configured; NO gyroscope\n");
                sleep_ms(100);
                for (unsigned int sample = 0U; sample < 20U; sample++)
                {
                    uint8_t data[6];
                    if (!read_registers(0x19U, 0xA8U, data, sizeof(data)))
                    {
                        break;
                    }
                    printf("SDK: accel_mg=%d,%d,%d\n",
                        (int) (accel_word(&data[0]) / 16 * 2),
                        (int) (accel_word(&data[2]) / 16 * 2),
                        (int) (accel_word(&data[4]) / 16 * 2));
                    sleep_ms(250);
                }
            }
            else
            {
                printf("SDK: DLHC identity/configuration failed; "
                       "raw sampling not started\n");
            }
        }
        else
        {
            printf("SDK: expected DLHC pair 0x19/0x1e not found. LSM303D "
                   "at 0x1d needs a different driver/register map.\n");
        }
        sleep_ms(3000);
    }
}
