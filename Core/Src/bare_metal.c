#include "stm32f4xx.h"

#define BMP280_ADDR         0x76
#define BMP280_CHIP_ID_REG  0xD0
#define BMP280_CHIP_ID      0x58

static void delay(volatile uint32_t count)
{
    while (count--)
    {
        __NOP();
    }
}

static void led_init(void)
{
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA5 as output
    GPIOA->MODER &= ~(3U << (5 * 2));
    GPIOA->MODER |=  (1U << (5 * 2));

    // Push-pull
    GPIOA->OTYPER &= ~(1U << 5);

    // Medium speed
    GPIOA->OSPEEDR &= ~(3U << (5 * 2));
    GPIOA->OSPEEDR |=  (1U << (5 * 2));

    // No pull-up/pull-down
    GPIOA->PUPDR &= ~(3U << (5 * 2));
}

static void led_on(void)
{
    GPIOA->BSRR = (1U << 5);
}

static void led_off(void)
{
    GPIOA->BSRR = (1U << (5 + 16));
}

static void led_toggle(void)
{
    GPIOA->ODR ^= (1U << 5);
}

static void i2c1_gpio_init(void)
{
    // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // PB8, PB9 -> Alternate Function mode
    GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->MODER |=  ((2U << (8 * 2)) | (2U << (9 * 2)));

    // Open-drain
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);

    // High speed
    GPIOB->OSPEEDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->OSPEEDR |=  ((3U << (8 * 2)) | (3U << (9 * 2)));

    // Pull-up
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->PUPDR |=  ((1U << (8 * 2)) | (1U << (9 * 2)));

    // AF4 for I2C1 on PB8 and PB9
    GPIOB->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((9 - 8) * 4)));
    GPIOB->AFR[1] |=  ((4U   << ((8 - 8) * 4)) | (4U   << ((9 - 8) * 4)));
}

static void i2c1_init(void)
{
    // Enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Disable I2C before configuring
    I2C1->CR1 &= ~I2C_CR1_PE;

    // Peripheral clock = 16 MHz
    I2C1->CR2 = 16;

    // Standard mode 100 kHz
    // CCR = Fpclk1 / (2 * Fscl) = 16MHz / (2 * 100kHz) = 80
    I2C1->CCR = 80;

    // Maximum rise time for standard mode
    // TRISE = Fpclk1(MHz) + 1 = 16 + 1 = 17
    I2C1->TRISE = 17;

    // Enable I2C
    I2C1->CR1 |= I2C_CR1_PE;
}

static void i2c1_stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

static int i2c1_check_address(uint8_t addr7)
{
    volatile uint32_t temp;
    uint32_t timeout = 100000;

    // Wait until bus is not busy
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)
        ;

    if (timeout == 0)
        return 0;

    // Generate START
    I2C1->CR1 |= I2C_CR1_START;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)
        ;

    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    // Send address with write bit
    I2C1->DR = (addr7 << 1);

    timeout = 100000;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && timeout--)
        ;

    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    // ACK received
    if (I2C1->SR1 & I2C_SR1_ADDR)
    {
        temp = I2C1->SR1;
        temp = I2C1->SR2;
        (void)temp;

        i2c1_stop();
        return 1;
    }

    // NACK received
    I2C1->SR1 &= ~I2C_SR1_AF;
    i2c1_stop();

    return 0;
}

static int i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t data)
{
    volatile uint32_t temp;
    uint32_t timeout = 100000;

    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)
        ;
    if (timeout == 0)
        return 0;

    // START
    I2C1->CR1 |= I2C_CR1_START;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)
        ;
    if (timeout == 0)
        return 0;

    // Address + write
    I2C1->DR = (addr7 << 1);

    timeout = 100000;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    if (I2C1->SR1 & I2C_SR1_AF)
    {
        I2C1->SR1 &= ~I2C_SR1_AF;
        i2c1_stop();
        return 0;
    }

    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;

    // Send register address
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    I2C1->DR = reg;

    // Send data
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    I2C1->DR = data;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    i2c1_stop();
    return 1;
}

static int i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *data)
{
    volatile uint32_t temp;
    uint32_t timeout = 100000;

    // Wait until bus free
    while ((I2C1->SR2 & I2C_SR2_BUSY) && timeout--)
        ;
    if (timeout == 0)
        return 0;

    // -------- First START: write register address --------
    I2C1->CR1 |= I2C_CR1_START;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)
        ;
    if (timeout == 0)
        return 0;

    // Address + write
    I2C1->DR = (addr7 << 1);

    timeout = 100000;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    if (I2C1->SR1 & I2C_SR1_AF)
    {
        I2C1->SR1 &= ~I2C_SR1_AF;
        i2c1_stop();
        return 0;
    }

    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;

    // Send register address
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    I2C1->DR = reg;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    // -------- Repeated START: read data --------
    I2C1->CR1 |= I2C_CR1_START;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    // Address + read
    I2C1->DR = (addr7 << 1) | 1U;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--)
        ;
    if (timeout == 0)
    {
        i2c1_stop();
        return 0;
    }

    // Single-byte read:
    // Disable ACK before clearing ADDR
    I2C1->CR1 &= ~I2C_CR1_ACK;

    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;

    // Generate STOP
    I2C1->CR1 |= I2C_CR1_STOP;

    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_RXNE) && timeout--)
        ;
    if (timeout == 0)
        return 0;

    *data = (uint8_t)I2C1->DR;

    // Re-enable ACK for future use
    I2C1->CR1 |= I2C_CR1_ACK;

    return 1;
}

int main(void)
{
    uint8_t chip_id = 0;
    int found = 0;
    int read_ok = 0;

    led_init();
    i2c1_gpio_init();
    i2c1_init();

    found = i2c1_check_address(BMP280_ADDR);

    if (found)
    {
        read_ok = i2c1_read_reg(BMP280_ADDR, BMP280_CHIP_ID_REG, &chip_id);
    }

    while (1)
    {
        if (found && read_ok && (chip_id == BMP280_CHIP_ID))
        {
            // Fast blink = success
            led_toggle();
            delay(200000);
        }
        else
        {
            // Slow blink = fail
            led_toggle();
            delay(800000);
        }
    }
}
