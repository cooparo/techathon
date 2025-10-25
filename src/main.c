#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h> // Include for printk

/* Get node IDs for the LEDs using aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

/* Check if the aliases are defined and okay */
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined or enabled"
#endif
#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined or enabled"
#endif
#if !DT_NODE_HAS_STATUS(LED2_NODE, okay)
#error "Unsupported board: led2 devicetree alias is not defined or enabled"
#endif
#if !DT_NODE_HAS_STATUS(LED3_NODE, okay)
#error "Unsupported board: led3 devicetree alias is not defined or enabled"
#endif

/* Get the gpio_dt_spec structs for the LEDs */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/* Define sleep time */
#define SLEEP_TIME_MS 500

void main(void)
{
    int ret;

    printk("Starting Blinky for nRF54L15 PDK...\n");

    /* Configure LED pins */
    if (!gpio_is_ready_dt(&led0)) {
        printk("Error: LED0 device %s is not ready\n", led0.port->name);
        return;
    }
    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: failed to configure LED0 pin %d\n", ret, led0.pin);
        return;
    }

    if (!gpio_is_ready_dt(&led1)) {
        printk("Error: LED1 device %s is not ready\n", led1.port->name);
        return;
    }
    ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: failed to configure LED1 pin %d\n", ret, led1.pin);
        return;
    }

    if (!gpio_is_ready_dt(&led2)) {
        printk("Error: LED2 device %s is not ready\n", led2.port->name);
        return;
    }
    ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: failed to configure LED2 pin %d\n", ret, led2.pin);
        return;
    }

    if (!gpio_is_ready_dt(&led3)) {
        printk("Error: LED3 device %s is not ready\n", led3.port->name);
        return;
    }
    ret = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: failed to configure LED3 pin %d\n", ret, led3.pin);
        return;
    }

    printk("LEDs configured. Starting blink loop.\n");

    /* Blink loop */
    while (1) {
        ret = gpio_pin_toggle_dt(&led0);
        if (ret < 0) {
            printk("Error toggling LED0: %d\n", ret);
        }
        ret = gpio_pin_toggle_dt(&led1);
        if (ret < 0) {
            printk("Error toggling LED1: %d\n", ret);
        }
        ret = gpio_pin_toggle_dt(&led2);
        if (ret < 0) {
            printk("Error toggling LED2: %d\n", ret);
        }
        ret = gpio_pin_toggle_dt(&led3);
        if (ret < 0) {
            printk("Error toggling LED3: %d\n", ret);
        }

        k_msleep(SLEEP_TIME_MS);
    }
}
