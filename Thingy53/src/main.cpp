#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include "ei-model/edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "ei-model/edge-impulse-sdk/dsp/numpy.hpp"
#include <nrfx_clock.h>

int ret;
uint8_t config[1] = {1}; // for i2c communications
bool flag = false;
bool anomalyDetected = false;
uint8_t counter = 0;

// the ring buffer
#define BUF_SIZE 15
float buffer[BUF_SIZE];
uint8_t write = 0;
uint8_t read = 0;

void put(float item)
{
	buffer[write++] = item;
	write %= BUF_SIZE;
	read = write;
}

float get()
{
	float item = buffer[read++];
	read %= BUF_SIZE;
	return item;
}

void get_buffer(float _buffer[], uint8_t SIZE)
{
	for (uint8_t i = 0; i < SIZE; i++)
	{
		_buffer[i] = get();
	}
}
//////////////////////////////////////////////

// to recieve the float value from Arduino over I2C (convert from bytes to float)
union FloatI2C
{
	float m_float;
	uint8_t m_bytes[sizeof(float)];
} num;

// features to perform the inference
static float features[BUF_SIZE];

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 200

#define LED0_NODE DT_ALIAS(led1)
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#define I2C1_NODE DT_NODELABEL(mysensor)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

int raw_features_get_data(size_t offset, size_t length, float *out_ptr)
{
	memcpy(out_ptr, features + offset, length * sizeof(float));
	return 0;
}

static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (anomalyDetected)
	{
		gpio_pin_set_dt(&led, 0);
		config[0] = 1;
		flag = false;
		counter = 0;
		anomalyDetected = false;
	}
}

int main(void)
{
	// This is needed so that output of printf is output immediately without buffering
	setvbuf(stdout, NULL, _IONBF, 0);
	nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ei_impulse_result_t result = {0};

	if (!device_is_ready(dev_i2c.bus))
		return 1;

	if (!device_is_ready(led.port))
		return 1;

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
		return 1;

	if (!device_is_ready(button.port))
		return 1;

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0)
		return 1;

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	gpio_pin_set_dt(&led, 0);
	i2c_write_dt(&dev_i2c, config, sizeof(config));
	k_msleep(5000);

	while (1)
	{
		if (anomalyDetected)
		{
			k_msleep(SLEEP_TIME_MS);
		}
		else
		{
			ret = i2c_write_read_dt(&dev_i2c, config, sizeof(config), num.m_bytes, 4);
			// printk("%f", num.m_float);
			put(num.m_float);
			counter++;
			if (!flag)
			{
				if (counter >= 50)
				{
					counter = 0;
					flag = true;
				}
			}
			else if (counter == 5)
			{
				counter = 0;
				get_buffer(features, BUF_SIZE);

				// anomaly edge-impulse
				signal_t features_signal;
				features_signal.total_length = sizeof(features) / sizeof(features[0]);
				features_signal.get_data = &raw_features_get_data;

				// invoke the impulse
				// printk("impulse time!\n\r");
				EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, true);
				// printk("run_classifier returned: %f\n", result.anomaly);
				if (result.anomaly < 0.4)	  // the threshold for anomaly
					gpio_pin_set_dt(&led, 0); // led off
				else
				{
					gpio_pin_set_dt(&led, 1); // led on, anomaly detected
					config[0] = 0;
					i2c_write_dt(&dev_i2c, config, sizeof(config));
					anomalyDetected = true;
				}
			}
			k_msleep(SLEEP_TIME_MS);
		}
	}
}