/*
 * Copyright (c) 2026 EmbedConsult
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_i2c

#include <errno.h>

#include <zephyr/drivers/clock_control/mspm0_clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ti/driverlib/dl_i2c.h>

#include "i2c-priv.h"

LOG_MODULE_REGISTER(i2c_mspm0, CONFIG_I2C_LOG_LEVEL);

#define MSPM0_I2C_IRQS                                                                        \
	(DL_I2C_INTERRUPT_TARGET_START | DL_I2C_INTERRUPT_TARGET_STOP |                       \
	 DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER | DL_I2C_INTERRUPT_TARGET_RX_DONE |          \
	 DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY | DL_I2C_INTERRUPT_TARGET_TX_DONE |            \
	 DL_I2C_INTERRUPT_TARGET_RXFIFO_OVERFLOW | DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW)

#define MSPM0_I2C_CLK_DIV(n) CONCAT(DL_I2C_CLOCK_DIVIDE_, DT_INST_PROP(n, clk_div))

struct i2c_mspm0_config {
	I2C_Regs *regs;
	const struct pinctrl_dev_config *pinctrl;
	DL_I2C_ClockConfig clock_cfg;
	uint32_t bitrate;
#ifdef CONFIG_I2C_TARGET
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct i2c_mspm0_data {
	uint32_t dev_config;
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target_cfg;
	enum {
		MSPM0_I2C_TARGET_IDLE,
		MSPM0_I2C_TARGET_READ,
		MSPM0_I2C_TARGET_WRITE,
	} direction;
	bool first_read;
#endif
};

#ifdef CONFIG_I2C_TARGET
static uint32_t i2c_mspm0_iidx_to_mask(DL_I2C_IIDX iidx)
{
	switch (iidx) {
	case DL_I2C_IIDX_TARGET_START:
		return DL_I2C_INTERRUPT_TARGET_START;
	case DL_I2C_IIDX_TARGET_STOP:
		return DL_I2C_INTERRUPT_TARGET_STOP;
	case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
		return DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER;
	case DL_I2C_IIDX_TARGET_RX_DONE:
		return DL_I2C_INTERRUPT_TARGET_RX_DONE;
	case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
		return DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY;
	case DL_I2C_IIDX_TARGET_TX_DONE:
		return DL_I2C_INTERRUPT_TARGET_TX_DONE;
	case DL_I2C_IIDX_TARGET_RXFIFO_OVERFLOW:
		return DL_I2C_INTERRUPT_TARGET_RXFIFO_OVERFLOW;
	case DL_I2C_IIDX_TARGET_TXFIFO_UNDERFLOW:
		return DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW;
	default:
		return 0U;
	}
}

static void i2c_mspm0_target_load_tx(const struct device *dev)
{
	struct i2c_mspm0_data *data = dev->data;
	struct i2c_target_config *target_cfg = data->target_cfg;
	uint8_t value = 0xffU;
	int ret;

	if (target_cfg == NULL) {
		return;
	}

	if (data->first_read) {
		ret = target_cfg->callbacks->read_requested(target_cfg, &value);
		data->first_read = false;
	} else if (target_cfg->callbacks->read_processed != NULL) {
		ret = target_cfg->callbacks->read_processed(target_cfg, &value);
	} else {
		ret = target_cfg->callbacks->read_requested(target_cfg, &value);
	}

	if (ret == 0) {
		DL_I2C_transmitTargetData(((const struct i2c_mspm0_config *)dev->config)->regs, value);
	} else {
		LOG_DBG("target read callback rejected byte: %d", ret);
	}
}

static void i2c_mspm0_target_drain_rx(const struct device *dev)
{
	const struct i2c_mspm0_config *config = dev->config;
	struct i2c_mspm0_data *data = dev->data;
	struct i2c_target_config *target_cfg = data->target_cfg;
	const struct i2c_target_callbacks *cb;
	uint8_t value;
	int ret;

	if (target_cfg == NULL) {
		return;
	}

	cb = target_cfg->callbacks;
	if (cb->write_received == NULL) {
		while (DL_I2C_receiveTargetDataCheck(config->regs, &value)) {
		}
		return;
	}

	while (DL_I2C_receiveTargetDataCheck(config->regs, &value)) {
		ret = cb->write_received(target_cfg, value);
		if (ret != 0) {
			LOG_DBG("target write callback rejected byte: %d", ret);
			break;
		}
	}
}

static int i2c_mspm0_target_sync_direction(const struct device *dev)
{
	const struct i2c_mspm0_config *config = dev->config;
	struct i2c_mspm0_data *data = dev->data;
	struct i2c_target_config *target_cfg = data->target_cfg;
	const struct i2c_target_callbacks *cb;
	uint32_t status;
	int previous_direction = data->direction;

	if (target_cfg == NULL) {
		return MSPM0_I2C_TARGET_IDLE;
	}

	status = DL_I2C_getTargetStatus(config->regs);
	cb = target_cfg->callbacks;

	if (status & DL_I2C_TARGET_STATUS_TRANSMIT_REQUEST) {
		if (previous_direction != MSPM0_I2C_TARGET_READ) {
			data->direction = MSPM0_I2C_TARGET_READ;
			data->first_read = true;
			i2c_mspm0_target_load_tx(dev);
		}
		return previous_direction;
	}

	if (status & DL_I2C_TARGET_STATUS_RECEIVE_REQUEST) {
		if (previous_direction != MSPM0_I2C_TARGET_WRITE) {
			data->direction = MSPM0_I2C_TARGET_WRITE;
			data->first_read = true;
			if (cb->write_requested != NULL) {
				(void)cb->write_requested(target_cfg);
			}
		}
		return previous_direction;
	}

	return previous_direction;
}

static void i2c_mspm0_target_start(const struct device *dev)
{
	const struct i2c_mspm0_config *config = dev->config;
	struct i2c_mspm0_data *data = dev->data;

	DL_I2C_flushTargetRXFIFO(config->regs);
	DL_I2C_flushTargetTXFIFO(config->regs);
	data->direction = MSPM0_I2C_TARGET_IDLE;
	data->first_read = true;
	(void)i2c_mspm0_target_sync_direction(dev);
}

static void i2c_mspm0_target_stop(const struct device *dev)
{
	struct i2c_mspm0_data *data = dev->data;
	struct i2c_target_config *target_cfg = data->target_cfg;

	if ((target_cfg != NULL) && (target_cfg->callbacks->stop != NULL)) {
		(void)target_cfg->callbacks->stop(target_cfg);
	}

	data->direction = MSPM0_I2C_TARGET_IDLE;
	data->first_read = true;
}

static void i2c_mspm0_isr(const struct device *dev)
{
	const struct i2c_mspm0_config *config = dev->config;
	struct i2c_mspm0_data *data = dev->data;
	DL_I2C_IIDX iidx;
	uint32_t mask;

	if (data->target_cfg == NULL) {
		DL_I2C_clearInterruptStatus(config->regs,
					    DL_I2C_getEnabledInterruptStatus(config->regs,
									 MSPM0_I2C_IRQS));
		return;
	}

	for (iidx = DL_I2C_getPendingInterrupt(config->regs);
	     iidx != DL_I2C_IIDX_NO_INT;
	     iidx = DL_I2C_getPendingInterrupt(config->regs)) {
		mask = i2c_mspm0_iidx_to_mask(iidx);

		switch (iidx) {
		case DL_I2C_IIDX_TARGET_START:
			i2c_mspm0_target_start(dev);
			break;
		case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
		case DL_I2C_IIDX_TARGET_RX_DONE:
			(void)i2c_mspm0_target_sync_direction(dev);
			if (data->direction == MSPM0_I2C_TARGET_WRITE) {
				i2c_mspm0_target_drain_rx(dev);
			}
			break;
		case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
		case DL_I2C_IIDX_TARGET_TX_DONE:
			if (i2c_mspm0_target_sync_direction(dev) == MSPM0_I2C_TARGET_READ &&
			    data->direction == MSPM0_I2C_TARGET_READ) {
				i2c_mspm0_target_load_tx(dev);
			}
			break;
		case DL_I2C_IIDX_TARGET_STOP:
			i2c_mspm0_target_drain_rx(dev);
			i2c_mspm0_target_stop(dev);
			break;
		case DL_I2C_IIDX_TARGET_RXFIFO_OVERFLOW:
		case DL_I2C_IIDX_TARGET_TXFIFO_UNDERFLOW:
			LOG_DBG("target fifo error iidx=%u", iidx);
			break;
		default:
			break;
		}

		if (mask != 0U) {
			DL_I2C_clearInterruptStatus(config->regs, mask);
		}
	}
}

static int i2c_mspm0_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct i2c_mspm0_config *config = dev->config;
	struct i2c_mspm0_data *data = dev->data;
	unsigned int key;

	if (cfg == NULL) {
		return -EINVAL;
	}

	if (cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	key = irq_lock();
	if (data->target_cfg != NULL) {
		irq_unlock(key);
		return -EBUSY;
	}

	data->target_cfg = cfg;
	data->direction = MSPM0_I2C_TARGET_IDLE;
	data->first_read = true;

	DL_I2C_disableInterrupt(config->regs, MSPM0_I2C_IRQS);
	DL_I2C_clearInterruptStatus(config->regs, MSPM0_I2C_IRQS);
	DL_I2C_flushTargetRXFIFO(config->regs);
	DL_I2C_flushTargetTXFIFO(config->regs);
	DL_I2C_setTargetAddressingMode(config->regs, DL_I2C_TARGET_ADDRESSING_MODE_7_BIT);
	DL_I2C_setTargetOwnAddress(config->regs, cfg->address);
	DL_I2C_enableTargetOwnAddress(config->regs);
	DL_I2C_setTargetRXFIFOThreshold(config->regs, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
	DL_I2C_setTargetTXFIFOThreshold(config->regs, DL_I2C_TX_FIFO_LEVEL_EMPTY);
	DL_I2C_setTargetACKOverrideValue(config->regs, DL_I2C_TARGET_RESPONSE_OVERRIDE_VALUE_ACK);
	DL_I2C_disableTargetACKOverride(config->regs);
	DL_I2C_disableACKOverrideOnStart(config->regs);
	DL_I2C_enableTargetClockStretching(config->regs);
	DL_I2C_enableTargetRXFullOnRXRequest(config->regs);
	DL_I2C_enableTargetTXEmptyOnTXRequest(config->regs);
	DL_I2C_enableTarget(config->regs);
	DL_I2C_enableInterrupt(config->regs, MSPM0_I2C_IRQS);
	irq_unlock(key);

	return 0;
}

static int i2c_mspm0_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct i2c_mspm0_config *config = dev->config;
	struct i2c_mspm0_data *data = dev->data;
	unsigned int key;

	ARG_UNUSED(cfg);

	key = irq_lock();
	if (data->target_cfg == NULL) {
		irq_unlock(key);
		return -EINVAL;
	}

	DL_I2C_disableInterrupt(config->regs, MSPM0_I2C_IRQS);
	DL_I2C_clearInterruptStatus(config->regs, MSPM0_I2C_IRQS);
	DL_I2C_disableTargetOwnAddress(config->regs);
	DL_I2C_disableTarget(config->regs);
	DL_I2C_flushTargetRXFIFO(config->regs);
	DL_I2C_flushTargetTXFIFO(config->regs);
	data->target_cfg = NULL;
	data->direction = MSPM0_I2C_TARGET_IDLE;
	data->first_read = true;
	irq_unlock(key);

	return 0;
}
#endif

static int i2c_mspm0_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_mspm0_data *data = dev->data;

	if (dev_config & I2C_MODE_CONTROLLER) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
	case I2C_SPEED_FAST:
	case I2C_SPEED_DT:
		break;
	default:
		return -EINVAL;
	}

	data->dev_config = dev_config;

	return 0;
}

static int i2c_mspm0_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mspm0_data *data = dev->data;

	*dev_config = data->dev_config;

	return 0;
}

static int i2c_mspm0_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			      uint16_t addr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(msgs);
	ARG_UNUSED(num_msgs);
	ARG_UNUSED(addr);

	return -ENOTSUP;
}

static int i2c_mspm0_init(const struct device *dev)
{
	const struct i2c_mspm0_config *config = dev->config;
	struct i2c_mspm0_data *data = dev->data;
	int ret;

	DL_I2C_reset(config->regs);
	DL_I2C_enablePower(config->regs);
	delay_cycles(CONFIG_MSPM0_PERIPH_STARTUP_DELAY);

	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	DL_I2C_setClockConfig(config->regs, &config->clock_cfg);
	DL_I2C_enableAnalogGlitchFilter(config->regs);
	DL_I2C_setTargetACKOverrideValue(config->regs, DL_I2C_TARGET_RESPONSE_OVERRIDE_VALUE_ACK);
	DL_I2C_disableTargetACKOverride(config->regs);
	DL_I2C_disableACKOverrideOnStart(config->regs);
	DL_I2C_disableInterrupt(config->regs, MSPM0_I2C_IRQS);
	DL_I2C_clearInterruptStatus(config->regs, MSPM0_I2C_IRQS);

	data->dev_config = i2c_map_dt_bitrate(config->bitrate);

#ifdef CONFIG_I2C_TARGET
	config->irq_config_func(dev);
#endif

	return 0;
}

static DEVICE_API(i2c, i2c_mspm0_driver_api) = {
	.configure = i2c_mspm0_configure,
	.get_config = i2c_mspm0_get_config,
	.transfer = i2c_mspm0_transfer,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_mspm0_target_register,
	.target_unregister = i2c_mspm0_target_unregister,
#endif
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

#ifdef CONFIG_I2C_TARGET
#define MSPM0_I2C_IRQ_DEFINE(inst)                                                                \
	static void i2c_mspm0_irq_config_##inst(const struct device *dev)                        \
	{                                                                                         \
		ARG_UNUSED(dev);                                                                  \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), i2c_mspm0_isr,      \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                  \
	}
#else
#define MSPM0_I2C_IRQ_DEFINE(inst)
#endif

#define MSPM0_I2C_INIT(inst)                                                                      \
	PINCTRL_DT_INST_DEFINE(inst);                                                             \
	MSPM0_I2C_IRQ_DEFINE(inst);                                                               \
	static const struct i2c_mspm0_config i2c_mspm0_cfg_##inst = {                            \
		.regs = (I2C_Regs *)DT_INST_REG_ADDR(inst),                                       \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                  \
		.bitrate = DT_INST_PROP(inst, clock_frequency),                                   \
		.clock_cfg = {                                                                    \
			.clockSel = MSPM0_CLOCK_PERIPH_REG_MASK(DT_INST_CLOCKS_CELL(inst, clk)), \
			.divideRatio = MSPM0_I2C_CLK_DIV(inst),                                  \
		},                                                                                \
		IF_ENABLED(CONFIG_I2C_TARGET, (.irq_config_func = i2c_mspm0_irq_config_##inst,)) \
	};                                                                                        \
	static struct i2c_mspm0_data i2c_mspm0_data_##inst;                                       \
	I2C_DEVICE_DT_INST_DEFINE(inst, i2c_mspm0_init, NULL, &i2c_mspm0_data_##inst,            \
				  &i2c_mspm0_cfg_##inst, POST_KERNEL,                           \
				  CONFIG_I2C_INIT_PRIORITY, &i2c_mspm0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSPM0_I2C_INIT)
