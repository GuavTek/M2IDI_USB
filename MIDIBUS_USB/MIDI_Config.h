/*
 * MIDI_Config.h
 *
 * Created: 15/10/2021 22:31:05
 *  Author: GuavTek
 */ 

// Configurations for MIDI application

#ifndef MIDI_CONFIG_H_
#define MIDI_CONFIG_H_

#include "MCP2517.h"
#include "SPI_SAMD.h"

// Define CAN filters
CAN_Filter_t CAN_FLT0 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = false,
	.ID = (0 << 10),
	.matchBothIDTypes = false,
	.maskID = 1 << 10
};

CAN_Filter_t CAN_FLT1 = {
	.enabled = true,
	.fifoDestination = 1,
	.extendedID = false,
	.ID = (1 << 10) | 6969,
	.matchBothIDTypes = false,
	.maskID = 0x7ff
};

// Define FIFO configurations
CAN_FIFO_t CAN_FIFO1 = {
	.enabled = true,
	.payloadSize = 16,
	.fifoDepth = 31,
	.retransmitAttempt = CAN_FIFO_t::unlimited,
	.messagePriority = 0,
	.txEnable = false,
	.autoRemote = false,
	.receiveTimestamp = false,
	.exhaustedTxInterrupt = false,
	.overflowInterrupt = false,
	.fullEmptyInterrupt = false,
	.halfFullEmptyInterrupt = false,
	.notFullEmptyInterrupt = true
};

CAN_FIFO_t CAN_FIFO2 = {
	.enabled = true,
	.payloadSize = 16,
	.fifoDepth = 31,
	.retransmitAttempt = CAN_FIFO_t::unlimited,
	.messagePriority = 0,
	.txEnable = true,
	.autoRemote = false,
	.receiveTimestamp = false,
	.exhaustedTxInterrupt = false,
	.overflowInterrupt = false,
	.fullEmptyInterrupt = false,
	.halfFullEmptyInterrupt = false,
	.notFullEmptyInterrupt = false
};


const spi_config_t SPI_CONF = {
	.sercomNum = 5,
	.dipoVal = 0x3,
	.dopoVal = 0x0,
	.speed = 8000000,
	.pinmux_mosi = PINMUX_PB02D_SERCOM5_PAD0,
	.pinmux_miso = PINMUX_PB23D_SERCOM5_PAD3,
	.pinmux_sck = PINMUX_PB03D_SERCOM5_PAD1,
	.num_cs = 1,
	.pin_cs = {PIN_PB22}
};

const CAN_Config_t CAN_CONF = {
	.comSlaveNum = 0,
	.clkOutDiv = CAN_Config_t::clkOutDiv1,
	.sysClkDiv = false,
	.clkDisable = false,
	.pllEnable = false,
	.txBandwidthShare = CAN_Config_t::BW_Share4,
	.opMode = CAN_MODE_E::Normal_FD,
	.txQueueEnable = false,
	.txEventStore = false,
	.listenOnlyOnError = false,
	.restrictRetransmit = false,
	.disableBitrateSwitch = false,
	.wakeFilter = CAN_Config_t::Wake_Filter_40_75ns,
	.enableWakeFilter = false,
	.disableProtocolException = false,
	.enableIsoCrc = true,
	.deviceNetFilterBits = 0,
	.nominalBaudPrescaler = 1,	// Time quanta prescaler Tq = 2/20MHz = 100ns
	.nominalTSEG1 = 5,			// Time segment 1 = 6 Tq
	.nominalTSEG2 = 2,			// Time segment 2 = 3 Tq
	.nominalSyncJump = 0,		// Sync jump width = 1 Tq
	.dataBaudPrescaler = 1,
	.dataTSEG1 = 5,
	.dataTSEG2 = 2,
	.dataSyncJump = 0,
	.enableEdgeFilter = true,
	.enableSID11 = false,
	.txDelayCompensation = CAN_Config_t::TdcAuto,
	.txDelayCompOffset = 0,
	.enableIntInvalidMessage = false,
	.enableIntWakeup = false,
	.enableIntBusError = false,
	.enableIntSysError = false,
	.enableIntRxOverflow = false,
	.enableIntTxAttempt = false,
	.enableIntCrcError = false,
	.enableIntEccError = false,
	.enableIntTxEvent = false,
	.enableIntModeChange = false,
	.enableIntTimebaseCount = false,
	.enableIntRxFifo = true,
	.enableIntTxFifo = false
};


#endif /* MIDI_CONFIG_H_ */