/*
 * DMA_driver.cpp
 *
 * Created: 06/07/2022 00:51:10
 *  Author: GuavTek
 */

#include "DMA_driver.h" 

DMA_Descriptor_t base_descriptor[DMA_CHANNELS] __attribute__ ((aligned(16)));
DMA_Descriptor_t wrback_descriptor[DMA_CHANNELS] __attribute__ ((aligned(16)));
DMA_Descriptor_t transact_descriptor[DMA_CHANNELS] __attribute__ ((aligned(16)));

void dma_init(DMA_Descriptor_t* base_address, DMA_Descriptor_t* wrb_address){
	// Enable clocks
	PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
	PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
	
	// Disable DMA
	DMAC->CTRL.bit.DMAENABLE = 0;
	DMAC->CTRL.bit.CRCENABLE = 0;
	
	// Enable arbitration levels
	DMAC->CTRL.reg =	DMAC_CTRL_LVLEN0 |
						DMAC_CTRL_LVLEN1 |
						DMAC_CTRL_LVLEN2;
	
	DMAC->PRICTRL0.bit.RRLVLEN2 = 1;
	DMAC->QOSCTRL.reg = DMAC_QOSCTRL_DQOS_HIGH | DMAC_QOSCTRL_FQOS_HIGH;
	DMAC->BASEADDR.reg = (uint32_t) base_address;
	DMAC->WRBADDR.reg = (uint32_t) wrb_address;
	
	//NVIC_EnableIRQ(DMAC_IRQn);
	
	// Enable DMA
	DMAC->CTRL.bit.DMAENABLE = 1;
}

void dma_attach(uint8_t channel, const DMA_Channel_config config){
	if (DMAC->BUSYCH.reg & (1 << channel)){
		// Channel is already started
		return;
	}
	// write one to valid bit
	DMAC->CHID.reg = channel;
	
	DMAC->CHCTRLB.reg = (config.trigger_action << DMAC_CHCTRLB_TRIGACT_Pos) |
		(config.trigger_src << DMAC_CHCTRLB_TRIGSRC_Pos) |
		(config.arbitration_lvl << DMAC_CHCTRLB_LVL_Pos);
	
	DMAC->CHINTENSET.reg =	(config.int_enable_suspend << DMAC_CHINTENSET_SUSP_Pos) |
		(config.int_enable_complete << DMAC_CHINTENSET_TCMPL_Pos) |
		(config.int_enable_error << DMAC_CHINTENSET_TERR_Pos);
	
	DMAC->CHINTFLAG.bit.TERR = 1;
	
	DMAC->CHCTRLA.bit.ENABLE = 1;
	DMAC->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
}

void dma_detach(uint8_t channel){
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.bit.ENABLE = 0;
}

void DMAC_Handler(){
	uint32_t tempFlag = DMAC->INTPEND.bit.ID;
	DMAC->CHID.reg = tempFlag;
//  	if (!wrback_descriptor[tempFlag].btctrl.valid){
//  		DMAC->CHINTENCLR.bit.SUSP = 1;
//  		return;
//  	}
	
	PORT->Group[0].OUTTGL.reg = 1 << 17;
	
	dma_resume(tempFlag);
	DMAC->CHINTENCLR.bit.SUSP = 1;
}

