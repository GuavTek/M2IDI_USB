/*
 * DMA_driver.h
 *
 * Created: 06/07/2022 00:51:27
 *  Author: GuavTek
 */ 


#ifndef DMA_DRIVER_H_
#define DMA_DRIVER_H_

#include "sam.h"

#ifdef __cplusplus
extern "C" {
#endif

struct DMA_Channel_config {
	bool int_enable_complete;
	bool int_enable_suspend;
	bool int_enable_error;
	uint8_t trigger_src;
	uint8_t trigger_action;
	uint8_t arbitration_lvl;
};

struct DMA_BTCTRL_t {
	union {
		struct {
			uint16_t valid :1;
			uint16_t evosel :2;
			uint16_t blockact :2;
			uint16_t reserved :3;
			uint16_t beatsize :2;
			uint16_t srcinc :1;
			uint16_t dstinc :1;
			uint16_t stepsel :1;
			uint16_t stepsize :3;
		};
		uint16_t word;
	};
};

typedef struct DMA_Descriptor_t {
	struct DMA_BTCTRL_t btctrl;
	uint16_t beatcount;
	uint32_t* src;
	uint32_t* dst;
	DMA_Descriptor_t* next_descriptor;
};

#ifdef __cplusplus
}
#endif

#define DMA_CHANNELS 2
extern DMA_Descriptor_t base_descriptor[DMA_CHANNELS] __attribute__ ((aligned(16)));
extern DMA_Descriptor_t wrback_descriptor[DMA_CHANNELS] __attribute__ ((aligned(16)));
extern DMA_Descriptor_t transact_descriptor[DMA_CHANNELS] __attribute__ ((aligned(16)));

void dma_init(DMA_Descriptor_t* base_address, DMA_Descriptor_t* wrb_address);

void dma_attach(uint8_t channel, const DMA_Channel_config config);
void dma_detach(uint8_t channel);
//void dma_add_transaction(uint8_t channel);

inline void dma_resume(uint8_t channel){
	DMAC->CHID.reg = channel;
	
	if (DMAC->CHINTFLAG.bit.SUSP){
	
	DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_SUSP | DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR;
	DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
	DMAC->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
	}
}

inline void dma_suspend(uint8_t channel){
	DMAC->CHID.reg = channel;
	
	DMAC->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
}

inline void dma_set_descriptor(DMA_Descriptor_t* descriptor, uint16_t beatcount, uint8_t beatsize){
	descriptor->beatcount = beatcount;
	descriptor->btctrl.beatsize = beatsize;
	descriptor->btctrl.valid = 1;
}

inline void dma_set_descriptor(DMA_Descriptor_t* descriptor, uint16_t beatcount, uint32_t* const src, uint32_t* const dst, DMA_Descriptor_t* next, DMA_BTCTRL_t ctrl){
	uint32_t endPoint = beatcount * (ctrl.beatsize + 1);// * (1 << ctrl.stepsize);
	uint32_t* tempSrc;
	uint32_t* tempDst;
	if (ctrl.srcinc){
		if (ctrl.stepsel == DMAC_BTCTRL_STEPSEL_SRC_Val){
			tempSrc = (uint32_t*) (((uint32_t) src) + endPoint * (1 << ctrl.stepsize));
		} else {
			tempSrc = (uint32_t*) (((uint32_t) src) + endPoint);
		}
	} else {
		tempSrc = src;
	}
	
	if (ctrl.dstinc){
		if (ctrl.stepsel == DMAC_BTCTRL_STEPSEL_DST_Val){
			tempDst = (uint32_t*) (((uint32_t) dst) + endPoint * (1 << ctrl.stepsize));
		} else {
			tempDst = (uint32_t*) (((uint32_t) dst) + endPoint);
		}
	} else {
		tempDst = dst;
	}
	
	descriptor->beatcount = beatcount;
	descriptor->src = tempSrc;
	descriptor->dst = tempDst;
	descriptor->next_descriptor = next;
	descriptor->btctrl = ctrl;
}


#endif /* DMA_DRIVER_H_ */