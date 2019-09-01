// #include <memorymap.h> // should it?

#define DMAMUX DMAMUX_BASE

// note dma channels are 1-indexed and cooresponding dmamux
// channels are 0-indexed
#define DMAMUX_CxCR(base, channel) MMIO32((base) + 0x04 * (channel))
#define DMAMUX_C0CR(base) DMAMUX_CxCR(base, DMA_CHANNEL1 - 1)
#define DMAMUX_C1CR(base) DMAMUX_CxCR(base, DMA_CHANNEL2 - 1)
#define DMAMUX_C2CR(base) DMAMUX_CxCR(base, DMA_CHANNEL3 - 1)
#define DMAMUX_C3CR(base) DMAMUX_CxCR(base, DMA_CHANNEL4 - 1)
#define DMAMUX_C4CR(base) DMAMUX_CxCR(base, DMA_CHANNEL5 - 1)
#define DMAMUX_C5CR(base) DMAMUX_CxCR(base, DMA_CHANNEL6 - 1)
#define DMAMUX_C6CR(base) DMAMUX_CxCR(base, DMA_CHANNEL7 - 1)

#define DMAMUX_CSR(base) MMIO32((dma_base) + 0x80))
#define DMAMUX_CFR(base) MMIO32((dma_base) + 0x84))
#define DMAMUX_RG0CR(base) MMIO32((dma_base) + 0x100))
#define DMAMUX_RG1CR(base) MMIO32((dma_base) + 0x104))
#define DMAMUX_RG2CR(base) MMIO32((dma_base) + 0x108))
#define DMAMUX_RG3CR(base) MMIO32((dma_base) + 0x10c))
#define DMAMUX_RGSR(base) MMIO32((dma_base) + 0x140))
#define DMAMUX_RGCFR(base) MMIO32((dma_base) + 0x144))

#define DMAMUX_REQID_TIM2_CH1 26
#define DMAMUX_REQID_TIM2_UP 31
