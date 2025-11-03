/*
* G233 Board SPI Controller
*
* Copyright (c) 2025 G233 Project
*
* This provides an emulation of the G233 Board SPI Controller
*/

#ifndef HW_G233_SPI_H
#define HW_G233_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/fifo8.h"
#include "qom/object.h"
 
#define TYPE_G233_SPI "g233-spi"
OBJECT_DECLARE_SIMPLE_TYPE(G233SPIState, G233_SPI)

#define FIFO_CAPACITY 64

/* 寄存器偏移 */
enum {
    R_CR1    = 0x00 / 4,  /* Control Register 1 */
    R_CR2    = 0x04 / 4,  /* Control Register 2 */
    R_SR     = 0x08 / 4,  /* Status Register */
    R_DR     = 0x0C / 4,  /* Data Register */
    R_CSCTRL = 0x10 / 4,  /* Chip Select Control */
    R_MAX    = 0x14 / 4
};

/* CR1 位定义 */
#define CR1_SPE     (1 << 6)  /* SPI Enable */
#define CR1_MSTR    (1 << 2)  /* Master Selection */

/* CR2 位定义 */
#define CR2_TXEIE   (1 << 7)  /* TX Empty Interrupt Enable */
#define CR2_RXNEIE  (1 << 6)  /* RX Not Empty Interrupt Enable */
#define CR2_ERRIE   (1 << 5)  /* Error Interrupt Enable */
#define CR2_SSOE    (1 << 4)  /* 软件从设备选择输出使能 1: SS 输出使能 */
// #define CR2_SSOE    (1 << 2)  /* SS Output Enable */

/* SR 位定义 */
#define SR_BSY      (1 << 7)  /* Busy Flag */
#define SR_OVERRUN  (1 << 3)  /* Overrun Flag */
#define SR_UNDERRUN (1 << 2)  /* Underrun Flag */
#define SR_TXE      (1 << 1)  /* Transmit Buffer Empty */
#define SR_RXNE     (1 << 0)  /* Receive Buffer Not Empty */

/* CS 控制寄存器位定义 */
#define SPI_CS_ENABLE   (1 << 0)   /* Enable CS0 */
#define SPI_CS_ACTIVE   (1 << 4)   /* Activate CS0 */

struct G233SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    /* SPI 总线连接 */
    SSIBus *spi;

    uint32_t regs[R_MAX];     /* 寄存器数组 */
    Fifo8 tx_fifo;            /* 发送 FIFO - 注意是 Fifo8 类型 */
    Fifo8 rx_fifo;            /* 接收 FIFO - 注意是 Fifo8 类型 */
    
    qemu_irq *cs_lines;       /* 片选信号数组 */
    uint32_t num_cs;          /* 片选数量 */

    bool rx_data_available;  /* 用于跟踪是否有未读取的数据 */

    // /* SPI 寄存器（删除这些重复的字段，使用 regs[] 数组即可） */
    // uint32_t cr1;
    // uint32_t cr2;
    // uint32_t sr;
    // uint32_t dr;
    // uint32_t csctrl;

    // /* 内部状态（删除这些错误的字段定义） */
    // uint8_t tx_fifo;
    // uint8_t rx_fifo;
    // bool rx_fifo_valid;
};

#endif /* HW_G233_SPI_H */
