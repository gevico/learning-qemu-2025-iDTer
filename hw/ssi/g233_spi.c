/*
* G233 Board SPI Controller
*
* Copyright (c) 2025 G233 Project
*/
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/fifo8.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ssi/g233_spi.h"
#include "migration/vmstate.h"
#include "trace.h"


static void g233_spi_txfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->tx_fifo);
    s->regs[R_SR] |= SR_TXE;
}

static void g233_spi_rxfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->rx_fifo);
    s->regs[R_SR] &= ~SR_RXNE;
    s->rx_data_available = false;
}

static void g233_spi_update_cs(G233SPIState *s)
{
    uint32_t csctrl = s->regs[R_CSCTRL];
    int i;
    
    for (i = 0; i < s->num_cs; i++) {
        bool cs_en = !!(csctrl & (1 << i));          /* CS_EN bit */
        bool cs_act = !!(csctrl & (1 << (i + 4)));   /* CS_ACT bit */
        
        if (cs_en) {
            /* SSI 的 CS 是低电平有效，所以需要取反 */
            qemu_set_irq(s->cs_lines[i], !cs_act);
        } else {
            /* CS 未使能，保持非激活（高电平） */
            qemu_set_irq(s->cs_lines[i], 1);
        }
    }
}

/* 检查并触发中断 */
static void g233_spi_update_irq(G233SPIState *s)
{
    bool irq_level = false;

    /* TXE 中断 */
    if ((s->regs[R_CR2] & CR2_TXEIE) && (s->regs[R_SR] & SR_TXE)) {
        irq_level = true;
    }

    /* RXNE 中断 */
    if ((s->regs[R_CR2] & CR2_RXNEIE) && (s->regs[R_SR] & SR_RXNE)) {
        irq_level = true;
    }

    /* 错误中断 */
    if ((s->regs[R_CR2] & CR2_ERRIE) && 
        (s->regs[R_SR] & (SR_OVERRUN | SR_UNDERRUN))) {
        irq_level = true;

        // qemu_log_mask(LOG_GUEST_ERROR,
        //     "g233_spi: Error interrupt triggered - OVR=%d UDR=%d\n",
        //     !!(s->regs[R_SR] & SR_OVERRUN),
        //     !!(s->regs[R_SR] & SR_UNDERRUN));
    }

    qemu_set_irq(s->irq, irq_level);
}

static void g233_spi_flush_txfifo(G233SPIState *s)
{
    uint8_t tx;
    uint8_t rx;

    /* 只有在 SPI 使能且为主模式时才执行传输 */
    if (!(s->regs[R_CR1] & CR1_SPE)) {
        return;
    }

    if (!(s->regs[R_CR1] & CR1_MSTR)) {
        // qemu_log_mask(LOG_UNIMP, 
        //               "%s: slave mode not implemented\n", __func__);
        return;
    }

    while (!fifo8_is_empty(&s->tx_fifo)) {
        /* 设置忙标志 */
        s->regs[R_SR] |= SR_BSY;

        tx = fifo8_pop(&s->tx_fifo);
        // qemu_log_mask(LOG_TRACE, "g233_spi: SSI transfer TX=0x%02x\n", tx);

        rx = ssi_transfer(s->spi, tx);
        // qemu_log_mask(LOG_TRACE, "g233_spi: SSI transfer RX=0x%02x\n", rx);

        if (!fifo8_is_full(&s->rx_fifo)) {
            fifo8_push(&s->rx_fifo, rx);
            s->regs[R_SR] |= SR_RXNE;
            s->rx_data_available = true;
        }

        /* 接收数据处理 */
        // if (!fifo8_is_full(&s->rx_fifo)) {
        //     fifo8_push(&s->rx_fifo, rx);
        //     s->regs[R_SR] |= SR_RXNE;
        // } else {
        //     /* 接收 FIFO 已满，产生溢出错误 */
        //     s->regs[R_SR] |= SR_OVERRUN;
        // }

        /* 清除忙标志 */
        s->regs[R_SR] &= ~SR_BSY;
    }
}

static void g233_spi_reset(DeviceState *d)
{
    G233SPIState *s = G233_SPI(d);

    memset(s->regs, 0, sizeof(s->regs));

    /* 设置复位默认值 */
    s->regs[R_CR1] = 0x00000000;
    s->regs[R_CR2] = 0x00000000;
    s->regs[R_SR] = 0x00000002;   /* TXE = 1 */
    s->regs[R_DR] = 0x0000000C;
    s->regs[R_CSCTRL] = 0x00000000;

    g233_spi_txfifo_reset(s);
    g233_spi_rxfifo_reset(s);
    g233_spi_update_cs(s);
    g233_spi_update_irq(s);

    qemu_log_mask(LOG_TRACE, "g233_spi: device reset\n");
}

static bool g233_spi_is_bad_reg(hwaddr addr)
{
    /* 检查地址是否超出范围 */
    // 使用 R_MAX 而不是 G233_SPI_REG_NUM
    if (addr >= (R_MAX << 2)) {
        qemu_log_mask(LOG_GUEST_ERROR,
            "g233_spi: Invalid register offset 0x%"HWADDR_PRIx"\n",
            addr);
        return true;
    }

    /* 所有定义的寄存器都可以访问 */
    return false;
}

static uint64_t g233_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    G233SPIState *s = G233_SPI(opaque);
    uint32_t r = 0;

    if (g233_spi_is_bad_reg(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read at address 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }

    addr >>= 2;

    switch (addr) {
    case R_DR:
        /* 读取接收数据 */
        if (!fifo8_is_empty(&s->rx_fifo)) {
            r = fifo8_pop(&s->rx_fifo);
            if (fifo8_is_empty(&s->rx_fifo)) {
                s->regs[R_SR] &= ~SR_RXNE;
                s->rx_data_available = false;
            }
            // qemu_log_mask(LOG_TRACE, "g233_spi: read DR = 0x%02x\n", r);
        } else {
            /* 读取空 FIFO，产生下溢错误 */
            s->regs[R_SR] |= SR_UNDERRUN;
            r = 0xFF;  /* 返回默认值 */
            qemu_log_mask(LOG_GUEST_ERROR, "g233_spi: UNDERRUN on DR read\n");
        }
        break;

    case R_SR:
        /* 状态寄存器只读 */
        r = s->regs[R_SR];
        break;

    default:
        r = s->regs[addr];
        break;
    }

    g233_spi_update_irq(s);
    return r;
}

static void g233_spi_write(void *opaque, hwaddr addr,
                           uint64_t val64, unsigned int size)
{
    G233SPIState *s = G233_SPI(opaque);
    uint32_t value = val64;

    if (g233_spi_is_bad_reg(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write at address 0x%" HWADDR_PRIx
                      " value=0x%x\n", __func__, addr, value);
        return;
    }

    addr >>= 2;

    switch (addr) {
    case R_CR1:
        /* 只保留 SPE 和 MSTR 位 */
        s->regs[R_CR1] = value & (CR1_SPE | CR1_MSTR);
        // qemu_log_mask(LOG_TRACE, "g233_spi: CR1 = 0x%08x (SPE=%d, MSTR=%d)\n",
        //              value, !!(value & CR1_SPE), !!(value & CR1_MSTR));
        /* TODO: 处理 SPE 和 MSTR 位变化 */
        break;

    case R_CR2:
        /* 保留中断使能位和 SSOE 位 */
        s->regs[R_CR2] = value & (CR2_TXEIE | CR2_RXNEIE | CR2_ERRIE | CR2_SSOE);
        // qemu_log_mask(LOG_TRACE, "g233_spi: CR2 = 0x%08x (ERRIE=%d, RXNEIE=%d, TXEIE=%d)\n",
        //              value, !!(value & CR2_ERRIE), 
        //              !!(value & CR2_RXNEIE), !!(value & CR2_TXEIE));
        /* TODO: 处理中断使能位变化 */
        g233_spi_update_irq(s);
        break;

    case R_SR:
        /* 写 1 清除错误标志 */
        if (value & SR_OVERRUN) {
            s->regs[R_SR] &= ~SR_OVERRUN;
            qemu_log_mask(LOG_TRACE, "g233_spi: OVERRUN flag cleared\n");
        }
        if (value & SR_UNDERRUN) {
            s->regs[R_SR] &= ~SR_UNDERRUN;
            qemu_log_mask(LOG_TRACE, "g233_spi: UNDERRUN flag cleared\n");
        }
        g233_spi_update_irq(s);
        break;

    case R_DR:
        // qemu_log_mask(LOG_TRACE, "g233_spi: write DR = 0x%02x (RXNE=%d, rx_data_available=%d)\n",
        //              (uint8_t)value, !!(s->regs[R_SR] & SR_RXNE), s->rx_data_available);
        
        /* 检查是否有未读取的数据 */
        if (s->rx_data_available || (s->regs[R_SR] & SR_RXNE)) {
            /* 溢出条件：RXNE=1 时再次写入 DR */
            s->regs[R_SR] |= SR_OVERRUN;
            qemu_log_mask(LOG_GUEST_ERROR,
                         "g233_spi: OVERRUN detected! RXNE was set when writing DR\n");
            
            /* 触发中断 */
            g233_spi_update_irq(s);
            
            /* 溢出时新数据会丢失，不执行传输 */
            return;
        }
    
        /* 正常传输，写入发送数据 */
        if (!(s->regs[R_SR] & SR_BSY)) {
            if (!fifo8_is_full(&s->tx_fifo)) {
                fifo8_push(&s->tx_fifo, (uint8_t)value);
                s->regs[R_SR] &= ~SR_TXE;
                
                /* 执行传输 */
                g233_spi_flush_txfifo(s);
                
                /* 传输完成后，发送缓冲区变为空 */
                if (fifo8_is_empty(&s->tx_fifo)) {
                    s->regs[R_SR] |= SR_TXE;
                }
                
                g233_spi_update_irq(s);
            }
        }
        break;

    case R_CSCTRL:
        s->regs[R_CSCTRL] = value & 0xFF;
        // qemu_log_mask(LOG_TRACE, "g233_spi: CSCTRL = 0x%02x\n", value & 0xFF);
        g233_spi_update_cs(s);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid write to register 0x%"
                      HWADDR_PRIx " with 0x%x\n", __func__, addr << 2, value);
        break;
    }
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233SPIState *s = G233_SPI(dev);

    /* 创建 SPI 总线 */
    s->spi = ssi_create_bus(dev, "spi");

    /* 初始化中断 */
    sysbus_init_irq(sbd, &s->irq);

    /* 初始化 4 个 CS 引脚（对应寄存器文档中的 CS0-CS3）
    * 使用 qdev_init_gpio_out_named 初始化 CS 引脚
    */
    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    qdev_init_gpio_out_named(dev, s->cs_lines, SSI_GPIO_CS, s->num_cs);
    
    /* 创建内存区域 */
    memory_region_init_io(&s->mmio, OBJECT(s), &g233_spi_ops, s,
                          TYPE_G233_SPI, 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);

    /* 初始化 FIFO */
    fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    fifo8_create(&s->rx_fifo, FIFO_CAPACITY);

    /* 初始化寄存器复位值 */
    s->regs[R_SR] = SR_TXE;  /* 发送缓冲区初始为空 */
    s->rx_data_available = false;

    // qemu_log_mask(LOG_TRACE, "g233_spi: device realized\n");
}

static Property g233_spi_properties[] = {
    DEFINE_PROP_UINT32("num-cs", G233SPIState, num_cs, 4)
    // DEFINE_PROP_END_OF_LIST()  /* 标准终止符（新版本QEMU已移除） */
};

static const VMStateDescription vmstate_g233_spi = {
    .name = TYPE_G233_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, G233SPIState, R_MAX),
        VMSTATE_FIFO8(tx_fifo, G233SPIState),
        VMSTATE_FIFO8(rx_fifo, G233SPIState),
        VMSTATE_BOOL(rx_data_available, G233SPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void g233_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    // device_class_set_props(dc, g233_spi_properties);
    dc->realize = g233_spi_realize;
    device_class_set_legacy_reset(dc, g233_spi_reset);
    dc->vmsd = &vmstate_g233_spi;

    device_class_set_props_n(dc, g233_spi_properties, 1);  /* 明确告诉有 1 个属性 */
    // device_class_set_props(dc, g233_spi_properties);  /* 自动检测数量 */

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo g233_spi_info = {
    .name           = TYPE_G233_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233SPIState),
    .class_init     = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}

type_init(g233_spi_register_types)
