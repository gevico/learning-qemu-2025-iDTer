/*
 * RISC-V Emulation Helpers for QEMU.
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017-2018 SiFive, Inc.
 * Copyright (c) 2022      VRULL GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "internals.h"
#include "exec/cputlb.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/probe.h"
#include "exec/helper-proto.h"
#include "exec/tlb-flags.h"
#include "trace.h"
#include "qemu/log.h"
#include "qemu/log-for-trace.h"

/* Exceptions processing helpers */
G_NORETURN void riscv_raise_exception(CPURISCVState *env,
                                      RISCVException exception,
                                      uintptr_t pc)
{
    CPUState *cs = env_cpu(env);

    trace_riscv_exception(exception,
                          riscv_cpu_get_trap_name(exception, false),
                          env->pc);

    cs->exception_index = exception;
    cpu_loop_exit_restore(cs, pc);
}

void helper_raise_exception(CPURISCVState *env, uint32_t exception)
{
    riscv_raise_exception(env, exception, 0);
}

target_ulong helper_csrr(CPURISCVState *env, int csr)
{
    /*
     * The seed CSR must be accessed with a read-write instruction. A
     * read-only instruction such as CSRRS/CSRRC with rs1=x0 or CSRRSI/
     * CSRRCI with uimm=0 will raise an illegal instruction exception.
     */
    if (csr == CSR_SEED) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    target_ulong val = 0;
    RISCVException ret = riscv_csrr(env, csr, &val);

    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, GETPC());
    }
    return val;
}

void helper_csrw(CPURISCVState *env, int csr, target_ulong src)
{
    target_ulong mask = env->xl == MXL_RV32 ? UINT32_MAX : (target_ulong)-1;
    RISCVException ret = riscv_csrrw(env, csr, NULL, src, mask, GETPC());

    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, GETPC());
    }
}

target_ulong helper_csrrw(CPURISCVState *env, int csr,
                          target_ulong src, target_ulong write_mask)
{
    target_ulong val = 0;
    RISCVException ret = riscv_csrrw(env, csr, &val, src, write_mask, GETPC());

    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, GETPC());
    }
    return val;
}

target_ulong helper_csrr_i128(CPURISCVState *env, int csr)
{
    Int128 rv = int128_zero();
    RISCVException ret = riscv_csrr_i128(env, csr, &rv);

    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, GETPC());
    }

    env->retxh = int128_gethi(rv);
    return int128_getlo(rv);
}

void helper_csrw_i128(CPURISCVState *env, int csr,
                      target_ulong srcl, target_ulong srch)
{
    RISCVException ret = riscv_csrrw_i128(env, csr, NULL,
                                          int128_make128(srcl, srch),
                                          UINT128_MAX, GETPC());

    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, GETPC());
    }
}

target_ulong helper_csrrw_i128(CPURISCVState *env, int csr,
                               target_ulong srcl, target_ulong srch,
                               target_ulong maskl, target_ulong maskh)
{
    Int128 rv = int128_zero();
    RISCVException ret = riscv_csrrw_i128(env, csr, &rv,
                                          int128_make128(srcl, srch),
                                          int128_make128(maskl, maskh),
                                          GETPC());

    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, GETPC());
    }

    env->retxh = int128_gethi(rv);
    return int128_getlo(rv);
}


/*
 * G233 DMA 矩阵转置指令 Helper
 * 
 * 参数：
 *   - dst: 目标内存地址
 *   - src: 源内存地址
 *   - grain: 粒度参数 (0=8x8, 1=16x16, 2=32x32)
 */
void helper_dma(CPURISCVState *env, target_ulong dst, 
                        target_ulong src, target_ulong grain)
{
    int matrix_size;
    int total_elements;
    uint32_t *src_matrix;
    uint32_t *dst_matrix;
    // CPUState *cs = env_cpu(env);

    /* 根据粒度计算矩阵大小：N = 8 * 2^grain */
    if (grain > 2) {
        /* 非法粒度值，触发异常 */
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
        return;
    }

    matrix_size = 8 << grain;  /* 8, 16, 32 */
    total_elements = matrix_size * matrix_size;

    /* 分配临时缓冲区 */
    src_matrix = g_malloc(total_elements * sizeof(uint32_t));
    dst_matrix = g_malloc(total_elements * sizeof(uint32_t));

    /* 从源地址读取整个矩阵（按行优先顺序） */
    for (int i = 0; i < total_elements; i++) {
        src_matrix[i] = cpu_ldl_data_ra(env, src + i * 4, GETPC());
    }

    /* 执行矩阵转置：dst[i][j] = src[j][i] */
    for (int i = 0; i < matrix_size; i++) {
        for (int j = 0; j < matrix_size; j++) {
            dst_matrix[i * matrix_size + j] = src_matrix[j * matrix_size + i];
        }
    }

    /* 将转置后的矩阵写入目标地址 */
    for (int i = 0; i < total_elements; i++) {
        cpu_stl_data_ra(env, dst + i * 4, dst_matrix[i], GETPC());
    }

    /* 清理临时缓冲区 */
    g_free(src_matrix);
    g_free(dst_matrix);

    /* 添加调试日志 */
    // qemu_log_mask(CPU_LOG_EXEC, "DMA Transpose: matrix_size=%d, src=0x%lx, dst=0x%lx\n",
    //             matrix_size, (unsigned long)src, (unsigned long)dst);
}

/*
 * Sort 排序指令
 * 
 * 指令格式：sort rd, rs1, rs2
 *   - rd: 排序长度（要排序的元素数量）
 *   - rs1: 数组地址
 *   - rs2: 数组大小（用于边界检查）
 */
void helper_sort(CPURISCVState *env, target_ulong addr,
                target_ulong sort_num, target_ulong array_size)
{
    uint32_t *array;
    target_ulong i, j;
    uint32_t temp;
    
    /* 参数验证 */
    if (sort_num == 0) {
        /* 排序 0 个元素，直接返回 */
        return;
    }

    if (sort_num > array_size) {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "SORT: sort_num (%lu) > array_size (%lu)\n", 
                      sort_num, array_size);
        /* 将排序数量限制为数组大小 */
        sort_num = array_size;
    }
    
    if (sort_num > 1024) {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "SORT: sort_num too large (%lu), capping at 1024\n", 
                      sort_num);
        sort_num = 1024;
    }

    /* 分配临时缓冲区存储数组 */
    array = g_malloc(sort_num * sizeof(uint32_t));
    
    /* 从内存读取数组数据 */
    for (i = 0; i < sort_num; i++) {
        array[i] = cpu_ldq_data_ra(env, addr + i * sizeof(uint32_t), GETPC());
    }

    /* 
     * 冒泡排序算法（升序）
     */
    for (i = 0; i < sort_num - 1; i++) {
        for (j = 0; j < sort_num - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                /* 交换元素 */
                temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
    
    /* 将排序后的数组写回内存 */
    for (i = 0; i < sort_num; i++) {
        cpu_stq_data_ra(env, addr + i * sizeof(uint32_t), array[i], GETPC());
    }
    
    /* 清理临时缓冲区 */
    g_free(array);
    
    /* 添加调试日志 */
    qemu_log_mask(CPU_LOG_EXEC, 
        "SORT: addr=0x%lx, sorted %lu/%lu elements\n",
        (unsigned long)addr, sort_num, array_size);
}

/* 
 * G233 Crush 压缩指令 Helper
 * 
 * 功能：将 8 位数组元素的低 4 位两两打包成新的 8 位数组
 * 
 * 参数：
 *   - env: CPU 环境
 *   - dst_addr: 目标数组地址（压缩后的数据存储位置）
 *   - src_addr: 源数组地址（待压缩的数据）
 *   - src_num: 源数组元素数量
 * 
 * 压缩规则：
 *   dst[i] = (src[2*i] & 0xF) | ((src[2*i+1] & 0xF) << 4)
 * 
 * 示例：
 *   输入：{0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0x1, 0x2, 0x3, 0x4}
 *   输出：{0xBA, 0xDC, 0xFE, 0x21, 0x43}
 *   
 *   解析：
 *     dst[0] = (0xA & 0xF) | ((0xB & 0xF) << 4) = 0x0A | 0xB0 = 0xBA
 *     dst[1] = (0xC & 0xF) | ((0xD & 0xF) << 4) = 0x0C | 0xD0 = 0xDC
 *     dst[2] = (0xE & 0xF) | ((0xF & 0xF) << 4) = 0x0E | 0xF0 = 0xFE
 *     dst[3] = (0x1 & 0xF) | ((0x2 & 0xF) << 4) = 0x01 | 0x20 = 0x21
 *     dst[4] = (0x3 & 0xF) | ((0x4 & 0xF) << 4) = 0x03 | 0x40 = 0x43
 */
void helper_crush(CPURISCVState *env, target_ulong dst_addr, 
                target_ulong src_addr, target_ulong src_num)
{
    target_ulong i;
    uint8_t low_nibble, high_nibble, packed_byte;
    target_ulong dst_num;

    /* 参数验证 */
    if (src_num == 0) {
        /* 没有数据需要压缩 */
        return;
    }
    
    if (src_num > 2048) {
        qemu_log_mask(LOG_GUEST_ERROR, 
                      "CRUSH: src_num too large (%lu), capping at 2048\n", 
                      src_num);
        src_num = 2048;
    }

    /* 计算目标数组大小（向上取整） */
    dst_num = (src_num + 1) / 2;
    
    /* 
     * 压缩算法：
     * 遍历源数组，每次处理两个元素，打包成一个目标元素
     */
    for (i = 0; i < dst_num; i++) {
        /* 读取第一个元素的低 4 位 */
        low_nibble = cpu_ldub_data_ra(env, src_addr + 2 * i, GETPC()) & 0x0F;
        
        /* 读取第二个元素的低 4 位（如果存在） */
        if (2 * i + 1 < src_num) {
            high_nibble = cpu_ldub_data_ra(env, src_addr + 2 * i + 1, GETPC()) & 0x0F;
        } else {
            /* 奇数个元素时，最后一个元素的高 4 位填充 0 */
            high_nibble = 0;
        }
        
        /* 打包：低 4 位放在低位，高 4 位放在高位 */
        packed_byte = low_nibble | (high_nibble << 4);
        
        /* 写入目标数组 */
        cpu_stb_data_ra(env, dst_addr + i, packed_byte, GETPC());
    }

    /* 添加调试日志 */
    // qemu_log_mask(CPU_LOG_EXEC, 
    //     "CRUSH: src=0x%lx, dst=0x%lx, src_num=%lu, dst_num=%lu\n",
    //     (unsigned long)src_addr, (unsigned long)dst_addr, 
    //     src_num, dst_num);

    // if (qemu_loglevel_mask(CPU_LOG_EXEC) && dst_num > 0) {
    //     qemu_log("CRUSH: First few packed bytes: ");
    //     for (i = 0; i < (dst_num < 5 ? dst_num : 5); i++) {
    //         uint8_t byte = cpu_ldub_data_ra(env, dst_addr + i, GETPC());
    //         qemu_log("0x%02X ", byte);
    //     }
    //     qemu_log("\n");
    // }
}

/*
* Expand 扩展指令实现
*/
void helper_expand(CPURISCVState *env,
      target_ulong dst,
      target_ulong src,
      target_ulong num)
{
    /* TODO: 实现扩展逻辑 */
    qemu_log("Expand instruction not yet implemented\n");
}



/*
 * check_zicbo_envcfg
 *
 * Raise virtual exceptions and illegal instruction exceptions for
 * Zicbo[mz] instructions based on the settings of [mhs]envcfg as
 * specified in section 2.5.1 of the CMO specification.
 */
static void check_zicbo_envcfg(CPURISCVState *env, target_ulong envbits,
                                uintptr_t ra)
{
#ifndef CONFIG_USER_ONLY
    if ((env->priv < PRV_M) && !get_field(env->menvcfg, envbits)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, ra);
    }

    if (env->virt_enabled &&
        (((env->priv <= PRV_S) && !get_field(env->henvcfg, envbits)) ||
         ((env->priv < PRV_S) && !get_field(env->senvcfg, envbits)))) {
        riscv_raise_exception(env, RISCV_EXCP_VIRT_INSTRUCTION_FAULT, ra);
    }

    if ((env->priv < PRV_S) && !get_field(env->senvcfg, envbits)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, ra);
    }
#endif
}

void helper_cbo_zero(CPURISCVState *env, target_ulong address)
{
    RISCVCPU *cpu = env_archcpu(env);
    uint16_t cbozlen = cpu->cfg.cboz_blocksize;
    int mmu_idx = riscv_env_mmu_index(env, false);
    uintptr_t ra = GETPC();
    void *mem;

    check_zicbo_envcfg(env, MENVCFG_CBZE, ra);

    /* Mask off low-bits to align-down to the cache-block. */
    address &= ~(cbozlen - 1);

    /*
     * cbo.zero requires MMU_DATA_STORE access. Do a probe_write()
     * to raise any exceptions, including PMP.
     */
    mem = probe_write(env, address, cbozlen, mmu_idx, ra);

    if (likely(mem)) {
        memset(mem, 0, cbozlen);
    } else {
        /*
         * This means that we're dealing with an I/O page. Section 4.2
         * of cmobase v1.0.1 says:
         *
         * "Cache-block zero instructions store zeros independently
         * of whether data from the underlying memory locations are
         * cacheable."
         *
         * Write zeros in address + cbozlen regardless of not being
         * a RAM page.
         */
        for (int i = 0; i < cbozlen; i++) {
            cpu_stb_mmuidx_ra(env, address + i, 0, mmu_idx, ra);
        }
    }
}

/*
 * check_zicbom_access
 *
 * Check access permissions (LOAD, STORE or FETCH as specified in
 * section 2.5.2 of the CMO specification) for Zicbom, raising
 * either store page-fault (non-virtualized) or store guest-page
 * fault (virtualized).
 */
static void check_zicbom_access(CPURISCVState *env,
                                target_ulong address,
                                uintptr_t ra)
{
    RISCVCPU *cpu = env_archcpu(env);
    int mmu_idx = riscv_env_mmu_index(env, false);
    uint16_t cbomlen = cpu->cfg.cbom_blocksize;
    void *phost;
    int ret;

    /* Mask off low-bits to align-down to the cache-block. */
    address &= ~(cbomlen - 1);

    /*
     * Section 2.5.2 of cmobase v1.0.1:
     *
     * "A cache-block management instruction is permitted to
     * access the specified cache block whenever a load instruction
     * or store instruction is permitted to access the corresponding
     * physical addresses. If neither a load instruction nor store
     * instruction is permitted to access the physical addresses,
     * but an instruction fetch is permitted to access the physical
     * addresses, whether a cache-block management instruction is
     * permitted to access the cache block is UNSPECIFIED."
     */
    ret = probe_access_flags(env, address, cbomlen, MMU_DATA_LOAD,
                             mmu_idx, true, &phost, ra);
    if (ret != TLB_INVALID_MASK) {
        /* Success: readable */
        return;
    }

    /*
     * Since not readable, must be writable. On failure, store
     * fault/store guest amo fault will be raised by
     * riscv_cpu_tlb_fill(). PMP exceptions will be caught
     * there as well.
     */
    probe_write(env, address, cbomlen, mmu_idx, ra);
}

void helper_cbo_clean_flush(CPURISCVState *env, target_ulong address)
{
    uintptr_t ra = GETPC();
    check_zicbo_envcfg(env, MENVCFG_CBCFE, ra);
    check_zicbom_access(env, address, ra);

    /* We don't emulate the cache-hierarchy, so we're done. */
}

void helper_cbo_inval(CPURISCVState *env, target_ulong address)
{
    uintptr_t ra = GETPC();
    check_zicbo_envcfg(env, MENVCFG_CBIE, ra);
    check_zicbom_access(env, address, ra);

    /* We don't emulate the cache-hierarchy, so we're done. */
}

#ifndef CONFIG_USER_ONLY

target_ulong helper_sret(CPURISCVState *env)
{
    uint64_t mstatus;
    target_ulong prev_priv, prev_virt = env->virt_enabled;
    const target_ulong src_priv = env->priv;
    const bool src_virt = env->virt_enabled;

    if (!(env->priv >= PRV_S)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    target_ulong retpc = env->sepc & get_xepc_mask(env);
    if (!riscv_cpu_allow_16bit_insn(&env_archcpu(env)->cfg,
                                    env->priv_ver,
                                    env->misa_ext) && (retpc & 0x3)) {
        riscv_raise_exception(env, RISCV_EXCP_INST_ADDR_MIS, GETPC());
    }

    if (get_field(env->mstatus, MSTATUS_TSR) && !(env->priv >= PRV_M)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    if (env->virt_enabled && get_field(env->hstatus, HSTATUS_VTSR)) {
        riscv_raise_exception(env, RISCV_EXCP_VIRT_INSTRUCTION_FAULT, GETPC());
    }

    mstatus = env->mstatus;
    prev_priv = get_field(mstatus, MSTATUS_SPP);
    mstatus = set_field(mstatus, MSTATUS_SIE,
                        get_field(mstatus, MSTATUS_SPIE));
    mstatus = set_field(mstatus, MSTATUS_SPIE, 1);
    mstatus = set_field(mstatus, MSTATUS_SPP, PRV_U);

    if (riscv_cpu_cfg(env)->ext_ssdbltrp) {
        if (riscv_has_ext(env, RVH)) {
            target_ulong prev_vu = get_field(env->hstatus, HSTATUS_SPV) &&
                                   prev_priv == PRV_U;
            /* Returning to VU from HS, vsstatus.sdt = 0 */
            if (!env->virt_enabled && prev_vu) {
                env->vsstatus = set_field(env->vsstatus, MSTATUS_SDT, 0);
            }
        }
        mstatus = set_field(mstatus, MSTATUS_SDT, 0);
    }
    if (riscv_cpu_cfg(env)->ext_smdbltrp && env->priv >= PRV_M) {
        mstatus = set_field(mstatus, MSTATUS_MDT, 0);
    }
    if (env->priv_ver >= PRIV_VERSION_1_12_0) {
        mstatus = set_field(mstatus, MSTATUS_MPRV, 0);
    }
    env->mstatus = mstatus;

    if (riscv_has_ext(env, RVH) && !env->virt_enabled) {
        /* We support Hypervisor extensions and virtulisation is disabled */
        target_ulong hstatus = env->hstatus;

        prev_virt = get_field(hstatus, HSTATUS_SPV);
        hstatus = set_field(hstatus, HSTATUS_SPV, 0);

        env->hstatus = hstatus;

        if (prev_virt) {
            riscv_cpu_swap_hypervisor_regs(env);
        }
    }

    riscv_cpu_set_mode(env, prev_priv, prev_virt);

    /*
     * If forward cfi enabled for new priv, restore elp status
     * and clear spelp in mstatus
     */
    if (cpu_get_fcfien(env)) {
        env->elp = get_field(env->mstatus, MSTATUS_SPELP);
    }
    env->mstatus = set_field(env->mstatus, MSTATUS_SPELP, 0);

    if (riscv_cpu_cfg(env)->ext_smctr || riscv_cpu_cfg(env)->ext_ssctr) {
        riscv_ctr_add_entry(env, env->pc, retpc, CTRDATA_TYPE_EXCEP_INT_RET,
                            src_priv, src_virt);
    }

    return retpc;
}

static void check_ret_from_m_mode(CPURISCVState *env, target_ulong retpc,
                                  target_ulong prev_priv,
                                  uintptr_t ra)
{
    if (!(env->priv >= PRV_M)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, ra);
    }

    if (!riscv_cpu_allow_16bit_insn(&env_archcpu(env)->cfg,
                                    env->priv_ver,
                                    env->misa_ext) && (retpc & 0x3)) {
        riscv_raise_exception(env, RISCV_EXCP_INST_ADDR_MIS, ra);
    }

    if (riscv_cpu_cfg(env)->pmp &&
        !pmp_get_num_rules(env) && (prev_priv != PRV_M)) {
        riscv_raise_exception(env, RISCV_EXCP_INST_ACCESS_FAULT, ra);
    }
}
static target_ulong ssdbltrp_mxret(CPURISCVState *env, target_ulong mstatus,
                                   target_ulong prev_priv,
                                   target_ulong prev_virt)
{
    /* If returning to U, VS or VU, sstatus.sdt = 0 */
    if (prev_priv == PRV_U || (prev_virt &&
        (prev_priv == PRV_S || prev_priv == PRV_U))) {
        mstatus = set_field(mstatus, MSTATUS_SDT, 0);
        /* If returning to VU, vsstatus.sdt = 0 */
        if (prev_virt && prev_priv == PRV_U) {
            env->vsstatus = set_field(env->vsstatus, MSTATUS_SDT, 0);
        }
    }

    return mstatus;
}

target_ulong helper_mret(CPURISCVState *env)
{
    target_ulong retpc = env->mepc & get_xepc_mask(env);
    uint64_t mstatus = env->mstatus;
    target_ulong prev_priv = get_field(mstatus, MSTATUS_MPP);
    uintptr_t ra = GETPC();

    check_ret_from_m_mode(env, retpc, prev_priv, ra);

    target_ulong prev_virt = get_field(env->mstatus, MSTATUS_MPV) &&
                             (prev_priv != PRV_M);
    mstatus = set_field(mstatus, MSTATUS_MIE,
                        get_field(mstatus, MSTATUS_MPIE));
    mstatus = set_field(mstatus, MSTATUS_MPIE, 1);
    mstatus = set_field(mstatus, MSTATUS_MPP,
                        riscv_has_ext(env, RVU) ? PRV_U : PRV_M);
    mstatus = set_field(mstatus, MSTATUS_MPV, 0);
    if (riscv_cpu_cfg(env)->ext_ssdbltrp) {
        mstatus = ssdbltrp_mxret(env, mstatus, prev_priv, prev_virt);
    }
    if (riscv_cpu_cfg(env)->ext_smdbltrp) {
        mstatus = set_field(mstatus, MSTATUS_MDT, 0);
    }
    if ((env->priv_ver >= PRIV_VERSION_1_12_0) && (prev_priv != PRV_M)) {
        mstatus = set_field(mstatus, MSTATUS_MPRV, 0);
    }
    env->mstatus = mstatus;

    if (riscv_has_ext(env, RVH) && prev_virt) {
        riscv_cpu_swap_hypervisor_regs(env);
    }

    riscv_cpu_set_mode(env, prev_priv, prev_virt);
    /*
     * If forward cfi enabled for new priv, restore elp status
     * and clear mpelp in mstatus
     */
    if (cpu_get_fcfien(env)) {
        env->elp = get_field(env->mstatus, MSTATUS_MPELP);
    }
    env->mstatus = set_field(env->mstatus, MSTATUS_MPELP, 0);

    if (riscv_cpu_cfg(env)->ext_smctr || riscv_cpu_cfg(env)->ext_ssctr) {
        riscv_ctr_add_entry(env, env->pc, retpc, CTRDATA_TYPE_EXCEP_INT_RET,
                            PRV_M, false);
    }

    return retpc;
}

target_ulong helper_mnret(CPURISCVState *env)
{
    target_ulong retpc = env->mnepc;
    target_ulong prev_priv = get_field(env->mnstatus, MNSTATUS_MNPP);
    target_ulong prev_virt;
    uintptr_t ra = GETPC();

    check_ret_from_m_mode(env, retpc, prev_priv, ra);

    prev_virt = get_field(env->mnstatus, MNSTATUS_MNPV) &&
                (prev_priv != PRV_M);
    env->mnstatus = set_field(env->mnstatus, MNSTATUS_NMIE, true);

    /*
     * If MNRET changes the privilege mode to a mode
     * less privileged than M, it also sets mstatus.MPRV to 0.
     */
    if (prev_priv < PRV_M) {
        env->mstatus = set_field(env->mstatus, MSTATUS_MPRV, false);
    }
    if (riscv_cpu_cfg(env)->ext_ssdbltrp) {
        env->mstatus = ssdbltrp_mxret(env, env->mstatus, prev_priv, prev_virt);
    }

    if (riscv_cpu_cfg(env)->ext_smdbltrp) {
        if (prev_priv < PRV_M) {
            env->mstatus = set_field(env->mstatus, MSTATUS_MDT, 0);
        }
    }

    if (riscv_has_ext(env, RVH) && prev_virt) {
        riscv_cpu_swap_hypervisor_regs(env);
    }

    riscv_cpu_set_mode(env, prev_priv, prev_virt);

    /*
     * If forward cfi enabled for new priv, restore elp status
     * and clear mnpelp in mnstatus
     */
    if (cpu_get_fcfien(env)) {
        env->elp = get_field(env->mnstatus, MNSTATUS_MNPELP);
    }
    env->mnstatus = set_field(env->mnstatus, MNSTATUS_MNPELP, 0);

    return retpc;
}

void helper_ctr_add_entry(CPURISCVState *env, target_ulong src,
                          target_ulong dest, target_ulong type)
{
    riscv_ctr_add_entry(env, src, dest, (enum CTRType)type,
                        env->priv, env->virt_enabled);
}

void helper_ctr_clear(CPURISCVState *env)
{
    /*
     * It's safe to call smstateen_acc_ok() for umode access regardless of the
     * state of bit 54 (CTR bit in case of m/hstateen) of sstateen. If the bit
     * is zero, smstateen_acc_ok() will return the correct exception code and
     * if it's one, smstateen_acc_ok() will return RISCV_EXCP_NONE. In that
     * scenario the U-mode check below will handle that case.
     */
    RISCVException ret = smstateen_acc_ok(env, 0, SMSTATEEN0_CTR);
    if (ret != RISCV_EXCP_NONE) {
        riscv_raise_exception(env, ret, GETPC());
    }

    if (env->priv == PRV_U) {
        /*
         * One corner case is when sctrclr is executed from VU-mode and
         * mstateen.CTR = 0, in which case we are supposed to raise
         * RISCV_EXCP_ILLEGAL_INST. This case is already handled in
         * smstateen_acc_ok().
         */
        uint32_t excep = env->virt_enabled ? RISCV_EXCP_VIRT_INSTRUCTION_FAULT :
            RISCV_EXCP_ILLEGAL_INST;
        riscv_raise_exception(env, excep, GETPC());
    }

    riscv_ctr_clear(env);
}

void helper_wfi(CPURISCVState *env)
{
    CPUState *cs = env_cpu(env);
    bool rvs = riscv_has_ext(env, RVS);
    bool prv_u = env->priv == PRV_U;
    bool prv_s = env->priv == PRV_S;

    if (((prv_s || (!rvs && prv_u)) && get_field(env->mstatus, MSTATUS_TW)) ||
        (rvs && prv_u && !env->virt_enabled)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    } else if (env->virt_enabled &&
               (prv_u || (prv_s && get_field(env->hstatus, HSTATUS_VTW)))) {
        riscv_raise_exception(env, RISCV_EXCP_VIRT_INSTRUCTION_FAULT, GETPC());
    } else {
        cs->halted = 1;
        cs->exception_index = EXCP_HLT;
        cpu_loop_exit(cs);
    }
}

void helper_wrs_nto(CPURISCVState *env)
{
    if (env->virt_enabled && (env->priv == PRV_S || env->priv == PRV_U) &&
        get_field(env->hstatus, HSTATUS_VTW) &&
        !get_field(env->mstatus, MSTATUS_TW)) {
        riscv_raise_exception(env, RISCV_EXCP_VIRT_INSTRUCTION_FAULT, GETPC());
    } else if (env->priv != PRV_M && get_field(env->mstatus, MSTATUS_TW)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
}

void helper_tlb_flush(CPURISCVState *env)
{
    CPUState *cs = env_cpu(env);
    if (!env->virt_enabled &&
        (env->priv == PRV_U ||
         (env->priv == PRV_S && get_field(env->mstatus, MSTATUS_TVM)))) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    } else if (env->virt_enabled &&
               (env->priv == PRV_U || get_field(env->hstatus, HSTATUS_VTVM))) {
        riscv_raise_exception(env, RISCV_EXCP_VIRT_INSTRUCTION_FAULT, GETPC());
    } else {
        tlb_flush(cs);
    }
}

void helper_tlb_flush_all(CPURISCVState *env)
{
    CPUState *cs = env_cpu(env);
    tlb_flush_all_cpus_synced(cs);
}

void helper_hyp_tlb_flush(CPURISCVState *env)
{
    CPUState *cs = env_cpu(env);

    if (env->virt_enabled) {
        riscv_raise_exception(env, RISCV_EXCP_VIRT_INSTRUCTION_FAULT, GETPC());
    }

    if (env->priv == PRV_M ||
        (env->priv == PRV_S && !env->virt_enabled)) {
        tlb_flush(cs);
        return;
    }

    riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
}

void helper_hyp_gvma_tlb_flush(CPURISCVState *env)
{
    if (env->priv == PRV_S && !env->virt_enabled &&
        get_field(env->mstatus, MSTATUS_TVM)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    helper_hyp_tlb_flush(env);
}

static int check_access_hlsv(CPURISCVState *env, bool x, uintptr_t ra)
{
    if (env->priv == PRV_M) {
        /* always allowed */
    } else if (env->virt_enabled) {
        riscv_raise_exception(env, RISCV_EXCP_VIRT_INSTRUCTION_FAULT, ra);
    } else if (env->priv == PRV_U && !get_field(env->hstatus, HSTATUS_HU)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, ra);
    }

    int mode = get_field(env->hstatus, HSTATUS_SPVP);
    if (!x && mode == PRV_S && get_field(env->vsstatus, MSTATUS_SUM)) {
        mode = MMUIdx_S_SUM;
    }
    return mode | MMU_2STAGE_BIT;
}

target_ulong helper_hyp_hlv_bu(CPURISCVState *env, target_ulong addr)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_UB, mmu_idx);

    return cpu_ldb_mmu(env, adjust_addr_virt(env, addr), oi, ra);
}

target_ulong helper_hyp_hlv_hu(CPURISCVState *env, target_ulong addr)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUW, mmu_idx);

    return cpu_ldw_mmu(env, adjust_addr_virt(env, addr), oi, ra);
}

target_ulong helper_hyp_hlv_wu(CPURISCVState *env, target_ulong addr)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUL, mmu_idx);

    return cpu_ldl_mmu(env, adjust_addr_virt(env, addr), oi, ra);
}

target_ulong helper_hyp_hlv_d(CPURISCVState *env, target_ulong addr)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUQ, mmu_idx);

    return cpu_ldq_mmu(env, adjust_addr_virt(env, addr), oi, ra);
}

void helper_hyp_hsv_b(CPURISCVState *env, target_ulong addr, target_ulong val)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_UB, mmu_idx);

    cpu_stb_mmu(env, adjust_addr_virt(env, addr), val, oi, ra);
}

void helper_hyp_hsv_h(CPURISCVState *env, target_ulong addr, target_ulong val)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUW, mmu_idx);

    cpu_stw_mmu(env, adjust_addr_virt(env, addr), val, oi, ra);
}

void helper_hyp_hsv_w(CPURISCVState *env, target_ulong addr, target_ulong val)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUL, mmu_idx);

    cpu_stl_mmu(env, adjust_addr_virt(env, addr), val, oi, ra);
}

void helper_hyp_hsv_d(CPURISCVState *env, target_ulong addr, target_ulong val)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, false, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUQ, mmu_idx);

    cpu_stq_mmu(env, adjust_addr_virt(env, addr), val, oi, ra);
}

/*
 * TODO: These implementations are not quite correct.  They perform the
 * access using execute permission just fine, but the final PMP check
 * is supposed to have read permission as well.  Without replicating
 * a fair fraction of cputlb.c, fixing this requires adding new mmu_idx
 * which would imply that exact check in tlb_fill.
 */
target_ulong helper_hyp_hlvx_hu(CPURISCVState *env, target_ulong addr)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, true, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUW, mmu_idx);

    return cpu_ldw_code_mmu(env, addr, oi, GETPC());
}

target_ulong helper_hyp_hlvx_wu(CPURISCVState *env, target_ulong addr)
{
    uintptr_t ra = GETPC();
    int mmu_idx = check_access_hlsv(env, true, ra);
    MemOpIdx oi = make_memop_idx(MO_TEUL, mmu_idx);

    return cpu_ldl_code_mmu(env, addr, oi, ra);
}

#endif /* !CONFIG_USER_ONLY */
