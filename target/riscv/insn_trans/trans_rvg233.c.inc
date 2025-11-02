/*
 * DMA 矩阵转置指令
 * 功能：将源地址的矩阵按指定粒度转置到目标地址
 */
static bool trans_dma(DisasContext *ctx, arg_dma *a)
{
    TCGv dst = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv gran = get_gpr(ctx, a->rs2, EXT_NONE);
    /* 调用 helper 函数执行 DMA 转置 */
    gen_helper_dma(tcg_env, dst, src, gran);
    return true;
}

/*
 * Sort 排序指令
 * 功能：对数组进行排序
 */
static bool trans_sort(DisasContext *ctx, arg_sort *a)
{
    TCGv addr = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv sort_num = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv array_size = get_gpr(ctx, a->rs2, EXT_NONE);

    gen_helper_sort(tcg_env, addr, sort_num, array_size);

    return true;
}

/*
 * Crush 压缩指令
 * 功能：将 8 位数组的低 4 位两两打包
 * 
 * 指令格式：crush rd, rs1, rs2
 *   - rd: 目标数组地址（打包后的数组）
 *   - rs1: 源数组地址（待打包的数组）
 *   - rs2: 源数组元素数量
 * 
 * 压缩规则：
 *   dst[i] = (src[2*i] & 0xF) | ((src[2*i+1] & 0xF) << 4)
 */
static bool trans_crush(DisasContext *ctx, arg_crush *a)
{
    TCGv dst = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv num = get_gpr(ctx, a->rs2, EXT_NONE);

    /* 调用 helper 函数执行压缩 */
    gen_helper_crush(tcg_env, dst, src, num);

    return true;
}

/*
 * Expand 扩展指令
 * 功能：数据扩展操作
 */
static bool trans_expand(DisasContext *ctx, arg_expand *a)
{
    TCGv dst = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv num = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_expand(tcg_env, dst, src, num);
    return true;
}
