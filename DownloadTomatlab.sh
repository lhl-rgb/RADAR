#!/bin/bash
# 文件名: export_all_clutter_patches.sh
# 用途: 批量导出四种杂波分布的 patch 数据，并复制到 Windows 共享目录
# 位置: ~/Radar/export_all_clutter_patches.sh

set -e

# ============================================
# 配置区 - 根据需要修改
# ============================================

# 本地输出目录（WSL2 本地）
LOCAL_OUT_DIR="$HOME/Radar/out/clutter_patches"

# Windows 共享目录（挂载到 WSL2 的 Windows 盘）
# 默认: D:\Project\RaderEchoSimulation\四种杂波仿真函数\out
WINDOWS_OUT_DIR="/mnt/d/Project/RaderEchoSimulation/四种杂波仿真函数/out"

# ============================================
# 默认测试参数
# ============================================
SAMPLE_COUNT=20000
RANGE_IDX=1000
AZ_IDX=0
FIR_LENGTH=13
SIGMA_F_HZ=40
DOPPLER_CENTER=0
PRF_HZ=1600
SEED=2026

# ============================================
# 函数定义
# ============================================

print_banner() {
    echo ""
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║        🚀 雷达杂波 Patch 批量导出工具 v1.0                ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo ""
}

print_matlab_validation() {
    echo ""
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║           📊 MATLAB 验证代码                              ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📍 数据目录（Windows）:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "   cd 'D:\\Project\\RaderEchoSimulation\\四种杂波仿真函数\\out'"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📍 MATLAB 代码:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "% 1. 添加路径"
    echo "addpath('test')"
    echo "addpath('D:\\Project\\RaderEchoSimulation\\四种杂波仿真函数\\out')"
    echo ""
    echo "% 2. 验证 Rayleigh 分布"
    echo "fprintf('\\n========== Rayleigh ==========\\n');"
    echo "validate_clutter_patch_trace('clutter_patch_rayleigh_trace.txt', ..."
    echo "                             'clutter_patch_rayleigh_psd.txt');"
    echo ""
    echo "% 3. 验证 Weibull 分布"
    echo "fprintf('\\n========== Weibull ==========\\n');"
    echo "validate_clutter_patch_trace('clutter_patch_weibull_trace.txt', ..."
    echo "                             'clutter_patch_weibull_psd.txt');"
    echo ""
    echo "% 4. 验证 LogNormal 分布"
    echo "fprintf('\\n========== LogNormal ==========\\n');"
    echo "validate_clutter_patch_trace('clutter_patch_lognormal_trace.txt', ..."
    echo "                             'clutter_patch_lognormal_psd.txt');"
    echo ""
    echo "% 5. 验证 K 分布"
    echo "fprintf('\\n========== K Distribution ==========\\n');"
    echo "validate_clutter_patch_trace('clutter_patch_k_trace.txt', ..."
    echo "                             'clutter_patch_k_psd.txt');"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📍 或者一次性验证所有分布:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo 'distributions = {...'
    echo '    struct(''name'', ''Rayleigh'', ...'
    echo '            ''trace'', ''clutter_patch_rayleigh_trace.txt'', ...'
    echo '            ''psd'', ''clutter_patch_rayleigh_psd.txt''), ...'
    echo '    struct(''name'', ''Weibull'', ...'
    echo '            ''trace'', ''clutter_patch_weibull_trace.txt'', ...'
    echo '            ''psd'', ''clutter_patch_weibull_psd.txt''), ...'
    echo '    struct(''name'', ''LogNormal'', ...'
    echo '            ''trace'', ''clutter_patch_lognormal_trace.txt'', ...'
    echo '            ''psd'', ''clutter_patch_lognormal_psd.txt''), ...'
    echo '    struct(''name'', ''K'', ...'
    echo '            ''trace'', ''clutter_patch_k_trace.txt'', ...'
    echo '            ''psd'', ''clutter_patch_k_psd.txt'') ...'
    echo '};'
    echo ""
    echo 'for i = 1:length(distributions)'
    echo '    fprintf("\\n========== %s ==========\\n", distributions(i).name);'
    echo '    validate_clutter_patch_trace(distributions(i).trace, ...'
    echo '                                 distributions(i).psd);'
    echo 'end'
}

print_summary() {
    echo ""
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║           ✅ 导出完成 - 文件清单                           ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo ""
    echo "📁 本地目录 (WSL2): $LOCAL_OUT_DIR"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if [ -d "$LOCAL_OUT_DIR" ]; then
        ls -lh "$LOCAL_OUT_DIR"/*.txt 2>/dev/null | awk '{print "   "$9"  ("$5")"}'
    else
        echo "   (目录不存在)"
    fi
    echo ""
    
    if [ "$COPY_TO_WINDOWS" = true ]; then
        echo "📁 Windows 目录: D:\Project\RaderEchoSimulation\四种杂波仿真函数\out"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        ls -lh "$WINDOWS_OUT_DIR"/*.txt 2>/dev/null | awk '{print "   "$9"  ("$5")"}'
        echo ""
        echo "💡 直接复制到 MATLAB 工作目录即可验证"
    else
        echo "⚠️  Windows 目录不可访问，跳过复制"
    fi
    
    echo ""
    echo "📊 共生成文件: 8 个 (.txt)"
    echo "   • 4 个 *_trace.txt (慢时间 IQ 数据)"
    echo "   • 4 个 *_psd.txt (功率谱密度数据)"
}

# ============================================
# 主流程
# ============================================

main() {
    print_banner
    
    echo "📍 本地输出: $LOCAL_OUT_DIR"
    echo "📍 Windows 输出: $WINDOWS_OUT_DIR"
    echo ""
    
    # 检查并创建目录
    echo "📁 创建本地输出目录..."
    mkdir -p "$LOCAL_OUT_DIR"
    
    COPY_TO_WINDOWS=true
    if [ -d "$(dirname "$WINDOWS_OUT_DIR")" ]; then
        mkdir -p "$WINDOWS_OUT_DIR"
        echo "✅ Windows 目录已就绪"
    else
        echo "⚠️  警告: Windows 目录不可访问，将跳过复制步骤"
        COPY_TO_WINDOWS=false
    fi
    
    # 切换到 Radar 目录
    cd ~/Radar
    
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📦 开始导出四种杂波分布..."
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # 1. Rayleigh 分布
    echo ""
    echo "🔧 [1/4] 导出 Rayleigh 分布..."
    ./out/bin/export_clutter_patch_trace \
        $SAMPLE_COUNT $RANGE_IDX $AZ_IDX $FIR_LENGTH \
        $SIGMA_F_HZ $DOPPLER_CENTER $PRF_HZ $SEED \
        "$LOCAL_OUT_DIR/clutter_patch_rayleigh" \
        0
    echo "   ✅ clutter_patch_rayleigh_trace.txt"
    echo "   ✅ clutter_patch_rayleigh_psd.txt"
    
    # 2. Weibull 分布
    echo ""
    echo "🔧 [2/4] 导出 Weibull 分布 (shape=1.5, scale=1.2)..."
    ./out/bin/export_clutter_patch_trace \
        $SAMPLE_COUNT $RANGE_IDX $AZ_IDX $FIR_LENGTH \
        $SIGMA_F_HZ $DOPPLER_CENTER $PRF_HZ $SEED \
        "$LOCAL_OUT_DIR/clutter_patch_weibull" \
        1 1.5 1.2
    echo "   ✅ clutter_patch_weibull_trace.txt"
    echo "   ✅ clutter_patch_weibull_psd.txt"
    
    # 3. LogNormal 分布
    echo ""
    echo "🔧 [3/4] 导出 LogNormal 分布 (μ=-0.5, σ=0.5)..."
    ./out/bin/export_clutter_patch_trace \
        $SAMPLE_COUNT $RANGE_IDX $AZ_IDX $FIR_LENGTH \
        $SIGMA_F_HZ $DOPPLER_CENTER $PRF_HZ $SEED \
        "$LOCAL_OUT_DIR/clutter_patch_lognormal" \
        2 1.5 1.2 -0.5 0.5
    echo "   ✅ clutter_patch_lognormal_trace.txt"
    echo "   ✅ clutter_patch_lognormal_psd.txt"
    
    # 4. K 分布
    echo ""
    echo "🔧 [4/4] 导出 K 分布 (ν=2.0, bw=2.0)..."
    ./out/bin/export_clutter_patch_trace \
        $SAMPLE_COUNT $RANGE_IDX $AZ_IDX $FIR_LENGTH \
        $SIGMA_F_HZ $DOPPLER_CENTER $PRF_HZ $SEED \
        "$LOCAL_OUT_DIR/clutter_patch_k" \
        3 1.5 1.2 -0.5 0.5 2.0 2.0 4096
    echo "   ✅ clutter_patch_k_trace.txt"
    echo "   ✅ clutter_patch_k_psd.txt"
    
    # 复制到 Windows
    if [ "$COPY_TO_WINDOWS" = true ]; then
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "📂 复制文件到 Windows 目录..."
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        cp -v "$LOCAL_OUT_DIR"/*.txt "$WINDOWS_OUT_DIR/"
    fi
    
    # 输出总结和 MATLAB 验证代码
    print_summary
    print_matlab_validation
    
    echo ""
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                  🎉 全部完成！                             ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo ""
}

# 运行
main "$@"