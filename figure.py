import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import ScalarFormatter

# 1. 加载数据
file_path = '/Users/sunking/Desktop/QEM.xlsx' 
try:
    df = pd.read_excel(file_path)
except:
    df = pd.read_csv('QEM.xlsx - Sheet1.csv')

# 2. 筛选 ratio 和模型
target_ratios = [0.2, 0.4, 0.6, 0.8]
df_filtered = df[df['ratio'].isin(target_ratios)].copy()
models = df['model'].unique()[:10]

# 3. 绘图配置
fig, axes = plt.subplots(2, 5, figsize=(18, 9))
axes = axes.flatten()

bar_width = 0.18
metrics = ['V_in', 'V_out', 'F_in', 'F_out']
colors = ['#A0CBE8','#4E79A7', '#FF9D9A','#E15759'] 

# 4. 循环绘制
for i, model in enumerate(models):
    ax1 = axes[i]
    model_data = df_filtered[df_filtered['model'] == model].sort_values('ratio')
    
    if model_data.empty: continue

    x_indices = np.arange(len(target_ratios))
    
    # --- 绘制柱状图 (左轴 ax1) ---
    for j, metric in enumerate(metrics):
        pos = x_indices + (j - 1.5) * bar_width
        ax1.bar(pos, model_data[metric], bar_width, label=metric, color=colors[j], edgecolor='white')
    
    ax1.set_yscale('log')
    ax1.yaxis.set_major_formatter(ScalarFormatter())
    
    # 【修改1：左轴边框加粗】
    for spine in ax1.spines.values():
        spine.set_linewidth(2)  # 设置边框粗细为2
        spine.set_edgecolor('black')

    ax1.tick_params(axis='both', which='both', colors='black', width=2) # 刻度线也加粗
    
    # --- 绘制折线图 (右轴 ax2) ---
    ax2 = ax1.twinx()
    times = model_data['time_total_ms'].values
    ax2.plot(x_indices, times, color='red', marker='o', markersize=6, linewidth=2, label='Time')
    
    # 【修改2：右轴数值与边框加粗】
    # 设置右轴刻度直接显示表格中的实际数值
    ax2.set_yticks(times) 
    
    # 设置右轴不使用科学计数法
    y_formatter = ScalarFormatter(useOffset=False)
    y_formatter.set_scientific(False)
    ax2.yaxis.set_major_formatter(y_formatter)

    # 右轴边框加粗
    ax2.spines['right'].set_linewidth(2)
    ax2.spines['top'].set_linewidth(2) # twinx 共享 top spine
    ax2.tick_params(axis='y', colors='black', width=2)

    # --- Label 控制 ---
    if i % 5 == 0:
        ax1.set_ylabel('Geometry Count (V/F)', fontsize=12, fontweight='bold', color='black')
    
    if (i + 1) % 5 == 0:
        ax2.set_ylabel('Total Time (ms)', fontsize=12, fontweight='bold', color='black')

    # --- 标题与横轴 ---
    ax1.set_title(model, fontsize=15, pad=15, fontweight='bold')
    ax1.set_xticks(x_indices)
    ax1.set_xticklabels(target_ratios) 
    
    if i >= 5:
        ax1.set_xlabel('Simplification Ratio', fontsize=11, fontweight='bold')

# 5. 全局图例
h1, l1 = axes[0].get_legend_handles_labels()
h2, l2 = ax2.get_legend_handles_labels()
fig.legend(h1 + h2, l1 + l2, loc='upper center', bbox_to_anchor=(0.5, 0.98), 
           ncol=5, fontsize=13, frameon=False)

plt.tight_layout(rect=[0, 0.03, 1, 0.94])
plt.savefig('simplification_final_plot_bold.png', dpi=300, bbox_inches='tight')
plt.show()