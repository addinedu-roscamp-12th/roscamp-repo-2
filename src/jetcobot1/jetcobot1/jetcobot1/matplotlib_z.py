import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

targets = [-10,-20,-30,-40,-50,-60,-70,-80,-90,-100]
actuals = [-7.6,-17.6,-27.4,-37.4,-47.3,-57.2,-67.1,-77.1,-87.2,-97.2]
errors = [2.4,2.4,2.6,2.6,2.7,2.8,2.9,2.9,2.8,2.8]

sns.set_theme(style="whitegrid")
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# 1. 목표 vs 실제
ax1.plot(targets, targets, '--', color='steelblue', label='목표값', linewidth=1.5)
ax1.plot(targets, actuals, 'o-', color='tomato', label='실제값', linewidth=2)
ax1.set_xlabel("목표 이동 (mm)")
ax1.set_ylabel("실제 이동 (mm)")
ax1.set_title("Z축: 목표 vs 실제")
ax1.legend()

# 2. 절대 오차
ax2.bar(targets, errors, color='mediumseagreen', width=7, label='절대 오차')
ax2.axhline(np.mean(errors), color='gray', linestyle='--', linewidth=1.5, label=f'평균 {np.mean(errors):.2f}mm')
ax2.set_xlabel("목표 이동 (mm)")
ax2.set_ylabel("오차 (mm)")
ax2.set_title("Z축: 절대 오차")
ax2.set_ylim(0, 4)
ax2.legend()

plt.tight_layout()
plt.savefig("z_axis_error.png", dpi=150)
plt.show()