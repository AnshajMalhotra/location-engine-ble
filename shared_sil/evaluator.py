import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List, Dict, Tuple

class PerformanceEvaluator:
    """
    后处理与评估引擎 (Evaluator)
    职责: 计算量化指标 (RMSE 等) 并渲染可视化孪生图表
    """
    def __init__(self, rooms: List, walls: List, anchors: np.ndarray):
        self.rooms = rooms
        self.walls = walls
        self.anchors = anchors

    def calculate_rmse(self, true_path: np.ndarray, est_path: np.ndarray) -> float:
        """计算核心指标：均方根误差 (RMSE)"""
        if len(true_path) == 0 or len(est_path) == 0:
            return 0.0
        # 计算每一帧的欧氏距离误差的平方均值，再开根号
        mse = np.mean(np.linalg.norm(true_path - est_path, axis=1)**2)
        return float(np.sqrt(mse))

    def render(self, true_path: np.ndarray, results: Dict[str, np.ndarray], save_path: str = "eval_result.png"):
        """
        渲染对比图表，自动计算并标注 RMSE
        results: 字典格式，例如 {'Raw': path_raw, 'Naive PF': path_naive, 'WAF PF': path_waf}
        """
        fig, ax = plt.subplots(figsize=(14, 7))

        # 1. 绘制数字孪生底图 (NavMesh & Walls)
        for r in self.rooms:
            rect = patches.Rectangle((r[0], r[1]), r[2]-r[0], r[3]-r[1], 
                                     linewidth=1, facecolor='#f8f9fa', edgecolor='gray', alpha=0.5, zorder=0)
            ax.add_patch(rect)
            
        for w in self.walls:
            ax.plot([w[0][0], w[1][0]], [w[0][1], w[1][1]], 'k-', linewidth=3, zorder=10)
            
        # 2. 绘制基站与 Ground Truth
        ax.scatter(self.anchors[:, 0], self.anchors[:, 1], c='blue', marker='o', s=80, label='Anchors', zorder=15)
        ax.plot(true_path[:, 0], true_path[:, 1], 'g--', linewidth=2.5, label='Ground Truth', zorder=20)

        # 3. 动态绘制各算法结果并计算 RMSE
        colors = ['magenta', 'red', 'blue', 'orange']
        styles = ['x', '-', '-', '-']
        
        for idx, (model_name, est_path) in enumerate(results.items()):
            color = colors[idx % len(colors)]
            
            # 特殊处理 Raw Solver (散点展示，不连线)
            if 'Raw' in model_name:
                ax.scatter(est_path[:, 0], est_path[:, 1], c=color, marker='x', s=20, alpha=0.6, label=f'{model_name} (Chaos)')
            else:
                rmse = self.calculate_rmse(true_path, est_path)
                label_text = f'{model_name} (RMSE: {rmse:.2f}m)'
                ax.plot(est_path[:, 0], est_path[:, 1], color=color, linestyle=styles[idx % len(styles)], 
                        linewidth=2.5, alpha=0.8, label=label_text)

        # 4. 图表自适应排版
        ax.autoscale() # 自动根据散点扩展坐标轴
        ax.set_aspect('equal')
        ax.set_title("Localization Engine Performance Evaluation (SiL)", fontsize=14)
        ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1)) # 图例放在外侧防遮挡
        ax.grid(True, linestyle=':', alpha=0.7)

        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        plt.show()