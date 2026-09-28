"""
Generates an annotated mechanical schematic diagram of Spencer's Modified Bouc-Wen MR Damper.
"""

from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def generate_schematic():
    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 7)
    ax.axis('off')

    # Color palette
    c_frame = '#2b2b2b'
    c_spring = '#1f77b4'
    c_dash = '#ff7f0e'
    c_bw = '#d62728'
    c_acc = '#2ca02c'
    c_text = '#111111'

    # Left boundary (fixed base / housing)
    ax.plot([0, 0], [0.5, 6], color=c_frame, lw=3)
    for y in np.linspace(0.5, 6, 12):
        ax.plot([0, -0.4], [y, y - 0.3], color=c_frame, lw=1.5)
    ax.text(-0.6, 3.25, 'Base\n(Fixed)', ha='right', va='center', fontweight='bold', fontsize=11)

    # Right boundary (piston shaft / attachment x)
    ax.plot([10, 10], [0.5, 6], color=c_frame, lw=3)
    ax.plot([10, 10.8], [3.25, 3.25], color=c_frame, lw=3)
    ax.annotate('', xy=(10.8, 3.25), xytext=(9.8, 3.25),
                arrowprops=dict(arrowstyle="-|>", lw=2, color='red'))
    ax.text(10.9, 3.25, '$x(t), \\dot{x}(t)$', ha='left', va='center', fontsize=13, fontweight='bold', color='red')

    # Intermediate node y
    ax.plot([4.8, 4.8], [1.5, 5.8], color='#555555', lw=2.5, ls='--')
    ax.text(4.8, 6.1, 'Internal Node $y(t)$', ha='center', va='bottom', fontsize=11, fontweight='bold', color='#444444')

    # Accumulator spring k1 (x to base) at bottom (y = 1.0)
    ax.plot([0, 2.5], [1.0, 1.0], color=c_spring, lw=2)
    # Spring zigzag
    xs_k1 = np.linspace(2.5, 7.5, 30)
    ys_k1 = 1.0 + 0.35 * np.sin(np.linspace(0, 8 * np.pi, 30))
    ax.plot(xs_k1, ys_k1, color=c_spring, lw=2)
    ax.plot([7.5, 10], [1.0, 1.0], color=c_spring, lw=2)
    ax.text(5.0, 0.4, 'Accumulator Stiffness: $k_1 (x - x_0)$', ha='center', va='top', fontsize=11, color=c_spring, fontweight='bold')

    # Dashpot c1 (y to base) (y = 2.5)
    ax.plot([0, 2.0], [2.5, 2.5], color=c_acc, lw=2)
    # Dashpot box
    rect_c1 = patches.Rectangle((2.0, 2.2), 1.2, 0.6, fill=False, edgecolor=c_acc, lw=2)
    ax.add_patch(rect_c1)
    ax.plot([2.5, 2.5], [2.3, 2.7], color=c_acc, lw=2)
    ax.plot([2.5, 4.8], [2.5, 2.5], color=c_acc, lw=2)
    ax.text(2.6, 3.0, 'Accumulator Viscous: $c_1$', ha='center', va='bottom', fontsize=10, color=c_acc, fontweight='bold')

    # Upper branch between node y and shaft x (y = 3.8 to 5.5)
    # Top: Dashpot c0 (y = 5.2)
    ax.plot([4.8, 6.6], [5.2, 5.2], color=c_dash, lw=2)
    rect_c0 = patches.Rectangle((6.6, 4.9), 1.2, 0.6, fill=False, edgecolor=c_dash, lw=2)
    ax.add_patch(rect_c0)
    ax.plot([7.1, 7.1], [5.0, 5.4], color=c_dash, lw=2)
    ax.plot([7.1, 10], [5.2, 5.2], color=c_dash, lw=2)
    ax.text(7.2, 5.65, 'Viscous Damping: $c_0$', ha='center', va='bottom', fontsize=10, color=c_dash, fontweight='bold')

    # Middle: Spring k0 (y = 4.2)
    ax.plot([4.8, 6.2], [4.2, 4.2], color='#8c564b', lw=2)
    xs_k0 = np.linspace(6.2, 8.2, 20)
    ys_k0 = 4.2 + 0.25 * np.sin(np.linspace(0, 6 * np.pi, 20))
    ax.plot(xs_k0, ys_k0, color='#8c564b', lw=2)
    ax.plot([8.2, 10], [4.2, 4.2], color='#8c564b', lw=2)
    ax.text(7.2, 3.75, 'Stiffness: $k_0$', ha='center', va='top', fontsize=10, color='#8c564b', fontweight='bold')

    # Lower: Bouc-Wen Element alpha*z (y = 3.2)
    ax.plot([4.8, 6.2], [3.2, 3.2], color=c_bw, lw=2)
    rect_bw = patches.Rectangle((6.2, 2.9), 2.0, 0.6, facecolor='#ffebee', edgecolor=c_bw, lw=2)
    ax.add_patch(rect_bw)
    ax.text(7.2, 3.2, 'Bouc-Wen: $\\alpha z$', ha='center', va='center', fontsize=11, color=c_bw, fontweight='bold')
    ax.plot([8.2, 10], [3.2, 3.2], color=c_bw, lw=2)

    # Total Force arrow at right
    ax.annotate('', xy=(8.5, -0.2), xytext=(1.5, -0.2),
                arrowprops=dict(arrowstyle="<|-|>", lw=2.5, color='#111111'))
    ax.text(5.0, -0.6, 'Total Output Force: $F_{MR} = c_1 \\dot{y} + k_1(x - x_0)$',
            ha='center', va='center', fontsize=12, fontweight='bold', color='#111111')

    # Title
    ax.set_title("Spencer's Modified Bouc-Wen Mechanical Model Architecture",
                 fontsize=14, fontweight='bold', pad=20)

    fig.tight_layout()
    out_path = Path(__file__).resolve().parent / "model_schematic.png"
    fig.savefig(out_path)
    plt.close(fig)
    print(f"Schematic successfully generated at {out_path}")

if __name__ == '__main__':
    import numpy as np
    generate_schematic()
