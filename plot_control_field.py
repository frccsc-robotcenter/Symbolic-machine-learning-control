#!/usr/bin/env python3
"""Карта управления NetOper / П-регулятора на сетке (dist, bearing_err).

Рисует u_left, u_right и (u_left - u_right) — разницу колёс (поворот).
Помогает увидеть мёртвые зоны и «закрученность».
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def p_control(dist, bearing, kv=0.4, kw=0.8):
    u_l = kv * dist + kw * bearing
    u_r = kv * dist - kw * bearing
    # clamp like Controller
    u_l = np.clip(u_l, -1.0, 1.0)
    u_r = np.clip(u_r, -1.0, 1.0)
    return u_l, u_r


def main():
    dist = np.linspace(0.0, 8.0, 41)
    bearing = np.linspace(-np.pi, np.pi, 41)
    D, B = np.meshgrid(dist, bearing, indexing="xy")
    UL, UR = p_control(D, B)

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))

    im0 = axes[0].pcolormesh(D, B, UL, shading="auto", cmap="coolwarm", vmin=-1, vmax=1)
    axes[0].set_title("u_left (P-controller)")
    axes[0].set_xlabel("dist")
    axes[0].set_ylabel("bearing_err")
    fig.colorbar(im0, ax=axes[0])

    im1 = axes[1].pcolormesh(D, B, UR, shading="auto", cmap="coolwarm", vmin=-1, vmax=1)
    axes[1].set_title("u_right (P-controller)")
    axes[1].set_xlabel("dist")
    fig.colorbar(im1, ax=axes[1])

    im2 = axes[2].pcolormesh(D, B, UL - UR, shading="auto", cmap="coolwarm", vmin=-2, vmax=2)
    axes[2].set_title("u_left - u_right (turn)")
    axes[2].set_xlabel("dist")
    fig.colorbar(im2, ax=axes[2])

    for ax in axes:
        ax.axhline(0.0, color="k", lw=0.5, ls="--")

    fig.tight_layout()
    out = "control_field_p.png"
    fig.savefig(out, dpi=120)
    print(f"saved {out}")

    # Скачать поле в CSV для внешней отрисовки NetOper
    with open("control_field_p.csv", "w") as f:
        f.write("dist,bearing,u_left,u_right\n")
        for i in range(D.shape[0]):
            for j in range(D.shape[1]):
                f.write(f"{D[i,j]:.4f},{B[i,j]:.4f},{UL[i,j]:.4f},{UR[i,j]:.4f}\n")
    print("saved control_field_p.csv")


if __name__ == "__main__":
    main()
