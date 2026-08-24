"""B5 CHECKPOINT B -- mission-control HUD, rendered offline."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib.patches import Patch
from PIL import Image

OUT = "hud_frames"
KEY = "hud_keyframes"
PHASES = [("SURVEY", "#00d4ff"), ("MAP READY", "#ffd400"),
          ("ROVER DRIVING", "#ff8c00"), ("ARRIVED", "#3ddc57")]


def build_frames():
    z = np.load("b5_hud_timeline.npz")
    n_s = len(z["t"])
    rover = z["rover_path"]
    n_hold, n_rov, n_end = 8, min(60, len(rover)), 8
    plan = ([("SURVEY", i, None) for i in range(n_s)]
            + [("MAP READY", n_s-1, None) for _ in range(n_hold)]
            + [("ROVER DRIVING", n_s-1, int(k*(len(rover)-1)/(n_rov-1)))
               for k in range(n_rov)]
            + [("ARRIVED", n_s-1, len(rover)-1) for _ in range(n_end)])
    return z, plan


def render(z, plan, idxs, outdir, tag=""):
    os.makedirs(outdir, exist_ok=True)
    hm = z["hmap"]; cell = float(z["cell"]); org = z["origin"]
    ext = [org[0], org[0]+hm.shape[0]*cell, org[1], org[1]+hm.shape[1]*cell]
    ls = LightSource(azdeg=315, altdeg=45)
    shade = ls.hillshade(hm.T, vert_exag=2.0, dx=cell, dy=cell)
    t = z["t"]; err = z["err"]; cov = z["cov"]
    dtrue = z["drone_true"]; dest = z["drone_est"]
    rover = z["rover_path"]; rs = z["rover_start"]; rg = z["rover_goal"]
    paths = []
    for fi, (phase, si, ri) in enumerate(plan):
        if fi not in idxs:
            continue
        known = z["known"][si]; haz = z["haz"][si]
        fig = plt.figure(figsize=(16, 9), facecolor="#0b0e13")
        gs = fig.add_gridspec(3, 3, width_ratios=[2.3, 1, 1],
                              hspace=0.35, wspace=0.25)

        a = fig.add_subplot(gs[:, 0]); a.set_facecolor("#0b0e13")
        a.imshow(shade, origin="lower", extent=ext, cmap="copper", alpha=0.95)
        ov = np.zeros((*hm.shape, 4))
        ov[known & ~haz] = [0.15, 0.85, 0.35, 0.34]
        ov[haz] = [0.95, 0.15, 0.15, 0.72]
        ov[~known] = [0.05, 0.06, 0.09, 0.80]
        a.imshow(np.transpose(ov, (1, 0, 2)), origin="lower", extent=ext)

        a.plot(dtrue[:si+1, 0], dtrue[:si+1, 1], color="#00d4ff", lw=1.6,
               alpha=0.9, label="drone survey track")
        a.plot(*dtrue[si, :2], "o", color="#00d4ff",
               ms=15 if phase == "SURVEY" else 10, mec="w", mew=1.5,
               label="DRONE" if phase == "SURVEY" else "drone (survey complete)",
               zorder=6)
        if ri is None:
            a.plot(*rs, "s", color="#888", ms=13, mec="w", label="rover (waiting)")
        else:
            a.plot(rover[:ri+1, 0], rover[:ri+1, 1], color="#ffd400", lw=3.0,
                   label="rover route driven")
            a.plot(rover[:, 0], rover[:, 1], color="#ffd400", lw=1.0, ls=":",
                   alpha=0.5)
            a.plot(*rover[ri], "s", color="#ff8c00", ms=15, mec="w", mew=1.5,
                   label="ROVER", zorder=6)
        a.plot(*rg, "*", color="#3ddc57", ms=20, mec="k", label="rover goal")
        a.set_xlim(ext[0], ext[1]); a.set_ylim(ext[2], ext[3])
        a.set_xlabel("x [m]", color="#9aa"); a.set_ylabel("y [m]", color="#9aa")
        a.tick_params(colors="#667")
        for s in a.spines.values():
            s.set_color("#333")
        a.legend(loc="lower left", fontsize=8, facecolor="#12161d",
                 edgecolor="#333", labelcolor="#cde")
        a.set_title("SkyScout — Mars scout & rover", color="#dfe", fontsize=15, pad=12)

        b = fig.add_subplot(gs[0, 1:]); b.axis("off"); b.set_facecolor("#0b0e13")
        col = dict(PHASES)[phase]
        b.add_patch(plt.Rectangle((0.02, 0.45), 0.96, 0.42, color=col, alpha=0.22,
                                  transform=b.transAxes))
        b.text(0.5, 0.66, phase, transform=b.transAxes, ha="center", va="center",
               fontsize=30, color=col, fontweight="bold")
        order = " → ".join(p for p, _ in PHASES)
        b.text(0.5, 0.30, order, transform=b.transAxes, ha="center",
               fontsize=10, color="#667")
        b.text(0.5, 0.10, f"t = {t[si]:6.1f} s", transform=b.transAxes,
               ha="center", fontsize=12, color="#9aa")

        c = fig.add_subplot(gs[1, 1]); c.set_facecolor("#12161d")
        c.plot(t[:si+1], 100*cov[:si+1], color="#3ddc57", lw=2)
        c.fill_between(t[:si+1], 0, 100*cov[:si+1], color="#3ddc57", alpha=0.2)
        c.set_ylim(0, 105); c.set_xlim(0, t[-1])
        c.set_title(f"map coverage  {100*cov[si]:.0f}%", color="#dfe", fontsize=10)
        c.tick_params(colors="#667", labelsize=7)
        for s in c.spines.values(): s.set_color("#333")

        d = fig.add_subplot(gs[1, 2]); d.set_facecolor("#12161d")
        d.plot(t[:si+1], err[:si+1], color="#ff5c5c", lw=1.6)
        d.axhline(err[:si+1].mean(), color="#ffd400", ls=":", lw=1)
        d.set_ylim(0, max(1.6, err.max()*1.1)); d.set_xlim(0, t[-1])
        d.set_title(f"ESKF error  {err[si]:.2f} m", color="#dfe", fontsize=10)
        d.tick_params(colors="#667", labelsize=7)
        for s in d.spines.values(): s.set_color("#333")

        e = fig.add_subplot(gs[2, 1:]); e.axis("off"); e.set_facecolor("#0b0e13")
        e.text(0.02, 0.86, "DRONE NAVIGATION (GPS-denied)", fontsize=10,
               color="#8ab", transform=e.transAxes, fontweight="bold")
        e.text(0.02, 0.66, f"believed   {dest[si,0]:7.2f}, {dest[si,1]:7.2f} m",
               fontsize=11, color="#ffd400", transform=e.transAxes, family="monospace")
        e.text(0.02, 0.50, f"actual     {dtrue[si,0]:7.2f}, {dtrue[si,1]:7.2f} m",
               fontsize=11, color="#8fa", transform=e.transAxes, family="monospace")
        e.text(0.02, 0.34, f"error      {err[si]:5.2f} m", fontsize=11,
               color="#ff5c5c", transform=e.transAxes, family="monospace")
        e.text(0.02, 0.14, "actual position is dev-only — the drone never sees it",
               fontsize=8, color="#556", transform=e.transAxes, style="italic")
        e.legend(handles=[
            Patch(facecolor="#1a1207", edgecolor="#8a7", lw=1.4,
                  label="UNKNOWN — never treated as safe"),
            Patch(facecolor="#4f8f52", edgecolor="#27d959", lw=1.2,
                  label="mapped safe"),
            Patch(facecolor="#b03434", edgecolor="#f22626", lw=1.2,
                  label="mapped hazard")],
            loc="lower right", fontsize=9, facecolor="#12161d",
            edgecolor="#445", labelcolor="#cde", framealpha=1.0)

        p = os.path.join(outdir, f"{tag}f{fi:04d}.png")
        fig.savefig(p, dpi=80, facecolor="#0b0e13")
        plt.close(fig)
        paths.append(p)
    return paths


def main():
    z, plan = build_frames()
    n = len(plan)
    print(f"HUD timeline: {n} frames  "
          f"({sum(1 for p in plan if p[0]=='SURVEY')} survey, "
          f"{sum(1 for p in plan if p[0]=='ROVER DRIVING')} rover)")

    keys = [0, 25, 55, 77, 77+4, 77+8+20, 77+8+50, n-1]
    keys = sorted(set(min(k, n-1) for k in keys))
    print(f"rendering {len(keys)} keyframes for review ...")
    kp = render(z, plan, set(keys), KEY, tag="key_")
    for p, k in zip(kp, keys):
        print(f"   {p}   phase={plan[k][0]}")

    print(f"rendering all {n} frames ...")
    render(z, plan, set(range(n)), OUT)
    frames = sorted(os.listdir(OUT))
    print(f"   {len(frames)} frames in {OUT}/")

    imgs = [Image.open(os.path.join(OUT, f)).convert("P", palette=Image.ADAPTIVE)
            for f in frames]
    imgs[0].save("skyscout_mission.gif", save_all=True, append_images=imgs[1:],
                 duration=90, loop=0, optimize=True)
    mb = os.path.getsize("skyscout_mission.gif")/1e6
    print(f"   skyscout_mission.gif  ({mb:.1f} MB, {len(imgs)} frames)")
    print("   (no ffmpeg on this box -- GIF is the portable fallback; frames are")
    print("    on disk as PNGs so an mp4 is one ffmpeg call away if you install it)")


if __name__ == "__main__":
    main()
