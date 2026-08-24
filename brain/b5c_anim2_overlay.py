"""B5C v2 -- composite the SCAN MINIMAP onto the rendered frames."""
import json, os, sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "renders", "anim2_raw")
FIN = os.path.join(HERE, "renders", "anim2_final")
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_M = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"

MAP_PX = 300
MARGIN = 22
FOOTPRINT_M = 13.9

COL = {
    "unknown":  (26, 22, 20),
    "safe":     (44, 132, 62),
    "hazard":   (196, 46, 38),
    "track":    (0, 208, 255),
    "drone":    (120, 235, 255),
    "rover":    (255, 176, 48),
    "panel":    (12, 14, 18),
    "edge":     (86, 96, 110),
    "text":     (226, 234, 242),
    "dim":      (128, 140, 154),
    "logged":   (86, 220, 120),
    "authored": (255, 190, 60),
}


def font(sz, mono=False):
    try:
        return ImageFont.truetype(FONT_M if mono else FONT, sz)
    except Exception:
        return ImageFont.load_default()


def hillshade(h):
    gy, gx = np.gradient(h.T.astype(float))
    az, alt = np.radians(315.0), np.radians(45.0)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    s = (np.sin(alt) * np.cos(slope)
         + np.cos(alt) * np.sin(slope) * np.cos(az - aspect))
    s = np.clip((s + 0.2) / 1.2, 0, 1)
    return s


def build_map_rgb(shade, known, haz):
    """Terrain shade tinted by map state. Unknown stays dark and opaque."""
    n = shade.shape[0]
    base = np.zeros((n, n, 3), float)
    g = shade[..., None]
    base[:] = np.array(COL["unknown"], float) * (0.55 + 0.9 * g)
    k = known.T
    if k.any():
        safe = np.array(COL["safe"], float) * (0.45 + 1.0 * g)
        base[k] = safe[k]
    hz = haz.T
    if hz.any():
        hazard = np.array(COL["hazard"], float) * (0.55 + 0.85 * g)
        base[hz] = hazard[hz]
    return np.clip(base, 0, 255).astype(np.uint8)[::-1]


def main():
    tl = json.load(open(os.path.join(HERE, "b5c_anim2.json")))
    frames = tl["frames"]
    z = np.load(os.path.join(HERE, "b5_hud_timeline.npz"))
    hm, cell, org = z["hmap"], float(z["cell"]), z["origin"]
    known_s, haz_s, cov_s, err_s = z["known"], z["haz"], z["cov"], z["err"]
    dtrue = z["drone_true"]
    rover_path = z["rover_path"]
    n = hm.shape[0]
    extent = n * cell
    shade = hillshade(hm)

    os.makedirs(FIN, exist_ok=True)
    f_big, f_med, f_sm, f_mono = font(30), font(17), font(13), font(14, True)
    cache = {}

    def w2m(x, y):
        """world metres -> minimap pixel (y flipped so north is up)"""
        u = (x - org[0]) / extent * MAP_PX
        v = MAP_PX - (y - org[1]) / extent * MAP_PX
        return u, v

    files = sorted(f for f in os.listdir(RAW) if f.endswith(".png"))
    if not files:
        print("no rendered frames yet in", RAW); return
    print(f"compositing {len(files)} frames -> {FIN}")

    for fi, fn in enumerate(files):
        idx = int(fn[1:5]) if fn[0] == "f" else fi
        fr = frames[min(idx, len(frames) - 1)]
        si = fr["scan_idx"]

        img = Image.open(os.path.join(RAW, fn)).convert("RGB")
        W, H = img.size
        d = ImageDraw.Draw(img, "RGBA")

        prov = fr["prov"]
        pc = COL["logged"] if prov == "LOGGED" else COL["authored"]
        d.rectangle([0, 0, W, 62], fill=(8, 10, 13, 165))
        d.text((MARGIN, 12), fr["label"], font=f_big, fill=COL["text"])
        tw = d.textlength(prov, font=f_sm)
        d.rectangle([MARGIN, 46, MARGIN + tw + 14, 62], fill=(*pc, 55))
        d.text((MARGIN + 7, 47), prov, font=f_sm, fill=pc)
        d.text((MARGIN + tw + 26, 47), fr["source"], font=f_sm, fill=COL["dim"])

        if si not in cache:
            if si < 0:
                k = np.zeros((n, n), bool); hz = np.zeros((n, n), bool)
            else:
                k, hz = known_s[si], haz_s[si]
            cache[si] = Image.fromarray(build_map_rgb(shade, k, hz))
        mp = cache[si].resize((MAP_PX, MAP_PX), Image.NEAREST)
        md = ImageDraw.Draw(mp, "RGBA")

        if si >= 1:
            pts = [w2m(p[0], p[1]) for p in dtrue[:si + 1]]
            if len(pts) > 1:
                md.line(pts, fill=(*COL["track"], 190), width=2)

        rspec = fr.get("rover")
        if rspec is not None:
            rx, ry = rspec["base"][0][3], rspec["base"][1][3]
            u, v = w2m(rx, ry)
            md.rectangle([u - 4, v - 4, u + 4, v + 4],
                         fill=COL["rover"], outline=(255, 255, 255), width=1)

        dspec = fr.get("drone")
        if dspec is not None:
            dx, dy = dspec["base"][0][3], dspec["base"][1][3]
            u, v = w2m(dx, dy)
            if fr["beat"] == "SCOUT":
                half = FOOTPRINT_M / 2 / extent * MAP_PX
                md.rectangle([u - half, v - half, u + half, v + half],
                             outline=(*COL["drone"], 235), width=2)
                md.rectangle([u - half, v - half, u + half, v + half],
                             fill=(*COL["drone"], 42))
            md.ellipse([u - 5, v - 5, u + 5, v + 5],
                       fill=COL["drone"], outline=(255, 255, 255), width=1)

        mx, my = W - MAP_PX - MARGIN, H - MAP_PX - MARGIN - 26
        d.rectangle([mx - 9, my - 30, mx + MAP_PX + 9, my + MAP_PX + 30],
                    fill=(*COL["panel"], 205), outline=COL["edge"], width=1)
        d.text((mx, my - 25), "PERCEIVED HAZARD MAP", font=f_sm, fill=COL["dim"])
        img.paste(mp, (mx, my))
        d.rectangle([mx, my, mx + MAP_PX, my + MAP_PX], outline=COL["edge"], width=1)

        ly = my + MAP_PX + 20
        for lbl, c in (("unknown", (46, 41, 37)), ("safe", (64, 168, 86)),
                       ("hazard", (214, 66, 56))):
            d.rectangle([mx, ly - 9, mx + 11, ly + 2], fill=c,
                        outline=(120, 130, 142))
            d.text((mx + 16, ly - 10), lbl, font=f_sm, fill=COL["dim"])
            mx += 16 + int(d.textlength(lbl, font=f_sm)) + 18

        cov = cov_s[si] if si >= 0 else 0.0
        err = err_s[si] if si >= 0 else 0.0
        bx, by = MARGIN, H - MARGIN - 74
        d.rectangle([bx - 10, by - 12, bx + 268, by + 74],
                    fill=(*COL["panel"], 195), outline=COL["edge"], width=1)
        d.text((bx, by - 6), "TERRAIN MAPPED", font=f_sm, fill=COL["dim"])
        d.rectangle([bx, by + 14, bx + 248, by + 30], outline=COL["edge"], width=1)
        d.rectangle([bx + 1, by + 15, bx + 1 + int(246 * cov), by + 29],
                    fill=COL["safe"])
        d.text((bx + 256 - d.textlength(f"{100*cov:.0f}%", font=f_med),
                by + 13), f"{100*cov:.0f}%", font=f_med, fill=COL["text"])
        d.text((bx, by + 40), f"GPS-DENIED NAV ERROR  {err:5.2f} m",
               font=f_mono, fill=COL["text"] if err < 1.2 else (255, 150, 120))
        d.text((bx, by + 57), f"t {idx/tl['fps']:5.1f} s   frame {idx:3d}/{len(frames)-1}",
               font=f_mono, fill=COL["dim"])

        img.save(os.path.join(FIN, fn))
        if fi % 60 == 0:
            print(f"  {fi}/{len(files)}", flush=True)

    print(f"done: {len(files)} frames in {FIN}")


if __name__ == "__main__":
    main()
