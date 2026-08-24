"""B1 -- whole-map overview render + a GPU-independent elevation map."""
import numpy as np
from PIL import Image, ImageDraw
from skyscout_sim import SkyScoutSim
from render import FollowRenderer
from mars_world import (build_heightmap, EXTENT, CRATER_CENTRE, CRATER_RADIUS)

h, feats, rocks = build_heightmap()
img = h.T[::-1]
lo, hi = img.min(), img.max()
t = (img - lo) / (hi - lo)
ramp = np.array([[40,45,70],[90,70,60],[150,95,60],[200,140,85],[240,205,170],[255,250,245]], float)
i = t*(len(ramp)-1); i0 = np.floor(i).astype(int); i1 = np.minimum(i0+1, len(ramp)-1)
rgb = ramp[i0]*(1-(i-i0)[...,None]) + ramp[i1]*(i-i0)[...,None]
gy, gx = np.gradient(img, EXTENT/(img.shape[0]-1))
rgb = np.clip(rgb*np.clip((-gx*0.7-gy*0.7+1.2)/2.0, 0.45, 1.35)[...,None], 0, 255).astype(np.uint8)
im = Image.fromarray(rgb).resize((760,760), Image.NEAREST); d = ImageDraw.Draw(im)
px = lambda x,y: ((x/EXTENT+0.5)*760, (0.5-y/EXTENT)*760)
cx,cy = px(*CRATER_CENTRE); r = CRATER_RADIUS/EXTENT*760
d.ellipse([cx-r,cy-r,cx+r,cy+r], outline=(0,255,255), width=3); d.text((cx-22,cy-6),'CRATER',fill=(0,255,255))
for rx,ry,rr,rh in rocks:
    a,b = px(rx,ry); q = max(4, rr/EXTENT*760)
    d.ellipse([a-q,b-q,a+q,b+q], outline=(255,80,80), width=2)
x0,y0 = px(0,0); x1,y1 = px(12,0)
d.line([x0,y0,x1,y1], fill=(0,255,0), width=4); d.text((x1+6,y1-6),'B1 flight path',fill=(0,255,0))
d.text((8,8), f'Mars terrain {EXTENT:.0f}x{EXTENT:.0f} m  elev {lo:.2f}..{hi:.2f} m  seed 20260808', fill=(255,255,255))
d.text((8,26),'cyan=crater  red=boulders  bright=high', fill=(255,255,255))
im.save('b1_elevation_map.png'); print('saved b1_elevation_map.png')

sim = SkyScoutSim(start_pos=(0,0,10.27), logging_level='error', camera_res=(960,600))
rnd = FollowRenderer(sim, res=(960,600))
print(f'warm-up (throwaway): {rnd.warm_up():.1f} s')
for eye,tgt,name in ((( 55, 55, 42),(0,0,0),'b1_overview_iso.png'),
                     (( 0.1, 0.1, 78),(0,0,0),'b1_overview_top.png')):
    sim.camera.set_pose(pos=eye, lookat=tgt)
    a = np.asarray(sim.camera.render(rgb=True)[0], dtype=np.uint8)[:,:,:3]
    Image.fromarray(a).save(name); print('saved', name)
