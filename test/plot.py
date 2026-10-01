# side + top view of a lattice dump: python3 test/plot.py dump.json out.png
import sys, json
from PIL import Image, ImageDraw
d = json.load(open(sys.argv[1])); S = 150
im = Image.new('RGB', (1400, 700), (20, 20, 24)); dr = ImageDraw.Draw(im)
cols = [(200,200,200),(120,120,255),(80,80,255),(255,120,40),(120,255,255),(90,90,90),(255,255,0),(255,0,255),(0,255,0),(200,160,100),(255,0,0),(255,80,80),(0,255,120)]
for c,x,y,z,rx,ry,rz in d:
  for (u,v,ox,oy) in [(z, -y, 700, 330), (z, x, 700, 560)]:
    px, py = ox + u*S - 0*S, oy + v*S
    dr.ellipse([px-2,py-2,px+2,py+2], fill=cols[c])
  for (u,v,ox,oy) in [(rz, -ry, 700, 330)]:
    px, py = ox + u*S, oy + v*S; dr.point((px,py), fill=(70,70,70))
dr.line([(700,0),(700,700)], fill=(255,60,60))
im.save(sys.argv[2])
