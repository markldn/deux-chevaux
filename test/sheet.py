# contact sheet: python3 test/sheet.py out.jpg a.png b.png ... (3 columns)
import sys
from PIL import Image
out, files = sys.argv[1], sys.argv[2:]
ims = [Image.open(f).convert('RGB') for f in files]
w, h = 640, 360
cols = 3 if len(ims) > 2 else len(ims)
rows = (len(ims) + cols - 1) // cols
S = Image.new('RGB', (w * cols, h * rows))
for i, im in enumerate(ims): S.paste(im.resize((w, h)), ((i % cols) * w, (i // cols) * h))
S.save(out, quality=88)
