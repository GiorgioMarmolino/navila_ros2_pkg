import glob, os
from PIL import Image
import pillow_heif
pillow_heif.register_heif_opener()

src = "/home/ros_ws/NaVILA_test"
dst = "/home/ros_ws/NaVILA_test_jpg"
os.makedirs(dst, exist_ok=True)
for p in sorted(glob.glob(os.path.join(src, "*.heic"))):
    name = os.path.splitext(os.path.basename(p))[0] + ".jpg"
    Image.open(p).convert("RGB").save(os.path.join(dst, name), "JPEG", quality=92)
    print("ok", name)
