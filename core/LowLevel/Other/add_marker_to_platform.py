# core/LowLevel/Other/add_marker_to_platform.py
from PIL import Image

N = 200
M = N * 2

base = Image.open("Assets/skydio_x2/assets/donut_texture.png")
marker = Image.open("Assets/skydio_x2/assets/ArUco_marker_4x4_1000-0.png").resize((M, M))

base.paste(marker, (462 - N, 462 - N))

base.save("donut_with_marker.png")
