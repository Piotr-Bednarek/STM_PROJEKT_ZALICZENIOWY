from pathlib import Path
from PIL import Image

docs_dir = Path(__file__).parent

for webp in docs_dir.rglob("*.webp"):
    png = webp.with_suffix(".png")
    Image.open(webp).save(png, "PNG")
    print(f"{webp.relative_to(docs_dir)} -> {png.name}")
