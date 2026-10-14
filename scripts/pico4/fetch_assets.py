"""Fetch pinned upstream releases; SHA256 is from the GitHub releases API."""
import hashlib
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOWNLOADS = ROOT / ".local/pico4/downloads"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def fetch_large(asset, destination):
    """Short requests tolerate the local network's intermittent long-transfer stalls."""
    chunk_size = 2 * 1024 * 1024
    directory = DOWNLOADS / (asset["name"] + ".chunks")
    directory.mkdir(exist_ok=True)
    ranges = [(start, min(start + chunk_size, asset["size"]) - 1)
              for start in range(0, asset["size"], chunk_size)]

    def fetch_chunk(bounds):
        start, end = bounds
        chunk = directory / str(start)
        if chunk.exists() and chunk.stat().st_size == end - start + 1:
            return chunk
        part = chunk.with_suffix(".part")
        subprocess.run([
            "curl", "-fLsS", "--http1.1", "--connect-timeout", "12", "--max-time", "45",
            "--retry", "3", "--retry-all-errors", "-r", f"{start}-{end}",
            asset["url"] + f"?pico4_setup_range={start}", "-o", str(part),
        ], check=True)
        if part.stat().st_size != end - start + 1:
            raise RuntimeError(f"服务器未返回预期的文件分块：{start}-{end}")
        part.replace(chunk)
        return chunk

    with ThreadPoolExecutor(max_workers=4) as pool:
        chunks = list(pool.map(fetch_chunk, ranges))
    with destination.open("wb") as stream:
        for chunk in chunks:
            stream.write(chunk.read_bytes())


def main():
    DOWNLOADS.mkdir(parents=True, exist_ok=True)
    assets = json.loads(Path(__file__).with_name("assets.json").read_text())
    for asset in assets:
        path = DOWNLOADS / asset["name"]
        if not path.exists() or digest(path) != asset["sha256"]:
            part = path.with_suffix(path.suffix + ".part")
            print("Downloading", asset["name"], flush=True)
            if asset["size"] > 64 * 1024 * 1024:
                fetch_large(asset, part)
            else:
                subprocess.run([
                    "curl", "-fLsS", "--connect-timeout", "20", "--max-time", "300",
                    "--retry", "3", "--retry-all-errors", "--speed-limit", "1024",
                    "--speed-time", "30", "-C", "-", asset["url"], "-o", str(part),
                ], check=True)
            if digest(part) != asset["sha256"]:
                raise RuntimeError(f"SHA256 校验失败：{part}；请移走该 .part 文件后重试。")
            part.replace(path)
        print("SHA256 OK", path.name, flush=True)
    (DOWNLOADS / "sources.json").write_text(json.dumps(assets, indent=2) + "\n")


if __name__ == "__main__":
    main()
