# 安装资源缓存

Git 仓库仅保存下载清单；`bash setup.sh pico` 从官方地址下载 PC Service、APK 和 ADB 到 `.local/pico4/downloads/`。

PC Service/APK 版本、地址及固定 SHA-256 在 `scripts/pico4/assets.json`。
ADB 使用 Google 官方 Platform Tools 下载地址。也可以将本地完整包中的安装文件拷贝到本目录，再运行安装。
安装文件、运行环境与下载缓存都由 `.gitignore` 排除。
