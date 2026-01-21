# pixel_to_map.py 使用指南

简洁脚本，用于在 ROS 栅格地图（PGM + YAML）与图像像素坐标之间互相转换，并可选检测像素的占用状态。

- 文件：`pixel_to_map.py`
- 位置：本目录
- 依赖：Python 3（无需第三方库）

## 功能概览

- 像素 → 地图：输入 PGM 像素 `(u, v)`，输出地图坐标 `(x, y)`。
- 地图 → 像素：输入地图坐标 `(x, y)`，输出像素坐标 `(u, v)`（浮点，支持取整）。
- 可选占用查询：读取 PGM 像素值，并依据 YAML 的 `negate / occupied_thresh / free_thresh` 给出“占用/可行/未知”。
- 自动处理分辨率 `resolution`、原点与旋转 `origin = [ox, oy, yaw]`、图像坐标系上下翻转。

## 坐标系约定

- 图像像素系：原点在左上，`u` 向右为正，`v` 向下为正，0-based。
- ROS 地图：`origin = [ox, oy, yaw]` 表示“栅格左下角”在地图系中的位姿；需要对图像垂直方向做翻转并考虑 `yaw` 旋转。

## 基本用法

以下示例中的 YAML 路径支持直接写文件名（脚本会在 `map/simulation` 与 `map/reality` 中自动搜索），或填写完整路径。

### 1) 像素 → 地图

```bash
python3 pixel_to_map.py \
  --yaml rmul_2024.yaml \
  --pixel 100 200
```

可选：

- `--one-based`：若 `(u, v)` 来自 1-based 读数（某些图像工具如此）。
- `--corner`：按像素左上角而非中心点进行换算（默认使用中心）。
- `--check-occupancy`：输出该像素的占用/可行/未知（PGM 为 P5 时可随机访问）。

### 2) 地图 → 像素

```bash
python3 pixel_to_map.py \
  --yaml rmul_2024.yaml \
  --map-to-pixel 1.23 4.56
```

可选：

- `--rounding {round|floor|ceil|none}`：控制像素输出取整方式（默认 `round`）。
- `--corner`：对应像素左上角/中心的选取，影响 0.5 偏移。
- `--check-occupancy`：对取整后的像素做占用查询。

### 3) 一键查询“map 原点”的像素位置

```bash
python3 pixel_to_map.py \
  --yaml rmul_2024.yaml \
  --map-origin
```

## 参数说明

- `--yaml <path>`：地图 YAML 路径（需包含 `image`、`resolution`、`origin`）。
- `--pixel U V`：像素坐标（0-based，左上为原点）。
- `--one-based`：像素输入按 1-based 解释。
- `--corner`：用像素左上角（不使用 0.5 偏移）；默认使用像素中心。
- `--check-occupancy`：基于 PGM 像素值与 YAML 阈值给出占用分类。
- `--map-to-pixel X Y`：将地图坐标转为像素坐标（返回浮点并可取整）。
- `--map-origin`：等价于 `--map-to-pixel 0 0`。
- `--rounding`：`round|floor|ceil|none`，控制地图→像素结果的取整打印。

## 注意事项

- YAML 必需字段：
  - `image`: PGM 文件路径（相对路径相对于 YAML 所在目录）。
  - `resolution`: 米/像素。
  - `origin`: `[ox, oy, yaw]`（若 YAML 给出 2 元素将自动补 `yaw=0`）。
- PGM 建议使用 P5（二进制）。ASCII P2 无法高效随机访问，`--check-occupancy` 将显示 N/A。
- 越界像素会报错或提示 out-of-bounds。

## 示例输出参考

```text
map (x,y): (0.000000, 0.000000) -> pixel (u,v): (74.500, 118.700)
pixel (rounded): (74, 119) within image bounds 272x210
```

## 公式（像素中心）

给定 `resolution = r`、图像高 `H`、`origin = [ox, oy, θ]`：

- 像素 → 地图：
  - `xl = (u + 0.5)*r`, `yl = (H - v - 0.5)*r`
  - `x = ox + xl*cosθ - yl*sinθ`
  - `y = oy + xl*sinθ + yl*cosθ`
- 地图 → 像素：
  - `dx = x-ox`, `dy = y-oy`
  - `xl =  cosθ*dx + sinθ*dy`
  - `yl = -sinθ*dx + cosθ*dy`
  - `u = xl/r - 0.5`, `v = H - yl/r - 0.5`

<!-- 如需将像素左上角作为代表点，去掉上述式子的 `0.5` 偏移即可（或在命令中使用 `--corner`）。 -->