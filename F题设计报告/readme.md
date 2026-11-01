# F题设计报告：阅读与修改

这份报告围绕 2026 F题《李萨如图形显示控制装置》展开，介绍 MSPM0G3507、AD9959、电锁相与 UVC 视觉控制怎样组合成整套装置。

直接阅读可打开[已有 PDF](电赛F题_设计报告.pdf)。想修改内容，从 [main.typ](main.typ) 开始；它依次包含题目分析、系统方案、理论计算、电路与程序设计、测试结果。

## 阅读时可以抓住三条线

第一条是控制量：频率比、相位和幅度怎样对应到对角线、圆和“∞”。

第二条是反馈来源：要求 1～4 使用电输入与 PFD，要求 5 则从屏幕图像中取得频率和几何信息。

第三条是软硬件分工：MCU 负责采样、DDS 和交互，Python/OpenCV 负责轨迹分析与形状控制。[方案文章](../docs/SOLUTION.md)对这部分作了更适合对照源码的展开，[演进与测试记录](../docs/ARCHIVE.md)保存各阶段实验条件。

## 硬件资料与文件组织

| 文件 | 内容 |
| --- | --- |
| [main.typ](main.typ) | 报告正文 |
| [refs.bib](refs.bib) | 参考文献 |
| [figures](figures/) | 系统框图、软件流程和电路插图 |
| [package](package/) | Typst 排版组件 |
| [主控原理图](SCH_MSPM0G3507番茄派_v1.1_2026-05-09.pdf) | MSPM0G3507 最小系统 |
| [鉴频鉴相器](SCH_鉴频鉴相器_2026-07-15.pdf) | PFD 与反馈电路 |
| [OPA695 放大器](SCH_OPA695同相放大器_v2_2026-06-10.pdf) | 输出放大链路 |

## 重新导出

模板原说明基于 Typst 0.13.1。在本目录编译为一个新 PDF，便于与已有导出件对照：

```bash
typst compile main.typ 电赛F题_设计报告_重新编译.pdf
```

导出后看一遍中文字体、图片、参考文献和分页。以后修改正文或图表时，同时更新对应 PDF，并记录使用的 Typst 与字体环境。

## 模板来源

排版组件及 [LICENSE](LICENSE) 沿用原模板。[原模板说明](https://github.com/liusuan110/ti-last/blob/0cfb725fba361fb958c790d13a437626d5d84539/F%E9%A2%98%E8%AE%BE%E8%AE%A1%E6%8A%A5%E5%91%8A/readme.md)保留作者介绍和参赛随笔。

模板致谢 [SandOcean-ovo 电赛 LaTeX 模板](https://github.com/SandOcean-ovo/Template-for-Electrical-Competition-Report)与 [a-kkiri CUMCM Typst 模板](https://github.com/a-kkiri/CUMCM-typst-template)。
