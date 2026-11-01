# 2026 F题设计报告｜李萨如图形显示控制装置

这里保存 `ti-last` 的 UVC 摄像头方案报告、已有 PDF、框图和原理图。完整工程从[仓库首页](../README.md)进入，方案与验证边界见[方案说明](../docs/SOLUTION.md)和[归档说明](../docs/ARCHIVE.md)。

## 文件入口

- [main.typ](main.typ)：报告正文，2026 年 F《李萨如图形显示控制装置》。
- [电赛F题_设计报告.pdf](电赛F题_设计报告.pdf)：已有导出件，本次未重新生成，不保证它与当前 Typst 正文逐字同步。
- [refs.bib](refs.bib)：参考文献；`figures/`：框图与插图；`package/`：Typst 排版组件。
- [MSPM0G3507 主控原理图](SCH_MSPM0G3507番茄派_v1.1_2026-05-09.pdf)、[OPA695 放大器](SCH_OPA695同相放大器_v2_2026-06-10.pdf)、[鉴频鉴相器](SCH_鉴频鉴相器_2026-07-15.pdf)。

## 阅读时的版本区别

本文以 UVC + Python/OpenCV 为要求 5 的后续方案，记录 CCD 路线的取舍。备赛库中的 [F题设计报告](https://github.com/liusuan110/2026-Ti/tree/main/F%E9%A2%98%E8%AE%BE%E8%AE%A1%E6%8A%A5%E5%91%8A) 是早期 CCD 草稿，不是相同目录的镜像。

报告原件保留了阶段性测试汇总及最终结论，两处对要求 5 的达标表述并不完全一致。应以具体测试条件与日志评估，不将报告结论扩展成独立验收证明。

## 重新导出

上游模板说明基于 Typst 0.13.1。可从本目录尝试编译到新文件，避免覆盖既有 PDF：

```bash
typst compile main.typ 电赛F题_设计报告_重新编译.pdf
```

本次未试编，尚无经本次验证的固定 Typst / 字体环境。导出后检查中文字体、插图、引用和分页，并记录版本。

## 模板来源与致谢

[整理前的模板说明](https://github.com/liusuan110/ti-last/blob/0cfb725fba361fb958c790d13a437626d5d84539/F%E9%A2%98%E8%AE%BE%E8%AE%A1%E6%8A%A5%E5%91%8A/readme.md) 包含上游作者 April 的模板介绍和 2025 年参赛经历；这些经历与获奖叙述不属于本项目。

模板原说明致谢 [SandOcean-ovo 的电赛 LaTeX 模板](https://github.com/SandOcean-ovo/Template-for-Electrical-Competition-Report) 和 [a-kkiri 的 CUMCM Typst 模板](https://github.com/a-kkiri/CUMCM-typst-template)。保留本目录 [LICENSE](LICENSE)，使用模板时遵守其声明。
