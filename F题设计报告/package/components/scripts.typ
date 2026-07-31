#import "@preview/ctheorems:1.1.3": *
#let bib(bibliography-file) = {
  show bibliography: set text(10.5pt)
  // 手动渲染"参考文献"标题，禁用 ctheorems 自带标题（避免计入 heading 序列）
  block(
    width: 100%,
    above: 26pt,
    below: 18pt,
    breakable: false,
  )[
    #set align(center)
    #set text(font: ("Times New Roman", "SimHei"), size: 16pt, weight: "bold")
    #set par(first-line-indent: 0em, spacing: 0pt)
    参考文献
  ]
  set bibliography(title: none, style: "gb-7714-2015-numeric")
  bibliography-file
  v(12pt)
}

#let appendix-num = counter("appendix")

#let appendix(title, body) = {
  appendix-num.step()
  // 附录标题：使用居中加粗的大字号，手动编号，不计入正文 heading 序列
  block(
    width: 100%,
    above: 28pt,
    below: 18pt,
    breakable: false,
  )[
    #set align(center)
    #set text(font: ("Times New Roman", "SimHei"), size: 16pt, weight: "bold")
    #set par(first-line-indent: 0em, spacing: 0pt)
    附录 #context appendix-num.display()：#title
  ]
  set table(
    stroke: (x, y) => (bottom: if y == 0 { 0.6pt } else { 0pt }),
    inset: (x: 8pt, y: 6pt),
    align: (x, y) => (if y == 0 { center } else { left }) + horizon,
    fill: none,
  )
  show table: it => block(stroke: (top: 0.9pt, bottom: 0.9pt), inset: 0pt, it)
  show table.cell.where(y: 0): set text(weight: "bold")
  show table: set text(font: ("Times New Roman", "SimSun"), size: 10.5pt)
  body
}

#let split_table(
  symbols,
  caption: "符号约定表",
  splits: (), // 自定义切分点
  add_pagebreak: true, // 是否在表格之间添加分页符
) = {
  // 如果没有指定切分点，则将整个表格作为一个部分
  let split_points = if splits.len() == 0 { (symbols.len(),) } else { splits }

  // 验证切分点是否有效
  assert(
    split_points.all(p => p > 0 and p <= symbols.len()),
    message: "切分点必须在1到符号数量之间",
  )

  let parts = split_points.len()
  let start_idx = 0

  for (i, end_idx) in split_points.enumerate() {
    // 获取当前部分的符号
    let current_symbols = symbols.slice(start_idx, end_idx)

    // 构建当前部分的表格
    figure(
      table(
        columns: 2,
        align: (left, left),
        stroke: none,
        inset: 5pt,
        table.hline(),
        table.header(
          [*符号*],
          [*约定*],
        ),
        table.hline(stroke: 0.4pt),
        ..current_symbols.flatten(),
        table.hline()
      ),
      caption: if parts > 1 {
        caption + " (续表 " + str(i + 1) + "/" + str(parts) + ")"
      } else {
        caption
      },
    )

    // 更新下一部分的起始索引
    start_idx = end_idx

    // 如果不是最后一个部分且需要分页，添加分页符
    if i < parts - 1 and add_pagebreak {
      pagebreak()
    }
  }
}

// 使用示例

// 示例1: 不切分表格
// #split_table(all_symbols)

// 示例2: 将表格分成3部分，在第10行和第20行处切分
// #split_table(all_symbols, splits: (14, all_symbols.len()))

// 示例3: 按照自然分组切分，不添加分页符
// #split_table(
//   all_symbols,
//   splits: (12, 20, all_symbols.len()),
//   add_pagebreak: false
// )
