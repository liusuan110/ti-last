// 设置页面格式
#import "@preview/cuti:0.3.0": show-cn-fakebold
#import "@preview/ctheorems:1.1.3": *

#import "components/cover.typ": generate-cover
#import "components/abstract.typ": abstract-page
#show: show-cn-fakebold
#show: thmrules
#let NUEDC-report(
  year: str,
  problem-id: str,
  problem-name: str,
  team-id: str,
  school: str,
  team-members: array,
  teachers: array,
  abstract: content,
  keywords: array,
  show-teachers: true,
  show-cover: true,
  show-information: true,
  show-outline: false,
  body,
) = {
  // 初始化相关页面、文本和段落样式
  set page(
    paper: "a4",
    margin: (top: 3cm, bottom: 3cm, left: 3.2cm, right: 3.2cm),
    footer: context {
      if counter(page).get().first() > 0 [
        #align(right)[
          #text[#counter(page).get().first()]
        ]
      ]
    },
  )
  counter(page).update(0)
  // 设置字体（参考文档全文宋体）
  set text(
    font: "SimSun",
    size: 12pt,
    weight: "regular",
    lang: "zh",
  )
  // 粗体强调：保留行内属性，仅加细微描边模拟粗体
  show strong: it => {
    show regex("[\p{hani}\s]+"): set text(stroke: 0.02857em)
    it
  }
  // 正文首行缩进和行距设置
  // Typst 中 SimSun 12pt 自然行高 = 8.2pt（实测）
  // 固定行距 22pt = 8.2 + leading  →  leading = 13.8pt
  // spacing 取同值使段间距 = 行间距，标题即正文改字号，无额外间距
  set par(
    first-line-indent: (amount: 0.85cm, all: true),
    leading: 13.8pt,
    spacing: 13.8pt,
    linebreaks: "optimized",
    justify: true,
  )
  // 块级公式与正文的上下间距（与行距一致）
  show math.equation.where(block: true): set block(above: 13.8pt, below: 13.8pt)
  // 列表整体与正文、列表项之间留出稳定间距
  show list: set block(above: 13.8pt, below: 13.8pt)
  show enum: set block(above: 13.8pt, below: 13.8pt)
  set list(spacing: 13.8pt, indent: 1em, body-indent: 0.6em)
  set enum(spacing: 13.8pt, indent: 1em, body-indent: 0.8em)
  // 图表标题样式（参考文档 ~10pt）
  show figure.caption: it => {
    text(font: "SimSun", size: 10pt)[#it.body]
  }
  // 表格：题注在上，表格本体在下
  show figure.where(kind: table): it => [
    #v(0pt)
    #align(center)[
      #stack(
        spacing: 4pt,
        {
          set text(font: "SimSun", size: 10pt)
          [表#counter(figure.where(kind: table)).display("1")#h(0.5em)#it.caption.body]
        },
        it.body,
      )
    ]
    #v(0pt)
  ]
  // 图：图名在图下，居中
  show figure.where(kind: image): it => [
    #v(0pt)
    #align(center)[#it.body]
    #v(2pt)
    #align(center)[
      #set text(font: "SimSun", size: 10pt)
      图#counter(figure.where(kind: image)).display("1")#h(0.5em)#it.caption.body
    ]
    #v(0pt)
  ]
  show figure.where(kind: raw): it => {
    set block(width: 100%, breakable: true)
    it
  }
  // 标题样式：标题就是正文的基础上改了字号和粗体，
  // 不加额外 block 上下间距，行距与正文一致（固定 22pt）
  set heading(numbering: "1.")
  show heading.where(level: 1): it => [
    #counter(heading.where(level: 2)).update(1)
    #counter(heading.where(level: 3)).update(1)
    #let num_1 = counter(heading.where(level: 1)).get().at(0)

    #par(first-line-indent: (amount: 0em, all: true), leading: 12.4pt, spacing: 13.8pt)[
      #text(font: "SimSun", size: 14pt, weight: "bold", stroke: 0.05em)[
        #if num_1 > 0 {
          numbering("1.", num_1)
          h(0.5em)
        }
        #it.body
      ]
    ]
  ]
  // 二级标题样式
  show heading.where(level: 2): it => [
    #counter(heading.where(level: 2)).step()
    #counter(heading.where(level: 3)).update(1)
    #let num_1 = counter(heading.where(level: 1)).get().at(0)
    #let num_2 = counter(heading.where(level: 2)).get().at(0)

    #par(first-line-indent: (amount: 0em, all: true), leading: 13.8pt, spacing: 13.8pt)[
      #text(font: "SimSun", size: 12pt, weight: "bold", stroke: 0.04em)[
        #numbering("1.1.", num_1, num_2)
        #h(0.5em)
        #it.body
      ]
    ]
  ]
  // 三级标题样式
  show heading.where(level: 3): it => [
    #counter(heading.where(level: 3)).step()
    #let num_1 = counter(heading.where(level: 1)).get().at(0)
    #let num_2 = counter(heading.where(level: 2)).get().at(0)
    #let num_3 = counter(heading.where(level: 3)).get().at(0)

    #par(first-line-indent: (amount: 0em, all: true), leading: 13.8pt, spacing: 13.8pt)[
      #text(font: "SimSun", size: 12pt, weight: "bold", stroke: 0.04em)[
        #numbering("1.1.1.", num_1, num_2 - 1, num_3)
        #h(0.5em)
        #it.body
      ]
    ]
  ]
  // 代码标题样式
  show raw.where(block: true): block => [
    #pad(left: 2.5em)[
      #block
    ]
  ]

  set table(
    stroke: 0.4pt,
    inset: (x: 8pt, y: 6pt),
    align: (x, y) => (if y == 0 { center } else { left }) + horizon,
    fill: none,
  )
  show table.cell.where(y: 0): set text(weight: "bold")
  show table: set text(font: "SimSun", size: 10pt)

  // 正文声明
  if show-cover {
    // 生成封面
    generate-cover(
      year: year,
      problem-id: problem-id,
      problem-name: problem-name,
      team-id: team-id,
      school: school,
      team-members: team-members,
      teachers: teachers,
      show-teachers: show-teachers,
      show-information: show-information,
    )
  }
  counter(page).update(1)
  abstract-page(
    body: abstract,
    keywords: keywords,
    title: if show-cover { none } else { problem-name + "设计报告" },
  )
  if show-outline {
    outline()
    pagebreak()
  }
  body
}
