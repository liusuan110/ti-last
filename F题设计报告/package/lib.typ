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
    margin: (top: 3cm, bottom: 2.5cm, left: 2.5cm, right: 2.5cm),
    footer: context {
      if counter(page).get().first() > 0 [
        #align(right)[
          #text[#counter(page).get().first()]
        ]
      ]
    },
  )
  counter(page).update(0)
  // 设置字体
  set text(
    font: ("Times New Roman", "SimSun"),
    size: 12pt,
    weight: "regular",
    lang: "zh",
  )
  // 粗体
  show strong: it => {
    show regex("[\p{hani}\s]+"): set text(stroke: 0.02857em)
    it
  }
  // 正文首行缩进和行距设置（行距固定 22pt，符合赛题排版要求）
  set par(
    first-line-indent: (amount: 2em, all: true),
    leading: 22pt,
    spacing: 22pt,
    linebreaks: "optimized",
    justify: true,
  )
  // 图表标题样式
  show figure.caption: it => {
    text(font: ("Times New Roman", "SimSun"), size: 10.5pt)[#it.body]
  }
  // 表格：三线表风格（表名在表上，居中）
  show figure.where(kind: table): it => [
    #v(10pt, weak: true)
    #align(center)[
      #it.caption
      #v(4pt)
      #it.body
    ]
    #v(10pt, weak: true)
  ]
  // 图：图名在图下，居中
  show figure.where(kind: image): it => [
    #v(10pt, weak: true)
    #align(center)[#it]
    #v(10pt, weak: true)
  ]
  show figure.where(kind: raw): it => [
    #v(6pt, weak: true)
    #it
    #v(6pt, weak: true)
  ]
  show figure.where(kind: raw): it => {
    set block(width: 100%, breakable: true)
    it
  }
  // 设置heading样式
  set heading(outlined: true, numbering: "1.1.1")
  // 一级标题样式
  show heading.where(level: 1): it => [
    // #counter(heading.where(level: 1)).step()
    #counter(heading.where(level: 2)).update(1)
    #counter(heading.where(level: 3)).update(1)
    #let num_1 = counter(heading.where(level: 1)).get().at(0)

    #v(18pt, weak: true)
    #par(first-line-indent: (amount: 0em, all: true), leading: 22pt, spacing: 0pt)[
      #align(center)[
        #text(font: ("Times New Roman", "SimHei"), size: 16pt, weight: "bold")[
          #if num_1 > 0 {
            numbering("1", num_1)
            h(0.5em)
          }
          #it.body
        ]
      ]
    ]
    #v(12pt, weak: true)
  ]
  // 二级标题样式
  show heading.where(level: 2): it => [
    // 计算二级标题的编号
    #counter(heading.where(level: 2)).step()
    #counter(heading.where(level: 3)).update(1)
    #let num_1 = counter(heading.where(level: 1)).get().at(0)
    #let num_2 = counter(heading.where(level: 2)).get().at(0)

    // 二级标题样式具体设置
    #v(14pt, weak: true)
    #par(first-line-indent: (amount: 0em, all: true), leading: 22pt, spacing: 0pt)[
      #text(font: ("Times New Roman", "SimHei"), size: 14pt, weight: "bold")[
        #numbering("1.1", num_1, num_2)
        #h(0.5em)
        #it.body
      ]
    ]
    #v(8pt, weak: true)
  ]
  // 三级标题样式
  show heading.where(level: 3): it => [
    // 计算三级标题的编号
    #counter(heading.where(level: 3)).step()
    #let num_1 = counter(heading.where(level: 1)).get().at(0)
    #let num_2 = counter(heading.where(level: 2)).get().at(0)
    #let num_3 = counter(heading.where(level: 3)).get().at(0)

    // 三级标题样式具体设置
    #v(10pt, weak: true)
    #par(first-line-indent: (amount: 0em, all: true), leading: 22pt, spacing: 0pt)[
      #text(font: ("Times New Roman", "SimHei"), size: 12pt, weight: "bold")[
        #numbering("1.1.1", num_1, num_2 - 1, num_3)
        #h(0.5em)
        #it.body
      ]
    ]
    #v(6pt, weak: true)
  ]
  // 代码标题样式
  show raw.where(block: true): block => [
    #pad(left: 2.5em)[
      #block
    ]
  ]

  // 三线表全局样式：顶线 + 表头下分隔线 + 底线；表内文字五号
  set table(
    stroke: (x, y) => (bottom: if y == 0 { 0.6pt } else { 0pt }),
    inset: (x: 8pt, y: 6pt),
    align: (x, y) => (if y == 0 { center } else { left }) + horizon,
    fill: none,
  )
  show table: it => block(stroke: (top: 0.9pt, bottom: 0.9pt), inset: 0pt, it)
  show table.cell.where(y: 0): set text(weight: "bold")
  show table: set text(font: ("Times New Roman", "SimSun"), size: 10.5pt)

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
  )
  if show-outline {
    outline()
    pagebreak()
  }
  body
}
