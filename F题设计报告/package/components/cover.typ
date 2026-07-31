#import "@preview/cuti:0.3.0": show-cn-fakebold
#show: show-cn-fakebold
#let generate-cover(
  year: str,
  problem-id: str,
  problem-name: str,
  team-id: str,
  school: str,
  team-members: array,
  teachers: array,
  show-teachers: true,
  show-information: true,
) = {
  let info-row(label, content: none, width: 220pt) = {
    grid(
      columns: (90pt, width),
      gutter: 8pt,
      align(left + horizon)[
        #text(font: "SimSun", size: 14pt)[#label]
      ],
      [
        #stack(
          spacing: 2pt,
          if content != none {
            align(center)[#text(font: "SimSun", size: 14pt)[#content]]
          } else {
            v(10pt)
          },
          line(length: width),
        )
      ],
    )
  }

  if show-information {
    align(center)[
      #v(1.2cm)
      #text(font: "SimSun", size: 22pt, weight: "bold")[#year 年全国大学生电子设计竞赛]
      #v(0.5cm)
      #text(font: "SimSun", size: 22pt, weight: "bold")[#problem-id 题：#problem-name]
      #v(0.8cm)
      #box(height: 3.0cm)[
        #align(center)[
          #image("../assets/logo.png", height: 2.4cm)
        ]
      ]
      #v(1.0cm)
      #text(font: "SimSun", size: 20pt, weight: "bold")[设 计 报 告]
      #v(2.0cm)
      #align(left)[
        #box(width: 360pt)[
          #info-row("选    题：", content: problem-id + " 题  " + problem-name)
          #v(10pt)
          #info-row("赛区编号：", content: team-id)
          #v(10pt)
          #info-row("参赛学校：", content: school)
          #for (idx, member) in team-members.enumerate() [
            #v(10pt)
            #if idx == 0 [
              #info-row("参赛队员：", content: member)
            ] else [
              #info-row("        ", content: member)
            ]
          ]
          #if show-teachers [
            #for (idx, member) in teachers.enumerate() [
              #v(10pt)
              #if idx == 0 [
                #info-row("指导教师：", content: member)
              ] else [
                #info-row("        ", content: member)
              ]
            ]
          ]
        ]
      ]
      #v(4.0cm)
      #text(font: "SimSun", size: 14pt)[#datetime.today().display("[year]年[month]月[day]日")]
    ]
  } else {
    set page(margin: (top: 0pt, bottom: 0pt, left: 0pt, right: 0pt))
    let d = datetime.today()
    let month = int(d.display("[month]"))
    let day = int(d.display("[day]"))

    place(top + center, dy: 200pt)[
      #set par(first-line-indent: 0em, spacing: 0pt)
      #text(size: 18pt)[
        #text(font: "Times New Roman", weight: "bold")[#year]#h(0.2em)#text(font: "SimSun", weight: "bold")[年全国大学生电子设计竞赛]
      ]
    ]

    place(top + center, dy: 262pt)[
      #set par(first-line-indent: 0em, spacing: 0pt)
      #text(size: 18pt, font: "SimSun", weight: "bold")[
        #problem-name（#text(font: "Times New Roman", weight: "bold")[#problem-id]#h(0.2em)题）
      ]
    ]

    place(top + center, dy: 356pt)[
      #image("../assets/logo.png", height: 4.45cm)
    ]

    place(top + center, dy: 589pt)[
      #set par(first-line-indent: 0em, spacing: 0pt)
      #text(size: 18pt)[
        #text(font: "Times New Roman", weight: "bold")[#d.display("[year]")]#h(0.2em)#text(font: "SimSun", weight: "bold")[年]#h(0.2em)
        #text(font: "Times New Roman", weight: "bold")[#month]#h(0.2em)#text(font: "SimSun", weight: "bold")[月]#h(0.2em)
        #text(font: "Times New Roman", weight: "bold")[#day]#h(0.2em)#text(font: "SimSun", weight: "bold")[日]
      ]
    ]
  }
  pagebreak()
}
