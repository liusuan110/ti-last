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

  align(center)[
    #v(1.2cm)
    #text(font: "SimSun", size: 22pt, weight: "bold")[#year 年全国大学生电子设计竞赛]
    #v(0.5cm)
    #text(font: "SimSun", size: 22pt, weight: "bold")[#problem-id 题：#problem-name]
    #v(1.0cm)
    #text(font: "SimSun", size: 20pt, weight: "bold")[设 计 报 告]
    #v(2.0cm)
    #align(left)[
      #box(width: 360pt)[
        #info-row("编    号：")
        #v(10pt)
        #info-row("题    目：", content: problem-id + " 题  " + problem-name)
        #if show-information [
          #v(10pt)
          #info-row("参赛队号：", content: team-id)
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
    ]
    #v(4.0cm)
    #text(font: "SimSun", size: 14pt)[#datetime.today().display("[year]年[month]月[day]日")]
  ]
  pagebreak()
}
