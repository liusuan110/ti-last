#import "@preview/cuti:0.3.0": show-cn-fakebold
#show: show-cn-fakebold
#let abstract-page(
  body: content,
  keywords: array,
) = {
  v(1.2cm)
  align(center)[
    #text(font: ("Times New Roman", "SimHei"), size: 16pt, weight: "bold")[摘要]
  ]
  v(1cm)
  text(font: ("Times New Roman", "SimSun"), size: 12pt, weight: "bold")[#body]

  v(1cm)
  let keyword_list = keywords.join("； ")
  align(left)[
    #text(font: ("Times New Roman", "SimSun"), size: 12pt)[*关键词：* #keyword_list]
  ]
  pagebreak()
}
