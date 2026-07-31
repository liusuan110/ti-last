#import "@preview/cuti:0.3.0": show-cn-fakebold
#show: show-cn-fakebold
#let abstract-page(
  body: content,
  keywords: array,
  title: none,
) = {
  if title != none {
    v(22pt)
    align(center)[
      #set par(first-line-indent: 0em, spacing: 0pt, leading: 11.1pt)
      #text(font: "SimSun", size: 16pt, weight: "bold", stroke: 0.06em)[#title]
    ]
    v(22pt)
  } else {
    v(22pt)
  }
  set par(first-line-indent: (amount: 0em, all: true), leading: 13.8pt, spacing: 0pt)
  text(font: "SimSun", size: 14pt, weight: "bold", stroke: 0.05em)[摘要：] + text(font: "SimSun", size: 12pt)[#body]
  v(22pt)
  let keyword_list = keywords.join("； ")
  text(font: "SimSun", size: 14pt, weight: "bold", stroke: 0.05em)[关键词：] + text(font: "SimSun", size: 12pt)[#keyword_list]
  pagebreak()
}
