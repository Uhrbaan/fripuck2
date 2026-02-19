#set page(
  paper: "a4",
)

#place(top, float: true, grid(
  align: top,
  columns: 3,
  image("assets/unifr.svg", height: 4em), h(1fr), image("assets/humanist.svg", height: 4em),
))

#let student-email = "firstname.lastname@unifr.ch"

#grid(
  align: center + horizon,
  columns: 1fr,
  rows: (1fr, 1fr, 1fr),
  [
    #text(size: 26pt)[Fripuck2]

    #text(size: 18pt)[Improving the e-puck2 robot as an educational tool.]

    Léonard Clément#footnote[#link("mailto:" + student-email), Bachelor thesis with HUMAN-IST institute, DIUF, University of Fribourg.]

    Supervised by Prof. Dr. Julien Nembrini.
  ],
  [#smallcaps[May] 15#super[th] 2027],
  [
    #smallcaps(text(size: 14pt)[Department of Informatics - Bachelor Thesis])

    #set text(size: 10pt)
    Département d'Informatique -- Departement für Informatik • Université de Fribourg --
    Universität Freiburg • Boulevard de Pérolles 90 • 1700 Fribourg • Switzerland

    #grid(
      columns: 4,
      column-gutter: 2em,
      [phone +41 (26) 300 84 65],
      [fax +41 (26) 300 97 31],
      link("mailto:Diuf-secr-pe@unifr.ch"),
      link("http://diuf.unifr.ch"),
    )
  ]
)

#counter(page).update(0)
#set page(numbering: "1")

#grid(
  align: horizon + center,
  rows: (1fr, 1fr),
  [
    *Abstract*

    #lorem(120)

    *Keywords*: #lorem(7)],
  [
    #v(2em)
    *Acknowlegments*

    I would like to express my sincere gratitude to #lorem(20)
  ],
)

#import "@preview/latex-lookalike:0.1.4"
#show: latex-lookalike.style-outline
#outline()
#pagebreak()

#set heading(numbering: "1.1  ")
= Introduction
= Current state
= Design
= Implementation
= Results
= Discussion
= Conclusion
