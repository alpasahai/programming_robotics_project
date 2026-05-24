// TODO: alpa go into header.typ and fill it out with your s number.
// also we have to figure out their emails and stuff to add them to the git repo
#set page(
  margin: 2.5cm,
  numbering: "1",
  number-align: bottom + center,
)
#set text(size: 10pt)
#include "0-header.typ"
#pagebreak()

#outline()
#outline(
  title: [List of Figures],
  target: figure,
)
#set heading(numbering: "1.")

#pagebreak()
// Project Sections.
#include "1-project-overview.typ"
#include "2-technical-requirements-compliance.typ"
#include "3-system-architecture.typ"
#include "4-behaviour-model.typ"
#include "5-perception-sensor-processing.typ"
#include "6-safety-and-robustness.typ"
#include "7-code-structure.typ"
#include "8-key-code-snippits.typ"
#include "9-results-discussion-summary.typ"
#include "10-individual-contributions.typ"
