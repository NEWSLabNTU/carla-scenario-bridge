// Shared look of the README figures. Compile each figure with
//   typst compile docs/figures/<name>.typ docs/figures/<name>.svg
#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge

#let scen = rgb("#dfe4f7")   // scenario side (SSv2's domain)
#let world = rgb("#d5ecea")  // world servers
#let sim = rgb("#e8e8e8")    // simulators
#let veh = rgb("#e3efd6")    // vehicle side (vehicle bridges, the agent)
#let aw = rgb("#c9e0b4")     // Autoware
#let spec = rgb("#fbe7c0")   // the vehicle spec
#let accent = rgb("#0b6e69") // the two boundaries: world protocol, I5 contract

#let setup(body) = {
  set page(width: auto, height: auto, margin: 6mm, fill: white)
  set text(font: "DejaVu Sans", size: 8pt)
  body
}

#let b(pos, body, fill, name, width: 42mm) = node(pos, body, fill: fill, name: name,
  width: width, corner-radius: 3pt, stroke: 0.6pt + fill.darken(45%), inset: 5pt)
// A dashed group around `nodes`; its title is a node of its own (`title`), enclosed with them.
#let grp(..nodes) = node(enclose: nodes.pos(), inset: 6pt, corner-radius: 5pt,
  stroke: (paint: gray.darken(25%), dash: "dashed", thickness: 0.7pt))
#let title(pos, body) = node(pos, text(size: 7pt, fill: gray.darken(45%), weight: "bold",
  body), stroke: none, inset: 2pt)
#let tag(body) = text(size: 6.5pt, fill: accent, weight: "bold", body)
#let mono(body) = text(font: "DejaVu Sans Mono", size: 7pt, body)
