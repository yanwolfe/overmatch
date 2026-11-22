# Fix pass: report

Subject: the game's building model (game state only). `lane/bounded`, `6e01d03a` to `e0fbf280` plus this report.

**Verdict: ready to merge.** fmt, CI clippy, the server check and the whole lib suite are green.

## Fix list
1. Done. `bound_contacts` and `bound_belts` price each owner as a task (`per_owner`; floor `PARALLEL_OWNERS` 2, labelled unmeasured) and commit in owner and body order.
2. Done. All three fields deleted.
3. Done. One test through the mounted plugin; two mutants run and caught (sign flipped, cells never queued).
4. Done. `Standing::give` deleted. The belt keeps its behaviour, with the reason beside it: a staged force has no contact constraint, so no solve says whether the cells stopped the belt.
5. Done. One outcome moved: a block whose body left the world inside the tick is skipped, where it was read at zero speed.
6. Done. `resolve` refuses a chunk no joint path holds to the ground; the spawn panic is gone; `spawn` takes `Site`. A view fixture without anchors gained them.
7. Done. With the panic gone the assertion can fail.
8. Done, all ten.
9. Done. `SAME_SIDE` keeps 0.99 with its real reason (cost per side priced); the allowances are a derived bound or rounding.

Backlog: nine rows added (destruction-04..10, dev-23, net-16).

## Bit for bit
`ram-house-face`, plain brick wall, hall wall, hall junction, governed, dev builds of `6e01d03a` and `e0fbf280` (MEASURED): 741 rows each from 100 ticks before contact, whole hull rows (pose, velocity, every block's price and return) and end structure identical.

## Suite
2,846 passed, 0 failed, 53 ignored (MEASURED, after the last source change).

## Where the brick goes
A deliberate, named law removes it; nothing changed. `debris::crumble::route` keeps a fragment as a body only at `server_rubble_min_size_m` = 0.5 m (`Taste`, `physical_laws.ron`; cube root of volume). Smaller matter becomes decoration: no collider, drawn 6 to 12 s (`Taste`, `construction_decor.lifetime_s`), at most 256 at once.

`house_0001` face, 20 km/h, one run with temporary log lines (MEASURED; held passes do not repeat):

| | Cells |
|---|---|
| Left the structure | 325 |
| In bodies at the end | 135, in 13 bodies |
| Decoration, by the size floor | 208, in 115 fragments, 2.93 m³ |
| Retired below the floor, capped, crushed | 0 |

204 of the 208 went at the contact release. 75 fragments are one cell; sizes 0.16 to 0.48 m, median 0.26 m. The 13 bodies rest frozen: 6 on the ground 3 to 11 m past the face, 7 on standing matter 2.3 to 4.0 m above grade; none on the hull; none retired in the 10 s watched.

The clip's own run: 242 left, 34 in 3 bodies. Main's law, same recipe: 965 left, 901 in 54 bodies. Plain wall at 10 km/h: 167 left, 53 in 4 bodies; main 410 and 393.

Why (DERIVED from `bounded::cut`): the contact law frees one small plug per block per tick, and `merge_fragments` joins a small fragment only to one leaving in the same event; a plug's parted joints lead to standing matter.
