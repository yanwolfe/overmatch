# Devlog index

Media: `/Users/Yan/Movies/overmatch-devlog/`. Rules: `README.md`. Newest first.

| date | commit | state | shows | recipe | file |
|---|---|---|---|---|---|
| 2026-11-21 | `dd83eac1` | candidate | The hall, a 12 x 16 m brick building with a framed roof, standing among the village houses, a tank in front | none recorded: the hall lane's capture plan was not kept; re-shoot with `scripts/capture/` at the hall (`village_hall_0001`, kalinovo, about (−385, 140)) | `2026-11-21-hall-on-the-map.png` |
| 2026-11-21 | `0a063f89` (`lane/cutter`, never merged) | candidate | Two panels: today's game drives through a house and the house falls; the first cutter build keeps the house standing and the tank stops dead in the wall | `scripts/debris-lab run scripts/perf/ram-face-on.json <out> --plan scripts/capture/debris-lab-side.ron`, then `scripts/debris-lab compare` (on that branch) | `2026-11-21-cutter-build-1-tank-stopped-compare.mp4` |
| 2026-11-21 | `84170520` | candidate | A tank drives through a brick house and the whole house comes down at once (the behaviour the cutter work replaces) | `scripts/debris-lab run scripts/perf/ram-face-on.json <out> --plan scripts/capture/debris-lab-side.ron` | `2026-11-21-house-falls-on-contact-today.mp4` |
| 2026-11-21 | `23d4faca` | candidate | A broken brick wall close up: irregular lumps and rough break faces (inner face still a flat colour, texture to come) | the fracture lane's close capture plan (`plan-close.ron`, not committed); re-shoot with `scripts/capture/` | `2026-11-21-fracture-break-face-close.jpg` |
| 2026-11-21 | `832bee3e` against `23d4faca` | candidate | The same wall after the same hit, old brick-grid break beside the new irregular break | as above, one capture per commit | `2026-11-21-fracture-old-vs-new-wall.jpg` |
