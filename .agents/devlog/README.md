# Devlog clip store

A running record of how the game looked at each visible milestone, kept for the development
video (#406) and any later devlog. The record is the recipe; the pixels are a preview. Media
lives outside git at `/Users/Yan/Movies/overmatch-devlog/` (the repo carries no image or video);
`INDEX.md` here holds one row per moment.

## When a moment is recorded

A moment is a change another player could see. The orchestrator adds a **candidate** row,
unasked, at each of these, in the same turn as the landing:

- a lane lands whose clip or still the owner judged (the judged clip is the candidate);
- an epic closes or a checkpoint on the playtest board changes state;
- a new asset, map feature, vehicle or effect appears in the shipped game;
- a failure worth showing (a tank stopped dead by a wall, a house that bursts) is replaced
  by its fix: keep the before.

A candidate costs one row: date, commit, what it shows, the recipe, the preview's file name.
No capture work is done for a candidate beyond copying a clip the work already produced.

## The owner judges the capture

A candidate is not a recording. When candidates exist, the orchestrator's next report to the
owner ends with one line naming them and asking which to record. For each yes, the owner says
what the shot should show (framing, the action, the length); the orchestrator captures it at
publication quality, shows it, and marks the row `recorded` once the owner accepts it. The
owner's "I want this recorded" at any time makes a row and goes straight to that step.
A candidate the owner passes on stays a candidate: its commit and recipe keep it recoverable.

## A row

`date | commit | state | shows | recipe | file`

- **state**: `candidate`, `recorded` (owner accepted a publication-quality capture) or
  `dropped` (the owner said no).
- **recipe**: the command and capture plan that reproduce the shot from that commit, or
  `none` with what would be needed.
- **file**: the name under the media folder; previews keep the instrument's quality.
