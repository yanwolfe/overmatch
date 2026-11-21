# Devlog clip store

A curated record of how the game looked at its visible milestones, kept for the development
video (#406) and any later devlog. Nothing enters the store without the owner's approval:
it holds few shots, each one chosen. The record is the recipe; a file is a rendering of it.
Media lives outside git at `/Users/Yan/Movies/overmatch-devlog/` (the repo carries no image or
video); `INDEX.md` here holds one row per approved shot.

## Proposing

A moment is a change another player could see: a lane lands whose clip the owner judged, an
epic closes, a checkpoint changes state, a new asset or effect ships, a failure worth showing
is replaced by its fix. At such a moment the orchestrator proposes, it does not add: one line
at the end of its next report naming the moment and its recommendation (shoot, or skip and
why). A preview the work already produced may sit in the media folder's `proposed/` until
the owner answers; a declined one is deleted. The orchestrator curates before it proposes:
one shot per moment, none that an approved shot already covers.

## Shooting

For each yes the owner says what the shot should show. Any camera serves: the capture plans'
fixed pose, orbit, follow and player views (`src/dev_capture/plan.rs`), the ordinary client's
own view, or a cinematic move; a camera the tooling lacks is asked for, not approximated.
The orchestrator shoots at publication quality, shows it, and reshoots on the owner's notes.
The shot enters the store, and its row the index, when the owner accepts it.

## A row

`date | commit | shows | camera | recipe | file`

- **recipe**: the command and capture plan that reproduce the shot from that commit, so it
  can be rendered again at another size or length.
- **file**: the name in the media folder.
