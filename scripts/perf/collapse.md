The video game's environment-destruction instrument uses real match clients and ordinary server simulation. Run `collapse.sh --help` for the full interface. A rehearsal records counts on the local machine; the hosted server supplies pricing evidence.

`rest-1.json`, `rest-3.json`, and `rest-6.json` each fell their named map houses together at 2 wall seconds after combat admission. They schedule no further breaks. Observe for 65 seconds and check `counts.json`: each target must have `falls > 0`, `awake == 0`, `sleeping + frozen > 0`, and at least 30 seconds in its unchanged resting suffix. These are sampled per-second observations. `falls` and `dust` count chunks; `awake`, `sleeping` and `frozen` count physical groups (`src/debris/freeze.rs`). Replicated pose carriers do not count as additional matter.

```sh
scripts/perf/collapse.sh --target local --rehearsal   --schedule scripts/perf/rest-1.json --tanks 30 --clients 2 --duration 65   --out target/harness-spread/clients-2
```

Repeat the same recipe with `--clients 4` and `--clients 8` for replication comparisons. Remaining tanks are ordinary bots. Each real client is a separate GPU-less process, with `OVERMATCH_UNATTENDED=1`, `OVERMATCH_WINDOW=headless`; it creates no window. For prebuilt binaries add `--bin /absolute/binary/directory` and set `BEVY_ASSET_ROOT` to this checkout.

`counts.json` retains bytes and covered wall seconds per server link, whether that link appeared in every admitted stats window, and update-frame counts for every client. Bytes/s is bytes divided by exact covered seconds, including encrypted netcode overhead and excluding IP/UDP headers; it counts queued sends rather than acknowledged delivery. Client frames come from the existing `SPIKE_FRAME_COST` recorder. The two boundary seconds are omitted for admission skew and buffered output. A client keeps up when at least 90% of its frames fit within one fixed tick and its frames occupy at least 90% of the 64 Hz wall-clock slots. `all_clients_keeping_up` records that gate. Run validity describes test fairness: clients must keep up, every link must appear in every admitted window, and the harness must observe complete, compatible data. A failure retains artifacts, prints `INVALID RUN` (or the observation failure), and exits nonzero. Exclude these unfair captures from workload count comparisons.

A server falling behind is a measured result. The saved `capacity` object keeps `valid` true when the test is fair, and separately records `server_over_budget`, `server_keeping_up`, and measured `tick_hz`. The console prints a `MEASURED server_over_budget=...` line with that rate; a fair slow-server run prints `COMPLETE` and exits zero. Only the multi-client population-capacity comparison uses `capacity_supported`: it requires both fairness and the server sustaining its fixed rate over complete windows (one boundary tick allowed). A false `capacity_supported` does not invalidate workload counts. Ram captures use the same fairness and server-performance reporting, omitting pre-capture and admission-straddling server periods. Server periods retain their full precision for the rate calculation.

Every armed `server_stats` period includes `send_clients` entries (`link`, `bytes`, `bytes_per_s`), aggregate `send_bytes`, `send_us`, and `send_us_per_client_s`. The last field amortizes shared send work over connected clients and wall seconds; it is not an independently timed path per client. The byte collector is timed as `send_census_us` and excluded from `send_us`. Without the stats lever, none of those collection systems is installed.

For an operator-started hosted match, print its environment first, then run the same client population. Replace `HOSTED_ADDRESS` and `DEPLOYMENT_ID` below with the existing deployment's values. These commands neither create nor stop a deployment.

```sh
scripts/perf/collapse.sh --target HOSTED_ADDRESS --deployment DEPLOYMENT_ID   --schedule scripts/perf/rest-1.json --tanks 30 --clients 8 --duration 65   --print-server-env

BEVY_ASSET_ROOT="$PWD" scripts/perf/collapse.sh   --target HOSTED_ADDRESS --deployment DEPLOYMENT_ID   --schedule scripts/perf/rest-1.json --tanks 30 --clients 8 --duration 65   --bin "$PWD/target/debug" --out "$PWD/target/harness-spread/hosted-8"
```

The operator passes the printed environment to the hosted server. It selects 22 bots for 8 clients (DERIVED: 30 total minus 8 real players). Repeat with the other client counts and generate the matching environment each time.

The contact census snapshots the completed physics step before sleeping and structural retirement.
`contact_census_us` and `sweep_census_us` are subtractable tick leaves; the latter is excluded from
`sweep_us`. The census precedes the sleep timer's opening bracket explicitly. First-contact
fields describe first fall. `last_impact` records the latest solver point closing faster than the
piece's own linear sleep threshold, against any body, pieces included; waking in place is no impact.
`impact_to_sleep_s` includes any subsequent sliding or rolling and is not a visibly-still wait.

`last_moved` tracks a held pose anchor: translation plus rotation angle times the piece's bounding
radius must exceed the labelled visibility tolerance before the anchor refreshes. The radius
encloses all scaled colliders and their offsets at first observation. `thresholds.visible_motion_m`
reports that instrument's tolerance. Per-piece `still_s` is sleep minus last-moved in seconds;
`islands_at_sleep.still_s` is its members' minimum. Every sleep cohort participates, including
pieces that never carried ground load (`never_ground_loaded`) or impacted (`never_impacted`).
`below_sleep_speed_s` is a separate velocity proxy, not a visibility measure.

`piece_census.episodes` preserves completed sleep episodes across wakes. Pose anchors persist
through sleep and wake, so repeated episodes' stillness can include earlier sleeping time; use
`sleeps == 1` for first-sleep observations. `casts_per_source` retains each source's cast count
summed over all its owners, including zero-cast sources.
On Linux, `linux_host.host_probe_us` exposes the periodic `/proc` walk outside `frame_us`.
Lifecycle observations between fixed ticks are reported as `piece_census.lifecycle_outside_tick_us`,
separate from additive tick leaves. `awake_ticks` remains a piece's cumulative awake duration;
completed episodes retain their own immutable snapshots.

Each ram row's `hull` is the tank after that tick's physics step, and what moved it along its own
forward over the step (`src/net/ram.rs`, `HullRow`): solver momentum by the other body's kind and
the sweep's exchange, N s, and the sweep's verdicts on the cuts the hull met. The belts' implicit
support and traction and gravity are not channels, so the channels do not sum to m·Δv.
`impact.speed_m_s` is the horizontal speed of the row before first contact. `result.json` adds
`min_forward_m_s`, `largest_forward_change_m_s`, `crossing_s` and `stall_s`, from the contact tick
on, each defined in its `definitions`. The server's last line adds `structure`, the struck
construction at the capture's end (`src/net/ram/structure.rs`): the share of its volume built above
grade still in place (every corner within the recipe's `in_place_m`, a labelled `Unit`), the
volume-weighted displacement of what still stands on a body, the opening in `plan.panel` (the
component the approaching tank meets first, chosen before play), and the largest loose piece.

A ram pass file (`ram*.json`) is one instance met on one line, at each of its named `contacts`.
`heading_deg` turns the approach from `-Z` toward `-X`; `lateral_m` moves the line to the heading's
right of the instance's origin. A contact is one of three (`src/net/ram.rs`, `Contact`):

- `{"Governed": {"minimum_fraction", "runup_margin"}}`: full throttle to governed speed, admitted
  at that fraction of it or over.
- `{"Held": {"speed_m_s", "within_m_s", "runup_margin"}}`: the scripted driver holds that speed
  over the run-up and through the contact, its demand falling from full to none across 0.5 m/s
  under it (`src/net/harness.rs`, `HOLD_BAND_M_S`), as a player holding a speed drives. Admitted
  within `within_m_s` of it. The driver settles a little under the held speed, so the measured
  contact speed is the row's fact, not the name.
- `{"Rest": {"gap_m", "at_most_m_s"}}`: the hull stands with its nearest point `gap_m` short of
  the face it meets, settles for the harness's four seconds, then takes full throttle. Admitted at
  `at_most_m_s` or under.

The run-up is free distance: from the hull's nearest point to the face it meets along the line
(`plan.runup_m`), so a deep building costs no more of it than a thin wall. `runup_margin` scales
the ideal distance to the contact's speed for terrain and rolling losses; below top gear the ideal
overstates the distance, which only lengthens the approach. `--contact NAME` runs one contact
into `--out`; without it every contact runs in turn, each into `--out/NAME`. A deployment runs
one contact.

The proving ground is a row of plain walls on kalinovo's level ground, each a construction of its
own (`tools/constructions/wall_*.wall.ron`):

| instance | at (x, z) | what | passes (the hull's travel) |
|---|---|---|---|
| `wall_brick_double_0001` | (0, -50) | two one-brick leaves, 8 m | `ram-wall-double` (south) |
| `wall_brick_0001` | (25, -50) | one-brick leaf, 8 m × 2.7 m, the reference | `ram-wall-brick` (north), `ram-wall-brick-45` (south-east) |
| `wall_brick_coarse_0001` | (50, -50) | the reference at a 0.55 m cell | `ram-wall-coarse` (north) |
| `wall_brick_fine_0001` | (75, -50) | the reference at a 0.15 m cell | `ram-wall-fine` (north) |
| `wall_boards_0001` | (100, -50) | boards on posts over a brick plinth, 8 m | `ram-wall-boards` (north, meeting the posts' side) |
| `wall_brick_corner_0001` | (125, -50) | an L of two 5 m runs, turned 45° so its inside faces north | `ram-wall-corner` (south, into the inside) |
| `wall_brick_long_0001` | (165, 10) | one-brick leaf, 12 m, laid along `Z` | `ram-wall-long-along` (south, end-on), `ram-wall-long` (east, face-on) |

The walls stand 25 m apart, so a pass through one leaves its neighbours alone; the 45° pass
comes from the north-west and passes 15 m from the double wall's end. Ground under each footprint lies within 0.16 m (MEASURED from `terrain_height.png`).
The ground falls gently northward: a northbound approach descends 3.5 to 4 m over its 140 m and
meets its wall at governed speed; southbound, the lanes to the coarse and the board wall climb 2
to 3.4 m and the hull met them at 9.7 and 9.1 m/s, short of the 9.98 m/s admission under margins
of 3 and 2 (MEASURED, one local run each), so the reference wall, its two cuts and the board wall
are met northbound. The corner's governed contact, southbound, states a margin of 3.

Three passes state a fact of the run beside the route:

- `ram-fir.json` and `ram-wall-long-along.json` lay their line 1.035 m off the target. The planner
  finds the met face by rays from the hull's collision points, none of which leads on the centre
  line, so a trunk or a wall's end is laid on the line of the point that does (MEASURED: the
  reference wall's `plan.panel.at`; backlog dev-22). The scripted driver does not steer, and over
  the governed run-up to the long wall the hull drifted 1.2 m to its left (MEASURED, one run:
  with the line to the wall's left it passed the wall untouched), so that line lies to the wall's
  right and the slower contacts meet the wall's end nearer the hull's left side.
- `ram-wall-boards.json` stands the hull 0.1 m short for its contact from rest: at the others'
  0.02 m its lowest point met the plinth as it settled, before the capture's first row.
- `ram-fir.json` admits its contact from rest up to 1.0 m/s: the trunk's round face stands
  further from the hull's point than the ray's entry, and the hull met it at 0.54 m/s.

Beside them: `ram.json` (the village house `house_0001` end-on through its gable),
`ram-house-face.json` (another village house, `house_0046` at (152, -296), through a long wall:
`house_0001`'s long walls are reached only up a 2.7 m climb, on which the hull met the wall at
9.25 m/s under a margin of 3, MEASURED, one local run, or from beyond a 12 m drop),
`ram-hall-wall.json` (the village hall's west long wall at its middle, between the cross walls),
`ram-hall-junction.json` (the same wall where the north cross wall meets it, `lateral_m` 3),
`ram-hall-gable.json` (its north gable at the doorway) and `ram-fir.json` (`fir_a_0144`, standing
clear at (340, -342)).

`scripts/perf/proving.py OUT` runs every shipped pass at every contact, one local rehearsal each
into `OUT/PASS/CONTACT`, and writes `OUT/table.md`; it waits while another lane's local run holds
the instrument's port, and keeps each `result.json` without its per-tick lists (the run's
`server.log` holds them); it skips what `OUT` already holds, `--only
STEM` runs one pass file, `--table` prints without running. A row is one run. Two concrete walls
the design asks for are absent because the game has no concrete material row, and no loose object
(boulder, wreck) is placed.

`--clip PLAN`, local `--ram` with one contact, is `scripts/debris-lab`'s (its plan follows behind
the hull; `scripts/capture/proving-side.ron` follows square off its side and
`scripts/capture/proving-raised.ron` holds the proving ground's row of six from a fixed raised
camera): the driving client renders the
capture plan's frames on a hidden window, in the run's folder (the plan's relative paths land
there) and with its own settings folder. Its frames open on the tank's approach to the recipe's
construction (`dev_capture`, `Anchor::Approach`), and the run completes only once the client logs
`capture: N of N frames saved`. Its tick times compare only with another clip run of the same plan
on the same machine.
