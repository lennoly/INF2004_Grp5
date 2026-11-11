# Videos

Short videos of tests and demos on the real car. A video shows what a log
cannot: the car turning at a junction, a hump being counted, an obstacle
being bypassed. List **every** video in the table at the bottom.

## What to record

- **Short:** 15–60 seconds, one test or one feature per video.
- **Say what it is:** show or say the test ID (e.g. `S2-U03`, `INT-02`)
  or the demo name at the start.
- **Show the commit:** run `git rev-parse --short HEAD` before recording
  and keep it in shot for a moment, or put it in the table below.
- **Show the result:** keep the car and, if it matters, the dashboard or
  console in view. Landscape, steady, good light.

## How to share it: unlisted YouTube only

1. Upload to YouTube with visibility **Unlisted** (not Private: the link
   would not work for the team or the instructor; not Public).
2. Copy the link (e.g. `https://youtu.be/...`) into the table below.

**Never commit video files.** A video stays in the git history for good and
slows every clone; `.gitignore` blocks the usual video formats. If a video is
evidence for a test, also put its link in that test's result row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-09 | System | SYS-01 full lap | f73ab1d | 2500343 | https://youtube.com/shorts/8sh7tQbx5eY | line following, no obstacles |
| 2026-11-09 | System | SYS-04 box left of the line | f73ab1d | 2500343 | https://youtube.com/shorts/l5bpn10cjaI | passed on the right, line rejoined |
| 2026-11-09 | System | SYS-04 box centred on the line | f73ab1d | 2500343 | https://youtube.com/shorts/Y-Oc0GmpVPY | passed on the left, line rejoined |
| 2026-11-09 | System | SYS-04 box right of the line | f73ab1d | 2500343 | https://youtube.com/shorts/N_EDJyTXCak | passed on the left, line rejoined |
| 2026-11-10 | System | SYS-02 LEFT turn | f73ab1d | 2500343 | https://youtube.com/shorts/DCXBFoi3twk | barcode A mapped to LEFT |
| 2026-11-10 | System | SYS-02 RIGHT turn | f73ab1d | 2500343 | https://youtube.com/shorts/xc2hqJAoMPM | barcode A mapped to RIGHT |
| 2026-11-10 | System | SYS-02 STRAIGHT | f73ab1d | 2500343 | https://youtube.com/shorts/UIX6vHWPrgY | barcode A mapped to STRAIGHT |
| 2026-11-10 | System | SYS-02 U-TURN | f73ab1d | 2500343 | https://youtube.com/shorts/ACjdW0TdYBc | U-turn plus the dashboard stats |
| 2026-11-10 | System | SYS-03 hump | f73ab1d | 2500343 | https://youtube.com/shorts/E7yLJYYMkGQ | 10 mm hump, dashboard shows 12.2 mm |
