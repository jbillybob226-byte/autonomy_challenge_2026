 # autonomy team application challenge

the robot knows where it is, knows where it wants to go, and sees obstacles only a few metres ahead.
drive to the goal without hitting anything, and keep going when the world changes.

python 3.12+, stdlib only, fully in sim.

## what you get

| file | what |
|---|---|
| `agent.py` | template -- start here. its docstrings are the interface contract |
| `sim.py` | the simulator: field, dynamics, noisy pose, local scan, events |
| `harness.py` | scorer + scenario loading. `python harness.py agent.py` is the number we get |
| `viz.py` | terminal visualizer: `python harness.py agent.py --viz --nav 06` |
| `scenarios/` | twelve scenarios, a few lines of python each |
| `ruff.toml` | our lint config (2-space indent, 150 columns) |

## the world

12 m x 8 m at 0.1 m cells, robot radius 0.30 m, v_max 2.0 m/s, a_max 4.0 m/s^2, dt 0.02 s, pose noise sigma 2 cm per axis, 51 x 51 scan window (2.5 m each way).
the goal is always reachable in scored scenarios.

scenario files are just geometry.
the comment at the top of each says what it tests.

## what to build

1. a map: stitch scans into your own grid, unknown until seen. decide how planning treats unknown space, and say why.
2. a planner: a path must not slip between two blocked cells that touch only at a corner.
              if the start or goal sits inside your inflation, move it to the nearest free cell.
3. a follower: absorbs pose noise and the acceleration limit without oscillating.
               do not cut corners into inflated space, at 2 m/s a 90-degree turn drifts you about 0.7 m.
4. the loop: sense, update, replan when the path is no longer valid.
             don't stop dead when the map changes, and don't drive through something that just appeared.
5. comments: explain what parts of the code do, and write a top level comment about the overall approach, the unknown-space and replan decisions, and what is missing if any.

## scoring

noise is seeded per scenario and hashing pinned, so a deterministic agent stays deterministic.
non-determinism is allowed. say so in a comment.

100 points split evenly over the scenarios (8.33 each), 3000-tick budget per scenario (60 s of sim time).
a scenario scores 0 on collision, on not getting within 0.25 m of the goal in budget, on a raise, or on a return that is not two finite numbers. otherwise:

    score = points * (0.6 * min(1, l_ref / l) + 0.4 * min(1, t_ref / t))

`l` is distance travelled, `t` ticks to goal, `l_ref` the straight-line start-to-goal distance minus the goal tolerance, `t_ref = l_ref / v_max / dt`.
both are lower bounds, not targets.
only the ranking matters.

ensure your code passes `ruff check --config ruff.toml agent.py`.
don't take more than 30 s of wall time on any scenario: up to 3000 step() calls must fit in it, about 10 ms each.
make sure the code is readable and well commented. we will read it.

## rules

- stdlib only, no libraries allowed. the harness imports nothing outside `agent.py`.
- partial work is fine if the comments say what is missing.

## running

`python harness.py agent.py` scores, `--nav 05` runs one scenario, `--viz --nav 06` watches (`--speed 4` for 4x).
in the visualizer: dark grey = unseen wall, light grey = what your `debug()` calls blocked, blue = path, orange = robot, green = goal, dark-blue box = scan window.

## submission

just your `agent.py`.
