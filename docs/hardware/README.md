# Hardware documents

Design F3: the robot carries no water. A water trolley behind the robot carries the bag, the pump and its pack.

| Document | Content |
|---|---|
| [electrical.md](electrical.md) | Power tree, brake resistors, pump power and control on the trolley, fuses, e-stop chain, computer pins, grounds, power-on test procedure |
| [schematic/](schematic/README.md) | Drawings SOB-E rev C: system overview, sheet 1 power distribution and the water trolley, sheet 2 control and communication (SVG and A3 PDF) |
| [harness.csv](harness.csv) | Every wire: ends, connectors, gauge, colour, length, fuse, current |
| [trolley.md](trolley.md) | Water trolley: stand, bag, pump box, pack, kill switch, boom, hose, pushing, tip-over check, hose force, parts, tests |
| [trolley.py](trolley.py) | Heights and masses of the trolley, tip-over and hose numbers. Draws [trolley_side.svg](trolley_side.svg) and `trolley_side.png` |
| [water.md](water.md) | Water system: bag, pump, hose, check valves, nozzle, jet range, spray time, bench tests |
| [order_list.md](order_list.md) | Parts to order that are not in the course BOM, with shops and prices |
| [power_calc.py](power_calc.py) | Power numbers from the simulation, the robot and trolley battery time, and the harness check. Output: [power_calc.md](power_calc.md) |

The mechanical design and the print kit are in [cad/final/](../../cad/final/README.md).

Check the wiring after every change:

```
uv run python docs/hardware/power_calc.py check
```

Check the trolley numbers after every change of `trolley.py`, then run `sim/water/water.py` again:

```
uv run python docs/hardware/trolley.py check
```
