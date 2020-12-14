## MIST

This repository contains the source code and benchmark instances of the AAAI'21 paper **Choosing the Initial State for Online Replanning** by Maximilian Fickert, Ivan Gavran, Ivan Fedotov, Jörg Hoffmann, Rupak Majumdar, and Wheeler Ruml.

Most of the MIST code is implemented in `src/search/online`. The MIST search engine is declared as `online_pruning`, the recoverability variant is called `online_recoverability`. For example, the best-performing MIST configuration of the evaluation in the paper is invoked with:

```
./fast-downward.py <domain.pddl> <problem.pddl> --search "online_pruning(ff(), num_reference_nodes=8, moving_average_size=100, reset_expansion_delay=true)"
```

The `benchmarks` directory contains the benchmark set on which we evaluated the algorithms described in the paper. The `split_goals.py` script can be used to split the goals of a standard PPDL instance into multiple sets of goals that appear after a certain amount of time.

## Fast Downward

Fast Downward is a domain-independent planning system.

For documentation and contact information see http://www.fast-downward.org/.

The following directories are not part of Fast Downward as covered by this
license:

* ./src/search/ext

For the rest, the following license applies:

```
Fast Downward is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Fast Downward is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
```
