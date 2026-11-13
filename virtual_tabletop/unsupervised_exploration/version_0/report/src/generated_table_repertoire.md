| Cluster | contact | grasp | lift | patches | disp | rot | fell | path | min dist | palm | thumb | index | middle | ring | pinky |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B0 | 0.00 | 0.00 | 0.00 | 0.01 | 0.00 | 0.01 | 0.00 | 0.35 | 0.92 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| B1 | 0.18 | 0.01 | 0.00 | 0.06 | 0.05 | 0.13 | 0.00 | 0.29 | 0.08 | 0.04 | 0.05 | 0.06 | 0.04 | 0.03 | 0.03 |
| B2 | 0.73 | 0.04 | 0.00 | 0.28 | 0.03 | 0.08 | 0.00 | 0.24 | 0.00 | 0.32 | 0.47 | 0.29 | 0.11 | 0.06 | 0.03 |
| B3 | 0.73 | 0.03 | 0.00 | 0.33 | 0.06 | 0.12 | 0.00 | 0.25 | 0.00 | 0.19 | 0.16 | 0.25 | 0.28 | 0.33 | 0.34 |
| B4 | 0.18 | 0.01 | 0.00 | 0.07 | 0.99 | 0.93 | 1.00 | 0.47 | 0.04 | 0.06 | 0.05 | 0.05 | 0.05 | 0.05 | 0.06 |
| B5 | 0.79 | 0.39 | 0.00 | 0.67 | 0.01 | 0.05 | 0.00 | 0.21 | 0.00 | 0.39 | 0.51 | 0.54 | 0.52 | 0.53 | 0.34 |
| B6 | 0.53 | 0.44 | 0.30 | 0.33 | 0.43 | 0.31 | 0.05 | 0.31 | 0.00 | 0.00 | 0.46 | 0.49 | 0.39 | 0.20 | 0.08 |

| Method | Repertoire entropy (nats) | Clusters used (≥5% of trials) |
|---|---|---|
| Goal babbling v2 + reflex | 1.63 | B0, B1, B2, B3, B4, B5 |
| Contact + reflex + curriculum | 1.58 | B1, B2, B3, B4, B5 |
| RND (seed 1) | 1.57 | B1, B2, B3, B4, B5 |
| RND (full state) | 1.56 | B1, B2, B3, B4, B5 |
| RND (interaction) | 1.54 | B1, B2, B3, B4, B5 |
| Contact + restarts | 1.53 | B1, B2, B3, B4, B5 |
| RND + restarts | 1.52 | B1, B2, B3, B4, B5 |
| Disagreement + free restarts | 1.50 | B0, B1, B2, B3, B4 |
| RND (1.4M steps) | 1.48 | B1, B2, B3, B4, B5 |
| RND + contact | 1.48 | B1, B2, B3, B4, B5 |
| RND + pink noise + DEP | 1.46 | B1, B2, B3, B4, B5 |
| Contact coverage (seed 1) | 1.45 | B1, B2, B3, B4, B5 |
| Scripted grasp | 1.44 | B1, B2, B3, B4, B6 |
| Contact coverage (CCGE-lite) | 1.40 | B1, B2, B3, B4, B5 |
| Goal babbling v2 (+contact) | 1.37 | B0, B1, B2, B3, B4 |
| Disagreement (interaction) | 1.36 | B1, B2, B3, B4 |
| RND + free-block restarts | 1.31 | B1, B2, B3, B4 |
| MUSIC v3 + contact | 1.26 | B0, B1, B2, B4 |
| Tactile surprise | 1.14 | B0, B1, B4 |
| Shaped + free restarts (supervised) | 1.09 | B1, B2, B3, B4 |
| APT (kNN entropy) | 1.04 | B2, B3, B4 |
| Shaped reward (supervised) | 1.03 | B1, B2, B4 |
| DIAYN skills | 1.02 | B0, B1, B2 |
| DEP | 0.57 | B0, B1 |
| CSF skills | 0.29 | B0, B1 |
| Max-entropy (no reward) | 0.26 | B0 |
| Goal babbling (IMGEP+HER) | 0.05 | B0 |