| Method | Type | Steps | Touch | Pinch | Grasp | Contact time | Time to touch (s) |
|---|---|---|---|---|---|---|---|
| Scripted grasp | hand-coded | – | 98% | 83% | 73% | 35% | 1.30 |
| RND (1.4M steps) | reward-free (2x steps) | 1400k | 100% | 95% | 68% | 78% | 0.66 |
| RND + restarts | reward-free + restarts | 700k | 100% | 88% | 63% | 74% | 0.77 |
| RND (interaction) | reward-free | 700k | 100% | 97% | 67% | 76% | 0.70 |
| RND (seed 1) | reward-free (seed 1) | 700k | 100% | 90% | 52% | 74% | 0.66 |
| Contact coverage (seed 1) | reward-free (touch; seed 1) | 700k | 100% | 77% | 48% | 74% | 0.72 |
| RND + pink noise + DEP | reward-free + DEP/pink noise | 700k | 100% | 53% | 40% | 77% | 0.63 |
| Shaped reward (supervised) | **supervised** | 700k | 100% | 97% | 57% | 78% | 0.60 |
| Disagreement (interaction) | reward-free | 700k | 100% | 88% | 23% | 69% | 0.66 |
| Contact + restarts | reward-free + restarts | 700k | 100% | 80% | 52% | 79% | 0.63 |
| RND (full state) | reward-free (full state) | 700k | 100% | 93% | 55% | 76% | 0.62 |
| Contact coverage (CCGE-lite) | reward-free (touch) | 700k | 100% | 83% | 43% | 76% | 0.66 |
| Contact + reflex + curriculum | reward-free + priors | 700k | 98% | 75% | 48% | 75% | 0.68 |
| RND + contact | reward-free | 700k | 100% | 70% | 28% | 74% | 0.68 |
| Disagreement + free restarts | reward-free + restarts | 600k | 98% | 55% | 17% | 62% | 0.77 |
| RND + free-block restarts | reward-free + restarts | 600k | 88% | 65% | 15% | 52% | 0.83 |
| Shaped + free restarts (supervised) | **supervised** + restarts | 600k | 100% | 78% | 5% | 72% | 0.67 |
| APT (kNN entropy) | reward-free | 700k | 100% | 85% | 12% | 77% | 0.66 |
| Goal babbling v2 + reflex | reward-free goals + priors | 700k | 87% | 65% | 28% | 64% | 0.68 |
| Goal babbling v2 (+contact) | reward-free goals | 700k | 77% | 35% | 12% | 43% | 0.88 |
| Tactile surprise | reward-free (touch) | 700k | 15% | 7% | 2% | 4% | 1.84 |
| DIAYN skills | reward-free skills | 700k | 38% | 18% | 3% | 15% | 1.31 |
| MUSIC v3 + contact | reward-free | 700k | 20% | 12% | 2% | 10% | 1.23 |
| DEP | reward-free (no NN) | – | 12% | 0% | 0% | 2% | 1.58 |
| Max-entropy (no reward) | control (no reward) | 700k | 3% | 0% | 0% | 0% | 2.40 |
| Random actions | control | – | 0% | 0% | 0% | 0% | – |
| Still | control | – | 0% | 0% | 0% | 0% | – |
| CSF skills | reward-free skills | 700k | 0% | 0% | 0% | 0% | – |
| Goal babbling (IMGEP+HER) | reward-free goals | 700k | 0% | 0% | 0% | 0% | – |