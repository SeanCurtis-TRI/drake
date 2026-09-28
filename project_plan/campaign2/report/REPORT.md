# Campaign2 cross-experiment report

Wall clock and phase timers come from the **timing pass** (collect_heavy_stats OFF, serial execution). Counters include rejected steps. Phase times: t_build = ICF problem assembly minus geometry queries; t_geom = hydroelastic contact-surface queries inside the builder; t_feas = IsFeasibleTrajectory (CCD); t_solve = convex solves.

## accuracy = 0.1, beta = 1

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.1-beta1 | ok | yes | 4,148 | 12,444 | 120 | 444 | 1,823 | 898 | 1,823 | 69.2 | 0.8376 | 15.1 | 12.8 | 40.3 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.1-beta1 | ok | yes | 4,148 | 12,444 | 120 | 477 | 1,950 | 961 | 1,950 | 160.1 | 2.2 | 42.7 | 16.3 | 98.4 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.1-beta1 | ok | yes | 4,148 | 12,444 | 120 | 248 | 926 | 523 | 926 | 32.0 | 0.3728 | 7.7 | 8.0 | 15.9 |
| clutter | mesh20-bar0.001-mar0.001-acc0.1-beta1 | ok | yes | 4,148 | 12,444 | 120 | 231 | 933 | 476 | 933 | 82.4 | 0.9418 | 19.0 | 7.4 | 54.9 |
| clutter | prim-bar0.0001-mar0.0001-acc0.1-beta1 | ok | yes | 6,456 | 19,368 | 120 | 337 | 1,646 | 706 | 1,646 | 22.5 | 0.2566 | 2.8 | 4.5 | 14.9 |
| clutter | prim-bar0.0001-mar0.001-acc0.1-beta1 | ok | yes | 6,456 | 19,368 | 120 | 233 | 969 | 466 | 969 | 20.9 | 0.3255 | 3.8 | 2.6 | 14.1 |
| clutter | prim-bar0.001-mar0.0001-acc0.1-beta1 | ok | yes | 6,456 | 19,368 | 120 | 146 | 570 | 294 | 570 | 8.4 | 0.08939 | 0.9897 | 2.0 | 5.2 |
| clutter | prim-bar0.001-mar0.001-acc0.1-beta1 | ok | yes | 6,456 | 19,368 | 120 | 91 | 336 | 183 | 336 | 8.9 | 0.1076 | 1.4 | 1.2 | 6.3 |
| hero | barrier-acc0.1-beta1 | ok | yes | 19,036 | 55,840 | 42 | 18,954 | 78,485 | 37,909 | 78,485 | 722.5 | 13.0 | 159.8 | 279.8 | 248.1 |
| hero | volumetric-acc0.1-beta1 | ok | yes | 18,988 | 72,588 | 42 | 10,002 | 30,006 | 20,004 | 30,006 | 169.3 | 4.6 | 108.0 | 3.9 | 47.8 |
| nut_and_bolt | acc0.1-beta1 | ok | yes | 17,614 | 52,842 | 6 | 476 | 2,408 | 957 | 2,408 | 916.3 | 2.7 | 839.4 | 47.2 | 26.5 |
| spiral | bm0.0001-acc0.1-beta1 | heavy-only | NO(stuck) | 10,080 | 54,792 | 6 | 298 | 1,347 | 618 | 1,347 | - | - | - | - | - |
| spiral | bm1e-05-acc0.1-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,873 | 10,623 | 4,094 | 10,623 | - | - | - | - | - |
| spiral | bm2e-05-acc0.1-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,115 | 6,164 | 2,407 | 6,164 | - | - | - | - | - |
| spiral | bm5e-05-acc0.1-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 809 | 4,677 | 1,902 | 4,677 | - | - | - | - | - |

## accuracy = 0.1, beta = 0.1

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.1-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 336 | 1,358 | 692 | 1,358 | 58.5 | 0.599 | 11.8 | 9.6 | 36.4 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.1-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 148 | 613 | 311 | 613 | 58.4 | 0.4495 | 8.4 | 5.2 | 44.2 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.1-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 161 | 670 | 344 | 670 | 29.6 | 0.2599 | 5.4 | 7.2 | 16.7 |
| clutter | mesh20-bar0.001-mar0.001-acc0.1-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 119 | 496 | 259 | 496 | 35.1 | 0.3289 | 6.4 | 4.1 | 24.2 |
| clutter | prim-bar0.0001-mar0.0001-acc0.1-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 304 | 1,319 | 630 | 1,319 | 19.5 | 0.2085 | 2.3 | 3.8 | 13.2 |
| clutter | prim-bar0.0001-mar0.001-acc0.1-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 119 | 485 | 246 | 485 | 13.4 | 0.1447 | 1.6 | 1.6 | 10.0 |
| clutter | prim-bar0.001-mar0.0001-acc0.1-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 149 | 572 | 299 | 572 | 8.4 | 0.0911 | 0.9889 | 1.6 | 5.7 |
| clutter | prim-bar0.001-mar0.001-acc0.1-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 90 | 352 | 185 | 352 | 9.9 | 0.1048 | 1.3 | 1.2 | 7.3 |
| hero | barrier-acc0.1-beta0.1 | ok | yes | 19,036 | 55,840 | 42 | 10,084 | 30,442 | 20,168 | 30,442 | 254.7 | 7.0 | 87.2 | 108.3 | 46.2 |
| hero | volumetric-acc0.1-beta0.1 | ok | yes | 18,988 | 72,588 | 42 | 10,002 | 30,006 | 20,004 | 30,006 | 136.0 | 4.0 | 100.3 | 3.9 | 22.8 |
| nut_and_bolt | acc0.1-beta0.1 | heavy-only | yes | 17,614 | 52,842 | 6 | 445 | 2,300 | 913 | 2,300 | - | - | - | - | - |
| spiral | bm0.0001-acc0.1-beta0.1 | heavy-only | NO(stuck) | 10,080 | 54,792 | 6 | 175 | 760 | 386 | 760 | - | - | - | - | - |
| spiral | bm1e-05-acc0.1-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,919 | 11,188 | 4,374 | 11,188 | - | - | - | - | - |
| spiral | bm2e-05-acc0.1-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 918 | 4,911 | 1,947 | 4,911 | - | - | - | - | - |
| spiral | bm5e-05-acc0.1-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 664 | 3,911 | 1,597 | 3,911 | - | - | - | - | - |

## accuracy = 0.1, beta = 0.01

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.1-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 365 | 1,545 | 760 | 1,545 | 106.5 | 0.87 | 16.3 | 10.1 | 78.9 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.1-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 196 | 980 | 463 | 980 | 122.4 | 0.6806 | 12.8 | 8.8 | 99.9 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.1-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 156 | 630 | 326 | 630 | 27.7 | 0.2587 | 5.0 | 4.4 | 17.9 |
| clutter | mesh20-bar0.001-mar0.001-acc0.1-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 126 | 508 | 270 | 508 | 42.0 | 0.3827 | 7.5 | 3.3 | 30.7 |
| clutter | prim-bar0.0001-mar0.0001-acc0.1-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 304 | 1,299 | 627 | 1,299 | 21.8 | 0.2022 | 2.2 | 3.5 | 15.7 |
| clutter | prim-bar0.0001-mar0.001-acc0.1-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 117 | 491 | 241 | 491 | 20.0 | 0.1575 | 1.8 | 1.5 | 16.4 |
| clutter | prim-bar0.001-mar0.0001-acc0.1-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 137 | 524 | 276 | 524 | 8.6 | 0.08417 | 0.9845 | 1.7 | 5.8 |
| clutter | prim-bar0.001-mar0.001-acc0.1-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 96 | 367 | 195 | 367 | 11.1 | 0.1075 | 1.3 | 1.1 | 8.5 |
| hero | barrier-acc0.1-beta0.01 | ok | yes | 19,036 | 55,840 | 42 | 10,088 | 30,469 | 20,178 | 30,469 | 286.7 | 7.2 | 92.5 | 135.1 | 45.7 |
| hero | volumetric-acc0.1-beta0.01 | ok | yes | 18,988 | 72,588 | 42 | 10,002 | 30,006 | 20,004 | 30,006 | 126.5 | 4.0 | 90.3 | 3.9 | 23.4 |
| nut_and_bolt | acc0.1-beta0.01 | heavy-only | NO(jam) | 17,614 | 52,842 | 6 | 140 | 674 | 304 | 674 | - | - | - | - | - |
| spiral | bm0.0001-acc0.1-beta0.01 | heavy-only | NO(stuck) | 10,080 | 54,792 | 6 | 166 | 718 | 368 | 718 | - | - | - | - | - |
| spiral | bm1e-05-acc0.1-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,765 | 10,352 | 4,071 | 10,352 | - | - | - | - | - |
| spiral | bm2e-05-acc0.1-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 912 | 5,124 | 2,044 | 5,124 | - | - | - | - | - |
| spiral | bm5e-05-acc0.1-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 699 | 4,118 | 1,655 | 4,118 | - | - | - | - | - |

## accuracy = 0.01, beta = 1

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.01-beta1 | ok | yes | 4,148 | 12,444 | 120 | 539 | 2,270 | 1,123 | 2,270 | 122.4 | 1.6 | 30.8 | 16.8 | 72.7 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.01-beta1 | ok | yes | 4,148 | 12,444 | 120 | 768 | 3,266 | 1,718 | 3,266 | 217.2 | 2.8 | 52.4 | 28.4 | 133.0 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.01-beta1 | ok | yes | 4,148 | 12,444 | 120 | 309 | 1,317 | 724 | 1,317 | 59.7 | 0.5847 | 11.1 | 14.4 | 33.5 |
| clutter | mesh20-bar0.001-mar0.001-acc0.01-beta1 | ok | yes | 4,148 | 12,444 | 120 | 321 | 1,334 | 759 | 1,334 | 88.6 | 1.2 | 24.5 | 9.3 | 53.3 |
| clutter | prim-bar0.0001-mar0.0001-acc0.01-beta1 | ok | yes | 6,456 | 19,368 | 120 | 284 | 1,210 | 583 | 1,210 | 17.2 | 0.1938 | 2.1 | 2.7 | 12.1 |
| clutter | prim-bar0.0001-mar0.001-acc0.01-beta1 | ok | yes | 6,456 | 19,368 | 120 | 256 | 1,003 | 537 | 1,003 | 27.1 | 0.3478 | 4.0 | 2.3 | 20.4 |
| clutter | prim-bar0.001-mar0.0001-acc0.01-beta1 | ok | yes | 6,456 | 19,368 | 120 | 169 | 715 | 380 | 715 | 10.6 | 0.117 | 1.4 | 1.7 | 7.4 |
| clutter | prim-bar0.001-mar0.001-acc0.01-beta1 | ok | yes | 6,456 | 19,368 | 120 | 184 | 828 | 450 | 828 | 21.9 | 0.2841 | 3.6 | 1.7 | 16.3 |
| hero | barrier-acc0.01-beta1 | ok | yes | 19,036 | 55,840 | 42 | 18,600 | 75,964 | 37,202 | 75,964 | 798.4 | 13.9 | 166.6 | 297.2 | 299.6 |
| hero | volumetric-acc0.01-beta1 | ok | yes | 18,988 | 72,588 | 42 | 10,011 | 30,102 | 20,045 | 30,102 | 144.5 | 4.0 | 90.1 | 3.9 | 41.6 |
| nut_and_bolt | acc0.01-beta1 | heavy-only | yes | 17,614 | 52,842 | 6 | 495 | 2,256 | 1,011 | 2,256 | - | - | - | - | - |
| spiral | bm0.0001-acc0.01-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 602 | 2,969 | 1,494 | 2,969 | - | - | - | - | - |
| spiral | bm1e-05-acc0.01-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,997 | 11,030 | 4,354 | 11,030 | - | - | - | - | - |
| spiral | bm2e-05-acc0.01-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,395 | 7,794 | 3,203 | 7,794 | - | - | - | - | - |
| spiral | bm5e-05-acc0.01-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 733 | 3,802 | 1,617 | 3,802 | - | - | - | - | - |

## accuracy = 0.01, beta = 0.1

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.01-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 325 | 1,374 | 707 | 1,374 | 67.0 | 0.6656 | 11.9 | 7.1 | 47.2 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.01-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 335 | 1,742 | 885 | 1,742 | 154.8 | 1.4 | 25.8 | 11.7 | 115.5 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.01-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 302 | 1,438 | 761 | 1,438 | 52.3 | 0.5866 | 11.9 | 10.1 | 29.6 |
| clutter | mesh20-bar0.001-mar0.001-acc0.01-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 360 | 1,748 | 933 | 1,748 | 118.4 | 1.2 | 21.1 | 12.1 | 83.7 |
| clutter | prim-bar0.0001-mar0.0001-acc0.01-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 271 | 1,132 | 572 | 1,132 | 16.0 | 0.1829 | 2.0 | 2.7 | 11.0 |
| clutter | prim-bar0.0001-mar0.001-acc0.01-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 234 | 963 | 535 | 963 | 30.7 | 0.3607 | 4.1 | 2.1 | 24.1 |
| clutter | prim-bar0.001-mar0.0001-acc0.01-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 165 | 694 | 373 | 694 | 10.2 | 0.1241 | 1.3 | 1.3 | 7.4 |
| clutter | prim-bar0.001-mar0.001-acc0.01-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 150 | 661 | 357 | 661 | 18.1 | 0.2107 | 2.7 | 1.6 | 13.5 |
| hero | barrier-acc0.01-beta0.1 | ok | yes | 19,036 | 55,840 | 42 | 10,114 | 30,612 | 20,248 | 30,612 | 268.3 | 6.6 | 81.2 | 121.5 | 53.0 |
| hero | volumetric-acc0.01-beta0.1 | ok | yes | 18,988 | 72,588 | 42 | 10,009 | 30,084 | 20,037 | 30,084 | 135.2 | 4.1 | 96.0 | 4.0 | 26.1 |
| nut_and_bolt | acc0.01-beta0.1 | heavy-only | yes | 17,614 | 52,842 | 6 | 425 | 1,999 | 861 | 1,999 | - | - | - | - | - |
| spiral | bm0.0001-acc0.01-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 624 | 3,341 | 1,676 | 3,341 | - | - | - | - | - |
| spiral | bm1e-05-acc0.01-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,896 | 10,615 | 4,246 | 10,615 | - | - | - | - | - |
| spiral | bm2e-05-acc0.01-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,266 | 7,445 | 3,109 | 7,445 | - | - | - | - | - |
| spiral | bm5e-05-acc0.01-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 684 | 3,704 | 1,586 | 3,704 | - | - | - | - | - |

## accuracy = 0.01, beta = 0.01

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.01-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 412 | 1,794 | 914 | 1,794 | 95.6 | 0.7758 | 14.3 | 13.5 | 66.8 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.01-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 349 | 1,717 | 894 | 1,717 | 141.5 | 1.1 | 18.9 | 12.3 | 108.9 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.01-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 301 | 1,355 | 730 | 1,355 | 48.9 | 0.4717 | 8.7 | 9.2 | 30.5 |
| clutter | mesh20-bar0.001-mar0.001-acc0.01-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 337 | 1,655 | 878 | 1,655 | 126.7 | 1.2 | 25.0 | 13.3 | 86.9 |
| clutter | prim-bar0.0001-mar0.0001-acc0.01-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 276 | 1,104 | 565 | 1,104 | 17.1 | 0.1784 | 2.0 | 2.4 | 12.5 |
| clutter | prim-bar0.0001-mar0.001-acc0.01-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 212 | 950 | 507 | 950 | 38.9 | 0.3295 | 3.8 | 2.0 | 32.6 |
| clutter | prim-bar0.001-mar0.0001-acc0.01-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 164 | 684 | 368 | 684 | 10.5 | 0.1122 | 1.2 | 1.4 | 7.7 |
| clutter | prim-bar0.001-mar0.001-acc0.01-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 174 | 761 | 418 | 761 | 19.0 | 0.2481 | 3.1 | 1.6 | 14.0 |
| hero | barrier-acc0.01-beta0.01 | ok | yes | 19,036 | 55,840 | 42 | 10,119 | 30,651 | 20,263 | 30,651 | 289.6 | 7.1 | 85.9 | 126.6 | 64.0 |
| hero | volumetric-acc0.01-beta0.01 | ok | yes | 18,988 | 72,588 | 42 | 10,012 | 30,105 | 20,047 | 30,105 | 136.2 | 4.1 | 95.9 | 3.9 | 27.2 |
| nut_and_bolt | acc0.01-beta0.01 | heavy-only | yes | 17,614 | 52,842 | 6 | 477 | 2,276 | 984 | 2,276 | - | - | - | - | - |
| spiral | bm0.0001-acc0.01-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 601 | 3,122 | 1,580 | 3,122 | - | - | - | - | - |
| spiral | bm1e-05-acc0.01-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,758 | 9,795 | 3,966 | 9,795 | - | - | - | - | - |
| spiral | bm2e-05-acc0.01-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,293 | 7,166 | 2,971 | 7,166 | - | - | - | - | - |
| spiral | bm5e-05-acc0.01-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 671 | 3,684 | 1,577 | 3,684 | - | - | - | - | - |

## accuracy = 0.001, beta = 1

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.001-beta1 | ok | yes | 4,148 | 12,444 | 120 | 1,329 | 5,480 | 3,122 | 5,480 | 213.2 | 3.0 | 56.5 | 45.5 | 107.6 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.001-beta1 | ok | yes | 4,148 | 12,444 | 120 | 1,665 | 7,277 | 4,088 | 7,277 | 362.4 | 6.1 | 110.2 | 42.3 | 202.5 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.001-beta1 | ok | yes | 4,148 | 12,444 | 120 | 1,216 | 5,743 | 3,125 | 5,743 | 186.1 | 2.6 | 48.5 | 35.0 | 99.4 |
| clutter | mesh20-bar0.001-mar0.001-acc0.001-beta1 | ok | yes | 4,148 | 12,444 | 120 | 1,669 | 7,042 | 4,015 | 7,042 | 345.6 | 5.6 | 109.5 | 62.2 | 167.2 |
| clutter | prim-bar0.0001-mar0.0001-acc0.001-beta1 | ok | yes | 6,456 | 19,368 | 120 | 636 | 2,693 | 1,494 | 2,693 | 42.4 | 0.543 | 6.5 | 6.0 | 29.2 |
| clutter | prim-bar0.0001-mar0.001-acc0.001-beta1 | ok | yes | 6,456 | 19,368 | 120 | 853 | 3,899 | 2,142 | 3,899 | 104.9 | 1.6 | 18.9 | 9.0 | 75.2 |
| clutter | prim-bar0.001-mar0.0001-acc0.001-beta1 | ok | yes | 6,456 | 19,368 | 120 | 558 | 2,517 | 1,383 | 2,517 | 33.3 | 0.4467 | 5.7 | 5.6 | 21.5 |
| clutter | prim-bar0.001-mar0.001-acc0.001-beta1 | ok | yes | 6,456 | 19,368 | 120 | 770 | 3,887 | 2,061 | 3,887 | 88.7 | 1.3 | 16.0 | 7.8 | 63.3 |
| hero | barrier-acc0.001-beta1 | ok | yes | 19,036 | 55,840 | 42 | 21,820 | 88,387 | 47,303 | 88,387 | 945.0 | 16.0 | 201.4 | 361.0 | 345.8 |
| hero | volumetric-acc0.001-beta1 | ok | yes | 18,988 | 72,588 | 42 | 11,889 | 38,931 | 24,866 | 38,931 | 184.5 | 4.8 | 109.1 | 5.1 | 59.2 |
| nut_and_bolt | acc0.001-beta1 | heavy-only | yes | 17,614 | 52,842 | 6 | 816 | 4,077 | 2,086 | 4,077 | - | - | - | - | - |
| spiral | bm0.0001-acc0.001-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,610 | 8,488 | 4,365 | 8,488 | - | - | - | - | - |
| spiral | bm1e-05-acc0.001-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 2,231 | 11,215 | 4,953 | 11,215 | - | - | - | - | - |
| spiral | bm2e-05-acc0.001-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,607 | 7,774 | 3,629 | 7,774 | - | - | - | - | - |
| spiral | bm5e-05-acc0.001-beta1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,065 | 4,940 | 2,527 | 4,940 | - | - | - | - | - |

## accuracy = 0.001, beta = 0.1

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.001-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 1,077 | 4,885 | 2,671 | 4,885 | 158.3 | 2.0 | 34.5 | 41.5 | 79.8 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.001-beta0.1 | timeout | ? | - | - | - | - | - | - | - | - | - | - | - | - |
| clutter | mesh20-bar0.001-mar0.0001-acc0.001-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 913 | 4,525 | 2,414 | 4,525 | 156.4 | 2.2 | 39.8 | 24.5 | 89.3 |
| clutter | mesh20-bar0.001-mar0.001-acc0.001-beta0.1 | ok | yes | 4,148 | 12,444 | 120 | 2,210 | 10,458 | 5,694 | 10,458 | 631.5 | 9.7 | 198.4 | 67.6 | 353.6 |
| clutter | prim-bar0.0001-mar0.0001-acc0.001-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 659 | 2,829 | 1,562 | 2,829 | 40.6 | 0.6229 | 6.5 | 6.3 | 27.0 |
| clutter | prim-bar0.0001-mar0.001-acc0.001-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 894 | 3,778 | 2,144 | 3,778 | 85.8 | 1.5 | 17.1 | 7.6 | 59.3 |
| clutter | prim-bar0.001-mar0.0001-acc0.001-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 465 | 2,186 | 1,181 | 2,186 | 29.4 | 0.3722 | 4.3 | 4.2 | 20.4 |
| clutter | prim-bar0.001-mar0.001-acc0.001-beta0.1 | ok | yes | 6,456 | 19,368 | 120 | 665 | 3,198 | 1,726 | 3,198 | 60.9 | 1.0 | 12.4 | 5.2 | 42.0 |
| hero | barrier-acc0.001-beta0.1 | ok | yes | 19,036 | 55,840 | 42 | 11,650 | 39,113 | 24,640 | 39,113 | 327.3 | 7.8 | 87.5 | 146.4 | 78.0 |
| hero | volumetric-acc0.001-beta0.1 | ok | yes | 18,988 | 72,588 | 42 | 10,181 | 31,212 | 20,585 | 31,212 | 149.7 | 4.3 | 103.5 | 4.0 | 32.7 |
| nut_and_bolt | acc0.001-beta0.1 | heavy-only | yes | 17,614 | 52,842 | 6 | 667 | 2,694 | 1,474 | 2,694 | - | - | - | - | - |
| spiral | bm0.0001-acc0.001-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,678 | 8,621 | 4,497 | 8,621 | - | - | - | - | - |
| spiral | bm1e-05-acc0.001-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,932 | 9,839 | 4,371 | 9,839 | - | - | - | - | - |
| spiral | bm2e-05-acc0.001-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,596 | 7,903 | 3,674 | 7,903 | - | - | - | - | - |
| spiral | bm5e-05-acc0.001-beta0.1 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,037 | 4,846 | 2,443 | 4,846 | - | - | - | - | - |

## accuracy = 0.001, beta = 0.01

| experiment | config | status | success | tris | tets | nv | steps | solves | geom q | feas q | wall [s] | t_build | t_geom | t_feas | t_solve |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.0001-acc0.001-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 937 | 4,332 | 2,349 | 4,332 | 146.0 | 1.8 | 30.3 | 34.8 | 78.6 |
| clutter | mesh20-bar0.0001-mar0.001-acc0.001-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 1,339 | 6,796 | 3,600 | 6,796 | 405.7 | 4.9 | 85.5 | 60.8 | 253.3 |
| clutter | mesh20-bar0.001-mar0.0001-acc0.001-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 1,056 | 5,421 | 2,859 | 5,421 | 198.2 | 2.2 | 43.1 | 48.2 | 104.1 |
| clutter | mesh20-bar0.001-mar0.001-acc0.001-beta0.01 | ok | yes | 4,148 | 12,444 | 120 | 1,235 | 6,068 | 3,255 | 6,068 | 317.2 | 4.2 | 85.5 | 53.7 | 172.9 |
| clutter | prim-bar0.0001-mar0.0001-acc0.001-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 513 | 2,292 | 1,239 | 2,292 | 34.2 | 0.4304 | 4.7 | 4.6 | 24.4 |
| clutter | prim-bar0.0001-mar0.001-acc0.001-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 674 | 3,510 | 1,835 | 3,510 | 90.7 | 1.2 | 13.8 | 7.6 | 67.8 |
| clutter | prim-bar0.001-mar0.0001-acc0.001-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 609 | 2,775 | 1,522 | 2,775 | 40.0 | 0.5189 | 6.2 | 6.1 | 27.0 |
| clutter | prim-bar0.001-mar0.001-acc0.001-beta0.01 | ok | yes | 6,456 | 19,368 | 120 | 629 | 2,984 | 1,621 | 2,984 | 63.2 | 0.9996 | 12.0 | 5.5 | 44.5 |
| hero | barrier-acc0.001-beta0.01 | ok | yes | 19,036 | 55,840 | 42 | 10,902 | 35,268 | 22,610 | 35,268 | 341.0 | 7.8 | 94.8 | 153.7 | 77.6 |
| hero | volumetric-acc0.001-beta0.01 | ok | yes | 18,988 | 72,588 | 42 | 10,212 | 31,359 | 20,665 | 31,359 | 135.0 | 4.1 | 90.1 | 4.1 | 31.5 |
| nut_and_bolt | acc0.001-beta0.01 | heavy-only | yes | 17,614 | 52,842 | 6 | 491 | 1,928 | 1,068 | 1,928 | - | - | - | - | - |
| spiral | bm0.0001-acc0.001-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,562 | 8,011 | 4,175 | 8,011 | - | - | - | - | - |
| spiral | bm1e-05-acc0.001-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 2,049 | 10,471 | 4,613 | 10,471 | - | - | - | - | - |
| spiral | bm2e-05-acc0.001-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 1,388 | 6,805 | 3,202 | 6,805 | - | - | - | - | - |
| spiral | bm5e-05-acc0.001-beta0.01 | heavy-only | yes | 10,080 | 54,792 | 6 | 997 | 4,805 | 2,424 | 4,805 | - | - | - | - | - |

## Failures / timeouts

| experiment | config | timing pass | heavy pass |
|---|---|---|---|
| clutter | mesh20-bar0.0001-mar0.001-acc0.001-beta0.1 | timeout | timeout |

Additionally, 44 config(s) are `heavy-only`: the heavy pass succeeded but the serial timing pass was not run (campaign stopped early). Their counters, scene stats, and success columns are valid (timing-independent, from the heavy pass); wall clock and phase timers are omitted. Resume with `run_all.py --passes timing` to fill them in.
