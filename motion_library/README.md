# p8_beta

See report for approach and observations.

## Example usage

After having woken Jinx up at the usual starting position and
launching `cwru_base`, run the below.

Run this command to configure everything except the trajectory
publisher and the steerer.  This should open RViz with AMCL's current
best guess; if the cloud of hypotheses covers a large set of
possibilities, it may be desirable to manually take Jinx for a walk
down the hall, to let her observe more features and thus shrink the
cloud.  The reason for this is that the steerer's starting point comes
from AMCL, and too much uncertainty would produce unfavorable results.

``` $ roslaunch p8_beta jinx_amcl.launch ```

Run this command to make her start moving

```
$ roslaunch p8_beta jinx_vending.launch
```

## Prompt

Repeat PS7, but this time using feedback with respect to the world, with estimates of pose that combine odometry and LIDAR-based localization.

One submission per group.  Includes code, video and brief report with approach and observations.

Your video will hopefully demonstrate that Jinx is quite repeatable, even when subjected to disturbances.
