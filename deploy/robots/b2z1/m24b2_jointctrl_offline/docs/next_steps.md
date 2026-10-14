# Future M2.4b3 / M2.4c acceptance checklist

1. **Electrical and mechanical inspection:** identify fuse location/rating, reason for previous fuse failure, supply voltage/current, ground, connectors, motor faults, and front-lidar clearance. No rerun until safe.
2. **Official independent baseline:** verify vendor `z1_ctrl k` works without B2 and no `z1_ros2` or duplicate controller process. This is a separate supervised exercise, not included here.
3. **SDK receive mechanism review:** determine how the local SDK actually confirms new UDP datagrams, handles UDP timeout, and synchronizes data access; never equate a `steady_clock::now()` call or unchanged `lowstate` values to confirmed reception.
4. **Read-only real Z1 status test:** independent process/explicit operator run, no LOWCMD or JOINTCTRL transition; verify q/dq/tau, motor errors, temperature, sample age, reconnection. Even "read-only" SDK setup may open a control channel and must be reviewed before powering on.
5. **Official JOINTCTRL controlled test:** only after the above, design supervised transition to JOINTCTRL from *measured actual joint pose*, trajectory bounds matching physical Z1 joint soft limits, no automatic `backToStart`, and physically accessible E-stop.
6. **Control-fault actions:** separately verify what happens on stop, program exit, network drop, and motor overcurrent/overheat; do not assume stopping transmissions stops the actuator.
7. **799 observations:** verify 12 B2 SDK2 joints + 6 *actual* Z1 SDK joints in the intended order, fresh data from both streams, quaternion conventions, target EE frame, no synthetic fallback; dry-run ONNX before outputs enabled.
8. **Joint WBC stand test:** B2 zero velocity ONNX stance plus restricted Z1 IK; initially no simultaneous large movement. Later slowly expand stationary workspace after collision checking.
9. **Walking:** Z1 remains inside validated small workspace; B2 locomotion only when safety readiness conditions true.

The offline M2.4b2 safety core's approval booleans are exercise stimuli, not credible evidence of actual electrical inspection or SDK mode transition.
