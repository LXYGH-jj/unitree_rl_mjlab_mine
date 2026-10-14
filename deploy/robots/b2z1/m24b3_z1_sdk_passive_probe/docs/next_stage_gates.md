# M2.4b3 acceptance checklist

1. Offline feedback validation tests and SDK x86_64 compile/link succeeded.
2. Code review: `unitreeArm::sendRecv` is bidirectional and NOT a passive observer. SDK `lowstate` timestamp is NOT new-packet proof.
3. Source review: no JOINTCTRL/LOWCMD transitions or position targets in this monitor; no own `z1_ctrl` process spawn.
4. Prior fuse event: obtain physical electrical/connector/ground checks and clarify whether Z1/B2 share the protection circuit.
5. Establish an independently verified fresh-packet signal from Z1 SDK RX path before permitting use in Interactive 799 observations.
6. For any future hardware test, physical emergency stop, empty workspace, checked Z1 motor current/fault history, and no competing Z1 control clients are required. Do not run this stage's hardware flag until supervised signoff.
7. The final JOINTCTRL motion adapter belongs in M2.4b4 after actual receive verification; it cannot be enabled by simply changing the `z1_sdk` YAML booleans.
