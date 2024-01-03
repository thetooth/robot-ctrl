# robot-ctrl

Basic EtherCAT control test of 2 axis SCARA robot based on SOEM and ruckig...

A web based visualization tool is available [here](https://github.com/thetooth/robot-gui)

You need a linux platform prepared for realtime, see [here](util/tuned/Readme.md)

And a pair of Delta B3-E servo drives, although you can use any drive after checking the PDO mapping.

Control is done over NATS:

```bash
# Receive status payload (120Hz broadcast rate)
nats sub 'motion.status'
{
    "run": true,
    "alarm": false,
    "state": "Tracking",
    "diagMsg": "Errors should end up here...",
    "dx": -8.082951410415262e-06,
    "dy": 150.00000688454662,
    "dAlpha": 157.97568918665038,
    "dBeta": -135.95137219838105
}
# Start motion tracking
nats pub 'motion.command' '{"command": "start"}'
# Interrupt tracking
nats pub 'motion.command' '{"command": "stop"}'
# Reset homing or alarms
nats pub 'motion.command' '{"command": "reset"}'
# Follow position
nats pub 'motion.command' '{"command": "goto", "position": {"x": 0.0, "y": 150.0}}'
```
