# robot-ctrl

Basic EtherCAT control test of 4 axis SCARA robot based on SOEM and ruckig...

A web based visualization tool is available [here](https://github.com/thetooth/robot-gui)
Inverse kinematics are based on [this](https://www.cs.columbia.edu/~allen/F15/NOTES/invkin.pdf) paper. You can find the implementation in `src/Robot/IK/scara.cpp` and interactive desmos demo [here](https://www.desmos.com/calculator/h8drgyqyyc).

You need a linux platform prepared for realtime:

### tuned profile

Install the tuned service then place the files in this folder into `/etc/tuned/your-profile-name`
You must check that the isolated and general purpose cores listed in `realtime-variables.conf` match your system or it wont boot again.

### Hardware

And some Delta B3-E servo drives, although you can use any drive after checking the PDO mapping.

### Communication

Control is done over NATS:

```bash
# Receive status payload (250Hz broadcast rate)
nats sub 'motion.status'
{
  "alarm": true,
  "diagMsg": "",
  "drives": [
    {
      "actualTorque": 0.0,
      "controlWord": 0,
      "errorCode": 33072,
      "fault": true,
      "followingError": 0.0,
      "lastFault": "Drive 1 error code 33072",
      "slaveID": 1,
      "statusWord": 1560
    },
    ...
  ],
  "ethercat": {
    "compensation": 4,
    "integral": 324,
    "interval": 1000000,
    "sync0": 49160
  },
  "otg": { "result": 0 },
  "pose": {
    "alpha": 96.4540360062657,
    "alphaVelocity": 0.0,
    "beta": -66.03271506126215,
    "betaVelocity": 0.0,
    "phi": 0.0,
    "phiVelocity": 0.0,
    "r": 59.578679054996456,
    "theta": 0.0,
    "thetaVelocity": 0.0,
    "x": 149.98383853124818,
    "y": 300.0034076877155,
    "z": 0.0
  },
  "run": false,
  "state": "Idle"
}
# Start motion tracking
nats pub 'motion.command' '{"command": "start"}'
# Interrupt tracking
nats pub 'motion.command' '{"command": "stop"}'
# Reset homing or alarms
nats pub 'motion.command' '{"command": "reset"}'
# Follow position (time optimal)
nats pub 'motion.command' '{"command":"goto","pose":{"x":150,"y":300,"z":100,"r":0}}'
# Move linearly (indirect, jerk limited)
nats pub 'motion.command' '{"command":"moveLinear", "duration": 5.2, "pose":{"x":150,"y":300,"z":100,"r":0}}'
```
