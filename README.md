# robot-ctrl

Basic EtherCAT control test of 2 axis SCARA robot based on SOEM and ruckig...

You need a linux platform prepared for realtime, linux-rt kernel with the following flags:
```
skew_tick=1 rcu_nocb_poll rcu_nocbs=1-2 nohz=on nohz_full=1-2 kthread_cpus=0 irqaffinity=0 isolcpus=managed_irq,domain,1-2 intel_pstate=disable nosoftlockup tsc=nowatchdog
```
And a pair of Delta B3-E servo drives, although you can use any drive after checking the PDO mapping.

Control is done over NATS
```
nats pub 'motion.command' '{"command": "start"}'
```