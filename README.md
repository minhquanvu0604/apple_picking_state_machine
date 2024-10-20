# apple_picking_state_machine
A revised state machine for the active perception set up in the apple picking project - MVPS
It is based on the state machine module in MVPS, with modifications suitable for my capstone setup

```
roslaunch depth_processing rs_rgbdn.launch publish:=false
```

```
rosrun apple_picking_state_machine apple_picking_state_machine
```

Start the execution loop
```
rosservice call /mvps/state_machine/start 
```

# Dependency
`indicators` https://github.com/p-ranav/indicators?tab=readme-ov-file#building-samples 
Clone the above header-only library into any directory and export its path
```
export INDICATORS_PATH=/path/to/indicators
```


