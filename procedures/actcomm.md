# ACTCOMM (Actuator Communications) Checklist

*Role: In charge of actuator setup, pneumatic actuation testing, and ensuring correct NC/NO states.*

## Setup
- [ ] Coordinate with Avionics Manager to ensure all solenoids are correctly wired between the DAQ box and the solenoids.
- [ ] Verify all solenoids are properly mounted, secured, and use the correct hardware (screws, fittings, and brackets).
- [ ] Confirm pneumatic supply lines are correctly routed and labeled.
- [ ] Verify solenoid valve flow directions match P&ID.
- [ ] Confirm correct Normally Closed (NC) / Normally Open (NO) state for each valve.
- [ ] Manually cycle pneumatic actuators to confirm full open/close travel.
- [ ] Confirm no binding or mechanical interference.
- [ ] Inspect fittings for leaks after pneumatic cycling.
- [ ] Verify no audible leaks, no pressure decays, or fitting movement/backing out during cycling.
- [ ] Confirm electrical connectors to solenoids are secure.
- [ ] Confirm each actuator is tagged/identifiable.
- [ ] **TELL GSE MANAGER ACTUATORS ARE CONNECTED PROPERLY.**
- [ ] **TELL PAD MANAGER ACTCOMM HARDWARE IS READY.**

## Dry Run & Testing
- [ ] **BREAK HERE. WAIT FOR MISSION CONTROL MANAGER TO COMMENCE REMOTE OPERATIONS / HOTFIRE ATTEMPT.**
- [ ] Actively communicate with Mission Control Team to confirm all actuators are correctly mapped and respond to the intended commands.
- [ ] Monitor solenoid actuation during dry run.
- [ ] For every command, verify the correct actuator responded and the actuator moved in the correct direction. 
- [ ] Confirm the resulting physical valve state matches the commanded state, including that all ball valves driven by pneumatic actuators rotate the full 90° between open and closed.
- [ ] Confirm that no uncommanded neighboring hardware moved.
- [ ] Verify actuation timing is nominal.
- [ ] Verify state transitions twice during dry run.
- [ ] Confirm abort state forces valves to correct safe positions.
- [ ] **TELL MISSION CONTROL / AVIONICS MANAGER OF ANY ACTUATOR ANOMALY.**

## Press Proof
- [ ] Verify all rocket-side solenoids remain physically connected.
- [ ] Verify all ground system solenoids remain physically connected.
- [ ] Support GSE team in connecting and verifying all pneumatic hose lines; confirm hoses are properly seated, secured, and clear of snag/heat/motion hazards.
- [ ] Perform final visual inspection with GSE Manager of fittings, manifolds, connectors, harness routing, brackets/mounts, and clearance around all moving hardware.
- [ ] Confirm no tools remain on actuator hardware.
- [ ] Verify all previously noted anomalies (if any) are fully resolved. 
- [ ] **TELL PAD MANAGER ACTCOMM IS SECURE FOR HOTFIRE.**

## Fill
- [ ] **WAIT FOR MISSION CONTROL MANAGER TO PROCEED TO FILL.**

## Fire
- [ ] **WAIT FOR MISSION CONTROL MANAGER TO PROCEED TO FIRE.**