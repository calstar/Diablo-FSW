# Avionics Manager Checklist

*Role: Main manager of all electrical systems. Oversees power systems, DAQ, and maintains coordination with Mission Control Manager.*

## Setup
- [ ] Confirm reasonable ambient readings across: - [ ] 7 PTs - [ ] 3 HPTs - [ ] 3 LCs - [ ] 4 TCs - [ ] 4 RTDs
- [ ] Inspect overall setup of electrical power systems: 2 PT boards, load cells, 2 actuator boards (12 volt & 24 volt), TC board, RTD board, and power distribution board.
- [ ] Confirm that the 12 volt solenoids are plugged into the 12 volt board.
- [ ] Confirm that the 24 volt solenoids are plugged into the 24 volt board. 
- [ ] Verify DAQ system is physically secured and powered correctly.
- [ ] Confirm power supplies are set to correct voltage/current limits.
- [ ] Ensure DAQ POWER harness is disconnected prior to energizing supply.
- [ ] Ensure telemetry is connected. 
- [ ] Ensure actuator control wiring is secure and correctly wired. Limit exposure of wires to the environment using sheathing.
- [ ] Confirm grounding and environmental protection of harnessing.
- [ ] Establish direct communication line with Mission Control Manager.
- [ ] Confirm backup laptops and power sources are available.
- [ ] **TELL MISSION CONTROL MANAGER ALL ELECTRICAL SYSTEMS ARE SET UP.**
- [ ] **BREAK HERE. WAIT FOR OPERATIONS MANAGER TO GIVE THE GO TO BEGIN.**
- [ ] Turn on DAQ power upon command. 
- [ ] Verify DAQ boots up correctly and connects to the telemetry interface.
- [ ] Confirm the power supply is operating in constant voltage mode.
- [ ] Ensure the DAQ briefcase is operating properly
- [ ] Confirm the valve actuation commands register properly


## Dry Run + Testing  
- [ ] Coordinate dry run with ECOMM and ACTCOMM.
- [ ] Communicate board status and performance with Mission Control Manager.
- [ ] Ensure that all states actuate the proper solenoids and ball valves.
- [ ] Confirm that all valve actuation commands register properly.
- [ ] Confirm that the **ABORT SYSTEM** opens the tank vents.
- [ ] Confirm no unexpected current spikes.
- [ ] **TELL MISSION CONTROL MANAGER AVIONICS SIGNAL IS NOMINAL.**

## Fill 
- [ ] Monitor all current and voltage levels of the system during fill. Ensure all states are actuating correctly.
- [ ] Monitor temperature sensors for abnormal rise.
- [ ] Watch for sensor dropouts or frozen values.
- [ ] Watch for pressure oscillations or rapid deviations.
- [ ] **HOLD DURING FILL UNTIL GIVEN THE GO BY MISSION CONTROL MANAGER.**

## Hotfire  
- [ ] Ensure data rates and sensors are all reading appropriate values with no spikes. 
- [ ] Monitor power consumption during fill and pressurization.
- [ ] **TELL MISSION CONTROL MANAGER AVIONICS IS STEADY FOR IGNITION.**
- [ ] Remain prepared to de-energize immediately upon ABORT call.
- [ ] **BREAK HERE. WAIT FOR MISSION CONTROL MANAGER TO TELL YOU TO DE-ENERGIZE WHEN SAFE.**
- [ ] De-energize power electronics upon command from Mission Control/Pad Manager.
- [ ] Confirm voltage drops to zero.
- [ ] Verify DAQ shutdown sequence is completed properly.
- [ ] Confirm system data files saved with ECOMM.
- [ ] Secure power supplies.
- [ ] **TELL MISSION CONTROL MANAGER AVIONICS ARE DE-ENERGIZED AND DATA SAVED.**
