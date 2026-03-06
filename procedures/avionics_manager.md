# Avionics Manager Checklist

*Role: Main manager of all electrical systems. Oversees power systems, DAQ, and maintains coordination with Mission Control Manager.*

## Electrical Setup & Overview
- [ ] Inspect overall setup of electrical power systems.
- [ ] Verify DAQ system is physically secured and powered correctly.
- [ ] Confirm power supplies are set to correct voltage/current limits.
- [ ] Ensure DAQ POWER harness is disconnected prior to energizing supply.
- [ ] Ensure telemetry is connected. 
- [ ] Ensure actuator control wiring is secure and correctly wired.
- [ ] Confirm grounding and environmental protection of harnessing.
- [ ] Establish direct communication line with Mission Control Manager.
- [ ] Confirm backup laptops and power sources are available.
- [ ] **TELL MISSION CONTROL MANAGER ALL ELECTRICAL SYSTEMS ARE SET UP.**

## Pre-Test Procedures
- [ ] **BREAK HERE. WAIT FOR OPERATIONS MANAGER TO GIVE THE GO TO BEGIN.**
- [ ] Turn on DAQ power upon command. 
- [ ] Verify DAQ boots up correctly and connects to the telemetry interface.
- [ ] Confirm the power supply is operating in constant voltage mode.
- [ ] Conduct a dry run of all the states in coordination with ECOMM and ACTCOMM. Ensure all states actuate the correct solenoids and ball valves.
- [ ] Confirm all valve actuation commands register properly.
- [ ] Verify that the **ABORT SYSTEM** opens the tank vents during the dry run.
- [ ] Confirm no unexpected current spikes.
- [ ] **TELL MISSION CONTROL MANAGER AVIONICS SIGNAL IS NOMINAL.**

## Hotfire Operations
- [ ] Ensure data rates and sensors are all reading appropriate values with no spikes. 
- [ ] Monitor power consumption during fill and pressurization.
- [ ] **TELL MISSION CONTROL MANAGER AVIONICS IS STEADY FOR IGNITION.**
- [ ] Remain prepared to de-energize immediately upon ABORT call.
- [ ] **BREAK HERE. WAIT FOR MISSION CONTROL MANAGER TO TELL YOU TO DE-ENERGIZE WHEN SAFE.**

## System Safing
- [ ] De-energize power electronics upon command from Mission Control/Pad Manager.
- [ ] Confirm voltage drops to zero.
- [ ] Verify DAQ shutdown sequence is completed properly.
- [ ] Confirm system data files saved with ECOMM.
- [ ] Secure power supplies.
- [ ] **TELL MISSION CONTROL MANAGER AVIONICS ARE DE-ENERGIZED AND DATA SAVED.**
