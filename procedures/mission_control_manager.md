# Mission Control Manager Checklist

*Role: Main manager in the bunker. Controls state machine, monitors pressures, and directs launch operations.*

## Setup
- [ ] **BREAK HERE. WAIT FOR OPERATIONS MANAGER TO GIVE THE GO TO BEGIN.**
- [ ] Confirm State Machine configuration is loaded correctly.
- [ ] Confirm DAQ is streaming live data.
- [ ] Establish comms with Pad Manager.
- [ ] Establish comms with Avionics Manager.
- [ ] Confirm ECOMM telemetry is live and nominal.
- [ ] Confirm ACTCOMM reports valve control nominal.
- [ ] Command System to **Idle** (close vents) per master procedure.
- [ ] Perform initial Stable Pressure Check.
- [ ] Confirm ambient readings are nominal across all sensors.
- [ ] **WAIT FOR PAD MANAGER, ECOMM, ACTCOMM, GSE MANAGER, AND AVIONICS MANAGER TO CONFIRM SETUP COMPLETE BEFORE PROCEEDING TO DRY RUN & TESTING.**

## Dry Run & Testing
- [ ] **TELL PAD MANAGER TO CONFIRM PAD CLEAR.**
- [ ] Confirm Operations Manager acknowledges clear zone.
- [ ] Command System to **Armed** state.
- [ ] Verify state change confirmation in telemetry.
- [ ] Confirm tank pressures within expected pre-press range.
- [ ] **WAIT FOR ECOMM DRY RUN COMPLETION, ACTCOMM NOMINAL REPORT, AND AVIONICS MANAGER DRY RUN NOMINAL BEFORE PROCEEDING TO PRESS PROOF.**

## Press Proof
- [ ] **TELL OPERATIONS MANAGER YOU ARE ARMING AND PRESSURIZING.**
- [ ] Command **Pressurize System**. Pressurize COPV tank first using medium pressure GSE which will be done manually. This will only be done to 600 psi. Pressurize both tanks to 600 psi. 
- [ ] Monitor tank pressures continuously during pressurization.
- [ ] Watch for overpressure, instability, or unexpected sensor behavior.
- [ ] **BREAK HERE. WAIT FOR TANK PRESSURES TO STABILIZE (STABLE PRESSURE CHECK).**
- [ ] Confirm stabilization flag and nominal pressure values.
- [ ] **WAIT FOR GN2/HIGH PRESS FILL AND ECOMM TO CONFIRM NOMINAL PRESSURES.**
- [ ] Once confirmed all systems are mostly leak proof and all solenouds and ball valves work well, vent the entire system.

## Fill
- [ ] Command state transitions to support Fuel Fill and LOX Fill per master procedure (e.g., **Fuel Fill**, **Armed**, and cryogen fill-related states).
- [ ] Coordinate timing with Pad Manager and Operations Manager for fuel and LOX fill authorizations.
- [ ] Monitor tank levels and pressures via telemetry during fill.
- [ ] **WAIT FOR ETHANOL FILL OPERATOR AND LOX FILL OPERATOR TO CONFIRM FILL COMPLETE BEFORE PROCEEDING TO FIRE.**

## Fire
- [ ] **TELL AVIONICS MANAGER TO PERFORM IGNITION VERBAL CHECK.**
- [ ] Confirm igniter continuity and voltage verification.
- [ ] **IGNITE.**
- [ ] Command System to FIRE (open main valves).
- [ ] Monitor system pressures continuously.
- [ ] Monitor thrust and load cell data.
- [ ] Monitor temperature sensors (TCs, RTDs).
- [ ] Confirm burn duration within expected window.
- [ ] **TELL OPERATIONS MANAGER IF ANOMALY DETECTED.**
- [ ] **EXECUTE ABORT IF NECESSARY.**
- [ ] Confirm system transitions out of burn state appropriately.
- [ ] Command System to VENT (remotely open vents).
- [ ] Monitor pressure decay.
- [ ] **BREAK HERE. WAIT FOR SYSTEM PRESSURE TO BE BELOW 25 PSI.**
- [ ] Confirm all tanks below threshold.
- [ ] **TELL PAD MANAGER SYSTEM IS SAFE TO APPROACH.**
- [ ] Command return to **Idle**.
- [ ] Confirm vents closed after safing complete.
- [ ] **TELL AVIONICS MANAGER TO DE-ENERGIZE POWER ELECTRONICS.**
- [ ] Confirm DAQ shutdown sequence complete.
- [ ] Check data file saves with ECOMM.
- [ ] Confirm redundant backups created if required.
