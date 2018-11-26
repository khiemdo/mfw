# MotorDude programming hints and thoughts for creating the lib

## Run parameter
- PWM mit 32768 kHz ~ 31 µs cycle time

## Register settings
- Config Reg 0 (R: 0x0000 / W: 0x1000)
    * Commutation blank time CB[1:0] - **0x1** = 100 µs - Should be sufficient - otherwise **0x2** = 400 µs can be used
    * Blank time BT[3:0] - **0x1** = **1** × 400 ns - This value may need to be modified if current trips occur
    * Dead time DT[5:0] - **0x4** = **4** × 50 ns = 200 ns - This should be a save value, switching duration is approx. 20 ns
- Config Reg 1 (R: 0x2000 / W: 0x3000)
    * Current limitation VR[3:0] - **0x5** = (**5** + 1) × 4.125 A = 24.75 A
    * VDS Treshold VT[5:0] - **0x8** = **8** × 25 mV = 200 mV ⇨ 200 mV / 1.6 mOhm = 125 A short circuit current
- Config Reg 2 (R: 0x4000 / W: 0x5000)
    * Fixed off-time PT[4:0] - **0xD** = 10 µs + (**13** × 1.6 µs) = 30.8 µs
- Config Reg 3 (R: 0x6000 / W: 0x7000)
    * IDS - **0x0** = Current limited start-up
    * Startup current limit HQ[3:0] - **0xB** = (**11** + 1) × 4.125 A = 49.5 A
    * Hold-time HT[3:0] - **0x4** = 2 ms + (**4** × 8 ms) = 34 ms
- Config Reg 4 (R: 0x8000 / W: 0x9000)
    * End commutation time EC[3:0] - **0x7** = (**7** + 1) × 0.2 ms = 1.6 ms
    * Start commutation time SC[3:0] - **0x2** = (**2** + 1) × 8 ms = 24 ms
- Config Reg 5 (R: 0xA000 / W: 0xB000)
    * Phase Advance PA[3:0] - **0x8** = **8** × 1.875° = 15°
    * Ramp-up current limit RQ[3:0] - **0x7** = (**7** + 1) × 4.125 A = 33.0 A
    * Ramp rate RR[3:0] - **0x4** = (**4** + 1) × 0.2 ms = 1 ms
- Mask Reg (R: 0xC000 / W: 0xD000)
- Run Reg (R: 0xE000 / W: 0xF000)
    * BEMF Hysteresis BH[1:0] - **0x0** = Auto
    * BEMF Window BW[2:0] - **0x5** = 12.8 µs
    * Enable Stop on Fail ESF - **0x0** = No stop on fail
    * Select DIAG output DG[1:0] - **0x0** = !Fault
    * Restart control RSC - **0x0** = No restart when BEMF lost
    * Brake - **0x0** = No brake
    * Direction of Rotation DIR - **0x0** = Forward
    * Run enable RUN - **0x1** = Start and run