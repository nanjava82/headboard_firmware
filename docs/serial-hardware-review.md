# Nano USB / RS-232 Review

## Scope and status

Reviewed the supplied schematic: Arduino Nano classic, U1 ST3232BTR,
J5 RS-232 connector, and the existing LTC firmware. No editable schematic
or PCB file was found under Documents. This is a proposed hardware change,
not a modified PCB or a verified assembled-board result. Firmware is unchanged.

## Confirmed connection conflict

- U1 pin 12 (R1OUT) connects directly to Nano D0/RX (module pin 2).
- Nano's onboard USB-UART transmitter also drives the MCU RX node.
- U1 has no receiver-output enable or shutdown pin. Firmware cannot disable
  this output. Removing the J5 cable does not disconnect R1OUT from D0.
- This topology permits output contention and explains the reported difference
  between standalone Nano USB operation and operation installed in the PCB.
  Confirm the electrical mechanism by isolating R1OUT and retesting.

## Minimum modification for installed-board USB operation

Replace only the connection from U1 pin 12 to the PCB RX net with a removable
link, named JP_USB_RX_ISOLATE here (proposed reference, not an existing part).

```text
U1 pin 12 R1OUT ---- JP_USB_RX_ISOLATE ---- Nano D0/RX
                         OPEN for USB

Nano onboard USB-UART TX ---------------- MCU RX (unchanged)
Nano D1/TX ------------------------------ U1 pin 11 T1IN (unchanged)
```

For a board rework, power everything off and isolate pin 12 from its PCB pad,
or interrupt only the PCB trace between R1OUT and RX. Inspect for shorts before
powering up. Do not interrupt the Nano's own onboard USB-UART connection.

Keep the link OPEN for USB calibration and USB bootloader uploads. If RS-232
reception is unused, leave this connection permanently open. U1 must remain
properly powered when its T1IN is connected to the driven Nano TX line.

Closing the link restores the original circuit, not a guaranteed contention-free
dual-interface design. For reliable RS-232 and USB receive selection, a future
revision must select between both transmit sources before MCU RX; the onboard
USB-UART path inside the Nano must be accounted for. A switch added solely in
the external path guarantees isolation in USB mode only.

Do not use an added series resistor, J5 cable removal, or U1 power removal as
the primary isolation solution. None guarantees that competing logic outputs
are disconnected. U1 power removal can also cause back-powering via signal pins.
Do not move LTC to SoftwareSerial or reuse Timer1 for communication.

## Separate RS-232 supply improvements

ST3232B datasheet Rev 12, Table 9 specifies the following application values
for VCC = 4.5..5.5 V. Map by IC pins, not by capacitor reference names in ST's
example, which differ from this PCB:

| PCB part | Connection | Supplied drawing | ST application value at 5 V |
| --- | --- | --- | --- |
| C1 | U1 pins 1 to 3 | 0.1 uF | 0.047 uF |
| C4 | U1 pins 4 to 5 | 0.1 uF | 0.33 uF |
| C5 | U1 pin 6 to GND | 0.1 uF | 0.33 uF |
| C6 | U1 pin 2 to GND | 0.1 uF | 0.33 uF |
| C3 | U1 pin 16 to GND | 0.1 uF | 0.33 uF |
| C2 | Supply bulk capacitor | 10 uF | Retain |

Use suitably rated ceramic capacitors and short return paths. ST also lists
electrical test conditions with 0.1 uF capacitors; therefore the table difference
alone is not proof of the reported failure. Apply the RX isolation first and
validate independently before changing capacitors.

U1 pin 10/T2IN and pin 8/R2IN are tied to GND in the supplied drawing;
unused outputs are unconnected. No change is needed for that termination.
R5/R6 are 47-ohm resistors on the RS-232 side, not RX isolation resistors on
the Nano side. Leave them unchanged for this fix. D1 is PESD15VL2BT, a 15 V
bidirectional protection device, not a 5 V UART clamp; do not substitute a
5 V unidirectional clamp on the bipolar RS-232 signals.

## Acceptance checks

1. Keep the Nano installed; set the RX link OPEN and leave J5 disconnected.
2. Open the correct USB monitor at 115200 baud. Send U and D separately;
   each must produce RECEIVED then SAVED with a 0.1 change in the requested
   direction. No new T/R/S commands are required.
3. Wait for SAVED, power-cycle, and verify LOADED reports the saved value.
4. Stop the monitor before upload. Upload using the environment matching that
   Nano's bootloader. Require flash verification success, not just build success.
5. Reopen the monitor and repeat U/D. Confirm D9 LTC and D4-to-D7 REC operation
   while installed. Do not infer camera lock from serial output alone.
6. Repeat USB tests with the normal J5 connection present while the RX link
   remains OPEN. Note that Nano TX can still send logs to J5 through U1.

## Sources

- ST3232B datasheet, pin descriptions and Table 9:
  https://www.st.com/resource/en/datasheet/st3232b.pdf
- Arduino upload troubleshooting, disconnect external RX/TX connections:
  https://support.arduino.cc/hc/en-us/articles/4403365313810-If-your-sketch-doesn-t-upload
- PESD15VL2BT datasheet:
  https://assets.nexperia.com/documents/data-sheet/PESD15VL2BT.pdf
