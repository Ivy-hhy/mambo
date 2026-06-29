# Work Log

This file is the running record for work completed in this workspace.
Future entries should be appended after each completed task.

## Entry Format

### YYYY-MM-DD | Short Title
- Project:
- Path:
- Request:
- Changes:
- Result:
- Tech:
- Resume Draft:

## Entries

### 2026-06-28 | Team USB Module Integration Completion
- Project: `team_usb_integrated`
- Path: `/home/hhy/my-workspace/mambo/samples/a/team_usb_integrated`
- Request: Finish wiring the shared USB control packet into the three existing mechanism modules so one controller protocol can drive the combined board firmware.
- Changes: Completed button-to-action mapping for the lift, R1 catch, and `sbus_2` modules; added device readiness checks and module-level init gating; implemented lift gear stepping on A/Y edge triggers; mapped catch actions to dedicated buttons with one-shot behavior; mapped `sbus_2` virtual buttons to M3508/M2006 positions and relay outputs; added throttled runtime logging for packet-driven state visibility.
- Result: The integrated firmware can now fan out one `0x0303` USB control packet format into three previously separate mechanism control paths while tolerating unavailable hardware modules at startup.
- Tech: Zephyr, USB bulk receive path, embedded C, motor drivers, GPIO relay control, input event mapping
- Resume Draft: Integrated three independent robot subsystems behind a shared USB control protocol, adding hardware readiness checks, event-driven button mappings, and runtime observability for a combined embedded control application.

### 2026-06-27 | Shared USB Control Integration Scaffold
- Project: `team_usb_integrated`
- Path: `/home/hhy/my-workspace/mambo/samples/a/team_usb_integrated`
- Request: Create a unified application skeleton that receives one USB control packet and dispatches it to multiple mechanism modules.
- Changes: Added the `team_usb_integrated` app structure, defined a shared control packet model for 7-float payloads and 48 button bits, implemented the common USB bulk receive layer and packet decode path, added a unified main loop that initializes each module independently and dispatches updates, and documented the protocol layout and button assignments in the project README.
- Result: The workspace now has a single integration target that standardizes controller input decoding and gives each subsystem a common interface instead of separate per-project entry points.
- Tech: Zephyr, ARES USB interface, dual protocol sync packets, embedded architecture, interface unification
- Resume Draft: Built a reusable embedded integration layer that standardized USB controller packet decoding and dispatched shared input data across multiple robot subsystems within one Zephyr application.

### 2026-06-26 | USB Control Packet Logger
- Project: `usb_float51_logger`
- Path: `/home/hhy/my-workspace/mambo/samples/a/usb_float51_logger`
- Request: Add a lightweight firmware utility to verify that incoming USB control packets are decoded correctly before connecting them to actuator logic.
- Changes: Implemented a USB packet logger for the 7-float control protocol, decoded the first three floats into 48 button bits, added named button press/release logs, added joystick value logging with change thresholds and rate limiting, and documented the packet format for bring-up use.
- Result: Controller protocol validation can now be done in isolation, making it easier to confirm button and stick mappings before debugging downstream motor or relay behavior.
- Tech: Zephyr logging, USB bulk packets, protocol debugging, embedded tooling, input decoding
- Resume Draft: Developed a firmware-side controller protocol logger that accelerated USB input bring-up by decoding packed button fields and analog stick data into readable diagnostic events.

### 2026-06-27 | Worklog Setup
- Project: workspace process
- Path: `/home/hhy/my-workspace/mambo/samples/a`
- Request: Keep a persistent record of work done here for future resume writing.
- Changes: Added `WORKLOG.md` with a standard entry format for ongoing task tracking.
- Result: Future work in this workspace can now be logged in a consistent, resume-friendly format.
- Tech: documentation, engineering tracking, resume source material
- Resume Draft: Established a structured engineering worklog process to capture project scope, implementation details, and outcomes for later resume and portfolio use.
