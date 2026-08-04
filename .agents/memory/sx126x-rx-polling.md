---
name: SX126x new-packet detection
description: Never poll getPacketLength() to detect a new packet on SX126x; use the DIO1 RX-done interrupt.
---

## Rule
On SX126x (RadioLib), a new packet must be detected via the DIO1 RX-done interrupt (`setDio1Action` + volatile flag). Never use `getPacketLength()` as a "packet arrived" test.

**Why:** GET_RX_BUFFER_STATUS is not cleared by `readData()`, so after the first reception it stays nonzero forever. Any RX loop gated on it re-reads the stale first packet (which the replay filter then rejects) and never sees new ones — symptom: "works exactly once" (node ACK window heard only the first ACK; same bug earlier in the gateway-bridge).

**How to apply:** clear flag → `setDio1Action(isr)` → `startReceive()`; in the loop wait on the flag, `getPacketLength()` fresh right after the IRQ, `readData`, re-`startReceive()`; `clearDio1Action()` before `standby()` on every exit path.
