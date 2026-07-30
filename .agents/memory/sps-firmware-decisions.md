---
name: SPS firmware decisions
description: Non-obvious security/protocol decisions for the seismic sensor firmware in esp32/
---

- **SEQ/nonce safety rule:** SEQ (24-bit, nonce base) must never repeat under a key. Node reserves a SEQ range in NVS *before* use (write-ahead, +64), fast-forwards to the NVS bound on every boot regardless of reset cause, and hard-halts TX at SPS_SEQ_MAX. Wrap is prohibited — key rotation (`keygen` + `seqreset`) resets the space. **Why:** brownout/WDT resets can lose RTC memory; nonce reuse breaks ChaCha20-Poly1305. Any future tier (relay ACK sourcing, gateway) issuing SEQs must follow the same write-ahead scheme.
- Relay dedupe stores exact (node_id, seq) as u64 — an earlier 8-bit fold caused cross-node collisions (caught in code review; regression test exists in esp32/test/host).
- Heartbeats are single-shot TX (LoRaLink::sendOnce), alerts use retransmit+ACK (sendReliable). Don't "unify" them.
- Host test suite (gcc, esp32/test/host) is the contract gate — run it after any sps_proto change. Struct memcmp in tests fails on padding; compare fields.
- Hardware note: `taptest` CLI path exists for radio-only bench verification.
