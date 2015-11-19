# CANCMDDC
A DC Command Station for use on the MERG CBUS. This is based on an Arduino Mega, although others may be used,
an MCP2515 based CANBUS interface and a number of H-Bridges.

The concept is to provide a CBUS interface that can either co-exist with a MERG CANCMD or substitute for a
CANCMD and be used to run DC trains using the MERG CANCAB or JMRI to drive these DC tracks as if they where
DCC locomotives. A jumper is used to determine if this software should mimic the CANCMD or work alongside a
CANCMD on the CBUS.
