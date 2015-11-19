/**
 * CANCMDDC - A DC "command station" for use with MERG CBUS systems
 * Copyright (c) 2015 Mark Riddoch
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 3 of the License
 */

/*
 * This sketch makes use of the MCP_CAN library, modified to accept
 * either an 8MHz clock or a 16MHz clock.
 * Hardware requires are an MCP2515 based CBUS interface and a set
 * of H-Bridges that can be used to drive the DC tracks.
 */
#include <SPI.h>
#include <mcp_can.h>

#define DEBUG        0

#define BASEADDR  9000     // Address of first DC track

/**
 * The following block of #defines configures the pins that are
 * used for various special functions:
 *   CHIPSELECT  is the select pin for the CANBUS interface
 *   INTPIN      is the "interrupt" pin used by the CANBUS interface
 *               to signal that a CANBUS packet is ready to be read
 *   MODESELECT  is the jumper used to determine the operating mode
 */
#define LED         13     // Pin number for the LED
#define CHIPSELECT  10
#define INTPIN       2
#define MODESELECT  14

/*
 * The following are the PWN outputs used to drive the tracks.
 *
 * Pins are used in pairs and fed to H Bridges, the first pair is
 * assigned to the first address and so on.
 */
static int pwmpins[] = {
  3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 0
};

/**
 * The mode select macro tests the jumper int he mode select pin to
 * determine the mode of operation. It returns true if the CANDC is
 * working on a CBUS that does not also have a CANCMD present.
 */
#define  ISSTANDALONE    (digitalRead(MODESELECT) == 1)


int n_tracks = 0;

/**
 * The CBUS interface object
 */
MCP_CAN CAN0(CHIPSELECT);                        // MCP CAN library

#define MAX_SESSIONS  64                         // Max number of sessions we can deal with

struct {
  int      dcc_address;
  uint8_t  speed;
  uint8_t  flags;
} sessions[MAX_SESSIONS];

/**
 * Definitions of the flags bits
 */
#define SF_REV   0x01      // Train is running in reverse
#define SF_FREE  0x80      // The session is currently unused

/**
 * Arduino setup routine
 * Configures the I/O pins, initialises the CANBUS library
 * and sets up the initial session stack
 */
void setup()
{
  int i;
  
  for (i = 0; i < MAX_SESSIONS; i++)
    sessions[i].flags = SF_FREE;
    
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  CAN0.begin(CAN_125KBPS, 8);                   // init can bus : baudrate = 500k 
  pinMode(INTPIN, INPUT);                       // Setting pin 2 for /INT input
  pinMode(MODESELECT, INPUT);
  i = 0;
  n_tracks = 0;
  while (pwmpins[i] != 0)
  {
    pinMode(pwmpins[i], OUTPUT);
    n_tracks++;
    i++;
  }
  n_tracks /= 2;
  if (ISSTANDALONE)
    Serial.print("Standalone ");
  Serial.println("CANDC starting.");
}

/**
 * Arduino loop routine. Essentially look for CBUS messages
 * and dispatch them to the appropriate handlers
 */
void loop()
{
  int id;
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  
    if(!digitalRead(INTPIN))                     // If pin 2 is low, read receive buffer
    {
      CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
      rxId = CAN0.getCanId();                    // Get message ID
      switch (rxBuf[0])
      {
        case 0x21:                              // Release loco command
          releaseLoco(rxBuf[1]);
          break;
          
        case 0x47:                              // Session speed and direction
          locoSpeed(rxBuf[1], rxBuf[2] & 0x80, rxBuf[2] & 0x7f);
          break;
          
        case 0x40:                              // Request loco sesison (RLOC)
           id = rxBuf[2] + ((rxBuf[1] & 0x3f) << 8);
           locoRequest(id, 0);
           break;
           
        case 0x61:                              // Request loco sesison (GLOC)
           id = rxBuf[2] + ((rxBuf[1] & 0x3f) << 8);
           locoRequest(id, rxBuf[3]);
           break;
           
        case 0xE1:                              // PLOC
          id = rxBuf[3] + ((rxBuf[2] & 0x3f) << 8);
          locoSession(rxBuf[1], id,  rxBuf[4] & 0x80, rxBuf[4] & 0x7f);
          break;
          
        case 0x44:                              // Set Speed Step Range
          setSpeedSteps(rxBuf[1],rxBuf[2]);
          break;
          
        case 0x0A:                              // Emergency stop all
          emergencyStopAll();
          break;
          
        default:
 #if DEBUG
          Serial.print("ID: ");
          Serial.print(rxId, HEX);
          Serial.print("  Data: ");
          for(int i = 0; i<len; i++)                // Print each byte of the data
          {
            if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
            {
              Serial.print("0");
            }
            Serial.print(rxBuf[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
#endif
          break;
      }
    }
}

/*
 * A loco release command for the given session
 */
void
releaseLoco(int session)
{
  if (session >= MAX_SESSIONS)
    return;
  sessions[session].flags = SF_FREE;
}

/*
 * A speed and direction packet for a particular session.
 * Validation the session ID is within range and has not be released
 */
void
locoSpeed(int session, int reverse, int speed)
{
  if (session >= MAX_SESSIONS && (sessions[session].flags & SF_FREE) == SF_FREE)
    return;
  sessions[session].speed = speed;
  if (sessions[session].dcc_address >= BASEADDR &&
        sessions[session].dcc_address < BASEADDR + n_tracks)
  {
    setDCTrack(session, reverse, speed);
  }
}

/*
 * A throttle has requested access to a particular loco address
 * This routine is only used if there is no CANCMD on the bus that will
 * allocate sessions.
 */
void
locoRequest(int address, int flags)
{
  int i;
  if (ISSTANDALONE)
  {
    for (i = 0; i < MAX_SESSIONS; i++)
    {
      if (sessions[i].dcc_address == address && (sessions[i].flags & SF_FREE) == 0)
      {
        // Loco is already used in a session
        if (flags == 0)
          sendError(address, 2);    // Send a Taken error
        else if (flags == 1)        // Steal
        {
          sendError(address, 8);
          sessions[i].flags = SF_FREE;
          break;
        }
        else if (flags == 2)        // Share
        {
          sendPLOC(i);
        }
        else
          sendError(address, 7);    // Invalid request
        return;
      }
    }
    
    // If we have got this far then the loco is not in use
    for (i = 0; i < MAX_SESSIONS; i++)
    {
      if ((sessions[i].flags & SF_FREE) == 0)
      {
        locoSession(i, address, 0, 0);
        sendPLOC(i);
        return;
      }
    }
    sendError(address, 1);      // No free slots
  }
}

/*
 * The command station has allocated a session to a locomotive
 */
void
locoSession(int session, int address, int reverse, int speed)
{
  if (session >= MAX_SESSIONS)
    return;
  sessions[session].dcc_address = address;
  sessions[session].speed = speed;
  sessions[session].flags = reverse ? SF_REV : 0;
  if (sessions[session].dcc_address >= BASEADDR &&
        sessions[session].dcc_address < BASEADDR + n_tracks)
  {
    setDCTrack(session, reverse, speed);
  }
}

void
setDCTrack(int session, int reverse, int speed)
{
  int pinidx = (sessions[session].dcc_address - BASEADDR) * 2;
  
#if DEBUG
  Serial.print("Set ");
  Serial.print(session);
  if (reverse)
    Serial.print(" backwards ");
  else
    Serial.print(" forwards ");
  Serial.println(speed);
#endif
  
  if (reverse)
  {
    digitalWrite(pwmpins[pinidx], LOW);
    analogWrite(pwmpins[pinidx+1], speed << 1);
  }
  else
  {
    analogWrite(pwmpins[pinidx], speed << 1);
    digitalWrite(pwmpins[pinidx+1], LOW);
  }
}

/**
 * Send a PLOC message in response to a CAB requesting a session for
 * a DCC address
 */
void
sendPLOC(int session)
{
  unsigned char buf[8];
  
  buf[0] = 0xE1;
  buf[1] = session;
  buf[2] = (sessions[session].dcc_address >> 8) & 0x3f;
  buf[3] = sessions[session].dcc_address & 0xff;
  buf[4] = sessions[session].speed | ((sessions[session].flags & SF_REV) ? 0x80 : 0);
  buf[5] = 0;  // Zero function bytes
  buf[6] = 0;
  buf[7] = 0;
  CAN0.sendMsgBuf(0, 0, 8, buf);
}

/**
 * Send an error packet in response the a loco request
 */
void
sendError(int addr, int code)
{
  unsigned char buf[8];
  
  buf[0] = 0x63;
  buf[1] = (addr >> 8) & 0x3f;
  buf[2] = addr & 0xff;
  buf[3] = code;
  CAN0.sendMsgBuf(0, 0, 4, buf);
}

void
setSpeedSteps(int session, int steps)
{
}

/**
 * Stop all DC tracks
 * Loop over every session and if it is not free set
 * the speed to 0
 */
void
emergencyStopAll()
{
  int i;
  
  for (i = 0; i < MAX_SESSIONS; i++)
  {
    if ((sessions[i].flags & SF_FREE) == 0)
      locoSpeed(i, 0, 0);
  }
}
