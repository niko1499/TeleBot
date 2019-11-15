#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 27 May 2015

###########################################################################
# Copyright (c) 2015 iRobot Corporation
# http://www.irobot.com/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

from Tkinter import *
import tkMessageBox
import tkSimpleDialog

import struct
import sys, glob  # for listing serial ports

import time
import threading

try:
    import serial
except ImportError:
    tkMessageBox.showerror('Import error', 'Please install pyserial.')
    raise

connection = None

TEXTWIDTH = 40  # window width, in characters
TEXTHEIGHT = 16  # window height, in lines

AccelerationDuration = 3
TimeStep = .25
Ramping = True
VelocityChanged = False

VelocityStep = AccelerationDuration / TimeStep

MaximumVelocity = 500
VELOCITYCHANGE = 200
ROTATIONCHANGE = 300
if VelocityStep == 0:
    VELOCITYCHANGE = MaximumVelocity
helpText = """\
Supported Keys:
P\tPassive
S\tSafe
F\tFull
C\tClean
D\tDock
R\tReset
Space\tBeep
Arrows\tMotion

If nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.
"""


class TetheredDriveApp(Tk):
    # static variables for keyboard callback -- I know, this is icky
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ''

    def __init__(self):
        Tk.__init__(self)
        self.title("RSI-TeleBot")
        self.option_add('*tearOff', FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        createMenu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.statusLBL = Label(self, text="Status")
        self.statusLBL.grid(row=0, column=0)

        self.accelerationLBL = Label(self, text="Seconds reach max velocity:")
        self.accelerationLBL.grid(row=1, column=0)
        self.acceleration = Scale(self, from_=0, to=20, tickinterval=1, orient=HORIZONTAL, length=600)
        self.acceleration.set(2)
        self.acceleration.grid(row=1, column=1)

        self.velocityLBL = Label(self, text="Maximum velocity:")
        self.velocityLBL.grid(row=2, column=0)
        self.velocity = Scale(self, from_=0, to=200, tickinterval=25, orient=HORIZONTAL, length=600)
        self.velocity.set(200)
        self.velocity.grid(row=2, column=1)

        self.text = Text(self, height=TEXTHEIGHT, width=TEXTWIDTH, wrap=WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        # self.text.grid (row=10,column=0)
        # self.scroll.grid (row=10,column=0)

        self.text.insert(END, helpText)

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)
        self.customstartup()

    def rampCB(self):
        VelocityChanged = True
        global VELOCITYCHANGE
        if Ramping == True:
            VELOCITYCHANGE += VelocityStep
        else:
            VELOCITYCHANGE -= VelocityStep

        if VELOCITYCHANGE > MaximumVelocity:
            VELOCITYCHANGE = MaximumVelocity
        if VELOCITYCHANGE < 0:
            VELOCITYCHANGE = 0

        print("CB: ", VELOCITYCHANGE)
        timer = threading.Timer(TimeStep, self.rampCB).start()


        self.move()

    def customstartup(self):
        self.directConnect("COM6")
        time.sleep(.1)
        self.sendCommandASCII('128')
        self.sendCommandASCII('131')
        print "test"
        print TimeStep
        timer = threading.Timer(TimeStep, self.rampCB).start()

    # sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
    def sendCommandASCII(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))

        self.sendCommandRaw(cmd)

    # sendCommandRaw takes a string interpreted as a byte array
    def sendCommandRaw(self, command):
        global connection

        try:
            if connection is not None:
                connection.write(command)
            else:
                tkMessageBox.showerror('Not connected!', 'Not connected to a robot!')
                print "Not connected."
        except serial.SerialException:
            print "Lost connection"
            tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None

        print ' '.join([str(ord(c)) for c in command])
        self.text.insert(END, ' '.join([str(ord(c)) for c in command]))
        self.text.insert(END, '\n')
        self.text.see(END)

    # getDecodedBytes returns a n-byte value decoded using a format string.
    # Whether it blocks is based on how the connection was set up.
    def getDecodedBytes(self, n, fmt):
        global connection

        try:
            return struct.unpack(fmt, connection.read(n))[0]
        except serial.SerialException:
            print "Lost connection"
            tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None
            return None
        except struct.error:
            print "Got unexpected data from serial port."
            return None

    # get8Unsigned returns an 8-bit unsigned value.
    def get8Unsigned(self):
        return getDecodedBytes(1, "B")

    # get8Signed returns an 8-bit signed value.
    def get8Signed(self):
        return getDecodedBytes(1, "b")

    # get16Unsigned returns a 16-bit unsigned value.
    def get16Unsigned(self):
        return getDecodedBytes(2, ">H")

    # get16Signed returns a 16-bit signed value.
    def get16Signed(self):
        return getDecodedBytes(2, ">h")

    # A handler for keyboard events. Feel free to add more!
    def callbackKey(self, event):
        k = event.keysym.upper()
        motionChange = False

        if event.type == '2':  # KeyPress; need to figure out how to get constant
            if k == 'P':  # Passive
                self.sendCommandASCII('128')
            elif k == 'S':  # Safe
                self.sendCommandASCII('131')
            elif k == 'F':  # Full
                self.sendCommandASCII('132')
            elif k == 'C':  # Clean
                self.sendCommandASCII('135')
            elif k == 'D':  # Dock
                self.sendCommandASCII('143')
            elif k == 'SPACE':  # Beep
                self.sendCommandASCII('140 3 1 64 16 141 3')
            elif k == 'R':  # Reset
                self.sendCommandASCII('7')
            elif k == 'UP':
                self.callbackKeyUp = True
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = True
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = True
                motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = True
                motionChange = True
            else:
                print repr(k), "not handled"
        elif event.type == '3':  # KeyRelease; need to figure out how to get constant
            if k == 'UP':
                self.callbackKeyUp = False
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = False
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = False
                motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = False
                motionChange = True

        if motionChange == True:

            if self.callbackKeyDown or self.callbackKeyUp:
                self.Ramping = True
            else:
                self.Ramping = False
            self.move()

            """
            velocity = 0
            velocity += self.VELOCITYCHANGE if self.callbackKeyUp is True else 0
            velocity -= self.VELOCITYCHANGE if self.callbackKeyDown is True else 0
            rotation = 0
            rotation += self.ROTATIONCHANGE if self.callbackKeyLeft is True else 0
            rotation -= self.ROTATIONCHANGE if self.callbackKeyRight is True else 0

            # compute left and right wheel velocities
            vr = velocity + (rotation / 2)
            vl = velocity - (rotation / 2)

            if self.callbackKeyLeft or self.callbackKeyRight:
                self.commandmove(vr, vl)
   
            # create drive command
            cmd = struct.pack(">Bhh", 145, vr, vl)
            if cmd != self.callbackKeyLastDriveCommand:
                self.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd
            """

    def move(self):
        if self.callbackKeyDown or self.callbackKeyUp:
            self.Ramping = True
        else:
            self.Ramping = False

        velocity = 0
        velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
        velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
        rotation = 0
        rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
        rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

        # compute left and right wheel velocities
        vr = velocity + (rotation / 2)
        vl = velocity - (rotation / 2)

        if self.callbackKeyLeft or self.callbackKeyRight:
            self.commandmove(vr, vl)

    def commandmove(self, vr, vl):
        # create drive command
        cmd = struct.pack(">Bhh", 145, vr, vl)
        if cmd != self.callbackKeyLastDriveCommand:
            self.sendCommandRaw(cmd)

    def ramp(self, start, end, time):
        steps = abs(end - start)
        timeStep = time / steps
        timer = threading.Timer(timeStep, )

    def directConnect(self, port):
        global connection

        if connection is not None:
            tkMessageBox.showinfo('Oops', "You're already connected!")
            return

        if port is not None:
            print "Trying " + str(port) + "... "
            try:
                connection = serial.Serial(port, baudrate=115200, timeout=1)
                print "Connected!"
                # tkMessageBox.showinfo('Connected', "Connection succeeded!")
            except:
                print "Failed."
                tkMessageBox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))

    def onConnect(self):
        global connection

        if connection is not None:
            connection = None

            # tkMessageBox.showinfo('Oops', "You're already connected!")
            # return

        try:
            ports = self.getSerialPorts()
            port = tkSimpleDialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        except EnvironmentError:
            port = tkSimpleDialog.askstring('Port?', 'Enter COM port to open.')

        if port is not None:
            print "Trying " + str(port) + "... "
            try:
                connection = serial.Serial(port, baudrate=115200, timeout=1)
                print "Connected!"
                self.s
                # tkMessageBox.showinfo('Connected', "Connection succeeded!")
            except:
                print "Failed."
                tkMessageBox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))

    def onHelp(self):
        tkMessageBox.showinfo('Help', helpText)

    def onQuit(self):
        if tkMessageBox.askyesno('Really?', 'Are you sure you want to quit?'):
            self.destroy()

    def getSerialPorts(self):
        """Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result


if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
