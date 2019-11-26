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

# import sys,tty,termios


import time
import keyboard

connection = None

TEXTWIDTH = 40  # window width, in characters
TEXTHEIGHT = 16  # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 300

VELOCITYMAX = 500
VELOCITYMIN = 125
VELOCITYLAST = 0
VELOCITYDELTA = 20
TIMERDONE = False
OLDTIME = time.time()

helpText = """\
Supported Keys:
P\tPassive
S\tSafe
F\tFull
C\tClean
D\tDock
R\tReset
S\tE-Stop
Space\tBeep
Arrows\tMotion

If nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.
"""


def getch():  # define non-Windows version
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class TetheredDriveApp(Tk):
    # static variables for keyboard callback -- I know, this is icky
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ''

    def __init__(self):
        Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        createMenu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.text = Text(self, height=TEXTHEIGHT, width=TEXTWIDTH, wrap=WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=LEFT, fill=BOTH, expand=True)
        self.scroll.pack(side=RIGHT, fill=Y)

        self.text.insert(END, helpText)

        # self.bind("<Key>", self.callbackKey)
        # self.bind("<KeyRelease>", self.callbackKey)

        # global char
        # char = None
        #        _thread.start_new_thread(self.keypressCB, ())

        self.customstartup()
        #    keyboard.add_hotkey('up')
        keyboard.hook(self.callbackKey)

    # def keyboardCB(self):

    def keypressCB(self):
        global char
        char = getch()

    def customstartup(self):
        self.directConnect("COM6")
        time.sleep(.1)
        self.sendCommandASCII('128')
        self.sendCommandASCII('131')
        self.timer = threading.Timer(0.005, self.timerCB)

    # self.timer.start()

    def timerCB(self):
        global TIMERDONE
        TIMERDONE = True
        self.timer.run()
        print TIMERDONE

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
        global VELOCITYMAX
        global VELOCITYMIN
        global VELOCITYLAST
        global VELOCITYDELTA
        global TIMERDONE
        global OLDTIME
        # k = event.keysym.upper()
        k = event.name
        k = k.upper()
        print"---------------------------"

        motionChange = False
        if event.event_type == 'down':  # KeyPress; need to figure out how to get constant
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
            elif k=='S':
                self.stop()
            else:
                print repr(k), "not handled"
        elif event.event_type == 'up':  # KeyRelease; need to figure out how to get constant
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

            isup = keyboard.is_pressed('up')
            isdown = keyboard.is_pressed('down')
            isleft = keyboard.is_pressed('left')
            isright = keyboard.is_pressed('right')


            velChange = True
            while velChange:
                if self.callbackKeyRight or self.callbackKeyLeft:
                    rot = True
                else:
                    rot = False

                if self.callbackKeyUp:
                    dir = 1
                elif self.callbackKeyDown:
                    dir = -1
                else:
                    dir = 0


                velocity = 0
                velocity += VELOCITYLAST if self.callbackKeyUp is True else 0
                velocity -= VELOCITYLAST if self.callbackKeyDown is True else 0
                rotation = 0
                rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
                rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

                currentTime = time.time()

                if (currentTime - OLDTIME) > 0.03:
                    # if TIMERDONE:
                    print "TIMER DONE, DIR:", dir, "VEL: ", VELOCITYLAST
                    TIMERDONE = False
                    OLDTIME = currentTime
                    if dir == 1:  # accelerate to max
                        if VELOCITYLAST < VELOCITYMAX:
                            velocity = VELOCITYLAST + VELOCITYDELTA
                        else:
                            velocity = VELOCITYMAX
                    elif dir == 0:  # decelerate to 0
                        if VELOCITYLAST > 0:
                            velocity = VELOCITYLAST - VELOCITYDELTA
                        elif VELOCITYLAST < 0:
                            velocity = VELOCITYLAST + VELOCITYDELTA
                        else:
                            velocity = 0
                    elif dir == -1:  # accelerate to - max
                        if VELOCITYLAST > -1 * VELOCITYMAX:
                            velocity = VELOCITYLAST - VELOCITYDELTA
                        else:
                            velocity = -1 * VELOCITYMAX

                    # compute left and right wheel velocities
                    vr = velocity + (rotation / 2)
                    vl = velocity - (rotation / 2)

                    # create drive command
                    cmd = struct.pack(">Bhh", 145, vr, vl)
                    if cmd != self.callbackKeyLastDriveCommand:
                        self.sendCommandRaw(cmd)
                        self.callbackKeyLastDriveCommand = cmd
                        print "COMMAND SEND"

                    VELOCITYLAST = velocity

                    if keyboard.is_pressed('s'):
                        velChange=False
                        velocity=0
                        vl=0
                        vr=0
                        VELOCITYLAST=0
                        self.stop()
                    if abs(VELOCITYLAST) >= VELOCITYMAX or VELOCITYLAST == 0:
                        velChange = False
                    if isup != keyboard.is_pressed('up') or isdown != keyboard.is_pressed(
                            'down') or isleft != keyboard.is_pressed('left') or isright != keyboard.is_pressed('right'):
                        velChange = False

    def stop(self):
        cmd = struct.pack(">Bhh", 145, 0, 0)
        if cmd != self.callbackKeyLastDriveCommand:
            self.sendCommandRaw(cmd)
            self.callbackKeyLastDriveCommand = cmd

    def onConnect(self):
        global connection

        if connection is not None:
            tkMessageBox.showinfo('Oops', "You're already connected!")
            return

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
                tkMessageBox.showinfo('Connected', "Connection succeeded!")
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

    def directConnect(self, port):
        global connection
        self.text.insert(END,("Automatically Connecting to ", port))
        self.text.insert(END, '\n')

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


if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
