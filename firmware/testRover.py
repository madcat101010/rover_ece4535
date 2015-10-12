#Tests the rover
#python 2.7.10
#reference: http://stackoverflow.com/questions/676172/full-examples-of-using-pyserial-package
#!/usr/bin/python
import serial, time
#initialization and open the port
#possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call
ser = serial.Serial(6, 57600, timeout=1)
#ser.port = "/dev/ttyUSB0"
#ser.port = "/dev/cu.usbserial-A702ZKVY"
#ser.baudrate = 57600
#ser.bytesize = serial.EIGHTBITS #number of bits per bytes
#ser.parity = serial.PARITY_NONE #set parity check: no parity
#ser.stopbits = serial.STOPBITS_ONE #number of stop bits
#ser.timeout = None          #block read
#ser.timeout = 1            #non-block read
#ser.timeout = 2              #timeout block read
ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 3     #timeout for write
try:
    if ser.isOpen():
        ser.close()
    ser.open()
except Exception, e:
    print "error open serial port: " + str(e)
    exit()
if ser.isOpen():
    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
        #write data
        numOfLines = 0
        count = 0
        speedtest = 1
        while True:
            xmit = "\x80" + chr(numOfLines) + "00010010"
            ser.write(xmit)
            print(str(numOfLines) + " write data: " + xmit)
            if speedtest:
                #sleep 20ms (0.020 seconds) = some messages dropped still
                time.sleep(0.02)  #give the serial port sometime to receive the data
            else:
                response = ser.read(10)
                print("read data: " + response)
                if chr(numOfLines) != response[1]:
                    print "ERROR ON: " + str(numOfLines)
                    break;
                count = count+1
                if (count >= 9999999):
                    break
            numOfLines = numOfLines + 1
            if numOfLines == 127:
                numOfLines = 0
        ser.close()
    except Exception, e1:
        print "error communicating...: " + str(e1)
        ser.close()
else:
    print "cannot open serial port "
