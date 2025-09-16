#!/usr/bin/env python
import json
import argparse
import numpy as np
from serial import Serial
from pynmeagps import NMEAMessage
# from time import perf_counter, sleep
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL, SET, POLL
from enum import Enum
import time

class Ports():
    I2C   = 0
    UART1 = 1
    UART2 = 2
    USB   = 3
    SPI   = 4

class MSGClass():
    NAV    =  0x1
    RTCM33 = 0xF5

class NavMSG():
    SVIN = 0x3B
    PVT  = 0x07

class RTCM3MSG():
    R1005 = 0x05
    R1077 = 0x4D
    R1087 = 0x57
    R1230 = 0xE6

class TMODE3FLAGS():
    DISABLED  = 0
    SURVEY_IN = 1
    FIXED_MODE = 2

class SVINSTATE(Enum):
    STARTING = 1
    RUNNING = 2
    DONE = 3


def createCfgPrt(target,baudrate, inUBX=1,inNMEA=0, inRTCM=0, inRTCM3=1,
                 outUBX=1, outNMEA=1, outRTCM=0, outRTCM3=1) -> UBXMessage:
    msg = UBXMessage("CFG","CFG-PRT",SET, portID=target, baudRate=baudrate,
                     inUBX=inUBX, inNMEA=inNMEA, inRTCM=inRTCM, inRTCM3=inRTCM3,
                     outUBX=outUBX, outNMEA=outNMEA, outRTCM=outRTCM, outRTCM3=outRTCM3,
                     parity=4, charLen=3,nStopBits=0)
    return msg

def createCfgMsg(msgClass, msgID, rateUART1=0, rateUART2=0,rateUSB=0,rateSPI=0) -> UBXMessage:
    msg = UBXMessage("CFG","CFG-MSG",SET,msgClass=msgClass,msgID=msgID, rateUART1=rateUART1, rateUART2=rateUART2,rateUSB=rateUSB,rateSPI=rateSPI,
                     )
    return msg

def createCfgTModeMsg(rcvrMode, svinMinDur=60000, svinAccLimit=10000,
                      ecefX=0, ecefY=0,ecefZ=0,
                      ecefXHP=0, ecefYHP=0, ecefZHP=0, fixAcc=0) -> UBXMessage:

    msg = UBXMessage("CFG","CFG-TMODE3",SET,rcvrMode=rcvrMode,svinMinDur=svinMinDur,svinAccLimit=svinAccLimit,
                     ecefXOrLat=ecefX, ecefYOrLon=ecefY, ecefZOrAlt=ecefZ,
                     ecefXOrLatHP=ecefXHP, ecefYOrLonHP=ecefYHP, ecefZOrAltHP=ecefZHP,
                     fixedPosAcc=fixAcc)
    return msg

def navSinToJson(msg : UBXMessage):
    data = {}
    data["ecefX"] = msg.meanX
    data["ecefY"] = msg.meanY
    data["ecefZ"] = msg.meanZ
    data["ecefXHP"] = msg.meanXHP
    data["ecefYHP"] = msg.meanYHP
    data["ecefZHP"] = msg.meanZHP
    data["fixAcc"] = msg.meanAcc

    timestamp = time.strftime("%Y-%m-%d-%H-%M-%S")
    file_name = f"{timestamp}.json"
    with open(file_name,"w") as f:
        json.dump(data,f)

    return

def main()-> None:
    parser = argparse.ArgumentParser(
        prog="C94-M8P_Base",
        description="This program configures an C94-M8P board as a base, and starts survey in, or sets the fixed mode.",
        epilog="In survey in, both conditions must be true to the survey be completed. (fixed mode not implemented yet)"
    )

    parser.add_argument("port",
                        help="Serial port where the board is connected.",
                        type=str)
    parser.add_argument("mode",
                        help='''
                                Position setting mode. If SVIN for survey-in, and fixed for fixed mode.
                                In fixed mode, --ecefX, --ecefY, --ecefZ and --fix_acc must be specified.
                            ''',
                            type=str
                        )
    parser.add_argument("-t","--min_time",
                        default=600,
                        type=int,
                        help="Minimum observation time on survey in. Unit: seconds. Default: 600."
                        )
    
    parser.add_argument("-a","--min_acc",
                        default=10000,
                        type=int,
                        help="Minimum accurary of survey in position. Unit: 0.1mm. Default: 10000."
                        )
    
    parser.add_argument("-f","--filename",
                        default="",
                        type=str,
                        help="Input file with base data."
                        )

    
    

    args    = parser.parse_args()
    port    = args.port
    min_dur = args.min_time
    min_acc = args.min_acc
    ecefX   = 0
    ecefY   = 0
    ecefZ   = 0
    ecefXHP = 0
    ecefYHP = 0
    ecefZHP = 0
    fixACC  = 0

    if args.mode != "SVIN" and args.mode != "FIXED":
        print(f"Invalid mode, expected 'SVIN' or 'FIXED', got '{args.mode}'")
        return
    
    if args.mode == "FIXED":
        if args.filename == "":
            print("File must be explicit set when using fixed mode.")
            return
        with open(args.filename) as f:
            data = json.load(f)
            ecefX   = data["ecefX"]
            ecefY   = data["ecefY"]
            ecefZ   = data["ecefZ"]
            ecefXHP = data["ecefXHP"]
            ecefYHP = data["ecefYHP"]
            ecefZHP = data["ecefZHP"]
            fixACC  = data["fixAcc"]
            
    tmodeSetMSG = createCfgTModeMsg(TMODE3FLAGS.SURVEY_IN if args.mode == "SVIN" else TMODE3FLAGS.FIXED_MODE,svinMinDur=min_dur,svinAccLimit=min_acc,
                                        ecefX=ecefX, ecefY=ecefY, ecefZ=ecefZ,
                                        ecefXHP=ecefXHP, ecefYHP=ecefYHP, ecefZHP=ecefZHP,
                                        fixAcc=fixACC)

    state : SVINSTATE = SVINSTATE.STARTING

    with Serial(port, 115200, timeout=3) as stream:
        ubr = UBXReader(stream, protfilter=NMEA_PROTOCOL | UBX_PROTOCOL)
        
        cfgUART1 = createCfgPrt(Ports.UART1, baudrate=19200, inUBX=0,inRTCM3=0,inNMEA=0,outUBX=0,outNMEA=0,outRTCM3=1)
        # print(cfgUART1.serialize())
        # return
        stream.write(cfgUART1.serialize())
        
        cfgUSB = createCfgPrt(Ports.USB, baudrate=115200, inUBX=1,inRTCM3=0,outUBX=1,outRTCM3=1)
        stream.write(cfgUSB.serialize())


        svinSetMSG = createCfgMsg(MSGClass.NAV, msgID=NavMSG.SVIN, rateUSB=1)
        stream.write(svinSetMSG.serialize())

        pvtSetMSG = createCfgMsg(MSGClass.NAV, msgID=NavMSG.PVT, rateUSB=1)
        stream.write(pvtSetMSG.serialize())

        r1005SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1005, rateUART1=1)
        stream.write(r1005SetMSG.serialize())

        r1077SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1077, rateUART1=1)
        stream.write(r1077SetMSG.serialize())

        r1087SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1087, rateUART1=1)
        stream.write(r1087SetMSG.serialize())

        r1230SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1230, rateUART1=10)
        stream.write(r1230SetMSG.serialize())

        tmodeSetMSG = createCfgTModeMsg(TMODE3FLAGS.SURVEY_IN if args.mode == "SVIN" else TMODE3FLAGS.FIXED_MODE,svinMinDur=min_dur,svinAccLimit=min_acc,
                                        ecefX=ecefX, ecefY=ecefY, ecefZ=ecefZ,
                                        ecefXHP=ecefXHP, ecefYHP=ecefYHP, ecefZHP=ecefZHP,
                                        fixAcc=fixACC)
        stream.write(tmodeSetMSG.serialize())


        # last = float("-inf")
        while True:
            _, parsed_data = ubr.read()
            if parsed_data is not None and not isinstance(parsed_data, NMEAMessage):
                print(parsed_data)
                if parsed_data.identity == "NAV-SVIN":
                    match state:
                        case SVINSTATE.STARTING:
                            if parsed_data.active == 1:
                                state = SVINSTATE.RUNNING
                        case SVINSTATE.RUNNING:
                            if parsed_data.active == 0:
                                navSinToJson(parsed_data)
                                state = SVINSTATE.DONE
                # if isinstance(parsed_data,UBXMessage) and parsed_data.msg_id == "NAV-SVIN":
                #     print("OI"*10E3)
            
            # now = perf_counter()
            # if now - last > 1:
            #     poll = UBXMessage("CFG","CFG-PRT",POLL,portID=Ports.UART1)
            #     stream.write(poll.serialize())
            #     last = now

if __name__ == "__main__":
    main()