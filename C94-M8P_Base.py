#!/usr/bin/env python
import argparse
from serial import Serial
from pynmeagps import NMEAMessage
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL, SET, POLL
from time import perf_counter, sleep

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


def createCfgPrt(target,baudrate, inUBX=1,inNMEA=0, inRTCM=0, inRTCM3=1,
                 outUBX=1, outNMEA=1, outRTCM=0, outRTCM3=1) -> UBXMessage:
    msg = UBXMessage("CFG","CFG-PRT",SET, portID=target, baudRate=baudrate,
                     inUBX=inUBX, inNMEA=inNMEA, inRTCM=inRTCM, inRTCM3=inRTCM3,
                     outUBX=outUBX, outNMEA=outNMEA, outRTCM=outRTCM, outRTCM3=outRTCM3)
    return msg

def createCfgMsg(msgClass, msgID, rateUART1=0, rateUART2=0,rateUSB=0,rateSPI=0) -> UBXMessage:
    msg = UBXMessage("CFG","CFG-MSG",SET,msgClass=msgClass,msgID=msgID, rateUART1=rateUART1, rateUART2=rateUART2,rateUSB=rateUSB,rateSPI=rateSPI)
    return msg

def createCfgTModeMsg(rcvrMode, svinMinDur=60000, svinAccLimit=10000) -> UBXMessage:
    msg = UBXMessage("CFG","CFG-TMODE3",SET,rcvrMode=rcvrMode,svinMinDur=svinMinDur,svinAccLimit=svinAccLimit)
    return msg

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
    
    args = parser.parse_args()
    port = args.port
    min_dur = args.min_time
    min_acc = args.min_acc

    if args.mode != "SVIN" and args.mode != "FIXED":
        print(f"Invalid mode, expected 'SVIN' or 'FIXED', got '{args.mode}'")
        return



    with Serial(port, 115200, timeout=3) as stream:
        ubr = UBXReader(stream, protfilter=NMEA_PROTOCOL | UBX_PROTOCOL)
        
        cfgUSB = createCfgPrt(Ports.USB, baudrate=115200, inUBX=1,inRTCM3=0,outUBX=1,outRTCM3=1)
        stream.write(cfgUSB.serialize())

        cfgUART1 = createCfgPrt(Ports.UART1, baudrate=19200, inUBX=0,inRTCM3=0,inNMEA=0,outUBX=0,outNMEA=0,outRTCM3=1)
        stream.write(cfgUART1.serialize())
        
        svinSetMSG = createCfgMsg(MSGClass.NAV, msgID=NavMSG.SVIN, rateUSB=1)
        stream.write(svinSetMSG.serialize())

        pvtSetMSG = createCfgMsg(MSGClass.NAV, msgID=NavMSG.PVT, rateUSB=1)
        stream.write(pvtSetMSG.serialize())

        r1005SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1005, rateUSB=1)
        stream.write(r1005SetMSG.serialize())

        r1077SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1077, rateUSB=1)
        stream.write(r1077SetMSG.serialize())

        r1087SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1087, rateUSB=1)
        stream.write(r1087SetMSG.serialize())

        r1230SetMSG = createCfgMsg(MSGClass.RTCM33, msgID=RTCM3MSG.R1230, rateUSB=10)
        stream.write(r1230SetMSG.serialize())

        tmodeSetMSG = createCfgTModeMsg(TMODE3FLAGS.SURVEY_IN,svinMinDur=min_dur,svinAccLimit=min_acc)
        stream.write(tmodeSetMSG.serialize())

        last = float("-inf")
        while True:
            _, parsed_data = ubr.read()
            if parsed_data is not None and not isinstance(parsed_data, NMEAMessage):
                print(parsed_data)
                if isinstance(parsed_data,UBXMessage) and parsed_data.msg_id == "NAV-SVIN":
                    print("OI"*10E3)
            
            now = perf_counter()
            if now - last > 1:
                poll = UBXMessage("CFG","CFG-PRT",POLL,portID=Ports.UART1)
                stream.write(poll.serialize())
                last = now

if __name__ == "__main__":
    main()