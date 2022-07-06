import os
import threading
import traceback
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log

# Log
log: Log = Log("motor.py")
runtime_log = RuntimeLog("motor.py")

def startThreadMotor(port: str):
    global t_motor
    t_motor.append(threading.Thread(target=startMotor, args=[port]))
    t_motor[len(t_motor)-1].start()


def startMotor(args):
    os.system("./controller.out " + args)


t_motor = []

try:
    startThreadMotor("/dev/ttyACM0")
    startThreadMotor("/dev/ttyACM1")
    while True:
        pass
except Exception as e:
    for u_motor in t_motor:
        u_motor.close()
    log.error(traceback.format_exc())
    runtime_log.error("Motor finished. Check log file.")
