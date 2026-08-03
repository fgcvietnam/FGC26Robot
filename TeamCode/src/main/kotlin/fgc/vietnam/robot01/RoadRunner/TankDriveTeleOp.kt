package fgc.vietnam.robot01.RoadRunner

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import fgc.vietnam.robot01.DriveControlMode
import fgc.vietnam.robot01.SharedDriveTeleOp

@TeleOp(name = "FGC: Tank Drive", group = "FGC Vietnam")
class TankDriveTeleOp :
    SharedDriveTeleOp(DriveControlMode.TANK)
