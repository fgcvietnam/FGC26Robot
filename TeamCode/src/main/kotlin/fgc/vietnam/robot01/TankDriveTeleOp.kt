package fgc.vietnam.robot01

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "BLUE: Competition teleop", group = "FGC Vietnam")
class BlueCompTeleOp : CompDriveTeleOp(Alliance.BLUE)

@TeleOp(name = "RED: Competition teleop", group = "FGC Vietnam")
class RedCompTeleOp : CompDriveTeleOp(Alliance.RED)
