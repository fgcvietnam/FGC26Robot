package fgc.vietnam.robot01

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "FGC: Tank Drive", group = "FGC Vietnam")
class TankDriveTeleOp :
    SharedDriveTeleOp(DriveControlMode.TANK)

@TeleOp(name = "FGC: Arcade Drive", group = "FGC Vietnam")
class DrivetrainTeleOp :
    SharedDriveTeleOp(DriveControlMode.ARCADE)

@TeleOp(name = "FGC: Split Arcade Drive", group = "FGC Vietnam")
class DrivetrainSplitArcadeTeleOp :
    SharedDriveTeleOp(DriveControlMode.SPLIT_ARCADE)
