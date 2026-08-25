package FGC.Vietnam

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "BLUE: Competition teleop", group = "FGC Vietnam")
class BlueCompTeleOp : CompDriveTeleOp(Alliance.BLUE)

@TeleOp(name = "RED: Competition teleop", group = "FGC Vietnam")
class RedCompTeleOp : CompDriveTeleOp(Alliance.RED)

@TeleOp(name = "BLUE: Transfer Test Datalog TeleOp", group = "Datalog")
class BlueTransferTestTeleOp : CompDriveTeleOp(Alliance.BLUE, transferDatalogOnly = true)

@TeleOp(name = "RED: Transfer Test Datalog TeleOp", group = "Datalog")
class RedTransferTestTeleOp : CompDriveTeleOp(Alliance.RED, transferDatalogOnly = true)

@TeleOp(name = "Transfer Test Datalog TeleOp", group = "Datalog")
class TransferTestTeleOp : CompDriveTeleOp(Alliance.BLUE, transferDatalogOnly = true)
