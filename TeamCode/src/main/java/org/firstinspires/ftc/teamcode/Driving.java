package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.Utilities.ControllerInput;
import org.firstinspires.ftc.teamcode.Utilities.IntakeWatchdog;
import org.firstinspires.ftc.teamcode.Wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;
import org.firstinspires.ftc.teamcode.Wrappers.MeasuringTapeTurret;

@TeleOp()
public class Driving extends LinearOpMode {
    ControllerInput controller1, controller2;

    MecanumDriveImpl drive;
    Intake intake;
    Lifter lifter;
    IntakeWatchdog intakeWatchdog;
    MeasuringTapeTurret turret;
    DuckMechanism duckMechanism;

    public double baseServoPosition, angleServoPosition;
    public double deltaBase = 0.008, deltaAngle = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDriveImpl(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        turret = new MeasuringTapeTurret(hardwareMap);
        duckMechanism = new DuckMechanism(hardwareMap);
        intakeWatchdog = new IntakeWatchdog(intake, hardwareMap, telemetry, gamepad1, gamepad2);
        intakeWatchdog.enable();

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);
        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();
            lifter.update();
            intakeWatchdog.update();

            telemetry.addData("Ticks", lifter.getLifterPosition());
            telemetry.update();

            double leftStickY = -controller1.left_stick_y;
            double leftStickX = -controller1.left_stick_x;
            double rotation = -controller1.right_stick_x;

            drive.setWeightedDrivePower(new Pose2d(leftStickY, leftStickX, rotation));

            //Duck Mechanism
            if (controller2.YOnce()) {
                //start or stop duck motor
                if (duckMechanism.running) {
                    duckMechanism.stopSpin();
                } else duckMechanism.startSpin();
            }

            //Tape Mechanism
            if (controller1.rightBumper()) {
                turret.startExtend();
            } else if (controller1.leftBumper()) {
                turret.startRetract();
            } else turret.stop();

            if (controller1.dpadLeft()) {
                //move base left
                baseServoPosition = Range.clip(baseServoPosition + deltaBase, 0.1, 1.00);
                turret.setBasePos(baseServoPosition);
            }

            if (controller1.dpadRight()) {
                //move base right
                baseServoPosition = Range.clip(baseServoPosition - deltaBase, 0.1, 1.00);
                turret.setBasePos(baseServoPosition);
            }

            if (controller1.dpadUp()) {
                //move angle up
                angleServoPosition = Range.clip(angleServoPosition + deltaAngle, 0, 0.45);
                turret.setAnglePos(angleServoPosition);
            }

            if (controller1.dpadDown()) {
                //move angle down
                angleServoPosition = Range.clip(angleServoPosition - deltaAngle, 0, 0.45);
                turret.setAnglePos(angleServoPosition);
            }

            //Intake servos
            if (controller2.dpadDownOnce()) {
                intake.lowerIntake();
            }
            if (controller2.dpadUpOnce()) {
                intake.raiseIntake();
                intake.stopIntake();
            }

            //Intake Motor
            if (controller2.AOnce() && !gamepad2.start) {
                intake.lowerIntake();
                sleep(100);
                intake.startIntake();
            }
            if (controller2.BOnce() && !gamepad2.start) {
                intake.stopIntake();
            }
            if (controller2.XOnce()) {
                intake.stopIntake();
                sleep(200);
                intake.reverseIntake();
            }

            //Lifter
            if(controller2.leftBumperOnce()){
                turret.setBasePos(0.98);
                sleep(200);
                lifter.goThird();
                lifter.intermediateBoxPosition(200);
                lifter.depositMineral(500);

                lifter.closeBox();
                sleep(1200);
                lifter.goDown();
            }

//            if (controller2.rightBumperOnce()) {
//                turret.setBasePos(0.98);
//                sleep(200);
//                lifter.goThird();
//                //lifter.goToPosition(0, Lifter.LEVEL.THIRD);
//                lifter.intermediateBoxPosition(200);
//                lifter.depositMineral(500);
//                //lifter.goToPosition(2000, Lifter.LEVEL.DOWN);
//            }
//
//            if (controller2.leftBumperOnce()) {
//                lifter.closeBox();
//                //lifter.goToPosition(0, Lifter.LEVEL.DOWN);
//                sleep(200);
//                lifter.goDown();
//            }

            //-----------MANUAL CONTROL---------------
//            double lifterUp = controller2.right_trigger;
//            double lifterDown = controller2.left_trigger;
//
//            if (lifterUp > 0 && lifterDown == 0) {
//                //go up
//                lifter.setLifterPower(lifterUp * 0.2);
//            } else {
//                if (lifterDown > 0 && lifterUp == 0) {
//                    //go down
//                    lifter.setLifterPower(-lifterDown * 0.2);
//                } else {
//                    //stop
//                    lifter.setLifterPower(0.0);
//                }
//            }

        }
    }

}