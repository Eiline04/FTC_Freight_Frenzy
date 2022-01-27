package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.ControllerInput;
import org.firstinspires.ftc.teamcode.Utilities.IntakeWatchdog;
import org.firstinspires.ftc.teamcode.Wrappers.DuckMechanism;
import org.firstinspires.ftc.teamcode.Wrappers.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Lifter;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.MecanumDriveImpl;
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

    //Encoders
    DcMotorEx encoderLeft;
    DcMotorEx encoderRight;
    DcMotorEx encoderHorizontal;

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

        //---------Encoders-------------
        encoderLeft = hardwareMap.get(DcMotorEx.class, "FR");
        encoderRight = hardwareMap.get(DcMotorEx.class, "BR");
        encoderHorizontal = hardwareMap.get(DcMotorEx.class, "FL");

        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();
            lifter.update();
            intakeWatchdog.update();

            telemetry.addData("Left Encoder", encoderLeft.getCurrentPosition());
            telemetry.addData("Right Encoder", encoderRight.getCurrentPosition());
            telemetry.addData("Horizontal Encoder", encoderHorizontal.getCurrentPosition());
            telemetry.update();

            double leftStickY = controller1.left_stick_y;
            double leftStickX = controller1.left_stick_x;
            double rotation = controller1.right_stick_x;

            drive.setWeightedDrivePower(new Pose2d(leftStickY, leftStickX, rotation));

            //Duck Mechanism
            if (controller1.AOnce()) {
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
                baseServoPosition = Range.clip(baseServoPosition + deltaBase, 0.35, 0.85);
                turret.setBasePos(baseServoPosition);
            }

            if (controller1.dpadRight()) {
                //move base right
                baseServoPosition = Range.clip(baseServoPosition - deltaBase, 0.35, 0.85);
                turret.setBasePos(baseServoPosition);
            }

            if (controller1.dpadUp()) {
                //move angle up
                angleServoPosition = Range.clip(angleServoPosition + deltaAngle, 0, 0.35);
                turret.setAnglePos(angleServoPosition);
            }

            if (controller1.dpadDown()) {
                //move angle down
                angleServoPosition = Range.clip(angleServoPosition - deltaAngle, 0, 0.35);
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
            if (controller2.rightBumperOnce()) {
                long waitForTurret = 0;
                if (Math.abs(turret.getBasePos() - 0.95) > 0.1) {
                    turret.setBasePos(0.95);
                    waitForTurret = 700;
                }
                lifter.goToPosition(waitForTurret, 450);
                lifter.intermediateBoxPosition(300 + waitForTurret);
                sleep(600 + waitForTurret);
                lifter.depositMineral();
                lifter.goToPosition(500 + waitForTurret, 0);
            }

            if (controller2.leftBumperOnce()) {
                lifter.closeBox();
                lifter.goToPosition(0, 0);
            }

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