package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Wrappers.Lifter;

@Config
@TeleOp()
public class LifterPIDTuner extends LinearOpMode {

    ControllerInput controller1;
    Lifter lifter;

    public static double kP = 10;
    public static double kI = 0;
    public static double kD = 0;
    public static double f = 0;

    public static int maxHeight = 450;

    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new ControllerInput(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lifter = new Lifter(hardwareMap, telemetry);

        //Default values: kP: 10   kI: 0.05  kD: 0 F: 0

        PIDFCoefficients defaultPID = lifter.getPIDFCoefficients();
        telemetry.addLine("Current Values:");
        telemetry.addData("kP:", defaultPID.p);
        telemetry.addData("kI:", defaultPID.i);
        telemetry.addData("kD:", defaultPID.d);
        telemetry.addData("F:", defaultPID.f);

        telemetry.addLine();
        telemetry.addLine("Press Play when ready");
        telemetry.update();

        waitForStart();

        telemetryThread threadObj = new telemetryThread();
        Thread thread1 = new Thread(threadObj);
        thread1.start();

        while (opModeIsActive()) {
            controller1.update();

            if (controller1.AOnce()) {
                //go up
                PIDFCoefficients newPID = new PIDFCoefficients(kP, kI, kD, f);
                lifter.setPIDFCoefficients(newPID);
                sleep(100);
                lifter.goToPosition(0, maxHeight);
            }

            if (controller1.BOnce()) {
                //go down
                PIDFCoefficients newPID = new PIDFCoefficients(kP, kI, kD, f);
                lifter.setPIDFCoefficients(newPID);
                sleep(100);
                lifter.goToPosition(0, 0);
            }
        }
    }

    class telemetryThread implements Runnable {
        @Override
        public void run() {
            telemetry.log().clear();
            while (opModeIsActive()) {
                lifter.update();
                telemetry.addData("Current Velocity", lifter.getLifterPosition());
                telemetry.update();
                sleep(5);
            }
        }
    }
}
