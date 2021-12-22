package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp()
public class ManualLifterTest extends LinearOpMode {
    DcMotor lifter;

    @Override
    public void runOpMode() throws InterruptedException {
        lifter = hardwareMap.get(DcMotorEx.class, "lifter");

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setPower(0.0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Mode", lifter.getZeroPowerBehavior());
            telemetry.addData("Ticks", lifter.getCurrentPosition());
            telemetry.update();

            double lifterUp = gamepad2.right_trigger;
            double lifterDown = gamepad2.left_trigger;

            if (lifterUp > 0 && lifterDown == 0) {
                //go up
                lifter.setPower(lifterUp);
            } else {
                if (lifterDown > 0 && lifterUp == 0) {
                    //go down
                    lifter.setPower(-lifterDown * 0.5);
                } else {
                    //stop
                    lifter.setPower(0.0);
                }
            }
        }
    }
}
