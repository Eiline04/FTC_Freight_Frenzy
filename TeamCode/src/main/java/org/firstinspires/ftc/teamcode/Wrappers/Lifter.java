package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Lifter {
    private DcMotorEx lifter;  //470 ticks max
    private HardwareMap hardwareMap;
    private Servo dumpingBox;

    public Lifter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        dumpingBox = hardwareMap.get(Servo.class, "dumpingBox");

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setPower(0.0);

        MotorConfigurationType motorConfigurationType = lifter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lifter.setMotorType(motorConfigurationType);
    }

    public void setLifterPower(double power) {
        lifter.setPower(power);
    }

    public int getLifterPosition() {
        return lifter.getCurrentPosition();
    }

    public void openBox() {
        dumpingBox.setPosition(0.8);
    }

    public void closeBox() {
        dumpingBox.setPosition(0.0);
    }
}
