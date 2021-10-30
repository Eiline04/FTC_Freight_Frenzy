package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotorEx intake;
    private int direction = 1;

    public Servo leftServo;
    public Servo rightServo;
    private HardwareMap hardwareMap;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
        rightServo = hardwareMap.get(Servo.class, "intakeRightServo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        setIntakePosition(0.0);
    }

    public void startIntake() {
        intake.setPower(0.5 * direction);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void reverseIntake() {
        direction *= -1;
    }

    public void setIntakePosition(double pos) {
        leftServo.setPosition(pos);
        rightServo.setPosition(1 - pos);
    }

    public void raiseIntake() {
        setIntakePosition(0.35);
    }

    public void lowerIntake() {
        setIntakePosition(0.0);
    }
}
