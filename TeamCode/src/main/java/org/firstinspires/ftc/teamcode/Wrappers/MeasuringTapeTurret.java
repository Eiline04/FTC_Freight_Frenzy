package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class MeasuringTapeTurret {
    public CRServo extender;
    public Servo baseServo;
    public Servo angleServo;

    public MeasuringTapeTurret(HardwareMap hardwareMap) {
        extender = hardwareMap.get(CRServo.class, "tapeExtender");
        baseServo = hardwareMap.get(Servo.class, "tapeBase");
        angleServo = hardwareMap.get(Servo.class, "tapeAngle");

        extender.setPower(0.0);

        angleServo.setPosition(0.0); //default 0, max 0.25
        baseServo.setPosition(0.63); // default 0.63 , min 0, max 1
    }

    public void startExtend() {
        extender.setPower(1.0);
    }

    public void stop() {
        extender.setPower(0.0);
    }

    public void startRetract() {
        extender.setPower(-1.0);
    }

    public void setAnglePos(double pos) { //up-down
        if (pos > 0.25) angleServo.setPosition(0.25);
        angleServo.setPosition(pos);
    }

    public void setBasePos(double pos) {
        baseServo.setPosition(pos);
    }

}
