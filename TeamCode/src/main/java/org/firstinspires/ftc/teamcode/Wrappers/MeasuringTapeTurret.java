package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MeasuringTapeTurret {
    public CRServo extender; //this might be a DcMotor
    //public Servo yawServo; //LEFT-RIGHT
    public Servo angleServo; //UP-DOWN
    public DcMotorEx encoder;

    public MeasuringTapeTurret(HardwareMap hardwareMap) {
        extender = hardwareMap.get(CRServo.class, "tapeExtender");
        //yawServo = hardwareMap.get();
        angleServo = hardwareMap.get(Servo.class,"tapeAngle");
        //encoder = hardwareMap.get(DcMotorEx.class, "tapeEncoder");

        //encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //encoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extender.setPower(0.0);

        angleServo.setPosition(0.5);
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

//    public int getCurrentPosition() {
//        return encoder.getCurrentPosition();
//    }

    public void setAngleServoPos(double pos) { //up-down
        angleServo.setPosition(pos);
    }

}
