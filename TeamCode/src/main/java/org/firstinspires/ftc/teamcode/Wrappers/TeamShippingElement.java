package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamShippingElement {

    public Servo tseGrip;
    public Servo tseArm;
    public Servo tsePlacing;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public TeamShippingElement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        tseGrip = hardwareMap.get(Servo.class, "TSEGrip");
        tseArm = hardwareMap.get(Servo.class, "TSEArm");
        tsePlacing = hardwareMap.get(Servo.class, "TSEPlacing");
        initTSEArm();
        tseGrip(false);
        tsePlacingPos(false);

    }

    public void tsePlacingPos(boolean state) {
        if (!state) tsePlacing.setPosition(0);
        else tsePlacing.setPosition(1);
    }

    public void tseGrip(boolean state) {
        if (!state) tseGrip.setPosition(0.6);
        else tseGrip.setPosition(0.7);
    }

    public void lowerTSEArm() {
        tseArm.setPosition(0.05);
    }

    public void raiseTSEArm() {
        tseArm.setPosition(0.50);
    }

    public void initTSEArm() {
        tseArm.setPosition(0.37);
    }
}