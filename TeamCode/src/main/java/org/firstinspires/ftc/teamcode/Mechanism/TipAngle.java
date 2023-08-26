package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class TipAngle extends HWMap {

    Orientation lastAngle = new Orientation();
    private double globalPitchAngle;

    public enum TIP {
        TIPPING, NOT_TIPPING, ON_STACKS
    }

    private TIP tip = TIP.NOT_TIPPING;

    public TipAngle(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }

    public double getPitch() {
        /*return imu.getAngularOrientation().thirdAngle;*/
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        float deltaPitchAngle = angles.thirdAngle - lastAngle.thirdAngle;//This is subtracting roll angle
//        // It's going to record angles between -180 and 180
//        globalPitchAngle += deltaPitchAngle;
//        lastAngle = angles;
        return angles.thirdAngle;
    }

    public void activateTip(int degForward, int degBackwards) {
        telemetry.addData("Pitch angle: ", getPitch());
        if (getPitch() <= degForward && getPitch() >= 10) {
            //Here it checks if the tip angle exceeds 8 degrees
            telemetry.addData("-", "tip is activated(TIPPING FORWARD)");
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            leftFrontMotor.setPower(1.0);
            rightFrontMotor.setPower(1.0);
        } else if (getPitch() >= degBackwards) {
            telemetry.addData("-", "tip is activated(TIPPING BACKWARD)");
            rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
            rightBackMotor.setPower(1.0);
            leftBackMotor.setPower(1.0);
        } else {
            rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        telemetry.update();
    }


    public TIP getTIP() {
        return tip;
    }

    public void setTIP(TIP tip) {
        this.tip = tip;
    }


}

