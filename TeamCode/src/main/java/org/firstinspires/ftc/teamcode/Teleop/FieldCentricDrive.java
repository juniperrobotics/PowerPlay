package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class FieldCentricDrive extends HWMap {

    public double STRAFE_TOGGLE_FACTOR = 0.5;
    public double ROTATION_TOGGLE_FACTOR = 0.5;
    public Orientation imuMeasure;
    public double firstAngle;
    public double secondAngle;
    public double thirdAngle;
    public double leftBackPower;
    public double rightBackPower;
    public double rightFrontPower;
    public double leftFrontPower;
    public double leftBackEncoder;
    public double rightBackEncoder;
    public double rightFrontEncoder;
    public double leftFrontEncoder;
    public double rotationEffectivness = 0.7;
    public double xyEffectivness = 0.9;
    public float globalPitchAngle;
    private ElapsedTime loopTimer = new ElapsedTime();

    Orientation lastAngle = new Orientation();

    public FieldCentricDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }



    public void addTelemetry(){
        telemetry.addData("Left Front", leftFrontPower);
        telemetry.addData("Right Front", rightFrontPower);
        telemetry.addData("Left Back", leftBackPower);
        telemetry.addData("Right Back", rightBackPower);
        telemetry.addData("Left Front MAH", leftFrontMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Front MAH", rightFrontMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Left Back MAH", leftBackMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Back MAH", rightBackMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }

/*    public float getPitch() {
        Orientation angles = .imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float deltaPitchAngle = angles.thirdAngle - lastAngle.thirdAngle;//This is subtracting roll angle
        // It's going to record angles between -180 and 180
        globalPitchAngle += deltaPitchAngle;
        lastAngle = angles;
        return globalPitchAngle;
    }*/

    public void drive(double gamepadX, double gamepadY, double gamepadRot, boolean rotationToggle, boolean strafeToggle) {
        if (rotationToggle) {
            gamepadRot *= ROTATION_TOGGLE_FACTOR;
        }
        if (strafeToggle) {
            gamepadX *= STRAFE_TOGGLE_FACTOR;
            gamepadY *= STRAFE_TOGGLE_FACTOR;
        }

        // gamepadRot negated because in math, a counterclockwise rotation is positive
        // (think unit circle), but on the controller, we expect the robot to rotate clockwise when
        // we push the stick to the right. Pushing the stick to the right outputs a positive value.
        double turn = -gamepadRot * rotationEffectivness;
        double controllerX = gamepadX * xyEffectivness;
        double controllerY = gamepadY * xyEffectivness;
        double[] controllerVector = {controllerX, controllerY};
//        telemetry.addData("controllerVector[0]: ", controllerVector[0]);
//        telemetry.addData("controllerVector[1]: ", controllerVector[1]);

        imuMeasure = readFromIMU();
        firstAngle = imuMeasure.firstAngle;
        secondAngle = imuMeasure.secondAngle;
        thirdAngle = imuMeasure.thirdAngle;

        double[] rotatedVector = rotate(controllerVector, firstAngle);
        double rotatedX = rotatedVector[0];
        double rotatedY = rotatedVector[1];
//        telemetry.addData("rotatedX: ", rotatedX);
//        telemetry.addData("rotatedY: ", rotatedY);


        double theta = Math.atan2(rotatedY, rotatedX);
//        telemetry.addData("theta: ", theta);
        double power = Math.hypot(rotatedX, rotatedY);
//        telemetry.addData("power: ", power);
        double sin = Math.sin(theta - Math.PI / 4);
//        telemetry.addData("sin: ", sin);
        double cos = Math.cos(theta - Math.PI / 4);
//        telemetry.addData("cos: ", cos);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
//        telemetry.addData("max: ", max);

        leftBackPower = power * sin / max + turn;
        leftFrontPower = power * cos / max + turn;
        rightBackPower = power * cos / max - turn;
        rightFrontPower = power * sin / max - turn;
        // Encoder values (312 RPM Motors)
        leftBackEncoder = leftBackPower;
        leftFrontEncoder = leftFrontPower;
        rightBackEncoder = rightBackPower;
        rightFrontEncoder = rightFrontPower;


        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + Math.abs(turn);
            rightFrontPower /= power + Math.abs(turn);
            leftBackPower /= power + Math.abs(turn);
            rightBackPower /= power + Math.abs(turn);
        }

        leftBackMotor.setPower(leftBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightBackMotor.setPower(rightBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        loopTimer.reset();
    }

    public static double[] rotate(double[] vector, double angle) {
        final double[] newVector = {0, 0};
        newVector[0] = Math.cos(angle) * vector[0] + (-Math.sin(angle)) * vector[1];
        newVector[1] = Math.sin(angle) * vector[0] + Math.cos(angle) * vector[1];
        return newVector;
    }
}