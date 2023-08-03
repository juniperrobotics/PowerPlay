package org.firstinspires.ftc.teamcode.Teleop;/*
package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Controller;
import org.firstinspires.ftc.teamcode.Mechanism.ConeTransporter;
import org.firstinspires.ftc.teamcode.Mechanism.TipAngle;
import static org.firstinspires.ftc.teamcode.Mechanism.TipAngle.TIP;


import java.util.Objects;

@TeleOp(name = "Tele-op")
public class Teleop extends LinearOpMode {

    // declare class variables here
    private Controller controller;
    private FieldCentricDrive fieldCentricDrive;
    private ConeTransporter coneTransporter;
    private TipAngle tipAngle;
    // Check if B is pressed
    private boolean b_Press = false;
    public int triggerPressCount = 0;
    private boolean stackState = false;

    public void runOpMode() {
        telemetry.clear();
        try {
            // setup
            controller = new Controller(gamepad1, gamepad2);
            fieldCentricDrive = new FieldCentricDrive(telemetry, hardwareMap);
            coneTransporter = new ConeTransporter(telemetry, hardwareMap);


        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
            if (Objects.equals(exception.getMessage(), "The IMU was not initialized")) {
                telemetry.addData("-", "Detected");
                telemetry.update();
            }

        }

        telemetry.addData("Test", "The code has uploaded");
        telemetry.update();
        coneTransporter.init();
        waitForStart();
        while (opModeIsActive()) {
            try {
                controller.update();
                //FIELDCENTER_______________________________________________________________________________
                double gamepadX;
                double gamepadY;
                double gamepadRot;
                boolean rotationToggle = false;
                boolean strafeToggle = false;
                if (tipAngle.getTIP() == TIP.NOT_TIPPING || tipAngle.getTIP() == TIP.ON_STACKS) {
                    if (Math.abs(controller.gamepad1X) > 0.01) {
                        gamepadX = controller.gamepad1X;
                    } else if (Math.abs(controller.gamepad2X) > 0.01) {
                        gamepadX = controller.gamepad2X;
                    } else {
                        gamepadX = 0;
                    }
                    if (Math.abs(controller.gamepad1Y) > 0.01) {
                        gamepadY = controller.gamepad1Y;
                    } else if (Math.abs(controller.gamepad2Y) > 0.01) {
                        gamepadY = controller.gamepad2Y;
                    } else {
                        gamepadY = 0;
                    }
                    if (Math.abs(controller.gamepad1Rot) > 0.01) {
                        gamepadRot = -controller.gamepad1Rot;
                    } else if (Math.abs(controller.gamepad2Rot) > 0.01) {
                        gamepadRot = -controller.gamepad2Rot;
                    } else {
                        gamepadRot = 0;
                    }
                    fieldCentricDrive.drive(gamepadX, gamepadY, gamepadRot, rotationToggle, strafeToggle);
                }

                if (controller.dpadDown){
                    coneTransporter.zeroMode = true;
                }

                //Code for each stack Height
                if (controller.y) {
                    coneTransporter.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    coneTransporter.setHeight(coneTransporter.equate(coneTransporter.LINEAR_SLIDES_HIGH));
                    coneTransporter.ledTimer.reset();
                } else if (controller.a) {
                    coneTransporter.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    coneTransporter.setHeight(coneTransporter.equate(coneTransporter.LINEAR_SLIDES_LOW));
                    coneTransporter.ledTimer.reset();

                } else if (controller.x) {
                    coneTransporter.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    coneTransporter.setHeight(coneTransporter.equate(coneTransporter.LINEAR_SLIDES_MEDIUM));
                    coneTransporter.ledTimer.reset();
                } else if (controller.rightTrigger) {
                    coneTransporter.automation = false;
                    stackState = false;
                    triggerPressCount = 0;
                    coneTransporter.setHeight(coneTransporter.equate(32));
                }

                //This will check if b is pressed if yes then it will check the position of the slides and decide where it should go
                if (controller.b & !b_Press) {
                    coneTransporter.ledTimer.reset();
                    coneTransporter.automation = false;
                    b_Press = true;
                    stackState = false;
                    triggerPressCount = 0;
                    if (coneTransporter.target == coneTransporter.equate(coneTransporter.LINEAR_SLIDES_NORM)) {
                        coneTransporter.setHeight(coneTransporter.equate(coneTransporter.LINEAR_SLIDES_IN_CONE));
                    } else {
                        coneTransporter.setHeight(coneTransporter.equate(coneTransporter.LINEAR_SLIDES_NORM));
                    }
                } else {
                    b_Press = false;
                }

                if (controller.leftTrigger) {
                    if(!stackState && (coneTransporter.linearSlides.getTargetPosition() != coneTransporter.equate(coneTransporter.AUTO_LINEAR_SLIDES_15))){
                        coneTransporter.ledTimer.reset();
                        coneTransporter.automation = false;
                        coneTransporter.setHeight((coneTransporter.equate(coneTransporter.AUTO_LINEAR_SLIDES_15)));
                        stackState = true;
                    } else {
                        coneTransporter.automation = true;
                        coneTransporter.setGripperPosition(1.0);
                        coneTransporter.coneSense();
                        stackState = false;
                    }
                    tipAngle.setTIP(TIP.ON_STACKS);
                }
                coneTransporter.coneSense();

                //GRIPPER__________________________________________________________________________________
                if (controller.leftBumper && !(controller.rightBumper)) {
                    triggerPressCount = 0;
                    coneTransporter.setGripperPosition(.75);
                }

                if (controller.rightBumper && !(controller.leftBumper)) {
                    triggerPressCount = 0;
                    coneTransporter.setGripperPosition(1.0);
                }

                //tipAngle.activateTip();
                //telemetry.addData("-", "tip is activated");
                //telemetry.addData("Pitch:", tipAngle.getPitch());

                fieldCentricDrive.addTelemetry();
                coneTransporter.loop();
                coneTransporter.zeroSlides();

            } catch (Exception exception) {
                telemetry.addLine("Inside of the while loop:");
                telemetry.clear();
                telemetry.addLine(exception.getMessage());
            }
        }
    }
}
*/
