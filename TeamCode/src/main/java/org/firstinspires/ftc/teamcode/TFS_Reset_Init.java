/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "TFS_Reset", group = "FTC2023")
//@Disabled
// Resetting the zero position of the Arm PAN, Tilt (x2)  and Extend motors, before running this code, the robot arm should point straight up and turret pointing forward with marking aligned

public class TFS_Reset_Init extends LinearOpMode {


    boolean                 isPratice = false;
    double                  wrist_Tilt_Offset = -0.025;
    double                  wrist_Flip_Offset = 0.01;
    double                  claw_Left_Offset =  0;
    double                  claw_Right_Offset = 0;

    int                     isLeftSide = 0;
    // Hardware Devices
    BNO055IMU imu;
    DcMotor                 LFMotor;
    DcMotor                 RFMotor;
    DcMotor                 LRMotor;
    DcMotor                 RRMotor;
    DcMotorEx               Arm_PanMotor, Arm_lift_Left_Motor, Arm_lift_Right_Motor;
    DcMotor                 Arm_ExtendMotor;
    Servo ClawServo_Left,ClawServo_Right, Wrist_FlipServo, Wrist_TiltServo;
    TouchSensor             arm_extendSwitch;
    Orientation lastAngles = new Orientation();
    double                  globalAngle;
    int                     lfPosition_last =0, rfPosition_last =0,lrPosition_last =0,rrPosition_last =0;
    double[]                robotPosition = new double[]{0,0,0};
    private ElapsedTime runtime = new ElapsedTime();
    int                     objectDetectionCount =0;

    /******************************Robot & Field Specific Constants ********************************/
    double                  wheel_diameter = 0.095, encoderPPR = 384.5; // 1150RPM with 1:2 Gear downdouble                  turret_Pulse_per_Degree = 384.5*2*70/10/360.0; //version 435RPM Motor with 1:2 bevel gear and 10T to 70T gear chain
    double                  turret_Pulse_per_Degree = 384.5*2*70/10/360.0; //version 435RPM Motor with 1:2 bevel gear and 10T to 70T gear chain
    double                  lift_Pulse_per_Degree = 751.8*2*42/10/360.0; //version 435RPM Motor with 1:2 bevel gear and 10T to 70T gear chain
    double                  Arm_extend_distanceRatio = 2952/0.88; //max extension 74.5cm
    int                     servo_rest_position = 110; //wrist servo max position
    double                  maxExtendArm = 0.6;
    double                  minExtendArm = 0.01;
    double                  speedExtend = 0.1;
    double                  speedLift = 15;
    double                  speedPan = 15;

    double                  maxWristTiltArm = 135;
    double                  minWristTiltArm = -135;
    double                  maxWristFlipArm = 135;
    double                  minWristFlipArm = -135;
    double                  speedWristTilt = 10;
    double                  maxPanArm = 270;
    double                  minPanArm = -270;
    double                  maxLiftArm = 140;
    double                  minLiftArm = -140;
    double                  maxClawArm = 90;
    double                  minClawArm = -30;

    double                  armLiftResolution = 1; //Arm_lift position control resolution
    double                  armPanResolution = 1; //Arm_Pan position control resolution
    double                  armExtendResolution = 0.0025; //Arm_Extend position control resolution

    double                  control1SpeedFactor = 1;
    double                  control2SpeedFactor = 1;
    int                     parkingLocation = 3;

    /************************************ Vision and Landmark *******************************/
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AUQob4f/////AAABmQ82GhD2+k9wr5jVo5UQGnBH/bCXN5Wrz779hXv0waiwot5yzzqtV6VN/g5J4CNGkuQZBL5n2OHp/Gaxs9iBlCm+/gnzFueYuzatB1DORan9mh1PlF89vFtvWfJ1Bz0TUJHNMUv8zlbwpiHSJlZld6pwdyMxfs9UNp/F2CE0hCJ/sCU3dYh7crxnPmOpFJnekTtOWiNZnWx3xk0ZJIm7VCTF+6bsOIrUZYU/X0XHRnkRkGLiqJ1+4a2VSqR+lq6An8m3qKfXuC5nSiGAJFoEWcLdHLPNE+2LcUZHaJQYDLfa5XVl8JQBbQytwIhEOGBeqWMSAWyhCCVTobBNus1/wSy5oz9szorVMRgJBp6XVH9V";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /************************************** Task and thread control Flags ******************************/
    boolean                 isArmOnlyTask = false;
    boolean                 isInAuto = false; // during tele-op, set this to true in your task thread, this prevent multiple semi-auto task
    boolean                 isAutoEnable = true;        // To cancel the auto task, set this to false
    boolean                 isManualOp = false;         //To prevent semi-Auto
    boolean                 Chassis_motorEnabled = true;    // to enable and cancel chassis task
    boolean                 Arm_pan_motorEnabled = true;    // to enable and cancel Turret Pan task
    boolean                 Arm_lift_motorEnabled = true;   // to enable and cancel Arm Lift task
    boolean                 Arm_extend_motorEnabled = true; // to enable and cancel Arm Extend task
    boolean                 isLiftArmProtected = false; //Enable time out and anti-stuck for Arm_lift Joint;
    boolean                 isPanArmProtected = false; //Enable time out and anti-stuck for Arm_Pan Joint;
    boolean                 isExtendArmProtected = false; //Enable time out and anti-stuck for Arm_Extend Joint;


    @Override
    public void runOpMode() {

        Init();
        resetTurret(); // only reset the encoder, make sure you set the turret pointing straight forward manually with markings aligned
        resetLiftArm(); // only reset the encoder, make sure you set the arm straight up.
        resetArm_Extend(); // using sensor, fully retract and sensor light should turn blue when pressed.

        telemetry.addData("Reset Completed, Press Stop Button to EXIT", "!");
        telemetry.addData("Robot is ready to Compete", "!");
        telemetry.addData("Robot is ready to Compete", "!");
        telemetry.update();

//        wrist_Flip_Servo(-90);
//        claw_Servo(0);
//        sleep(3000);
//        claw_Servo(90);
//        sleep(1500);
//        wrist_Tilt_Servo(-120);

//        Arm_Pan_position(25,0.4);
//        Arm_Lift_Position(-142,0.5);
//        Arm_Extend_Position(0.03, 0.3);

        waitForStart();

        while (!isStopRequested()) idle();
        stop();

    }

    void Init() {

        Arm_PanMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        Arm_PanMotor.setDirection(DcMotor.Direction.FORWARD); //version 1, reverse
        Arm_PanMotor.setPositionPIDFCoefficients(5);

        Arm_lift_Left_Motor = hardwareMap.get(DcMotorEx.class, "Left_Shoulder");
        Arm_lift_Left_Motor.setPositionPIDFCoefficients(5);
        Arm_lift_Right_Motor = hardwareMap.get(DcMotorEx.class, "Right_Shoulder");
        Arm_lift_Right_Motor.setPositionPIDFCoefficients(5);
        Arm_lift_Left_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm_lift_Right_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm_ExtendMotor  = hardwareMap.dcMotor.get("Extension");
        Arm_ExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm_PanMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_lift_Left_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_lift_Right_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ClawServo_Left = hardwareMap.get(Servo.class, "Left_Claw");
        ClawServo_Right = hardwareMap.get(Servo.class, "Right_Claw");
        Wrist_TiltServo = hardwareMap.get(Servo.class, "Lift");
        Wrist_FlipServo = hardwareMap.get(Servo.class, "Flip");
        Wrist_TiltServo.setDirection(Servo.Direction.REVERSE);
        Wrist_FlipServo.setDirection(Servo.Direction.REVERSE);

        ClawServo_Left.setDirection(Servo.Direction.FORWARD);
        ClawServo_Left.setDirection(Servo.Direction.REVERSE);

        arm_extendSwitch = hardwareMap.get(TouchSensor.class,"Touch");

    }

    private void resetTurret ()        {
        Arm_PanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void resetLiftArm()        {
        Arm_lift_Left_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_lift_Right_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void resetArm_Extend ()        {
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (!arm_extendSwitch.isPressed())
        {Arm_ExtendMotor.setPower(-0.5);}
        Arm_ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Arm_ExtendMotor.setTargetPosition(10);
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_ExtendMotor.setPower(0.15);
    }
    /*************************** Vision ******************/
    private void initVuforia () {
        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.*/
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod () {
        //     Initialize the TensorFlow Object Detection engine.
        //     0: Ball,  1: Cube, 2: Duck, 3: Marker (duck location tape marker)
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private int  identifyTeamObjectLocation( ) {
        int trialCount = 0;
        int boltCnt = 0;
        int bulbCnt = 0;
        int panelCnt = 0;

        while (opModeIsActive() && trialCount < 3) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    trialCount++;
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);

                        if (recognition.getLabel().equals("1 Bolt")) {
                            boltCnt++;
                        } else if (recognition.getLabel().equals("2 Bulb")) {
                            bulbCnt++;
                        } else if (recognition.getLabel().equals("3 Panel")) {
                            panelCnt++;
                        }
                        telemetry.addData("- Cnt", "%d / %d / %d", boltCnt, bulbCnt, panelCnt);
                    }
                    telemetry.update();
                }
            }
        }
        if (panelCnt >= boltCnt && panelCnt >= bulbCnt)
        {
            return 3; /// location 3
        }else if (boltCnt >= panelCnt && boltCnt >= bulbCnt)
        {
            return 1; /// location 1
        }else // if (bulbCnt >= panelCnt && bulbCnt >= boltCnt)
        {
            return 2; /// location 3
        }

    }

    /*********************Arm Control ****************/
    private void Arm_Pan_position ( double target, double power){
        Arm_pan_motorEnabled = true;
        if (target <minPanArm) target =  minPanArm;
        if (target > maxPanArm) target = maxPanArm;

        Arm_PanMotor.setTargetPosition((int) (target * turret_Pulse_per_Degree));
        Arm_PanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_PanMotor.setPower(power);

        while(Arm_pan_motorEnabled && Math.abs(target-getArmPanPosition()) > armPanResolution) // wait until finish
        {
            //we could do something here to prevent getting stuck
            idle();
        }
        Arm_PanMotor.setPower(0.2);
    }
    private void Arm_Lift_Position ( double target, double power){
        Arm_lift_motorEnabled = true;
        if (target <minLiftArm) target = minLiftArm;
        if (target > maxLiftArm) target = maxLiftArm;

        Arm_lift_Left_Motor.setTargetPosition((int) (target * lift_Pulse_per_Degree));
        Arm_lift_Right_Motor.setTargetPosition((int) (target * lift_Pulse_per_Degree));
        Arm_lift_Left_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_lift_Right_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm_lift_Left_Motor.setPower(power);
        Arm_lift_Right_Motor.setPower(power);

        while (Arm_lift_motorEnabled && Math.abs(target-getArmLiftPosition()) > armLiftResolution) {
            idle();
            //we could do something here to prevent getting stuck
        }
        // Hold the arm at the current position after target is reached
        Arm_lift_Left_Motor.setPower(0.5); // the 0 brake power is not enough
        Arm_lift_Right_Motor.setPower(0.5); // the 0 brake power is not enough
    }

    private void Arm_Extend_Position (double target, double power)        { //target in meter
        Arm_extend_motorEnabled = true;
        if (target <minExtendArm) target = minExtendArm;
        if (target > maxExtendArm) target = maxExtendArm;

        Arm_ExtendMotor.setTargetPosition((int) (target *Arm_extend_distanceRatio));
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_ExtendMotor.setPower(power);

        while (Arm_extend_motorEnabled && Math.abs(target-getArmExtendPosition()) > armExtendResolution)
        {
            idle();
        }
        Arm_ExtendMotor.setPower(0.2); // the 0 brake power is not enough
    }

    private void wrist_Tilt_Servo ( double target)        { //gobilda servo 0.33 per 90 degrees
        if (target > maxWristTiltArm) target = maxWristTiltArm;
        if (target < minWristTiltArm) target = minWristTiltArm;
        double servoIncrement = target / 90 * 0.33;
        Wrist_TiltServo.setPosition(0.5 + wrist_Tilt_Offset + servoIncrement);
    }

    private void wrist_Flip_Servo ( double target)        { //gobilda servo 0.33 per 90 degrees
        if (target > maxWristFlipArm) target = maxWristFlipArm;
        if (target < minWristFlipArm) target = minWristFlipArm;
        double servoIncrement = target / 90 * 0.33;
        Wrist_FlipServo.setPosition(0.5 + wrist_Flip_Offset + servoIncrement);
    }

    private void claw_Servo (double target)
    {
        if (target > maxClawArm) target = maxClawArm;
        if (target < minClawArm) target = minClawArm;
        double servoIncrement = target / 90 * 0.33;
        ClawServo_Left.setPosition(0.5 + claw_Left_Offset + servoIncrement);
        ClawServo_Right.setPosition(0.5 + claw_Right_Offset + servoIncrement);

    }

    private double getArmPanPosition(){return Arm_PanMotor.getCurrentPosition() / turret_Pulse_per_Degree;}
    private double getArmLiftPosition(){return (Arm_lift_Left_Motor.getCurrentPosition() + Arm_lift_Right_Motor.getCurrentPosition() ) /2.0/ lift_Pulse_per_Degree;}
    private double getArmExtendPosition(){return Arm_ExtendMotor.getCurrentPosition() / Arm_extend_distanceRatio;}
    private double getArmWristTiltPosition(){ return 90*(Wrist_TiltServo.getPosition()-(0.5+ wrist_Tilt_Offset))/0.33; }
    private double getArmWristFlipPosition(){ return 90*(Wrist_FlipServo.getPosition()-(0.5+ wrist_Flip_Offset))/0.33; }


}
