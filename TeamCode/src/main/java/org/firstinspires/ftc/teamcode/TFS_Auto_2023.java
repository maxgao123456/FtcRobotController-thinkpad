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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "TFS_Auto", group = "FTC2023")
//@Disabled
public class TFS_Auto_2023 extends LinearOpMode
{


    double                  wrist_Tilt_Offset = -0.025;
    double                  wrist_Flip_Offset = 0.01;
    double                  claw_Left_Offset =  0;
    double                  claw_Right_Offset = 0;
    int                  scannedturretangle = 78;
    int                   scannedturretanglepole = 29 ;

    int                     isLeftSide = 0;
    int                     stackCount = 0;

    // Hardware Devices
    BNO055IMU               imu;
    DcMotor                 LFMotor;
    DcMotor                 RFMotor;
    DcMotor                 LRMotor;
    DcMotor                 RRMotor;
    DcMotorEx               Arm_PanMotor, Arm_lift_Left_Motor, Arm_lift_Right_Motor;
    DcMotor                 Arm_ExtendMotor;
    Servo                   ClawServo_Left,ClawServo_Right, Wrist_FlipServo, Wrist_TiltServo;
    TouchSensor             arm_extendSwitch;
    DistanceSensor          rightd,leftd,centerd;

    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    int                     lfPosition_last =0, rfPosition_last =0,lrPosition_last =0,rrPosition_last =0;
    double[]                robotPosition = new double[]{0,0,0};
    private ElapsedTime     runtime = new ElapsedTime();


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
    double                  scanextenddroppos = 0.35; //-29
    int                     parkingLocation = 1;


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

    /*********************************** Thread Tasks   *************************************/

    goXYnTurnThread goXYnTurnThread = null;
    Arm_Lift_PositionThread  arm_liftThread = null;
    Arm_Pan_PositionThread  arm_panThread = null;
    Arm_Extend_PositionThread arm_extendThread = null;
    Arm_Wrist_PositionThread arm_wristThread = null;
    ScanConeThread scanCone = null;
    ScanJunctionThread scanJunction = null;

    /************************************** Task and thread control Flags ******************************/

    boolean                 isInAuto = false; // during tele-op, set this to true in your task thread, this prevent multiple semi-auto task
    boolean                 Chassis_motorEnabled = true;    // to enable and cancel chassis task
    boolean                 Arm_pan_motorEnabled = true;    // to enable and cancel Turret Pan task
    boolean                 Arm_lift_motorEnabled = true;   // to enable and cancel Arm Lift task
    boolean                 Arm_extend_motorEnabled = true; // to enable and cancel Arm Extend task
    boolean                 scanFlag = false;


    @Override public void runOpMode() {

        telemetry.addData(":@@@@@@@@@@@@@@@@@@@@@@@ ","" );
        telemetry.addLine();
        telemetry.addData("Press X for ","Left Side" );
        telemetry.addData("Press B for ","Right Side" );
        telemetry.addLine();
        telemetry.addData(":@@@@@@@@@@@@@@@@@@@@@@@@ ","" );

        telemetry.update();

        while (isLeftSide ==0 && !isStopRequested())
        {
            if (gamepad1.x || gamepad2.x) {
                isLeftSide = 1;
                telemetry.addData("Program set to ","Left Side" );
                telemetry.update();
                break;
            }
            if (gamepad1.b || gamepad2.b) {
                isLeftSide = -1;
                telemetry.addData("Program set to ","Right Side" );
                telemetry.update();
                break;
            }
            sleep(50);
        }

        Init();
        claw_Close();
        // Starting Routine Tasks
        resetAngle();
        resetEncoders();


        routineTaskThread  routineTaskThread = new routineTaskThread();
        routineTaskThread.start();
        // only for testing, in real game, you should only identify the object after pressing START button
        parkingLocation = identifyTeamObjectLocation();
        telemetry.addData("Parking Loc:", parkingLocation);

        telemetry.addData("WAITING FOR ", "START");
        telemetry.update();
        if(isLeftSide == -1){
            scannedturretangle = -78;
            scannedturretanglepole = -29;

        }

        waitForStart();
        runtime.reset();

        // Identify the parking spot
        parkingLocation = identifyTeamObjectLocation();
        telemetry.addData("Parking Loc:", parkingLocation);

//        // Hold the robot at the current position or go to a desired position , otherwise tele-operation will move the arm
//        Arm_Lift_Position(getArmLiftPosition(), 0.5);
//        Arm_Pan_position(getArmPanPosition(),0.5);
//        Arm_Extend_Position(getArmExtendPosition(),0.5);

        while (opModeIsActive()) {
            telemetry.addData("hLeftDis", "%.2f", leftd.getDistance(DistanceUnit.MM));
            telemetry.addData("hRightDis", "%.2f", rightd.getDistance(DistanceUnit.MM));
            telemetry.addData("Scanned Angle",scannedturretangle);
            telemetry.update();

            // Auto Tasks come here //
            autoPreloadedConetoPoleTask();
            // 1st cone from stack
            autoscanconepickup();

            autoConeStacktoHighPoleTask();
            // 2nd cone from stack
            autoPickupTask();
            autoConeStacktoHighPoleTask();
            // 3rd cone from stack
            autoPickupTask();
            autoConeStacktoHighPoleTask();
            // 4th cone from stack
            autoPickupTask();
            autoConeStacktoHighPoleTask();

            //Bring the arm at up right rest positions
            armUpRightPosition();

            // Move back onto the second tile row and prepare to park
            goXYnTurnThread = new goXYnTurnThread(0, 0.75, 0, 0.5, 0.01, 0.05, 1,0,true,true,0);
            goXYnTurnThread.start();
            while (!isStopRequested() && goXYnTurnThread.isAlive()) idle();

            if (parkingLocation ==2) // "2" Mid Parking
            {
                //goXYnTurnThread = new goXYnTurnThread(0, 0.68, 0, 0.5, 0.01, 0.05, 1,0,true,true,0);
                //goXYnTurnThread.start();

            } else if (parkingLocation == 1) // "1" Left Parking
            {
                goXYnTurnThread = new goXYnTurnThread(0, 0.75, 0.6, 0.5, 0.01, 0.05, 1,0,true,true,0);
                goXYnTurnThread.start();
            } else  // "3" Right Parking
            {
                goXYnTurnThread = new goXYnTurnThread(0, 0.75, -0.6, 0.5, 0.01, 0.05, 1,0,true,true,0);
                goXYnTurnThread.start();
            }
            telemetry.addData(" Finished at ", runtime.milliseconds() / 1000);
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) idle(); // wait for time end
        }
    }

    /*************** Utilities for Resetting *******************************/
    void Init() {

        LFMotor = hardwareMap.dcMotor.get("lf");
        RFMotor = hardwareMap.dcMotor.get("rf");
        LRMotor = hardwareMap.dcMotor.get("lr");
        RRMotor = hardwareMap.dcMotor.get("rr");
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        LRMotor.setDirection(DcMotor.Direction.REVERSE);
        RRMotor.setDirection(DcMotor.Direction.FORWARD);
        RRMotor.setDirection(DcMotor.Direction.FORWARD);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        leftd = hardwareMap.get(DistanceSensor.class, "Left_D");
        centerd = hardwareMap.get(DistanceSensor.class, "Center_D");
        rightd = hardwareMap.get(DistanceSensor.class, "Right_D");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Check if the joystick is operating properly
        while (Math.abs(gamepad1.left_stick_x) >0.01 || Math.abs(gamepad1.left_stick_y) >0.01 || Math.abs(gamepad2.left_stick_x) >0.01 || Math.abs(gamepad2.left_stick_y) >0.01 || Math.abs(gamepad1.right_stick_x) >0.01 || Math.abs(gamepad1.right_stick_y) >0.01 ||Math.abs(gamepad2.right_stick_x) >0.01 ||Math.abs(gamepad2.right_stick_y) >0.01 )
        {
            telemetry.addData(" Gamepad ERROR" , "Restart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Restart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Restart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Restart the Driver Hub");
            telemetry.addData(" Gamepad ERROR" , "Restart the Driver Hub");
            telemetry.update();
        }
        //************************************************
        telemetry.addData(" Gamepad Ready" , "!");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating Imu...");

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("imu Initialization Completed", "!");


        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.8, 1.5 / 1);
        }
        sleep(500);
        telemetry.addData("Vuforia and TensorFlow", " Initialized");

    }
    private void resetTurret ()        {
        Arm_PanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret reset done
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

    private void localization (){
        //estimate robot location on the field using forward kinematic, DOES NOT assume "no turning"
        double Y_Adjustment = 0.85; // lost of distance due to slipping
        int deltalfPosition = LFMotor.getCurrentPosition()-lfPosition_last;
        int deltarfPosition = RFMotor.getCurrentPosition()-rfPosition_last;
        int deltalrPosition = LRMotor.getCurrentPosition()-lrPosition_last;
        int deltarrPosition = RRMotor.getCurrentPosition()-rrPosition_last;

        lfPosition_last += deltalfPosition;
        rfPosition_last += deltarfPosition;
        lrPosition_last += deltalrPosition;
        rrPosition_last += deltarrPosition;

        double angle = Math.toRadians(getAngle());

        double deltaXLocal = 0.25 * (deltalfPosition + deltarfPosition + deltalrPosition + deltarrPosition) / encoderPPR * 3.14 * wheel_diameter;
        double deltaYLocal = Y_Adjustment * 0.25 * (-deltalfPosition + deltarfPosition + deltalrPosition - deltarrPosition) / encoderPPR * 3.14 * wheel_diameter;

        double deltaXGlobal = Math.cos(angle)*deltaXLocal - Math.sin(angle) * deltaYLocal;
        double deltaYGlobal = Math.sin(angle)*deltaXLocal + Math.cos(angle) * deltaYLocal;

        robotPosition[0]+= deltaXGlobal;
        robotPosition[1]+= deltaYGlobal;
        robotPosition[2] = Math.toDegrees(angle);
//            telemetry.addData("X:    ", robotPosition[0]);
//            telemetry.addData("Y:    ", robotPosition[1]);
//            telemetry.addData("Angle:", robotPosition[2]);
//            telemetry.update();
    }

    /*************************** IMU & Angle Utilities*************************/
    private void resetAngle ()        {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle ()        {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
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

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.25f;
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
        if (panelCnt > boltCnt && panelCnt > bulbCnt)
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


    /**************************Navigation and Chassis Control ************************************/

    private void resetEncoders (){
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resetting Localization memory
        lfPosition_last =0; rfPosition_last=0; lrPosition_last=0; rrPosition_last=0;

        //resetting global position
        robotPosition [0]=0;
        robotPosition [1]=0;
    }

    private void brakeAll (boolean isLockPosition, boolean isBrakeOn){

        if (isLockPosition)
        {
            LFMotor.setTargetPosition(LFMotor.getCurrentPosition());
            RFMotor.setTargetPosition(RFMotor.getCurrentPosition());
            LRMotor.setTargetPosition(LRMotor.getCurrentPosition());
            RRMotor.setTargetPosition(RRMotor.getCurrentPosition());
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LFMotor.setPower(0.3);
            RFMotor.setPower(0.3);
            LRMotor.setPower(0.3);
            RRMotor.setPower(0.3);
        } else if (isBrakeOn)
        {
            LFMotor.setPower(0);
            LRMotor.setPower(0);
            RFMotor.setPower(0);
            RRMotor.setPower(0);
        }
    }
    private void MotorsPower ( double LFPower, double RFPower, double LRPower, double RRPower)
    {
        double max;
        max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
        max = Math.max(max, Math.abs(LRPower));
        max = Math.max(max, Math.abs(RRPower));

        if (max > 1.0) { //Normalized
            LFPower /= max;
            RFPower /= max;
            LRPower /= max;
            RRPower /= max;
        }
        LFMotor.setPower(LFPower*control1SpeedFactor);
        RFMotor.setPower(RFPower*control1SpeedFactor);
        LRMotor.setPower(LRPower*control1SpeedFactor);
        RRMotor.setPower(RRPower*control1SpeedFactor);
    }
    private double speed_planning ( double V_max, double current_d, double target_d)
    {
        double V1, V3;
        double V_start = 0.2; //starting power without getting wheel slip
        double V_end = 0.1; //end speed to approach destination

        double Kup = 8, Kdown = 2/ (Math.abs(target_d) + 2.5); // Acc and de-acc rate  Power/count
        if (target_d > 0) {
            V1 = V_start + Kup * current_d;
            if (V1 < V_start) V1= V_start;
            V3 = V_end + Kdown * (target_d - current_d);
            if (V3<V_end) V3 = V_end;

            return Math.min(V1, Math.min(V_max, V3));
        } else {
            V1 = -V_start + Kup * current_d;
            if (V1 > -V_start) V1= -V_start;
            V3 = -V_end + Kdown * (target_d - current_d);
            if (V3>-V_end) V3 = -V_end;
            return Math.max(V1, Math.max(-V_max, V3));
        }
    }

    private boolean goXYnTurn ( double heading, double targetX, double targetY, double V_max,
                                double Tracking_resolution, double V_max_Turn, double turn_resolution, int wallEnable, boolean isLockPosition, boolean isBrakeOn){
        Chassis_motorEnabled = true;
        double drive_powerX = 0, drive_powerY=0;
        //resetEncoders();
        double x0 = robotPosition[0], y0= robotPosition[1];
        double V_XLocal=0, V_YLocal=0, Turn_power=0;
        double distance2Target = Math.hypot((targetX - robotPosition[0]),(targetY-robotPosition[1]));
        double distance2Go;
        boolean isTargetPosArrived = false, isHeadingAchieved = false;
        double traveledDistance =0;
        double Vp;
        double pathAngle =0;
        double robotHeading =0;
        double Kp = 0.003; // Proportional gain for turning control 0.01 means full power(speed) at 100 degree turn
        double V_turn_min = 0.04;
        double angle2turn;

        while (Chassis_motorEnabled  && !isStopRequested() && (!isHeadingAchieved || !isTargetPosArrived)) {
            distance2Go = Math.hypot((targetX - robotPosition[0]),(targetY-robotPosition[1]));
            angle2turn = heading - getAngle();

            if (distance2Go  > Tracking_resolution && traveledDistance < distance2Target )
            {//distance to target spot
                traveledDistance = Math.hypot((robotPosition[0] - x0), (robotPosition[1] - y0));
                Vp = speed_planning(V_max, traveledDistance, distance2Target  - Tracking_resolution );
                pathAngle = Math.atan2(targetY - robotPosition[1], targetX - robotPosition[0]);
                robotHeading = Math.toRadians(getAngle());
                V_XLocal = Vp * Math.cos(robotHeading - pathAngle);
                V_YLocal = -Vp * Math.sin(robotHeading - pathAngle);
                isTargetPosArrived = false;
            } else
            {V_XLocal =0;V_YLocal =0; isTargetPosArrived = true;}

            if (Math.abs(angle2turn) > turn_resolution)
            {//Turning to target heading
                Turn_power = Kp * angle2turn;

                if (Math.abs(Turn_power) < V_turn_min) {
                    Turn_power = V_turn_min * Math.signum(Turn_power);
                }
                if (Math.abs(Turn_power) > V_max_Turn) {
                    Turn_power = V_max_Turn * Math.signum(Turn_power);
                }
                isHeadingAchieved = false;
            } else {Turn_power =0; isHeadingAchieved = true;}

            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double LFPower = V_XLocal - V_YLocal - Turn_power ;
            double RFPower = V_XLocal + V_YLocal + Turn_power ;
            double LRPower = V_XLocal + V_YLocal - Turn_power ;
            double RRPower = V_XLocal - V_YLocal + Turn_power ;

            MotorsPower(LFPower, RFPower, LRPower, RRPower);
        }
        brakeAll(isLockPosition, isBrakeOn);
        return true;
    }

    /*********************Arm Control ****************/
    private void Arm_Pan_position ( double target, double power){
        Arm_pan_motorEnabled = true;
        if (target <minPanArm) target =  minPanArm;
        if (target > maxPanArm) target = maxPanArm;

        Arm_PanMotor.setTargetPosition((int) (target * turret_Pulse_per_Degree));
        Arm_PanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_PanMotor.setPower(power);

        while((Arm_pan_motorEnabled && Math.abs(target-getArmPanPosition()) > armPanResolution) && !isStopRequested()) // wait until finish
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

        while ((Arm_lift_motorEnabled && Math.abs(target-getArmLiftPosition()) > armLiftResolution) && !isStopRequested()) {
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

        while ((Arm_extend_motorEnabled && Math.abs(target-getArmExtendPosition()) > armExtendResolution) && !isStopRequested())
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
    private void claw_Close() {claw_Servo(25);}
    private void claw_Scan(){ ClawServo_Left.setPosition(0.9);ClawServo_Right.setPosition(0.9);}
    private void claw_Open() {claw_Servo(90);}

    // to drop the cone at front
    private void wrist_cone_front_ready(double adjustmentAngle)
    { // maintain 45 degree ``above`` level
        wrist_Flip_Servo(90);
        wrist_Tilt_Servo (-80-getArmLiftPosition() + adjustmentAngle);
    }
    private void wrist_cone_front_drop(double adjustmentAngle)
    { // maintain 15 degree below level
        wrist_Tilt_Servo (-150-getArmLiftPosition() + adjustmentAngle);
        sleep(250);
        claw_Open();
        sleep(100);
        //lift up the wrist by 90
        wrist_Tilt_Servo (-60-getArmLiftPosition() + adjustmentAngle);
        sleep(200);
    }
    private void wrist_conestack_scan(double adjustmentAngle)
    {
        // maintain 30 degree above level
        wrist_Flip_Servo(90);
        wrist_Tilt_Servo (-90-getArmLiftPosition() + adjustmentAngle);
    }
    // to pick up the cone at the front during auto
    private void wrist_cone_pickup_front_ready(double adjustmentAngle)
    { // maintain  level
        wrist_Flip_Servo(90);
        wrist_Tilt_Servo (-135-getArmLiftPosition() + adjustmentAngle);
    }
    private void wrist_cone_pickup_front_Up(double adjustmentAngle)
    { // above level by 60
        wrist_Tilt_Servo (-75-getArmLiftPosition() + adjustmentAngle);
    }
    // to pick up the cone fron the back during tele-op
    private void wrist_cone_pickup_back_ready(double adjustmentAngle)
    { // maintain  level
        wrist_Flip_Servo(-90);
        wrist_Tilt_Servo (45-getArmLiftPosition() + adjustmentAngle);
    }
    private void wrist_cone_pickup_back_Up(double adjustmentAngle)
    { // above level by 60
        wrist_Flip_Servo(-90);
        wrist_Tilt_Servo (-15-getArmLiftPosition() + adjustmentAngle);
    }
    public void scanextend(double targetdis){
        if(scanJunction.isFoundJunction()&&targetdis<60){
            double middleDis = centerd.getDistance(DistanceUnit.MM) - 22;
            double diffrence = (middleDis-targetdis)/1000 ;
            arm_extendThread = new Arm_Extend_PositionThread(getArmExtendPosition()+diffrence, 0.9, 0);
            arm_extendThread.start();
            scanextenddroppos = getArmExtendPosition()+diffrence;
            while(arm_extendThread.isAlive()) idle();
        }
    }

    private double getArmPanPosition(){return Arm_PanMotor.getCurrentPosition() / turret_Pulse_per_Degree;}
    private double getArmLiftPosition(){return (Arm_lift_Left_Motor.getCurrentPosition() + Arm_lift_Right_Motor.getCurrentPosition() ) /2.0/ lift_Pulse_per_Degree;}
    private double getArmExtendPosition(){return Arm_ExtendMotor.getCurrentPosition() / Arm_extend_distanceRatio;}
    private double getArmWristTiltPosition(){ return 90*(Wrist_TiltServo.getPosition()-(0.5+ wrist_Tilt_Offset))/0.33; }
    private double getArmWristFlipPosition(){ return 90*(Wrist_FlipServo.getPosition()-(0.5+ wrist_Flip_Offset))/0.33; }

    /*********************** Thread Classes ********************/
    private class routineTaskThread extends Thread        {
        public routineTaskThread()
        {
            this.setName("routineTaskThread");
        }
        @Override
        public void run()
        { // runs routine task at 50ms cycles
            try
            {
                while (!isInterrupted()  && !isStopRequested())
                {
                    localization();
//                    telemetry.addData("IMU", robotPosition[2]);
//                    telemetry.update();

                    idle();

                }
            }
            // an error occurred in the run loop.
            catch (Exception e) {}
        }
    }
    private class Arm_Lift_PositionThread extends Thread        {
        int target,  delay_start;
        double power;
        public Arm_Lift_PositionThread(int target, double power, int delay_start)
        {
            this.setName("Arm_Lift_PositionThread");
            this.target = target;
            this.power = power;
            this.delay_start = delay_start;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delay_start);
                Arm_Lift_Position(target, power);
            }
            // an error occurred in the run loop.
            catch (Exception e) {;}
        }
    }
    private class Arm_Wrist_PositionThread extends Thread        {
        int target,  delay_start;
        public Arm_Wrist_PositionThread(int target, int delay_start)
        {
            this.setName("Arm_Wrist_PositionThread");
            this.target = target;
            this.delay_start = delay_start;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delay_start);
                wrist_Tilt_Servo(target);
            }
            // an error occurred in the run loop.
            catch (Exception e) {;}
        }
    }
    private class Arm_Pan_PositionThread extends Thread        {
        double target;
        int delayMS;
        double power;
        public Arm_Pan_PositionThread(double target, double power, int delayMS)
        {
            this.setName("Arm_Pan_PositionThread");
            this.target = target;
            this.power = power;
            this.delayMS = delayMS;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delayMS);
                Arm_Pan_position(target, power);
            }
            // an error occurred in the run loop.
            catch (Exception e) {;}
        }
    }
    private class Arm_Extend_PositionThread extends Thread        {
        double target;
        int  delayMS;
        double power;
        public Arm_Extend_PositionThread(double target, double power, int delayMS)
        {
            this.setName("Arm_Extend_PositionThread");
            this.target = target;
            this.power = power;
            this.delayMS = delayMS;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delayMS);
                Arm_Extend_Position(target, power);
            }
            // an error occurred in the run loop.
            catch (Exception e) {}
        }
    }

    private class goXYnTurnThread extends Thread        {
        double heading, targetX, targetY, V_max, Tracking_resolution, V_max_Turn,turn_resolution;
        boolean isLockPosition;
        public boolean isgoXYnTurn_finished = false;
        int delayMS;
        boolean isBrakeOn;
        int wallEnable;

        public goXYnTurnThread( double heading, double targetX, double targetY, double V_max,
                                double Tracking_resolution, double V_max_Turn, double turn_resolution, int wallEnable, boolean isLockPosition, boolean isBrakeOn,  int delayMS)
        {
            this.heading = heading;
            this.targetX= targetX;          this.targetY= targetY;
            this.V_max= V_max;                this.Tracking_resolution= Tracking_resolution;
            this.V_max_Turn = V_max_Turn;         this.turn_resolution = turn_resolution;
            this.isLockPosition = isLockPosition;
            this.delayMS = delayMS;
            this.isBrakeOn = isBrakeOn;
            this.wallEnable = wallEnable;
        }
        @Override
        public void run()
        {
            try
            {
                sleep(delayMS);
                isgoXYnTurn_finished =  goXYnTurn(heading, targetX, targetY, V_max, Tracking_resolution, V_max_Turn, turn_resolution, wallEnable, isLockPosition, isBrakeOn);
            }
            catch (Exception e) {;}
        }
    }


    // change here
    // Auto task functions //
    void autoPreloadedConetoPoleTask(){ //Go forward and deliver the pre-load cone
        int PanAngle = -29;
        double extendLength = 0.35;

        if (isLeftSide == -1)// on the left side
        {
            PanAngle = 29;
            extendLength = 0.3;//value nerfed for scanning
        }
        claw_Close();
        arm_extendThread = new Arm_Extend_PositionThread(0.01, 0.9, 0);
        arm_extendThread.start();
        goXYnTurnThread = new goXYnTurnThread(0, 1.07, 0, 0.4, 0.01, 0.05, 1,0,true,true,0);
        goXYnTurnThread.start();
        arm_liftThread = new Arm_Lift_PositionThread( -48,0.6, 0);
        arm_liftThread.start();
        arm_panThread = new Arm_Pan_PositionThread(PanAngle, 0.3,0);
        arm_panThread.start();
        sleep(500);
        arm_extendThread = new Arm_Extend_PositionThread(extendLength, 0.7, 0);
        arm_extendThread.start();

        sleep(200);
        wrist_cone_front_ready(0);

        while  (arm_liftThread.isAlive() && !isStopRequested()) idle();
        wrist_cone_front_ready(0);

        //Wait until it reach the destination
        while (!isStopRequested() &&(goXYnTurnThread.isAlive()|| arm_panThread.isAlive()|| arm_liftThread.isAlive() || arm_extendThread.isAlive()) ){
            idle();
        }
        wrist_Flip_Servo(90);
        if(isLeftSide == -1){
            scanJunction = new ScanJunctionThread(-10,-45);
            scanJunction.start();
        }
        else{
            scanJunction = new ScanJunctionThread(10,45);
            scanJunction.start();
        }
        while (!scanJunction.isFoundJunction() && !isStopRequested()) idle();
        scanextend(26);
        wrist_cone_front_drop(0);
        isInAuto = false;
    }
    void autoscanconepickup(){
        int PanAngle = 74;
        double extendLength = 0.17;

        if (isLeftSide == -1)// on the left side
        {
            PanAngle = -74; //add 2 degree from (-left)
            extendLength = 0.17;
        }
        goXYnTurnThread = new goXYnTurnThread(0, robotPosition[0], robotPosition[1], 0.4, 0.01, 0.05, 1,0,true,true,0);
        goXYnTurnThread.start();
        arm_extendThread = new Arm_Extend_PositionThread(extendLength + ((double)stackCount * 0.005), 0.9, 0);
        arm_extendThread.start();
        arm_liftThread = new Arm_Lift_PositionThread( -107 - (int)(stackCount * 2),0.6, 200);
        arm_liftThread.start();
        arm_panThread = new Arm_Pan_PositionThread(PanAngle, 0.45,0);
        arm_panThread.start();
        stackCount += 1;
        if(stackCount == 5) stackCount = 0;

        while ((arm_panThread.isAlive()|| arm_liftThread.isAlive()) && !isStopRequested()) idle();
        arm_extendThread = new Arm_Extend_PositionThread(0.43, 0.75, 0);
        arm_extendThread.start();
        claw_Scan();
        wrist_Flip_Servo(90);

        while ((arm_extendThread.isAlive() ) && !isStopRequested()) idle();

        scanCone = new ScanConeThread();
        scanCone.start();

        while (scanCone.isFoundCone() == false && !isStopRequested()) idle();

        arm_extendThread = new Arm_Extend_PositionThread(0.455, 0.75, 0);
        arm_extendThread.start();




        while ( arm_extendThread.isAlive() && !isStopRequested()) idle();

        wrist_cone_pickup_front_ready(0);
        claw_Close();
        wrist_cone_pickup_front_Up(0);
        sleep(200);

        isInAuto = false;
    }



    // change here
    void autoConeStacktoHighPoleTask(){ // deliver the cone from stack to high pole
        int PanAngle = scannedturretanglepole; //-29
        double extendLength = scanextenddroppos;//0.35

        if (isLeftSide == -1)// on the left side
        {
            PanAngle = 29;
        }
        claw_Close();
        arm_extendThread = new Arm_Extend_PositionThread(0.01, 0.9, 0);
        arm_extendThread.start();
        //goXYnTurnThread = new goXYnTurnThread(0, robotPosition[0], robotPosition[1], 0.4, 0.01, 0.05, 1,0,true,false,0);
        //goXYnTurnThread.start();
        arm_liftThread = new Arm_Lift_PositionThread( -48,0.6, 0);
        arm_liftThread.start();
        arm_panThread = new Arm_Pan_PositionThread(scannedturretanglepole, 0.45,150);
        arm_panThread.start();

        sleep(500);
        arm_extendThread = new Arm_Extend_PositionThread(extendLength, 0.7, 0);
        arm_extendThread.start();
        while  (arm_liftThread.isAlive() && !isStopRequested()) idle();
        wrist_cone_front_ready(0);

        //Wait until it reach the destination
        //while (!isStopRequested() &&(goXYnTurnThread.isAlive() || arm_panThread.isAlive()|| arm_liftThread.isAlive() || arm_extendThread.isAlive()) )
        while (!isStopRequested() &&( arm_panThread.isAlive()|| arm_liftThread.isAlive() || arm_extendThread.isAlive()) )
            idle();
        wrist_cone_front_drop(0);
        isInAuto = false;
    }

    // change here
    void autoPickupTask( ){ // Pickup from the cone stack during auto
        int PanAngle = 78;
        double extendLength = 0.2;

        if (isLeftSide == -1)// on the left side
        {
            PanAngle = -74; //add 2 degree from (-left)
            extendLength = 0.2;
        }
        goXYnTurnThread = new goXYnTurnThread(0, robotPosition[0], robotPosition[1], 0.4, 0.01, 0.05, 1,0,true,true,0);
        goXYnTurnThread.start();

        arm_extendThread = new Arm_Extend_PositionThread(extendLength + ((double)stackCount * 0.004), 0.9, 0);
        arm_extendThread.start();
        arm_liftThread = new Arm_Lift_PositionThread( -107 - (int)(stackCount * 2),0.6, 200);
        arm_liftThread.start();
        arm_panThread = new Arm_Pan_PositionThread(scannedturretangle, 0.45,0);
        arm_panThread.start();
        stackCount += 1;
        if(stackCount == 5) stackCount = 0;

        while ((arm_panThread.isAlive()|| arm_liftThread.isAlive()) && !isStopRequested()) idle();

        wrist_cone_pickup_front_ready(0);
        arm_extendThread = new Arm_Extend_PositionThread(0.475, 0.75, 0);
        arm_extendThread.start();

        while (arm_extendThread.isAlive() && !isStopRequested()) idle();

        claw_Close();
        sleep(200);
        wrist_cone_pickup_front_Up(0);
        sleep(200);
        arm_extendThread = new Arm_Extend_PositionThread(0.25, 0.9, 0);

        isInAuto = false;
    }
    private void armUpRightPosition()
    {
        arm_extendThread = new Arm_Extend_PositionThread(0.01, 0.95, 0);
        arm_extendThread.start();
        arm_liftThread = new Arm_Lift_PositionThread( -10,0.95, 0);
        arm_liftThread.start();
        arm_panThread = new Arm_Pan_PositionThread(0, 0.6,0);
        arm_panThread.start();
        sleep(300);
        wrist_Tilt_Servo(-45);
    }

    private void stopAllMotors(){
        brakeAll(false, true);
        Arm_PanMotor.setPower(0);
        //Arm_liftMotor.setPower(0);
        Arm_ExtendMotor.setPower(0);
    }
    private class ScanJunctionThread extends Thread
    {
        public boolean runningFlag = false;
        public boolean resetFlag = false;

        private double leftDis = 2000;
        private double rightDis = 2000;
        private double middleDis = 2000;
        private final double DETECT_DIS = 250; // 50cm = 0.5m

        private double scanServoAngle = 0;
        private boolean foundJunction = false;
        private int scanlowrange;
        private int scanhighrange;
        public ScanJunctionThread(int low, int high)
        {
            scanServoAngle =  getArmPanPosition();
            runningFlag = true;
            foundJunction = false;
            scanFlag = true;
            scanlowrange = low;
            scanhighrange = high;
        }

        @Override
        public void run()
        {
            double ctrlValue = 0;
            scanServoAngle = getArmPanPosition();
            double scanServoStep = 10;
            boolean scanDir = true;
            double scanStopAngle = 50;
            // now scan +/- 15 degree
            double scanHigh = scanServoAngle + 15;
            scanHigh = scanHigh > scanhighrange ? scanhighrange : scanHigh;

            double scanLow = scanServoAngle - 15;
            scanLow = scanLow < scanlowrange ? scanlowrange : scanLow;

            while(opModeIsActive() && runningFlag)
            {
                leftDis = leftd.getDistance(DistanceUnit.MM);
                middleDis = centerd.getDistance(DistanceUnit.MM);
                rightDis = rightd.getDistance(DistanceUnit.MM);

                if (leftDis > DETECT_DIS && rightDis > DETECT_DIS && middleDis > DETECT_DIS){
                    if (scanDir) {
                        scanServoAngle += scanServoStep;
                        if (scanServoAngle > scanHigh) {
                            scanServoAngle = 50;
                            scanDir = false;
                        }
                    } else {
                        scanServoAngle -= scanServoStep;
                        if (scanServoAngle < scanLow) {
                            scanServoAngle = -90;
                            scanDir = true;

                        }
                    }
                }
                else  if (leftDis < middleDis && leftDis < DETECT_DIS){
                    scanServoAngle = scanServoAngle - 2;

                }else if(rightDis < middleDis && rightDis < DETECT_DIS){
                    scanServoAngle = scanServoAngle + 2;
                }
                arm_panThread = new Arm_Pan_PositionThread(scanServoAngle, 0.6,0);
                arm_panThread.start();
                if(middleDis < leftDis && middleDis < rightDis && middleDis < DETECT_DIS)
                {
                    scannedturretanglepole = (int) getArmPanPosition();
                    foundJunction = true;

                }

            }
        }

        public boolean isFoundJunction(){
            return foundJunction;
        }
        public void stopScan(){
            runningFlag = false;
            scanFlag = false;
        }
    }
    private class ScanConeThread extends Thread
    {
        public boolean runningFlag = false;
        public boolean resetFlag = false;

        private double leftDis = 2000;
        private double rightDis = 2000;
        private final double DETECT_DIS = 150; // 50cm = 0.5m

        private double scanAngle = 0;
        private boolean foundCone = false;
        public ScanConeThread()
        {
            scanAngle = getArmPanPosition();
            runningFlag = true;
            foundCone = false;
            scanFlag = true;
        }

        @Override
        public void run()
        {
            double ctrlValue = 1;
            while(opModeIsActive() && runningFlag && !foundCone)
            {
                leftDis = leftd.getDistance(DistanceUnit.MM);
                rightDis = rightd.getDistance(DistanceUnit.MM);

                if ((leftDis < DETECT_DIS && rightDis < DETECT_DIS)&&(leftDis - rightDis > 10)&&!foundCone){
                    scannedturretangle = (int) getArmPanPosition();
                    foundCone = true;


                }
                else{
                    scanAngle += ctrlValue;
                    Arm_PanMotor.setTargetPosition((int) (scanAngle * turret_Pulse_per_Degree));
                    Arm_PanMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm_PanMotor.setPower(scanAngle);
                    foundCone = false;
                }

            }
        }

        public boolean isFoundCone(){
            return foundCone;
        }
        public void stopScan(){
            runningFlag = false;
            scanFlag = false;
        }
    }

}