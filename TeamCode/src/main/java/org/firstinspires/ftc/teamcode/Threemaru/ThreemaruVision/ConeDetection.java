package org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.vuforia.Rectangle;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.ArrayList;

public class ConeDetection extends OpenCvPipeline {


    /*
        YELLOW  = Parking Left
        CYAN    = Parking Middle
        MAGENTA = Parking Right
         */
    public enum ParkingPosition {
        NOTSEEN, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE
    }
    public enum RedParkingPosition {
        NOTSEEN, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE
    }
    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(265, 10);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 22;
    public static int REGION_HEIGHT = 200;

    // Color definitions
    private final Scalar
            BLUE = new Scalar(0,0,255),
            GREEN  = new Scalar(0, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            RED = new Scalar(255, 0, 0),

            WHITE = new Scalar(255,255,255);

    // Anchor point definitions
    /*
        Point sleeve_pointA = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_pointB = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point sleeve_point1A = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x -50,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_point1B = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x - 50 + REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point sleeve_point2A = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x - 100,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_point2B = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x - 100 + REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point sleeve_point3A = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x - 150,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_point3B = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x - 150 + REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point sleeve_point4A = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x-200,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_point4B = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x-200 + REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point sleeve_point5A = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x-250,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_point5B = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x-250 + REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

*/      Point Bar_point1A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point1B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point2A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-22,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point2B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-22 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point3A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-44,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point3B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-44 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point4A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-66,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point4B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-66 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point5A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-88,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point5B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-88 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point6A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-110,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point6B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-110 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point7A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-132,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point7B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-132 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point8A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-154,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point8B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-154 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y+ REGION_HEIGHT);
    Point Bar_point9A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-176,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point9B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-176 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point10A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-198,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point10B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-198 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point11A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-220,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point11B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-220 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point Bar_point12A = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-242,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point Bar_point12B = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x-242 + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    // Running variable storing the parking position
    private static volatile ConeDetection.ParkingPosition bluePosition = ParkingPosition.NOTSEEN;
    private static volatile ConeDetection.RedParkingPosition redPosition = RedParkingPosition.NOTSEEN;

    ArrayList<Integer> barAmounts= new ArrayList<Integer>();
    @Override
    public Mat processFrame(Mat input) {
        // Get the submat frame, and then sum all the values
        //Used for telemetry will remove
        //These are the tolerances for the detection of red and blue cones
        //To decrease tolerance(harder to detect cone but less likely to think a wall is a cone) increase averageBlueValueForCone
        //or you can decrease averageRedAndGreenValueForCone
        //To increase tolerance(harder to detect cone but less likely to think a wall is a cone) decrease averageBlueValueForCone
        //or you can increase averageRedAndGreenValueForCone
        double redTolerance = 0.7;
        double blueTolerance = 0.7;
        int addedBars = 0;
        //Base bar is blue I'm just to lazy to name it blueBar
        //These are not used for telemetry
        int driveBar1= 1;
        int driveBar2= 2;
        int driveBar3= 3;
        int driveBar4 = 4;
        int driveBar5= 5;
        int driveBar6= 6;
        int driveBar7= 7;
        int driveBar8= 8;
        int driveBar9= 9;
        int driveBar10= 10;
        int driveBar11= 11;
        int driveBar12= 12;
        //Base driveBar is blue I'm just to lazy to name it driveBlueBar
        int otherCenterOfBars = 0;
        int otherRedCenterOfBars = 0;
        int redNumberOfBarsFilled = 0;
        int numberOfBarsFilled = 0;
        int centerOfBars = 0;
        int redCenterOfBars = 0;
        Mat areaMat1 = input.submat(new Rect(Bar_point1A, Bar_point1B));
        Scalar sumColors1 = Core.sumElems(areaMat1);
        Mat areaMat2 = input.submat(new Rect(Bar_point2A, Bar_point2B));
        Scalar sumColors2 = Core.sumElems(areaMat2);
        Mat areaMat3 = input.submat(new Rect(Bar_point3A, Bar_point3B));
        Scalar sumColors3 = Core.sumElems(areaMat3);
        Mat areaMat4 = input.submat(new Rect(Bar_point4A, Bar_point4B));
        Scalar sumColors4 = Core.sumElems(areaMat4);
        Mat areaMat5 = input.submat(new Rect(Bar_point5A, Bar_point5B));
        Scalar sumColors5 = Core.sumElems(areaMat5);
        Mat areaMat6 = input.submat(new Rect(Bar_point6A, Bar_point6B));
        Scalar sumColors6 = Core.sumElems(areaMat6);
        Mat areaMat7 = input.submat(new Rect(Bar_point7A, Bar_point7B));
        Scalar sumColors7 = Core.sumElems(areaMat7);
        Mat areaMat8 = input.submat(new Rect(Bar_point8A, Bar_point8B));
        Scalar sumColors8 = Core.sumElems(areaMat8);
        Mat areaMat9 = input.submat(new Rect(Bar_point9A, Bar_point9B));
        Scalar sumColors9 = Core.sumElems(areaMat9);
        Mat areaMat10 = input.submat(new Rect(Bar_point10A, Bar_point10B));
        Scalar sumColors10 = Core.sumElems(areaMat10);
        Mat areaMat11 = input.submat(new Rect(Bar_point11A, Bar_point11B));
        Scalar sumColors11 = Core.sumElems(areaMat11);
        Mat areaMat12 = input.submat(new Rect(Bar_point12A, Bar_point12B));
        Scalar sumColors12 = Core.sumElems(areaMat12);
        //Use for finding Tolerance
        telemetry.addData("Box1RGBvalues",sumColors1);
        telemetry.addData("Box2RGBvalues",sumColors2);
        telemetry.addData("Box3RGBvalues",sumColors3);
        telemetry.addData("Box4RGBvalues",sumColors4);
        telemetry.addData("Box5RGBvalues",sumColors5);
        telemetry.addData("Box6RGBvalues",sumColors6);
        telemetry.addData("Box7RGBvalues",sumColors7);
        telemetry.addData("Box8RGBvalues",sumColors8);
        telemetry.addData("Box9RGBvalues",sumColors9);
        telemetry.addData("Box10RGBvalues",sumColors10);
        telemetry.addData("Box11RGBvalues",sumColors11);
        telemetry.addData("Box12RGBvalues",sumColors12);
        telemetry.addData("Box1RedTolerance",(sumColors1.val[0] / (sumColors1.val[2] + sumColors1.val[1])));
        telemetry.addData("Box2RedTolerance",(sumColors2.val[0] / (sumColors2.val[2] + sumColors2.val[1])));
        telemetry.addData("Box3RedTolerance",(sumColors3.val[0] / (sumColors3.val[2] + sumColors3.val[1])));
        telemetry.addData("Box4RedTolerance",(sumColors4.val[0] / (sumColors4.val[2] + sumColors4.val[1])));
        telemetry.addData("Box5RedTolerance",(sumColors5.val[0] / (sumColors5.val[2] + sumColors5.val[1])));
        telemetry.addData("Box6RedTolerance",(sumColors6.val[0] / (sumColors6.val[2] + sumColors6.val[1])));
        telemetry.addData("Box7RedTolerance",(sumColors7.val[0] / (sumColors7.val[2] + sumColors7.val[1])));
        telemetry.addData("Box8RedTolerance",(sumColors8.val[0] / (sumColors8.val[2] + sumColors8.val[1])));
        telemetry.addData("Box9RedTolerance",(sumColors9.val[0] / (sumColors9.val[2] + sumColors9.val[1])));
        telemetry.addData("Box10RedTolerance",(sumColors10.val[0] / (sumColors10.val[2] + sumColors10.val[1])));
        telemetry.addData("Box11RedTolerance",(sumColors11.val[0] / (sumColors11.val[2] + sumColors11.val[1])));
        telemetry.addData("Box12RedTolerance",(sumColors12.val[0] / (sumColors12.val[2] + sumColors12.val[1])));
        telemetry.addData("Box1BlueTolerance",(sumColors1.val[2] / (sumColors1.val[0] + sumColors1.val[1])));
        telemetry.addData("Box2BlueTolerance",(sumColors2.val[2] / (sumColors2.val[0] + sumColors2.val[1])));
        telemetry.addData("Box3BlueTolerance",(sumColors3.val[2] / (sumColors3.val[0] + sumColors3.val[1])));
        telemetry.addData("Box4BlueTolerance",(sumColors4.val[2] / (sumColors4.val[0] + sumColors4.val[1])));
        telemetry.addData("Box5BlueTolerance",(sumColors5.val[2] / (sumColors5.val[0] + sumColors5.val[1])));
        telemetry.addData("Box6BlueTolerance",(sumColors6.val[2] / (sumColors6.val[0] + sumColors6.val[1])));
        telemetry.addData("Box7BlueTolerance",(sumColors7.val[2] / (sumColors7.val[0] + sumColors7.val[1])));
        telemetry.addData("Box8BlueTolerance",(sumColors8.val[2] / (sumColors8.val[0] + sumColors8.val[1])));
        telemetry.addData("Box9BlueTolerance",(sumColors9.val[2] / (sumColors9.val[0] + sumColors9.val[1])));
        telemetry.addData("Box10BlueTolerance",(sumColors10.val[2] / (sumColors10.val[0] + sumColors10.val[1])));
        telemetry.addData("Box11BlueTolerance",(sumColors11.val[2] / (sumColors11.val[0] + sumColors11.val[1])));
        telemetry.addData("Box12BlueTolerance",(sumColors12.val[2] / (sumColors12.val[0] + sumColors12.val[1])));
        telemetry.update();
            for(int i = 0; i<12; i++){
                Point newBar_point1A = new Point(
                        SLEEVE_TOPLEFT_ANCHOR_POINT.x-(22*i),
                        SLEEVE_TOPLEFT_ANCHOR_POINT.y);
                Point newBar_point1B = new Point(
                        SLEEVE_TOPLEFT_ANCHOR_POINT.x-(22*i) + REGION_WIDTH,
                        SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
                Imgproc.rectangle(
                        input,
                        newBar_point1A,
                        newBar_point1B,
                        WHITE,
                        2
                );
            }
            /*
            Imgproc.rectangle(
                    input,
                    Bar_point1A,
                    Bar_point1B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point2A,
                    Bar_point2B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point3A,
                    Bar_point3B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point4A,
                    Bar_point4B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point5A,
                    Bar_point5B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point6A,
                    Bar_point6B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point7A,
                    Bar_point7B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point8A,
                    Bar_point8B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point9A,
                    Bar_point9B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point10A,
                    Bar_point10B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point11A,
                    Bar_point11B,
                    WHITE,
                    2
            );
            Imgproc.rectangle(
                    input,
                    Bar_point12A,
                    Bar_point12B,
                    WHITE,
                    2
            );
            */
        /*
            if ((sumColors1.val[2] / (sumColors1.val[0] + sumColors1.val[1])) > blueTolerance) {
                numberOfBarsFilled++;
                addedBars = addedBars + 254;
                otherCenterOfBars = driveBar12 + otherCenterOfBars;
                Imgproc.rectangle(
                        input,
                        Bar_point1A,
                        Bar_point1B,
                        BLUE,
                        2
                );
            }
            if ((sumColors2.val[2] / (sumColors2.val[0] + sumColors2.val[1])) > blueTolerance) {
                numberOfBarsFilled++;
                addedBars = addedBars + 232;
                otherCenterOfBars = driveBar11 + otherCenterOfBars;
                Imgproc.rectangle(
                        input,
                        Bar_point2A,
                        Bar_point2B,
                        BLUE,
                        2
                );
            }
            if ((sumColors3.val[2] / (sumColors3.val[0] + sumColors3.val[1])) > blueTolerance) {
                numberOfBarsFilled++;
                otherCenterOfBars = driveBar10 + otherCenterOfBars;
                addedBars = addedBars + 210;
                Imgproc.rectangle(
                        input,
                        Bar_point3A,
                        Bar_point3B,
                        BLUE,
                        2
                );
            }
            if ((sumColors4.val[2] / (sumColors4.val[0] + sumColors4.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar9 + otherCenterOfBars;
                numberOfBarsFilled++;
                addedBars = addedBars + 188;
                Imgproc.rectangle(
                        input,
                        Bar_point4A,
                        Bar_point4B,
                        BLUE,
                        2
                );
            }

         */
            if ((sumColors5.val[2] / (sumColors5.val[0] + sumColors5.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar8 + otherCenterOfBars;
                numberOfBarsFilled++;
                addedBars = addedBars + 166;
                Imgproc.rectangle(
                        input,
                        Bar_point5A,
                        Bar_point5B,
                        BLUE,
                        2
                );
            }
            if ((sumColors6.val[2] / (sumColors6.val[0] + sumColors6.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar7 + otherCenterOfBars;
                numberOfBarsFilled++;
                addedBars = addedBars + 144;
                Imgproc.rectangle(
                        input,
                        Bar_point6A,
                        Bar_point6B,
                        BLUE,
                        2
                );
            }
            if ((sumColors7.val[2] / (sumColors7.val[0] + sumColors7.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar6 + otherCenterOfBars;
                numberOfBarsFilled++;
                addedBars = addedBars + 122;
                Imgproc.rectangle(
                        input,
                        Bar_point7A,
                        Bar_point7B,
                        BLUE,
                        2
                );
            }
            if ((sumColors8.val[2] / (sumColors8.val[0] + sumColors8.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar5 + otherCenterOfBars;
                addedBars = addedBars + 100;
                numberOfBarsFilled++;
                Imgproc.rectangle(
                        input,
                        Bar_point8A,
                        Bar_point8B,
                        BLUE,
                        2
                );
            }
            if ((sumColors9.val[2] / (sumColors9.val[0] + sumColors9.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar4 + otherCenterOfBars;
                numberOfBarsFilled++;
                addedBars = addedBars + 78;
                Imgproc.rectangle(
                        input,
                        Bar_point9A,
                        Bar_point9B,
                        BLUE,
                        2
                );
            }
            if ((sumColors10.val[2] / (sumColors10.val[0] + sumColors10.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar3 + otherCenterOfBars;
                numberOfBarsFilled++;
                addedBars = addedBars + 56;

                Imgproc.rectangle(
                        input,
                        Bar_point10A,
                        Bar_point10B,
                        BLUE,
                        2
                );
            }
            if ((sumColors11.val[2] / (sumColors11.val[0] + sumColors11.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar2 + otherCenterOfBars;
                numberOfBarsFilled++;
                addedBars = addedBars + 34;
                Imgproc.rectangle(
                        input,
                        Bar_point11A,
                        Bar_point11B,
                        BLUE,
                        2
                );
            }
            if ((sumColors12.val[2] / (sumColors12.val[0] + sumColors12.val[1])) > blueTolerance) {
                otherCenterOfBars = driveBar1 + otherCenterOfBars;
                addedBars = addedBars + 12;
                numberOfBarsFilled++;
                Imgproc.rectangle(
                        input,
                        Bar_point12A,
                        Bar_point12B,
                        BLUE,
                        2
                );}
            if (numberOfBarsFilled > 0) {
                centerOfBars = addedBars/numberOfBarsFilled;
                addedBars = 0;

                Point CenterofBarsPointA = new Point(
                        centerOfBars + 25,
                        SLEEVE_TOPLEFT_ANCHOR_POINT.y + 95);
                Point CenterofBarsPointB = new Point(
                        centerOfBars + 15,
                        SLEEVE_TOPLEFT_ANCHOR_POINT.y + 105);
                Imgproc.rectangle(
                        input,
                        CenterofBarsPointA,
                        CenterofBarsPointB,
                        CYAN,
                        2
                );
                barAmounts.clear();
            }

        if ((sumColors1.val[0] / (sumColors1.val[1] + sumColors1.val[2])) > redTolerance) {
            redNumberOfBarsFilled++;
            addedBars = addedBars + 254;
            otherRedCenterOfBars = driveBar12 + otherRedCenterOfBars;
            Imgproc.rectangle(
                    input,
                    Bar_point1A,
                    Bar_point1B,
                    RED,
                    2
            );
        }
        if ((sumColors2.val[0] / (sumColors2.val[1] + sumColors2.val[2])) > redTolerance) {
            redNumberOfBarsFilled++;
            addedBars = addedBars + 232;

            otherRedCenterOfBars = driveBar11 + otherRedCenterOfBars;
            Imgproc.rectangle(
                    input,
                    Bar_point2A,
                    Bar_point2B,
                    RED,
                    2
            );
        }
        if ((sumColors3.val[0] / (sumColors3.val[1] + sumColors3.val[2])) > redTolerance) {
            redNumberOfBarsFilled++;
            otherRedCenterOfBars = driveBar10 + otherRedCenterOfBars;
            addedBars = addedBars + 210;
            Imgproc.rectangle(
                    input,
                    Bar_point3A,
                    Bar_point3B,
                    RED,
                    2
            );
        }
        if ((sumColors4.val[0] / (sumColors4.val[1] + sumColors4.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar9 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 188;
            Imgproc.rectangle(
                    input,
                    Bar_point4A,
                    Bar_point4B,
                    RED,
                    2
            );
        }
        if ((sumColors5.val[0] / (sumColors5.val[1] + sumColors5.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar8 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 166;
            Imgproc.rectangle(
                    input,
                    Bar_point5A,
                    Bar_point5B,
                    RED,
                    2
            );
        }
        if ((sumColors6.val[0] / (sumColors6.val[1] + sumColors6.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar7 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 144;
            Imgproc.rectangle(
                    input,
                    Bar_point6A,
                    Bar_point6B,
                    RED,
                    2
            );
        }
        if ((sumColors7.val[0] / (sumColors7.val[1] + sumColors7.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar6 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 122;
            Imgproc.rectangle(
                    input,
                    Bar_point7A,
                    Bar_point7B,
                    RED,
                    2
            );
        }
        if ((sumColors8.val[0] / (sumColors8.val[1] + sumColors8.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar5 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 100;
            Imgproc.rectangle(
                    input,
                    Bar_point8A,
                    Bar_point8B,
                    RED,
                    2
            );
        }
        if ((sumColors9.val[0] / (sumColors9.val[1] + sumColors9.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar4 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 78;
            Imgproc.rectangle(
                    input,
                    Bar_point9A,
                    Bar_point9B,
                    RED,
                    2
            );
        }
        if ((sumColors10.val[0] / (sumColors10.val[1] + sumColors10.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar3 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 56;

            Imgproc.rectangle(
                    input,
                    Bar_point10A,
                    Bar_point10B,
                    RED,
                    2
            );
        }
        if ((sumColors11.val[0] / (sumColors11.val[1] + sumColors11.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar2 + otherRedCenterOfBars;
            redNumberOfBarsFilled++;
            addedBars = addedBars + 34;
            Imgproc.rectangle(
                    input,
                    Bar_point11A,
                    Bar_point11B,
                    RED,
                    2
            );
        }
        if ((sumColors12.val[0] / (sumColors12.val[1] + sumColors12.val[2])) > redTolerance) {
            otherRedCenterOfBars = driveBar1 + otherRedCenterOfBars;
            addedBars = addedBars + 12;
            redNumberOfBarsFilled++;
            Imgproc.rectangle(
                    input,
                    Bar_point12A,
                    Bar_point12B,
                    RED,
                    2
            );
        }
        if (redNumberOfBarsFilled > 0){
        redCenterOfBars = addedBars / redNumberOfBarsFilled;
        Point redCenterofBarsPointA = new Point(
                redCenterOfBars + 25,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + 95);
        Point redCenterofBarsPointB = new Point(
                redCenterOfBars + 15,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + 105);
        Imgproc.rectangle(
                input,
                redCenterofBarsPointA,
                redCenterofBarsPointB,
                GREEN,
                2
        );
        barAmounts.clear();}
        if(numberOfBarsFilled>0){
            otherCenterOfBars = otherCenterOfBars/numberOfBarsFilled;
        }
        if(redNumberOfBarsFilled>0){
            otherRedCenterOfBars = otherRedCenterOfBars/redNumberOfBarsFilled;
        }
        if(otherCenterOfBars == 0){
            bluePosition = ConeDetection.ParkingPosition.NOTSEEN;
        }
        else if(otherCenterOfBars == 1){
            bluePosition = ConeDetection.ParkingPosition.ONE;
        }else if(otherCenterOfBars == 2){
            bluePosition = ConeDetection.ParkingPosition.TWO;
        }else if(otherCenterOfBars == 3){
            bluePosition = ConeDetection.ParkingPosition.THREE;
        }else if(otherCenterOfBars == 4){
            bluePosition = ConeDetection.ParkingPosition.FOUR;
        }else if(otherCenterOfBars == 5){
            bluePosition = ConeDetection.ParkingPosition.FIVE;
        }else if(otherCenterOfBars == 6){
            bluePosition = ConeDetection.ParkingPosition.SIX;
        }else if(otherCenterOfBars == 7){
            bluePosition = ConeDetection.ParkingPosition.SEVEN;
        }else if(otherCenterOfBars == 8){
            bluePosition = ConeDetection.ParkingPosition.EIGHT;
        }else if(otherCenterOfBars == 9){
            bluePosition = ConeDetection.ParkingPosition.NINE;
        }else if(otherCenterOfBars == 10){
            bluePosition = ConeDetection.ParkingPosition.TEN;
        }else if(otherCenterOfBars == 11){
            bluePosition = ConeDetection.ParkingPosition.ELEVEN;
        }else if(otherCenterOfBars == 12){
            bluePosition = ConeDetection.ParkingPosition.TWELVE;
        }
        if(otherRedCenterOfBars == 0) {
            redPosition = ConeDetection.RedParkingPosition.NOTSEEN;
        }
            else if (otherRedCenterOfBars == 1) {
                redPosition = ConeDetection.RedParkingPosition.ONE;
            } else if (otherRedCenterOfBars == 2) {
                redPosition = ConeDetection.RedParkingPosition.TWO;
            } else if (otherRedCenterOfBars == 3) {
                redPosition = ConeDetection.RedParkingPosition.THREE;
            } else if (otherRedCenterOfBars == 4) {
                redPosition = ConeDetection.RedParkingPosition.FOUR;
            } else if (otherRedCenterOfBars == 5) {
                redPosition = ConeDetection.RedParkingPosition.FIVE;
            } else if (otherRedCenterOfBars == 6) {
                redPosition = ConeDetection.RedParkingPosition.SIX;
            } else if (otherRedCenterOfBars == 7) {
                redPosition = ConeDetection.RedParkingPosition.SEVEN;
            } else if (otherRedCenterOfBars == 8) {
                redPosition = ConeDetection.RedParkingPosition.EIGHT;
            } else if (otherRedCenterOfBars == 9) {
                redPosition = ConeDetection.RedParkingPosition.NINE;
            } else if (otherRedCenterOfBars == 10) {
                redPosition = ConeDetection.RedParkingPosition.TEN;
            } else if (otherRedCenterOfBars == 11) {
                redPosition = ConeDetection.RedParkingPosition.ELEVEN;
            } else if (otherRedCenterOfBars == 12) {
                redPosition = ConeDetection.RedParkingPosition.TWELVE;
            }
        // Get the minimum RGB value from every single channel
        //double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[1]));
        // Change the bounding box color based on the sleeve color

        // Release and return input
            areaMat1.release();
            areaMat2.release();
            areaMat3.release();
            areaMat4.release();
            areaMat5.release();
            areaMat6.release();
            areaMat7.release();
            areaMat8.release();
            areaMat9.release();
            areaMat10.release();
            areaMat11.release();
            areaMat12.release();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public static ParkingPosition getBluePosition(){return bluePosition;}
    public static RedParkingPosition getRedPosition(){return redPosition;}}


