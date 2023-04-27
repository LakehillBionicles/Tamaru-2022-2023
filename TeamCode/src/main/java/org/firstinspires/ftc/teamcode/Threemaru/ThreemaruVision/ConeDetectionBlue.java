package org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetectionBlue extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */
    public enum TelemetryBars{
        ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE

    }
    public enum ParkingPosition {
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
            RED = new Scalar(255, 0, 0);

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
    private volatile ParkingPosition position = ParkingPosition.NOTSEEN;
    private volatile TelemetryBars barsPosition = TelemetryBars.ZERO;

    @Override
    public Mat processFrame(Mat input) {
        // Get the submat frame, and then sum all the values
        //Used for telemetry will remove
        int bar1= 0;
        int bar2= 0;
        int bar3= 0;
        int bar4 = 0;
        int bar5= 0;
        int bar6= 0;
        int bar7= 0;
        int bar8= 0;
        int bar9= 0;
        int bar10= 0;
        int bar11= 0;
        int bar12= 0;
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
        int otherCenterOfBars = 0;
        int numberOfBarsFilled = 0;
        int centerOfBars = 0;
        Mat areaMat = input.submat(new Rect(Bar_point1A, Bar_point1B));
        Scalar sumColors1 = Core.sumElems(areaMat);
        Mat areaMat1 = input.submat(new Rect(Bar_point2A, Bar_point2B));
        Scalar sumColors2 = Core.sumElems(areaMat1);
        Mat areaMat2 = input.submat(new Rect(Bar_point3A, Bar_point3B));
        Scalar sumColors3 = Core.sumElems(areaMat2);
        Mat areaMat3 = input.submat(new Rect(Bar_point4A, Bar_point4B));
        Scalar sumColors4 = Core.sumElems(areaMat3);
        Mat areaMat4 = input.submat(new Rect(Bar_point5A, Bar_point5B));
        Scalar sumColors5 = Core.sumElems(areaMat4);
        Mat areaMat5 = input.submat(new Rect(Bar_point6A, Bar_point6B));
        Scalar sumColors6 = Core.sumElems(areaMat5);
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

        Imgproc.rectangle(
                input,
                Bar_point1A,
                Bar_point1B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point2A,
                Bar_point2B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point3A,
                Bar_point3B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point4A,
                Bar_point4B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point5A,
                Bar_point5B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point6A,
                Bar_point6B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point7A,
                Bar_point7B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point8A,
                Bar_point8B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point9A,
                Bar_point9B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point10A,
                Bar_point10B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point11A,
                Bar_point11B,
                RED,
                2
        );
        Imgproc.rectangle(
                input,
                Bar_point12A,
                Bar_point12B,
                RED,
                2
        );
        if((sumColors1.val[2]/(sumColors1.val[0]+sumColors1.val[1]))>0.7){
            numberOfBarsFilled++;
            bar1 = 254;
            otherCenterOfBars = driveBar12 + otherCenterOfBars;
            Imgproc.rectangle(
                    input,
                    Bar_point1A,
                    Bar_point1B,
                    BLUE,
                    2
            );}
        if((sumColors2.val[2]/(sumColors2.val[0]+sumColors2.val[1]))>0.7){
            numberOfBarsFilled++;
            bar2 = 232;
            otherCenterOfBars = driveBar11 + otherCenterOfBars;
            Imgproc.rectangle(
                    input,
                    Bar_point2A,
                    Bar_point2B,
                    BLUE,
                    2
            );}
        if((sumColors3.val[2]/(sumColors3.val[0]+sumColors3.val[1]))>0.7){
            numberOfBarsFilled++;
            otherCenterOfBars = driveBar10 + otherCenterOfBars;
            bar3 = 210;
            Imgproc.rectangle(
                    input,
                    Bar_point3A,
                    Bar_point3B,
                    BLUE,
                    2
            );}
        if((sumColors4.val[2]/(sumColors4.val[0]+sumColors4.val[1]))>0.7){
            otherCenterOfBars = driveBar9 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar4 = 188;
            Imgproc.rectangle(
                    input,
                    Bar_point4A,
                    Bar_point4B,
                    BLUE,
                    2
            );}
        if((sumColors5.val[2]/(sumColors5.val[0]+sumColors5.val[1]))>0.7){
            otherCenterOfBars = driveBar8 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar5 = 166;
            Imgproc.rectangle(
                    input,
                    Bar_point5A,
                    Bar_point5B,
                    BLUE,
                    2
            );}
        if((sumColors6.val[2]/(sumColors6.val[0]+sumColors6.val[1]))>0.7){
            otherCenterOfBars = driveBar7 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar6 = 144;
            Imgproc.rectangle(
                    input,
                    Bar_point6A,
                    Bar_point6B,
                    BLUE,
                    2
            );}
        if((sumColors7.val[2]/(sumColors7.val[0]+sumColors7.val[1]))>0.7){
            otherCenterOfBars = driveBar6 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar7 = 122;
            Imgproc.rectangle(
                    input,
                    Bar_point7A,
                    Bar_point7B,
                    BLUE,
                    2
            );}
        if((sumColors8.val[2]/(sumColors8.val[0]+sumColors8.val[1]))>0.7){
            otherCenterOfBars = driveBar5 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar8 = 100;
            Imgproc.rectangle(
                    input,
                    Bar_point8A,
                    Bar_point8B,
                    BLUE,
                    2
            );}
        if((sumColors9.val[2]/(sumColors9.val[0]+sumColors9.val[1]))>0.7){
            otherCenterOfBars = driveBar4 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar9 = 78;
            Imgproc.rectangle(
                    input,
                    Bar_point9A,
                    Bar_point9B,
                    BLUE,
                    2
            );}
        if((sumColors10.val[2]/(sumColors10.val[0]+sumColors10.val[1]))>0.7){
            otherCenterOfBars = driveBar3 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar10 = 56;

            Imgproc.rectangle(
                    input,
                    Bar_point10A,
                    Bar_point10B,
                    BLUE,
                    2
            );}
        if((sumColors11.val[2]/(sumColors11.val[0]+sumColors11.val[1]))>0.7){
            otherCenterOfBars = driveBar2 + otherCenterOfBars;
            numberOfBarsFilled++;
            bar11 = 34;
            Imgproc.rectangle(
                    input,
                    Bar_point11A,
                    Bar_point11B,
                    BLUE,
                    2
            );}
        if((sumColors12.val[2]/(sumColors12.val[0]+sumColors12.val[1]))>0.7){
            otherCenterOfBars = driveBar1 + otherCenterOfBars;
            bar12= 12;
            numberOfBarsFilled++;
            Imgproc.rectangle(
                    input,
                    Bar_point12A,
                    Bar_point12B,
                    BLUE,
                    2
            );}
        if(numberOfBarsFilled>0 ){
            centerOfBars = (bar12 + bar11 + bar10 + bar9 + bar8 + bar7 + bar6 + bar5 + bar4 + bar3 + bar2 + bar1)/numberOfBarsFilled;
            Point CenterofBarsPointA = new Point(
                    centerOfBars+25,
                    SLEEVE_TOPLEFT_ANCHOR_POINT.y+95);
            Point CenterofBarsPointB = new Point(
                    centerOfBars+15,
                    SLEEVE_TOPLEFT_ANCHOR_POINT.y+105);
            Imgproc.rectangle(
                    input,
                    CenterofBarsPointA,
                    CenterofBarsPointB,
                    GREEN,
                    2
            );}
        if(numberOfBarsFilled>0){
            otherCenterOfBars = otherCenterOfBars/numberOfBarsFilled;
        }
        if(otherCenterOfBars == 0){
            position = ParkingPosition.NOTSEEN;
        }
        else if(otherCenterOfBars == 1){
            position = ParkingPosition.ONE;
        }else if(otherCenterOfBars == 2){
            position = ParkingPosition.TWO;
        }else if(otherCenterOfBars == 3){
            position = ParkingPosition.THREE;
        }else if(otherCenterOfBars == 4){
            position = ParkingPosition.FOUR;
        }else if(otherCenterOfBars == 5){
            position = ParkingPosition.FIVE;
        }else if(otherCenterOfBars == 6){
            position = ParkingPosition.SIX;
        }else if(otherCenterOfBars == 7){
            position = ParkingPosition.SEVEN;
        }else if(otherCenterOfBars == 8){
            position = ParkingPosition.EIGHT;
        }else if(otherCenterOfBars == 9){
            position = ParkingPosition.NINE;
        }else if(otherCenterOfBars == 10){
            position = ParkingPosition.TEN;
        }else if(otherCenterOfBars == 11){
            position = ParkingPosition.ELEVEN;
        }else if(otherCenterOfBars == 12){
            position = ParkingPosition.TWELVE;
        }
        // Get the minimum RGB value from every single channel
        //double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[1]));
        // Change the bounding box color based on the sleeve color

        // Release and return input
        areaMat.release();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }

}
