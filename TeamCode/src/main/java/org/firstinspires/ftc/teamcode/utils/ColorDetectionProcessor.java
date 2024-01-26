//package org.firstinspires.ftc.teamcode.utils;
//
//import android.graphics.Canvas;
//
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//
//import java.util.ArrayList;
//
//public class ColorDetectionProcessor implements VisionProcessor {
//
//    private final int threshold = 25;
//    // private int blue[] = new int[3]; //array with blue pixels, 0 is left, 1 is
//    // center, 2 is right
//    // private int red[] = new int [3]; //array with red pixels, 0 is left, 1 is
//    // center, 2 is right
//    private int[] color_counts = new int[3];
//    private final ArrayList<Integer> leftCnt = new ArrayList<Integer>();
//    private final ArrayList<Integer> centerCnt = new ArrayList<Integer>();
//    private final ArrayList<Integer> rightCnt = new ArrayList<Integer>();
//    private final int startY = 0;
//    private final int endY = 240;
//    private final int startX = 0;
//    private final int endX = 320;
//    private final int x1 = 100; // first x division
//    private final int x2 = 220; // second x division
//    private String color;
//    Selected selection = Selected.NONE; Mat submat = new Mat();
//    Mat hsvMat = new Mat();
//
//    public Selected getSelection() {
//        return selection;
//    }
//    public enum Selected {
//        NONE,
//        LEFT,
//        MIDDLE,
//        RIGHT
//    }
//    int targetIndex = 0;
//    public enum StartingPosition {
//        LEFT,
//        CENTER,
//        RIGHT,
//        NONE
//    }
//    Point leftA = new Point(
//            startX,
//            startY);
//    Point leftB = new Point(
//            x1,
//            endY);
//    Point centerA = new Point(
//            x1 + 1,
//            startY);
//    Point centerB = new Point(
//            x2,
//            endY);
//    Point rightA = new Point(
//            x2 + 1,
//            startY);
//    Point rightB = new Point(
//            endX,
//            endY);
//
//    public Rect rectLeft = new Rect(110, 330, 135, 130);
//    public Rect rectMiddle = new Rect(450, 330, 135, 110);
//
//    public ColorDetectionProcessor() {
//        //default
//        position = StartingPosition.CENTER;
//    }
//
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }
//
//    @Override
//    public Object processFrame(Mat frame, long captureTimeNanos) {
////        color_counts = new int[]{0, 0, 0};
////
////        for (int i = startY; i < endY; i++) {
////            for (int j = startX; j < endX; j++) {
////                if (frame.get(i, j)[targetIndex] > (frame.get(i, j)[1] + threshold)
////                        && frame.get(i, j)[targetIndex] > (frame.get(i, j)[2 - targetIndex] + threshold)) {
////                    if (j < x1)
////                        color_counts[0]++;
////                    else if (j < x2)
////                        color_counts[1]++;
////                    else
////                        color_counts[2]++;
////                }
////            }
////        }
////
////        leftCnt.add(color_counts[0]);
////        centerCnt.add(color_counts[1]);
////        rightCnt.add(color_counts[2]);
////
////        if (leftCnt.size() > 5) {
////            leftCnt.remove(0);
////            centerCnt.remove(0);
////            rightCnt.remove(0);
////        }
////
////        // take the average of each arraylist and compare them to determine position
////        int leftAvg = 0, centerAvg = 0, rightAvg = 0;
////        for (int k = 0; k < leftCnt.size(); k++) {
////            leftAvg += leftCnt.get(k);
////            centerAvg += centerCnt.get(k);
////            rightAvg += rightCnt.get(k);
////        }
////
////        leftAvg /= leftCnt.size();
////        centerAvg /= centerCnt.size();
////        rightAvg /= rightCnt.size();
//
////        if (leftAvg > centerAvg)
////            position = StartingPosition.LEFT;
////        else if (centerAvg > leftAvg)
////            position = StartingPosition.CENTER;
////        else if (rightAvg > leftAvg && rightAvg > centerAvg)
////            position = StartingPosition.RIGHT;
////        else
////            position = StartingPosition.CENTER;
//
//        // int maxRed = Math.max(red[0], Math.max(red[1], red[2]));
//        // int maxBlue = Math.max(blue[0], Math.max(blue[1], blue[2]));
//
//        // if (maxRed>maxBlue) {
//        // if(maxRed == red[0])
//        // position = StartingPosition.LEFT;
//        // else if(maxRed == red[1])
//        // position = StartingPosition.CENTER;
//        // else
//        // position = StartingPosition.RIGHT;
//        // }
//        // else {
//        // if(maxBlue == blue[0])
//        // position = StartingPosition.LEFT;
//        // else if(maxBlue == blue[1])
//        // position = StartingPosition.CENTER;
//        // else
//        // position = StartingPosition.RIGHT;
//        // }
//
//
//        Imgproc.rectangle(frame, rectLeft, new Scalar(0, 0, 255));
//        Imgproc.rectangle(frame, rectMiddle, new Scalar(0, 0, 255));
//
//        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
//        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
//        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
//        double satRectRight = getAvgSaturation(hsvMat, rectRight);
//        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
//            return Selected.LEFT;
//        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
//            return Selected.MIDDLE;
//        }
//        return Selected.RIGHT;
////        Imgproc.rectangle(frame, leftA, leftB, new Scalar(0, 0, 255), 1);
////        Imgproc.rectangle(frame, centerA, centerB, new Scalar(0, 0, 255), 1);
////        Imgproc.rectangle(frame, rightA, rightB, new Scalar(0, 0, 255), 1);
//
////        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft
////                , scaleBmpPxToCanvasPx);
////        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle
////                , scaleBmpPxToCanvasPx);
////        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight
////                , scaleBmpPxToCanvasPx);
//
//        return null;
//    }
//
//
//    protected double getAvgSaturation(Mat input, Rect rect) {
//        submat = input.submat(rect);
//        Scalar color = Core.mean(submat);
//        return color.val[1];
//    }
//    public StartingPosition getPosition() { return position; }
//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//
//    }
//}