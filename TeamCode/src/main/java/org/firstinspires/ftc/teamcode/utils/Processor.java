package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas; import android.graphics.Color; import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration; import org.firstinspires.ftc.vision.VisionProcessor; import org.opencv.core.Core; import org.opencv.core.Mat; import org.opencv.core.Rect; import org.opencv.core.Scalar; import org.opencv.imgproc.Imgproc;

public class Processor implements VisionProcessor {
    public double satRectLeft, satRectMiddle;
    public Rect rectLeft = new Rect(0, 250, 135, 130);
    public Rect rectMiddle = new Rect(350, 210, 170, 110);
    Selected selection = Selected.NONE; Mat submat = new Mat();
    Mat hsvMat = new Mat();
    //https://www.reddit.com/r/FTC/comments/17b74hf/vision_portal_and_processor/
    @Override
    public void init(int i, int i1, CameraCalibration cameraCalibration) {

    }

    @Override
    public Object processFrame(Mat frame, long l) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        // if (satRectleft and satRectMiddle are within range of 10, select RIGHT)
        if (Math.abs(satRectLeft - satRectMiddle) < 15) {
            return Selected.RIGHT;

        } else if ((satRectLeft > satRectMiddle)) {
            return Selected.LEFT;

        } else if ((satRectMiddle > satRectLeft)) {
            return Selected.MIDDLE;
        }
        return Selected.MIDDLE;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                      float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft
                , scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle
                , scaleBmpPxToCanvasPx);
        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);

                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
        }
    }
    public Selected getSelection() {
        return selection;
    }
    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }}