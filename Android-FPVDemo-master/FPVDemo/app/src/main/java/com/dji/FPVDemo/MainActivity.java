package com.dji.FPVDemo;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.TextureView.SurfaceTextureListener;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.dji.FPVDemo.renderer.Renderer3D;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import dji.common.camera.SettingsDefinitions;
import dji.common.camera.SystemState;
import dji.common.error.DJIError;
import dji.common.flightcontroller.simulator.SimulatorState;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.product.Model;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import dji.sdk.useraccount.UserAccountManager;

public class MainActivity extends Activity implements SurfaceTextureListener,OnClickListener{

    private static final String TAG = MainActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;

    // Codec for video live view
    protected DJICodecManager mCodecManager = null;

    protected TextureView mVideoSurface = null;
    protected ImageView mImageSurface;
    private Button mCaptureBtn, mShootPhotoModeBtn, mRecordVideoModeBtn;
    private ToggleButton mRecordBtn;
    private TextView recordingTime;
    private Handler handler;


    private Mat rgb;
    private Mat gray;
    private CameraBridgeViewBase camera;
    private MatOfInt ids;
    private List<Mat> corners;
    private Dictionary dictionary;
    private DetectorParameters parameters;
    private Renderer3D renderer;
    double rtx, rty,rtz,rtpitch,rtroll,rtyaw;
    double p11,p12,p21,p22,p31,p32,p41,p42;
    double p11f,p12f,p21f,p22f,p31f,p32f,p41f,p42f;
    double per_px_r=0,per_py_r=0,per_px_bl=0,per_py_bl=0,per_px_g=0,per_py_g=0,per_px_b=0,per_py_b=0;
    int w,h;

    private FlightController mFlightController;
    private TextView mTextView;

    private final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(this){
        @Override
        public void onManagerConnected(int status){
            if(status == LoaderCallbackInterface.SUCCESS){
                String message = "";
                Toast.makeText(MainActivity.this,  message,  Toast.LENGTH_SHORT).show();
            }
            else {
                super.onManagerConnected(status);
            }
        }
    };



    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        renderer = new Renderer3D(this);

        handler = new Handler();

        initUI();

        // The callback for receiving the raw H264 video data for camera live view
        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if (mCodecManager != null) {
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };

        Camera camera = FPVDemoApplication.getCameraInstance();

        if (camera != null) {

            camera.setSystemStateCallback(new SystemState.Callback() {
                @Override
                public void onUpdate(SystemState cameraSystemState) {
                    if (null != cameraSystemState) {

                        int recordTime = cameraSystemState.getCurrentVideoRecordingTimeInSeconds();
                        int minutes = (recordTime % 3600) / 60;
                        int seconds = recordTime % 60;

                        final String timeString = String.format("%02d:%02d", minutes, seconds);
                        final boolean isVideoRecording = cameraSystemState.isRecording();

                        MainActivity.this.runOnUiThread(new Runnable() {

                            @Override
                            public void run() {

                                recordingTime.setText(timeString);

                                /*
                                 * Update recordingTime TextView visibility and mRecordBtn's check state
                                 */
                                if (isVideoRecording){
                                    recordingTime.setVisibility(View.VISIBLE);
                                }else
                                {
                                    recordingTime.setVisibility(View.INVISIBLE);
                                }
                            }
                        });
                    }
                }
            });

        }

    }

    protected void onProductChange() {
        initPreviewer();
    }


    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initPreviewer();
        onProductChange();
        if(OpenCVLoader.initDebug())
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);

        if(mVideoSurface == null) {
            Log.e(TAG, "mVideoSurface is null");
        }
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        uninitPreviewer();
        super.onPause();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    public void onReturn(View view){
        Log.e(TAG, "onReturn");
        this.finish();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        uninitPreviewer();
        super.onDestroy();
    }

    private void initUI() {
        // init mVideoSurface
        mVideoSurface = (TextureView)findViewById(R.id.video_previewer_surface);
        mImageSurface = (ImageView) findViewById(R.id.flight_image_previewer_surface);
        recordingTime = (TextView) findViewById(R.id.timer);
        mCaptureBtn = (Button) findViewById(R.id.btn_capture);
        mRecordBtn = (ToggleButton) findViewById(R.id.btn_record);
        mShootPhotoModeBtn = (Button) findViewById(R.id.btn_shoot_photo_mode);
        mRecordVideoModeBtn = (Button) findViewById(R.id.btn_record_video_mode);

        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);
        }

        mCaptureBtn.setOnClickListener(this);
        mRecordBtn.setOnClickListener(this);
        mShootPhotoModeBtn.setOnClickListener(this);
        mRecordVideoModeBtn.setOnClickListener(this);

        recordingTime.setVisibility(View.INVISIBLE);

        mRecordBtn.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    startRecord();
                } else {
                    stopRecord();
                }
            }
        });
    }

    private void initPreviewer() {

        BaseProduct product = FPVDemoApplication.getProductInstance();

        if (product == null || !product.isConnected()) {
            showToast(getString(R.string.disconnected));
        } else {
            if (null != mVideoSurface) {
                mVideoSurface.setSurfaceTextureListener(this);
            }
            if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
            }
        }
    }

    private void uninitPreviewer() {
        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null){
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {

        Log.e(TAG, "onSurfaceTextureAvailable");
        if (mCodecManager == null) {
            mCodecManager = new DJICodecManager(this, surface, width, height);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureSizeChanged");
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        Log.e(TAG,"onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }

        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        ArucoDectector();


    }


    private void ArucoDectector() {
        rgb = new Mat();
        corners = new LinkedList<>();
        ids = new MatOfInt();
        corners.clear();
        parameters = DetectorParameters.create();
        dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_50);
        Bitmap sourceBitmap = Bitmap.createScaledBitmap(mVideoSurface.getBitmap(),1200,605, false);
        Mat droneImage = new Mat();
        Mat grayImage = new Mat();

        Utils.bitmapToMat(sourceBitmap, droneImage);
        Imgproc.cvtColor(droneImage, grayImage, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(droneImage, rgb, Imgproc.COLOR_RGBA2RGB);

        Aruco.detectMarkers(grayImage, dictionary, corners, ids, parameters);

        if(corners.size()>0){

            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Mat rvec1 = new Mat();
            Mat tvec1 = new Mat();

 // Camera Matrix 2

            Mat cameraMatrix = Mat.zeros(3, 3, CvType.CV_64F);
            cameraMatrix.put(0, 0, 2766.93281); //fx
            cameraMatrix.put(1, 1, 2811.88175); //fy
            cameraMatrix.put(0, 2, 1942.48783); //cx
            cameraMatrix.put(1, 2, 936.313862); //cy
            cameraMatrix.put(2, 2, 1);

 // Distorstion coefficients

            Mat distCoeffs = Mat.zeros(5, 1, CvType.CV_64F);
            distCoeffs.put(0,0, 0.38628748);
            distCoeffs.put(1,0,-0.81343147);
            distCoeffs.put(2,0,-0.03587456);
            distCoeffs.put(3,0, -0.01739278);
            distCoeffs.put(4,0, 1.03496557);

            //Objects Points

            MatOfPoint3f objPoints = new MatOfPoint3f(new Point3(-86.5, 86.5, 0), new Point3(86.5, 86.5, 0),
                    new Point3(-86.5, -86.5, 0),new Point3(86.5, -86.5, 0));

            Aruco.drawDetectedMarkers(rgb, corners, ids);

            Aruco.estimatePoseSingleMarkers(corners, 0.04f, cameraMatrix, distCoeffs, rvecs, tvecs);

            List<Point3> corners4 = new ArrayList<>(4);
            corners4.add(new Point3(-0.02f,0.02f,0));		// Top-Left
            corners4.add(new Point3(0.02f,0.02f,0));		// Top-Right
            corners4.add(new Point3(-0.02f,-0.02f,0));		// Bottom-Left
            corners4.add(new Point3(0.02f,-0.02f,0));		// Bottom-Right

            MatOfPoint3f mcorners = new MatOfPoint3f();
            mcorners.fromList(corners4);


            for(int i = 0;i<ids.toArray().length;i++){

                transformModel(tvecs.row(0), rvecs.row(0));
                Aruco.drawAxis(rgb, cameraMatrix, distCoeffs, rvecs.row(i), tvecs.row(i), 0.02f);

                distCoeffs = new MatOfDouble(distCoeffs);

                MatOfPoint2f projected = new MatOfPoint2f();
                Calib3d.projectPoints(mcorners, rvecs.row(i), tvecs.row(i), cameraMatrix, (MatOfDouble) distCoeffs, projected);

                Point[] points = projected.toArray();
                if(points != null){
                    for(Point point:points){
                        Imgproc.circle(rgb, point, 10, new Scalar(0, 255, 0, 150), 4);
                        p11 = (int) points[0].x;
                        p12 = (int) points[0].y;
                        p21 = (int) points[1].x;
                        p22 = (int) points[1].y;
                        p31 = (int) points[2].x;
                        p32 = (int) points[2].y;
                        p41 = (int) points[3].x;
                        p42 = (int) points[3].y;
                    }
                }

 //Image Points
                w = 1200;
                h = 605;

                per_px_r = (100*p11)/w;
                per_py_r = (100*p12)/h;

                per_px_bl = (100*p21)/w;
                per_py_bl = (100*p22)/h;

                per_px_g = (100*p31)/w;
                per_py_g = (100*p32)/h;

                per_px_b = (100*p41)/w;
                per_py_b = (100*p42)/h;

                p11f = (per_px_r*4000)/100;
                p12f = (per_py_r*2250)/100;

                p21f = (per_px_bl*4000)/100;
                p22f = (per_py_bl*2250)/100;

                p31f = (per_px_g*4000)/100;
                p32f = (per_py_g*2250)/100;

                p41f = (per_px_b*4000)/100;
                p42f = (per_py_b*2250)/100;

                MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(p11f, p12f),
                        new Point(p21f, p22f), new Point(p31f, p32f), new Point(p41f, p42f));

//     SolvePNP();

                Calib3d.solvePnP(objPoints, imagePoints, cameraMatrix, (MatOfDouble) distCoeffs, rvec1, tvec1);

                Mat rotationMatrix = new Mat();
                Calib3d.Rodrigues (rvec1, rotationMatrix);
                Mat projectionMatrix = new Mat(4, 4, CvType.CV_64F);
                projectionMatrix.put(0,0,
                rotationMatrix.get(0, 0)[0],rotationMatrix.get(0, 1)[0],rotationMatrix.get(0, 2)[0],tvec1.get(0, 0)[0],
                rotationMatrix.get(1, 0)[0],rotationMatrix.get(1, 1)[0],rotationMatrix.get(1, 2)[0], tvec1.get(1, 0)[0],
                rotationMatrix.get(2, 0)[0],rotationMatrix.get(2, 1)[0],rotationMatrix.get(2, 2)[0], tvec1.get(2, 0)[0],
                0, 0, 0, 1);

                Mat b1 = new Mat(3, 4, CvType.CV_64F);

                Mat b = projectionMatrix.inv();
                b1.put(0, 0,
                b.get(0, 0)[0], b.get(0, 1)[0], b.get(0, 2)[0], b.get(0, 3)[0],
                b.get(1, 0)[0], b.get(1, 1)[0], b.get(1, 2)[0], b.get(1, 3)[0],
                b.get(2, 0)[0], b.get(2, 1)[0], b.get(2, 2)[0], b.get(2, 3)[0]
        );
                Mat cameraMatrix1 = new Mat();
                Mat rotMatrix = new Mat();
                Mat transVect = new Mat();
                Mat rotMatrixX = new Mat();
                Mat rotMatrixY = new Mat();
                Mat rotMatrixZ = new Mat();
                Mat eulerAngles = new Mat();

                Calib3d.decomposeProjectionMatrix(b1, cameraMatrix1, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);

                rtx = (b1.get(0, 3)[0])/1000;
                rty = (b1.get(1, 3)[0])/1000;
                rtz = (b1.get(2, 3)[0])/1000;
                rtroll = (eulerAngles.get(2,0)[0]);
                rtpitch = (eulerAngles.get(0,0)[0]);
                rtyaw = (eulerAngles.get(1,0)[0]);

                TextView theTextView1 = (TextView) findViewById(R.id.textView1);
                TextView theTextView2 = (TextView) findViewById(R.id.textView2);
                TextView theTextView3 = (TextView) findViewById(R.id.textView3);
                TextView theTextView4 = (TextView) findViewById(R.id.textView4);
                TextView theTextView5 = (TextView) findViewById(R.id.textView5);
                TextView theTextView6 = (TextView) findViewById(R.id.textView6);

                theTextView1.setText("X: " + String.format("%.2f",rtx));
                theTextView1.setTextColor(Color.RED);

                theTextView2.setText("Y: " +  String.format("%.2f",rty));
                theTextView2.setTextColor(Color.RED);

                theTextView3.setText("Z: " +  String.format("%.2f",rtz));
                theTextView3.setTextColor(Color.RED);

                theTextView4.setText("Roll: " +  String.format("%.2f",rtroll) );
                theTextView4.setTextColor(Color.RED);

                theTextView5.setText( "Pitch: "  + String.format("%.2f",rtpitch));
                theTextView5.setTextColor(Color.RED);

                theTextView6.setText("Yaw: "  + String.format("%.2f",rtyaw));
                theTextView6.setTextColor(Color.RED);
//                TextView theTextView26 = (TextView) findViewById(R.id.textView26);
//                theTextView26.setText(projectionMatrix.dump());
//                theTextView26.setTextColor(Color.RED);
                projected.release();

            }

        }

        Bitmap bmpImageSurface = Bitmap.createBitmap(rgb.cols(),rgb.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(rgb, bmpImageSurface);
        Bitmap displayBitmap = Bitmap.createScaledBitmap(bmpImageSurface, 1200,605, false);
        mImageSurface.setImageBitmap(null);
        mImageSurface.setImageBitmap(displayBitmap);

    }

    private void transformModel(final Mat tvec, final Mat rvec){
        runOnUiThread(new Runnable(){
            @Override
            public void run(){
                renderer.transform(
                        tvec.get(0, 0)[0]*50,
                        -tvec.get(0, 0)[1]*50,
                        -tvec.get(0, 0)[2]*50,

                        rvec.get(0, 0)[2], //yaw
                        rvec.get(0, 0)[1], //pitch
                        rvec.get(0, 0)[0] //roll

                );

            }
        });
    }


    public void showToast(final String msg) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });
    }

    @Override
    public void onClick(View v) {

        switch (v.getId()) {
            case R.id.btn_capture:
                captureAction();
                break;
            case R.id.btn_shoot_photo_mode:
                if (isMavicAir2() || isM300()) {
                    switchCameraFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_SINGLE);
                }else {
                    switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO);
                }
                break;
            case R.id.btn_record_video_mode:
                if (isMavicAir2() || isM300()) {
                    switchCameraFlatMode(SettingsDefinitions.FlatCameraMode.VIDEO_NORMAL);
                }else {
                    switchCameraMode(SettingsDefinitions.CameraMode.RECORD_VIDEO);
                }
                break;
            default:
                break;
        }
    }

    private void switchCameraFlatMode(SettingsDefinitions.FlatCameraMode flatCameraMode){
        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            camera.setFlatMode(flatCameraMode, error -> {
                if (error == null) {
                    showToast("Switch Camera Flat Mode Succeeded");
                } else {
                    showToast(error.getDescription());
                }
            });
        }
    }

    private void switchCameraMode(SettingsDefinitions.CameraMode cameraMode){
        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            camera.setMode(cameraMode, error -> {
                if (error == null) {
                    showToast("Switch Camera Mode Succeeded");
                } else {
                    showToast(error.getDescription());
                }
            });
        }
    }

    // Method for taking photo
    private void captureAction(){
        final Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            if (isMavicAir2() || isM300()) {
                camera.setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_SINGLE, djiError -> {
                    if (null == djiError) {
                        takePhoto();
                    }
                });
            }else {
                camera.setShootPhotoMode(SettingsDefinitions.ShootPhotoMode.SINGLE, djiError -> {
                    if (null == djiError) {
                        takePhoto();
                    }
                });
            }
        }
    }

    private void takePhoto(){
        final Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera == null){
            return;
        }
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                camera.startShootPhoto(djiError -> {
                    if (djiError == null) {
                        showToast("take photo: success");
                    } else {
                        showToast(djiError.getDescription());
                    }
                });
            }
        }, 2000);
    }

    // Method for starting recording
    private void startRecord(){

        final Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            camera.startRecordVideo(djiError -> {
                if (djiError == null) {
                    showToast("Record video: success");
                }else {
                    showToast(djiError.getDescription());
                }
            }); // Execute the startRecordVideo API
        }
    }

    // Method for stopping recording
    private void stopRecord(){

        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            camera.stopRecordVideo(djiError -> {
                if(djiError == null) {
                    showToast("Stop recording: success");
                }else {
                    showToast(djiError.getDescription());
                }
            }); // Execute the stopRecordVideo API
        }
    }

    private boolean isMavicAir2(){
        BaseProduct baseProduct = FPVDemoApplication.getProductInstance();
        if (baseProduct != null) {
            return baseProduct.getModel() == Model.MAVIC_AIR_2;
        }
        return false;
    }

    private boolean isM300(){
        BaseProduct baseProduct = FPVDemoApplication.getProductInstance();
        if (baseProduct != null) {
            return baseProduct.getModel() == Model.MATRICE_300_RTK;
        }
        return false;
    }
}
