package com.example.vectorcontroller;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.media.Image;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ImageButton;
import android.widget.SeekBar;

import java.io.IOException;
import java.io.OutputStream;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {


    private JoystickView joystick;
    private ImageButton connectButton;
    private static final int REQUEST_BLUETOOTH_PERMISSION = 1;
    public static int brushlessMotor=50, gripper=0, midServo=0, bottomServo=60, leftSpeed=0, rightSpeed=0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);

        connectButton = (ImageButton) findViewById(R.id.imageButton);
        connectButton.setImageResource(R.drawable.unlink);
        connectButton.setBackgroundResource(0);


        //region Provjera premmisiona
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH) != PackageManager.PERMISSION_GRANTED
                || ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_ADMIN) != PackageManager.PERMISSION_GRANTED
                || ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {

            ActivityCompat.requestPermissions(this, new String[]{
                    Manifest.permission.BLUETOOTH,
                    Manifest.permission.BLUETOOTH_ADMIN,
                    Manifest.permission.BLUETOOTH_CONNECT,
                    Manifest.permission.ACCESS_FINE_LOCATION
            }, REQUEST_BLUETOOTH_PERMISSION);
        }
        //endregion



        //region Manualna kontrola servo motora
        SeekBar seekBarBrushlessMotor = (SeekBar) findViewById(R.id.seekBarBrushlessMotor);
        SeekBar seekBarGripper = (SeekBar) findViewById(R.id.seekBarGripper);
        SeekBar seekBarMidServo = (SeekBar) findViewById(R.id.seekBarMidServo);
        SeekBar seekBarBottomServo = (SeekBar) findViewById(R.id.seekBarBottomServo);
        SeekBar seekBarSpeed = (SeekBar) findViewById(R.id.seekBarSpeed);

        seekBarBrushlessMotor.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                brushlessMotor = (int)Math.round(0.4 * i + 50);
                Log.d("BMT", String.valueOf(brushlessMotor));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        seekBarGripper.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                gripper=(int) Math.round(i * 1.1) ;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        seekBarMidServo.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                midServo = (int)Math.round(i * 1.8) ;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        seekBarBottomServo.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
               bottomServo=(int) Math.round(i * 1.2+10);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        //endregion

        joystick = (JoystickView) findViewById(R.id.joystick);
        joystick.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int angle, int strength) {
                float speedFactor = 1500 * ((float) seekBarSpeed.getProgress() / 5) * ((float) strength / 100);
                float leftSpeedTemp = speedFactor, rightSpeedTemp = speedFactor;

                if (angle <= 10 || angle >= 350) {
                    rightSpeedTemp = -speedFactor;
                } else if (angle < 80) {
                    rightSpeedTemp = speedFactor * (angle - 10) / 80;
                } else if (angle < 100) {

                } else if (angle < 170) {
                    leftSpeedTemp = speedFactor * Math.abs(angle - 170) / 80;
                } else if (angle < 190) {
                    leftSpeedTemp = -speedFactor;
                } else if (angle < 260) {
                    leftSpeedTemp = -speedFactor * (angle - 190) / 80;
                    rightSpeedTemp = -speedFactor;
                } else if (angle < 280) {
                    leftSpeedTemp = -speedFactor;
                    rightSpeedTemp = -speedFactor;
                } else {
                    rightSpeedTemp = -speedFactor * Math.abs(angle - 350) / 80;
                    leftSpeedTemp = -speedFactor;
                }
                leftSpeed =(int)Math.round(leftSpeedTemp);
                rightSpeed =(int)Math.round(rightSpeedTemp);
            }
        });
        MyAsyncTask myTask = new MyAsyncTask();
        myTask.execute();
    }
/*
    @SuppressLint("MissingPermission")
    public void sendBTMessage(String message) {
        if (!bluetoothSocket.isConnected()) {
            try {
                bluetoothSocket.connect();
            } catch (IOException e) {
                connectButton.setImageResource(R.drawable.unlink);
                e.printStackTrace();
            }
        }
        OutputStream outputStream;
        try {
            outputStream = bluetoothSocket.getOutputStream();
            outputStream.write(message.getBytes());
            outputStream.flush();
        } catch (IOException ex) {
            connectButton.setImageResource(R.drawable.unlink);
            ex.printStackTrace();
        }

    }*/



    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == REQUEST_BLUETOOTH_PERMISSION) {
            // Handle the permission request result here
        }
    }

    public class MyAsyncTask extends AsyncTask<Void, Void, Void> {

        private boolean isRunning;

        public MyAsyncTask() {
            isRunning = true;
        }

        @SuppressLint("MissingPermission")
        @Override
        protected Void doInBackground(Void... voids) {
             BluetoothAdapter bluetoothAdapter;
             BluetoothDevice esp32Device;
             BluetoothSocket bluetoothSocket = null;

            int oldbrushlessMotor=0, oldgripper=0, oldmidServo=0, oldbottomServo=60, oldleftSpeed=0, oldrightSpeed=0;
            long lastBTReset=0;
            while (isRunning) {
                if(lastBTReset+60000 <System.currentTimeMillis()){
                    //region Inicijalizacija bluetooth-a
                    bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
                    if (bluetoothAdapter == null) {
                        Log.w("MOJ ERROR", "Nema Adaptera");
                    }

                    esp32Device = bluetoothAdapter.getRemoteDevice("C8:F0:9E:A2:56:E2");



                    try {
                        bluetoothSocket = esp32Device.createRfcommSocketToServiceRecord(UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"));
                        bluetoothSocket.connect();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    connectButton.setImageResource(R.drawable.link);
                    //endregion
                    lastBTReset =System.currentTimeMillis();
                    connectButton.setImageResource(R.drawable.link);
                }




                String message = "";
                if(brushlessMotor!=oldbrushlessMotor){
                    message += "S" + brushlessMotor + "\n";
                    oldbrushlessMotor= brushlessMotor;
                }
                if(gripper!=oldgripper){
                    message += "G" + gripper + "\n";
                    oldgripper = gripper;
                }
                if(midServo!=oldmidServo){
                    message += "M" + midServo + "\n";
                    oldmidServo=midServo;
                }
                if(bottomServo!=oldbottomServo){
                    message += "B" + bottomServo + "\n";
                    oldbottomServo=bottomServo;
                }
                if(leftSpeed!=oldleftSpeed){
                    message += "L" + leftSpeed + "\n";
                    oldleftSpeed=leftSpeed;
                }
                if(rightSpeed!=oldrightSpeed){
                    message += "R" + rightSpeed + "\n";
                    oldrightSpeed=rightSpeed;
                }
                if (!message.equals("")) {
                    if (bluetoothSocket==null || !bluetoothSocket.isConnected()) {
                        try {
                            bluetoothSocket.connect();
                            connectButton.setImageResource(R.drawable.link);
                        } catch (IOException e) {
                            connectButton.setImageResource(R.drawable.unlink);
                            e.printStackTrace();
                        }
                    }
                    OutputStream outputStream;
                    try {
                        outputStream = bluetoothSocket.getOutputStream();
                        outputStream.write(message.getBytes());
                        outputStream.flush();
                        connectButton.setImageResource(R.drawable.link);
                    } catch (IOException ex) {
                        connectButton.setImageResource(R.drawable.unlink);
                        ex.printStackTrace();
                    }

                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            return null;
        }

        public void stop() {
            isRunning = false;
        }
    }
}