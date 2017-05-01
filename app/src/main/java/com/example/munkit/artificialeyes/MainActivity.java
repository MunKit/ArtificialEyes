package com.example.munkit.artificialeyes;

import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.speech.tts.TextToSpeech;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.DisconnectedBufferOptions;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import java.util.Locale;

import org.xwalk.core.XWalkView;

import static android.R.color.holo_green_light;
import static android.R.color.holo_orange_light;
import static android.R.color.holo_red_light;

public class MainActivity extends AppCompatActivity implements TextToSpeech.OnInitListener, LocationListener{

    //TTS object
    private TextToSpeech myTTS;
    //status check code
    private int MY_DATA_CHECK_CODE = 0;
    private static final String TAG = "Mymessage";

    private LocationManager locationManager;
    private String mprovider;
    private Location lastlocation;

    private MqttAndroidClient mqttAndroidClient;
    private final String serverUri = "tcp://iot.eclipse.org:1883";
    private final String clientId = "myAndClient";
    private final String subchannel = "artificialeyes/message";
    private final String subchannel2 = "artificialeyes/phone";
    private final String pubchannel = "artificialeyes/location";
    //private final XWalkView mXWalkView = (XWalkView) findViewById(R.id.webview);
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //init walkview website viewer
        XWalkView mXWalkView = (XWalkView) findViewById(R.id.webview);
        mXWalkView.load("http://zhuatang.com", null);
        //init goople location
        locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        Criteria criteria = new Criteria();

        mprovider = locationManager.getBestProvider(criteria, false);

        if (mprovider != null && !mprovider.equals("")) {
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                return;
            }
            Location location = locationManager.getLastKnownLocation(mprovider);
            locationManager.requestLocationUpdates(mprovider, 15000, 1, this);

            if (location != null)
                onLocationChanged(location);
            else
                Toast.makeText(getBaseContext(), "No Location Provider Found Check Your Code", Toast.LENGTH_SHORT).show();
        }
        //init text to speech
        Intent checkTTSIntent = new Intent();
        checkTTSIntent.setAction(TextToSpeech.Engine.ACTION_CHECK_TTS_DATA);
        startActivityForResult(checkTTSIntent, MY_DATA_CHECK_CODE);
        //init mqtt protocol
        mqttAndroidClient = new MqttAndroidClient(getApplicationContext(), serverUri, clientId);
        mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean reconnect, String serverURI) {

                TextView respondtext = (TextView) findViewById(R.id.respondtext);
                if (reconnect) {
                    respondtext.setText("Reconnected to : " + serverURI);
                    respondtext.setTextColor(getResources().getColor(holo_orange_light));
                    //Log.i(TAG,"Reconnected to : " + serverURI);
                    // Because Clean Session is true, we need to re-subscribe
                    //subscribeToTopic(subchannel);
                } else {
                    respondtext.setText("Connected to: " + serverURI);
                    respondtext.setTextColor(getResources().getColor(holo_green_light));
                    //Log.i(TAG,"Connected to: " + serverURI);
                }
            }

            @Override
            public void connectionLost(Throwable cause) {
                TextView respondtext = (TextView) findViewById(R.id.respondtext);
                respondtext.setText("The Connection was lost.");
                respondtext.setTextColor(getResources().getColor(holo_red_light));
                //Log.i(TAG,"The Connection was lost.");
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                Log.i(TAG,"Incoming message: " + new String(message.getPayload()) + topic);
                //set transition
                //transition = true;

                if (topic.equals("artificialeyes/message"))
                    speakWords(new String(message.getPayload()));
                if (topic.equals("artificialeyes/phone")){
                    XWalkView mXWalkView = (XWalkView) findViewById(R.id.webview);
                    if (new String(message.getPayload()).equals("call")) {
                        mXWalkView.load("https://webrtcsimple-error005.c9users.io/index.html?999", null);
                        if (lastlocation != null)
                            publishMessage(pubchannel,"{lat:"+Double.toString(lastlocation.getLatitude())+",lng:"+Double.toString(lastlocation.getLongitude())+"}");
                    }
                    else
                        mXWalkView.load("http://zhuatang.com", null);
                }


            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {

            }
        });

        MqttConnectOptions mqttConnectOptions = new MqttConnectOptions();
        mqttConnectOptions.setAutomaticReconnect(true);
        mqttConnectOptions.setCleanSession(false);

        try {
            mqttAndroidClient.connect(mqttConnectOptions, null, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    DisconnectedBufferOptions disconnectedBufferOptions = new DisconnectedBufferOptions();
                    disconnectedBufferOptions.setBufferEnabled(true);
                    disconnectedBufferOptions.setBufferSize(100);
                    disconnectedBufferOptions.setPersistBuffer(false);
                    disconnectedBufferOptions.setDeleteOldestMessages(false);
                    mqttAndroidClient.setBufferOpts(disconnectedBufferOptions);
                    subscribeToTopic(subchannel);
                    subscribeToTopic(subchannel2);
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    //Log.i(TAG,"Failed to connect to: " + serverUri);
                    //respondtext.setText("Failed to connect to: " + serverUri);
                    //respondtext.setTextColor(getResources().getColor(holo_red_light));
                }
            });


        } catch (MqttException ex){
            ex.printStackTrace();
        }
    }

    private void speakWords(String speech) {

        //speak straight away
        myTTS.speak(speech, TextToSpeech.QUEUE_FLUSH, null);
    }

    //act on result of TTS data check
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {

        if (requestCode == MY_DATA_CHECK_CODE) {
            if (resultCode == TextToSpeech.Engine.CHECK_VOICE_DATA_PASS) {
                //the user has the necessary data - create the TTS
                myTTS = new TextToSpeech(this, this);
            }
            else {
                //no data - install it now
                Intent installTTSIntent = new Intent();
                installTTSIntent.setAction(TextToSpeech.Engine.ACTION_INSTALL_TTS_DATA);
                startActivity(installTTSIntent);
            }
        }
    }

    //setup TTS
    public void onInit(int initStatus) {

        //check for successful instantiation
        if (initStatus == TextToSpeech.SUCCESS) {
            if(myTTS.isLanguageAvailable(Locale.US)==TextToSpeech.LANG_AVAILABLE)
                myTTS.setLanguage(Locale.US);
        }
        else if (initStatus == TextToSpeech.ERROR) {
            Toast.makeText(this, "Sorry! Text To Speech failed...", Toast.LENGTH_LONG).show();
        }
    }
    public void subscribeToTopic(String subscriptionTopic){
        try {
            mqttAndroidClient.subscribe(subscriptionTopic, 0, null, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Log.i(TAG,"Subscribed!");

                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Log.i(TAG,"Failed to subscribe");
                }
            });

        } catch (MqttException ex){
            Log.i(TAG,"Exception whilst subscribing");
            ex.printStackTrace();
        }
    }
    public void publishMessage(String Channel, String pubmessage){

        try {
            MqttMessage message = new MqttMessage();
            message.setPayload(pubmessage.getBytes());
            mqttAndroidClient.publish(Channel, message);
            if(!mqttAndroidClient.isConnected()){
                Log.i(TAG,mqttAndroidClient.getBufferedMessageCount() + " messages in buffer.");
            }
        } catch (MqttException e) {
            Log.i(TAG,"Error Publishing: " + e.getMessage());
            e.printStackTrace();
        }
    }
    @Override
    public void onLocationChanged(Location location) {
        lastlocation = location;
        TextView longitude = (TextView) findViewById(R.id.textView);
        TextView latitude = (TextView) findViewById(R.id.textView2);

        longitude.setText("Current Longitude:" + location.getLongitude());
        latitude.setText("Current Latitude:" + location.getLatitude());
    }

    @Override
    public void onStatusChanged(String s, int i, Bundle bundle) {

    }

    @Override
    public void onProviderEnabled(String s) {

    }

    @Override
    public void onProviderDisabled(String s) {

    }
}
