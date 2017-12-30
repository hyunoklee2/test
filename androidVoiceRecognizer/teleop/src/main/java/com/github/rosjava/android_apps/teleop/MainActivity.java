/*
 * Copyright (C) 2013 OSRF.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.android_apps.teleop;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.speech.RecognizerIntent;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.view.RosImageView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Locale;

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class MainActivity extends RosAppActivity {
	private RosImageView<sensor_msgs.CompressedImage> cameraView;
	private ExtraVirtualJoystickView virtualJoystickView;
	private Button backButton;
	private Button publishButton;
	private EditText sendTopic;
	private EditText sendCommand;
	private String TAG = "RosApp" ;
	private NodeMainExecutor gnodeMainExecutor ;
	private NodeConfiguration nodeConfiguration ;
	private Boolean voiceConrol = false ;
	private TextView txtSpeechInput;
	private ImageButton btnSpeak;
	private final int REQ_CODE_SPEECH_INPUT = 100;
	private LinearLayout side_layout;

	final String forward = "FORWARD" ;
	final String backward = "BACKWARD" ;
	final String right = "RIGHT" ;
	final String left = "LEFT" ;
	final String stop = "STOP" ;
	final String forwardArray[] = {  "앞으로", "앞", "고" ,"go","front","forward" } ;
	final String backwardArray[] = { "뒤로" , "뒤", "빽", "빼" , "back","rear"} ;
	final String leftArray[] = { "왼쪽" , "왼","left"  } ;
	final String rightArray[] = { "오른쪽" , "오른" ,"right" } ;
	final String stopArray[] = { "멈춰" , "정지", "스탑" ,"stop" } ;
	final String korean = "Korean" ;
	final String english = "English" ;
	private String setLanguage = korean ;
	private Spinner s ;

	public MainActivity() {
		// The RosActivity constructor configures the notification title and ticker messages.
		super("android teleop", "android teleop");
	}

	@SuppressLint("WrongViewCast")
	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {

		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main);
		super.onCreate(savedInstanceState);

        cameraView = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
        cameraView.setMessageType(sensor_msgs.CompressedImage._TYPE);
        cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        virtualJoystickView = (ExtraVirtualJoystickView) findViewById(R.id.virtual_joystick);
        backButton = (Button) findViewById(R.id.back_button);
		publishButton = (Button) findViewById(R.id.publish_button);
		sendTopic = (EditText) findViewById(R.id.send_topic);
		sendCommand = (EditText) findViewById(R.id.send_command);
		side_layout = (LinearLayout) findViewById(R.id.side_layout);
		txtSpeechInput = (TextView) findViewById(R.id.txtSpeechInput);
		btnSpeak = (ImageButton) findViewById(R.id.btnSpeak);
		s = (Spinner)findViewById(R.id.spinner1);

        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                onBackPressed();
            }
        });
		publishButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				String sTopic = sendTopic.getText().toString();
				String sCommand = sendCommand.getText().toString();
				if (sTopic.isEmpty()||sCommand.isEmpty()){
					Toast.makeText(getApplicationContext(), "there is no text.", Toast.LENGTH_SHORT).show();
				}else {
					virtualJoystickView.setExtraTopicName(sendTopic.getText().toString());
					virtualJoystickView.setExtraPublish(sendCommand.getText().toString());
				}
			}
		});

		Switch sw = (Switch)findViewById(R.id.voice_set_switch);
		sw.setOnCheckedChangeListener(new OnCheckedChangeListener() {
			@Override
			public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
				voiceConrol = isChecked;
				virtualJoystickView.setVoiceConrol( voiceConrol );
				if(voiceConrol){
					virtualJoystickView.setVisibility(View.INVISIBLE);
					btnSpeak.setVisibility(View.VISIBLE);
					txtSpeechInput.setVisibility(View.VISIBLE);
					s.setVisibility(View.VISIBLE);
				}else{
					virtualJoystickView.setVisibility(View.VISIBLE);
					btnSpeak.setVisibility(View.INVISIBLE);
					txtSpeechInput.setVisibility(View.INVISIBLE);
					s.setVisibility(View.INVISIBLE);
				}
			}
		});

		btnSpeak.setOnClickListener(new View.OnClickListener() {

			@Override
			public void onClick(View v) {
				promptSpeechInput();
			}
		});

		setLanguage = korean;
		s.setSelection(0);
		s.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
			@Override
			public void onItemSelected(AdapterView<?> parent, View view,
									   int position, long id) {

				if( parent.getItemAtPosition(position).toString().equals(korean)){
					setLanguage = korean;
				}else if( parent.getItemAtPosition(position).toString().equals(english)){
					setLanguage = english;
				}
			}
			@Override
			public void onNothingSelected(AdapterView<?> parent) {}
		});

	}



	/**
	 * Showing google speech input dialog
	 * */
	private void promptSpeechInput() {

		Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
		intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
				RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
		//intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault());
		if( setLanguage.equals(korean)) {
			intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "ko-KR");
		}else if( setLanguage.equals(english)) {
			intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "en-US");
		}

		intent.putExtra(RecognizerIntent.EXTRA_PROMPT,
				getString(R.string.speech_prompt));
		try {
			startActivityForResult(intent, REQ_CODE_SPEECH_INPUT);
		} catch (ActivityNotFoundException a) {
			Toast.makeText(getApplicationContext(),
					getString(R.string.speech_not_supported),
					Toast.LENGTH_SHORT).show();
		}
		virtualJoystickView.publishVoiceVelocity(0,0,0);
	}

	/**
	 * Receiving speech input
	 * */
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {

		switch (requestCode) {
			case REQ_CODE_SPEECH_INPUT: {
				if (resultCode == RESULT_OK && null != data) {

					ArrayList<String> result = data
							.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
					String sentance = result.get(0) ;
					String controlDirection = judgeDirection ( sentance );
					txtSpeechInput.setText( controlDirection + "_" + sentance);
					virtualJoystickView.setVoicePublish(sentance);
					if( voiceConrol ){
						voiceContorlDiretion( controlDirection );
					}
				}
				break;
			}
			default:
				super.onActivityResult(requestCode, resultCode, data);
				break;
		}
	}

	public void voiceContorlDiretion( String controlDirection ){

		double linearVelocityX  = 0, linearVelocityY = 0 , angularVelocityZ = 0;

			if(controlDirection.equals(forward)){
				linearVelocityX = 0.5 ;
				angularVelocityZ = 0 ;
			}else if(controlDirection.equals(backward)){
				linearVelocityX = -0.5 ;
				angularVelocityZ = 0 ;
			}else if(controlDirection.equals(left)){
				linearVelocityX = 0 ;
				angularVelocityZ = 0.5 ;
			}else if(controlDirection.equals(right)){
				linearVelocityX = 0 ;
				angularVelocityZ = -0.5 ;
			}else if(controlDirection.equals(stop)){
				linearVelocityX = 0 ;
				angularVelocityZ = 0 ;
			}

		virtualJoystickView.publishVoiceVelocity( linearVelocityX, linearVelocityY,angularVelocityZ );

	}

	public String judgeDirection( String sentance ){

		String currentControlState = stop;
		int i = 0 ;
		for( i = 0 ; i<forwardArray.length ; i++ ){
			if( sentance.contains(forwardArray[i]) ){
				currentControlState = forward ;
				break;
			}
		}
		for( i = 0 ; i<backwardArray.length ; i++ ){
			if( sentance.contains(backwardArray[i]) ){
				currentControlState = backward ;
				break;
			}
		}
		for( i = 0 ; i<leftArray.length ; i++ ){
			if( sentance.contains(leftArray[i]) ){
				currentControlState = left ;
				break;
			}
		}
		for( i = 0 ; i<rightArray.length ; i++ ){
			if( sentance.contains(rightArray[i]) ){
				currentControlState = right ;
				break;
			}
		}
		for( i = 0 ; i<stopArray.length ; i++ ){
			if( sentance.contains(stopArray[i]) ){
				currentControlState = stop ;
				break;
			}
		}
		return currentControlState ;
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		super.init(nodeMainExecutor);

        try {
            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            nodeConfiguration =
                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());

        String joyTopic = remaps.get(getString(R.string.joystick_topic));
        String joyExtraTopic = remaps.get(getString(R.string.joystick_extratopic));
        String joyVoiceTopic = remaps.get(getString(R.string.joystick_voicetopic));
        String camTopic = remaps.get(getString(R.string.camera_topic));

        NameResolver appNameSpace = getMasterNameSpace();
        joyTopic = appNameSpace.resolve(joyTopic).toString();
        camTopic = appNameSpace.resolve(camTopic).toString();
        joyVoiceTopic = appNameSpace.resolve(joyVoiceTopic).toString();
        joyExtraTopic = appNameSpace.resolve(joyExtraTopic).toString();

		cameraView.setTopicName(camTopic);
        virtualJoystickView.setTopicName(joyTopic, joyExtraTopic, joyVoiceTopic);

        //gnodeMainExecutor = nodeMainExecutor;
		nodeMainExecutor.execute(cameraView, nodeConfiguration
				.setNodeName("android/camera_view"));
		nodeMainExecutor.execute(virtualJoystickView,
				nodeConfiguration.setNodeName("android/virtual_joystick"));

        } catch (IOException e) {
            // Socket problem
        }

	}
	
	  @Override
	  public boolean onCreateOptionsMenu(Menu menu){
		  menu.add(0,0,0,R.string.stop_app);

		  return super.onCreateOptionsMenu(menu);
	  }
	  
	  @Override
	  public boolean onOptionsItemSelected(MenuItem item){
		  super.onOptionsItemSelected(item);
		  switch (item.getItemId()){
		  case 0:
			  onDestroy();
			  break;
		  }
		  return true;
	  }
}
