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

import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;

import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MessageCallable;
import org.ros.android.view.RosImageView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import java.io.IOException;

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class MainActivity extends RosAppActivity {
	private RosImageView<sensor_msgs.CompressedImage> cameraView;
	private JoystickOnlyView joystickOnlyView;
	private Button backButton;
	private Context mContext;
	private String TAG = "teleop" ;

	private SubMessage<nav_msgs.Odometry> subOdometry;
	private PubMessage<geometry_msgs.Twist> pubTwist;
	private geometry_msgs.Twist currentVelocityCommand;

	public MainActivity() {
		// The RosActivity constructor configures the notification title and ticker messages.
		super("android teleop", "android teleop");
	}

	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {

		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main);
		super.onCreate(savedInstanceState);
		mContext = getApplicationContext();

        cameraView = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
        cameraView.setMessageType(sensor_msgs.CompressedImage._TYPE);
        cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
		joystickOnlyView = (JoystickOnlyView) findViewById(R.id.virtual_joystick);
        backButton = (Button) findViewById(R.id.back_button);
        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                onBackPressed();
            }
        });

    	///////////////subscribe Odometry////////////////////////////////////////////////////////////////////////////////////////

        subOdometry =  new SubMessage<nav_msgs.Odometry>(mContext) ;
        subOdometry.setTopicName("odom");
        subOdometry.setMessageType(nav_msgs.Odometry._TYPE);

        subOdometry.setMessageCallable(new MessageCallable< String, nav_msgs.Odometry>() {
			@Override
			public String call(nav_msgs.Odometry message) {

			    if(pubTwist.message != null) {
                    pubTwist.message.getLinear().setX(message.getPose().getPose().getOrientation().getW());
                    pubTwist.publish();
                    Log.d("test","publish");
                }
                Log.d("test","odome" + message.getPose().getPose().getOrientation().getW());
				return "test" ;
			}
		});

		////////////////publish Twist //////////////////////////////////////////////////////////////////////////////////////

        pubTwist =  new PubMessage<geometry_msgs.Twist>(mContext) ;
        pubTwist.setTopicName("/cmd_vel2");
        pubTwist.setMessageType(geometry_msgs.Twist._TYPE);

	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		
		super.init(nodeMainExecutor);

        try {
            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration =
                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());

        String camTopic = remaps.get(getString(R.string.camera_topic));

        NameResolver appNameSpace = getMasterNameSpace();
        camTopic = appNameSpace.resolve(camTopic).toString();

		cameraView.setTopicName(camTopic);
		
		nodeMainExecutor.execute(cameraView, nodeConfiguration
				.setNodeName("android/camera_view"));
		//nodeMainExecutor.execute(subOdometry, nodeConfiguration
		//		.setNodeName("android/subOdometry"));
		joystickOnlyView.onStart();

		nodeMainExecutor.execute(subOdometry,
			nodeConfiguration.setNodeName("android/subscribe_odom"));

		nodeMainExecutor.execute(pubTwist,
			nodeConfiguration.setNodeName("android/publish_twist"));

        } catch (IOException e) {
            // Socket problem
        }

	}

	public void getOdometry( double messageOdometry[] ){
		joystickOnlyView.setMessage(messageOdometry);
		Log.d(TAG,"Odometry : " + messageOdometry[0] + "," + messageOdometry[1]
				+ "," +messageOdometry[2]);
	}

	public void getTwist( double twistLinear[] , double twistArgular[] ){
		Log.d(TAG,"twist : " + twistLinear[0] + "," + twistLinear[1] +","+ twistLinear[2] +
				"/" + twistArgular[0]+ "," + twistArgular[1] + "," + twistArgular[2]);
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
