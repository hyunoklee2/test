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
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;

import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MessageCallable;
import org.ros.android.view.RosImageView;
import org.ros.android.view.VirtualJoystickView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

//import std_msgs.String;

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class MainActivity extends RosAppActivity {
	private RosImageView<sensor_msgs.CompressedImage> cameraView;
	private VirtualJoystickView virtualJoystickView;
	private Button backButton;
	private RosTextView<std_msgs.String> rosTextView;
	private RosTextView2<nav_msgs.Odometry> rosTextView2;
	private RosTextView3<geometry_msgs.Twist> rosTextView3;
	private geometry_msgs.Twist currentVelocityCommand;
	private Context mContext;

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

        cameraView = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
        cameraView.setMessageType(sensor_msgs.CompressedImage._TYPE);
        cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);
        backButton = (Button) findViewById(R.id.back_button);
        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                onBackPressed();
            }
        });

		mContext = getApplicationContext() ;

		////////////////origin subcribe///////////////////////////////////////////////////////////////////////////////

		rosTextView =  new RosTextView<std_msgs.String>(mContext) ;
		rosTextView.setTopicName("chatter");
		rosTextView.setMessageType(std_msgs.String._TYPE);
		rosTextView.setMessageToStringCallable(new MessageCallable<String, std_msgs.String>() {
			@Override
			public String call(std_msgs.String message) {
				return message.getData();
			}
		});

		///////////////subscribe////////////////////////////////////////////////////////////////////////////////////////

		rosTextView2 =  new RosTextView2<nav_msgs.Odometry>(mContext) ;
		rosTextView2.setTopicName("odom");
		rosTextView2.setMessageType(nav_msgs.Odometry._TYPE);
		rosTextView2.setMessageToStringCallable(new MessageCallable< String, nav_msgs.Odometry>() {
			@Override
			public String call(nav_msgs.Odometry message) {
				//return message.getData();
				//return message.getTwist();
				return "test" ;
			}
		});

		////////////////publish//////////////////////////////////////////////////////////////////////////////////////

		rosTextView3 =  new RosTextView3<geometry_msgs.Twist>(mContext) ;
		rosTextView3.setTopicName("~cmd_vel");
		rosTextView3.setMessageType(geometry_msgs.Twist._TYPE);


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

        String joyTopic = remaps.get(getString(R.string.joystick_topic));
        String camTopic = remaps.get(getString(R.string.camera_topic));

        NameResolver appNameSpace = getMasterNameSpace();
        joyTopic = appNameSpace.resolve(joyTopic).toString();
        camTopic = appNameSpace.resolve(camTopic).toString();

		cameraView.setTopicName(camTopic);
        virtualJoystickView.setTopicName(joyTopic);
		
		nodeMainExecutor.execute(cameraView, nodeConfiguration
				.setNodeName("android/camera_view"));

		nodeMainExecutor.execute(virtualJoystickView,
				nodeConfiguration.setNodeName("android/virtual_joystick"));

		nodeMainExecutor.execute(rosTextView,
					nodeConfiguration.setNodeName("android/test1"));

		nodeMainExecutor.execute(rosTextView2,
					nodeConfiguration.setNodeName("android/test2"));

		nodeMainExecutor.execute(rosTextView3,
					nodeConfiguration.setNodeName("android/test3"));

		rosTextView3.newMessage(currentVelocityCommand);

        } catch (IOException e) {
            // Socket problem
        }

	}

	public void setVelocityCommand(double linear[], double angular[]){
		currentVelocityCommand.getLinear().setX(linear[0]);
		currentVelocityCommand.getLinear().setY(-linear[1]);
		currentVelocityCommand.getLinear().setZ(0);
		currentVelocityCommand.getAngular().setX(0);
		currentVelocityCommand.getAngular().setY(0);
		currentVelocityCommand.getAngular().setZ(-angular[2]);
		rosTextView3.publish(currentVelocityCommand);
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
