/*
 * Copyright (C) 2011 Google Inc.
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

import android.annotation.SuppressLint;
import android.content.Context;
import android.util.AttributeSet;
import android.widget.TextView;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

//import std_msgs.String;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
@SuppressLint("AppCompatCustomView")
public class RosTextView3<T> extends TextView implements NodeMain {

  private String topicName;
  private String messageType;
  private Publisher<T> publisher;

  public RosTextView3(Context context) {
    super(context);
  }

  public RosTextView3(Context context, AttributeSet attrs) {
    super(context, attrs);
  }

  public RosTextView3(Context context, AttributeSet attrs, int defStyle) {
    super(context, attrs, defStyle);
  }

  public void setTopicName(String topicName) {
    this.topicName = topicName;
  }

  public void setMessageType(String messageType) {
    this.messageType = messageType;
  }


  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("android_gingerbread/ros_text_view");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
      publisher = connectedNode.newPublisher(topicName, messageType);
      //T message = publisher.newMessage();
  }

  public T newMessage(T message) {
    message = publisher.newMessage();
    return message ;
  }

  public void publish(T message) {
      publisher.publish(message);
  }

  @Override
  public void onShutdown(Node node) {
  }

  @Override
  public void onShutdownComplete(Node node) {
  }

  @Override
  public void onError(Node node, Throwable throwable) {
  }


}
