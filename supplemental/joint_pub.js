#!/usr/bin/env node

const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;

setImmediate(async function(){
  const rosNode=await ros.initNode('joint_pub');
  const pub_js=rosNode.advertise('/joint_states',sensor_msgs.JointState);
  const joint=new sensor_msgs.JointState();
  joint.name=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'];
  setInterval(function(){
    joint.header.seq++;
    joint.header.stamp = ros.Time.now();
    joint.position[0]=-49.5*3.14/180;
    joint.position[1]=145.2*3.14/180;
    joint.position[2]=-28.9*3.14/180;
    joint.position[3]=0.2*3.14/180;
    joint.position[4]=-126.3*3.14/180;
    joint.position[5]=87.1*3.14/180;
    pub_js.publish(joint);
  },1000);
});
