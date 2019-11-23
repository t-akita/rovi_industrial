#!/usr/bin/env node

const net=require('net');
const EventEmitter=require('events').EventEmitter;
const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;

let Config={
  address:'127.0.0.1',
  port:8888,
};

setImmediate(async function(){
  const emitter=new EventEmitter();
  const rosNode=await ros.initNode('rjoint');
  const pub_js=rosNode.advertise('/joint_states',sensor_msgs.JointState);
  const pub_conn=rosNode.advertise('/rjoint/stat',std_msgs.Bool);
  const joint=new sensor_msgs.JointState();
  joint.name=['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

  const client = new net.Socket();
  const cstat=new std_msgs.Bool();
  client.connect(Config.port,Config.address);
  client.on('connect', function(data) {
  	console.log('Joint::streaming::Connected');
    let f=new std_msgs.Bool();
    cstat.data=true;
    pub_conn.publish(cstat);
  });
  client.on('data', function(data) {
    let j=JSON.parse('['+data+']');
    joint.header.seq++;
    joint.header.stamp = ros.Time.now();
    joint.position=j;
    pub_js.publish(joint);
  });
  client.on('close', function() {
	  if(cstat.data) console.log('Joint::streaming::closed');
    cstat.data=false;
    pub_conn.publish(cstat);
    setTimeout(function(){
      client.connect(Config.port,Config.address);
    },1000);
  });
  client.on('error', function() {
	  if(cstat.data) console.log('Joint::streaming::error');
  });
});
