#!/usr/bin/env node

const net=require('net');
const popen = require('child_process');
const EventEmitter=require('events').EventEmitter;
const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;

let Config={
  protocol:'fanuc',
  port:3000,
  timeout:0,
  delay:100,
  update_frame_id:'J6',
  source_frame_id:'camera/master0',
  target_frame_id:'solve0'
};

setImmediate(async function(){
  const emitter=new EventEmitter();
  const rosNode=await ros.initNode('rsocket');
  try{
    let co=await rosNode.getParam('/config/rsocket');
    Object.assign(Config,co);
  }
  catch(e){
  }
  const protocol=require('./'+Config.protocol+'.js');
  Object.assign(process.env,{stdio:['pipe','pipe',2]});
  const tf_lookup=popen.exec('tf_lookup.py',{env:process.env});

//ROS events//////////////////
  rosNode.subscribe('/response/clear',std_msgs.Bool,async function(ret){
    emitter.emit('clear',ret.data);
  });
  rosNode.subscribe('/response/capture',std_msgs.Bool,async function(ret){
    emitter.emit('capture',ret.data);
  });
  rosNode.subscribe('/response/solve',std_msgs.Bool,async function(ret){
    emitter.emit('solve',ret.data);
  });
  rosNode.subscribe('/response/recipe_load',std_msgs.Bool,async function(ret){
    emitter.emit('recipe',ret.data);
  });
  const pub_tf=rosNode.advertise('/update/config_tf',geometry_msgs.TransformStamped);
  const pub_clear=rosNode.advertise('/request/clear',std_msgs.Bool);
  const pub_capture=rosNode.advertise('/request/capture',std_msgs.Bool);
  const pub_solve=rosNode.advertise('/request/solve',std_msgs.Bool);
  const pub_recipe=rosNode.advertise('/request/recipe_load',std_msgs.String);

//Socket events//////////////////
  const server = net.createServer(function(conn){
    conn.setTimeout(Config.timeout);
    let msg='';
    conn.on('data', function(data){
      msg+=data.toString();
      if(msg.indexOf('(')*msg.indexOf(')')<0) return;
      if(msg.startsWith('P1')){
        let tf=new geometry_msgs.TransformStamped();
        tf.header.stamp=ros.Time.now();
        tf.header.frame_id='world';
        tf.child_frame_id=Config.update_frame_id;
        tf.transform=protocol.decode(msg.substr(2));
        if(tf.transform!=null) pub_tf.publish(tf);
      }
      else if(msg.startsWith('X0')){   // [X0] ROVI_CLEAR
        let f=new std_msgs.Bool();
        f.data=true;
        pub_clear.publish(f);
        conn.write('OK\n');
      }
      else if(msg.startsWith('X1')){   // [X1] ROVI_CAPTURE
        let tf=new geometry_msgs.TransformStamped();
        tf.header.stamp=ros.Time.now();
        tf.header.frame_id='world';
        tf.child_frame_id=Config.update_frame_id;
        tf.transform=protocol.decode(msg.substr(2).trim());
        if(tf.transform!=null) pub_tf.publish(tf);
        setTimeout(function(){
          let f=new std_msgs.Bool();
          f.data=true;
          pub_capture.publish(f);
        },Config.delay);
        emitter.once('capture',function(ret){
          if (ret) {
            conn.write('OK\n');
          } else {
            conn.write('NG\n');
            conn.write('999');
          }
        });
      }
      else if(msg.startsWith('X2')){   // [X2] ROVI_SOLVE
        f=new std_msgs.Bool();
        f.data=true;
        pub_solve.publish(f);
        emitter.once('solve',function(ret){
          tf_lookup.stdin.write(Config.source_frame_id+' '+Config.target_frame_id+'\n');
          emitter.once('tf_transform',function(tf){
            if(tf.hasOwnProperty('translation')){
              let cod=protocol.encode(tf);
              ros.log.info('r_socket encode:'+cod);
              conn.write('OK\n');
              conn.write(cod+"\n");
            }
            else{
              conn.write('NG\n');
              conn.write('999');
            }
          });
        });
      }
      else if(msg.startsWith('X3')){   // [X3] ROVI_RECIPE
        f=new std_msgs.String();
        f.data=msg.trim().replace(/\).*/g, '').replace(/.*\(/, '');  //remove "*(" ")*"
        pub_recipe.publish(f);
        console.log("("+f.data+")");
        emitter.once('recipe',function(ret){
          if (ret) {
            conn.write('OK\n');
          } else {
            conn.write('NG\n');
            conn.write('999');
          }
        });
      }
      msg='';
    });
    conn.on('close', function(){
      ros.log.warn('r_socket CLOSED.');
    });
    conn.on('timeout',function(){
      emitter.removeAllListeners();
      conn.write('NG\n'+"'408'"); //Request timeout
      ros.log.error('r_socket TIMEOUT');
      conn.destroy();
    });
  }).listen(Config.port);

//tf_lookup events//////////////////
  tf_lookup.stdout.on('data',function(data){
    let tf=JSON.parse(data);
    emitter.emit('tf_transform',tf);
  });
});
