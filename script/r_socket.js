#!/usr/bin/env node

const net=require('net');
const popen = require('child_process');
const EventEmitter=require('events').EventEmitter;
const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;

let Config={
  protocol:'fanuc',
  port:3000,
  socket_timeout:0,
  delay:100,
  capt_timeout:5,
  solve_timeout:10,
  recipe_timeout:3,
  base_frame_id:'world',
  source_frame_id:'camera/master0',
  target_frame_id:'solve0',
  update_frame_id:'',
  reverse_frame_id:''
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
  if(Config.reverse_frame_id.length>0){
    let rf=Config.reverse_frame_id;
    let ctf=await rosNode.getParam('/config_tf');
    if(ctf.hasOwnProperty(rf)){
      rosNode.subscribe('/joint_states',sensor_msgs.JointState,async function(joint){
        let tf=new geometry_msgs.TransformStamped();
        tf.header.stamp=ros.Time.now();
        tf.header.frame_id=ctf[rf].parent_frame_id;
        tf.child_frame_id=rf;
        let rot2=joint.position[5]*0.5;
        tf.transform.rotation.x=0;
        tf.transform.rotation.y=0;
        tf.transform.rotation.z=Math.sin(rot2);
        tf.transform.rotation.w=Math.cos(rot2);
        pub_tf.publish(tf);
      });
    }
  }
  const pub_tf=rosNode.advertise('/update/config_tf',geometry_msgs.TransformStamped);
  const pub_clear=rosNode.advertise('/request/clear',std_msgs.Bool);
  const pub_capture=rosNode.advertise('/request/capture',std_msgs.Bool);
  const pub_solve=rosNode.advertise('/request/solve',std_msgs.Bool);
  const pub_recipe=rosNode.advertise('/request/recipe_load',std_msgs.String);
  const pub_path=rosNode.advertise('/request/path',geometry_msgs.PoseArray);

//Socket events//////////////////
  const server = net.createServer(function(conn){
    conn.setTimeout(Config.socket_timeout*1000);
    let msg='';
    conn.on('data', function(data){
      msg+=data.toString();
      if(msg.indexOf('(')*msg.indexOf(')')<0) return;
      if(msg.startsWith('P1') && Config.update_frame_id.length>0){
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
        let tfs=protocol.decode(msg.substr(2).trim());
        if(tfs.length>0 && Config.update_frame_id.length>0){
          let tf=new geometry_msgs.TransformStamped();
          tf.header.stamp=ros.Time.now();
          tf.header.frame_id='world';
          tf.child_frame_id=Config.update_frame_id;
          tf.transform=tfs[0];
          pub_tf.publish(tf);
        }
        setTimeout(function(){
          let f=new std_msgs.Bool();
          f.data=true;
          pub_capture.publish(f);
        },Config.delay);
        let wdt=setTimeout(function(){
            emitter.removeAllListeners('capture');
            conn.write('NG\n');
            conn.write('911');//timeout to capture
        },Config.capt_timeout*1000);
        emitter.removeAllListeners('capture');
        emitter.once('capture',function(ret){
          clearTimeout(wdt);
          if (ret) {
            conn.write('OK\n');
          } else {
            conn.write('NG\n');
            conn.write('912');//failed to capture
          }
        });
      }
      else if(msg.startsWith('X2')){   // [X2] ROVI_SOLVE
        let tfs=protocol.decode(msg.substr(2).trim());
        const path=new geometry_msgs.PoseArray();
        path.header.frame_id='world';
        path.header.stamp=ros.Time.now();
        tfs.forEach(function(t){
          let p=new geometry_msgs.Pose();
          p.position.x=t.translation.x;
          p.position.y=t.translation.y;
          p.position.z=t.translation.z;
          p.orientation.x=t.rotation.x;
          p.orientation.y=t.rotation.y;
          p.orientation.z=t.rotation.z;
          p.orientation.w=t.rotation.w;
          path.poses.push(p);
        });
        pub_path.publish(path);
        let f=new std_msgs.Bool();
        f.data=true;
        pub_solve.publish(f);
        let wdt=setTimeout(function(){
            emitter.removeAllListeners('solve');
            conn.write('NG\n');
            conn.write('921');//timeout to solve
        },Config.solve_timeout*1000);
        emitter.removeAllListeners('solve');
        emitter.once('solve',function(ret){
          clearTimeout(wdt);
          if(!ret){
              conn.write('NG\n');
              conn.write('922');//failed to solve
          }
          tf_lookup.stdin.write(Config.source_frame_id+' '+Config.target_frame_id+'\n');
          emitter.once('tf_transform',function(tf){
            if(tf.hasOwnProperty('translation')){
              let cod=protocol.encode([tf]);
              ros.log.info('r_socket encode:'+cod);
              conn.write('OK\n');
              conn.write(cod+"\n");
            }
            else{
              conn.write('NG\n');
              conn.write('923');//failed to lookup
            }
          });
        });
      }
      else if(msg.startsWith('X3')){   // [X3] ROVI_RECIPE
        f=new std_msgs.String();
        f.data=msg.trim().replace(/\).*/g, '').replace(/.*\(/, '');  //remove "*(" ")*"
        pub_recipe.publish(f);
        console.log("("+f.data+")");
        let wdt=setTimeout(function(){
            emitter.removeAllListeners('recipe');
            conn.write('NG\n');
            conn.write('931');//timeout to recipe
        },Config.recipe_timeout*1000);
        emitter.removeAllListeners('recipe');
        emitter.once('recipe',function(ret){
          clearTimeout(wdt);
          if (ret) {
            conn.write('OK\n');
          } else {
            conn.write('NG\n');
            conn.write('932');
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
