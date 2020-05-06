#!/usr/bin/env node

const net=require('net');
const ping=require('ping');
const EventEmitter=require('events').EventEmitter;
const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const utils_srvs=ros.require('rovi_utils').srv;
let tf_lookup=null;

let Config={
  protocol:'fanuc',
  port:8888,
  socket_timeout:0,
  delay:100,
  capt_timeout:10,
  solve_timeout:10,
  recipe_timeout:3,
  base_frame_id:'base',
  source_frame_id:'camera/master0',
  target_frame_id:'solve0',
  update_frame_id:'',
  reverse_frame_id:'',
  reverse_direction:1
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
  protocol.node(rosNode);
  if(protocol.autoclose==undefined) protocol.autoclose=false;
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
  const pub_conn=rosNode.advertise('/rsocket/stat',std_msgs.Bool);
  const pub_tf=rosNode.advertise('/update/config_tf',geometry_msgs.TransformStamped);
  const pub_clear=rosNode.advertise('/request/clear',std_msgs.Bool);
  const pub_capture=rosNode.advertise('/request/capture',std_msgs.Bool);
  const pub_solve=rosNode.advertise('/request/solve',std_msgs.Bool);
  const pub_recipe=rosNode.advertise('/request/recipe_load',std_msgs.String);
  const pub_path=rosNode.advertise('/request/path',geometry_msgs.PoseArray);
  rosNode.subscribe('/rsocket/ping',std_msgs.Bool,async function(ret){
    let res = await ping.promise.probe(Config.robot_ip,{timeout:3});
    if(res.alive){
      let stat=new std_msgs.Bool();
      stat.data=true;
      pub_conn.publish(stat);
    }
  });
  tf_lookup=rosNode.serviceClient('/tf_lookup/query', utils_srvs.TextFilter, { persist: true });
  if (!await rosNode.waitForService(tf_lookup.getService(), 2000)) {
    ros.log.error('tf_lookup service not available');
    return;
  }
//Function///////////////
  let reverse_frame_updater=null;
//Rotate J7 by J6////////////////
  if(Config.reverse_frame_id.length>0){
    let rf=Config.reverse_frame_id;
    let rd=Config.reverse_direction;
    let ctf=await rosNode.getParam('/config_tf');
    if(ctf.hasOwnProperty(rf)){
      reverse_frame_updater=function(j6){
        let tf=new geometry_msgs.TransformStamped();
        tf.header.stamp=ros.Time.now();
        tf.header.frame_id=ctf[rf].parent_frame_id;
        tf.child_frame_id=rf;
        let rot2=j6*0.5*rd;
        tf.transform.rotation.x=0;
        tf.transform.rotation.y=0;
        tf.transform.rotation.z=Math.sin(rot2);
        tf.transform.rotation.w=Math.cos(rot2);
        pub_tf.publish(tf);
      }
      rosNode.subscribe('/joint_states',sensor_msgs.JointState,async function(joint){
        reverse_frame_updater(joint.position[joint.position.length-1]);
      });
    }
  }

  let stat_out_timer=null;
  function stat_out(f){
    let stat=new std_msgs.Bool();
    stat.data=f;
    pub_conn.publish(stat);
    if(f){
      if(stat_out_timer==null) stat_out_timer=setInterval(function(){ stat_out(f);},500);
    }
    else{
      if(stat_out_timer!=null){
        clearInterval(stat_out_timer);
        stat_out_timer=null;
      }
    }
  }

//Socket events//////////////////
  function respNG(conn,err,delim,lf){
    let l1='NG'+delim;
    let l2=''+err+lf;
    if(delim==lf){ conn.write(l1); conn.write(l2);}
    else conn.write(l1+l2);
  }
  let TX1;
  const server = net.createServer(function(conn){
    conn.setTimeout(Config.socket_timeout*1000);
    let msg='';
    let wdt=null;
    let stat;
    conn.on('data',async function(data){
      stat_out(true);
      msg+=data.toString();
      if(msg.indexOf('(')*msg.indexOf(')')<0) return;//until msg will like "??(???)"
      ros.log.info("rsocket "+msg);
      if(msg.startsWith('P1') && Config.update_frame_id.length>0){
        let tf=new geometry_msgs.TransformStamped();
        tf.header.stamp=ros.Time.now();
        tf.header.frame_id=Config.base_frame_id;
        tf.child_frame_id=Config.update_frame_id;
        let tfs=await protocol.decode(msg.substr(2));
        if(tfs.length>0){
          tf.transform=tfs[0];
          pub_tf.publish(tf);
        }
        return;
      }
      if(wdt!=null){
        ros.log.warn("rsocket::Xcmd busy");
        respNG(conn,900,protocol.delim,protocol.lf); //busy
        return;
      }
      if(msg.startsWith('X0')){//--------------------[X0] ROVI_CLEAR
        let f=new std_msgs.Bool();
        f.data=true;
        pub_clear.publish(f);
        conn.write('OK'+protocol.lf);
        if(protocol.autoclose) conn.destroy();
      }
      else if(msg.startsWith('X1')){//--------------------[X1] ROVI_CAPTURE
        let tfs;
        try{
          tfs=await protocol.decode(msg.substr(2).trim());
        }
        catch(e){
          ros.log.error('r_socket::pose decode error '+e);
          respNG(conn,999,protocol.delim,protocol.lf); //capture timeout
          if(protocol.autoclose) conn.destroy();
          return;
        }
        let t0=ros.Time.now();
        TX1=t0;
        if(tfs.length>0)
          ros.log.warn("rsocket::tf parse "+JSON.stringify(tfs));
        else ros.log.warn("rsocket::tf parse nothing");
        if(tfs.length>0 && tfs[0].hasOwnProperty('translation') && Config.update_frame_id.length>0){
          let tf=new geometry_msgs.TransformStamped();
          tf.header.stamp=ros.Time.now();
          tf.header.frame_id=Config.base_frame_id;
          tf.child_frame_id=Config.update_frame_id;
          tf.transform=tfs[0];
          pub_tf.publish(tf);
        }
        setTimeout(function(){
          let f=new std_msgs.Bool();
          f.data=true;
          pub_capture.publish(f);
        },Config.delay);
        wdt=setTimeout(function(){
          wdt=null;
          emitter.removeAllListeners('capture');
          ros.log.warn("rsocket::capture timeout");
          respNG(conn,911,protocol.delim,protocol.lf); //capture timeout
          if(protocol.autoclose) conn.destroy();
        },Config.capt_timeout*1000);
        emitter.removeAllListeners('capture');
        emitter.once('capture',function(ret){
          clearTimeout(wdt);
          wdt=null;
          let t1=ros.Time.now();
          ros.log.info("rsocket::capture done "+ret+" "+(ros.Time.toSeconds(t1)-ros.Time.toSeconds(t0)));
          if(ret) conn.write('OK'+protocol.lf);
          else respNG(conn,912,protocol.delim,protocol.lf); //failed to capture
          if(protocol.autoclose) conn.destroy();
        });
      }
      else if(msg.startsWith('X2')){//--------------------[X2] ROVI_SOLVE
        let t0=ros.Time.now();
        let f=new std_msgs.Bool();
        f.data=true;
        pub_solve.publish(f);
        wdt=setTimeout(function(){
          wdt=null;
          emitter.removeAllListeners('solve');
          ros.log.warn("rsocket::solve timeout");
          respNG(conn,921,protocol.delim,protocol.lf); //solve timeout
          if(protocol.autoclose) conn.destroy();
        },Config.solve_timeout*1000);
        emitter.removeAllListeners('solve');
        emitter.once('solve',async function(ret){
          clearTimeout(wdt);
          wdt=null;
          try{
            let t1=ros.Time.now();
            let tx2=ros.Time.toSeconds(t1)-ros.Time.toSeconds(t0);
            let tx12=ros.Time.toSeconds(t1)-ros.Time.toSeconds(TX1);
            ros.log.info("rsocket::solve done "+ret+" "+tx2+" "+tx12);
          }
          catch(err){ }
          if(!ret){
            respNG(conn,922,protocol.delim,protocol.lf); //failed to solve
            if(protocol.autoclose) conn.destroy();
            return;
          }
          let req=new utils_srvs.TextFilter.Request();
          req.data=Config.base_frame_id+' '+Config.source_frame_id+' '+Config.target_frame_id;
          let res;
          try{
            res=await tf_lookup.call(req);
          }
          catch(err){
            ros.log.error('tf_lookup call error');
            respNG(conn,923,protocol.delim,protocol.lf); //failed to lookup
            if(protocol.autoclose) conn.destroy();
          }
          if(res.data.length==0){
            ros.log.error('tf_lookup returned null');
            respNG(conn,923,protocol.delim,protocol.lf); //failed to lookup
            if(protocol.autoclose) conn.destroy();
          }
          let tf=JSON.parse(res.data);
          if(!tf.hasOwnProperty('translation')){
            ros.log.error('tf_lookup returned but Transform');
            respNG(conn,923,protocol.delim,protocol.lf); //failed to lookup
            if(protocol.autoclose) conn.destroy();
          }
          ros.log.info("rsocket tf:"+res.data);
          let cod;
          try{
            cod=await protocol.encode([tf]);
          }
          catch(e){
            ros.log.error('r_socket::pose encode error '+e);
            respNG(conn,999,protocol.delim,protocol.lf);
            if(protocol.autoclose) conn.destroy();
            return;
          }
          ros.log.info("rsocket encode:"+cod);
          let l1='OK'+protocol.delim;
          let l2=cod+protocol.lf;
          if(protocol.delim==protocol.lf){ conn.write(l1); conn.write(l2);}
          else conn.write(l1+l2);
          if(protocol.autoclose) conn.destroy();
        });
      }
      else if(msg.startsWith('X3')){//--------------------[X3] ROVI_RECIPE
        f=new std_msgs.String();
        f.data=msg.trim().replace(/\).*/g, '').replace(/.*\(/, '');  //remove "*(" ")*"
        pub_recipe.publish(f);
        console.log("("+f.data+")");
        wdt=setTimeout(function(){
          wdt=null;
          emitter.removeAllListeners('recipe');
          ros.log.warn("rsocket::recipe timeout");
          respNG(conn,931,protocol.delim,protocol.lf);
          if(protocol.autoclose) conn.destroy();
        },Config.recipe_timeout*1000);
        emitter.removeAllListeners('recipe');
        emitter.once('recipe',function(ret){
          clearTimeout(wdt);
          wdt=null;
          ros.log.info("rsocket::recipe done "+ret);
          if(ret) conn.write('OK'+protocol.lf);
          else respNG(conn,932,protocol.delim,protocol.lf);
          if(protocol.autoclose) conn.destroy();
        });
      }
      else if(msg.startsWith('J6')){
        let j6=await protocol.decode(msg.substr(2).trim());
        console.log("J6 "+j6[0][0])
        if(reverse_frame_updater!=null){
          reverse_frame_updater(j6[0][0]);
        }
      }
      msg='';
    });
    conn.on('close', function(){
      if(wdt!=null) clearTimeout(wdt);
      wdt=null;
      stat_out(false);
      ros.log.warn('r_socket CLOSED.');
    });
    conn.on('timeout',function(){
      emitter.removeAllListeners();
      ros.log.warn('rsocket TIMEOUT');
      conn.write("NG"+protocol.delim); //Request timeout
      conn.write("408\n");
      conn.destroy();
    });
    conn.on('error', function(err){
      ros.log.warn('Net:socket error '+err);
    });
  }).listen(Config.port);
});
