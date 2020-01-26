#!/usr/bin/env node

module.exports={
//let protocol={
  encode:async function(tf){
    let euler=await this.tflib.toEuler(tf[0]);
    let tarr=[euler[0].toFixed(6),euler[1].toFixed(6),euler[2].toFixed(6)]
    let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
    return "["+tarr.concat(rarr).join(",")+"]";
  },
  decode:async function(msg){
    const who=this;
    let ary=msg.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/;/, '],[').replace(/E\+/g, 'E').replace(/\+/g, '');
   if(ary.length<=2) return [];
    ary=JSON.parse('['+ary+']');
    return await Promise.all(
      ary.map(async function(a){
        return await who.tflib.fromEuler(a);
      })
    );
  },
  joints:['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  jdecode_:function(msg){
    let ary=msg.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/E\+/g, 'E').replace(/\+/g, '');
    let jnt=JSON.parse(ary);
    if(jnt.length<this.joints.length) throw new Error("Joint number does not match");
    return jnt.map(function(e){
      return e*Math.PI/180;
    }).slice(0,this.joints.length);
  },
  jdecode:function(msg){
    return this.jdecoce_(msg);
  },
  reboot:function(){
    console.log("protocol::reboot");
    this.tflib.set();
  },
  tflib:null,
  delim:"\n",
  lf:"\n"
}

/*
const tflib=require('./tflib');
tflib.set('rzyx deg');
protocol.tflib=tflib;

setImmediate(async function(){
  let tf=await protocol.decode("(1,2,3,30,20,AA)");
  console.log("result "+JSON.stringify(tf));
  let d=await protocol.encode(tf);
  console.log("result "+JSON.stringify(d));
});
*/

