#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='sxyz deg';

const protocol=require('./protocol');
protocol.tflib=tflib;

protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[3].toFixed(3),euler[4].toFixed(3),euler[5].toFixed(3)]
  return tarr.concat(rarr).join(",");
}
protocol.decode=async function(msg){
  let ary=protocol.decode_(msg);
  return await this.tflib.fromEuler(ary);
}
protocol.jdecode=function(msg){
  let jnts=this.decode_(msg);
  return jnts.map((jnt)=>{
    if(jnt.length<this.joints.length) throw new Error("Joint number not enough");
    return jnt.map(function(j){
      return j*Math.PI/180;
    });
  }).slice(0,this.joints.length);
}

protocol.delim=",";
protocol.lf="\n";
protocol.joints=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'];

module.exports=protocol;
