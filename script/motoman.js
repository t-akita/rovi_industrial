#!/usr/bin/env node

const tflib=require('./tflib');
tflib.set('sxyz deg');

const protocol=require('./protocol');
protocol.tflib=tflib;

protocol.encode=async function(tf){
  let euler=await this.tflib.toEuler(tf[0]);
  let tarr=[Math.floor(euler[0]*1000000),Math.floor(euler[1]*1000000),Math.floor(euler[2]*1000000)]
  let rarr=[Math.floor(euler[3]*10000),Math.floor(euler[4]*10000),Math.floor(euler[5]*10000)]
  return tarr.concat(rarr).join(",");
}
protocol.decode=async function(msg){
  const who=this;
  let ary=msg.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/;/, '],[').replace(/E\+/g, 'E').replace(/\+/g, '');
  if(ary.length<=2) return [];
  ary=JSON.parse('['+ary+']');
  return await Promise.all(
    ary.map(async function(a){
      a[0]*=0.000001;
      a[1]*=0.000001;
      a[2]*=0.000001;
      a[3]*=0.0001;
      a[4]*=0.0001;
      a[5]*=0.0001;
      return await who.tflib.fromEuler(a);
    })
  );
}
protocol.jdecode=function(msg){
  let jnt=this.jdecode_(msg);
  return jnt.map(function(e){
    return e*0.0001;
  });
}

protocol.delim=",";
protocol.lf="\n";
protocol.joints=['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

module.exports=protocol;
