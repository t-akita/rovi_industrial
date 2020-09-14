#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='rzyx deg';

const protocol=require('./protocol');
protocol.tflib=tflib;

function zeropad(N){
  return (N.toFixed(3)+'00000').slice(10);
}
protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  return euler.map(function(N){
    return (N.toFixed(3)+'0000000').slice(0,10);
  }).join(",");
}
protocol.decode=async function(msg){
  let ary=protocol.decode_(msg);
  if(ary.length==0) return [];
  return await this.tflib.fromEuler(ary);
}
protocol.jdecode=function(msg){
  let jnts=this.decode_(msg);
  return jnts;
}

protocol.delim=",";
protocol.lf="\n";
protocol.autoclose=true;
protocol.joints=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'];

module.exports=protocol;
