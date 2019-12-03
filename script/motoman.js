#!/usr/bin/env node

const tflib=require('./tflib');
tflib.set('rzyx deg');

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=async function(tf){
  let euler=await this.tflib.toEuler(tf[0]);
  let tarr=[Math.floor(euler[0]*1000000),Math.floor(euler[1]*1000000),Math.floor(euler[2]*1000000)]
  let rarr=[Math.floor(euler[3]*10000),Math.floor(euler[4]*10000),Math.floor(euler[5]*10000)]
  return tarr.concat(rarr).join(",");
}

protocol.delim=",";
protocol.lf="\n";

module.exports=protocol;
