#!/usr/bin/env node

const tflib=require('./tflib');
tflib.set('rzyx deg');

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=async function(tf){
  let euler=await this.tflib.toEuler(tf[0]);
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[3].toFixed(3),euler[4].toFixed(3),euler[5].toFixed(3)]
  return "("+tarr.concat(rarr).join(",")+")(7,0)";
}
protocol.delim="\n";
protocol.lf="\n";

protocol.delim="\n";
protocol.lf="\n";

module.exports=protocol;

