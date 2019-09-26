#!/usr/bin/env node

const tflib=require('./tflib');
tflib.set('rzyz deg');

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=function(tf){
  let euler=await this.tflib.toEuler(tf[0]);
  let tarr=[euler[0].toFixed(6),euler[1].toFixed(6),euler[2].toFixed(6)]
  let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
  return tarr.concat(rarr).join(",");
}

protocol.delim=",";
protocol.lf="";

module.exports=protocol;
