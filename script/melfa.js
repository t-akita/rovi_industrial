#!/usr/bin/env node

const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const parser=require('./rparser');
const tflib=require('./tflib');
let protocol={}

protocol.decode=function(msg){
  try{
    let euler=toCoords(msg);
    return euler.map(function(e){
      return tflib.xyz2quat(e,'cba');
    });
  }
  catch(e){
    ros.log.error('melfa::'+e);
    return [];
  }  
}

protocol.encode=function(tf){
  let RT=tflib.toRT(tf[0]);
  let euler=tflib.fromRTtoEulerCBA(RT);
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[3].toFixed(3),euler[4].toFixed(3),euler[5].toFixed(3)]
  return "("+tarr.concat(rarr).join(",")+")(7,0)";
}

module.exports=protocol;

