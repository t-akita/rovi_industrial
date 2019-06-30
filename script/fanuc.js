#!/usr/bin/env node

const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const parser=require('./rparser');
const tflib=require('./tflib');
let protocol={}

protocol.decode=function(msg){
  try{
    let euler=parser.parse(msg);
    return euler.map(function(e){
      return tflib.xyz2quat(e,'abc');
    });
  }
  catch(e){
    ros.log.error('fanuc::'+e);
    return [];
  }
}

protocol.encode=function(tf){
  let RT=tflib.toRT(tf[0]);
  let euler=tflib.fromRTtoEulerABC(RT);
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[3].toFixed(3),euler[4].toFixed(3),euler[5].toFixed(3)]
  return "'"+tarr.concat(rarr).join("''")+"'";
}

module.exports=protocol;

/*
//  const msg='X2(1,2,3,4,5,6;7,6,5,4,3,2)(0,1)';
  const msg='X2(1,2,3,4,5,6;7,6)';
  let coods=protocol.decode(msg.substr(2).trim());
  console.log(coods);
*/