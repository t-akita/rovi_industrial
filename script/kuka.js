#!/usr/bin/env node

const tflib=require('./tflib');
tflib.set('rzyx rad');

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=async function(tf){
  let euler=await this.tflib.toEuler(tf[0]);
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
  return "("+tarr.concat(rarr).join(",")+")";
}

protocol.decode=async function(msg){
  let ary=msg.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/;/, '],[').replace(/E\+/g, 'E').replace(/\+/g, '');
  ary=JSON.parse('['+ary+']');
  let tfs=[];
  for(let i=0;i<ary.length;i++){
    let a=ary[i];
    if(a.length>=6){
      let tf=await who.tflib.fromEuler(a);
      tfs.push(tf);
    }
    else{
      tfs.push(a);
    }
  }
  return tfs;
}

protocol.delim="\n";
protocol.lf="\r\n";

module.exports=protocol;
