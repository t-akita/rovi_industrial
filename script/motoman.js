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
  ary=JSON.parse('['+ary+']');
  let tfs=[];
  for(let i=0;i<ary.length;i++){
    let a=ary[i];
    if(a.length>=6){
      a[0]*=0.000001;
      a[1]*=0.000001;
      a[2]*=0.000001;
      a[3]*=0.0001;
      a[4]*=0.0001;
      a[5]*=0.0001;
      let tf=await who.tflib.fromEuler(a);
      tfs.push(tf);
    }
    else{
      tfs.push(a);
    }
  }
  return tfs;
}
protocol.delim=",";
protocol.lf="\n";

module.exports=protocol;
