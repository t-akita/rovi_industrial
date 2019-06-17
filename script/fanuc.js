const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const tflib=require('./tflib');
let protocol={}

function toCoords(str) {
  // data format is assumed as '***(X,Y,Z,A,B,C)\n'.
  const ary = str.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/E\+/g, 'E').replace(/\+/g, '');
  let coords = [];
  try{
    coords = JSON.parse('[' + ary + ']');
  }
  catch(e){
    ros.log.error('fanuc:'+e);
  }
  let tf=new geometry_msgs.Transform();
  tf.translation.x=coords[0][0];
  tf.translation.y=coords[0][1];
  tf.translation.z=coords[0][2];
  tf.rotation.x=coords[0][3];
  tf.rotation.y=coords[0][4];
  tf.rotation.z=coords[0][5];
  tf.rotation.w=1;
  return tf;
}

protocol.decode=function(msg){
  let euler=toCoords(msg);
  return tflib.xyz2quat(euler,'abc');
}

protocol.encode=function(tf){
  let RT=tflib.toRT(tf);
  let euler=tflib.fromRTtoEulerABC(RT);
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[3].toFixed(3),euler[4].toFixed(3),euler[5].toFixed(3)]
  return "'"+tarr.concat(rarr).join("''")+"'";
}

module.exports=protocol;

