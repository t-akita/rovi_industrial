const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;

module.exports = {
toRT: function(tf) {
  let x=tf.rotation.x;
  let y=tf.rotation.y;
  let z=tf.rotation.z;
  let w=tf.rotation.w;
  let xx=x*x;
  let yy=y*y;
  let zz=z*z;
  let ww=w*w;
  let RT=[[xx-yy-zz+ww,2.*(x*y-w*z),2.*(x*z+w*y),tf.translation.x],
          [2.*(x*y+w*z),yy+ww-xx-zz,2.*(y*z-w*x),tf.translation.y],
          [2.*(x*z-w*y),2.*(y*z+w*x),zz+ww-xx-yy,tf.translation.z],
          [0,0,0,1]];
  return RT;
},

fromRT: function(rt) {
  let s=qw=qx=qy=qz=0.0;
  if (rt[0][0]+rt[1][1]+rt[2][2]>0){
    s=Math.sqrt(1.0+rt[0][0]+rt[1][1]+rt[2][2])*2;  //s=qw*4
    qw=s/4;
    qx=(rt[2][1]-rt[1][2])/s;
    qy=(rt[0][2]-rt[2][0])/s;
    qz=(rt[1][0]-rt[0][1])/s;
  } else if ((rt[0][0]>rt[1][1]) && (rt[0][0]>rt[2][2])) {
    s=Math.sqrt(1.0+rt[0][0]-rt[1][1]-rt[2][2])*2;  //s=qx*4
    qw=(rt[2][1]-rt[1][2])/s;
    qx=s/4;
    qy=(rt[0][1]+rt[1][0])/s;
    qz=(rt[0][2]+rt[2][0])/s;
  } else if (rt[1][1]>rt[2][2]) {
    s=Math.sqrt(1.0-rt[0][0]+rt[1][1]-rt[2][2])*2;  //s=qy*4
    qw=(rt[0][2]-rt[2][0])/s;
    qx=(rt[0][1]+rt[1][0])/s;
    qy=s/4;
    qz=(rt[1][2]+rt[2][1])/s;
  } else {
    s=Math.sqrt(1.0-rt[0][0]-rt[1][1]+rt[2][2])*2;  //s=qz*4
    qw=(rt[1][0]-rt[0][1])/s;
    qx=(rt[0][2]+rt[2][0])/s;
    qy=(rt[1][2]+rt[2][1])/s;
    qz=s/4;
  }
  let tf=new geometry_msgs.Transform();
  tf.rotation.w=qw;
  tf.rotation.x=qx;
  tf.rotation.y=qy;
  tf.rotation.z=qz;
  tf.translation.x=rt[0][3];
  tf.translation.y=rt[1][3];
  tf.translation.z=rt[2][3];
  return tf;
},

// RTからtfへの変換(CBA)
fromRTtoEulerCBA: function(matrix44){ // RobotRTToRxyzCBA
  let A=0.0;
  let B=0.0;
  let C=0.0;
  let Rx=0.0;
  let Ry=0.0;
  let Rz=0.0;
  let half_pi=Math.PI/2.0;
  let req_limit=1e-8;

  if (Math.abs(matrix44[2][0]+1)<=req_limit || Math.abs(matrix44[2][0]-1)<=req_limit){
    A=0;
    if (Math.abs(matrix44[2][0]-1)<=req_limit) B=-half_pi;
    if (Math.abs(matrix44[2][0]+1)<=req_limit) B=half_pi;
    C=Math.atan2(-matrix44[0][1],matrix44[1][1]);
  }else{
    B=Math.asin(-1*matrix44[2][0]);
    C=Math.atan2(matrix44[1][0],matrix44[0][0]);
    A=Math.atan2(matrix44[2][1],matrix44[2][2]);
  }
  Rx=matrix44[0][3];
  Ry=matrix44[1][3];
  Rz=matrix44[2][3];
  A_degree=A/Math.PI*180.0;
  B_degree=B/Math.PI*180.0;
  C_degree=C/Math.PI*180.0;

  //console.log('Rx:'+Rx+' Ry:'+Ry+' Rz:'+Rz+' A:'+A_degree+' B:'+B_degree+' C:'+C_degree);
  let vec=new Array();
  vec.push(Rx);
  vec.push(Ry);
  vec.push(Rz);
  vec.push(A_degree);
  vec.push(B_degree);
  vec.push(C_degree);
  vec.push(0);

  return vec
},

// RTからtfへの変換(ABC)
fromRTtoEulerABC: function(matrix44){  // RobotRTToRxyzABC
  let A=0.0;
  let B=0.0;
  let C=0.0;
  let Rx=0.0;
  let Ry=0.0;
  let Rz=0.0;
  let half_pi=Math.PI/2.0;
  let req_limit=1e-8;

  if (Math.abs(matrix44[0][2]+1)<=req_limit || Math.abs(matrix44[0][2]-1)<=req_limit){
    A=0
    if (Math.abs(matrix44[0][2]-1)<=req_limit) B=half_pi;
    if (Math.abs(matrix44[0,2]+1)<=req_limit) B=-half_pi;
    C=Math.atan2(matrix44[1,0],matrix44[1,1]);
  }else{
    B=Math.asin(matrix44[0][2]);
    C=Math.atan2(-matrix44[0][1], matrix44[0][0]);
    A=Math.atan2(-matrix44[1][2], matrix44[2][2]);
  }
  Rx=matrix44[0][3];
  Ry=matrix44[1][3];
  Rz=matrix44[2][3];
  A_degree=A/Math.PI*180.0;
  B_degree=B/Math.PI*180.0;
  C_degree=C/Math.PI*180.0;

  //console.log('Rx:'+Rx+' Ry:'+Ry+' Rz:'+Rz+' A:'+A_degree+' B:'+B_degree+' C:'+C_degree);
  let vec=new Array();
  vec.push(Rx);
  vec.push(Ry);
  vec.push(Rz);
  vec.push(A_degree);
  vec.push(B_degree);
  vec.push(C_degree);
  vec.push(0);

  return vec
},

xyz2quat: function(e,rOrder) {
  let tf=Object.assign({},e);
  let k = Math.PI / 180 * 0.5;
  let cx = Math.cos(e.rotation.x * k);
  let cy = Math.cos(e.rotation.y * k);
  let cz = Math.cos(e.rotation.z * k);
  let sx = Math.sin(e.rotation.x * k);
  let sy = Math.sin(e.rotation.y * k);
  let sz = Math.sin(e.rotation.z * k);
  if (rOrder=="cba") {
    tf.rotation.x = cy * cz * sx - cx * sypub_tf * sz;
    tf.rotation.y = cy * sx * sz + cx * cz * sy;
    tf.rotation.z = cx * cy * sz - cz * sx * sy;
    tf.rotation.w = sx * sy * sz + cx * cy * cz;
  } else {
    tf.rotation.x = sx * cy * cz - cx * sy * sz;
    tf.rotation.y = cx * sy * cz + sx * cy * sz;
    tf.rotation.z = cx * cy * sz - sx * sy * cz;
    tf.rotation.w = cx * cy * cz + sx * sy * sz;
  }
  return tf;
}
};

