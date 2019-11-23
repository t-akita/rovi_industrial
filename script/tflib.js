const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const popen = require('child_process');
const EventEmitter=require('events').EventEmitter;

module.exports={
  tf_euler: null,
  emitter: null,
  option:'rxyz rad',
  set: function(arg){
    if(this.tf_euler==null){
      const who=this;
      this.option=arg;
      this.emitter=new EventEmitter();
      Object.assign(process.env,{stdio:['pipe','pipe',2]})
      this.tf_euler=popen.exec('tf_euler.py '+arg,{env:process.env});
      this.tf_euler.stdout.on('data',function(data){
        let vec=JSON.parse("["+data+"]");
        who.emitter.emit('vector',vec);
      });
    }
    return this;
  },
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
  toEuler: async function(tf){
    const who=this;
    return new Promise(function(resolve){
      who.emitter.once('vector',function(v){
        let vec=[tf.translation.x,tf.translation.y,tf.translation.z,v[0],v[1],v[2]];
        resolve(vec);
      });
      who.tf_euler.stdin.write(tf.rotation.x+','+tf.rotation.y+','+tf.rotation.z+','+tf.rotation.w+'\n');
    });
  },
  fromEuler: async function(pos){
    const who=this;
    return new Promise(function(resolve){
      who.emitter.once('vector',function(v){
        let tf=new geometry_msgs.Transform();
        tf.translation.x=pos[0];
        tf.translation.y=pos[1];
        tf.translation.z=pos[2];
        tf.rotation.x=v[0];
        tf.rotation.y=v[1];
        tf.rotation.z=v[2];
        tf.rotation.w=v[3];
        resolve(tf);
      });
      who.tf_euler.stdin.write(pos[3]+','+pos[4]+','+pos[5]+'\n');
    });
  }
};

