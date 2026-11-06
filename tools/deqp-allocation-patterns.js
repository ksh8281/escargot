/*-------------------------------------------------------------------------
 * drawElements Quality Program OpenGL ES Utilities
 * ------------------------------------------------
 *
 * Copyright 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Acorn 5.5.3 excerpts (Node and SourceLocation), MIT license:
Copyright (C) 2012-2018 by various contributors (see AUTHORS)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
/*
 * Standalone allocation-pattern research benchmark, 2026-11-06.
 * dEQP excerpts: Copyright 2014 The Android Open Source Project, Apache-2.0.
 * https://www.apache.org/licenses/LICENSE-2.0
 * Acorn excerpts: Copyright 2012-2018 various contributors, MIT license.
 * https://github.com/acornjs/acorn/blob/5.5.3/LICENSE
 * Original functions are copied without modifying their bodies.
 * Harness and RayTrace-like workload are new research code (MIT).
 * This is neither the dEQP conformance suite nor an Octane/WTB score.
 */
(function(global){
"use strict";
var deMath={},tcuFloat={description32:{totalByteSize:4}},tcuTexture={RGBA8View:function(){}};
// framework/delibs/debase/deMath.js:36
deMath.deInRange32 = function(a, mn, mx) {
    return (a >= mn) && (a <= mx);
};
// framework/delibs/debase/deMath.js:108
deMath.clamp = function(val, minParm, maxParm) {
    return Math.min(Math.max(val, minParm), maxParm);
};
// framework/delibs/debase/deMath.js:241
deMath.scale = function(a, b) {
    var dst = [];
    for (var i = 0; i < a.length; i++)
        dst.push(a[i] * b);
    return dst;
};
// framework/delibs/debase/deMath.js:271
deMath.add = function(a, b) {
    if (a.length != b.length)
        throw new Error('Arrays must have the same size');
    var dst = [];
    for (var i = 0; i < a.length; i++)
        dst.push(a[i] + b[i]);
    return dst;
};
// framework/delibs/debase/deMath.js:958
deMath.toIVec = function(a) {
    /** @type {Array<number>} */ var res = [];
    for (var i = 0; i < a.length; i++)
        res.push(deMath.clamp(Math.floor(a[i] * 255), 0, 255));
    return res;
};
// framework/delibs/debase/deMath.js:579
deMath.split32 = function(x) {
    var ret = [];
    ret[1] = Math.floor(x / 0x100000000);
    ret[0] = x - ret[1] * 0x100000000;
    return ret;
};
// framework/delibs/debase/deMath.js:604
deMath.join32 = function(x) {
    var v0 = x[0] >= 0 ? x[0] : 0x100000000 + x[0];
    var v1 = x[1];
    var val = v1 * 0x100000000 + v0;
    return val;
};
// framework/delibs/debase/deMath.js:763
deMath.shiftLeft = function(value, steps) {
    //quick path
    if (steps < 31) {
        var v = value * (1 << steps);
        if (deMath.deInRange32(v, -0x80000000, 0x7FFFFFFF))
            return v;
    }

    if (steps == 0)
        return value;
    else if (steps < 32) {
        var mask = (1 << 32 - steps) - 1;
        var x = deMath.split32(value);
        var highBits = x[0] & (~mask);
        var y = highBits >> (32 - steps);
        if (highBits < 0) {
            var m = (1 << steps) - 1;
            y &= m;
        }
        var result = [];
        result[0] = x[0] << steps;
        result[1] = x[1] << steps;
        result[1] |= y;

        return deMath.join32(result);
    } else {
        var x = deMath.split32(value);
        var result = [];
        result[0] = 0;
        result[1] = x[0] << steps - 32;
        return deMath.join32(result);
    }
};
// framework/delibs/debase/deMath.js:815
deMath.shiftRight = function(value, steps) {
    //quick path
    if (deMath.deInRange32(value, -0x80000000, 0x7FFFFFFF) && steps < 32)
        return value >> steps;

    if (steps == 0)
        return value;
    else if (steps < 32) {
        if (steps == 0)
            return value;
        var mask = (1 << steps) - 1;
        var x = deMath.split32(value);
        var lowBits = x[1] & mask;
        var result = [];
        var m = (1 << 32 - steps) - 1;
        result[0] = (x[0] >> steps) & m;
        result[1] = x[1] >> steps;
        result[0] |= lowBits << 32 - steps;
        return deMath.join32(result);
    } else {
        var x = deMath.split32(value);
        var result = [];
        result[0] = x[1] >> steps - 32;
        result[1] = value < 0 ? -1 : 0;
        return deMath.join32(result);
    }
};
// framework/delibs/debase/deMath.js:548
deMath.numberToArray = function(array, number) {
    for (var byteNdx = 0; byteNdx < array.length; byteNdx++) {
        /** @type {number} */ var acumzndx = !byteNdx ? number : Math.floor(number / Math.pow(256, byteNdx));
        array[byteNdx] = acumzndx & 0xFF;
    }
};
// framework/common/tcuFloat.js:282
tcuFloat.deFloat = function() {
    this.description = tcuFloat.description32;

    this.m_buffer = null;
    this.m_array = null;
    this.bitValue = undefined;
    this.signValue = undefined;
    this.expValue = undefined;
    this.mantissaValue = undefined;

    this.m_value = 0;
};
// framework/common/tcuFloat.js:299
tcuFloat.deFloat.prototype.buffer = function() {
    if (!this.m_buffer)
        this.m_buffer = new ArrayBuffer(this.description.totalByteSize);
    return this.m_buffer;
};
// framework/common/tcuFloat.js:309
tcuFloat.deFloat.prototype.array = function() {
    if (!this.m_array)
        this.m_array = new Uint8Array(this.buffer());
    return this.m_array;
};
// framework/common/tcuFloat.js:321
tcuFloat.deFloat.prototype.deFloatNumber = function(jsnumber) {
    var view32 = new DataView(this.buffer());
    view32.setFloat32(0, jsnumber, true); //little-endian
    this.m_value = view32.getFloat32(0, true); //little-endian

    // Clear cached values
    this.bitValue = undefined;
    this.signValue = undefined;
    this.expValue = undefined;
    this.mantissaValue = undefined;

    return this;
};
// framework/common/tcuFloat.js:408
tcuFloat.deFloat.prototype.deFloatParameters = function(jsnumber, description) {
    this.description = description;

    this.m_buffer = new ArrayBuffer(this.description.totalByteSize);
    this.m_array = new Uint8Array(this.m_buffer);

    return this.deFloatParametersNumber(jsnumber);
};
// framework/common/tcuFloat.js:386
tcuFloat.deFloat.prototype.deFloatParametersNumber = function(jsnumber) {
    this.m_value = jsnumber;
    deMath.numberToArray(this.m_array, jsnumber);

    // Clear cached values
    this.bitValue = undefined;
    this.signValue = undefined;
    this.expValue = undefined;
    this.mantissaValue = undefined;

    return this;
};
// framework/common/tcuTexture.js:3594
tcuTexture.RGBA8View.prototype.read = function(x, y, numChannels) {
    numChannels = numChannels || 4;
    var offset = y * this.stride + x * this.pixelSize;
    /* Always return a vec4 */
    var result = [0, 0, 0, 255];
    for (var i = 0; i < numChannels; i++)
        result[i] = this.data[offset + i];
    return result;
};
// acorn@5.5.3/dist/acorn.js:287
var SourceLocation = function SourceLocation(p, start, end) {
  this.start = start;
  this.end = end;
  if (p.sourceFile !== null) { this.source = p.sourceFile; }
};
// acorn@5.5.3/dist/acorn.js:2838
var Node = function Node(parser, pos, loc) {
  this.type = "";
  this.start = pos;
  this.end = 0;
  if (parser.options.locations)
    { this.loc = new SourceLocation(parser, loc); }
  if (parser.options.directSourceFile)
    { this.sourceFile = parser.options.directSourceFile; }
  if (parser.options.ranges)
    { this.range = [pos, 0]; }
};
// Configuration before loading: BENCH_CONFIG={scale:1,samples:5,warmups:1}.
// Browser: load this file in a script tag. Node/Escargot: run directly.
var config=global.BENCH_CONFIG||{};
var scale=config.scale===undefined?1:config.scale;
var samples=config.samples===undefined?5:config.samples;
var warmups=config.warmups===undefined?1:config.warmups;
if(!(scale>0)||samples<1||samples%1||warmups<0||warmups%1)throw Error('Invalid benchmark configuration');
var log=typeof console!=='undefined'&&typeof console.log==='function'?function(s){console.log(s);}:print;
var sink=global.__allocationBenchSink={value:0,last:null};
var inputs=[];
for(var ii=0;ii<256;ii++)inputs.push([ii/257,(255-ii)/257,(ii%17)/17,1]);
var pixel=Object.create(tcuTexture.RGBA8View.prototype);
pixel.stride=256;pixel.pixelSize=4;pixel.data=new Uint8Array(64*64*4);
for(var pi=0;pi<pixel.data.length;pi++)pixel.data[pi]=(pi*13+7)&255;
function Color(r,g,b){this.red=r;this.green=g;this.blue=b;}
// Equivalent allocation shape to Octane RayTrace; independently written.
function colorScale(c,f){return new Color(c.red*f,c.green*f,c.blue*f);}
function colorAdd(a,b){return new Color(a.red+b.red,a.green+b.green,a.blue+b.blue);}
var colors=[];for(var ci=0;ci<256;ci++)colors.push(new Color(inputs[ci][0],inputs[ci][1],inputs[ci][2]));
var parser={options:{locations:true,ranges:true,directSourceFile:false},sourceFile:null};
var workloads=[
 {name:'deqp-split-shift',base:100000,unit:'shift-pair',fn:function(n){
  var sum=0;
  for(var i=0;i<n;i++){
   var value=4294967296+(i%65536)*17,steps=1+(i%16);
   var result=deMath.shiftLeft(value,steps);
   sum+=(deMath.shiftRight(result,steps)-value);
   // Also observe the high-width result; all values stay below 2^53.
   sum+=(result%65521);
  }
  return sum;
 }},
 {name:'deqp-vector-chain',base:100000,unit:'vec4 scale-add-convert',fn:function(n){
  var sum=0,last;
  for(var i=0;i<n;i++){
   last=deMath.toIVec(deMath.add(inputs[(i+1)&255],deMath.scale(inputs[i&255],0.125)));
   sum+=last[0]+last[1]+last[2]+last[3];
  }
  sink.last=last;return sum;
 }},
 {name:'deqp-rgba-read',base:200000,unit:'RGBA pixel read',fn:function(n){
  var sum=0,last;
  for(var i=0;i<n;i++){
   last=pixel.read(i&63,(i>>>6)&63,4);
   sum+=last[0]+last[1]+last[2]+last[3];
  }
  sink.last=last;return sum;
 }},
 {name:'deqp-float-buffer-cycle',base:50000,unit:'one number plus two bit-pattern constructions',fn:function(n){
  var sum=0,last;
  for(var i=0;i<n;i++){
   var number=new tcuFloat.deFloat().deFloatNumber((i%1024)/13.0);
   var a=new tcuFloat.deFloat().deFloatParameters(0x3f800000+(i%65536),tcuFloat.description32);
   var b=new tcuFloat.deFloat().deFloatParameters(0x3f800000,tcuFloat.description32);
   sum+=number.array()[i&3]+a.m_array[0]+b.m_array[3];last=number;
  }
  sink.last=last;return sum;
 }},
 {name:'octane-raytrace-like',base:100000,unit:'two color scales plus color add',fn:function(n){
  var sum=0,last;
  for(var i=0;i<n;i++){
   last=colorAdd(colorScale(colors[i&255],0.75),colorScale(colors[(i+1)&255],0.25));
   sum+=last.red+last.green+last.blue;
  }
  sink.last=last;return sum;
 }},
 {name:'wtb-acorn-node-shape',base:100000,unit:'located/ranged AST node retained in a 256-node batch',fn:function(n){
  var sum=0,batch=[];
  for(var i=0;i<n;i++){
   var loc={line:1+(i>>>6),column:i&63};
   var node=new Node(parser,i,loc);
   node.type='Identifier';node.end=i+1;node.range[1]=i+1;node.loc.end=loc;
   batch.push(node);
   if(batch.length===256){sum+=batch[0].range[0]+batch[255].end;batch=[];}
  }
  for(var j=0;j<batch.length;j++)sum+=batch[j].end;
  sink.last=batch;return sum;
 }}
];
// Independent scalar/reference computations check numerical outputs outside timing.
function reference(name,n){
 var sum=0;
 for(var i=0;i<n;i++){
  if(name==='deqp-split-shift'){sum+=((4294967296+(i%65536)*17)*Math.pow(2,1+(i%16)))%65521;}
  else if(name==='deqp-vector-chain'){
   for(var j=0;j<4;j++){var v=Math.floor((inputs[(i+1)&255][j]+inputs[i&255][j]*0.125)*255);sum+=Math.min(255,Math.max(0,v));}
  }else if(name==='deqp-rgba-read'){
   var offset=(((i>>>6)&63)*64+(i&63))*4;
   for(var k=0;k<4;k++)sum+=pixel.data[offset+k];
  }else if(name==='deqp-float-buffer-cycle'){
   var buffer=new ArrayBuffer(4),view=new DataView(buffer);
   view.setFloat32(0,(i%1024)/13.0,true);
   sum+=view.getUint8(i&3)+(i%256)+63;
  }else if(name==='octane-raytrace-like'){
   var a=colors[i&255],b=colors[(i+1)&255];
   sum+=(a.red*0.75+b.red*0.25)+(a.green*0.75+b.green*0.25)+(a.blue*0.75+b.blue*0.25);
  }
 }
 if(name==='wtb-acorn-node-shape'){
  var full=Math.floor(n/256);sum=256*full*full;
  for(var t=full*256;t<n;t++)sum+=t+1;
 }
 return sum;
}
function close(a,b){return isFinite(a)&&isFinite(b)&&Math.abs(a-b)<=1e-10*Math.max(1,Math.abs(b));}
var results=[];
for(var wi=0;wi<workloads.length;wi++){
 var w=workloads[wi],n=Math.max(1,Math.floor(w.base*scale));
 // Validate both a boundary-crossing probe and the entire fixed workload.
 if(!close(w.fn(513),reference(w.name,513)))throw Error('Probe mismatch: '+w.name);
 var expected=reference(w.name,n);
 for(var warm=0;warm<warmups;warm++)if(!close(w.fn(n),expected))throw Error('Warmup mismatch: '+w.name);
 var times=[],checksum;
 for(var s=0;s<samples;s++){
  var start=Date.now();checksum=w.fn(n);var elapsed=Date.now()-start;
  if(!close(checksum,expected))throw Error('Checksum mismatch: '+w.name);
  sink.value=checksum;times.push(elapsed);
 }
 var ordered=times.slice().sort(function(a,b){return a-b;});
 var mid=Math.floor(ordered.length/2),median=ordered.length%2?ordered[mid]:(ordered[mid-1]+ordered[mid])/2;
 var record={name:w.name,iterations:n,unit:w.unit,checksum:checksum,expected:expected,timesMs:times,medianMs:median};
 results.push(record);log('ALLOCATION BENCH '+JSON.stringify(record));
}
global.__allocationBenchResults=results;
log('ALLOCATION BENCH COMPLETE '+JSON.stringify({workloads:results.length,samples:samples,warmups:warmups,scale:scale}));

})(typeof globalThis!=="undefined"?globalThis:this);
