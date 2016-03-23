// Copyright 2008 the V8 project authors. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//     * Neither the name of Google Inc. nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Regression test cases for issue 1178598.

/*
// Make sure const-initialization doesn't conflict
// with heap-allocated locals for catch variables.
var value = (function(){
  try { } catch(e) {
    // Force the 'e' variable to be heap-allocated
    // by capturing it in a function closure.
    (function() { e; });
  }
  // Make sure the two definitions of 'e' do
  // not conflict in any way.
  eval("const e=1");
  return e;
})();

assertEquals(1, value);
*/



// Make sure that catch variables can be accessed using eval.
var value = (function() {
  var result;
  try {
    throw 42;
  } catch (e) {
    result = eval("e");
  }
  return result;
})();

assertEquals(42, value);



// Make sure that heap-allocated locals for catch variables aren't
// visible outside the catch scope and that they are visible from
// within.
var value = (function() {
  var result;
  try {
    throw 87;
  } catch(e) {
    // Force the 'e' variable to be heap-allocated
    // by capturing it in a function closure.
    (function() { e; });
    result = eval("e");
  }

  // Expect accessing 'e' to yield an exception because
  // it is not defined in the current scope.
  try {
    eval("e");
    assertTrue(false);  // should throw exception
  } catch(exception) {
    assertTrue(exception instanceof ReferenceError);
    return result;
  }
})();

assertEquals(87, value);
