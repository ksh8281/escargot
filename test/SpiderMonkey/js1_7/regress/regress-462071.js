// escargot-skip: let keyword & for each..in statement not supported
/* -*- indent-tabs-mode: nil; js-indent-level: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

//-----------------------------------------------------------------------------
var BUGNUMBER = 462071;
var summary = 'Do not assert: !ti->stackTypeMap.matches(ti_other->stackTypeMap)';
var actual = '';
var expect = '';


//-----------------------------------------------------------------------------
test();
//-----------------------------------------------------------------------------

function test()
{
  enterFunc ('test');
  printBugNumber(BUGNUMBER);
  printStatus (summary);
 
  jit(true);

  for each (let i in [{}, 0, 0, {}, 0, {}, 0]) { }

  jit(false);

  reportCompare(expect, actual, summary);

  exitFunc ('test');
}
